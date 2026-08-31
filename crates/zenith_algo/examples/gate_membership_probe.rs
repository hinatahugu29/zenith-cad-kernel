//! ゲートが正しい答えを断っている4件だけを、速く測る。
//!
//! `foreign_boolean_probe` は 60 配置 180 演算を回すので、傾けた切り手まで
//! 含めると 30 分かかります。**ゲートの判定そのものを直すときに、そこまで
//! 回すのは測定の妨げ**なので、既知の4件——`sphere` に角の箱を当てた差と和、
//! およびそれを 27 度傾けたもの——だけをここで測ります。
//!
//! 出すのは3つです。
//!
//! 1. 検証なしの口が答えを返すか（＝形が作れているか）
//! 2. 検証つきの口が受け取るか（＝ゲートが通すか）
//! 3. 通らないなら、384 点のうち何点が食い違ったか
//!
//! 1 が通って 2 が落ちるなら、**形は正しいのに判定が断っている**側です。
//!
//! ```bash
//! cargo run --release -p zenith_algo --example gate_membership_probe
//! ```

use std::path::PathBuf;

use zenith_algo::{
    exact_inside, nearest_boundary_projection, BooleanEngine, BooleanOpType, BooleanResultVerifier,
    BrepTransform, PrimitiveBuilder,
};
use zenith_io::StepImporter;
use zenith_math::{Point3, Tolerance, Transform3, Vec3};
use zenith_tess::TriangleMesh;
use zenith_tess::{tessellate_solid, TessellationParams};
use zenith_topo::Solid;

fn params() -> TessellationParams {
    TessellationParams {
        u_divisions: 16,
        v_divisions: 16,
    }
}

fn fixture(name: &str) -> PathBuf {
    PathBuf::from(concat!(env!("CARGO_MANIFEST_DIR"), "/tests/fixtures"))
        .join(format!("occ_reference_{name}.step"))
}

fn mesh_bounds(solid: &Solid) -> (Point3, Point3) {
    let mesh = tessellate_solid(solid, &params());
    let mut low = Point3::new(f64::INFINITY, f64::INFINITY, f64::INFINITY);
    let mut high = Point3::new(f64::NEG_INFINITY, f64::NEG_INFINITY, f64::NEG_INFINITY);
    for point in &mesh.positions {
        low.x = low.x.min(point.x);
        low.y = low.y.min(point.y);
        low.z = low.z.min(point.z);
        high.x = high.x.max(point.x);
        high.y = high.y.max(point.y);
        high.z = high.z.max(point.z);
    }
    (low, high)
}

/// `foreign_boolean_probe` の `corner_block` と同じ置き方。
fn corner_block(low: &Point3, high: &Point3) -> Result<Solid, String> {
    let size = Vec3::new(high.x - low.x, high.y - low.y, high.z - low.z);
    let solid = PrimitiveBuilder::make_box(size.x * 0.45, size.y * 0.45, size.z * 0.45)?;
    Ok(BrepTransform::translate_solid(
        &solid,
        Vec3::new(
            high.x - size.x * 0.30,
            high.y - size.y * 0.30,
            high.z - size.z * 0.30,
        ),
    ))
}

/// `foreign_boolean_probe` の `tilt_about_centre` と同じ 27 度。
fn tilt_about_centre(solid: &Solid, low: &Point3, high: &Point3) -> Result<Solid, String> {
    let centre = Vec3::new(
        (low.x + high.x) * 0.5,
        (low.y + high.y) * 0.5,
        (low.z + high.z) * 0.5,
    );
    let axis = Vec3::new(1.0, 1.0, 1.0);
    let transform = Transform3::from_translation(centre)
        .compose(&Transform3::from_axis_angle(&axis, 27f64.to_radians()))
        .compose(&Transform3::from_translation(-centre));
    BrepTransform::transform_solid(solid, &transform)
}

/// 検証と同じ Halton 列。同じ標本点を見るために揃えます。
fn halton(mut index: usize, base: usize) -> f64 {
    let mut result = 0.0;
    let mut fraction = 1.0 / base as f64;
    while index > 0 {
        result += (index % base) as f64 * fraction;
        index /= base;
        fraction /= base as f64;
    }
    result
}

fn mesh_bbox(mesh: &TriangleMesh) -> Option<(Point3, Point3)> {
    if mesh.positions.is_empty() {
        return None;
    }
    let mut low = Point3::new(f64::INFINITY, f64::INFINITY, f64::INFINITY);
    let mut high = Point3::new(f64::NEG_INFINITY, f64::NEG_INFINITY, f64::NEG_INFINITY);
    for point in &mesh.positions {
        low.x = low.x.min(point.x);
        low.y = low.y.min(point.y);
        low.z = low.z.min(point.z);
        high.x = high.x.max(point.x);
        high.y = high.y.max(point.y);
        high.z = high.z.max(point.z);
    }
    Some((low, high))
}

/// 断られた演算の標本を1点ずつ見る。**なぜ厳密判定が決められないのか**を
/// 名指しするためのもので、`ZENITH_GATE_DETAIL=1` でのみ出します。
fn detail(solid_a: &Solid, solid_b: &Solid, result: &[Solid], op: BooleanOpType, tol: &Tolerance) {
    let tess = TessellationParams {
        u_divisions: 12,
        v_divisions: 12,
    };
    let mesh_a = tessellate_solid(solid_a, &tess);
    let mesh_b = tessellate_solid(solid_b, &tess);
    let mut mesh_r = TriangleMesh::new();
    for solid in result {
        mesh_r.merge(&tessellate_solid(solid, &tess));
    }
    let mut low = Point3::new(f64::INFINITY, f64::INFINITY, f64::INFINITY);
    let mut high = Point3::new(f64::NEG_INFINITY, f64::NEG_INFINITY, f64::NEG_INFINITY);
    for bbox in [mesh_bbox(&mesh_a), mesh_bbox(&mesh_b), mesh_bbox(&mesh_r)]
        .into_iter()
        .flatten()
    {
        low.x = low.x.min(bbox.0.x);
        low.y = low.y.min(bbox.0.y);
        low.z = low.z.min(bbox.0.z);
        high.x = high.x.max(bbox.1.x);
        high.y = high.y.max(bbox.1.y);
        high.z = high.z.max(bbox.1.z);
    }
    let span = Vec3::new(high.x - low.x, high.y - low.y, high.z - low.z);

    println!(
        "      {:>5} {:>24} {:>26} {:>10} {:>10} {:>10}",
        "index", "point", "exact a / b / r", "d(a)", "d(b)", "d(r)"
    );
    for index in 0..384usize {
        let point = Point3::new(
            low.x + span.x * halton(index + 1, 2),
            low.y + span.y * halton(index + 1, 3),
            low.z + span.z * halton(index + 1, 5),
        );
        let ea = exact_inside(point, solid_a, tol);
        let eb = exact_inside(point, solid_b, tol);
        let er = result.iter().map(|s| exact_inside(point, s, tol)).fold(
            Some(false),
            |acc, side| match (acc, side) {
                (Some(true), _) => Some(true),
                (_, Some(true)) => Some(true),
                (None, _) | (_, None) => None,
                (Some(false), Some(false)) => Some(false),
            },
        );
        let expected = match (ea, eb) {
            (Some(a), Some(b)) => Some(match op {
                BooleanOpType::Union => a || b,
                BooleanOpType::Intersection => a && b,
                BooleanOpType::Difference => a && !b,
            }),
            _ => None,
        };
        let disagrees = match (expected, er) {
            (Some(e), Some(r)) => e != r,
            _ => true,
        };
        if !disagrees {
            continue;
        }
        let show = |side: Option<bool>| match side {
            Some(true) => "in",
            Some(false) => "out",
            None => "?",
        };
        let dist = |solid: &Solid| {
            nearest_boundary_projection(point, solid)
                .map(|p| p.distance)
                .unwrap_or(f64::NAN)
        };
        println!(
            "      {index:>5} ({:>6.2} {:>6.2} {:>6.2}) {:>26} {:>10.5} {:>10.5} {:>10.5}",
            point.x,
            point.y,
            point.z,
            format!("{} / {} / {}", show(ea), show(eb), show(er)),
            dist(solid_a),
            dist(solid_b),
            result.iter().map(dist).fold(f64::INFINITY, f64::min),
        );
    }
}

fn main() {
    let tol = Tolerance::default();

    let solids = match StepImporter::import_solids_from_file(&fixture("sphere")) {
        Ok(solids) if !solids.is_empty() => solids,
        Ok(_) => {
            eprintln!("occ_reference_sphere.step held no solids");
            std::process::exit(1);
        }
        Err(err) => {
            eprintln!("could not read occ_reference_sphere.step: {err}");
            std::process::exit(1);
        }
    };
    let sphere = &solids[0];
    let (low, high) = mesh_bounds(sphere);

    let axis_aligned = corner_block(&low, &high).expect("corner block");
    let tilted = tilt_about_centre(&axis_aligned, &low, &high).expect("tilted corner block");

    println!("sphere (read from OpenCASCADE) against a corner block");
    println!();
    println!(
        "{:<16} {:<13} {:>9} {:>7} {:>14} {}",
        "cutter", "op", "unverified", "gate", "mismatch/384", "why the gate refused"
    );
    println!("{}", "-".repeat(112));

    let mut refused = 0usize;
    let mut total = 0usize;

    for (cutter_name, cutter) in [("corner block", &axis_aligned), ("tilted corner", &tilted)] {
        for (op_name, op) in [
            ("difference", BooleanOpType::Difference),
            ("intersection", BooleanOpType::Intersection),
            ("union", BooleanOpType::Union),
        ] {
            total += 1;
            let unverified =
                BooleanEngine::boolean_solids_exact_result_unverified(sphere, cutter, op, &tol);
            let Ok(result) = unverified else {
                println!(
                    "{cutter_name:<16} {op_name:<13} {:>9} {:>7} {:>14}  {}",
                    "refused", "-", "-", "the pipeline itself refused"
                );
                refused += 1;
                continue;
            };

            let report = BooleanResultVerifier::verify(sphere, cutter, &result.solids, op, &tol);
            let passed = report.is_valid();
            if !passed {
                refused += 1;
            }
            println!(
                "{cutter_name:<16} {op_name:<13} {:>9} {:>7} {:>14}  {}",
                "ok",
                if passed { "ok" } else { "REFUSED" },
                format!(
                    "{}/{}",
                    report.membership_mismatch_count, report.classified_sample_count
                ),
                report
                    .errors
                    .first()
                    .map(|e| e.chars().take(60).collect::<String>())
                    .unwrap_or_default()
            );

            if !passed && std::env::var_os("ZENITH_GATE_DETAIL").is_some() {
                detail(sphere, cutter, &result.solids, op, &tol);
            }
        }
    }

    println!("{}", "-".repeat(112));
    println!("{} of {} refused", refused, total);
    if refused > 0 {
        std::process::exit(1);
    }
}
