//! **H8 の口 20 本のうち、どれが `torus × corner_block` の `A^B` を
//! 動かすかを切り分けます**（4-752 の宿題。ROADMAP.md 柱①）。
//!
//! 4-752 で、H8 の口 20 本を `linkrods` 以外の 10 検体に立てたら、
//! **`torus` / `corner block` / `A^B` が最悪 7.573e-8 動く**と分かり
//! ました。**どの口（単独、または組）が動かすのかは測っていません。**
//!
//! ここでは、**その 1 件だけ**を、**口を 1 本ずつ立てた 20 通り**で
//! 測り、**口なしとの相対差**を出します。
//!
//! ```bash
//! cargo run --release -p zenith_algo --example h8_ports_bisect_probe
//! ```
use std::panic::{catch_unwind, AssertUnwindSafe};
use std::path::PathBuf;

use zenith_algo::{BooleanEngine, BooleanOpType, BrepTransform, MassCalculator, PrimitiveBuilder};
use zenith_io::StepImporter;
use zenith_math::{Point3, Tolerance, Vec3};
use zenith_tess::{tessellate_solid, TessellationParams, TriangleMesh};
use zenith_topo::Solid;

fn params() -> TessellationParams {
    TessellationParams {
        u_divisions: 64,
        v_divisions: 64,
    }
}

fn volume(solid: &Solid) -> f64 {
    MassCalculator::compute_from_brep(solid, &params()).volume
}

fn total_volume(solids: &[Solid]) -> f64 {
    solids.iter().map(volume).sum()
}

fn mesh_bounds(mesh: &TriangleMesh) -> (Point3, Point3) {
    let mut low = Point3::new(f64::MAX, f64::MAX, f64::MAX);
    let mut high = Point3::new(f64::MIN, f64::MIN, f64::MIN);
    for vertex in &mesh.positions {
        low.x = low.x.min(vertex.x);
        low.y = low.y.min(vertex.y);
        low.z = low.z.min(vertex.z);
        high.x = high.x.max(vertex.x);
        high.y = high.y.max(vertex.y);
        high.z = high.z.max(vertex.z);
    }
    (low, high)
}

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

const H8_PORTS: &[(&str, &str)] = &[
    ("ZENITH_SSI_EMPTY_RETRY", "1"),
    ("ZENITH_SSI_EXTRA_BRANCHES", "1"),
    ("ZENITH_WELD_ENDS", "2e-4"),
    ("ZENITH_LEFTOVER_LOOPS", "1"),
    ("ZENITH_HOLE_CUT", "1"),
    ("ZENITH_SPLIT_ONE_PIECE", "1"),
    ("ZENITH_DEDUP_SELECTED", "1"),
    ("ZENITH_ID_WELD", "1"),
    ("ZENITH_HOLE_BOX_GUARD", "1"),
    ("ZENITH_FACE_SNAP", "1"),
    ("ZENITH_CHAIN_AREA_TOL", "1"),
    ("ZENITH_HOLECUT_SIGN", "1"),
    ("ZENITH_WRAPPED_AREA", "1"),
    ("ZENITH_KEEP_MIXED_HOLES", "1"),
    ("ZENITH_EDGE_TOL_FROM_FACES", "1"),
    ("ZENITH_EDGE_TOL_BOUNDARY", "1"),
    ("ZENITH_IMPORT_ROUGHNESS_SAMPLES", "64"),
    ("ZENITH_VERTEX_IMPRINT_TOL", "1"),
    ("ZENITH_CAP_ORIENT_BY_GEOMETRY", "1"),
    ("ZENITH_ID_WELD_PLANES", "1"),
];

fn clear_all() {
    for (name, _) in H8_PORTS {
        std::env::remove_var(name);
    }
}

enum Outcome {
    Volume(f64),
    Refused(String),
    Panicked,
}

fn run(a: &Solid, b: &Solid, op: BooleanOpType, tol: &Tolerance) -> Outcome {
    match catch_unwind(AssertUnwindSafe(|| {
        BooleanEngine::boolean_solids_exact_result_unverified(a, b, op, tol)
    })) {
        Err(_) => Outcome::Panicked,
        Ok(Err(err)) => Outcome::Refused(err.split(';').next().unwrap_or(&err).to_string()),
        Ok(Ok(result)) => Outcome::Volume(total_volume(&result.solids)),
    }
}

fn main() {
    let tol = Tolerance::default();
    // **4-753 の追測**（ROADMAP 柱①）。`ZENITH_BISECT_SUBJECT` で検体を
    // 選べます（既定 `torus`）——`cone`（`A^B` 体積 0.003238、退化に
    // 近い組。4-753 で `SUBDIV_THIRD` を外しても 1.251e-8 が残った組）
    // を切り分けるために足しました。
    let subject = std::env::var("ZENITH_BISECT_SUBJECT").unwrap_or_else(|_| "torus".to_string());
    let path = PathBuf::from(concat!(env!("CARGO_MANIFEST_DIR"), "/tests/fixtures"))
        .join(format!("occ_reference_{subject}.step"));
    let solids =
        StepImporter::import_solids_from_file(&path).unwrap_or_else(|_| panic!("{subject} を読めません"));
    let a = solids.first().expect("立体が無い");
    let mesh = tessellate_solid(a, &params());
    let (low, high) = mesh_bounds(&mesh);
    let b = corner_block(&low, &high).expect("切り手を作れません");

    println!("{subject} / corner block / A^B を、H8 の口 1 本ずつで測ります（4-752 の宿題）");
    println!();

    clear_all();
    let baseline = run(a, &b, BooleanOpType::Intersection, &tol);
    let base_value = match &baseline {
        Outcome::Volume(v) => *v,
        _ => {
            println!("口なしの基準値が取れませんでした。終了します。");
            return;
        }
    };
    println!("口なし（基準）: 体積 {base_value:.6}");
    println!();
    println!("{:<34} {:>16} {:>12}", "口", "体積", "相対差");
    println!("{}", "-".repeat(66));

    for (name, value) in H8_PORTS {
        clear_all();
        std::env::set_var(name, value);
        let outcome = run(a, &b, BooleanOpType::Intersection, &tol);
        match outcome {
            Outcome::Volume(v) => {
                let relative = (v - base_value).abs() / base_value.abs().max(1e-12);
                let flag = if relative > 1e-12 { "  <-- 動く" } else { "" };
                println!("{name:<34} {v:>16.9} {relative:>12.3e}{flag}");
            }
            Outcome::Refused(why) => println!("{name:<34}   断りました（{why}）"),
            Outcome::Panicked => println!("{name:<34}   落ちました"),
        }
        clear_all();
    }

    // **4-752 の再測で、A^B だけ `ZENITH_SUBDIV_THIRD` を落としていた
    // ことに気づいた**——`h8_ports_on_others_probe` の `enter_h8_ports`
    // は積のときだけこれも立てます。単独でも測ります。
    clear_all();
    std::env::set_var("ZENITH_SUBDIV_THIRD", "1");
    let subdiv_outcome = run(a, &b, BooleanOpType::Intersection, &tol);
    clear_all();
    if let Outcome::Volume(v) = subdiv_outcome {
        let relative = (v - base_value).abs() / base_value.abs().max(1e-12);
        let flag = if relative > 1e-12 { "  <-- 動く" } else { "" };
        println!("{:<34} {v:>16.9} {relative:>12.3e}{flag}", "ZENITH_SUBDIV_THIRD");
    }

    // 全部立てた場合（4-752 の 7.573e-8 を再現できるか）。
    // A^B なので `ZENITH_SUBDIV_THIRD` も立てます（`enter_h8_ports` と同じ）。
    for (name, value) in H8_PORTS {
        std::env::set_var(name, value);
    }
    std::env::set_var("ZENITH_SUBDIV_THIRD", "1");
    let all_on = run(a, &b, BooleanOpType::Intersection, &tol);
    clear_all();
    std::env::remove_var("ZENITH_SUBDIV_THIRD");
    if let Outcome::Volume(v) = all_on {
        let relative = (v - base_value).abs() / base_value.abs().max(1e-12);
        println!();
        println!("全 20 本: 体積 {v:.9}  相対差 {relative:.3e}（4-752 の再現確認）");
    }
}
