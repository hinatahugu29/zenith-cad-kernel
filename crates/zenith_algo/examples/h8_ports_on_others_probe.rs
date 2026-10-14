//! **H8 の口 20 本を、`linkrods` 以外の 10 検体にも立てたら答えが動くか**
//! （ROADMAP.md 柱①の次の段。`tools/h8_ports.sh` が立てる 20 本）。
//!
//! **これまでの `*_probe` 群（`trim_polygon_cache_probe` など）は、
//! H8 の口を `linkrods` 1 検体にしか立てていません**——**`foreign_boolean_probe`
//! と同じ他の 10 検体では、一度も H8 の口を立てて測ったことがありません。**
//! **口を既定へ吸収できるかを考える前に、まずここを測ります。**
//!
//! ここでは `foreign_boolean_probe` と**同じ 10 検体・3 配置・3 演算**
//! （90 件）を、**H8 の口 20 本を立てない場合と立てる場合の両方**で
//! 回して突き合わせます（`ZENITH_SUBDIV_THIRD` は H8 の積だけに要る
//! 口で、断りを誤答に変えるため含めません。4-558）。
//!
//! `linkrods` も最後に 1 回、口を立てた状態で返るかだけ確かめます
//! （比較ではなく生存確認。口が無いと断られるので比較になりません）。
//!
//! ```bash
//! cargo run --release -p zenith_algo --example h8_ports_on_others_probe
//! ```
//!
//! **赤にするのは 3 つ**——**体積が許す幅を超えて動いた**、
//! **片方だけが断られた**、**片方だけが落ちた**。
use std::panic::{catch_unwind, AssertUnwindSafe};
use std::path::PathBuf;

use zenith_algo::{
    BooleanEngine, BooleanOpType, BrepTransform, MassCalculator, PrimitiveBuilder,
};
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

/// 切り手は `foreign_boolean_probe` と同じ 3 つ。
fn half_slab(low: &Point3, high: &Point3) -> Result<Solid, String> {
    let size = Vec3::new(high.x - low.x, high.y - low.y, high.z - low.z);
    let solid = PrimitiveBuilder::make_box(size.x * 0.6, size.y * 2.0, size.z * 2.0)?;
    Ok(BrepTransform::translate_solid(
        &solid,
        Vec3::new(
            low.x - size.x * 0.11,
            low.y - size.y * 0.5,
            low.z - size.z * 0.5,
        ),
    ))
}

fn centre_drill(low: &Point3, high: &Point3) -> Result<Solid, String> {
    let size = Vec3::new(high.x - low.x, high.y - low.y, high.z - low.z);
    let radius = size.x.min(size.y) * 0.18;
    let height = size.z * 3.0;
    let solid = PrimitiveBuilder::make_cylinder(radius, height)?;
    Ok(BrepTransform::translate_solid(
        &solid,
        Vec3::new(
            (low.x + high.x) * 0.5,
            (low.y + high.y) * 0.5,
            low.z - size.z,
        ),
    ))
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

/// **H8 の切り手**。**境界箱を x/y で 3% 内側に寄せ、z は下から 47%**。
fn h8_box(low: &Point3, high: &Point3) -> Result<Solid, String> {
    let span = Vec3::new(high.x - low.x, high.y - low.y, high.z - low.z);
    let (inset, height) = (0.03, 0.47);
    let solid = PrimitiveBuilder::make_box(
        span.x * (1.0 - inset * 2.0),
        span.y * (1.0 - inset * 2.0),
        span.z * height,
    )?;
    Ok(BrepTransform::translate_solid(
        &solid,
        Vec3::new(
            low.x + span.x * inset,
            low.y + span.y * inset,
            low.z + span.z * (height * 0.5),
        ),
    ))
}

/// **`tools/h8_ports.sh` が立てる 20 本**（`ZENITH_SUBDIV_THIRD` は
/// 含めません——断りを誤答に変えるため、積にしか立てません。4-558）。
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

/// **4-752・4-753 の追測**（ROADMAP 柱①）。`ZENITH_H8_PORTS_NO_SUBDIV3=1`
/// で `ZENITH_SUBDIV_THIRD` を、`ZENITH_H8_PORTS_NO_WELD_ENDS=1` で
/// `ZENITH_WELD_ENDS` を外せます——**`h8_ports_bisect_probe` が
/// `torus`（`SUBDIV_THIRD`）と `cone`（`WELD_ENDS`）でそれぞれ単独
/// 再現した 2 本を、90 件でも外して確かめます**。
fn enter_h8_ports(op: BooleanOpType) {
    let skip_weld_ends = std::env::var_os("ZENITH_H8_PORTS_NO_WELD_ENDS").is_some();
    for (name, value) in H8_PORTS {
        if skip_weld_ends && *name == "ZENITH_WELD_ENDS" {
            continue;
        }
        std::env::set_var(name, value);
    }
    let skip_subdiv3 = std::env::var_os("ZENITH_H8_PORTS_NO_SUBDIV3").is_some();
    if matches!(op, BooleanOpType::Intersection) && !skip_subdiv3 {
        std::env::set_var("ZENITH_SUBDIV_THIRD", "1");
    }
}

fn leave_h8_ports() {
    for (name, _) in H8_PORTS {
        std::env::remove_var(name);
    }
    std::env::remove_var("ZENITH_SUBDIV_THIRD");
}

fn fixture(name: &str) -> PathBuf {
    if name == "linkrods" {
        return PathBuf::from(concat!(env!("CARGO_MANIFEST_DIR"), "/../.."))
            .join("reference/OCCT/data/step/linkrods.step");
    }
    PathBuf::from(concat!(env!("CARGO_MANIFEST_DIR"), "/tests/fixtures"))
        .join(format!("occ_reference_{name}.step"))
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

fn describe(outcome: &Outcome) -> String {
    match outcome {
        Outcome::Volume(value) => format!("体積 {value:.6}"),
        Outcome::Refused(why) => format!("断りました（{why}）"),
        Outcome::Panicked => "落ちました".to_string(),
    }
}

fn main() {
    let tol = Tolerance::default();

    println!("H8 の口 20 本を、linkrods 以外の 10 検体にも立てたら答えが動くか");
    println!();
    println!("**口なし**（いまの既定）と**H8 の口 20 本**で、同じ演算を 2 度回して");
    println!("突き合わせます。`foreign_boolean_probe` と同じ 10 検体・3 配置・3 演算です。");
    println!();

    let subjects = [
        "cone",
        "cone_full",
        "cylinder_nurbs",
        "elliptic_prism",
        "extruded_spline",
        "revolved_ring",
        "sphere",
        "sphere_capped",
        "torus",
        "torus_segment",
    ];
    let cutters: [(&str, fn(&Point3, &Point3) -> Result<Solid, String>); 3] = [
        ("half slab", half_slab),
        ("centre drill", centre_drill),
        ("corner block", corner_block),
    ];
    let ops = [
        ("A-B", BooleanOpType::Difference),
        ("A^B", BooleanOpType::Intersection),
        ("AuB", BooleanOpType::Union),
    ];

    println!(
        "{:<18} {:<14} {:<5} {:>16} {:>16} {:>12}",
        "subject", "cutter", "op", "口なし", "口20本", "相対のずれ"
    );
    println!("{}", "-".repeat(88));

    let mut compared = 0usize;
    let mut skipped = 0usize;
    let mut worst = 0.0f64;
    let mut disagreed: Vec<String> = Vec::new();

    let only = std::env::var("ZENITH_SUBJECT_ONLY").ok();
    for name in subjects {
        if let Some(wanted) = only.as_deref() {
            if name != wanted {
                continue;
            }
        }
        let Ok(solids) = StepImporter::import_solids_from_file(&fixture(name)) else {
            println!("{name:<18} 読めませんでした");
            skipped += 9;
            continue;
        };
        let Some(a) = solids.first() else {
            println!("{name:<18} 立体がありません");
            skipped += 9;
            continue;
        };
        let mesh = tessellate_solid(a, &params());
        let (low, high) = mesh_bounds(&mesh);

        for &(cutter_name, build) in &cutters {
            let Ok(b) = build(&low, &high) else {
                skipped += 3;
                continue;
            };
            for (op_name, op) in ops {
                // 口なし（いまの既定）。
                let plain = run(a, &b, op, &tol);
                // H8 の口 20 本。
                enter_h8_ports(op);
                let ported = run(a, &b, op, &tol);
                leave_h8_ports();

                match (&plain, &ported) {
                    (Outcome::Volume(x), Outcome::Volume(y)) => {
                        let scale = x.abs().max(y.abs()).max(1e-12);
                        let relative = (x - y).abs() / scale;
                        worst = worst.max(relative);
                        compared += 1;
                        println!(
                            "{name:<18} {cutter_name:<14} {op_name:<5} {x:>16.6} {y:>16.6} {relative:>12.3e}"
                        );
                    }
                    (Outcome::Panicked, Outcome::Panicked) => {
                        skipped += 1;
                        println!(
                            "{name:<18} {cutter_name:<14} {op_name:<5}   両方とも落ちました"
                        );
                    }
                    (Outcome::Refused(a_why), Outcome::Refused(b_why)) => {
                        skipped += 1;
                        if a_why == b_why {
                            println!(
                                "{name:<18} {cutter_name:<14} {op_name:<5}   両方とも断りました: {a_why}"
                            );
                        } else {
                            println!(
                                "{name:<18} {cutter_name:<14} {op_name:<5}   両方とも断りました: 口なしは「{a_why}」、口20本は「{b_why}」"
                            );
                        }
                    }
                    _ => {
                        disagreed.push(format!(
                            "{name} / {cutter_name} / {op_name}: 口なしは{}、口20本は{}",
                            describe(&plain),
                            describe(&ported)
                        ));
                        println!(
                            "{name:<18} {cutter_name:<14} {op_name:<5}   **片方だけ違います**"
                        );
                    }
                }
            }
        }
    }

    println!();
    println!("突き合わせた件数: {compared}、採点していない件数: {skipped}");
    println!("相対のいちばん大きなずれ: {worst:.3e}");
    if !disagreed.is_empty() {
        println!();
        println!("**片方だけ違った件**:");
        for line in &disagreed {
            println!("  {line}");
        }
    }

    // linkrods の生存確認（比較ではない——口が無いと断られるので）。
    //
    // **読み込む前に口を立てます**（4-717 と同じ理由。`ZENITH_IMPORT_ROUGHNESS_SAMPLES`
    // と `ZENITH_EDGE_TOL_FROM_FACES` は読み込みに効く口なので、読んだ後に
    // 立てても遅い）。
    println!();
    println!("**`linkrods`（口 20 本。比較ではなく生存確認）**:");
    enter_h8_ports(BooleanOpType::Difference);
    if let Ok(solids) = StepImporter::import_solids_from_file(&fixture("linkrods")) {
        if let Some(a) = solids.iter().max_by_key(|solid| solid.outer_shell.faces.len()) {
            let mesh = tessellate_solid(a, &TessellationParams::default());
            let (low, high) = mesh_bounds(&mesh);
            if let Ok(b) = h8_box(&low, &high) {
                for (op_name, op) in ops {
                    enter_h8_ports(op);
                    let outcome = run(a, &b, op, &tol);
                    leave_h8_ports();
                    println!("  {op_name:<5} {}", describe(&outcome));
                }
            }
        }
    }

    println!();
    println!("**見たのはこの {compared} 件だけ**です。**動く形が無いことを");
    println!("示したわけではありません。**");
    println!();

    let allowance = 1e-6;
    let over = usize::from(worst > allowance) + disagreed.len();
    if over > 0 {
        println!("**{over} over the allowance**（許す幅 {allowance:.0e}）");
        println!("H8 の口 20 本は、他の検体の答えも動かします。個別に切り分けてください。");
        std::process::exit(1);
    }
    if compared == 0 {
        println!("**1 miss**  1 件も突き合わせられませんでした。");
        std::process::exit(1);
    }
    println!("**答えは動きません**（許す幅 {allowance:.0e} の内、{compared} 件）。");
}
