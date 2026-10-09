//! **ずれの標本に上限を置くと、答えが動くか**（4-710 の測り）。
//!
//! 4-709 で、`ZENITH_SSI_FITSAMPLES` に 256 を置くと **H8 の `A−B` が
//! 61 秒 → 44 秒**になり、**体積は 5 桁ぜんぶ同じ**でした。
//! **ただし測ったのは 1 つの形の 1 つの演算だけ**です。
//!
//! ここでは `foreign_boolean_probe` と**同じ 10 検体・3 配置・3 演算**を、
//! **上限オフと上限 256 の両方**で回して突き合わせます。
//!
//! ```bash
//! cargo run --release -p zenith_algo --example fit_samples_probe
//! ```
//!
//! **赤にするのは 3 つ**——**体積が許す幅を超えて動いた**、
//! **片方だけが断られた**、**片方だけが落ちた**。
//!
//! **測れる範囲を、そのまま言います**: ここで見るのは**この 90 件**です。
//! **動く形が無いことを示したわけではありません。**
//!
//! # なぜ上限を 16（いちばん厳しい値）にするのか
//!
//! **標本の数は `点の数 × 4 + 1`** です。**上限 256 だと、点が 63 以下の組では
//! 一度も効きません**——**効いていない比較は「同じ」になって当然**で、
//! 証拠になりません。**実測でそうでした**: **上限 256 では 90 件ぜんぶ
//! 相対のずれ 0**、**上限 16 にすると 18 件が動きました**（最大 2.994e-9）。
//!
//! **だから既定はいちばん厳しい 16** です。**ここが通れば、256 はそれより
//! 緩い**（標本が多いほど、上限なしに近づきます）。
//! `ZENITH_FIT_SAMPLES_CAP` で変えられます。
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

/// 切り手は `foreign_boolean_probe` と同じ 3 つ。傾けたものは足しません。
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

fn fixture(name: &str) -> PathBuf {
    PathBuf::from(concat!(env!("CARGO_MANIFEST_DIR"), "/tests/fixtures"))
        .join(format!("occ_reference_{name}.step"))
}

enum Outcome {
    Volume(f64),
    Refused,
    Panicked,
}

fn run(a: &Solid, b: &Solid, op: BooleanOpType, tol: &Tolerance) -> Outcome {
    match catch_unwind(AssertUnwindSafe(|| {
        BooleanEngine::boolean_solids_exact_result_unverified(a, b, op, tol)
    })) {
        Err(_) => Outcome::Panicked,
        Ok(Err(_)) => Outcome::Refused,
        Ok(Ok(result)) => Outcome::Volume(total_volume(&result.solids)),
    }
}

fn describe(outcome: &Outcome) -> String {
    match outcome {
        Outcome::Volume(value) => format!("体積 {value:.6}"),
        Outcome::Refused => "断りました".to_string(),
        Outcome::Panicked => "落ちました".to_string(),
    }
}

fn main() {
    let tol = Tolerance::default();
    let cap: usize = std::env::var("ZENITH_FIT_SAMPLES_CAP")
        .ok()
        .and_then(|text| text.parse().ok())
        .filter(|value| *value >= 16)
        .unwrap_or(16);

    println!("ずれの標本に上限を置くと、答えが動くか（4-710）");
    println!();
    println!("**上限オフ**（いまの既定）と**上限 {cap}** で、同じ演算を 2 度回して");
    println!("突き合わせます。`ZENITH_SSI_FITSAMPLES` は既定オフの口（4-495）です。");
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
        "subject", "cutter", "op", "上限オフ", "上限あり", "相対のずれ"
    );
    println!("{}", "-".repeat(88));

    let mut compared = 0usize;
    let mut skipped = 0usize;
    let mut worst = 0.0f64;
    // 片方だけが断った・落ちた件数。どちらも赤にします。
    let mut disagreed: Vec<String> = Vec::new();

    for name in subjects {
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

        for (cutter_name, build) in cutters {
            let Ok(b) = build(&low, &high) else {
                skipped += 3;
                continue;
            };
            for (op_name, op) in ops {
                // 上限オフ。
                std::env::remove_var("ZENITH_SSI_FITSAMPLES");
                let plain = run(a, &b, op, &tol);
                // 上限あり。
                std::env::set_var("ZENITH_SSI_FITSAMPLES", cap.to_string());
                let capped = run(a, &b, op, &tol);
                std::env::remove_var("ZENITH_SSI_FITSAMPLES");

                match (&plain, &capped) {
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
                            "{name:<18} {cutter_name:<14} {op_name:<5}   両方とも落ちました（上限とは別の話）"
                        );
                    }
                    (Outcome::Refused, Outcome::Refused) => {
                        skipped += 1;
                        println!(
                            "{name:<18} {cutter_name:<14} {op_name:<5}   両方とも断りました（上限とは別の話）"
                        );
                    }
                    _ => {
                        disagreed.push(format!(
                            "{name} / {cutter_name} / {op_name}: 上限オフは{}、上限ありは{}",
                            describe(&plain),
                            describe(&capped)
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
    println!();
    println!("**見たのはこの {compared} 件だけ**です。**動く形が無いことを");
    println!("示したわけではありません。**");
    println!();

    // 許す幅。体積は刻みで決まる量なので、刻みの揺れぶんは許します。
    // 4-709 では H8 で 5 桁ぜんぶ同じ（相対 0）でした。
    let allowance = 1e-6;
    let over = usize::from(worst > allowance) + disagreed.len();
    if over > 0 {
        println!("**{over} over the allowance**（許す幅 {allowance:.0e}）");
        println!("上限を置くと答えが動きます。既定を変えてはいけません。");
        std::process::exit(1);
    }
    if compared == 0 {
        println!("**1 miss**  1 件も突き合わせられませんでした。");
        std::process::exit(1);
    }
    println!("**答えは動きません**（許す幅 {allowance:.0e} の内、{compared} 件）。");
}
