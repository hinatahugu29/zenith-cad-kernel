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

/// **H8 の口**（`tools/h8_ports.sh` の 20 本と、積だけの 1 本）。
/// **`linkrods` は既定では切れません**（4-716）。
fn enter_h8_ports(op: BooleanOpType) {
    for (name, value) in [
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
    ] {
        std::env::set_var(name, value);
    }
    if matches!(op, BooleanOpType::Intersection) {
        std::env::set_var("ZENITH_SUBDIV_THIRD", "1");
    }
}

/// 立てた口を外します。**次の検体に漏らしてはいけません。**
fn leave_h8_ports() {
    for name in [
        "ZENITH_SSI_EMPTY_RETRY",
        "ZENITH_SSI_EXTRA_BRANCHES",
        "ZENITH_WELD_ENDS",
        "ZENITH_LEFTOVER_LOOPS",
        "ZENITH_HOLE_CUT",
        "ZENITH_SPLIT_ONE_PIECE",
        "ZENITH_DEDUP_SELECTED",
        "ZENITH_ID_WELD",
        "ZENITH_HOLE_BOX_GUARD",
        "ZENITH_FACE_SNAP",
        "ZENITH_CHAIN_AREA_TOL",
        "ZENITH_HOLECUT_SIGN",
        "ZENITH_WRAPPED_AREA",
        "ZENITH_KEEP_MIXED_HOLES",
        "ZENITH_EDGE_TOL_FROM_FACES",
        "ZENITH_EDGE_TOL_BOUNDARY",
        "ZENITH_IMPORT_ROUGHNESS_SAMPLES",
        "ZENITH_VERTEX_IMPRINT_TOL",
        "ZENITH_CAP_ORIENT_BY_GEOMETRY",
        "ZENITH_ID_WELD_PLANES",
        "ZENITH_SUBDIV_THIRD",
    ] {
        std::env::remove_var(name);
    }
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
    println!("**上限オフ**（4-751 で既定オンに昇格する前の旧既定）と**上限 {cap}** で、");
    println!("同じ演算を 2 度回して突き合わせます。");
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
        // **交線が長い検体**（4-742）。**上限がしっかり効くのはここだけ**です
        // ——他の 10 検体は標本が 257〜513 点で、1024 に届きません（4-741）。
        // **既定では切れません**ので、H8 の口を立てて回します。
        "linkrods",
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
        // **読み込む前に口を立てます**（4-717）。**許容は読み込みのときに
        // 決まる**ので、読み込んだ後では遅い。
        if name == "linkrods" {
            enter_h8_ports(BooleanOpType::Difference);
        }
        let Ok(solids) = StepImporter::import_solids_from_file(&fixture(name)) else {
            println!("{name:<18} 読めませんでした");
            skipped += 9;
            continue;
        };
        // **`linkrods` は立体が複数**——**H8 は面数が最大のものを選びます。**
        let picked = if name == "linkrods" {
            solids
                .iter()
                .max_by_key(|solid| solid.outer_shell.faces.len())
        } else {
            solids.first()
        };
        let Some(a) = picked else {
            println!("{name:<18} 立体がありません");
            skipped += 9;
            continue;
        };
        // **`linkrods` の箱は既定の刻みで**（H8 と同じ。4-716）。
        let mesh = if name == "linkrods" {
            tessellate_solid(a, &TessellationParams::default())
        } else {
            tessellate_solid(a, &params())
        };
        let (low, high) = mesh_bounds(&mesh);

        // **`linkrods` だけは H8 の箱ひとつ。**
        let h8_only: [(&str, fn(&Point3, &Point3) -> Result<Solid, String>); 1] =
            [("H8 の箱", h8_box)];
        let chosen: &[(&str, fn(&Point3, &Point3) -> Result<Solid, String>)] =
            if name == "linkrods" { &h8_only } else { &cutters };
        for &(cutter_name, build) in chosen {
            let Ok(b) = build(&low, &high) else {
                skipped += 3;
                continue;
            };
            for (op_name, op) in ops {
                let h8 = name == "linkrods";
                if h8 {
                    enter_h8_ports(op);
                }
                // 上限オフ（4-751 で既定が 1024 になったので、`0` を
                // 明示しないと上限なしになりません）。
                std::env::set_var("ZENITH_SSI_FITSAMPLES", "0");
                let plain = run(a, &b, op, &tol);
                // 上限あり。
                std::env::set_var("ZENITH_SSI_FITSAMPLES", cap.to_string());
                let capped = run(a, &b, op, &tol);
                std::env::remove_var("ZENITH_SSI_FITSAMPLES");

                if h8 {
                    leave_h8_ports();
                }
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
