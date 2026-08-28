//! **他カーネルの立体どうし**を、**異なる検体**で総当たりに掛ける。
//!
//! `boolean_foreign_pair_test` は「同じ検体を2つ（片方をずらす）」です。
//! ここは**違う形どうし**を、中心を揃えて必ず重ねます。
//!
//! ## なぜ要るのか
//!
//! 2026/08/28 に、この掃き方で**誤答**が1件出ました（4-142）。
//! `cylinder × elliptic_prism` で恒等式の残差 1.2e-2。**積は正確なのに、
//! 和と差だけがまったく同じ量（150.95）だけ過大**という形です。
//!
//! **そのときの測定は使い捨ての例で行われ、残っていませんでした。**
//! 直す前に、まず再現をここに固定します。
//!
//! ## 何を見るか
//!
//! 閉じた式はありません。**恒等式**で見ます。
//!
//! ```text
//! |A ∪ B| + |A ∩ B| = |A| + |B|
//! |A \ B| + |A ∩ B| = |A|
//! ```
//!
//! **断るのは赤にしません。** 3演算そろった組だけ突き合わせます。
//! 赤にするのは「返ってきたのに恒等式が破れている」ほうだけです。
//!
//! ```bash
//! cargo run --release -p zenith_algo --example foreign_cross_pair_probe
//! ZENITH_PAIR_FILTER="cylinder+elliptic_prism" \
//!   cargo run --release -p zenith_algo --example foreign_cross_pair_probe
//! ```

use std::path::PathBuf;

use zenith_algo::{
    BooleanEngine, BooleanOpType, BrepTransform, MassCalculator, Regularizer,
};
use zenith_io::StepImporter;
use zenith_math::{Tolerance, Vec3};
use zenith_tess::TessellationParams;
use zenith_topo::Solid;

fn params() -> TessellationParams {
    TessellationParams {
        u_divisions: 24,
        v_divisions: 24,
    }
}

fn volume(solids: &[Solid]) -> f64 {
    solids
        .iter()
        .map(|solid| MassCalculator::compute_from_brep(solid, &params()).volume)
        .sum()
}

fn load(name: &str, tol: &Tolerance) -> Option<Solid> {
    let path = PathBuf::from(concat!(env!("CARGO_MANIFEST_DIR"), "/tests/fixtures"))
        .join(format!("occ_reference_{name}.step"));
    let solids = StepImporter::import_solids_from_file(&path).ok()?;
    solids
        .first()
        .map(|solid| Regularizer::hold_like_our_own(solid, tol))
}

/// 立体の中心が原点に来るように移す。**必ず重ねる**ため。
fn centred(solid: &Solid) -> Solid {
    let bbox = solid.bounding_box();
    let centre = Vec3::new(
        (bbox.min.x + bbox.max.x) * 0.5,
        (bbox.min.y + bbox.max.y) * 0.5,
        (bbox.min.z + bbox.max.z) * 0.5,
    );
    BrepTransform::translate_solid(solid, -centre)
}

fn main() {
    let tol = Tolerance::default();
    let filter = std::env::var("ZENITH_PAIR_FILTER").ok();

    // **既定は3検体・3組です。実行時間を測って絞りました。**
    //
    // 6検体（15組）にすると**10分で終わりません**（実測）。CI の1本あたりの
    // 上限は 1200 秒なので、そのままでは常設に置けません。**4-142 の現場
    // （`cylinder × elliptic_prism`）を含む最小の組**を既定にしてあります。
    //
    // **広げるときは、まず実行時間を測ってください。** `ZENITH_PAIR_EXTRA=1`
    // で、時間の掛かる検体（トーラス・段付き軸・空洞つき）も足せます。
    let mut names = vec!["cylinder", "elliptic_prism", "sphere"];
    if std::env::var_os("ZENITH_PAIR_EXTRA").is_some() {
        names.extend(["torus", "stepped_shaft", "hollow_box"]);
    }

    let mut loaded: Vec<(&str, Solid)> = Vec::new();
    for name in names {
        if let Some(solid) = load(name, &tol) {
            loaded.push((name, centred(&solid)));
        }
    }

    println!("他カーネルの立体どうし（異なる検体、中心を揃えて必ず重ねる）");
    println!();
    println!(
        "{:<34} {:>13} {:>13} {:>12} {:>12}  {}",
        "組", "|A|", "|B|", "包除の残差", "分割の残差", "判定"
    );
    println!("{}", "-".repeat(106));

    let mut compared = 0usize;
    let mut refused = 0usize;
    let mut wrong = 0usize;
    let mut worst = 0.0f64;

    for left in 0..loaded.len() {
        for right in (left + 1)..loaded.len() {
            let label = format!("{}+{}", loaded[left].0, loaded[right].0);
            if let Some(needle) = &filter {
                if !label.contains(needle.as_str()) {
                    continue;
                }
            }
            let (a, b) = (&loaded[left].1, &loaded[right].1);

            let volume_a = MassCalculator::compute_from_brep(a, &params()).volume;
            let volume_b = MassCalculator::compute_from_brep(b, &params()).volume;

            let run = |op| {
                BooleanEngine::boolean_solids_exact_result(a, b, op, &tol)
                    .ok()
                    .map(|result| volume(&result.solids))
            };
            let union = run(BooleanOpType::Union);
            let meet = run(BooleanOpType::Intersection);
            let cut = run(BooleanOpType::Difference);

            let (Some(union), Some(meet), Some(cut)) = (union, meet, cut) else {
                refused += 1;
                println!(
                    "{label:<34} {volume_a:>13.6} {volume_b:>13.6} {:>12} {:>12}  3演算そろわず（断りを含む）",
                    "-", "-"
                );
                continue;
            };
            compared += 1;

            let scale = volume_a.abs().max(1.0);
            let inclusion = ((union + meet) - (volume_a + volume_b)).abs() / scale;
            let split = ((cut + meet) - volume_a).abs() / scale;
            let residual = inclusion.max(split);
            worst = worst.max(residual);

            if std::env::var_os("ZENITH_PAIR_VOLUMES").is_some() {
                // **恒等式が破れたとき、どの演算が外しているかは残差だけでは
                // 分かりません。** 3つの値をそのまま出します。
                eprintln!(
                    "PAIRVOL {label}: 和 {union:.6}、積 {meet:.6}、差 {cut:.6}（|A| {volume_a:.6}、|B| {volume_b:.6}）"
                );
                eprintln!(
                    "PAIRVOL   和 − (|A|+|B|−積) = {:.6}、差 − (|A|−積) = {:.6}",
                    union - (volume_a + volume_b - meet),
                    cut - (volume_a - meet)
                );
            }

            let verdict = if residual > 1e-5 {
                wrong += 1;
                "**恒等式が破れています**"
            } else {
                "ok"
            };
            println!(
                "{label:<34} {volume_a:>13.6} {volume_b:>13.6} {inclusion:>12.3e} {split:>12.3e}  {verdict}"
            );
        }
    }

    println!("{}", "-".repeat(106));
    println!(
        "{compared} 組が3演算そろい、{refused} 組はそろいませんでした。**恒等式が破れた組 {wrong}**、残差の最悪 {worst:.3e}。"
    );
    println!();
    println!("**断るのは赤にしません。** 返ってきたのに恒等式が破れているほうだけを赤にします。");

    if wrong > 0 {
        eprintln!("GATE ERROR: identity broken on {wrong} pair(s)");
        std::process::exit(1);
    }
}
