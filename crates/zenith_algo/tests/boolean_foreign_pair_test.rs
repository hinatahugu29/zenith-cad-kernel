//! **他カーネルが書いた立体どうし**のブーリアン。
//!
//! いままで測っていたのは「他カーネルの立体 × 自作の切り手」だけでした
//! （`foreign_boolean_probe`）。ここは**両方とも OpenCASCADE が書いた
//! STEP** です。
//!
//! ## なぜ要るのか
//!
//! 2026/08/28 に、この組み合わせで**パニック**が出ました（4-141）。
//! 空洞のある立体（`hollow_box`）で
//! `index out of bounds: the len is 6 but the index is 8`。組み立ては
//! 内側シェル込みの並びで添字を作るのに、報告側が外側シェルだけを
//! 渡していたためです。**カーネルがパニックするのは、誤答より悪い**
//! ので、ここに固定します。
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
//! **断るのは赤にしません。** 3演算そろったものだけ突き合わせます。

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

/// **空洞のある立体を、別の形で切る。**
///
/// 2026/08/28 に、`hollow_box` は**あらゆる相手との3演算が断られて**
/// いました（4-144）。空洞の壁は実効法線が材料の中を向いていて、外側
/// シェルと規約が逆だったためです。切り手が空洞を貫くと壁が外側の境界に
/// 繋がるので、そこで巻きが食い違い「同方向の稜」が 24 件出ていました。
///
/// **中空部品は実務で普通に出てくる形**なので、ここに固定します。
/// **断るのは赤にしません**——3演算そろったものだけ突き合わせます。
#[test]
fn booleans_on_a_solid_with_a_cavity_hold_the_identities() {
    let tol = Tolerance::default();
    let Some(hollow) = load("hollow_box", &tol) else {
        return;
    };
    let hollow = centred(&hollow);

    let mut compared = 0usize;
    let mut failures: Vec<String> = Vec::new();

    for name in ["elliptic_prism", "sphere", "stepped_shaft"] {
        let Some(other) = load(name, &tol) else {
            continue;
        };
        let other = centred(&other);

        let volume_a = MassCalculator::compute_from_brep(&hollow, &params()).volume;
        let volume_b = MassCalculator::compute_from_brep(&other, &params()).volume;

        let run = |op| {
            BooleanEngine::boolean_solids_exact_result(&hollow, &other, op, &tol)
                .ok()
                .map(|result| volume(&result.solids))
        };
        let (Some(union), Some(meet), Some(cut)) = (
            run(BooleanOpType::Union),
            run(BooleanOpType::Intersection),
            run(BooleanOpType::Difference),
        ) else {
            continue;
        };
        compared += 1;

        let scale = volume_a.abs().max(1.0);
        let inclusion = ((union + meet) - (volume_a + volume_b)).abs() / scale;
        let split = ((cut + meet) - volume_a).abs() / scale;
        if inclusion > 1e-5 {
            failures.push(format!("hollow_box x {name}: 包除が {inclusion:.3e} ずれる"));
        }
        if split > 1e-5 {
            failures.push(format!("hollow_box x {name}: 分割が {split:.3e} ずれる"));
        }
    }

    assert!(failures.is_empty(), "{failures:#?}");
    assert!(
        compared >= 3,
        "空洞のある立体で3演算そろったのは {compared} 組でした。
         2026/08/28 の実測では 3 組（elliptic_prism / sphere / stepped_shaft）です。
         減ったなら、通っていたものが通らなくなっています（4-144、4-145）。"
    );

    // **`hollow_box` から円柱を引くのは、断るのが正しい配置です。**
    //
    // 中心を揃えると、半径 10 の円柱は箱の面 `y = ±10` に線で接します。
    // そこを抜くと材料は接点だけで2つに分かれるので、**真の答えが本当に
    // 非多様体**です。規約（3-N-1）どおり、**場所を名指しして断る**ことを
    // ここに固定します。
    //
    // **逆向き（`cylinder − hollow_box`）は通ります**（4-145）。順序で
    // 変わるのは、この配置では A の材料が割れるかどうかが順序で決まる
    // からです。
    let Some(cylinder) = load("cylinder", &tol) else {
        return;
    };
    let cylinder = centred(&cylinder);
    let refusal = BooleanEngine::boolean_solids_exact_result(
        &hollow,
        &cylinder,
        BooleanOpType::Difference,
        &tol,
    )
    .err();
    let refusal = refusal.expect(
        "hollow_box から円柱を引くと、材料が接点だけで2つに分かれます。
         真の答えが非多様体なので、返ってきたらそれは誤答です（3-N-1）。",
    );
    assert!(
        refusal.contains("non-manifold at"),
        "断るのは正しいのですが、**場所を名指ししていません**。
         規約は「答えが本当に非多様体なら、場所を名指して断る」です。
         実際の断り文: {refusal}"
    );

    // 逆向きは通ります。ここが断られるようになったら、4-145 が戻っています。
    for op in [
        BooleanOpType::Union,
        BooleanOpType::Intersection,
        BooleanOpType::Difference,
    ] {
        assert!(
            BooleanEngine::boolean_solids_exact_result(&cylinder, &hollow, op, &tol).is_ok(),
            "cylinder から見た {op:?} が断られました。4-145 が戻っています。"
        );
    }
}

/// 立体の中心を原点へ。**必ず重ねる**ため。
fn centred(solid: &Solid) -> Solid {
    let bbox = solid.bounding_box();
    let centre = Vec3::new(
        (bbox.min.x + bbox.max.x) * 0.5,
        (bbox.min.y + bbox.max.y) * 0.5,
        (bbox.min.z + bbox.max.z) * 0.5,
    );
    BrepTransform::translate_solid(solid, -centre)
}

/// **パニックしないこと**と、**返ったものが恒等式を満たすこと**。
#[test]
fn booleans_between_two_foreign_solids_hold_the_identities() {
    let tol = Tolerance::default();
    // **断られるだけの検体は、時間しか使いません。** `cone` と
    // `chamfered_box` は3演算とも断られたので外しました（実測 300秒 →
    // 下の実測）。**空洞のある2つは残します**——そこがパニックの現場です。
    let names = [
        "cylinder",
        "sphere",
        "torus",
        // **これがパニックしていた検体です**（空洞つき）。
        "hollow_box",
        "stepped_shaft",
        "plate_with_holes",
    ];

    let mut failures: Vec<String> = Vec::new();
    let mut compared = 0usize;
    let mut loaded = 0usize;

    for name in names {
        let Some(a) = load(name, &tol) else {
            continue;
        };
        loaded += 1;
        // 必ず重なるように、境界箱の対角ぶんだけずらす。
        let bbox = a.bounding_box();
        let span = bbox.max - bbox.min;
        let b = BrepTransform::translate_solid(
            &a,
            Vec3::new(span.x * 0.4, span.y * 0.12, span.z * 0.08),
        );

        let volume_a = MassCalculator::compute_from_brep(&a, &params()).volume;
        let volume_b = MassCalculator::compute_from_brep(&b, &params()).volume;

        // **ここで落ちないことが第一の主張です。**
        let union = BooleanEngine::boolean_solids_exact_result(&a, &b, BooleanOpType::Union, &tol)
            .ok()
            .map(|r| volume(&r.solids));
        let meet =
            BooleanEngine::boolean_solids_exact_result(&a, &b, BooleanOpType::Intersection, &tol)
                .ok()
                .map(|r| volume(&r.solids));
        let cut =
            BooleanEngine::boolean_solids_exact_result(&a, &b, BooleanOpType::Difference, &tol)
                .ok()
                .map(|r| volume(&r.solids));

        let (Some(union), Some(meet), Some(cut)) = (union, meet, cut) else {
            // 断るのは赤にしません。
            continue;
        };
        compared += 1;

        let scale = volume_a.abs().max(1.0);
        let inclusion_exclusion = ((union + meet) - (volume_a + volume_b)).abs() / scale;
        let split = ((cut + meet) - volume_a).abs() / scale;
        if inclusion_exclusion > 1e-5 {
            failures.push(format!(
                "{name}: |A u B| + |A n B| is off by {inclusion_exclusion:.3e} relative"
            ));
        }
        if split > 1e-5 {
            failures.push(format!(
                "{name}: |A minus B| + |A n B| is off by {split:.3e} relative"
            ));
        }
    }

    assert!(loaded >= 5, "only {loaded} fixtures could be read");
    assert!(
        compared >= 3,
        "only {compared} fixtures gave all three operations"
    );
    assert!(
        failures.is_empty(),
        "{} identity failure(s):\n{}",
        failures.len(),
        failures.join("\n")
    );
}
