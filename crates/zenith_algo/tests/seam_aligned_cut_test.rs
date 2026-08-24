//! 切る平面が、相手の立体の**継ぎ目にちょうど重なる**配置。
//!
//! # なぜ難しかったか
//!
//! 球を中心を通る平面で切ると、切り口の大円は球のパッチの**経線そのもの**に
//! なります。このとき、どのパッチも平面を内部で横切っていません——境界で
//! 接しているだけです。だから交線を辿る側は**何も見つけません**（実測: 16組
//! すべてで「0 branch」。HANDOVER 4-78）。
//!
//! 辿る必要はありませんでした。**交線は、相手が既に稜として持っています**
//! （実測: 球の8本の稜が `x = 20` の上に厳密に乗っている。長さは πr/2）。
//!
//! ここは「継ぎ目に重なっても答えが出ること」を、**閉じた式**で押さえます。
//! 半球の体積には式があるので、面数や位相だけでなく大きさで見られます。

use zenith_algo::{BooleanEngine, BooleanOpType, BrepTransform, MassCalculator, PrimitiveBuilder};
use zenith_math::{Tolerance, Vec3};
use zenith_tess::TessellationParams;
use zenith_topo::Solid;

fn volume(solids: &[Solid]) -> f64 {
    solids
        .iter()
        .map(|solid| {
            MassCalculator::compute_from_brep(
                solid,
                &TessellationParams {
                    u_divisions: 64,
                    v_divisions: 64,
                },
            )
            .volume
        })
        .sum()
}

/// メッシュの稜のうち、ちょうど2枚の三角形に共有されていない本数。
fn non_manifold_edges(solid: &Solid) -> usize {
    let mesh = zenith_tess::tessellate_solid(
        solid,
        &TessellationParams {
            u_divisions: 24,
            v_divisions: 24,
        },
    );
    let mut uses: std::collections::HashMap<(u32, u32), usize> = std::collections::HashMap::new();
    for triangle in &mesh.indices {
        for step in 0..3 {
            let (a, b) = (triangle[step], triangle[(step + 1) % 3]);
            if a == b {
                continue;
            }
            let key = if a < b { (a, b) } else { (b, a) };
            *uses.entry(key).or_insert(0) += 1;
        }
    }
    uses.values().filter(|count| **count != 2).count()
}

/// 箱 20³ と、面 `x = 20` に中心を置いた半径10の球。
///
/// 球は箱の y・z の幅にちょうど内接し、**切る平面は球の軸を含みます**。
/// だから切り口は極を通る大円で、球の経線と重なります。
#[test]
fn a_cut_that_lands_on_the_other_solids_seam_still_gives_the_closed_form() {
    let tol = Tolerance::default();
    let boxa = PrimitiveBuilder::make_box(20.0, 20.0, 20.0).expect("box");
    let sphere = BrepTransform::translate_solid(
        &PrimitiveBuilder::make_sphere(10.0).expect("sphere"),
        Vec3::new(20.0, 10.0, 10.0),
    );

    let ball = 4.0 / 3.0 * std::f64::consts::PI * 1000.0;
    let hemisphere = ball / 2.0;

    for (op, want) in [
        (BooleanOpType::Union, 8000.0 + ball - hemisphere),
        (BooleanOpType::Difference, 8000.0 - hemisphere),
        (BooleanOpType::Intersection, hemisphere),
    ] {
        let result = BooleanEngine::boolean_solids_exact_result(&boxa, &sphere, op, &tol)
            .unwrap_or_else(|err| panic!("{op:?} was refused: {err}"));
        assert_eq!(result.solids.len(), 1, "{op:?} should give one solid");

        let got = volume(&result.solids);
        assert!(
            (got - want).abs() <= want * 1e-9,
            "{op:?} volume {got} does not match the closed form {want}"
        );

        for solid in &result.solids {
            assert_eq!(
                non_manifold_edges(solid),
                0,
                "{op:?} returned a solid that is not manifold"
            );
        }
    }
}

// **回した球は、まだ通りません**（2026/08/25 実測。3-N）。
//
// 同じ球を自分の軸まわりに 45 度回すと、継ぎ目は平面から外れます。今度は
// 交線が4本の弧として辿れ、球のパッチも割れます（8 applied）。それでも
// **縫合で 16 本の稜が相手を見つけられません**。継ぎ目に重なる配置とは
// 別の欠陥で、こちらは未解決です。
//
// **ここにテストは置きません。** 通らないものを assert すると、赤が
// 常設になります。再現は HANDOVER 4-80 にあります。
