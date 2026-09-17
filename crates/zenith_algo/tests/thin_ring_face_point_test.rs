//! **細い輪の面片が、自分の穴のまん中で内外を訊かれない**（4-477）。
//!
//! # なぜ要るのか
//!
//! 4-477 で平面 × トーラスを掃いたところ、**てっぺんを 1e-3 以下で
//! 切ると和だけが断られ**ました（`非多様体の稜の使用 24 件`）。
//!
//! 根は**面の中の代表点の選び方**でした。箱の底面はトーラスの切り口で
//! 3 つに割れます——外側・**細い輪**・島。**輪の代表点を探す格子は
//! 24 分割の固定**で、**幅 0.126・差し渡し 12 の輪には 1 点も入りません**。
//! 入らないと外周の平均に落ち、**それは穴のまん中**——トーラスの外です。
//! **輪が `Outside` と読まれ、和が面を二重に採って**いました。
//!
//! **輪の刻み（8 点）も同じ話**でした。半径 6.03 の円を 32 点で結ぶと
//! 弦のたわみが 0.029 で、**幅 0.057 の輪は多角形の段階で消えます**。
//! **刻みと格子を、見つかるまで一緒に細かく**しました。
//!
//! # 何を測るか
//!
//! **4-477 の時点で和が断られていた深さを 3 つ。**
//! **3 演算とも返り、3 つとも閉じた式（パップス）に乗ること。**

use zenith_algo::{BooleanEngine, BooleanOpType, BrepTransform, MassCalculator, PrimitiveBuilder};
use zenith_math::{Tolerance, Vec3};
use zenith_tess::TessellationParams;
use zenith_topo::Solid;

const R_MAJOR: f64 = 6.0;
const R_MINOR: f64 = 2.0;
const BOX_SIDE: f64 = 40.0;

fn volume_of(solids: &[Solid]) -> f64 {
    solids
        .iter()
        .map(|solid| MassCalculator::compute_from_brep(solid, &TessellationParams::default()).volume)
        .sum()
}

/// **`z ≥ z0` に残るトーラスの体積**（パップス）。
fn crown_volume(z0: f64) -> f64 {
    let r = R_MINOR;
    let segment = r * r * (z0 / r).acos() - z0 * (r * r - z0 * z0).sqrt();
    2.0 * std::f64::consts::PI * R_MAJOR * segment
}

#[test]
fn a_thin_ring_cut_of_a_torus_is_unioned_and_matches_the_closed_form() {
    let tol = Tolerance::default();
    let torus = 2.0 * std::f64::consts::PI.powi(2) * R_MAJOR * R_MINOR * R_MINOR;
    let box_volume = BOX_SIDE * BOX_SIDE * BOX_SIDE;

    for depth in [1e-4_f64, 5e-4, 1e-3] {
        let z0 = R_MINOR - depth;
        let crown = crown_volume(z0);

        let a = PrimitiveBuilder::make_torus(R_MAJOR, R_MINOR).expect("トーラス");
        let b = BrepTransform::translate_solid(
            &PrimitiveBuilder::make_box(BOX_SIDE, BOX_SIDE, BOX_SIDE).expect("箱"),
            Vec3::new(-BOX_SIDE * 0.5, -BOX_SIDE * 0.5, z0),
        );

        for (op, want) in [
            (BooleanOpType::Union, torus + box_volume - crown),
            (BooleanOpType::Intersection, crown),
            (BooleanOpType::Difference, torus - crown),
        ] {
            let solids = BooleanEngine::boolean_solids_exact_result(&a, &b, op, &tol)
                .map(|outcome| outcome.solids)
                .unwrap_or_else(|error| panic!("深さ {depth:e} の {op:?} が断られました: {error}"));
            let measured = volume_of(&solids);
            let relative = (measured - want).abs() / want.abs().max(1.0);
            assert!(
                relative <= 1e-4,
                "深さ {depth:e} の {op:?} が {measured:.9}、閉じた式は {want:.9}（相対差 {relative:.3e}）"
            );
        }
    }
}
