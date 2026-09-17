//! **小さいけれど真っ当な平面の面を、潰れていると言わない**（4-476）。
//!
//! # なぜ要るのか
//!
//! 4-476 で円錐の頂点の近くを掃いたところ、**深さ 1e-4 で積と差が
//! 断られ**ました。理由は
//! `planar p-curve outer loop is degenerate; area 7.79e-9`。
//!
//! **その面は潰れていません。** 半径 5e-5 の真っ当な円板で、面積が
//! π(5e-5)² = 7.85e-9 だっただけです。**面積を、長さの緩み
//! （`tol.parametric` = 1e-7）と直に比べていた**ので、**小さい面ほど
//! 潰れて見えて**いました。**潰れているとは「差し渡しに対して幅が
//! 無い」こと**なので、枡をループ自身の差し渡しに合わせました。
//!
//! # 何を測るか
//!
//! **底面半径 5・高さ 10 の円錐を、頂点から 1e-4 下で切る**。
//! **3 演算とも返り、3 つとも閉じた式（相似な小円錐）に乗ること。**

use zenith_algo::{BooleanEngine, BooleanOpType, BrepTransform, MassCalculator, PrimitiveBuilder};
use zenith_math::{Tolerance, Vec3};
use zenith_tess::TessellationParams;
use zenith_topo::Solid;

const RB: f64 = 5.0;
const H: f64 = 10.0;
const BOX_SIDE: f64 = 40.0;

fn volume_of(solids: &[Solid]) -> f64 {
    solids
        .iter()
        .map(|solid| MassCalculator::compute_from_brep(solid, &TessellationParams::default()).volume)
        .sum()
}

#[test]
fn a_tiny_cone_tip_is_cut_and_matches_the_closed_form() {
    let tol = Tolerance::default();
    let cone = std::f64::consts::PI * RB * RB * H / 3.0;
    let box_volume = BOX_SIDE * BOX_SIDE * BOX_SIDE;

    // **4-476 の時点で断られていた深さ**。1e-5 より浅い所は、
    // 切り口の半径が線形公差の数倍しかなく、まだ返りません。
    for depth in [1e-4_f64, 3e-4, 1e-3] {
        let z0 = H - depth;
        let r = RB * depth / H;
        let tip = std::f64::consts::PI * r * r * depth / 3.0;

        let a = PrimitiveBuilder::make_cone(RB, 0.0, H).expect("円錐");
        let b = BrepTransform::translate_solid(
            &PrimitiveBuilder::make_box(BOX_SIDE, BOX_SIDE, BOX_SIDE).expect("箱"),
            Vec3::new(-BOX_SIDE * 0.5, -BOX_SIDE * 0.5, z0),
        );

        for (op, want) in [
            (BooleanOpType::Union, cone + box_volume - tip),
            (BooleanOpType::Intersection, tip),
            (BooleanOpType::Difference, cone - tip),
        ] {
            let solids = BooleanEngine::boolean_solids_exact_result(&a, &b, op, &tol)
                .map(|outcome| outcome.solids)
                .unwrap_or_else(|error| panic!("深さ {depth:e} の {op:?} が断られました: {error}"));
            let measured = volume_of(&solids);
            let relative = (measured - want).abs() / want.abs().max(1.0);
            assert!(
                relative <= 1e-5,
                "深さ {depth:e} の {op:?} が {measured:.9}、閉じた式は {want:.9}（相対差 {relative:.3e}）"
            );
        }
    }
}
