//! **球どうしの薄い重なりで、3 演算が通るか**（4-471）。
//!
//! # なぜ要るのか
//!
//! 4-468 が、**半径 5 と 3 の球を薄く重ねると差が断られる**所を
//! 見つけました。4-469 で **0.0005 刻みで 44 点**掃くと、**帯ではなく
//! 「接するほど断りが増える勾配」**だと分かりました（断り 22 点）。
//!
//! 根は **交線の重複**でした（4-471）——**辿って出した交線が、端点は
//! ぴたり同じ、中点だけ 5.7e-6 違う弧を 2 本**出しており、
//! **2 本目が 1 本目の割った跡に乗って「割れなかった」**と数えられて
//! いました。**面へ配る前にまとめる**ようにして、**断り 22 → 3**。
//!
//! **閉じたものは、開いたときに鳴るようにしておきます。**
//!
//! # 何を測るか
//!
//! **4-469 の時点で断られていた食い込みを 4 つ。**
//!
//! **3 演算とも返ること**と、**3 つとも閉じた式に乗ること**。
//! **「返ったこと」だけでは足りません**（4-462 の誤答は返っていました）。

use zenith_algo::{
    BooleanEngine, BooleanOpType, BrepTransform, MassCalculator, PrimitiveBuilder,
};
use zenith_math::{Tolerance, Vec3};
use zenith_tess::TessellationParams;
use zenith_topo::Solid;

const R1: f64 = 5.0;
const R2: f64 = 3.0;

fn volume_of(solids: &[Solid]) -> f64 {
    solids
        .iter()
        .map(|solid| MassCalculator::compute_from_brep(solid, &TessellationParams::default()).volume)
        .sum()
}

/// **2 つの球が重なる体積**。
fn lens_volume(d: f64) -> f64 {
    let head = std::f64::consts::PI * (R1 + R2 - d).powi(2);
    let tail = d * d + 2.0 * d * R2 - 3.0 * R2 * R2 + 2.0 * d * R1 + 6.0 * R1 * R2 - 3.0 * R1 * R1;
    head * tail / (12.0 * d)
}

#[test]
fn thin_overlaps_between_spheres_match_the_closed_form() {
    let tol = Tolerance::default();
    let ball_a = 4.0 / 3.0 * std::f64::consts::PI * R1.powi(3);
    let ball_b = 4.0 / 3.0 * std::f64::consts::PI * R2.powi(3);

    // **4-469 の時点で、差が断られていた 4 つ**（2.05e-2、1.8e-2、
    // 1.5e-2、1.25e-2）。**1.15e-2 より浅い所は、まだ残っています**
    // ので入れていません（4-471）。
    for overlap in [2.05e-2_f64, 1.8e-2, 1.5e-2, 1.25e-2] {
        let d = R1 + R2 - overlap;
        let a = PrimitiveBuilder::make_sphere(R1).expect("大きい球");
        let b = BrepTransform::translate_solid(
            &PrimitiveBuilder::make_sphere(R2).expect("小さい球"),
            Vec3::new(d, 0.0, 0.0),
        );

        let lens = lens_volume(d);
        for (op, want) in [
            (BooleanOpType::Union, ball_a + ball_b - lens),
            (BooleanOpType::Intersection, lens),
            (BooleanOpType::Difference, ball_a - lens),
        ] {
            let solids = BooleanEngine::boolean_solids_exact_result(&a, &b, op, &tol)
                .map(|outcome| outcome.solids)
                .unwrap_or_else(|error| {
                    panic!("食い込み {overlap:e} の {op:?} が断られました: {error}")
                });
            let measured = volume_of(&solids);
            let relative = (measured - want).abs() / want.abs().max(1.0);
            assert!(
                relative <= 1e-5,
                "食い込み {overlap:e} の {op:?} が {measured:.9}、閉じた式は {want:.9}（相対差 {relative:.3e}）"
            );
        }
    }
}
