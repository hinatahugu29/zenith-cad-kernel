//! **曲面どうしの薄い重なりで、3 演算が通るか**（4-465）。
//!
//! # なぜ要るのか
//!
//! 4-459 が、**半径 5 と 3 の平行な円柱を、食い込み 2e-3 より薄く
//! 重ねると断られる**帯を見つけました。**交線は全部正しく**、
//! 落ちていたのは**内外の分類**です。
//!
//! **その途中で、もっと悪いものが出ました**（4-462）——
//! **差が「A そのもの」（切り込みの無い立体）を返していました。**
//! **体積は 471.238898、閉じた式は 471.238408。** **もっともらしい
//! 誤答**で、**門は緑のまま**でした。
//!
//! 4-465 で閉じました。**閉じたものは、開いたときに鳴るように
//! しておきます。**
//!
//! # 何を測るか
//!
//! **食い込みを 4 つ**（1.8e-3、1.0e-3、6e-4、2e-4——**4-462 の
//! 時点で断られるか、誤答を返していた所**）。
//!
//! **3 演算とも返ること**と、**3 つとも閉じた式に乗ること**。
//!
//! **「返ったこと」だけでは足りません。** **4-462 の誤答は、
//! 返っていました。**

use zenith_algo::{
    BooleanEngine, BooleanOpType, BrepTransform, MassCalculator, PrimitiveBuilder,
};
use zenith_math::{Tolerance, Vec3};
use zenith_tess::TessellationParams;
use zenith_topo::Solid;

const R1: f64 = 5.0;
const R2: f64 = 3.0;
const HEIGHT: f64 = 6.0;

fn volume_of(solids: &[Solid]) -> f64 {
    solids
        .iter()
        .map(|solid| MassCalculator::compute_from_brep(solid, &TessellationParams::default()).volume)
        .sum()
}

/// **2 つの円が重なる面積**（レンズ）。
fn lens_area(d: f64) -> f64 {
    let first = R1 * R1
        * ((d * d + R1 * R1 - R2 * R2) / (2.0 * d * R1))
            .clamp(-1.0, 1.0)
            .acos();
    let second = R2 * R2
        * ((d * d + R2 * R2 - R1 * R1) / (2.0 * d * R2))
            .clamp(-1.0, 1.0)
            .acos();
    let root = ((-d + R1 + R2) * (d + R1 - R2) * (d - R1 + R2) * (d + R1 + R2)).max(0.0);
    first + second - 0.5 * root.sqrt()
}

#[test]
fn thin_overlaps_between_parallel_cylinders_match_the_closed_form() {
    let tol = Tolerance::default();
    let disc_a = std::f64::consts::PI * R1 * R1 * HEIGHT;
    let disc_b = std::f64::consts::PI * R2 * R2 * HEIGHT;

    // **4-462 の時点で、断られるか誤答を返していた 4 つ。**
    for overlap in [1.8e-3_f64, 1.0e-3, 6e-4, 2e-4] {
        let d = R1 + R2 - overlap;
        let a = PrimitiveBuilder::make_cylinder(R1, HEIGHT).expect("大きい円柱");
        let b = BrepTransform::translate_solid(
            &PrimitiveBuilder::make_cylinder(R2, HEIGHT).expect("小さい円柱"),
            Vec3::new(d, 0.0, 0.0),
        );

        let lens = lens_area(d) * HEIGHT;
        for (op, want) in [
            (BooleanOpType::Union, disc_a + disc_b - lens),
            (BooleanOpType::Intersection, lens),
            (BooleanOpType::Difference, disc_a - lens),
        ] {
            let solids = BooleanEngine::boolean_solids_exact_result(&a, &b, op, &tol)
                .map(|outcome| outcome.solids)
                .unwrap_or_else(|error| {
                    panic!("食い込み {overlap:e} の {op:?} が断られました: {error}")
                });
            let measured = volume_of(&solids);
            let relative = (measured - want).abs() / want.abs().max(1.0);
            // **差が「A そのもの」を返していたときの相対差は 1.0e-6**
            // でした（4-462）。**1e-7 で見張ります。**
            assert!(
                relative <= 1e-7,
                "食い込み {overlap:e} の {op:?} が {measured:.9}、閉じた式は {want:.9}（相対差 {relative:.3e}）"
            );
        }
    }
}
