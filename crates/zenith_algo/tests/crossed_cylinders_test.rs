//! **長い円柱を直交させても、シュタインメッツが出る**（4-478）。
//!
//! # なぜ要るのか
//!
//! 4-478 で軸が平行でない円柱を掃いたところ、**長さが半径の 17 倍を
//! 超えると、積が空で返り**ました。**誤答です**——直交する同半径の
//! 円柱の重なりは `16r³/3` で、長さには依りません。
//!
//! 根は**種の探し方**でした。`find_seeds` の格子は**パラメータ空間で
//! 一様**なので、**面が長いほど、格子の目が世界の中で粗く**なります。
//! 長さ 36・半径 2 では目の間隔が 3——**交わりの幅 4 とほぼ同じ**で、
//! **種が 1 つも近づきません**。
//!
//! **格子を細かくすると `find_seeds` は格子の 4 乗で重くなる**ので、
//! **細かくせずに狭めました**——**交線は相手の外接箱の中にしか
//! 居られません**。
//!
//! # 何を測るか
//!
//! **長さを変えて、直交する円柱 2 本の重なりが `16r³/3` に乗ること。**
//! **短いほう（20）は前から通っていました**——**長いほう（36、160）が
//! 誤答でした**。

use zenith_algo::{BooleanEngine, BooleanOpType, BrepTransform, MassCalculator, PrimitiveBuilder};
use zenith_math::{Tolerance, Transform3, Vec3};
use zenith_tess::TessellationParams;
use zenith_topo::Solid;

const R: f64 = 2.0;

fn volume_of(solids: &[Solid]) -> f64 {
    solids
        .iter()
        .map(|solid| MassCalculator::compute_from_brep(solid, &TessellationParams::default()).volume)
        .sum()
}

fn lying_cylinder(length: f64, angle: f64) -> Solid {
    let upright = PrimitiveBuilder::make_cylinder(R, length).expect("円柱");
    let centred = BrepTransform::translate_solid(&upright, Vec3::new(0.0, 0.0, -length * 0.5));
    let lay = Transform3::from_axis_angle(&Vec3::y(), std::f64::consts::FRAC_PI_2);
    let turn = Transform3::from_axis_angle(&Vec3::z(), angle);
    BrepTransform::transform_solid(&centred, &turn.compose(&lay)).expect("回す")
}

#[test]
fn crossed_cylinders_give_the_steinmetz_volume_at_any_length() {
    let tol = Tolerance::default();
    let want = 16.0 * R.powi(3) / 3.0;

    for length in [20.0_f64, 36.0, 160.0] {
        let a = lying_cylinder(length, 0.0);
        let b = lying_cylinder(length, std::f64::consts::FRAC_PI_2);
        let solids = BooleanEngine::boolean_solids_exact_result(
            &a,
            &b,
            BooleanOpType::Intersection,
            &tol,
        )
        .map(|outcome| outcome.solids)
        .unwrap_or_else(|error| panic!("長さ {length} の積が断られました: {error}"));

        let measured = volume_of(&solids);
        let relative = (measured - want).abs() / want;
        assert!(
            relative <= 1e-5,
            "長さ {length} の積が {measured:.9}、閉じた式は {want:.9}（相対差 {relative:.3e}）"
        );
    }
}
