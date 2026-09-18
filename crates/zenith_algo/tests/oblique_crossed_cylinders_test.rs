//! **斜めに交わる円柱も、閉じた式に乗る**（4-480）。
//!
//! # なぜ要るのか
//!
//! 4-478 で軸が平行でない円柱の根を**種の格子**まで測りましたが、
//! **直したのは軸に平行な置き方だけ**でした。**斜め（75°〜20°）は
//! 「未実装」のまま**です。
//!
//! 4-478 の窓は**軸に平行な箱**で作ります。**斜めに寝た長い円柱では、
//! 箱がほとんど空**です——長さ 160 を 45° に倒すと、箱は一辺 113 の
//! 立方体に近づき、2 つ重ねても狭くなりません。
//!
//! **箱をやめて、距離で当たりを決める**ようにしました——`find_seeds` の
//! **粗い当たりの上位 3 か所について、その周りをもう一度見ます**
//! （8 × 8 を、近づかなくなるまで最大 4 段）。
//!
//! # 何を測るか
//!
//! **同半径の円柱 2 本の軸が角度 θ で交わるとき、重なりは
//! `16r³/(3 sin θ)`。** **4-478 の時点で断られていた角度**を当てます。

use zenith_algo::{BooleanEngine, BooleanOpType, BrepTransform, MassCalculator, PrimitiveBuilder};
use zenith_math::{Tolerance, Transform3, Vec3};
use zenith_tess::TessellationParams;
use zenith_topo::Solid;

const R: f64 = 2.0;
const LENGTH: f64 = 160.0;

fn volume_of(solids: &[Solid]) -> f64 {
    solids
        .iter()
        .map(|solid| MassCalculator::compute_from_brep(solid, &TessellationParams::default()).volume)
        .sum()
}

fn lying_cylinder(angle: f64) -> Solid {
    let upright = PrimitiveBuilder::make_cylinder(R, LENGTH).expect("円柱");
    let centred = BrepTransform::translate_solid(&upright, Vec3::new(0.0, 0.0, -LENGTH * 0.5));
    let lay = Transform3::from_axis_angle(&Vec3::y(), std::f64::consts::FRAC_PI_2);
    let turn = Transform3::from_axis_angle(&Vec3::z(), angle);
    BrepTransform::transform_solid(&centred, &turn.compose(&lay)).expect("回す")
}

#[test]
fn obliquely_crossed_cylinders_match_the_closed_form() {
    let tol = Tolerance::default();

    // **4-478 の時点で「未実装」だった 5 つ**（75° 〜 20°）。
    for degrees in [75.0_f64, 60.0, 45.0, 30.0, 20.0] {
        let theta = degrees.to_radians();
        let want = 16.0 * R.powi(3) / (3.0 * theta.sin());

        let a = lying_cylinder(0.0);
        let b = lying_cylinder(theta);
        let solids =
            BooleanEngine::boolean_solids_exact_result(&a, &b, BooleanOpType::Intersection, &tol)
                .map(|outcome| outcome.solids)
                .unwrap_or_else(|error| panic!("角度 {degrees} の積が断られました: {error}"));

        let measured = volume_of(&solids);
        let relative = (measured - want).abs() / want;
        assert!(
            relative <= 1e-5,
            "角度 {degrees} の積が {measured:.9}、閉じた式は {want:.9}（相対差 {relative:.3e}）"
        );
    }
}
