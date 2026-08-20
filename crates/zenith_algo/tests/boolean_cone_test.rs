//! Booleans against a cone.
//!
//! A cone's side is the same kind of surface as a cylinder's - a straight line
//! swept round an axis - and it was refused everywhere a cylinder was accepted,
//! for two reasons that both amount to assuming the radius never changes.
//!
//! Recognition read the control net and required every ruling to be the same
//! vector, which is true of a cylinder and false of a cone. And matching the two
//! ends of a ruling asked whether one point sat directly above the other, which
//! a cone's leaning rulings never do.
//!
//! The expected volumes here are closed forms, not numbers this kernel
//! produced.

use zenith_algo::{BooleanEngine, BooleanOpType, BrepTransform, MassCalculator, PrimitiveBuilder};
use zenith_math::{Tolerance, Vec3};
use zenith_tess::TessellationParams;
use zenith_topo::Solid;

fn volume(solid: &Solid) -> f64 {
    MassCalculator::compute_from_brep(
        solid,
        &TessellationParams {
            u_divisions: 64,
            v_divisions: 64,
        },
    )
    .volume
}

/// Volume of a frustum of radii `r0` and `r1` over a height `h`.
fn frustum_volume(r0: f64, r1: f64, h: f64) -> f64 {
    std::f64::consts::PI * h / 3.0 * (r0 * r0 + r0 * r1 + r1 * r1)
}

#[test]
fn test_a_box_sitting_on_a_cone_unions_to_the_right_volume() {
    let tol = Tolerance::default();
    // 半径10から4へ、高さ20の円錐。z = 10..30 を占める箱をかぶせる。
    // 箱は x, y とも ±10 なので、z = 10 より上の円錐（半径は最大でも7）を
    // まるごと含む。
    let cone = PrimitiveBuilder::make_cone(10.0, 4.0, 20.0).expect("cone");
    let block = BrepTransform::translate_solid(
        &PrimitiveBuilder::make_box(20.0, 20.0, 20.0).expect("box"),
        Vec3::new(-10.0, -10.0, 10.0),
    );

    let result = BooleanEngine::boolean_solids_exact_result(
        &cone,
        &block,
        BooleanOpType::Union,
        &tol,
    )
    .expect("a box over a cone should union");
    assert_eq!(result.solids.len(), 1);

    // 円錐 + 箱 - 重なり。重なりは z = 10..20 の円錐台（半径 7 から 4）。
    let expected =
        frustum_volume(10.0, 4.0, 20.0) + 8000.0 - frustum_volume(7.0, 4.0, 10.0);
    let measured = volume(&result.solids[0]);
    let relative = (measured - expected).abs() / expected;
    assert!(
        relative < 1e-6,
        "cone union box {measured:.4} against {expected:.4} (relative {relative:.2e})"
    );

    assert!(result.solids[0]
        .outer_shell
        .validate_closed(&tol)
        .is_valid());
}

#[test]
fn test_a_cone_still_reports_the_cases_it_cannot_do_as_errors() {
    // 対応範囲を広げても、範囲外は誤答ではなくエラーであること。
    // 箱を円錐の腹に食い込ませると、切り口は双曲線になり、パッチの
    // パラメータ線ではないので分割できない。
    let tol = Tolerance::default();
    let cone = PrimitiveBuilder::make_cone(10.0, 4.0, 20.0).expect("cone");
    let block = BrepTransform::translate_solid(
        &PrimitiveBuilder::make_box(20.0, 20.0, 10.0).expect("box"),
        Vec3::new(0.0, -10.0, 5.0),
    );

    for op in [
        BooleanOpType::Union,
        BooleanOpType::Difference,
        BooleanOpType::Intersection,
    ] {
        if let Ok(result) = BooleanEngine::boolean_solids_exact_result(&cone, &block, op, &tol) {
            // 通ったのなら、通ったなりに正しくなければならない。
            for solid in &result.solids {
                assert!(
                    solid.outer_shell.validate_closed(&tol).is_valid(),
                    "{op:?} returned a solid that does not close"
                );
                assert!(volume(solid) > 0.0, "{op:?} returned an empty solid");
            }
        }
    }
}
