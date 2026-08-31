//! **中に完全に入っている立体との3演算を、総当たりで測る。**
//!
//! 真の答えはすべて閉じた式で書けます——和 = 外側、差 = 外側 − 内側（空洞）、
//! 積 = 内側。**`volume > 0` では何も分かりません**（誤って足した答えにも
//! 体積はあります）。
//!
//! ## なぜ総当たりなのか
//!
//! 2026/08/28 まで、**通っていたのは「箱から小さい球を引く」1通りだけ**
//! でした（4-133）。空洞シェルの向きが揃っておらず、円錐からだと**触れても
//! いない**のに体積が `A + B` になります。1通りのテストが緑だったので、
//! ずっと通っていることになっていました。
//!
//! **1通りは「通っている」ではありません。** 外側4種 × 内側4種 × 3演算 =
//! 48通りを、全部閉じた式と突き合わせます。

use std::f64::consts::PI;

use zenith_algo::{BooleanEngine, BooleanOpType, BrepTransform, MassCalculator, PrimitiveBuilder};
use zenith_math::{Tolerance, Vec3};
use zenith_tess::TessellationParams;
use zenith_topo::Solid;

fn params() -> TessellationParams {
    TessellationParams {
        u_divisions: 48,
        v_divisions: 48,
    }
}

/// 外側になる立体と、その体積の閉じた式。
fn outers() -> Vec<(&'static str, Solid, f64)> {
    vec![
        (
            "box(40)",
            BrepTransform::translate_solid(
                &PrimitiveBuilder::make_box(40.0, 40.0, 40.0).unwrap(),
                Vec3::new(-20.0, -20.0, -20.0),
            ),
            64000.0,
        ),
        (
            "cylinder(r15,h40)",
            BrepTransform::translate_solid(
                &PrimitiveBuilder::make_cylinder(15.0, 40.0).unwrap(),
                Vec3::new(0.0, 0.0, -20.0),
            ),
            PI * 225.0 * 40.0,
        ),
        (
            "sphere(r15)",
            PrimitiveBuilder::make_sphere(15.0).unwrap(),
            4.0 / 3.0 * PI * 3375.0,
        ),
        (
            "cone(r20,h40)",
            BrepTransform::translate_solid(
                &PrimitiveBuilder::make_cone(20.0, 0.0, 40.0).unwrap(),
                Vec3::new(0.0, 0.0, -20.0),
            ),
            PI * 400.0 * 40.0 / 3.0,
        ),
    ]
}

/// 内側になる立体。**どれも原点まわりの小さい立体で、外側の中に完全に
/// 入っています**（どこにも触れません）。
fn inners() -> Vec<(&'static str, Solid, f64)> {
    vec![
        (
            "box(4)",
            BrepTransform::translate_solid(
                &PrimitiveBuilder::make_box(4.0, 4.0, 4.0).unwrap(),
                Vec3::new(-2.0, -2.0, -2.0),
            ),
            64.0,
        ),
        (
            "cylinder(r2,h4)",
            BrepTransform::translate_solid(
                &PrimitiveBuilder::make_cylinder(2.0, 4.0).unwrap(),
                Vec3::new(0.0, 0.0, -2.0),
            ),
            PI * 4.0 * 4.0,
        ),
        (
            "sphere(r2)",
            PrimitiveBuilder::make_sphere(2.0).unwrap(),
            4.0 / 3.0 * PI * 8.0,
        ),
        (
            "cone(r2,h4)",
            BrepTransform::translate_solid(
                &PrimitiveBuilder::make_cone(2.0, 0.0, 4.0).unwrap(),
                Vec3::new(0.0, 0.0, -2.0),
            ),
            PI * 4.0 * 4.0 / 3.0,
        ),
    ]
}

#[test]
fn every_contained_pair_matches_the_closed_form_for_all_three_operations() {
    let tol = Tolerance::default();
    let mut failures: Vec<String> = Vec::new();
    let mut checked = 0usize;

    for (outer_name, outer, outer_volume) in outers() {
        for (inner_name, inner, inner_volume) in inners() {
            for (label, op, expected) in [
                ("union", BooleanOpType::Union, outer_volume),
                (
                    "difference",
                    BooleanOpType::Difference,
                    outer_volume - inner_volume,
                ),
                ("intersection", BooleanOpType::Intersection, inner_volume),
            ] {
                checked += 1;
                match BooleanEngine::boolean_solids_exact_result(&outer, &inner, op, &tol) {
                    Ok(result) => {
                        let measured: f64 = result
                            .solids
                            .iter()
                            .map(|solid| MassCalculator::compute_from_brep(solid, &params()).volume)
                            .sum();
                        // メッシュの弦誤差ぶんは許します。**符号や取り違えは
                        // これでは通りません**（桁が違うので）。
                        if (measured - expected).abs() > expected.abs().max(1.0) * 2e-3 {
                            failures.push(format!(
                                "{outer_name} / {inner_name} / {label}: {measured} is not {expected}"
                            ));
                        }
                    }
                    Err(err) => failures.push(format!(
                        "{outer_name} / {inner_name} / {label}: refused: {err}"
                    )),
                }
            }
        }
    }

    assert_eq!(checked, 48, "the matrix should cover 4 x 4 x 3 cases");
    assert!(
        failures.is_empty(),
        "{} of {checked} contained cases are wrong:
{}",
        failures.len(),
        failures.join(
            "
"
        )
    );
}
