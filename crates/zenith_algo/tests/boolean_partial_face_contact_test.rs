//! **面の一部だけが重なる配置。**
//!
//! 面の重なりの判定は「面全体の一致」（重心と広がり）で見ているので、
//! **部分的な重なりは捕まりません**——コードのコメントに前からそう
//! 書いてあります（4-124）。**では、どこまで通るのか。** 測りました。
//!
//! 結果は「通ります」でした（2026/08/28、4-140）。**通らないと思って
//! いたところが通っていた**ので、通ることのほうを固定します。
//!
//! - ブロックを横に並べて y と z にずらす（面 `x = 10` の一部だけが接する）
//! - 同軸の円柱で、側面や蓋が一部だけ重なる（差が2つに割れるものも含む）

use std::f64::consts::PI;

use zenith_algo::{BooleanEngine, BooleanOpType, BrepTransform, MassCalculator, PrimitiveBuilder};
use zenith_math::{Tolerance, Vec3};
use zenith_tess::TessellationParams;
use zenith_topo::Solid;

fn params() -> TessellationParams {
    TessellationParams {
        u_divisions: 32,
        v_divisions: 32,
    }
}

fn volume(solids: &[Solid]) -> f64 {
    solids
        .iter()
        .map(|solid| MassCalculator::compute_from_brep(solid, &params()).volume)
        .sum()
}

/// **ブロックを横に並べて、ずらす。** 面 `x = 10` の一部だけが接します。
///
/// 触れているだけなので、和は 2000（繋がって立体1つ）、差は 1000、
/// 積は空です。**どれだけずらしても変わりません。**
#[test]
fn two_blocks_sharing_part_of_a_face_combine_cleanly() {
    let tol = Tolerance::default();
    let base = PrimitiveBuilder::make_box(10.0, 10.0, 10.0).unwrap();
    let mut failures: Vec<String> = Vec::new();

    for (dy, dz) in [
        (0.0_f64, 0.0_f64),
        (5.0, 0.0),
        (5.0, 5.0),
        (9.0, 0.0),
        (2.5, 7.5),
    ] {
        let other = BrepTransform::translate_solid(&base, Vec3::new(10.0, dy, dz));
        for (label, op, expected) in [
            ("union", BooleanOpType::Union, 2000.0),
            ("difference", BooleanOpType::Difference, 1000.0),
            ("intersection", BooleanOpType::Intersection, 0.0),
        ] {
            match BooleanEngine::boolean_solids_exact_result(&base, &other, op, &tol) {
                Ok(result) => {
                    let measured = volume(&result.solids);
                    if (measured - expected).abs() > 2000.0 * 2e-3 {
                        failures.push(format!(
                            "shift (10,{dy},{dz}) / {label}: volume {measured} is not {expected}"
                        ));
                    }
                }
                Err(err) => {
                    failures.push(format!("shift (10,{dy},{dz}) / {label}: refused: {err}"))
                }
            }
        }
    }

    assert!(
        failures.is_empty(),
        "{} case(s) are wrong:\n{}",
        failures.len(),
        failures.join("\n")
    );
}

/// **同軸の円柱。** 側面や蓋が一部だけ重なります。
///
/// 1件目は**差が2つに割れます**——同じ半径の短い円柱を真ん中から抜くので、
/// 上下の円板が残ります。立体の数も見ます。
#[test]
fn coaxial_cylinders_sharing_part_of_a_surface_combine_cleanly() {
    let tol = Tolerance::default();
    let cylinder = |radius: f64, height: f64, z: f64| {
        BrepTransform::translate_solid(
            &PrimitiveBuilder::make_cylinder(radius, height).unwrap(),
            Vec3::new(0.0, 0.0, z),
        )
    };
    let mut failures: Vec<String> = Vec::new();

    // (名前, A, B, 和, 差, 積, 差の立体数)
    let cases: Vec<(&str, Solid, Solid, f64, f64, f64, usize)> = vec![
        (
            "same radius, B inside A along the axis",
            cylinder(5.0, 10.0, 0.0),
            cylinder(5.0, 4.0, 3.0),
            PI * 25.0 * 10.0,
            PI * 25.0 * 6.0,
            PI * 25.0 * 4.0,
            2,
        ),
        (
            "a thinner rod of the same length",
            cylinder(5.0, 10.0, 0.0),
            cylinder(3.0, 10.0, 0.0),
            PI * 25.0 * 10.0,
            PI * 16.0 * 10.0,
            PI * 9.0 * 10.0,
            1,
        ),
        (
            "a thinner rod sticking out both ends",
            cylinder(5.0, 10.0, 0.0),
            cylinder(3.0, 20.0, -5.0),
            PI * 25.0 * 10.0 + PI * 9.0 * 10.0,
            PI * 16.0 * 10.0,
            PI * 9.0 * 10.0,
            1,
        ),
        (
            "a short thinner rod buried inside",
            cylinder(5.0, 10.0, 0.0),
            cylinder(3.0, 4.0, 3.0),
            PI * 25.0 * 10.0,
            PI * 25.0 * 10.0 - PI * 9.0 * 4.0,
            PI * 9.0 * 4.0,
            1,
        ),
    ];

    for (name, a, b, union, difference, intersection, difference_solids) in cases {
        for (label, op, expected, want_solids) in [
            ("union", BooleanOpType::Union, union, 1usize),
            (
                "difference",
                BooleanOpType::Difference,
                difference,
                difference_solids,
            ),
            ("intersection", BooleanOpType::Intersection, intersection, 1),
        ] {
            match BooleanEngine::boolean_solids_exact_result(&a, &b, op, &tol) {
                Ok(result) => {
                    if result.solids.len() != want_solids {
                        failures.push(format!(
                            "{name} / {label}: {} solids, expected {want_solids}",
                            result.solids.len()
                        ));
                        continue;
                    }
                    let measured = volume(&result.solids);
                    if (measured - expected).abs() > expected * 3e-3 {
                        failures.push(format!(
                            "{name} / {label}: volume {measured} is not {expected}"
                        ));
                    }
                }
                Err(err) => failures.push(format!("{name} / {label}: refused: {err}")),
            }
        }
    }

    assert!(
        failures.is_empty(),
        "{} case(s) are wrong:\n{}",
        failures.len(),
        failures.join("\n")
    );
}
