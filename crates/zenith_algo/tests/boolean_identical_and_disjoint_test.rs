//! **まったく同じ立体どうし**と、**完全に離れている立体どうし**。
//!
//! どちらも真の答えが閉じた式で書けます。
//!
//! - `A ∪ A = A`、`A \ A = ∅`、`A ∩ A = A`
//! - 離れていれば、和は立体2つ・体積は2つぶん、差は A、積は空
//!
//! ## なぜ要るのか
//!
//! 2026/08/28 まで、**`A ∪ A` が重なった立体を2つ返していました**
//! （球とトーラス。体積は2倍。4-134）。**検証ゲートも通ります**——和の
//! 体積は A より大きく、内外判定の 384 点もすべて一致するからです
//! （重なった2つの立体は、どの点から見ても「中は中」）。**位相だけが
//! 違い、位相を見る検査がありませんでした。**
//!
//! ここは**立体の数**も見ます。体積だけでは、重なった2つを見分けられません。

use std::f64::consts::PI;

use zenith_algo::{
    BooleanEngine, BooleanOpType, BrepTransform, MassCalculator, PrimitiveBuilder,
};
use zenith_math::{Tolerance, Vec3};
use zenith_tess::TessellationParams;
use zenith_topo::Solid;

fn params() -> TessellationParams {
    TessellationParams {
        u_divisions: 48,
        v_divisions: 48,
    }
}

fn shapes() -> Vec<(&'static str, Solid, f64)> {
    vec![
        (
            "box(10)",
            PrimitiveBuilder::make_box(10.0, 10.0, 10.0).unwrap(),
            1000.0,
        ),
        (
            "cylinder(r5,h10)",
            PrimitiveBuilder::make_cylinder(5.0, 10.0).unwrap(),
            PI * 25.0 * 10.0,
        ),
        (
            "sphere(r5)",
            PrimitiveBuilder::make_sphere(5.0).unwrap(),
            4.0 / 3.0 * PI * 125.0,
        ),
        (
            "cone(r5,h10)",
            PrimitiveBuilder::make_cone(5.0, 0.0, 10.0).unwrap(),
            PI * 25.0 * 10.0 / 3.0,
        ),
        (
            "torus(R8,r3)",
            PrimitiveBuilder::make_torus(8.0, 3.0).unwrap(),
            2.0 * PI * PI * 8.0 * 9.0,
        ),
    ]
}

fn volume(solids: &[Solid]) -> f64 {
    solids
        .iter()
        .map(|solid| MassCalculator::compute_from_brep(solid, &params()).volume)
        .sum()
}

/// **同じ立体どうしの和と積は、その立体そのものです。**
///
/// 差は現在いくつかの形で断られます（円柱と円錐）。**断るのは誤答では
/// ないので、ここでは赤にしません。** 返ってきたときだけ、空であることを
/// 確かめます。
#[test]
fn a_solid_combined_with_itself_gives_itself() {
    let tol = Tolerance::default();
    let mut failures: Vec<String> = Vec::new();

    for (name, solid, expected) in shapes() {
        // 和と積は**返らなければ赤**です。答えは立体そのもので、多様体です。
        for (label, op) in [
            ("union", BooleanOpType::Union),
            ("intersection", BooleanOpType::Intersection),
        ] {
            match BooleanEngine::boolean_solids_exact_result(&solid, &solid.clone(), op, &tol) {
                Ok(result) => {
                    // **立体の数を見ます。** 重なった2つでも体積は2倍に
                    // なるだけで、内外判定は通ってしまいます。
                    if result.solids.len() != 1 {
                        failures.push(format!(
                            "{name} / {label}: returned {} solids, expected 1",
                            result.solids.len()
                        ));
                        continue;
                    }
                    let measured = volume(&result.solids);
                    if (measured - expected).abs() > expected * 2e-3 {
                        failures.push(format!(
                            "{name} / {label}: volume {measured} is not {expected}"
                        ));
                    }
                }
                Err(err) => failures.push(format!("{name} / {label}: refused: {err}")),
            }
        }

        // 差は、返ったときだけ「空」を確かめます。
        if let Ok(result) =
            BooleanEngine::boolean_solids_exact_result(&solid, &solid.clone(), BooleanOpType::Difference, &tol)
        {
            let measured = volume(&result.solids);
            if measured.abs() > expected * 2e-3 {
                failures.push(format!(
                    "{name} / difference: volume {measured} should be empty"
                ));
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

/// **触れてもいない2つ**。和は立体2つ、差は A、積は空。
#[test]
fn two_solids_far_apart_do_not_interact() {
    let tol = Tolerance::default();
    let mut failures: Vec<String> = Vec::new();

    for (name, solid, expected) in shapes() {
        let away = BrepTransform::translate_solid(&solid, Vec3::new(100.0, 0.0, 0.0));
        for (label, op, want_volume, want_solids) in [
            ("union", BooleanOpType::Union, expected * 2.0, 2usize),
            ("difference", BooleanOpType::Difference, expected, 1),
            ("intersection", BooleanOpType::Intersection, 0.0, 0),
        ] {
            match BooleanEngine::boolean_solids_exact_result(&solid, &away, op, &tol) {
                Ok(result) => {
                    if result.solids.len() != want_solids {
                        failures.push(format!(
                            "{name} / {label}: returned {} solids, expected {want_solids}",
                            result.solids.len()
                        ));
                        continue;
                    }
                    let measured = volume(&result.solids);
                    if (measured - want_volume).abs() > want_volume.max(1.0) * 2e-3 {
                        failures.push(format!(
                            "{name} / {label}: volume {measured} is not {want_volume}"
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
