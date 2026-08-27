//! **半分だけ重ねる**——同じ形を、体積の半分ほどが重なる位置に置く。
//!
//! 真の答えは全部閉じた式です。等しい半径 `r` の球2つを距離 `d` 離した
//! ときの重なりは
//!
//! ```text
//! V = (pi / 12) (4r + d) (2r - d)^2
//! ```
//!
//! で、`d -> 0` で `4/3 pi r^3` に戻ります（この一致もテストします——
//! **期待値を出す道具が狂っていたら、その先の一致に意味はありません**）。
//!
//! あわせて**引数の順序**も見ます。和と積は可換なので、`A op B` と
//! `B op A` は同じ答えでなければなりません。**閉じた式が要らない検査**
//! なので、閉じた式が書けない配置にもそのまま使えます。

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

fn volume(solids: &[Solid]) -> f64 {
    solids
        .iter()
        .map(|solid| MassCalculator::compute_from_brep(solid, &params()).volume)
        .sum()
}

/// 等しい半径の球2つが、中心間距離 `d` で重なる体積。
fn sphere_lens(radius: f64, distance: f64) -> f64 {
    PI / 12.0 * (4.0 * radius + distance) * (2.0 * radius - distance).powi(2)
}

#[test]
fn the_lens_formula_agrees_with_the_whole_sphere_when_the_centres_meet() {
    let whole = 4.0 / 3.0 * PI * 125.0;
    let lens = sphere_lens(5.0, 0.0);
    assert!(
        (lens - whole).abs() / whole < 1e-12,
        "the lens formula should give the whole sphere at d = 0, got {lens} and {whole}"
    );
}

/// 「形、ずらし、体積、重なりの体積」。
fn cases() -> Vec<(&'static str, Solid, Vec3, f64, f64)> {
    vec![
        (
            "box(10) shifted 5 in x",
            PrimitiveBuilder::make_box(10.0, 10.0, 10.0).unwrap(),
            Vec3::new(5.0, 0.0, 0.0),
            1000.0,
            500.0,
        ),
        (
            "cylinder(r5,h10) shifted 5 along its axis",
            PrimitiveBuilder::make_cylinder(5.0, 10.0).unwrap(),
            Vec3::new(0.0, 0.0, 5.0),
            PI * 250.0,
            PI * 25.0 * 5.0,
        ),
        (
            "sphere(r5) shifted 5 in x",
            PrimitiveBuilder::make_sphere(5.0).unwrap(),
            Vec3::new(5.0, 0.0, 0.0),
            4.0 / 3.0 * PI * 125.0,
            sphere_lens(5.0, 5.0),
        ),
        (
            "sphere(r5) shifted 3 in x",
            PrimitiveBuilder::make_sphere(5.0).unwrap(),
            Vec3::new(3.0, 0.0, 0.0),
            4.0 / 3.0 * PI * 125.0,
            sphere_lens(5.0, 3.0),
        ),
    ]
}

#[test]
fn half_overlapping_duplicates_match_the_closed_form() {
    let tol = Tolerance::default();
    let mut failures: Vec<String> = Vec::new();

    for (name, solid, shift, whole, overlap) in cases() {
        let moved = BrepTransform::translate_solid(&solid, shift);
        for (label, op, expected) in [
            ("union", BooleanOpType::Union, 2.0 * whole - overlap),
            ("difference", BooleanOpType::Difference, whole - overlap),
            ("intersection", BooleanOpType::Intersection, overlap),
        ] {
            match BooleanEngine::boolean_solids_exact_result(&solid, &moved, op, &tol) {
                Ok(result) => {
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
    }

    assert!(
        failures.is_empty(),
        "{} case(s) are wrong:\n{}",
        failures.len(),
        failures.join("\n")
    );
}

/// **和と積は可換です。** 引数の順序で答えが変わってはいけません。
#[test]
fn union_and_intersection_do_not_depend_on_the_operand_order() {
    let tol = Tolerance::default();
    let mut failures: Vec<String> = Vec::new();

    for (name, solid, shift, _, _) in cases() {
        let moved = BrepTransform::translate_solid(&solid, shift);
        for (label, op) in [
            ("union", BooleanOpType::Union),
            ("intersection", BooleanOpType::Intersection),
        ] {
            let forward = BooleanEngine::boolean_solids_exact_result(&solid, &moved, op, &tol);
            let backward = BooleanEngine::boolean_solids_exact_result(&moved, &solid, op, &tol);
            match (forward, backward) {
                (Ok(a), Ok(b)) => {
                    if a.solids.len() != b.solids.len() {
                        failures.push(format!(
                            "{name} / {label}: {} solids one way, {} the other",
                            a.solids.len(),
                            b.solids.len()
                        ));
                        continue;
                    }
                    let (x, y) = (volume(&a.solids), volume(&b.solids));
                    if (x - y).abs() > x.abs().max(1.0) * 2e-3 {
                        failures.push(format!("{name} / {label}: {x} one way, {y} the other"));
                    }
                }
                // **片方だけ返るのも食い違いです。**
                (Ok(_), Err(err)) => {
                    failures.push(format!("{name} / {label}: B op A was refused: {err}"))
                }
                (Err(err), Ok(_)) => {
                    failures.push(format!("{name} / {label}: A op B was refused: {err}"))
                }
                (Err(_), Err(_)) => {}
            }
        }
    }

    assert!(
        failures.is_empty(),
        "{} case(s) depend on the operand order:\n{}",
        failures.len(),
        failures.join("\n")
    );
}
