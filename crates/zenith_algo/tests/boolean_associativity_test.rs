//! **3つの立体の結合則。** `(A op B) op C` と `A op (B op C)` は同じ答えの
//! はずです（和と積）。
//!
//! **閉じた式が要らない検査です。** 答えが分からない形にもそのまま使えます。
//!
//! ## 何を赤にするか
//!
//! **両方返ったのに答えが違う**ときだけです。片方が断られるのは赤に
//! しません——2回目の演算は1回目の結果（面の増えた立体）に当たるので、
//! 順序によって未実装の経路に入ります。実測（4-138、2026/08/28）:
//! 4組 × 2演算のうち **4件が「片方だけ返る」**で、**数値の食い違いは 0**
//! でした。断り文はどれも「まだ実装していない」です。
//!
//! **「片方だけ返る」は、実務では効きます**——履歴の順序を入れ替えただけで
//! 通らなくなる、ということだからです。数として残しておきます。

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

/// 1回目の結果が立体1つのときだけ、2回目に進む。
fn chain(first: &Solid, second: &Solid, third: &Solid, op: BooleanOpType) -> Option<f64> {
    let tol = Tolerance::default();
    let middle = BooleanEngine::boolean_solids_exact_result(first, second, op, &tol).ok()?;
    if middle.solids.len() != 1 {
        return None;
    }
    let result =
        BooleanEngine::boolean_solids_exact_result(&middle.solids[0], third, op, &tol).ok()?;
    Some(volume(&result.solids))
}

fn triples() -> Vec<(&'static str, Solid, Solid, Solid)> {
    vec![
        (
            "box, box, box",
            PrimitiveBuilder::make_box(10.0, 10.0, 10.0).unwrap(),
            BrepTransform::translate_solid(
                &PrimitiveBuilder::make_box(10.0, 10.0, 10.0).unwrap(),
                Vec3::new(5.0, 0.0, 0.0),
            ),
            BrepTransform::translate_solid(
                &PrimitiveBuilder::make_box(10.0, 10.0, 10.0).unwrap(),
                Vec3::new(2.5, 5.0, 0.0),
            ),
        ),
        (
            "box, cylinder, sphere",
            PrimitiveBuilder::make_box(10.0, 10.0, 10.0).unwrap(),
            BrepTransform::translate_solid(
                &PrimitiveBuilder::make_cylinder(3.0, 20.0).unwrap(),
                Vec3::new(5.0, 5.0, -5.0),
            ),
            BrepTransform::translate_solid(
                &PrimitiveBuilder::make_sphere(4.0).unwrap(),
                Vec3::new(10.0, 5.0, 5.0),
            ),
        ),
        (
            "sphere, sphere, sphere",
            PrimitiveBuilder::make_sphere(5.0).unwrap(),
            BrepTransform::translate_solid(
                &PrimitiveBuilder::make_sphere(5.0).unwrap(),
                Vec3::new(4.0, 0.0, 0.0),
            ),
            BrepTransform::translate_solid(
                &PrimitiveBuilder::make_sphere(5.0).unwrap(),
                Vec3::new(2.0, 4.0, 0.0),
            ),
        ),
        (
            "box, cone, cylinder",
            PrimitiveBuilder::make_box(20.0, 20.0, 20.0).unwrap(),
            BrepTransform::translate_solid(
                &PrimitiveBuilder::make_cone(6.0, 0.0, 15.0).unwrap(),
                Vec3::new(10.0, 10.0, 10.0),
            ),
            BrepTransform::translate_solid(
                &PrimitiveBuilder::make_cylinder(4.0, 40.0).unwrap(),
                Vec3::new(5.0, 5.0, -10.0),
            ),
        ),
    ]
}

#[test]
fn chaining_three_solids_does_not_depend_on_the_grouping() {
    let mut failures: Vec<String> = Vec::new();
    let mut compared = 0usize;

    for (name, a, b, c) in triples() {
        for (label, op) in [
            ("union", BooleanOpType::Union),
            ("intersection", BooleanOpType::Intersection),
        ] {
            // (A op B) op C
            let left = chain(&a, &b, &c, op);
            // A op (B op C) — 同じ関数で、括弧の位置だけ変える。
            let right = chain(&b, &c, &a, op);
            if let (Some(x), Some(y)) = (left, right) {
                compared += 1;
                if (x - y).abs() > x.abs().max(1.0) * 2e-3 {
                    failures.push(format!(
                        "{name} / {label}: (A op B) op C = {x}, A op (B op C) = {y}"
                    ));
                }
            }
        }
    }

    assert!(
        compared >= 3,
        "at least three groupings should be comparable, only {compared} were"
    );
    assert!(
        failures.is_empty(),
        "{} case(s) depend on the grouping:\n{}",
        failures.len(),
        failures.join("\n")
    );
}
