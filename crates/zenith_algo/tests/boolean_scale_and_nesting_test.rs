//! **桁の離れたスケール**と、**入れ子3段**。
//!
//! `boolean_validation` のコメントに「実務のアセンブリは桁の違う部品を
//! 含むので、これは効きます」とあります。ここはそれを測ります——辺 1e2
//! から 1e6 の箱の中に、半径 1 の球を置いて3演算。体積の比は最大で
//! **1e18 対 4.19** です。
//!
//! 入れ子は、空洞のある立体にさらに小さい立体を足す／掛けるところまで
//! 見ます。空洞の中に浮かぶ島は、**重ならない2つの立体**として返るのが
//! 正しい答えです。

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

/// **小さいほうの答えは、大きいほうの桁に埋もれてはいけません。**
///
/// 積は半径 1 の球（4.18879…）で、箱がどれだけ大きくても変わりません。
#[test]
fn a_tiny_tool_inside_a_huge_block_keeps_its_own_scale() {
    let tol = Tolerance::default();
    let ball = 4.0 / 3.0 * PI;
    let mut failures: Vec<String> = Vec::new();

    for big in [1e2_f64, 1e3, 1e4, 1e5, 1e6] {
        let outer = BrepTransform::translate_solid(
            &PrimitiveBuilder::make_box(big, big, big).unwrap(),
            Vec3::new(-big / 2.0, -big / 2.0, -big / 2.0),
        );
        let inner = PrimitiveBuilder::make_sphere(1.0).unwrap();
        let block = big * big * big;

        for (label, op, expected) in [
            ("union", BooleanOpType::Union, block),
            ("difference", BooleanOpType::Difference, block - ball),
            ("intersection", BooleanOpType::Intersection, ball),
        ] {
            match BooleanEngine::boolean_solids_exact_result(&outer, &inner, op, &tol) {
                Ok(result) => {
                    let measured = volume(&result.solids);
                    // **相対で見ます。** 1e18 の箱から 4.19 を引いた差は、
                    // 倍精度では箱そのものと同じ数です。そこは幾何ではなく
                    // 数の限界なので、相対で許します。積のほうは絶対に
                    // 効きます——箱がいくら大きくても 4.18879 のままです。
                    if (measured - expected).abs() > expected.abs().max(1.0) * 5e-3 {
                        failures.push(format!(
                            "box {big:e} / {label}: volume {measured} is not {expected}"
                        ));
                    }
                }
                Err(err) => failures.push(format!("box {big:e} / {label}: refused: {err}")),
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

/// **入れ子3段。** 空洞のある立体に、空洞の中の島を足す。
#[test]
fn a_cavity_can_hold_an_island() {
    let tol = Tolerance::default();
    let outer = BrepTransform::translate_solid(
        &PrimitiveBuilder::make_box(40.0, 40.0, 40.0).unwrap(),
        Vec3::new(-20.0, -20.0, -20.0),
    );
    let middle = BrepTransform::translate_solid(
        &PrimitiveBuilder::make_box(20.0, 20.0, 20.0).unwrap(),
        Vec3::new(-10.0, -10.0, -10.0),
    );
    let island = BrepTransform::translate_solid(
        &PrimitiveBuilder::make_box(8.0, 8.0, 8.0).unwrap(),
        Vec3::new(-4.0, -4.0, -4.0),
    );

    let shell = BooleanEngine::boolean_solids_exact_result(
        &outer,
        &middle,
        BooleanOpType::Difference,
        &tol,
    )
    .expect("a contained difference should give a cavity");
    assert_eq!(shell.solids.len(), 1);
    assert_eq!(shell.solids[0].inner_shells.len(), 1);
    let hollow = volume(&shell.solids);
    assert!(
        (hollow - 56000.0).abs() < 1.0,
        "the hollow block should be 56000, got {hollow}"
    );

    // **島は空洞の中に浮かびます。** 触れていないので、立体は2つです。
    let with_island = BooleanEngine::boolean_solids_exact_result(
        &shell.solids[0],
        &island,
        BooleanOpType::Union,
        &tol,
    )
    .expect("adding an island inside the cavity should work");
    assert_eq!(
        with_island.solids.len(),
        2,
        "the island does not touch the shell, so the result is two solids"
    );
    let total = volume(&with_island.solids);
    assert!(
        (total - 56512.0).abs() < 1.0,
        "hollow block plus island should be 56512, got {total}"
    );

    // 島は空洞（材料ではない）の中にいるので、積は空です。
    let overlap = BooleanEngine::boolean_solids_exact_result(
        &shell.solids[0],
        &island,
        BooleanOpType::Intersection,
        &tol,
    )
    .expect("the intersection should be empty, not refused");
    assert!(
        volume(&overlap.solids).abs() < 1.0,
        "the island sits in the void, so the intersection is empty"
    );
}
