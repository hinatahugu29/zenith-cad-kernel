//! **頂点のまわりが 1 周か**を、鳴らして確かめる（4-450）。
//!
//! # なぜこの 4 本か
//!
//! **鳴らない検査は、置いても意味がありません。** 4-450 でこの検査を
//! 入れたとき、**既存の検体では 1 度も鳴りませんでした**——**門も試験も
//! 全部緑のまま**です。
//!
//! **それは「効いている」証拠ではありません。** **いつも 0 を返す関数**
//! でも、同じ結果になります。
//!
//! **だから、鳴る形をこちらで組みます。**

use zenith_algo::{BrepTransform, PrimitiveBuilder};
use zenith_math::{Tolerance, Vec3};
use zenith_topo::Shell;

/// 角だけで触れる 2 つの箱を、**1 つの殻**に入れる。
fn two_boxes_touching_at_a_corner() -> Shell {
    let first = PrimitiveBuilder::make_box(10.0, 10.0, 10.0).expect("箱");
    let second = BrepTransform::translate_solid(
        &PrimitiveBuilder::make_box(10.0, 10.0, 10.0).expect("箱"),
        Vec3::new(10.0, 10.0, 10.0),
    );
    let mut faces = first.outer_shell.faces;
    faces.extend(second.outer_shell.faces);
    Shell::new(faces, true)
}

#[test]
fn an_ordinary_box_has_no_pinched_vertex() {
    let tol = Tolerance::default();
    let solid = PrimitiveBuilder::make_box(10.0, 10.0, 10.0).expect("箱");
    let report = solid.outer_shell.validate_closed(&tol);
    assert_eq!(
        report.pinched_vertex_count, 0,
        "ただの箱で鳴ってはいけません: {:?}",
        report.errors
    );
    assert!(report.is_valid(), "ただの箱が通りません: {:?}", report.errors);
}

#[test]
fn two_boxes_meeting_at_one_corner_pinch_that_corner() {
    let tol = Tolerance::default();
    // **稜の数え方では、これは通ります**——どの稜も 2 回ずつ使われて
    // います。**繋がっているのが稜ではなく点**だからです。
    let report = two_boxes_touching_at_a_corner().validate_closed(&tol);
    assert_eq!(
        report.unmatched_edge_use_count, 0,
        "稜は全部相手がいるはずです: {:?}",
        report.errors
    );
    assert_eq!(
        report.non_manifold_edge_use_count, 0,
        "稜は 2 回ずつのはずです: {:?}",
        report.errors
    );
    assert_eq!(
        report.pinched_vertex_count, 1,
        "触れている角が 1 つ鳴るはずです: {:?}",
        report.errors
    );
    assert!(
        report
            .errors
            .iter()
            .any(|error| error.contains("10.000000 10.000000 10.000000")),
        "鳴った場所を名指ししていません: {:?}",
        report.errors
    );
}

#[test]
fn a_shell_with_a_pinched_vertex_is_not_valid() {
    let tol = Tolerance::default();
    assert!(
        !two_boxes_touching_at_a_corner()
            .validate_closed(&tol)
            .is_valid(),
        "つままれた殻が通ってしまいます"
    );
}

#[test]
fn two_boxes_that_do_not_touch_have_no_pinched_vertex() {
    let tol = Tolerance::default();
    // **離れている 2 つの塊**。**1 つの殻に入っていても、頂点は 1 周**
    // です——**「塊が 2 つある」ことと「頂点がつままれている」ことは
    // 別**です。**ここを混ぜる物差しは、使えません。**
    let first = PrimitiveBuilder::make_box(10.0, 10.0, 10.0).expect("箱");
    let second = BrepTransform::translate_solid(
        &PrimitiveBuilder::make_box(10.0, 10.0, 10.0).expect("箱"),
        Vec3::new(50.0, 50.0, 50.0),
    );
    let mut faces = first.outer_shell.faces;
    faces.extend(second.outer_shell.faces);
    let report = Shell::new(faces, true).validate_closed(&tol);
    assert_eq!(
        report.pinched_vertex_count, 0,
        "離れた 2 つで鳴ってはいけません: {:?}",
        report.errors
    );
}
