//! ブーリアンの結果が、稜を実体として共有した B-Rep になっているか。
//!
//! 閉じたシェルであることは以前から確認できていました。確認できていな
//! かったのは、**同じ稜が両側の面から同じ実体として参照されているか**です。
//! 面片を1枚ずつ作る組み立てでは、同じ位置に別々の `Edge` が並んだまま
//! 出てきます。座標で見る閉性検査はそれを通してしまい、通った立体には
//! 「この稜を共有する2面」が引けません。稜を選ぶ演算子（フィレット・
//! 面取り・履歴）は、そこで全部止まります。

use std::collections::BTreeMap;

use zenith_algo::{
    BooleanEngine, BooleanOpType, BrepTransform, MassCalculator, PrimitiveBuilder, Sewer,
};
use zenith_math::{Tolerance, Vec3};
use zenith_tess::TessellationParams;
use zenith_topo::Solid;

fn volume_of(solid: &Solid) -> f64 {
    MassCalculator::compute_from_brep(
        solid,
        &TessellationParams {
            u_divisions: 48,
            v_divisions: 48,
        },
    )
    .volume
}

/// 稜 ID ごとの参照回数
fn edge_use_counts(solid: &Solid) -> BTreeMap<u64, usize> {
    let mut counts = BTreeMap::new();
    for face in &solid.outer_shell.faces {
        for wire in std::iter::once(&face.outer_wire).chain(face.inner_wires.iter()) {
            for oriented in &wire.edges {
                *counts.entry(oriented.edge.id).or_insert(0) += 1;
            }
        }
    }
    counts
}

/// 同じ位置にいくつの別 ID が並んでいるか
fn duplicate_edge_entities(solid: &Solid) -> usize {
    let quantize = |p: zenith_math::Point3| {
        (
            (p.x * 1e6).round() as i64,
            (p.y * 1e6).round() as i64,
            (p.z * 1e6).round() as i64,
        )
    };
    let mut by_place: BTreeMap<((i64, i64, i64), (i64, i64, i64)), Vec<u64>> = BTreeMap::new();
    for face in &solid.outer_shell.faces {
        for wire in std::iter::once(&face.outer_wire).chain(face.inner_wires.iter()) {
            for oriented in &wire.edges {
                let a = quantize(oriented.edge.start_vertex.point);
                let b = quantize(oriented.edge.end_vertex.point);
                let key = if a <= b { (a, b) } else { (b, a) };
                let slot = by_place.entry(key).or_default();
                if !slot.contains(&oriented.edge.id) {
                    slot.push(oriented.edge.id);
                }
            }
        }
    }
    by_place.values().map(|ids| ids.len() - 1).sum()
}

fn assert_shared(solid: &Solid, what: &str) {
    let counts = edge_use_counts(solid);
    let unshared: Vec<u64> = counts
        .iter()
        .filter(|(_, count)| **count != 2)
        .map(|(id, _)| *id)
        .collect();
    assert!(
        unshared.is_empty(),
        "{what}: {} of {} edges are not shared by exactly two faces",
        unshared.len(),
        counts.len()
    );
    assert_eq!(
        duplicate_edge_entities(solid),
        0,
        "{what}: the same place still carries more than one edge entity"
    );
}

#[test]
fn a_difference_comes_back_with_every_edge_shared_by_two_faces() {
    let tol = Tolerance::default();
    let base = PrimitiveBuilder::make_box(40.0, 40.0, 20.0).unwrap();
    let cutter = BrepTransform::translate_solid(
        &PrimitiveBuilder::make_box(20.0, 20.0, 20.0).unwrap(),
        Vec3::new(20.0, 20.0, 0.0),
    );

    let result = BooleanEngine::boolean_solids_exact(&base, &cutter, BooleanOpType::Difference, &tol)
        .expect("difference");

    assert_shared(&result, "box minus corner box");
    let volume = volume_of(&result);
    let expected = (40.0 * 40.0 - 20.0 * 20.0) * 20.0;
    assert!(
        (volume - expected).abs() / expected < 1e-12,
        "the sewn result measures {volume} against {expected}"
    );
}

#[test]
fn an_intersection_comes_back_with_every_edge_shared_by_two_faces() {
    let tol = Tolerance::default();
    let base = PrimitiveBuilder::make_box(40.0, 40.0, 20.0).unwrap();
    let other = BrepTransform::translate_solid(
        &PrimitiveBuilder::make_box(40.0, 40.0, 20.0).unwrap(),
        Vec3::new(20.0, 20.0, 10.0),
    );

    let result =
        BooleanEngine::boolean_solids_exact(&base, &other, BooleanOpType::Intersection, &tol)
            .expect("intersection");

    assert_shared(&result, "two overlapping boxes");
    assert_eq!(
        edge_use_counts(&result).len(),
        12,
        "the overlap of two boxes is a box, so twelve edges"
    );
    let volume = volume_of(&result);
    assert!(
        (volume - 20.0 * 20.0 * 10.0).abs() < 1e-9,
        "the sewn overlap measures {volume}"
    );
}

#[test]
fn a_union_stays_shared_and_keeps_its_volume() {
    let tol = Tolerance::default();
    let base = PrimitiveBuilder::make_box(40.0, 40.0, 20.0).unwrap();
    let block = BrepTransform::translate_solid(
        &PrimitiveBuilder::make_box(20.0, 20.0, 30.0).unwrap(),
        Vec3::new(10.0, 10.0, 20.0),
    );

    let result =
        BooleanEngine::boolean_solids_exact(&base, &block, BooleanOpType::Union, &tol)
            .expect("union");

    assert_shared(&result, "box with a block on top");
    let volume = volume_of(&result);
    let expected = 40.0 * 40.0 * 20.0 + 20.0 * 20.0 * 30.0;
    assert!(
        (volume - expected).abs() / expected < 1e-12,
        "the union measures {volume} against {expected}"
    );
}

#[test]
fn sewing_an_already_shared_solid_changes_nothing() {
    let tol = Tolerance::default();
    for solid in [
        PrimitiveBuilder::make_box(10.0, 20.0, 30.0).unwrap(),
        PrimitiveBuilder::make_cylinder(6.0, 15.0).unwrap(),
        PrimitiveBuilder::make_regular_prism(6, 10.0, 25.0).unwrap(),
    ] {
        let before = volume_of(&solid);
        let counts_before = edge_use_counts(&solid);

        let (sewn, report) = Sewer::sew_solid(&solid, &tol).expect("sew");

        assert_eq!(
            report.edges_before, report.edges_after,
            "a builder's own output has nothing to merge: {}",
            report.summary()
        );
        assert!(report.is_watertight(), "{}", report.summary());
        assert_eq!(edge_use_counts(&sewn).len(), counts_before.len());

        let after = volume_of(&sewn);
        assert!(
            (after - before).abs() / before < 1e-15,
            "sewing moved the volume: {after} against {before}"
        );
    }
}

#[test]
fn sewing_reports_how_many_entities_it_merged() {
    // 面ごとに別々の稜を持つ状態を、ブーリアンの生の出力から作る。
    let tol = Tolerance::default();
    let base = PrimitiveBuilder::make_box(40.0, 40.0, 20.0).unwrap();
    let cutter = BrepTransform::translate_solid(
        &PrimitiveBuilder::make_box(20.0, 20.0, 20.0).unwrap(),
        Vec3::new(20.0, 20.0, 0.0),
    );
    let raw = BooleanEngine::boolean_solids_exact_result_unverified(
        &base,
        &cutter,
        BooleanOpType::Difference,
        &tol,
    )
    .expect("raw difference")
    .solids
    .into_iter()
    .next()
    .expect("one solid");

    let raw_counts = edge_use_counts(&raw);
    let unshared_before = raw_counts.values().filter(|count| **count != 2).count();
    assert!(
        unshared_before > 0,
        "this test needs the raw pipeline to still hand back unshared edges"
    );

    let (sewn, report) = Sewer::sew_solid(&raw, &tol).expect("sew");
    assert!(
        report.edges_after < report.edges_before,
        "nothing was merged: {}",
        report.summary()
    );
    assert!(report.is_watertight(), "{}", report.summary());
    assert_shared(&sewn, "sewn raw difference");

    let before = volume_of(&raw);
    let after = volume_of(&sewn);
    assert!(
        (after - before).abs() / before < 1e-14,
        "sewing moved the volume: {after} against {before}"
    );
}
