use std::collections::BTreeMap;

use zenith_algo::{BooleanEngine, BooleanOpType, BrepTransform, EdgeBlender, PrimitiveBuilder};
use zenith_math::{Tolerance, Vec3};
use zenith_topo::Solid;

fn rectangular_boss() -> Solid {
    let tolerance = Tolerance::default();
    let base = PrimitiveBuilder::make_box(40.0, 40.0, 20.0).expect("base");
    let boss = BrepTransform::translate_solid(
        &PrimitiveBuilder::make_box(12.0, 10.0, 12.0).expect("boss"),
        Vec3::new(14.0, 15.0, 20.0),
    );
    BooleanEngine::boolean_solids_exact_simplified(&base, &boss, BooleanOpType::Union, &tolerance)
        .expect("rectangular boss union")
}

fn distinct_edges(solid: &Solid) -> BTreeMap<u64, zenith_topo::Edge> {
    let mut edges = BTreeMap::new();
    for face in &solid.outer_shell.faces {
        for wire in std::iter::once(&face.outer_wire).chain(face.inner_wires.iter()) {
            for oriented in &wire.edges {
                edges
                    .entry(oriented.edge.id)
                    .or_insert_with(|| oriented.edge.clone());
            }
        }
    }
    edges
}

#[test]
fn rectangular_boss_root_edges_report_concave_contract_without_mutation() {
    let solid = rectangular_boss();
    let before = solid.clone();
    let root_edges: Vec<_> = distinct_edges(&solid)
        .into_values()
        .filter(|edge| {
            let start = edge.start_vertex.point;
            let end = edge.end_vertex.point;
            (start.z - 20.0).abs() < 1e-9
                && (end.z - 20.0).abs() < 1e-9
                && [start.x, end.x].iter().all(|x| (14.0..=26.0).contains(x))
                && [start.y, end.y].iter().all(|y| (15.0..=25.0).contains(y))
        })
        .collect();

    assert_eq!(root_edges.len(), 4, "expected the four boss-root edges");
    for edge in &root_edges {
        let error = EdgeBlender::blendability(&solid, edge.id).expect_err("concave root");
        assert!(
            error.contains("270.000 deg interior angle")
                && error.contains("only convex edges are blended"),
            "unexpected diagnostic for edge {}: {error}",
            edge.id
        );
        assert!(
            EdgeBlender::chamfer_edge(&solid, edge.id, 1.0).is_err(),
            "unsupported root must not produce an approximate solid"
        );
    }

    assert_eq!(
        solid, before,
        "diagnostics and refused edits must be non-mutating"
    );
    let listed = EdgeBlender::blendable_edges(&solid);
    assert!(
        root_edges
            .iter()
            .all(|root| listed.iter().all(|candidate| candidate.edge_id != root.id)),
        "unsupported roots must not appear in the selectable-edge list"
    );
    assert!(solid
        .outer_shell
        .validate_closed(&Tolerance::default())
        .is_valid());
}

#[test]
fn blendability_matches_the_existing_candidate_listing() {
    let solid = PrimitiveBuilder::make_box(20.0, 30.0, 40.0).expect("box");
    let listed = EdgeBlender::blendable_edges(&solid);
    assert_eq!(listed.len(), 12);

    for candidate in listed {
        assert_eq!(
            EdgeBlender::blendability(&solid, candidate.edge_id).expect("listed edge"),
            candidate
        );
    }
    let missing = EdgeBlender::blendability(&solid, u64::MAX).expect_err("missing edge");
    assert!(missing.contains("is not in this solid"));
}

#[test]
fn stepped_shaft_blendability_diagnostics() {
    let tolerance = Tolerance::default();
    let lower = PrimitiveBuilder::make_cylinder(15.0, 20.0).expect("lower cylinder");
    let upper = BrepTransform::translate_solid(
        &PrimitiveBuilder::make_cylinder(8.0, 15.0).expect("upper cylinder"),
        Vec3::new(0.0, 0.0, 20.0),
    );
    let shaft = BooleanEngine::boolean_solids_exact_simplified(
        &lower,
        &upper,
        BooleanOpType::Union,
        &tolerance,
    )
    .expect("stepped shaft union");

    let before = shaft.clone();
    let listed = EdgeBlender::blendable_edges(&shaft);

    // 全ての blendable 候補に対して個別 blendability が一致すること
    for candidate in &listed {
        let direct = EdgeBlender::blendability(&shaft, candidate.edge_id).expect("blendable edge");
        assert_eq!(*candidate, direct);
        assert!(candidate.max_fillet_radius > 0.0);
        assert!(candidate.max_chamfer_distance > 0.0);
    }

    // 凹肩エッジ（Z=20.0, r=8.0）の診断確認
    let concave_shoulder_edges: Vec<_> = distinct_edges(&shaft)
        .into_values()
        .filter(|edge| {
            let start = edge.start_vertex.point;
            let end = edge.end_vertex.point;
            (start.z - 20.0).abs() < 1e-9
                && (end.z - 20.0).abs() < 1e-9
                && ((start.x.hypot(start.y) - 8.0).abs() < 1e-6
                    || (end.x.hypot(end.y) - 8.0).abs() < 1e-6)
        })
        .collect();

    assert_eq!(
        concave_shoulder_edges.len(),
        4,
        "stepped shaft shoulder should have 4 quarter-circle edges"
    );

    for edge in &concave_shoulder_edges {
        let diag = EdgeBlender::blendability(&shaft, edge.id)
            .expect("concave circular shoulder must be blendable");
        assert_eq!(diag.edge_id, edge.id);
        assert!((diag.dihedral_angle_deg - 270.0).abs() < 1e-6);
        assert!(diag.max_fillet_radius > 0.0);
        assert!(diag.max_chamfer_distance > 0.0);
    }

    // 実際に1本の肩エッジをフィレットして有効なソリッドが得られること
    let blend_target = concave_shoulder_edges[0].id;
    let filleted = EdgeBlender::fillet_edge(&shaft, blend_target, 1.5)
        .expect("shoulder root fillet should succeed");
    assert!(filleted.outer_shell.validate_closed(&tolerance).is_valid());

    // 診断呼び出しによる非破壊性の確認
    assert_eq!(shaft, before, "blendability must not mutate solid");
}

#[test]
fn hybrid_bosses_blendability_diagnostics() {
    let tolerance = Tolerance::default();
    let base = PrimitiveBuilder::make_box(60.0, 40.0, 20.0).expect("base box");
    let cyl_boss = BrepTransform::translate_solid(
        &PrimitiveBuilder::make_cylinder(6.0, 15.0).expect("cylinder boss"),
        Vec3::new(15.0, 20.0, 20.0),
    );
    let rect_boss = BrepTransform::translate_solid(
        &PrimitiveBuilder::make_box(10.0, 10.0, 15.0).expect("rectangular boss"),
        Vec3::new(40.0, 15.0, 20.0),
    );

    let with_cyl = BooleanEngine::boolean_solids_exact_simplified(
        &base,
        &cyl_boss,
        BooleanOpType::Union,
        &tolerance,
    )
    .expect("union cyl boss");

    let solid = BooleanEngine::boolean_solids_exact_simplified(
        &with_cyl,
        &rect_boss,
        BooleanOpType::Union,
        &tolerance,
    )
    .expect("union rect boss");

    let before = solid.clone();
    let all_edges = distinct_edges(&solid);

    // 円筒ボスの凹根元エッジ（Z=20.0, 中心 (15, 20), 半径 6.0）
    let cyl_root_edges: Vec<_> = all_edges
        .values()
        .filter(|edge| {
            let start = edge.start_vertex.point;
            let end = edge.end_vertex.point;
            (start.z - 20.0).abs() < 1e-9
                && (end.z - 20.0).abs() < 1e-9
                && (((start.x - 15.0).hypot(start.y - 20.0) - 6.0).abs() < 1e-6
                    || ((end.x - 15.0).hypot(end.y - 20.0) - 6.0).abs() < 1e-6)
        })
        .collect();

    assert_eq!(
        cyl_root_edges.len(),
        4,
        "cylinder boss root should have 4 edges"
    );
    for edge in &cyl_root_edges {
        let diag = EdgeBlender::blendability(&solid, edge.id)
            .expect("circular cylinder root must be blendable");
        assert_eq!(diag.edge_id, edge.id);
        assert!((diag.dihedral_angle_deg - 270.0).abs() < 1e-6);
        assert!(diag.max_fillet_radius > 0.0);
    }

    // 矩形ボスの凹根元エッジ（Z=20.0, X in 40..=50, Y in 15..=25）
    let rect_root_edges: Vec<_> = all_edges
        .values()
        .filter(|edge| {
            let start = edge.start_vertex.point;
            let end = edge.end_vertex.point;
            (start.z - 20.0).abs() < 1e-9
                && (end.z - 20.0).abs() < 1e-9
                && [start.x, end.x].iter().all(|x| (40.0..=50.0).contains(x))
                && [start.y, end.y].iter().all(|y| (15.0..=25.0).contains(y))
        })
        .collect();

    assert_eq!(
        rect_root_edges.len(),
        4,
        "rect boss root should have 4 edges"
    );
    for edge in &rect_root_edges {
        let err = EdgeBlender::blendability(&solid, edge.id)
            .expect_err("rect boss root must be rejected with diagnostic");
        assert!(
            err.contains("270.000 deg interior angle")
                && err.contains("only convex edges are blended"),
            "unexpected error for rect root edge {}: {err}",
            edge.id
        );
    }

    // blendable_edges の列挙と blendability の結果が完全一致すること
    let listed = EdgeBlender::blendable_edges(&solid);
    assert!(
        cyl_root_edges
            .iter()
            .all(|cyl| listed.iter().any(|candidate| candidate.edge_id == cyl.id)),
        "circular root must be listed in selectable candidates"
    );
    assert!(
        rect_root_edges
            .iter()
            .all(|rect| listed.iter().all(|candidate| candidate.edge_id != rect.id)),
        "rect root must not be listed in selectable candidates"
    );

    assert_eq!(solid, before, "blendability must not mutate solid");
}
