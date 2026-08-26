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
