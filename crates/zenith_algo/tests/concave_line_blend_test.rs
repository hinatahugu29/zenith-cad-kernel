//! 凹の直線稜のフィレット・面取り（ROADMAP 柱⑦、HANDOVER 4-938、`ZENITH_CONCAVE_LINE_BLEND`）。
//!
//! L字の角柱の内側の縦稜を丸めると、材料が**足される**。足される体積は
//! 90度の凹稜で `L r^2 (1 - π/4)`（面取りは `L c^2 / 2`）。

use std::f64::consts::PI;

use zenith_algo::{BlendKind, EdgeBlender, ExtrudeBuilder, MassCalculator};
use zenith_math::{Point3, Tolerance, Vec3};
use zenith_tess::{tessellate_solid, TessellationParams};
use zenith_topo::{Edge, OrientedEdge, Solid, Vertex, Wire};

fn volume(solid: &Solid) -> f64 {
    MassCalculator::compute_volume_from_brep(solid, &TessellationParams { u_divisions: 64, v_divisions: 64 })
}

/// 20×30 から x 10..20, y 15..30 を欠いた L字を高さ 40 押し出す。凹の縦稜は (10, 15)。
fn l_prism() -> Solid {
    let corners = [(0.0, 0.0), (20.0, 0.0), (20.0, 15.0), (10.0, 15.0), (10.0, 30.0), (0.0, 30.0)];
    let vertices: Vec<Vertex> = corners.iter().map(|(x, y)| Vertex::from_point(Point3::new(*x, *y, 0.0))).collect();
    let edges = (0..vertices.len())
        .map(|i| OrientedEdge::forward(Edge::line_between(vertices[i].clone(), vertices[(i + 1) % vertices.len()].clone()).unwrap()))
        .collect();
    ExtrudeBuilder::extrude_wire(&Wire::new(edges), Vec3::new(0.0, 0.0, 40.0), &Tolerance::default()).expect("L prism")
}

fn concave_edge(solid: &Solid) -> u64 {
    solid
        .outer_shell
        .faces
        .iter()
        .flat_map(|face| face.outer_wire.edges.iter())
        .find(|oriented| {
            let a = oriented.edge.start_vertex.point;
            let b = oriented.edge.end_vertex.point;
            (a.x - 10.0).abs() < 1e-9 && (a.y - 15.0).abs() < 1e-9 && (b.x - 10.0).abs() < 1e-9 && (b.y - 15.0).abs() < 1e-9
        })
        .map(|oriented| oriented.edge.id)
        .expect("concave edge")
}

fn assert_closed(solid: &Solid, what: &str) {
    assert!(solid.outer_shell.validate_closed(&Tolerance::default()).is_valid(), "{what}: open B-Rep");
    for divisions in [8, 16, 32] {
        let mesh = tessellate_solid(solid, &TessellationParams { u_divisions: divisions, v_divisions: divisions });
        let mut uses: std::collections::HashMap<(u32, u32), usize> = Default::default();
        for t in &mesh.indices {
            for k in 0..3 {
                let (a, b) = (t[k], t[(k + 1) % 3]);
                if a != b {
                    *uses.entry((a.min(b), a.max(b))).or_insert(0) += 1;
                }
            }
        }
        assert_eq!(uses.values().filter(|c| **c != 2).count(), 0, "{what}: open mesh at {divisions}");
    }
}

#[test]
fn a_concave_edge_fillet_adds_the_closed_form_volume() {
    std::env::set_var("ZENITH_CONCAVE_LINE_BLEND", "1");
    let solid = l_prism();
    let before = volume(&solid);
    assert!((before - 18000.0).abs() < 1e-6);
    let edge = concave_edge(&solid);
    for r in [0.5, 3.0, 9.9] {
        let (result, report) = EdgeBlender::blend_edge(&solid, edge, BlendKind::Fillet { radius: r }).expect("fillet");
        let added = volume(&result) - before;
        let expected = 40.0 * r * r * (1.0 - PI / 4.0);
        assert!((report.predicted_removed_volume + expected).abs() < 1e-9 * expected);
        assert!((added - expected).abs() < 1e-9 * before, "r={r}: added {added}, expected {expected}");
        assert_closed(&result, &format!("concave fillet r={r}"));
    }
    // 隣の稜（長さ 10）を食い切る大きさは断る
    assert!(EdgeBlender::fillet_edge(&solid, edge, 10.0).is_err());
}

#[test]
fn a_concave_edge_chamfer_adds_the_closed_form_volume() {
    std::env::set_var("ZENITH_CONCAVE_LINE_BLEND", "1");
    let solid = l_prism();
    let before = volume(&solid);
    let edge = concave_edge(&solid);
    let c = 4.0;
    let (result, _) = EdgeBlender::blend_edge(&solid, edge, BlendKind::Chamfer { distance: c }).expect("chamfer");
    let added = volume(&result) - before;
    assert!((added - 40.0 * c * c * 0.5).abs() < 1e-9 * before, "added {added}");
    assert_closed(&result, "concave chamfer");
}
