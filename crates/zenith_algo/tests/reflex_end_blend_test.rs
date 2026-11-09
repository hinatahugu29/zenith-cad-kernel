//! 凹の角に端が来る凸の稜のフィレット・面取り（HANDOVER 4-940、`ZENITH_REFLEX_END`）。
//!
//! L字の切り欠きの上下の縁は、端が凹の縦稜に当たる。上面の隣の稜は退く向きと
//! 逆に走るので、詰めずに延ばす。端の面（もう一方の壁）は稜と直交するので、
//! 削れる体積は `L r^2 (1 - π/4)`（面取り `L c^2 / 2`）のまま。

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

#[allow(dead_code)]
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


/// 切り欠きの縁: 端の片方が凹の縦稜 (10, 15) に当たる、上下の4本と長さ
fn notch_rims(solid: &Solid) -> Vec<(u64, f64)> {
    let mut out: Vec<(u64, f64)> = Vec::new();
    for face in &solid.outer_shell.faces {
        for oriented in &face.outer_wire.edges {
            let a = oriented.edge.start_vertex.point;
            let b = oriented.edge.end_vertex.point;
            let flat = (a.z - b.z).abs() < 1e-9;
            let at = |p: Point3| (p.x - 10.0).abs() < 1e-9 && (p.y - 15.0).abs() < 1e-9;
            if flat && (at(a) || at(b)) && !out.iter().any(|(id, _)| *id == oriented.edge.id) {
                out.push((oriented.edge.id, (b - a).norm()));
            }
        }
    }
    out
}

#[test]
fn a_rim_ending_at_a_reflex_corner_is_blended() {
    std::env::set_var("ZENITH_REFLEX_END", "1");
    let solid = l_prism();
    let before = volume(&solid);
    let rims = notch_rims(&solid);
    assert_eq!(rims.len(), 4, "four notch rims");
    for (edge, length) in rims {
        for r in [1.0, 4.0] {
            let (result, report) =
                EdgeBlender::blend_edge(&solid, edge, BlendKind::Fillet { radius: r }).expect("fillet");
            let removed = before - volume(&result);
            let expected = length * r * r * (1.0 - PI / 4.0);
            assert!((report.predicted_removed_volume - expected).abs() < 1e-9 * expected);
            assert!((removed - expected).abs() < 1e-9 * before, "edge {edge} r={r}: removed {removed}, expected {expected}");
            assert_closed(&result, &format!("reflex fillet {edge} r={r}"));
        }
        let c = 3.0;
        let (result, _) = EdgeBlender::blend_edge(&solid, edge, BlendKind::Chamfer { distance: c }).expect("chamfer");
        let removed = before - volume(&result);
        assert!((removed - length * c * c * 0.5).abs() < 1e-9 * before, "chamfer removed {removed}");
        assert_closed(&result, "reflex chamfer");
    }
}

