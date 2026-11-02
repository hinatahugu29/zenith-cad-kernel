//! 鎖の両端が**外周の同じ1本の稜の内側**に乗るときの `split_by_chain`
//! を最小の形で測る（4-852 の残り。`ring_rim_probe` の A面197 が
//! 1本の線を向きちがいで2回使う件）。
//!
//! 板の天面（20x10、z=5）を、y=0 の辺の x=5.3 と x=8 の間から立てた
//! 3辺の鎖（(5.3,0)->(5.3,3)->(8,3)->(8,0)）で割る。
//! 期待する形は「長方形 2.7x3」と「残り」の2枚で、**残りは
//! x 5.3..8 の辺を含みません**。

use zenith_algo::FaceSplitter;
use zenith_math::{Point3, Tolerance};
use zenith_topo::{Edge, Face, FaceGeometry, OrientedEdge, Orientation, Vertex, Wire};
use zenith_geom::PlaneSurface3;
use zenith_math::Vec3;

fn wire(points: &[Point3]) -> Wire {
    let vertices: Vec<Vertex> = points.iter().map(|p| Vertex::from_point(*p)).collect();
    let n = vertices.len();
    let edges = (0..n)
        .map(|i| {
            OrientedEdge::forward(
                Edge::line_between(vertices[i].clone(), vertices[(i + 1) % n].clone()).unwrap(),
            )
        })
        .collect();
    Wire::new(edges)
}

fn run(label: &str, outer_points: &[Point3], hole: bool) {
    let tol = Tolerance::default();
    println!("=== {label}");
    // y=0 の辺は 1 本（x 0..20）のまま。鎖の端はその内側に乗る。
    let outer = wire(outer_points);
    let plane = PlaneSurface3::new(
        Point3::new(0.0, 0.0, 5.0),
        Vec3::new(1.0, 0.0, 0.0),
        Vec3::new(0.0, 1.0, 0.0),
    )
    .expect("plane");
    let inner = if hole {
        vec![wire(&[
            Point3::new(7.0, 2.0, 5.0),
            Point3::new(7.0, 8.0, 5.0),
            Point3::new(13.0, 8.0, 5.0),
            Point3::new(13.0, 2.0, 5.0),
        ])]
    } else {
        Vec::new()
    };
    let face = Face::new(
        FaceGeometry::Plane(plane),
        outer,
        inner,
        Orientation::Forward,
        tol.linear,
    )
    ;

    let pts = [
        Point3::new(5.3, 0.0, 5.0),
        Point3::new(5.3, 3.0, 5.0),
        Point3::new(8.0, 3.0, 5.0),
        Point3::new(8.0, 0.0, 5.0),
    ];
    let chain: Vec<Edge> = (0..3)
        .map(|i| {
            Edge::line_between(
                Vertex::from_point(pts[i]),
                Vertex::from_point(pts[i + 1]),
            )
            .unwrap()
        })
        .collect();

    match FaceSplitter::split_by_chain(&face, &chain, &tol) {
        Ok((pieces, _report)) => {
            println!("片 {} 枚", pieces.len());
            for (index, piece) in pieces.iter().enumerate() {
                println!(
                    "  片{index}: 外周 {} 本、内側の輪 {} 個",
                    piece.outer_wire.edges.len(),
                    piece.inner_wires.len()
                );
                let mut seen: Vec<(Point3, Point3)> = Vec::new();
                for oriented in &piece.outer_wire.edges {
                    let (s, e) = match oriented.orientation {
                        Orientation::Forward => (
                            oriented.edge.start_vertex.point,
                            oriented.edge.end_vertex.point,
                        ),
                        Orientation::Reversed => (
                            oriented.edge.end_vertex.point,
                            oriented.edge.start_vertex.point,
                        ),
                    };
                    println!(
                        "    ({:.3},{:.3},{:.3}) -> ({:.3},{:.3},{:.3})",
                        s.x, s.y, s.z, e.x, e.y, e.z
                    );
                    seen.push((s, e));
                }
                for (wi, w) in piece.inner_wires.iter().enumerate() {
                    let pts: Vec<String> = w
                        .edges
                        .iter()
                        .map(|oe| {
                            let p = oe.edge.start_vertex.point;
                            format!("({:.2},{:.2})", p.x, p.y)
                        })
                        .collect();
                    println!("    内側の輪{wi}: {}", pts.join(" "));
                }
                let doubled = seen.iter().filter(|(s, e)| {
                    seen.iter().any(|(s2, e2)| (*s2 - *e).norm() < 1e-9 && (*e2 - *s).norm() < 1e-9)
                }).count();
                if doubled > 0 {
                    println!("    **向きちがいで2回使っている辺 {doubled} 本**");
                }
            }
        }
        Err(reason) => println!("断り — {reason}"),
    }
}

fn main() {
    let plain = [
        Point3::new(0.0, 0.0, 5.0),
        Point3::new(20.0, 0.0, 5.0),
        Point3::new(20.0, 10.0, 5.0),
        Point3::new(0.0, 10.0, 5.0),
    ];
    // y=0 の辺が、鎖の端のところで**すでに刻まれている**形。
    let imprinted = [
        Point3::new(0.0, 0.0, 5.0),
        Point3::new(5.3, 0.0, 5.0),
        Point3::new(8.0, 0.0, 5.0),
        Point3::new(20.0, 0.0, 5.0),
        Point3::new(20.0, 10.0, 5.0),
        Point3::new(0.0, 10.0, 5.0),
    ];
    run("1本の辺・穴なし", &plain, false);
    run("1本の辺・穴あり", &plain, true);
    run("刻まれた辺・穴なし", &imprinted, false);
    run("刻まれた辺・穴あり", &imprinted, true);
    println!("**これは診断です。何も直していません。**");
}
