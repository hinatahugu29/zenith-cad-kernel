//! **板を厚くする**口が、**縁の弧と内側の輪**を落としていないことを見る。
//!
//! 4-409 で、`thicken_planar_face` は外周の稜の**始点だけ**を拾って直線で
//! 結び直しており、**内側の輪は一度も読んでいません**でした。どちらも
//! **閉じた多様体で返る**ので、形の検査では捕まりません——**大きさだけが
//! 違います**。ここは閉じた式に当てます。

use std::f64::consts::PI;

use zenith_algo::{MassCalculator, ProfileBuilder, ThickenBuilder};
use zenith_geom::PlaneSurface3;
use zenith_math::{Point3, Tolerance, Vec3};
use zenith_tess::TessellationParams;
use zenith_topo::{Edge, Face, FaceGeometry, Orientation, OrientedEdge, Vertex, Wire};

fn xy_plane() -> PlaneSurface3 {
    PlaneSurface3::new(
        Point3::new(0.0, 0.0, 0.0),
        Vec3::new(1.0, 0.0, 0.0),
        Vec3::new(0.0, 1.0, 0.0),
    )
    .unwrap()
}

fn planar_face(outer: Wire, inner: Vec<Wire>) -> Face {
    Face::new(
        FaceGeometry::Plane(xy_plane()),
        outer,
        inner,
        Orientation::Forward,
        1e-6,
    )
}

fn rect_wire(x0: f64, y0: f64, w: f64, h: f64) -> Wire {
    let pts = [
        Point3::new(x0, y0, 0.0),
        Point3::new(x0 + w, y0, 0.0),
        Point3::new(x0 + w, y0 + h, 0.0),
        Point3::new(x0, y0 + h, 0.0),
    ];
    let verts: Vec<Vertex> = pts.iter().map(|p| Vertex::from_point(*p)).collect();
    let mut edges = Vec::with_capacity(4);
    for index in 0..4 {
        let next = (index + 1) % 4;
        edges.push(OrientedEdge::forward(
            Edge::line_between(verts[index].clone(), verts[next].clone()).unwrap(),
        ));
    }
    Wire::new(edges)
}

fn circle(radius: f64, cx: f64, cy: f64) -> Wire {
    ProfileBuilder::make_circle(
        radius,
        Point3::new(cx, cy, 0.0),
        Vec3::new(0.0, 0.0, 1.0),
        Vec3::new(1.0, 0.0, 0.0),
    )
    .unwrap()
}

fn volume_of(face: &Face, thickness: f64) -> f64 {
    let solid = ThickenBuilder::thicken_face(face, thickness, &Tolerance::default())
        .expect("a planar sheet should thicken");
    MassCalculator::compute_from_brep(&solid, &TessellationParams::default()).volume
}

#[test]
fn a_round_sheet_thickens_to_a_disc_not_to_its_inscribed_square() {
    // 弧の端点だけを直線で結ぶと、対角 2r の内接正方形（体積 2r²t）に
    // なります。**それも閉じた立体なので、体積でしか見分けられません。**
    let volume = volume_of(&planar_face(circle(10.0, 0.0, 0.0), vec![]), 3.0);
    let expected = PI * 100.0 * 3.0;
    assert!(
        (volume - expected).abs() / expected < 1e-9,
        "丸板の体積が {volume}、閉じた式は {expected}（内接正方形なら 600）"
    );
}

#[test]
fn a_hole_in_the_sheet_survives_thickening() {
    let face = planar_face(rect_wire(0.0, 0.0, 40.0, 30.0), vec![circle(5.0, 20.0, 15.0)]);
    let volume = volume_of(&face, 3.0);
    let expected = (40.0 * 30.0 - PI * 25.0) * 3.0;
    assert!(
        (volume - expected).abs() / expected < 1e-9,
        "穴あき板の体積が {volume}、閉じた式は {expected}（穴を落とせば 3600）"
    );
}

#[test]
fn two_holes_both_survive_thickening() {
    let face = planar_face(
        rect_wire(0.0, 0.0, 40.0, 30.0),
        vec![circle(4.0, 12.0, 15.0), circle(3.0, 28.0, 15.0)],
    );
    let volume = volume_of(&face, 3.0);
    let expected = (40.0 * 30.0 - PI * 16.0 - PI * 9.0) * 3.0;
    assert!(
        (volume - expected).abs() / expected < 1e-9,
        "穴 2 つの板の体積が {volume}、閉じた式は {expected}"
    );
}

#[test]
fn a_rounded_rectangle_keeps_its_corner_arcs() {
    let outer = ProfileBuilder::make_rounded_rectangle(
        40.0,
        30.0,
        6.0,
        Point3::new(0.0, 0.0, 0.0),
        Vec3::new(0.0, 0.0, 1.0),
        Vec3::new(1.0, 0.0, 0.0),
    )
    .unwrap();
    let volume = volume_of(&planar_face(outer, vec![]), 3.0);
    let expected = (40.0 * 30.0 - (4.0 - PI) * 36.0) * 3.0;
    assert!(
        (volume - expected).abs() / expected < 1e-9,
        "角丸の板の体積が {volume}、閉じた式は {expected}"
    );
}

#[test]
fn a_negative_thickness_lands_on_the_other_side() {
    // **体積は表裏で同じ**なので、体積だけでは向きを見分けられません。
    // z の範囲を見ます。
    let solid = ThickenBuilder::thicken_face(
        &planar_face(circle(10.0, 0.0, 0.0), vec![]),
        -3.0,
        &Tolerance::default(),
    )
    .expect("a negative thickness should build the sheet on the other side");
    let bbox = solid.bounding_box();
    assert!(
        (bbox.min.z + 3.0).abs() < 1e-9 && bbox.max.z.abs() < 1e-9,
        "負の厚みの立体が z [{}, {}]、あるべきは [-3, 0]",
        bbox.min.z,
        bbox.max.z
    );
    let volume = MassCalculator::compute_from_brep(&solid, &TessellationParams::default()).volume;
    let expected = PI * 100.0 * 3.0;
    assert!((volume - expected).abs() / expected < 1e-9);
}
