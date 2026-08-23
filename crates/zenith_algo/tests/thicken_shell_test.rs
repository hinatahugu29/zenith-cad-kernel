use zenith_algo::{MassCalculator, ThickenBuilder};
use zenith_geom::PlaneSurface3;
use zenith_math::{Point3, Tolerance};
use zenith_tess::TessellationParams;
use zenith_topo::{Edge, Face, FaceGeometry, Orientation, OrientedEdge, Shell, Vertex, Wire};

fn make_planar_quad_face(p0: Point3, p1: Point3, p2: Point3, p3: Point3) -> Face {
    let v0 = Vertex::from_point(p0);
    let v1 = Vertex::from_point(p1);
    let v2 = Vertex::from_point(p2);
    let v3 = Vertex::from_point(p3);

    let e0 = Edge::line_between(v0.clone(), v1.clone()).unwrap();
    let e1 = Edge::line_between(v1.clone(), v2.clone()).unwrap();
    let e2 = Edge::line_between(v2.clone(), v3.clone()).unwrap();
    let e3 = Edge::line_between(v3.clone(), v0.clone()).unwrap();

    let wire = Wire::new(vec![
        OrientedEdge::forward(e0),
        OrientedEdge::forward(e1),
        OrientedEdge::forward(e2),
        OrientedEdge::forward(e3),
    ]);

    let u_axis = (p1 - p0).normalize();
    let v_axis = (p3 - p0).normalize();
    let plane = PlaneSurface3::new(p0, u_axis, v_axis).unwrap();

    Face::new(
        FaceGeometry::Plane(plane),
        wire,
        vec![],
        Orientation::Forward,
        1e-6,
    )
}

#[test]
fn test_thicken_single_face_matches_volume() {
    let tol = Tolerance::default();
    let face = make_planar_quad_face(
        Point3::new(0.0, 0.0, 0.0),
        Point3::new(20.0, 0.0, 0.0),
        Point3::new(20.0, 30.0, 0.0),
        Point3::new(0.0, 30.0, 0.0),
    );

    let solid = ThickenBuilder::thicken_face(&face, 4.0, &tol).expect("thicken face");
    assert!(solid.outer_shell.validate_closed(&tol).is_valid());

    let mass = MassCalculator::compute_from_brep(&solid, &TessellationParams::default());
    let expected_volume = 20.0 * 30.0 * 4.0;
    assert!((mass.volume - expected_volume).abs() < 1e-4);
}

#[test]
fn test_thicken_open_shell_composite() {
    let tol = Tolerance::default();
    let face1 = make_planar_quad_face(
        Point3::new(0.0, 0.0, 0.0),
        Point3::new(20.0, 0.0, 0.0),
        Point3::new(20.0, 30.0, 0.0),
        Point3::new(0.0, 30.0, 0.0),
    );
    let face2 = make_planar_quad_face(
        Point3::new(20.0, 0.0, 0.0),
        Point3::new(40.0, 0.0, 0.0),
        Point3::new(40.0, 30.0, 0.0),
        Point3::new(20.0, 30.0, 0.0),
    );

    let shell = Shell::new(vec![face1, face2], false);
    let solid = ThickenBuilder::thicken_shell(&shell, 3.0, &tol).expect("thicken shell");
    assert!(solid.outer_shell.validate_closed(&tol).is_valid());

    let mass = MassCalculator::compute_from_brep(&solid, &TessellationParams::default());
    let expected_volume = 40.0 * 30.0 * 3.0;
    assert!((mass.volume - expected_volume).abs() < 1e-3);
}
