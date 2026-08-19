//! Every builder, against a closed form where one exists and against
//! invariants where one does not.
//!
//! The defects found in this kernel so far shared a shape: a result that looks
//! reasonable and is wrong, or an integral that never settles. Both are
//! invisible unless something outside the builder checks it, so this is that
//! check, applied uniformly rather than per builder.
//!
//! Mirrors `cargo run --release -p zenith_algo --example builder_audit`.

use std::f64::consts::PI;

use zenith_algo::{
    ChamferBuilder, ExtrudeBuilder, FilletBuilder, GearBuilder, HelixBuilder, HoleBuilder,
    LoftBuilder, MassCalculator, MirrorBuilder, PatternBuilder, PrimitiveBuilder, RevolveBuilder,
    ShellingBuilder, SweepBuilder,
};
use zenith_geom::NurbsCurve3;
use zenith_math::{Point3, Tolerance, Vec3};
use zenith_tess::TessellationParams;
use zenith_topo::{Edge, OrientedEdge, Solid, Vertex, Wire};

fn volume_at(solid: &Solid, divisions: usize) -> f64 {
    MassCalculator::compute_from_brep(
        solid,
        &TessellationParams {
            u_divisions: divisions,
            v_divisions: divisions,
        },
    )
    .volume
}

/// Every solid this kernel hands out must satisfy these, whatever built it.
fn assert_sound(name: &str, solid: &Solid, analytic_volume: Option<f64>) {
    let tol = Tolerance::default();

    let report = solid.outer_shell.validate_closed(&tol);
    assert!(
        report.is_valid(),
        "{name}: shell is not a valid closed shell: {:?}",
        report.errors
    );

    let coarse = volume_at(solid, 24);
    let fine = volume_at(solid, 96);

    assert!(fine > 0.0, "{name}: volume {fine} is not positive");
    assert!(fine.is_finite(), "{name}: volume is not finite");

    let convergence = (fine - coarse).abs() / fine.abs();
    assert!(
        convergence < 1e-8,
        "{name}: volume does not settle under refinement ({coarse} then {fine}, relative {convergence:.3e})"
    );

    if let Some(expected) = analytic_volume {
        let error = (fine - expected).abs() / expected.abs();
        assert!(
            error < 1e-6,
            "{name}: volume {fine} differs from the analytic {expected} by {error:.3e}"
        );
    }
}

fn rect_wire(half_x: f64, half_y: f64, z: f64) -> Wire {
    let points = [
        Point3::new(-half_x, -half_y, z),
        Point3::new(half_x, -half_y, z),
        Point3::new(half_x, half_y, z),
        Point3::new(-half_x, half_y, z),
    ];
    let vertices: Vec<Vertex> = points.into_iter().map(Vertex::from_point).collect();
    let edges = (0..4)
        .map(|index| {
            let edge =
                Edge::line_between(vertices[index].clone(), vertices[(index + 1) % 4].clone())
                    .unwrap();
            OrientedEdge::forward(edge)
        })
        .collect();
    Wire::new(edges)
}

#[test]
fn test_primitives_match_their_closed_forms() {
    assert_sound(
        "box",
        &PrimitiveBuilder::make_box(20.0, 30.0, 40.0).unwrap(),
        Some(24000.0),
    );
    assert_sound(
        "cylinder",
        &PrimitiveBuilder::make_cylinder(10.0, 40.0).unwrap(),
        Some(PI * 100.0 * 40.0),
    );
    assert_sound(
        "sphere",
        &PrimitiveBuilder::make_sphere(10.0).unwrap(),
        Some(4.0 / 3.0 * PI * 1000.0),
    );
    assert_sound(
        "cone",
        &PrimitiveBuilder::make_cone(10.0, 4.0, 20.0).unwrap(),
        Some(PI * 20.0 / 3.0 * (100.0 + 40.0 + 16.0)),
    );
    assert_sound(
        "torus",
        &PrimitiveBuilder::make_torus(12.0, 4.0).unwrap(),
        Some(2.0 * PI * PI * 12.0 * 16.0),
    );
}

#[test]
fn test_edge_treatments_match_their_closed_forms() {
    let tol = Tolerance::default();

    // 縦稜4本を半径 r で丸めると、断面積が (4 - pi) r^2 減る。
    let radius = 4.0;
    assert_sound(
        "filleted box",
        &FilletBuilder::fillet_box_z_edges(20.0, 30.0, 40.0, radius, &tol).unwrap(),
        Some((20.0 * 30.0 - (4.0 - PI) * radius * radius) * 40.0),
    );

    // 面取りは各角から直角二等辺三角形を落とす。
    let chamfer = 4.0;
    assert_sound(
        "chamfered box",
        &ChamferBuilder::chamfer_box_z_edges(20.0, 30.0, 40.0, chamfer, &tol).unwrap(),
        Some((20.0 * 30.0 - 2.0 * chamfer * chamfer) * 40.0),
    );
}

#[test]
fn test_hole_and_shelling_match_their_closed_forms() {
    assert_sound(
        "drilled box",
        &HoleBuilder::make_drilled_box(30.0, 30.0, 15.0, 5.0).unwrap(),
        Some(30.0 * 30.0 * 15.0 - PI * 25.0 * 15.0),
    );
    assert_sound(
        "open box",
        &ShellingBuilder::make_open_box(40.0, 30.0, 20.0, 2.0).unwrap(),
        Some(40.0 * 30.0 * 20.0 - 36.0 * 26.0 * 18.0),
    );
}

#[test]
fn test_extrude_revolve_and_loft_match_their_closed_forms() {
    let tol = Tolerance::default();

    assert_sound(
        "extrusion",
        &ExtrudeBuilder::extrude_wire(
            &rect_wire(15.0, 10.0, 0.0),
            Vec3::new(0.0, 0.0, 25.0),
            &tol,
        )
        .unwrap(),
        Some(30.0 * 20.0 * 25.0),
    );

    assert_sound(
        "hollow extrusion",
        &ExtrudeBuilder::extrude_face_with_holes(
            &rect_wire(15.0, 10.0, 0.0),
            &[rect_wire(8.0, 5.0, 0.0)],
            Vec3::new(0.0, 0.0, 25.0),
            &tol,
        )
        .unwrap(),
        Some((30.0 * 20.0 - 16.0 * 10.0) * 25.0),
    );

    // XZ平面の矩形をZ軸まわりに回すと、内径4・外径8・高さ10のリングになる。
    let profile_points = [
        Point3::new(4.0, 0.0, 0.0),
        Point3::new(8.0, 0.0, 0.0),
        Point3::new(8.0, 0.0, 10.0),
        Point3::new(4.0, 0.0, 10.0),
    ];
    let profile_vertices: Vec<Vertex> =
        profile_points.into_iter().map(Vertex::from_point).collect();
    let profile_edges = (0..4)
        .map(|index| {
            let edge = Edge::line_between(
                profile_vertices[index].clone(),
                profile_vertices[(index + 1) % 4].clone(),
            )
            .unwrap();
            OrientedEdge::forward(edge)
        })
        .collect();

    assert_sound(
        "revolved ring",
        &RevolveBuilder::revolve_wire_solid(
            &Wire::new(profile_edges),
            Point3::new(0.0, 0.0, 0.0),
            Vec3::new(0.0, 0.0, 1.0),
            &tol,
        )
        .unwrap(),
        Some(PI * (64.0 - 16.0) * 10.0),
    );

    // 同じ断面を2枚重ねたロフトは角柱になる。
    assert_sound(
        "loft prism",
        &LoftBuilder::loft_solid(
            &[rect_wire(10.0, 10.0, 0.0), rect_wire(10.0, 10.0, 30.0)],
            1,
            &tol,
        )
        .unwrap(),
        Some(20.0 * 20.0 * 30.0),
    );
}

#[test]
fn test_mirror_and_pattern_preserve_volume() {
    let tol = Tolerance::default();
    let box_solid = PrimitiveBuilder::make_box(20.0, 30.0, 40.0).unwrap();

    assert_sound(
        "mirrored box",
        &MirrorBuilder::mirror_solid(
            &box_solid,
            Point3::new(0.0, 0.0, 0.0),
            Vec3::new(1.0, 0.0, 0.0),
            &tol,
        )
        .unwrap(),
        Some(24000.0),
    );

    let copies =
        PatternBuilder::linear_pattern(&box_solid, Vec3::new(1.0, 0.0, 0.0), 50.0, 3).unwrap();
    assert_eq!(copies.len(), 3);
    for (index, copy) in copies.iter().enumerate() {
        assert_sound(&format!("pattern copy {index}"), copy, Some(24000.0));
    }
}

#[test]
fn test_sweeps_and_gear_are_sound() {
    let tol = Tolerance::default();

    let curved = NurbsCurve3::bspline_from_points(
        3,
        vec![
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(10.0, 0.0, 10.0),
            Point3::new(20.0, 20.0, 25.0),
            Point3::new(30.0, 20.0, 40.0),
        ],
    )
    .unwrap();
    assert_sound(
        "curved sweep",
        &SweepBuilder::sweep_circle_along_curve(&curved, 3.5, 16).unwrap(),
        None,
    );

    assert_sound(
        "helix sweep",
        &HelixBuilder::sweep_wire_along_helix(
            &rect_wire(1.0, 1.0, 0.0),
            10.0,
            6.0,
            2.0,
            Point3::new(0.0, 0.0, 0.0),
            Vec3::new(0.0, 0.0, 1.0),
            64,
            &tol,
        )
        .unwrap(),
        None,
    );

    assert_sound(
        "spur gear",
        &GearBuilder::make_spur_gear(2.0, 18, 20.0, 8.0, 6.0).unwrap(),
        None,
    );
}
