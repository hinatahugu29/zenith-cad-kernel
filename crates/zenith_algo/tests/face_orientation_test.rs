//! Solids have to be built the right way out.
//!
//! A closed shell can be described two ways: normals pointing out of the
//! material, or into it. Only one is the solid you meant, and everything
//! downstream that uses a normal depends on which you have.
//!
//! The divergence theorem says which. Integrating over a shell whose normals
//! point outward gives the enclosed volume; inward gives its negative. That
//! reading was being taken and then thrown away - MassProperties returned the
//! absolute value - so three builders had been producing inside-out solids
//! with nothing to notice: the sphere, the torus and the revolved solid.
//!
//! It cost real work. Cutting a torus with a plane could not be made to close,
//! because the cap that was correct met torus pieces that were not, and the
//! only reason any torus boolean worked at all was that the cap builder tries
//! both orientations and keeps whichever stitches.
//!
//! This measures the sign per face, so it cannot be hidden again.

use zenith_algo::{
    ExtrudeBuilder, HoleBuilder, LoftBuilder, MassCalculator, PrimitiveBuilder, RevolveBuilder,
    ShellBuilder,
};
use zenith_math::{Point3, Tolerance, Vec3};
use zenith_tess::TessellationParams;
use zenith_topo::{Edge, OrientedEdge, Solid, Vertex, Wire};

fn params() -> TessellationParams {
    TessellationParams {
        u_divisions: 64,
        v_divisions: 64,
    }
}

/// The volume a shell encloses, taken with its sign rather than its size.
fn signed_volume(solid: &Solid) -> f64 {
    solid
        .outer_shell
        .faces
        .iter()
        .map(|face| MassCalculator::compute_face_integral(face, &params()).1)
        .sum()
}

fn closed_wire(points: [Point3; 4]) -> Wire {
    let vertices: Vec<Vertex> = points.into_iter().map(Vertex::from_point).collect();
    Wire::new(
        (0..4)
            .map(|index| {
                OrientedEdge::forward(
                    Edge::line_between(vertices[index].clone(), vertices[(index + 1) % 4].clone())
                        .expect("edge"),
                )
            })
            .collect(),
    )
}

fn rect_wire(half_x: f64, half_y: f64, z: f64) -> Wire {
    closed_wire([
        Point3::new(-half_x, -half_y, z),
        Point3::new(half_x, -half_y, z),
        Point3::new(half_x, half_y, z),
        Point3::new(-half_x, half_y, z),
    ])
}

#[test]
fn test_every_builder_makes_a_solid_that_is_the_right_way_out() {
    let tol = Tolerance::default();
    let subjects: Vec<(&str, Solid)> = vec![
        ("box", PrimitiveBuilder::make_box(20.0, 30.0, 40.0).unwrap()),
        (
            "cylinder",
            PrimitiveBuilder::make_cylinder(10.0, 40.0).unwrap(),
        ),
        ("sphere", PrimitiveBuilder::make_sphere(10.0).unwrap()),
        (
            "cone",
            PrimitiveBuilder::make_cone(10.0, 4.0, 20.0).unwrap(),
        ),
        ("torus", PrimitiveBuilder::make_torus(12.0, 4.0).unwrap()),
        (
            "extrusion",
            ExtrudeBuilder::extrude_wire(
                &rect_wire(15.0, 10.0, 0.0),
                Vec3::new(0.0, 0.0, 25.0),
                &tol,
            )
            .unwrap(),
        ),
        (
            "hollow box",
            ShellBuilder::make_hollow_box(40.0, 30.0, 20.0, 2.0, 1).unwrap(),
        ),
        (
            "drilled box",
            HoleBuilder::make_drilled_box(30.0, 30.0, 15.0, 5.0).unwrap(),
        ),
        (
            "revolved ring",
            RevolveBuilder::revolve_wire_solid(
                &closed_wire([
                    Point3::new(4.0, 0.0, 0.0),
                    Point3::new(8.0, 0.0, 0.0),
                    Point3::new(8.0, 0.0, 10.0),
                    Point3::new(4.0, 0.0, 10.0),
                ]),
                Point3::new(0.0, 0.0, 0.0),
                Vec3::new(0.0, 0.0, 1.0),
                &tol,
            )
            .unwrap(),
        ),
        (
            "lofted prism",
            LoftBuilder::loft_solid(
                &[rect_wire(10.0, 10.0, 0.0), rect_wire(10.0, 10.0, 30.0)],
                1,
                &tol,
            )
            .unwrap(),
        ),
    ];

    for (name, solid) in subjects {
        let signed = signed_volume(&solid);
        assert!(
            signed > 0.0,
            "{name} is inside out: its shell encloses {signed:.4}, which is negative"
        );

        // 符号を捨てた値と一致すること。捨てているせいで裏返りが見えなかった。
        let reported = MassCalculator::compute_from_brep(&solid, &params()).volume;
        let relative = (reported - signed).abs() / signed;
        assert!(
            relative < 1e-9,
            "{name}: reported {reported:.6} against the signed {signed:.6}"
        );
    }
}

#[test]
fn test_the_three_that_were_inverted_now_match_their_closed_forms_with_the_sign() {
    // 裏返っていた3つ。符号を捨てると通ってしまうので、符号付きで測る。
    let tol = Tolerance::default();

    let sphere = PrimitiveBuilder::make_sphere(10.0).unwrap();
    let torus = PrimitiveBuilder::make_torus(12.0, 4.0).unwrap();
    let ring = RevolveBuilder::revolve_wire_solid(
        &closed_wire([
            Point3::new(4.0, 0.0, 0.0),
            Point3::new(8.0, 0.0, 0.0),
            Point3::new(8.0, 0.0, 10.0),
            Point3::new(4.0, 0.0, 10.0),
        ]),
        Point3::new(0.0, 0.0, 0.0),
        Vec3::new(0.0, 0.0, 1.0),
        &tol,
    )
    .unwrap();

    let pi = std::f64::consts::PI;
    let subjects = [
        ("sphere", &sphere, 4.0 / 3.0 * pi * 1000.0),
        ("torus", &torus, 2.0 * pi * pi * 12.0 * 16.0),
        ("revolved ring", &ring, pi * (64.0 - 16.0) * 10.0),
    ];

    for (name, solid, expected) in subjects {
        let signed = signed_volume(solid);
        let relative = (signed - expected).abs() / expected;
        assert!(
            relative < 1e-6,
            "{name}: signed {signed:.4}, closed form {expected:.4} (relative {relative:.2e})"
        );
    }
}
