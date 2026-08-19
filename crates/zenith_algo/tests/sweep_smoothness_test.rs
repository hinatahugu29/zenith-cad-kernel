//! Pins the swept surfaces as smooth and their integrals as convergent.
//!
//! Sweeps used to join their sections with a degree-1 ruling, so a "pipe" was
//! really a chain of flat strips with a tangent break at every section. That
//! showed up two ways: mass integration wandered at the fourth decimal no
//! matter how far the domain was refined, and OpenCASCADE disagreed with this
//! kernel by 3e-3 on the same exported B-Rep. Interpolating the sections with a
//! cubic fixed both, so both properties are locked here.

use std::f64::consts::PI;

use zenith_algo::{HelixBuilder, MassCalculator, SweepBuilder};
use zenith_geom::NurbsCurve3;
use zenith_math::{Point3, Tolerance, Vec3};
use zenith_tess::TessellationParams;
use zenith_topo::{Edge, FaceGeometry, OrientedEdge, Solid, Vertex, Wire};

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

fn curved_path() -> NurbsCurve3 {
    NurbsCurve3::bspline_from_points(
        3,
        vec![
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(10.0, 0.0, 10.0),
            Point3::new(20.0, 20.0, 25.0),
            Point3::new(30.0, 20.0, 40.0),
        ],
    )
    .unwrap()
}

fn square_profile(cx: f64, half: f64) -> Wire {
    let points = [
        Point3::new(cx - half, -half, 0.0),
        Point3::new(cx + half, -half, 0.0),
        Point3::new(cx + half, half, 0.0),
        Point3::new(cx - half, half, 0.0),
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
fn test_swept_pipe_side_surfaces_are_cubic_along_the_path() {
    let pipe = SweepBuilder::sweep_circle_along_curve(&curved_path(), 3.5, 16).unwrap();

    let mut cubic_sides = 0;
    for face in &pipe.outer_shell.faces {
        let FaceGeometry::Nurbs(surface) = &face.geometry else {
            continue;
        };
        // 側面は断面方向が2次（有理円弧）、掃引方向が3次。
        if surface.degree_u == 2 && surface.control_points[0].len() > 4 {
            assert_eq!(
                surface.degree_v, 3,
                "swept side patches must interpolate the sections with a cubic, not a ruling"
            );
            cubic_sides += 1;
        }
    }
    assert_eq!(cubic_sides, 4, "a circular sweep has four side patches");
}

#[test]
fn test_swept_pipe_volume_converges_under_refinement() {
    let pipe = SweepBuilder::sweep_circle_along_curve(&curved_path(), 3.5, 16).unwrap();

    let coarse = volume_at(&pipe, 24);
    let fine = volume_at(&pipe, 96);

    assert!(
        (fine - coarse).abs() / fine < 1e-8,
        "swept pipe volume must settle under refinement: {coarse} then {fine}"
    );
}

#[test]
fn test_straight_path_sweep_matches_the_analytic_cylinder() {
    // 直線経路の掃引は厳密に円柱なので、解析解で答え合わせできる。
    let straight = NurbsCurve3::bspline_from_points(
        3,
        vec![
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(0.0, 0.0, 10.0),
            Point3::new(0.0, 0.0, 20.0),
            Point3::new(0.0, 0.0, 30.0),
        ],
    )
    .unwrap();

    let pipe = SweepBuilder::sweep_circle_along_curve(&straight, 5.0, 16).unwrap();
    let analytic = PI * 25.0 * 30.0;
    let volume = volume_at(&pipe, 48);

    assert!(
        (volume - analytic).abs() / analytic < 1e-12,
        "straight sweep volume {volume} should equal the analytic cylinder {analytic}"
    );
}

#[test]
fn test_helix_sweep_volume_converges_under_refinement() {
    let tol = Tolerance::default();
    let helix = HelixBuilder::sweep_wire_along_helix(
        &square_profile(10.0, 1.0),
        10.0,
        6.0,
        2.0,
        Point3::new(0.0, 0.0, 0.0),
        Vec3::new(0.0, 0.0, 1.0),
        64,
        &tol,
    )
    .unwrap();

    let coarse = volume_at(&helix, 24);
    let fine = volume_at(&helix, 96);

    assert!(
        (fine - coarse).abs() / fine < 1e-8,
        "helix volume must settle under refinement: {coarse} then {fine}"
    );

    let report = helix.outer_shell.validate_closed(&tol);
    assert!(
        report.is_valid(),
        "helix shell validation failed: {:?}",
        report.errors
    );
}
