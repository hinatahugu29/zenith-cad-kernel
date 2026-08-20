//! Reading analytic surfaces out of files another kernel wrote.
//!
//! STEP states a cone, a sphere or a torus as an unbounded surface plus a
//! radius; the face's own boundary is the only thing that says which piece is
//! in use. The reader used to build a fixed-size patch instead, so the boundary
//! sat off the surface and every one of these files was refused. Nothing inside
//! this kernel could show that, because our own exporter writes these shapes as
//! B-splines and never takes the analytic path.
//!
//! The fixtures were written by OpenCASCADE 7.8 (`tools/occ_reference_export.py`)
//! and the expected volumes are what OpenCASCADE itself reports for them, so a
//! disagreement here is a disagreement with another kernel rather than with a
//! number this repository chose.

use zenith_algo::MassCalculator;
use zenith_io::StepImporter;
use zenith_math::Tolerance;
use zenith_tess::TessellationParams;
use zenith_topo::Solid;

fn volume(solid: &Solid) -> f64 {
    MassCalculator::compute_from_brep(
        solid,
        &TessellationParams {
            u_divisions: 64,
            v_divisions: 64,
        },
    )
    .volume
}

fn read_fixture(name: &str) -> Solid {
    let text = match name {
        "cone" => include_str!("fixtures/occ_reference_cone.step"),
        "cone_full" => include_str!("fixtures/occ_reference_cone_full.step"),
        "sphere_capped" => include_str!("fixtures/occ_reference_sphere_capped.step"),
        "torus_segment" => include_str!("fixtures/occ_reference_torus_segment.step"),
        "torus" => include_str!("fixtures/occ_reference_torus.step"),
        other => panic!("no fixture named {other}"),
    };

    let solids = StepImporter::import_solids_from_str(text)
        .unwrap_or_else(|err| panic!("{name} should import: {err}"));
    assert_eq!(solids.len(), 1, "{name} should hold one solid");

    let solid = solids.into_iter().next().unwrap();
    let report = solid.outer_shell.validate_closed(&Tolerance::default());
    assert!(
        report.is_valid(),
        "{name} should close: {:?}",
        report.errors.first()
    );
    solid
}

fn assert_volume(name: &str, expected: f64) {
    let measured = volume(&read_fixture(name));
    let relative = (measured - expected).abs() / expected.abs();
    assert!(
        relative < 1e-3,
        "{name}: read {measured:.4}, OpenCASCADE says {expected:.4} (relative {relative:.2e})"
    );
}

#[test]
fn test_a_conical_face_is_sized_from_its_boundary() {
    // Part.makeCone(10, 4, 20)
    assert_volume("cone", 3267.2564);
}

#[test]
fn test_a_conical_face_running_to_the_apex_is_readable() {
    // Part.makeCone(10, 0, 20). The apex end has zero radius, which is a
    // degenerate row rather than a reason to refuse the face.
    assert_volume("cone_full", 2094.3951);
}

#[test]
fn test_a_spherical_face_bounded_by_real_edges_is_readable() {
    // A sphere of radius 10 cut in half. The spherical face's loop walks its
    // seam meridian up and back down again before going round the equator, so
    // one edge is used twice by the one face.
    assert_volume("sphere_capped", 2094.3951);
}

#[test]
fn test_a_toroidal_face_is_sized_from_its_boundary() {
    // A quarter of a torus, R=12 r=4: the elbow shape a pipe run is made of.
    assert_volume("torus_segment", 947.4820);
}

#[test]
fn test_a_torus_written_as_one_face_is_readable() {
    // OpenCASCADE writes a whole torus as a single face whose bound is nothing
    // but seam: two circles, each walked once each way. Such a loop covers the
    // whole parameter domain, but its p-curves cannot say so, because a point
    // on the seam maps to both ends of the domain. Read from the p-curves the
    // face came out at exactly half the surface, and so did the volume.
    assert_volume("torus", 3789.9281);

    let solid = read_fixture("torus");
    assert_eq!(solid.outer_shell.faces.len(), 1);
    let area = MassCalculator::compute_face_integral(
        &solid.outer_shell.faces[0],
        &TessellationParams {
            u_divisions: 64,
            v_divisions: 64,
        },
    )
    .0;
    // 4 pi^2 R r
    let expected = 4.0 * std::f64::consts::PI * std::f64::consts::PI * 12.0 * 4.0;
    assert!(
        (area - expected).abs() / expected < 1e-3,
        "torus surface area {area:.4}, closed form {expected:.4}"
    );
}

#[test]
fn test_the_analytic_faces_carry_their_analytic_area() {
    // Volume alone can be right while a face is the wrong size, so the areas
    // are checked against the closed forms as well.
    let params = TessellationParams {
        u_divisions: 64,
        v_divisions: 64,
    };
    let subjects: [(&str, f64); 3] = [
        // Lateral surface of a frustum: pi (r1 + r2) * slant.
        (
            "cone",
            std::f64::consts::PI * 14.0 * (20.0f64 * 20.0 + 6.0 * 6.0).sqrt(),
        ),
        // Half a sphere of radius 10: 2 pi r^2.
        ("sphere_capped", 2.0 * std::f64::consts::PI * 100.0),
        // A quarter of a torus: (2 pi R)(2 pi r) / 4.
        (
            "torus_segment",
            std::f64::consts::PI * std::f64::consts::PI * 12.0 * 4.0,
        ),
    ];

    for (name, expected) in subjects {
        let solid = read_fixture(name);
        // The analytic face is the one carrying the most area.
        let largest = solid
            .outer_shell
            .faces
            .iter()
            .map(|face| MassCalculator::compute_face_integral(face, &params).0)
            .fold(0.0f64, f64::max);
        let relative = (largest - expected).abs() / expected;
        assert!(
            relative < 1e-3,
            "{name}: analytic face area {largest:.4}, closed form {expected:.4} (relative {relative:.2e})"
        );
    }
}
