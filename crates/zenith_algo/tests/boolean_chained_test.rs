//! Booleans applied to the results of booleans.
//!
//! A plate with four bolt holes is four subtractions in a row, and a
//! counterbore is a second, wider cut on the same axis as the first. Both feed
//! a boolean result straight back into the engine, which is where an operand
//! stops being a clean primitive: its faces carry reversed orientation flags
//! and its planar faces already have holes. Both used to fail for exactly
//! those reasons.

use std::f64::consts::PI;

use zenith_algo::{BooleanEngine, BooleanOpType, BrepTransform, MassCalculator, PrimitiveBuilder};
use zenith_math::{Tolerance, Vec3};
use zenith_tess::TessellationParams;
use zenith_topo::Solid;

fn volume(solid: &Solid) -> f64 {
    MassCalculator::compute_from_brep(
        solid,
        &TessellationParams {
            u_divisions: 48,
            v_divisions: 48,
        },
    )
    .volume
}

fn drill(solid: &Solid, radius: f64, height: f64, at: Vec3, tol: &Tolerance) -> Solid {
    let cutter = BrepTransform::translate_solid(
        &PrimitiveBuilder::make_cylinder(radius, height).unwrap(),
        at,
    );
    BooleanEngine::boolean_solids_exact_result(solid, &cutter, BooleanOpType::Difference, tol)
        .unwrap_or_else(|err| panic!("drilling failed: {err}"))
        .solids
        .into_iter()
        .next()
        .expect("a difference leaves one solid")
}

#[test]
fn test_four_bolt_holes_in_a_plate() {
    let tol = Tolerance::default();
    let mut plate = PrimitiveBuilder::make_box(80.0, 60.0, 20.0).unwrap();

    let radius = 5.0;
    let hole_volume = PI * radius * radius * 20.0;
    let centres = [(15.0, 15.0), (65.0, 15.0), (65.0, 45.0), (15.0, 45.0)];

    for (index, (x, y)) in centres.iter().enumerate() {
        plate = drill(&plate, radius, 60.0, Vec3::new(*x, *y, -20.0), &tol);

        let expected = 80.0 * 60.0 * 20.0 - hole_volume * (index + 1) as f64;
        let actual = volume(&plate);
        assert!(
            (actual - expected).abs() / expected < 1e-9,
            "after {} hole(s) the volume {actual} should be {expected}",
            index + 1
        );

        let report = plate.outer_shell.validate_closed(&tol);
        assert!(
            report.is_valid(),
            "after {} hole(s) the shell is invalid: {:?}",
            index + 1,
            report.errors
        );
    }

    // 4穴とも上下面を貫通しているので、穴あき平面は上下の2枚。
    let holed_faces = plate
        .outer_shell
        .faces
        .iter()
        .filter(|face| !face.inner_wires.is_empty())
        .count();
    assert_eq!(holed_faces, 2);
}

#[test]
fn test_counterbore_on_an_already_drilled_block() {
    let tol = Tolerance::default();
    let block = PrimitiveBuilder::make_box(40.0, 40.0, 20.0).unwrap();

    // 半径5の貫通穴。
    let drilled = drill(&block, 5.0, 60.0, Vec3::new(20.0, 20.0, -20.0), &tol);

    // 同軸に半径9、深さ6の座ぐり。既存の穴を囲むループを天面に刻むことになる。
    let counterbored = drill(&drilled, 9.0, 40.0, Vec3::new(20.0, 20.0, 14.0), &tol);

    let expected = 40.0 * 40.0 * 20.0 - PI * 25.0 * 20.0 - (PI * 81.0 - PI * 25.0) * 6.0;
    let actual = volume(&counterbored);
    assert!(
        (actual - expected).abs() / expected < 1e-9,
        "counterbore volume {actual} should be {expected}"
    );

    let report = counterbored.outer_shell.validate_closed(&tol);
    assert!(
        report.is_valid(),
        "counterbored shell is invalid: {:?}",
        report.errors
    );
}

#[test]
fn test_counterbore_from_the_bottom_face() {
    // 天面と裏面では刻印される面の向きが逆になる。片方だけ通る状態を防ぐ。
    let tol = Tolerance::default();
    let block = PrimitiveBuilder::make_box(40.0, 40.0, 20.0).unwrap();
    let drilled = drill(&block, 5.0, 60.0, Vec3::new(20.0, 20.0, -20.0), &tol);
    let bored = drill(&drilled, 9.0, 40.0, Vec3::new(20.0, 20.0, -34.0), &tol);

    let expected = 40.0 * 40.0 * 20.0 - PI * 25.0 * 20.0 - (PI * 81.0 - PI * 25.0) * 6.0;
    let actual = volume(&bored);
    assert!(
        (actual - expected).abs() / expected < 1e-9,
        "bottom counterbore volume {actual} should be {expected}"
    );
}

#[test]
fn test_two_separate_holes_through_the_same_faces() {
    let tol = Tolerance::default();
    let block = PrimitiveBuilder::make_box(40.0, 40.0, 20.0).unwrap();
    let once = drill(&block, 5.0, 60.0, Vec3::new(20.0, 20.0, -20.0), &tol);
    let twice = drill(&once, 5.0, 60.0, Vec3::new(8.0, 8.0, -20.0), &tol);

    let expected = 40.0 * 40.0 * 20.0 - 2.0 * PI * 25.0 * 20.0;
    let actual = volume(&twice);
    assert!(
        (actual - expected).abs() / expected < 1e-9,
        "two-hole volume {actual} should be {expected}"
    );

    // 2つ目の穴は、既に穴のある平面へ2本目の内側ループを足す。
    let top_holes = twice
        .outer_shell
        .faces
        .iter()
        .map(|face| face.inner_wires.len())
        .max()
        .unwrap_or(0);
    assert_eq!(
        top_holes, 2,
        "a face through which both holes pass has two loops"
    );
}

#[test]
fn test_chained_result_still_exports_with_shared_edges() {
    let tol = Tolerance::default();
    let plate = PrimitiveBuilder::make_box(60.0, 40.0, 15.0).unwrap();
    let once = drill(&plate, 4.0, 40.0, Vec3::new(15.0, 20.0, -10.0), &tol);
    let twice = drill(&once, 4.0, 40.0, Vec3::new(45.0, 20.0, -10.0), &tol);

    let step = zenith_io::StepExporter::export_solid_to_string(&twice, "TWO_HOLES");
    assert!(step.contains("MANIFOLD_SOLID_BREP"));

    let edge_curves = step.matches("EDGE_CURVE(").count();
    let oriented_edges = step.matches("ORIENTED_EDGE(").count();
    assert_eq!(
        oriented_edges,
        edge_curves * 2,
        "chaining must not leave duplicated edges: {edge_curves} curves, {oriented_edges} uses"
    );
}
