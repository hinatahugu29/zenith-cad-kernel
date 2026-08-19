//! Drilling a block through with a cylinder, the most ordinary boolean a CAD
//! kernel is asked for.
//!
//! This used to fail with a non-manifold stitch. The cause was that splitting a
//! face leaves its surface untouched and only changes its wire, while the
//! cylinder splitter asked the surface where the face was - so it would happily
//! "split" a piece with an arc lying outside it and emit overlapping pieces.
//! The split now reads the face's own boundary, and all three operations land
//! on their closed forms.

use std::f64::consts::PI;

use zenith_algo::{
    BooleanEngine, BooleanOpType, BrepTransform, MassCalculator, PrimitiveBuilder,
};
use zenith_math::{Tolerance, Vec3};
use zenith_tess::TessellationParams;
use zenith_topo::Solid;

/// 40x40x20 のブロックを、半径6の円柱が中央で貫通する構成。
fn block_and_drill() -> (Solid, Solid) {
    let block = PrimitiveBuilder::make_box(40.0, 40.0, 20.0).unwrap();
    let drill = BrepTransform::translate_solid(
        &PrimitiveBuilder::make_cylinder(6.0, 60.0).unwrap(),
        Vec3::new(20.0, 20.0, -20.0),
    );
    (block, drill)
}

fn result_volume(solids: &[Solid]) -> f64 {
    solids
        .iter()
        .map(|solid| {
            MassCalculator::compute_from_brep(
                solid,
                &TessellationParams {
                    u_divisions: 48,
                    v_divisions: 48,
                },
            )
            .volume
        })
        .sum()
}

#[test]
fn test_block_minus_through_drill_matches_the_closed_form() {
    let tol = Tolerance::default();
    let (block, drill) = block_and_drill();

    let result =
        BooleanEngine::boolean_solids_exact_result(&block, &drill, BooleanOpType::Difference, &tol)
            .expect("drilling a block should succeed");

    assert_eq!(result.solids.len(), 1);

    let expected = 40.0 * 40.0 * 20.0 - PI * 36.0 * 20.0;
    let volume = result_volume(&result.solids);
    assert!(
        (volume - expected).abs() / expected < 1e-9,
        "drilled volume {volume} should equal {expected}"
    );

    let report = result.solids[0].outer_shell.validate_closed(&tol);
    assert!(
        report.is_valid(),
        "drilled solid shell is invalid: {:?}",
        report.errors
    );

    // 貫通穴なので、上下面はそれぞれ内側ループを1本持つ。
    let holed_faces = result.solids[0]
        .outer_shell
        .faces
        .iter()
        .filter(|face| !face.inner_wires.is_empty())
        .count();
    assert_eq!(holed_faces, 2, "the drill passes through two faces");
}

#[test]
fn test_block_intersect_drill_is_the_plug() {
    let tol = Tolerance::default();
    let (block, drill) = block_and_drill();

    let result = BooleanEngine::boolean_solids_exact_result(
        &block,
        &drill,
        BooleanOpType::Intersection,
        &tol,
    )
    .expect("intersecting a block with a drill should succeed");

    let expected = PI * 36.0 * 20.0;
    let volume = result_volume(&result.solids);
    assert!(
        (volume - expected).abs() / expected < 1e-9,
        "plug volume {volume} should equal {expected}"
    );
}

#[test]
fn test_block_union_drill_matches_the_closed_form() {
    let tol = Tolerance::default();
    let (block, drill) = block_and_drill();

    let result =
        BooleanEngine::boolean_solids_exact_result(&block, &drill, BooleanOpType::Union, &tol)
            .expect("union of a block and a drill should succeed");

    // 箱の体積 + 箱の外に出ている円柱の体積。
    let expected = 40.0 * 40.0 * 20.0 + PI * 36.0 * 60.0 - PI * 36.0 * 20.0;
    let volume = result_volume(&result.solids);
    assert!(
        (volume - expected).abs() / expected < 1e-9,
        "union volume {volume} should equal {expected}"
    );
}

#[test]
fn test_difference_and_intersection_partition_the_block() {
    let tol = Tolerance::default();
    let (block, drill) = block_and_drill();

    let difference =
        BooleanEngine::boolean_solids_exact_result(&block, &drill, BooleanOpType::Difference, &tol)
            .expect("difference");
    let intersection = BooleanEngine::boolean_solids_exact_result(
        &block,
        &drill,
        BooleanOpType::Intersection,
        &tol,
    )
    .expect("intersection");

    // V(A-B) + V(A*B) = V(A) は演算の定義そのもの。
    let total = result_volume(&difference.solids) + result_volume(&intersection.solids);
    let block_volume = 40.0 * 40.0 * 20.0;
    assert!(
        (total - block_volume).abs() / block_volume < 1e-9,
        "difference plus intersection {total} should equal the block {block_volume}"
    );
}

#[test]
fn test_blind_hole_matches_the_closed_form() {
    let tol = Tolerance::default();
    let block = PrimitiveBuilder::make_box(40.0, 40.0, 20.0).unwrap();
    // 下端が箱の内部 (z=8) で止まる円柱。天面から深さ12の止まり穴。
    let drill = BrepTransform::translate_solid(
        &PrimitiveBuilder::make_cylinder(6.0, 40.0).unwrap(),
        Vec3::new(20.0, 20.0, 8.0),
    );

    let result =
        BooleanEngine::boolean_solids_exact_result(&block, &drill, BooleanOpType::Difference, &tol)
            .expect("a blind hole should succeed");

    let expected = 40.0 * 40.0 * 20.0 - PI * 36.0 * 12.0;
    let volume = result_volume(&result.solids);
    assert!(
        (volume - expected).abs() / expected < 1e-9,
        "blind hole volume {volume} should equal {expected}"
    );

    // 止まり穴なので、穴が抜けている面は天面だけ。
    let holed_faces = result.solids[0]
        .outer_shell
        .faces
        .iter()
        .filter(|face| !face.inner_wires.is_empty())
        .count();
    assert_eq!(holed_faces, 1, "a blind hole breaks through only one face");
}

#[test]
fn test_through_hole_along_x_matches_the_closed_form() {
    let tol = Tolerance::default();
    let block = PrimitiveBuilder::make_box(20.0, 20.0, 20.0).unwrap();

    let rotation = zenith_math::Transform3::from_axis_angle(
        &Vec3::new(0.0, 1.0, 0.0),
        std::f64::consts::FRAC_PI_2,
    );
    let along_x = BrepTransform::transform_solid(
        &PrimitiveBuilder::make_cylinder(5.0, 40.0).unwrap(),
        &rotation,
    )
    .unwrap();
    let drill = BrepTransform::translate_solid(&along_x, Vec3::new(-10.0, 10.0, 10.0));

    let result =
        BooleanEngine::boolean_solids_exact_result(&block, &drill, BooleanOpType::Difference, &tol)
            .expect("a hole along X should succeed");

    let expected = 8000.0 - PI * 25.0 * 20.0;
    let volume = result_volume(&result.solids);
    assert!(
        (volume - expected).abs() / expected < 1e-9,
        "X-axis hole volume {volume} should equal {expected}"
    );
}

#[test]
fn test_disjoint_union_returns_both_solids() {
    let tol = Tolerance::default();
    let a = PrimitiveBuilder::make_box(20.0, 20.0, 20.0).unwrap();
    let b = BrepTransform::translate_solid(&a, Vec3::new(100.0, 0.0, 0.0));

    let result = BooleanEngine::boolean_solids_exact_result(&a, &b, BooleanOpType::Union, &tol)
        .expect("the union of two disjoint solids is a two-solid result, not an error");

    assert_eq!(result.solids.len(), 2);
    let volume = result_volume(&result.solids);
    assert!((volume - 16000.0).abs() / 16000.0 < 1e-12);
}

#[test]
fn test_drilled_result_exports_as_a_manifold_solid() {
    let tol = Tolerance::default();
    let (block, drill) = block_and_drill();

    let result =
        BooleanEngine::boolean_solids_exact_result(&block, &drill, BooleanOpType::Difference, &tol)
            .expect("difference");

    let step = zenith_io::StepExporter::export_solid_to_string(&result.solids[0], "DRILLED");
    assert!(step.contains("MANIFOLD_SOLID_BREP"));
    assert!(step.contains("CLOSED_SHELL"));
    // 穴は内側ループとして出るので FACE_BOUND が現れる。
    assert!(step.contains("FACE_BOUND"));

    // 各面は独立に分割されるため、放っておくと隣り合う面が継ぎ目に別々の
    // エッジを作る。幾何的には閉じていても、エッジの実体が共有されていないと
    // OpenCASCADE は Solid ではなく Shell として読む。
    let edge_curves = step.matches("EDGE_CURVE(").count();
    let oriented_edges = step.matches("ORIENTED_EDGE(").count();
    assert_eq!(
        oriented_edges,
        edge_curves * 2,
        "every edge in a closed manifold is used exactly twice: {edge_curves} curves, {oriented_edges} uses"
    );
}
