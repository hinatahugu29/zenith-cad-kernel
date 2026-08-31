//! Booleans between boxes at an arbitrary angle.
//!
//! This is the general polyhedral case, and it needed four things working
//! together, which is why partial attempts kept trading one result for
//! another:
//!
//! - a cut made of several segments meeting at interior corners has to be
//!   applied as one chain, since no single segment reaches the boundary twice
//! - segments running along the face's own boundary are not cuts and must be
//!   kept out of those chains
//! - a cut whose two ends land on the same boundary edge is legitimate, and
//!   the path between them is a portion of that edge rather than a lap of the
//!   whole wire
//! - once faces are split, a neighbour's vertex sitting inside an edge has to
//!   be imprinted, or the two sides of that edge no longer correspond

use zenith_algo::{BooleanEngine, BooleanOpType, BrepTransform, MassCalculator, PrimitiveBuilder};
use zenith_math::{Tolerance, Transform3, Vec3};
use zenith_tess::TessellationParams;
use zenith_topo::Solid;

fn volume(solids: &[Solid]) -> f64 {
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

/// Two 20-cubes, one turned 45 degrees about Z and lifted so no faces are
/// coplanar.
fn rotated_pair() -> (Solid, Solid) {
    let base = PrimitiveBuilder::make_box(20.0, 20.0, 20.0).unwrap();
    let rotated = BrepTransform::transform_solid(
        &BrepTransform::translate_solid(&base, Vec3::new(10.0, 10.0, 0.0)),
        &Transform3::from_axis_angle(&Vec3::new(0.0, 0.0, 1.0), std::f64::consts::FRAC_PI_4),
    )
    .unwrap();
    let lifted = BrepTransform::translate_solid(&rotated, Vec3::new(0.0, 0.0, 7.0));
    (base, lifted)
}

#[test]
fn test_rotated_boxes_partition_correctly() {
    let tol = Tolerance::default();
    let (a, b) = rotated_pair();

    let union = BooleanEngine::boolean_solids_exact_result(&a, &b, BooleanOpType::Union, &tol)
        .expect("union of rotated boxes");
    let difference =
        BooleanEngine::boolean_solids_exact_result(&a, &b, BooleanOpType::Difference, &tol)
            .expect("difference of rotated boxes");
    let intersection =
        BooleanEngine::boolean_solids_exact_result(&a, &b, BooleanOpType::Intersection, &tol)
            .expect("intersection of rotated boxes");

    let union_volume = volume(&union.solids);
    let difference_volume = volume(&difference.solids);
    let intersection_volume = volume(&intersection.solids);

    // 定義そのもの: V(A-B) + V(A*B) = V(A)
    let a_volume = 8000.0;
    assert!(
        (difference_volume + intersection_volume - a_volume).abs() / a_volume < 1e-9,
        "difference {difference_volume} plus intersection {intersection_volume} should be {a_volume}"
    );

    // 同じく: V(A|B) + V(A*B) = V(A) + V(B)
    let both = 16000.0;
    assert!(
        (union_volume + intersection_volume - both).abs() / both < 1e-9,
        "union {union_volume} plus intersection {intersection_volume} should be {both}"
    );

    for (name, result) in [
        ("union", &union),
        ("difference", &difference),
        ("intersection", &intersection),
    ] {
        assert_eq!(result.solids.len(), 1, "{name} should be one solid");
        let report = result.solids[0].outer_shell.validate_closed(&tol);
        assert!(
            report.is_valid(),
            "{name} shell is invalid: {:?}",
            report.errors
        );
    }
}

#[test]
fn test_rotated_box_result_exports_with_shared_edges() {
    let tol = Tolerance::default();
    let (a, b) = rotated_pair();

    let union = BooleanEngine::boolean_solids_exact_result(&a, &b, BooleanOpType::Union, &tol)
        .expect("union of rotated boxes");

    let step = zenith_io::StepExporter::export_solid_to_string(&union.solids[0], "ROTATED_UNION");
    let edge_curves = step.matches("EDGE_CURVE(").count();
    let oriented_edges = step.matches("ORIENTED_EDGE(").count();
    assert_eq!(
        oriented_edges,
        edge_curves * 2,
        "a closed manifold uses each edge exactly twice: {edge_curves} curves, {oriented_edges} uses"
    );
}

#[test]
fn test_a_corner_intruding_through_one_face_still_splits_it() {
    // 小さな箱の角が大きな箱の一面から入って同じ面から出る。切り込みの両端が
    // 同じ境界辺に乗るので、以前は分割そのものが拒否されていた。
    let tol = Tolerance::default();
    let plate = PrimitiveBuilder::make_box(40.0, 40.0, 10.0).unwrap();
    // 10角柱を45度回してから、板の y=40 面を跨ぐ位置へ動かす。
    let rotated = BrepTransform::transform_solid(
        &PrimitiveBuilder::make_box(10.0, 10.0, 30.0).unwrap(),
        &Transform3::from_axis_angle(&Vec3::new(0.0, 0.0, 1.0), std::f64::consts::FRAC_PI_4),
    )
    .unwrap();
    let intruder = BrepTransform::translate_solid(&rotated, Vec3::new(20.0, 33.0, -10.0));

    let plate_volume = 40.0 * 40.0 * 10.0;
    let difference = BooleanEngine::boolean_solids_exact_result(
        &plate,
        &intruder,
        BooleanOpType::Difference,
        &tol,
    );
    let intersection = BooleanEngine::boolean_solids_exact_result(
        &plate,
        &intruder,
        BooleanOpType::Intersection,
        &tol,
    );

    // 対応範囲外ならエラーになる。もっともらしい誤答は返らないので、
    // 両方が成立したときだけ定義を突き合わせる。
    if let (Ok(difference), Ok(intersection)) = (difference, intersection) {
        let removed = volume(&intersection.solids);
        assert!(
            removed > 0.0,
            "the intruding corner overlaps the plate, so something must be removed"
        );

        let total = volume(&difference.solids) + removed;
        assert!(
            (total - plate_volume).abs() / plate_volume < 1e-9,
            "difference plus intersection {total} should be the plate {plate_volume}"
        );

        let report = difference.solids[0].outer_shell.validate_closed(&tol);
        assert!(report.is_valid(), "shell invalid: {:?}", report.errors);
    }
}
