//! Locks in the boolean correctness gate.
//!
//! A closed manifold shell does not prove a boolean is right - returning one
//! operand untouched is manifold and wrong. These tests pin both directions:
//! results that are genuinely correct must still get through, and the known
//! silent-wrong curved-surface cases must now surface as errors instead of
//! plausible-looking solids.

use zenith_algo::{
    BooleanEngine, BooleanOpType, BooleanResultVerifier, BrepTransform, MassCalculator,
    PrimitiveBuilder,
};
use zenith_math::{Tolerance, Vec3};
use zenith_tess::TessellationParams;

fn fine_tessellation() -> TessellationParams {
    TessellationParams {
        u_divisions: 32,
        v_divisions: 32,
    }
}

#[test]
fn test_gate_lets_correct_axis_aligned_box_booleans_through() {
    let tol = Tolerance::default();
    let a = PrimitiveBuilder::make_box(20.0, 20.0, 20.0).unwrap();
    let b = BrepTransform::translate_solid(&a, Vec3::new(10.0, 10.0, 10.0));

    // 重なり 10^3 の角重なり配置。和 15000 / 差 7000 / 積 1000。
    for (op, expected) in [
        (BooleanOpType::Union, 15000.0),
        (BooleanOpType::Difference, 7000.0),
        (BooleanOpType::Intersection, 1000.0),
    ] {
        let result = BooleanEngine::boolean_solids_exact_result(&a, &b, op, &tol)
            .unwrap_or_else(|err| panic!("{op:?} should succeed but was rejected: {err}"));

        let volume: f64 = result
            .solids
            .iter()
            .map(|s| MassCalculator::compute_from_brep(s, &fine_tessellation()).volume)
            .sum();

        assert!(
            (volume - expected).abs() / expected < 1e-9,
            "{op:?} volume {volume} does not match the analytic {expected}"
        );
    }
}

#[test]
fn test_gate_rejects_overlapping_sphere_boolean_that_returns_one_operand() {
    let tol = Tolerance::default();
    let a = PrimitiveBuilder::make_sphere(10.0).unwrap();
    let b = BrepTransform::translate_solid(&a, Vec3::new(10.0, 0.0, 0.0));

    // 半径10・中心間距離10の2球。和は 7068.583、積は 1308.997 が正解であり、
    // どちらも球1個ぶん (4188.790) にはなり得ない。
    for op in [BooleanOpType::Union, BooleanOpType::Intersection] {
        let result = BooleanEngine::boolean_solids_exact_result(&a, &b, op, &tol);
        assert!(
            result.is_err(),
            "{op:?} of two overlapping spheres must not report success; got a solid instead"
        );
    }
}

#[test]
fn test_gate_rejects_cone_box_difference_and_intersection_overlap() {
    let tol = Tolerance::default();
    let cone = PrimitiveBuilder::make_cone(10.0, 4.0, 20.0).unwrap();
    let cutter = BrepTransform::translate_solid(
        &PrimitiveBuilder::make_box(20.0, 20.0, 20.0).unwrap(),
        Vec3::new(-10.0, -10.0, 10.0),
    );

    // 差と積が同値 (3141.593) を返していた。V(A-B) + V(A*B) = V(A) を満たさない。
    for op in [BooleanOpType::Difference, BooleanOpType::Intersection] {
        assert!(
            BooleanEngine::boolean_solids_exact_result(&cone, &cutter, op, &tol).is_err(),
            "{op:?} of cone and box must not report success"
        );
    }
}

#[test]
fn test_gate_rejects_torus_box_union_smaller_than_an_operand() {
    let tol = Tolerance::default();
    let torus = PrimitiveBuilder::make_torus(12.0, 4.0).unwrap();
    let boxed = BrepTransform::translate_solid(
        &PrimitiveBuilder::make_box(20.0, 20.0, 20.0).unwrap(),
        Vec3::new(-10.0, -10.0, -2.0),
    );

    // 和集合が 4210.072 で、ボックス単体の 8000 を下回っていた。
    assert!(
        BooleanEngine::boolean_solids_exact_result(&torus, &boxed, BooleanOpType::Union, &tol)
            .is_err(),
        "a union smaller than one of its operands must not report success"
    );
}

#[test]
fn test_verifier_rejects_an_operand_passed_through_as_the_union() {
    let tol = Tolerance::default();
    let a = PrimitiveBuilder::make_box(20.0, 20.0, 20.0).unwrap();
    let b = BrepTransform::translate_solid(&a, Vec3::new(10.0, 10.0, 10.0));

    // 検証器そのものを、エンジンを介さずに直接試す。
    let report = BooleanResultVerifier::verify(
        &a,
        &b,
        std::slice::from_ref(&a),
        BooleanOpType::Union,
        &tol,
    );

    assert!(
        !report.is_valid(),
        "handing operand A back as the union must be rejected: {}",
        report.summary()
    );
    assert!(
        report.membership_mismatch_count > 0,
        "the point membership check should be what catches it: {}",
        report.summary()
    );
}

#[test]
fn test_verifier_accepts_a_genuine_union_result() {
    let tol = Tolerance::default();
    let a = PrimitiveBuilder::make_box(20.0, 20.0, 20.0).unwrap();
    let b = BrepTransform::translate_solid(&a, Vec3::new(10.0, 10.0, 10.0));

    let union = BooleanEngine::boolean_solids_exact_result(&a, &b, BooleanOpType::Union, &tol)
        .expect("axis-aligned box union should succeed");

    let report =
        BooleanResultVerifier::verify(&a, &b, &union.solids, BooleanOpType::Union, &tol);

    assert!(
        report.is_valid(),
        "a correct union must not be flagged: {}",
        report.summary()
    );
    assert_eq!(report.membership_mismatch_count, 0);
}
