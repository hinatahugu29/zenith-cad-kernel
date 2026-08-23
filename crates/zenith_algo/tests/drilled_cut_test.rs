use zenith_algo::{BooleanEngine, BooleanOpType, BrepTransform, HoleBuilder, MassCalculator, PrimitiveBuilder};
use zenith_math::{Tolerance, Vec3};
use zenith_tess::TessellationParams;

#[test]
fn test_drilled_box_side_slab_cut_matches_analytic_volume() {
    let tol = Tolerance::default();
    let drilled = HoleBuilder::make_drilled_box(30.0, 30.0, 15.0, 5.0).expect("drilled");
    let slab = BrepTransform::translate_solid(
        &PrimitiveBuilder::make_box(20.0, 60.0, 40.0).expect("side slab"),
        Vec3::new(24.0, -15.0, -10.0),
    );

    let result = BooleanEngine::boolean_solids_exact_result(
        &drilled,
        &slab,
        BooleanOpType::Difference,
        &tol,
    )
    .expect("side slab cut difference on drilled box");

    assert_eq!(result.solids.len(), 1);
    let volume = MassCalculator::compute_from_brep(&result.solids[0], &TessellationParams::default()).volume;

    // 解析解: (30*30 - pi*5^2)*15 - (6*30*15) = 12321.902755 - 2700 = 9621.902755
    let expected = (30.0 * 30.0 - std::f64::consts::PI * 25.0) * 15.0 - (6.0 * 30.0 * 15.0);
    let rel_err = (volume - expected).abs() / expected;
    assert!(
        rel_err < 1e-6,
        "volume {volume} differed from expected {expected} by {rel_err:.3e}"
    );
}

#[test]
fn test_drilled_box_corner_block_cut_matches_analytic_volume() {
    let tol = Tolerance::default();
    let drilled = HoleBuilder::make_drilled_box(30.0, 30.0, 15.0, 5.0).expect("drilled");
    let corner = BrepTransform::translate_solid(
        &PrimitiveBuilder::make_box(6.0, 6.0, 40.0).expect("corner block"),
        Vec3::new(-3.0, -3.0, -10.0),
    );

    let result = BooleanEngine::boolean_solids_exact_result(
        &drilled,
        &corner,
        BooleanOpType::Difference,
        &tol,
    )
    .expect("corner block cut difference on drilled box");

    assert_eq!(result.solids.len(), 1);
    let volume = MassCalculator::compute_from_brep(&result.solids[0], &TessellationParams::default()).volume;

    // 解析解: (30*30 - pi*5^2)*15 - (3*3*15) = 12321.902755 - 135 = 12186.902755
    let expected = (30.0 * 30.0 - std::f64::consts::PI * 25.0) * 15.0 - (3.0 * 3.0 * 15.0);
    let rel_err = (volume - expected).abs() / expected;
    assert!(
        rel_err < 1e-6,
        "volume {volume} differed from expected {expected} by {rel_err:.3e}"
    );
}
