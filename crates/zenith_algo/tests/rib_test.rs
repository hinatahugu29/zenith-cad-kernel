use zenith_algo::{MassCalculator, RibBuilder};
use zenith_io::{StepExporter, StepImporter};
use zenith_math::Tolerance;
use zenith_tess::TessellationParams;

#[test]
fn test_triangular_rib() {
    let tol = Tolerance::default();
    let length = 30.0;
    let height = 25.0;
    let thickness = 6.0;

    let solid =
        RibBuilder::make_triangular_rib(length, height, thickness, &tol).expect("triangular rib");

    // 1. B-Rep 閉多様体検証
    assert!(
        solid.outer_shell.validate_closed(&tol).is_valid(),
        "Rib solid must be valid closed manifold"
    );

    // 2. 閉形式体積一致検証
    let expected_vol = 0.5 * length * height * thickness;
    let params = TessellationParams {
        u_divisions: 24,
        v_divisions: 24,
    };
    let mass = MassCalculator::compute_from_brep(&solid, &params);
    let vol_diff = (mass.volume - expected_vol).abs() / expected_vol;
    assert!(
        vol_diff < 1e-12,
        "Volume mismatch: computed={}, expected={}, diff={vol_diff}",
        mass.volume,
        expected_vol
    );

    // 3. STEP 往復検証
    let step_str = StepExporter::export_solid_to_string(&solid, "TriangularRib");
    let reimported = StepImporter::import_solid_from_str(&step_str).expect("import STEP");
    assert!(
        reimported.outer_shell.validate_closed(&tol).is_valid(),
        "Reimported rib must be valid closed"
    );
}

#[test]
fn test_slanted_triangular_rib() {
    let tol = Tolerance::default();
    let length = 45.0;
    let height = 35.0;
    let thickness = 8.0;

    let solid = RibBuilder::make_triangular_rib(length, height, thickness, &tol)
        .expect("slanted triangular rib");

    // 1. B-Rep 閉多様体検証
    assert!(
        solid.outer_shell.validate_closed(&tol).is_valid(),
        "Slanted rib must be valid closed manifold"
    );

    // 2. 閉形式体積一致検証
    let expected_vol = 0.5 * length * height * thickness;
    let params = TessellationParams {
        u_divisions: 24,
        v_divisions: 24,
    };
    let mass = MassCalculator::compute_from_brep(&solid, &params);
    let vol_diff = (mass.volume - expected_vol).abs() / expected_vol;
    assert!(
        vol_diff < 1e-12,
        "Volume mismatch: computed={}, expected={}, diff={vol_diff}",
        mass.volume,
        expected_vol
    );

    // 3. STEP 往復検証
    let step_str = StepExporter::export_solid_to_string(&solid, "SlantedTriangularRib");
    let reimported = StepImporter::import_solid_from_str(&step_str).expect("import STEP");
    assert!(
        reimported.outer_shell.validate_closed(&tol).is_valid(),
        "Reimported slanted rib must be valid closed"
    );
}
