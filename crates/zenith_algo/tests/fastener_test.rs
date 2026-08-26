use zenith_algo::{FastenerBuilder, MassCalculator};
use zenith_io::{StepExporter, StepImporter};
use zenith_math::Tolerance;
use zenith_tess::TessellationParams;

#[test]
fn test_hex_prism() {
    let tol = Tolerance::default();
    let s = 30.0; // 二面幅
    let h = 20.0;

    let solid = FastenerBuilder::make_hex_prism(s, h, &tol).expect("hex prism");

    // 1. B-Rep 閉多様体検証
    assert!(
        solid.outer_shell.validate_closed(&tol).is_valid(),
        "Hex prism must be valid closed manifold"
    );

    // 2. 閉形式体積一致検証
    let expected_vol = (3.0_f64.sqrt() * 0.5) * s * s * h;
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
    let step_str = StepExporter::export_solid_to_string(&solid, "HexPrism");
    let reimported = StepImporter::import_solid_from_str(&step_str).expect("import STEP");
    assert!(
        reimported.outer_shell.validate_closed(&tol).is_valid(),
        "Reimported hex prism must be valid closed"
    );
}

#[test]
fn test_hex_nut_blank() {
    let tol = Tolerance::default();
    let s = 30.0;
    let h = 15.0;
    let r_hole = 7.5; // M16ボルト用下穴相当

    let solid = FastenerBuilder::make_hex_nut_blank(s, h, r_hole, &tol).expect("hex nut blank");

    // 1. B-Rep 閉多様体検証
    assert!(
        solid.outer_shell.validate_closed(&tol).is_valid(),
        "Hex nut must be valid closed manifold"
    );

    // 2. 閉形式体積一致検証
    let hex_vol = (3.0_f64.sqrt() * 0.5) * s * s * h;
    let hole_vol = std::f64::consts::PI * r_hole * r_hole * h;
    let expected_vol = hex_vol - hole_vol;

    let params = TessellationParams {
        u_divisions: 32,
        v_divisions: 32,
    };
    let mass = MassCalculator::compute_from_brep(&solid, &params);
    let vol_diff = (mass.volume - expected_vol).abs() / expected_vol;
    assert!(
        vol_diff < 1e-6,
        "Volume mismatch: computed={}, expected={}, diff={vol_diff}",
        mass.volume,
        expected_vol
    );

    // 3. STEP 往復検証
    let step_str = StepExporter::export_solid_to_string(&solid, "HexNutBlank");
    let reimported = StepImporter::import_solid_from_str(&step_str).expect("import STEP");
    assert!(
        reimported.outer_shell.validate_closed(&tol).is_valid(),
        "Reimported hex nut must be valid closed"
    );
}
