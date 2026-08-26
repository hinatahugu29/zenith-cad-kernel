use zenith_algo::{HoleBuilder, MassCalculator};
use zenith_io::{StepExporter, StepImporter};
use zenith_math::Tolerance;
use zenith_tess::TessellationParams;

#[test]
fn test_countersink_hole_block() {
    let tol = Tolerance::default();
    let box_w = 60.0;
    let box_d = 50.0;
    let box_h = 20.0;
    let cx = 30.0;
    let cy = 25.0;
    let hole_r = 5.0;
    let cs_r = 9.0;
    let cs_angle_deg = 90.0; // 90度皿ビス

    let solid = HoleBuilder::make_countersink_hole_box(
        box_w,
        box_d,
        box_h,
        hole_r,
        cs_r,
        cs_angle_deg,
        cx,
        cy,
    )
    .expect("countersink hole block");

    // 1. B-Rep 閉多様体検証
    assert!(
        solid.outer_shell.validate_closed(&tol).is_valid(),
        "Countersink solid must be valid closed manifold"
    );

    // 2. 閉形式体積一致検証
    let half_angle_rad: f64 = (cs_angle_deg * 0.5).to_radians();
    let tan_half = half_angle_rad.tan();
    let cs_depth = (cs_r - hole_r) / tan_half; // (9 - 5) / 1.0 = 4.0
    let pi = std::f64::consts::PI;
    let v_box = box_w * box_d * box_h;
    let v_drill = pi * hole_r * hole_r * (box_h - cs_depth);
    let v_cone = (pi * cs_depth / 3.0) * (cs_r * cs_r + cs_r * hole_r + hole_r * hole_r);
    let expected_vol = v_box - v_drill - v_cone;

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
    let step_str = StepExporter::export_solid_to_string(&solid, "CountersinkHoleBlock");
    let reimported = StepImporter::import_solid_from_str(&step_str).expect("import STEP");
    assert!(
        reimported.outer_shell.validate_closed(&tol).is_valid(),
        "Reimported countersink must be valid closed"
    );
}
