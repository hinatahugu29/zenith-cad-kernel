use zenith_algo::{HoleBuilder, MassCalculator};
use zenith_io::{StepExporter, StepImporter};
use zenith_math::Tolerance;
use zenith_tess::TessellationParams;

#[test]
fn test_counterbored_slot_box() {
    let tol = Tolerance::default();
    let box_w = 80.0;
    let box_d = 60.0;
    let box_h = 20.0;
    let slot_l = 20.0;
    let slot_r = 5.0;
    let cb_l = 20.0;
    let cb_r = 8.0;
    let cb_d = 6.0;
    let cx = 40.0;
    let cy = 30.0;

    let solid = HoleBuilder::make_counterbored_slot_box(
        box_w,
        box_d,
        box_h,
        slot_l,
        slot_r,
        cb_l,
        cb_r,
        cb_d,
        cx,
        cy,
    )
    .expect("counterbored slot box");

    // 1. B-Rep 閉多様体検証
    assert!(
        solid.outer_shell.validate_closed(&tol).is_valid(),
        "Counterbored slot solid must be valid closed manifold"
    );

    // 2. 閉形式体積一致検証
    let pi = std::f64::consts::PI;
    let s_thru = slot_l * (2.0 * slot_r) + pi * slot_r * slot_r;
    let s_cb = cb_l * (2.0 * cb_r) + pi * cb_r * cb_r;
    let v_box = box_w * box_d * box_h;
    let v_thru = s_thru * (box_h - cb_d);
    let v_cb = s_cb * cb_d;
    let expected_vol = v_box - v_thru - v_cb;

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
    let step_str = StepExporter::export_solid_to_string(&solid, "CounterboredSlotBox");
    let reimported = StepImporter::import_solid_from_str(&step_str).expect("import STEP");
    assert!(
        reimported.outer_shell.validate_closed(&tol).is_valid(),
        "Reimported counterbored slot must be valid closed"
    );
}
