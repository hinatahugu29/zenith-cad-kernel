use std::f64::consts::PI;
use zenith_algo::{MassCalculator, PrimitiveBuilder};
use zenith_math::Tolerance;
use zenith_tess::{tessellate_solid, TessellationParams};

#[test]
fn test_slot_prism_exact_volume_and_area() {
    let tol = Tolerance::default();
    let length = 20.0;
    let radius = 5.0;
    let height = 30.0;

    let slot = PrimitiveBuilder::make_slot_prism(length, radius, height)
        .expect("slot prism should build");

    // 閉多様体検証
    let valid = slot.outer_shell.validate_closed(&tol);
    assert!(valid.is_valid(), "slot prism must be a valid closed shell");

    let params = TessellationParams {
        u_divisions: 64,
        v_divisions: 64,
    };
    let mass = MassCalculator::compute_from_brep(&slot, &params);

    // 体積の解析解
    let expected_volume = (length * 2.0 * radius + PI * radius * radius) * height;
    let actual_volume = mass.volume;
    let vol_err = (actual_volume - expected_volume).abs() / expected_volume;
    assert!(vol_err < 1e-4, "Volume error {vol_err:.3e} exceeds 1e-4 (actual {actual_volume}, expected {expected_volume})");

    // 表面積の解析解
    let expected_area = 2.0 * (length * 2.0 * radius + PI * radius * radius)
        + (2.0 * length + 2.0 * PI * radius) * height;
    let actual_area = mass.surface_area;
    let area_err = (actual_area - expected_area).abs() / expected_area;
    assert!(area_err < 1e-4, "Area error {area_err:.3e} exceeds 1e-4 (actual {actual_area}, expected {expected_area})");

    // メッシュ水密性検証
    let mesh = tessellate_solid(&slot, &params);
    assert!(!mesh.indices.is_empty(), "Mesh should have triangles");
}
