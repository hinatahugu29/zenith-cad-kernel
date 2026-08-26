use std::f64::consts::PI;
use zenith_algo::{MassCalculator, ShellingBuilder};
use zenith_io::{StepExporter, StepImporter};
use zenith_math::Tolerance;
use zenith_tess::{tessellate_solid, TessellationParams};

#[test]
fn test_open_box_shelling() {
    let tol = Tolerance::default();
    let dx = 60.0;
    let dy = 40.0;
    let dz = 30.0;
    let t = 3.0;

    let solid = ShellingBuilder::make_open_box(dx, dy, dz, t).expect("open box");
    assert!(
        solid.outer_shell.validate_closed(&tol).is_valid(),
        "Open-box validation failed"
    );

    let expected_vol = (dx * dy * dz) - ((dx - 2.0 * t) * (dy - 2.0 * t) * (dz - t));
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

    let mesh = tessellate_solid(&solid, &params);
    assert!(mesh.num_triangles() > 0, "Mesh should have triangles");

    let step_str = StepExporter::export_solid_to_string(&solid, "OpenBox");
    let reimported = StepImporter::import_solid_from_str(&step_str).expect("import STEP");
    assert!(reimported.outer_shell.validate_closed(&tol).is_valid(), "Reimported solid must be valid closed");
}

#[test]
fn test_open_cylinder_shelling() {
    let tol = Tolerance::default();
    let radius = 25.0;
    let height = 40.0;
    let t = 2.5;

    let solid = ShellingBuilder::make_open_cylinder(radius, height, t).expect("open cylinder");
    assert!(
        solid.outer_shell.validate_closed(&tol).is_valid(),
        "Open-cylinder validation failed"
    );

    let expected_vol = (PI * radius * radius * height)
        - (PI * (radius - t) * (radius - t) * (height - t));
    let params = TessellationParams {
        u_divisions: 48,
        v_divisions: 48,
    };
    let mass = MassCalculator::compute_from_brep(&solid, &params);
    let vol_diff = (mass.volume - expected_vol).abs() / expected_vol;
    assert!(
        vol_diff < 1e-12,
        "Volume mismatch: computed={}, expected={}, diff={vol_diff}",
        mass.volume,
        expected_vol
    );

    let mesh = tessellate_solid(&solid, &params);
    assert!(mesh.num_triangles() > 0, "Mesh should have triangles");

    let step_str = StepExporter::export_solid_to_string(&solid, "OpenCylinder");
    let reimported = StepImporter::import_solid_from_str(&step_str).expect("import STEP");
    assert!(reimported.outer_shell.validate_closed(&tol).is_valid(), "Reimported solid must be valid closed");
}

#[test]
fn test_open_slot_prism_shelling() {
    let tol = Tolerance::default();
    let length = 30.0;
    let radius = 12.0;
    let height = 35.0;
    let t = 2.0;

    let solid = ShellingBuilder::make_open_slot_prism(length, radius, height, t).expect("open slot tray");
    assert!(
        solid.outer_shell.validate_closed(&tol).is_valid(),
        "Open-slot-prism validation failed"
    );

    let r_out = radius;
    let r_in = radius - t;
    let h_out = height;
    let h_in = height - t;
    let v_out = (2.0 * length * r_out + PI * r_out * r_out) * h_out;
    let v_in = (2.0 * length * r_in + PI * r_in * r_in) * h_in;
    let expected_vol = v_out - v_in;

    let params = TessellationParams {
        u_divisions: 48,
        v_divisions: 48,
    };
    let mass = MassCalculator::compute_from_brep(&solid, &params);
    let vol_diff = (mass.volume - expected_vol).abs() / expected_vol;
    assert!(
        vol_diff < 1e-12,
        "Volume mismatch: computed={}, expected={}, diff={vol_diff}",
        mass.volume,
        expected_vol
    );

    let mesh = tessellate_solid(&solid, &params);
    assert!(mesh.num_triangles() > 0, "Mesh should have triangles");

    let step_str = StepExporter::export_solid_to_string(&solid, "OpenSlotTray");
    let reimported = StepImporter::import_solid_from_str(&step_str).expect("import STEP");
    assert!(reimported.outer_shell.validate_closed(&tol).is_valid(), "Reimported solid must be valid closed");
}
