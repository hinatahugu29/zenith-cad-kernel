use zenith_algo::{DraftBuilder, MassCalculator};
use zenith_io::{StepExporter, StepImporter};
use zenith_math::Tolerance;
use zenith_tess::{tessellate_solid, TessellationParams};

#[test]
fn test_drafted_block() {
    let tol = Tolerance::default();
    let dx = 40.0;
    let dy = 30.0;
    let dz = 20.0;
    let angle_deg = 5.0;
    let angle_rad = angle_deg * std::f64::consts::PI / 180.0;

    let solid =
        DraftBuilder::make_drafted_block(dx, dy, dz, angle_rad, &tol).expect("drafted block");

    // 1. B-Rep 閉多様体検証
    assert!(
        solid.outer_shell.validate_closed(&tol).is_valid(),
        "Drafted block must be valid closed manifold"
    );

    // 2. 抜き勾配角錐台の厳密な閉形式体積一致検証
    // 直方体本体: dx * dy * dz
    // 左右・前後4つの三角柱: dz * delta * (dx + dy)
    // 四隅4つの四角錐: (4/3) * dz * delta^2
    let tan_a = angle_rad.tan();
    let delta = dz * tan_a;
    let expected_vol = dz * (dx * dy + delta * (dx + dy) + (4.0 / 3.0) * delta * delta);

    let params = TessellationParams {
        u_divisions: 32,
        v_divisions: 32,
    };
    let mass = MassCalculator::compute_from_brep(&solid, &params);
    let vol_diff = (mass.volume - expected_vol).abs() / expected_vol;
    assert!(
        vol_diff < 1e-12,
        "Volume mismatch: computed={}, expected={}, diff={vol_diff}",
        mass.volume,
        expected_vol
    );

    // 3. テッセレーション検証
    let mesh = tessellate_solid(&solid, &params);
    assert!(mesh.num_triangles() > 0, "Mesh should have triangles");

    // 4. STEP 往復検証
    let step_str = StepExporter::export_solid_to_string(&solid, "DraftedBlock");
    let reimported = StepImporter::import_solid_from_str(&step_str).expect("import STEP");
    assert!(
        reimported.outer_shell.validate_closed(&tol).is_valid(),
        "Reimported solid must be valid closed"
    );
}
