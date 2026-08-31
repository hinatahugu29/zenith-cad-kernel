use zenith_algo::{DirectModeling, MassCalculator};
use zenith_io::{StepExporter, StepImporter};
use zenith_math::Tolerance;
use zenith_tess::{tessellate_solid, TessellationParams};

#[test]
fn test_chamfer_box_single_edge_all_four_corners() {
    let tol = Tolerance::default();
    let dx = 30.0;
    let dy = 20.0;
    let dz = 15.0;
    let chamfer_dist = 4.0;

    let expected_volume = (dx * dy * dz) - (0.5 * chamfer_dist * chamfer_dist * dz);

    for edge_idx in 0..4 {
        let solid = DirectModeling::chamfer_box_single_edge(dx, dy, dz, edge_idx, chamfer_dist)
            .unwrap_or_else(|e| panic!("chamfer_box_single_edge({}) failed: {}", edge_idx, e));

        // 1. トポロジー検証
        let report = solid.outer_shell.validate_closed(&tol);
        assert!(
            report.is_valid(),
            "Chamfer solid (edge {}) failed closed validation: {:?}",
            edge_idx,
            report.errors
        );
        assert_eq!(solid.outer_shell.faces.len(), 7); // 側面5 + 底面1 + 天面1 = 7面

        // 2. 物性値検証（解析体積との一致）
        let tess_params = TessellationParams::default();
        let mesh = tessellate_solid(&solid, &tess_params);
        let mass = MassCalculator::compute_from_mesh(&mesh);
        assert!(
            (mass.volume - expected_volume).abs() < 1.0,
            "Edge {} volume error: got {}, expected {}",
            edge_idx,
            mass.volume,
            expected_volume
        );

        // 3. STEP ラウンドトリップ
        let step_str = StepExporter::export_solid_to_string(
            &solid,
            &format!("ZENITH_CHAMFER_EDGE_{}", edge_idx),
        );
        let imported = StepImporter::import_solid_from_str(&step_str)
            .expect("STEP import of chamfer solid should succeed");
        assert!(imported.outer_shell.validate_closed(&tol).is_valid());
    }
}
