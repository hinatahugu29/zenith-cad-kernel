use zenith_algo::{MassCalculator, PolylineBuilder};
use zenith_io::StepExporter;
use zenith_math::{Point3, Tolerance};

#[test]
fn test_polyline_fillet_and_sweep_pipe_solid() {
    let tol = Tolerance::default();

    // 直角に曲がる3Dポリライン配管パス (0,0,0) -> (50,0,0) -> (50,50,0) -> (50,50,50)
    let path = vec![
        Point3::new(0.0, 0.0, 0.0),
        Point3::new(50.0, 0.0, 0.0),
        Point3::new(50.0, 50.0, 0.0),
        Point3::new(50.0, 50.0, 50.0),
    ];

    let pipe_radius = 4.0;
    let corner_radius = 12.0;

    let solid = PolylineBuilder::sweep_pipe_polyline(&path, pipe_radius, corner_radius, &tol)
        .expect("sweep_pipe_polyline failed");

    // 閉シェル検証
    let report = solid.outer_shell.validate_closed(&tol);
    assert!(
        report.is_valid(),
        "Polyline pipe shell validation failed: {:?}",
        report.errors
    );

    // 体積計算
    let params = zenith_tess::TessellationParams {
        u_divisions: 16,
        v_divisions: 16,
    };
    let mass = MassCalculator::compute_from_brep(&solid, &params);
    println!("Polyline Pipe Volume: {:.2} mm^3", mass.volume);
    assert!(mass.volume > 5000.0, "Pipe volume is too small");

    // STEP 出力テスト
    let step_str = StepExporter::export_solid_to_string(&solid, "ZENITH_POLYLINE_PIPE");
    assert!(step_str.contains("MANIFOLD_SOLID_BREP"));
    assert!(step_str.contains("B_SPLINE_SURFACE_WITH_KNOTS"));
}
