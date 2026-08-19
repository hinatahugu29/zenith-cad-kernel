use zenith_algo::{MassCalculator, SweepBuilder};
use zenith_geom::NurbsCurve3;
use zenith_io::{StepExporter, StepImporter};
use zenith_math::{Point3, Tolerance};
use zenith_tess::{tessellate_solid, TessellationParams};
use zenith_topo::{Edge, OrientedEdge, Vertex, Wire};

#[test]
fn test_sweep_rectangular_wire_along_3d_spline_path() {
    let tol = Tolerance::default();

    // 1. 2D 長方形断面ワイヤ (XY平面, 幅 8.0, 高さ 4.0)
    let p0 = Point3::new(-4.0, -2.0, 0.0);
    let p1 = Point3::new(4.0, -2.0, 0.0);
    let p2 = Point3::new(4.0, 2.0, 0.0);
    let p3 = Point3::new(-4.0, 2.0, 0.0);

    let v0 = Vertex::from_point(p0);
    let v1 = Vertex::from_point(p1);
    let v2 = Vertex::from_point(p2);
    let v3 = Vertex::from_point(p3);

    let e0 = Edge::line_between(v0.clone(), v1.clone()).unwrap();
    let e1 = Edge::line_between(v1.clone(), v2.clone()).unwrap();
    let e2 = Edge::line_between(v2.clone(), v3.clone()).unwrap();
    let e3 = Edge::line_between(v3.clone(), v0.clone()).unwrap();

    let profile_wire = Wire::new(vec![
        OrientedEdge::forward(e0),
        OrientedEdge::forward(e1),
        OrientedEdge::forward(e2),
        OrientedEdge::forward(e3),
    ]);

    // 2. 3次元 S 字スプラインパス曲線
    let path_points = vec![
        Point3::new(0.0, 0.0, 0.0),
        Point3::new(10.0, 20.0, 15.0),
        Point3::new(30.0, -10.0, 35.0),
        Point3::new(50.0, 10.0, 60.0),
    ];
    let path = NurbsCurve3::bspline_from_points(3, path_points)
        .expect("3D spline path creation should succeed");

    // 3. スイープソリッドの生成
    let sweep_solid = SweepBuilder::sweep_wire_along_curve(&profile_wire, &path, 16, &tol)
        .expect("SweepBuilder::sweep_wire_along_curve should succeed");

    // 4. B-Rep トポロジー閉シェル検証
    let report = sweep_solid.outer_shell.validate_closed(&tol);
    assert!(
        report.is_valid(),
        "Sweep wire solid validation failed: {:?}",
        report.errors
    );
    assert_eq!(sweep_solid.outer_shell.faces.len(), 4 + 2); // 4側面 + 始端面 + 終端面 = 6面

    // 5. テッセレーション ＆ 物性値計算
    let tess_params = TessellationParams::default();
    let mesh = tessellate_solid(&sweep_solid, &tess_params);
    assert!(!mesh.positions.is_empty());
    assert!(!mesh.indices.is_empty());

    let mass = MassCalculator::compute_from_mesh(&mesh);
    assert!(mass.volume > 0.0, "Sweep solid volume must be positive: got {}", mass.volume);
    assert!(mass.surface_area > 0.0, "Surface area must be positive: got {}", mass.surface_area);

    // 6. STEP ラウンドトリップ検証
    let step_str = StepExporter::export_solid_to_string(&sweep_solid, "ZENITH_SWEEP_WIRE_SOLID");
    let imported_solid = StepImporter::import_solid_from_str(&step_str)
        .expect("STEP import of sweep wire solid should succeed");

    let imported_report = imported_solid.outer_shell.validate_closed(&tol);
    assert!(
        imported_report.is_valid(),
        "Imported sweep wire solid validation failed: {:?}",
        imported_report.errors
    );
}
