use zenith_algo::{ExtrudeBuilder, MassCalculator};
use zenith_io::{StepExporter, StepImporter};
use zenith_math::{Point3, Tolerance, Vec3};
use zenith_tess::{tessellate_solid, TessellationParams};
use zenith_topo::{Edge, OrientedEdge, Vertex, Wire};

#[test]
fn test_extrude_hollow_rectangular_tube_solid() {
    let tol = Tolerance::default();

    // 1. 外側長方形ワイヤ (30 x 20, 原点中心, CCW)
    let p_out = vec![
        Point3::new(-15.0, -10.0, 0.0),
        Point3::new(15.0, -10.0, 0.0),
        Point3::new(15.0, 10.0, 0.0),
        Point3::new(-15.0, 10.0, 0.0),
    ];
    let v_out: Vec<Vertex> = p_out.into_iter().map(Vertex::from_point).collect();
    let mut e_out = Vec::new();
    for i in 0..4 {
        let next_i = (i + 1) % 4;
        let e = Edge::line_between(v_out[i].clone(), v_out[next_i].clone()).unwrap();
        e_out.push(OrientedEdge::forward(e));
    }
    let outer_wire = Wire::new(e_out);

    // 2. 内側角穴ワイヤ (16 x 10, 原点中心, CCW)
    let p_in = vec![
        Point3::new(-8.0, -5.0, 0.0),
        Point3::new(8.0, -5.0, 0.0),
        Point3::new(8.0, 5.0, 0.0),
        Point3::new(-8.0, 5.0, 0.0),
    ];
    let v_in: Vec<Vertex> = p_in.into_iter().map(Vertex::from_point).collect();
    let mut e_in = Vec::new();
    for i in 0..4 {
        let next_i = (i + 1) % 4;
        let e = Edge::line_between(v_in[i].clone(), v_in[next_i].clone()).unwrap();
        e_in.push(OrientedEdge::forward(e));
    }
    let inner_wire = Wire::new(e_in);

    // 3. 押し出し実行 (高さ 25.0, +Z 方向)
    let dir = Vec3::new(0.0, 0.0, 25.0);
    let hollow_tube = ExtrudeBuilder::extrude_face_with_holes(&outer_wire, &[inner_wire], dir, &tol)
        .expect("Extrude hollow tube should succeed");

    // 4. トポロジー検証
    let report = hollow_tube.outer_shell.validate_closed(&tol);
    assert!(
        report.is_valid(),
        "Hollow tube solid validation failed: {:?}",
        report.errors
    );
    assert_eq!(hollow_tube.outer_shell.faces.len(), 4 + 4 + 2); // 外壁4 + 内壁4 + 底面1 + 天面1 = 10面

    // 5. 物性値検証（解析体積との一致）
    let expected_volume = (30.0 * 20.0 - 16.0 * 10.0) * 25.0; // 440 * 25 = 11000
    let tess_params = TessellationParams::default();
    let mesh = tessellate_solid(&hollow_tube, &tess_params);
    let mass = MassCalculator::compute_from_mesh(&mesh);
    assert!(
        (mass.volume - expected_volume).abs() < 1.0,
        "Hollow tube volume error: got {}, expected {}",
        mass.volume,
        expected_volume
    );

    // 6. STEP ラウンドトリップ検証
    let step_str = StepExporter::export_solid_to_string(&hollow_tube, "ZENITH_HOLLOW_TUBE");
    let imported_solid = StepImporter::import_solid_from_str(&step_str)
        .expect("STEP import of hollow tube should succeed");

    let imported_report = imported_solid.outer_shell.validate_closed(&tol);
    assert!(
        imported_report.is_valid(),
        "Imported hollow tube validation failed: {:?}",
        imported_report.errors
    );
}
