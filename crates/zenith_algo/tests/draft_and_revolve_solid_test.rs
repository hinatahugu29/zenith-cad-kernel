use zenith_algo::{ExtrudeBuilder, MassCalculator, RevolveBuilder};
use zenith_io::{StepExporter, StepImporter};
use zenith_math::{Point3, Tolerance, Vec3};
use zenith_tess::{tessellate_solid, TessellationParams};
use zenith_topo::{Edge, OrientedEdge, Vertex, Wire};

fn make_rect_wire(min_x: f64, max_x: f64, min_y_or_z: f64, max_y_or_z: f64, is_xz: bool) -> Wire {
    let pts = if is_xz {
        vec![
            Point3::new(min_x, 0.0, min_y_or_z),
            Point3::new(max_x, 0.0, min_y_or_z),
            Point3::new(max_x, 0.0, max_y_or_z),
            Point3::new(min_x, 0.0, max_y_or_z),
        ]
    } else {
        vec![
            Point3::new(min_x, min_y_or_z, 0.0),
            Point3::new(max_x, min_y_or_z, 0.0),
            Point3::new(max_x, max_y_or_z, 0.0),
            Point3::new(min_x, max_y_or_z, 0.0),
        ]
    };

    let vertices: Vec<Vertex> = pts.into_iter().map(Vertex::from_point).collect();
    let mut edges = Vec::with_capacity(4);
    for i in 0..4 {
        let next_i = (i + 1) % 4;
        let edge = Edge::line_between(vertices[i].clone(), vertices[next_i].clone()).unwrap();
        edges.push(OrientedEdge::forward(edge));
    }
    Wire::new(edges)
}

#[test]
fn test_extrude_wire_with_draft() {
    let tol = Tolerance::default();
    let wire = make_rect_wire(-15.0, 15.0, -10.0, 10.0, false); // 30 x 20
    let dir = Vec3::new(0.0, 0.0, 20.0);
    let draft_angle = 5.0_f64.to_radians();

    let solid = ExtrudeBuilder::extrude_wire_with_draft(&wire, dir, draft_angle, &tol)
        .expect("draft extrude solid");

    // 1. トポロジー検証（6面閉シェル）
    assert_eq!(solid.outer_shell.faces.len(), 6);
    let report = solid.outer_shell.validate_closed(&tol);
    assert!(report.is_valid(), "Validation errors: {:?}", report.errors);

    // 2. 体積検証（角錐台の体積計算）
    let mesh = tessellate_solid(&solid, &TessellationParams::default());
    let mass = MassCalculator::compute_from_mesh(&mesh);
    assert!(mass.volume > 30.0 * 20.0 * 20.0, "Volume should be larger due to positive draft");

    // 3. STEP ラウンドトリップ
    let step_path = "test_draft_extrude_roundtrip.step";
    StepExporter::export_solid_to_file(&solid, step_path, "DRAFT_EXTRUDE_SOLID")
        .expect("STEP export failed");
    let imported = StepImporter::import_solid_from_file(step_path).expect("STEP import failed");
    let _ = std::fs::remove_file(step_path);

    assert_eq!(imported.outer_shell.faces.len(), 6);
}

#[test]
fn test_revolve_wire_solid_hollow_cylinder() {
    let tol = Tolerance::default();
    // X in [10, 15], Z in [0, 20] (幅5, 高さ20, 内半径10, 外半径15)
    let wire = make_rect_wire(10.0, 15.0, 0.0, 20.0, true);
    let axis_origin = Point3::new(0.0, 0.0, 0.0);
    let axis_dir = Vec3::new(0.0, 0.0, 1.0);

    let solid = RevolveBuilder::revolve_wire_solid(&wire, axis_origin, axis_dir, &tol)
        .expect("revolve wire solid");

    // 1. トポロジー検証（4セグメント x 4エッジ = 16面閉シェル）
    assert_eq!(solid.outer_shell.faces.len(), 16);
    let report = solid.outer_shell.validate_closed(&tol);
    assert!(report.is_valid(), "Validation errors: {:?}", report.errors);

    // 2. 体積検証（解析体積: V = pi * (15^2 - 10^2) * 20 = 2500 * pi = 7853.98 mm^3）
    let params = TessellationParams {
        u_divisions: 16,
        v_divisions: 16,
    };
    let mesh = tessellate_solid(&solid, &params);
    let mass = MassCalculator::compute_from_mesh(&mesh);
    let expected_vol = std::f64::consts::PI * (15.0 * 15.0 - 10.0 * 10.0) * 20.0;
    let rel_err = (mass.volume - expected_vol).abs() / expected_vol;
    assert!(
        rel_err < 0.02,
        "Revolve volume relative error too large: got {}, expected {}",
        mass.volume,
        expected_vol
    );

    // 3. STEP ラウンドトリップ
    let step_path = "test_revolve_solid_roundtrip.step";
    StepExporter::export_solid_to_file(&solid, step_path, "REVOLVE_SOLID")
        .expect("STEP export failed");
    let imported = StepImporter::import_solid_from_file(step_path).expect("STEP import failed");
    let _ = std::fs::remove_file(step_path);

    assert_eq!(imported.outer_shell.faces.len(), 16);
}
