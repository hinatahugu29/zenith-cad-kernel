use zenith_algo::{MassCalculator, PatternBuilder, PrimitiveBuilder, RevolveBuilder};
use zenith_io::{StepExporter, StepImporter};
use zenith_math::{Point3, Tolerance, Vec3};
use zenith_tess::{tessellate_solid, TessellationParams};
use zenith_topo::{Edge, OrientedEdge, Vertex, Wire};

fn make_rect_wire_xz(min_x: f64, max_x: f64, min_z: f64, max_z: f64) -> Wire {
    let pts = vec![
        Point3::new(min_x, 0.0, min_z),
        Point3::new(max_x, 0.0, min_z),
        Point3::new(max_x, 0.0, max_z),
        Point3::new(min_x, 0.0, max_z),
    ];
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
fn test_partial_revolve_90_and_180_deg() {
    let tol = Tolerance::default();
    let wire = make_rect_wire_xz(10.0, 15.0, 0.0, 20.0);
    let axis_origin = Point3::new(0.0, 0.0, 0.0);
    let axis_dir = Vec3::new(0.0, 0.0, 1.0);

    // 1. 90度回転ソリッド（1セグメント x 4エッジ + 2端面 = 6面閉シェル）
    let angle_90 = std::f64::consts::FRAC_PI_2;
    let solid_90 = RevolveBuilder::revolve_wire_partial_solid(&wire, axis_origin, axis_dir, angle_90, &tol)
        .expect("90 deg revolve solid");

    assert_eq!(solid_90.outer_shell.faces.len(), 6);
    let report_90 = solid_90.outer_shell.validate_closed(&tol);
    assert!(report_90.is_valid(), "90 deg validation errors: {:?}", report_90.errors);

    let params = TessellationParams {
        u_divisions: 16,
        v_divisions: 16,
    };
    let mesh_90 = tessellate_solid(&solid_90, &params);
    let mass_90 = MassCalculator::compute_from_mesh(&mesh_90);
    let expected_90 = std::f64::consts::PI * (15.0 * 15.0 - 10.0 * 10.0) * 20.0 * (90.0 / 360.0);
    assert!(
        (mass_90.volume - expected_90).abs() / expected_90 < 0.02,
        "90 deg volume error: got {}, expected {}",
        mass_90.volume,
        expected_90
    );

    // STEPラウンドトリップ
    let step_path = "test_revolve_90_roundtrip.step";
    StepExporter::export_solid_to_file(&solid_90, step_path, "REVOLVE_90")
        .expect("STEP export failed");
    let imported_90 = StepImporter::import_solid_from_file(step_path).expect("STEP import failed");
    let _ = std::fs::remove_file(step_path);
    assert_eq!(imported_90.outer_shell.faces.len(), 6);

    // 2. 180度回転ソリッド（2セグメント x 4エッジ + 2端面 = 10面閉シェル）
    let angle_180 = std::f64::consts::PI;
    let solid_180 = RevolveBuilder::revolve_wire_partial_solid(&wire, axis_origin, axis_dir, angle_180, &tol)
        .expect("180 deg revolve solid");

    assert_eq!(solid_180.outer_shell.faces.len(), 10);
    let report_180 = solid_180.outer_shell.validate_closed(&tol);
    assert!(report_180.is_valid(), "180 deg validation errors: {:?}", report_180.errors);
}

#[test]
fn test_linear_and_circular_pattern() {
    let tol = Tolerance::default();
    let base_box = PrimitiveBuilder::make_box(10.0, 10.0, 10.0).expect("make box");

    // 1. 直線パターン（X方向に間隔 15.0 で 4個）
    let dir = Vec3::new(1.0, 0.0, 0.0);
    let linear_solids = PatternBuilder::linear_pattern(&base_box, dir, 15.0, 4)
        .expect("linear pattern");

    assert_eq!(linear_solids.len(), 4);
    for (i, s) in linear_solids.iter().enumerate() {
        let r = s.outer_shell.validate_closed(&tol);
        assert!(r.is_valid(), "Linear instance {} invalid: {:?}", i, r.errors);
    }

    // 2. 円形パターン（Z軸まわりに 6個 360度等間隔）
    let base_cyl = PrimitiveBuilder::make_cylinder(3.0, 15.0).expect("make cylinder");
    let circ_solids = PatternBuilder::circular_pattern(
        &base_cyl,
        Point3::new(20.0, 0.0, 0.0),
        Vec3::new(0.0, 0.0, 1.0),
        std::f64::consts::TAU,
        6,
    )
    .expect("circular pattern");

    assert_eq!(circ_solids.len(), 6);
    for (i, s) in circ_solids.iter().enumerate() {
        let r = s.outer_shell.validate_closed(&tol);
        assert!(r.is_valid(), "Circular instance {} invalid: {:?}", i, r.errors);
    }

    // Compound Shape STEP ラウンドトリップ
    let compound_shape = PatternBuilder::circular_pattern_shape(
        &base_cyl,
        Point3::new(20.0, 0.0, 0.0),
        Vec3::new(0.0, 0.0, 1.0),
        std::f64::consts::TAU,
        6,
    )
    .expect("circular compound");

    let step_path = "test_pattern_compound.step";
    StepExporter::export_shape_to_file(&compound_shape, step_path, "PATTERN_COMPOUND")
        .expect("STEP compound export");
    let imported_shape = StepImporter::import_shape_from_file(step_path).expect("STEP compound import");
    let _ = std::fs::remove_file(step_path);

    match imported_shape {
        zenith_topo::Shape::Compound(solids) => assert_eq!(solids.len(), 6),
        _ => panic!("Expected compound shape"),
    }
}
