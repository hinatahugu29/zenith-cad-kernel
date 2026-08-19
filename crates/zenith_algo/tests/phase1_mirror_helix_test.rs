use zenith_algo::{HelixBuilder, MassCalculator, MirrorBuilder, PrimitiveBuilder};
use zenith_io::{StepExporter, StepImporter};
use zenith_math::{Point3, Tolerance, Vec3};
use zenith_tess::{tessellate_solid, TessellationParams};
use zenith_topo::{Edge, OrientedEdge, Vertex, Wire};

fn make_rect_wire(min_x: f64, max_x: f64, min_y: f64, max_y: f64) -> Wire {
    let pts = vec![
        Point3::new(min_x, min_y, 0.0),
        Point3::new(max_x, min_y, 0.0),
        Point3::new(max_x, max_y, 0.0),
        Point3::new(min_x, max_y, 0.0),
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
fn test_mirror_box_and_cylinder() {
    let tol = Tolerance::default();
    let base_box = PrimitiveBuilder::make_box(10.0, 20.0, 30.0).expect("make box");

    // 1. X=0 平面（法線 (1,0,0)）に対する鏡像反転
    let plane_origin = Point3::new(0.0, 0.0, 0.0);
    let plane_normal = Vec3::new(1.0, 0.0, 0.0);

    let mirrored_box =
        MirrorBuilder::mirror_solid(&base_box, plane_origin, plane_normal, &tol)
            .expect("mirror box");

    assert_eq!(mirrored_box.outer_shell.faces.len(), 6);
    let report = mirrored_box.outer_shell.validate_closed(&tol);
    assert!(report.is_valid(), "Mirrored box invalid: {:?}", report.errors);

    let mesh_orig = tessellate_solid(&base_box, &TessellationParams::default());
    let mesh_mir = tessellate_solid(&mirrored_box, &TessellationParams::default());
    let mass_orig = MassCalculator::compute_from_mesh(&mesh_orig);
    let mass_mir = MassCalculator::compute_from_mesh(&mesh_mir);
    assert!(
        (mass_orig.volume - mass_mir.volume).abs() < 1e-3,
        "Volume must match after mirror"
    );

    // 2. 円柱の斜め平面ミラー
    let base_cyl = PrimitiveBuilder::make_cylinder(5.0, 20.0).expect("make cyl");
    let diag_normal = Vec3::new(1.0, 1.0, 0.0).normalize();
    let mirrored_cyl =
        MirrorBuilder::mirror_solid(&base_cyl, Point3::new(10.0, 0.0, 0.0), diag_normal, &tol)
            .expect("mirror cyl");
    assert_eq!(mirrored_cyl.outer_shell.faces.len(), 6);
    let r_cyl = mirrored_cyl.outer_shell.validate_closed(&tol);
    assert!(r_cyl.is_valid(), "Mirrored cyl invalid: {:?}", r_cyl.errors);

    // 3. 複合Compound STEPラウンドトリップ
    let compound_shape = MirrorBuilder::mirror_compound(&base_box, plane_origin, plane_normal, &tol)
        .expect("mirror compound");
    let step_path = "test_mirror_compound.step";
    StepExporter::export_shape_to_file(&compound_shape, step_path, "MIRROR_COMPOUND")
        .expect("STEP export failed");
    let imported_shape = StepImporter::import_shape_from_file(step_path).expect("STEP import failed");
    let _ = std::fs::remove_file(step_path);

    match imported_shape {
        zenith_topo::Shape::Compound(solids) => assert_eq!(solids.len(), 2),
        _ => panic!("Expected compound shape"),
    }
}

#[test]
fn test_helix_spring_solid() {
    let tol = Tolerance::default();
    // 2.0 x 2.0 正方形断面
    let profile = make_rect_wire(-1.0, 1.0, -1.0, 1.0);
    let radius = 15.0;
    let pitch = 10.0;
    let turns = 2.0; // 2巻き (全高 20.0)
    let axis_origin = Point3::new(0.0, 0.0, 0.0);
    let axis_dir = Vec3::new(0.0, 0.0, 1.0);

    let helix_solid = HelixBuilder::sweep_wire_along_helix(
        &profile,
        radius,
        pitch,
        turns,
        axis_origin,
        axis_dir,
        32,
        &tol,
    )
    .expect("sweep helix solid");

    // 1. トポロジー閉シェル検証
    let report = helix_solid.outer_shell.validate_closed(&tol);
    assert!(report.is_valid(), "Helix solid invalid: {:?}", report.errors);

    // 2. 解析体積検証（断面積 4.0 * 螺旋弧長）
    let params = TessellationParams {
        u_divisions: 8,
        v_divisions: 8,
    };
    let mesh = tessellate_solid(&helix_solid, &params);
    let mass = MassCalculator::compute_from_mesh(&mesh);
    let helix_length = turns * ((2.0 * std::f64::consts::PI * radius).powi(2) + pitch.powi(2)).sqrt();
    let expected_vol = 4.0 * helix_length;
    let rel_err = (mass.volume - expected_vol).abs() / expected_vol;
    assert!(
        rel_err < 0.05,
        "Helix volume relative error too large: got {}, expected {}",
        mass.volume,
        expected_vol
    );

    // 3. STEPラウンドトリップ
    let step_path = "test_helix_solid_roundtrip.step";
    StepExporter::export_solid_to_file(&helix_solid, step_path, "HELIX_SOLID")
        .expect("STEP export failed");
    let imported = StepImporter::import_solid_from_file(step_path).expect("STEP import failed");
    let _ = std::fs::remove_file(step_path);
    assert_eq!(imported.outer_shell.faces.len(), helix_solid.outer_shell.faces.len());
}
