use zenith_algo::{LoftBuilder, MassCalculator, ShellBuilder};
use zenith_geom::{ControlPoint3, KnotVector, NurbsCurve3};
use zenith_io::{StepExporter, StepImporter};
use zenith_math::{Point3, Tolerance};
use zenith_tess::{tessellate_solid, TessellationParams};
use zenith_topo::{Edge, OrientedEdge, Vertex, Wire};

fn make_rect_wire_at_z(size_x: f64, size_y: f64, z: f64) -> Wire {
    let hx = size_x / 2.0;
    let hy = size_y / 2.0;
    let pts = vec![
        Point3::new(-hx, -hy, z),
        Point3::new(hx, -hy, z),
        Point3::new(hx, hy, z),
        Point3::new(-hx, hy, z),
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
fn test_guided_loft_solid() {
    let tol = Tolerance::default();
    let w_bot = make_rect_wire_at_z(20.0, 20.0, 0.0);
    let w_top = make_rect_wire_at_z(20.0, 20.0, 30.0);

    // 中央（Z=15）で外側に+5mm膨らむガイドレール曲線
    let guide = NurbsCurve3::new(
        2,
        vec![
            ControlPoint3::unweighted(Point3::new(10.0, -10.0, 0.0)),
            ControlPoint3::unweighted(Point3::new(15.0, -10.0, 15.0)),
            ControlPoint3::unweighted(Point3::new(10.0, -10.0, 30.0)),
        ],
        KnotVector::clamped_uniform(3, 2),
    )
    .expect("guide curve");

    let solid = LoftBuilder::loft_solid_guided(&[w_bot, w_top], &[guide], 2, &tol)
        .expect("guided loft solid");

    // 1. トポロジー閉シェル検証
    assert_eq!(solid.outer_shell.faces.len(), 6);
    let report = solid.outer_shell.validate_closed(&tol);
    assert!(
        report.is_valid(),
        "Guided loft invalid: {:?}",
        report.errors
    );

    // 2. STEPラウンドトリップ
    let step_path = "test_guided_loft_roundtrip.step";
    StepExporter::export_solid_to_file(&solid, step_path, "GUIDED_LOFT")
        .expect("STEP export failed");
    let imported = StepImporter::import_solid_from_file(step_path).expect("STEP import failed");
    let _ = std::fs::remove_file(step_path);
    assert_eq!(imported.outer_shell.faces.len(), 6);
}

#[test]
fn test_through_hollow_box_tube() {
    let tol = Tolerance::default();
    // 30 x 20 x 40, 肉厚 t=2.0
    let solid =
        ShellBuilder::make_through_hollow_box(30.0, 20.0, 40.0, 2.0).expect("make through tube");

    // 1. トポロジー検証（16面閉シェル）
    assert_eq!(solid.outer_shell.faces.len(), 16);
    let report = solid.outer_shell.validate_closed(&tol);
    assert!(
        report.is_valid(),
        "Through tube invalid: {:?}",
        report.errors
    );

    // 2. 解析体積検証: V = (30*20 - 26*16) * 40 = (600 - 416) * 40 = 7360.0 mm^3
    let mesh = tessellate_solid(&solid, &TessellationParams::default());
    let mass = MassCalculator::compute_from_mesh(&mesh);
    let expected_vol = 7360.0;
    let rel_err = (mass.volume - expected_vol).abs() / expected_vol;
    assert!(
        rel_err < 0.001,
        "Through tube volume error: got {}, expected {}",
        mass.volume,
        expected_vol
    );

    // 3. STEPラウンドトリップ
    let step_path = "test_through_tube_roundtrip.step";
    StepExporter::export_solid_to_file(&solid, step_path, "THROUGH_TUBE")
        .expect("STEP export failed");
    let imported = StepImporter::import_solid_from_file(step_path).expect("STEP import failed");
    let _ = std::fs::remove_file(step_path);
    assert_eq!(imported.outer_shell.faces.len(), 16);
}
