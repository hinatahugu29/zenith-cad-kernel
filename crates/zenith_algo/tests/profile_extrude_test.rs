use std::f64::consts::PI;
use zenith_algo::{ExtrudeBuilder, MassCalculator, ProfileBuilder, RevolveBuilder};
use zenith_io::{StepExporter, StepImporter};
use zenith_math::{Point3, Tolerance, Vec3};
use zenith_tess::{tessellate_solid, TessellationParams};
use zenith_topo::{Edge, OrientedEdge, Vertex, Wire};

#[test]
fn test_extruded_rounded_rectangle_with_hole() {
    let tol = Tolerance::default();
    let w = 60.0;
    let h = 40.0;
    let r_corner = 5.0;
    let r_hole = 8.0;
    let height = 25.0;

    let center = Point3::new(0.0, 0.0, 0.0);
    let normal = Vec3::new(0.0, 0.0, 1.0);
    let x_axis = Vec3::new(1.0, 0.0, 0.0);

    let outer_wire = ProfileBuilder::make_rounded_rectangle(w, h, r_corner, center, normal, x_axis)
        .expect("rounded rect wire");
    let hole_wire =
        ProfileBuilder::make_circle(r_hole, center, normal, x_axis).expect("circle hole wire");

    let solid = ExtrudeBuilder::extrude_face_with_holes(
        &outer_wire,
        &[hole_wire],
        Vec3::new(0.0, 0.0, height),
        &tol,
    )
    .expect("extrude solid");

    // 1. B-Rep 閉多様体検証
    assert!(
        solid.outer_shell.validate_closed(&tol).is_valid(),
        "Extruded solid must be valid closed manifold"
    );

    // 2. 閉形式体積一致検証
    let outer_area = (w * h) - 4.0 * r_corner * r_corner + PI * r_corner * r_corner;
    let hole_area = PI * r_hole * r_hole;
    let expected_vol = (outer_area - hole_area) * height;

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

    // 3. テッセレーション検証
    let mesh = tessellate_solid(&solid, &params);
    assert!(mesh.num_triangles() > 0, "Mesh should have triangles");

    // 4. STEP 往復検証
    let step_str = StepExporter::export_solid_to_string(&solid, "ExtrudedRoundedRectWithHole");
    let reimported = StepImporter::import_solid_from_str(&step_str).expect("import STEP");
    assert!(
        reimported.outer_shell.validate_closed(&tol).is_valid(),
        "Reimported solid must be valid closed"
    );
}

#[test]
fn test_revolved_flanged_cup() {
    let tol = Tolerance::default();
    // フランジ付きカップ断面プロファイル（r-z 平面: x=半径, z=高さ）
    // 6頂点による閉ループ: (10, 0) -> (25, 0) -> (25, 5) -> (15, 5) -> (15, 30) -> (10, 30) -> (10, 0)
    let pts = [
        Point3::new(10.0, 0.0, 0.0),
        Point3::new(25.0, 0.0, 0.0),
        Point3::new(25.0, 0.0, 5.0),
        Point3::new(15.0, 0.0, 5.0),
        Point3::new(15.0, 0.0, 30.0),
        Point3::new(10.0, 0.0, 30.0),
    ];
    let verts: Vec<Vertex> = pts.iter().map(|&p| Vertex::from_point(p)).collect();
    let mut edges = Vec::with_capacity(6);
    for i in 0..6 {
        let next = (i + 1) % 6;
        let line = Edge::line_between(verts[i].clone(), verts[next].clone()).expect("line");
        edges.push(OrientedEdge::forward(line));
    }
    let wire = Wire::new(edges);

    let axis_origin = Point3::new(0.0, 0.0, 0.0);
    let axis_dir = Vec3::new(0.0, 0.0, 1.0);

    let solid = RevolveBuilder::revolve_wire_solid(&wire, axis_origin, axis_dir, &tol)
        .expect("revolve flanged cup");

    // 1. B-Rep 閉多様体検証
    assert!(
        solid.outer_shell.validate_closed(&tol).is_valid(),
        "Revolved solid must be valid closed manifold"
    );

    // 2. 閉形式体積一致検証
    // 下部フランジ体積: PI * (25^2 - 10^2) * 5 = PI * (625 - 100) * 5 = 2625 * PI
    // 上部円筒体積: PI * (15^2 - 10^2) * 25 = PI * (225 - 100) * 25 = 3125 * PI
    // 合計体積 = 5750 * PI
    let expected_vol = 5750.0 * PI;

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

    // 3. STEP 往復検証
    let step_str = StepExporter::export_solid_to_string(&solid, "RevolvedFlangedCup");
    let reimported = StepImporter::import_solid_from_str(&step_str).expect("import STEP");
    assert!(
        reimported.outer_shell.validate_closed(&tol).is_valid(),
        "Reimported solid must be valid closed"
    );
}
