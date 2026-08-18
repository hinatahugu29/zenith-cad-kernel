use zenith_algo::{ExtrudeBuilder, LoftBuilder, RevolveBuilder};
use zenith_geom::{NurbsCurve3, Surface3};
use zenith_io::ObjExporter;
use zenith_math::{Point3, Tolerance, Vec3};
use zenith_tess::{tessellate_face, tessellate_solid, tessellate_surface, TessellationParams};
use zenith_topo::{Edge, Face, FaceGeometry, Orientation, OrientedEdge, Shell, Vertex, Wire};

#[test]
fn test_extrude_polygon_wire() {
    let tol = Tolerance::default();

    // 正方形ワイヤ (0,0,0) -> (10,0,0) -> (10,10,0) -> (0,10,0)
    let p0 = Point3::new(0.0, 0.0, 0.0);
    let p1 = Point3::new(10.0, 0.0, 0.0);
    let p2 = Point3::new(10.0, 10.0, 0.0);
    let p3 = Point3::new(0.0, 10.0, 0.0);

    let v0 = Vertex::new(p0, tol.linear);
    let v1 = Vertex::new(p1, tol.linear);
    let v2 = Vertex::new(p2, tol.linear);
    let v3 = Vertex::new(p3, tol.linear);

    let e0 = Edge::line_between(v0.clone(), v1.clone()).unwrap();
    let e1 = Edge::line_between(v1.clone(), v2.clone()).unwrap();
    let e2 = Edge::line_between(v2.clone(), v3.clone()).unwrap();
    let e3 = Edge::line_between(v3.clone(), v0.clone()).unwrap();

    let wire = Wire::new(vec![
        OrientedEdge::forward(e0),
        OrientedEdge::forward(e1),
        OrientedEdge::forward(e2),
        OrientedEdge::forward(e3),
    ]);

    // +Z方向に高さ25mm押し出し
    let solid = ExtrudeBuilder::extrude_wire(&wire, Vec3::new(0.0, 0.0, 25.0), &tol)
        .expect("Extrude failed");

    // 4側面 + 底面 + 天面 = 6面
    assert_eq!(solid.outer_shell.faces.len(), 6);

    let params = TessellationParams {
        u_divisions: 4,
        v_divisions: 4,
    };
    let mesh = tessellate_solid(&solid, &params);
    assert!(mesh.num_triangles() > 0);

    ObjExporter::export_to_file(&mesh, "target/samples/extruded_solid.obj", "extruded_solid")
        .unwrap();
}

#[test]
fn test_revolve_curve_to_nurbs_surface() {
    let tol = Tolerance::default();

    // 母線プロファイル（花瓶・ボトルの輪郭線）
    let profile = NurbsCurve3::bspline_from_points(
        2,
        vec![
            Point3::new(5.0, 0.0, 0.0),
            Point3::new(8.0, 0.0, 10.0),
            Point3::new(3.0, 0.0, 20.0),
            Point3::new(6.0, 0.0, 30.0),
        ],
    )
    .unwrap();

    // Z軸まわりに 360度 (2*PI) 回転
    let rev_surface = RevolveBuilder::revolve_curve(
        &profile,
        Point3::new(0.0, 0.0, 0.0),
        Vec3::new(0.0, 0.0, 1.0),
        std::f64::consts::PI * 2.0,
        &tol,
    )
    .expect("Revolve failed");

    let params = TessellationParams {
        u_divisions: 32,
        v_divisions: 32,
    };
    let mesh = tessellate_surface(&rev_surface, &params, Orientation::Forward);
    assert!(mesh.num_triangles() > 0);

    ObjExporter::export_to_file(&mesh, "target/samples/revolved_vase.obj", "revolved_vase")
        .unwrap();
}

#[test]
fn test_loft_through_profiles() {
    let tol = Tolerance::default();

    // 3つの断面プロファイル（下層、中層、上層）
    let p0 = NurbsCurve3::bspline_from_points(
        2,
        vec![
            Point3::new(-5.0, -5.0, 0.0),
            Point3::new(0.0, -8.0, 0.0),
            Point3::new(5.0, -5.0, 0.0),
        ],
    )
    .unwrap();

    let p1 = NurbsCurve3::bspline_from_points(
        2,
        vec![
            Point3::new(-8.0, 0.0, 15.0),
            Point3::new(0.0, 0.0, 15.0),
            Point3::new(8.0, 0.0, 15.0),
        ],
    )
    .unwrap();

    let p2 = NurbsCurve3::bspline_from_points(
        2,
        vec![
            Point3::new(-3.0, 5.0, 30.0),
            Point3::new(0.0, 8.0, 30.0),
            Point3::new(3.0, 5.0, 30.0),
        ],
    )
    .unwrap();

    let loft_surface = LoftBuilder::loft_curves(&[p0, p1, p2], 2, &tol).expect("Loft failed");

    let params = TessellationParams {
        u_divisions: 24,
        v_divisions: 24,
    };
    let mesh = tessellate_surface(&loft_surface, &params, Orientation::Forward);
    assert!(mesh.num_triangles() > 0);

    ObjExporter::export_to_file(&mesh, "target/samples/lofted_wing.obj", "lofted_wing").unwrap();
}

#[test]
fn test_boolean_operations() {
    let _tol = Tolerance::default();
    let tess_params = TessellationParams {
        u_divisions: 4,
        v_divisions: 4,
    };

    // 直方体 A: [0, 10] x [0, 10] x [0, 10]
    let solid_a = zenith_algo::PrimitiveBuilder::make_box(10.0, 10.0, 10.0).unwrap();

    // 直方体 B: [3, 13] x [3, 13] x [3, 13]
    let solid_b_base = zenith_algo::PrimitiveBuilder::make_box(10.0, 10.0, 10.0).unwrap();
    // Bを (3, 3, 3) に配置
    let mut mesh_b = zenith_tess::tessellate_solid(&solid_b_base, &tess_params);
    for p in &mut mesh_b.positions {
        *p += Vec3::new(3.0, 3.0, 3.0);
    }

    let mesh_a = zenith_tess::tessellate_solid(&solid_a, &tess_params);

    // 1. Difference (A - B)
    let diff_mesh = zenith_algo::BooleanEngine::boolean_meshes(
        &mesh_a,
        &mesh_b,
        zenith_algo::BooleanOpType::Difference,
    )
    .expect("Boolean Difference failed");
    assert!(diff_mesh.num_triangles() > 0);
    ObjExporter::export_to_file(
        &diff_mesh,
        "target/samples/boolean_difference.obj",
        "bool_diff",
    )
    .unwrap();

    // 2. Union (A + B)
    let union_mesh = zenith_algo::BooleanEngine::boolean_meshes(
        &mesh_a,
        &mesh_b,
        zenith_algo::BooleanOpType::Union,
    )
    .expect("Boolean Union failed");
    assert!(union_mesh.num_triangles() > 0);
    ObjExporter::export_to_file(
        &union_mesh,
        "target/samples/boolean_union.obj",
        "bool_union",
    )
    .unwrap();

    // 3. Intersection (A * B)
    let isect_mesh = zenith_algo::BooleanEngine::boolean_meshes(
        &mesh_a,
        &mesh_b,
        zenith_algo::BooleanOpType::Intersection,
    )
    .expect("Boolean Intersection failed");
    assert!(isect_mesh.num_triangles() > 0);
    ObjExporter::export_to_file(
        &isect_mesh,
        "target/samples/boolean_intersection.obj",
        "bool_isect",
    )
    .unwrap();
}

#[test]
fn test_step_export_solid() {
    let solid = zenith_algo::PrimitiveBuilder::make_box(15.0, 25.0, 35.0).unwrap();
    zenith_io::StepExporter::export_solid_to_file(
        &solid,
        "target/samples/box_solid.stp",
        "ZENITH_BOX_SOLID",
    )
    .expect("STEP export failed");

    let step_content = std::fs::read_to_string("target/samples/box_solid.stp").unwrap();
    assert!(step_content.contains("ISO-10303-21;"));
    assert!(step_content.contains("MANIFOLD_SOLID_BREP"));
    assert!(step_content.contains("CLOSED_SHELL"));
    assert!(step_content.contains("END-ISO-10303-21;"));
}

#[test]
fn test_fillet_box_solid() {
    let tol = Tolerance::default();
    let filleted_solid =
        zenith_algo::FilletBuilder::fillet_box_z_edges(20.0, 30.0, 40.0, 4.0, &tol)
            .expect("Fillet box failed");

    // 10面（4平面側面 + 4円弧フィレット面 + 上下2つの8角形底面）
    assert_eq!(filleted_solid.outer_shell.faces.len(), 10);

    let params = TessellationParams {
        u_divisions: 16,
        v_divisions: 16,
    };
    let mesh = zenith_tess::tessellate_solid(&filleted_solid, &params);
    assert!(mesh.num_triangles() > 0);

    ObjExporter::export_to_file(&mesh, "target/samples/filleted_box.obj", "filleted_box").unwrap();

    // STEPエクスポート（NURBS円弧フィレット面を含むB-Rep）
    zenith_io::StepExporter::export_solid_to_file(
        &filleted_solid,
        "target/samples/filleted_box.stp",
        "ZENITH_FILLETED_BOX",
    )
    .expect("STEP export failed for filleted solid");

    let step_content = std::fs::read_to_string("target/samples/filleted_box.stp").unwrap();
    assert!(step_content.contains("B_SPLINE_SURFACE_WITH_KNOTS"));
}

#[test]
fn test_drilled_hole_box_solid() {
    let drilled_solid = zenith_algo::HoleBuilder::make_drilled_box(30.0, 30.0, 15.0, 5.0)
        .expect("Drilled hole box failed");

    // 10面（外側4平面 + 内側穴4円筒面 + 上下2つの穴あき平面）
    assert_eq!(drilled_solid.outer_shell.faces.len(), 10);

    let params = TessellationParams {
        u_divisions: 16,
        v_divisions: 16,
    };
    let mesh = zenith_tess::tessellate_solid(&drilled_solid, &params);
    assert!(mesh.num_triangles() > 0);

    ObjExporter::export_to_file(&mesh, "target/samples/drilled_box.obj", "drilled_box").unwrap();

    // STEPエクスポート（FACE_BOUND 内側穴ループ付き）
    zenith_io::StepExporter::export_solid_to_file(
        &drilled_solid,
        "target/samples/drilled_box.stp",
        "ZENITH_DRILLED_BOX",
    )
    .expect("STEP export failed for drilled box");

    let step_content = std::fs::read_to_string("target/samples/drilled_box.stp").unwrap();
    assert!(step_content.contains("FACE_BOUND"));
    assert!(step_content.contains("MANIFOLD_SOLID_BREP"));
}

#[test]
fn test_sweep_pipe_solid() {
    // 3D S字カーブ軌道
    let path = NurbsCurve3::bspline_from_points(
        3,
        vec![
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(10.0, 0.0, 10.0),
            Point3::new(20.0, 20.0, 25.0),
            Point3::new(30.0, 20.0, 40.0),
        ],
    )
    .unwrap();

    let pipe_solid = zenith_algo::SweepBuilder::sweep_circle_along_curve(&path, 3.5, 16)
        .expect("Sweep pipe failed");

    // 6面（4つの四分円筒スイープ側面 + 始点終点2つの端面）
    assert_eq!(pipe_solid.outer_shell.faces.len(), 6);

    let params = TessellationParams {
        u_divisions: 16,
        v_divisions: 16,
    };
    let mesh = zenith_tess::tessellate_solid(&pipe_solid, &params);
    assert!(mesh.num_triangles() > 0);

    ObjExporter::export_to_file(&mesh, "target/samples/sweep_pipe.obj", "sweep_pipe").unwrap();

    // STEPエクスポート
    zenith_io::StepExporter::export_solid_to_file(
        &pipe_solid,
        "target/samples/sweep_pipe.stp",
        "ZENITH_SWEEP_PIPE",
    )
    .expect("STEP export failed for sweep pipe");

    let step_content = std::fs::read_to_string("target/samples/sweep_pipe.stp").unwrap();
    assert!(step_content.contains("MANIFOLD_SOLID_BREP"));
    assert!(step_content.contains("B_SPLINE_SURFACE_WITH_KNOTS"));
}

#[test]
fn test_chamfer_box_solid_and_step() {
    let tol = Tolerance::default();
    let chamfered = zenith_algo::ChamferBuilder::chamfer_box_z_edges(20.0, 30.0, 40.0, 3.0, &tol)
        .expect("Chamfer box failed");

    // 10面（4平面側面 + 4面取り平面 + 上下2つの8角形底面・天面）
    assert_eq!(chamfered.outer_shell.faces.len(), 10);

    let params = TessellationParams {
        u_divisions: 8,
        v_divisions: 8,
    };
    let mesh = zenith_tess::tessellate_solid(&chamfered, &params);
    assert!(mesh.num_triangles() > 0);

    // 物性値（体積・表面積）の検証
    let props = zenith_algo::MassCalculator::compute_from_mesh(&mesh);
    // 直方体体積 20*30*40 = 24000 から 4隅の三角柱 (0.5 * 3 * 3 * 40 * 4 = 720) を引いた値 = 23280
    assert!((props.volume - 23280.0).abs() < 50.0);
    assert!(props.surface_area > 0.0);

    // STEPエクスポート
    zenith_io::StepExporter::export_solid_to_file(
        &chamfered,
        "target/samples/chamfered_box.stp",
        "ZENITH_CHAMFERED_BOX",
    )
    .expect("STEP export failed for chamfered box");

    // STLエクスポート
    zenith_io::StlExporter::export_binary(&mesh, "target/samples/chamfered_box.stl")
        .expect("STL export failed");
}

#[test]
fn test_cylinder_solid_and_step() {
    let cyl =
        zenith_algo::PrimitiveBuilder::make_cylinder(10.0, 30.0).expect("Cylinder creation failed");
    // 6面（4つの四分円筒側面 + 上下2つの底面・天面）
    assert_eq!(cyl.outer_shell.faces.len(), 6);

    let params = TessellationParams {
        u_divisions: 32,
        v_divisions: 16,
    };
    let mesh = zenith_tess::tessellate_solid(&cyl, &params);
    assert!(mesh.num_triangles() > 0);

    // 体積検証: 離散化メッシュ体積が理論値 (約9424.8) に近いこと (90%以上)
    let props = zenith_algo::MassCalculator::compute_from_mesh(&mesh);
    let expected_vol = std::f64::consts::PI * 100.0 * 30.0;
    assert!(props.volume > expected_vol * 0.85 && props.volume < expected_vol * 1.05);

    zenith_io::StepExporter::export_solid_to_file(
        &cyl,
        "target/samples/cylinder.stp",
        "ZENITH_CYLINDER",
    )
    .expect("STEP export failed for cylinder");
    zenith_io::StlExporter::export_binary(&mesh, "target/samples/cylinder.stl")
        .expect("STL export failed for cylinder");
}

#[test]
fn test_cylinder_caps_sample_curved_boundary() {
    let cyl =
        zenith_algo::PrimitiveBuilder::make_cylinder(10.0, 30.0).expect("Cylinder creation failed");
    let params = TessellationParams {
        u_divisions: 32,
        v_divisions: 16,
    };

    let bottom_cap = tessellate_face(&cyl.outer_shell.faces[4], &params);
    let top_cap = tessellate_face(&cyl.outer_shell.faces[5], &params);

    assert!(
        bottom_cap.positions.len() > 4,
        "bottom cap should sample circular edge curves, not only topology vertices"
    );
    assert!(
        top_cap.positions.len() > 4,
        "top cap should sample circular edge curves, not only topology vertices"
    );
    assert!(bottom_cap.num_triangles() > 2);
    assert!(top_cap.num_triangles() > 2);
}

#[test]
fn test_cylinder_cap_curve_sampling_refines_with_tessellation_params() {
    let cyl =
        zenith_algo::PrimitiveBuilder::make_cylinder(10.0, 30.0).expect("Cylinder creation failed");
    let low = TessellationParams {
        u_divisions: 8,
        v_divisions: 8,
    };
    let high = TessellationParams {
        u_divisions: 64,
        v_divisions: 16,
    };

    let low_cap = tessellate_face(&cyl.outer_shell.faces[5], &low);
    let high_cap = tessellate_face(&cyl.outer_shell.faces[5], &high);

    assert!(
        low_cap.positions.len() > 4,
        "even coarse tessellation should preserve curved cap boundaries"
    );
    assert!(
        high_cap.positions.len() > low_cap.positions.len(),
        "higher tessellation settings should refine curved p-curve boundaries"
    );
    assert!(high_cap.num_triangles() > low_cap.num_triangles());
}

#[test]
fn test_closed_shell_validation_for_primitives() {
    let tol = Tolerance::default();
    let box_solid = zenith_algo::PrimitiveBuilder::make_box(10.0, 20.0, 30.0).unwrap();
    let cylinder = zenith_algo::PrimitiveBuilder::make_cylinder(10.0, 30.0).unwrap();

    assert!(box_solid.is_topologically_valid(&tol));
    assert!(cylinder.is_topologically_valid(&tol));
    assert_eq!(
        box_solid
            .outer_shell
            .validate_closed(&tol)
            .pcurve_mismatch_count,
        0
    );
    assert_eq!(
        cylinder
            .outer_shell
            .validate_closed(&tol)
            .pcurve_mismatch_count,
        0
    );
    assert_eq!(
        box_solid
            .outer_shell
            .validate_closed(&tol)
            .same_direction_edge_use_count,
        0
    );
    assert_eq!(
        cylinder
            .outer_shell
            .validate_closed(&tol)
            .same_direction_edge_use_count,
        0
    );
}

#[test]
fn test_closed_shell_validation_rejects_missing_face() {
    let tol = Tolerance::default();
    let box_solid = zenith_algo::PrimitiveBuilder::make_box(10.0, 20.0, 30.0).unwrap();
    let mut faces = box_solid.outer_shell.faces.clone();
    faces.pop();

    let broken_shell = Shell::closed(faces);
    let report = broken_shell.validate_closed(&tol);

    assert!(!report.is_valid());
    assert!(report.unmatched_edge_use_count > 0);
}

#[test]
fn test_face_boundary_validation_rejects_wire_off_surface() {
    let tol = Tolerance::default();
    let box_solid = zenith_algo::PrimitiveBuilder::make_box(10.0, 20.0, 30.0).unwrap();
    let bottom_face = &box_solid.outer_shell.faces[0];
    let top_face = &box_solid.outer_shell.faces[1];

    let invalid_face = Face::simple(bottom_face.geometry.clone(), top_face.outer_wire.clone());
    let report = invalid_face.validate_boundary_on_surface(&tol, 4);

    assert!(!report.is_valid());
    assert!(report.off_surface_point_count > 0);
    assert!(report.max_distance > 1.0);
}

#[test]
fn test_closed_shell_validation_rejects_corrupted_pcurve() {
    let tol = Tolerance::default();
    let box_solid = zenith_algo::PrimitiveBuilder::make_box(10.0, 20.0, 30.0).unwrap();
    let mut faces = box_solid.outer_shell.faces.clone();

    let pcurves = faces[0].pcurves.as_mut().expect("plane p-curves");
    pcurves.outer_loop.segments[0].curve.control_points[0]
        .point
        .x += 1.0;

    let corrupted_shell = Shell::closed(faces);
    let report = corrupted_shell.validate_closed(&tol);

    assert!(!report.is_valid());
    assert!(report.pcurve_mismatch_count > 0);
    assert!(report.max_pcurve_distance > 0.5);
}

#[test]
fn test_closed_shell_validation_rejects_same_direction_shared_edge() {
    let tol = Tolerance::default();
    let box_solid = zenith_algo::PrimitiveBuilder::make_box(10.0, 20.0, 30.0).unwrap();
    let mut faces = box_solid.outer_shell.faces.clone();
    for edge in &mut faces[0].outer_wire.edges {
        edge.orientation = edge.orientation.reversed();
    }
    faces[0].outer_wire.edges.reverse();
    faces[0].pcurves = None;

    let corrupted_shell = Shell::closed(faces);
    let report = corrupted_shell.validate_closed(&tol);

    assert!(!report.is_valid());
    assert!(report.same_direction_edge_use_count > 0);
}

#[test]
fn test_closed_shell_validation_rejects_edge_curve_endpoint_mismatch() {
    let tol = Tolerance::default();
    let box_solid = zenith_algo::PrimitiveBuilder::make_box(10.0, 20.0, 30.0).unwrap();
    let mut faces = box_solid.outer_shell.faces.clone();
    let corrupt_edge = &mut faces[0].outer_wire.edges[0].edge;
    corrupt_edge.curve = NurbsCurve3::bspline_from_points(
        1,
        vec![
            corrupt_edge.start_vertex.point + Vec3::new(1.0, 0.0, 0.0),
            corrupt_edge.end_vertex.point + Vec3::new(1.0, 0.0, 0.0),
        ],
    )
    .unwrap();
    faces[0].pcurves = None;

    let corrupted_shell = Shell::closed(faces);
    let report = corrupted_shell.validate_closed(&tol);

    assert!(!report.is_valid());
    assert!(report.edge_curve_endpoint_mismatch_count > 0);
    assert!(report.max_edge_curve_endpoint_distance > 0.5);
}

#[test]
fn test_closed_shell_validation_rejects_degenerate_edge_use() {
    let tol = Tolerance::default();
    let box_solid = zenith_algo::PrimitiveBuilder::make_box(10.0, 20.0, 30.0).unwrap();
    let mut faces = box_solid.outer_shell.faces.clone();
    let corrupt_edge = &mut faces[0].outer_wire.edges[0].edge;
    let point = corrupt_edge.start_vertex.point;
    corrupt_edge.end_vertex = Vertex::from_point(point);
    corrupt_edge.curve = NurbsCurve3::bspline_from_points(1, vec![point, point]).unwrap();
    faces[0].pcurves = None;

    let corrupted_shell = Shell::closed(faces);
    let report = corrupted_shell.validate_closed(&tol);

    assert!(!report.is_valid());
    assert!(report.degenerate_edge_use_count > 0);
    assert!(report.min_edge_use_length <= tol.linear);
}

#[test]
fn test_closed_shell_validation_rejects_inward_planar_face_orientation() {
    let tol = Tolerance::default();
    let box_solid = zenith_algo::PrimitiveBuilder::make_box(10.0, 20.0, 30.0).unwrap();
    let mut faces = box_solid.outer_shell.faces.clone();
    faces[0].orientation = Orientation::Reversed;

    let corrupted_shell = Shell::closed(faces);
    let report = corrupted_shell.validate_closed(&tol);

    assert!(!report.is_valid());
    assert!(report.planar_face_orientation_mismatch_count > 0);
    assert!(report.min_planar_face_oriented_area < 0.0);
}

#[test]
fn test_closed_shell_validation_rejects_non_finite_edge_curve_points() {
    let tol = Tolerance::default();
    let box_solid = zenith_algo::PrimitiveBuilder::make_box(10.0, 20.0, 30.0).unwrap();
    let mut faces = box_solid.outer_shell.faces.clone();
    faces[0].outer_wire.edges[0].edge.curve.control_points[0]
        .point
        .x = f64::NAN;
    faces[0].pcurves = None;

    let corrupted_shell = Shell::closed(faces);
    let report = corrupted_shell.validate_closed(&tol);

    assert!(!report.is_valid());
    assert!(report.non_finite_point_count > 0);
}

#[test]
fn test_closed_shell_validation_reports_duplicate_face_and_edge_uses() {
    let tol = Tolerance::default();
    let box_solid = zenith_algo::PrimitiveBuilder::make_box(10.0, 20.0, 30.0).unwrap();
    let mut faces = box_solid.outer_shell.faces.clone();
    faces.push(faces[0].clone());

    let corrupted_shell = Shell::closed(faces);
    let report = corrupted_shell.validate_closed(&tol);

    assert!(!report.is_valid());
    assert!(report.duplicate_face_count > 0);
    assert!(report.duplicate_edge_use_count > 0);
}

#[test]
fn test_solid_try_simple_accepts_valid_primitives() {
    let tol = Tolerance::default();
    let box_solid = zenith_algo::PrimitiveBuilder::make_box(10.0, 20.0, 30.0).unwrap();
    let cylinder = zenith_algo::PrimitiveBuilder::make_cylinder(10.0, 30.0).unwrap();

    assert!(zenith_topo::Solid::try_simple(box_solid.outer_shell.clone(), &tol).is_ok());
    assert!(zenith_topo::Solid::try_simple(cylinder.outer_shell.clone(), &tol).is_ok());
}

#[test]
fn test_solid_try_simple_rejects_invalid_shell() {
    let tol = Tolerance::default();
    let box_solid = zenith_algo::PrimitiveBuilder::make_box(10.0, 20.0, 30.0).unwrap();
    let mut faces = box_solid.outer_shell.faces.clone();
    faces.pop();

    let broken_shell = Shell::closed(faces);
    let err = zenith_topo::Solid::try_simple(broken_shell, &tol).expect_err("must reject shell");

    assert!(err.outer_shell_report.unmatched_edge_use_count > 0);
    assert!(err.inner_shell_reports.is_empty());
}

#[test]
fn test_planar_cylinder_cap_pcurves_match_3d_edges() {
    let cyl = zenith_algo::PrimitiveBuilder::make_cylinder(10.0, 30.0).unwrap();
    let cap_face = &cyl.outer_shell.faces[5];
    let FaceGeometry::Plane(plane) = &cap_face.geometry else {
        panic!("Cylinder cap should be planar");
    };

    let pcurves = cap_face.derive_plane_pcurves().expect("derive p-curves");
    assert_eq!(
        pcurves.outer_loop.segments.len(),
        cap_face.outer_wire.edges.len()
    );
    assert!(pcurves.inner_loops.is_empty());

    for (oriented_edge, pcurve_segment) in cap_face
        .outer_wire
        .edges
        .iter()
        .zip(pcurves.outer_loop.segments.iter())
    {
        assert_eq!(pcurve_segment.edge_id, oriented_edge.edge.id);

        let (t_min, t_max) = pcurve_segment.curve.param_range();
        for i in 0..=4 {
            let t = i as f64 / 4.0;
            let curve_t = t_min + t * (t_max - t_min);
            let uv = pcurve_segment.curve.evaluate(curve_t);
            let from_pcurve = plane.evaluate(uv.x, uv.y);
            let from_edge = oriented_edge.evaluate_normalized(t);
            assert!((from_pcurve - from_edge).norm() < 1e-6);
        }
    }
}

#[test]
fn test_planar_pcurve_validation_accepts_cylinder_cap() {
    let tol = Tolerance::default();
    let cyl = zenith_algo::PrimitiveBuilder::make_cylinder(10.0, 30.0).unwrap();
    let cap_face = &cyl.outer_shell.faces[5];

    let report = cap_face
        .validate_plane_pcurves(&tol, 8)
        .expect("validate plane p-curves");

    assert!(report.is_valid());
    assert_eq!(report.mismatch_count, 0);
    assert!(report.max_distance < tol.linear);
}

#[test]
fn test_planar_face_can_store_pcurves() {
    let tol = Tolerance::default();
    let cyl = zenith_algo::PrimitiveBuilder::make_cylinder(10.0, 30.0).unwrap();
    let cap_face = cyl.outer_shell.faces[5]
        .clone()
        .with_plane_pcurves()
        .expect("attach p-curves");

    assert!(cap_face.pcurves.is_some());

    let report = cap_face
        .validate_plane_pcurves(&tol, 8)
        .expect("validate stored p-curves");
    assert!(report.is_valid());
}

#[test]
fn test_planar_faces_store_pcurves_by_default() {
    let box_solid = zenith_algo::PrimitiveBuilder::make_box(10.0, 20.0, 30.0).unwrap();
    let cylinder = zenith_algo::PrimitiveBuilder::make_cylinder(10.0, 30.0).unwrap();
    let drilled = zenith_algo::HoleBuilder::make_drilled_box(30.0, 30.0, 15.0, 5.0).unwrap();

    for face in &box_solid.outer_shell.faces {
        if matches!(face.geometry, FaceGeometry::Plane(_)) {
            assert!(face.pcurves.is_some());
        }
    }

    for face in &cylinder.outer_shell.faces {
        if matches!(face.geometry, FaceGeometry::Plane(_)) {
            assert!(face.pcurves.is_some());
        }
    }

    for face in &drilled.outer_shell.faces {
        if matches!(face.geometry, FaceGeometry::Plane(_)) {
            assert!(face.pcurves.is_some());
        }
    }
}

#[test]
fn test_planar_pcurve_validation_rejects_off_plane_boundary() {
    let tol = Tolerance::default();
    let box_solid = zenith_algo::PrimitiveBuilder::make_box(10.0, 20.0, 30.0).unwrap();
    let bottom_face = &box_solid.outer_shell.faces[0];
    let top_face = &box_solid.outer_shell.faces[1];
    let invalid_face = Face::simple(bottom_face.geometry.clone(), top_face.outer_wire.clone());

    let report = invalid_face
        .validate_plane_pcurves(&tol, 4)
        .expect("validate plane p-curves");

    assert!(!report.is_valid());
    assert!(report.mismatch_count > 0);
    assert!(report.max_distance > 1.0);
}

#[test]
fn test_planar_pcurves_include_inner_hole_loops() {
    let tol = Tolerance::default();
    let drilled =
        zenith_algo::HoleBuilder::make_drilled_box(30.0, 30.0, 15.0, 5.0).expect("drilled box");
    let holed_face = drilled
        .outer_shell
        .faces
        .iter()
        .find(|face| !face.inner_wires.is_empty())
        .expect("holed planar face");

    let pcurves = holed_face.derive_plane_pcurves().expect("derive p-curves");
    assert_eq!(pcurves.inner_loops.len(), holed_face.inner_wires.len());
    assert!(pcurves.inner_loops[0].segments.len() >= 4);

    let report = holed_face
        .validate_plane_pcurves(&tol, 8)
        .expect("validate p-curves");
    assert!(report.is_valid());
}

#[test]
fn test_nurbs_cylinder_side_boundary_pcurves_match_3d_edges() {
    let tol = Tolerance::default();
    let cyl = zenith_algo::PrimitiveBuilder::make_cylinder(10.0, 30.0).unwrap();
    let side_face = &cyl.outer_shell.faces[0];
    let FaceGeometry::Nurbs(surface) = &side_face.geometry else {
        panic!("Cylinder side should be a NURBS face");
    };

    let pcurves = side_face
        .derive_nurbs_boundary_pcurves(&tol, 8)
        .expect("derive NURBS boundary p-curves");

    assert_eq!(
        pcurves.outer_loop.segments.len(),
        side_face.outer_wire.edges.len()
    );

    for (edge, segment) in side_face
        .outer_wire
        .edges
        .iter()
        .zip(pcurves.outer_loop.segments.iter())
    {
        let (t_min, t_max) = segment.curve.param_range();
        for i in 0..=8 {
            let t = i as f64 / 8.0;
            let uv_t = t_min + t * (t_max - t_min);
            let uv = segment.curve.evaluate(uv_t);
            let from_pcurve = surface.evaluate(uv.x, uv.y);
            let from_edge = edge.evaluate_normalized(t);
            assert!((from_pcurve - from_edge).norm() < 1e-5);
        }
    }
}

#[test]
fn test_nurbs_cylinder_side_can_store_boundary_pcurves() {
    let tol = Tolerance::default();
    let cyl = zenith_algo::PrimitiveBuilder::make_cylinder(10.0, 30.0).unwrap();
    let side_face = cyl.outer_shell.faces[0]
        .clone()
        .with_nurbs_boundary_pcurves(&tol, 8)
        .expect("attach NURBS boundary p-curves");

    assert!(side_face.pcurves.is_some());
    let pcurves = side_face.pcurves(&tol).expect("stored p-curves");
    assert_eq!(
        pcurves.outer_loop.segments.len(),
        side_face.outer_wire.edges.len()
    );
}

#[test]
fn test_nurbs_cylinder_side_faces_store_boundary_pcurves_by_default() {
    let cyl = zenith_algo::PrimitiveBuilder::make_cylinder(10.0, 30.0).unwrap();

    for face in cyl
        .outer_shell
        .faces
        .iter()
        .filter(|face| matches!(face.geometry, FaceGeometry::Nurbs(_)))
    {
        assert!(face.pcurves.is_some());
        let pcurves = face.pcurves.as_ref().unwrap();
        assert_eq!(
            pcurves.outer_loop.segments.len(),
            face.outer_wire.edges.len()
        );
    }
}

#[test]
fn test_nurbs_projected_pcurve_for_non_boundary_iso_edge() {
    let tol = Tolerance::default();
    let cyl = zenith_algo::PrimitiveBuilder::make_cylinder(10.0, 30.0).unwrap();
    let side_face = &cyl.outer_shell.faces[0];
    let FaceGeometry::Nurbs(surface) = &side_face.geometry else {
        panic!("Cylinder side should be a NURBS face");
    };

    let ((u_min, u_max), (v_min, v_max)) = surface.param_range();
    let u_mid = (u_min + u_max) * 0.5;
    let p0 = surface.evaluate(u_mid, v_min);
    let p1 = surface.evaluate(u_mid, v_max);
    let edge = Edge::line_between(Vertex::from_point(p0), Vertex::from_point(p1)).unwrap();
    let wire = Wire::new(vec![OrientedEdge::forward(edge)]);
    let projected_face = Face::simple(FaceGeometry::Nurbs(surface.clone()), wire);

    let pcurves = projected_face
        .pcurves(&tol)
        .expect("projected NURBS p-curve");
    assert_eq!(pcurves.outer_loop.segments.len(), 1);

    let report = projected_face
        .validate_pcurves(&tol, 8)
        .expect("validate projected p-curve");
    assert!(report.is_valid());
    assert!(report.max_distance < 1e-5);
}

#[test]
fn test_sphere_solid() {
    let sphere = zenith_algo::PrimitiveBuilder::make_sphere(15.0).expect("Sphere creation failed");
    assert_eq!(sphere.outer_shell.faces.len(), 1);

    let params = TessellationParams {
        u_divisions: 32,
        v_divisions: 32,
    };
    let mesh = zenith_tess::tessellate_solid(&sphere, &params);
    assert!(mesh.num_triangles() > 0);

    // 体積検証: 離散化メッシュ体積が理論値 (約14137.2) に近いこと (95%以上)
    let props = zenith_algo::MassCalculator::compute_from_mesh(&mesh);
    let expected_vol = 4.0 / 3.0 * std::f64::consts::PI * 15.0_f64.powi(3);
    assert!(props.volume > expected_vol * 0.90 && props.volume < expected_vol * 1.05);

    zenith_io::StlExporter::export_binary(&mesh, "target/samples/sphere.stl")
        .expect("STL export failed for sphere");
}

#[test]
fn test_direct_modeling_inspection_and_push_pull() {
    // 10 x 20 x 30 の直方体
    let solid = zenith_algo::PrimitiveBuilder::make_box(10.0, 20.0, 30.0).unwrap();

    // 天面 (face_index = 1: Top face) の幾何クエリ
    let top_face = &solid.outer_shell.faces[1];
    let insp = zenith_algo::DirectModeling::inspect_face(top_face).expect("Inspect failed");

    // 面積 = 10 * 20 = 200
    assert!((insp.area - 200.0).abs() < 1e-4);
    // 法線は +Z (0, 0, 1) -> XY平面との角度 = 0度
    assert!((insp.angle_to_xy_deg - 0.0).abs() < 1e-4);

    // Push-Pull: 天面を +Z 方向に 10mm 引っ張る (高さ 30 -> 40 に拡大)
    let modified_solid =
        zenith_algo::DirectModeling::push_pull_face(&solid, 1, 10.0).expect("Push-pull failed");

    let params = TessellationParams {
        u_divisions: 4,
        v_divisions: 4,
    };
    let mesh = zenith_tess::tessellate_solid(&modified_solid, &params);
    let mass = zenith_algo::MassCalculator::compute_from_mesh(&mesh);
    println!(
        "Push-Pull Volume: computed = {}, expected = 8000.0",
        mass.volume
    );
    assert!((mass.volume - 8000.0).abs() < 50.0);
}

#[test]
fn test_hollow_box_shell_solid_and_step() {
    let dx = 30.0;
    let dy = 40.0;
    let dz = 25.0;
    let t = 2.5;

    let hollow = zenith_algo::ShellBuilder::make_hollow_box(dx, dy, dz, t, 1)
        .expect("Hollow box shell failed");

    // 外側5面 + 内側5面 + 開口部リム4面 = 全14面
    assert_eq!(hollow.outer_shell.faces.len(), 14);

    let params = TessellationParams {
        u_divisions: 4,
        v_divisions: 4,
    };
    let mesh = zenith_tess::tessellate_solid(&hollow, &params);
    assert!(mesh.num_triangles() > 0);

    // 理論体積: 30*40*25 - (30 - 5)*(40 - 5)*(25 - 2.5) = 30000 - 25*35*22.5 = 30000 - 19687.5 = 10312.5
    let mass = zenith_algo::MassCalculator::compute_from_mesh(&mesh);
    let expected_vol = 10312.5;
    println!(
        "Hollow Box Volume: computed = {}, expected = {}",
        mass.volume, expected_vol
    );
    assert!((mass.volume - expected_vol).abs() < 50.0);

    // STEPエクスポート
    zenith_io::StepExporter::export_solid_to_file(
        &hollow,
        "target/samples/hollow_box.stp",
        "ZENITH_HOLLOW_BOX",
    )
    .expect("STEP export failed for hollow box");

    // STLエクスポート
    zenith_io::StlExporter::export_binary(&mesh, "target/samples/hollow_box.stl")
        .expect("STL export failed for hollow box");
}

#[test]
fn test_hollow_box_face_volume_contributions_are_oriented() {
    let hollow = zenith_algo::ShellBuilder::make_hollow_box(30.0, 40.0, 25.0, 2.5, 1)
        .expect("Hollow box shell failed");
    let params = TessellationParams {
        u_divisions: 4,
        v_divisions: 4,
    };

    let mut total: f64 = 0.0;
    for face in &hollow.outer_shell.faces {
        let mesh = zenith_tess::tessellate_face(face, &params);
        for tri in &mesh.indices {
            let p0 = mesh.positions[tri[0] as usize];
            let p1 = mesh.positions[tri[1] as usize];
            let p2 = mesh.positions[tri[2] as usize];
            let det = p0.x * (p1.y * p2.z - p1.z * p2.y) - p0.y * (p1.x * p2.z - p1.z * p2.x)
                + p0.z * (p1.x * p2.y - p1.y * p2.x);
            total += det / 6.0;
        }
    }

    assert!((total.abs() - 10312.5).abs() < 50.0);
}

#[test]
fn test_cone_solid_and_step() {
    let r_bot = 15.0;
    let r_top = 5.0;
    let h = 30.0;

    let cone =
        zenith_algo::PrimitiveBuilder::make_cone(r_bot, r_top, h).expect("Cone creation failed");

    // 4側面 + 底面 + 天面 = 6面
    assert_eq!(cone.outer_shell.faces.len(), 6);

    let params = TessellationParams {
        u_divisions: 32,
        v_divisions: 16,
    };
    let mesh = zenith_tess::tessellate_solid(&cone, &params);
    assert!(mesh.num_triangles() > 0);

    // 円錐台理論体積: 1/3 * pi * h * (r1^2 + r1*r2 + r2^2) = 1/3 * pi * 30 * (225 + 75 + 25) = 10210.18
    let mass = zenith_algo::MassCalculator::compute_from_mesh(&mesh);
    let expected_vol =
        std::f64::consts::PI / 3.0 * h * (r_bot.powi(2) + r_bot * r_top + r_top.powi(2));
    println!(
        "Cone Volume: computed = {}, expected = {}",
        mass.volume, expected_vol
    );
    assert!((mass.volume - expected_vol).abs() < 500.0);

    zenith_io::StepExporter::export_solid_to_file(&cone, "target/samples/cone.stp", "ZENITH_CONE")
        .expect("STEP export failed for cone");
    zenith_io::StlExporter::export_binary(&mesh, "target/samples/cone.stl")
        .expect("STL export failed for cone");
}

#[test]
fn test_torus_solid() {
    let r_maj = 20.0;
    let r_min = 5.0;

    let torus =
        zenith_algo::PrimitiveBuilder::make_torus(r_maj, r_min).expect("Torus creation failed");

    assert_eq!(torus.outer_shell.faces.len(), 1);

    let params = TessellationParams {
        u_divisions: 32,
        v_divisions: 16,
    };
    let mesh = zenith_tess::tessellate_solid(&torus, &params);
    assert!(mesh.num_triangles() > 0);

    // トーラス理論体積: 2 * pi^2 * R * r^2 = 2 * pi^2 * 20 * 25 ≈ 9869.6
    let mass = zenith_algo::MassCalculator::compute_from_mesh(&mesh);
    let expected_vol = 2.0 * std::f64::consts::PI.powi(2) * r_maj * r_min.powi(2);
    println!(
        "Torus Volume: computed = {}, expected = {}",
        mass.volume, expected_vol
    );
    assert!((mass.volume - expected_vol).abs() < 500.0);

    zenith_io::StlExporter::export_binary(&mesh, "target/samples/torus.stl")
        .expect("STL export failed for torus");
}

#[test]
fn test_dihedral_angle_inspection() {
    let solid = zenith_algo::PrimitiveBuilder::make_box(10.0, 20.0, 30.0).unwrap();
    // 最初の面の最初のエッジID
    let edge_id = solid.outer_shell.faces[0].outer_wire.edges[0].edge.id;

    let insp = zenith_algo::DirectModeling::inspect_solid_edge(&solid, edge_id)
        .expect("Edge inspection failed");

    // 長さは 10 または 20 または 30
    assert!(insp.length > 0.0);
    // 直方体の二面角は 90度
    if let Some(angle) = insp.dihedral_angle_deg {
        assert!(
            (angle - 90.0).abs() < 1e-3,
            "Box dihedral angle should be 90 deg"
        );
    }
}

#[test]
fn test_single_edge_fillet() {
    let dx = 20.0;
    let dy = 30.0;
    let dz = 20.0;
    let r = 4.0;

    let solid = zenith_algo::DirectModeling::fillet_box_single_edge(dx, dy, dz, 0, r)
        .expect("Single edge fillet failed");

    // 4側面 + 1円弧フィレット面 + 底面 + 天面 = 7面
    assert_eq!(solid.outer_shell.faces.len(), 7);

    let params = TessellationParams {
        u_divisions: 16,
        v_divisions: 16,
    };
    let mesh = zenith_tess::tessellate_solid(&solid, &params);
    assert!(mesh.num_triangles() > 0);

    // 理論体積: dx*dy*dz - (4 - pi) * r^2 * dz
    let mass = zenith_algo::MassCalculator::compute_from_mesh(&mesh);
    let expected_vol = dx * dy * dz - (1.0 - std::f64::consts::PI * 0.25) * r.powi(2) * dz;
    println!(
        "Single Edge Fillet Volume: computed = {}, expected = {}",
        mass.volume, expected_vol
    );
    assert!((mass.volume - expected_vol).abs() < 50.0);

    zenith_io::StepExporter::export_solid_to_file(
        &solid,
        "target/samples/single_fillet_box.stp",
        "ZENITH_SINGLE_FILLET_BOX",
    )
    .expect("STEP export failed for single fillet box");
    zenith_io::StlExporter::export_binary(&mesh, "target/samples/single_fillet_box.stl")
        .expect("STL export failed for single fillet box");
}

#[test]
fn test_step_import_roundtrip() {
    // 1. 直方体ソリッドのSTEP出力
    let box_solid = zenith_algo::PrimitiveBuilder::make_box(15.0, 25.0, 35.0).unwrap();
    let step_path = "target/samples/roundtrip_box.stp";
    zenith_io::StepExporter::export_solid_to_file(&box_solid, step_path, "ROUNDTRIP_BOX")
        .expect("Export failed");

    // 2. STEPファイルのインポート
    let imported_solid =
        zenith_io::StepImporter::import_solid_from_file(step_path).expect("Import failed");

    assert_eq!(imported_solid.outer_shell.faces.len(), 6);

    let params = TessellationParams {
        u_divisions: 4,
        v_divisions: 4,
    };
    let mesh = zenith_tess::tessellate_solid(&imported_solid, &params);
    let mass = zenith_algo::MassCalculator::compute_from_mesh(&mesh);
    let expected_vol = 15.0 * 25.0 * 35.0; // = 13125.0

    println!(
        "Imported Box Volume: computed = {}, expected = {}",
        mass.volume, expected_vol
    );
    assert!((mass.volume - expected_vol).abs() < 50.0);
}

#[test]
fn test_cylinder_step_import_roundtrip_preserves_curved_faces() {
    let cylinder = zenith_algo::PrimitiveBuilder::make_cylinder(10.0, 30.0).unwrap();
    let step = zenith_io::StepExporter::export_solid_to_string(&cylinder, "ROUNDTRIP_CYLINDER");
    let imported_solid =
        zenith_io::StepImporter::import_solid_from_str(&step).expect("Import failed");

    assert_eq!(imported_solid.outer_shell.faces.len(), 6);
    assert!(imported_solid.is_topologically_valid(&Tolerance::default()));

    let nurbs_face_count = imported_solid
        .outer_shell
        .faces
        .iter()
        .filter(|face| matches!(face.geometry, FaceGeometry::Nurbs(_)))
        .count();
    assert_eq!(nurbs_face_count, 4);

    let params = TessellationParams {
        u_divisions: 32,
        v_divisions: 16,
    };
    let mesh = zenith_tess::tessellate_solid(&imported_solid, &params);
    let mass = zenith_algo::MassCalculator::compute_from_mesh(&mesh);
    let expected_vol = std::f64::consts::PI * 100.0 * 30.0;

    assert!(mass.volume > expected_vol * 0.85 && mass.volume < expected_vol * 1.05);
}

#[test]
fn test_offset_multiple_faces() {
    let solid = zenith_algo::PrimitiveBuilder::make_box(20.0, 30.0, 20.0).unwrap();
    // 天面（index 1: +Z）を +10mm, 正面（index 2: -Y）を +10mm 移動
    let modified =
        zenith_algo::DirectModeling::offset_multiple_faces(&solid, &[(1, 10.0), (2, 10.0)])
            .expect("Offset multiple faces failed");

    let params = TessellationParams {
        u_divisions: 4,
        v_divisions: 4,
    };
    let mesh = zenith_tess::tessellate_solid(&modified, &params);
    let mass = zenith_algo::MassCalculator::compute_from_mesh(&mesh);
    // 新しい寸法: 20 x 40 x 30 = 24000.0
    let expected_vol = 20.0 * 40.0 * 30.0;
    println!(
        "Offset Multiple Faces Volume: computed = {}, expected = {}",
        mass.volume, expected_vol
    );
    assert!((mass.volume - expected_vol).abs() < 100.0);
}

#[test]
fn test_extend_edge() {
    let vs = zenith_topo::Vertex::from_point(Point3::new(0.0, 0.0, 0.0));
    let ve = zenith_topo::Vertex::from_point(Point3::new(10.0, 0.0, 0.0));
    let edge = zenith_topo::Edge::line_between(vs, ve).unwrap();

    let extended =
        zenith_algo::DirectModeling::extend_edge(&edge, 5.0, 5.0).expect("Extend edge failed");

    let insp = zenith_algo::DirectModeling::inspect_edge(&extended);
    assert!((insp.length - 20.0).abs() < 1e-6);
    assert!((insp.start_point.x - (-5.0)).abs() < 1e-6);
    assert!((insp.end_point.x - 15.0).abs() < 1e-6);
}

#[test]
fn test_gltf_export() {
    let solid = zenith_algo::PrimitiveBuilder::make_box(10.0, 10.0, 10.0).unwrap();
    let params = TessellationParams {
        u_divisions: 4,
        v_divisions: 4,
    };
    let mesh = zenith_tess::tessellate_solid(&solid, &params);

    let gltf_path = "target/samples/box.gltf";
    zenith_io::GltfExporter::export_to_file(&mesh, gltf_path).expect("glTF export failed");

    let content = std::fs::read_to_string(gltf_path).unwrap();
    assert!(content.contains("\"version\": \"2.0\""));
    assert!(content.contains("Zenith_Solid_Mesh"));
}

#[test]
fn test_iges_export() {
    let solid = zenith_algo::PrimitiveBuilder::make_box(12.0, 15.0, 18.0).unwrap();
    let iges_path = "target/samples/box.igs";
    zenith_io::IgesExporter::export_solid_to_file(&solid, iges_path, "ZENITH_BOX_IGES")
        .expect("IGES export failed");

    let content = std::fs::read_to_string(iges_path).unwrap();
    assert!(content.contains("ZENITH_BOX_IGES"));
    assert!(content.contains("186")); // Manifold Solid B-Rep Type 186
}

#[test]
fn test_thicken_planar_face() {
    let box_solid = zenith_algo::PrimitiveBuilder::make_box(30.0, 40.0, 10.0).unwrap();
    // 底面 (index 0) を取り出して厚み 5.0mm でソリッド化
    let bottom_face = &box_solid.outer_shell.faces[0];
    let tol = zenith_math::Tolerance::default();
    let thickened = zenith_algo::ThickenBuilder::thicken_face(bottom_face, 5.0, &tol)
        .expect("Thicken face failed");

    assert_eq!(thickened.outer_shell.faces.len(), 6);

    let params = TessellationParams {
        u_divisions: 4,
        v_divisions: 4,
    };
    let mesh = zenith_tess::tessellate_solid(&thickened, &params);
    let mass = zenith_algo::MassCalculator::compute_from_mesh(&mesh);
    let expected_vol = 30.0 * 40.0 * 5.0; // = 6000.0

    println!(
        "Thickened Face Volume: computed = {}, expected = {}",
        mass.volume, expected_vol
    );
    assert!((mass.volume - expected_vol).abs() < 50.0);
}

#[test]
fn test_assembly_hierarchy() {
    let box1 = zenith_algo::PrimitiveBuilder::make_box(10.0, 10.0, 10.0).unwrap();
    let box2 = zenith_algo::PrimitiveBuilder::make_box(20.0, 20.0, 20.0).unwrap();

    let mut asm = zenith_topo::Assembly::new("Engine_Assembly");
    asm.add_instance(zenith_topo::ComponentInstance::new(
        "Piston_1",
        box1,
        zenith_topo::Transform3::identity(),
    ));

    let mut sub_asm = zenith_topo::Assembly::new("Crankcase_SubAssembly");
    sub_asm.add_instance(zenith_topo::ComponentInstance::new(
        "Block",
        box2,
        zenith_topo::Transform3::translation(0.0, 50.0, 0.0),
    ));

    asm.add_sub_assembly(sub_asm);

    assert_eq!(asm.total_instance_count(), 2);
    assert_eq!(asm.sub_assemblies.len(), 1);
}

#[test]
fn test_shader_brep_payload() {
    let solid = zenith_algo::PrimitiveBuilder::make_box(25.0, 35.0, 15.0).unwrap();
    let payload = zenith_topo::ShaderBRepPayload::from_solid(&solid);

    assert_eq!(payload.faces.len(), 6);
    assert_eq!(payload.edges.len(), 24); // 各面4エッジ x 6面
    assert!((payload.bbox_max[0] - 25.0).abs() < 1e-4);
    assert!((payload.bbox_max[1] - 35.0).abs() < 1e-4);
    assert!((payload.bbox_max[2] - 15.0).abs() < 1e-4);
}

#[test]
fn test_parametric_feature_tree_recompute() {
    let mut tree = zenith_algo::FeatureTree::new();

    // 1. Feature 1: 直方体 (20x30x20)
    let f1_id = tree.add_feature(
        "Base_Box",
        zenith_algo::FeatureOp::CreateBox {
            dx: 20.0,
            dy: 30.0,
            dz: 20.0,
        },
    );

    // 初期ソリッドの天面 (+Z) シグネチャを取得
    let base_solid = zenith_algo::PrimitiveBuilder::make_box(20.0, 30.0, 20.0).unwrap();
    let top_face_sig = zenith_topo::GeometricSignature::from_face(&base_solid.outer_shell.faces[1]);

    // 2. Feature 2: 天面を +10mm 移動
    tree.add_feature(
        "Move_Top_Face",
        zenith_algo::FeatureOp::PushPullFace {
            target_signature: top_face_sig.clone(),
            distance: 10.0,
        },
    );

    // 3. 初回再計算: 20 x 30 x (20+10) = 18000.0
    let res1 = tree.recompute().expect("Recompute 1 failed");
    let params = TessellationParams {
        u_divisions: 4,
        v_divisions: 4,
    };
    let mesh1 = zenith_tess::tessellate_solid(&res1, &params);
    let mass1 = zenith_algo::MassCalculator::compute_from_mesh(&mesh1);
    assert!((mass1.volume - 18000.0).abs() < 50.0);

    // 4. 【過去の寸法変更】Feature 1 の寸法を (30x40x20) に変更！
    tree.update_feature_op(
        &f1_id,
        zenith_algo::FeatureOp::CreateBox {
            dx: 30.0,
            dy: 40.0,
            dz: 20.0,
        },
    )
    .expect("Update op failed");

    // 5. 【TNP自己修復・再計算】天面が自動同定されて +10mm 移動: 30 x 40 x (20+10) = 36000.0
    let res2 = tree
        .recompute()
        .expect("Recompute 2 with TNP recovery failed");
    let mesh2 = zenith_tess::tessellate_solid(&res2, &params);
    let mass2 = zenith_algo::MassCalculator::compute_from_mesh(&mesh2);
    println!(
        "TNP Recovered Feature Tree Volume: computed = {}, expected = 36000.0",
        mass2.volume
    );
    assert!((mass2.volume - 36000.0).abs() < 100.0);
}

#[test]
fn test_sketch_solver_rectangle_and_tangent() {
    let mut solver = zenith_algo::SketchSolver::new();

    // 4つの頂点で四角形を定義
    let p0 = solver.add_fixed_point(0.0, 0.0); // 原点固定
    let p1 = solver.add_point(9.5, 0.5); // (10, 0) 近傍
    let p2 = solver.add_point(10.2, 19.8); // (10, 20) 近傍
    let p3 = solver.add_point(0.3, 20.1); // (0, 20) 近傍

    let l01 = solver.add_line(p0, p1);
    let _l12 = solver.add_line(p1, p2);
    let _l23 = solver.add_line(p2, p3);
    let _l30 = solver.add_line(p3, p0);

    // 拘束: 水平・垂直・寸法拘束 (10mm x 20mm)
    solver.add_constraint(zenith_algo::Constraint::Horizontal(p0, p1));
    solver.add_constraint(zenith_algo::Constraint::Vertical(p1, p2));
    solver.add_constraint(zenith_algo::Constraint::Horizontal(p3, p2));
    solver.add_constraint(zenith_algo::Constraint::Vertical(p0, p3));
    solver.add_constraint(zenith_algo::Constraint::Distance(p0, p1, 10.0));
    solver.add_constraint(zenith_algo::Constraint::Distance(p1, p2, 20.0));

    // 円を追加し、l01 に接する拘束
    let p_center = solver.add_point(5.0, 6.0);
    let c1 = solver.add_circle(p_center, 4.0);
    solver.add_constraint(zenith_algo::Constraint::HorizontalDistance(
        p0, p_center, 5.0,
    ));
    solver.add_constraint(zenith_algo::Constraint::TangentLineCircle(l01, c1));

    let iters = solver.solve(50, 1e-6).expect("Sketch solve failed");
    println!("Sketch solver converged in {} iterations", iters);

    let pt0 = solver.get_point(p0).unwrap();
    let pt1 = solver.get_point(p1).unwrap();
    let pt2 = solver.get_point(p2).unwrap();
    let pt3 = solver.get_point(p3).unwrap();
    let pt_c = solver.get_point(p_center).unwrap();

    assert!((pt0[0] - 0.0).abs() < 1e-4);
    assert!((pt0[1] - 0.0).abs() < 1e-4);
    assert!((pt1[0] - 10.0).abs() < 1e-4);
    assert!((pt1[1] - 0.0).abs() < 1e-4);
    assert!((pt2[0] - 10.0).abs() < 1e-4);
    assert!((pt2[1] - 20.0).abs() < 1e-4);
    assert!((pt3[0] - 0.0).abs() < 1e-4);
    assert!((pt3[1] - 20.0).abs() < 1e-4);

    // 円の中心 (5.0, 4.0) で底辺 y=0 に接する (距離=半径4.0)
    assert!((pt_c[0] - 5.0).abs() < 1e-3);
    assert!((pt_c[1] - 4.0).abs() < 1e-3);
}

#[test]
fn test_g2_surface_blend() {
    let pts1 = vec![
        zenith_geom::nurbs_curve::ControlPoint3::unweighted(zenith_math::Point3::new(
            0.0, 0.0, 0.0,
        )),
        zenith_geom::nurbs_curve::ControlPoint3::unweighted(zenith_math::Point3::new(
            10.0, 0.0, 0.0,
        )),
    ];
    let pts2 = vec![
        zenith_geom::nurbs_curve::ControlPoint3::unweighted(zenith_math::Point3::new(
            0.0, 10.0, 5.0,
        )),
        zenith_geom::nurbs_curve::ControlPoint3::unweighted(zenith_math::Point3::new(
            10.0, 10.0, 5.0,
        )),
    ];
    let knots = zenith_geom::bspline_basis::KnotVector::clamped_uniform(2, 1);
    let rail1 = zenith_geom::nurbs_curve::NurbsCurve3::new(1, pts1, knots.clone()).unwrap();
    let rail2 = zenith_geom::nurbs_curve::NurbsCurve3::new(1, pts2, knots).unwrap();

    let tol = zenith_math::Tolerance::default();
    let g2_blend = zenith_geom::SurfaceBlend3::create_g2_blend(rail1, rail2, 1.0, 1.0, &tol)
        .expect("G2 Blend creation failed");

    assert_eq!(g2_blend.blend_surface.degree_v, 5); // 5次 (Class-A)
    let p_mid = g2_blend.evaluate(0.5, 0.5);
    assert!(p_mid.y > 0.0 && p_mid.y < 10.0);
}
