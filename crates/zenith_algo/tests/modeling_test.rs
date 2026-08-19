use zenith_algo::{ExtrudeBuilder, LoftBuilder, RevolveBuilder};
use zenith_geom::{
    ControlPoint2, ControlPoint3, KnotVector, NurbsCurve2, NurbsCurve3, NurbsSurface3,
    PlaneSurface3, Surface3,
};
use zenith_io::ObjExporter;
use zenith_math::{Point3, Tolerance, Vec3};
use zenith_tess::{tessellate_face, tessellate_solid, tessellate_surface, TessellationParams};
use zenith_topo::{
    Edge, Face, FaceGeometry, FacePcurveLoop, FacePcurveSegment, FacePcurves, Orientation,
    OrientedEdge, Shell, Vertex, Wire,
};

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
fn test_exact_brep_boolean_entry_is_separate_from_mesh_preview() {
    let tol = Tolerance::default();
    let tess_params = TessellationParams {
        u_divisions: 4,
        v_divisions: 4,
    };
    let solid_a = zenith_algo::PrimitiveBuilder::make_box(10.0, 10.0, 10.0).unwrap();
    let solid_b = zenith_algo::PrimitiveBuilder::make_cylinder(3.0, 10.0).unwrap();

    let preview_mesh = zenith_algo::BooleanEngine::boolean_solids_mesh_preview(
        &solid_a,
        &solid_b,
        zenith_algo::BooleanOpType::Union,
        &tess_params,
        &tol,
    )
    .expect("mesh preview boolean should remain available");
    assert!(preview_mesh.num_triangles() > 0);

    let err = zenith_algo::BooleanEngine::boolean_solids_exact(
        &solid_a,
        &solid_b,
        zenith_algo::BooleanOpType::Union,
        &tol,
    )
    .expect_err("exact B-Rep boolean must not silently fall back to mesh output");
    assert!(err.contains("Exact B-Rep boolean is not implemented yet"));
}

#[test]
fn test_exact_brep_boolean_preparation_reports_pipeline_counts() {
    let tol = Tolerance::default();
    let solid_a = zenith_algo::PrimitiveBuilder::make_box(10.0, 10.0, 10.0).unwrap();
    let solid_b = zenith_algo::PrimitiveBuilder::make_cylinder(3.0, 10.0).unwrap();

    let report = zenith_algo::BooleanEngine::prepare_exact_boolean(
        &solid_a,
        &solid_b,
        zenith_algo::BooleanOpType::Union,
        &tol,
    )
    .expect("exact boolean preparation report");

    assert!(report.face_pair_candidate_count > 0);
    assert!(report.intersection_edge_candidate_count <= report.face_pair_candidate_count);
    assert!(report.planar_split_candidate_count <= report.intersection_edge_candidate_count);
    assert!(
        report.planar_batch_applied_split_count + report.planar_batch_skipped_split_count
            <= report.intersection_edge_candidate_count * 2
    );
    assert!(report.classified_split_candidate_count <= report.planar_split_candidate_count);
    assert!(report.selected_face_piece_count > 0);
    assert!(report.selected_with_caps_face_piece_count >= report.selected_face_piece_count);
    assert!(report.planar_cap_face_count <= report.planar_cap_loop_count);
}

#[test]
fn test_exact_boolean_preparation_reaches_cylinder_side_splits_for_slab_cut() {
    let tol = Tolerance::default();
    let slab = zenith_algo::BrepTransform::translate_solid(
        &zenith_algo::PrimitiveBuilder::make_box(24.0, 24.0, 6.0).unwrap(),
        Vec3::new(-12.0, -12.0, 12.0),
    );
    let cylinder = zenith_algo::PrimitiveBuilder::make_cylinder(10.0, 30.0).unwrap();

    let report = zenith_algo::BooleanEngine::prepare_exact_boolean(
        &slab,
        &cylinder,
        zenith_algo::BooleanOpType::Intersection,
        &tol,
    )
    .expect("slab-cylinder exact boolean preparation report");

    assert!(report.face_pair_candidate_count > 0);
    assert!(report.intersection_edge_candidate_count > 0);
    assert!(
        report.planar_batch_applied_split_count > 0,
        "horizontal slab faces should drive cylinder-side split preparation"
    );
    assert!(report.selected_face_piece_count > 0);
}

#[test]
fn test_exact_brep_boolean_returns_cylinder_slice_for_slab_intersection() {
    let tol = Tolerance::default();
    let slab = zenith_algo::BrepTransform::translate_solid(
        &zenith_algo::PrimitiveBuilder::make_box(24.0, 24.0, 6.0).unwrap(),
        Vec3::new(-12.0, -12.0, 12.0),
    );
    let cylinder = zenith_algo::PrimitiveBuilder::make_cylinder(10.0, 30.0).unwrap();

    let slice = zenith_algo::BooleanEngine::boolean_solids_exact(
        &slab,
        &cylinder,
        zenith_algo::BooleanOpType::Intersection,
        &tol,
    )
    .expect("slab-cylinder exact intersection should return a B-Rep cylinder slice");

    assert_eq!(slice.outer_shell.faces.len(), 6);
    assert!(slice.is_topologically_valid(&tol));
    let params = TessellationParams {
        u_divisions: 48,
        v_divisions: 8,
    };
    let mesh = tessellate_solid(&slice, &params);
    let props = zenith_algo::MassCalculator::compute_from_mesh(&mesh);
    let expected_volume = std::f64::consts::PI * 10.0 * 10.0 * 6.0;
    assert!(props.volume > expected_volume * 0.9);
    assert!(props.volume < expected_volume * 1.05);

    let step = zenith_io::StepExporter::export_solid_to_string(&slice, "CYLINDER_SLICE");
    assert!(step.contains("MANIFOLD_SOLID_BREP"));
    assert!(step.contains("B_SPLINE_SURFACE_WITH_KNOTS"));
}

#[test]
fn test_exact_brep_boolean_cylinder_slab_intersection_is_operand_order_independent() {
    let tol = Tolerance::default();
    let slab = zenith_algo::BrepTransform::translate_solid(
        &zenith_algo::PrimitiveBuilder::make_box(24.0, 24.0, 6.0).unwrap(),
        Vec3::new(-12.0, -12.0, 12.0),
    );
    let cylinder = zenith_algo::PrimitiveBuilder::make_cylinder(10.0, 30.0).unwrap();

    let a = zenith_algo::BooleanEngine::boolean_solids_exact(
        &slab,
        &cylinder,
        zenith_algo::BooleanOpType::Intersection,
        &tol,
    )
    .expect("slab-cylinder exact intersection");
    let b = zenith_algo::BooleanEngine::boolean_solids_exact(
        &cylinder,
        &slab,
        zenith_algo::BooleanOpType::Intersection,
        &tol,
    )
    .expect("cylinder-slab exact intersection");

    assert_eq!(a.outer_shell.faces.len(), b.outer_shell.faces.len());
    assert!(a.is_topologically_valid(&tol));
    assert!(b.is_topologically_valid(&tol));
    let params = TessellationParams {
        u_divisions: 48,
        v_divisions: 8,
    };
    let volume_a =
        zenith_algo::MassCalculator::compute_from_mesh(&tessellate_solid(&a, &params)).volume;
    let volume_b =
        zenith_algo::MassCalculator::compute_from_mesh(&tessellate_solid(&b, &params)).volume;
    assert!((volume_a - volume_b).abs() < 1e-6);
}

#[test]
fn test_exact_brep_boolean_trims_cylinder_end_for_slab_difference() {
    let tol = Tolerance::default();
    let cylinder = zenith_algo::PrimitiveBuilder::make_cylinder(10.0, 30.0).unwrap();
    let bottom_slab = zenith_algo::BrepTransform::translate_solid(
        &zenith_algo::PrimitiveBuilder::make_box(24.0, 24.0, 6.0).unwrap(),
        Vec3::new(-12.0, -12.0, 0.0),
    );

    let trimmed = zenith_algo::BooleanEngine::boolean_solids_exact(
        &cylinder,
        &bottom_slab,
        zenith_algo::BooleanOpType::Difference,
        &tol,
    )
    .expect("cylinder minus end slab should return a shortened B-Rep cylinder");

    assert_eq!(trimmed.outer_shell.faces.len(), 6);
    assert!(trimmed.is_topologically_valid(&tol));
    let sampled: Vec<Point3> = trimmed
        .outer_shell
        .faces
        .iter()
        .flat_map(|face| face.outer_wire.sample_points(8))
        .collect();
    let z_min = sampled
        .iter()
        .map(|point| point.z)
        .fold(f64::INFINITY, f64::min);
    let z_max = sampled
        .iter()
        .map(|point| point.z)
        .fold(f64::NEG_INFINITY, f64::max);
    assert!((z_min - 6.0).abs() < 1e-6);
    assert!((z_max - 30.0).abs() < 1e-6);

    let params = TessellationParams {
        u_divisions: 48,
        v_divisions: 8,
    };
    let mesh = tessellate_solid(&trimmed, &params);
    let props = zenith_algo::MassCalculator::compute_from_mesh(&mesh);
    let expected_volume = std::f64::consts::PI * 10.0 * 10.0 * 24.0;
    assert!(props.volume > expected_volume * 0.9);
    assert!(props.volume < expected_volume * 1.05);
}

#[test]
fn test_exact_brep_boolean_trims_cylinder_top_for_slab_difference() {
    let tol = Tolerance::default();
    let cylinder = zenith_algo::PrimitiveBuilder::make_cylinder(10.0, 30.0).unwrap();
    let top_slab = zenith_algo::BrepTransform::translate_solid(
        &zenith_algo::PrimitiveBuilder::make_box(24.0, 24.0, 6.0).unwrap(),
        Vec3::new(-12.0, -12.0, 24.0),
    );

    let trimmed = zenith_algo::BooleanEngine::boolean_solids_exact(
        &cylinder,
        &top_slab,
        zenith_algo::BooleanOpType::Difference,
        &tol,
    )
    .expect("cylinder minus top slab should return a shortened B-Rep cylinder");

    assert!(trimmed.is_topologically_valid(&tol));
    let sampled: Vec<Point3> = trimmed
        .outer_shell
        .faces
        .iter()
        .flat_map(|face| face.outer_wire.sample_points(8))
        .collect();
    let z_min = sampled
        .iter()
        .map(|point| point.z)
        .fold(f64::INFINITY, f64::min);
    let z_max = sampled
        .iter()
        .map(|point| point.z)
        .fold(f64::NEG_INFINITY, f64::max);
    assert!(z_min.abs() < 1e-6);
    assert!((z_max - 24.0).abs() < 1e-6);
}

#[test]
fn test_exact_brep_boolean_rejects_middle_slab_cylinder_difference_as_compound() {
    let tol = Tolerance::default();
    let cylinder = zenith_algo::PrimitiveBuilder::make_cylinder(10.0, 30.0).unwrap();
    let middle_slab = zenith_algo::BrepTransform::translate_solid(
        &zenith_algo::PrimitiveBuilder::make_box(24.0, 24.0, 6.0).unwrap(),
        Vec3::new(-12.0, -12.0, 12.0),
    );

    let err = zenith_algo::BooleanEngine::boolean_solids_exact(
        &cylinder,
        &middle_slab,
        zenith_algo::BooleanOpType::Difference,
        &tol,
    )
    .expect_err("middle slab difference needs compound result support");

    assert!(err.contains("disjoint solids"));
}

#[test]
fn test_exact_brep_boolean_result_returns_two_cylinders_for_middle_slab_difference() {
    let tol = Tolerance::default();
    let cylinder = zenith_algo::PrimitiveBuilder::make_cylinder(10.0, 30.0).unwrap();
    let middle_slab = zenith_algo::BrepTransform::translate_solid(
        &zenith_algo::PrimitiveBuilder::make_box(24.0, 24.0, 6.0).unwrap(),
        Vec3::new(-12.0, -12.0, 12.0),
    );

    let result = zenith_algo::BooleanEngine::boolean_solids_exact_result(
        &cylinder,
        &middle_slab,
        zenith_algo::BooleanOpType::Difference,
        &tol,
    )
    .expect("middle slab difference should return a multi-solid exact result");

    assert_eq!(result.solids.len(), 2);
    let params = TessellationParams {
        u_divisions: 48,
        v_divisions: 8,
    };
    let result_mesh = result.tessellate(&params);
    let total_volume = zenith_algo::MassCalculator::compute_from_mesh(&result_mesh).volume;
    let mut z_spans = Vec::new();
    for solid in &result.solids {
        assert_eq!(solid.outer_shell.faces.len(), 6);
        assert!(solid.is_topologically_valid(&tol));
        let sampled: Vec<Point3> = solid
            .outer_shell
            .faces
            .iter()
            .flat_map(|face| face.outer_wire.sample_points(8))
            .collect();
        let z_min = sampled
            .iter()
            .map(|point| point.z)
            .fold(f64::INFINITY, f64::min);
        let z_max = sampled
            .iter()
            .map(|point| point.z)
            .fold(f64::NEG_INFINITY, f64::max);
        z_spans.push((z_min, z_max));
    }
    z_spans.sort_by(|a, b| a.0.total_cmp(&b.0));

    assert!((z_spans[0].0 - 0.0).abs() < 1e-6);
    assert!((z_spans[0].1 - 12.0).abs() < 1e-6);
    assert!((z_spans[1].0 - 18.0).abs() < 1e-6);
    assert!((z_spans[1].1 - 30.0).abs() < 1e-6);
    let expected_volume = std::f64::consts::PI * 10.0 * 10.0 * 24.0;
    assert!(total_volume > expected_volume * 0.9);
    assert!(total_volume < expected_volume * 1.05);
}

#[test]
fn test_step_export_writes_multi_solid_boolean_result() {
    let tol = Tolerance::default();
    let cylinder = zenith_algo::PrimitiveBuilder::make_cylinder(10.0, 30.0).unwrap();
    let middle_slab = zenith_algo::BrepTransform::translate_solid(
        &zenith_algo::PrimitiveBuilder::make_box(24.0, 24.0, 6.0).unwrap(),
        Vec3::new(-12.0, -12.0, 12.0),
    );
    let result = zenith_algo::BooleanEngine::boolean_solids_exact_result(
        &cylinder,
        &middle_slab,
        zenith_algo::BooleanOpType::Difference,
        &tol,
    )
    .expect("multi-solid cylinder difference");

    let step =
        zenith_io::StepExporter::export_solids_to_string(&result.solids, "CYLINDER_MIDDLE_CUT");

    assert_eq!(step.matches("MANIFOLD_SOLID_BREP").count(), 2);
    assert!(step.contains("ADVANCED_BREP_SHAPE_REPRESENTATION"));
    assert!(step.contains("CYLINDER_MIDDLE_CUT_1"));
    assert!(step.contains("B_SPLINE_SURFACE_WITH_KNOTS"));
}

#[test]
fn test_step_import_roundtrips_multi_solid_boolean_result() {
    let tol = Tolerance::default();
    let cylinder = zenith_algo::PrimitiveBuilder::make_cylinder(10.0, 30.0).unwrap();
    let middle_slab = zenith_algo::BrepTransform::translate_solid(
        &zenith_algo::PrimitiveBuilder::make_box(24.0, 24.0, 6.0).unwrap(),
        Vec3::new(-12.0, -12.0, 12.0),
    );
    let result = zenith_algo::BooleanEngine::boolean_solids_exact_result(
        &cylinder,
        &middle_slab,
        zenith_algo::BooleanOpType::Difference,
        &tol,
    )
    .expect("multi-solid cylinder difference");
    let step =
        zenith_io::StepExporter::export_solids_to_string(&result.solids, "CYLINDER_MIDDLE_CUT");

    let imported = zenith_io::StepImporter::import_solids_from_str(&step)
        .expect("multi-solid STEP should import");

    assert_eq!(imported.len(), 2);
    assert!(imported
        .iter()
        .all(|solid| solid.is_topologically_valid(&tol)));
    assert!(imported.iter().all(|solid| solid
        .outer_shell
        .faces
        .iter()
        .any(|face| matches!(face.geometry, FaceGeometry::Nurbs(_)))));

    let params = TessellationParams {
        u_divisions: 48,
        v_divisions: 12,
    };
    let mut z_spans = Vec::new();
    let total_volume: f64 = imported
        .iter()
        .map(|solid| {
            let mesh = tessellate_solid(solid, &params);
            let z_min = mesh
                .positions
                .iter()
                .map(|point| point.z)
                .fold(f64::INFINITY, f64::min);
            let z_max = mesh
                .positions
                .iter()
                .map(|point| point.z)
                .fold(f64::NEG_INFINITY, f64::max);
            z_spans.push((z_min, z_max));
            zenith_algo::MassCalculator::compute_from_mesh(&mesh).volume
        })
        .sum();
    z_spans.sort_by(|a, b| a.0.total_cmp(&b.0));

    assert!((z_spans[0].0 - 0.0).abs() < 1e-6);
    assert!((z_spans[0].1 - 12.0).abs() < 1e-6);
    assert!((z_spans[1].0 - 18.0).abs() < 1e-6);
    assert!((z_spans[1].1 - 30.0).abs() < 1e-6);
    let expected_volume = std::f64::consts::PI * 10.0 * 10.0 * 24.0;
    assert!(total_volume > expected_volume * 0.9);
    assert!(total_volume < expected_volume * 1.05);
}

#[test]
fn test_exact_boolean_result_shape_step_roundtrip_preserves_compound_solids() {
    let tol = Tolerance::default();
    let cylinder = zenith_algo::PrimitiveBuilder::make_cylinder(10.0, 30.0).unwrap();
    let middle_slab = zenith_algo::BrepTransform::translate_solid(
        &zenith_algo::PrimitiveBuilder::make_box(24.0, 24.0, 6.0).unwrap(),
        Vec3::new(-12.0, -12.0, 12.0),
    );
    let result = zenith_algo::BooleanEngine::boolean_solids_exact_result(
        &cylinder,
        &middle_slab,
        zenith_algo::BooleanOpType::Difference,
        &tol,
    )
    .expect("multi-solid cylinder difference");

    let shape = result.to_shape();
    assert_eq!(shape.solid_count(), 2);
    assert!(matches!(shape, zenith_topo::Shape::Compound(_)));

    let step = zenith_io::StepExporter::export_shape_to_string(&shape, "BOOLEAN_COMPOUND")
        .expect("compound shape STEP export");
    let imported_shape =
        zenith_io::StepImporter::import_shape_from_str(&step).expect("compound shape STEP import");

    assert_eq!(imported_shape.solid_count(), 2);
    assert!(matches!(imported_shape, zenith_topo::Shape::Compound(_)));
    assert!(imported_shape
        .solids()
        .iter()
        .all(|solid| solid.is_topologically_valid(&tol)));
}

#[test]
fn test_exact_brep_boolean_returns_solid_for_identical_union_and_intersection() {
    let tol = Tolerance::default();
    let solid = zenith_algo::PrimitiveBuilder::make_box(10.0, 10.0, 10.0).unwrap();

    let union = zenith_algo::BooleanEngine::boolean_solids_exact(
        &solid,
        &solid,
        zenith_algo::BooleanOpType::Union,
        &tol,
    )
    .expect("identical exact union should return the same B-Rep solid");
    assert_eq!(union.outer_shell.faces.len(), solid.outer_shell.faces.len());
    assert!(union.is_topologically_valid(&tol));

    let intersection = zenith_algo::BooleanEngine::boolean_solids_exact(
        &solid,
        &solid,
        zenith_algo::BooleanOpType::Intersection,
        &tol,
    )
    .expect("identical exact intersection should return the same B-Rep solid");
    assert_eq!(
        intersection.outer_shell.faces.len(),
        solid.outer_shell.faces.len()
    );
    assert!(intersection.is_topologically_valid(&tol));
}

#[test]
fn test_brep_transform_translates_solid_without_breaking_topology() {
    let tol = Tolerance::default();
    let solid = zenith_algo::PrimitiveBuilder::make_box(10.0, 20.0, 30.0).unwrap();
    let moved = zenith_algo::BrepTransform::translate_solid(&solid, Vec3::new(3.0, -4.0, 5.0));

    assert!(moved.is_topologically_valid(&tol));
    assert_eq!(moved.outer_shell.faces.len(), solid.outer_shell.faces.len());

    let original_start = solid.outer_shell.faces[0].outer_wire.edges[0]
        .start_vertex()
        .point;
    let moved_start = moved.outer_shell.faces[0].outer_wire.edges[0]
        .start_vertex()
        .point;
    assert!((moved_start - original_start - Vec3::new(3.0, -4.0, 5.0)).norm() <= tol.linear);
}

#[test]
fn test_brep_transform_reverses_shell_orientation_for_cavities() {
    let tol = Tolerance::default();
    let solid = zenith_algo::PrimitiveBuilder::make_box(3.0, 3.0, 3.0).unwrap();
    let reversed = zenith_algo::BrepTransform::reverse_shell_orientation(&solid.outer_shell);

    assert!(reversed.is_topologically_closed(&tol));
    assert_eq!(reversed.faces.len(), solid.outer_shell.faces.len());
    assert_ne!(
        reversed.faces[0].orientation,
        solid.outer_shell.faces[0].orientation
    );
    assert_eq!(
        reversed.faces[0].outer_wire.edges[0].orientation,
        solid.outer_shell.faces[0].outer_wire.edges[3]
            .orientation
            .reversed()
    );
    assert_eq!(
        reversed.faces[0].outer_wire.edges[0].edge.id,
        solid.outer_shell.faces[0].outer_wire.edges[3].edge.id
    );
}

#[test]
fn test_exact_brep_boolean_returns_inner_solid_for_contained_intersection() {
    let tol = Tolerance::default();
    let outer = zenith_algo::PrimitiveBuilder::make_box(10.0, 10.0, 10.0).unwrap();
    let inner = zenith_algo::BrepTransform::translate_solid(
        &zenith_algo::PrimitiveBuilder::make_box(3.0, 3.0, 3.0).unwrap(),
        Vec3::new(2.0, 2.0, 2.0),
    );

    let intersection = zenith_algo::BooleanEngine::boolean_solids_exact(
        &outer,
        &inner,
        zenith_algo::BooleanOpType::Intersection,
        &tol,
    )
    .expect("contained exact intersection should return the inner B-Rep solid");

    assert_eq!(
        intersection.outer_shell.faces.len(),
        inner.outer_shell.faces.len()
    );
    assert!(intersection.is_topologically_valid(&tol));
}

#[test]
fn test_exact_brep_boolean_returns_overlap_box_for_partial_box_intersection() {
    let tol = Tolerance::default();
    let solid_a = zenith_algo::PrimitiveBuilder::make_box(10.0, 10.0, 10.0).unwrap();
    let solid_b = zenith_algo::BrepTransform::translate_solid(
        &zenith_algo::PrimitiveBuilder::make_box(8.0, 8.0, 8.0).unwrap(),
        Vec3::new(4.0, 5.0, 6.0),
    );

    let intersection = zenith_algo::BooleanEngine::boolean_solids_exact(
        &solid_a,
        &solid_b,
        zenith_algo::BooleanOpType::Intersection,
        &tol,
    )
    .expect("partially overlapping boxes should return their overlap B-Rep box");

    assert_eq!(intersection.outer_shell.faces.len(), 6);
    assert!(intersection.inner_shells.is_empty());
    assert!(intersection.is_topologically_valid(&tol));

    let mesh = tessellate_solid(
        &intersection,
        &TessellationParams {
            u_divisions: 4,
            v_divisions: 4,
        },
    );
    let mass = zenith_algo::MassCalculator::compute_from_mesh(&mesh);
    assert!((mass.volume - (6.0 * 5.0 * 4.0)).abs() < 1.0);
}

#[test]
fn test_exact_brep_boolean_returns_merged_box_for_aligned_box_union() {
    let tol = Tolerance::default();
    let solid_a = zenith_algo::PrimitiveBuilder::make_box(6.0, 4.0, 3.0).unwrap();
    let solid_b = zenith_algo::BrepTransform::translate_solid(
        &zenith_algo::PrimitiveBuilder::make_box(6.0, 4.0, 3.0).unwrap(),
        Vec3::new(4.0, 0.0, 0.0),
    );

    let union = zenith_algo::BooleanEngine::boolean_solids_exact(
        &solid_a,
        &solid_b,
        zenith_algo::BooleanOpType::Union,
        &tol,
    )
    .expect("aligned overlapping boxes should merge into one exact B-Rep box");

    assert_eq!(union.outer_shell.faces.len(), 6);
    assert!(union.inner_shells.is_empty());
    assert!(union.is_topologically_valid(&tol));

    let mesh = tessellate_solid(
        &union,
        &TessellationParams {
            u_divisions: 4,
            v_divisions: 4,
        },
    );
    let mass = zenith_algo::MassCalculator::compute_from_mesh(&mesh);
    assert!((mass.volume - (10.0 * 4.0 * 3.0)).abs() < 1.0);
}

#[test]
fn test_exact_brep_boolean_returns_trimmed_box_for_aligned_edge_difference() {
    let tol = Tolerance::default();
    let solid_a = zenith_algo::PrimitiveBuilder::make_box(10.0, 4.0, 3.0).unwrap();
    let solid_b = zenith_algo::BrepTransform::translate_solid(
        &zenith_algo::PrimitiveBuilder::make_box(6.0, 4.0, 3.0).unwrap(),
        Vec3::new(6.0, 0.0, 0.0),
    );

    let difference = zenith_algo::BooleanEngine::boolean_solids_exact(
        &solid_a,
        &solid_b,
        zenith_algo::BooleanOpType::Difference,
        &tol,
    )
    .expect("aligned edge overlap should trim input A into one exact B-Rep box");

    assert_eq!(difference.outer_shell.faces.len(), 6);
    assert!(difference.inner_shells.is_empty());
    assert!(difference.is_topologically_valid(&tol));

    let mesh = tessellate_solid(
        &difference,
        &TessellationParams {
            u_divisions: 4,
            v_divisions: 4,
        },
    );
    let mass = zenith_algo::MassCalculator::compute_from_mesh(&mesh);
    assert!((mass.volume - (6.0 * 4.0 * 3.0)).abs() < 1.0);
}

#[test]
fn test_exact_brep_boolean_returns_orthogonal_l_shape_for_box_union() {
    let tol = Tolerance::default();
    let solid_a = zenith_algo::PrimitiveBuilder::make_box(6.0, 4.0, 3.0).unwrap();
    let solid_b = zenith_algo::BrepTransform::translate_solid(
        &zenith_algo::PrimitiveBuilder::make_box(4.0, 6.0, 3.0).unwrap(),
        Vec3::new(2.0, 2.0, 0.0),
    );

    let union = zenith_algo::BooleanEngine::boolean_solids_exact(
        &solid_a,
        &solid_b,
        zenith_algo::BooleanOpType::Union,
        &tol,
    )
    .expect("overlapping axis-aligned boxes should produce an orthogonal B-Rep union");

    assert!(union.outer_shell.faces.len() > 6);
    assert!(union.inner_shells.is_empty());
    assert!(union.is_topologically_valid(&tol));

    let mesh = tessellate_solid(
        &union,
        &TessellationParams {
            u_divisions: 4,
            v_divisions: 4,
        },
    );
    let mass = zenith_algo::MassCalculator::compute_from_mesh(&mesh);
    assert!((mass.volume - (6.0 * 4.0 * 3.0 + 4.0 * 6.0 * 3.0 - 4.0 * 2.0 * 3.0)).abs() < 1.0);
}

#[test]
fn test_exact_brep_boolean_returns_corner_notch_for_box_difference() {
    let tol = Tolerance::default();
    let solid_a = zenith_algo::PrimitiveBuilder::make_box(10.0, 10.0, 4.0).unwrap();
    let solid_b = zenith_algo::BrepTransform::translate_solid(
        &zenith_algo::PrimitiveBuilder::make_box(6.0, 6.0, 4.0).unwrap(),
        Vec3::new(6.0, 6.0, 0.0),
    );

    let difference = zenith_algo::BooleanEngine::boolean_solids_exact(
        &solid_a,
        &solid_b,
        zenith_algo::BooleanOpType::Difference,
        &tol,
    )
    .expect("corner-overlapping box should cut an orthogonal B-Rep notch");

    assert!(difference.outer_shell.faces.len() > 6);
    assert!(difference.inner_shells.is_empty());
    assert!(difference.is_topologically_valid(&tol));

    let mesh = tessellate_solid(
        &difference,
        &TessellationParams {
            u_divisions: 4,
            v_divisions: 4,
        },
    );
    let mass = zenith_algo::MassCalculator::compute_from_mesh(&mesh);
    assert!((mass.volume - (10.0 * 10.0 * 4.0 - 4.0 * 4.0 * 4.0)).abs() < 1.0);
}

#[test]
fn test_exact_brep_boolean_returns_outer_solid_for_contained_union() {
    let tol = Tolerance::default();
    let outer = zenith_algo::PrimitiveBuilder::make_box(10.0, 10.0, 10.0).unwrap();
    let inner = zenith_algo::BrepTransform::translate_solid(
        &zenith_algo::PrimitiveBuilder::make_box(3.0, 3.0, 3.0).unwrap(),
        Vec3::new(2.0, 2.0, 2.0),
    );

    let union = zenith_algo::BooleanEngine::boolean_solids_exact(
        &outer,
        &inner,
        zenith_algo::BooleanOpType::Union,
        &tol,
    )
    .expect("contained exact union should return the outer B-Rep solid");

    assert_eq!(union.outer_shell.faces.len(), outer.outer_shell.faces.len());
    assert!(union.is_topologically_valid(&tol));
}

#[test]
fn test_exact_brep_boolean_returns_cavity_for_contained_difference() {
    let tol = Tolerance::default();
    let outer = zenith_algo::PrimitiveBuilder::make_box(10.0, 10.0, 10.0).unwrap();
    let inner = zenith_algo::BrepTransform::translate_solid(
        &zenith_algo::PrimitiveBuilder::make_box(3.0, 3.0, 3.0).unwrap(),
        Vec3::new(2.0, 2.0, 2.0),
    );

    let difference = zenith_algo::BooleanEngine::boolean_solids_exact(
        &outer,
        &inner,
        zenith_algo::BooleanOpType::Difference,
        &tol,
    )
    .expect("contained exact difference should return an inner-shell cavity");

    assert_eq!(
        difference.outer_shell.faces.len(),
        outer.outer_shell.faces.len()
    );
    assert_eq!(difference.inner_shells.len(), 1);
    assert_eq!(
        difference.inner_shells[0].faces.len(),
        inner.outer_shell.faces.len()
    );
    assert!(difference.is_topologically_valid(&tol));

    let mesh = tessellate_solid(
        &difference,
        &TessellationParams {
            u_divisions: 4,
            v_divisions: 4,
        },
    );
    let mass = zenith_algo::MassCalculator::compute_from_mesh(&mesh);
    assert!(
        (mass.volume - (10.0 * 10.0 * 10.0 - 3.0 * 3.0 * 3.0)).abs() < 1.0,
        "volume was {}",
        mass.volume
    );
}

#[test]
fn test_step_export_uses_brep_with_voids_for_inner_shells() {
    let tol = Tolerance::default();
    let outer = zenith_algo::PrimitiveBuilder::make_box(10.0, 10.0, 10.0).unwrap();
    let inner = zenith_algo::BrepTransform::translate_solid(
        &zenith_algo::PrimitiveBuilder::make_box(3.0, 3.0, 3.0).unwrap(),
        Vec3::new(2.0, 2.0, 2.0),
    );
    let difference = zenith_algo::BooleanEngine::boolean_solids_exact(
        &outer,
        &inner,
        zenith_algo::BooleanOpType::Difference,
        &tol,
    )
    .expect("contained exact difference should return an inner-shell cavity");

    let step = zenith_io::StepExporter::export_solid_to_string(&difference, "BOX_WITH_VOID");

    assert!(step.contains("BREP_WITH_VOIDS"));
    assert!(step.contains("ORIENTED_CLOSED_SHELL"));
    assert_eq!(step.matches(" = CLOSED_SHELL").count(), 2);
    assert!(!step.contains("MANIFOLD_SOLID_BREP('BOX_WITH_VOID'"));
}

#[test]
fn test_step_roundtrip_preserves_brep_with_voids_inner_shells() {
    let tol = Tolerance::default();
    let outer = zenith_algo::PrimitiveBuilder::make_box(10.0, 10.0, 10.0).unwrap();
    let inner = zenith_algo::BrepTransform::translate_solid(
        &zenith_algo::PrimitiveBuilder::make_box(3.0, 3.0, 3.0).unwrap(),
        Vec3::new(2.0, 2.0, 2.0),
    );
    let difference = zenith_algo::BooleanEngine::boolean_solids_exact(
        &outer,
        &inner,
        zenith_algo::BooleanOpType::Difference,
        &tol,
    )
    .expect("contained exact difference should return an inner-shell cavity");
    let step = zenith_io::StepExporter::export_solid_to_string(&difference, "VOID_ROUNDTRIP");

    let imported = zenith_io::StepImporter::import_solid_from_str(&step)
        .expect("BREP_WITH_VOIDS STEP should import");

    assert_eq!(
        imported.outer_shell.faces.len(),
        difference.outer_shell.faces.len()
    );
    assert_eq!(imported.inner_shells.len(), 1);
    assert_eq!(
        imported.inner_shells[0].faces.len(),
        difference.inner_shells[0].faces.len()
    );
    assert!(imported.is_topologically_valid(&tol));

    let mesh = tessellate_solid(
        &imported,
        &TessellationParams {
            u_divisions: 4,
            v_divisions: 4,
        },
    );
    let mass = zenith_algo::MassCalculator::compute_from_mesh(&mesh);
    assert!(
        (mass.volume - (10.0 * 10.0 * 10.0 - 3.0 * 3.0 * 3.0)).abs() < 1.0,
        "volume was {}",
        mass.volume
    );
}

#[test]
fn test_exact_brep_boolean_returns_left_solid_for_disjoint_difference() {
    let tol = Tolerance::default();
    let solid_a = zenith_algo::PrimitiveBuilder::make_box(10.0, 10.0, 10.0).unwrap();
    let solid_b = zenith_algo::BrepTransform::translate_solid(
        &zenith_algo::PrimitiveBuilder::make_box(2.0, 2.0, 2.0).unwrap(),
        Vec3::new(20.0, 20.0, 20.0),
    );

    let difference = zenith_algo::BooleanEngine::boolean_solids_exact(
        &solid_a,
        &solid_b,
        zenith_algo::BooleanOpType::Difference,
        &tol,
    )
    .expect("disjoint exact difference should return input A");

    assert_eq!(
        difference.outer_shell.faces.len(),
        solid_a.outer_shell.faces.len()
    );
    assert!(difference.is_topologically_valid(&tol));
}

#[test]
fn test_exact_brep_boolean_rejects_empty_disjoint_intersection() {
    let tol = Tolerance::default();
    let solid_a = zenith_algo::PrimitiveBuilder::make_box(10.0, 10.0, 10.0).unwrap();
    let solid_b = zenith_algo::BrepTransform::translate_solid(
        &zenith_algo::PrimitiveBuilder::make_box(2.0, 2.0, 2.0).unwrap(),
        Vec3::new(20.0, 20.0, 20.0),
    );

    let err = zenith_algo::BooleanEngine::boolean_solids_exact(
        &solid_a,
        &solid_b,
        zenith_algo::BooleanOpType::Intersection,
        &tol,
    )
    .expect_err("disjoint exact intersection should not build an empty solid");

    assert!(err.contains("intersection is empty for disjoint solids"));
}

#[test]
fn test_brep_intersection_collects_plane_plane_candidates() {
    let tol = Tolerance::default();
    let solid_a = zenith_algo::PrimitiveBuilder::make_box(10.0, 10.0, 10.0).unwrap();
    let solid_b = zenith_algo::PrimitiveBuilder::make_box(10.0, 10.0, 10.0).unwrap();

    let candidates = zenith_algo::BrepIntersectionBuilder::collect_face_pair_candidates(
        &solid_a.outer_shell.faces,
        &solid_b.outer_shell.faces,
        &tol,
    );

    assert!(candidates.iter().any(|candidate| matches!(
        candidate.kind,
        zenith_algo::FaceIntersectionKind::Line { .. }
    )));
    for candidate in &candidates {
        if let zenith_algo::FaceIntersectionKind::Line {
            point,
            direction,
            segment_start,
            segment_end,
        } = candidate.kind
        {
            assert!(direction.norm() > 0.99);
            assert!((segment_end - segment_start).norm() > tol.linear);
            let FaceGeometry::Plane(plane_a) =
                &solid_a.outer_shell.faces[candidate.face_a_index].geometry
            else {
                continue;
            };
            let FaceGeometry::Plane(plane_b) =
                &solid_b.outer_shell.faces[candidate.face_b_index].geometry
            else {
                continue;
            };
            assert!((point - plane_a.origin).dot(&plane_a.normal).abs() <= tol.linear * 10.0);
            assert!((point - plane_b.origin).dot(&plane_b.normal).abs() <= tol.linear * 10.0);
            assert!(
                (segment_start - plane_a.origin).dot(&plane_a.normal).abs() <= tol.linear * 10.0
            );
            assert!((segment_end - plane_b.origin).dot(&plane_b.normal).abs() <= tol.linear * 10.0);
        }
    }
    assert!(candidates.iter().any(|candidate| {
        candidate.face_a_index == 0
            && candidate.face_b_index == 0
            && matches!(
                candidate.kind,
                zenith_algo::FaceIntersectionKind::Coincident
            )
    }));
}

#[test]
fn test_brep_intersection_broad_phase_skips_disjoint_face_bounds() {
    let tol = Tolerance::default();
    let make_face = |points: [Point3; 4], normal: Vec3| {
        let vertices: Vec<Vertex> = points
            .iter()
            .map(|point| Vertex::from_point(*point))
            .collect();
        let edges = vec![
            Edge::line_between(vertices[0].clone(), vertices[1].clone()).unwrap(),
            Edge::line_between(vertices[1].clone(), vertices[2].clone()).unwrap(),
            Edge::line_between(vertices[2].clone(), vertices[3].clone()).unwrap(),
            Edge::line_between(vertices[3].clone(), vertices[0].clone()).unwrap(),
        ];
        let wire = Wire::new(edges.into_iter().map(OrientedEdge::forward).collect());
        let u_axis = (points[1] - points[0]).normalize();
        let v_axis = normal.cross(&u_axis).normalize();
        let plane = PlaneSurface3::new(points[0], u_axis, v_axis).unwrap();
        Face::simple(FaceGeometry::Plane(plane), wire)
    };

    let face_a = make_face(
        [
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(1.0, 0.0, 0.0),
            Point3::new(1.0, 1.0, 0.0),
            Point3::new(0.0, 1.0, 0.0),
        ],
        Vec3::new(0.0, 0.0, 1.0),
    );
    let face_b = make_face(
        [
            Point3::new(100.0, 0.0, -1.0),
            Point3::new(101.0, 0.0, -1.0),
            Point3::new(101.0, 0.0, 1.0),
            Point3::new(100.0, 0.0, 1.0),
        ],
        Vec3::new(0.0, -1.0, 0.0),
    );

    let candidates = zenith_algo::BrepIntersectionBuilder::collect_face_pair_candidates(
        &[face_a],
        &[face_b],
        &tol,
    );

    assert!(candidates.is_empty());
}

#[test]
fn test_brep_intersection_clips_plane_plane_line_to_face_bbox_overlap() {
    let tol = Tolerance::default();
    let make_face = |points: [Point3; 4], normal: Vec3| {
        let vertices: Vec<Vertex> = points
            .iter()
            .map(|point| Vertex::from_point(*point))
            .collect();
        let edges = vec![
            Edge::line_between(vertices[0].clone(), vertices[1].clone()).unwrap(),
            Edge::line_between(vertices[1].clone(), vertices[2].clone()).unwrap(),
            Edge::line_between(vertices[2].clone(), vertices[3].clone()).unwrap(),
            Edge::line_between(vertices[3].clone(), vertices[0].clone()).unwrap(),
        ];
        let wire = Wire::new(edges.into_iter().map(OrientedEdge::forward).collect());
        let u_axis = (points[1] - points[0]).normalize();
        let v_axis = normal.cross(&u_axis).normalize();
        let plane = PlaneSurface3::new(points[0], u_axis, v_axis).unwrap();
        Face::simple(FaceGeometry::Plane(plane), wire)
    };

    let face_a = make_face(
        [
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(4.0, 0.0, 0.0),
            Point3::new(4.0, 4.0, 0.0),
            Point3::new(0.0, 4.0, 0.0),
        ],
        Vec3::new(0.0, 0.0, 1.0),
    );
    let face_b = make_face(
        [
            Point3::new(2.0, 2.0, -1.0),
            Point3::new(2.0, 2.0, 1.0),
            Point3::new(2.0, 5.0, 1.0),
            Point3::new(2.0, 5.0, -1.0),
        ],
        Vec3::new(-1.0, 0.0, 0.0),
    );

    let candidates = zenith_algo::BrepIntersectionBuilder::collect_face_pair_candidates(
        &[face_a],
        &[face_b],
        &tol,
    );
    let line = candidates
        .iter()
        .find_map(|candidate| match candidate.kind {
            zenith_algo::FaceIntersectionKind::Line {
                segment_start,
                segment_end,
                ..
            } => Some((segment_start, segment_end)),
            _ => None,
        })
        .expect("plane-plane candidate line");

    for point in [line.0, line.1] {
        assert!((point.x - 2.0).abs() <= tol.linear * 10.0);
        assert!(point.y >= 2.0 - tol.linear * 10.0 && point.y <= 4.0 + tol.linear * 10.0);
        assert!(point.z.abs() <= tol.linear * 10.0);
    }
    assert!((line.1 - line.0).norm() > 1.5);
}

#[test]
fn test_brep_intersection_clips_plane_plane_line_to_planar_trim() {
    let tol = Tolerance::default();
    let make_triangle_face = |points: [Point3; 3], normal: Vec3| {
        let vertices: Vec<Vertex> = points
            .iter()
            .map(|point| Vertex::from_point(*point))
            .collect();
        let edges = vec![
            Edge::line_between(vertices[0].clone(), vertices[1].clone()).unwrap(),
            Edge::line_between(vertices[1].clone(), vertices[2].clone()).unwrap(),
            Edge::line_between(vertices[2].clone(), vertices[0].clone()).unwrap(),
        ];
        let wire = Wire::new(edges.into_iter().map(OrientedEdge::forward).collect());
        let u_axis = (points[1] - points[0]).normalize();
        let v_axis = normal.cross(&u_axis).normalize();
        let plane = PlaneSurface3::new(points[0], u_axis, v_axis).unwrap();
        Face::simple(FaceGeometry::Plane(plane), wire)
    };
    let make_quad_face = |points: [Point3; 4], normal: Vec3| {
        let vertices: Vec<Vertex> = points
            .iter()
            .map(|point| Vertex::from_point(*point))
            .collect();
        let edges = vec![
            Edge::line_between(vertices[0].clone(), vertices[1].clone()).unwrap(),
            Edge::line_between(vertices[1].clone(), vertices[2].clone()).unwrap(),
            Edge::line_between(vertices[2].clone(), vertices[3].clone()).unwrap(),
            Edge::line_between(vertices[3].clone(), vertices[0].clone()).unwrap(),
        ];
        let wire = Wire::new(edges.into_iter().map(OrientedEdge::forward).collect());
        let u_axis = (points[1] - points[0]).normalize();
        let v_axis = normal.cross(&u_axis).normalize();
        let plane = PlaneSurface3::new(points[0], u_axis, v_axis).unwrap();
        Face::simple(FaceGeometry::Plane(plane), wire)
    };

    let face_a = make_triangle_face(
        [
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(4.0, 0.0, 0.0),
            Point3::new(0.0, 4.0, 0.0),
        ],
        Vec3::new(0.0, 0.0, 1.0),
    );
    let face_b = make_quad_face(
        [
            Point3::new(3.0, 0.0, -1.0),
            Point3::new(3.0, 0.0, 1.0),
            Point3::new(3.0, 4.0, 1.0),
            Point3::new(3.0, 4.0, -1.0),
        ],
        Vec3::new(-1.0, 0.0, 0.0),
    );

    let candidates = zenith_algo::BrepIntersectionBuilder::collect_face_pair_candidates(
        &[face_a],
        &[face_b],
        &tol,
    );
    let (segment_start, segment_end) = candidates
        .iter()
        .find_map(|candidate| match candidate.kind {
            zenith_algo::FaceIntersectionKind::Line {
                segment_start,
                segment_end,
                ..
            } => Some((segment_start, segment_end)),
            _ => None,
        })
        .expect("trimmed plane-plane candidate");

    let y_min = segment_start.y.min(segment_end.y);
    let y_max = segment_start.y.max(segment_end.y);
    assert!(y_min <= tol.linear * 10.0);
    assert!(y_max <= 1.0 + tol.linear * 10.0);
    assert!((segment_end - segment_start).norm() < 1.1);
}

#[test]
fn test_brep_intersection_promotes_trimmed_lines_to_edge_candidates() {
    let tol = Tolerance::default();
    let solid_a = zenith_algo::PrimitiveBuilder::make_box(10.0, 10.0, 10.0).unwrap();
    let solid_b = zenith_algo::PrimitiveBuilder::make_box(10.0, 10.0, 10.0).unwrap();

    let edges = zenith_algo::BrepIntersectionBuilder::collect_intersection_edge_candidates(
        &solid_a.outer_shell.faces,
        &solid_b.outer_shell.faces,
        &tol,
    );

    assert!(!edges.is_empty());
    for candidate in edges {
        assert!(candidate.edge.tolerance >= tol.linear);
        assert!(
            (candidate.edge.end_vertex.point - candidate.edge.start_vertex.point).norm()
                > tol.linear
        );

        let (u_min, u_max) = candidate.edge.curve.param_range();
        assert!(
            (candidate.edge.evaluate(u_min) - candidate.edge.start_vertex.point).norm()
                <= tol.linear * 10.0
        );
        assert!(
            (candidate.edge.evaluate(u_max) - candidate.edge.end_vertex.point).norm()
                <= tol.linear * 10.0
        );
    }
}

#[test]
fn test_brep_intersection_collects_plane_cylinder_curve_candidate() {
    let tol = Tolerance::default();
    let radius = 10.0;
    let z = 15.0;
    let cylinder = zenith_algo::PrimitiveBuilder::make_cylinder(radius, 30.0).unwrap();
    let side_face = cylinder.outer_shell.faces[0].clone();

    let plane = PlaneSurface3::new(
        Point3::new(-12.0, -12.0, z),
        Vec3::new(24.0, 0.0, 0.0),
        Vec3::new(0.0, 24.0, 0.0),
    )
    .unwrap();
    let points = [
        Point3::new(-12.0, -12.0, z),
        Point3::new(12.0, -12.0, z),
        Point3::new(12.0, 12.0, z),
        Point3::new(-12.0, 12.0, z),
    ];
    let vertices: Vec<Vertex> = points
        .iter()
        .map(|point| Vertex::from_point(*point))
        .collect();
    let edges = vec![
        Edge::line_between(vertices[0].clone(), vertices[1].clone()).unwrap(),
        Edge::line_between(vertices[1].clone(), vertices[2].clone()).unwrap(),
        Edge::line_between(vertices[2].clone(), vertices[3].clone()).unwrap(),
        Edge::line_between(vertices[3].clone(), vertices[0].clone()).unwrap(),
    ];
    let cut_face = Face::simple(
        FaceGeometry::Plane(plane),
        Wire::new(edges.into_iter().map(OrientedEdge::forward).collect()),
    );

    let candidates = zenith_algo::BrepIntersectionBuilder::collect_face_pair_candidates(
        &[cut_face],
        &[side_face],
        &tol,
    );

    let edge = candidates
        .iter()
        .find_map(|candidate| match &candidate.kind {
            zenith_algo::FaceIntersectionKind::Curve { edge } => Some(edge),
            _ => None,
        })
        .expect("plane-cylinder curve intersection candidate");

    assert_eq!(edge.curve.degree, 2);
    assert_eq!(edge.curve.control_points.len(), 3);
    let (t_min, t_max) = edge.curve.param_range();
    for step in 0..=8 {
        let t = t_min + (t_max - t_min) * (step as f64 / 8.0);
        let point = edge.curve.evaluate(t);
        let radial_distance = (point.x * point.x + point.y * point.y).sqrt();
        assert!((point.z - z).abs() < 1e-6);
        assert!((radial_distance - radius).abs() < 1e-6);
    }
}

#[test]
fn test_brep_intersection_promotes_plane_cylinder_curve_to_edge_candidate() {
    let tol = Tolerance::default();
    let radius = 10.0;
    let z = 15.0;
    let cylinder = zenith_algo::PrimitiveBuilder::make_cylinder(radius, 30.0).unwrap();
    let side_face = cylinder.outer_shell.faces[0].clone();

    let cut_plane = PlaneSurface3::new(
        Point3::new(-12.0, -12.0, z),
        Vec3::new(24.0, 0.0, 0.0),
        Vec3::new(0.0, 24.0, 0.0),
    )
    .unwrap();
    let points = [
        Point3::new(-12.0, -12.0, z),
        Point3::new(12.0, -12.0, z),
        Point3::new(12.0, 12.0, z),
        Point3::new(-12.0, 12.0, z),
    ];
    let vertices: Vec<Vertex> = points
        .iter()
        .map(|point| Vertex::from_point(*point))
        .collect();
    let cut_edges = vec![
        Edge::line_between(vertices[0].clone(), vertices[1].clone()).unwrap(),
        Edge::line_between(vertices[1].clone(), vertices[2].clone()).unwrap(),
        Edge::line_between(vertices[2].clone(), vertices[3].clone()).unwrap(),
        Edge::line_between(vertices[3].clone(), vertices[0].clone()).unwrap(),
    ];
    let cut_face = Face::simple(
        FaceGeometry::Plane(cut_plane),
        Wire::new(cut_edges.into_iter().map(OrientedEdge::forward).collect()),
    );

    let edges = zenith_algo::BrepIntersectionBuilder::collect_intersection_edge_candidates(
        &[cut_face],
        &[side_face],
        &tol,
    );

    let edge = edges
        .first()
        .expect("plane-cylinder curve should become an edge candidate");
    assert_eq!(edge.face_a_index, 0);
    assert_eq!(edge.face_b_index, 0);
    assert_eq!(edge.edge.curve.degree, 2);

    let (t_min, t_max) = edge.edge.curve.param_range();
    for step in 0..=8 {
        let t = t_min + (t_max - t_min) * (step as f64 / 8.0);
        let point = edge.edge.curve.evaluate(t);
        let radial_distance = (point.x * point.x + point.y * point.y).sqrt();
        assert!((point.z - z).abs() < 1e-6);
        assert!((radial_distance - radius).abs() < 1e-6);
    }
}

#[test]
fn test_brep_intersection_collects_vertical_plane_cylinder_line_candidate() {
    let tol = Tolerance::default();
    let radius = 10.0;
    let height = 30.0;
    let plane_x: f64 = 6.0;
    let expected_y = (radius * radius - plane_x * plane_x).sqrt();
    let cylinder = zenith_algo::PrimitiveBuilder::make_cylinder(radius, height).unwrap();
    let side_face = cylinder.outer_shell.faces[0].clone();
    let cut_face = vertical_cut_face(plane_x);

    let candidates = zenith_algo::BrepIntersectionBuilder::collect_face_pair_candidates(
        &[cut_face],
        &[side_face],
        &tol,
    );

    let (segment_start, segment_end) = candidates
        .iter()
        .find_map(|candidate| match &candidate.kind {
            zenith_algo::FaceIntersectionKind::Line {
                segment_start,
                segment_end,
                ..
            } => Some((*segment_start, *segment_end)),
            _ => None,
        })
        .expect("vertical plane-cylinder line intersection candidate");

    for point in [segment_start, segment_end] {
        assert!((point.x - plane_x).abs() < 1e-6);
        assert!((point.y - expected_y).abs() < 1e-6);
        let radial_distance = (point.x * point.x + point.y * point.y).sqrt();
        assert!((radial_distance - radius).abs() < 1e-6);
    }

    let z_low = segment_start.z.min(segment_end.z);
    let z_high = segment_start.z.max(segment_end.z);
    assert!(z_low.abs() < 1e-5);
    assert!((z_high - height).abs() < 1e-5);
}

#[test]
fn test_brep_intersection_promotes_vertical_plane_cylinder_line_to_edge_candidate() {
    let tol = Tolerance::default();
    let radius = 10.0;
    let height = 30.0;
    let plane_x: f64 = 6.0;
    let expected_y = (radius * radius - plane_x * plane_x).sqrt();
    let cylinder = zenith_algo::PrimitiveBuilder::make_cylinder(radius, height).unwrap();
    let side_face = cylinder.outer_shell.faces[0].clone();
    let cut_face = vertical_cut_face(plane_x);

    let edges = zenith_algo::BrepIntersectionBuilder::collect_intersection_edge_candidates(
        &[cut_face],
        &[side_face],
        &tol,
    );

    let candidate = edges
        .first()
        .expect("vertical plane-cylinder line should become an edge candidate");
    assert_eq!(candidate.face_a_index, 0);
    assert_eq!(candidate.face_b_index, 0);
    assert_eq!(candidate.edge.curve.degree, 1);

    let (t_min, t_max) = candidate.edge.curve.param_range();
    for step in 0..=8 {
        let t = t_min + (t_max - t_min) * (step as f64 / 8.0);
        let point = candidate.edge.curve.evaluate(t);
        assert!((point.x - plane_x).abs() < 1e-6);
        assert!((point.y - expected_y).abs() < 1e-6);
        assert!(point.z >= -1e-5 && point.z <= height + 1e-5);
    }
}

#[test]
fn test_brep_intersection_skips_vertical_plane_outside_cylinder_radius() {
    let tol = Tolerance::default();
    let cylinder = zenith_algo::PrimitiveBuilder::make_cylinder(10.0, 30.0).unwrap();
    let side_face = cylinder.outer_shell.faces[0].clone();
    let cut_face = vertical_cut_face(14.0);

    let candidates = zenith_algo::BrepIntersectionBuilder::collect_face_pair_candidates(
        &[cut_face],
        &[side_face],
        &tol,
    );

    assert!(
        candidates.is_empty(),
        "a plane beyond the cylinder radius must not produce an intersection candidate"
    );
}

#[test]
fn test_brep_intersection_skips_vertical_plane_missing_cylinder_quadrant() {
    let tol = Tolerance::default();
    let cylinder = zenith_algo::PrimitiveBuilder::make_cylinder(10.0, 30.0).unwrap();
    let opposite_quadrant_face = cylinder.outer_shell.faces[1].clone();
    let cut_face = vertical_cut_face(6.0);

    let candidates = zenith_algo::BrepIntersectionBuilder::collect_face_pair_candidates(
        &[cut_face],
        &[opposite_quadrant_face],
        &tol,
    );

    assert!(
        candidates.is_empty(),
        "the ruling at x = 6 lies outside the second quadrant patch angular span"
    );
}

#[test]
fn test_cone_primitive_has_a_true_apex() {
    let tol = Tolerance::default();
    let radius: f64 = 10.0;
    let height: f64 = 20.0;
    let cone = zenith_algo::PrimitiveBuilder::make_cone(radius, 0.0, height).unwrap();

    // 頂点は1点に縮退し、天面は存在しない
    assert!(cone.is_topologically_valid(&tol));
    assert_eq!(cone.outer_shell.faces.len(), 5);
    let apex = Point3::new(0.0, 0.0, height);
    let side_faces = cone
        .outer_shell
        .faces
        .iter()
        .filter(|face| matches!(face.geometry, FaceGeometry::Nurbs(_)))
        .count();
    assert_eq!(side_faces, 4);
    for face in &cone.outer_shell.faces {
        if !matches!(face.geometry, FaceGeometry::Nurbs(_)) {
            continue;
        }
        // 側面は底面円弧＋稜線2本の3辺で閉じる
        assert_eq!(face.outer_wire.edges.len(), 3);
        assert!(face
            .outer_wire
            .sample_points(4)
            .iter()
            .any(|point| (point - apex).norm() < 1e-9));
    }

    // 母線上の点が解析円錐に乗る
    for face in &cone.outer_shell.faces {
        let FaceGeometry::Nurbs(surface) = &face.geometry else {
            continue;
        };
        for i in 0..=8 {
            for j in 0..=8 {
                let (u, v) = (i as f64 / 8.0, j as f64 / 8.0);
                let point = surface.evaluate(u, v);
                let expected = radius * (1.0 - point.z / height);
                let actual = (point.x * point.x + point.y * point.y).sqrt();
                assert!(
                    (actual - expected).abs() < 1e-9,
                    "cone radius {actual} vs {expected}"
                );
            }
        }
    }

    // 体積・表面積・重心が解析値と一致する（極小天面による誤差がない）
    let params = TessellationParams {
        u_divisions: 32,
        v_divisions: 32,
    };
    let mass = zenith_algo::MassCalculator::compute_from_brep(&cone, &params);
    let slant = (radius * radius + height * height).sqrt();
    let expected_volume = std::f64::consts::PI * radius * radius * height / 3.0;
    let expected_area = std::f64::consts::PI * radius * (slant + radius);
    assert!(
        (mass.volume - expected_volume).abs() < expected_volume * 1e-9,
        "cone volume {} vs analytic {expected_volume}",
        mass.volume
    );
    assert!(
        (mass.surface_area - expected_area).abs() < expected_area * 1e-9,
        "cone area {} vs analytic {expected_area}",
        mass.surface_area
    );
    assert!((mass.center_of_mass.z - height / 4.0).abs() < 1e-9);
}

#[test]
fn test_true_cone_survives_a_step_roundtrip() {
    let tol = Tolerance::default();
    let radius: f64 = 10.0;
    let height: f64 = 20.0;
    let cone = zenith_algo::PrimitiveBuilder::make_cone(radius, 0.0, height).unwrap();

    let step = zenith_io::StepExporter::export_solid_to_string(&cone, "cone");
    let imported = zenith_io::StepImporter::import_solid_from_str(&step)
        .expect("a cone with a true apex should round-trip through STEP");

    assert!(imported.is_topologically_valid(&tol));
    assert_eq!(imported.outer_shell.faces.len(), 5);

    let params = TessellationParams {
        u_divisions: 32,
        v_divisions: 32,
    };
    let mass = zenith_algo::MassCalculator::compute_from_brep(&imported, &params);
    let expected = std::f64::consts::PI * radius * radius * height / 3.0;
    assert!(
        (mass.volume - expected).abs() < expected * 1e-6,
        "imported cone volume {} vs analytic {expected}",
        mass.volume
    );
}

#[test]
fn test_frustum_primitive_stays_analytic() {
    let params = TessellationParams {
        u_divisions: 32,
        v_divisions: 32,
    };
    let (r_bottom, r_top, height) = (10.0_f64, 4.0_f64, 20.0_f64);
    let frustum = zenith_algo::PrimitiveBuilder::make_cone(r_bottom, r_top, height).unwrap();

    let mass = zenith_algo::MassCalculator::compute_from_brep(&frustum, &params);
    let expected =
        std::f64::consts::PI * height * (r_bottom * r_bottom + r_bottom * r_top + r_top * r_top)
            / 3.0;
    assert!(
        (mass.volume - expected).abs() < expected * 1e-9,
        "frustum volume {} vs analytic {expected}",
        mass.volume
    );
}

#[test]
fn test_torus_primitive_stays_analytic() {
    let params = TessellationParams {
        u_divisions: 48,
        v_divisions: 48,
    };
    let (major, minor) = (20.0_f64, 5.0_f64);
    let torus = zenith_algo::PrimitiveBuilder::make_torus(major, minor).unwrap();

    let mass = zenith_algo::MassCalculator::compute_from_brep(&torus, &params);
    let expected_volume = 2.0 * std::f64::consts::PI.powi(2) * major * minor * minor;
    let expected_area = 4.0 * std::f64::consts::PI.powi(2) * major * minor;
    assert!(
        (mass.volume - expected_volume).abs() < expected_volume * 1e-9,
        "torus volume {} vs analytic {expected_volume}",
        mass.volume
    );
    assert!(
        (mass.surface_area - expected_area).abs() < expected_area * 1e-9,
        "torus area {} vs analytic {expected_area}",
        mass.surface_area
    );
}

#[test]
fn test_sphere_primitive_is_an_exact_rational_sphere() {
    let radius: f64 = 15.0;
    let sphere = zenith_algo::PrimitiveBuilder::make_sphere(radius).unwrap();

    // 曲面上の点が解析球から外れないこと（回転体構築の厳密性）
    for face in &sphere.outer_shell.faces {
        let FaceGeometry::Nurbs(surface) = &face.geometry else {
            panic!("sphere face should be a NURBS patch");
        };
        let ((u_min, u_max), (v_min, v_max)) = surface.param_range();
        for i in 0..=16 {
            for j in 0..=16 {
                let u = u_min + (u_max - u_min) * (i as f64 / 16.0);
                let v = v_min + (v_max - v_min) * (j as f64 / 16.0);
                let point = surface.evaluate(u, v);
                assert!(
                    (point.coords.norm() - radius).abs() < 1e-9,
                    "sphere surface drifted to radius {} at ({u}, {v})",
                    point.coords.norm()
                );
            }
        }
    }

    // 厳密積分でも解析値と一致する
    let params = TessellationParams {
        u_divisions: 32,
        v_divisions: 32,
    };
    let mass = zenith_algo::MassCalculator::compute_from_brep(&sphere, &params);
    let expected_volume = 4.0 / 3.0 * std::f64::consts::PI * radius.powi(3);
    let expected_area = 4.0 * std::f64::consts::PI * radius * radius;
    assert!(
        (mass.volume - expected_volume).abs() < expected_volume * 1e-6,
        "sphere volume {} should match analytic {expected_volume}",
        mass.volume
    );
    assert!(
        (mass.surface_area - expected_area).abs() < expected_area * 1e-6,
        "sphere area {} should match analytic {expected_area}",
        mass.surface_area
    );
    for axis in 0..3 {
        assert!(mass.center_of_mass[axis].abs() < 1e-6);
    }
}

#[test]
fn test_revolved_surface_keeps_on_axis_profile_points_exact() {
    let tol = Tolerance::default();
    let radius: f64 = 8.0;

    // 軸上の点を含むプロファイル（円錐の母線）を1回転させる
    let profile = NurbsCurve3::bspline_from_points(
        1,
        vec![Point3::new(0.0, 0.0, 12.0), Point3::new(radius, 0.0, 0.0)],
    )
    .unwrap();
    let surface = zenith_algo::RevolveBuilder::revolve_curve(
        &profile,
        Point3::new(0.0, 0.0, 0.0),
        Vec3::new(0.0, 0.0, 1.0),
        std::f64::consts::TAU,
        &tol,
    )
    .unwrap();

    // 円錐面上では半径が高さに線形比例する
    let ((u_min, u_max), (v_min, v_max)) = surface.param_range();
    for i in 0..=12 {
        for j in 0..=12 {
            let u = u_min + (u_max - u_min) * (i as f64 / 12.0);
            let v = v_min + (v_max - v_min) * (j as f64 / 12.0);
            let point = surface.evaluate(u, v);
            let expected = radius * (1.0 - point.z / 12.0);
            let actual = (point.x * point.x + point.y * point.y).sqrt();
            assert!(
                (actual - expected).abs() < 1e-9,
                "revolved cone radius {actual} should be {expected} at ({u}, {v})"
            );
        }
    }
}

#[test]
fn test_brep_mass_properties_are_analytic_for_a_box() {
    let params = TessellationParams {
        u_divisions: 8,
        v_divisions: 8,
    };
    let solid = zenith_algo::PrimitiveBuilder::make_box(10.0, 10.0, 10.0).unwrap();
    let mass = zenith_algo::MassCalculator::compute_from_brep(&solid, &params);

    assert!(
        (mass.volume - 1000.0).abs() < 1e-9,
        "volume {}",
        mass.volume
    );
    assert!(
        (mass.surface_area - 600.0).abs() < 1e-9,
        "area {}",
        mass.surface_area
    );
    for axis in 0..3 {
        assert!((mass.center_of_mass[axis] - 5.0).abs() < 1e-9);
        // 一辺 a の立方体（角が原点）: Ixx = 2 * a^5 / 3
        assert!((mass.inertia_diagonal[axis] - 2.0 * 10.0_f64.powi(5) / 3.0).abs() < 1e-6);
    }
}

#[test]
fn test_brep_mass_properties_beat_the_mesh_on_a_cylinder() {
    let radius: f64 = 10.0;
    let height: f64 = 30.0;
    let params = TessellationParams {
        u_divisions: 24,
        v_divisions: 24,
    };
    let solid = zenith_algo::PrimitiveBuilder::make_cylinder(radius, height).unwrap();

    let expected_volume = std::f64::consts::PI * radius * radius * height;
    let expected_area = 2.0 * std::f64::consts::PI * radius * (radius + height);
    let expected_izz = 0.5 * expected_volume * radius * radius;

    let brep = zenith_algo::MassCalculator::compute_from_brep(&solid, &params);
    assert!(
        (brep.volume - expected_volume).abs() < expected_volume * 1e-12,
        "brep volume {} should be analytic {expected_volume}",
        brep.volume
    );
    assert!(
        (brep.surface_area - expected_area).abs() < expected_area * 1e-12,
        "brep area {} should be analytic {expected_area}",
        brep.surface_area
    );
    assert!((brep.center_of_mass.z - height / 2.0).abs() < 1e-9);
    assert!(
        (brep.inertia_diagonal.z - expected_izz).abs() < expected_izz * 1e-9,
        "brep Izz {} should be analytic {expected_izz}",
        brep.inertia_diagonal.z
    );

    // メッシュ由来の値は同じ設定でも桁違いに粗い
    let mesh = zenith_algo::MassCalculator::compute_from_mesh(&tessellate_solid(&solid, &params));
    let mesh_error = (mesh.volume - expected_volume).abs();
    let brep_error = (brep.volume - expected_volume).abs();
    assert!(
        mesh_error > brep_error * 1e6,
        "the mesh path should stay clearly coarser: mesh {mesh_error}, brep {brep_error}"
    );
}

#[test]
fn test_brep_mass_properties_subtract_void_shells() {
    let tol = Tolerance::default();
    let params = TessellationParams {
        u_divisions: 8,
        v_divisions: 8,
    };
    let outer = zenith_algo::PrimitiveBuilder::make_box(10.0, 10.0, 10.0).unwrap();
    let inner = zenith_algo::BrepTransform::translate_solid(
        &zenith_algo::PrimitiveBuilder::make_box(3.0, 3.0, 3.0).unwrap(),
        Vec3::new(2.0, 2.0, 2.0),
    );
    let hollow = zenith_algo::BooleanEngine::boolean_solids_exact(
        &outer,
        &inner,
        zenith_algo::BooleanOpType::Difference,
        &tol,
    )
    .expect("contained difference should produce a cavity");

    let mass = zenith_algo::MassCalculator::compute_from_brep(&hollow, &params);
    assert!(
        (mass.volume - (1000.0 - 27.0)).abs() < 1e-9,
        "cavity volume {}",
        mass.volume
    );
    // 表面積は外殻と空洞の両方を数える
    assert!(
        (mass.surface_area - (600.0 + 54.0)).abs() < 1e-9,
        "cavity area {}",
        mass.surface_area
    );
}

#[test]
fn test_brep_mass_properties_integrate_a_boolean_result() {
    let tol = Tolerance::default();
    let radius: f64 = 10.0;
    let height = 30.0;
    let cut_x: f64 = 6.0;
    let params = TessellationParams {
        u_divisions: 32,
        v_divisions: 32,
    };

    let cylinder = zenith_algo::PrimitiveBuilder::make_cylinder(radius, height).unwrap();
    let cutter = zenith_algo::BrepTransform::translate_solid(
        &zenith_algo::PrimitiveBuilder::make_box(20.0, 40.0, 50.0).unwrap(),
        Vec3::new(cut_x, -20.0, -10.0),
    );
    let result = zenith_algo::BooleanEngine::boolean_solids_exact(
        &cylinder,
        &cutter,
        zenith_algo::BooleanOpType::Difference,
        &tol,
    )
    .unwrap();

    let segment_area = radius * radius * (cut_x / radius).acos()
        - cut_x * (radius * radius - cut_x * cut_x).sqrt();
    let expected = (std::f64::consts::PI * radius * radius - segment_area) * height;

    let mass = zenith_algo::MassCalculator::compute_from_brep(&result, &params);
    assert!(
        (mass.volume - expected).abs() < expected * 1e-6,
        "boolean volume {} should match analytic {expected}",
        mass.volume
    );
}

#[test]
fn test_exact_brep_boolean_cuts_cylinder_lengthwise_with_box() {
    let tol = Tolerance::default();
    let radius: f64 = 10.0;
    let height = 30.0;
    let cut_x: f64 = 6.0;
    let cylinder = zenith_algo::PrimitiveBuilder::make_cylinder(radius, height).unwrap();
    let cutter = zenith_algo::BrepTransform::translate_solid(
        &zenith_algo::PrimitiveBuilder::make_box(20.0, 40.0, 50.0).unwrap(),
        Vec3::new(cut_x, -20.0, -10.0),
    );

    let result = zenith_algo::BooleanEngine::boolean_solids_exact_result(
        &cylinder,
        &cutter,
        zenith_algo::BooleanOpType::Difference,
        &tol,
    )
    .expect("cutting a cylinder lengthwise should return an exact solid");
    assert_eq!(result.len(), 1);

    let solid = &result.solids[0];
    assert!(solid.is_topologically_valid(&tol));
    assert_eq!(solid.outer_shell.faces.len(), 7);

    // 側面は NURBS のまま残り、平面はキャップ2枚＋切断面1枚
    let nurbs_faces = solid
        .outer_shell
        .faces
        .iter()
        .filter(|face| matches!(face.geometry, FaceGeometry::Nurbs(_)))
        .count();
    assert_eq!(nurbs_faces, 4);

    // 残った形状は切断平面を越えない
    for face in &solid.outer_shell.faces {
        for point in face.outer_wire.sample_points(8) {
            assert!(
                point.x <= cut_x + 1e-6,
                "material left beyond the cut plane"
            );
        }
    }

    // 体積は円柱から円弓形を除いた解析値と一致する
    let segment_area = radius * radius * (cut_x / radius).acos()
        - cut_x * (radius * radius - cut_x * cut_x).sqrt();
    let expected = (std::f64::consts::PI * radius * radius - segment_area) * height;
    let mesh = tessellate_solid(
        solid,
        &TessellationParams {
            u_divisions: 96,
            v_divisions: 16,
        },
    );
    let mass = zenith_algo::MassCalculator::compute_from_mesh(&mesh);
    assert!(
        (mass.volume - expected).abs() < expected * 5e-3,
        "volume {} should match analytic {expected}",
        mass.volume
    );
}

#[test]
fn test_exact_brep_boolean_cuts_cylinder_with_an_oblique_plane() {
    let tol = Tolerance::default();
    let radius: f64 = 10.0;
    let height = 30.0;
    let cut_z = 15.0;
    let tilt = 20.0_f64.to_radians();

    let cylinder = zenith_algo::PrimitiveBuilder::make_cylinder(radius, height).unwrap();

    // 底面が傾いた平面になるカッターで、円柱の上部を斜めに削ぎ落とす
    let pivot = Vec3::new(0.0, 0.0, cut_z);
    let rotation = zenith_math::Transform3::from_translation(pivot)
        .compose(&zenith_math::Transform3::from_axis_angle(
            &Vec3::new(1.0, 0.0, 0.0),
            tilt,
        ))
        .compose(&zenith_math::Transform3::from_translation(-pivot));
    let cutter = zenith_algo::BrepTransform::transform_solid(
        &zenith_algo::BrepTransform::translate_solid(
            &zenith_algo::PrimitiveBuilder::make_box(60.0, 60.0, 60.0).unwrap(),
            Vec3::new(-30.0, -30.0, cut_z),
        ),
        &rotation,
    )
    .unwrap();

    let result = zenith_algo::BooleanEngine::boolean_solids_exact_result(
        &cylinder,
        &cutter,
        zenith_algo::BooleanOpType::Difference,
        &tol,
    )
    .expect("an oblique cut should return an exact solid");
    assert_eq!(result.len(), 1);

    let solid = &result.solids[0];
    assert!(solid.is_topologically_valid(&tol));
    assert_eq!(solid.outer_shell.faces.len(), 6);
    assert_eq!(
        solid
            .outer_shell
            .faces
            .iter()
            .filter(|face| matches!(face.geometry, FaceGeometry::Nurbs(_)))
            .count(),
        4
    );

    // 切断面より上に材料が残っていないこと（平面は z = cut_z + y*tan(tilt)）
    for face in &solid.outer_shell.faces {
        for point in face.outer_wire.sample_points(8) {
            let plane_z = cut_z + point.y * tilt.tan();
            assert!(
                point.z <= plane_z + 1e-6,
                "material left above the cut plane"
            );
        }
    }

    // 傾いた平面は軸上の cut_z を通るので、残る体積は円柱の下半分ちょうど
    let expected = std::f64::consts::PI * radius * radius * cut_z;
    let mesh = tessellate_solid(
        solid,
        &TessellationParams {
            u_divisions: 96,
            v_divisions: 16,
        },
    );
    let mass = zenith_algo::MassCalculator::compute_from_mesh(&mesh);
    assert!(
        (mass.volume - expected).abs() < expected * 5e-3,
        "volume {} should match analytic {expected}",
        mass.volume
    );
}

#[test]
fn test_exact_brep_boolean_cuts_rotated_cylinder_lengthwise() {
    let tol = Tolerance::default();
    let radius: f64 = 10.0;
    let height = 30.0;
    let cut_x: f64 = 6.0;

    // 円柱軸をZから傾けても、同じ切断が同じ結果を返さなければならない
    let rotation = zenith_math::Transform3::from_axis_angle(
        &Vec3::new(1.0, 0.0, 0.0),
        std::f64::consts::FRAC_PI_2,
    );
    let cylinder = zenith_algo::BrepTransform::transform_solid(
        &zenith_algo::PrimitiveBuilder::make_cylinder(radius, height).unwrap(),
        &rotation,
    )
    .unwrap();
    let cutter = zenith_algo::BrepTransform::transform_solid(
        &zenith_algo::BrepTransform::translate_solid(
            &zenith_algo::PrimitiveBuilder::make_box(20.0, 40.0, 50.0).unwrap(),
            Vec3::new(cut_x, -20.0, -10.0),
        ),
        &rotation,
    )
    .unwrap();

    let result = zenith_algo::BooleanEngine::boolean_solids_exact_result(
        &cylinder,
        &cutter,
        zenith_algo::BooleanOpType::Difference,
        &tol,
    )
    .expect("a rotated cylinder must cut exactly like an axis-aligned one");
    assert_eq!(result.len(), 1);

    let solid = &result.solids[0];
    assert!(solid.is_topologically_valid(&tol));
    assert_eq!(solid.outer_shell.faces.len(), 7);
    assert_eq!(
        solid
            .outer_shell
            .faces
            .iter()
            .filter(|face| matches!(face.geometry, FaceGeometry::Nurbs(_)))
            .count(),
        4
    );

    // X軸まわりの回転なので円柱軸は -Y に倒れ、切断平面 x = cut_x はそのまま
    for face in &solid.outer_shell.faces {
        for point in face.outer_wire.sample_points(8) {
            assert!(
                point.x <= cut_x + 1e-6,
                "material left beyond the cut plane"
            );
        }
        if let FaceGeometry::Nurbs(surface) = &face.geometry {
            let ruling = surface.control_points[0][1].point - surface.control_points[0][0].point;
            assert!(
                ruling.normalize().x.abs() < 1e-9 && ruling.normalize().z.abs() < 1e-9,
                "the recognized patch axis should be along Y after the rotation"
            );
        }
    }

    let segment_area = radius * radius * (cut_x / radius).acos()
        - cut_x * (radius * radius - cut_x * cut_x).sqrt();
    let expected = (std::f64::consts::PI * radius * radius - segment_area) * height;
    let mesh = tessellate_solid(
        solid,
        &TessellationParams {
            u_divisions: 96,
            v_divisions: 16,
        },
    );
    let mass = zenith_algo::MassCalculator::compute_from_mesh(&mesh);
    assert!(
        (mass.volume - expected).abs() < expected * 5e-3,
        "volume {} should match analytic {expected}",
        mass.volume
    );
}

#[test]
fn test_rigid_transform_preserves_brep_and_rejects_scaling() {
    let tol = Tolerance::default();
    let cylinder = zenith_algo::PrimitiveBuilder::make_cylinder(10.0, 30.0).unwrap();

    let rotation = zenith_math::Transform3::from_axis_angle(
        &Vec3::new(0.0, 1.0, 0.0),
        std::f64::consts::FRAC_PI_4,
    );
    let rotated = zenith_algo::BrepTransform::transform_solid(&cylinder, &rotation).unwrap();
    assert!(rotated.is_topologically_valid(&tol));

    // 有理重みが保たれ、側面は傾いた軸まわりの真円柱のまま
    let axis = rotation.transform_vector(&Vec3::new(0.0, 0.0, 1.0));
    let base = rotation.transform_point(&Point3::new(0.0, 0.0, 0.0));
    for face in &rotated.outer_shell.faces {
        let FaceGeometry::Nurbs(surface) = &face.geometry else {
            continue;
        };
        for step in 0..=8 {
            let u = step as f64 / 8.0;
            let point = surface.evaluate(u, 0.5);
            let offset = point - base;
            let radial = (offset - axis * offset.dot(&axis)).norm();
            assert!((radial - 10.0).abs() < 1e-9, "radius drifted to {radial}");
        }
    }

    let scaling = zenith_math::Transform3::from_scale(2.0);
    assert!(zenith_algo::BrepTransform::transform_solid(&cylinder, &scaling).is_err());
}

#[test]
fn test_planar_face_imprint_by_interior_loop() {
    let tol = Tolerance::default();
    let solid = zenith_algo::PrimitiveBuilder::make_box(10.0, 10.0, 10.0).unwrap();
    let face = solid
        .outer_shell
        .faces
        .iter()
        .find(|face| {
            face.outer_wire
                .sample_points(1)
                .iter()
                .all(|point| point.z.abs() < 1e-9)
        })
        .expect("bottom face")
        .clone();

    let corners = [
        Point3::new(3.0, 3.0, 0.0),
        Point3::new(7.0, 3.0, 0.0),
        Point3::new(7.0, 7.0, 0.0),
        Point3::new(3.0, 7.0, 0.0),
    ];
    let loop_edges: Vec<Edge> = (0..4)
        .map(|i| {
            Edge::line_between(
                Vertex::from_point(corners[i]),
                Vertex::from_point(corners[(i + 1) % 4]),
            )
            .unwrap()
        })
        .collect();

    let pieces = zenith_algo::BrepIntersectionBuilder::split_planar_face_by_interior_loop(
        &face,
        &loop_edges,
        &tol,
    )
    .expect("an interior loop should imprint the face");
    assert_eq!(pieces.len(), 2);

    let inner = &pieces[0];
    let outer = &pieces[1];
    assert!(inner.inner_wires.is_empty());
    assert_eq!(outer.inner_wires.len(), 1);

    let params = TessellationParams {
        u_divisions: 8,
        v_divisions: 8,
    };
    let inner_area = triangle_mesh_area(&tessellate_face(inner, &params));
    let outer_area = triangle_mesh_area(&tessellate_face(outer, &params));
    assert!((inner_area - 16.0).abs() < 1e-6, "inner area {inner_area}");
    assert!(
        (outer_area - (100.0 - 16.0)).abs() < 1e-6,
        "outer area {outer_area}"
    );

    // 穴の開いた面の代表点は穴の中に落ちてはならない
    let location = zenith_algo::BrepIntersectionBuilder::classify_face_against_solid(
        outer,
        &zenith_algo::BrepTransform::translate_solid(
            &zenith_algo::PrimitiveBuilder::make_box(4.0, 4.0, 4.0).unwrap(),
            Vec3::new(3.0, 3.0, -2.0),
        ),
        &tol,
    );
    assert_eq!(location, zenith_algo::FaceRegionLocation::Outside);
}

#[test]
fn test_reversed_planar_face_flips_its_volume_contribution() {
    let params = TessellationParams {
        u_divisions: 4,
        v_divisions: 4,
    };
    let solid = zenith_algo::PrimitiveBuilder::make_box(10.0, 10.0, 10.0).unwrap();
    let face = solid.outer_shell.faces[0].clone();

    let forward = signed_mesh_volume(&tessellate_face(&face, &params));
    let reversed_face = Face::new(
        face.geometry.clone(),
        Wire::new(
            face.outer_wire
                .edges
                .iter()
                .rev()
                .map(|edge| OrientedEdge::new(edge.edge.clone(), edge.orientation.reversed()))
                .collect(),
        ),
        Vec::new(),
        face.orientation.reversed(),
        face.tolerance,
    );
    let reversed = signed_mesh_volume(&tessellate_face(&reversed_face, &params));

    assert!(
        (forward + reversed).abs() < 1e-9,
        "reversing a face must flip its divergence contribution: {forward} vs {reversed}"
    );
}

fn signed_mesh_volume(mesh: &zenith_tess::TriangleMesh) -> f64 {
    mesh.indices
        .iter()
        .map(|triangle| {
            let a = mesh.positions[triangle[0] as usize];
            let b = mesh.positions[triangle[1] as usize];
            let c = mesh.positions[triangle[2] as usize];
            a.coords.dot(&b.coords.cross(&c.coords)) / 6.0
        })
        .sum()
}

#[test]
fn test_oblique_plane_cylinder_intersection_is_an_exact_ellipse_arc() {
    let tol = Tolerance::default();
    let radius: f64 = 10.0;
    let cylinder = zenith_algo::PrimitiveBuilder::make_cylinder(radius, 30.0).unwrap();
    let side_face = cylinder.outer_shell.faces[0].clone();
    let (cut_face, plane) = oblique_cut_face(15.0, 20.0_f64.to_radians());

    let candidates = zenith_algo::BrepIntersectionBuilder::collect_face_pair_candidates(
        &[cut_face],
        std::slice::from_ref(&side_face),
        &tol,
    );
    let edge = candidates
        .iter()
        .find_map(|candidate| match &candidate.kind {
            zenith_algo::FaceIntersectionKind::Curve { edge } => Some(edge),
            _ => None,
        })
        .expect("an oblique plane should cut the cylinder in an elliptical arc");

    // 有理2次のまま、円弧と同じ重み構造で楕円弧を厳密表現している
    assert_eq!(edge.curve.degree, 2);
    assert_eq!(edge.curve.control_points.len(), 3);

    let (t_min, t_max) = edge.curve.param_range();
    let mut min_z: f64 = f64::INFINITY;
    let mut max_z: f64 = f64::NEG_INFINITY;
    for step in 0..=32 {
        let t = t_min + (t_max - t_min) * (step as f64 / 32.0);
        let point = edge.curve.evaluate(t);

        // 円柱面上にあること
        let radial = (point.x * point.x + point.y * point.y).sqrt();
        assert!(
            (radial - radius).abs() < 1e-9,
            "ellipse arc left the cylinder: {radial}"
        );

        // 切断平面上にあること
        let offset = (point - plane.origin).dot(&plane.normal);
        assert!(
            offset.abs() < 1e-9,
            "ellipse arc left the cut plane: {offset}"
        );

        min_z = min_z.min(point.z);
        max_z = max_z.max(point.z);
    }

    // 水平断面（円）ではなく本当に傾いていること
    assert!(
        max_z - min_z > 1.0,
        "the section should be tilted, not a horizontal circle"
    );
}

#[test]
fn test_cylinder_side_face_splits_along_an_elliptical_section() {
    let tol = Tolerance::default();
    let radius: f64 = 10.0;
    let height = 30.0;
    let cylinder = zenith_algo::PrimitiveBuilder::make_cylinder(radius, height).unwrap();
    let side_face = cylinder.outer_shell.faces[0].clone();
    let (cut_face, _) = oblique_cut_face(15.0, 20.0_f64.to_radians());

    let candidates = zenith_algo::BrepIntersectionBuilder::collect_intersection_edge_candidates(
        &[cut_face],
        std::slice::from_ref(&side_face),
        &tol,
    );
    let split_edge = candidates
        .first()
        .expect("elliptical edge candidate")
        .edge
        .clone();

    let pieces =
        zenith_algo::BrepIntersectionBuilder::split_face_by_edge(&side_face, &split_edge, &tol)
            .expect("a cylinder patch should split along an elliptical section");
    assert_eq!(pieces.len(), 2);

    for piece in &pieces {
        assert!(piece.outer_wire.is_closed(&tol));
        let report = piece.validate_pcurves(&tol, 8).unwrap();
        assert!(report.is_valid(), "split piece p-curves must stay valid");
        for point in piece.outer_wire.sample_points(8) {
            let radial = (point.x * point.x + point.y * point.y).sqrt();
            assert!((radial - radius).abs() < 1e-6, "boundary left the cylinder");
        }
    }

    // 面積が保存される（元の1/4パッチ）
    let params = TessellationParams {
        u_divisions: 64,
        v_divisions: 16,
    };
    let original_area = triangle_mesh_area(&tessellate_face(&side_face, &params));
    let split_area: f64 = pieces
        .iter()
        .map(|piece| triangle_mesh_area(&tessellate_face(piece, &params)))
        .sum();
    assert!(
        (split_area - original_area).abs() < original_area * 2e-3,
        "split area {split_area} should match original {original_area}"
    );
}

/// Builds a large cutting face on a plane tilted about the X axis by `tilt`,
/// passing through `height` on the cylinder axis.
fn oblique_cut_face(height: f64, tilt: f64) -> (Face, PlaneSurface3) {
    let origin = Point3::new(0.0, 0.0, height);
    let u_axis = Vec3::new(1.0, 0.0, 0.0);
    let v_axis = Vec3::new(0.0, tilt.cos(), -tilt.sin());
    let plane = PlaneSurface3::new(origin, u_axis, v_axis).unwrap();

    let corners = [
        origin - u_axis * 20.0 - v_axis * 20.0,
        origin + u_axis * 20.0 - v_axis * 20.0,
        origin + u_axis * 20.0 + v_axis * 20.0,
        origin - u_axis * 20.0 + v_axis * 20.0,
    ];
    let vertices: Vec<Vertex> = corners
        .iter()
        .map(|point| Vertex::from_point(*point))
        .collect();
    let edges: Vec<OrientedEdge> = (0..4)
        .map(|i| {
            OrientedEdge::forward(
                Edge::line_between(vertices[i].clone(), vertices[(i + 1) % 4].clone()).unwrap(),
            )
        })
        .collect();

    (
        Face::simple(FaceGeometry::Plane(plane), Wire::new(edges)),
        plane,
    )
}

#[test]
fn test_cylinder_side_face_splits_along_vertical_ruling() {
    let tol = Tolerance::default();
    let radius: f64 = 10.0;
    let height = 30.0;
    let plane_x: f64 = 6.0;
    let expected_y = (radius * radius - plane_x * plane_x).sqrt();
    let cylinder = zenith_algo::PrimitiveBuilder::make_cylinder(radius, height).unwrap();
    let side_face = cylinder.outer_shell.faces[0].clone();
    let cut_face = vertical_cut_face(plane_x);

    let candidates = zenith_algo::BrepIntersectionBuilder::collect_intersection_edge_candidates(
        &[cut_face],
        std::slice::from_ref(&side_face),
        &tol,
    );
    let split_edge = candidates
        .first()
        .expect("vertical ruling edge candidate")
        .edge
        .clone();

    let pieces =
        zenith_algo::BrepIntersectionBuilder::split_face_by_edge(&side_face, &split_edge, &tol)
            .expect("cylinder side face should split along a vertical ruling");
    assert_eq!(pieces.len(), 2);

    for piece in &pieces {
        assert!(matches!(piece.geometry, FaceGeometry::Nurbs(_)));
        assert!(piece.outer_wire.is_closed(&tol));
        assert_eq!(piece.outer_wire.edges.len(), 4);
        let report = piece.validate_pcurves(&tol, 8).unwrap();
        assert!(report.is_valid(), "split piece p-curves must stay valid");

        // 分割後も境界は解析円柱面上に乗り続ける
        for point in piece.outer_wire.sample_points(8) {
            let radial = (point.x * point.x + point.y * point.y).sqrt();
            assert!((radial - radius).abs() < 1e-6, "boundary left the cylinder");
            assert!(point.z >= -1e-6 && point.z <= height + 1e-6);
        }
    }

    // 分割線は両ピースが共有し、切断平面上にある
    let shared: Vec<Point3> = pieces[0]
        .outer_wire
        .sample_points(1)
        .into_iter()
        .filter(|point| (point.x - plane_x).abs() < 1e-6)
        .collect();
    assert!(
        shared.len() >= 2,
        "the split piece must carry the ruling at the cut plane"
    );
    for point in shared {
        assert!((point.y - expected_y).abs() < 1e-6);
    }

    // 面積が保存される（もとの1/4パッチ = 半径 * 掃引角 * 高さ）
    let params = TessellationParams {
        u_divisions: 64,
        v_divisions: 8,
    };
    let original_area = triangle_mesh_area(&tessellate_face(&side_face, &params));
    let split_area: f64 = pieces
        .iter()
        .map(|piece| triangle_mesh_area(&tessellate_face(piece, &params)))
        .sum();
    assert!(
        (split_area - original_area).abs() < original_area * 1e-3,
        "split area {split_area} should match original {original_area}"
    );
}

#[test]
fn test_cylinder_side_face_rejects_ruling_split_outside_patch() {
    let tol = Tolerance::default();
    let cylinder = zenith_algo::PrimitiveBuilder::make_cylinder(10.0, 30.0).unwrap();
    let side_face = cylinder.outer_shell.faces[0].clone();

    // 第4象限のルーリングは第1象限パッチを横切らない
    let outside_ruling = Edge::line_between(
        Vertex::from_point(Point3::new(6.0, -8.0, 0.0)),
        Vertex::from_point(Point3::new(6.0, -8.0, 30.0)),
    )
    .unwrap();
    assert!(zenith_algo::BrepIntersectionBuilder::split_face_by_edge(
        &side_face,
        &outside_ruling,
        &tol
    )
    .is_err());

    // パッチ高さの一部しか覆わないルーリングはまだ対象外
    let partial_ruling = Edge::line_between(
        Vertex::from_point(Point3::new(6.0, 8.0, 5.0)),
        Vertex::from_point(Point3::new(6.0, 8.0, 20.0)),
    )
    .unwrap();
    assert!(zenith_algo::BrepIntersectionBuilder::split_face_by_edge(
        &side_face,
        &partial_ruling,
        &tol
    )
    .is_err());

    // パッチ境界そのものに一致するルーリングは内部を横切らない
    let boundary_ruling = Edge::line_between(
        Vertex::from_point(Point3::new(10.0, 0.0, 0.0)),
        Vertex::from_point(Point3::new(10.0, 0.0, 30.0)),
    )
    .unwrap();
    assert!(zenith_algo::BrepIntersectionBuilder::split_face_by_edge(
        &side_face,
        &boundary_ruling,
        &tol
    )
    .is_err());
}

#[test]
fn test_horizontally_split_cylinder_side_faces_tessellate_their_own_band() {
    let tol = Tolerance::default();
    let radius = 10.0;
    let height = 30.0;
    let split_z = 12.0;
    let cylinder = zenith_algo::PrimitiveBuilder::make_cylinder(radius, height).unwrap();
    let side_face = cylinder.outer_shell.faces[0].clone();
    let FaceGeometry::Nurbs(surface) = &side_face.geometry else {
        panic!("cylinder side face should be a NURBS patch");
    };

    let section = zenith_geom::NurbsCurve3::new(
        surface.degree_u,
        surface
            .control_points
            .iter()
            .map(|row| {
                let alpha = split_z / height;
                let bottom = row[0].to_homogeneous();
                let top = row[1].to_homogeneous();
                zenith_geom::ControlPoint3::from_homogeneous(
                    &(bottom * (1.0 - alpha) + top * alpha),
                )
            })
            .collect(),
        KnotVector::new(surface.knots_u.knots.clone()),
    )
    .unwrap();
    let (t_min, t_max) = section.param_range();
    let split_edge = Edge::new(
        section.clone(),
        Vertex::from_point(section.evaluate(t_min)),
        Vertex::from_point(section.evaluate(t_max)),
        tol.linear,
    );

    let pieces =
        zenith_algo::BrepIntersectionBuilder::split_face_by_edge(&side_face, &split_edge, &tol)
            .expect("cylinder side face should split along a horizontal arc");
    assert_eq!(pieces.len(), 2);

    // 分割された帯はそれぞれ自分の高さ範囲だけをテッセレートする
    let params = TessellationParams {
        u_divisions: 64,
        v_divisions: 16,
    };
    let original_area = triangle_mesh_area(&tessellate_face(&side_face, &params));
    let mut piece_areas = Vec::new();
    for piece in &pieces {
        let mesh = tessellate_face(piece, &params);
        assert!(mesh.num_triangles() > 0);
        let z_low = mesh
            .positions
            .iter()
            .fold(f64::INFINITY, |acc, p| acc.min(p.z));
        let z_high = mesh
            .positions
            .iter()
            .fold(f64::NEG_INFINITY, |acc, p| acc.max(p.z));
        assert!(
            z_high - z_low < height - 1e-6,
            "piece still spans the full patch"
        );
        piece_areas.push(triangle_mesh_area(&mesh));
    }

    let split_area: f64 = piece_areas.iter().sum();
    assert!(
        (split_area - original_area).abs() < original_area * 1e-3,
        "split area {split_area} should match original {original_area}"
    );
}

#[test]
fn test_trim_clipping_lands_exactly_on_circular_face_boundary() {
    let tol = Tolerance::default();
    let radius: f64 = 10.0;
    let height = 30.0;
    let plane_x: f64 = 6.0;
    let expected_y = (radius * radius - plane_x * plane_x).sqrt();
    let cylinder = zenith_algo::PrimitiveBuilder::make_cylinder(radius, height).unwrap();
    let cut_face = vertical_cut_face(plane_x);

    let candidates = zenith_algo::BrepIntersectionBuilder::collect_face_pair_candidates(
        &[cut_face],
        &cylinder.outer_shell.faces,
        &tol,
    );

    // 円形キャップ面を横切る弦は、サンプリング折れ線ではなく厳密な円弧上で止まる
    let mut chord_count = 0;
    for candidate in &candidates {
        let zenith_algo::FaceIntersectionKind::Line {
            segment_start,
            segment_end,
            ..
        } = &candidate.kind
        else {
            continue;
        };
        if !matches!(
            cylinder.outer_shell.faces[candidate.face_b_index].geometry,
            FaceGeometry::Plane(_)
        ) {
            continue;
        }

        chord_count += 1;
        for point in [segment_start, segment_end] {
            let radial = (point.x * point.x + point.y * point.y).sqrt();
            assert!(
                (radial - radius).abs() < 1e-9,
                "chord endpoint {radial} should sit on the exact circle, not a sampled chord"
            );
            assert!((point.y.abs() - expected_y).abs() < 1e-9);
        }
    }
    assert_eq!(chord_count, 2, "both cylinder caps should yield a chord");

    // 側面のルーリングは面のZ範囲を越えてはみ出さない
    for candidate in &candidates {
        let zenith_algo::FaceIntersectionKind::Line {
            segment_start,
            segment_end,
            ..
        } = &candidate.kind
        else {
            continue;
        };
        for point in [segment_start, segment_end] {
            assert!(point.z >= -1e-9 && point.z <= height + 1e-9);
        }
    }
}

fn triangle_mesh_area(mesh: &zenith_tess::TriangleMesh) -> f64 {
    mesh.indices
        .iter()
        .map(|triangle| {
            let a = mesh.positions[triangle[0] as usize];
            let b = mesh.positions[triangle[1] as usize];
            let c = mesh.positions[triangle[2] as usize];
            (b - a).cross(&(c - a)).norm() * 0.5
        })
        .sum()
}

/// Builds an axis-parallel vertical cutting face at `plane_x`, spanning the full
/// height and diameter of the 10 x 30 test cylinder.
fn vertical_cut_face(plane_x: f64) -> Face {
    let plane = PlaneSurface3::new(
        Point3::new(plane_x, -12.0, -2.0),
        Vec3::new(0.0, 24.0, 0.0),
        Vec3::new(0.0, 0.0, 34.0),
    )
    .unwrap();
    let points = [
        Point3::new(plane_x, -12.0, -2.0),
        Point3::new(plane_x, 12.0, -2.0),
        Point3::new(plane_x, 12.0, 32.0),
        Point3::new(plane_x, -12.0, 32.0),
    ];
    let vertices: Vec<Vertex> = points
        .iter()
        .map(|point| Vertex::from_point(*point))
        .collect();
    let edges = vec![
        Edge::line_between(vertices[0].clone(), vertices[1].clone()).unwrap(),
        Edge::line_between(vertices[1].clone(), vertices[2].clone()).unwrap(),
        Edge::line_between(vertices[2].clone(), vertices[3].clone()).unwrap(),
        Edge::line_between(vertices[3].clone(), vertices[0].clone()).unwrap(),
    ];

    Face::simple(
        FaceGeometry::Plane(plane),
        Wire::new(edges.into_iter().map(OrientedEdge::forward).collect()),
    )
}

#[test]
fn test_planar_split_reports_curved_plane_cylinder_edge_as_skipped() {
    let tol = Tolerance::default();
    let z = 15.0;
    let cylinder = zenith_algo::PrimitiveBuilder::make_cylinder(10.0, 30.0).unwrap();
    let side_face = cylinder.outer_shell.faces[0].clone();

    let cut_plane = PlaneSurface3::new(
        Point3::new(-12.0, -12.0, z),
        Vec3::new(24.0, 0.0, 0.0),
        Vec3::new(0.0, 24.0, 0.0),
    )
    .unwrap();
    let points = [
        Point3::new(-12.0, -12.0, z),
        Point3::new(12.0, -12.0, z),
        Point3::new(12.0, 12.0, z),
        Point3::new(-12.0, 12.0, z),
    ];
    let vertices: Vec<Vertex> = points
        .iter()
        .map(|point| Vertex::from_point(*point))
        .collect();
    let cut_edges = vec![
        Edge::line_between(vertices[0].clone(), vertices[1].clone()).unwrap(),
        Edge::line_between(vertices[1].clone(), vertices[2].clone()).unwrap(),
        Edge::line_between(vertices[2].clone(), vertices[3].clone()).unwrap(),
        Edge::line_between(vertices[3].clone(), vertices[0].clone()).unwrap(),
    ];
    let cut_face = Face::simple(
        FaceGeometry::Plane(cut_plane),
        Wire::new(cut_edges.into_iter().map(OrientedEdge::forward).collect()),
    );
    let edge_candidate =
        zenith_algo::BrepIntersectionBuilder::collect_intersection_edge_candidates(
            &[cut_face.clone()],
            &[side_face],
            &tol,
        )
        .into_iter()
        .next()
        .expect("plane-cylinder curve should become an edge candidate");

    let result = zenith_algo::BrepIntersectionBuilder::split_planar_face_by_edges(
        &cut_face,
        &[edge_candidate.edge],
        &tol,
    )
    .expect("planar multi-split report");

    assert_eq!(result.applied_split_count, 0);
    assert_eq!(result.skipped_split_count, 1);
    assert_eq!(result.faces.len(), 1);
}

#[test]
fn test_planar_split_can_preserve_curved_split_edge() {
    let tol = Tolerance::default();
    let radius = 10.0;
    let z = 15.0;
    let cylinder = zenith_algo::PrimitiveBuilder::make_cylinder(radius, 30.0).unwrap();
    let side_face = cylinder.outer_shell.faces[0].clone();

    let cut_plane = PlaneSurface3::new(
        Point3::new(10.0, 0.0, z),
        Vec3::new(1.0, 0.0, 0.0),
        Vec3::new(0.0, 1.0, 0.0),
    )
    .unwrap();
    let points = [
        Point3::new(10.0, 0.0, z),
        Point3::new(12.0, 12.0, z),
        Point3::new(0.0, 10.0, z),
    ];
    let vertices: Vec<Vertex> = points
        .iter()
        .map(|point| Vertex::from_point(*point))
        .collect();
    let boundary_edges = vec![
        Edge::line_between(vertices[0].clone(), vertices[1].clone()).unwrap(),
        Edge::line_between(vertices[1].clone(), vertices[2].clone()).unwrap(),
        Edge::line_between(vertices[2].clone(), vertices[0].clone()).unwrap(),
    ];
    let cut_face = Face::simple(
        FaceGeometry::Plane(cut_plane),
        Wire::new(
            boundary_edges
                .into_iter()
                .map(OrientedEdge::forward)
                .collect(),
        ),
    );
    let curved_edge = zenith_algo::BrepIntersectionBuilder::collect_intersection_edge_candidates(
        &[cut_face.clone()],
        &[side_face],
        &tol,
    )
    .into_iter()
    .next()
    .expect("plane-cylinder curve should become an edge candidate")
    .edge;

    let split_faces = zenith_algo::BrepIntersectionBuilder::split_planar_face_by_edge(
        &cut_face,
        &curved_edge,
        &tol,
    )
    .expect("curved planar face split");

    assert_eq!(split_faces.len(), 2);
    for face in split_faces {
        assert!(face.outer_wire.is_closed(&tol));
        assert!(face.outer_wire.edges.iter().any(|edge| {
            edge.edge.id == curved_edge.id && edge.edge.curve.degree == curved_edge.curve.degree
        }));
        assert!(face.pcurves.is_some());
        assert!(face.validate_pcurves(&tol, 8).unwrap().is_valid());
    }
}

#[test]
fn test_batch_split_applies_plane_cylinder_curved_edge_to_planar_operand() {
    let tol = Tolerance::default();
    let z = 15.0;
    let cylinder = zenith_algo::PrimitiveBuilder::make_cylinder(10.0, 30.0).unwrap();
    let side_face = cylinder.outer_shell.faces[0].clone();

    let cut_plane = PlaneSurface3::new(
        Point3::new(10.0, 0.0, z),
        Vec3::new(1.0, 0.0, 0.0),
        Vec3::new(0.0, 1.0, 0.0),
    )
    .unwrap();
    let points = [
        Point3::new(10.0, 0.0, z),
        Point3::new(12.0, 12.0, z),
        Point3::new(0.0, 10.0, z),
    ];
    let vertices: Vec<Vertex> = points
        .iter()
        .map(|point| Vertex::from_point(*point))
        .collect();
    let boundary_edges = vec![
        Edge::line_between(vertices[0].clone(), vertices[1].clone()).unwrap(),
        Edge::line_between(vertices[1].clone(), vertices[2].clone()).unwrap(),
        Edge::line_between(vertices[2].clone(), vertices[0].clone()).unwrap(),
    ];
    let cut_face = Face::simple(
        FaceGeometry::Plane(cut_plane),
        Wire::new(
            boundary_edges
                .into_iter()
                .map(OrientedEdge::forward)
                .collect(),
        ),
    );

    let batches = zenith_algo::BrepIntersectionBuilder::collect_planar_face_batch_splits(
        &[cut_face],
        &[side_face],
        &tol,
    );

    assert_eq!(batches.splits_a.len(), 1);
    assert_eq!(batches.splits_b.len(), 1);
    assert_eq!(batches.splits_a[0].split_edge_count, 1);
    assert_eq!(batches.splits_a[0].result.applied_split_count, 1);
    assert_eq!(batches.splits_a[0].result.skipped_split_count, 0);
    assert_eq!(batches.splits_a[0].result.faces.len(), 2);
    for face in &batches.splits_a[0].result.faces {
        assert!(face.outer_wire.is_closed(&tol));
        assert!(face
            .outer_wire
            .edges
            .iter()
            .any(|edge| edge.edge.curve.degree == 2));
        assert!(face.validate_pcurves(&tol, 8).unwrap().is_valid());
    }
    assert_eq!(batches.splits_b[0].split_edge_count, 1);
    assert_eq!(batches.splits_b[0].result.applied_split_count, 1);
    assert_eq!(batches.splits_b[0].result.skipped_split_count, 0);
    assert_eq!(batches.splits_b[0].result.faces.len(), 2);
    for face in &batches.splits_b[0].result.faces {
        assert!(matches!(face.geometry, FaceGeometry::Nurbs(_)));
        assert!(face.outer_wire.is_closed(&tol));
        assert!(face.validate_pcurves(&tol, 8).unwrap().is_valid());
    }
}

#[test]
fn test_curved_planar_split_tessellates_arc_boundary() {
    let tol = Tolerance::default();
    let z = 15.0;
    let cylinder = zenith_algo::PrimitiveBuilder::make_cylinder(10.0, 30.0).unwrap();
    let side_face = cylinder.outer_shell.faces[0].clone();

    let cut_plane = PlaneSurface3::new(
        Point3::new(10.0, 0.0, z),
        Vec3::new(1.0, 0.0, 0.0),
        Vec3::new(0.0, 1.0, 0.0),
    )
    .unwrap();
    let points = [
        Point3::new(10.0, 0.0, z),
        Point3::new(12.0, 12.0, z),
        Point3::new(0.0, 10.0, z),
    ];
    let vertices: Vec<Vertex> = points
        .iter()
        .map(|point| Vertex::from_point(*point))
        .collect();
    let boundary_edges = vec![
        Edge::line_between(vertices[0].clone(), vertices[1].clone()).unwrap(),
        Edge::line_between(vertices[1].clone(), vertices[2].clone()).unwrap(),
        Edge::line_between(vertices[2].clone(), vertices[0].clone()).unwrap(),
    ];
    let cut_face = Face::simple(
        FaceGeometry::Plane(cut_plane),
        Wire::new(
            boundary_edges
                .into_iter()
                .map(OrientedEdge::forward)
                .collect(),
        ),
    );
    let curved_edge = zenith_algo::BrepIntersectionBuilder::collect_intersection_edge_candidates(
        &[cut_face.clone()],
        &[side_face],
        &tol,
    )
    .into_iter()
    .next()
    .expect("plane-cylinder curve should become an edge candidate")
    .edge;
    let split_faces = zenith_algo::BrepIntersectionBuilder::split_planar_face_by_edge(
        &cut_face,
        &curved_edge,
        &tol,
    )
    .expect("curved planar face split");

    let params = TessellationParams {
        u_divisions: 32,
        v_divisions: 8,
    };
    let arc_bounded_face = split_faces
        .iter()
        .find(|face| face.outer_wire.edges.len() == 2)
        .expect("arc plus chord split face");
    let mesh = tessellate_face(arc_bounded_face, &params);

    assert!(mesh.positions.len() > 4);
    assert!(mesh.num_triangles() > 1);
    assert!(mesh.positions.iter().any(|point| {
        let radial_distance = (point.x * point.x + point.y * point.y).sqrt();
        (radial_distance - 10.0).abs() < 1e-5
            && point.x > 1.0
            && point.y > 1.0
            && point.z > z - 1e-6
            && point.z < z + 1e-6
    }));
}

#[test]
fn test_nurbs_cylinder_side_split_by_horizontal_arc_edge() {
    let tol = Tolerance::default();
    let z = 15.0;
    let cylinder = zenith_algo::PrimitiveBuilder::make_cylinder(10.0, 30.0).unwrap();
    let side_face = cylinder.outer_shell.faces[0].clone();

    let cut_plane = PlaneSurface3::new(
        Point3::new(10.0, 0.0, z),
        Vec3::new(1.0, 0.0, 0.0),
        Vec3::new(0.0, 1.0, 0.0),
    )
    .unwrap();
    let points = [
        Point3::new(10.0, 0.0, z),
        Point3::new(12.0, 12.0, z),
        Point3::new(0.0, 10.0, z),
    ];
    let vertices: Vec<Vertex> = points
        .iter()
        .map(|point| Vertex::from_point(*point))
        .collect();
    let boundary_edges = vec![
        Edge::line_between(vertices[0].clone(), vertices[1].clone()).unwrap(),
        Edge::line_between(vertices[1].clone(), vertices[2].clone()).unwrap(),
        Edge::line_between(vertices[2].clone(), vertices[0].clone()).unwrap(),
    ];
    let cut_face = Face::simple(
        FaceGeometry::Plane(cut_plane),
        Wire::new(
            boundary_edges
                .into_iter()
                .map(OrientedEdge::forward)
                .collect(),
        ),
    );
    let curved_edge = zenith_algo::BrepIntersectionBuilder::collect_intersection_edge_candidates(
        &[cut_face],
        &[side_face.clone()],
        &tol,
    )
    .into_iter()
    .next()
    .expect("plane-cylinder curve should become an edge candidate")
    .edge;

    let split_faces =
        zenith_algo::BrepIntersectionBuilder::split_face_by_edge(&side_face, &curved_edge, &tol)
            .expect("cylinder side split");

    assert_eq!(split_faces.len(), 2);
    for face in split_faces {
        assert!(matches!(face.geometry, FaceGeometry::Nurbs(_)));
        assert!(face.outer_wire.is_closed(&tol));
        assert!(face.outer_wire.edges.iter().any(|edge| {
            edge.edge.id == curved_edge.id && edge.edge.curve.degree == curved_edge.curve.degree
        }));
        assert!(face.pcurves.is_some());
        assert!(face.validate_pcurves(&tol, 8).unwrap().is_valid());
    }
}

#[test]
fn test_plane_cylinder_curve_split_candidate_splits_both_faces() {
    let tol = Tolerance::default();
    let z = 15.0;
    let cylinder = zenith_algo::PrimitiveBuilder::make_cylinder(10.0, 30.0).unwrap();
    let side_face = cylinder.outer_shell.faces[0].clone();

    let cut_plane = PlaneSurface3::new(
        Point3::new(10.0, 0.0, z),
        Vec3::new(1.0, 0.0, 0.0),
        Vec3::new(0.0, 1.0, 0.0),
    )
    .unwrap();
    let points = [
        Point3::new(10.0, 0.0, z),
        Point3::new(12.0, 12.0, z),
        Point3::new(0.0, 10.0, z),
    ];
    let vertices: Vec<Vertex> = points
        .iter()
        .map(|point| Vertex::from_point(*point))
        .collect();
    let boundary_edges = vec![
        Edge::line_between(vertices[0].clone(), vertices[1].clone()).unwrap(),
        Edge::line_between(vertices[1].clone(), vertices[2].clone()).unwrap(),
        Edge::line_between(vertices[2].clone(), vertices[0].clone()).unwrap(),
    ];
    let cut_face = Face::simple(
        FaceGeometry::Plane(cut_plane),
        Wire::new(
            boundary_edges
                .into_iter()
                .map(OrientedEdge::forward)
                .collect(),
        ),
    );

    let splits = zenith_algo::BrepIntersectionBuilder::collect_planar_face_split_candidates(
        &[cut_face],
        &[side_face],
        &tol,
    );

    assert_eq!(splits.len(), 1);
    assert_eq!(splits[0].split_edge.curve.degree, 2);
    assert_eq!(splits[0].split_faces_a.len(), 2);
    assert_eq!(splits[0].split_faces_b.len(), 2);
    assert!(splits[0]
        .split_faces_a
        .iter()
        .all(|face| matches!(face.geometry, FaceGeometry::Plane(_))));
    assert!(splits[0]
        .split_faces_b
        .iter()
        .all(|face| matches!(face.geometry, FaceGeometry::Nurbs(_))));
}

#[test]
fn test_brep_intersection_edge_splits_planar_face() {
    let tol = Tolerance::default();
    let make_face = |points: [Point3; 4], normal: Vec3| {
        let vertices: Vec<Vertex> = points
            .iter()
            .map(|point| Vertex::from_point(*point))
            .collect();
        let edges = vec![
            Edge::line_between(vertices[0].clone(), vertices[1].clone()).unwrap(),
            Edge::line_between(vertices[1].clone(), vertices[2].clone()).unwrap(),
            Edge::line_between(vertices[2].clone(), vertices[3].clone()).unwrap(),
            Edge::line_between(vertices[3].clone(), vertices[0].clone()).unwrap(),
        ];
        let wire = Wire::new(edges.into_iter().map(OrientedEdge::forward).collect());
        let u_axis = (points[1] - points[0]).normalize();
        let v_axis = normal.cross(&u_axis).normalize();
        let plane = PlaneSurface3::new(points[0], u_axis, v_axis).unwrap();
        Face::simple(FaceGeometry::Plane(plane), wire)
    };
    let face_a = make_face(
        [
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(4.0, 0.0, 0.0),
            Point3::new(4.0, 4.0, 0.0),
            Point3::new(0.0, 4.0, 0.0),
        ],
        Vec3::new(0.0, 0.0, 1.0),
    );
    let face_b = make_face(
        [
            Point3::new(2.0, 0.0, -1.0),
            Point3::new(2.0, 0.0, 1.0),
            Point3::new(2.0, 4.0, 1.0),
            Point3::new(2.0, 4.0, -1.0),
        ],
        Vec3::new(-1.0, 0.0, 0.0),
    );

    let split = zenith_algo::BrepIntersectionBuilder::collect_intersection_edge_candidates(
        &[face_a.clone()],
        &[face_b],
        &tol,
    )
    .into_iter()
    .next()
    .expect("an intersection edge crossing the planar face interior");

    let split_faces =
        zenith_algo::BrepIntersectionBuilder::split_planar_face_by_edge(&face_a, &split.edge, &tol)
            .expect("planar face split");

    assert_eq!(split_faces.len(), 2);
    for face in split_faces {
        assert!(matches!(face.geometry, FaceGeometry::Plane(_)));
        assert!(face.inner_wires.is_empty());
        assert!(face.outer_wire.is_closed(&tol));
        assert!(face.pcurves.is_some());
        assert!(face.validate_pcurves(&tol, 4).unwrap().is_valid());
        assert!(face.outer_wire.edges.iter().any(|edge| {
            let start = edge.start_vertex().point;
            let end = edge.end_vertex().point;
            ((start - split.edge.start_vertex.point).norm() <= tol.linear * 10.0
                && (end - split.edge.end_vertex.point).norm() <= tol.linear * 10.0)
                || ((start - split.edge.end_vertex.point).norm() <= tol.linear * 10.0
                    && (end - split.edge.start_vertex.point).norm() <= tol.linear * 10.0)
        }));
    }
}

#[test]
fn test_brep_intersection_splits_planar_face_by_multiple_edges() {
    let tol = Tolerance::default();
    let points = [
        Point3::new(0.0, 0.0, 0.0),
        Point3::new(4.0, 0.0, 0.0),
        Point3::new(4.0, 4.0, 0.0),
        Point3::new(0.0, 4.0, 0.0),
    ];
    let vertices: Vec<Vertex> = points
        .iter()
        .map(|point| Vertex::from_point(*point))
        .collect();
    let edges = vec![
        Edge::line_between(vertices[0].clone(), vertices[1].clone()).unwrap(),
        Edge::line_between(vertices[1].clone(), vertices[2].clone()).unwrap(),
        Edge::line_between(vertices[2].clone(), vertices[3].clone()).unwrap(),
        Edge::line_between(vertices[3].clone(), vertices[0].clone()).unwrap(),
    ];
    let wire = Wire::new(edges.into_iter().map(OrientedEdge::forward).collect());
    let plane = PlaneSurface3::new(
        points[0],
        Vec3::new(1.0, 0.0, 0.0),
        Vec3::new(0.0, 1.0, 0.0),
    )
    .unwrap();
    let face = Face::simple(FaceGeometry::Plane(plane), wire);
    let split_edges = vec![
        Edge::line_between(
            Vertex::new(Point3::new(1.0, 0.0, 0.0), tol.linear),
            Vertex::new(Point3::new(1.0, 4.0, 0.0), tol.linear),
        )
        .unwrap(),
        Edge::line_between(
            Vertex::new(Point3::new(3.0, 0.0, 0.0), tol.linear),
            Vertex::new(Point3::new(3.0, 4.0, 0.0), tol.linear),
        )
        .unwrap(),
    ];

    let result =
        zenith_algo::BrepIntersectionBuilder::split_planar_face_by_edges(&face, &split_edges, &tol)
            .expect("multi split planar face");

    assert_eq!(result.applied_split_count, 2);
    assert_eq!(result.skipped_split_count, 0);
    assert_eq!(result.faces.len(), 3);
    for split_face in result.faces {
        assert!(split_face.outer_wire.is_closed(&tol));
        assert!(split_face.pcurves.is_some());
        assert!(split_face.validate_pcurves(&tol, 4).unwrap().is_valid());
    }
}

#[test]
fn test_brep_intersection_collects_batch_splits_by_face() {
    let tol = Tolerance::default();
    let make_face = |points: [Point3; 4], normal: Vec3| {
        let vertices: Vec<Vertex> = points
            .iter()
            .map(|point| Vertex::from_point(*point))
            .collect();
        let edges = vec![
            Edge::line_between(vertices[0].clone(), vertices[1].clone()).unwrap(),
            Edge::line_between(vertices[1].clone(), vertices[2].clone()).unwrap(),
            Edge::line_between(vertices[2].clone(), vertices[3].clone()).unwrap(),
            Edge::line_between(vertices[3].clone(), vertices[0].clone()).unwrap(),
        ];
        let wire = Wire::new(edges.into_iter().map(OrientedEdge::forward).collect());
        let u_axis = (points[1] - points[0]).normalize();
        let v_axis = normal.cross(&u_axis).normalize();
        let plane = PlaneSurface3::new(points[0], u_axis, v_axis).unwrap();
        Face::simple(FaceGeometry::Plane(plane), wire)
    };
    let face_a = make_face(
        [
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(4.0, 0.0, 0.0),
            Point3::new(4.0, 4.0, 0.0),
            Point3::new(0.0, 4.0, 0.0),
        ],
        Vec3::new(0.0, 0.0, 1.0),
    );
    let face_b1 = make_face(
        [
            Point3::new(1.0, 0.0, -1.0),
            Point3::new(1.0, 0.0, 1.0),
            Point3::new(1.0, 4.0, 1.0),
            Point3::new(1.0, 4.0, -1.0),
        ],
        Vec3::new(-1.0, 0.0, 0.0),
    );
    let face_b2 = make_face(
        [
            Point3::new(3.0, 0.0, -1.0),
            Point3::new(3.0, 0.0, 1.0),
            Point3::new(3.0, 4.0, 1.0),
            Point3::new(3.0, 4.0, -1.0),
        ],
        Vec3::new(-1.0, 0.0, 0.0),
    );

    let batches = zenith_algo::BrepIntersectionBuilder::collect_planar_face_batch_splits(
        &[face_a],
        &[face_b1, face_b2],
        &tol,
    );

    assert_eq!(batches.splits_a.len(), 1);
    assert_eq!(batches.splits_a[0].face_index, 0);
    assert_eq!(batches.splits_a[0].split_edge_count, 2);
    assert_eq!(batches.splits_a[0].result.applied_split_count, 2);
    assert_eq!(batches.splits_a[0].result.faces.len(), 3);
    assert_eq!(batches.splits_b.len(), 2);
    assert!(batches
        .splits_b
        .iter()
        .all(|split| split.result.faces.len() == 2));
}

#[test]
fn test_brep_intersection_collects_planar_face_split_candidates() {
    let tol = Tolerance::default();
    let make_face = |points: [Point3; 4], normal: Vec3| {
        let vertices: Vec<Vertex> = points
            .iter()
            .map(|point| Vertex::from_point(*point))
            .collect();
        let edges = vec![
            Edge::line_between(vertices[0].clone(), vertices[1].clone()).unwrap(),
            Edge::line_between(vertices[1].clone(), vertices[2].clone()).unwrap(),
            Edge::line_between(vertices[2].clone(), vertices[3].clone()).unwrap(),
            Edge::line_between(vertices[3].clone(), vertices[0].clone()).unwrap(),
        ];
        let wire = Wire::new(edges.into_iter().map(OrientedEdge::forward).collect());
        let u_axis = (points[1] - points[0]).normalize();
        let v_axis = normal.cross(&u_axis).normalize();
        let plane = PlaneSurface3::new(points[0], u_axis, v_axis).unwrap();
        Face::simple(FaceGeometry::Plane(plane), wire)
    };
    let face_a = make_face(
        [
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(4.0, 0.0, 0.0),
            Point3::new(4.0, 4.0, 0.0),
            Point3::new(0.0, 4.0, 0.0),
        ],
        Vec3::new(0.0, 0.0, 1.0),
    );
    let face_b = make_face(
        [
            Point3::new(2.0, 0.0, -1.0),
            Point3::new(2.0, 0.0, 1.0),
            Point3::new(2.0, 4.0, 1.0),
            Point3::new(2.0, 4.0, -1.0),
        ],
        Vec3::new(-1.0, 0.0, 0.0),
    );

    let splits = zenith_algo::BrepIntersectionBuilder::collect_planar_face_split_candidates(
        &[face_a],
        &[face_b],
        &tol,
    );

    assert_eq!(splits.len(), 1);
    let split = &splits[0];
    assert_eq!(split.face_a_index, 0);
    assert_eq!(split.face_b_index, 0);
    assert_eq!(split.split_faces_a.len(), 2);
    assert_eq!(split.split_faces_b.len(), 2);
    assert!((split.split_edge.end_vertex.point - split.split_edge.start_vertex.point).norm() > 3.9);

    for face in split.split_faces_a.iter().chain(split.split_faces_b.iter()) {
        assert!(face.outer_wire.is_closed(&tol));
        assert!(face.validate_pcurves(&tol, 4).unwrap().is_valid());
    }
}

#[test]
fn test_brep_face_classification_against_solid() {
    let tol = Tolerance::default();
    let solid = zenith_algo::PrimitiveBuilder::make_box(10.0, 10.0, 10.0).unwrap();
    let make_face = |points: [Point3; 4], normal: Vec3| {
        let vertices: Vec<Vertex> = points
            .iter()
            .map(|point| Vertex::from_point(*point))
            .collect();
        let edges = vec![
            Edge::line_between(vertices[0].clone(), vertices[1].clone()).unwrap(),
            Edge::line_between(vertices[1].clone(), vertices[2].clone()).unwrap(),
            Edge::line_between(vertices[2].clone(), vertices[3].clone()).unwrap(),
            Edge::line_between(vertices[3].clone(), vertices[0].clone()).unwrap(),
        ];
        let wire = Wire::new(edges.into_iter().map(OrientedEdge::forward).collect());
        let u_axis = (points[1] - points[0]).normalize();
        let v_axis = normal.cross(&u_axis).normalize();
        let plane = PlaneSurface3::new(points[0], u_axis, v_axis).unwrap();
        Face::simple(FaceGeometry::Plane(plane), wire)
    };

    let inside_face = make_face(
        [
            Point3::new(2.0, 2.0, 5.0),
            Point3::new(4.0, 2.0, 5.0),
            Point3::new(4.0, 4.0, 5.0),
            Point3::new(2.0, 4.0, 5.0),
        ],
        Vec3::new(0.0, 0.0, 1.0),
    );
    let outside_face = make_face(
        [
            Point3::new(12.0, 2.0, 5.0),
            Point3::new(14.0, 2.0, 5.0),
            Point3::new(14.0, 4.0, 5.0),
            Point3::new(12.0, 4.0, 5.0),
        ],
        Vec3::new(0.0, 0.0, 1.0),
    );
    let boundary_face = make_face(
        [
            Point3::new(2.0, 2.0, 0.0),
            Point3::new(4.0, 2.0, 0.0),
            Point3::new(4.0, 4.0, 0.0),
            Point3::new(2.0, 4.0, 0.0),
        ],
        Vec3::new(0.0, 0.0, 1.0),
    );

    assert_eq!(
        zenith_algo::BrepIntersectionBuilder::classify_face_against_solid(
            &inside_face,
            &solid,
            &tol
        ),
        zenith_algo::FaceRegionLocation::Inside
    );
    assert_eq!(
        zenith_algo::BrepIntersectionBuilder::classify_face_against_solid(
            &outside_face,
            &solid,
            &tol
        ),
        zenith_algo::FaceRegionLocation::Outside
    );
    assert_eq!(
        zenith_algo::BrepIntersectionBuilder::classify_face_against_solid(
            &boundary_face,
            &solid,
            &tol
        ),
        zenith_algo::FaceRegionLocation::Boundary
    );
}

#[test]
fn test_brep_collects_selected_boolean_faces_after_batch_splits() {
    let tol = Tolerance::default();
    let solid_a = zenith_algo::PrimitiveBuilder::make_box(10.0, 10.0, 10.0).unwrap();
    let solid_b = zenith_algo::PrimitiveBuilder::make_cylinder(3.0, 10.0).unwrap();

    let selection = zenith_algo::BrepIntersectionBuilder::collect_selected_boolean_face_pieces(
        &solid_a,
        &solid_b,
        zenith_algo::BooleanOpType::Union,
        &tol,
    );

    assert!(!selection.selected_face_pieces.is_empty());
    assert_eq!(
        selection.stitch_report.face_piece_count,
        selection.selected_face_pieces.len()
    );
}

#[test]
fn test_brep_boolean_face_piece_selection_from_classification() {
    let tol = Tolerance::default();
    let points = [
        Point3::new(0.0, 0.0, 0.0),
        Point3::new(1.0, 0.0, 0.0),
        Point3::new(1.0, 1.0, 0.0),
        Point3::new(0.0, 1.0, 0.0),
    ];
    let vertices: Vec<Vertex> = points
        .iter()
        .map(|point| Vertex::from_point(*point))
        .collect();
    let edges = vec![
        Edge::line_between(vertices[0].clone(), vertices[1].clone()).unwrap(),
        Edge::line_between(vertices[1].clone(), vertices[2].clone()).unwrap(),
        Edge::line_between(vertices[2].clone(), vertices[3].clone()).unwrap(),
        Edge::line_between(vertices[3].clone(), vertices[0].clone()).unwrap(),
    ];
    let wire = Wire::new(edges.into_iter().map(OrientedEdge::forward).collect());
    let plane = PlaneSurface3::new(
        points[0],
        Vec3::new(1.0, 0.0, 0.0),
        Vec3::new(0.0, 1.0, 0.0),
    )
    .unwrap();
    let face = Face::simple(FaceGeometry::Plane(plane), wire);
    let split_edge = Edge::line_between(
        Vertex::new(Point3::new(0.5, 0.0, 0.0), tol.linear),
        Vertex::new(Point3::new(0.5, 1.0, 0.0), tol.linear),
    )
    .unwrap();
    let candidate = zenith_algo::ClassifiedPlanarFaceSplitCandidate {
        face_a_index: 0,
        face_b_index: 0,
        split_edge,
        split_faces_a: vec![
            zenith_algo::ClassifiedFacePiece {
                face: face.clone(),
                location: zenith_algo::FaceRegionLocation::Outside,
            },
            zenith_algo::ClassifiedFacePiece {
                face: face.clone(),
                location: zenith_algo::FaceRegionLocation::Inside,
            },
        ],
        split_faces_b: vec![
            zenith_algo::ClassifiedFacePiece {
                face: face.clone(),
                location: zenith_algo::FaceRegionLocation::Outside,
            },
            zenith_algo::ClassifiedFacePiece {
                face,
                location: zenith_algo::FaceRegionLocation::Inside,
            },
        ],
    };

    let union = zenith_algo::BrepIntersectionBuilder::select_boolean_face_pieces(
        &candidate,
        zenith_algo::BooleanOpType::Union,
    );
    assert_eq!(union.len(), 2);
    assert!(union
        .iter()
        .all(|piece| piece.location == zenith_algo::FaceRegionLocation::Outside));

    let intersection = zenith_algo::BrepIntersectionBuilder::select_boolean_face_pieces(
        &candidate,
        zenith_algo::BooleanOpType::Intersection,
    );
    assert_eq!(intersection.len(), 2);
    assert!(intersection
        .iter()
        .all(|piece| piece.location == zenith_algo::FaceRegionLocation::Inside));

    let difference = zenith_algo::BrepIntersectionBuilder::select_boolean_face_pieces(
        &candidate,
        zenith_algo::BooleanOpType::Difference,
    );
    assert_eq!(difference.len(), 2);
    assert!(difference.iter().any(|piece| {
        piece.operand == zenith_algo::BooleanOperand::A && !piece.reverse_orientation
    }));
    assert!(difference.iter().any(|piece| {
        piece.operand == zenith_algo::BooleanOperand::B && piece.reverse_orientation
    }));
}

#[test]
fn test_brep_selected_face_stitching_diagnostics() {
    let tol = Tolerance::default();
    let solid = zenith_algo::PrimitiveBuilder::make_box(10.0, 10.0, 10.0).unwrap();
    let pieces: Vec<_> = solid
        .outer_shell
        .faces
        .iter()
        .cloned()
        .map(|face| zenith_algo::SelectedBooleanFacePiece {
            operand: zenith_algo::BooleanOperand::A,
            face,
            location: zenith_algo::FaceRegionLocation::Outside,
            reverse_orientation: false,
        })
        .collect();

    let report =
        zenith_algo::BrepIntersectionBuilder::diagnose_selected_face_stitching(&pieces, &tol);
    assert_eq!(report.face_piece_count, 6);
    assert_eq!(report.edge_use_count, 24);
    assert_eq!(report.matched_edge_pair_count, 12);
    assert_eq!(report.unmatched_edge_use_count, 0);
    assert_eq!(report.non_manifold_edge_use_count, 0);
    assert_eq!(report.same_direction_edge_use_count, 0);
    assert!(report.is_closed_manifold());

    let open_report =
        zenith_algo::BrepIntersectionBuilder::diagnose_selected_face_stitching(&pieces[0..1], &tol);
    assert_eq!(open_report.edge_use_count, 4);
    assert_eq!(open_report.unmatched_edge_use_count, 4);
    assert!(!open_report.is_closed_manifold());

    let duplicate_report = zenith_algo::BrepIntersectionBuilder::diagnose_selected_face_stitching(
        &[pieces[0].clone(), pieces[0].clone()],
        &tol,
    );
    assert!(duplicate_report.same_direction_edge_use_count > 0);
    assert!(!duplicate_report.is_closed_manifold());
}

#[test]
fn test_brep_builds_solid_from_stitched_selected_face_pieces() {
    let tol = Tolerance::default();
    let solid = zenith_algo::PrimitiveBuilder::make_box(10.0, 10.0, 10.0).unwrap();
    let pieces: Vec<_> = solid
        .outer_shell
        .faces
        .iter()
        .cloned()
        .map(|face| zenith_algo::SelectedBooleanFacePiece {
            operand: zenith_algo::BooleanOperand::A,
            face,
            location: zenith_algo::FaceRegionLocation::Outside,
            reverse_orientation: false,
        })
        .collect();

    let rebuilt =
        zenith_algo::BrepIntersectionBuilder::build_solid_from_selected_face_pieces(&pieces, &tol)
            .expect("stitched selected faces should rebuild a valid solid");

    assert_eq!(rebuilt.outer_shell.faces.len(), 6);
    assert!(rebuilt.is_topologically_valid(&tol));
}

#[test]
fn test_brep_builds_planar_cap_from_unordered_edge_loop() {
    let tol = Tolerance::default();
    let p0 = Point3::new(0.0, 0.0, 0.0);
    let p1 = Point3::new(2.0, 0.0, 0.0);
    let p2 = Point3::new(2.0, 3.0, 0.0);
    let p3 = Point3::new(0.0, 3.0, 0.0);
    let e0 = Edge::line_between(Vertex::new(p0, tol.linear), Vertex::new(p1, tol.linear)).unwrap();
    let e1 = Edge::line_between(Vertex::new(p1, tol.linear), Vertex::new(p2, tol.linear)).unwrap();
    let e2 = Edge::line_between(Vertex::new(p3, tol.linear), Vertex::new(p2, tol.linear)).unwrap();
    let e3 = Edge::line_between(Vertex::new(p3, tol.linear), Vertex::new(p0, tol.linear)).unwrap();

    let cap = zenith_algo::BrepIntersectionBuilder::build_planar_cap_from_edge_loop(
        &[e2, e0, e3, e1],
        &tol,
    )
    .expect("unordered cap edge loop should build a planar face");

    assert!(matches!(cap.geometry, FaceGeometry::Plane(_)));
    assert!(cap.outer_wire.is_closed(&tol));
    assert!(cap.inner_wires.is_empty());
    assert!(cap.pcurves.is_some());
    assert!(cap.validate_pcurves(&tol, 4).unwrap().is_valid());
}

#[test]
fn test_brep_assembles_selected_faces_with_caps_for_stitching() {
    let tol = Tolerance::default();
    let solid = zenith_algo::PrimitiveBuilder::make_box(10.0, 10.0, 10.0).unwrap();
    let cap = zenith_algo::BrepIntersectionBuilder::build_planar_cap_from_edge_loop(
        &solid.outer_shell.faces[1]
            .outer_wire
            .edges
            .iter()
            .map(|edge| edge.edge.clone())
            .collect::<Vec<_>>(),
        &tol,
    )
    .expect("top cap from the top face boundary");
    let pieces: Vec<_> = solid
        .outer_shell
        .faces
        .iter()
        .enumerate()
        .filter(|(index, _)| *index != 1)
        .map(|(_, face)| zenith_algo::SelectedBooleanFacePiece {
            operand: zenith_algo::BooleanOperand::A,
            face: face.clone(),
            location: zenith_algo::FaceRegionLocation::Outside,
            reverse_orientation: false,
        })
        .collect();

    let assembly = zenith_algo::BrepIntersectionBuilder::assemble_selected_face_pieces_with_caps(
        &pieces,
        &[cap],
        &tol,
    );

    assert_eq!(assembly.cap_face_count, 1);
    assert_eq!(assembly.selected_face_pieces.len(), 6);
    assert!(assembly.stitch_report.is_closed_manifold());
}

#[test]
fn test_brep_assembly_orients_reversed_caps_for_stitching() {
    let tol = Tolerance::default();
    let solid = zenith_algo::PrimitiveBuilder::make_box(10.0, 10.0, 10.0).unwrap();
    let cap = zenith_algo::BrepIntersectionBuilder::build_planar_cap_from_edge_loop(
        &solid.outer_shell.faces[1]
            .outer_wire
            .edges
            .iter()
            .map(|edge| edge.edge.clone())
            .collect::<Vec<_>>(),
        &tol,
    )
    .expect("top cap from the top face boundary");
    let reversed_cap = Face::new(
        cap.geometry.clone(),
        Wire::new(
            cap.outer_wire
                .edges
                .iter()
                .rev()
                .map(|edge| OrientedEdge::new(edge.edge.clone(), edge.orientation.reversed()))
                .collect(),
        ),
        cap.inner_wires
            .iter()
            .rev()
            .map(|wire| {
                Wire::new(
                    wire.edges
                        .iter()
                        .rev()
                        .map(|edge| {
                            OrientedEdge::new(edge.edge.clone(), edge.orientation.reversed())
                        })
                        .collect(),
                )
            })
            .collect(),
        cap.orientation.reversed(),
        cap.tolerance,
    );
    let pieces: Vec<_> = solid
        .outer_shell
        .faces
        .iter()
        .enumerate()
        .filter(|(index, _)| *index != 1)
        .map(|(_, face)| zenith_algo::SelectedBooleanFacePiece {
            operand: zenith_algo::BooleanOperand::A,
            face: face.clone(),
            location: zenith_algo::FaceRegionLocation::Outside,
            reverse_orientation: false,
        })
        .collect();

    let assembly = zenith_algo::BrepIntersectionBuilder::assemble_selected_face_pieces_with_caps(
        &pieces,
        &[reversed_cap],
        &tol,
    );

    assert_eq!(assembly.cap_face_count, 1);
    assert_eq!(assembly.selected_face_pieces.len(), 6);
    assert!(assembly.stitch_report.is_closed_manifold());
    assert!(
        assembly
            .selected_face_pieces
            .last()
            .unwrap()
            .reverse_orientation
    );
}

#[test]
fn test_brep_extracts_intersection_edge_loops_and_builds_caps() {
    let tol = Tolerance::default();
    let make_edge = |a: Point3, b: Point3| {
        Edge::line_between(Vertex::new(a, tol.linear), Vertex::new(b, tol.linear)).unwrap()
    };
    let a0 = Point3::new(0.0, 0.0, 0.0);
    let a1 = Point3::new(2.0, 0.0, 0.0);
    let a2 = Point3::new(2.0, 2.0, 0.0);
    let a3 = Point3::new(0.0, 2.0, 0.0);
    let b0 = Point3::new(5.0, 0.0, 1.0);
    let b1 = Point3::new(7.0, 0.0, 1.0);
    let b2 = Point3::new(7.0, 2.0, 1.0);
    let b3 = Point3::new(5.0, 2.0, 1.0);
    let edges = vec![
        make_edge(a2, a3),
        make_edge(b0, b1),
        make_edge(a1, a2),
        make_edge(b3, b2),
        make_edge(a0, a1),
        make_edge(b3, b0),
        make_edge(a3, a0),
        make_edge(b1, b2),
    ];

    let extraction =
        zenith_algo::BrepIntersectionBuilder::collect_closed_intersection_edge_loops(&edges, &tol);
    assert_eq!(extraction.loops.len(), 2);
    assert_eq!(extraction.skipped_edge_count, 0);
    assert!(extraction
        .loops
        .iter()
        .all(|edge_loop| edge_loop.edges.len() == 4));

    let caps = zenith_algo::BrepIntersectionBuilder::build_planar_caps_from_intersection_edges(
        &edges, &tol,
    );
    assert_eq!(caps.edge_loop_extraction.loops.len(), 2);
    assert_eq!(caps.cap_faces.len(), 2);
    assert_eq!(caps.failed_loop_count, 0);
    for cap in caps.cap_faces {
        assert!(matches!(cap.geometry, FaceGeometry::Plane(_)));
        assert!(cap.outer_wire.is_closed(&tol));
        assert!(cap.validate_pcurves(&tol, 4).unwrap().is_valid());
    }
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
fn test_cylinder_side_nurbs_patch_stays_on_analytic_cylinder() {
    let radius = 10.0;
    let height = 30.0;
    let cyl = zenith_algo::PrimitiveBuilder::make_cylinder(radius, height)
        .expect("Cylinder creation failed");

    for face in cyl.outer_shell.faces.iter().take(4) {
        let FaceGeometry::Nurbs(surface) = &face.geometry else {
            panic!("Cylinder side should be a NURBS face");
        };
        let ((u_min, u_max), (v_min, v_max)) = surface.param_range();

        for u_step in 0..=8 {
            for v_step in 0..=4 {
                let u = u_min + (u_max - u_min) * (u_step as f64 / 8.0);
                let v = v_min + (v_max - v_min) * (v_step as f64 / 4.0);
                let point = surface.evaluate(u, v);
                let radial_distance = (point.x * point.x + point.y * point.y).sqrt();

                assert!(
                    (radial_distance - radius).abs() < 1e-6,
                    "side patch point should stay on the cylinder radius"
                );
                assert!(
                    point.z >= -1e-6 && point.z <= height + 1e-6,
                    "side patch point should remain within cylinder height"
                );
            }
        }
    }
}

#[test]
fn test_cylinder_cap_boundaries_share_side_edge_circles() {
    let radius = 10.0;
    let cyl = zenith_algo::PrimitiveBuilder::make_cylinder(radius, 30.0)
        .expect("Cylinder creation failed");

    for cap_face in cyl.outer_shell.faces.iter().skip(4) {
        let FaceGeometry::Plane(plane) = &cap_face.geometry else {
            panic!("Cylinder cap should be planar");
        };
        let pcurves = cap_face
            .pcurves
            .as_ref()
            .expect("cap should store p-curves");
        assert_eq!(pcurves.outer_loop.segments.len(), 4);

        for (edge, segment) in cap_face
            .outer_wire
            .edges
            .iter()
            .zip(pcurves.outer_loop.segments.iter())
        {
            assert_eq!(segment.edge_id, edge.edge.id);
            let (t_min, t_max) = segment.curve.param_range();
            for step in 0..=8 {
                let t = step as f64 / 8.0;
                let point_from_edge = edge.evaluate_normalized(t);
                let uv_t = t_min + (t_max - t_min) * t;
                let uv = segment.curve.evaluate(uv_t);
                let point_from_cap = plane.evaluate(uv.x, uv.y);
                let radial_distance = (point_from_cap.x * point_from_cap.x
                    + point_from_cap.y * point_from_cap.y)
                    .sqrt();

                assert!((point_from_cap - point_from_edge).norm() < 1e-6);
                assert!((radial_distance - radius).abs() < 1e-6);
            }
        }
    }
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
fn test_closed_shell_validation_rejects_degenerate_planar_face_area() {
    let tol = Tolerance::default();
    let box_solid = zenith_algo::PrimitiveBuilder::make_box(10.0, 20.0, 30.0).unwrap();
    let mut faces = box_solid.outer_shell.faces.clone();
    let pcurves = faces[0].pcurves.as_mut().expect("plane p-curves");
    for segment in &mut pcurves.outer_loop.segments {
        for control in &mut segment.curve.control_points {
            control.point.y = 0.0;
        }
    }

    let corrupted_shell = Shell::closed(faces);
    let report = corrupted_shell.validate_closed(&tol);

    assert!(!report.is_valid());
    assert!(report.degenerate_face_count > 0);
    assert!(report.min_planar_face_area <= tol.parametric);
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
fn test_trimmed_nurbs_tessellation_follows_a_diagonal_loop() {
    let radius: f64 = 10.0;
    let cylinder = zenith_algo::PrimitiveBuilder::make_cylinder(radius, 30.0).unwrap();
    let FaceGeometry::Nurbs(surface) = cylinder.outer_shell.faces[0].geometry.clone() else {
        panic!("cylinder side face should be a NURBS patch");
    };

    // UV上の三角形トリム。軸平行な部分矩形ではないので、以前は矩形全体を
    // 貼るか、境界だけの1三角形にしかならなかった領域。
    let ((u_min, u_max), (v_min, v_max)) = surface.param_range();
    let uv_at = |u: f64, v: f64| {
        zenith_math::Point2::new(u_min + (u_max - u_min) * u, v_min + (v_max - v_min) * v)
    };
    let loop_uv = [uv_at(0.1, 0.1), uv_at(0.9, 0.25), uv_at(0.35, 0.9)];

    let mut edges = Vec::new();
    let mut segments = Vec::new();
    for i in 0..3 {
        let (start_uv, end_uv) = (loop_uv[i], loop_uv[(i + 1) % 3]);
        let edge = Edge::line_between(
            Vertex::from_point(surface.evaluate(start_uv.x, start_uv.y)),
            Vertex::from_point(surface.evaluate(end_uv.x, end_uv.y)),
        )
        .unwrap();
        let pcurve = NurbsCurve2::new(
            1,
            vec![
                ControlPoint2::unweighted(start_uv),
                ControlPoint2::unweighted(end_uv),
            ],
            KnotVector::clamped_uniform(2, 1),
        )
        .unwrap();
        segments.push(FacePcurveSegment {
            edge_id: edge.id,
            orientation: Orientation::Forward,
            curve: pcurve,
        });
        edges.push(OrientedEdge::forward(edge));
    }

    let mut face = Face::new(
        FaceGeometry::Nurbs(surface.clone()),
        Wire::new(edges),
        Vec::new(),
        Orientation::Forward,
        1e-6,
    );
    face.pcurves = Some(FacePcurves {
        outer_loop: FacePcurveLoop { segments },
        inner_loops: Vec::new(),
    });

    let mesh = tessellate_face(
        &face,
        &TessellationParams {
            u_divisions: 32,
            v_divisions: 32,
        },
    );

    // 境界だけの三角化なら1枚で終わる。内部が細分されていることを要求する。
    assert!(
        mesh.num_triangles() > 200,
        "trimmed NURBS interior was not refined: {} triangles",
        mesh.num_triangles()
    );

    // 全頂点がトリム三角形の内側に留まる（矩形全体を貼っていない）
    let inside = |point: zenith_math::Point2| {
        let sign = |a: zenith_math::Point2, b: zenith_math::Point2| {
            (b.x - a.x) * (point.y - a.y) - (b.y - a.y) * (point.x - a.x)
        };
        let s0 = sign(loop_uv[0], loop_uv[1]);
        let s1 = sign(loop_uv[1], loop_uv[2]);
        let s2 = sign(loop_uv[2], loop_uv[0]);
        let tolerance = 1e-9;
        (s0 >= -tolerance && s1 >= -tolerance && s2 >= -tolerance)
            || (s0 <= tolerance && s1 <= tolerance && s2 <= tolerance)
    };
    for uv in &mesh.uvs {
        assert!(
            inside(zenith_math::Point2::new(uv.x, uv.y)),
            "tessellation escaped the trim loop at {uv:?}"
        );
    }

    // 内部が平面で埋められていないこと: 三角形の重心も解析円柱の近くにある
    for triangle in &mesh.indices {
        let a = mesh.positions[triangle[0] as usize];
        let b = mesh.positions[triangle[1] as usize];
        let c = mesh.positions[triangle[2] as usize];
        let centroid = Point3::from((a.coords + b.coords + c.coords) / 3.0);
        let radial = (centroid.x * centroid.x + centroid.y * centroid.y).sqrt();
        assert!(
            (radial - radius).abs() < 0.05,
            "interior triangle centroid drifted off the cylinder: {radial}"
        );
    }
}

#[test]
fn test_nurbs_face_tessellation_respects_inner_pcurve_trim_loop() {
    let surface = NurbsSurface3::new(
        1,
        1,
        vec![
            vec![
                ControlPoint3::unweighted(Point3::new(0.0, 0.0, 0.0)),
                ControlPoint3::unweighted(Point3::new(0.0, 10.0, 0.0)),
            ],
            vec![
                ControlPoint3::unweighted(Point3::new(10.0, 0.0, 0.0)),
                ControlPoint3::unweighted(Point3::new(10.0, 10.0, 0.0)),
            ],
        ],
        KnotVector::clamped_uniform(2, 1),
        KnotVector::clamped_uniform(2, 1),
    )
    .expect("NURBS surface");

    let make_segment = |uv0: (f64, f64), uv1: (f64, f64)| {
        let p0 = surface.evaluate(uv0.0, uv0.1);
        let p1 = surface.evaluate(uv1.0, uv1.1);
        let edge =
            Edge::line_between(Vertex::from_point(p0), Vertex::from_point(p1)).expect("trim edge");
        let pcurve = NurbsCurve2::new(
            1,
            vec![
                ControlPoint2::unweighted(zenith_math::Point2::new(uv0.0, uv0.1)),
                ControlPoint2::unweighted(zenith_math::Point2::new(uv1.0, uv1.1)),
            ],
            KnotVector::clamped_uniform(2, 1),
        )
        .expect("trim p-curve");
        let oriented_edge = OrientedEdge::forward(edge.clone());
        let segment = FacePcurveSegment {
            edge_id: edge.id,
            orientation: Orientation::Forward,
            curve: pcurve,
        };
        (oriented_edge, segment)
    };

    let outer_uv = [(0.0, 0.0), (1.0, 0.0), (1.0, 1.0), (0.0, 1.0)];
    let inner_uv = [(0.4, 0.4), (0.6, 0.4), (0.6, 0.6), (0.4, 0.6)];
    let mut outer_edges = Vec::new();
    let mut outer_segments = Vec::new();
    let mut inner_edges = Vec::new();
    let mut inner_segments = Vec::new();

    for i in 0..4 {
        let (edge, segment) = make_segment(outer_uv[i], outer_uv[(i + 1) % 4]);
        outer_edges.push(edge);
        outer_segments.push(segment);
    }
    for i in 0..4 {
        let (edge, segment) = make_segment(inner_uv[i], inner_uv[(i + 1) % 4]);
        inner_edges.push(edge);
        inner_segments.push(segment);
    }

    let mut face = Face::new(
        FaceGeometry::Nurbs(surface),
        Wire::new(outer_edges),
        vec![Wire::new(inner_edges)],
        Orientation::Forward,
        1e-6,
    );
    face.pcurves = Some(FacePcurves {
        outer_loop: FacePcurveLoop {
            segments: outer_segments,
        },
        inner_loops: vec![FacePcurveLoop {
            segments: inner_segments,
        }],
    });

    let mesh = tessellate_face(
        &face,
        &TessellationParams {
            u_divisions: 8,
            v_divisions: 8,
        },
    );

    assert!(mesh.num_triangles() > 0);
    for tri in &mesh.indices {
        let a = mesh.positions[tri[0] as usize];
        let b = mesh.positions[tri[1] as usize];
        let c = mesh.positions[tri[2] as usize];
        let centroid_x = (a.x + b.x + c.x) / 30.0;
        let centroid_y = (a.y + b.y + c.y) / 30.0;
        assert!(
            !(centroid_x > 0.4 && centroid_x < 0.6 && centroid_y > 0.4 && centroid_y < 0.6),
            "trimmed NURBS tessellation filled the inner p-curve hole"
        );
    }
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
fn test_taper_face_produces_a_valid_tilted_solid() {
    let tol = Tolerance::default();
    let (dx, dy, dz) = (10.0_f64, 20.0_f64, 30.0_f64);
    let angle = 10.0_f64;
    let solid = zenith_algo::PrimitiveBuilder::make_box(dx, dy, dz).unwrap();

    // 天面を、その稜線 (y = 0, z = dz) まわりに傾ける
    let tapered = zenith_algo::DirectModeling::taper_face(
        &solid,
        1,
        Point3::new(0.0, 0.0, dz),
        Vec3::new(1.0, 0.0, 0.0),
        angle,
    )
    .expect("tapering a box top face should produce a valid solid");

    assert!(tapered.is_topologically_valid(&tol));
    let report = tapered.outer_shell.validate_closed(&tol);
    assert_eq!(report.unmatched_edge_use_count, 0);
    assert_eq!(report.same_direction_edge_use_count, 0);
    assert!(report.errors.is_empty(), "{:?}", report.errors);

    // 回転軸上の頂点は動かない
    for face in &tapered.outer_shell.faces {
        for point in face.outer_wire.sample_points(2) {
            assert!(point.z >= -1e-9);
        }
    }

    // 体積は YZ 断面（台形）× dx と一致する
    let radians = angle.to_radians();
    let cross_section = 0.5 * (dy * (dz + dy * radians.sin()) + dy * radians.cos() * dz);
    let params = TessellationParams {
        u_divisions: 8,
        v_divisions: 8,
    };
    let mass = zenith_algo::MassCalculator::compute_from_brep(&tapered, &params);
    assert!(
        (mass.volume - cross_section * dx).abs() < 1e-6,
        "taper volume {} vs analytic {}",
        mass.volume,
        cross_section * dx
    );
}

#[test]
fn test_push_pull_keeps_a_cylinder_exact() {
    let tol = Tolerance::default();
    let radius: f64 = 10.0;
    let height: f64 = 30.0;
    let growth: f64 = 10.0;
    let cylinder = zenith_algo::PrimitiveBuilder::make_cylinder(radius, height).unwrap();

    // 天面（+Z のキャップ）を引き上げる
    let top_index = cylinder
        .outer_shell
        .faces
        .iter()
        .position(|face| {
            matches!(face.geometry, FaceGeometry::Plane(_))
                && face
                    .outer_wire
                    .sample_points(2)
                    .iter()
                    .all(|point| (point.z - height).abs() < 1e-9)
        })
        .expect("top cap");

    let taller = zenith_algo::DirectModeling::push_pull_face(&cylinder, top_index, growth).unwrap();
    assert!(taller.is_topologically_valid(&tol));

    // 側面は NURBS のまま、境界の円弧も2次のまま残る
    assert_eq!(
        taller
            .outer_shell
            .faces
            .iter()
            .filter(|face| matches!(face.geometry, FaceGeometry::Nurbs(_)))
            .count(),
        4
    );
    let arc_uses = taller
        .outer_shell
        .faces
        .iter()
        .flat_map(|face| face.outer_wire.edges.iter())
        .filter(|edge| edge.edge.curve.degree == 2)
        .count();
    assert_eq!(
        arc_uses, 16,
        "every circular arc use should survive the edit"
    );

    // 境界が解析円柱から外れない
    for face in &taller.outer_shell.faces {
        for point in face.outer_wire.sample_points(8) {
            let radial = (point.x * point.x + point.y * point.y).sqrt();
            assert!(
                (radial - radius).abs() < 1e-9,
                "edited boundary left the cylinder at radius {radial}"
            );
            assert!(point.z >= -1e-9 && point.z <= height + growth + 1e-9);
        }
    }

    // 体積は解析値どおりに増える
    let params = TessellationParams {
        u_divisions: 32,
        v_divisions: 32,
    };
    let mass = zenith_algo::MassCalculator::compute_from_brep(&taller, &params);
    let expected = std::f64::consts::PI * radius * radius * (height + growth);
    assert!(
        (mass.volume - expected).abs() < expected * 1e-9,
        "push-pull volume {} vs analytic {expected}",
        mass.volume
    );
}

#[test]
fn test_push_pull_refuses_edits_it_cannot_represent() {
    let cylinder = zenith_algo::PrimitiveBuilder::make_cylinder(10.0, 30.0).unwrap();

    // 側面パッチを法線方向に押すと隣接曲面の延長・再トリムが必要になる。
    // 直線で近似せず、明示的に失敗しなければならない。
    let side_index = cylinder
        .outer_shell
        .faces
        .iter()
        .position(|face| matches!(face.geometry, FaceGeometry::Nurbs(_)))
        .expect("side patch");
    let result = zenith_algo::DirectModeling::push_pull_face(&cylinder, side_index, 5.0);
    assert!(
        result.is_err(),
        "an unsupported push-pull must fail instead of degrading the geometry"
    );
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
    assert!(step.contains("PCURVE"));
    assert!(step.contains("SURFACE_CURVE"));

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
