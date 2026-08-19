use zenith_algo::{LoftBuilder, MassCalculator};
use zenith_geom::Curve3;
use zenith_io::{StepExporter, StepImporter};
use zenith_math::{Point3, Tolerance, Vec3};
use zenith_tess::{tessellate_solid, TessellationParams};
use zenith_topo::{Edge, OrientedEdge, Vertex, Wire};

#[test]
fn test_loft_solid_rectangular_pyramid_frustum() {
    let tol = Tolerance::default();

    // 1. 底面四角形ワイヤ (z = 0, 20x20)
    let pb0 = Point3::new(-10.0, -10.0, 0.0);
    let pb1 = Point3::new(10.0, -10.0, 0.0);
    let pb2 = Point3::new(10.0, 10.0, 0.0);
    let pb3 = Point3::new(-10.0, 10.0, 0.0);

    let vb0 = Vertex::from_point(pb0);
    let vb1 = Vertex::from_point(pb1);
    let vb2 = Vertex::from_point(pb2);
    let vb3 = Vertex::from_point(pb3);

    let eb0 = Edge::line_between(vb0.clone(), vb1.clone()).unwrap();
    let eb1 = Edge::line_between(vb1.clone(), vb2.clone()).unwrap();
    let eb2 = Edge::line_between(vb2.clone(), vb3.clone()).unwrap();
    let eb3 = Edge::line_between(vb3.clone(), vb0.clone()).unwrap();

    let wire_bottom = Wire::new(vec![
        OrientedEdge::forward(eb0),
        OrientedEdge::forward(eb1),
        OrientedEdge::forward(eb2),
        OrientedEdge::forward(eb3),
    ]);

    // 2. 中間四角形ワイヤ (z = 15, 14x14)
    let pm0 = Point3::new(-7.0, -7.0, 15.0);
    let pm1 = Point3::new(7.0, -7.0, 15.0);
    let pm2 = Point3::new(7.0, 7.0, 15.0);
    let pm3 = Point3::new(-7.0, 7.0, 15.0);

    let vm0 = Vertex::from_point(pm0);
    let vm1 = Vertex::from_point(pm1);
    let vm2 = Vertex::from_point(pm2);
    let vm3 = Vertex::from_point(pm3);

    let em0 = Edge::line_between(vm0.clone(), vm1.clone()).unwrap();
    let em1 = Edge::line_between(vm1.clone(), vm2.clone()).unwrap();
    let em2 = Edge::line_between(vm2.clone(), vm3.clone()).unwrap();
    let em3 = Edge::line_between(vm3.clone(), vm0.clone()).unwrap();

    let wire_mid = Wire::new(vec![
        OrientedEdge::forward(em0),
        OrientedEdge::forward(em1),
        OrientedEdge::forward(em2),
        OrientedEdge::forward(em3),
    ]);

    // 3. 天面四角形ワイヤ (z = 30, 6x6)
    let pt0 = Point3::new(-3.0, -3.0, 30.0);
    let pt1 = Point3::new(3.0, -3.0, 30.0);
    let pt2 = Point3::new(3.0, 3.0, 30.0);
    let pt3 = Point3::new(-3.0, 3.0, 30.0);

    let vt0 = Vertex::from_point(pt0);
    let vt1 = Vertex::from_point(pt1);
    let vt2 = Vertex::from_point(pt2);
    let vt3 = Vertex::from_point(pt3);

    let et0 = Edge::line_between(vt0.clone(), vt1.clone()).unwrap();
    let et1 = Edge::line_between(vt1.clone(), vt2.clone()).unwrap();
    let et2 = Edge::line_between(vt2.clone(), vt3.clone()).unwrap();
    let et3 = Edge::line_between(vt3.clone(), vt0.clone()).unwrap();

    let wire_top = Wire::new(vec![
        OrientedEdge::forward(et0),
        OrientedEdge::forward(et1),
        OrientedEdge::forward(et2),
        OrientedEdge::forward(et3),
    ]);

    // 3断面ロフトソリッドの生成
    let loft_solid = LoftBuilder::loft_solid(
        &[wire_bottom, wire_mid, wire_top],
        2,
        &tol,
    )
    .expect("LoftBuilder::loft_solid should succeed");

    // トポロジー検証
    let report = loft_solid.outer_shell.validate_closed(&tol);
    assert!(
        report.is_valid(),
        "Loft solid validation failed: {:?}",
        report.errors
    );
    assert_eq!(loft_solid.outer_shell.faces.len(), 4 * 2 + 2); // 4側面x2区間 + 底面 + 天面 = 10面

    // 物性値検証
    let tess_params = TessellationParams::default();
    let mesh = tessellate_solid(&loft_solid, &tess_params);
    assert!(!mesh.positions.is_empty());
    assert!(!mesh.indices.is_empty());

    let mass = MassCalculator::compute_from_mesh(&mesh);
    assert!(mass.volume > 0.0, "Loft solid volume must be positive: got {}", mass.volume);
    assert!(mass.surface_area > 0.0, "Surface area must be positive: got {}", mass.surface_area);

    // STEP ラウンドトリップ検証
    let step_str = StepExporter::export_solid_to_string(&loft_solid, "ZENITH_LOFT_SOLID");
    let imported_solid = StepImporter::import_solid_from_str(&step_str)
        .expect("STEP import of loft solid should succeed");

    let imported_report = imported_solid.outer_shell.validate_closed(&tol);
    assert!(
        imported_report.is_valid(),
        "Imported loft solid validation failed: {:?}",
        imported_report.errors
    );
}

#[test]
fn test_loft_solid_circular_cone_frustum() {
    let tol = Tolerance::default();

    // 補助関数: 4本の90度有理円弧エッジから円形ワイヤを生成
    let make_circular_wire = |center: Point3, radius: f64| -> Wire {
        let n = Vec3::new(0.0, 0.0, 1.0);
        let c0 = zenith_geom::Circle3::new(center, radius, n, 0.0, std::f64::consts::FRAC_PI_2).unwrap();
        let c1 = zenith_geom::Circle3::new(center, radius, n, std::f64::consts::FRAC_PI_2, std::f64::consts::PI).unwrap();
        let c2 = zenith_geom::Circle3::new(center, radius, n, std::f64::consts::PI, 3.0 * std::f64::consts::FRAC_PI_2).unwrap();
        let c3 = zenith_geom::Circle3::new(center, radius, n, 3.0 * std::f64::consts::FRAC_PI_2, 2.0 * std::f64::consts::PI).unwrap();

        let p0 = c0.evaluate(0.0);
        let p1 = c1.evaluate(std::f64::consts::FRAC_PI_2);
        let p2 = c2.evaluate(std::f64::consts::PI);
        let p3 = c3.evaluate(3.0 * std::f64::consts::FRAC_PI_2);

        let v0 = Vertex::from_point(p0);
        let v1 = Vertex::from_point(p1);
        let v2 = Vertex::from_point(p2);
        let v3 = Vertex::from_point(p3);

        let e0 = Edge::new(c0.to_nurbs().unwrap(), v0.clone(), v1.clone(), tol.linear);
        let e1 = Edge::new(c1.to_nurbs().unwrap(), v1.clone(), v2.clone(), tol.linear);
        let e2 = Edge::new(c2.to_nurbs().unwrap(), v2.clone(), v3.clone(), tol.linear);
        let e3 = Edge::new(c3.to_nurbs().unwrap(), v3.clone(), v0.clone(), tol.linear);

        Wire::new(vec![
            OrientedEdge::forward(e0),
            OrientedEdge::forward(e1),
            OrientedEdge::forward(e2),
            OrientedEdge::forward(e3),
        ])
    };

    let wire_bot = make_circular_wire(Point3::new(0.0, 0.0, 0.0), 15.0);
    eprintln!("wire_bot is_closed = {}", wire_bot.is_closed(&tol));
    let wire_mid = make_circular_wire(Point3::new(0.0, 0.0, 15.0), 11.0);
    eprintln!("wire_mid is_closed = {}", wire_mid.is_closed(&tol));
    let wire_top = make_circular_wire(Point3::new(0.0, 0.0, 30.0), 6.0);
    eprintln!("wire_top is_closed = {}", wire_top.is_closed(&tol));

    let loft_cone = match LoftBuilder::loft_solid(&[wire_bot, wire_mid, wire_top], 2, &tol) {
        Ok(s) => s,
        Err(e) => {
            eprintln!("=== LOFT ERROR: {} ===", e);
            panic!("Loft error: {}", e);
        }
    };

    let report = loft_cone.outer_shell.validate_closed(&tol);
    assert!(
        report.is_valid(),
        "Circular loft solid validation failed: {:?}",
        report.errors
    );

    // テッセレーションと物性値
    let mesh = tessellate_solid(&loft_cone, &TessellationParams::default());
    let mass = MassCalculator::compute_from_mesh(&mesh);
    assert!(mass.volume > 0.0, "Volume must be positive: got {}", mass.volume);

    // STEP ラウンドトリップ
    let step_str = StepExporter::export_solid_to_string(&loft_cone, "ZENITH_CIRCULAR_LOFT");
    let imported = StepImporter::import_solid_from_str(&step_str)
        .expect("STEP import of circular loft solid should succeed");
    assert!(imported.outer_shell.validate_closed(&tol).is_valid());
}

