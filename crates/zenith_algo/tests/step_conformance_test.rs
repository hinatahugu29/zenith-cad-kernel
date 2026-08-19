//! Guards the STEP export details that decide whether another kernel can read
//! the result as a solid at all.
//!
//! These were found by round-tripping through FreeCAD / OpenCASCADE: a missing
//! `CURVE()` in the complex curve entity made OCC silently drop the bound of
//! every plane trimmed by spline arcs, which turned cylinders, cones, drilled
//! boxes and swept pipes into unusable Compounds. The failure was invisible
//! from inside this kernel, so it is pinned here at the text level.

use zenith_algo::{HoleBuilder, PrimitiveBuilder, SweepBuilder};
use zenith_geom::NurbsCurve3;
use zenith_io::StepExporter;
use zenith_math::Point3;

fn export(name: &str, solid: &zenith_topo::Solid) -> String {
    StepExporter::export_solid_to_string(solid, name)
}

#[test]
fn test_rational_curves_declare_the_full_complex_entity() {
    let step = export(
        "CYL",
        &PrimitiveBuilder::make_cylinder(10.0, 40.0).unwrap(),
    );

    assert!(
        step.contains("RATIONAL_B_SPLINE_CURVE"),
        "a cylinder must export rational arcs"
    );

    // AP214 の複合エンティティは全スーパータイプを列挙する必要がある。
    // CURVE() が欠けると OpenCASCADE が境界ループを捨てる。
    for occurrence in step.match_indices("RATIONAL_B_SPLINE_CURVE") {
        let prefix = &step[..occurrence.0];
        let entity_start = prefix.rfind("( BOUNDED_CURVE()").expect("complex curve entity");
        let entity = &step[entity_start..occurrence.0];
        assert!(
            entity.contains(" CURVE() "),
            "complex curve entity is missing CURVE(): {entity}"
        );
    }
}

#[test]
fn test_rational_surfaces_declare_the_full_complex_entity() {
    let step = export("SPH", &PrimitiveBuilder::make_sphere(10.0).unwrap());

    assert!(step.contains("RATIONAL_B_SPLINE_SURFACE"));
    for occurrence in step.match_indices("RATIONAL_B_SPLINE_SURFACE") {
        let prefix = &step[..occurrence.0];
        let entity_start = prefix
            .rfind("( BOUNDED_SURFACE()")
            .expect("complex surface entity");
        let entity = &step[entity_start..];
        let entity_end = entity.find(");").unwrap_or(entity.len());
        let entity = &entity[..entity_end];
        assert!(
            entity.contains("SURFACE()"),
            "complex surface entity is missing SURFACE(): {entity}"
        );
    }
}

#[test]
fn test_planar_faces_bounded_by_arcs_are_exported_with_shared_edges() {
    // 円柱の端面は平面をスプライン円弧で囲んだ面。側面と同じ EDGE_CURVE を
    // 共有していないと、読み手が閉シェルを組み立てられない。
    let step = export(
        "CYL",
        &PrimitiveBuilder::make_cylinder(10.0, 40.0).unwrap(),
    );

    let edge_curve_count = step.matches("EDGE_CURVE(").count();
    let oriented_edge_count = step.matches("ORIENTED_EDGE(").count();

    assert!(
        oriented_edge_count > edge_curve_count,
        "every edge should be referenced twice: {edge_curve_count} curves, {oriented_edge_count} uses"
    );
    assert_eq!(
        oriented_edge_count,
        edge_curve_count * 2,
        "a closed manifold uses each edge exactly twice"
    );
}

#[test]
fn test_closed_surfaces_declare_their_closed_flags() {
    // トーラス・球は分割済みなので個々のパッチは閉じていない。閉フラグは
    // 幾何から判定されるので、開いたパッチでは .F. のままであるべき。
    let step = export("TOR", &PrimitiveBuilder::make_torus(12.0, 4.0).unwrap());
    assert!(step.contains("B_SPLINE_SURFACE(2,2"));
    assert!(
        !step.contains(".T.,.T.,.F.)"),
        "individual torus patches are not closed surfaces"
    );
}

#[test]
fn test_curved_solids_export_a_manifold_solid_brep() {
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

    let subjects = [
        ("CYL", PrimitiveBuilder::make_cylinder(10.0, 40.0).unwrap()),
        (
            "CONE",
            PrimitiveBuilder::make_cone(10.0, 4.0, 20.0).unwrap(),
        ),
        ("SPH", PrimitiveBuilder::make_sphere(10.0).unwrap()),
        ("TOR", PrimitiveBuilder::make_torus(12.0, 4.0).unwrap()),
        (
            "DRILL",
            HoleBuilder::make_drilled_box(30.0, 30.0, 15.0, 5.0).unwrap(),
        ),
        (
            "PIPE",
            SweepBuilder::sweep_circle_along_curve(&path, 3.5, 16).unwrap(),
        ),
    ];

    for (name, solid) in subjects {
        let step = export(name, &solid);
        assert!(
            step.contains("MANIFOLD_SOLID_BREP"),
            "{name} must export as a solid, not a surface model"
        );
        assert!(
            step.contains("CLOSED_SHELL"),
            "{name} must export a closed shell"
        );
    }
}
