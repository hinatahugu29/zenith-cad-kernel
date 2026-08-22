use zenith_geom::{ControlPoint3, CornerBlendN, GregoryPatch4, KnotVector, NurbsCurve3, Surface3};
use zenith_math::{Point3, Tolerance};

fn make_line_curve(p0: Point3, p1: Point3) -> NurbsCurve3 {
    NurbsCurve3::new(
        1,
        vec![
            ControlPoint3::unweighted(p0),
            ControlPoint3::unweighted(p1),
        ],
        KnotVector::clamped_uniform(2, 1),
    ).unwrap()
}

#[test]
fn test_gregory_patch_boundary_interpolation() {
    let tol = Tolerance::default();
    let p00 = Point3::new(0.0, 0.0, 0.0);
    let p10 = Point3::new(10.0, 0.0, 2.0);
    let p11 = Point3::new(10.0, 10.0, 5.0);
    let p01 = Point3::new(0.0, 10.0, 1.0);

    let c0 = make_line_curve(p00, p10);
    let c1 = make_line_curve(p10, p11);
    let c2 = make_line_curve(p01, p11);
    let c3 = make_line_curve(p00, p01);

    let patch = GregoryPatch4::new(c0, c1, c2, c3, &tol).expect("Gregory patch creation");

    // 4隅の補間精度
    let ep00 = patch.evaluate(0.0, 0.0);
    let ep10 = patch.evaluate(1.0, 0.0);
    let ep11 = patch.evaluate(1.0, 1.0);
    let ep01 = patch.evaluate(0.0, 1.0);

    assert!((ep00 - p00).norm() < 1e-9);
    assert!((ep10 - p10).norm() < 1e-9);
    assert!((ep11 - p11).norm() < 1e-9);
    assert!((ep01 - p01).norm() < 1e-9);

    // 内部点の評価と法線
    let mid = patch.evaluate(0.5, 0.5);
    assert!(mid.z > 0.0);
    let normal = patch.normal(0.5, 0.5).expect("valid normal");
    assert!(normal.norm() > 0.99);
}

#[test]
fn test_n_sided_corner_blend_creation() {
    let tol = Tolerance::default();
    let p0 = Point3::new(10.0, 0.0, 0.0);
    let p1 = Point3::new(0.0, 10.0, 0.0);
    let p2 = Point3::new(0.0, 0.0, 10.0);

    let c0 = make_line_curve(p0, p1);
    let c1 = make_line_curve(p1, p2);
    let c2 = make_line_curve(p2, p0);

    let blend = CornerBlendN::create_n_sided_blend(vec![c0, c1, c2], &tol)
        .expect("3-sided corner blend");

    assert_eq!(blend.boundary_curves.len(), 3);
    assert!((blend.center_point.x - 3.333).abs() < 0.1);
    assert!((blend.center_point.y - 3.333).abs() < 0.1);
    assert!((blend.center_point.z - 3.333).abs() < 0.1);
}
