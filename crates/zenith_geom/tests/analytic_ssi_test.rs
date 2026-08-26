use zenith_geom::{
    AnalyticIntersection, AnalyticIntersectionResult, Curve3, PlaneSurface3,
};
use zenith_math::{Point3, Tolerance, Vec3};

#[test]
fn test_plane_plane_intersection() {
    let tol = Tolerance::default();

    // xy平面 (法線 +Z, 原点 0,0,0) と yz平面 (法線 +X, 原点 0,0,0)
    let p_xy = PlaneSurface3::new(
        Point3::new(0.0, 0.0, 0.0),
        Vec3::new(1.0, 0.0, 0.0),
        Vec3::new(0.0, 1.0, 0.0),
    )
    .unwrap();
    let p_yz = PlaneSurface3::new(
        Point3::new(0.0, 0.0, 0.0),
        Vec3::new(0.0, 1.0, 0.0),
        Vec3::new(0.0, 0.0, 1.0),
    )
    .unwrap();

    let line = AnalyticIntersection::intersect_plane_plane(&p_xy, &p_yz, &tol).expect("intersect");
    let dir = line.direction().expect("dir");

    // 交線は Y軸に平行
    assert!((dir.x).abs() < 1e-12);
    assert!((dir.y.abs() - 1.0).abs() < 1e-12);
    assert!((dir.z).abs() < 1e-12);

    // 平行平面
    let p_xy_offset = PlaneSurface3::new(
        Point3::new(0.0, 0.0, 10.0),
        Vec3::new(1.0, 0.0, 0.0),
        Vec3::new(0.0, 1.0, 0.0),
    )
    .unwrap();
    assert!(AnalyticIntersection::intersect_plane_plane(&p_xy, &p_xy_offset, &tol).is_none());
}

#[test]
fn test_plane_sphere_intersection() {
    let tol = Tolerance::default();
    let sphere_center = Point3::new(0.0, 0.0, 0.0);
    let sphere_radius = 10.0;

    // 1. 赤道断面 (z=0)
    let p_eq = PlaneSurface3::new(
        Point3::new(0.0, 0.0, 0.0),
        Vec3::new(1.0, 0.0, 0.0),
        Vec3::new(0.0, 1.0, 0.0),
    )
    .unwrap();
    let res_eq = AnalyticIntersection::intersect_plane_sphere(&p_eq, sphere_center, sphere_radius, &tol);
    match res_eq {
        AnalyticIntersectionResult::Circle(c) => {
            assert!((c.radius - 10.0).abs() < 1e-12);
            assert!((c.center - sphere_center).norm() < 1e-12);
        }
        _ => panic!("Expected Circle for equatorial section"),
    }

    // 2. 平行オフセット断面 (z=6) => 半径 sqrt(100 - 36) = 8
    let p_z6 = PlaneSurface3::new(
        Point3::new(0.0, 0.0, 6.0),
        Vec3::new(1.0, 0.0, 0.0),
        Vec3::new(0.0, 1.0, 0.0),
    )
    .unwrap();
    let res_z6 = AnalyticIntersection::intersect_plane_sphere(&p_z6, sphere_center, sphere_radius, &tol);
    match res_z6 {
        AnalyticIntersectionResult::Circle(c) => {
            assert!((c.radius - 8.0).abs() < 1e-12);
            assert!((c.center - Point3::new(0.0, 0.0, 6.0)).norm() < 1e-12);
        }
        _ => panic!("Expected Circle of radius 8.0"),
    }

    // 3. 離脱平面 (z=15)
    let p_z15 = PlaneSurface3::new(
        Point3::new(0.0, 0.0, 15.0),
        Vec3::new(1.0, 0.0, 0.0),
        Vec3::new(0.0, 1.0, 0.0),
    )
    .unwrap();
    let res_z15 = AnalyticIntersection::intersect_plane_sphere(&p_z15, sphere_center, sphere_radius, &tol);
    assert_eq!(res_z15, AnalyticIntersectionResult::Empty);
}

#[test]
fn test_plane_cylinder_intersection() {
    let tol = Tolerance::default();
    let cyl_axis_pt = Point3::new(0.0, 0.0, 0.0);
    let cyl_axis_dir = Vec3::new(0.0, 0.0, 1.0);
    let cyl_radius = 5.0;

    // 1. 垂直断面 (z=10) => 半径 5.0 の真円
    let p_perp = PlaneSurface3::new(
        Point3::new(0.0, 0.0, 10.0),
        Vec3::new(1.0, 0.0, 0.0),
        Vec3::new(0.0, 1.0, 0.0),
    )
    .unwrap();
    let res_perp = AnalyticIntersection::intersect_plane_cylinder(
        &p_perp,
        cyl_axis_pt,
        cyl_axis_dir,
        cyl_radius,
        &tol,
    );
    match res_perp {
        AnalyticIntersectionResult::Circle(c) => {
            assert!((c.radius - 5.0).abs() < 1e-12);
            assert!((c.center - Point3::new(0.0, 0.0, 10.0)).norm() < 1e-12);
        }
        _ => panic!("Expected Circle for perpendicular cylinder section"),
    }

    // 2. 軸平行割線断面 (x=3) => y = +-4 の2本の平行直線
    let p_parallel = PlaneSurface3::new(
        Point3::new(3.0, 0.0, 0.0),
        Vec3::new(0.0, 1.0, 0.0),
        Vec3::new(0.0, 0.0, 1.0),
    )
    .unwrap();
    let res_par = AnalyticIntersection::intersect_plane_cylinder(
        &p_parallel,
        cyl_axis_pt,
        cyl_axis_dir,
        cyl_radius,
        &tol,
    );
    match res_par {
        AnalyticIntersectionResult::TwoLines(l1, l2) => {
            let p1 = l1.start;
            let p2 = l2.start;
            assert!((p1.x - 3.0).abs() < 1e-12);
            assert!((p2.x - 3.0).abs() < 1e-12);
            assert!((p1.y.abs() - 4.0).abs() < 1e-12);
            assert!((p2.y.abs() - 4.0).abs() < 1e-12);
        }
        _ => panic!("Expected TwoLines for parallel cylinder secant section"),
    }

    // 3. 斜め断面 (45度傾斜平面: 法線 (-1/sqrt(2), 0, 1/sqrt(2)))
    let p_diag = PlaneSurface3::new(
        Point3::new(0.0, 0.0, 0.0),
        Vec3::new(0.0, 1.0, 0.0),
        Vec3::new(-1.0, 0.0, 1.0).normalize(),
    )
    .unwrap();
    let res_diag = AnalyticIntersection::intersect_plane_cylinder(
        &p_diag,
        cyl_axis_pt,
        cyl_axis_dir,
        cyl_radius,
        &tol,
    );
    match res_diag {
        AnalyticIntersectionResult::Ellipse(ellipse) => {
            // 短軸半径 b = 5.0, 長軸半径 a = 5.0 / cos(45deg) = 5.0 * sqrt(2) = 7.0710678...
            let expected_a = 5.0 * std::f64::consts::SQRT_2;
            let expected_b = 5.0;
            assert!((ellipse.major_radius - expected_a).abs() < 1e-10);
            assert!((ellipse.minor_radius - expected_b).abs() < 1e-10);

            // 有理2次NURBS曲線への変換と幾何軌跡の厳密検証
            let nurbs_ellipse = ellipse.to_nurbs().expect("ellipse to NURBS");
            let (u_min, u_max) = nurbs_ellipse.param_range();

            // 1. 各セグメントのノット点（0, 90, 180, 270, 360度）での完全一致
            for k in 0..=4 {
                let u = k as f64;
                let theta = k as f64 * std::f64::consts::FRAC_PI_2;
                let pt_analytic = ellipse.evaluate(theta);
                let pt_nurbs = nurbs_ellipse.evaluate(u);
                let diff = (pt_analytic - pt_nurbs).norm();
                assert!(diff < 1e-12, "Knot point {k} mismatch: analytic={pt_analytic:?}, nurbs={pt_nurbs:?}, diff={diff}");
            }

            // 2. 任意のサンプリングパラメータ u において、点が厳密に楕円上（円柱面かつ平面上）に乗ること
            for i in 0..=100 {
                let u = u_min + (u_max - u_min) * (i as f64 / 100.0);
                let pt_nurbs = nurbs_ellipse.evaluate(u);

                // 点が円柱面 (x^2 + y^2 = 25) 上にあること (10^-12)
                let r_cyl = (pt_nurbs.x * pt_nurbs.x + pt_nurbs.y * pt_nurbs.y).sqrt();
                assert!((r_cyl - 5.0).abs() < 1e-12, "Point must lie exactly on cylinder surface");

                // 点が平面上にあること (10^-12)
                let dist_plane = (pt_nurbs - p_diag.origin).dot(&p_diag.normal).abs();
                assert!(dist_plane < 1e-12, "Point must lie exactly on plane");
            }
        }
        _ => panic!("Expected Ellipse for oblique cylinder section"),
    }
}

#[test]
fn test_sphere_cylinder_coaxial() {
    let tol = Tolerance::default();
    let sphere_center = Point3::new(0.0, 0.0, 0.0);
    let sphere_radius = 10.0;
    let cyl_axis_pt = Point3::new(0.0, 0.0, -50.0);
    let cyl_axis_dir = Vec3::new(0.0, 0.0, 1.0);
    let cyl_radius = 6.0;

    let res = AnalyticIntersection::intersect_sphere_cylinder_coaxial(
        sphere_center,
        sphere_radius,
        cyl_axis_pt,
        cyl_axis_dir,
        cyl_radius,
        &tol,
    );

    match res {
        AnalyticIntersectionResult::TwoCircles(c1, c2) => {
            assert!((c1.radius - 6.0).abs() < 1e-12);
            assert!((c2.radius - 6.0).abs() < 1e-12);
            // 高さ z = +- sqrt(100 - 36) = +- 8.0
            assert!((c1.center.z - 8.0).abs() < 1e-12 || (c1.center.z + 8.0).abs() < 1e-12);
            assert!((c2.center.z - 8.0).abs() < 1e-12 || (c2.center.z + 8.0).abs() < 1e-12);
            assert!((c1.center.z - c2.center.z).abs() > 15.0);
        }
        _ => panic!("Expected TwoCircles for coaxial sphere-cylinder intersection"),
    }
}
