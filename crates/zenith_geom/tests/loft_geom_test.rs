use std::f64::consts::{FRAC_PI_2, PI};
use zenith_geom::{Circle3, Curve3};
use zenith_math::{Point3, Vec3};

#[test]
fn test_circle3_to_nurbs_accuracy() {
    let center = Point3::new(10.0, 20.0, 30.0);
    let radius = 15.0;
    let normal = Vec3::new(0.0, 0.0, 1.0);

    // 1. 90度円弧 (0 -> PI/2)
    let circle_90 = Circle3::new(center, radius, normal, 0.0, FRAC_PI_2).unwrap();
    let nurbs_90 = circle_90.to_nurbs().expect("Failed to convert circle to NURBS");

    let (u_min, u_max) = nurbs_90.param_range();
    let steps = 50;
    for i in 0..=steps {
        let frac = i as f64 / steps as f64;
        let u = u_min + frac * (u_max - u_min);

        let pt_nurbs = nurbs_90.evaluate(u);

        // 真円性検証: 半径が厳密に radius と一致するか
        let dist_to_center = (pt_nurbs - center).norm();
        assert!(
            (dist_to_center - radius).abs() < 1e-12,
            "NURBS circle radius mismatch at u={u}: got {dist_to_center}, expected {radius}"
        );

        // 平面性検証: 法線方向の変位がゼロか
        let plane_dist = (pt_nurbs - center).dot(&normal).abs();
        assert!(
            plane_dist < 1e-12,
            "NURBS circle plane mismatch at u={u}: got {plane_dist}"
        );
    }

    // 始点・中点・終点が解析解と一致すること
    let p_start = nurbs_90.evaluate(u_min);
    let p_end = nurbs_90.evaluate(u_max);
    assert!((p_start - circle_90.evaluate(0.0)).norm() < 1e-12);
    assert!((p_end - circle_90.evaluate(FRAC_PI_2)).norm() < 1e-12);

    // 2. 360度完全真円 (0 -> 2*PI)
    let circle_360 = Circle3::new(center, radius, normal, 0.0, 2.0 * PI).unwrap();
    let nurbs_360 = circle_360.to_nurbs().expect("Failed to convert full circle to NURBS");

    let (u_min360, u_max360) = nurbs_360.param_range();
    for i in 0..=100 {
        let frac = i as f64 / 100.0;
        let u = u_min360 + frac * (u_max360 - u_min360);

        let pt = nurbs_360.evaluate(u);
        let dist = (pt - center).norm();
        assert!(
            (dist - radius).abs() < 1e-12,
            "360deg full circle radius mismatch at u={u}: got {dist}, expected {radius}"
        );
    }

    // 3. 傾いた平面上の円弧
    let tilted_normal = Vec3::new(1.0, 2.0, 3.0).normalize();
    let circle_tilted = Circle3::new(center, radius, tilted_normal, 0.5, 3.8).unwrap();
    let nurbs_tilted = circle_tilted.to_nurbs().expect("Failed to convert tilted circle");

    let (u_mint, u_maxt) = nurbs_tilted.param_range();
    for i in 0..=60 {
        let frac = i as f64 / 60.0;
        let u = u_mint + frac * (u_maxt - u_mint);

        let pt = nurbs_tilted.evaluate(u);
        let dist = (pt - center).norm();
        assert!(
            (dist - radius).abs() < 1e-12,
            "Tilted circle radius mismatch at u={u}: got {dist}, expected {radius}"
        );

        let plane_dist = (pt - center).dot(&tilted_normal).abs();
        assert!(
            plane_dist < 1e-12,
            "Tilted circle plane mismatch at u={u}: got {plane_dist}"
        );
    }
}

