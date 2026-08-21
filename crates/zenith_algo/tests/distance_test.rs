use zenith_algo::{BrepTransform, DistanceEngine, PrimitiveBuilder};
use zenith_math::{Tolerance, Vec3};

#[test]
fn test_distance_between_disjoint_spheres() {
    let tol = Tolerance::default();
    let r1 = 10.0;
    let r2 = 15.0;

    let sphere_a = PrimitiveBuilder::make_sphere(r1).expect("sphere a");
    let sphere_b = PrimitiveBuilder::make_sphere(r2).expect("sphere b");

    // 球Bを X 軸方向に 50mm 移動 (中心間距離 = 50mm)
    let sphere_b = BrepTransform::translate_solid(&sphere_b, Vec3::new(50.0, 0.0, 0.0));

    let result = DistanceEngine::compute_min_distance(&sphere_a, &sphere_b, &tol);

    // 解析的最短距離: 中心間距離 - r1 - r2 = 50 - 10 - 15 = 25mm
    let expected_distance = 25.0;
    let diff = (result.min_distance - expected_distance).abs();

    assert!(
        diff < 0.5,
        "sphere distance {} vs expected {}, diff {}",
        result.min_distance,
        expected_distance,
        diff
    );
}

#[test]
fn test_distance_between_separated_boxes() {
    let tol = Tolerance::default();
    let box_a = PrimitiveBuilder::make_box(20.0, 20.0, 20.0).expect("box a");
    let box_b = PrimitiveBuilder::make_box(20.0, 20.0, 20.0).expect("box b");

    // 箱Bを X 軸方向に 35mm 移動 (X=[35, 55]) -> 面間距離 = 35 - 20 = 15mm
    let box_b = BrepTransform::translate_solid(&box_b, Vec3::new(35.0, 0.0, 0.0));

    let result = DistanceEngine::compute_min_distance(&box_a, &box_b, &tol);

    let expected_distance = 15.0;
    let diff = (result.min_distance - expected_distance).abs();

    assert!(
        diff < 1e-4,
        "box distance {} vs expected {}, diff {}",
        result.min_distance,
        expected_distance,
        diff
    );
}
