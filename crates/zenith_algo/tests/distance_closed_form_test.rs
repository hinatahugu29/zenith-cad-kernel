//! 最短距離が、閉じた式に乗るか。
//!
//! 以前の実装はテッセレーションした2つのメッシュの**頂点どうし**を総当たりして
//! いました。三角形の内側を見ないので、最近接点が頂点に来ない配置で答えが桁で
//! 外れます。直方体の頂点は8個しかないため、板の上に置いた物体は必ず**板の隅**
//! との距離で測られていました。
//!
//! | 配置 | 正しい距離 | 旧実装 |
//! | :--- | --: | --: |
//! | 200x200x2 の板の中央の上 3 mm に小球 | 3.0 | 136.6 |
//! | 同じ板の上 0.5 mm に小さな角材 | 0.5 | 138.6 |
//! | 5 mm めり込んだ2つの直方体 | 0.0 | 5.0 |
//!
//! クリアランス検証に使う値なので、干渉している設計が「隙間あり」と出ます。

use zenith_algo::{BrepTransform, DistanceEngine, PrimitiveBuilder};
use zenith_math::{Tolerance, Vec3};
use zenith_tess::TessellationParams;
use zenith_topo::Solid;

fn distance(a: &Solid, b: &Solid) -> zenith_algo::DistanceResult {
    DistanceEngine::compute_min_distance(a, b, &Tolerance::default())
}

#[test]
fn the_closest_points_can_be_in_the_middle_of_a_face() {
    // 板の隅は 140 mm 以上離れている。頂点しか見ない実装はそちらを返す。
    let plate = PrimitiveBuilder::make_box(200.0, 200.0, 2.0).unwrap();
    let ball = BrepTransform::translate_solid(
        &PrimitiveBuilder::make_sphere(5.0).unwrap(),
        Vec3::new(100.0, 100.0, 10.0),
    );

    let result = distance(&plate, &ball);
    assert!(
        (result.min_distance - 3.0).abs() < 1e-9,
        "a ball 3 above the middle of a plate: {}",
        result.min_distance
    );
    // 最近接点は板の面の中央あたりで、隅ではない
    assert!(
        (result.closest_point_a.z - 2.0).abs() < 1e-9,
        "the point on the plate should be on its top face: {:?}",
        result.closest_point_a
    );
    assert!(
        (result.closest_point_a.x - 100.0).abs() < 1e-6
            && (result.closest_point_a.y - 100.0).abs() < 1e-6,
        "the point on the plate should be under the ball: {:?}",
        result.closest_point_a
    );
}

#[test]
fn a_face_to_face_gap_is_the_gap_not_the_corner_to_corner_distance() {
    let a = PrimitiveBuilder::make_box(10.0, 10.0, 10.0).unwrap();
    let b = BrepTransform::translate_solid(
        &PrimitiveBuilder::make_box(10.0, 10.0, 10.0).unwrap(),
        Vec3::new(20.0, 0.0, 0.0),
    );
    assert!((distance(&a, &b).min_distance - 10.0).abs() < 1e-12);

    // 板の上に細い角材。最近接は面と面。
    let plate = PrimitiveBuilder::make_box(200.0, 200.0, 2.0).unwrap();
    let bar = BrepTransform::translate_solid(
        &PrimitiveBuilder::make_box(4.0, 4.0, 4.0).unwrap(),
        Vec3::new(98.0, 98.0, 2.5),
    );
    let result = distance(&plate, &bar);
    assert!(
        (result.min_distance - 0.5).abs() < 1e-9,
        "a bar 0.5 above a plate: {}",
        result.min_distance
    );
}

#[test]
fn curved_solids_land_on_the_closed_form_exactly() {
    let sphere = PrimitiveBuilder::make_sphere(10.0).unwrap();
    let far = BrepTransform::translate_solid(
        &PrimitiveBuilder::make_sphere(10.0).unwrap(),
        Vec3::new(40.0, 0.0, 0.0),
    );
    assert!(
        (distance(&sphere, &far).min_distance - 20.0).abs() < 1e-9,
        "two spheres 20 apart: {}",
        distance(&sphere, &far).min_distance
    );

    let cylinder = PrimitiveBuilder::make_cylinder(5.0, 20.0).unwrap();
    let beside = BrepTransform::translate_solid(
        &PrimitiveBuilder::make_cylinder(5.0, 20.0).unwrap(),
        Vec3::new(30.0, 0.0, 0.0),
    );
    assert!(
        (distance(&cylinder, &beside).min_distance - 20.0).abs() < 1e-9,
        "two parallel cylinders 20 apart: {}",
        distance(&cylinder, &beside).min_distance
    );
}

#[test]
fn solids_that_touch_or_overlap_have_no_clearance() {
    let a = PrimitiveBuilder::make_box(10.0, 10.0, 10.0).unwrap();
    for offset in [10.0f64, 5.0, 0.0] {
        let b = BrepTransform::translate_solid(
            &PrimitiveBuilder::make_box(10.0, 10.0, 10.0).unwrap(),
            Vec3::new(offset, 0.0, 0.0),
        );
        assert_eq!(
            distance(&a, &b).min_distance,
            0.0,
            "boxes offset by {offset} are not clear of each other"
        );
    }

    // 片方が他方の内側に完全に入っている場合も 0
    let outer = PrimitiveBuilder::make_box(40.0, 40.0, 40.0).unwrap();
    let inner = BrepTransform::translate_solid(
        &PrimitiveBuilder::make_box(4.0, 4.0, 4.0).unwrap(),
        Vec3::new(18.0, 18.0, 18.0),
    );
    assert_eq!(distance(&outer, &inner).min_distance, 0.0);
}

#[test]
fn the_answer_does_not_move_when_the_tessellation_changes() {
    // 刻みは探索の出発点にしか効かない。答えは B-Rep の面へ詰めるので動かない。
    let plate = PrimitiveBuilder::make_box(200.0, 200.0, 2.0).unwrap();
    let ball = BrepTransform::translate_solid(
        &PrimitiveBuilder::make_sphere(5.0).unwrap(),
        Vec3::new(100.0, 100.0, 10.0),
    );

    let values: Vec<f64> = [8usize, 16, 32]
        .into_iter()
        .map(|divisions| {
            DistanceEngine::compute_min_distance_with_tessellation(
                &plate,
                &ball,
                &Tolerance::default(),
                &TessellationParams {
                    u_divisions: divisions,
                    v_divisions: divisions,
                },
            )
            .min_distance
        })
        .collect();

    for value in &values {
        assert!(
            (value - 3.0).abs() < 1e-9,
            "the distance moved with the tessellation: {values:?}"
        );
    }
}
