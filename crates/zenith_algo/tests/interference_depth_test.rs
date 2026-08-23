//! どこまで浅い食い込みを、食い込みとして検出できるか。
//!
//! 干渉判定で危ないのは**見落とし**です。離れているものを干渉と言えば設計者が
//! 確かめて終わりますが、食い込んでいるものを「隙間あり」と言えばそのまま
//! 製造に流れます。
//!
//! 板に 0.01 mm 押し込んだ球は「隙間 0.010」と報告されていました。原因は2つ
//! 重なっています。重なり体積を 20^3 の格子で数えるので格子の目より薄い
//! 食い込みが落ちること、そして三角形どうしの距離は辺と辺・頂点と面しか
//! 見ないので**交差している**三角形の組では正の値になることです。

use zenith_algo::{
    BrepTransform, ClashStatus, DistanceEngine, InterferenceChecker, PrimitiveBuilder,
};
use zenith_math::{Tolerance, Vec3};

const DEPTHS: [f64; 7] = [5.0, 1.0, 0.5, 0.1, 0.05, 0.01, 0.001];

#[test]
fn a_ball_pressed_into_a_plate_is_a_clash_however_shallow() {
    let tol = Tolerance::default();
    for depth in DEPTHS {
        let plate = PrimitiveBuilder::make_box(60.0, 60.0, 10.0).unwrap();
        let ball = BrepTransform::translate_solid(
            &PrimitiveBuilder::make_sphere(5.0).unwrap(),
            Vec3::new(30.0, 30.0, 15.0 - depth),
        );

        let report = InterferenceChecker::check(&plate, &ball, &tol);
        assert_eq!(
            report.status,
            ClashStatus::Clash,
            "a ball {depth} into a plate came back as {:?} ({})",
            report.status,
            report.message
        );
        assert_eq!(report.min_distance, 0.0);
    }
}

#[test]
fn a_pin_and_two_boxes_are_a_clash_however_shallow() {
    let tol = Tolerance::default();
    for depth in DEPTHS {
        let plate = PrimitiveBuilder::make_box(60.0, 60.0, 10.0).unwrap();
        let pin = BrepTransform::translate_solid(
            &PrimitiveBuilder::make_cylinder(2.0, 20.0).unwrap(),
            Vec3::new(30.0, 30.0, 10.0 - depth),
        );
        assert_eq!(
            InterferenceChecker::check(&plate, &pin, &tol).status,
            ClashStatus::Clash,
            "a pin {depth} into a plate"
        );

        let a = PrimitiveBuilder::make_box(20.0, 20.0, 20.0).unwrap();
        let b = BrepTransform::translate_solid(
            &PrimitiveBuilder::make_box(20.0, 20.0, 20.0).unwrap(),
            Vec3::new(20.0 - depth, 0.0, 0.0),
        );
        assert_eq!(
            InterferenceChecker::check(&a, &b, &tol).status,
            ClashStatus::Clash,
            "two boxes {depth} into each other"
        );
    }
}

#[test]
fn the_reported_gap_is_the_gap() {
    let tol = Tolerance::default();
    for gap in [10.0f64, 1.0, 0.1] {
        let plate = PrimitiveBuilder::make_box(200.0, 200.0, 2.0).unwrap();
        let ball = BrepTransform::translate_solid(
            &PrimitiveBuilder::make_sphere(5.0).unwrap(),
            Vec3::new(100.0, 100.0, 7.0 + gap),
        );

        let report = InterferenceChecker::check(&plate, &ball, &tol);
        assert_eq!(report.status, ClashStatus::Clearance, "gap {gap}");
        assert!(
            (report.min_distance - gap).abs() < 1e-9,
            "a {gap} gap was reported as {}",
            report.min_distance
        );
        // 干渉判定と距離計算は同じ値を答えなければならない
        assert!(
            (DistanceEngine::compute_min_distance(&plate, &ball, &tol).min_distance - gap).abs()
                < 1e-9
        );
    }
}

#[test]
fn solids_that_only_touch_are_not_a_clash() {
    let tol = Tolerance::default();
    let a = PrimitiveBuilder::make_box(20.0, 20.0, 20.0).unwrap();
    let b = BrepTransform::translate_solid(
        &PrimitiveBuilder::make_box(20.0, 20.0, 20.0).unwrap(),
        Vec3::new(20.0, 0.0, 0.0),
    );
    let report = InterferenceChecker::check(&a, &b, &tol);
    assert_eq!(
        report.status,
        ClashStatus::Touching,
        "boxes sharing a face: {}",
        report.message
    );
    assert_eq!(report.min_distance, 0.0);
}
