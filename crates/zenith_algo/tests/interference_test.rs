//! 干渉判定が、答えの分かっている配置で正しく答えるか。
//!
//! 以前は軸並行の箱だけで判定しており、**離れている立体を `Clash` と答えて
//! いました**。半径5の球と、隅が (3,3,3) の箱は 0.196 離れていますが、箱が
//! 重なるので干渉と報告し、`overlap_volume` には箱の重なり 8.00 を入れて
//! いました。ここはその配置を含めて固定します。

use zenith_algo::{BrepTransform, ClashStatus, InterferenceChecker, PrimitiveBuilder};
use zenith_math::{Tolerance, Vec3};

fn cube(size: f64) -> zenith_topo::Solid {
    PrimitiveBuilder::make_box(size, size, size).unwrap()
}

fn shifted(solid: &zenith_topo::Solid, x: f64, y: f64, z: f64) -> zenith_topo::Solid {
    BrepTransform::translate_solid(solid, Vec3::new(x, y, z))
}

#[test]
fn solids_that_are_apart_are_reported_apart_with_their_real_distance() {
    let tol = Tolerance::default();

    let report = InterferenceChecker::check(&cube(20.0), &shifted(&cube(20.0), 50.0, 0.0, 0.0), &tol);
    assert_eq!(report.status, ClashStatus::Clearance);
    assert!(
        (report.min_distance - 30.0).abs() < 1e-9,
        "two cubes 30 apart read {}",
        report.min_distance
    );
    assert_eq!(report.overlap_volume, 0.0);
}

/// **箱で見ていた頃はここが `Clash` だった。**
///
/// 半径5の球の外に、隅が (3,3,3) の箱がある。箱のいちばん近い点は原点から
/// sqrt(27) = 5.196 なので、球の表面とは 0.196 離れている。箱同士は重なる
/// ので、箱だけを見れば干渉に見える。
#[test]
fn a_box_whose_corner_misses_a_sphere_is_not_a_clash() {
    let tol = Tolerance::default();
    let sphere = PrimitiveBuilder::make_sphere(5.0).unwrap();
    let boxed = shifted(&cube(7.0), 3.0, 3.0, 3.0);

    let report = InterferenceChecker::check(&sphere, &boxed, &tol);
    assert_eq!(
        report.status,
        ClashStatus::Clearance,
        "they are {:.6} apart, not clashing",
        27.0f64.sqrt() - 5.0
    );
    assert_eq!(report.overlap_volume, 0.0);

    // 距離は三角形に割った表面の上で測るので、球は内接多角形になり、
    // 真の距離よりわずかに大きく出る。向きが決まっているので、そう書く。
    let truth = 27.0f64.sqrt() - 5.0;
    assert!(
        report.min_distance >= truth - 1e-9,
        "an inscribed sphere cannot read shorter than {truth}, got {}",
        report.min_distance
    );
    assert!(
        report.min_distance - truth < 2e-2,
        "the distance {} is too far from {truth} for {} divisions",
        report.min_distance,
        report.sample_divisions
    );
}

#[test]
fn solids_that_share_a_face_are_touching_and_not_clashing() {
    let tol = Tolerance::default();
    let report = InterferenceChecker::check(&cube(20.0), &shifted(&cube(20.0), 20.0, 0.0, 0.0), &tol);
    assert_eq!(report.status, ClashStatus::Touching);
    assert_eq!(report.min_distance, 0.0);
    assert_eq!(report.overlap_volume, 0.0);
}

#[test]
fn overlapping_solids_report_the_volume_they_share() {
    let tol = Tolerance::default();

    // 角で 10^3 だけ重なる2つの立方体。
    let report = InterferenceChecker::check(
        &cube(20.0),
        &shifted(&cube(20.0), 10.0, 10.0, 10.0),
        &tol,
    );
    assert_eq!(report.status, ClashStatus::Clash);
    assert!(
        (report.overlap_volume - 1000.0).abs() / 1000.0 < 0.05,
        "the shared corner is 1000, read {}",
        report.overlap_volume
    );

    // 丸ごと入っている場合は、小さいほうの体積。
    let report = InterferenceChecker::check(&cube(20.0), &shifted(&cube(4.0), 8.0, 8.0, 8.0), &tol);
    assert_eq!(report.status, ClashStatus::Clash);
    assert!(
        (report.overlap_volume - 64.0).abs() / 64.0 < 0.05,
        "the contained cube is 64, read {}",
        report.overlap_volume
    );
}

/// 直交する2本の角棒。**どの頂点も相手の内側に来ない**が、互いを貫いている。
///
/// 表面の頂点だけを見る書き方ではここを取り逃がし、「19.0 離れている」と
/// 答えました。平面の面は隅にしか頂点を持たないためです。
#[test]
fn two_rods_that_cross_are_a_clash_even_though_no_vertex_is_inside_the_other() {
    let tol = Tolerance::default();
    let along_x = BrepTransform::translate_solid(
        &PrimitiveBuilder::make_box(40.0, 2.0, 2.0).unwrap(),
        Vec3::new(-20.0, -1.0, -1.0),
    );
    let along_y = BrepTransform::translate_solid(
        &PrimitiveBuilder::make_box(2.0, 40.0, 2.0).unwrap(),
        Vec3::new(-1.0, -20.0, -1.0),
    );

    let report = InterferenceChecker::check(&along_x, &along_y, &tol);
    assert_eq!(report.status, ClashStatus::Clash);
    assert!(
        (report.overlap_volume - 8.0).abs() / 8.0 < 0.05,
        "the crossing is a 2x2x2 cube, read {}",
        report.overlap_volume
    );
}

/// 分割を細かくすれば、距離は真の値へ寄る。
///
/// 寄る向きは決まっている——内接多角形なので、必ず**上から**近づく。
#[test]
fn refining_the_sampling_brings_the_distance_down_toward_the_truth() {
    let tol = Tolerance::default();
    let sphere = PrimitiveBuilder::make_sphere(5.0).unwrap();
    let boxed = shifted(&cube(7.0), 3.0, 3.0, 3.0);
    let truth = 27.0f64.sqrt() - 5.0;

    let coarse = InterferenceChecker::check_with_divisions(&sphere, &boxed, 8, &tol).min_distance;
    let fine = InterferenceChecker::check_with_divisions(&sphere, &boxed, 32, &tol).min_distance;

    assert!(coarse >= truth - 1e-9 && fine >= truth - 1e-9);
    assert!(
        fine < coarse,
        "refining should shorten the reading: coarse {coarse}, fine {fine}"
    );
    assert!(
        fine - truth < coarse - truth,
        "refining should close the gap to {truth}"
    );
}

#[test]
fn test_interference_checker_check_exact_returns_exact_brep_solid() {
    let tol = Tolerance::default();
    let cube_a = cube(20.0);
    let cube_b = shifted(&cube(20.0), 10.0, 10.0, 10.0);

    let (report, clash_solid) = InterferenceChecker::check_exact(&cube_a, &cube_b, &tol);
    assert_eq!(report.status, ClashStatus::Clash);
    assert!(clash_solid.is_some(), "clash solid should be extracted");

    let solid = clash_solid.unwrap();
    assert!(solid.is_topologically_valid(&tol), "clash solid must be valid solid");
    let diff = (report.overlap_volume - 1000.0).abs();
    assert!(diff < 1e-6, "exact overlap volume should be 1000.0, got {}, diff {}", report.overlap_volume, diff);
}
