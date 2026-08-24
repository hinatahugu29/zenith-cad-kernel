//! 他カーネルから読んだ立体への距離と内外判定。
//!
//! OpenCASCADE が書く球や円柱は**全周1枚の面**で来ます。外側ワイヤの稜は
//! 0本、p-curve のセグメントも 0本です。射影がこれを「内側には何も無い」と
//! 読んでいたので、読んだ球は**面を1枚も持たない立体**として扱われ、距離も
//! 内外も面を見ずに答えていました。自前ビルダーの立体は必ず境界ワイヤを
//! 持つので、既存の検体では1つも捕まりませんでした。
//!
//! もう1つは、足がトリムの外に落ちた面をそこで捨てていた件です。直方体の
//! 角の外にいる点は、6面すべてが「自分の長方形の外」と答え、立体への足を
//! 1つも持ちませんでした。

use std::path::PathBuf;

use zenith_algo::{exact_inside, nearest_boundary_projection, PrimitiveBuilder};
use zenith_io::StepImporter;
use zenith_math::{Point3, Tolerance};

fn foreign_sphere() -> zenith_topo::Solid {
    let path = PathBuf::from(concat!(env!("CARGO_MANIFEST_DIR"), "/tests/fixtures"))
        .join("occ_reference_sphere.step");
    let solids = StepImporter::import_solids_from_file(&path).expect("read the sphere");
    solids.into_iter().next().expect("one solid")
}

/// 検体が本当に「境界ワイヤを持たない全周1枚の面」であること。
///
/// ここが崩れると、この検体はもう当該の欠陥を測っていません。
#[test]
fn the_foreign_sphere_is_one_untrimmed_face() {
    let sphere = foreign_sphere();
    assert_eq!(sphere.outer_shell.faces.len(), 1);
    let face = &sphere.outer_shell.faces[0];
    assert!(face.outer_wire.edges.is_empty());
    assert!(face.inner_wires.is_empty());
}

#[test]
fn the_projection_finds_the_face_of_an_untrimmed_sphere() {
    let sphere = foreign_sphere();

    // 半径 10 の球。どこから測っても |P| との差が距離。
    for point in [
        Point3::new(0.0, 0.0, 0.0),
        Point3::new(20.0, 0.0, 0.0),
        Point3::new(3.0, 4.0, 1.0),
        Point3::new(-7.0, 2.0, -5.0),
    ] {
        let projection =
            nearest_boundary_projection(point, &sphere).expect("the sphere has a face to hit");
        let expected = (point.coords.norm() - 10.0).abs();
        assert!(
            (projection.distance - expected).abs() < 1e-9,
            "{point:?}: expected {expected}, got {}",
            projection.distance
        );
        // 足は球面の上。
        assert!((projection.foot.coords.norm() - 10.0).abs() < 1e-9);
    }
}

#[test]
fn the_untrimmed_sphere_says_which_points_are_inside() {
    let sphere = foreign_sphere();
    let tol = Tolerance::default();

    assert_eq!(exact_inside(Point3::new(0.0, 0.0, 0.0), &sphere, &tol), Some(true));
    assert_eq!(exact_inside(Point3::new(3.0, 4.0, 1.0), &sphere, &tol), Some(true));
    assert_eq!(exact_inside(Point3::new(20.0, 0.0, 0.0), &sphere, &tol), Some(false));
    assert_eq!(exact_inside(Point3::new(9.0, 9.0, 9.0), &sphere, &tol), Some(false));

    // 面のすぐ内と、すぐ外。**メッシュでは割れる幅**でも動かない。
    assert_eq!(exact_inside(Point3::new(9.9998, 0.0, 0.0), &sphere, &tol), Some(true));
    assert_eq!(exact_inside(Point3::new(10.0002, 0.0, 0.0), &sphere, &tol), Some(false));
}

/// 直方体の角の外。足はどの面のトリムにも落ちないので、境界の稜へ寄せる。
#[test]
fn a_point_off_the_corner_of_a_box_still_has_a_foot() {
    let unit = PrimitiveBuilder::make_box(9.0, 9.0, 9.0).expect("box");
    let tol = Tolerance::default();

    // 角 (9,9,9) の外へ 1,1,1 だけ出た点。最近点は角そのもの。
    let point = Point3::new(10.0, 10.0, 10.0);
    let projection = nearest_boundary_projection(point, &unit).expect("a foot on the box");
    assert!(
        (projection.distance - 3f64.sqrt()).abs() < 1e-6,
        "expected sqrt(3), got {}",
        projection.distance
    );
    assert_eq!(exact_inside(point, &unit, &tol), Some(false));

    // 稜の外。最近点は稜の上。
    let point = Point3::new(12.0, 12.0, 4.5);
    let projection = nearest_boundary_projection(point, &unit).expect("a foot on the box");
    assert!(
        (projection.distance - (2f64 * 9.0f64).sqrt()).abs() < 1e-6,
        "expected sqrt(18), got {}",
        projection.distance
    );
    assert_eq!(exact_inside(point, &unit, &tol), Some(false));

    // 中は中。
    assert_eq!(exact_inside(Point3::new(4.5, 4.5, 4.5), &unit, &tol), Some(true));
}

/// 回転面の**極**。最近点がちょうど極に来る配置。
///
/// 極では片方の偏微分が消えるので `NurbsSurface3::normal` は `None` を
/// 返します。それは点そのものについては正しいのですが、面はそこで滑らか
/// なので、**まわりからの極限**が採れます（`normal_or_limit`）。採らずに
/// いたとき、読んだ球の**中心線の上にある点は1つも足を持ちません**でした。
/// 中心線は実務でいくらでも出てきます。
#[test]
fn a_point_on_the_axis_of_a_read_sphere_has_a_foot() {
    let sphere = foreign_sphere();
    let tol = Tolerance::default();

    for (point, distance, inside) in [
        (Point3::new(0.0, 0.0, 25.0), 15.0, false),
        (Point3::new(0.0, 0.0, -25.0), 15.0, false),
        (Point3::new(0.0, 0.0, 5.0), 5.0, true),
        (Point3::new(0.0, 0.0, -5.0), 5.0, true),
    ] {
        let projection =
            nearest_boundary_projection(point, &sphere).expect("the pole is still a point of the face");
        assert!(
            (projection.distance - distance).abs() < 1e-9,
            "{point:?}: expected {distance}, got {}",
            projection.distance
        );
        assert_eq!(exact_inside(point, &sphere, &tol), Some(inside), "{point:?}");
    }
}

/// 2枚の面が縁でちょうど同着になる配置。
///
/// 読んだ円錐（r10 → 0、高さ 20）の底の縁の斜め外。底の平面から見れば内、
/// 母線から見れば外で、符号が割れます。**割れたら決めない**にしていた
/// ときは、境界から 11 も離れた点が「決められない」になっていました。
///
/// 稜が最近点になるのは凸稜なら外側の点だけなので、1枚でも「外」と言えば
/// 外で決まります。
#[test]
fn a_point_off_the_rim_of_a_read_cone_is_outside() {
    let path = PathBuf::from(concat!(env!("CARGO_MANIFEST_DIR"), "/tests/fixtures"))
        .join("occ_reference_cone_full.step");
    let cone = StepImporter::import_solids_from_file(&path)
        .expect("read the cone")
        .into_iter()
        .next()
        .expect("one solid");
    let tol = Tolerance::default();

    // 底の縁 (10, 0, 0) の斜め外。最近点は縁そのもの。
    let point = Point3::new(20.0, 0.0, 5.0);
    let projection = nearest_boundary_projection(point, &cone).expect("a foot on the cone");
    assert!(
        (projection.distance - 125f64.sqrt()).abs() < 1e-9,
        "expected sqrt(125), got {}",
        projection.distance
    );
    assert_eq!(exact_inside(point, &cone, &tol), Some(false));

    // 頂点の真上。最近点は頂点。
    let point = Point3::new(0.0, 0.0, 30.0);
    let projection = nearest_boundary_projection(point, &cone).expect("a foot on the cone");
    assert!(
        (projection.distance - 10.0).abs() < 1e-9,
        "expected 10, got {}",
        projection.distance
    );
    assert_eq!(exact_inside(point, &cone, &tol), Some(false));

    // 中は中。
    assert_eq!(exact_inside(Point3::new(0.0, 0.0, 10.0), &cone, &tol), Some(true));
}
