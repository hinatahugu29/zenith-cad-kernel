//! 重心と慣性が、閉じた式に乗るか。
//!
//! 体積と表面積は `builder_audit` が解析解と突き合わせていましたが、**重心と
//! 慣性はどこも突き合わせていません**でした。慣性は体積より一段高い次数の
//! 積分なので、体積が通る精度で通るとは限りません。
//!
//! さらに `inertia_diagonal` は「慣性モーメント主成分」と書かれていました。
//! 実際に返るのは**原点を通る座標軸まわりの対角成分**で、主慣性モーメント
//! ではありません。対称でない形ではこの2つは一致しないので、そのまま主値
//! として使うと答えが変わります。ここで両方を別々に測ります。

use std::f64::consts::PI;

use zenith_algo::{BrepTransform, MassCalculator, PrimitiveBuilder};
use zenith_math::{Point3, Transform3, Vec3};
use zenith_tess::TessellationParams;

fn params() -> TessellationParams {
    TessellationParams {
        u_divisions: 48,
        v_divisions: 48,
    }
}

fn close(actual: f64, expected: f64, relative: f64) -> bool {
    (actual - expected).abs() / expected.abs().max(1e-12) < relative
}

#[test]
fn the_diagonal_is_measured_about_the_origin_not_the_centre_of_mass() {
    // 原点を角に置いた直方体。原点まわりなら V(b^2+c^2)/3、重心まわりなら
    // V(b^2+c^2)/12。4倍違うので、どちらを返しているか一意に決まる。
    let (a, b, c) = (20.0f64, 30.0, 40.0);
    let volume = a * b * c;
    let measured =
        MassCalculator::compute_from_brep(&PrimitiveBuilder::make_box(a, b, c).unwrap(), &params());

    assert!(
        close(
            measured.inertia_diagonal.x,
            volume * (b * b + c * c) / 3.0,
            1e-12
        ),
        "Ixx {} is not the value about the origin",
        measured.inertia_diagonal.x
    );
    assert!(
        close(
            measured.inertia_diagonal.y,
            volume * (a * a + c * c) / 3.0,
            1e-12
        ),
        "Iyy {}",
        measured.inertia_diagonal.y
    );
    assert!(
        close(
            measured.inertia_diagonal.z,
            volume * (a * a + b * b) / 3.0,
            1e-12
        ),
        "Izz {}",
        measured.inertia_diagonal.z
    );
}

#[test]
fn the_centre_of_mass_of_a_cone_sits_a_quarter_of_the_way_up() {
    let (radius, height) = (10.0f64, 20.0);
    let measured = MassCalculator::compute_from_brep(
        &PrimitiveBuilder::make_cone(radius, 0.0, height).unwrap(),
        &params(),
    );

    assert!(
        (measured.center_of_mass - Point3::new(0.0, 0.0, height * 0.25)).norm() < 1e-9,
        "a cone's centre of mass is a quarter of the way up: {:?}",
        measured.center_of_mass
    );

    // 底面中心まわり: Izz = 3 V R^2 / 10、Ixx = V(3R^2/20 + H^2/10)
    let volume = PI * radius * radius * height / 3.0;
    assert!(
        close(
            measured.inertia_diagonal.z,
            3.0 * volume * radius * radius / 10.0,
            1e-11
        ),
        "cone Izz {}",
        measured.inertia_diagonal.z
    );
    assert!(
        close(
            measured.inertia_diagonal.x,
            volume * (3.0 * radius * radius / 20.0 + height * height / 10.0),
            1e-11
        ),
        "cone Ixx {}",
        measured.inertia_diagonal.x
    );
}

#[test]
fn a_curved_solid_lands_on_the_closed_form_for_its_inertia() {
    // 球と円柱。曲面の積分が体積だけでなく慣性でも効いているかを見る。
    let radius = 12.0f64;
    let volume = 4.0 / 3.0 * PI * radius.powi(3);
    let sphere = MassCalculator::compute_from_brep(
        &PrimitiveBuilder::make_sphere(radius).unwrap(),
        &params(),
    );
    for axis in 0..3 {
        assert!(
            close(
                sphere.inertia_diagonal[axis],
                0.4 * volume * radius * radius,
                1e-11
            ),
            "sphere axis {axis}: {}",
            sphere.inertia_diagonal[axis]
        );
    }

    let (r, h) = (10.0f64, 25.0);
    let volume = PI * r * r * h;
    let cylinder = MassCalculator::compute_from_brep(
        &PrimitiveBuilder::make_cylinder(r, h).unwrap(),
        &params(),
    );
    assert!(
        close(cylinder.inertia_diagonal.z, volume * r * r * 0.5, 1e-11),
        "cylinder Izz {}",
        cylinder.inertia_diagonal.z
    );
    assert!(
        close(
            cylinder.inertia_diagonal.x,
            volume * (r * r * 0.25 + h * h / 3.0),
            1e-11
        ),
        "cylinder Ixx {}",
        cylinder.inertia_diagonal.x
    );
}

#[test]
fn the_products_of_inertia_land_on_the_closed_form() {
    let (a, b, c) = (20.0f64, 30.0, 40.0);
    let volume = a * b * c;
    let measured =
        MassCalculator::compute_from_brep(&PrimitiveBuilder::make_box(a, b, c).unwrap(), &params());

    // 原点を角に置いた直方体: ∫xy dV = V a b / 4
    for (value, expected, name) in [
        (measured.inertia_products.x, volume * a * b / 4.0, "xy"),
        (measured.inertia_products.y, volume * b * c / 4.0, "yz"),
        (measured.inertia_products.z, volume * c * a / 4.0, "zx"),
    ] {
        assert!(
            close(value, expected, 1e-12),
            "product {name}: {value} against {expected}"
        );
    }

    // 原点を中心に置けば、対称性から慣性積は 0
    let centred = BrepTransform::translate_solid(
        &PrimitiveBuilder::make_box(a, b, c).unwrap(),
        Vec3::new(-a * 0.5, -b * 0.5, -c * 0.5),
    );
    let centred = MassCalculator::compute_from_brep(&centred, &params());
    for axis in 0..3 {
        assert!(
            centred.inertia_products[axis].abs() < 1e-6 * volume,
            "a box centred on the origin should have no products: {:?}",
            centred.inertia_products
        );
    }
}

#[test]
fn the_principal_moments_do_not_change_when_the_solid_is_moved_or_turned() {
    // 主慣性モーメントは剛体変換で不変。テンソルの組み立て・平行軸・固有値の
    // どこか1つでも間違っていれば、回した瞬間に値が動く。
    let (a, b, c) = (20.0f64, 30.0, 40.0);
    let volume = a * b * c;
    let mut expected = [
        volume * (b * b + c * c) / 12.0,
        volume * (a * a + c * c) / 12.0,
        volume * (a * a + b * b) / 12.0,
    ];
    expected.sort_by(f64::total_cmp);

    let boxed = PrimitiveBuilder::make_box(a, b, c).unwrap();
    let subjects = [
        ("as built", boxed.clone()),
        (
            "moved far away",
            BrepTransform::translate_solid(&boxed, Vec3::new(137.0, -44.0, 9.5)),
        ),
        (
            "turned about a skew axis",
            BrepTransform::transform_solid(
                &boxed,
                &Transform3::from_axis_angle(&Vec3::new(1.0, 2.0, 3.0), 35.0f64.to_radians()),
            )
            .unwrap(),
        ),
    ];

    for (name, solid) in subjects {
        let principal = MassCalculator::compute_from_brep(&solid, &params()).principal_moments();
        for axis in 0..3 {
            assert!(
                close(principal[axis], expected[axis], 1e-11),
                "{name}, axis {axis}: {} against {}",
                principal[axis],
                expected[axis]
            );
        }
    }
}

#[test]
fn the_tensor_can_be_moved_to_any_point() {
    let (a, b, c) = (10.0f64, 10.0, 10.0);
    let volume = a * b * c;
    let boxed = PrimitiveBuilder::make_box(a, b, c).unwrap();
    let measured = MassCalculator::compute_from_brep(&boxed, &params());

    // 原点まわりへ移し直すと、元の対角成分に戻る
    let back = measured.inertia_tensor_about(Point3::origin());
    for axis in 0..3 {
        assert!(
            close(back[axis][axis], measured.inertia_diagonal[axis], 1e-10),
            "moving the tensor back to the origin changed it: {} against {}",
            back[axis][axis],
            measured.inertia_diagonal[axis]
        );
    }

    // 重心まわりは V(b^2+c^2)/12、慣性積は 0
    let centre = measured.inertia_tensor_about_center_of_mass();
    assert!(
        close(centre[0][0], volume * (b * b + c * c) / 12.0, 1e-10),
        "about the centre of mass: {}",
        centre[0][0]
    );
    for row in 0..3 {
        for column in 0..3 {
            if row != column {
                assert!(
                    centre[row][column].abs() < 1e-6 * volume,
                    "a box about its own centre has no products: {}",
                    centre[row][column]
                );
            }
        }
    }
}
