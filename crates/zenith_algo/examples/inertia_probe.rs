//! 重心と慣性モーメントが、閉じた式に乗るか。
//!
//! 体積と表面積は `builder_audit` が解析解と突き合わせている。**重心と慣性は
//! どこも突き合わせていない**。体積が合っていても重心が合っているとは限らず、
//! 慣性はさらに一段高い次数の積分なので、体積が通る精度で通るとは限らない。
//!
//! 慣性は「どの点まわりか」で値が変わる。ここでは `MassProperties` が返す値が
//! **原点まわり**なのか**重心まわり**なのかも、閉じた式との一致で判定する。

use zenith_algo::{BrepTransform, MassCalculator, PrimitiveBuilder};
use zenith_math::{Point3, Vec3};
use zenith_tess::TessellationParams;
use zenith_topo::Solid;

struct Subject {
    name: &'static str,
    solid: Solid,
    /// 体積の閉じた式
    volume: f64,
    /// 重心の閉じた式
    centroid: Point3,
    /// 原点まわりの (Ixx, Iyy, Izz)（密度1）
    inertia_about_origin: Vec3,
}

fn main() {
    let params = TessellationParams {
        u_divisions: 64,
        v_divisions: 64,
    };

    let mut subjects: Vec<Subject> = Vec::new();

    // 直方体: 原点を角に置く。原点まわり Ixx = V (b^2 + c^2) / 3。
    let (a, b, c) = (20.0f64, 30.0, 40.0);
    let v = a * b * c;
    subjects.push(Subject {
        name: "box 20x30x40 (corner at origin)",
        solid: PrimitiveBuilder::make_box(a, b, c).unwrap(),
        volume: v,
        centroid: Point3::new(a * 0.5, b * 0.5, c * 0.5),
        inertia_about_origin: Vec3::new(
            v * (b * b + c * c) / 3.0,
            v * (a * a + c * c) / 3.0,
            v * (a * a + b * b) / 3.0,
        ),
    });

    // 円柱: 軸 +Z、底面中心が原点。原点まわり Izz = V r^2 / 2、
    // Ixx = Iyy = V (r^2 / 4 + h^2 / 3)。
    let (r, h) = (10.0f64, 25.0);
    let v = std::f64::consts::PI * r * r * h;
    subjects.push(Subject {
        name: "cylinder r10 h25 (base at origin)",
        solid: PrimitiveBuilder::make_cylinder(r, h).unwrap(),
        volume: v,
        centroid: Point3::new(0.0, 0.0, h * 0.5),
        inertia_about_origin: Vec3::new(
            v * (r * r * 0.25 + h * h / 3.0),
            v * (r * r * 0.25 + h * h / 3.0),
            v * r * r * 0.5,
        ),
    });

    // 球: 中心が原点。原点まわり = 重心まわり = 2 V R^2 / 5。
    let radius = 12.0f64;
    let v = 4.0 / 3.0 * std::f64::consts::PI * radius.powi(3);
    subjects.push(Subject {
        name: "sphere r12 (centred on origin)",
        solid: PrimitiveBuilder::make_sphere(radius).unwrap(),
        volume: v,
        centroid: Point3::new(0.0, 0.0, 0.0),
        inertia_about_origin: Vec3::new(
            0.4 * v * radius * radius,
            0.4 * v * radius * radius,
            0.4 * v * radius * radius,
        ),
    });

    // 円錐: 底面半径 R、高さ H、底面中心が原点。重心は底から H/4。
    // 原点まわり Izz = 3 V R^2 / 10、Ixx = V (3 R^2 / 20 + 3 H^2 / 10)。
    let (cr, ch) = (10.0f64, 20.0);
    let v = std::f64::consts::PI * cr * cr * ch / 3.0;
    subjects.push(Subject {
        name: "cone r10 h20 (base at origin)",
        solid: PrimitiveBuilder::make_cone(cr, 0.0, ch).unwrap(),
        volume: v,
        centroid: Point3::new(0.0, 0.0, ch * 0.25),
        inertia_about_origin: Vec3::new(
            // 底面中心まわり: 頂点まわり m(3R^2/20 + 3H^2/5) を重心 (底から H/4) 経由で
            // 移すと m(3R^2/20 + H^2/10)。頂点まわりの式をそのまま使うと 59% ずれる。
            v * (3.0 * cr * cr / 20.0 + ch * ch / 10.0),
            v * (3.0 * cr * cr / 20.0 + ch * ch / 10.0),
            3.0 * v * cr * cr / 10.0,
        ),
    });

    // 原点から離した直方体。平行軸の定理が効くかを見る。
    let (a, b, c) = (10.0f64, 10.0, 10.0);
    let v = a * b * c;
    let shift = Vec3::new(50.0, 0.0, 0.0);
    let moved =
        BrepTransform::translate_solid(&PrimitiveBuilder::make_box(a, b, c).unwrap(), shift);
    // 原点まわり Ixx は動かない（x 方向の移動は y, z を変えない）
    // Iyy, Izz は V d^2 だけ増える（d は重心の x 座標）
    let centre = Point3::new(50.0 + a * 0.5, b * 0.5, c * 0.5);
    subjects.push(Subject {
        name: "box 10^3 moved 50 along x",
        solid: moved,
        volume: v,
        centroid: centre,
        inertia_about_origin: Vec3::new(
            v * (b * b + c * c) / 3.0,
            v * (c * c / 3.0) + v * (centre.x * centre.x + a * a / 12.0),
            v * (b * b / 3.0) + v * (centre.x * centre.x + a * a / 12.0),
        ),
    });

    println!(
        "{:<34}{:>12}{:>12}{:>14}{:>14}{:>14}",
        "subject", "volume", "centroid", "Ixx", "Iyy", "Izz"
    );
    println!("{}", "-".repeat(102));

    let mut worst_volume: f64 = 0.0;
    let mut worst_centroid: f64 = 0.0;
    let mut worst_inertia: f64 = 0.0;

    for subject in &subjects {
        let measured = MassCalculator::compute_from_brep(&subject.solid, &params);

        let volume_error = (measured.volume - subject.volume).abs() / subject.volume.abs();
        let centroid_error = (measured.center_of_mass - subject.centroid).norm()
            / subject.centroid.coords.norm().max(1.0);
        let inertia_error = [
            (measured.inertia_diagonal.x - subject.inertia_about_origin.x).abs()
                / subject.inertia_about_origin.x.abs(),
            (measured.inertia_diagonal.y - subject.inertia_about_origin.y).abs()
                / subject.inertia_about_origin.y.abs(),
            (measured.inertia_diagonal.z - subject.inertia_about_origin.z).abs()
                / subject.inertia_about_origin.z.abs(),
        ];

        println!(
            "{:<34}{:>12.2e}{:>12.2e}{:>14.2e}{:>14.2e}{:>14.2e}",
            subject.name,
            volume_error,
            centroid_error,
            inertia_error[0],
            inertia_error[1],
            inertia_error[2]
        );

        worst_volume = worst_volume.max(volume_error);
        worst_centroid = worst_centroid.max(centroid_error);
        worst_inertia = worst_inertia.max(inertia_error.iter().cloned().fold(0.0, f64::max));
    }

    println!("{}", "-".repeat(102));
    println!("relative error against the closed form (about the ORIGIN for the inertia)");
    println!("  worst volume   {worst_volume:.2e}");
    println!("  worst centroid {worst_centroid:.2e}");
    println!("  worst inertia  {worst_inertia:.2e}");

    if worst_inertia > 1e-9 {
        println!();
        println!("The inertia does not land on the closed form. Either it is measured about");
        println!("a different point than the origin, or the integral itself is wrong.");
        std::process::exit(1);
    }

    check_products_and_principal_moments();
}

/// 慣性積と主慣性モーメントが閉じた式に乗るか。
///
/// 対角成分だけでは、対称でない形の慣性は表せない。原点から離した直方体を
/// 使うと、慣性積が 0 でない値を取り、主慣性モーメント（重心まわりの固有値）
/// とは明確に食い違う。両方が正しいかを別々に測る。
fn check_products_and_principal_moments() {
    let params = TessellationParams {
        u_divisions: 48,
        v_divisions: 48,
    };
    let mut worst: f64 = 0.0;

    println!();
    println!("products of inertia and principal moments");
    println!("{}", "-".repeat(102));

    // 原点を角に置いた直方体: ∫xy dV = V a b / 4 など
    let (a, b, c) = (20.0f64, 30.0, 40.0);
    let v = a * b * c;
    let boxed = PrimitiveBuilder::make_box(a, b, c).unwrap();
    let measured = MassCalculator::compute_from_brep(&boxed, &params);
    let expected = Vec3::new(v * a * b / 4.0, v * b * c / 4.0, v * c * a / 4.0);
    for axis in 0..3 {
        let error = (measured.inertia_products[axis] - expected[axis]).abs() / expected[axis].abs();
        worst = worst.max(error);
    }
    println!(
        "box corner at origin: products {:>12.4} {:>12.4} {:>12.4}  (want {:.4} {:.4} {:.4})",
        measured.inertia_products.x,
        measured.inertia_products.y,
        measured.inertia_products.z,
        expected.x,
        expected.y,
        expected.z
    );

    // 同じ直方体の主慣性モーメント: 重心まわりは V(b^2+c^2)/12 など。
    // 座標軸に平行なので主軸も座標軸に一致する。
    let mut want = [
        v * (b * b + c * c) / 12.0,
        v * (a * a + c * c) / 12.0,
        v * (a * a + b * b) / 12.0,
    ];
    want.sort_by(f64::total_cmp);
    let principal = measured.principal_moments();
    for axis in 0..3 {
        let error = (principal[axis] - want[axis]).abs() / want[axis].abs();
        worst = worst.max(error);
    }
    println!(
        "box principal moments {:>14.4} {:>14.4} {:>14.4}  (want {:.4} {:.4} {:.4})",
        principal.x, principal.y, principal.z, want[0], want[1], want[2]
    );

    // 45 度まわした直方体。慣性積は 0 でなくなるが、**主慣性モーメントは
    // 回しても変わらない**。回転不変であることが、テンソルの扱いが正しい
    // ことの一番強い証拠になる。
    let turned = BrepTransform::transform_solid(
        &PrimitiveBuilder::make_box(a, b, c).unwrap(),
        &zenith_math::Transform3::from_axis_angle(&Vec3::new(1.0, 2.0, 3.0), 35.0f64.to_radians()),
    )
    .unwrap();
    let turned_measured = MassCalculator::compute_from_brep(&turned, &params);
    let turned_principal = turned_measured.principal_moments();
    for axis in 0..3 {
        let error = (turned_principal[axis] - want[axis]).abs() / want[axis].abs();
        worst = worst.max(error);
    }
    println!(
        "turned box principal  {:>14.4} {:>14.4} {:>14.4}  (must not change)",
        turned_principal.x, turned_principal.y, turned_principal.z
    );

    // 平行移動しても主慣性モーメントは変わらない
    let moved = BrepTransform::translate_solid(&boxed, Vec3::new(137.0, -44.0, 9.5));
    let moved_principal = MassCalculator::compute_from_brep(&moved, &params).principal_moments();
    for axis in 0..3 {
        let error = (moved_principal[axis] - want[axis]).abs() / want[axis].abs();
        worst = worst.max(error);
    }
    println!(
        "moved box principal   {:>14.4} {:>14.4} {:>14.4}  (must not change)",
        moved_principal.x, moved_principal.y, moved_principal.z
    );

    println!("{}", "-".repeat(102));
    println!("  worst products / principal moments {worst:.2e}");
    if worst > 1e-9 {
        println!();
        println!("The inertia tensor does not survive being moved or turned.");
        std::process::exit(1);
    }
}
