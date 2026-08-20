//! 曲面同士の交線が、本当に両方の曲面の上にあるか。
//!
//! ここは引継書の 3-1（真の SSI）の入口である。交線が取れることと、
//! それで面を割れることは別の問題で、割るほうは
//! [`zenith_algo::FaceSplitter`] と `face_split_probe` が受け持つ。
//!
//! 測り方に一つ気をつける点がある。**辿るのに使った (u, v) で測っては
//! ならない。** そこは構成上ぴったりなので、何を作っても 0 が出る。
//! 点を改めて曲面へ射影して距離を測る。
//!
//! 解析解のある配置では、交線そのものも突き合わせる。半径の等しい2本の
//! 直交円柱（Steinmetz）の交線は**平面上の楕円ちょうど2本**なので、辿った点が
//! 1枚の平面に乗るかを測れば、形まで確かめられる。
//!
//! 走らせ方: cargo run --release -p zenith_algo --example ssi_probe

use std::f64::consts::FRAC_1_SQRT_2;

use zenith_geom::{ControlPoint3, IntersectionMarcher, KnotVector, NurbsSurface3};
use zenith_math::{Point3, Tolerance, Vec3};

/// 軸 `axis` まわり、半径 `r`、`along` 方向に長さ `length` の円柱の
/// 四半パッチ。`quadrant` で 90 度ずつ回す。
fn cylinder_patch(
    r: f64,
    length: f64,
    axis: Vec3,
    x_axis: Vec3,
    origin: Point3,
    quadrant: usize,
) -> NurbsSurface3 {
    let w = FRAC_1_SQRT_2;
    let y_axis = axis.cross(&x_axis).normalize();
    let angle = std::f64::consts::FRAC_PI_2 * quadrant as f64;
    let at = |theta: f64, scale: f64| {
        origin + (x_axis * theta.cos() + y_axis * theta.sin()) * (r * scale)
    };
    let corner = {
        let next = angle + std::f64::consts::FRAC_PI_2;
        origin
            + (x_axis * (angle.cos() + next.cos()) + y_axis * (angle.sin() + next.sin())) * r
    };
    let ring = [
        (at(angle, 1.0), 1.0),
        (corner, w),
        (at(angle + std::f64::consts::FRAC_PI_2, 1.0), 1.0),
    ];
    let grid: Vec<Vec<ControlPoint3>> = ring
        .iter()
        .map(|(point, weight)| {
            vec![
                ControlPoint3::new(*point - axis * (length * 0.5), *weight),
                ControlPoint3::new(*point + axis * (length * 0.5), *weight),
            ]
        })
        .collect();
    NurbsSurface3::new(
        2,
        1,
        grid,
        KnotVector::clamped_uniform(3, 2),
        KnotVector::clamped_uniform(2, 1),
    )
    .unwrap()
}

fn sphere_patch(r: f64, centre: Point3) -> NurbsSurface3 {
    // 北半球の第1象限（経度 0..90 度、緯度 0..90 度）
    let w = FRAC_1_SQRT_2;
    let rows = [
        // 赤道の行、45 度の行（重み付き）、極の行
        (0.0, 1.0),
        (1.0, w),
        (2.0, 1.0),
    ];
    let grid: Vec<Vec<ControlPoint3>> = rows
        .iter()
        .map(|(index, weight_u)| {
            let (radial, height) = match *index as usize {
                0 => (r, 0.0),
                1 => (r, r),
                _ => (0.0, r),
            };
            vec![
                ControlPoint3::new(centre + Vec3::new(radial, 0.0, height), *weight_u),
                ControlPoint3::new(
                    centre + Vec3::new(radial, radial, height),
                    weight_u * w,
                ),
                ControlPoint3::new(centre + Vec3::new(0.0, radial, height), *weight_u),
            ]
        })
        .collect();
    NurbsSurface3::new(
        2,
        2,
        grid,
        KnotVector::clamped_uniform(3, 2),
        KnotVector::clamped_uniform(3, 2),
    )
    .unwrap()
}

struct Subject {
    name: &'static str,
    a: NurbsSurface3,
    b: NurbsSurface3,
    /// 交線が1枚の平面に乗るはずなら true（等半径の直交円柱など）。
    planar: bool,
}

/// 点列が1枚の平面からどれだけ外れているか。
///
/// 平面は**最小二乗**で取る。離れた3点から法線を作る書き方だと、その3点の
/// 取り方しだいで値が変わり、実際に一度 1.8e-5 という誤った値を見た（真の値は
/// 1e-11 台）。測り方が答えを作ってはいけない。
fn planarity(points: &[Point3]) -> f64 {
    if points.len() < 4 {
        return 0.0;
    }
    let centroid = points
        .iter()
        .fold(Vec3::new(0.0, 0.0, 0.0), |sum, p| sum + p.coords)
        / points.len() as f64;
    let mut covariance = nalgebra::Matrix3::<f64>::zeros();
    for point in points {
        let d = point.coords - centroid;
        covariance += d * d.transpose();
    }
    let eigen = nalgebra::SymmetricEigen::new(covariance);
    let mut smallest = 0usize;
    for index in 1..3 {
        if eigen.eigenvalues[index] < eigen.eigenvalues[smallest] {
            smallest = index;
        }
    }
    let normal = eigen.eigenvectors.column(smallest).into_owned();
    points.iter().fold(0.0f64, |worst, point| {
        worst.max((point.coords - centroid).dot(&normal).abs())
    })
}

fn main() {
    let tol = Tolerance::default();
    let mut subjects = Vec::new();

    // 半径の等しい直交円柱。交線は平面上の楕円ちょうど2本になる。
    subjects.push(Subject {
        name: "cylinder x cylinder, equal radii, perpendicular",
        a: cylinder_patch(
            10.0,
            60.0,
            Vec3::new(0.0, 0.0, 1.0),
            Vec3::new(1.0, 0.0, 0.0),
            Point3::new(0.0, 0.0, 0.0),
            0,
        ),
        b: cylinder_patch(
            10.0,
            60.0,
            Vec3::new(1.0, 0.0, 0.0),
            Vec3::new(0.0, 1.0, 0.0),
            Point3::new(0.0, 0.0, 0.0),
            0,
        ),
        planar: true,
    });

    // 半径が違う直交円柱。交線は平面には乗らない（4次曲線）。
    subjects.push(Subject {
        name: "cylinder x cylinder, unequal radii",
        a: cylinder_patch(
            10.0,
            60.0,
            Vec3::new(0.0, 0.0, 1.0),
            Vec3::new(1.0, 0.0, 0.0),
            Point3::new(0.0, 0.0, 0.0),
            0,
        ),
        b: cylinder_patch(
            6.0,
            60.0,
            Vec3::new(1.0, 0.0, 0.0),
            Vec3::new(0.0, 1.0, 0.0),
            Point3::new(0.0, 0.0, 0.0),
            0,
        ),
        planar: false,
    });

    // 球と円柱。円柱の軸が球の中心を外すので、交線は平面に乗らない。
    subjects.push(Subject {
        name: "sphere x cylinder, offset axis",
        a: sphere_patch(12.0, Point3::new(0.0, 0.0, 0.0)),
        b: cylinder_patch(
            5.0,
            40.0,
            Vec3::new(0.0, 0.0, 1.0),
            Vec3::new(1.0, 0.0, 0.0),
            Point3::new(4.0, 0.0, 0.0),
            0,
        ),
        planar: false,
    });

    println!(
        "{:<48} {:>7} {:>7} {:>7} {:>9} {:>13} {:>14}",
        "subject", "points", "closed", "edge", "tangency", "off surface", "out of plane"
    );
    println!("{}", "-".repeat(112));

    let mut clean = 0usize;
    let mut problems = 0usize;

    for subject in &subjects {
        // 種は自分で選ばせる。渡す位置しだいで接点へ落ちる配置があり、
        // そこでは残差を詰めても交線上の位置が決まらない。
        match IntersectionMarcher::march_from_best_seed(&subject.a, &subject.b, 16, 0.5, 4096, &tol)
        {
            Some(curve) => {
                let points: Vec<Point3> = curve.points.iter().map(|p| p.point).collect();
                let flatness = planarity(&points);
                let bad = curve.worst_off_surface > 1e-9
                    || points.len() < 4
                    || (subject.planar && flatness > 1e-9);
                if bad {
                    problems += 1;
                } else {
                    clean += 1;
                }
                println!(
                    "{:<48} {:>7} {:>7} {:>7} {:>9} {:>13.3e} {:>14}",
                    subject.name,
                    points.len(),
                    curve.closed,
                    curve.stopped_at_boundary,
                    curve.stopped_at_tangency,
                    curve.worst_off_surface,
                    if subject.planar {
                        format!("{flatness:.3e}")
                    } else {
                        format!("({flatness:.3e})")
                    }
                );
            }
            None => {
                problems += 1;
                println!("{:<48} {:>7}  no curve found", subject.name, "-");
            }
        }
    }

    println!("{}", "-".repeat(112));
    println!(
        "{clean} of {} intersections are clean, {problems} with problems",
        subjects.len()
    );
    println!();
    println!("off surface = the marched points re-projected onto both surfaces, not the");
    println!("              (u, v) they were marched with, which is exact by construction");
    println!("out of plane = distance from the least-squares plane through the curve; in brackets");
    println!("               when the curve is not expected to be planar, so it is context");
    println!("tangency    = the march stopped before the normals became parallel. There the");
    println!("               residual no longer pins the position: for equal-radius cylinders a");
    println!("               point 2.99e-5 off the curve satisfies both surfaces to 2.24e-11.");
}
