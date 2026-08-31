//! Hunts for discontinuities in surface evaluation.
//!
//! # この探索の前提は外れていました（解決済み）
//!
//! 元の問いは「掃引パイプの側面が 131072 三角形でも収束しないのは、滑らかな
//! 曲面には起こりえないから、評価器が壊れているのではないか」でした。
//! 評価器は壊れていません。折れは**実在し、しかも正当**でした——下の出力が
//! 見せているとおり、この側面は v 方向に内部ノットを13本持ちます。B-spline が
//! 滑らかなのは各ノット区間の内側だけです。
//!
//! 直すべきだったのは曲面ではなく**積分のほう**で、三角形をノット線で割って
//! から求積するようにしました（4-20）。掃引パイプの面積はいま24分割から
//! 1e-12 の刻みで動きません。
//!
//! **「滑らかなはずのものが収束しないなら評価器が壊れている」は、
//! 「滑らかなはず」が正しいときにだけ成り立ちます。** ここではその前提の
//! ほうが違っていました。プローブは残してあります——評価器に本物の折れが
//! 入ったときには、やはりここに出ます。
//!
//! This walks the parameter domain at fine steps and reports the largest jump in
//! position and in tangent between neighbouring samples, relative to the local
//! step, so a break in the evaluator shows up as an outlier.
//!
//! Run with: cargo run --release -p zenith_algo --example surface_smoothness_probe

use zenith_algo::{PrimitiveBuilder, SweepBuilder};
use zenith_geom::NurbsCurve3;
use zenith_math::Point3;
use zenith_topo::{FaceGeometry, Solid};

fn scan(name: &str, solid: &Solid, face_index: usize) {
    let face = &solid.outer_shell.faces[face_index];
    let FaceGeometry::Nurbs(surface) = &face.geometry else {
        println!("{name}: face {face_index} is not a NURBS face");
        return;
    };

    let ((u_min, u_max), (v_min, v_max)) = surface.param_range();
    let samples = 2000;

    let mut worst_position_jump = 0.0f64;
    let mut worst_position_u = 0.0;
    let mut worst_tangent_jump = 0.0f64;
    let mut worst_tangent_u = 0.0;

    let v_mid = 0.5 * (v_min + v_max);
    let mut previous_point: Option<Point3> = None;
    let mut previous_tangent: Option<Point3> = None;

    for index in 0..=samples {
        let t = index as f64 / samples as f64;
        let u = u_min + (u_max - u_min) * t;
        let point = surface.evaluate(u, v_mid);

        if let Some(previous) = previous_point {
            let step = (point - previous).norm();
            if step > worst_position_jump {
                worst_position_jump = step;
                worst_position_u = u;
            }

            let tangent = Point3::from((point - previous) / ((u_max - u_min) / samples as f64));
            if let Some(previous_tangent) = previous_tangent {
                let change = (tangent - previous_tangent).norm();
                if change > worst_tangent_jump {
                    worst_tangent_jump = change;
                    worst_tangent_u = u;
                }
            }
            previous_tangent = Some(tangent);
        }
        previous_point = Some(point);
    }

    let expected_step = worst_position_jump; // reported directly for comparison
    println!("{name}, face {face_index}:");
    println!("    u range [{u_min}, {u_max}], sampled {samples} steps along v = {v_mid}");
    println!("    largest position step   {expected_step:.9e} near u = {worst_position_u:.6}");
    println!("    largest tangent change  {worst_tangent_jump:.9e} near u = {worst_tangent_u:.6}");

    // Same scan in v.
    let u_mid = 0.5 * (u_min + u_max);
    let mut worst_v_tangent = 0.0f64;
    let mut worst_v = 0.0;
    let mut previous_point = None;
    let mut previous_tangent: Option<Point3> = None;
    for index in 0..=samples {
        let t = index as f64 / samples as f64;
        let v = v_min + (v_max - v_min) * t;
        let point = surface.evaluate(u_mid, v);
        if let Some(previous) = previous_point {
            let tangent = Point3::from((point - previous) / ((v_max - v_min) / samples as f64));
            if let Some(previous_tangent) = previous_tangent {
                let change: f64 = (tangent - previous_tangent).norm();
                if change > worst_v_tangent {
                    worst_v_tangent = change;
                    worst_v = v;
                }
            }
            previous_tangent = Some(tangent);
        }
        previous_point = Some(point);
    }
    println!("    largest tangent change in v {worst_v_tangent:.9e} near v = {worst_v:.6}");
    println!();
}

fn main() {
    let cylinder = PrimitiveBuilder::make_cylinder(10.0, 40.0).unwrap();
    scan("cylinder (converges to 1e-10)", &cylinder, 0);

    let path = NurbsCurve3::bspline_from_points(
        3,
        vec![
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(10.0, 0.0, 10.0),
            Point3::new(20.0, 20.0, 25.0),
            Point3::new(30.0, 20.0, 40.0),
        ],
    )
    .unwrap();
    let pipe = SweepBuilder::sweep_circle_along_curve(&path, 3.5, 16).unwrap();
    scan("swept pipe side (does not converge)", &pipe, 0);

    if let FaceGeometry::Nurbs(surface) = &pipe.outer_shell.faces[0].geometry {
        println!("swept pipe side patch:");
        println!(
            "    degree_u = {}, degree_v = {}",
            surface.degree_u, surface.degree_v
        );
        println!(
            "    control grid {} x {}",
            surface.control_points.len(),
            surface.control_points[0].len()
        );
        println!("    u knots {:?}", surface.knots_u.knots);
        println!("    v knots {:?}", surface.knots_v.knots);
    }
}
