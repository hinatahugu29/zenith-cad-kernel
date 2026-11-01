//! 継ぎ目をまたぐ窓の uv を見る（4-841 の次の一手を測るため）。
//!
//! 箱8を (5,-4,-4) に置いた和で、周期1面の球の面に届く交線6本を集め、
//! 曲面の uv へ落として並べる。継ぎ目（u=0/1）をまたいでいるか、
//! またいでいるとして**巻きを戻すと小さな閉じた輪になるか**を測る。
use zenith_algo::{BrepIntersectionBuilder, BrepTransform, PrimitiveBuilder};
use zenith_math::{Tolerance, Vec3};
use zenith_topo::FaceGeometry;

fn main() {
    let tol = Tolerance::default();
    let sphere = PrimitiveBuilder::make_sphere_periodic(10.0).unwrap();
    let tool = BrepTransform::translate_solid(
        &PrimitiveBuilder::make_box(8.0, 8.0, 8.0).unwrap(),
        Vec3::new(5.0, -4.0, -4.0),
    );
    let faces_a = BrepIntersectionBuilder::all_faces_of(&sphere);
    let faces_b = BrepIntersectionBuilder::all_faces_of(&tool);
    let candidates = BrepIntersectionBuilder::collect_intersection_edge_candidates(&faces_a, &faces_b, &tol);
    println!("交線 {} 本", candidates.len());

    let FaceGeometry::Nurbs(surface) = &faces_a[0].geometry else {
        println!("面0が NURBS ではない");
        return;
    };
    let ((u_min, u_max), (v_min, v_max)) = surface.param_range();
    println!("uv 範囲: u {u_min}..{u_max} v {v_min}..{v_max}");

    // 力ずくで uv を引く（軽い格子＋局所の細分）。
    let foot = |p: zenith_math::Point3| -> (f64, f64) {
        let mut best = (u_min, v_min, f64::INFINITY);
        let (mut n, mut lo_u, mut hi_u, mut lo_v, mut hi_v) = (200, u_min, u_max, v_min, v_max);
        for _ in 0..3 {
            for i in 0..=n {
                let u = lo_u + (hi_u - lo_u) * i as f64 / n as f64;
                for j in 0..=n {
                    let v = lo_v + (hi_v - lo_v) * j as f64 / n as f64;
                    let d = (surface.evaluate(u, v) - p).norm();
                    if d < best.2 {
                        best = (u, v, d);
                    }
                }
            }
            let (du, dv) = ((hi_u - lo_u) / n as f64 * 2.0, (hi_v - lo_v) / n as f64 * 2.0);
            lo_u = best.0 - du;
            hi_u = best.0 + du;
            lo_v = best.1 - dv;
            hi_v = best.1 + dv;
            n = 40;
        }
        (best.0, best.1)
    };

    for (index, candidate) in candidates.iter().enumerate() {
        if candidate.face_a_index != 0 {
            continue;
        }
        let mut row = Vec::new();
        for k in 0..=4 {
            let (t0, t1) = candidate.edge.curve.param_range();
            let p = candidate.edge.evaluate(t0 + (t1 - t0) * k as f64 / 4.0);
            let (u, v) = foot(p);
            row.push(format!("({u:.4},{v:.4})"));
        }
        let (t0, t1) = candidate.edge.curve.param_range();
        let mid = candidate.edge.evaluate((t0 + t1) * 0.5);
        println!(
            "交線{index} 中点 ({:.3},{:.3},{:.3}) uv {}",
            mid.x,
            mid.y,
            mid.z,
            row.join(" ")
        );
    }
}
