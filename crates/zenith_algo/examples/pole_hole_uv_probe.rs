//! ＼の結果の穴の輪を uv で見て、極の点がその穴に入ると判定されるかを見る（4-843）。
use zenith_algo::{BooleanEngine, BooleanOpType, BrepTransform, PrimitiveBuilder};
use zenith_geom::Surface3;
use zenith_math::{Point3, Tolerance, Vec3};

fn main() {
    let tol = Tolerance::default();
    let tool = BrepTransform::translate_solid(
        &PrimitiveBuilder::make_box(8.0, 8.0, 8.0).unwrap(),
        Vec3::new(-4.0, -4.0, 5.0),
    );
    let sphere = PrimitiveBuilder::make_sphere_periodic(10.0).unwrap();
    let res = BooleanEngine::boolean_solids_exact_result_unverified(
        &sphere, &tool, BooleanOpType::Difference, &tol,
    )
    .unwrap();
    let solid = &res.solids[0];
    let face = solid
        .outer_shell
        .faces
        .iter()
        .find(|f| matches!(f.geometry, zenith_topo::FaceGeometry::Nurbs(_)))
        .unwrap();
    let p = face.pcurves(&tol).unwrap();
    let surface = match &face.geometry {
        zenith_topo::FaceGeometry::Nurbs(s) => s.clone(),
        _ => unreachable!(),
    };
    println!("支持曲面の範囲 {:?}", surface.param_range());
    for (name, loops) in [("外周", vec![p.outer_loop.clone()]), ("穴", p.inner_loops.clone())] {
        for (li, l) in loops.iter().enumerate() {
            let mut us: Vec<f64> = Vec::new();
            let mut vs: Vec<f64> = Vec::new();
            for seg in &l.segments {
                let (t0, t1) = seg.curve.param_range();
                for k in 0..=8 {
                    let q = seg.curve.evaluate(t0 + (t1 - t0) * k as f64 / 8.0);
                    us.push(q.x);
                    vs.push(q.y);
                }
            }
            let (umin, umax) = (us.iter().cloned().fold(f64::INFINITY, f64::min), us.iter().cloned().fold(f64::NEG_INFINITY, f64::max));
            let (vmin, vmax) = (vs.iter().cloned().fold(f64::INFINITY, f64::min), vs.iter().cloned().fold(f64::NEG_INFINITY, f64::max));
            let first = l.segments.first().map(|s| s.curve.evaluate(s.curve.param_range().0));
            let last = l.segments.last().map(|s| s.curve.evaluate(s.curve.param_range().1));
            println!(
                "{name}{li}: 区間{} u[{umin:.4},{umax:.4}] v[{vmin:.4},{vmax:.4}] 始{:?} 終{:?}",
                l.segments.len(),
                first.map(|q| (format!("{:.4}", q.x), format!("{:.4}", q.y))),
                last.map(|q| (format!("{:.4}", q.x), format!("{:.4}", q.y))),
            );
        }
    }
    // 食い違った点のうち3つを、支持曲面の uv に落とす
    for pt in [
        Point3::new(0.3750, 0.2469, 9.8400),
        Point3::new(-0.1875, -0.1152, 9.7200),
        Point3::new(3.0469, 0.2881, 9.1728),
    ] {
        let dir = Vec3::new(pt.x, pt.y, pt.z).normalize() * 10.0;
        let on = Point3::new(dir.x, dir.y, dir.z);
        // 支持曲面を細かく撒いて、いちばん近い uv を取る
        let ((u0, u1), (v0, v1)) = surface.param_range();
        let mut best = (f64::INFINITY, 0.0, 0.0);
        for i in 0..=400 {
            for j in 0..=400 {
                let u = u0 + (u1 - u0) * i as f64 / 400.0;
                let v = v0 + (v1 - v0) * j as f64 / 400.0;
                let q = surface.evaluate(u, v);
                let d = (q - on).norm();
                if d < best.0 {
                    best = (d, u, v);
                }
            }
        }
        println!(
            "点({:.4},{:.4},{:.4}) → 球面上 uv=({:.4},{:.4}) 距離{:.3e}",
            pt.x, pt.y, pt.z, best.1, best.2, best.0
        );
    }
}
