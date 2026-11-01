//! 穴の輪と外周の輪の「回る向き」を、uv の符号つき面積で測る（4-843）。
use zenith_algo::{BooleanEngine, BooleanOpType, BrepTransform, PrimitiveBuilder};
use zenith_math::{Point2, Tolerance, Vec3};

fn sample(l: &zenith_topo::FacePcurveLoop) -> Vec<Point2> {
    let mut poly = Vec::new();
    for seg in &l.segments {
        let (t0, t1) = seg.curve.param_range();
        for k in 0..24 {
            poly.push(seg.curve.evaluate(t0 + (t1 - t0) * k as f64 / 24.0));
        }
    }
    poly
}

fn signed_area(poly: &[Point2]) -> f64 {
    let mut a = 0.0;
    for i in 0..poly.len() {
        let p = poly[i];
        let q = poly[(i + 1) % poly.len()];
        a += p.x * q.y - q.x * p.y;
    }
    a / 2.0
}

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
    let face = res.solids[0]
        .outer_shell
        .faces
        .iter()
        .find(|f| matches!(f.geometry, zenith_topo::FaceGeometry::Nurbs(_)))
        .unwrap();
    let p = face.pcurves(&tol).unwrap();
    let outer = sample(&p.outer_loop);
    println!("外周: 点{} 符号つき面積{:.6}", outer.len(), signed_area(&outer));
    for (i, h) in p.inner_loops.iter().enumerate() {
        let poly = sample(h);
        println!("穴{i}: 点{} 符号つき面積{:.6}（閉じ弦のまま）", poly.len(), signed_area(&poly));
        // v=1（北極）を通して閉じたときと、v=0（南極）を通して閉じたとき
        for (name, vend) in [("北極v=1", 1.0), ("南極v=0", 0.0)] {
            let mut closed = poly.clone();
            let first = poly[0];
            let last = poly[poly.len() - 1];
            closed.push(Point2::new(last.x, vend));
            closed.push(Point2::new(first.x, vend));
            println!("  {name}を通して閉じると {:.6}", signed_area(&closed));
        }
        // uの跳びの位置
        for k in 0..poly.len() {
            let a = poly[k];
            let b = poly[(k + 1) % poly.len()];
            if (b.x - a.x).abs() > 0.5 {
                println!("  uの跳び: ({:.4},{:.4}) → ({:.4},{:.4})", a.x, a.y, b.x, b.y);
            }
        }
    }
}
