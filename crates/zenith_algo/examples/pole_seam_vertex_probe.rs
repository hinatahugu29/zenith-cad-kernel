//! 窓の輪が継ぎ目(y=0, x>0)を横切るとき、そこに頂点が立っているかを見る（4-844）。
use zenith_algo::{BooleanEngine, BooleanOpType, BrepTransform, PrimitiveBuilder};
use zenith_geom::Curve3;
use zenith_math::{Tolerance, Vec3};

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
    for (name, wire) in std::iter::once(("外周", &face.outer_wire))
        .chain(face.inner_wires.iter().enumerate().map(|(i, w)| {
            (if i == 0 { "穴0" } else { "穴N" }, w)
        }))
    {
        println!("--- {name}: {}稜 ---", wire.edges.len());
        for (i, oe) in wire.edges.iter().enumerate() {
            let c = &oe.edge.curve;
            let (t0, t1) = c.param_range();
            let (a, b) = (c.evaluate(t0), c.evaluate(t1));
            // この稜が継ぎ目(y=0, x>0)を横切るか、細かく見る
            let mut crossings = Vec::new();
            let mut prev = a;
            for k in 1..=200 {
                let q = c.evaluate(t0 + (t1 - t0) * k as f64 / 200.0);
                if prev.y.signum() != q.y.signum() && (prev.x > 0.0 || q.x > 0.0) {
                    crossings.push((prev.x + q.x) * 0.5);
                }
                prev = q;
            }
            println!(
                "  稜{i} id{} 始({:.4},{:.4},{:.4}) 終({:.4},{:.4},{:.4}) 継ぎ目横断{:?}",
                oe.edge.id, a.x, a.y, a.z, b.x, b.y, b.z,
                crossings.iter().map(|x| format!("x={x:.4}")).collect::<Vec<_>>()
            );
        }
    }
}
