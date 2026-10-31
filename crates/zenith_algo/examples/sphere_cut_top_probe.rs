//! 周期1面の球の天辺を大きな箱で切ると、検証で断られる理由を見る検体（4-840）。
use zenith_algo::{BooleanEngine, BooleanOpType, BrepTransform, PrimitiveBuilder};
use zenith_math::{Tolerance, Vec3};

fn main() {
    let tol = Tolerance::default();
    let periodic = PrimitiveBuilder::make_sphere_periodic(10.0).unwrap();
    let tool = BrepTransform::translate_solid(
        &PrimitiveBuilder::make_box(30.0, 30.0, 30.0).unwrap(),
        Vec3::new(-15.0, -15.0, 4.0),
    );
    match BooleanEngine::boolean_solids_exact_result_unverified(&periodic, &tool, BooleanOpType::Intersection, &tol) {
        Ok(result) => {
            for solid in &result.solids {
                for (i, f) in solid.outer_shell.faces.iter().enumerate() {
                    let kind = if matches!(f.geometry, zenith_topo::FaceGeometry::Nurbs(_)) { "NURBS" } else { "平面" };
                    print!("面{i} {kind} 外周{}稜 穴{}", f.outer_wire.edges.len(), f.inner_wires.len());
                    match f.pcurves(&tol) {
                        Ok(p) => {
                            let ids: Vec<_> = f.outer_wire.edges.iter().map(|e| e.edge.id).collect();
                            print!(" 稜id{:?} 区間{}", ids, p.outer_loop.segments.len());
                            for seg in &p.outer_loop.segments {
                                let (t0, t1) = seg.curve.param_range();
                                let (a, b) = (seg.curve.evaluate(t0), seg.curve.evaluate(t1));
                                print!(" [({:.3},{:.3})→({:.3},{:.3})]", a.x, a.y, b.x, b.y);
                            }
                        }
                        Err(e) => print!(" p-curve無し: {e}"),
                    }
                    println!();
                }
            }
        }
        Err(e) => println!("断り: {e}"),
    }
}
