//! `PrimitiveBuilder::make_torus_periodic`・`make_cone_periodic`
//! （primitive.rsに追加した新しい並列の選択肢）が、単独でも正しい体積を
//! 返すかの最小確認。

use zenith_algo::{EdgeBlender, MassCalculator, PrimitiveBuilder};
use zenith_tess::TessellationParams;

fn main() {
    let params = TessellationParams { u_divisions: 64, v_divisions: 64 };

    let r_major = 12.0;
    let r_minor = 4.0;
    let torus = PrimitiveBuilder::make_torus_periodic(r_major, r_minor).unwrap();
    let torus_exact = 2.0 * std::f64::consts::PI.powi(2) * r_major * r_minor * r_minor;
    let torus_measured = MassCalculator::compute_volume_from_brep(&torus, &params);
    println!(
        "torus: faces={} volume={:.6} exact={:.6} rel_err={:.3e}",
        torus.outer_shell.faces.len(),
        torus_measured,
        torus_exact,
        (torus_measured - torus_exact).abs() / torus_exact
    );

    let r_bottom = 10.0;
    let height = 20.0;
    let cone = PrimitiveBuilder::make_cone_periodic(r_bottom, height).unwrap();
    let cone_exact = std::f64::consts::PI * r_bottom * r_bottom * height / 3.0;
    let cone_measured = MassCalculator::compute_volume_from_brep(&cone, &params);
    println!(
        "cone: faces={} volume={:.6} exact={:.6} rel_err={:.3e}",
        cone.outer_shell.faces.len(),
        cone_measured,
        cone_exact,
        (cone_measured - cone_exact).abs() / cone_exact
    );

    // **円錐の底縁をフィレットし、4分割版と比較**（4-814と同じ型）。
    let standard_cone = PrimitiveBuilder::make_cone(r_bottom, 0.0, height).unwrap();
    let standard_blendable = EdgeBlender::blendable_edges(&standard_cone);
    let periodic_blendable = EdgeBlender::blendable_edges(&cone);
    println!(
        "cone blendable edges: 4分割版={} 周期1面版={}",
        standard_blendable.len(),
        periodic_blendable.len()
    );
    if let (Some(sb), Some(pb)) = (standard_blendable.first(), periodic_blendable.first()) {
        let standard_filleted = EdgeBlender::fillet_edge(&standard_cone, sb.edge_id, 1.0);
        let periodic_filleted = EdgeBlender::fillet_edge(&cone, pb.edge_id, 1.0);
        match (standard_filleted, periodic_filleted) {
            (Ok(sa), Ok(pa)) => {
                let va = MassCalculator::compute_volume_from_brep(&sa, &params);
                let vb = MassCalculator::compute_volume_from_brep(&pa, &params);
                println!(
                    "cone fillet: 4分割版 体積={va:.6} / 周期1面版 体積={vb:.6} / 相対差={:.3e}",
                    (va - vb).abs() / va.abs().max(1e-9)
                );
            }
            (Err(e), _) => println!("4分割版のフィレットが断られた: {e}"),
            (_, Err(e)) => println!("周期1面版のフィレットが断られた: {e}"),
        }
    }
}
