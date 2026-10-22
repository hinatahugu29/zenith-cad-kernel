//! `PrimitiveBuilder::make_torus_periodic`・`make_cone_periodic`
//! （primitive.rsに追加した新しい並列の選択肢）が、単独でも正しい体積を
//! 返すかの最小確認。

use zenith_algo::{MassCalculator, PrimitiveBuilder};
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
}
