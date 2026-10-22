//! `periodic_cylinder_boolean_probe`と同じ型の検証を、トーラス・円錐にも
//! 広げる（ROADMAP柱③。4-815の続き）。

use zenith_algo::{BooleanEngine, BooleanOpType, BrepTransform, MassCalculator, PrimitiveBuilder};
use zenith_math::{Tolerance, Vec3};
use zenith_tess::TessellationParams;
use zenith_topo::Solid;

fn run(name: &str, a: &Solid, standard_b: &Solid, periodic_b: &Solid, tol: &Tolerance, params: &TessellationParams) {
    for op in [
        BooleanOpType::Difference,
        BooleanOpType::Intersection,
        BooleanOpType::Union,
    ] {
        let standard_result = BooleanEngine::boolean_solids_exact(a, standard_b, op, tol);
        let periodic_result = BooleanEngine::boolean_solids_exact(a, periodic_b, op, tol);
        match (standard_result, periodic_result) {
            (Ok(ra), Ok(rb)) => {
                let va = MassCalculator::compute_volume_from_brep(&ra, params);
                let vb = MassCalculator::compute_volume_from_brep(&rb, params);
                let fa = ra.outer_shell.faces.len();
                let fb = rb.outer_shell.faces.len();
                println!(
                    "{name} {op:?}: 4分割版 体積={va:.6} 面{fa}枚 / 周期1面版 体積={vb:.6} 面{fb}枚 / 相対差={:.3e}",
                    (va - vb).abs() / va.abs().max(1e-9)
                );
            }
            (Err(ea), Err(eb)) => println!("{name} {op:?}: 両方とも断られた（{ea} / {eb}）"),
            (Ok(_), Err(eb)) => println!("{name} {op:?}: 4分割版は通ったが周期1面版が断られた: {eb}"),
            (Err(ea), Ok(_)) => println!("{name} {op:?}: 周期1面版は通ったが4分割版が断られた: {ea}"),
        }
    }
}

fn main() {
    let tol = Tolerance::default();
    let params = TessellationParams { u_divisions: 64, v_divisions: 64 };

    // トーラス × 箱（箱がトーラスの一部を貫く配置）。
    let r_major = 12.0;
    let r_minor = 4.0;
    let standard_torus = PrimitiveBuilder::make_torus(r_major, r_minor).unwrap();
    let periodic_torus = PrimitiveBuilder::make_torus_periodic(r_major, r_minor).unwrap();
    let box_for_torus = PrimitiveBuilder::make_box(10.0, 10.0, 10.0).unwrap();
    let box_for_torus = BrepTransform::translate_solid(
        &box_for_torus,
        Vec3::new(r_major - 5.0, -5.0, -5.0),
    );
    run(
        "箱×トーラス",
        &box_for_torus,
        &standard_torus,
        &periodic_torus,
        &tol,
        &params,
    );

    // 円錐 × 箱。
    let r_bottom = 10.0;
    let height = 20.0;
    let standard_cone = PrimitiveBuilder::make_cone(r_bottom, 0.0, height).unwrap();
    let periodic_cone = PrimitiveBuilder::make_cone_periodic(r_bottom, height).unwrap();
    let box_for_cone = PrimitiveBuilder::make_box(30.0, 30.0, 30.0).unwrap();
    let box_for_cone =
        BrepTransform::translate_solid(&box_for_cone, Vec3::new(-15.0, -15.0, 5.0));
    run(
        "箱×円錐",
        &box_for_cone,
        &standard_cone,
        &periodic_cone,
        &tol,
        &params,
    );
}
