//! `periodic_cylinder_boolean_probe`と同じ型の検証を球にも広げる
//! （ROADMAP柱③。4-820の続き）。

use zenith_algo::{BooleanEngine, BooleanOpType, BrepTransform, MassCalculator, PrimitiveBuilder};
use zenith_math::{Tolerance, Vec3};
use zenith_tess::TessellationParams;

fn main() {
    let tol = Tolerance::default();
    let params = TessellationParams { u_divisions: 48, v_divisions: 48 };
    let radius = 10.0;

    let standard = PrimitiveBuilder::make_sphere(radius).unwrap();
    let periodic = PrimitiveBuilder::make_sphere_periodic(radius).unwrap();
    let box_solid = PrimitiveBuilder::make_box(12.0, 12.0, 12.0).unwrap();
    let box_centered = BrepTransform::translate_solid(&box_solid, Vec3::new(-6.0, -6.0, -6.0));

    for op in [BooleanOpType::Difference, BooleanOpType::Intersection, BooleanOpType::Union] {
        let standard_result = BooleanEngine::boolean_solids_exact(&box_centered, &standard, op, &tol);
        let periodic_result = BooleanEngine::boolean_solids_exact(&box_centered, &periodic, op, &tol);
        match (standard_result, periodic_result) {
            (Ok(a), Ok(b)) => {
                let va = MassCalculator::compute_volume_from_brep(&a, &params);
                let vb = MassCalculator::compute_volume_from_brep(&b, &params);
                let fa = a.outer_shell.faces.len();
                let fb = b.outer_shell.faces.len();
                println!(
                    "{op:?}: 4分割版 体積={va:.6} 面{fa}枚 / 周期1面版 体積={vb:.6} 面{fb}枚 / 相対差={:.3e}",
                    (va - vb).abs() / va.abs().max(1e-9)
                );
            }
            (Err(ea), Err(eb)) => println!("{op:?}: 両方とも断られた（{ea} / {eb}）"),
            (Ok(_), Err(eb)) => println!("{op:?}: 4分割版は通ったが周期1面版が断られた: {eb}"),
            (Err(ea), Ok(_)) => println!("{op:?}: 周期1面版は通ったが4分割版が断られた: {ea}"),
        }
    }
}
