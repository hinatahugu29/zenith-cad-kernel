//! 周期1面の球に箱の角を差し込んだ和の体積が、4分割版とずれる理由を見る（4-840）。
use zenith_algo::{BooleanEngine, BooleanOpType, BrepTransform, MassCalculator, PrimitiveBuilder};
use zenith_math::{Tolerance, Vec3};
use zenith_tess::TessellationParams;

fn main() {
    let tol = Tolerance::default();
    let tool = BrepTransform::translate_solid(
        &PrimitiveBuilder::make_box(10.0, 10.0, 10.0).unwrap(),
        Vec3::new(3.0, 3.0, 3.0),
    );
    for (name, sphere) in [
        ("4分割", PrimitiveBuilder::make_sphere(10.0).unwrap()),
        ("周期", PrimitiveBuilder::make_sphere_periodic(10.0).unwrap()),
    ] {
        let r = BooleanEngine::boolean_solids_exact(&sphere, &tool, BooleanOpType::Union, &tol).unwrap();
        for n in [24usize, 48, 96] {
            let p = TessellationParams { u_divisions: n, v_divisions: n };
            println!("{name} n={n}: {:.6}  面{}", MassCalculator::compute_volume_from_brep(&r, &p), r.outer_shell.faces.len());
        }
        for (i, f) in r.outer_shell.faces.iter().enumerate() {
            let (a, v) = MassCalculator::compute_face_integral(f, &TessellationParams::default());
            let kind = if matches!(f.geometry, zenith_topo::FaceGeometry::Nurbs(_)) { "NURBS" } else { "平面" };
            println!("  面{i} {kind} 穴{} 面積 {a:.6} 体積寄与 {v:.6} p-curve{}", f.inner_wires.len(), if f.pcurves.is_some() { "持たせた" } else { "導出" });
            if let (Some(p), true) = (&f.pcurves, !f.inner_wires.is_empty()) {
                for seg in &p.inner_loops[0].segments {
                    println!("    穴区間 次数{} 制御点{}", seg.curve.degree, seg.curve.control_points.len());
                }
            }
        }
        println!("{name} 単体: {:.6}", MassCalculator::compute_volume_from_brep(&sphere, &TessellationParams::default()));
    }
}
