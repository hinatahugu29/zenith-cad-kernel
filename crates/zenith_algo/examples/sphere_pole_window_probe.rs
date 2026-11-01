//! 北極に窓を開けた和で、周期1面の球の体積が4分割版とずれる件を測る（4-841）。
use zenith_algo::{BooleanEngine, BooleanOpType, BrepTransform, MassCalculator, PrimitiveBuilder};
use zenith_math::{Tolerance, Vec3};
use zenith_tess::TessellationParams;

fn main() {
    let tol = Tolerance::default();
    let tool = BrepTransform::translate_solid(
        &PrimitiveBuilder::make_box(8.0, 8.0, 8.0).unwrap(),
        Vec3::new(-4.0, -4.0, 5.0),
    );
    for (name, sphere) in [
        ("4分割", PrimitiveBuilder::make_sphere(10.0).unwrap()),
        ("周期", PrimitiveBuilder::make_sphere_periodic(10.0).unwrap()),
    ] {
        let r = BooleanEngine::boolean_solids_exact(&sphere, &tool, BooleanOpType::Union, &tol).unwrap();
        for n in [24usize, 48, 96] {
            let p = TessellationParams { u_divisions: n, v_divisions: n };
            // **メッシュから独立に測る**（4-841）。面の輪の解析求積を通りません。
            let mesh = zenith_tess::tessellate_solid(&r, &p);
            let from_mesh = MassCalculator::compute_from_mesh(&mesh).volume;
            println!(
                "{name} n={n}: 面の輪から {:.6} / メッシュから {from_mesh:.6}（三角形 {}）面{}",
                MassCalculator::compute_volume_from_brep(&r, &p),
                mesh.indices.len() / 3,
                r.outer_shell.faces.len()
            );
        }
        for (i, f) in r.outer_shell.faces.iter().enumerate() {
            let (a, v) = MassCalculator::compute_face_integral(f, &TessellationParams::default());
            let kind = if matches!(f.geometry, zenith_topo::FaceGeometry::Nurbs(_)) { "NURBS" } else { "平面" };
            println!("  面{i} {kind} 外周{}稜 穴{} 面積 {a:.6} 体積寄与 {v:.6}", f.outer_wire.edges.len(), f.inner_wires.len());
            if let Ok(p) = f.pcurves(&tol) {
                for (which, l) in std::iter::once(("外周", &p.outer_loop)).chain(p.inner_loops.iter().map(|l| ("穴", l))) {
                    let mut lo = (f64::MAX, f64::MAX);
                    let mut hi = (f64::MIN, f64::MIN);
                    for seg in &l.segments {
                        for c in &seg.curve.control_points {
                            lo = (lo.0.min(c.point.x), lo.1.min(c.point.y));
                            hi = (hi.0.max(c.point.x), hi.1.max(c.point.y));
                        }
                    }
                    println!("    {which} 区間{} uv ({:.4},{:.4})-({:.4},{:.4})", l.segments.len(), lo.0, lo.1, hi.0, hi.1);
                }
            }
        }
    }
}
