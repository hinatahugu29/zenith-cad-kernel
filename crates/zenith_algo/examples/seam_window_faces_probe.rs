//! 継ぎ目をまたぐ窓の和で、返った立体の面を1枚ずつ見る（4-847）。
use zenith_algo::{BooleanEngine, BooleanOpType, BrepTransform, MassCalculator, PrimitiveBuilder};
use zenith_math::{Tolerance, Vec3};
use zenith_tess::TessellationParams;
use zenith_topo::FaceGeometry;

fn main() {
    let tol = Tolerance::default();
    let tool = BrepTransform::translate_solid(
        &PrimitiveBuilder::make_box(8.0, 8.0, 8.0).unwrap(),
        Vec3::new(5.0, -4.0, -4.0),
    );
    let params = TessellationParams::default();
    for (name, sphere) in [
        ("4分割", PrimitiveBuilder::make_sphere(10.0).unwrap()),
        ("周期", PrimitiveBuilder::make_sphere_periodic(10.0).unwrap()),
    ] {
        let unverified = match BooleanEngine::boolean_solids_exact_result_unverified(
            &sphere,
            &tool,
            BooleanOpType::Union,
            &tol,
        ) {
            Ok(result) => result,
            Err(reason) => {
                println!("{name}: 断り {reason}");
                continue;
            }
        };
        let Some(result) = unverified.solids.into_iter().next() else {
            println!("{name}: 立体が0個");
            continue;
        };
        println!(
            "{name}: 面{} 体積 {:.6}",
            result.outer_shell.faces.len(),
            MassCalculator::compute_volume_from_brep(&result, &params)
        );
        let mut area_sum = 0.0;
        for (index, face) in result.outer_shell.faces.iter().enumerate() {
            let (area, volume) = MassCalculator::compute_face_integral(face, &params);
            area_sum += area;
            let kind = if matches!(face.geometry, FaceGeometry::Nurbs(_)) {
                "NURBS"
            } else {
                "平面"
            };
            if matches!(face.geometry, FaceGeometry::Nurbs(_)) {
                if let Ok(pcurves) = face.pcurves(&tol) {
                    for (which, seg) in pcurves.outer_loop.segments.iter().enumerate() {
                        let (t0, t1) = seg.curve.param_range();
                        let a = seg.curve.evaluate(t0);
                        let b = seg.curve.evaluate(t1);
                        println!(
                            "      p曲線{which}: ({:.4},{:.4})->({:.4},{:.4})",
                            a.x, a.y, b.x, b.y
                        );
                    }
                }
            }
            println!(
                "  面{index} {kind} 外周{}稜 穴{} 面積 {area:.6} 体積寄与 {volume:.6} 向き {:?}",
                face.outer_wire.edges.len(),
                face.inner_wires.len(),
                face.orientation
            );
        }
        println!("  面積の合計 {area_sum:.6}");
    }
}
