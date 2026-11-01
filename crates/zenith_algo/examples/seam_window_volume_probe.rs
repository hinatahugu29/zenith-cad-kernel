//! 継ぎ目をまたぐ窓の和を、4分割版と周期版で比べる（4-847）。
use zenith_algo::{BooleanEngine, BooleanOpType, BrepTransform, MassCalculator, PrimitiveBuilder};
use zenith_math::{Tolerance, Vec3};
use zenith_tess::TessellationParams;

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
        for (op_name, op) in [
            ("∪", BooleanOpType::Union),
            ("∩", BooleanOpType::Intersection),
            ("＼", BooleanOpType::Difference),
        ] {
            match BooleanEngine::boolean_solids_exact(&sphere, &tool, op, &tol) {
                Ok(result) => {
                    let mesh = zenith_tess::tessellate_solid(&result, &params);
                    println!(
                        "{name} {op_name}: 面{} 面の輪から {:.6} メッシュから {:.6}",
                        result.outer_shell.faces.len(),
                        MassCalculator::compute_volume_from_brep(&result, &params),
                        MassCalculator::compute_from_mesh(&mesh).volume
                    );
                }
                Err(reason) => println!("{name} {op_name}: 断り {reason}"),
            }
        }
    }
}
