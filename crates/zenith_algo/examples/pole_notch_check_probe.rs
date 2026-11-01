//! 切り欠きの付け替えで、極の窓の3演算が4分割版と合うかを測る（4-845）。
use zenith_algo::{BooleanEngine, BooleanOpType, BrepTransform, MassCalculator, PrimitiveBuilder};
use zenith_tess::TessellationParams;
use zenith_math::{Tolerance, Vec3};

fn main() {
    let tol = Tolerance::default();
    let tool = BrepTransform::translate_solid(
        &PrimitiveBuilder::make_box(8.0, 8.0, 8.0).unwrap(),
        Vec3::new(-4.0, -4.0, 5.0),
    );
    for (op, name) in [
        (BooleanOpType::Union, "∪"),
        (BooleanOpType::Intersection, "∩"),
        (BooleanOpType::Difference, "＼"),
    ] {
        for (kind, s) in [
            ("4分割", PrimitiveBuilder::make_sphere(10.0).unwrap()),
            ("周期", PrimitiveBuilder::make_sphere_periodic(10.0).unwrap()),
        ] {
            match BooleanEngine::boolean_solids_exact(&s, &tool, op, &tol) {
                Ok(r) => {
                    let p = TessellationParams::default();
                    let mesh = zenith_tess::tessellate_solid(&r, &p);
                    println!(
                        "{name} {kind}: 面{} 面の輪から{:.6} メッシュから{:.6}",
                        r.outer_shell.faces.len(),
                        MassCalculator::compute_volume_from_brep(&r, &p),
                        MassCalculator::compute_from_mesh(&mesh).volume
                    );
                }
                Err(e) => println!("{name} {kind}: 断り {}", &e.chars().take(130).collect::<String>()),
            }
        }
    }
}
