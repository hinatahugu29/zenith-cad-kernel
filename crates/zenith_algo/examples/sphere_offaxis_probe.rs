//! 箱8を軸から外して置くと、周期1面の球だけが断る理由を見る（4-841）。
use zenith_algo::{BooleanEngine, BooleanOpType, BrepTransform, PrimitiveBuilder};
use zenith_math::{Tolerance, Vec3};

fn main() {
    let tol = Tolerance::default();
    let tool = BrepTransform::translate_solid(
        &PrimitiveBuilder::make_box(8.0, 8.0, 8.0).unwrap(),
        Vec3::new(5.0, -4.0, -4.0),
    );
    for (name, sphere) in [
        ("4分割", PrimitiveBuilder::make_sphere(10.0).unwrap()),
        ("周期", PrimitiveBuilder::make_sphere_periodic(10.0).unwrap()),
    ] {
        match BooleanEngine::boolean_solids_exact(&sphere, &tool, BooleanOpType::Union, &tol) {
            Ok(s) => println!("{name}: 返った 面{}", s.outer_shell.faces.len()),
            Err(e) => println!("{name}: 断り\n  {e}"),
        }
    }
}
