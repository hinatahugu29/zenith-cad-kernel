//! 北極の窓の差（＼）が、まだ断られる理由を見る（4-842）。
use zenith_algo::{BooleanEngine, BooleanOpType, BrepTransform, PrimitiveBuilder};
use zenith_math::{Tolerance, Vec3};
fn main() {
    let tol = Tolerance::default();
    let tool = BrepTransform::translate_solid(
        &PrimitiveBuilder::make_box(8.0, 8.0, 8.0).unwrap(),
        Vec3::new(-4.0, -4.0, 5.0),
    );
    for (name, s) in [
        ("4分割", PrimitiveBuilder::make_sphere(10.0).unwrap()),
        ("周期", PrimitiveBuilder::make_sphere_periodic(10.0).unwrap()),
    ] {
        match BooleanEngine::boolean_solids_exact(&s, &tool, BooleanOpType::Difference, &tol) {
            Ok(r) => println!("{name}: 返った 面{}", r.outer_shell.faces.len()),
            Err(e) => println!("{name}: 断り {}", &e.chars().take(260).collect::<String>()),
        }
    }
}
