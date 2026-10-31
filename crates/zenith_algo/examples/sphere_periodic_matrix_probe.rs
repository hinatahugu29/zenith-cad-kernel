//! 周期1面の球が、4分割版の球と同じブーリアン結果を出すかを、
//! 置き方3つ × 演算3つで並べて測る検体（HANDOVER 4-840）。

use zenith_algo::{BooleanEngine, BooleanOpType, BrepTransform, MassCalculator, PrimitiveBuilder};
use zenith_math::{Tolerance, Vec3};
use zenith_tess::TessellationParams;

fn main() {
    let tol = Tolerance::default();
    let params = TessellationParams::default();
    let r = 10.0;
    let standard = PrimitiveBuilder::make_sphere(r).unwrap();
    let periodic = PrimitiveBuilder::make_sphere_periodic(r).unwrap();
    let cube = |s: f64, at: Vec3| {
        BrepTransform::translate_solid(&PrimitiveBuilder::make_box(s, s, s).unwrap(), at)
    };
    let placements = [
        ("box12 centered", cube(12.0, Vec3::new(-6.0, -6.0, -6.0))),
        ("box10 corner-in", cube(10.0, Vec3::new(3.0, 3.0, 3.0))),
        ("box30 cuts top", cube(30.0, Vec3::new(-15.0, -15.0, 4.0))),
        ("box8 off-axis", cube(8.0, Vec3::new(5.0, -4.0, -4.0))),
    ];
    for (name, tool) in &placements {
        for op in [BooleanOpType::Union, BooleanOpType::Difference, BooleanOpType::Intersection] {
            let vol = |sphere| {
                match BooleanEngine::boolean_solids_exact(sphere, tool, op, &tol) {
                    Ok(s) => format!("{:.6}", MassCalculator::compute_volume_from_brep(&s, &params)),
                    Err(e) => format!("断り: {}", &e.chars().take(70).collect::<String>()),
                }
            };
            println!("{name:16} {op:?}: 4分割 {} | 周期 {}", vol(&standard), vol(&periodic));
        }
    }
}
