//! 窓が継ぎ目（経度0°、x>0・y=0）をまたぐかどうかで、周期1面の球の
//! ブーリアンが返るかが決まるのかを測る（4-841の裏取り）。
use zenith_algo::{BooleanEngine, BooleanOpType, BrepTransform, MassCalculator, PrimitiveBuilder};
use zenith_math::{Tolerance, Vec3};
use zenith_tess::TessellationParams;

fn main() {
    let tol = Tolerance::default();
    let params = TessellationParams::default();
    let standard = PrimitiveBuilder::make_sphere(10.0).unwrap();
    let periodic = PrimitiveBuilder::make_sphere_periodic(10.0).unwrap();
    let cube = |s: f64, at: Vec3| {
        BrepTransform::translate_solid(&PrimitiveBuilder::make_box(s, s, s).unwrap(), at)
    };
    // どれも球の表面に窓を1つ開ける置き方。**継ぎ目をまたぐかだけが違う。**
    let placements = [
        ("+x（継ぎ目の上。またぐ）", cube(8.0, Vec3::new(5.0, -4.0, -4.0))),
        ("-x（継ぎ目の裏。またがない）", cube(8.0, Vec3::new(-13.0, -4.0, -4.0))),
        ("+y（継ぎ目から90°。またがない）", cube(8.0, Vec3::new(-4.0, 5.0, -4.0))),
        ("-y（継ぎ目から90°。またがない）", cube(8.0, Vec3::new(-4.0, -13.0, -4.0))),
        ("+z（北極。またがない？）", cube(8.0, Vec3::new(-4.0, -4.0, 5.0))),
    ];
    for (name, tool) in &placements {
        let shot = |sphere| match BooleanEngine::boolean_solids_exact(sphere, tool, BooleanOpType::Union, &tol) {
            Ok(s) => format!("{:.6}", MassCalculator::compute_volume_from_brep(&s, &params)),
            Err(e) => format!("断り（{}）", if e.contains("not implemented") { "未実装" } else { "検証" }),
        };
        println!("{name:34} 4分割 {} | 周期 {}", shot(&standard), shot(&periodic));
    }
}
