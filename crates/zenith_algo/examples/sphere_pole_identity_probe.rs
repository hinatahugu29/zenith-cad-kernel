//! 北極の窓の赤（4-842）が、恒等式 |A∪B|+|A∩B| = |A|+|B| で捕まるかを測る。
use zenith_algo::{BooleanEngine, BooleanOpType, BrepTransform, MassCalculator, PrimitiveBuilder};
use zenith_math::{Tolerance, Vec3};
use zenith_tess::TessellationParams;
use zenith_topo::Solid;

fn main() {
    let tol = Tolerance::default();
    let p = TessellationParams::default();
    let tool = BrepTransform::translate_solid(
        &PrimitiveBuilder::make_box(8.0, 8.0, 8.0).unwrap(),
        Vec3::new(-4.0, -4.0, 5.0),
    );
    let vol = |s: &Solid| MassCalculator::compute_volume_from_brep(s, &p);
    let mesh_vol = |s: &Solid| {
        MassCalculator::compute_from_mesh(&zenith_tess::tessellate_solid(
            s,
            &TessellationParams { u_divisions: 96, v_divisions: 96 },
        ))
        .volume
    };
    for (name, sphere) in [
        ("4分割", PrimitiveBuilder::make_sphere(10.0).unwrap()),
        ("周期", PrimitiveBuilder::make_sphere_periodic(10.0).unwrap()),
    ] {
        let a = vol(&sphere);
        let b = vol(&tool);
        let run = |op| BooleanEngine::boolean_solids_exact(&sphere, &tool, op, &tol);
        for op in [BooleanOpType::Union, BooleanOpType::Intersection, BooleanOpType::Difference] {
            match run(op) {
                Ok(s) => println!("  {name} {op:?}: 返った 面の輪から {:.6}", vol(&s)),
                Err(e) => println!("  {name} {op:?}: 断り（{}）", if e.contains("not implemented") { "未実装" } else { "検証" }),
            }
        }
        let (Ok(u), Ok(i), Ok(d)) = (
            run(BooleanOpType::Union),
            run(BooleanOpType::Intersection),
            run(BooleanOpType::Difference),
        ) else {
            println!("{name}: 3演算そろわないので恒等式は測れません");
            continue;
        };
        let (uv, iv, dv) = (vol(&u), vol(&i), vol(&d));
        println!("{name}: A {a:.6} B {b:.6} ∪ {uv:.6} ∩ {iv:.6} ＼ {dv:.6}");
        println!(
            "  包除 (∪+∩)-(A+B) = {:.3e}／分割 (＼+∩)-A = {:.3e}",
            (uv + iv) - (a + b),
            (dv + iv) - a
        );
        println!(
            "  メッシュ(96)から: A {:.6} ∪ {:.6} ∩ {:.6} ＼ {:.6}",
            mesh_vol(&sphere), mesh_vol(&u), mesh_vol(&i), mesh_vol(&d)
        );
    }
}
