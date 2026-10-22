//! **ROADMAP柱③の追加検証**——4-815・4-816は円柱・トーラス・円錐それぞれ
//! 1組の寸法でしか試していなかった。**寸法を変えても同じ結果になるか**
//! （細い/太い、短い/長い、主半径と管半径が近いトーラスなど）を、箱との
//! ブーリアン（差）1演算だけで広く浅く確かめる。

use zenith_algo::{BooleanEngine, BooleanOpType, BrepTransform, MassCalculator, PrimitiveBuilder};
use zenith_math::{Tolerance, Vec3};
use zenith_tess::TessellationParams;

fn main() {
    let tol = Tolerance::default();
    let params = TessellationParams { u_divisions: 48, v_divisions: 48 };

    println!("== 円柱 ==");
    for (radius, height) in [(2.0, 5.0), (10.0, 20.0), (30.0, 3.0), (1.0, 100.0), (50.0, 50.0)] {
        let standard = PrimitiveBuilder::make_cylinder(radius, height).unwrap();
        let periodic = PrimitiveBuilder::make_cylinder_periodic(radius, height).unwrap();
        let box_size = (radius * 3.0).max(height);
        let box_solid = PrimitiveBuilder::make_box(box_size, box_size, box_size).unwrap();
        let box_centered = BrepTransform::translate_solid(
            &box_solid,
            Vec3::new(-box_size / 2.0, -box_size / 2.0, height / 4.0),
        );
        let ra = BooleanEngine::boolean_solids_exact(&box_centered, &standard, BooleanOpType::Difference, &tol);
        let rb = BooleanEngine::boolean_solids_exact(&box_centered, &periodic, BooleanOpType::Difference, &tol);
        report(&format!("r={radius} h={height}"), ra, rb, &params);
    }

    println!("== トーラス ==");
    for (r_major, r_minor) in [(12.0, 4.0), (5.0, 4.9), (50.0, 2.0), (3.0, 1.0)] {
        let standard = PrimitiveBuilder::make_torus(r_major, r_minor).unwrap();
        let periodic = PrimitiveBuilder::make_torus_periodic(r_major, r_minor).unwrap();
        let box_size = (r_major + r_minor) * 0.6;
        let box_solid = PrimitiveBuilder::make_box(box_size, box_size, box_size).unwrap();
        let box_centered = BrepTransform::translate_solid(
            &box_solid,
            Vec3::new(r_major - box_size / 2.0, -box_size / 2.0, -box_size / 2.0),
        );
        let ra = BooleanEngine::boolean_solids_exact(&box_centered, &standard, BooleanOpType::Difference, &tol);
        let rb = BooleanEngine::boolean_solids_exact(&box_centered, &periodic, BooleanOpType::Difference, &tol);
        report(&format!("R={r_major} r={r_minor}"), ra, rb, &params);
    }

    println!("== 円錐 ==");
    for (r_bottom, height) in [(10.0, 20.0), (2.0, 50.0), (40.0, 5.0), (1.0, 1.0)] {
        let standard = PrimitiveBuilder::make_cone(r_bottom, 0.0, height).unwrap();
        let periodic = PrimitiveBuilder::make_cone_periodic(r_bottom, height).unwrap();
        let box_size = (r_bottom * 3.0).max(height);
        let box_solid = PrimitiveBuilder::make_box(box_size, box_size, box_size).unwrap();
        let box_centered = BrepTransform::translate_solid(
            &box_solid,
            Vec3::new(-box_size / 2.0, -box_size / 2.0, height / 4.0),
        );
        let ra = BooleanEngine::boolean_solids_exact(&box_centered, &standard, BooleanOpType::Difference, &tol);
        let rb = BooleanEngine::boolean_solids_exact(&box_centered, &periodic, BooleanOpType::Difference, &tol);
        report(&format!("r={r_bottom} h={height}"), ra, rb, &params);
    }
}

fn report(
    label: &str,
    standard: Result<zenith_topo::Solid, String>,
    periodic: Result<zenith_topo::Solid, String>,
    params: &TessellationParams,
) {
    match (standard, periodic) {
        (Ok(a), Ok(b)) => {
            let va = MassCalculator::compute_volume_from_brep(&a, params);
            let vb = MassCalculator::compute_volume_from_brep(&b, params);
            let fa = a.outer_shell.faces.len();
            let fb = b.outer_shell.faces.len();
            let rel = (va - vb).abs() / va.abs().max(1e-9);
            println!(
                "  {label}: 4分割版 体積={va:.6} 面{fa}枚 / 周期1面版 体積={vb:.6} 面{fb}枚 / 相対差={rel:.3e}  {}",
                if rel < 1e-6 { "OK" } else { "要確認" }
            );
        }
        (Err(ea), Err(eb)) => println!("  {label}: 両方とも断られた（{ea} / {eb}）"),
        (Ok(_), Err(eb)) => println!("  {label}: 4分割版は通ったが周期1面版が断られた: {eb}"),
        (Err(ea), Ok(_)) => println!("  {label}: 周期1面版は通ったが4分割版が断られた: {ea}"),
    }
}
