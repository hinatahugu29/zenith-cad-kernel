//! ＼の結果のメッシュが、極の穴を塞いでしまっていないかを見る（4-843）。
use zenith_algo::{BooleanEngine, BooleanOpType, BrepTransform, PrimitiveBuilder};
use zenith_math::{Tolerance, Vec3};
use zenith_tess::{tessellate_solid, TessellationParams};

fn mesh_volume(m: &zenith_tess::TriangleMesh) -> f64 {
    let mut v = 0.0;
    for t in &m.indices {
        let (a, b, c) = (m.positions[t[0] as usize], m.positions[t[1] as usize], m.positions[t[2] as usize]);
        v += (a.x * (b.y * c.z - c.y * b.z) - a.y * (b.x * c.z - c.x * b.z) + a.z * (b.x * c.y - c.x * b.y)) / 6.0;
    }
    v
}

fn main() {
    let tol = Tolerance::default();
    let params = TessellationParams::default();
    let tool = BrepTransform::translate_solid(
        &PrimitiveBuilder::make_box(8.0, 8.0, 8.0).unwrap(),
        Vec3::new(-4.0, -4.0, 5.0),
    );
    for (name, s) in [
        ("4分割", PrimitiveBuilder::make_sphere(10.0).unwrap()),
        ("周期", PrimitiveBuilder::make_sphere_periodic(10.0).unwrap()),
    ] {
        let r = BooleanEngine::boolean_solids_exact_result_unverified(&s, &tool, BooleanOpType::Difference, &tol);
        match r {
            Ok(res) => {
                for solid in &res.solids {
                    let m = tessellate_solid(solid, &params);
                    println!(
                        "{name}: 面{} 三角形{} メッシュ体積{:.6}",
                        solid.outer_shell.faces.len(),
                        m.indices.len(),
                        mesh_volume(&m)
                    );
                    // 極より上(z>9.1652)に頂点を持つ面を挙げる
                    for (i, f) in solid.outer_shell.faces.iter().enumerate() {
                        let kind = if matches!(f.geometry, zenith_topo::FaceGeometry::Nurbs(_)) { "NURBS" } else { "平面" };
                        println!(
                            "  面{i} {kind} 外周{}稜 穴{}",
                            f.outer_wire.edges.len(),
                            f.inner_wires.len()
                        );
                    }
                }
            }
            Err(e) => println!("{name}: 断り {}", &e.chars().take(160).collect::<String>()),
        }
    }
}
