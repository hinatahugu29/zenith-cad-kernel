//! **点を曲面へ落とす 1 回の値段**（4-699）。
//!
//! **H8 の差で 86 万回**（うち粗探索つき 9,826 回、ニュートン 490 万回。
//! 4-696）。**78% を 1 組の面が使っている**（4-698）ので、
//! **その 1 枚で測ります**——`linkrods` の面33。
//!
//! **2 通りあります**——**粗探索つき**（17×17 = 289 点を見てから詰める）と
//! **種つき**（前の答えを種にして詰めるだけ）。
use zenith_geom::ExtremumEngine;

fn main() {
    const STEP: &str = include_str!("../../../reference/OCCT/data/step/linkrods.step");
    let Ok(solids) = zenith_io::StepImporter::import_solids_from_str(STEP) else {
        println!("読めません");
        return;
    };
    let Some(solid) = solids.into_iter().max_by_key(|s| s.outer_shell.faces.len()) else {
        return;
    };
    for index in [33usize, 1, 6] {
        let Some(face) = solid.outer_shell.faces.get(index) else { continue };
        let zenith_topo::FaceGeometry::Nurbs(surface) = &face.geometry else {
            println!("面{index}: NURBS ではありません");
            continue;
        };
        let ((u0, u1), (v0, v1)) = surface.param_range();
        // **落とす点は、曲面から少し離した所**に置きます（真上だと楽すぎます）。
        let targets: Vec<_> = (0..64)
            .map(|k| {
                let a = k as f64 / 64.0;
                let p = surface.evaluate(u0 + (u1 - u0) * a, v0 + (v1 - v0) * (1.0 - a));
                zenith_math::Point3::new(p.x + 0.01, p.y - 0.01, p.z + 0.02)
            })
            .collect();
        let mut t = std::time::Instant::now();
        let mut sink = 0.0f64;
        for p in &targets {
            if let Ok(r) = ExtremumEngine::point_to_surface(*p, surface, 32, 1e-9) {
                sink += r.distance;
            }
        }
        let coarse = t.elapsed().as_secs_f64() / targets.len() as f64 * 1e6;
        t = std::time::Instant::now();
        for p in &targets {
            if let Ok(r) = ExtremumEngine::point_to_surface_seeded(
                *p,
                surface,
                (u0 + u1) * 0.5,
                (v0 + v1) * 0.5,
                32,
                1e-9,
            ) {
                sink += r.distance;
            }
        }
        let seeded = t.elapsed().as_secs_f64() / targets.len() as f64 * 1e6;
        println!(
            "面{index:<3} 粗探索つき {coarse:>8.1} us/回   種つき {seeded:>8.1} us/回   （sink={sink:.3e}）"
        );
    }
    println!();
    println!("H8 の差: 落とす 860,746 回（うち粗探索 9,826 回）、ニュートン 4,896,141 回");
}
