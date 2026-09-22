//! **`linkrods` の A面33 × 切り手の天面（z = 1.41）で、交線が 0 本になる理由**（4-525）。
//!
//! OCC はこの組で長さ 2.735 の交線を出します。こちらは `0 branch(es) marched`。
//! 曲面を細かく標本して **z = 1.41 の両側に点があるか**、種探しが何を返すかを出します。
//!
//! `cargo run --release -p zenith_algo --example ssi_face33_probe [面番号] [z]`

use std::path::PathBuf;
use zenith_geom::IntersectionMarcher;
use zenith_io::StepImporter;
use zenith_topo::FaceGeometry;

fn main() {
    let index: usize = std::env::args().nth(1).and_then(|v| v.parse().ok()).unwrap_or(33);
    let level: f64 = std::env::args().nth(2).and_then(|v| v.parse().ok()).unwrap_or(1.41);
    let path = PathBuf::from(concat!(env!("CARGO_MANIFEST_DIR"), "/../../reference/OCCT/data/step"))
        .join("linkrods.step");
    let solids = StepImporter::import_solids_from_file(&path).expect("読めません");
    let solid = solids.iter().max_by_key(|s| s.outer_shell.faces.len()).unwrap();
    let face = &solid.outer_shell.faces[index];
    let FaceGeometry::Nurbs(surface) = &face.geometry else {
        println!("面{index} は NURBS ではありません");
        return;
    };
    let ((u0, u1), (v0, v1)) = surface.param_range();
    println!("面{index}: 次数 {}x{}、制御点 {}x{}、u [{u0}, {u1}] v [{v0}, {v1}]",
        surface.degree_u, surface.degree_v, surface.control_points.len(), surface.control_points[0].len());
    let n = 400;
    let (mut above, mut below) = (0usize, 0usize);
    let (mut zmin, mut zmax) = (f64::INFINITY, f64::NEG_INFINITY);
    let (mut xmin, mut xmax, mut ymin, mut ymax) = (f64::INFINITY, f64::NEG_INFINITY, f64::INFINITY, f64::NEG_INFINITY);
    for i in 0..=n {
        for j in 0..=n {
            let p = surface.evaluate(u0 + (u1 - u0) * i as f64 / n as f64, v0 + (v1 - v0) * j as f64 / n as f64);
            if p.z > level { above += 1 } else { below += 1 }
            zmin = zmin.min(p.z); zmax = zmax.max(p.z);
            xmin = xmin.min(p.x); xmax = xmax.max(p.x); ymin = ymin.min(p.y); ymax = ymax.max(p.y);
        }
    }
    println!("パッチ全体（素）: z [{zmin:.4}, {zmax:.4}] x [{xmin:.3}, {xmax:.3}] y [{ymin:.3}, {ymax:.3}]、z={level} の上 {above} 点 / 下 {below} 点");
    // 制御点の z
    let (mut czmin, mut czmax) = (f64::INFINITY, f64::NEG_INFINITY);
    for row in &surface.control_points { for cp in row { czmin = czmin.min(cp.point.z); czmax = czmax.max(cp.point.z); } }
    println!("制御点の z [{czmin:.4}, {czmax:.4}]");

    // 平面 z = level を、切り手の天面ぶんの 1 次パッチにして種を探す。
    let corner = |x: f64, y: f64| zenith_geom::ControlPoint3::unweighted(zenith_math::Point3::new(x, y, level));
    let plane = zenith_geom::NurbsSurface3::new(
        1, 1,
        vec![vec![corner(3.2756, 2.545), corner(3.2756, 3.955)], vec![corner(7.9947, 2.545), corner(7.9947, 3.955)]],
        zenith_geom::KnotVector::clamped_uniform(2, 1),
        zenith_geom::KnotVector::clamped_uniform(2, 1),
    );
    match plane {
        Ok(plane) => {
            for grid in [12usize, 48] {
                let seeds = IntersectionMarcher::find_seeds(&plane, surface, grid, 8);
                let refined = IntersectionMarcher::find_seeds_refined(&plane, surface, grid, 8);
                println!("種（格子 {grid}）: 粗 {} 個 / 細 {} 個", seeds.len(), refined.len());
            }
            for (step, deviation) in [(0.4925, 1e-6), (0.05, 1e-6), (0.05, 1e-5), (0.01, 1e-4)] {
                let tol = zenith_math::Tolerance::default();
                let found = IntersectionMarcher::fit_all_branches(&plane, surface, step, deviation, 2, &tol);
                println!("辿り（歩幅 {step}、許容 {deviation:e}）: 枝 {} 本 {:?}", found.len(), found.iter().map(|(_, m, dev)| (m.points.len(), *dev)).collect::<Vec<_>>());
            }
        }
        Err(err) => println!("平面パッチが作れません: {err}"),
    }
}
