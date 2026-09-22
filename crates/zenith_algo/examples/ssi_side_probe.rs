//! **A面6 × 箱の側面（y = 3.955）で、2 本目の縦線が出ない理由**（4-528）。
//!
//! OCC は x = 3.6191 と x = 4.1309 の 2 本を出します。こちらは 1 本。
//! `cargo run --release -p zenith_algo --example ssi_side_probe [面] [y]`
use std::path::PathBuf;
use zenith_geom::{ControlPoint3, IntersectionMarcher, KnotVector, NurbsSurface3};
use zenith_io::StepImporter;
use zenith_math::{Point3, Tolerance};
use zenith_topo::FaceGeometry;

fn main() {
    let index: usize = std::env::args().nth(1).and_then(|v| v.parse().ok()).unwrap_or(6);
    let y: f64 = std::env::args().nth(2).and_then(|v| v.parse().ok()).unwrap_or(3.955);
    let path = PathBuf::from(concat!(env!("CARGO_MANIFEST_DIR"), "/../../reference/OCCT/data/step"))
        .join("linkrods.step");
    let solids = StepImporter::import_solids_from_file(&path).expect("読めません");
    let solid = solids.iter().max_by_key(|s| s.outer_shell.faces.len()).unwrap();
    let FaceGeometry::Nurbs(surface) = &solid.outer_shell.faces[index].geometry else {
        return;
    };
    let corner = |x: f64, z: f64| ControlPoint3::unweighted(Point3::new(x, y, z));
    let plane = NurbsSurface3::new(
        1, 1,
        vec![vec![corner(3.2756, 0.47), corner(3.2756, 1.41)], vec![corner(7.9947, 0.47), corner(7.9947, 1.41)]],
        KnotVector::clamped_uniform(2, 1),
        KnotVector::clamped_uniform(2, 1),
    ).expect("平面");
    for grid in [12usize, 24] {
        for limit in [4usize, 16] {
            let seeds = IntersectionMarcher::find_seeds(&plane, surface, grid, limit);
            let xs: Vec<String> = seeds.iter().map(|(u, v)| {
                let p = plane.evaluate(*u, *v);
                format!("({:.3},{:.3})", p.x, p.z)
            }).collect();
            println!("種 格子 {grid} 上限 {limit}: {} 個 {}", seeds.len(), xs.join(" "));
        }
    }
    let tol = Tolerance::default();
    for step in [0.48, 0.2, 0.05] {
        let found = IntersectionMarcher::fit_all_branches(&plane, surface, step, 1e-6, 4, &tol);
        let lines: Vec<String> = found.iter().map(|(curve, _, _)| {
            let (t0, t1) = curve.param_range();
            let (a, b) = (curve.evaluate(t0), curve.evaluate(t1));
            format!("x {:.4}→{:.4}", a.x, b.x)
        }).collect();
        println!("辿り 歩幅 {step}: {} 本 {}", found.len(), lines.join(" / "));
    }
}
