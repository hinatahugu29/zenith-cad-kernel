//! **点が、読んだ面の（素の）曲面の上にあるか**（4-528）。
//!
//! `cargo run --release -p zenith_algo --example point_on_face_probe -- <面> <x> <y> <z>`
use std::path::PathBuf;
use zenith_geom::ExtremumEngine;
use zenith_io::StepImporter;
use zenith_math::Point3;
use zenith_topo::FaceGeometry;

fn main() {
    let args: Vec<f64> = std::env::args().skip(1).filter_map(|v| v.parse().ok()).collect();
    let (index, point) = (args[0] as usize, Point3::new(args[1], args[2], args[3]));
    let path = PathBuf::from(concat!(env!("CARGO_MANIFEST_DIR"), "/../../reference/OCCT/data/step"))
        .join("linkrods.step");
    let solids = StepImporter::import_solids_from_file(&path).expect("読めません");
    let solid = solids.iter().max_by_key(|s| s.outer_shell.faces.len()).unwrap();
    let FaceGeometry::Nurbs(surface) = &solid.outer_shell.faces[index].geometry else {
        println!("面{index} は NURBS ではありません");
        return;
    };
    let ((u0, u1), (v0, v1)) = surface.param_range();
    let projection = ExtremumEngine::point_to_surface(point, surface, 64, 1e-13).unwrap();
    println!(
        "面{index}: u [{u0:.4}, {u1:.4}] v [{v0:.4}, {v1:.4}]、点 {point:?} → 最近点 uv ({:.4}, {:.4}) 距離 {:.3e}",
        projection.u, projection.v, projection.distance
    );
}
