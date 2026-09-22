//! **読んだ立体に、同じ面が 2 枚入っていないか**（4-531）。
use std::path::PathBuf;
use zenith_io::StepImporter;
use zenith_math::Vec3;

fn main() {
    let path = PathBuf::from(concat!(env!("CARGO_MANIFEST_DIR"), "/../../reference/OCCT/data/step"))
        .join("linkrods.step");
    let solids = StepImporter::import_solids_from_file(&path).expect("読めません");
    let solid = solids.iter().max_by_key(|s| s.outer_shell.faces.len()).unwrap();
    println!("外側シェル {} 枚、内側シェル {} 個", solid.outer_shell.faces.len(), solid.inner_shells.len());
    let mut keys: Vec<(usize, [i64; 3], usize)> = Vec::new();
    for (index, face) in solid.outer_shell.faces.iter().enumerate() {
        let points: Vec<_> = face.outer_wire.edges.iter().map(|o| o.start_vertex().point).collect();
        let mut centre = Vec3::zeros();
        for p in &points { centre += p.coords; }
        centre /= points.len().max(1) as f64;
        let round = |v: f64| (v * 1e6).round() as i64;
        keys.push((index, [round(centre.x), round(centre.y), round(centre.z)], face.outer_wire.edges.len()));
    }
    let mut duplicates = 0;
    for i in 0..keys.len() {
        for j in (i + 1)..keys.len() {
            if keys[i].1 == keys[j].1 && keys[i].2 == keys[j].2 {
                println!("面{} と 面{} が同じ重心・同じ稜の本数", keys[i].0, keys[j].0);
                duplicates += 1;
            }
        }
    }
    println!("重なり {duplicates} 組");
}
