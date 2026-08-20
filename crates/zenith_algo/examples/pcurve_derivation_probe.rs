//! ファイルが述べた p-curve と、境界から導出し直した p-curve の差。
//!
//! `pcurve_fidelity_probe` は、面が**保持している** p-curve が本当に辺の上に
//! あるかを測る。こちらは別のことを測る: その保持を捨てて、3D 境界から
//! 導出し直したときに**答えが変わるか**である。
//!
//! 変わってはいけない、というのが素朴な期待だが、実測では変わる。他カーネルの
//! トリム B-spline は、面の外まで伸びた曲面のどこに境界が乗るかを p-curve が
//! 決めており、境界を投影し直しただけでは同じ場所に戻らない。
//!
//! 面を組み直す処理（[`zenith_algo::Regularizer`]、ブーリアンの分割、
//! 直接編集）は、辺が変われば p-curve を作り直すことになる。ここが大きい面は
//! **組み直した瞬間に答えが変わる**ので、触る前に知っておく必要がある。
//!
//! 走らせ方: cargo run --release -p zenith_algo --example pcurve_derivation_probe

use std::fs;
use std::path::Path;

use zenith_algo::mass_properties::MassCalculator;
use zenith_io::StepImporter;
use zenith_tess::TessellationParams;
use zenith_topo::Face;

/// 保持している p-curve で積んだ面積・体積寄与と、捨てて導出し直したときの値。
fn both_ways(face: &Face) -> ((f64, f64), (f64, f64)) {
    let params = TessellationParams::default();
    let stored = MassCalculator::compute_face_integral(face, &params);
    let mut stripped = face.clone();
    stripped.pcurves = None;
    let derived = MassCalculator::compute_face_integral(&stripped, &params);
    (stored, derived)
}

fn main() {
    let directory = Path::new("target/validation");
    if !directory.exists() {
        println!("target/validation is not there yet. Run export_validation_suite first.");
        return;
    }

    println!(
        "{:<36} {:>4} {:>14} {:>14} {:>10} {:>10}",
        "file", "face", "area stored", "area derived", "rel area", "rel volume"
    );
    println!("{}", "-".repeat(94));

    let mut files: Vec<_> = fs::read_dir(directory)
        .unwrap()
        .filter_map(|entry| entry.ok())
        .map(|entry| entry.path())
        .filter(|path| path.extension().map(|e| e == "step").unwrap_or(false))
        .collect();
    files.sort();

    let mut worst: f64 = 0.0;
    let mut worst_where = String::new();

    for path in files {
        let Ok(solids) = StepImporter::import_solids_from_file(&path) else {
            continue;
        };
        let name = path.file_name().unwrap().to_string_lossy().to_string();
        for solid in &solids {
            for (index, face) in solid.outer_shell.faces.iter().enumerate() {
                if face.pcurves.is_none() {
                    continue;
                }
                let ((area_stored, volume_stored), (area_derived, volume_derived)) =
                    both_ways(face);
                let scale = area_stored.abs().max(volume_stored.abs()).max(1.0);
                let rel_area = (area_derived - area_stored).abs() / scale;
                let rel_volume = (volume_derived - volume_stored).abs() / scale;
                let worst_here = rel_area.max(rel_volume);
                if worst_here > worst {
                    worst = worst_here;
                    worst_where = format!("{name} face {index}");
                }
                // 一致している面は行が増えるだけなので、動いた面だけ出す。
                if worst_here > 1e-9 {
                    println!(
                        "{:<36} {:>4} {:>14.6} {:>14.6} {:>10.2e} {:>10.2e}",
                        name, index, area_stored, area_derived, rel_area, rel_volume
                    );
                }
            }
        }
    }

    println!("{}", "-".repeat(94));
    if worst <= 1e-9 {
        println!("every stored p-curve is reproduced by deriving it again (worst {worst:.2e})");
    } else {
        println!(
            "worst disagreement {worst:.3e} at {worst_where}. Faces listed above cannot be \
rebuilt without the p-curves the file stated."
        );
    }
}
