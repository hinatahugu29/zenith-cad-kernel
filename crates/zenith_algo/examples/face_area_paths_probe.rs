//! **面積を 2 通りで出して並べます**（4-680）。
//!
//! **なぜ要るか**: `screw` のトロイド 3 枚は、**求積の刻みを 4 から 128 に
//! 上げても、面積が小数 9 桁目まで動きません**（円柱の面は素直に収束します）。
//! **動かないのは「収束が速い」のか「別の道を通って刻みを見ていない」のか**
//! ——**並べれば分かります。**
//!
//! * **積分**: `MassCalculator::compute_face_integral`（解析の道を持つ）
//! * **三角形**: `tessellate_face` が返す三角形の面積の和（刻みに素直）
//!
//! **同じ面で 2 つが離れていて、三角形だけが刻みで動くなら、
//! 積分は刻みを見ていません。**
//!
//! ```bash
//! ZENITH_SUBJECT=<step> cargo run --release -p zenith_algo --example face_area_paths_probe
//! ```

use std::process::exit;

use zenith_algo::MassCalculator;
use zenith_io::StepImporter;
use zenith_tess::{tessellate_face, TessellationParams};

fn mesh_area(face: &zenith_topo::Face, params: &TessellationParams) -> f64 {
    let mesh = tessellate_face(face, params);
    let mut area = 0.0;
    for tri in &mesh.indices {
        let a = mesh.positions[tri[0] as usize];
        let b = mesh.positions[tri[1] as usize];
        let c = mesh.positions[tri[2] as usize];
        area += (b - a).cross(&(c - a)).norm() / 2.0;
    }
    area
}

fn main() {
    let path = std::env::var("ZENITH_SUBJECT")
        .unwrap_or_else(|_| "reference/OCCT/data/step/linkrods.step".to_string());
    let Ok(solids) = StepImporter::import_solids_from_file(&path) else {
        println!("WRONG 1 {path} が読めません");
        println!("**1 miss(es)**");
        exit(1);
    };
    let Some(read) = solids.into_iter().max_by_key(|s| s.outer_shell.faces.len()) else {
        println!("WRONG 1 立体がありません");
        println!("**1 miss(es)**");
        exit(1);
    };
    println!("{path}: 面 {} 枚", read.outer_shell.faces.len());
    println!();
    println!("面   積分(16)      積分(128)     三角形(16)    三角形(128)   積分が動いた  三角形が動いた");

    let grid = |n: usize| TessellationParams {
        u_divisions: n,
        v_divisions: n,
    };
    let mut frozen = 0usize;
    for (index, face) in read.outer_shell.faces.iter().enumerate() {
        let i16 = MassCalculator::compute_face_integral(face, &grid(16)).0;
        let i128 = MassCalculator::compute_face_integral(face, &grid(128)).0;
        let m16 = mesh_area(face, &grid(16));
        let m128 = mesh_area(face, &grid(128));
        // **積分が止まっていて、三角形は動いている面**を数えます。
        let integral_still = (i128 - i16).abs() <= 1e-12 * i16.abs().max(1.0);
        let mesh_moved = (m128 - m16).abs() > 1e-9 * m16.abs().max(1.0);
        if integral_still && mesh_moved {
            frozen += 1;
        }
        println!(
            "{index:<4} {i16:<13.6} {i128:<13.6} {m16:<13.6} {m128:<13.6} {:<13} {}",
            if integral_still { "いいえ" } else { "はい" },
            if mesh_moved { "はい" } else { "いいえ" },
        );
    }
    println!();
    if frozen == 0 {
        println!("**積分が刻みを見ていない面はありません。**");
    } else {
        println!("**積分が止まっていて三角形は動く面: {frozen} 枚**");
        println!("（**これは赤ではありません**——解析の道を通っていれば、止まって当然です。4-680）");
    }
}
