//! **周期的な曲面の上に、内側の輪はあるか**（4-411）。
//!
//! # なぜ要るのか
//!
//! 4-411 で 2 か所を直しました——**周期の端を隣に近いほうへ寄せる**のと、
//! **継ぎ目に沿う稜を、稜そのものを見て決める**のと。
//!
//! **どちらも、面の輪ごとに走ります**（`derive_wire_nurbs_boundary_pcurves`
//! は外側の輪と内側の輪の両方から呼ばれ、`settle_seam_segments` にはその
//! 輪が渡ります）。**作りの上では、内側の輪も同じように直ります。**
//!
//! **ただし、それを踏む検体を 1 つも測っていませんでした。**
//! **「直るはず」と「直ると測った」は別**です（この文書の 4-397 と同じ話）。
//!
//! # 何を数えるか
//!
//! **追跡済みの検体だけ**を読みます（`crates/zenith_algo/tests/fixtures/`）。
//! **`reference/` は要りません**——2026/09/08 に消えているからです。
//!
//! 面ごとに——
//!
//! - 曲面が **nurbs** か
//! - その軸が **周期的**か（両端が同じ 3D 点に写るか）
//! - **内側の輪**を持つか
//!
//! **「周期的な nurbs 面の、内側の輪」が 0 なら、そこは今も測れていません**
//! ——**検体が要ります**。0 でなければ、門はもう踏んでいます。
//!
//! # 読み方
//!
//! **これは診断です。赤にはしません。** 数を出すだけです。

use std::path::PathBuf;
use zenith_geom::NurbsSurface3;
use zenith_io::StepImporter;
use zenith_math::Tolerance;
use zenith_topo::{Face, FaceGeometry, Solid};

fn fixtures_dir() -> PathBuf {
    PathBuf::from(concat!(env!("CARGO_MANIFEST_DIR"), "/tests/fixtures"))
}

fn faces_of(solid: &Solid) -> Vec<Face> {
    let mut faces = solid.outer_shell.faces.clone();
    for inner in &solid.inner_shells {
        faces.extend(inner.faces.clone());
    }
    faces
}

/// **`settle_seam_segment_axis` と同じ判定**です——両端が同じ 3D 点に写るか。
fn periodic(surface: &NurbsSurface3, along_u: bool, tol: &Tolerance) -> bool {
    let ((u_lo, u_hi), (v_lo, v_hi)) = surface.param_range();
    let (lo, hi) = if along_u { (u_lo, u_hi) } else { (v_lo, v_hi) };
    if hi - lo <= 0.0 {
        return false;
    }
    let middle = if along_u {
        (v_lo + v_hi) * 0.5
    } else {
        (u_lo + u_hi) * 0.5
    };
    let (low, high) = if along_u {
        (surface.evaluate(lo, middle), surface.evaluate(hi, middle))
    } else {
        (surface.evaluate(middle, lo), surface.evaluate(middle, hi))
    };
    (high - low).norm() <= tol.linear.max(1e-9)
}

fn main() {
    let tol = Tolerance::default();

    println!("周期的な曲面の上に、内側の輪はあるか（4-411）");
    println!();
    println!("**追跡済みの検体だけ**を読みます。`reference/` は要りません。");
    println!();
    println!(
        "{:<44}{:>8}{:>10}{:>12}{:>14}",
        "検体", "面", "nurbs", "周期的", "内側の輪あり"
    );
    println!("{}", "-".repeat(90));

    let Ok(entries) = std::fs::read_dir(fixtures_dir()) else {
        println!("検体のあるところが読めません: {:?}", fixtures_dir());
        return;
    };
    let mut names: Vec<PathBuf> = entries
        .filter_map(|entry| entry.ok().map(|e| e.path()))
        .filter(|path| path.extension().and_then(|e| e.to_str()) == Some("step"))
        .collect();
    names.sort();

    let (mut all_faces, mut all_nurbs, mut all_periodic, mut all_inner) = (0, 0, 0, 0);
    let mut examples: Vec<String> = Vec::new();

    for path in &names {
        let Ok(solids) = StepImporter::import_solids_from_file(path) else {
            continue;
        };
        let (mut faces, mut nurbs, mut periodic_count, mut inner) = (0, 0, 0, 0);
        for solid in &solids {
            for face in faces_of(solid) {
                faces += 1;
                let FaceGeometry::Nurbs(surface) = &face.geometry else {
                    continue;
                };
                nurbs += 1;
                let is_periodic =
                    periodic(surface, true, &tol) || periodic(surface, false, &tol);
                if is_periodic {
                    periodic_count += 1;
                }
                if is_periodic && !face.inner_wires.is_empty() {
                    inner += 1;
                    examples.push(format!(
                        "  {}: 内側の輪 {} 本",
                        path.file_name().unwrap_or_default().to_string_lossy(),
                        face.inner_wires.len()
                    ));
                }
            }
        }
        if faces == 0 {
            continue;
        }
        println!(
            "{:<44}{:>8}{:>10}{:>12}{:>14}",
            path.file_name().unwrap_or_default().to_string_lossy(),
            faces,
            nurbs,
            periodic_count,
            inner
        );
        all_faces += faces;
        all_nurbs += nurbs;
        all_periodic += periodic_count;
        all_inner += inner;
    }

    println!("{}", "-".repeat(90));
    println!(
        "{:<44}{:>8}{:>10}{:>12}{:>14}",
        "合計", all_faces, all_nurbs, all_periodic, all_inner
    );
    println!();
    if all_inner == 0 {
        println!("**周期的な nurbs 面の内側の輪は、1 つもありません。**");
        println!("**4-411 の 2 つの直しは、作りの上では内側の輪にも走りますが、**");
        println!("**それを踏む検体がありません**——検体を作るところからです。");
    } else {
        println!("**踏んでいます**（{all_inner} 面）:");
        for line in examples.iter().take(10) {
            println!("{line}");
        }
    }
    println!();
    println!("**これは診断です。赤にはしません。**");
}
