//! **読んだ面の曲面は、どんな形をしているか**（4-371）。
//!
//! # なぜ要るのか
//!
//! 4-370 で数え直したら、**割れない理由の筆頭**はこれでした。
//!
//! ```text
//! Only recognized cylinder-side NURBS patches can be split   126
//! ```
//!
//! **「割れる形だと認識されていない」**です。**では何なのか**——それが
//! 分からないと、**次に何を書けばいいかが決まりません**。
//!
//! # 何を見るか
//!
//! 認識器（`recognize_cylinder_patch`）が要求するのは、**degree_u = 2、
//! degree_v = 1、制御点 3×2**——**四半円のパッチ 1 枚**です。
//!
//! 読んだファイルの面が、**その形からどれだけ離れているか**を数えます。
//!
//! # 赤にするか
//!
//! **診断です。** 読んだファイルの持ち方は相手が決めることで、
//! それ自体は欠陥ではありません（4-266）。
use std::collections::BTreeMap;
use std::path::PathBuf;
use zenith_io::StepImporter;
use zenith_topo::{FaceGeometry, Solid};

fn occt_sample(name: &str) -> PathBuf {
    PathBuf::from(concat!(
        env!("CARGO_MANIFEST_DIR"),
        "/../../reference/OCCT/data/step"
    ))
    .join(name)
}

fn all_faces(solid: &Solid) -> Vec<&zenith_topo::Face> {
    let mut faces: Vec<&zenith_topo::Face> = solid.outer_shell.faces.iter().collect();
    for inner in &solid.inner_shells {
        faces.extend(inner.faces.iter());
    }
    faces
}

fn main() {
    println!("**読んだ面の曲面は、どんな形をしているか**（4-371）");
    println!();
    println!("認識器が受け付けるのは **degree_u=2 / degree_v=1 / 制御点 3×2** だけです。");
    println!();

    for name in ["screw.step", "linkrods.step"] {
        let Ok(solids) = StepImporter::import_solids_from_file(&occt_sample(name)) else {
            println!("{name}: 読めません");
            continue;
        };
        let Some(subject) = solids
            .iter()
            .max_by(|left, right| all_faces(left).len().cmp(&all_faces(right).len()))
        else {
            continue;
        };

        let mut shapes: BTreeMap<String, usize> = BTreeMap::new();
        let mut planes = 0usize;
        let mut recognizable = 0usize;
        for face in all_faces(subject) {
            match &face.geometry {
                FaceGeometry::Plane(_) => planes += 1,
                FaceGeometry::Nurbs(surface) => {
                    let rows = surface.control_points.len();
                    let columns = surface
                        .control_points
                        .first()
                        .map(|row| row.len())
                        .unwrap_or(0);
                    let ragged = surface
                        .control_points
                        .iter()
                        .any(|row| row.len() != columns);
                    let key = format!(
                        "次数 {}×{}、制御点 {}×{}{}",
                        surface.degree_u,
                        surface.degree_v,
                        rows,
                        columns,
                        if ragged { "（不揃い）" } else { "" }
                    );
                    if surface.degree_u == 2 && surface.degree_v == 1 && rows == 3 && columns == 2 {
                        recognizable += 1;
                    }
                    *shapes.entry(key).or_insert(0) += 1;
                }
                _ => {
                    *shapes.entry("その他の曲面".to_string()).or_insert(0) += 1;
                }
            }
        }

        let nurbs: usize = shapes.values().sum();
        println!("## {name}");
        println!();
        println!("平面 {planes} 枚、曲面 {nurbs} 枚。");
        println!(
            "**認識器の形（次数 2×1、制御点 3×2）は {recognizable} 枚**（曲面の {:.0}%）。",
            if nurbs == 0 {
                0.0
            } else {
                100.0 * recognizable as f64 / nurbs as f64
            }
        );
        println!();
        println!("| 曲面の持ち方 | 枚数 |");
        println!("| :--- | ---: |");
        let mut rows: Vec<(&String, &usize)> = shapes.iter().collect();
        rows.sort_by(|left, right| right.1.cmp(left.1));
        for (shape, count) in rows {
            println!("| {shape} | {count} |");
        }
        println!();
    }

    println!("**これは診断です。赤にはしません。** 読んだファイルの持ち方は");
    println!("相手が決めることで、それ自体は欠陥ではありません（4-266）。");
}
