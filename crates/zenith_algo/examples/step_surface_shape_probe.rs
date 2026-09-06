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
use zenith_algo::{cylinder_patch_is_recognized, HoleBuilder, PrimitiveBuilder};
use zenith_io::StepImporter;
use zenith_math::Tolerance;
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

/// 1 つの立体を数えて、表を 1 つ出す。
fn report(title: &str, subject: &Solid, tol: &Tolerance) {
    let mut shapes: BTreeMap<String, usize> = BTreeMap::new();
    let mut planes = 0usize;
    let mut recognized = 0usize;
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
                // **推し量らずに、本物の認識器に訊きます**（4-372）。
                if cylinder_patch_is_recognized(surface, tol) {
                    recognized += 1;
                }
                *shapes.entry(key).or_insert(0) += 1;
            }
            _ => {
                *shapes.entry("その他の曲面".to_string()).or_insert(0) += 1;
            }
        }
    }

    let nurbs: usize = shapes.values().sum();
    println!("## {title}");
    println!();
    println!("平面 {planes} 枚、曲面 {nurbs} 枚。");
    println!(
        "**認識器が受け付けたのは {recognized} 枚**（曲面の {:.0}%）。",
        if nurbs == 0 {
            0.0
        } else {
            100.0 * recognized as f64 / nurbs as f64
        }
    );
    println!();
    println!("| 曲面の持ち方 | 枚数 |");
    println!("| :--- | ---: |");
    let mut rows: Vec<(&String, &usize)> = shapes.iter().collect();
    rows.sort_by(|left, right| right.1.cmp(left.1));
    for (shape, count) in rows.into_iter().take(10) {
        println!("| {shape} | {count} |");
    }
    println!();
}

fn main() {
    println!("**読んだ面の曲面は、どんな形をしているか**（4-371、4-372）");
    println!();
    println!("認識器が受け付けるのは **degree_u=2 / degree_v=1 / 制御点 3×2** だけです。");
    println!();

    let tol = Tolerance::default();

    // **自作の立体**（認識器はこの持ち方のために書かれています）。
    for (title, solid) in [
        (
            "自作: 円柱（`make_cylinder`）",
            PrimitiveBuilder::make_cylinder(10.0, 40.0).ok(),
        ),
        (
            "自作: 穴あきの箱（`make_drilled_box`）",
            HoleBuilder::make_drilled_box(40.0, 40.0, 20.0, 8.0).ok(),
        ),
    ] {
        if let Some(solid) = solid {
            report(title, &solid, &tol);
        }
    }

    // **読んだファイル。**
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
        report(&format!("読んだ: `{name}`"), subject, &tol);
    }

    println!("**これは診断です。赤にはしません。** 読んだファイルの持ち方は");
    println!("相手が決めることで、それ自体は欠陥ではありません（4-266）。");
}
