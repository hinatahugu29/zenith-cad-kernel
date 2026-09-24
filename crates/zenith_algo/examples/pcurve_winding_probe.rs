//! **輪を逆に組んだら、p-curve の輪の符号は変わるか**（4-542）。
//!
//! # なぜ要るのか
//!
//! `linkrods` の積で、**1 枚だけ隣とかみ合わない片**（A面818）があります
//! （4-541）。**割る段でその片の輪を反転しても、選ぶ段で読む巻き方は
//! 変わりません**（4-542）——**入口が ＋ でも − でも、出口は −**。
//!
//! **どこで決め直されているのか。** いちばん疑わしいのは、
//! **`Face::pcurves` が p-curve を導くとき**です。**測るのはそこ 1 点**——
//! **同じ面で、輪だけ反転して導き直し、符号が変わるかどうか。**
//!
//! * **変われば**、p-curve は輪に従っています（犯人は別）
//! * **変わらなければ**、**向きを決めているのは p-curve を導く側**です
//!
//! # 走らせ方
//!
//! ```text
//! cargo run --release -p zenith_algo --example pcurve_winding_probe
//! ```
//!
//! **読んだ面と、自作の面の両方**を見ます——**読んだ面だけだと、
//! ファイルの粗さのせいかどうかが言えません。**

use std::path::PathBuf;

use zenith_io::StepImporter;
use zenith_topo::{Face, OrientedEdge, Solid, Wire};

fn reversed_wire(wire: &Wire) -> Wire {
    Wire::new(
        wire.edges
            .iter()
            .rev()
            .map(|oriented| {
                OrientedEdge::new(oriented.edge.clone(), oriented.orientation.reversed())
            })
            .collect(),
    )
}

fn report(label: &str, face: &Face) {
    let forward = zenith_tess::face_signed_parameter_area(face);
    let flipped_face = Face::new(
        face.geometry.clone(),
        reversed_wire(&face.outer_wire),
        face.inner_wires.clone(),
        face.orientation,
        face.tolerance,
    );
    let backward = zenith_tess::face_signed_parameter_area(&flipped_face);
    let show = |value: Option<f64>| {
        value
            .map(|v| format!("{v:+.9e}"))
            .unwrap_or_else(|| "読めません".to_string())
    };
    let verdict = match (forward, backward) {
        (Some(a), Some(b)) if a.signum() != b.signum() => "**符号が変わります**（輪に従っています）",
        (Some(_), Some(_)) => "**符号が変わりません** ← **p-curve を導く側が決めています**",
        _ => "（どちらかが読めません）",
    };
    println!("  {label}: そのまま {} / 逆に組む {} → {verdict}", show(forward), show(backward));
}

fn main() {
    println!("輪を逆に組んだら、p-curve の輪の符号は変わるか（4-542）");
    println!();

    println!("自作の面（粗さ 1e-6）:");
    let box_solid = zenith_algo::PrimitiveBuilder::make_box(10.0, 6.0, 4.0).expect("箱");
    for (index, face) in box_solid.outer_shell.faces.iter().enumerate().take(2) {
        report(&format!("箱の面{index}（平面）"), face);
    }
    let cylinder = zenith_algo::PrimitiveBuilder::make_cylinder(3.0, 5.0).expect("円柱");
    for (index, face) in cylinder.outer_shell.faces.iter().enumerate() {
        report(&format!("円柱の面{index}"), face);
    }

    println!();
    println!("読んだ面（`linkrods.step`）:");
    let path = PathBuf::from("reference/OCCT/data/step/linkrods.step");
    let Ok(solids) = StepImporter::import_solids_from_file(&path) else {
        println!("  {} が読めません", path.display());
        return;
    };
    let Some(read): Option<&Solid> = solids.iter().max_by_key(|s| s.outer_shell.faces.len()) else {
        return;
    };
    // **4-541 の面**（A面1）と、その隣を数枚。
    for index in [1usize, 13, 14, 35] {
        let Some(face) = read.outer_shell.faces.get(index) else {
            continue;
        };
        report(&format!("A面{index}"), face);
    }
}
