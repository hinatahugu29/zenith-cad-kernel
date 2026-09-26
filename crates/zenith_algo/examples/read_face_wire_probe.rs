//! **読んだ面の輪を、そのまま出す口**（4-549）。
//!
//! **なぜ要るか**: 4-548 で救った A面1 の「外」の片は、
//! **外周が行って戻る**（同じ稜を順逆 2 度使う）形をしていました。
//! **割った結果がそうなのか、読んだ面が最初からそうなのか**が
//! 分からないと、直す場所が決まりません。**読んだ面を、割る前に見ます。**
//!
//! 使い方: `read_face_wire_probe <面番号|all>`
use zenith_io::StepImporter;

fn main() {
    let which = std::env::args().nth(1).unwrap_or_else(|| "1".to_string());
    let sample = "reference/OCCT/data/step/linkrods.step";
    let Ok(solids) = StepImporter::import_solids_from_file(sample) else {
        println!("{sample} が読めません");
        return;
    };
    let Some(read) = solids.into_iter().max_by_key(|s| s.outer_shell.faces.len()) else {
        return;
    };
    println!("面 {} 枚", read.outer_shell.faces.len());
    for (index, face) in read.outer_shell.faces.iter().enumerate() {
        if which != "all" && which != index.to_string() {
            continue;
        }
        println!(
            "面{index} {} 外周 {} 本 内輪 {} 本",
            surface_name(&face.geometry),
            face.outer_wire.edges.len(),
            face.inner_wires.len()
        );
        for (slot, wire) in std::iter::once(&face.outer_wire)
            .chain(face.inner_wires.iter())
            .enumerate()
        {
            println!("  輪{slot}（{} 本）", wire.edges.len());
            for oriented in &wire.edges {
                let start = oriented.evaluate_normalized(0.0);
                let end = oriented.evaluate_normalized(1.0);
                println!(
                    "    id {:>4} {} ({:9.5} {:9.5} {:9.5}) -> ({:9.5} {:9.5} {:9.5})",
                    oriented.edge.id,
                    if matches!(oriented.orientation, zenith_topo::Orientation::Reversed) { "逆" } else { "順" },
                    start.x,
                    start.y,
                    start.z,
                    end.x,
                    end.y,
                    end.z
                );
            }
        }
    }
}

fn surface_name(geometry: &zenith_topo::FaceGeometry) -> String {
    format!("{:?}", geometry)
        .split(['(', ' ', '{'])
        .next()
        .unwrap_or("?")
        .to_string()
}
