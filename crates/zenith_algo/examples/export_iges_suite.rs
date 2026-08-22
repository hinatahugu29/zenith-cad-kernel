//! IGES 5.3 の出力を書き出し、外から突き合わせるための検体を作る。
//!
//! 出力は `target/iges/` に置き、同じ場所に `manifest.json` を書く。
//! 突き合わせは `tools/verify_iges.py`（FreeCAD / OpenCASCADE）で行う。
//!
//! IGES 側はトリムを書いていない（Entity 128 の曲面だけ）ので、**体積では
//! 突き合わせられない**。読み込んだ曲面の枚数と、境界箱が一致するかを見る。
//! 曲面はトリム前の土台なので、境界箱は元の立体より大きくなることはあっても
//! 小さくなってはいけない。

use std::fs;

use zenith_algo::{HoleBuilder, PrimitiveBuilder};
use zenith_io::IgesExporter;
use zenith_math::Point3;
use zenith_topo::Solid;

struct Subject {
    name: &'static str,
    solid: Solid,
}

fn bounds(solid: &Solid) -> (Point3, Point3) {
    let mut low = Point3::new(f64::INFINITY, f64::INFINITY, f64::INFINITY);
    let mut high = Point3::new(f64::NEG_INFINITY, f64::NEG_INFINITY, f64::NEG_INFINITY);
    for shell in std::iter::once(&solid.outer_shell).chain(solid.inner_shells.iter()) {
        for face in &shell.faces {
            for wire in std::iter::once(&face.outer_wire).chain(face.inner_wires.iter()) {
                for oriented in &wire.edges {
                    for point in [
                        oriented.edge.start_vertex.point,
                        oriented.edge.end_vertex.point,
                    ] {
                        low = Point3::new(
                            low.x.min(point.x),
                            low.y.min(point.y),
                            low.z.min(point.z),
                        );
                        high = Point3::new(
                            high.x.max(point.x),
                            high.y.max(point.y),
                            high.z.max(point.z),
                        );
                    }
                }
            }
        }
    }
    (low, high)
}

fn main() {
    let subjects = vec![
        Subject {
            name: "box_20x30x40",
            solid: PrimitiveBuilder::make_box(20.0, 30.0, 40.0).unwrap(),
        },
        Subject {
            name: "cylinder_r10_h40",
            solid: PrimitiveBuilder::make_cylinder(10.0, 40.0).unwrap(),
        },
        Subject {
            name: "sphere_r10",
            solid: PrimitiveBuilder::make_sphere(10.0).unwrap(),
        },
        Subject {
            name: "cone_r10_r4_h20",
            solid: PrimitiveBuilder::make_cone(10.0, 4.0, 20.0).unwrap(),
        },
        Subject {
            name: "drilled_box_30x30x15_r5",
            solid: HoleBuilder::make_drilled_box(30.0, 30.0, 15.0, 5.0).unwrap(),
        },
    ];

    let directory = std::path::Path::new("target/iges");
    fs::create_dir_all(directory).expect("create target/iges");

    let mut manifest = String::from("[\n");
    println!(
        "{:<28}{:>8}{:>10}{:>12}",
        "subject", "faces", "records", "file"
    );
    println!("{}", "-".repeat(72));

    for (index, subject) in subjects.iter().enumerate() {
        let faces = subject.solid.outer_shell.faces.len()
            + subject
                .solid
                .inner_shells
                .iter()
                .map(|shell| shell.faces.len())
                .sum::<usize>();
        let path = directory.join(format!("{}.igs", subject.name));
        IgesExporter::export_solid_to_file(&subject.solid, &path, subject.name)
            .unwrap_or_else(|error| panic!("{}: {error}", subject.name));

        let text = fs::read_to_string(&path).expect("read back what we wrote");
        let records = text.lines().count();
        println!(
            "{:<28}{:>8}{:>10}{:>12}",
            subject.name,
            faces,
            records,
            path.file_name().unwrap().to_string_lossy()
        );

        let (low, high) = bounds(&subject.solid);
        manifest.push_str(&format!(
            "  {{\"name\": \"{}\", \"file\": \"{}.igs\", \"faces\": {}, \
\"low\": [{}, {}, {}], \"high\": [{}, {}, {}]}}{}\n",
            subject.name,
            subject.name,
            faces,
            low.x,
            low.y,
            low.z,
            high.x,
            high.y,
            high.z,
            if index + 1 == subjects.len() { "" } else { "," }
        ));
    }
    manifest.push_str("]\n");
    fs::write(directory.join("manifest.json"), manifest).expect("write manifest");

    println!("{}", "-".repeat(72));
    println!("wrote {} IGES file(s) to target/iges", subjects.len());
    println!("cross-check with: tools/verify_iges.py (FreeCAD / OpenCASCADE)");
}
