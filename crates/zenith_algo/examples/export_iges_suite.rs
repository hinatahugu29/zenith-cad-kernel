//! IGES 5.3 の出力を書き出し、外から突き合わせるための検体を作る。
//!
//! 出力は `target/iges/` に置き、同じ場所に `manifest.json` を書く。
//! 突き合わせは `tools/verify_iges.py`（FreeCAD / OpenCASCADE）で行う。
//!
//! **4-769 でトリム（Entity 144/142/126）を書けるようになった**——
//! `face.pcurves` を持つ面（自前ビルダーの面がほぼ全部）は、読み込んだ
//! 境界箱が元の立体とほぼ一致するはず。`pcurves` を持たない面
//! （他カーネルから読んだ面）は以前と同じ、トリム前の土台（Entity 128
//! の曲面だけ）のまま——境界箱は元の立体より大きくなることはあっても
//! 小さくなってはいけない。**ROADMAP 柱⑤の次の一手**として、
//! `shape_variety_probe` と同じ検体の広がり（トーラス・段付き軸）と、
//! 他カーネルから読んだ面（`pcurves` 無し）の両方をここに足した。

use std::fs;
use std::path::PathBuf;

use zenith_algo::{HoleBuilder, MassCalculator, PrimitiveBuilder, ShaftBuilder};
use zenith_io::IgesExporter;
use zenith_math::Point3;
use zenith_topo::Solid;

fn fixture(name: &str) -> PathBuf {
    PathBuf::from(concat!(env!("CARGO_MANIFEST_DIR"), "/tests/fixtures"))
        .join(format!("occ_reference_{name}.step"))
}

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
                        low =
                            Point3::new(low.x.min(point.x), low.y.min(point.y), low.z.min(point.z));
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
        Subject {
            name: "torus_R12_r4",
            solid: PrimitiveBuilder::make_torus(12.0, 4.0).unwrap(),
        },
        Subject {
            name: "stepped_shaft",
            solid: ShaftBuilder::make_stepped_shaft(&[(10.0, 20.0), (6.0, 15.0), (4.0, 10.0)])
                .unwrap(),
        },
    ];

    // **他カーネルから読んだ面（`pcurves` 無し）の経路も、この掃き出しに
    // 含める**——読めなければここで気づけるよう、`unwrap` せずに名指しで
    // 飛ばす（4-411 と同じ、「無い」を「緑」と読み違えない流儀）。
    let mut subjects = subjects;
    for name in ["cylinder", "torus"] {
        let path = fixture(name);
        if !path.exists() {
            eprintln!("skip foreign:{name} — {} が無い", path.display());
            continue;
        }
        match zenith_io::StepImporter::import_solid_from_file(&path) {
            Ok(solid) => subjects.push(Subject {
                name: Box::leak(format!("foreign_{name}").into_boxed_str()),
                solid,
            }),
            Err(error) => eprintln!("skip foreign:{name} — 読めません: {error}"),
        }
    }

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
        let all_pcurves = subject.solid.outer_shell.faces.iter().all(|f| f.pcurves.is_some())
            && subject
                .solid
                .inner_shells
                .iter()
                .flat_map(|shell| shell.faces.iter())
                .all(|f| f.pcurves.is_some());
        let params = zenith_tess::TessellationParams {
            u_divisions: 64,
            v_divisions: 64,
        };
        let surface_area = MassCalculator::compute_from_brep(&subject.solid, &params).surface_area;
        manifest.push_str(&format!(
            "  {{\"name\": \"{}\", \"file\": \"{}.igs\", \"faces\": {}, \
\"low\": [{}, {}, {}], \"high\": [{}, {}, {}], \"all_pcurves\": {}, \"surface_area\": {}}}{}\n",
            subject.name,
            subject.name,
            faces,
            low.x,
            low.y,
            low.z,
            high.x,
            high.y,
            high.z,
            all_pcurves,
            surface_area,
            if index + 1 == subjects.len() { "" } else { "," }
        ));
    }
    manifest.push_str("]\n");
    fs::write(directory.join("manifest.json"), manifest).expect("write manifest");

    println!("{}", "-".repeat(72));
    println!("wrote {} IGES file(s) to target/iges", subjects.len());
    println!("cross-check with: tools/verify_iges.py (FreeCAD / OpenCASCADE)");
}
