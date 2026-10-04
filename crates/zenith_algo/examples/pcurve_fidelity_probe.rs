//! How far a face's p-curve really is from its 3D edge.
//!
//! Shell validation asks `validate_pcurves(tol, 8)`, and a NURBS face's
//! p-curves are built by projecting each edge at 8 evenly spaced parameters.
//! The check therefore lands on the very points the p-curve was built from,
//! where it is exact by construction, and reports a deviation of zero for
//! curves that are nowhere near their edges in between.
//!
//! This probe measures the same distance at several sample counts. A p-curve
//! that is genuinely on its edge reads small at every count. One that is only
//! pinned at its construction points reads zero at 8 and large everywhere else,
//! and the gap between those two columns is the size of the illusion.
//!
//! Run with: cargo run --release -p zenith_algo --example pcurve_fidelity_probe

use std::fs;
use std::path::Path;

use zenith_geom::Surface3;
use zenith_io::StepImporter;
use zenith_math::{Point3, Tolerance};
use zenith_topo::{Face, FaceGeometry, Solid};

/// The worst distance between a face's p-curves and its 3D edges, measured at
/// `samples` evenly spaced parameters per edge - the same way validation does.
fn worst_pcurve_distance(face: &Face, samples: usize) -> Option<f64> {
    let tol = Tolerance::default();
    let pcurves = face.pcurves(&tol).ok()?;
    let evaluate = |u: f64, v: f64| -> Option<Point3> {
        match &face.geometry {
            FaceGeometry::Plane(plane) => Some(plane.evaluate(u, v)),
            FaceGeometry::Nurbs(surface) => Some(surface.evaluate(u, v)),
            _ => None,
        }
    };

    let mut worst: f64 = 0.0;
    for (edge, segment) in face
        .outer_wire
        .edges
        .iter()
        .zip(pcurves.outer_loop.segments.iter())
    {
        let (t_min, t_max) = segment.curve.param_range();
        for step in 0..=samples {
            let fraction = step as f64 / samples as f64;
            let uv = segment.curve.evaluate(t_min + (t_max - t_min) * fraction);
            let Some(from_pcurve) = evaluate(uv.x, uv.y) else {
                return None;
            };
            let from_edge = edge.evaluate_normalized(fraction);
            worst = worst.max((from_pcurve - from_edge).norm());
        }
    }
    Some(worst)
}

/// **閾値は持ち込みません**（4-663）。見張るのは**入れ子の形**だけです——
/// **8 ⊂ 16 ⊂ 64** は、同じ曲線を「前に見た所ぜんぶ＋もっと」で測った
/// ものなので、**最悪値は決して下がれません**。下がったら、測り方か
/// p-curve の評価が壊れています。
///
/// **9 と 37 は入れ子ではありません**（37 は 9 の倍数ではない）ので、
/// **ここでは見ません**——実測でも 128 行中 19 行で下がります。
/// **どの列をどの値で縛るかは、まだ決めていません**（持ち主の判断）。
/// **この床は、その決定を待たずに置けます。**
fn report(name: &str, solids: &[Solid], broken: &mut Vec<String>, rows: &mut usize) {
    let counts = [8usize, 9, 16, 37, 64];
    for solid in solids {
        for (index, face) in solid.outer_shell.faces.iter().enumerate() {
            let kind = match &face.geometry {
                FaceGeometry::Plane(_) => "plane",
                FaceGeometry::Nurbs(_) => "nurbs",
                _ => continue,
            };
            let raw: Vec<Option<f64>> = counts
                .iter()
                .map(|count| worst_pcurve_distance(face, *count))
                .collect();
            *rows += 1;
            // **入れ子の列だけ**（8 / 16 / 64 → 添字 0 / 2 / 4）。
            for (lo, hi) in [(0usize, 2usize), (2, 4)] {
                if let (Some(a), Some(b)) = (raw[lo], raw[hi]) {
                    if b < a * (1.0 - 1e-9) {
                        broken.push(format!(
                            "{name} face {index} {kind}: {} 点で {a:.3e}、{} 点で {b:.3e}（入れ子なのに下がりました）",
                            counts[lo], counts[hi]
                        ));
                    }
                }
            }
            let measured: Vec<String> = raw
                .iter()
                .map(|value| match value {
                    Some(distance) => format!("{distance:>10.3e}"),
                    None => "         -".to_string(),
                })
                .collect();
            println!(
                "{:<38} face {index:>2} {kind:<6} {}",
                name,
                measured.join(" ")
            );
        }
    }
}

fn main() {
    println!("worst p-curve to 3D edge distance, by how many samples the check takes");
    println!(
        "{:<38} {:>8} {}",
        "file",
        "face",
        [8usize, 9, 16, 37, 64]
            .iter()
            .map(|count| format!("{count:>10}"))
            .collect::<Vec<_>>()
            .join(" ")
    );
    println!("{}", "-".repeat(112));
    println!("validation asks for 8, and the p-curves are built from 8; the other columns");
    println!("are the same curves measured anywhere else.");
    println!();

    // **測る相手が居ないのは、緑ではありません**（4-656）。
    //
    // **この探りは、入力が無いことに気づいて、そう印字していました**
    // ——**そして `return` して 0 で終わっていました。** **検体 0 件で
    // 緑**です。**`export_validation_suite` は `run_gates.sh` の中で
    // `|| true` 付きで呼ばれる**（4-642）ので、**書き出しが黙って
    // 失敗したら、この門は何も測らずに通ります。**
    //
    // **閾値の判断は要りません**——**0 件は 0 件**です。
    let validation = Path::new("target/validation");
    if !validation.is_dir() {
        println!("target/validation is missing; run the export_validation_suite example first");
        std::process::exit(1);
    }

    let mut paths: Vec<_> = fs::read_dir(validation)
        .unwrap()
        .filter_map(|entry| entry.ok())
        .map(|entry| entry.path())
        .filter(|path| path.extension().map(|ext| ext == "step").unwrap_or(false))
        .filter(|path| {
            path.file_name()
                .map(|name| name.to_string_lossy().starts_with("occ_reference"))
                .unwrap_or(false)
        })
        .collect();
    paths.sort();
    if paths.is_empty() {
        println!("**target/validation に occ_reference の検体が 1 つもありません**（4-656）。");
        std::process::exit(1);
    }

    let mut broken: Vec<String> = Vec::new();
    let mut rows = 0usize;

    for path in paths {
        let name = path.file_name().unwrap().to_string_lossy().to_string();
        match StepImporter::import_solids_from_file(&path) {
            Ok(solids) => report(&name, &solids, &mut broken, &mut rows),
            Err(err) => println!(
                "{name:<38} refused: {}",
                err.chars().take(90).collect::<String>()
            ),
        }
    }

    println!();
    if rows == 0 {
        println!("**面を 1 枚も測っていません。**");
        std::process::exit(1);
    }
    if broken.is_empty() {
        println!("**{rows} 行すべてで 8 ≦ 16 ≦ 64 が成り立ちます**（入れ子の単調性。4-663）。");
    } else {
        for line in &broken {
            println!("WRONG 1 {line}");
        }
        println!("**{} miss(es)**", broken.len());
        std::process::exit(1);
    }
}
