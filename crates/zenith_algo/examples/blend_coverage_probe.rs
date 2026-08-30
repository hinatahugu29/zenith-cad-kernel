//! **フィレット／面取りが、どの稜をなぜ断るのか**（HANDOVER 9-H の H4）。
//!
//! `foreign_edit_probe` は「対象稜 0」としか言いません。**0 の理由は
//! そこに出ていません**——実装していないのか、実装してあるのに届いて
//! いないのかが区別できません。
//!
//! この探針は、検体ごとに**全部の稜を1本ずつ**当てて、断り文を数えます。
//!
//! ```bash
//! cargo run --release -p zenith_algo --example blend_coverage_probe
//! ```

use std::collections::BTreeMap;
use std::path::PathBuf;
use zenith_algo::EdgeBlender;
use zenith_io::StepImporter;
use zenith_math::{Point3, Vec3, Vec3Ext};
use zenith_topo::{FaceGeometry, Solid};

fn load(name: &str) -> Option<Solid> {
    let path = PathBuf::from(concat!(env!("CARGO_MANIFEST_DIR"), "/tests/fixtures"))
        .join(format!("occ_reference_{name}.step"));
    let text = std::fs::read_to_string(&path).ok()?;
    StepImporter::import_solids_from_str(&text)
        .ok()?
        .into_iter()
        .next()
}

/// 立体が持つ稜の `id` を、重複なく集める。
fn edge_ids(solid: &Solid) -> Vec<u64> {
    let mut ids: Vec<u64> = Vec::new();
    for face in &solid.outer_shell.faces {
        for wire in std::iter::once(&face.outer_wire).chain(face.inner_wires.iter()) {
            for oriented in &wire.edges {
                if !ids.contains(&oriented.edge.id) {
                    ids.push(oriented.edge.id);
                }
            }
        }
    }
    ids
}

/// 稜の**二面角**を、面の法線から測る。
///
/// # なぜ探針側で測るのか
///
/// `blendability` は「直線でない」を二面角より**先に**断ります。だから
/// 断り文からは、その稜が**接線連続（丸めるものが無い）なのか、丸めたいのに
/// 届いていないのか**が区別できません。
///
/// **その区別が H4 の意味を決めます**——既に丸めた箱（`filleted_box`）は、
/// 稜が全部なめらかなら **0 で当然**です。守備範囲の穴ではありません。
fn dihedral_at(solid: &Solid, edge_id: u64) -> Option<f64> {
    let faces = &solid.outer_shell.faces;
    let mut sides: Vec<(usize, Point3)> = Vec::new();
    for (index, face) in faces.iter().enumerate() {
        for wire in std::iter::once(&face.outer_wire).chain(face.inner_wires.iter()) {
            for oriented in &wire.edges {
                if oriented.edge.id != edge_id {
                    continue;
                }
                let (t_min, t_max) = oriented.edge.curve.param_range();
                let middle = oriented.edge.curve.evaluate((t_min + t_max) * 0.5);
                if !sides.iter().any(|(other, _)| *other == index) {
                    sides.push((index, middle));
                }
            }
        }
    }
    if sides.len() != 2 {
        return None;
    }
    let normal_of = |index: usize, point: Point3| -> Option<Vec3> {
        let face = &faces[index];
        let normal = match &face.geometry {
            FaceGeometry::Plane(plane) => plane.normal,
            FaceGeometry::Nurbs(surface) => {
                let projection =
                    zenith_geom::ExtremumEngine::point_to_surface(point, surface, 64, 1e-13)
                        .ok()?;
                let (_, du, dv) = surface.evaluate_derivatives_1st(projection.u, projection.v);
                du.cross(&dv).try_normalize_safe(1e-12)?
            }
            _ => return None,
        };
        Some(if matches!(face.orientation, zenith_topo::Orientation::Reversed) {
            -normal
        } else {
            normal
        })
    };
    let a = normal_of(sides[0].0, sides[0].1)?;
    let b = normal_of(sides[1].0, sides[1].1)?;
    Some(a.dot(&b).clamp(-1.0, 1.0).acos().to_degrees())
}

/// 断り文から、**数えられる見出し**を作る。長さや角度は落とします。
fn headline(reason: &str) -> String {
    let cut = reason
        .split(|c| c == '(' || c == ':')
        .next()
        .unwrap_or(reason)
        .trim();
    cut.chars().take(72).collect()
}

/// **1本の稜のまわりを、そのまま出す。** 推測を重ねる前に実物を見るための口。
///
/// ```bash
/// ZENITH_BLEND_DUMP=chamfered_box:14 cargo run --release -p zenith_algo ///   --example blend_coverage_probe
/// ```
fn dump(solid: &Solid, edge_id: u64) {
    let faces = &solid.outer_shell.faces;
    let mut target: Option<(Point3, Point3)> = None;
    for face in faces {
        for wire in std::iter::once(&face.outer_wire).chain(face.inner_wires.iter()) {
            for oriented in &wire.edges {
                if oriented.edge.id == edge_id {
                    target = Some((
                        oriented.edge.start_vertex.point,
                        oriented.edge.end_vertex.point,
                    ));
                }
            }
        }
    }
    let Some((start, end)) = target else {
        println!("稜 {edge_id} が見つかりません");
        return;
    };
    println!();
    println!("稜 {edge_id}: ({:.4} {:.4} {:.4}) -> ({:.4} {:.4} {:.4})", start.x, start.y, start.z, end.x, end.y, end.z);
    let dir = (end - start).normalize();
    println!("向き ({:.4} {:.4} {:.4})", dir.x, dir.y, dir.z);
    for (label, vertex) in [("始点", start), ("終点", end)] {
        println!("  {label} に集まる面:");
        for (index, face) in faces.iter().enumerate() {
            let ids: Vec<u64> = std::iter::once(&face.outer_wire)
                .chain(face.inner_wires.iter())
                .flat_map(|wire| wire.edges.iter())
                .filter(|oriented| {
                    (oriented.start_vertex().point - vertex).norm() <= 1e-9
                        || (oriented.end_vertex().point - vertex).norm() <= 1e-9
                })
                .map(|oriented| oriented.edge.id)
                .collect();
            if ids.is_empty() {
                continue;
            }
            let kind = match &face.geometry {
                FaceGeometry::Plane(plane) => format!(
                    "平面 法線 ({:.4} {:.4} {:.4}) 稜との内積 {:.4}",
                    plane.normal.x, plane.normal.y, plane.normal.z,
                    plane.normal.dot(&dir)
                ),
                FaceGeometry::Nurbs(_) => "曲面".to_string(),
                _ => "その他".to_string(),
            };
            println!("    面{index:<3} 稜 {ids:?}  {kind}");
        }
    }
}

fn main() {
    if let Ok(spec) = std::env::var("ZENITH_BLEND_DUMP") {
        let mut parts = spec.splitn(2, ':');
        let name = parts.next().unwrap_or("");
        let id: u64 = parts.next().and_then(|t| t.parse().ok()).unwrap_or(0);
        if let Some(solid) = load(name) {
            dump(&solid, id);
        } else {
            println!("{name} が読めません");
        }
        return;
    }
    let names = [
        "elliptic_prism",
        "sphere",
        "sphere_capped",
        "torus",
        "torus_segment",
        "chamfered_box",
        "filleted_box",
        "hollow_box",
        "stepped_shaft",
        "plate_with_holes",
    ];

    println!("フィレット／面取りの守備範囲（断る理由を数えます）");
    println!();
    println!(
        "{:<20} {:>5} {:>5} {:>7}  {}",
        "fixture", "edges", "鋭い", "blendable", "断る理由（多い順）"
    );
    println!("{}", "-".repeat(110));

    let mut zero = 0usize;
    let mut read = 0usize;
    let mut all_reasons: BTreeMap<String, usize> = BTreeMap::new();
    let mut sharp_total: Vec<(&str, usize, usize, usize)> = Vec::new();

    for name in names {
        let Some(solid) = load(name) else {
            println!("{name:<20} {:>5} {:>7}  読めません", "-", "-");
            continue;
        };
        read += 1;
        let ids = edge_ids(&solid);
        let mut ok = 0usize;
        // **丸める意味のある稜**を数えます。二面角が 180 度に近いなら
        // なめらかに繋がっているので、丸めるものがありません。
        let mut sharp = 0usize;
        let mut reasons: BTreeMap<String, usize> = BTreeMap::new();
        for id in &ids {
            // **なめらかな繋ぎでは、両側の法線が一致します（角 0度）。**
            //
            // 最初は 180 度と比べていました——**間違いです**。180 度は面が
            // 折り返している場合で、接線連続ではありません。実測で
            // `filleted_box` の 48稜すべてを「鋭い」と数えてしまい、そこで
            // 気づきました。
            let is_sharp = dihedral_at(&solid, *id)
                .map(|angle| angle > 1.0)
                .unwrap_or(false);
            if is_sharp {
                sharp += 1;
            }
            match EdgeBlender::blendability(&solid, *id) {
                Ok(_) => ok += 1,
                Err(reason) => {
                    let key = headline(&reason);
                    *reasons.entry(key.clone()).or_default() += 1;
                    *all_reasons.entry(key).or_default() += 1;
                }
            }
        }
        // **穴と数えるのは「丸める意味のある稜があるのに 0本」**だけです。
        if ok == 0 && sharp > 0 {
            zero += 1;
        }
        sharp_total.push((name, ids.len(), sharp, ok));
        let mut sorted: Vec<(&String, &usize)> = reasons.iter().collect();
        sorted.sort_by(|a, b| b.1.cmp(a.1));
        let shown: Vec<String> = sorted
            .iter()
            .take(2)
            .map(|(reason, count)| format!("{count}× {reason}"))
            .collect();
        println!(
            "{name:<20} {:>5} {:>5} {:>7}  {}",
            ids.len(),
            sharp,
            ok,
            if shown.is_empty() {
                "—".to_string()
            } else {
                shown.join(" / ")
            }
        );
    }

    println!("{}", "-".repeat(110));
    println!(
        "{read} 検体を読み、**{zero} 件が「丸める稜があるのに 0本」** です（9-H の H4 は 3件以下）。"
    );
    println!();
    println!("**「鋭い」が 0 の検体は、0本で当然です。** 稜が無いか、全部");
    println!("なめらかに繋がっているので、丸めるものがありません:");
    for (name, edges, sharp, ok) in &sharp_total {
        if *sharp == 0 {
            println!("  {name:<20} 稜 {edges:>3}、鋭い 0、丸められた {ok}");
        }
    }
    println!();
    println!("断る理由の合計:");
    let mut sorted: Vec<(&String, &usize)> = all_reasons.iter().collect();
    sorted.sort_by(|a, b| b.1.cmp(a.1));
    for (reason, count) in sorted.iter().take(12) {
        println!("  {count:>4}  {reason}");
    }
    println!();
    println!("**数えるのは「丸める意味のある稜があるのに、1本も掛からない」ほう**です。");
    println!("稜が無い立体や、全部なめらかに繋がっている立体は 0 で当然です。");
}
