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
use zenith_topo::Solid;

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

/// 断り文から、**数えられる見出し**を作る。長さや角度は落とします。
fn headline(reason: &str) -> String {
    let cut = reason
        .split(|c| c == '(' || c == ':')
        .next()
        .unwrap_or(reason)
        .trim();
    cut.chars().take(72).collect()
}

fn main() {
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
    println!("{:<20} {:>5} {:>7}  {}", "fixture", "edges", "blendable", "断る理由（多い順）");
    println!("{}", "-".repeat(110));

    let mut zero = 0usize;
    let mut read = 0usize;
    let mut all_reasons: BTreeMap<String, usize> = BTreeMap::new();

    for name in names {
        let Some(solid) = load(name) else {
            println!("{name:<20} {:>5} {:>7}  読めません", "-", "-");
            continue;
        };
        read += 1;
        let ids = edge_ids(&solid);
        let mut ok = 0usize;
        let mut reasons: BTreeMap<String, usize> = BTreeMap::new();
        for id in &ids {
            match EdgeBlender::blendability(&solid, *id) {
                Ok(_) => ok += 1,
                Err(reason) => {
                    let key = headline(&reason);
                    *reasons.entry(key.clone()).or_default() += 1;
                    *all_reasons.entry(key).or_default() += 1;
                }
            }
        }
        if ok == 0 {
            zero += 1;
        }
        let mut sorted: Vec<(&String, &usize)> = reasons.iter().collect();
        sorted.sort_by(|a, b| b.1.cmp(a.1));
        let shown: Vec<String> = sorted
            .iter()
            .take(2)
            .map(|(reason, count)| format!("{count}× {reason}"))
            .collect();
        println!(
            "{name:<20} {:>5} {:>7}  {}",
            ids.len(),
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
        "{read} 検体を読み、**{zero} 件が対象稜 0** です（9-H の H4 は 3件以下）。"
    );
    println!();
    println!("断る理由の合計:");
    let mut sorted: Vec<(&String, &usize)> = all_reasons.iter().collect();
    sorted.sort_by(|a, b| b.1.cmp(a.1));
    for (reason, count) in sorted.iter().take(12) {
        println!("  {count:>4}  {reason}");
    }
    println!();
    println!("**稜が無い立体（球・トーラス）は 0 で当然です。** 数えるのは");
    println!("「稜はあるのに 1本も掛からない」ほうです。");
}
