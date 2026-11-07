//! **フィレットの上にフィレットを掛けられるか**（ROADMAP 柱⑦の第1段）。
//!
//! CAD の日常操作は「丸めたあと、また別の稜を丸める」の繰り返しです。
//! この探針は、検体を1回（または数回）丸めたあと、**残りの稜を1本ずつ
//! 丸めてみて**、通った件数と断り文を数えます。直す前に、何件が・どの
//! 理由で断られるかを測るためのものです。
//!
//! 通った件は「体積が減ったか」も見ます（凸稜のフィレットは削る側）。
//!
//! ```bash
//! cargo run --release -p zenith_algo --example fillet_chain_probe
//! ```

use std::collections::BTreeMap;
use zenith_algo::{EdgeBlender, MassCalculator, PrimitiveBuilder};
use zenith_tess::TessellationParams;
use zenith_topo::{FaceGeometry, Solid};

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

/// 稜の両側の面の種類（"平面×曲面" など）。フィレット面は NURBS で持つので「曲面」。
fn sides(solid: &Solid, edge_id: u64) -> String {
    let mut kinds: Vec<&str> = Vec::new();
    for face in &solid.outer_shell.faces {
        let uses = std::iter::once(&face.outer_wire)
            .chain(face.inner_wires.iter())
            .any(|wire| wire.edges.iter().any(|oriented| oriented.edge.id == edge_id));
        if uses {
            kinds.push(match &face.geometry {
                FaceGeometry::Plane(_) => "平面",
                _ => "曲面",
            });
        }
    }
    kinds.sort_unstable();
    kinds.join("×")
}

fn volume(solid: &Solid) -> f64 {
    MassCalculator::compute_volume_from_brep(
        solid,
        &TessellationParams {
            u_divisions: 64,
            v_divisions: 64,
        },
    )
}

/// 断り文を短くまとめる（数字を落として同じ理由を束ねる）。
fn reason_key(text: &str) -> String {
    let short: String = text
        .chars()
        .map(|c| if c.is_ascii_digit() || c == '.' || c == '-' { '#' } else { c })
        .collect();
    let mut collapsed = String::new();
    for c in short.chars() {
        if c == '#' && collapsed.ends_with('#') {
            continue;
        }
        collapsed.push(c);
    }
    collapsed.chars().take(110).collect()
}

struct Case {
    name: &'static str,
    why: &'static str,
    solid: Result<Solid, String>,
    radius: f64,
}

fn vertical_edges(solid: &Solid) -> Vec<u64> {
    edge_ids(solid)
        .into_iter()
        .filter(|id| {
            solid
                .outer_shell
                .faces
                .iter()
                .flat_map(|face| std::iter::once(&face.outer_wire).chain(face.inner_wires.iter()))
                .flat_map(|wire| wire.edges.iter())
                .find(|oriented| oriented.edge.id == *id)
                .map(|oriented| {
                    let a = oriented.edge.start_vertex.point;
                    let b = oriented.edge.end_vertex.point;
                    (a.x - b.x).abs() < 1e-9 && (a.y - b.y).abs() < 1e-9 && (a.z - b.z).abs() > 1e-9
                })
                .unwrap_or(false)
        })
        .collect()
}

fn cases() -> Vec<Case> {
    let base = PrimitiveBuilder::make_box(20.0, 30.0, 40.0);
    let mut out = Vec::new();

    let one = base.clone().and_then(|solid| {
        let first = vertical_edges(&solid)[0];
        EdgeBlender::fillet_edge(&solid, first, 3.0)
    });
    out.push(Case {
        name: "box, 1 vertical edge filleted (r3)",
        why: "縦稜1本を丸めた直方体。残りの稜、特にフィレット面に接する上下の稜を丸める",
        solid: one,
        radius: 2.0,
    });

    let four = base.clone().and_then(|solid| {
        let edges: Vec<(u64, f64)> = vertical_edges(&solid).into_iter().map(|id| (id, 3.0)).collect();
        EdgeBlender::fillet_edges(&solid, &edges)
    });
    out.push(Case {
        name: "box, 4 vertical edges filleted (r3)",
        why: "縦稜4本を丸めた直方体（角丸の角柱）。上下の縁（直線4本＋円弧4本）を丸める",
        solid: four,
        radius: 2.0,
    });

    let chamfered = base.clone().and_then(|solid| {
        let first = vertical_edges(&solid)[0];
        EdgeBlender::chamfer_edge(&solid, first, 3.0)
    });
    out.push(Case {
        name: "box, 1 vertical edge chamfered (c3)",
        why: "縦稜1本を面取りした直方体。面取りは平面なので、上下の稜は「平面×平面」のまま",
        solid: chamfered,
        radius: 2.0,
    });

    out.push(Case {
        name: "box (no prior blend)",
        why: "比較用: 何も丸めていない直方体",
        solid: base,
        radius: 2.0,
    });
    out
}

fn main() {
    let mut total_ok = 0usize;
    let mut total_refused = 0usize;
    let mut total_wrong = 0usize;
    for case in cases() {
        println!("== {} ==", case.name);
        println!("   {}", case.why);
        let solid = match case.solid {
            Ok(solid) => solid,
            Err(err) => {
                println!("   下ごしらえで断られた: {err}");
                continue;
            }
        };
        let before = volume(&solid);
        let mut ok = 0usize;
        let mut wrong = 0usize;
        let mut reasons: BTreeMap<(String, String), usize> = BTreeMap::new();
        let mut ok_kinds: BTreeMap<String, usize> = BTreeMap::new();
        for id in edge_ids(&solid) {
            let kind = sides(&solid, id);
            match EdgeBlender::fillet_edge(&solid, id, case.radius) {
                Ok(result) => {
                    let after = volume(&result);
                    if after < before - 1e-9 {
                        ok += 1;
                        *ok_kinds.entry(kind).or_insert(0) += 1;
                    } else {
                        wrong += 1;
                        println!("   WRONG 稜 {id}（{kind}）: 体積 {before:.6} -> {after:.6}");
                    }
                }
                Err(err) => {
                    *reasons.entry((kind, reason_key(&err))).or_insert(0) += 1;
                }
            }
        }
        let refused: usize = reasons.values().sum();
        println!(
            "   体積 {before:.6}、稜 {} 本: 通った {ok}、断り {refused}、体積が減らない {wrong}",
            ok + refused + wrong
        );
        for (kind, count) in &ok_kinds {
            println!("     通った {count:>3}  [{kind}]");
        }
        for ((kind, reason), count) in &reasons {
            println!("     断り   {count:>3}  [{kind}] {reason}");
        }
        total_ok += ok;
        total_refused += refused;
        total_wrong += wrong;
    }
    println!("合計: 通った {total_ok}、断り {total_refused}、体積が減らない {total_wrong}");
}
