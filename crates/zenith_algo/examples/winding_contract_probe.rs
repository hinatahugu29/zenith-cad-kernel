//! **読んだ立体の表示メッシュが、巻き方まで揃っていることを門にします**（4-610）。
//!
//! # なぜ要るのか
//!
//! 4-582 と 4-602 で、**`linkrods`・`screw` の 2 ファイル × 刻み 7 通り
//! （8〜48）すべてで、折り返し 0・食い違い 0・穴 0・重なり 0** になりました。
//! **ただし、確かめ方は「`mesh_winding_probe` を手で回して目で見る」**
//! **だけ**でした。**門が無いので、黙って戻っても誰も気づきません。**
//!
//! **この掃き出しは、数が 1 本でも立っていたら赤になります**（rc=1）。
//!
//! # 使い方
//!
//! **口 2 本が要ります**（**既定では巻き方は揃っていないので、
//! 口なしで回すと当然赤**です。**それは退行ではなく、既定の姿**）。
//!
//! ```bash
//! ZENITH_TRIM_WINDING_AFTER_WELD=1 ZENITH_NO_SPLIT_FLAT=1 \
//!   cargo run --release -p zenith_algo --example winding_contract_probe
//! ```
//!
//! **口が立っていなければ、測らずに赤にします**——**「立て忘れて緑」**
//! **が、いちばん悪い**からです（**門から口が落ちても気づけるように**）。
//!
//! # 何を数えるか（4-569 で 2 つに割りました）
//!
//! * **折り返し**: **1 枚の面が、同じ辺を同じ向きに 2 回**使う
//!   （**その面の三角形分割が自分と重なっている**）
//! * **別の 2 枚が同じ向き**: **2 枚の面が、共有する辺を同じ向きに**使う
//!   （**閉じた多様体では起こり得ません**）
//! * **穴・重なり**: **向きを見ない数え方**（同じ辺が 1 回だけ／3 回以上）

use zenith_io::StepImporter;
use zenith_tess::{face_triangle_counts, tessellate_solid, TessellationParams};

/// **折り返しと「別の 2 枚が同じ向き」を分けて数えます**（4-569、4-574）。
fn count(
    positions: &[zenith_math::Point3],
    indices: &[[u32; 3]],
    counts: &[(u64, usize)],
) -> (usize, usize) {
    let cell = |point: zenith_math::Point3| {
        (
            (point.x / 1e-6).round() as i64,
            (point.y / 1e-6).round() as i64,
            (point.z / 1e-6).round() as i64,
        )
    };
    let mut directed: std::collections::BTreeMap<((i64, i64, i64), (i64, i64, i64)), usize> =
        std::collections::BTreeMap::new();
    for triangle in indices {
        for pair in [
            (triangle[0] as usize, triangle[1] as usize),
            (triangle[1] as usize, triangle[2] as usize),
            (triangle[2] as usize, triangle[0] as usize),
        ] {
            *directed
                .entry((cell(positions[pair.0]), cell(positions[pair.1])))
                .or_insert(0) += 1;
        }
    }
    let mut owner: std::collections::BTreeMap<((i64, i64, i64), (i64, i64, i64)), Vec<u64>> =
        std::collections::BTreeMap::new();
    let mut at = 0usize;
    for (face_id, triangle_count) in counts {
        for index in at..(at + triangle_count).min(indices.len()) {
            let triangle = indices[index];
            for pair in [
                (triangle[0] as usize, triangle[1] as usize),
                (triangle[1] as usize, triangle[2] as usize),
                (triangle[2] as usize, triangle[0] as usize),
            ] {
                let key = (cell(positions[pair.0]), cell(positions[pair.1]));
                if directed.get(&key).copied().unwrap_or(0) > 1 {
                    owner.entry(key).or_default().push(*face_id);
                }
            }
        }
        at += triangle_count;
    }
    let (mut folded, mut crossed) = (0usize, 0usize);
    for users in owner.values() {
        let mut unique = users.clone();
        unique.sort_unstable();
        unique.dedup();
        if unique.len() <= 1 {
            folded += 1;
        } else {
            crossed += 1;
        }
    }
    (folded, crossed)
}

fn main() {
    println!("読んだ立体の表示メッシュが、巻き方まで揃っているか（4-610）");
    println!();

    // **口が立っていなければ、測らずに赤**にします。
    let mut missing: Vec<&str> = Vec::new();
    for port in ["ZENITH_TRIM_WINDING_AFTER_WELD", "ZENITH_NO_SPLIT_FLAT"] {
        if std::env::var_os(port).is_none() {
            missing.push(port);
        }
    }
    if !missing.is_empty() {
        println!("**口が立っていません**: {missing:?}");
        println!();
        println!("**既定では巻き方は揃っていません**（4-582、4-602）。");
        println!("**この門は、口 2 本を立てて回すためのもの**です——");
        println!("  ZENITH_TRIM_WINDING_AFTER_WELD=1 ZENITH_NO_SPLIT_FLAT=1 \\");
        println!("    cargo run --release -p zenith_algo --example winding_contract_probe");
        println!();
        println!("**測らずに赤にします**——**立て忘れて緑が、いちばん悪い**からです。");
        std::process::exit(1);
    }

    let mut bad = 0usize;
    let mut rows = 0usize;
    for (name, path) in [
        ("screw", "reference/OCCT/data/step/screw.step"),
        ("linkrods", "reference/OCCT/data/step/linkrods.step"),
    ] {
        // **読み込みは 1 回だけ**です（**面の番号は読み込みごとに進むので、
        // 刻みごとに読み直すと同じ面が別の番号になります**。4-598）。
        let Ok(solids) = StepImporter::import_solids_from_file(path) else {
            println!("  {name}: **読めません**（{path}）");
            bad += 1;
            continue;
        };
        let Some(solid) = solids
            .into_iter()
            .max_by_key(|solid| solid.outer_shell.faces.len())
        else {
            println!("  {name}: **立体が 0 個**");
            bad += 1;
            continue;
        };
        for divisions in [8usize, 12, 16, 20, 24, 32, 48] {
            let params = TessellationParams {
                u_divisions: divisions,
                v_divisions: divisions,
            };
            let mesh = tessellate_solid(&solid, &params);
            let counts = face_triangle_counts(&solid, &params);
            let cell = |point: zenith_math::Point3| {
                (
                    (point.x / 1e-6).round() as i64,
                    (point.y / 1e-6).round() as i64,
                    (point.z / 1e-6).round() as i64,
                )
            };
            let mut undirected: std::collections::BTreeMap<
                ((i64, i64, i64), (i64, i64, i64)),
                usize,
            > = std::collections::BTreeMap::new();
            for triangle in &mesh.indices {
                for pair in [
                    (triangle[0] as usize, triangle[1] as usize),
                    (triangle[1] as usize, triangle[2] as usize),
                    (triangle[2] as usize, triangle[0] as usize),
                ] {
                    let (a, b) = (cell(mesh.positions[pair.0]), cell(mesh.positions[pair.1]));
                    let (lo, hi) = if a <= b { (a, b) } else { (b, a) };
                    *undirected.entry((lo, hi)).or_insert(0) += 1;
                }
            }
            let holes = undirected.values().filter(|count| **count == 1).count();
            let overlaps = undirected.values().filter(|count| **count > 2).count();
            let (folded, crossed) = count(&mesh.positions, &mesh.indices, &counts);
            let worst = holes + overlaps + folded + crossed;
            rows += 1;
            if worst > 0 {
                bad += 1;
            }
            println!(
                "  {name:<10} 刻み {divisions:>2}  三角形 {:>7}  穴 {holes:>3}  重なり {overlaps:>3}  折り返し {folded:>3}  別の 2 枚が同じ向き {crossed:>3}  {}",
                mesh.indices.len(),
                if worst == 0 { "緑" } else { "**赤**" }
            );
        }
    }

    println!();
    if bad == 0 {
        println!("**{rows} 通りすべてで 0 本です。**");
        println!("**穴・重なり・折り返し・別の 2 枚が同じ向き、どれも立っていません。**");
    } else {
        println!("**{bad} 通りで立っています**（{rows} 通り中）。");
        println!("**4-582・4-602 の結果が戻っています**——`mesh_winding_probe` の");
        println!("`ZENITH_FOLDED_FACES=1` で、どの面が持っているかを見てください。");
        std::process::exit(1);
    }
}
