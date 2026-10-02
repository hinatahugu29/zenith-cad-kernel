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
use zenith_tess::{tessellate_solid_stitched_with_faces, TessellationParams};

/// **折り返しと「別の 2 枚が同じ向き」を分けて数えます**（4-569、4-574）。
///
/// **面の割り当ては、溶接後の三角形ごとに受け取ります**（4-618）。
/// **以前は溶接前の面ごとの枚数を、溶接後の並びに当てていました**——
/// **溶接は三角形を落とす**ので（4-333）、**落ちた面から先が全部ずれます**。
/// **実測で 22 検体のうち 4 つが落としています**（4-612）。
fn count(
    positions: &[zenith_math::Point3],
    indices: &[[u32; 3]],
    face_of: &[Option<u64>],
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
    let mut owner: std::collections::BTreeMap<
        ((i64, i64, i64), (i64, i64, i64)),
        Vec<Option<u64>>,
    > = std::collections::BTreeMap::new();
    for (index, triangle) in indices.iter().enumerate() {
        let who = face_of.get(index).copied().flatten();
        for pair in [
            (triangle[0] as usize, triangle[1] as usize),
            (triangle[1] as usize, triangle[2] as usize),
            (triangle[2] as usize, triangle[0] as usize),
        ] {
            let key = (cell(positions[pair.0]), cell(positions[pair.1]));
            if directed.get(&key).copied().unwrap_or(0) > 1 {
                owner.entry(key).or_default().push(who);
            }
        }
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
    // **口が無くても、測って表は出します**（4-611）。**rc だけは必ず赤**に
    // します。
    //
    // **最初は「測らずに赤」にしていました**——**立て忘れて緑がいちばん悪い**
    // ので。**ところが、それでは診断に使えません**: **既定の姿を知りたいとき、
    // この掃き出しが何も教えてくれない**のです（**実際に `pipe_bend` の
    // 穴 240 本を既定と比べようとして詰まりました**）。
    //
    // **表は出す。rc は口で決める。** これなら両方できます。
    if !missing.is_empty() {
        println!("**口が立っていません**: {missing:?}");
        println!("**表は出しますが、rc は赤にします**——**立て忘れて緑が、");
        println!("いちばん悪い**からです。**既定の姿を見るにはこのまま読んでください。**");
        println!();
    }

    let mut bad = 0usize;
    // **既知の赤が、まだ赤いか**（4-611）。**直ったら教えます**。
    let mut known_still_red = 0usize;
    let mut rows = 0usize;
    // **OCCT が配っている実物 2 つ**と、**常設の検体（こちらが書いて OCCT が
    // 読み直した 20 個）**を並べます（4-611）。**どちらも刻み 7 通り**です。
    //
    // **最初は検体を刻み 24 の 1 通りだけ**にしていました——**20 個 × 7 通りは
    // 時間がかかりすぎる**と見込んだからです。**測ったら 49 秒でした**
    // （門の中で、通しで。4-611）。**それなら全部回せます**ので広げました。
    // **刻みを変えると出方が変わる**のは、**4-602 の折り返しが刻み 12 と 24
    // でだけ出た**ことで分かっています——**1 通りだけでは取り逃がします。**
    let mut subjects: Vec<(String, String, Vec<usize>)> = vec![
        (
            "screw".to_string(),
            "reference/OCCT/data/step/screw.step".to_string(),
            vec![8, 12, 16, 20, 24, 32, 48],
        ),
        (
            "linkrods".to_string(),
            "reference/OCCT/data/step/linkrods.step".to_string(),
            vec![8, 12, 16, 20, 24, 32, 48],
        ),
    ];
    if let Ok(entries) = std::fs::read_dir("crates/zenith_algo/tests/fixtures") {
        let mut fixtures: Vec<String> = entries
            .filter_map(|entry| entry.ok())
            .map(|entry| entry.path().to_string_lossy().to_string())
            .filter(|path| path.ends_with(".step"))
            .collect();
        fixtures.sort();
        for path in fixtures {
            let name = std::path::Path::new(&path)
                .file_stem()
                .map(|stem| stem.to_string_lossy().to_string())
                .unwrap_or_else(|| path.clone());
            subjects.push((name.replace("occ_reference_", "occ:"), path, vec![8, 12, 16, 20, 24, 32, 48]));
        }
    }
    for (name, path, densities) in &subjects {
        let (name, path) = (name.as_str(), path.as_str());
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
        for divisions in densities.iter().copied() {
            let params = TessellationParams {
                u_divisions: divisions,
                v_divisions: divisions,
            };
            // **溶接後の面の割り当てを、そのまま受け取ります**（4-618）。
            let (mesh, face_of) = tessellate_solid_stitched_with_faces(&solid, &params);
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
            let (folded, crossed) = count(&mesh.positions, &mesh.indices, &face_of);
            let worst = holes + overlaps + folded + crossed;
            rows += 1;
            // **既知の赤は、赤のまま見せますが rc には数えません**（4-611）。
            //
            // **`occ:pipe_bend` は、どの刻みでも穴が開いています**——
            // **穴の数は `10 × 刻み` ぴったり**（8→80、12→120、…48→480）。
            // **これは私が入れた口のせいではありません**（**既定・口 1 本ずつ・
            // 口 2 本の 4 通りすべてで同じ数**）。**ずっとそうだった**のに、
            // **誰も測っていませんでした**（`mesh_watertight_probe` にも
            // `mesh_density_probe` にも、この検体は入っていません）。
            //
            // **これを rc に数えると、門が最初から赤**になり、
            // **巻き方の退行を捕まえる役に立ちません。** **別の的**なので、
            // **見せる・数えない・直ったら教える**にします。
            let known_red = name == "occ:pipe_bend" && holes == divisions * 10 && overlaps == 0;
            if worst > 0 && !known_red {
                bad += 1;
            }
            if known_red {
                known_still_red += 1;
            }
            println!(
                "  {name:<10} 刻み {divisions:>2}  三角形 {:>7}  穴 {holes:>3}  重なり {overlaps:>3}  折り返し {folded:>3}  別の 2 枚が同じ向き {crossed:>3}  {}",
                mesh.indices.len(),
                if worst == 0 {
                    "緑"
                } else if known_red {
                    "**既知の赤**（4-611。rc には数えません）"
                } else {
                    "**赤**"
                }
            );
        }
    }

    println!();
    if !missing.is_empty() {
        println!("**口が立っていなかったので、rc は赤です**（{missing:?}）。");
        println!("**上の表は、その状態の実測**です。");
        std::process::exit(1);
    }
    // **既知の赤が直ったら、赤にして教えます**（4-611）。**直ったのに
    // 見逃しを続けると、次の退行を隠します。**
    if known_still_red == 0 {
        println!("**`occ:pipe_bend` の既知の赤が、出なくなりました。**");
        println!("**直ったのなら、この掃き出しの見逃し（4-611）を外してください**");
        println!("——**見逃しを残したままにすると、次の退行を隠します。**");
        std::process::exit(1);
    }
    if bad == 0 {
        println!(
            "**巻き方は {} 通りすべてで 0 本です**（既知の赤 {known_still_red} 通りを除く）。",
            rows - known_still_red
        );
        println!("**穴・重なり・折り返し・別の 2 枚が同じ向き、どれも立っていません。**");
        println!();
        println!("**`occ:pipe_bend` だけは、どの刻みでも穴が開いています**（4-611。");
        println!("**穴は `10 × 刻み` ぴったり。巻き方の口とは関係なく、ずっとそう**）。");
    } else {
        println!("**{bad} 通りで立っています**（{rows} 通り中）。");
        println!("**4-582・4-602 の結果が戻っています**——`mesh_winding_probe` の");
        println!("`ZENITH_FOLDED_FACES=1` で、どの面が持っているかを見てください。");
        std::process::exit(1);
    }
}
