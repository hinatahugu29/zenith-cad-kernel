//! **読んだ立体の表示メッシュが閉じているかを、常設で測ります**（4-612）。
//!
//! # なぜ要るのか
//!
//! **4-611 で、`occ_reference_pipe_bend.step` の表示メッシュが
//! どの刻みでも開いている**ことが分かりました。**ずっとそうだった**のに、
//! **誰も測っていませんでした**——**`mesh_watertight_probe` の検体は
//! 全部こちらが組んだ形**（箱・円柱・球・円錐・円環・穴あき板・
//! ブーリアンの答え）で、**読んだファイルは 1 つも入っていません**。
//!
//! **「体積が合っている」は「メッシュが閉じている」ではありません**——
//! **`pipe_bend` の体積は 4-410 で合っており、`shape_variety_probe` は
//! 1579.136704 対 1579.136704（1.11e-10）で ok** です。
//!
//! # もう 1 つ測ること——**溶接が三角形を落としたか**
//!
//! **`winding_contract_probe` の折り返し／食い違いの数え方は、
//! 溶接前の面ごとの枚数を、溶接後の三角形の並びに当てています。**
//! **溶接は三角形を落とす**ことがあるので（4-333）、
//! **落ちていれば、その面から先の割り当てが全部ずれます。**
//!
//! **穴と重なりは、面の割り当てを使わない**ので影響を受けません。
//! **影響を受けるのは折り返し／食い違いの内訳だけ**です。
//! **ここで「落ちた枚数」を測って、当てが効くかどうかを見ます。**
//!
//! ```bash
//! cargo run --release -p zenith_algo --example import_closure_probe
//! ```

use zenith_io::StepImporter;
use zenith_tess::{face_triangle_counts, tessellate_solid, TessellationParams};

fn main() {
    println!("読んだ立体の表示メッシュが閉じているか（4-612）");
    println!();

    let mut subjects: Vec<(String, String)> = vec![
        (
            "screw".to_string(),
            "reference/OCCT/data/step/screw.step".to_string(),
        ),
        (
            "linkrods".to_string(),
            "reference/OCCT/data/step/linkrods.step".to_string(),
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
            subjects.push((name.replace("occ_reference_", "occ:"), path));
        }
    }

    let divisions: usize = std::env::var("ZENITH_CLOSURE_DENSITY")
        .ok()
        .and_then(|value| value.parse().ok())
        .unwrap_or(24);
    let params = TessellationParams {
        u_divisions: divisions,
        v_divisions: divisions,
    };
    println!("  刻み {divisions}。**落ちた**は溶接で消えた三角形の枚数です。");
    println!();
    let mut open = 0usize;
    let mut dropped_any = 0usize;
    let mut known_still_red = 0usize;
    // **1 つだけ見たいとき**（`ZENITH_FACE_OWNER_WHY` と併せて使います）。
    let only = std::env::var("ZENITH_CLOSURE_ONLY").ok();
    for (name, path) in &subjects {
        if let Some(only) = &only {
            if name != only {
                continue;
            }
        }
        let Ok(solids) = StepImporter::import_solids_from_file(path) else {
            println!("  {name:<12} **読めません**");
            continue;
        };
        let Some(solid) = solids
            .into_iter()
            .max_by_key(|solid| solid.outer_shell.faces.len())
        else {
            println!("  {name:<12} **立体が 0 個**");
            continue;
        };
        let faces = solid.outer_shell.faces.len() + solid
            .inner_shells
            .iter()
            .map(|shell| shell.faces.len())
            .sum::<usize>();
        let mesh = tessellate_solid(&solid, &params);
        let before: usize = face_triangle_counts(&solid, &params)
            .iter()
            .map(|(_, count)| *count)
            .sum();
        let dropped = before.saturating_sub(mesh.indices.len());
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
            for step in 0..3 {
                let (a, b) = (
                    cell(mesh.positions[triangle[step] as usize]),
                    cell(mesh.positions[triangle[(step + 1) % 3] as usize]),
                );
                let (lo, hi) = if a <= b { (a, b) } else { (b, a) };
                *undirected.entry((lo, hi)).or_insert(0) += 1;
            }
        }
        let holes = undirected.values().filter(|count| **count == 1).count();
        let overlaps = undirected.values().filter(|count| **count > 2).count();
        // **既知の赤**（4-611、4-612）。**`occ:pipe_bend` だけは、どの刻みでも
        // 穴 = 10 × 刻み** です（**壁が p-curve を作れず、計画を見ない経路へ
        // 落ちて、蓋と刻みが 4 倍違う**。4-612）。**見せる・数えない・
        // 直ったら教える**にします。
        let known_red =
            name == "occ:pipe_bend" && holes == divisions * 10 && overlaps == 0;
        if holes + overlaps > 0 && !known_red {
            open += 1;
        }
        if known_red {
            known_still_red += 1;
        }
        if dropped > 0 {
            dropped_any += 1;
        }
        // **域の外のパラメータが、巻き戻るのか外挿するのか**（4-614。
        // `ZENITH_CLOSURE_WRAP=1`）。
        //
        // **閉じた輪の p-curve を「域の外へ伸ばす」直し方が成り立つか**を
        // 決めるのに要ります（4-614 の道 2）。**成り立つなら
        // `evaluate(v_max + d)` は `evaluate(v_min + d)` と同じ点**を返します。
        if std::env::var_os("ZENITH_CLOSURE_WRAP").is_some() {
            for face in std::iter::once(&solid.outer_shell)
                .chain(solid.inner_shells.iter())
                .flat_map(|shell| shell.faces.iter())
            {
                let zenith_topo::FaceGeometry::Nurbs(surface) = &face.geometry else {
                    continue;
                };
                let ((u_min, u_max), (v_min, v_max)) = surface.param_range();
                let (u_mid, v_span) = ((u_min + u_max) * 0.5, v_max - v_min);
                // **まず、本当に v で閉じているか**を測ります。
                let seam = (surface.evaluate(u_mid, v_min) - surface.evaluate(u_mid, v_max)).norm();
                let delta = v_span * 0.0625;
                let outside = surface.evaluate(u_mid, v_max + delta);
                let wrapped = surface.evaluate(u_mid, v_min + delta);
                println!(
                    "    面 {} v で閉じている隔たり {seam:.3e}、域外 v={:.6} と 巻き戻し v={:.6} の隔たり {:.6}",
                    face.id,
                    v_max + delta,
                    v_min + delta,
                    (outside - wrapped).norm()
                );
            }
        }
        // **稜ごとの分割数を、面ごとに並べます**（4-612。
        // **共有する稜で数が違えば、溶接できず両側に穴が開きます**）。
        if std::env::var_os("ZENITH_CLOSURE_SEGMENTS").is_some() {
            for face in std::iter::once(&solid.outer_shell)
                .chain(solid.inner_shells.iter())
                .flat_map(|shell| shell.faces.iter())
            {
                let counts = zenith_tess::face_edge_segment_counts(&solid, face.id, &params);
                println!("    面 {} の稜ごとの分割: {counts:?}", face.id);
            }
        }
        println!(
            "  {name:<12} 面 {faces:>3}  三角形 {:>7}  落ちた {dropped:>4}  穴 {holes:>4}  重なり {overlaps:>3}  {}",
            mesh.indices.len(),
            if holes + overlaps == 0 {
                "閉じている"
            } else if known_red {
                "**既知の赤**（4-612。rc には数えません）"
            } else {
                "**開いている**"
            }
        );
    }
    println!();
    println!("**開いている検体 {open} 個**（既知の赤を除く）、**溶接で三角形が落ちた検体 {dropped_any} 個**。");
    // **1 つだけ見ているときは、判定しません**（検体が揃っていないので）。
    if only.is_some() {
        return;
    }
    // **既知の赤が直ったら、赤にして教えます**（4-612）。**見逃しを残すと、
    // 次の退行を隠します。**
    if known_still_red == 0 {
        println!("**`occ:pipe_bend` の既知の赤が、出なくなりました。**");
        println!("**直ったのなら、この掃き出しの見逃し（4-612）を外してください。**");
        std::process::exit(1);
    }
    if open > 0 {
        println!("**読んだ立体の表示メッシュが開いています。**");
        println!("**`ZENITH_CLOSURE_ONLY=<名前> ZENITH_FACE_OWNER_WHY=1` で、");
        println!("どの面が出した穴かを見てください**（4-612）。");
        std::process::exit(1);
    }
}
