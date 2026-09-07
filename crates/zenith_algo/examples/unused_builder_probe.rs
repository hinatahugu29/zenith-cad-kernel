//! **呼び手が 1 人もいない公開関数**を、閉じた式で測る（4-395）。
//!
//! # なぜ要るのか
//!
//! 4-391 で「**受け皿だけあって呼び口が無い**」という形を見つけました
//! （`revolve_wire_solid` はあるのに、スケッチから呼べない）。
//! **同じ形が他にもないか**を、機械的に数えました。
//!
//! ```bash
//! # 公開関数のうち、tests/ にも examples/ にも出てこないもの
//! ```
//!
//! **8 つ出ました。** どれも **`src` の中でも呼ばれていません**——
//! **定義された 1 か所きり**です。
//!
//! | | 何ができるはずのものか |
//! | :--- | :--- |
//! | `mirror_solid` ほか | **鏡像**。標準的な CAD の操作です |
//! | `linear_pattern_shape` | **配列複製** |
//! | `make_slot` / `make_regular_polygon` | **輪郭**（長円・正多角形） |
//! | `planarize_shell` / `sew_shell` / `merge_coplanar` | 形の手入れ |
//!
//! **試験が 0 本です。** このリポジトリで見つかった欠陥は、ほぼ全部
//! 「**測っていなかった領域にプローブを当てたら出てきた**」ものでした
//! （HANDOVER 3-0-0）。
//!
//! # 何を測るか
//!
//! **閉じた式で出せるものだけ**です。
//!
//! ```text
//! 鏡像         体積は変わらない。2回かけると元の場所へ戻る
//! 配列複製     離れた N 個なら、体積は N 倍
//! 正多角形     面積 = (1/2) n R² sin(2π/n)
//! 長円         面積 = 2rL + πr²
//! 形の手入れ   掛けても体積は変わらない
//! ```
//!
//! # 手入れの道具は、2 通りに測ります
//!
//! **きれいな立体に掛けても壊さないこと**と、**汚れた立体で本当に働くこと**
//! は、別の話です。
//!
//! **きれいな円柱に掛けると、3 つとも何もしません**——平面化 0 枚、
//! 縫合 12 → 12 稜、併合 6 → 6 面。**正しい振る舞いですが、
//! それだけでは「直す力」を 1 度も動かしていません。**
//!
//! **併合だけは動かせました。** **同じ大きさの箱を 2 つ並べて和を取ると、
//! 結果がそのまま 1 つの箱になり**（面 6 枚）、**併合するものが残りません**。
//! **ずらして重ねる**と、**接する面が部分的にしか重ならない**ので、
//! 割れた同一平面の面が残ります——**面 12 → 10 枚**。
//!
//! **`planarize_shell` と `sew_shell` は、まだ「壊さないこと」しか
//! 測れていません。** **汚れた検体を用意するのが次の一手**です。
//!
//! **誤答が 1 件でも出たら exit 1。**

use std::f64::consts::PI;

use zenith_algo::{
    ExtrudeBuilder, MassCalculator, MirrorBuilder, PatternBuilder, PrimitiveBuilder, ProfileBuilder,
};
use zenith_math::{Point3, Tolerance, Vec3};
use zenith_tess::TessellationParams;
use zenith_topo::Solid;

/// 手入れした殻を立体に戻す。**閉じていなければ断ります**——
/// **手入れが殻を開けたら、そこで分かるようにします。**
fn solid_from(shell: zenith_topo::Shell) -> Result<Solid, String> {
    if !shell.is_closed {
        return Err("手入れの結果が閉じていません".to_string());
    }
    Ok(Solid::new(shell, Vec::new()))
}

fn volume(solid: &Solid) -> f64 {
    MassCalculator::compute_from_brep(solid, &TessellationParams::default()).volume
}

struct Row {
    name: &'static str,
    expected: f64,
    measured: Result<f64, String>,
}

fn main() {
    let tol = Tolerance::default();
    let mut rows: Vec<Row> = Vec::new();

    // ---- 鏡像 ----
    //
    // **体積は変わりません。** 裏返っても、大きさは同じです。
    let box_solid = PrimitiveBuilder::make_box(20.0, 30.0, 40.0).expect("box");
    let box_volume = volume(&box_solid);
    rows.push(Row {
        name: "鏡像: 体積は変わらない",
        expected: box_volume,
        measured: MirrorBuilder::mirror_solid(
            &box_solid,
            Point3::new(100.0, 0.0, 0.0),
            Vec3::new(1.0, 0.0, 0.0),
            &tol,
        )
        .map(|solid| volume(&solid)),
    });

    // **2 回かけると元の場所へ戻ります。** 体積だけでは裏返りを見逃すので、
    // **重心の x** も見ます。
    let once = MirrorBuilder::mirror_solid(
        &box_solid,
        Point3::new(100.0, 0.0, 0.0),
        Vec3::new(1.0, 0.0, 0.0),
        &tol,
    );
    let twice = once.as_ref().ok().and_then(|solid| {
        MirrorBuilder::mirror_solid(
            solid,
            Point3::new(100.0, 0.0, 0.0),
            Vec3::new(1.0, 0.0, 0.0),
            &tol,
        )
        .ok()
    });
    let centre_of = |solid: &Solid| {
        MassCalculator::compute_from_brep(solid, &TessellationParams::default())
            .center_of_mass
            .x
    };
    rows.push(Row {
        name: "鏡像: 2回で元の重心へ",
        expected: centre_of(&box_solid),
        measured: twice
            .as_ref()
            .map(|solid| centre_of(solid))
            .ok_or_else(|| "2回目が返りません".to_string()),
    });

    // **1 回かけた重心は、鏡の向こう側**です。
    // 箱は x ∈ [0, 20]、重心 x = 10。鏡は x = 100 なので、写ると x = 190。
    rows.push(Row {
        name: "鏡像: 1回で鏡の向こうへ",
        expected: 2.0 * 100.0 - centre_of(&box_solid),
        measured: once
            .as_ref()
            .map(|solid| centre_of(solid))
            .map_err(|reason| reason.clone()),
    });

    // ---- 配列複製 ----
    //
    // **離して並べた N 個**なら、体積は N 倍です。
    let count = 4usize;
    let pattern = PatternBuilder::linear_pattern(
        &box_solid,
        Vec3::new(1.0, 0.0, 0.0),
        // **重ならない間隔**。箱の x は 20 なので、50 なら触れもしません。
        50.0,
        count,
    );
    rows.push(Row {
        name: "配列複製: 体積は N 倍",
        expected: box_volume * count as f64,
        measured: pattern.map(|solids| solids.iter().map(volume).sum::<f64>()),
    });

    // ---- 正多角形 ----
    //
    // ```text
    // 面積 = (1/2) n R² sin(2π/n)
    // ```
    for sides in [3usize, 5, 6, 8, 12] {
        let radius = 10.0;
        let height = 7.0;
        let expected = 0.5 * sides as f64 * radius * radius * (2.0 * PI / sides as f64).sin();
        let measured = ProfileBuilder::make_regular_polygon(
            sides,
            radius,
            Point3::origin(),
            Vec3::new(0.0, 0.0, 1.0),
            Vec3::new(1.0, 0.0, 0.0),
        )
        .and_then(|wire| {
            ExtrudeBuilder::extrude_wire(&wire, Vec3::new(0.0, 0.0, height), &tol)
        })
        .map(|solid| volume(&solid));
        rows.push(Row {
            name: match sides {
                3 => "正三角形を押し出す",
                5 => "正五角形を押し出す",
                6 => "正六角形を押し出す",
                8 => "正八角形を押し出す",
                _ => "正十二角形を押し出す",
            },
            expected: expected * height,
            measured,
        });
    }

    // ---- 長円（スロット）----
    //
    // ```text
    // 面積 = 2rL + πr²      L は直線部の長さ、r は端の半径
    // ```
    //
    // **`make_slot` の `length` が「直線部」なのか「全長」なのかは、
    // 測って決めます**——名前では決めません。
    let (slot_length, slot_radius, slot_height) = (30.0, 6.0, 5.0);
    let slot = ProfileBuilder::make_slot(
        slot_length,
        slot_radius,
        Point3::origin(),
        Vec3::new(0.0, 0.0, 1.0),
        Vec3::new(1.0, 0.0, 0.0),
    )
    .and_then(|wire| ExtrudeBuilder::extrude_wire(&wire, Vec3::new(0.0, 0.0, slot_height), &tol))
    .map(|solid| volume(&solid));
    rows.push(Row {
        name: "長円を押し出す（直線部 = length）",
        expected: (2.0 * slot_radius * slot_length + PI * slot_radius * slot_radius) * slot_height,
        measured: slot,
    });

    // ---- 形の手入れ（`planarize_shell` / `sew_shell` / `merge_coplanar`）----
    //
    // **閉じた式は「変えないこと」**です。**既に正しい立体に掛けても、
    // 体積は 1 桁も動いてはいけません。**
    //
    // **手入れの道具は、直す力より「壊さない」ことのほうが大事**です
    // ——**壊れていないものに掛かる回数のほうが多い**からで、
    // **そこで体積が動いたら、直した先も信用できません。**
    let cylinder = PrimitiveBuilder::make_cylinder(10.0, 25.0).expect("cylinder");
    let cylinder_volume = volume(&cylinder);

    let (planarized, converted) =
        zenith_algo::FaceMerger::planarize_shell(&cylinder.outer_shell, &tol);
    rows.push(Row {
        name: "平面化: 体積は変わらない",
        expected: cylinder_volume,
        measured: solid_from(planarized.clone()).map(|solid| volume(&solid)),
    });

    let (sewn, sew_report) = zenith_algo::Sewer::sew_shell(&cylinder.outer_shell, &tol);
    rows.push(Row {
        name: "縫合: 体積は変わらない",
        expected: cylinder_volume,
        measured: solid_from(sewn.clone()).map(|solid| volume(&solid)),
    });

    let merged = zenith_algo::FaceMerger::merge_coplanar(&cylinder, &tol);
    rows.push(Row {
        name: "同一平面の併合: 体積は変わらない",
        expected: cylinder_volume,
        measured: merged
            .as_ref()
            .map(|(solid, _)| volume(solid))
            .map_err(|reason| reason.clone()),
    });

    // **箱でも同じことを見ます。** 円柱だけだと、平面の面が上下の蓋しか
    // ありません。
    let box_for_repair = PrimitiveBuilder::make_box(12.0, 18.0, 24.0).expect("box");
    let box_repair_volume = volume(&box_for_repair);
    rows.push(Row {
        name: "同一平面の併合（箱）: 体積は変わらない",
        expected: box_repair_volume,
        measured: zenith_algo::FaceMerger::merge_coplanar(&box_for_repair, &tol)
            .map(|(solid, _)| volume(&solid))
            .map_err(|reason| reason.clone()),
    });

    // **⚠ ここまでは「壊さないこと」しか測っていません。**
    //
    // 手入れの報告を見ると、**平面化 0 枚・縫合 12 → 12 稜・併合 6 → 6 面**
    // ——**きれいな立体には、するべき仕事がありません**。**直す力のほうは、
    // まだ 1 度も動いていません。**
    //
    // **併合だけは、動かせます。** **2 つの箱を並べて和を取ると、
    // 同一平面の面が割れたまま残ります**——そこへ掛けます。
    // **ぴったり同じ大きさで並べると、和がそのまま 1 つの箱になります**
    // （面 6 枚。併合するものが残りません）。**ずらして重ねます**——
    // **接する面が部分的にしか重ならない**ので、割れた同一平面の面が残ります。
    let left = PrimitiveBuilder::make_box(20.0, 20.0, 20.0).expect("box");
    let right = zenith_algo::BrepTransform::translate_solid(
        &PrimitiveBuilder::make_box(20.0, 10.0, 20.0).expect("box"),
        Vec3::new(20.0, 5.0, 0.0),
    );
    let welded = zenith_algo::BooleanEngine::boolean_solids_exact(
        &left,
        &right,
        zenith_algo::BooleanOpType::Union,
        &tol,
    );
    let (welded_faces, merged_faces, welded_volume) = match &welded {
        Ok(solid) => {
            let before = solid.outer_shell.faces.len();
            match zenith_algo::FaceMerger::merge_coplanar(solid, &tol) {
                Ok((merged, _)) => (
                    before,
                    merged.outer_shell.faces.len(),
                    Ok(volume(&merged)),
                ),
                Err(reason) => (before, 0, Err(reason)),
            }
        }
        Err(reason) => (0, 0, Err(reason.clone())),
    };
    rows.push(Row {
        name: "併合（和の結果）: 体積は変わらない",
        expected: 20.0 * 20.0 * 20.0 + 20.0 * 10.0 * 20.0,
        measured: welded_volume,
    });

    // ---- 出力 ----
    println!("呼び手が 1 人もいない公開関数を、閉じた式で測る（4-395）");
    println!();
    println!(
        "{:<34}{:>18}{:>18}{:>12}  {}",
        "測るもの", "閉じた式", "測った値", "相対差", "結果"
    );
    println!("{}", "-".repeat(104));

    let mut wrong = 0usize;
    let mut refused = 0usize;
    for row in &rows {
        match &row.measured {
            Ok(measured) => {
                let scale = row.expected.abs().max(1.0);
                let residual = (measured - row.expected).abs() / scale;
                if residual > 1e-6 {
                    wrong += 1;
                }
                println!(
                    "{:<34}{:>18.6}{:>18.6}{:>12.3e}  {}",
                    row.name,
                    row.expected,
                    measured,
                    residual,
                    if residual <= 1e-6 { "ok" } else { "**ちがう**" }
                );
            }
            Err(reason) => {
                refused += 1;
                println!(
                    "{:<34}{:>18.6}{:>18}{:>12}  **断られました**: {}",
                    row.name, row.expected, "-", "-", reason
                );
            }
        }
    }
    println!("{}", "-".repeat(104));
    println!();
    println!(
        "和の結果に併合を掛けると: 面 {welded_faces} → {merged_faces} 枚"
    );
    println!();
    println!(
        "手入れの報告（きれいな円柱に掛けたとき）: 平面化した面 {converted} 枚、縫合 {} → {} 稜、併合 {}",
        sew_report.edges_before,
        sew_report.edges_after,
        merged
            .as_ref()
            .map(|(_, report)| format!("{} → {} 面", report.faces_before, report.faces_after))
            .unwrap_or_else(|_| "-".to_string())
    );
    println!();
    println!("誤答 {wrong} 件、断り {refused} 件");
    if wrong > 0 || refused > 0 {
        std::process::exit(1);
    }
}
