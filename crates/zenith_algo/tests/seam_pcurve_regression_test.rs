//! **継ぎ目まわりの回帰を、`cargo test` で捕まえる**（4-411）。
//!
//! # なぜ要るのか
//!
//! 2026/09/08 に、**記録されている門のうち 5 つが `origin/main` の時点で
//! 赤**だと分かりました（4-410）。**①は 10 日間、②は 5 日間、誰も
//! 気づいていません。**
//!
//! **そのあいだ、通しテストはずっと緑**でした。
//!
//! > **門はテストではありません。** `cargo test` では 1 本も走りません。
//!
//! **ここが、その教訓を形にしたもの**です。**門が拾っていた信号のうち、
//! 外部ファイルを要さないものを、テストに移しました。**
//!
//! # ⚠ **門と同じ経路で測らないと、何も捕まえません**
//!
//! **最初に書いたものは、3 つの切替で壊しても 4 本とも通りました。**
//! 違っていたのは 3 つです——
//!
//! | | 最初に書いたもの | **門** |
//! | :--- | :--- | :--- |
//! | 読み方 | `Regularizer::hold_like_our_own` を通す | **生のまま** |
//! | 刻み | `TessellationParams::default()` | **64x64** |
//! | 立体 | いちばん面の多いもの 1 つ | **全部の和** |
//!
//! **`Regularizer` は p-curve を組み直すので、欠陥を覆い隠します。**
//! **「テストを書いた」と「回帰を捕まえる」は別**です——
//! **壊してみて、落ちることを確かめてください。**
//!
//! **確かめた結果**——
//!
//! | 壊し方 | 落ちるもの |
//! | :--- | :--- |
//! | `ZENITH_SEAM_MIXED=1`（②を戻す） | `no_turned_solid_collapses_flat` |
//! | `ZENITH_SUBDIV_GUARD=1`（①を戻す） | `the_bent_pipe_keeps_its_volume` |
//! | `ZENITH_NO_SEAM_ENDPICK=1` | **1 本も落ちません** |
//!
//! **3 つ目は、ここでは捕まりません。**
//! 周期の端の選び方が効くのを見せた検体は **`linkrods.step` だけ**で、
//! それは `reference/`（`.gitignore` 済み）にあり、**2026/09/08 に
//! 消えました**。実測では、追跡済みの検体では**どちらでも同じ答え**に
//! なります。**戻ってきたら、`read_and_cut_probe` で見てください**
//! ——穴 0 本 と 480 本で分かれます。
//!
//! # 何を見るか
//!
//! **追跡済みの検体だけ**を読みます（`tests/fixtures/`）。
//! **`reference/` は要りません。**

use std::path::PathBuf;
use zenith_algo::MassCalculator;
use zenith_io::StepImporter;
use zenith_math::BoundingBox3;
use zenith_tess::TessellationParams;
use zenith_topo::Solid;

fn fixture(name: &str) -> PathBuf {
    PathBuf::from(concat!(env!("CARGO_MANIFEST_DIR"), "/tests/fixtures"))
        .join(format!("occ_reference_{name}.step"))
}

/// **門と同じ読み方**——**生のまま**、**立体は全部**。
fn read(name: &str) -> Vec<Solid> {
    StepImporter::import_solids_from_file(&fixture(name))
        .unwrap_or_else(|error| panic!("{name} を読めません: {error}"))
}

/// **門と同じ刻み**（`shape_variety_probe` の `params()`）。
fn params() -> TessellationParams {
    TessellationParams {
        u_divisions: 64,
        v_divisions: 64,
    }
}

fn volume(solids: &[Solid]) -> f64 {
    solids
        .iter()
        .map(|solid| MassCalculator::compute_from_brep(solid, &params()).volume)
        .sum()
}

fn relative(measured: f64, expected: f64) -> f64 {
    (measured - expected).abs() / expected.abs().max(1.0)
}

/// **曲がり管の体積が、閉じた式と合うこと。**
///
/// 4-410 の①（`3570fb5`）が入っていたとき **335.103216**、あるべきは
/// **1579.136704**——**79% のずれ**です。**通しテストは緑のまま**でした。
///
/// 基準は `shape_variety_probe` が持っているのと同じ数です。
#[test]
fn the_bent_pipe_keeps_its_volume() {
    let measured = volume(&read("pipe_bend"));
    let expected = 1579.136704;
    let residual = relative(measured, expected);
    assert!(
        residual <= 1e-3,
        "曲がり管の体積が {measured}（{expected} のはず。相対差 {residual:.3e}）。\
         細分の段で継ぎ目をまたいでいないか見てください（4-411 の①）"
    );
}

/// **回転面の体積が、閉じた式と合うこと。**
///
/// **継ぎ目のある面ばかり**です——球・円柱・円錐・トーラスは、どれも
/// u が一周して戻ります。**継ぎ目の持ち方が崩れると、ここが動きます。**
#[test]
fn the_turned_solids_keep_their_volume() {
    let pi = std::f64::consts::PI;
    let cases: [(&str, f64); 4] = [
        ("sphere", 4.0 / 3.0 * pi * 10.0f64.powi(3)),
        ("cylinder", pi * 10.0f64.powi(2) * 40.0),
        ("cone_full", pi * 10.0f64.powi(2) * 20.0 / 3.0),
        ("torus", 2.0 * pi * pi * 12.0 * 4.0f64.powi(2)),
    ];
    for (name, expected) in cases {
        let measured = volume(&read(name));
        let residual = relative(measured, expected);
        assert!(
            residual <= 1e-3,
            "{name} の体積が {measured}（閉じた式は {expected}。\
             相対差 {residual:.3e}）。継ぎ目の持ち方を見てください（4-411）"
        );
    }
}

/// **メッシュの囲み箱が、平らに潰れないこと。**
///
/// 4-410 の②（`efa9c92` の `mixed`）が入っていたとき、**円錐の底円の
/// p-curve が潰され**、囲み箱が `[-10,-10,0]..[10,10,0]`——**高さ 0** と
/// 読まれました。`foreign_distance_probe` はこの円錐を**丸ごと飛ばして**
/// 36 → 29 チェックに減っていました。
///
/// **メッシュから測ります**——**p-curve が潰れると三角形が消える**ので、
/// そこがいちばん素直に出ます。**3D の稜を標本しても出ません**
/// （稜は p-curve を通らないからです。最初にそれで書いて、何も
/// 捕まえられませんでした）。
#[test]
fn no_turned_solid_collapses_flat() {
    let expected: [(&str, [f64; 3]); 5] = [
        ("sphere", [20.0, 20.0, 20.0]),
        ("cylinder", [20.0, 20.0, 40.0]),
        ("cone_full", [20.0, 20.0, 20.0]),
        ("torus", [32.0, 32.0, 8.0]),
        ("pipe_bend", [0.0, 0.0, 0.0]),
    ];
    for (name, sizes) in expected {
        let solids = read(name);
        let mut bbox: Option<BoundingBox3> = None;
        for solid in &solids {
            for point in zenith_tess::tessellate_solid(solid, &params()).positions {
                match &mut bbox {
                    Some(box3) => box3.extend_point(point),
                    None => bbox = Some(BoundingBox3::from_point(point)),
                }
            }
        }
        let bbox = bbox.unwrap_or_else(|| panic!("{name} のメッシュに点がありません"));
        let extents = [
            bbox.max.x - bbox.min.x,
            bbox.max.y - bbox.min.y,
            bbox.max.z - bbox.min.z,
        ];
        for (axis, extent) in ["x", "y", "z"].iter().zip(extents) {
            assert!(
                extent > 1e-6,
                "{name} のメッシュが {axis} 方向に潰れています（広がり {extent}）。\
                 継ぎ目に乗った区間を潰していないか見てください（4-411 の②）"
            );
        }
        // **寸法が分かっているものは、そこまで見ます**（`pipe_bend` は
        // 曲がっているので、閉じた式が手元にありません——潰れだけ見ます）。
        if sizes[0] > 0.0 {
            for (index, (axis, extent)) in ["x", "y", "z"].iter().zip(extents).enumerate() {
                let residual = relative(extent, sizes[index]);
                assert!(
                    residual <= 1e-3,
                    "{name} のメッシュの {axis} の広がりが {extent}\
                     （{} のはず。相対差 {residual:.3e}）",
                    sizes[index]
                );
            }
        }
    }
}
