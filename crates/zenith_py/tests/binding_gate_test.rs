//! **Blender へ届く層の門**（HANDOVER 9-H の H1）。
//!
//! # なぜこの試験が要るのか
//!
//! 2026/08/30 まで、このクレートは**常設の試験から外れていました**。
//! `pyo3` のビルドに Python 環境が要ると思われていたため、通しは
//! `--exclude zenith_py` で回っており、**130本 / 664件はこの層を1度も
//! 測っていませんでした**。
//!
//! ビルドのほうは `abi3` と `generate-import-lib` で解決しました
//! （`Cargo.toml` に理由を書いてあります）。**入れてみたら、試験が
//! 1本もありませんでした**——`running 0 tests`。門に入れても測るものが
//! 無ければ意味がないので、ここに置きます。
//!
//! # 何を測るのか
//!
//! **カーネルの再検査ではありません。** 幾何そのものは
//! `zenith_algo` 側の門が測っています。ここで測るのは**束ねている層**
//! ——Python から呼ばれる形のまま、答えが通るかどうかです。
//!
//! - プリミティブの体積が、**閉じた式**と合うか
//! - ブーリアンの**恒等式**が、この層を通しても閉じるか
//! - JSON の口（スケッチ・シェーダーペイロード）が、往復して壊れないか
//!
//! # GIL は要りません
//!
//! `#[pyfunction]` / `#[pymethods]` が付いていても、中身は素の Rust です。
//! `Python::with_gil` を通らない限り、インタプリタ無しで呼べます。
//! **失敗の中身は覗きません**——`PyErr` を表示すると GIL が要ります。

use std::f64::consts::PI;
use zenith_cad::PySolid;

/// 相対差で見ます。体積は桁が大きいので、絶対差では意味が変わります。
fn close(measured: f64, expected: f64, limit: f64) -> bool {
    let scale = expected.abs().max(1.0);
    (measured - expected).abs() / scale <= limit
}

#[test]
fn primitive_volumes_match_the_closed_forms() {
    // **束ねている層を通しても、閉じた式に合うか。**
    //
    // 分割は `volume()` の既定（32×32）です。面積分は解析的になったので
    // （4-156〜4-164）、曲面でも刻みに依りません。
    let cases: Vec<(&str, f64, f64)> = vec![
        (
            "box 10x20x30",
            PySolid::box_(10.0, 20.0, 30.0).expect("box").volume(),
            10.0 * 20.0 * 30.0,
        ),
        (
            "cylinder r=6 h=40",
            PySolid::cylinder(6.0, 40.0).expect("cylinder").volume(),
            PI * 36.0 * 40.0,
        ),
        (
            "sphere r=10",
            PySolid::sphere(10.0).expect("sphere").volume(),
            4.0 / 3.0 * PI * 1000.0,
        ),
        (
            "cone r=10 h=20",
            PySolid::cone(10.0, 0.0, 20.0).expect("cone").volume(),
            PI * 100.0 * 20.0 / 3.0,
        ),
        (
            "torus R=12 r=4",
            PySolid::torus(12.0, 4.0).expect("torus").volume(),
            2.0 * PI * PI * 12.0 * 16.0,
        ),
    ];

    let mut worst = 0.0f64;
    for (name, measured, expected) in &cases {
        let residual = (measured - expected).abs() / expected.abs().max(1.0);
        worst = worst.max(residual);
        assert!(
            close(*measured, *expected, 1e-9),
            "{name}: 束ねている層を通すと {measured:.9} で、閉じた式の {expected:.9} と \
             相対 {residual:.3e} 違います"
        );
    }
    assert!(worst <= 1e-9, "残差の最悪 {worst:.3e}");
}

#[test]
fn the_identity_holds_through_the_binding() {
    // **恒等式は、閉じた式が無くても誤答を映します**（4-142、4-191）。
    //
    // ```text
    // |A∪B| + |A∩B| = |A| + |B|
    // |A＼B| + |A∩B| = |A|
    // ```
    let a = PySolid::box_(20.0, 20.0, 20.0).expect("box");
    let b = PySolid::cylinder(6.0, 40.0)
        .expect("cylinder")
        .translated(10.0, 10.0, -10.0);

    let union: f64 = a.union(&b).expect("union").volume();
    let difference: f64 = a.difference(&b).expect("difference").volume();
    let intersection: f64 = a.intersection(&b).expect("intersection").volume();
    let (va, vb): (f64, f64) = (a.volume(), b.volume());

    let scale = (va + vb).abs().max(1.0);
    let first = ((union + intersection) - (va + vb)).abs() / scale;
    let second = ((difference + intersection) - va).abs() / scale;

    assert!(
        first <= 1e-9,
        "|A∪B| + |A∩B| = |A| + |B| が破れています（相対 {first:.3e}）: \
         ∪ {union:.9}, ∩ {intersection:.9}, |A| {va:.9}, |B| {vb:.9}"
    );
    assert!(
        second <= 1e-9,
        "|A＼B| + |A∩B| = |A| が破れています（相対 {second:.3e}）: \
         ＼ {difference:.9}, ∩ {intersection:.9}, |A| {va:.9}"
    );
}

#[test]
fn moving_a_solid_does_not_change_its_volume() {
    // **動かしても体積は変わりません。** 4-159 で、原点から離すと体積が
    // 6.67 倍になる誤りを見つけています——原点にある検体だけでは見えません。
    let solid = PySolid::torus(12.0, 4.0).expect("torus");
    let here: f64 = solid.volume();
    let there: f64 = solid.translated(137.0, -91.0, 53.0).volume();
    let residual = (there - here).abs() / here.abs().max(1.0);
    assert!(
        residual <= 1e-9,
        "動かしたら体積が {here:.9} から {there:.9} へ変わりました（相対 {residual:.3e}）"
    );

    let turned: f64 = solid
        .rotated([0.0, 0.0, 0.0], [1.0, 2.0, 3.0], 37.0)
        .expect("回せませんでした")
        .volume();
    let residual = (turned - here).abs() / here.abs().max(1.0);
    assert!(
        residual <= 1e-9,
        "回したら体積が {here:.9} から {turned:.9} へ変わりました（相対 {residual:.3e}）"
    );
}

#[test]
fn the_sketch_solver_returns_a_solution_that_satisfies_the_constraints() {
    // **JSON の口を往復させます。** Blender 側はこの形で呼びます。
    //
    // 点0は固定。点1を点0から水平に 10、点2を点1から鉛直に 5。
    let points = "[[0.0, 0.0], [3.0, 4.0], [9.0, 1.0]]";
    let constraints = r#"[
        {"type": "horizontal", "p1": 0, "p2": 1},
        {"type": "distance",   "p1": 0, "p2": 1, "value": 10.0},
        {"type": "vertical",   "p1": 1, "p2": 2},
        {"type": "distance",   "p1": 1, "p2": 2, "value": 5.0}
    ]"#;

    let solved = zenith_cad::payload::solve_2d_sketch(points, constraints)
        .expect("スケッチが解けませんでした");
    let out: Vec<[f64; 2]> = serde_json::from_str(&solved).expect("返ってきた JSON が読めません");
    assert_eq!(out.len(), 3, "点の数が変わりました: {solved}");

    // **返ってきた答えが、拘束を満たしているか測ります。**
    // 「解けた」と言われたことではなく、答えのほうを見ます。
    assert!(
        (out[0][0]).abs() <= 1e-6 && (out[0][1]).abs() <= 1e-6,
        "固定した点0が動きました: {:?}",
        out[0]
    );
    let horizontal = (out[1][1] - out[0][1]).abs();
    assert!(
        horizontal <= 1e-6,
        "点0-1 が水平になっていません: {horizontal:.3e}"
    );
    let first = ((out[1][0] - out[0][0]).powi(2) + (out[1][1] - out[0][1]).powi(2)).sqrt();
    assert!(
        (first - 10.0).abs() <= 1e-6,
        "点0-1 の距離が {first:.9} で、指定の 10 と違います"
    );
    let vertical = (out[2][0] - out[1][0]).abs();
    assert!(
        vertical <= 1e-6,
        "点1-2 が鉛直になっていません: {vertical:.3e}"
    );
    let second = ((out[2][0] - out[1][0]).powi(2) + (out[2][1] - out[1][1]).powi(2)).sqrt();
    assert!(
        (second - 5.0).abs() <= 1e-6,
        "点1-2 の距離が {second:.9} で、指定の 5 と違います"
    );
}

#[test]
fn the_shader_payload_is_readable_json() {
    for kind in ["box", "cylinder", "sphere", "cone", "torus"] {
        let json = zenith_cad::payload::get_primitive_shader_payload(kind, 10.0, 6.0, 20.0, 0.0)
            .unwrap_or_else(|_| panic!("{kind} のペイロードが作れません"));
        let value: serde_json::Value =
            serde_json::from_str(&json).unwrap_or_else(|_| panic!("{kind} の JSON が読めません"));
        assert!(
            value.is_object(),
            "{kind} のペイロードがオブジェクトではありません: {json}"
        );
    }

    // **知らない種類は断ること。** 黙って何かを返してはいけません。
    assert!(
        zenith_cad::payload::get_primitive_shader_payload("doughnut", 1.0, 1.0, 1.0, 0.0).is_err(),
        "知らない種類なのに何かが返りました"
    );
}

#[test]
fn a_box_payload_carries_the_shape() {
    let json = zenith_cad::payload::get_box_shader_payload(10.0, 20.0, 30.0)
        .expect("ペイロードが作れません");
    let value: serde_json::Value = serde_json::from_str(&json).expect("JSON が読めません");
    assert!(
        value.is_object(),
        "ペイロードがオブジェクトではありません: {json}"
    );
    // 中身の形は実装のものなので、**空でないことだけ**を測ります。
    assert!(
        value.as_object().map(|o| !o.is_empty()).unwrap_or(false),
        "ペイロードが空です: {json}"
    );
}

// ---- スケッチから立体へ、Python の口を通して（4-400）----
//
// **2026/09/08 まで、Python から呼べるのは `solve_2d_sketch` だけ**でした
// ——**拘束を解いて点を返すところまで**で、**立体にする口がありません**。
// **Blender 側は、スケッチを解けても形にできませんでした。**

#[test]
fn a_sketch_extrudes_through_the_binding() {
    // **閉じた式**: `面積 × 高さ`。
    let sketch = r#"{
        "points": [[0.0, 0.0], [40.0, 0.0], [40.0, 30.0], [0.0, 30.0]],
        "lines": [[0, 1], [1, 2], [2, 3], [3, 0]]
    }"#;
    let solid = PySolid::from_sketch_extruded(sketch, 7.0).expect("押し出せません");
    let expected = 40.0 * 30.0 * 7.0;
    let measured: f64 = solid.volume();
    assert!(
        close(measured, expected, 1e-9),
        "体積が {measured:.9} で、面積 × 高さ の {expected:.9} と違います"
    );
}

#[test]
fn a_sketch_with_a_hole_extrudes_through_the_binding() {
    // **閉じた式**: `(W·H − πr²) × 高さ`。**穴も Python から渡せます。**
    //
    // 点 0..3 が外形、点 4 が円の中心、点 5..8 が四半弧の継ぎ目。
    let sketch = r#"{
        "points": [[0.0, 0.0], [40.0, 0.0], [40.0, 30.0], [0.0, 30.0],
                   [20.0, 15.0],
                   [25.0, 15.0], [20.0, 20.0], [15.0, 15.0], [20.0, 10.0]],
        "lines": [[0, 1], [1, 2], [2, 3], [3, 0]],
        "arcs": [{"centre": 4, "start": 5, "end": 6},
                 {"centre": 4, "start": 6, "end": 7},
                 {"centre": 4, "start": 7, "end": 8},
                 {"centre": 4, "start": 8, "end": 5}]
    }"#;
    let solid = PySolid::from_sketch_extruded(sketch, 7.0).expect("押し出せません");
    let expected = (40.0 * 30.0 - PI * 25.0) * 7.0;
    let measured: f64 = solid.volume();
    assert!(
        close(measured, expected, 1e-9),
        "体積が {measured:.9} で、(W·H − πr²)·高さ の {expected:.9} と違います"
    );
}

#[test]
fn a_sketch_revolves_through_the_binding() {
    // **閉じた式**: パップスの定理 `2π × 重心の半径 × 面積`。
    let sketch = r#"{
        "points": [[10.0, 0.0], [14.0, 0.0], [14.0, 6.0], [10.0, 6.0]],
        "lines": [[0, 1], [1, 2], [2, 3], [3, 0]]
    }"#;
    let solid = PySolid::from_sketch_revolved(sketch, 0.0, 0.0, 0.0, 1.0).expect("回せません");
    let expected = 2.0 * PI * 12.0 * (4.0 * 6.0);
    let measured: f64 = solid.volume();
    assert!(
        close(measured, expected, 1e-9),
        "体積が {measured:.9} で、パップスの {expected:.9} と違います"
    );
}

#[test]
fn a_bad_sketch_is_refused_by_name_through_the_binding() {
    // **黙って 0 番に丸めてはいけません。** 間違った輪郭から、
    // もっともらしい立体ができます。
    let out_of_range = r#"{
        "points": [[0.0, 0.0], [10.0, 0.0], [10.0, 10.0]],
        "lines": [[0, 1], [1, 9], [9, 0]]
    }"#;
    assert!(
        PySolid::from_sketch_extruded(out_of_range, 5.0).is_err(),
        "点の番号が範囲の外なのに立体が返りました"
    );

    // **線も弧も無い図は断ります。**
    let no_segments = r#"{"points": [[0.0, 0.0], [10.0, 0.0], [10.0, 10.0]]}"#;
    assert!(
        PySolid::from_sketch_extruded(no_segments, 5.0).is_err(),
        "線も弧も無いのに立体が返りました"
    );

    // **軸をまたぐ輪は、回せません。**
    let crossing = r#"{
        "points": [[-5.0, 0.0], [5.0, 0.0], [5.0, 6.0], [-5.0, 6.0]],
        "lines": [[0, 1], [1, 2], [2, 3], [3, 0]]
    }"#;
    assert!(
        PySolid::from_sketch_revolved(crossing, 0.0, 0.0, 0.0, 1.0).is_err(),
        "軸をまたいでいるのに回りました"
    );
}

#[test]
fn a_sketch_constraint_that_cannot_be_understood_is_refused_not_ignored() {
    // **黙って落としません**（4-401）。
    //
    // ここは長らく、**知らない種類を無視**し、**番号が範囲の外なら拘束ごと
    // 捨て**、**`value` が無ければ 10.0 を入れて**いました。
    // **呼んだ側は「解けた」点を受け取ります**——**自分が掛けたつもりの
    // 拘束が 1 本も入っていなくても**です。
    let points = "[[0.0, 0.0], [3.0, 4.0], [9.0, 1.0]]";

    // **綴り違い。** これが黙って通ると、何も掛かっていない図が
    // 「解けた」と返ります。
    let misspelled = r#"[{"type": "horizontol", "p1": 0, "p2": 1}]"#;
    assert!(
        zenith_cad::payload::solve_2d_sketch(points, misspelled).is_err(),
        "知らない拘束の種類なのに解けたと返りました"
    );

    // **番号が範囲の外。**
    let out_of_range = r#"[{"type": "distance", "p1": 0, "p2": 9, "value": 10.0}]"#;
    assert!(
        zenith_cad::payload::solve_2d_sketch(points, out_of_range).is_err(),
        "点の番号が範囲の外なのに解けたと返りました"
    );

    // **`value` が無い。** 既定の 10.0 が入ると、**頼んでいない寸法**が
    // 掛かります。
    let no_value = r#"[{"type": "distance", "p1": 0, "p2": 1}]"#;
    assert!(
        zenith_cad::payload::solve_2d_sketch(points, no_value).is_err(),
        "value が無いのに解けたと返りました"
    );

    // **正しく書いたものは、通ります。**
    let good = r#"[{"type": "distance", "p1": 0, "p2": 1, "value": 10.0}]"#;
    assert!(
        zenith_cad::payload::solve_2d_sketch(points, good).is_ok(),
        "正しい拘束が断られました"
    );
}
