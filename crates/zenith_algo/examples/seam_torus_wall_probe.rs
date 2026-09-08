//! **H8 の壁を、基本形 2 つだけで出す**（4-412）。
//!
//! # なぜ要るのか
//!
//! H8（読んだ立体を切る）の壁は、9 日間 `linkrods.step` で見ていました
//! ——**相手のいない稜 132 本**。**そのファイルは失われました**
//! （`reference/` ごと。2026/09/08。5 章の落とし穴）。
//!
//! **同じ壁が、基本形 2 つから出ます。** `contact_placement_probe` に
//! 継ぎ目どうしを当てる置き方を足したときに出ました（4-412）——
//! **どちらも回したトーラス 2 つ**。
//!
//! ```text
//! 33 selected face pieces,
//! 12 unmatched edge uses, 3 non-manifold edge uses
//! ```
//!
//! **`linkrods` の 132 本より、12 本のほうが追えます。**
//!
//! # なぜ別の口にしたのか
//!
//! `contact_placement_probe` は **50 配置・150 演算**を回します。
//! `ZENITH_SPLIT_WHY=1` を立てると**全部の配置の断り文が混ざる**ので、
//! **この検体だけの内訳が数えられません**（実測: 1726 行のうち、
//! どれがこの検体のものか分けられませんでした）。
//!
//! **ここは、この 2 つしか作りません。**
//!
//! # 使い方
//!
//! ```bash
//! cargo run --release -p zenith_algo --example seam_torus_wall_probe
//!
//! # 断り文の内訳を数える（4-374 の道具がそのまま使えます）
//! ZENITH_SPLIT_WHY=1 cargo run --release -p zenith_algo \
//!   --example seam_torus_wall_probe > out.txt 2>&1
//! py tools/tally_split_reasons.py out.txt
//! ```
//!
//! # 読み方
//!
//! **これは診断です。赤にはしません**——**いま断られるのが分かっている
//! 検体**だからです。**赤にするのは `contact_placement_probe` の側**で、
//! そちらは `ZENITH_CONTACT_SEAMS=1` を立てたときだけこの置き方を含みます。

use zenith_algo::{BooleanEngine, BooleanOpType, BrepTransform, PrimitiveBuilder};
use zenith_math::{Tolerance, Transform3, Vec3};
use zenith_topo::Solid;

/// 稜のうち、ちょうど 2 つの面ループに使われていない本数。
///
/// **`id` ではなく位置で突き合わせます**——同じ弧を共有していても、
/// 面ごとに別の `Edge` の実体を持つことがあります（4-80）。
fn non_manifold_edges(solid: &Solid) -> usize {
    use std::collections::BTreeMap;
    let key = |edge: &zenith_topo::OrientedEdge| {
        let start = edge.evaluate_normalized(0.0);
        let middle = edge.evaluate_normalized(0.5);
        let end = edge.evaluate_normalized(1.0);
        let mut ends = [
            (
                (start.x * 1e7).round() as i64,
                (start.y * 1e7).round() as i64,
                (start.z * 1e7).round() as i64,
            ),
            (
                (end.x * 1e7).round() as i64,
                (end.y * 1e7).round() as i64,
                (end.z * 1e7).round() as i64,
            ),
        ];
        ends.sort();
        (
            ends[0],
            ends[1],
            (
                (middle.x * 1e7).round() as i64,
                (middle.y * 1e7).round() as i64,
                (middle.z * 1e7).round() as i64,
            ),
        )
    };
    let mut uses: BTreeMap<_, usize> = BTreeMap::new();
    for shell in std::iter::once(&solid.outer_shell).chain(solid.inner_shells.iter()) {
        for face in &shell.faces {
            for wire in std::iter::once(&face.outer_wire).chain(face.inner_wires.iter()) {
                for edge in &wire.edges {
                    *uses.entry(key(edge)).or_insert(0) += 1;
                }
            }
        }
    }
    uses.values().filter(|count| **count != 2).count()
}

fn main() {
    let tol = Tolerance::default();

    // **`contact_placement_probe` の `torus x torus (both turned, tubes
    // overlapping)` と、同じ 2 つ**です。値を変えたら、あちらも変えて
    // ください——**別の形を同じ名前で語ると、次の人が突き合わせられません。**
    let torus = PrimitiveBuilder::make_torus(12.0, 4.0).expect("torus");
    let spin = Transform3::from_axis_angle(&Vec3::new(0.0, 0.0, 1.0), 29f64.to_radians());
    let tip = Transform3::from_axis_angle(&Vec3::new(1.0, 0.0, 0.0), 61f64.to_radians());

    let a = BrepTransform::transform_solid(&torus, &spin).expect("turn");
    let b = BrepTransform::translate_solid(
        &BrepTransform::transform_solid(&torus, &tip).expect("tip"),
        Vec3::new(18.0, 0.0, 0.0),
    );

    println!("H8 の壁を、基本形 2 つだけで出す（4-412）");
    println!();
    println!("トーラス R=12 r=4 を 2 つ。**一方は継ぎ目を z まわりに 29 度**、");
    println!("**他方は x まわりに 61 度**傾け、x に 18 ずらして管どうしを交わらせます。");
    println!();
    println!("A: 面 {} 枚、B: 面 {} 枚", a.outer_shell.faces.len(), b.outer_shell.faces.len());
    println!();

    for (label, op) in [
        ("union", BooleanOpType::Union),
        ("difference", BooleanOpType::Difference),
        ("intersection", BooleanOpType::Intersection),
    ] {
        match BooleanEngine::boolean_solids_exact_result(&a, &b, op, &tol) {
            Ok(result) => {
                let solids = &result.solids;
                let bad: usize = solids.iter().map(non_manifold_edges).sum();
                let faces: usize = solids.iter().map(|s| s.outer_shell.faces.len()).sum();
                println!("{label:<14} 返りました: 立体 {} 個、面 {faces} 枚、非多様体の稜 {bad} 本", solids.len());
            }
            Err(reason) => {
                println!("{label:<14} **断られました**");
                // **断り文は、そのまま出します。** 要約すると、次の人が
                // 数え直せません（4-383 で、数え方の違いで筆頭が入れ替わりました）。
                for part in reason.split("; ") {
                    println!("    {part}");
                }
            }
        }
        println!();
    }

    println!("**これは診断です。赤にはしません**——いま断られるのが分かっている");
    println!("検体だからです。**赤にするのは `contact_placement_probe` の側**で、");
    println!("そちらは `ZENITH_CONTACT_SEAMS=1` のときだけこの置き方を含みます。");
    println!();
    println!("**断り文の内訳を数えるには**:");
    println!("  ZENITH_SPLIT_WHY=1 cargo run --release -p zenith_algo \\");
    println!("    --example seam_torus_wall_probe > out.txt 2>&1");
    println!("  py tools/tally_split_reasons.py out.txt");
    println!();
    check_loose_ends(&a, &b, &tol);
}

/// **浮いた端が、その組の両方のパッチに乗っているか**（4-412）。
///
/// 交線が 2 本足りないと数えたあと、**落ちた組を名指し**しました——
/// `A面15 × B面10` と `A面15 × B面8`、どちらも**辿れた枝 0 本**。
///
/// **「その組に交線が通るはずだ」は、まだ推測です。** ここで確かめます
/// ——**浮いた端を、両方のパッチへ射影**します。**両方に乗っていれば、
/// 交線はそこを通っており、追跡器が取り落としています。**
fn check_loose_ends(a: &Solid, b: &Solid, tol: &Tolerance) {
    use zenith_geom::ExtremumEngine;
    use zenith_math::Point3;
    use zenith_topo::FaceGeometry;

    let surface = |solid: &Solid, index: usize| match &solid.outer_shell.faces[index].geometry {
        FaceGeometry::Nurbs(nurbs) => Some(nurbs.clone()),
        _ => None,
    };
    let distance = |point: Point3, index: usize, solid: &Solid| -> String {
        match surface(solid, index) {
            Some(nurbs) => match ExtremumEngine::point_to_surface(point, &nurbs, 32, tol.parametric)
            {
                Ok(projection) => format!("{:.3e}", projection.distance),
                Err(_) => "射影できず".to_string(),
            },
            None => "nurbs ではない".to_string(),
        }
    };

    println!("**浮いた端は、その組の両方のパッチに乗っているか**（4-412）");
    println!();
    println!("{:<34}{:>14}{:>14}{:>14}", "浮いた端", "A面15 まで", "B面8 まで", "B面10 まで");
    println!("{}", "-".repeat(78));
    for (x, y, z) in [
        (11.422866, -2.207950, -3.983248),
        (11.632495, -2.947042, -4.000000),
        (10.226803, -6.277938, -4.000000),
        (7.773197, -6.542084, -3.551570),
    ] {
        let point = Point3::new(x, y, z);
        println!(
            "({:>9.4},{:>9.4},{:>9.4}){:>14}{:>14}{:>14}",
            x,
            y,
            z,
            distance(point, 15, a),
            distance(point, 8, b),
            distance(point, 10, b),
        );
    }
    println!("{}", "-".repeat(78));
    println!();
    println!("**両方に乗っていれば、交線はそこを通っています**——");
    println!("**取り落としているのは `fit_all_branches` のほう**です。");
}
