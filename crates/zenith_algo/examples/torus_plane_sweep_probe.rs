//! **平面がトーラスの外周に浅く当たる所**を掃く（4-477）。
//!
//! # なぜ要るのか
//!
//! 4-451〜4-476 で、**浅く当たる所**を 5 つの組で閉じました——
//! **平面 × 円柱**、**円柱 × 円柱**、**球 × 球**、**平面 × 球**、
//! **平面 × 円錐**。**トーラスは掃いていません。**
//!
//! トーラスは**この中でいちばん込み入った面**です——**同じ平面が
//! 輪を 2 本作る**（内側と外側）ことがあり、**切り口が繋がるとは
//! 限りません**。
//!
//! # 何を測るか
//!
//! **主半径 6・管半径 2 のトーラスを、`z ≥ z0` を占める箱で切ります。**
//! **`z0 = 2` が管のてっぺん**で、そこでちょうど接します。
//!
//! 閉じた式は**パップスの定理**です。切り口の断面は**円の切片**で、
//! 水平に切るので**重心の半径は主半径のまま**——だから
//!
//! ```text
//! A = r² acos(z0/r) − z0 √(r² − z0²)     （切片の面積）
//! V = 2π R A                              （回してできる体積）
//! ```
//!
//! * **積** = 上の V
//! * **和** = トーラス + 箱 − V
//! * **差**（トーラス − 箱）= トーラス − V
//!
//! # 読み方
//!
//! **断りは誤りではありません**（3-1）。**返ってきたのに閉じた式から
//! 外れているもの**だけを赤にします——**3 演算を 1 つずつ**当てます
//! （4-462 の教訓）。
//!
//! # 使い方
//!
//! ```bash
//! cargo run --release -p zenith_algo --example torus_plane_sweep_probe
//! ```

use zenith_algo::{BooleanEngine, BooleanOpType, BrepTransform, MassCalculator, PrimitiveBuilder};
use zenith_math::{Tolerance, Vec3};
use zenith_tess::TessellationParams;
use zenith_topo::Solid;

const R_MAJOR: f64 = 6.0;
const R_MINOR: f64 = 2.0;
const BOX_SIDE: f64 = 40.0;

fn volume_of(solids: &[Solid]) -> f64 {
    solids
        .iter()
        .map(|solid| MassCalculator::compute_from_brep(solid, &TessellationParams::default()).volume)
        .sum()
}

/// **`z ≥ z0` に残るトーラスの体積**（パップス）。離れていれば 0。
fn crown_volume(z0: f64) -> f64 {
    if z0 >= R_MINOR {
        return 0.0;
    }
    let r = R_MINOR;
    let segment = r * r * (z0 / r).acos() - z0 * (r * r - z0 * z0).sqrt();
    2.0 * std::f64::consts::PI * R_MAJOR * segment
}

fn main() {
    let tol = Tolerance::default();
    let torus = 2.0 * std::f64::consts::PI.powi(2) * R_MAJOR * R_MINOR * R_MINOR;
    let box_volume = BOX_SIDE * BOX_SIDE * BOX_SIDE;

    println!("平面がトーラスの外周に浅く当たる所を掃く（4-477）");
    println!();
    println!("**主半径 {R_MAJOR}・管半径 {R_MINOR} のトーラスを、`z ≥ z0` を占める箱で切ります。**");
    println!("**z0 = {R_MINOR} が管のてっぺんです。**");
    println!();
    println!(
        "{:>12}{:>9}{:>15}{:>15}{:>12}  {}",
        "箱の下の縁", "深さ", "|A∩B|", "閉じた式", "相対差", "3 演算（名=名指し 未=未実装 誤=閉じた式から外れ）"
    );
    println!("{}", "-".repeat(104));

    let mut wrong = 0usize;
    let mut refused_while_apart = 0usize;

    let mut depths: Vec<f64> = vec![
        -1.0, -0.1, -0.01, -1e-4, 0.0, 1e-4, 1e-3, 0.01, 0.05, 0.1, 0.3, 1.0, 2.0, 3.0,
    ];
    if let Ok(text) = std::env::var("ZENITH_TORUS_EXTRA") {
        for piece in text.split(',') {
            if let Ok(value) = piece.trim().parse::<f64>() {
                depths.push(value);
            }
        }
        depths.sort_by(|a, b| a.partial_cmp(b).unwrap());
    }
    let only: Option<f64> = std::env::var("ZENITH_TORUS_ONLY")
        .ok()
        .and_then(|text| text.parse().ok());

    for depth in depths {
        let z0 = R_MINOR - depth;
        if let Some(only) = only {
            if (depth - only).abs() > 1e-12 {
                continue;
            }
        }
        let a = PrimitiveBuilder::make_torus(R_MAJOR, R_MINOR).expect("トーラス");
        let b = BrepTransform::translate_solid(
            &PrimitiveBuilder::make_box(BOX_SIDE, BOX_SIDE, BOX_SIDE).expect("箱"),
            Vec3::new(-BOX_SIDE * 0.5, -BOX_SIDE * 0.5, z0),
        );

        let crown = crown_volume(z0);
        let mut per_op = ["和 ○", "積 ○", "差 ○"];
        let mut shown: Option<f64> = None;
        for (index, (op, want)) in [
            (BooleanOpType::Union, torus + box_volume - crown),
            (BooleanOpType::Intersection, crown),
            (BooleanOpType::Difference, torus - crown),
        ]
        .into_iter()
        .enumerate()
        {
            match BooleanEngine::boolean_solids_exact_result(&a, &b, op, &tol) {
                Ok(result) => {
                    let measured = volume_of(&result.solids);
                    if index == 1 {
                        shown = Some(measured);
                    }
                    let off = (measured - want).abs() / want.abs().max(1.0);
                    if off > 1e-4 {
                        wrong += 1;
                        per_op[index] = ["**和 誤**", "**積 誤**", "**差 誤**"][index];
                        println!(
                            "  **誤答** 深さ={depth:.6} {}: 返り {measured:.9}、閉じた式 {want:.9}（相対差 {off:.3e}）",
                            ["和", "積", "差"][index]
                        );
                    }
                }
                Err(reason) => {
                    per_op[index] = if reason.contains("not implemented") {
                        ["**和 未**", "**積 未**", "**差 未**"][index]
                    } else if reason.contains("refuses this placement") {
                        ["和 名", "積 名", "差 名"][index]
                    } else {
                        ["和 ×", "積 ×", "差 ×"][index]
                    };
                    if depth != 0.0 {
                        refused_while_apart += 1;
                    }
                    if std::env::var_os("ZENITH_TORUS_WHY").is_some() {
                        eprintln!(
                            "TORUSWHY 深さ={depth:.6} {} — {}",
                            ["和", "積", "差"][index],
                            reason.chars().take(600).collect::<String>()
                        );
                    }
                }
            }
        }

        match shown {
            Some(got) => {
                let relative = (got - crown).abs() / crown.abs().max(1.0);
                println!(
                    "{z0:>12.6}{depth:>9.0e}{got:>15.4}{crown:>15.4}{relative:>12.1e}  {}",
                    per_op.join(" ")
                );
            }
            None => println!(
                "{z0:>12.6}{depth:>9.0e}{:>15}{crown:>15.4}{:>12}  {}",
                "断り",
                "-",
                per_op.join(" ")
            ),
        }
    }

    println!();
    println!("**接する所より外での断り: {refused_while_apart} 件**");
    println!();
    if wrong > 0 {
        println!("**返ってきたのに閉じた式から外れた演算が {wrong} 件あります。**");
        std::process::exit(1);
    }
    println!("**返ったものは全部、閉じた式と 1e-4 以内で合っています。**");
    println!();
    println!("**断りは誤りではありません**（3-1）。**ここは赤にしません。**");
}
