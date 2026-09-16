//! **平面が球に浅く当たる所**を掃く（4-475）。
//!
//! # なぜ要るのか
//!
//! 4-451〜4-474 で、**浅く当たる所**の断りを 3 つの組で閉じました——
//! **平面 × 円柱**（穴の縁に帯）、**円柱 × 円柱**、**球 × 球**。
//!
//! **平面 × 球は掃いていません。** 球には**極**があり、そこは
//! パラメータが潰れます（4-470 で見ました）。**平面が極の近くを
//! 浅く切ると、どうなるか**は測っていません。
//!
//! # 何を測るか
//!
//! **半径 5 の球を、箱の面で切ります。** 箱は `x ≥ x0` を占め、
//! **`x0 = 5` でちょうど接します**（球の東の極）。
//!
//! 閉じた式は**球冠**です——深さ `h = 5 − x0` として
//! `V = πh²(3r − h) / 3`。
//!
//! * **積** = 球冠
//! * **和** = 球 + 箱 − 球冠
//! * **差**（球 − 箱）= 球 − 球冠
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
//! cargo run --release -p zenith_algo --example sphere_cap_sweep_probe
//! ```

use zenith_algo::{
    BooleanEngine, BooleanOpType, BrepTransform, MassCalculator, PrimitiveBuilder,
};
use zenith_math::{Tolerance, Vec3};
use zenith_tess::TessellationParams;
use zenith_topo::Solid;

const R: f64 = 5.0;
/// 箱の一辺。**球冠を丸ごと含み、他の面は遠くにある**だけの大きさ。
const BOX_SIDE: f64 = 20.0;

fn volume_of(solids: &[Solid]) -> f64 {
    solids
        .iter()
        .map(|solid| MassCalculator::compute_from_brep(solid, &TessellationParams::default()).volume)
        .sum()
}

/// **球冠の体積**。深さ `h`。離れていれば 0。
fn cap_volume(x0: f64) -> f64 {
    let h = R - x0;
    if h <= 0.0 {
        return 0.0;
    }
    std::f64::consts::PI * h * h * (3.0 * R - h) / 3.0
}

fn main() {
    let tol = Tolerance::default();
    let ball = 4.0 / 3.0 * std::f64::consts::PI * R.powi(3);
    let box_volume = BOX_SIDE * BOX_SIDE * BOX_SIDE;

    println!("平面が球に浅く当たる所を掃く（4-475）");
    println!();
    println!("**半径 {R} の球を、`x ≥ x0` を占める箱で切ります。**");
    println!("**x0 = {R} でちょうど接します**（球の東の極）。");
    println!();
    println!(
        "{:>12}{:>9}{:>15}{:>15}{:>12}  {}",
        "箱の西の縁", "深さ", "|A∩B|", "閉じた式", "相対差", "3 演算（名=名指し 未=未実装 誤=閉じた式から外れ）"
    );
    println!("{}", "-".repeat(104));

    let mut wrong = 0usize;
    let mut refused_while_apart = 0usize;

    // **深さ**（`R - x0`）で並べます。負は離れている側。
    let mut depths: Vec<f64> = vec![
        -1.0, -0.1, -0.01, -1e-4, 0.0, 1e-4, 1e-3, 0.01, 0.05, 0.1, 0.3, 1.0, 2.5,
    ];
    if let Ok(text) = std::env::var("ZENITH_CAP_EXTRA") {
        for piece in text.split(',') {
            if let Ok(value) = piece.trim().parse::<f64>() {
                depths.push(value);
            }
        }
        depths.sort_by(|a, b| a.partial_cmp(b).unwrap());
    }
    let only: Option<f64> = std::env::var("ZENITH_CAP_ONLY")
        .ok()
        .and_then(|text| text.parse().ok());

    for depth in depths {
        let x0 = R - depth;
        if let Some(only) = only {
            if (x0 - only).abs() > 1e-9 {
                continue;
            }
        }
        let a = PrimitiveBuilder::make_sphere(R).expect("球");
        let b = BrepTransform::translate_solid(
            &PrimitiveBuilder::make_box(BOX_SIDE, BOX_SIDE, BOX_SIDE).expect("箱"),
            Vec3::new(x0, -BOX_SIDE * 0.5, -BOX_SIDE * 0.5),
        );

        let cap = cap_volume(x0);
        let mut per_op = ["和 ○", "積 ○", "差 ○"];
        let mut shown: Option<f64> = None;
        for (index, (op, want)) in [
            (BooleanOpType::Union, ball + box_volume - cap),
            (BooleanOpType::Intersection, cap),
            (BooleanOpType::Difference, ball - cap),
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
                    if off > 1e-5 {
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
                    if std::env::var_os("ZENITH_CAP_WHY").is_some() {
                        eprintln!(
                            "CAPWHY 深さ={depth:.6} {} — {}",
                            ["和", "積", "差"][index],
                            reason.chars().take(600).collect::<String>()
                        );
                    }
                }
            }
        }

        match shown {
            Some(got) => {
                let relative = (got - cap).abs() / cap.abs().max(1.0);
                println!(
                    "{x0:>12.6}{depth:>9.0e}{got:>15.4}{cap:>15.4}{relative:>12.1e}  {}",
                    per_op.join(" ")
                );
            }
            None => println!(
                "{x0:>12.6}{depth:>9.0e}{:>15}{cap:>15.4}{:>12}  {}",
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
    println!("**返ったものは全部、閉じた式と 1e-5 以内で合っています。**");
    println!();
    println!("**断りは誤りではありません**（3-1）。**ここは赤にしません。**");
}
