//! **球どうしが接する寸前・ちょうど・過ぎた後**を掃く（4-468）。
//!
//! # なぜ要るのか
//!
//! 4-451〜4-467 で、**浅く当たる所**の帯を 2 つ閉じました——
//! **平面 × 曲面**（4-457）と、**円柱 × 円柱**（4-465、4-467）。
//!
//! **球どうしは掃いていません。** 円柱は**母線が直線**なので、
//! 交線も面の割り方も、その直線に助けられています。**球には
//! 直線がありません。** **同じ形の穴が空いていないか**は、
//! 測らなければ書けません。
//!
//! # 何を測るか
//!
//! **半径 5 と 3 の球を近づけます。** **中心間 8 でちょうど接します。**
//!
//! 閉じた式はレンズ（2 球の重なり）の体積——
//! `π (r₁+r₂−d)² (d² + 2d r₂ − 3r₂² + 2d r₁ + 6r₁r₂ − 3r₁²) / (12 d)`。
//!
//! # 読み方
//!
//! **断りは誤りではありません**（3-1）。**返ってきたのに閉じた式から
//! 外れているもの**だけを赤にします——**3 演算を 1 つずつ**当てます
//! （4-462 で、恒等式だけでは誤答を見逃すと分かりました）。
//!
//! # 使い方
//!
//! ```bash
//! cargo run --release -p zenith_algo --example sphere_tangency_sweep_probe
//! ```

use zenith_algo::{
    BooleanEngine, BooleanOpType, BrepTransform, MassCalculator, PrimitiveBuilder,
};
use zenith_math::{Tolerance, Vec3};
use zenith_tess::TessellationParams;
use zenith_topo::Solid;

const R1: f64 = 5.0;
const R2: f64 = 3.0;

fn volume_of(solids: &[Solid]) -> f64 {
    solids
        .iter()
        .map(|solid| MassCalculator::compute_from_brep(solid, &TessellationParams::default()).volume)
        .sum()
}

/// **2 つの球が重なる体積**。離れていれば 0。
fn lens_volume(d: f64) -> f64 {
    if d >= R1 + R2 {
        return 0.0;
    }
    if d <= (R1 - R2).abs() {
        return 4.0 / 3.0 * std::f64::consts::PI * R1.min(R2).powi(3);
    }
    let head = std::f64::consts::PI * (R1 + R2 - d).powi(2);
    let tail = d * d + 2.0 * d * R2 - 3.0 * R2 * R2 + 2.0 * d * R1 + 6.0 * R1 * R2 - 3.0 * R1 * R1;
    head * tail / (12.0 * d)
}

fn main() {
    let tol = Tolerance::default();
    let ball_a = 4.0 / 3.0 * std::f64::consts::PI * R1.powi(3);
    let ball_b = 4.0 / 3.0 * std::f64::consts::PI * R2.powi(3);

    println!("球どうしが接する寸前・ちょうど・過ぎた後を掃く（4-468）");
    println!();
    println!("**半径 {R1} と {R2} の球。中心間が {} でちょうど接します。**", R1 + R2);
    println!();
    println!(
        "{:>12}{:>9}{:>15}{:>15}{:>12}  {}",
        "中心間", "離れ", "|A∩B|", "閉じた式", "相対差", "3 演算（名=名指し 未=未実装 誤=閉じた式から外れ）"
    );
    println!("{}", "-".repeat(104));

    let touch = R1 + R2;
    let mut wrong = 0usize;
    let mut refused_while_apart = 0usize;

    let mut offsets: Vec<f64> = vec![
        -1.0, -0.3, -0.1, -0.03, -0.01, -1e-3, -1e-4, 0.0, 1e-4, 1e-3, 0.01, 0.1, 1.0,
    ];
    if let Ok(text) = std::env::var("ZENITH_SPHERE_EXTRA") {
        for piece in text.split(',') {
            if let Ok(value) = piece.trim().parse::<f64>() {
                offsets.push(value);
            }
        }
        offsets.sort_by(|a, b| a.partial_cmp(b).unwrap());
    }
    let only: Option<f64> = std::env::var("ZENITH_SPHERE_ONLY")
        .ok()
        .and_then(|text| text.parse().ok());

    for offset in offsets {
        let d = touch + offset;
        if let Some(only) = only {
            if (d - only).abs() > 1e-9 {
                continue;
            }
        }
        let a = PrimitiveBuilder::make_sphere(R1).expect("大きい球");
        let b = BrepTransform::translate_solid(
            &PrimitiveBuilder::make_sphere(R2).expect("小さい球"),
            Vec3::new(d, 0.0, 0.0),
        );

        let lens = lens_volume(d);
        let mut per_op = ["和 ○", "積 ○", "差 ○"];
        let mut shown: Option<f64> = None;
        for (index, (op, want)) in [
            (BooleanOpType::Union, ball_a + ball_b - lens),
            (BooleanOpType::Intersection, lens),
            (BooleanOpType::Difference, ball_a - lens),
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
                            "  **誤答** d={d:.6} {}: 返り {measured:.9}、閉じた式 {want:.9}（相対差 {off:.3e}）",
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
                    if offset != 0.0 {
                        refused_while_apart += 1;
                    }
                    if std::env::var_os("ZENITH_SPHERE_WHY").is_some() {
                        eprintln!(
                            "SPHEREWHY d={d:.6} {} — {}",
                            ["和", "積", "差"][index],
                            reason.chars().take(600).collect::<String>()
                        );
                    }
                }
            }
        }

        match shown {
            Some(got) => {
                let relative = (got - lens).abs() / lens.abs().max(1.0);
                println!(
                    "{d:>12.6}{offset:>9.0e}{got:>15.4}{lens:>15.4}{relative:>12.1e}  {}",
                    per_op.join(" ")
                );
            }
            None => println!(
                "{d:>12.6}{offset:>9.0e}{:>15}{lens:>15.4}{:>12}  {}",
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
