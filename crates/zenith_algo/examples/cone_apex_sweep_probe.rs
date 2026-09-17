//! **平面が円錐の頂点の近くを切る所**を掃く（4-476）。
//!
//! # なぜ要るのか
//!
//! 4-451〜4-475 で、**浅く当たる所**の断りを 4 つの組で閉じました——
//! **平面 × 円柱**、**円柱 × 円柱**、**球 × 球**、**平面 × 球**。
//!
//! **円錐は掃いていません。** 円錐には**頂点**があり、そこは球の極と
//! 同じくパラメータが潰れます。**平面が頂点のすぐ下を切ると、どうなるか**
//! は測っていません。
//!
//! # 何を測るか
//!
//! **底面半径 5・高さ 10 の円錐を、`z ≥ z0` を占める箱で切ります。**
//! **`z0 = 10` がちょうど頂点**です。
//!
//! 閉じた式は**相似な小円錐**です——深さ `d = 10 − z0` として
//! `V = π (rb d / h)² d / 3`。
//!
//! * **積** = 小円錐
//! * **和** = 円錐 + 箱 − 小円錐
//! * **差**（円錐 − 箱）= 円錐 − 小円錐
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
//! cargo run --release -p zenith_algo --example cone_apex_sweep_probe
//! ```

use zenith_algo::{BooleanEngine, BooleanOpType, BrepTransform, MassCalculator, PrimitiveBuilder};
use zenith_math::{Tolerance, Vec3};
use zenith_tess::TessellationParams;
use zenith_topo::Solid;

const RB: f64 = 5.0;
const H: f64 = 10.0;
/// 箱の一辺。**小円錐を丸ごと含み、他の面は遠くにある**だけの大きさ。
const BOX_SIDE: f64 = 40.0;

fn volume_of(solids: &[Solid]) -> f64 {
    solids
        .iter()
        .map(|solid| MassCalculator::compute_from_brep(solid, &TessellationParams::default()).volume)
        .sum()
}

/// **頂点から深さ `d` までの小円錐**。離れていれば 0。
fn tip_volume(d: f64) -> f64 {
    if d <= 0.0 {
        return 0.0;
    }
    let r = RB * d / H;
    std::f64::consts::PI * r * r * d / 3.0
}

fn main() {
    let tol = Tolerance::default();
    let cone = std::f64::consts::PI * RB * RB * H / 3.0;
    let box_volume = BOX_SIDE * BOX_SIDE * BOX_SIDE;

    println!("平面が円錐の頂点の近くを切る所を掃く（4-476）");
    println!();
    println!("**底面半径 {RB}・高さ {H} の円錐を、`z ≥ z0` を占める箱で切ります。**");
    println!("**z0 = {H} がちょうど頂点です。**");
    println!();
    println!(
        "{:>12}{:>9}{:>15}{:>15}{:>12}  {}",
        "箱の下の縁", "深さ", "|A∩B|", "閉じた式", "相対差", "3 演算（名=名指し 未=未実装 誤=閉じた式から外れ）"
    );
    println!("{}", "-".repeat(104));

    let mut wrong = 0usize;
    let mut refused_while_apart = 0usize;

    let mut depths: Vec<f64> = vec![
        -1.0, -0.1, -0.01, -1e-4, 0.0, 1e-4, 1e-3, 0.01, 0.05, 0.1, 0.3, 1.0, 2.5, 5.0,
    ];
    if let Ok(text) = std::env::var("ZENITH_CONE_EXTRA") {
        for piece in text.split(',') {
            if let Ok(value) = piece.trim().parse::<f64>() {
                depths.push(value);
            }
        }
        depths.sort_by(|a, b| a.partial_cmp(b).unwrap());
    }
    let only: Option<f64> = std::env::var("ZENITH_CONE_ONLY")
        .ok()
        .and_then(|text| text.parse().ok());

    for depth in depths {
        let z0 = H - depth;
        if let Some(only) = only {
            if (depth - only).abs() > 1e-12 {
                continue;
            }
        }
        let a = PrimitiveBuilder::make_cone(RB, 0.0, H).expect("円錐");
        let b = BrepTransform::translate_solid(
            &PrimitiveBuilder::make_box(BOX_SIDE, BOX_SIDE, BOX_SIDE).expect("箱"),
            Vec3::new(-BOX_SIDE * 0.5, -BOX_SIDE * 0.5, z0),
        );

        let tip = tip_volume(depth);
        let mut per_op = ["和 ○", "積 ○", "差 ○"];
        let mut shown: Option<f64> = None;
        for (index, (op, want)) in [
            (BooleanOpType::Union, cone + box_volume - tip),
            (BooleanOpType::Intersection, tip),
            (BooleanOpType::Difference, cone - tip),
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
                    if std::env::var_os("ZENITH_CONE_WHY").is_some() {
                        eprintln!(
                            "CONEWHY 深さ={depth:.6} {} — {}",
                            ["和", "積", "差"][index],
                            reason.chars().take(600).collect::<String>()
                        );
                    }
                }
            }
        }

        match shown {
            Some(got) => {
                let relative = (got - tip).abs() / tip.abs().max(1.0);
                println!(
                    "{z0:>12.6}{depth:>9.0e}{got:>15.4}{tip:>15.4}{relative:>12.1e}  {}",
                    per_op.join(" ")
                );
            }
            None => println!(
                "{z0:>12.6}{depth:>9.0e}{:>15}{tip:>15.4}{:>12}  {}",
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
