//! **曲面どうしが接する寸前・ちょうど・過ぎた後**を掃く（4-459）。
//!
//! # なぜ要るのか
//!
//! 4-451〜4-457 で、**平面が曲面の縁に浅く当たる所**に、幅 0.02 の帯が
//! ありました——**接していないのに、3 演算とも断られる**帯です。根は
//! **「境界の上」を光線の媒介変数で測っていたこと**で、4-457 で塞ぎ
//! ました。
//!
//! **塞いだのは、平面 × 曲面の切り詰めです。**
//!
//! **曲面どうしが浅く当たる所は、まだ掃いていません。** そこは別の道を
//! 通ります（`intersect_nurbs_patches`）。**同じ形の穴が空いていないか**は、
//! 測らなければ書けません。
//!
//! # 何を測るか
//!
//! **半径 5 と 3 の円柱を、軸を平行にしたまま近づけます。**
//! **中心間が 8 でちょうど接します**（外接）。
//!
//! * **8 より近い**——食い込んでいる。**3 演算とも返り、閉じた式に乗る**
//! * **8 ちょうど**——線で接している。**積は断られてよい**（規約 3-1）
//! * **8 より遠い**——離れている。**和は 2 つ、積は空、差は A**
//!
//! 閉じた式は**レンズの面積**です——
//! `r₁²·acos((d²+r₁²−r₂²)/(2dr₁)) + r₂²·acos((d²+r₂²−r₁²)/(2dr₂))
//!  − ½√((−d+r₁+r₂)(d+r₁−r₂)(d−r₁+r₂)(d+r₁+r₂))`。
//!
//! # 読み方
//!
//! **断りは誤りではありません**（3-1）。**返ってきたのに閉じた式から
//! 外れているもの**だけを赤にします。
//!
//! # 使い方
//!
//! ```bash
//! cargo run --release -p zenith_algo --example cylinder_tangency_sweep_probe
//! ```

use zenith_algo::{
    BooleanEngine, BooleanOpType, BrepTransform, MassCalculator, PrimitiveBuilder,
};
use zenith_math::{Tolerance, Vec3};
use zenith_tess::TessellationParams;
use zenith_topo::Solid;

const R1: f64 = 5.0;
const R2: f64 = 3.0;
const HEIGHT: f64 = 6.0;

fn volume_of(solids: &[Solid]) -> f64 {
    solids
        .iter()
        .map(|solid| MassCalculator::compute_from_brep(solid, &TessellationParams::default()).volume)
        .sum()
}

/// **2 つの円が重なる面積**（レンズ）。離れていれば 0。
fn lens_area(d: f64) -> f64 {
    if d >= R1 + R2 {
        return 0.0;
    }
    if d <= (R1 - R2).abs() {
        return std::f64::consts::PI * R1.min(R2).powi(2);
    }
    let first = R1 * R1 * ((d * d + R1 * R1 - R2 * R2) / (2.0 * d * R1)).clamp(-1.0, 1.0).acos();
    let second = R2 * R2 * ((d * d + R2 * R2 - R1 * R1) / (2.0 * d * R2)).clamp(-1.0, 1.0).acos();
    let root = ((-d + R1 + R2) * (d + R1 - R2) * (d - R1 + R2) * (d + R1 + R2)).max(0.0);
    first + second - 0.5 * root.sqrt()
}

fn placement(d: f64, tol: &Tolerance) -> Option<(Solid, Solid)> {
    let _ = tol;
    let a = PrimitiveBuilder::make_cylinder(R1, HEIGHT).ok()?;
    let b = PrimitiveBuilder::make_cylinder(R2, HEIGHT).ok()?;
    Some((a, BrepTransform::translate_solid(&b, Vec3::new(d, 0.0, 0.0))))
}

fn main() {
    let tol = Tolerance::default();

    println!("曲面どうしが接する寸前・ちょうど・過ぎた後を掃く（4-459）");
    println!();
    println!("**半径 {R1} と {R2} の円柱。軸は平行、高さ {HEIGHT}。**");
    println!("**中心間が {} でちょうど接します。**", R1 + R2);
    println!();
    println!(
        "{:>12}{:>9}{:>15}{:>15}{:>12}{:>13}  {}",
        "中心間", "離れ", "|A∩B|", "閉じた式", "相対差", "恒等式の残差", "3 演算（名=名指し 未=未実装）"
    );
    println!("{}", "-".repeat(100));

    let touch = R1 + R2;
    let mut broken = 0usize;
    let mut refused_while_apart = 0usize;
    // **返ってきたのに閉じた式から外れたもの**（4-462）。ここだけが赤です。
    let mut wrong = 0usize;

    // **近い側（食い込み）を負、遠い側（離れ）を正**に取ります。
    let mut offsets: Vec<f64> = vec![
        -1.0, -0.3, -0.1, -0.03, -0.01, -1e-4, 0.0, 1e-4, 0.01, 0.02, 0.03, 0.05, 0.1, 0.3, 1.0,
    ];
    // **刻みを足す口**（`ZENITH_TANGENT_EXTRA=-0.005,-0.002`）。
    // **門が測る並びは動かしません。**
    if let Ok(text) = std::env::var("ZENITH_TANGENT_EXTRA") {
        for piece in text.split(',') {
            if let Ok(value) = piece.trim().parse::<f64>() {
                offsets.push(value);
            }
        }
        offsets.sort_by(|a, b| a.partial_cmp(b).unwrap());
    }

    // **1 つだけ回す口**（`ZENITH_TANGENT_ONLY=7.999`）。
    //
    // **これが無いと、15 通りぶんの診断が混ざります。** 2026/09/15 に
    // 2 度、混ざった出力を 1 つの置き方のものと読みかけました
    // （`tangent_sweep_probe` には 4-454 で同じ口が付いています）。
    let only: Option<f64> = std::env::var("ZENITH_TANGENT_ONLY")
        .ok()
        .and_then(|text| text.parse().ok());

    for offset in offsets {
        let d = touch + offset;
        if let Some(only) = only {
            if (d - only).abs() > 1e-9 {
                continue;
            }
        }
        let Some((a, b)) = placement(d, &tol) else {
            println!("{d:>12.6}  **立体が作れません**");
            continue;
        };

        let mut volumes: [Option<f64>; 3] = [None; 3];
        let mut per_op = ["和 ○", "積 ○", "差 ○"];
        for (index, op) in [
            BooleanOpType::Union,
            BooleanOpType::Intersection,
            BooleanOpType::Difference,
        ]
        .into_iter()
        .enumerate()
        {
            match BooleanEngine::boolean_solids_exact_result(&a, &b, op, &tol) {
                Ok(result) => {
                    let measured = volume_of(&result.solids);
                    volumes[index] = Some(measured);
                    // **返ってきた演算は、1 つずつ閉じた式に当てます**
                    // （4-462）。
                    //
                    // **恒等式だけでは足りません。** 3 演算のうち 1 つでも
                    // 断られると恒等式が組めず、**残りが正しいかを見る
                    // 物差しが表から消えます**。**実測でそこに誤答が
                    // 隠れていました**——差が「A そのもの」（切り込みの
                    // 無い立体）を返していたのに、**表は「差 ○」**でした。
                    let disc_a = std::f64::consts::PI * R1 * R1 * HEIGHT;
                    let disc_b = std::f64::consts::PI * R2 * R2 * HEIGHT;
                    let lens = lens_area(d) * HEIGHT;
                    let want_op = [disc_a + disc_b - lens, lens, disc_a - lens][index];
                    let off = (measured - want_op).abs() / want_op.abs().max(1.0);
                    if off > 1e-5 {
                        wrong += 1;
                        per_op[index] = ["**和 誤**", "**積 誤**", "**差 誤**"][index];
                        println!(
                            "  **誤答** d={d:.6} {}: 返り {measured:.9}、閉じた式 {want_op:.9}（相対差 {off:.3e}）",
                            ["和", "積", "差"][index]
                        );
                    }
                    // **返った体積も出します**（4-462）。**返ったこと自体は
                    // 合っていることではありません**——3 演算のうち 1 つでも
                    // 断られると恒等式が組めないので、**残りが正しいかを
                    // 見る物差しが、表からは消えます。**
                    if std::env::var_os("ZENITH_TANGENT_WHY").is_some() {
                        let disc_a = std::f64::consts::PI * R1 * R1 * HEIGHT;
                        let disc_b = std::f64::consts::PI * R2 * R2 * HEIGHT;
                        let lens = lens_area(d) * HEIGHT;
                        let want = [disc_a + disc_b - lens, lens, disc_a - lens][index];
                        eprintln!(
                            "TANGENTVOL d={d:.6} {}: 返り {measured:.9}、閉じた式 {want:.9}、差 {:.3e}",
                            ["和", "積", "差"][index],
                            (measured - want).abs()
                        );
                    }
                }
                Err(reason) => {
                    // **断り方で分けます**（4-451 と同じ読み方）。
                    per_op[index] = if reason.contains("not implemented") {
                        ["**和 未**", "**積 未**", "**差 未**"][index]
                    } else if reason.contains("refuses this placement") {
                        ["和 名", "積 名", "差 名"][index]
                    } else {
                        ["和 ×", "積 ×", "差 ×"][index]
                    };
                    // **離れているのに断られたら、それは穴**です。
                    // **接している所（離れ 0）だけは、積が断られて当然**。
                    if offset != 0.0 {
                        refused_while_apart += 1;
                    }
                    if std::env::var_os("ZENITH_TANGENT_WHY").is_some() {
                        eprintln!(
                            "TANGENTWHY d={d:.6} {} — {}",
                            ["和", "積", "差"][index],
                            reason.chars().take(600).collect::<String>()
                        );
                    }
                }
            }
        }

        let want = lens_area(d) * HEIGHT;
        let (shown, residual_text) = match (volumes[0], volumes[1], volumes[2]) {
            (Some(union), Some(meet), Some(minus)) => {
                let va = volume_of(std::slice::from_ref(&a));
                let vb = volume_of(std::slice::from_ref(&b));
                let worst = (((union + meet) - (va + vb)).abs() / (va + vb).max(1.0))
                    .max((minus + meet - va).abs() / va.max(1.0));
                if worst > 1e-6 {
                    broken += 1;
                }
                (Some(meet), format!("{worst:.3e}"))
            }
            (_, Some(meet), _) => (Some(meet), "-".to_string()),
            _ => (None, "-".to_string()),
        };

        match shown {
            Some(got) => {
                let relative = (got - want).abs() / want.abs().max(1.0);
                if relative > 1e-5 {
                    broken += 1;
                }
                println!(
                    "{d:>12.6}{offset:>9.0e}{got:>15.4}{want:>15.4}{relative:>12.1e}{residual_text:>13}  {}",
                    per_op.join(" ")
                );
            }
            None => println!(
                "{d:>12.6}{offset:>9.0e}{:>15}{want:>15.4}{:>12}{:>13}  {}",
                "断り",
                "-",
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
    if broken > 0 {
        println!("**返ってきたのに答えが合わないものが {broken} 件あります。**");
        std::process::exit(1);
    }
    println!("**返ったものは全部、閉じた式と 1e-5 以内で合っています。**");
    println!();
    println!("**断りは誤りではありません**（3-1）。**ここは赤にしません。**");
}
