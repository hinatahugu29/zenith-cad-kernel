//! **円錐どうし・トーラスどうし**を掃く（4-479）。
//!
//! # なぜ要るのか
//!
//! 4-451〜4-478 で、**浅く当たる所**を 7 つの組で掃きました。
//! **残っているのは、円錐どうしとトーラスどうし**でした。
//! **種類の違う組**（円錐 × 円柱、円錐 × 球、トーラス × 円柱、
//! トーラス × 球、円柱 × 球）も、ここで掃きます（4-483）。
//!
//! **どちらも閉じた式がありません**（一般の置き方では）。
//! **だから測れない、ではありません**——**恒等式があります**。
//!
//! ```text
//! |A| + |B| = |A∪B| + |A∩B|
//! ```
//!
//! **3 演算が返ったときだけ**当てられる、**自分で自分を測る**式です。
//! **「返ったこと」だけでは足りません**（4-462 の誤答は返っていました）。
//! **もうひとつ当てます**——**差**は `|A| − |A∩B|` でなければ
//! なりません。**2 本の式に同時に嘘をつくのは、難しい**。
//!
//! # 読み方
//!
//! **断りは誤りではありません**（3-1）。**返ってきたのに 2 本の式に
//! 合わないもの**だけを赤にします。
//!
//! # つまみ
//!
//! * `ZENITH_PAIRID_WHY=1` — 断り文を出します
//! * `ZENITH_PAIRID_ONLY=<名前の一部>` — その置き方だけ測ります
//!
//! # 使い方
//!
//! ```bash
//! cargo run --release -p zenith_algo --example curved_pair_identity_probe
//! ```

use zenith_algo::{BooleanEngine, BooleanOpType, BrepTransform, MassCalculator, PrimitiveBuilder};
use zenith_math::{Tolerance, Transform3, Vec3};
use zenith_tess::TessellationParams;
use zenith_topo::Solid;

fn volume_of(solids: &[Solid]) -> f64 {
    solids
        .iter()
        .map(|solid| MassCalculator::compute_from_brep(solid, &TessellationParams::default()).volume)
        .sum()
}

fn run_ops(a: &Solid, b: &Solid, tol: &Tolerance) -> [Result<f64, String>; 3] {
    [
        BooleanOpType::Union,
        BooleanOpType::Intersection,
        BooleanOpType::Difference,
    ]
    .map(|op| {
        BooleanEngine::boolean_solids_exact_result(a, b, op, tol)
            .map(|outcome| volume_of(&outcome.solids))
    })
}

fn mark(result: &Result<f64, String>, index: usize, bad: bool) -> &'static str {
    match result {
        Ok(_) if bad => ["**和 誤**", "**積 誤**", "**差 誤**"][index],
        Ok(_) => ["和 ○", "積 ○", "差 ○"][index],
        Err(reason) if reason.contains("not implemented") => {
            ["**和 未**", "**積 未**", "**差 未**"][index]
        }
        Err(reason) if reason.contains("refuses this placement") => {
            ["和 名", "積 名", "差 名"][index]
        }
        Err(_) => ["和 x", "積 x", "差 x"][index],
    }
}

/// **1 つの置き方を測る。** 赤になるのは**返ったのに式に合わないとき**だけ。
fn measure(
    label: &str,
    a: &Solid,
    b: &Solid,
    tol: &Tolerance,
    wrong: &mut usize,
    refused: &mut usize,
) {
    // **1 つだけ見たいとき**の口（`ZENITH_PAIRID_ONLY=<名前の一部>`）。
    // 診断を出しながら 21 通り全部を回すと、読むほうが追えません。
    if let Ok(needle) = std::env::var("ZENITH_PAIRID_ONLY") {
        if !label.contains(needle.trim()) {
            return;
        }
    }
    let volume_a = volume_of(std::slice::from_ref(a));
    let volume_b = volume_of(std::slice::from_ref(b));
    let results = run_ops(a, b, tol);
    let scale = volume_a.abs().max(volume_b.abs()).max(1.0);

    let mut sum_residual: Option<f64> = None;
    let mut difference_residual: Option<f64> = None;
    if let (Ok(union), Ok(lens)) = (&results[0], &results[1]) {
        sum_residual = Some((volume_a + volume_b - union - lens).abs() / scale);
    }
    if let (Ok(lens), Ok(difference)) = (&results[1], &results[2]) {
        difference_residual = Some((volume_a - lens - difference).abs() / scale);
    }
    let bad = sum_residual.is_some_and(|value| value > 1e-6)
        || difference_residual.is_some_and(|value| value > 1e-6);
    if bad {
        *wrong += 1;
    }

    let mut marks = ["", "", ""];
    for index in 0..3 {
        if results[index].is_err() {
            *refused += 1;
            if std::env::var_os("ZENITH_PAIRID_WHY").is_some() {
                if let Err(reason) = &results[index] {
                    eprintln!(
                        "PAIRIDWHY {label} {} — {}",
                        ["和", "積", "差"][index],
                        reason.chars().take(500).collect::<String>()
                    );
                }
            }
        }
        marks[index] = mark(&results[index], index, bad && results[index].is_ok());
    }

    let show = |value: Option<f64>| match value {
        Some(residual) => format!("{residual:.1e}"),
        None => "-".to_string(),
    };
    let lens = match &results[1] {
        Ok(value) => format!("{value:.4}"),
        Err(_) => "断り".to_string(),
    };
    println!(
        "{label:<34}{lens:>13}{:>13}{:>13}  {}",
        show(sum_residual),
        show(difference_residual),
        marks.join(" ")
    );
}

fn main() {
    let tol = Tolerance::default();
    let mut wrong = 0usize;
    let mut refused = 0usize;

    println!("円錐どうし・トーラスどうしを掃く（4-479）");
    println!();
    println!("**閉じた式がないので、2 本の恒等式で測ります**——");
    println!("`|A| + |B| = |A∪B| + |A∩B|` と `|A| − |A∩B| = |A−B|`。");
    println!();
    println!(
        "{:<34}{:>13}{:>13}{:>13}  {}",
        "置き方", "|A∩B|", "和の残差", "差の残差", "3 演算"
    );
    println!("{}", "-".repeat(104));

    // ---- 円錐どうし ----
    let cone = || PrimitiveBuilder::make_cone(5.0, 0.0, 10.0).expect("円錐");
    for shift in [2.0_f64, 5.0, 8.0, 9.9, 10.0] {
        let a = cone();
        let b = BrepTransform::translate_solid(&cone(), Vec3::new(shift, 0.0, 0.0));
        measure(
            &format!("円錐 × 円錐 横に {shift}"),
            &a,
            &b,
            &tol,
            &mut wrong,
            &mut refused,
        );
    }
    for lift in [1.0_f64, 5.0, 9.0, 9.99] {
        let a = cone();
        let b = BrepTransform::translate_solid(&cone(), Vec3::new(0.0, 0.0, lift));
        measure(
            &format!("円錐 × 円錐 同軸で {lift} 上"),
            &a,
            &b,
            &tol,
            &mut wrong,
            &mut refused,
        );
    }
    let a = cone();
    let flip = Transform3::from_axis_angle(&Vec3::y(), std::f64::consts::PI);
    let flipped = BrepTransform::transform_solid(&cone(), &flip).expect("裏返す");
    for lift in [5.0_f64, 9.0, 10.0] {
        let b = BrepTransform::translate_solid(&flipped, Vec3::new(0.0, 0.0, lift));
        measure(
            &format!("円錐 × 逆さ円錐 {lift} 上"),
            &a,
            &b,
            &tol,
            &mut wrong,
            &mut refused,
        );
    }

    // ---- トーラスどうし ----
    let torus = || PrimitiveBuilder::make_torus(6.0, 2.0).expect("トーラス");
    for shift in [2.0_f64, 6.0, 12.0, 15.9, 16.0] {
        let a = torus();
        let b = BrepTransform::translate_solid(&torus(), Vec3::new(shift, 0.0, 0.0));
        measure(
            &format!("トーラス × トーラス 横に {shift}"),
            &a,
            &b,
            &tol,
            &mut wrong,
            &mut refused,
        );
    }
    for lift in [1.0_f64, 2.0, 3.9, 4.0] {
        let a = torus();
        let b = BrepTransform::translate_solid(&torus(), Vec3::new(0.0, 0.0, lift));
        measure(
            &format!("トーラス × トーラス 上に {lift}"),
            &a,
            &b,
            &tol,
            &mut wrong,
            &mut refused,
        );
    }

    // ---- 種類の違う曲面どうし（4-483）----
    //
    // **同じ種類どうしは掃きました**（円錐 × 円錐、トーラス × トーラス）。
    // **種類が違う組は、掃いていません。**
    let cylinder = || PrimitiveBuilder::make_cylinder(3.0, 10.0).expect("円柱");
    let sphere = || PrimitiveBuilder::make_sphere(4.0).expect("球");

    for shift in [0.0_f64, 2.0, 4.0, 7.9] {
        let a = cone();
        let b = BrepTransform::translate_solid(&cylinder(), Vec3::new(shift, 0.0, 2.0));
        measure(
            &format!("円錐 × 円柱 横に {shift}"),
            &a,
            &b,
            &tol,
            &mut wrong,
            &mut refused,
        );
    }
    for shift in [0.0_f64, 3.0, 6.0, 8.9] {
        let a = cone();
        let b = BrepTransform::translate_solid(&sphere(), Vec3::new(shift, 0.0, 5.0));
        measure(
            &format!("円錐 × 球 横に {shift}"),
            &a,
            &b,
            &tol,
            &mut wrong,
            &mut refused,
        );
    }
    for shift in [0.0_f64, 4.0, 6.0, 10.9] {
        let a = torus();
        let b = BrepTransform::translate_solid(&cylinder(), Vec3::new(shift, 0.0, -5.0));
        measure(
            &format!("トーラス × 円柱 横に {shift}"),
            &a,
            &b,
            &tol,
            &mut wrong,
            &mut refused,
        );
    }
    for shift in [0.0_f64, 6.0, 9.0, 11.9] {
        let a = torus();
        let b = BrepTransform::translate_solid(&sphere(), Vec3::new(shift, 0.0, 0.0));
        measure(
            &format!("トーラス × 球 横に {shift}"),
            &a,
            &b,
            &tol,
            &mut wrong,
            &mut refused,
        );
    }
    for shift in [0.0_f64, 3.0, 6.0, 6.9] {
        let a = cylinder();
        let b = BrepTransform::translate_solid(&sphere(), Vec3::new(shift, 0.0, 5.0));
        measure(
            &format!("円柱 × 球 横に {shift}"),
            &a,
            &b,
            &tol,
            &mut wrong,
            &mut refused,
        );
    }

    println!();
    println!("**断りの数: {refused} 件**（**断りは誤りではありません**——3-1）");
    println!();
    if wrong > 0 {
        println!("**返ってきたのに恒等式から外れた置き方が {wrong} 件あります。**");
        std::process::exit(1);
    }
    println!("**返ったものは全部、2 本の恒等式と 1e-6 以内で合っています。**");

    // **断りが増えたら、赤にします**（4-483）。
    //
    // **断りは誤りではありません**（3-1）。**それでも、いままで返って
    // いたものが返らなくなったら、それは失ったもの**です。
    //
    // **実際に、静かに失いました**——4-478 と 4-482 の直しで、
    // **トーラスどうしの「横に 2」と「横に 6」が返らなくなって**いて、
    // **門は緑のまま**でした（この門は**誤答だけ**を赤にしていたので）。
    // **数えるようにします。**
    //
    // **増やすときは、この数を測って書き換えてください**——
    // **下げるのは歓迎、上げるのは「何を失ったか」を書いてから**。
    const REFUSALS_MEASURED: usize = 7;
    if refused > REFUSALS_MEASURED {
        println!();
        println!(
            "**断りが {refused} 件に増えました**（測ってあるのは {REFUSALS_MEASURED} 件）。             **返っていたものが返らなくなっています。**"
        );
        std::process::exit(1);
    }
    if refused < REFUSALS_MEASURED {
        println!();
        println!(
            "**断りが {refused} 件に減りました**（測ってあるのは {REFUSALS_MEASURED} 件）。             **`REFUSALS_MEASURED` を {refused} に書き換えてください。**"
        );
    }
}
