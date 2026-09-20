//! **細長さ**を振って掃く（4-485）。
//!
//! # なぜこの軸か
//!
//! `scale_sweep_probe` は**一様に大きく／小さく**します（4-211）。
//! **形は変わりません。** ところが 4-478 の誤答は、**形を細長くした
//! ところ**で出ました——**直交する円柱 2 本で、長さが半径の 17 倍を
//! 超えると積が空で返る**。**一様な拡大では、一生出ません。**
//!
//! **細長さは、格子の目と形の関係を変えます。** パラメータ空間の格子は
//! 一様なので、**長い面では目が粗く**なります。**そこが今日の根**でした。
//!
//! # 何を測るか
//!
//! **3 つの族**を、細長さを変えながら。
//!
//! 1. **管の細いトーラス** × 箱（`r_minor` を 2 → 0.02 まで）
//! 2. **尖った円錐** × 箱（高さ / 底半径 を 2 → 200 まで）
//! 3. **長い円柱** × 球（長さ / 半径 を 4 → 400 まで）
//!
//! **当てるものは 2 つ**。
//!
//! * **素の体積が閉じた式に乗るか**（`2π²Rr²`、`πr²h/3`、`πr²h`）
//!   ——**細長い形は、メッシュの刻みが足りないと体積から狂います**
//! * **恒等式 2 本**（`|A|+|B| = |A∪B|+|A∩B|`、`|A|−|A∩B| = |A−B|`）
//!
//! **断りは誤りではありません**（3-1）。**返ってきたのに合わないもの**
//! だけを赤にします。
//!
//! # つまみ
//!
//! * `ZENITH_ASPECT_ONLY=<名前の一部>` — その行だけ測ります
//! * `ZENITH_ASPECT_WHY=1` — 断り文を出します
//!
//! ```bash
//! cargo run --release -p zenith_algo --example aspect_sweep_probe
//! ```

use zenith_algo::{BooleanEngine, BooleanOpType, BrepTransform, MassCalculator, PrimitiveBuilder};
use zenith_math::{Tolerance, Vec3};
use zenith_tess::TessellationParams;
use zenith_topo::Solid;

fn volume_of(solids: &[Solid]) -> f64 {
    solids
        .iter()
        .map(|solid| MassCalculator::compute_from_brep(solid, &TessellationParams::default()).volume)
        .sum()
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

/// **1 行ぶんを測る。** `closed` は A の素の体積の閉じた式。
fn measure(
    label: &str,
    a: &Solid,
    b: &Solid,
    closed: f64,
    tol: &Tolerance,
    wrong: &mut usize,
    refused: &mut usize,
) {
    if let Ok(needle) = std::env::var("ZENITH_ASPECT_ONLY") {
        if !label.contains(needle.trim()) {
            return;
        }
    }
    let volume_a = volume_of(std::slice::from_ref(a));
    let volume_b = volume_of(std::slice::from_ref(b));
    let body = (volume_a - closed).abs() / closed.abs().max(1.0);

    let results = [
        BooleanOpType::Union,
        BooleanOpType::Intersection,
        BooleanOpType::Difference,
    ]
    .map(|op| {
        BooleanEngine::boolean_solids_exact_result(a, b, op, tol)
            .map(|outcome| volume_of(&outcome.solids))
    });

    let scale = volume_a.abs().max(volume_b.abs()).max(1.0);
    let mut sum_residual: Option<f64> = None;
    let mut difference_residual: Option<f64> = None;
    if let (Ok(union), Ok(lens)) = (&results[0], &results[1]) {
        sum_residual = Some((volume_a + volume_b - union - lens).abs() / scale);
    }
    if let (Ok(lens), Ok(difference)) = (&results[1], &results[2]) {
        difference_residual = Some((volume_a - lens - difference).abs() / scale);
    }
    let bad_identity = sum_residual.is_some_and(|value| value > 1e-6)
        || difference_residual.is_some_and(|value| value > 1e-6);
    // **素の体積は、メッシュの刻みで決まります。** 細長い形ほど弦が
    // 粗くなるので、**恒等式より緩い線**（1e-3）で見ます。**ここは
    // 「ブーリアンが壊れた」ではなく「刻みが足りない」**の合図です。
    let bad_body = body > 1e-3;
    if bad_identity || bad_body {
        *wrong += 1;
    }

    let mut marks = ["", "", ""];
    for index in 0..3 {
        if results[index].is_err() {
            *refused += 1;
            if std::env::var_os("ZENITH_ASPECT_WHY").is_some() {
                if let Err(reason) = &results[index] {
                    eprintln!(
                        "ASPECTWHY {label} {} — {}",
                        ["和", "積", "差"][index],
                        reason.chars().take(400).collect::<String>()
                    );
                }
            }
        }
        marks[index] = mark(&results[index], index, bad_identity && results[index].is_ok());
    }

    let show = |value: Option<f64>| match value {
        Some(residual) => format!("{residual:.1e}"),
        None => "-".to_string(),
    };
    println!(
        "{label:<30}{:>12}{:>12}{:>12}  {}",
        format!("{body:.1e}"),
        show(sum_residual),
        show(difference_residual),
        marks.join(" ")
    );
}

fn main() {
    let tol = Tolerance::default();
    let mut wrong = 0usize;
    let mut refused = 0usize;

    println!("細長さを振って掃く（4-485）");
    println!();
    println!("**一様な拡大（`scale_sweep_probe`）では出ない所**を見ます。");
    println!("**4-478 の誤答は、長さ / 半径が 17 倍を超えた所**で出ました。");
    println!();
    println!(
        "{:<30}{:>12}{:>12}{:>12}  {}",
        "形", "素の体積", "和の残差", "差の残差", "3 演算"
    );
    println!("{}", "-".repeat(92));

    // 1. 管の細いトーラス × 箱（上半分を切る）
    for minor in [2.0_f64, 0.5, 0.1, 0.02] {
        let major = 6.0;
        let a = PrimitiveBuilder::make_torus(major, minor).expect("トーラス");
        let b = BrepTransform::translate_solid(
            &PrimitiveBuilder::make_box(40.0, 40.0, 40.0).expect("箱"),
            Vec3::new(-20.0, -20.0, 0.0),
        );
        let closed = 2.0 * std::f64::consts::PI.powi(2) * major * minor * minor;
        measure(
            &format!("トーラス 管 {minor} × 箱"),
            &a,
            &b,
            closed,
            &tol,
            &mut wrong,
            &mut refused,
        );
    }

    // 2. 尖った円錐 × 箱（横から切る）
    for height in [10.0_f64, 50.0, 200.0, 1000.0] {
        let radius = 5.0;
        let a = PrimitiveBuilder::make_cone(radius, 0.0, height).expect("円錐");
        let b = BrepTransform::translate_solid(
            &PrimitiveBuilder::make_box(40.0, 40.0, height * 4.0).expect("箱"),
            Vec3::new(2.0, -20.0, -height),
        );
        let closed = std::f64::consts::PI * radius * radius * height / 3.0;
        measure(
            &format!("円錐 高さ {height} × 箱"),
            &a,
            &b,
            closed,
            &tol,
            &mut wrong,
            &mut refused,
        );
    }

    // 3. 長い円柱 × 球（真ん中で重ねる）
    for length in [20.0_f64, 100.0, 400.0, 2000.0] {
        let radius = 5.0;
        let a = BrepTransform::translate_solid(
            &PrimitiveBuilder::make_cylinder(radius, length).expect("円柱"),
            Vec3::new(0.0, 0.0, -length * 0.5),
        );
        let b = PrimitiveBuilder::make_sphere(7.0).expect("球");
        let closed = std::f64::consts::PI * radius * radius * length;
        measure(
            &format!("円柱 長さ {length} × 球"),
            &a,
            &b,
            closed,
            &tol,
            &mut wrong,
            &mut refused,
        );
    }

    println!();
    println!("**断りの数: {refused} 件**（**断りは誤りではありません**——3-1）");
    println!();
    if wrong > 0 {
        println!("**合わない行が {wrong} 件あります。**");
        std::process::exit(1);
    }
    println!("**素の体積は閉じた式と 1e-3 以内、恒等式は 1e-6 以内で合っています。**");

    // **断りが増えたら赤にします**（4-483 の教訓）。
    //
    // **断りは誤りではありません**（3-1）。**それでも、返っていたものが
    // 返らなくなったら、それは失ったもの**です。**誤答だけを見ていると、
    // 静かに失います**——実際に失いました（4-483）。
    //
    // **いまの 3 件は、尖った円錐 1 つ**（高さ 1000。3 演算）。
    // **6 件から減りました**（4-498）——**高さ 200 が 3 演算とも返ります**
    // （恒等式 1.4e-15）。**効いたのは 2 つ組**です:
    // **切れた枝が出たときだけ、許容を場面の大きさに乗せて辿り直す**のと、
    // **p-curve の受け入れ幅も場面の大きさに乗せる**の。
    // **片方だけでは 1 件も動きません**（4-498 で両方向とも測りました）。
    //
    // **高さ 1000 は、まだ断ります。** **減らせたら、この数を書き換えて
    // ください。**
    const REFUSALS_MEASURED: usize = 3;
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
