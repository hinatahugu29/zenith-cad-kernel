//! **軸が平行でない円柱どうし**を掃く（4-478）。
//!
//! # なぜ要るのか
//!
//! 4-451〜4-477 で、**浅く当たる所**を 6 つの組で閉じました——
//! **平面 × 円柱**、**円柱 × 円柱（平行）**、**球 × 球**、
//! **平面 × 球**、**平面 × 円錐**、**平面 × トーラス**。
//!
//! **軸が平行でない円柱**は掃いていません。**平行な組とは別物**です
//! ——交線が**楕円でも円でもない曲線**になり、**角度が浅くなるほど
//! 細長く伸びます**。
//!
//! # 何を測るか（2 つ）
//!
//! ## 1. 角度を振る——**閉じた式があります**
//!
//! **同じ半径 `r` の円柱 2 本の軸が、角度 θ で交わる**とき、
//! 重なりの体積は **`16 r³ / (3 sin θ)`** です（θ = 90° で
//! シュタインメッツの `16r³/3`）。
//!
//! **円柱は有限**なので、**重なりが端からはみ出さない長さ**が要ります。
//!
//! ## 2. 軸をずらす——**恒等式で測ります**
//!
//! **直交する 2 本の軸を、共通垂線の向きにずらして**いき、
//! **`d = r1 + r2` で接する**所まで詰めます。**ここには閉じた式が
//! ありません**ので、**`|A| + |B| = |A∪B| + |A∩B|`** を使います。
//!
//! # 読み方
//!
//! **断りは誤りではありません**（3-1）。**返ってきたのに合わないもの**
//! だけを赤にします。
//!
//! # 使い方
//!
//! ```bash
//! cargo run --release -p zenith_algo --example cross_cylinder_sweep_probe
//! ```

use zenith_algo::{BooleanEngine, BooleanOpType, BrepTransform, MassCalculator, PrimitiveBuilder};
use zenith_math::{Tolerance, Transform3, Vec3};
use zenith_tess::TessellationParams;
use zenith_topo::Solid;

const R: f64 = 2.0;
/// **長さ**。重なりが端からはみ出さないだけ取ります（）。
fn length() -> f64 {
    std::env::var("ZENITH_CROSS_LENGTH")
        .ok()
        .and_then(|text| text.parse().ok())
        .unwrap_or(160.0)
}

fn volume_of(solids: &[Solid]) -> f64 {
    solids
        .iter()
        .map(|solid| MassCalculator::compute_from_brep(solid, &TessellationParams::default()).volume)
        .sum()
}

/// **立っている円柱を、中心を原点に合わせて寝かせる**。
/// `angle` は x 軸から測った、xy 平面内の向き。
fn lying_cylinder(radius: f64, angle: f64, offset: Vec3) -> Solid {
    let upright = PrimitiveBuilder::make_cylinder(radius, length()).expect("円柱");
    let centred = BrepTransform::translate_solid(&upright, Vec3::new(0.0, 0.0, -length() * 0.5));
    let lay = Transform3::from_axis_angle(&Vec3::y(), std::f64::consts::FRAC_PI_2);
    let turn = Transform3::from_axis_angle(&Vec3::z(), angle);
    let placed = BrepTransform::transform_solid(&centred, &turn.compose(&lay)).expect("回す");
    BrepTransform::translate_solid(&placed, offset)
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

fn why(tag: &str, label: String, results: &[Result<f64, String>; 3]) {
    if std::env::var_os("ZENITH_CROSS_WHY").is_none() {
        return;
    }
    for (index, result) in results.iter().enumerate() {
        if let Err(reason) = result {
            eprintln!(
                "{tag} {label} {} — {}",
                ["和", "積", "差"][index],
                reason.chars().take(600).collect::<String>()
            );
        }
    }
}

fn main() {
    let tol = Tolerance::default();
    let cylinder = std::f64::consts::PI * R * R * length();
    let mut wrong = 0usize;
    let mut refused = 0usize;

    println!("軸が平行でない円柱どうしを掃く（4-478）");
    println!();
    println!("## 1. 角度を振る（半径 {R}、軸は交わる。閉じた式 16r³/(3 sin θ)）");
    println!();
    println!(
        "{:>8}{:>15}{:>15}{:>12}  {}",
        "角度", "|A∩B|", "閉じた式", "相対差", "3 演算"
    );
    println!("{}", "-".repeat(72));

    let mut angles: Vec<f64> = vec![90.0, 75.0, 60.0, 45.0, 30.0, 20.0, 15.0, 10.0];
    if let Ok(text) = std::env::var("ZENITH_CROSS_ANGLES") {
        angles = text
            .split(',')
            .filter_map(|piece| piece.trim().parse().ok())
            .collect();
    }
    for degrees in angles {
        let theta: f64 = degrees.to_radians();
        let a = lying_cylinder(R, 0.0, Vec3::zeros());
        let b = lying_cylinder(R, theta, Vec3::zeros());
        let want_lens = 16.0 * R.powi(3) / (3.0 * theta.sin());

        let results = run_ops(&a, &b, &tol);
        why("CROSSWHY 角度", format!("{degrees}deg"), &results);
        let wants = [2.0 * cylinder - want_lens, want_lens, cylinder - want_lens];
        let mut marks = ["", "", ""];
        for index in 0..3 {
            let bad = match &results[index] {
                Ok(got) => (got - wants[index]).abs() / wants[index].abs().max(1.0) > 1e-4,
                Err(_) => false,
            };
            if bad {
                wrong += 1;
            }
            if results[index].is_err() {
                refused += 1;
            }
            marks[index] = mark(&results[index], index, bad);
        }
        match &results[1] {
            Ok(got) => println!(
                "{degrees:>7}o{got:>15.4}{want_lens:>15.4}{:>12.1e}  {}",
                (got - want_lens).abs() / want_lens,
                marks.join(" ")
            ),
            Err(_) => println!(
                "{degrees:>7}o{:>15}{want_lens:>15.4}{:>12}  {}",
                "断り",
                "-",
                marks.join(" ")
            ),
        }
    }

    println!();
    println!(
        "## 2. 直交したまま軸をずらす（半径 {R} と {R}。接するのは d = {}）",
        2.0 * R
    );
    println!();
    println!("**閉じた式がないので、恒等式 |A| + |B| = |A∪B| + |A∩B| で測ります。**");
    println!();
    println!(
        "{:>12}{:>9}{:>15}{:>13}  {}",
        "軸の隔たり", "食い込み", "|A∩B|", "恒等式の残差", "3 演算"
    );
    println!("{}", "-".repeat(76));

    let mut overlaps: Vec<f64> = vec![-0.1, -1e-3, 0.0, 1e-4, 1e-3, 1e-2, 0.1, 0.5, 1.0, 2.0, 4.0];
    if let Ok(text) = std::env::var("ZENITH_CROSS_EXTRA") {
        for piece in text.split(',') {
            if let Ok(value) = piece.trim().parse::<f64>() {
                overlaps.push(value);
            }
        }
        overlaps.sort_by(|a, b| a.partial_cmp(b).unwrap());
    }
    for overlap in overlaps {
        let d = 2.0 * R - overlap;
        let a = lying_cylinder(R, 0.0, Vec3::zeros());
        let b = lying_cylinder(R, std::f64::consts::FRAC_PI_2, Vec3::new(0.0, 0.0, d));

        let results = run_ops(&a, &b, &tol);
        why("CROSSWHY 隔たり", format!("{d:.6}"), &results);
        let mut marks = ["", "", ""];
        let mut residual: Option<f64> = None;
        if let (Ok(union), Ok(lens)) = (&results[0], &results[1]) {
            residual = Some((2.0 * cylinder - union - lens).abs() / cylinder);
        }
        let bad = residual.is_some_and(|value| value > 1e-6);
        for index in 0..3 {
            if results[index].is_err() {
                refused += 1;
            }
            marks[index] = mark(&results[index], index, bad && results[index].is_ok());
        }
        if bad {
            wrong += 1;
        }
        match (&results[1], residual) {
            (Ok(lens), Some(value)) => println!(
                "{d:>12.6}{overlap:>9.0e}{lens:>15.6}{value:>13.1e}  {}",
                marks.join(" ")
            ),
            (Ok(lens), None) => println!(
                "{d:>12.6}{overlap:>9.0e}{lens:>15.6}{:>13}  {}",
                "-",
                marks.join(" ")
            ),
            _ => println!(
                "{d:>12.6}{overlap:>9.0e}{:>15}{:>13}  {}",
                "断り",
                "-",
                marks.join(" ")
            ),
        }
    }

    println!();
    println!("**断りの数: {refused} 件**（接する所の断りも含みます。**断りは誤りではありません**）");
    println!();
    if wrong > 0 {
        println!("**返ってきたのに合わない所が {wrong} 件あります。**");
        std::process::exit(1);
    }
    println!("**返ったものは全部、閉じた式または恒等式と合っています。**");
}
