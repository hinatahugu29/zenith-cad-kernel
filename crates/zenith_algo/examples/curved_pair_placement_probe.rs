//! **曲面どうしの組を、動かしても同じ答えか**（4-486）。
//!
//! # なぜ要るのか
//!
//! `curved_placement_probe` は**円柱どうし**で、これを測っています
//! （剛体移動で答えが変わらないこと）。**円錐・トーラス・球の組は
//! 見ていません**——4-479 と 4-483 で掃いたばかりの所です。
//!
//! **閉じた式は要りません。** **同じ形を動かしただけ**なら、
//! **体積は 1 ビットも変わらないはず**です。**変われば、どこかが
//! 世界座標に依っています**（種の格子、継ぎ目の位置、投影の向き）。
//!
//! # 何を測るか
//!
//! **4 つの組**を、**原点に置いたとき**と、**回して動かしたとき**で。
//!
//! * 円錐 × 円錐（横にずらして重ねる）
//! * トーラス × トーラス（上にずらして重ねる）
//! * トーラス × 球
//! * 円錐 × 円柱
//!
//! **動かし方**は、**軸に平行でない向きへ 3 回転 + 平行移動**です
//! （`(1,2,3)` まわりに 37 度、`(0,1,0)` まわりに 23 度、原点から遠くへ）。
//!
//! **当てるもの**: **回すだけなら相対 1e-7、動かしたら 1e-5**（実測の最悪は
//! 1.3e-8 と 1.5e-6 で、どちらも 10 倍の余裕を取っています）。
//! **線が違うのは、体積積分が世界原点を基準に足しているから**です
//! （実測: 距離 ×0.1 / ×1 / ×10 で 1.5e-7 / 1.5e-6 / 1.5e-5 と**比例**）。
//! **ブーリアンの精度ではありません。**
//! **断りは誤りではありません**（3-1）——**片方だけが断ったら赤**に
//! します（**同じ形なのに、置き場所で通ったり通らなかったりする**のは、
//! 誤りではないにせよ、**知っておくべきこと**なので数えます）。
//!
//! # つまみ
//!
//! * `ZENITH_PLACE_ONLY=<名前の一部>` — その組だけ測ります
//! * `ZENITH_PLACE_WHY=1` — 断り文を出します
//!
//! ```bash
//! cargo run --release -p zenith_algo --example curved_pair_placement_probe
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

/// **軸に平行でない向きへ回すだけ。** 原点からは動かしません。
fn turned(solid: &Solid) -> Solid {
    let spin =
        Transform3::from_axis_angle(&Vec3::new(1.0, 2.0, 3.0).normalize(), 37f64.to_radians());
    let tilt = Transform3::from_axis_angle(&Vec3::y(), 23f64.to_radians());
    BrepTransform::transform_solid(solid, &tilt.compose(&spin)).expect("回す")
}

/// **回してから、原点から遠くへ。** 距離は `ZENITH_PLACE_FAR` 倍。
fn moved(solid: &Solid) -> Solid {
    let far: f64 = std::env::var("ZENITH_PLACE_FAR")
        .ok()
        .and_then(|text| text.parse().ok())
        .unwrap_or(1.0);
    BrepTransform::translate_solid(&turned(solid), Vec3::new(37.0 * far, -19.0 * far, 53.0 * far))
}

fn three_ops(a: &Solid, b: &Solid, tol: &Tolerance) -> [Result<f64, String>; 3] {
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

fn measure(label: &str, a: &Solid, b: &Solid, tol: &Tolerance, wrong: &mut usize, split: &mut usize) {
    if let Ok(needle) = std::env::var("ZENITH_PLACE_ONLY") {
        if !label.contains(needle.trim()) {
            return;
        }
    }
    let here = three_ops(a, b, tol);
    let spun = three_ops(&turned(a), &turned(b), tol);
    let there = three_ops(&moved(a), &moved(b), tol);

    let mut marks = ["", "", ""];
    let mut worst_spin = 0.0f64;
    let mut worst_move = 0.0f64;
    for index in 0..3 {
        let name = ["和", "積", "差"][index];
        // **回すだけ**は、きつい線で見ます。**積分の基準は動きません。**
        if let (Ok(before), Ok(after)) = (&here[index], &spun[index]) {
            let relative = (after - before).abs() / before.abs().max(1.0);
            worst_spin = worst_spin.max(relative);
            // **線は 1e-7**——**測った最悪（トーラス × 球の 1.3e-8）の
            // 10 倍**に置いています。**ぴったりに張ると、揺れで赤になります。**
            if relative > 1e-7 {
                *wrong += 1;
            }
        }
        match (&here[index], &there[index]) {
            (Ok(before), Ok(after)) => {
                let scale = before.abs().max(1.0);
                let relative = (after - before).abs() / scale;
                worst_move = worst_move.max(relative);
                // **動かすと、誤差は原点からの距離に比例して増えます**
                // （4-486 実測: 距離 0 / ×0.1 / ×1 / ×10 で
                // 9.9e-11 / 1.5e-7 / 1.5e-6 / 1.5e-5）。
                // **ブーリアンではなく、体積積分が世界原点を基準に
                // 足している**ためです（`curved_placement_probe` の
                // 頭に同じことが書いてあります）。**線はそこに合わせます。**
                marks[index] = if relative > 1e-5 {
                    *wrong += 1;
                    ["**和 誤**", "**積 誤**", "**差 誤**"][index]
                } else {
                    ["和 ○", "積 ○", "差 ○"][index]
                };
            }
            (Err(_), Err(_)) => {
                marks[index] = ["和 断", "積 断", "差 断"][index];
            }
            (Ok(_), Err(reason)) | (Err(reason), Ok(_)) => {
                *split += 1;
                marks[index] = ["**和 片**", "**積 片**", "**差 片**"][index];
                if std::env::var_os("ZENITH_PLACE_WHY").is_some() {
                    eprintln!(
                        "PLACEWHY {label} {name}: 片方だけ断る — {}",
                        reason.chars().take(300).collect::<String>()
                    );
                }
            }
        }
    }
    println!(
        "{label:<28}{:>12.1e}{:>12.1e}  {}",
        worst_spin,
        worst_move,
        marks.join(" ")
    );
}

fn main() {
    let tol = Tolerance::default();
    let mut wrong = 0usize;
    let mut split = 0usize;

    println!("曲面どうしの組を、動かしても同じ答えか（4-486）");
    println!();
    println!("**同じ形を回して動かしただけ**なら、**体積は変わらないはず**です。");
    println!("**閉じた式は要りません**——**自分自身が物差し**になります。");
    println!();
    println!(
        "{:<28}{:>12}{:>12}  {}",
        "組", "回すだけ", "動かす", "3 演算（片＝片方だけ断る）"
    );
    println!("{}", "-".repeat(80));

    let cone = || PrimitiveBuilder::make_cone(5.0, 0.0, 10.0).expect("円錐");
    let torus = || PrimitiveBuilder::make_torus(6.0, 2.0).expect("トーラス");

    let a = cone();
    let b = BrepTransform::translate_solid(&cone(), Vec3::new(5.0, 0.0, 0.0));
    measure("円錐 × 円錐", &a, &b, &tol, &mut wrong, &mut split);

    let a = torus();
    let b = BrepTransform::translate_solid(&torus(), Vec3::new(0.0, 0.0, 2.0));
    measure("トーラス × トーラス", &a, &b, &tol, &mut wrong, &mut split);

    let a = torus();
    let b = BrepTransform::translate_solid(
        &PrimitiveBuilder::make_sphere(4.0).expect("球"),
        Vec3::new(6.0, 0.0, 0.0),
    );
    measure("トーラス × 球", &a, &b, &tol, &mut wrong, &mut split);

    let a = cone();
    let b = BrepTransform::translate_solid(
        &PrimitiveBuilder::make_cylinder(3.0, 10.0).expect("円柱"),
        Vec3::new(2.0, 0.0, 2.0),
    );
    measure("円錐 × 円柱", &a, &b, &tol, &mut wrong, &mut split);

    println!();
    println!("**片方だけ断った演算: {split} 件**");
    println!();
    if wrong > 0 {
        println!("**動かしたら答えが変わった演算が {wrong} 件あります。**");
        std::process::exit(1);
    }
    if split > 0 {
        println!("**答えが変わったものはありません。**");
        println!("**ただし、片方だけ断るものが {split} 件あります**——");
        println!("**誤りではありませんが、置き場所で通ったり通らなかったりします。**");
        std::process::exit(1);
    }
    println!("**回すだけなら相対 1e-7 以内、動かしても 1e-5 以内で同じです。**");
    println!();
    println!("**動かすと誤差が増えるのは、体積積分が世界原点を基準に足しているから**");
    println!("です（4-486 実測: 距離 ×0.1 / ×1 / ×10 で 1.5e-7 / 1.5e-6 / 1.5e-5——**比例**）。");
    println!("**ブーリアンの精度ではありません。** `ZENITH_PLACE_FAR` で確かめられます。");
}
