//! **小さい模型で恒等式が破れるのは、「作り方」か「切り方」か**（9-H の H6。4-233）。
//!
//! # なぜ分けられるようになったか
//!
//! 4-232 で **B-Rep を一様に縮められる**ようになりました。おかげで、同じ形を
//! 2通りに用意できます。
//!
//! - **小さく作る**: 寸法を `s` 倍して作る（`scale_sweep_probe` と同じ）
//! - **作って縮める**: 桁 1 で作ってから `scale_solid` で縮める
//!
//! **どちらも厳密に相似**です。ここで恒等式を測って、
//!
//! | 結果 | 読み |
//! | :--- | :--- |
//! | どちらも破れる | **ブーリアンが、小さい模型で崩れている** |
//! | 小さく作ったほうだけ破れる | **素形状の作り方**が桁で違う |
//! | どちらも閉じる | 破れは**演算の相手の置き方**から来ている |
//!
//! 4-230 で交線の精度、4-231 で体積の積分が**それぞれ無罪**になったので、
//! 残っているのは「どこを切って、どの片を採るか」です。**その中を、さらに
//! 2つに割ります。**
use zenith_algo::{BooleanEngine, BooleanOpType, BrepTransform, MassCalculator, PrimitiveBuilder};
use zenith_math::{Tolerance, Vec3};
use zenith_tess::TessellationParams;
use zenith_topo::Solid;

fn params() -> TessellationParams {
    TessellationParams {
        u_divisions: 32,
        v_divisions: 32,
    }
}

fn volume(solids: &[Solid]) -> f64 {
    solids
        .iter()
        .map(|solid| MassCalculator::compute_from_brep(solid, &params()).volume)
        .sum()
}

/// 跳ぶ組。`scale_sweep_probe` の `torus × cylinder (rod through the hole)`。
fn built_small(s: f64) -> (Solid, Solid) {
    let torus = PrimitiveBuilder::make_torus(12.0 * s, 4.0 * s).expect("torus");
    let rod = BrepTransform::translate_solid(
        &PrimitiveBuilder::make_cylinder(9.0 * s, 40.0 * s).expect("cylinder"),
        Vec3::new(0.0, 0.0, -20.0 * s),
    );
    (torus, rod)
}

/// 桁 1 で作ってから縮める。**形は上とまったく同じ**はずです。
fn built_then_shrunk(s: f64) -> Option<(Solid, Solid)> {
    let (torus, rod) = built_small(1.0);
    Some((
        BrepTransform::scale_solid(&torus, s).ok()?,
        BrepTransform::scale_solid(&rod, s).ok()?,
    ))
}

/// 3演算を回して、恒等式の残差（相対）を返す。
fn identity_residual(a: &Solid, b: &Solid, tol: &Tolerance) -> Option<(f64, f64, f64)> {
    let mut volumes = [0.0f64; 3];
    for (index, op) in [
        BooleanOpType::Union,
        BooleanOpType::Difference,
        BooleanOpType::Intersection,
    ]
    .into_iter()
    .enumerate()
    {
        let result = BooleanEngine::boolean_solids_exact_result(a, b, op, tol).ok()?;
        volumes[index] = volume(&result.solids);
    }
    let (va, vb) = (
        volume(std::slice::from_ref(a)),
        volume(std::slice::from_ref(b)),
    );
    let scale = (va + vb).abs().max(f64::MIN_POSITIVE);
    let first = ((volumes[0] + volumes[2]) - (va + vb)).abs() / scale;
    let second = ((volumes[1] + volumes[2]) - va).abs() / scale;
    Some((first.max(second), volumes[1], volumes[2]))
}

fn main() {
    let tol = Tolerance::default();
    let scales = [1.0_f64, 0.1, 0.02, 0.01, 0.005];

    println!("恒等式の破れは「小さく作った」から来るのか、「小さい模型で切った」から来るのか");
    println!();
    println!(
        "{:>8}{:>18}{:>18}{:>18}{:>18}  {}",
        "scale", "作る（残差）", "縮める（残差）", "作る（積）", "縮める（積）", "verdict"
    );
    println!("{}", "-".repeat(104));

    for scale in scales {
        let (a1, b1) = built_small(scale);
        let built = identity_residual(&a1, &b1, &tol);

        let shrunk =
            built_then_shrunk(scale).and_then(|(a2, b2)| identity_residual(&a2, &b2, &tol));

        let show = |value: Option<(f64, f64, f64)>, pick: usize| -> String {
            match value {
                Some((residual, difference, intersection)) => match pick {
                    0 => format!("{residual:.3e}"),
                    1 => format!("{difference:.9}"),
                    _ => format!("{intersection:.9}"),
                },
                None => "断られた".to_string(),
            }
        };

        let verdict = match (built, shrunk) {
            (Some((left, _, _)), Some((right, _, _))) => {
                if left <= 1e-9 && right <= 1e-9 {
                    "どちらも閉じる"
                } else if left > 1e-9 && right > 1e-9 {
                    "**どちらも破れる**（ブーリアンが小さい模型で崩れている）"
                } else if left > 1e-9 {
                    "**作ったほうだけ破れる**（素形状の作り方）"
                } else {
                    "**縮めたほうだけ破れる**（縮め方）"
                }
            }
            _ => "測れません",
        };

        println!(
            "{:>8}{:>18}{:>18}{:>18}{:>18}  {}",
            scale,
            show(built, 0),
            show(shrunk, 0),
            show(built, 2),
            show(shrunk, 2),
            verdict
        );
    }

    println!("{}", "-".repeat(104));
    println!("**「積」の欄は、積の体積を `s³` で割らずにそのまま出しています**——");
    println!("桁ごとに違う数になるので、同じ桁の2つを見比べてください。**同じなら**");
    println!("形は一致していて、**違うなら**作り方と縮め方で別の立体になっています。");
}
