//! **1面あたり何枚か**を、64分割で測ります（9-H の H5）。
//!
//! H5 の目標は「64分割でも表示メッシュの1面が1万枚以下」です。ここは
//! `ZENITH_PATCH_WHY=1` の行を人が読んで最大を拾う代わりに、**数字を1つ
//! 置く**ための口です。4-204 の訂正——**演算を1つしか測っていなかった**——
//! を繰り返さないよう、和・差・積を必ず3つとも測ります。
//!
//! 実測（2026/08/31、直す前）: 最悪は 25度傾けたトーラスと箱の**和**で
//! **13,524 枚**。差は 9,784 枚、積は 10,880 枚です。
use zenith_algo::{BooleanEngine, BooleanOpType, BrepTransform, PrimitiveBuilder};
use zenith_math::{Tolerance, Transform3, Vec3};
use zenith_tess::TessellationParams;
use zenith_topo::Solid;

/// 1つの立体で、いちばん重い面の枚数と、面の数、全体の枚数。
///
/// **1面だけを立体にして測ってはいけません。** 刻みの計画は立体の大きさと
/// 隣の稜から決まるので、切り出すと本番と違う数が出ます（実測で 13,524 が
/// 12,244 に見えました）。`face_triangle_counts` は本番と同じ道を通ります。
fn worst_face(solid: &Solid, divisions: usize) -> (usize, usize, usize) {
    let params = TessellationParams {
        u_divisions: divisions,
        v_divisions: divisions,
    };
    let counts = zenith_tess::face_triangle_counts(solid, &params);
    let worst = counts.iter().map(|(_, count)| *count).max().unwrap_or(0);
    let total = counts.iter().map(|(_, count)| *count).sum();
    (worst, counts.len(), total)
}

fn main() {
    let tol = Tolerance::default();
    let divisions = std::env::var("ZENITH_DIVISIONS")
        .ok()
        .and_then(|value| value.parse().ok())
        .unwrap_or(64usize);

    let boxa = PrimitiveBuilder::make_box(20.0, 20.0, 20.0).expect("box");
    let torus = PrimitiveBuilder::make_torus(12.0, 4.0).expect("torus");
    let tilt = Transform3::from_axis_angle(&Vec3::new(1.0, 1.0, 0.0), 25f64.to_radians());
    let two_axes = Transform3::from_axis_angle(&Vec3::new(0.0, 1.0, 0.0), 41f64.to_radians());

    let tilted = BrepTransform::translate_solid(
        &BrepTransform::transform_solid(&torus, &tilt).expect("tilt"),
        Vec3::new(10.0, 10.0, 10.0),
    );
    let spun = BrepTransform::translate_solid(
        &BrepTransform::transform_solid(&torus, &two_axes).expect("spin"),
        Vec3::new(10.0, 10.0, 10.0),
    );

    let cases: Vec<(&str, Solid, Solid)> = vec![
        ("box x torus (inclined 25deg)", boxa.clone(), tilted),
        ("box x torus (two axes)", boxa.clone(), spun),
    ];
    let ops = [
        ("union", BooleanOpType::Union),
        ("difference", BooleanOpType::Difference),
        ("intersection", BooleanOpType::Intersection),
    ];

    println!("{divisions}分割での、1面あたりの最大枚数（9-H の H5 は 10,000 以下）");
    println!();
    println!("{:<32}{:<14}{:>8}{:>12}{:>12}  {}", "case", "op", "faces", "worst face", "total", "verdict");
    println!("{}", "-".repeat(92));

    let (mut worst_all, mut worst_where) = (0usize, String::from("-"));
    for (name, a, b) in &cases {
        for (label, op) in ops {
            let Ok(result) = BooleanEngine::boolean_solids_exact_result(a, b, op, &tol) else {
                println!("{:<32}{:<14}{:>8}{:>12}{:>12}  断られた", name, label, "-", "-", "-");
                continue;
            };
            let (mut worst, mut faces, mut total) = (0usize, 0usize, 0usize);
            for solid in &result.solids {
                let (w, f, t) = worst_face(solid, divisions);
                worst = worst.max(w);
                faces += f;
                total += t;
            }
            if worst > worst_all {
                worst_all = worst;
                worst_where = format!("{name} / {label}");
            }
            println!(
                "{:<32}{:<14}{:>8}{:>12}{:>12}  {}",
                name,
                label,
                faces,
                worst,
                total,
                if worst <= 10_000 { "ok" } else { "**H5 未達**" }
            );
        }
    }
    println!("{}", "-".repeat(92));
    println!("最悪は {worst_all} 枚（{worst_where}）。H5 は 10,000 以下。");

    // **2026/08/31 に達成したので、ここから先は赤にします**（4-210）。
    //
    // 達成する前は報告だけでした。**通ったものは門にする**——そうしないと、
    // 次の変更が静かに戻します。実測は 3,854 枚なので、上限まで 2.5 倍の
    // 余裕があります。**上げるときは、なぜ上げるのかを 9-H に書いてから
    // にしてください。**
    if worst_all > 10_000 {
        eprintln!(
            "H5 未達: 1面あたり最大 {worst_all} 枚（{worst_where}）。上限は 10,000 枚です。"
        );
        std::process::exit(1);
    }
}
