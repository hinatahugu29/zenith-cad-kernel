//! **帯で解いた補間が、密と同じ答えを出すか**（4-706 の測り）。
//!
//! 4-705 で、交線の当てはめ 171.8 秒のうち **93% が `interpolate_points` の
//! 密 LU** だと測りました。`ZENITH_BANDED_INTERPOLATE` は同じ行列を帯として
//! 解く口（既定オフ）です。**速いかどうかの前に、同じ答えかどうか**を測ります。
//!
//!     cargo run --release -p zenith_algo --example banded_interpolate_probe
//!
//! 出る数は 2 つ——**制御点のいちばん大きなずれ**と、**曲線を 4 倍の標本で
//! 評価したときのいちばん大きなずれ**。どちらも 0 に近くなければ、帯では
//! 解けません。
use std::time::Instant;
use zenith_geom::nurbs_curve::NurbsCurve3;
use zenith_math::Point3;

/// 交線の行進が吐くような点列（曲がりながら、間隔が一定でない）。
fn sample_points(count: usize, kind: &str) -> Vec<Point3> {
    (0..count)
        .map(|index| {
            let t = index as f64 / (count - 1) as f64;
            match kind {
                // らせん。
                "helix" => {
                    let angle = t * 12.0;
                    Point3::new(angle.cos() * 3.0, angle.sin() * 3.0, t * 7.0)
                }
                // 間隔が端で詰まる曲線（弦長の媒介変数が偏る所）。
                "crowded" => {
                    let u = t * t * (3.0 - 2.0 * t);
                    Point3::new(u * 10.0, (u * 9.0).sin() * 2.0, (u * 4.0).cos())
                }
                // ほぼ直線に、小さな揺れ。
                _ => Point3::new(t * 20.0, (t * 60.0).sin() * 1e-3, t * 1e-2),
            }
        })
        .collect()
}

fn main() {
    println!("帯で解いた補間と、密で解いた補間を比べます（4-706）");
    println!();
    println!("**既定は密**です。`ZENITH_BANDED_INTERPOLATE` は既定オフの口で、");
    println!("この測りの中だけ立てます。");
    println!();
    println!(
        "{:<10} {:>6} {:>10} {:>10} {:>12} {:>12}",
        "点の列", "点", "密 秒", "帯 秒", "制御点のずれ", "曲線のずれ"
    );

    let mut worst_control = 0.0f64;
    let mut worst_curve = 0.0f64;
    let mut refused = 0usize;

    for kind in ["helix", "crowded", "wobble"] {
        for count in [16usize, 64, 503, 2048, 2503, 4097] {
            let points = sample_points(count, kind);

            std::env::remove_var("ZENITH_BANDED_INTERPOLATE");
            let dense_began = Instant::now();
            let dense = NurbsCurve3::interpolate_points(3, &points);
            let dense_secs = dense_began.elapsed().as_secs_f64();

            std::env::set_var("ZENITH_BANDED_INTERPOLATE", "1");
            let banded_began = Instant::now();
            let banded = NurbsCurve3::interpolate_points(3, &points);
            let banded_secs = banded_began.elapsed().as_secs_f64();
            std::env::remove_var("ZENITH_BANDED_INTERPOLATE");

            let (Ok(dense), Ok(banded)) = (dense, banded) else {
                println!("{kind:<10} {count:>6}   どちらかが補間できませんでした");
                refused += 1;
                continue;
            };

            let mut control = 0.0f64;
            if dense.control_points.len() == banded.control_points.len() {
                for (a, b) in dense
                    .control_points
                    .iter()
                    .zip(banded.control_points.iter())
                {
                    control = control.max((a.point - b.point).norm());
                }
            } else {
                control = f64::INFINITY;
            }

            let mut curve = 0.0f64;
            let (t0, t1) = dense.param_range();
            let samples = count * 4;
            for step in 0..=samples {
                let t = t0 + (t1 - t0) * (step as f64 / samples as f64);
                curve = curve.max((dense.evaluate(t) - banded.evaluate(t)).norm());
            }

            worst_control = worst_control.max(control);
            worst_curve = worst_curve.max(curve);
            println!(
                "{kind:<10} {count:>6} {dense_secs:>10.3} {banded_secs:>10.3} {control:>12.3e} {curve:>12.3e}"
            );
        }
    }

    println!();
    println!("制御点のいちばん大きなずれ: {worst_control:.3e}");
    println!("曲線のいちばん大きなずれ:   {worst_curve:.3e}");
    if refused > 0 {
        println!("補間できなかった組: {refused} 件");
    }
    println!();
    // 許す幅は、座標が 20 のあたりなので、倍精度の丸めが積み上がる余地を見て 1e-9。
    let allowance = 1e-9;
    if worst_curve > allowance || worst_control > allowance {
        println!("**{} over the allowance**（許す幅 {allowance:.0e}）", 1);
        println!("帯では解けません。既定を変えてはいけません。");
        std::process::exit(1);
    }
    println!("**同じ答えです**（許す幅 {allowance:.0e} の内）。");
}
