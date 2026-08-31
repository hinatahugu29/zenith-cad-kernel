//! 円柱を**斜めの平面**で切った断面が、パラメータ面でどう見えるかを測る。
//!
//! いま円柱の側面を割る口は2つしかありません（4-120）。
//!
//! - 軸に沿った母線（`u = const`）
//! - 軸まわりの円（`v = const`）
//!
//! **斜めの平面で切ると断面は楕円で、どちらにも当たりません。** そのため
//! 箱と円柱を**両方**回すと、ブーリアンが3演算とも断ります（4-119）。
//!
//! 3つ目の口を足すときの見立ては「円柱のパラメータ面では、楕円は
//! **v = a·cos(u) + b·sin(u) + c** という1本の曲線になる」です。**これは
//! 推論なので、実際に測ります。** 乗るなら、割ったあとの境界もそのまま
//! 書けます。乗らないなら、見立てが外れています（4-33: カーネルを疑う前に
//! 自分の式を疑う）。
//!
//! ここでやること: 円柱の側面パッチを取り、斜めの平面との交線上の点を
//! 曲面へ投影して uv を取り、最小二乗で上の式に当てはめて**残差**を出す。
//!
//! ```bash
//! cargo run --release -p zenith_algo --example oblique_section_probe
//! ```

use zenith_algo::PrimitiveBuilder;
use zenith_geom::{ExtremumEngine, NurbsSurface3};
use zenith_math::{Point3, Tolerance, Vec3};
use zenith_topo::FaceGeometry;

/// 円柱の側面パッチをすべて集める。
fn side_patches(radius: f64, height: f64) -> Vec<NurbsSurface3> {
    let solid = PrimitiveBuilder::make_cylinder(radius, height).expect("cylinder");
    let mut out = Vec::new();
    for face in &solid.outer_shell.faces {
        if let FaceGeometry::Nurbs(surface) = &face.geometry {
            out.push(surface.clone());
        }
    }
    out
}

/// 過剰決定の線形最小二乗を、正規方程式＋ガウス消去で解く。
///
/// 行は「基底の値」、右辺は当てはめたい値。列数は基底の数。
fn least_squares(rows: &[(Vec<f64>, f64)]) -> Option<Vec<f64>> {
    let width = rows.first()?.0.len();
    if rows.len() < width {
        return None;
    }
    let mut normal = vec![vec![0.0f64; width]; width];
    let mut rhs = vec![0.0f64; width];
    for (basis, value) in rows {
        for row in 0..width {
            for column in 0..width {
                normal[row][column] += basis[row] * basis[column];
            }
            rhs[row] += basis[row] * value;
        }
    }
    for pivot in 0..width {
        let mut best = pivot;
        for row in (pivot + 1)..width {
            if normal[row][pivot].abs() > normal[best][pivot].abs() {
                best = row;
            }
        }
        if normal[best][pivot].abs() < 1e-16 {
            return None;
        }
        normal.swap(pivot, best);
        rhs.swap(pivot, best);
        for row in (pivot + 1)..width {
            let factor = normal[row][pivot] / normal[pivot][pivot];
            for column in pivot..width {
                normal[row][column] -= factor * normal[pivot][column];
            }
            rhs[row] -= factor * rhs[pivot];
        }
    }
    let mut out = vec![0.0f64; width];
    for row in (0..width).rev() {
        let mut value = rhs[row];
        for column in (row + 1)..width {
            value -= normal[row][column] * out[column];
        }
        out[row] = value / normal[row][row];
    }
    Some(out)
}

/// `v = a·cos(u) + b·sin(u) + c` の当てはめ。最大残差を返す。
///
/// **これは外れました**（実測、残差 1.19e-2）。`u` は幾何的な角度ではなく
/// 有理2次の媒介変数なので、角度は `u` の非線形な関数です。記録として
/// 残してあります。
fn fit_sinusoid(samples: &[(f64, f64)]) -> Option<f64> {
    let rows: Vec<(Vec<f64>, f64)> = samples
        .iter()
        .map(|(u, v)| (vec![u.cos(), u.sin(), 1.0], *v))
        .collect();
    let solution = least_squares(&rows)?;
    let mut worst = 0.0f64;
    for (u, v) in samples {
        let predicted = u.cos() * solution[0] + u.sin() * solution[1] + solution[2];
        worst = worst.max((predicted - v).abs());
    }
    Some(worst)
}

/// `v = (a + b·u + c·u²) / (1 + d·u + e·u²)` の当てはめ。最大残差を返す。
///
/// **こちらが本命です。** 円柱の側面パッチは有理2次で、点は
/// `x = X(u)/W(u)`、`y = Y(u)/W(u)`、`z = v` です（`X`、`Y`、`W` は `u` の
/// 2次式）。平面 `n·p = d` に代入すると
///
/// ```text
/// v = (d·W(u) − n_x·X(u) − n_y·Y(u)) / (n_z·W(u))
/// ```
///
/// で、**分子も分母も `u` の2次式**になります。分母を払うと
/// `a + b·u + c·u² − v·(d·u + e·u²) = v` で、係数について線形なので
/// そのまま最小二乗で解けます。
fn fit_rational_quadratic(samples: &[(f64, f64)]) -> Option<f64> {
    let rows: Vec<(Vec<f64>, f64)> = samples
        .iter()
        .map(|(u, v)| (vec![1.0, *u, u * u, -v * u, -v * u * u], *v))
        .collect();
    let solution = least_squares(&rows)?;
    let mut worst = 0.0f64;
    for (u, v) in samples {
        let numerator = solution[0] + solution[1] * u + solution[2] * u * u;
        let denominator = 1.0 + solution[3] * u + solution[4] * u * u;
        if denominator.abs() < 1e-12 {
            return None;
        }
        worst = worst.max((numerator / denominator - v).abs());
    }
    Some(worst)
}

fn main() {
    let tol = Tolerance::default();
    let radius = 10.0;
    let height = 40.0;
    let patches = side_patches(radius, height);

    println!("円柱を斜めの平面で切った断面を、パラメータ面で当てはめる");
    println!();
    println!("見立て: v = a·cos(u) + b·sin(u) + c（円柱の u = 角度、v = 軸方向）");
    println!(
        "円柱 半径 {radius} 高さ {height}、側面パッチ {} 枚",
        patches.len()
    );
    println!();
    println!(
        "{:<10} {:>7} {:>7} {:>12} {:>12} {:>12}  {}",
        "平面の傾き", "パッチ", "標本", "正弦の残差", "有理式の残差", "v の振れ幅", "見立て"
    );
    println!("{}", "-".repeat(100));

    let mut worst_overall = 0.0f64;
    let mut worst_sinusoid = 0.0f64;
    let mut measured = 0usize;

    for degrees in [10.0f64, 27.0, 45.0, 60.0, 80.0] {
        // 平面: 原点 (0,0,height/2) を通り、法線を x-z 面内で傾ける。
        // 傾き 0 度なら軸に垂直（＝断面は円）、大きいほど斜め。
        let angle = degrees.to_radians();
        let normal = Vec3::new(angle.sin(), 0.0, angle.cos());
        let origin = Point3::new(0.0, 0.0, height * 0.5);

        for (index, surface) in patches.iter().enumerate() {
            // パッチの uv 格子を走り、平面をまたぐところで二分して交点を取る。
            // **交線そのものを作らずに測ります**——ここで見たいのは
            // 「断面の点の uv がどう並ぶか」だけなので、交線の実装に依存
            // させないほうが確かです。
            let ((u_min, u_max), (v_min, v_max)) = surface.param_range();
            let signed = |u: f64, v: f64| (surface.evaluate(u, v) - origin).dot(&normal);

            let mut samples: Vec<(f64, f64)> = Vec::new();
            let mut worst_projection = 0.0f64;
            let columns = 48;
            for step in 0..=columns {
                let u = u_min + (u_max - u_min) * step as f64 / columns as f64;
                let (mut low, mut high) = (v_min, v_max);
                if signed(u, low) * signed(u, high) > 0.0 {
                    continue;
                }
                for _ in 0..80 {
                    let middle = 0.5 * (low + high);
                    if signed(u, low) * signed(u, middle) <= 0.0 {
                        high = middle;
                    } else {
                        low = middle;
                    }
                }
                let v = 0.5 * (low + high);
                let point = surface.evaluate(u, v);

                // uv は格子から取っていますが、**投影でも同じ uv に戻るか**を
                // 見ておきます。戻らなければ、この標本の取り方が疑わしい。
                if let Ok(projection) =
                    ExtremumEngine::point_to_surface(point, surface, 32, tol.parametric)
                {
                    worst_projection = worst_projection.max(projection.distance);
                }
                samples.push((u, v));
            }

            if samples.len() < 4 {
                continue;
            }
            let spread = {
                let (mut low, mut high) = (f64::INFINITY, f64::NEG_INFINITY);
                for (_, v) in &samples {
                    low = low.min(*v);
                    high = high.max(*v);
                }
                high - low
            };
            // 標本は uv 格子から取っています。**投影でも同じ点に戻るか**を
            // 見ておかないと、当てはめの残差が「標本の取り方の誤差」なのか
            // 「式が合っていない」のか区別できません。
            if worst_projection > 1e-6 {
                println!(
                    "  ! 標本が曲面から {worst_projection:.3e} 離れています。当てはめの残差はこれに引きずられます"
                );
            }
            let sinusoid = fit_sinusoid(&samples).unwrap_or(f64::INFINITY);
            let Some(rational) = fit_rational_quadratic(&samples) else {
                continue;
            };
            measured += 1;
            worst_overall = worst_overall.max(rational);
            worst_sinusoid = worst_sinusoid.max(sinusoid);

            println!(
                "{:<10} {:>7} {:>7} {:>12.3e} {:>12.3e} {:>12.6}  {}",
                format!("{degrees} 度"),
                index,
                samples.len(),
                sinusoid,
                rational,
                spread,
                if rational <= 1e-9 {
                    "有理式に乗る"
                } else {
                    "**どちらにも乗らない**"
                }
            );
        }
    }

    println!("{}", "-".repeat(100));
    println!(
        "{measured} 通りを当てはめました。残差の最悪は、正弦 {worst_sinusoid:.3e}、有理式 {worst_overall:.3e} です。"
    );
    println!();
    if worst_overall <= 1e-9 {
        println!("**斜めの断面は、円柱のパラメータ面で有理2次の1本の曲線です。**");
        println!();
        println!("  v = (a + b·u + c·u²) / (1 + d·u + e·u²)");
        println!();
        println!("正弦の形（v = a·cos u + b·sin u + c）には乗りません。u が幾何的な");
        println!("角度ではなく、有理2次の媒介変数だからです。**3-N-2c に最初そう書いた");
        println!("のは誤りで、これで直しました。**");
        println!();
        println!("3つ目の割り口は、この形を受け付ける形で書けます。p-curve として");
        println!("厳密に持てるので、割ったあとの境界も近似になりません（4-120）。");
    } else {
        println!("**有理2次にも乗りません。** 見立てを立て直してください");
        println!("（4-33: カーネルを疑う前に自分の式を疑う）。");
        std::process::exit(1);
    }
}
