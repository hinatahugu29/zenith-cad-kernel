//! **平らな板は通し、曲がった縁は名指しで断る**（4-416）。
//!
//! # 何があったか
//!
//! `thicken_coons_face` は **4 隅から角柱を組むだけ**です——底も天も
//! **隅どうしを結んだ直線**で輪を作り、側面は平面。**境界曲線は
//! 一度も読んでいません。**
//!
//! **縁がまっすぐなら、それが正しい答え**です。**曲がっていたら、
//! 中身の違う立体**が返ります（実測: 閉じた式 210 / 230 / 260 に対して
//! 203.3 / 210.0 / 219.9）。**閉じた多様体で、形も板**なので、
//! **形の検査では捕まりません。**
//!
//! **2026/09/09 まで、そこは「たまたま」断られていました**——
//! 面の境界検査が **16x16 の標本の中でいちばん近い点**までの距離を
//! 「曲面までの距離」と呼んでおり、その粗さで落ちていたのです。
//! **同じ粗さが、正しい平らな板まで断っていました**——**制御点が
//! 等間隔でないと 8.203e-2 外れていると報告されます**（本当は 6.7e-14）。
//!
//! **検査を本物の距離に直し**、**この作りが 4 隅の箱しか組めないことを
//! 名指しで断る**ようにしました（4-416）。**両方要ります**——
//! **片方だけだと、断りが誤答に変わります。**
//!
//! # そのあと、作りのほうを直しました（4-417）
//!
//! **輪を境界曲線そのものから組み、側面をその曲線とオフセット曲線の
//! あいだの線織面**にしました。**曲がった縁の板が、閉じた式と合います**
//! ——`(100 + 5b) * 2` に対して、刻み 8 → 256 で相対差
//! **1.019e-3 → 9.951e-7**（**2 次で 0 へ**）。
//!
//! # そのあと、曲がったシートも通しました（4-418）
//!
//! **天面は制御点を法線方向へずらして作ります。** **平らなら厳密な
//! 平行移動**ですが、**曲がっていると本当のオフセットではありません**
//! ——4-417 の時点で **1.1% ずれ、刻みでは縮みません**でした。
//!
//! **ノット挿入で、境界曲線を形を変えずに細かく**してから、
//! **Greville の横座標**で法線を当てるようにしました。
//! **制御多角形が曲線に寄れば、そのずれは縮みます**——
//! **1.1% → 8.2e-6**（下の試験）。

use zenith_algo::{CurvePatchBuilder, MassCalculator, ThickenBuilder};
use zenith_geom::NurbsCurve3;
use zenith_math::{Point3, Tolerance};
use zenith_tess::TessellationParams;

fn curve(points: Vec<Point3>) -> NurbsCurve3 {
    NurbsCurve3::bspline_from_points(3, points).expect("curve")
}

/// 10 x 10 の平らな板。`xs` が縁の制御点の置き方。
fn flat_sheet(xs: &[f64]) -> zenith_topo::Face {
    let tol = Tolerance::default();
    CurvePatchBuilder::build_from_4_curves(
        curve(xs.iter().map(|x| Point3::new(*x, 0.0, 0.0)).collect()),
        curve(xs.iter().map(|x| Point3::new(*x, 10.0, 0.0)).collect()),
        curve(xs.iter().map(|y| Point3::new(0.0, *y, 0.0)).collect()),
        curve(xs.iter().map(|y| Point3::new(10.0, *y, 0.0)).collect()),
        &tol,
    )
    .expect("patch")
}

/// **制御点の置き方は、答えを変えてはいけません。**
///
/// **縁の形は 3 つとも同じ直線**で、**変わるのは媒介変数だけ**です。
/// **2026/09/09 まで、等間隔のときしか通りませんでした。**
#[test]
fn a_flat_sheet_thickens_however_its_edges_are_parameterised() {
    let tol = Tolerance::default();
    for xs in [
        vec![0.0, 10.0 / 3.0, 20.0 / 3.0, 10.0],
        vec![0.0, 3.0, 7.0, 10.0],
        vec![0.0, 0.5, 9.5, 10.0],
    ] {
        let solid = ThickenBuilder::thicken_face(&flat_sheet(&xs), 2.0, &tol)
            .unwrap_or_else(|error| panic!("a flat 10x10 sheet should thicken, got {error}"));
        let volume =
            MassCalculator::compute_from_brep(&solid, &TessellationParams::default()).volume;
        assert!(
            (volume - 200.0).abs() <= 1e-9,
            "10 x 10 x 2 is 200, got {volume} for {xs:?}"
        );
    }
}

/// **曲がった縁の板は、閉じた式と合うこと**（4-417）。
///
/// **2026/09/09 の朝まで、ここは名指しで断っていました**（4-416）
/// ——**4 隅から角柱を組むだけ**で、**黙って中身の違う立体**を返して
/// いたからです。**輪を境界曲線から組むようにして、通しました。**
#[test]
fn a_bowed_boundary_thickens_to_the_closed_form() {
    let tol = Tolerance::default();
    // 片側の縁を、制御点 y = 0, -b, -b, 0 の 3 次で膨らませる。
    // 増える面積は ∫ y dx = 5b（x は制御点が等間隔なので x(t) = 10t）。
    for bow in [1.0, 3.0, 6.0] {
        let face = CurvePatchBuilder::build_from_4_curves(
            curve(vec![
                Point3::new(0.0, 0.0, 0.0),
                Point3::new(10.0 / 3.0, -bow, 0.0),
                Point3::new(20.0 / 3.0, -bow, 0.0),
                Point3::new(10.0, 0.0, 0.0),
            ]),
            curve(vec![
                Point3::new(0.0, 10.0, 0.0),
                Point3::new(10.0 / 3.0, 10.0, 0.0),
                Point3::new(20.0 / 3.0, 10.0, 0.0),
                Point3::new(10.0, 10.0, 0.0),
            ]),
            curve(vec![
                Point3::new(0.0, 0.0, 0.0),
                Point3::new(0.0, 10.0 / 3.0, 0.0),
                Point3::new(0.0, 20.0 / 3.0, 0.0),
                Point3::new(0.0, 10.0, 0.0),
            ]),
            curve(vec![
                Point3::new(10.0, 0.0, 0.0),
                Point3::new(10.0, 10.0 / 3.0, 0.0),
                Point3::new(10.0, 20.0 / 3.0, 0.0),
                Point3::new(10.0, 10.0, 0.0),
            ]),
            &tol,
        )
        .expect("patch");

        let solid = ThickenBuilder::thicken_face(&face, 2.0, &tol)
            .unwrap_or_else(|error| panic!("a bowed boundary (bow {bow}) should thicken, got {error}"));
        let params = TessellationParams {
            u_divisions: 64,
            v_divisions: 64,
        };
        let volume = MassCalculator::compute_from_brep(&solid, &params).volume;
        let expected = (100.0 + 5.0 * bow) * 2.0;
        let residual = (volume - expected).abs() / expected;
        assert!(
            residual <= 1e-4,
            "bow {bow}: expected {expected}, got {volume} (relative {residual:.3e})"
        );
    }
}

/// **曲がったシートも、独立に積んだ値と合うこと**（4-418）。
///
/// **参照は、カーネルの厚み付けを使わずに作ります**——**曲線を密に
/// 標本して長さと回転角を積み**、`帯の面積 = 長さ × t + 回転角 × t² / 2`
/// から出します。**同じ道具で測り直しても、偏りは消えません**（4-364）。
///
/// **4-417 の時点では 1.1% ずれ、刻みを 8 倍にしても縮みません**でした。
#[test]
fn a_curved_sheet_matches_a_reference_built_without_the_thickener() {
    let tol = Tolerance::default();
    let radius = 10.0;
    let length = 20.0;
    let thickness = 1.0;
    let count = 7;
    let arc_points: Vec<Point3> = (0..count)
        .map(|i| {
            let angle = std::f64::consts::FRAC_PI_2 * i as f64 / (count - 1) as f64;
            Point3::new(radius * angle.cos(), radius * angle.sin(), 0.0)
        })
        .collect();
    let arc = curve(arc_points.clone());

    // **参照**——曲線を密に標本して、長さと回転角を積む。
    let steps = 200_000;
    let (t0, t1) = arc.param_range();
    let samples: Vec<Point3> = (0..=steps)
        .map(|i| arc.evaluate(t0 + (t1 - t0) * i as f64 / steps as f64))
        .collect();
    let arc_length: f64 = samples.windows(2).map(|pair| (pair[1] - pair[0]).norm()).sum();
    let heading = |from: Point3, to: Point3| (to.y - from.y).atan2(to.x - from.x);
    let mut turning = 0.0f64;
    let mut previous = heading(samples[0], samples[1]);
    for window in samples.windows(2).skip(1) {
        let current = heading(window[0], window[1]);
        let mut step = current - previous;
        while step > std::f64::consts::PI {
            step -= 2.0 * std::f64::consts::PI;
        }
        while step < -std::f64::consts::PI {
            step += 2.0 * std::f64::consts::PI;
        }
        turning += step.abs();
        previous = current;
    }
    let expected = (arc_length * thickness + turning * thickness * thickness / 2.0) * length;

    let face = CurvePatchBuilder::build_from_4_curves(
        curve(arc_points.clone()),
        curve(
            arc_points
                .iter()
                .map(|p| Point3::new(p.x, p.y, length))
                .collect(),
        ),
        curve(
            (0..count)
                .map(|i| Point3::new(radius, 0.0, length * i as f64 / (count - 1) as f64))
                .collect(),
        ),
        curve(
            (0..count)
                .map(|i| Point3::new(0.0, radius, length * i as f64 / (count - 1) as f64))
                .collect(),
        ),
        &tol,
    )
    .expect("patch");

    let solid = ThickenBuilder::thicken_face(&face, thickness, &tol)
        .unwrap_or_else(|error| panic!("a curved sheet should thicken, got {error}"));
    let params = TessellationParams {
        u_divisions: 128,
        v_divisions: 128,
    };
    let volume = MassCalculator::compute_from_brep(&solid, &params).volume;
    let residual = (volume - expected).abs() / expected;
    assert!(
        residual <= 1e-4,
        "expected {expected} (arc length {arc_length}, turning {turning}), got {volume} (relative {residual:.3e})"
    );
}
