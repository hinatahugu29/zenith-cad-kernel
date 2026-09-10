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
//! 名指しで断る**ようにしました。**両方要ります**——**片方だけだと、
//! 断りが誤答に変わります。**

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

/// **曲がった縁は、名指しで断ること。**
///
/// **黙って角柱を返すほうが、ずっと困ります**——**大きさだけが違う
/// 立体**は、形の検査では捕まりません。
#[test]
fn a_bowed_boundary_is_refused_by_name() {
    let tol = Tolerance::default();
    for bow in [1e-3, 1.0, 6.0] {
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

        let message = ThickenBuilder::thicken_face(&face, 2.0, &tol)
            .err()
            .unwrap_or_else(|| panic!("a bowed boundary (bow {bow}) must be refused, not thickened"));
        assert!(
            message.contains("bows") && message.contains("not implemented"),
            "the refusal should name the bow and say it is not implemented, got {message}"
        );
    }
}
