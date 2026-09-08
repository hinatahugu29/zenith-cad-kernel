//! **曲面のシートを厚くする**口が、**面のトリムを読んでいるか**を測る。
//!
//! # なぜ要るのか
//!
//! `thicken_nurbs_face` は、受け取った `Face` の**曲面だけ**を見て、
//! **パラメータ域の端から端まで**を厚くします。**面の輪（トリム）は
//! 引数に取っていながら一度も読んでいません**（`_face`）。
//!
//! 面が素のパッチちょうどなら、それで正しい答えになります。**素のパッチの
//! 一部だけを使う面**——つまり普通のトリム面——では、**使っていない所まで
//! 厚くした立体**が返ります。
//!
//! # 何を測るか
//!
//! 半径 `r`・高さ `h` の円柱の四半パッチを台にして、**輪だけを `h/2` で
//! 止めた面**を作ります。曲面は同じ、輪だけが違います。
//!
//! ```text
//! 素のパッチ    体積 = (π/4)((r+t)² − r²) h
//! 半分に切った  体積 = (π/4)((r+t)² − r²) h/2
//! ```
//!
//! **同じ数字が 2 回出たら、トリムを読んでいません。**
//!
//! # 読み方
//!
//! **断られたら、それが正しい答え**です。**扱えない面を名指しで断るのは
//! 規約どおり**——**困るのは、閉じた多様体で、形も一見それらしいのに、
//! 使っていない所まで実体になっている立体が返ってくること**です。

use std::f64::consts::{FRAC_1_SQRT_2, FRAC_PI_4};

use zenith_algo::{MassCalculator, ThickenBuilder};
use zenith_geom::{ControlPoint3, KnotVector, NurbsCurve3, NurbsSurface3};
use zenith_math::{Point3, Tolerance};
use zenith_tess::TessellationParams;
use zenith_topo::{Edge, Face, FaceGeometry, Orientation, OrientedEdge, Vertex, Wire};

/// 円柱の四半パッチ。`top` まででトリムした輪を張る（`top == h` なら素のパッチ）。
fn cylinder_quarter_trimmed(r: f64, h: f64, top: f64) -> Face {
    let w = FRAC_1_SQRT_2;
    let grid: Vec<Vec<ControlPoint3>> = [(r, 0.0, 1.0), (r, r, w), (0.0, r, 1.0)]
        .iter()
        .map(|(x, y, weight)| {
            vec![
                ControlPoint3::new(Point3::new(*x, *y, 0.0), *weight),
                ControlPoint3::new(Point3::new(*x, *y, h), *weight),
            ]
        })
        .collect();
    let surface = NurbsSurface3::new(
        2,
        1,
        grid,
        KnotVector::clamped_uniform(3, 2),
        KnotVector::clamped_uniform(2, 1),
    )
    .unwrap();
    let arc = |z: f64| {
        NurbsCurve3::new(
            2,
            vec![
                ControlPoint3::unweighted(Point3::new(r, 0.0, z)),
                ControlPoint3::new(Point3::new(r, r, z), w),
                ControlPoint3::unweighted(Point3::new(0.0, r, z)),
            ],
            KnotVector::clamped_uniform(3, 2),
        )
        .unwrap()
    };
    let bottom_start = Vertex::from_point(Point3::new(r, 0.0, 0.0));
    let bottom_end = Vertex::from_point(Point3::new(0.0, r, 0.0));
    let top_start = Vertex::from_point(Point3::new(r, 0.0, top));
    let top_end = Vertex::from_point(Point3::new(0.0, r, top));
    Face::new(
        FaceGeometry::Nurbs(surface),
        Wire::new(vec![
            OrientedEdge::forward(Edge::new(
                arc(0.0),
                bottom_start.clone(),
                bottom_end.clone(),
                1e-6,
            )),
            OrientedEdge::forward(Edge::line_between(bottom_end, top_end.clone()).unwrap()),
            OrientedEdge::reversed(Edge::new(arc(top), top_start.clone(), top_end, 1e-6)),
            OrientedEdge::reversed(Edge::line_between(bottom_start, top_start).unwrap()),
        ]),
        Vec::new(),
        Orientation::Forward,
        1e-6,
    )
}

fn main() {
    let tol = Tolerance::default();
    let params = TessellationParams {
        u_divisions: 96,
        v_divisions: 96,
    };

    let (r, h, t) = (10.0_f64, 20.0_f64, 1.0_f64);
    let shell_ring = FRAC_PI_4 * ((r + t).powi(2) - r * r);

    println!("曲面のシートを厚くする口が、面のトリムを読んでいるか");
    println!();
    println!(
        "{:<30}{:>16}{:>16}{:>12}  {}",
        "面", "閉じた式", "測った体積", "相対差", "結果"
    );
    println!("{}", "-".repeat(96));

    let mut wrong = 0usize;
    let mut measured: Vec<f64> = Vec::new();

    for (name, top) in [("素のパッチ（輪 = 端から端）", h), ("輪を半分で止めた面", h * 0.5)] {
        let face = cylinder_quarter_trimmed(r, h, top);
        let expected = shell_ring * top;
        match ThickenBuilder::thicken_face(&face, t, &tol) {
            Ok(solid) => {
                let volume = MassCalculator::compute_from_brep(&solid, &params).volume;
                measured.push(volume);
                let residual = (volume - expected).abs() / expected;
                // 曲面のオフセットは標本の細かさぶん近似になります。
                // 既存の試験と同じ 1e-4 で見ます。
                let ok = residual <= 1e-4;
                if !ok {
                    wrong += 1;
                }
                println!(
                    "{:<30}{:>16.6}{:>16.6}{:>12.3e}  {}",
                    name,
                    expected,
                    volume,
                    residual,
                    if ok { "ok" } else { "**ちがう**" }
                );
            }
            Err(reason) => {
                println!(
                    "{:<30}{:>16.6}{:>16}{:>12}  断り「{reason}」",
                    name, expected, "-", "-"
                );
            }
        }
    }

    if measured.len() == 2 {
        let gap = (measured[0] - measured[1]).abs() / measured[0].abs().max(1.0);
        println!();
        if gap <= 1e-9 {
            println!(
                "**輪を変えても、同じ体積が返っています**（{:.6}）——面のトリムを読んでいません。",
                measured[0]
            );
        } else {
            println!("輪を変えると体積が変わります（相対差 {gap:.3e}）。");
        }
    }

    println!();
    println!("誤答 {wrong} 件");
}
