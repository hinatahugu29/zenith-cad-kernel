//! パラメータ線でない曲線1本で面を割れるか、割ったあと面積が合うかを測る。
//!
//! これが曲面同士の交差（SSI）の本当の壁である。交線そのものは
//! `zenith_geom::ssi` から取れる見込みがあるが、既存の分割は
//!
//! > 交線はパッチのパラメータ線であり、面の端から端まで届く1本である
//!
//! ことを前提に組まれている。交線だけ供給しても envelope は動かない。
//! ここは前提を「面の上にあり、両端が境界の上にある」の1つに減らした
//! [`FaceSplitter`] を、解析解の分かる配置で測る。
//!
//! 合否は**面積の和**で見る。閉じたワイヤになったこと、p-curve が辺に乗って
//! いることだけでは、領域の取り違え（重複・取りこぼし）は分からない。
//!
//! 走らせ方: cargo run --release -p zenith_algo --example face_split_probe

use std::f64::consts::{FRAC_1_SQRT_2, FRAC_PI_2};

use zenith_algo::{FaceSplitter, MassCalculator};
use zenith_tess::TessellationParams;
use zenith_geom::{ControlPoint3, KnotVector, NurbsCurve3, NurbsSurface3};
use zenith_math::{Point3, Tolerance, Vec3};
use zenith_topo::{Edge, Face, FaceGeometry, OrientedEdge, Vertex, Wire};

/// 半径 `r`、高さ `0..h` の円柱側面のうち、0度から90度の四半パッチ。
fn cylinder_quarter(r: f64, h: f64) -> (NurbsSurface3, Face) {
    let w = FRAC_1_SQRT_2;
    let ring = [
        (r, 0.0, 1.0),
        (r, r, w),
        (0.0, r, 1.0),
    ];
    let grid: Vec<Vec<ControlPoint3>> = ring
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

    // 境界: 下の弧、右の縦線、上の弧（逆）、左の縦線（逆）
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
    let corner = |x: f64, y: f64, z: f64| Vertex::from_point(Point3::new(x, y, z));
    let bottom_start = corner(r, 0.0, 0.0);
    let bottom_end = corner(0.0, r, 0.0);
    let top_start = corner(r, 0.0, h);
    let top_end = corner(0.0, r, h);

    let bottom = Edge::new(arc(0.0), bottom_start.clone(), bottom_end.clone(), 1e-6);
    let top = Edge::new(arc(h), top_start.clone(), top_end.clone(), 1e-6);
    let right = Edge::line_between(bottom_end.clone(), top_end.clone()).unwrap();
    let left = Edge::line_between(bottom_start.clone(), top_start.clone()).unwrap();

    let wire = Wire::new(vec![
        OrientedEdge::forward(bottom),
        OrientedEdge::forward(right),
        OrientedEdge::reversed(top),
        OrientedEdge::reversed(left),
    ]);
    let face = Face::new(
        FaceGeometry::Nurbs(surface.clone()),
        wire,
        Vec::new(),
        zenith_topo::Orientation::Forward,
        1e-6,
    );
    (surface, face)
}

/// 傾いた平面 `z = z0 + slope * x` が円柱を切ってできる楕円弧。
///
/// 楕円は円のアフィン像なので、円弧の制御点にそのまま同じ写像をかければ
/// **厳密に**表せる。折れ線で近づける必要はない。この曲線は円柱の上に
/// 乗っているが、パラメータ線ではない（両端の高さが違う）。
fn tilted_section(r: f64, z0: f64, slope: f64) -> NurbsCurve3 {
    let w = FRAC_1_SQRT_2;
    let lift = |x: f64, y: f64| Point3::new(x, y, z0 + slope * x);
    NurbsCurve3::new(
        2,
        vec![
            ControlPoint3::unweighted(lift(r, 0.0)),
            ControlPoint3::new(lift(r, r), w),
            ControlPoint3::unweighted(lift(0.0, r)),
        ],
        KnotVector::clamped_uniform(3, 2),
    )
    .unwrap()
}

/// 平面の四角い面。
fn planar_square(side: f64) -> Face {
    let plane = zenith_geom::PlaneSurface3::new(
        Point3::new(0.0, 0.0, 0.0),
        Vec3::new(1.0, 0.0, 0.0),
        Vec3::new(0.0, 1.0, 0.0),
    )
    .unwrap();
    let points = [
        Point3::new(0.0, 0.0, 0.0),
        Point3::new(side, 0.0, 0.0),
        Point3::new(side, side, 0.0),
        Point3::new(0.0, side, 0.0),
    ];
    let vertices: Vec<Vertex> = points.into_iter().map(Vertex::from_point).collect();
    let edges = (0..4)
        .map(|index| {
            OrientedEdge::forward(
                Edge::line_between(vertices[index].clone(), vertices[(index + 1) % 4].clone())
                    .unwrap(),
            )
        })
        .collect();
    Face::new(
        FaceGeometry::Plane(plane),
        Wire::new(edges),
        Vec::new(),
        zenith_topo::Orientation::Forward,
        1e-6,
    )
}

struct Subject {
    name: &'static str,
    face: Face,
    split: Edge,
    /// 解析解が分かるなら、片方の面積。
    expected_piece: Option<f64>,
}

/// 片の **3D の面積**。判定はパラメータ面積でしますが（4-76）、閉じた式と
/// 突き合わせるのはこちらです。
fn areas_3d(pieces: &[Face]) -> Vec<f64> {
    let params = TessellationParams::default();
    pieces
        .iter()
        .map(|piece| MassCalculator::compute_face_integral(piece, &params).0)
        .collect()
}

fn main() {
    let tol = Tolerance::default();
    let mut subjects: Vec<Subject> = Vec::new();

    // 1. 平面を対角線で割る。片方は元の半分。
    {
        let face = planar_square(10.0);
        let split = Edge::line_between(
            Vertex::from_point(Point3::new(10.0, 0.0, 0.0)),
            Vertex::from_point(Point3::new(0.0, 10.0, 0.0)),
        )
        .unwrap();
        subjects.push(Subject {
            name: "plane 10x10 cut corner to corner",
            face,
            split,
            expected_piece: Some(50.0),
        });
    }

    // 2. 平面を、辺の途中から辺の途中へ割る。台形と三角形になる。
    {
        let face = planar_square(10.0);
        let split = Edge::line_between(
            Vertex::from_point(Point3::new(10.0, 4.0, 0.0)),
            Vertex::from_point(Point3::new(3.0, 10.0, 0.0)),
        )
        .unwrap();
        // 角 (10,10) を切り落とす三角形
        subjects.push(Subject {
            name: "plane 10x10 cut mid-edge to mid-edge",
            face,
            split,
            expected_piece: Some(0.5 * 6.0 * 7.0),
        });
    }

    // 3. 円柱の四半パッチを、傾いた平面の切り口（楕円弧）で割る。
    //    これがパラメータ線でない切り方である。下側の面積は
    //    円柱を展開して積める: 半径 r、角 0..pi/2 で高さ z0 + slope * r cos t。
    {
        let (_, face) = cylinder_quarter(10.0, 40.0);
        let curve = tilted_section(10.0, 20.0, 0.6);
        let (t0, t1) = curve.param_range();
        let split = Edge::new(
            curve.clone(),
            Vertex::from_point(curve.evaluate(t0)),
            Vertex::from_point(curve.evaluate(t1)),
            1e-6,
        );
        // 面積 = ∫ r dt * (z0 + slope * r cos t), t = 0..pi/2
        let lower = 10.0 * (20.0 * FRAC_PI_2 + 0.6 * 10.0);
        subjects.push(Subject {
            name: "cylinder quarter cut by a tilted plane",
            face,
            split,
            expected_piece: Some(lower),
        });
    }

    // 4. 同じ円柱の四半パッチを、傾きの向きを変えて割る。
    {
        let (_, face) = cylinder_quarter(10.0, 40.0);
        let curve = tilted_section(10.0, 25.0, -0.9);
        let (t0, t1) = curve.param_range();
        let split = Edge::new(
            curve.clone(),
            Vertex::from_point(curve.evaluate(t0)),
            Vertex::from_point(curve.evaluate(t1)),
            1e-6,
        );
        let lower = 10.0 * (25.0 * FRAC_PI_2 - 0.9 * 10.0);
        subjects.push(Subject {
            name: "cylinder quarter cut the other way",
            face,
            split,
            expected_piece: Some(lower),
        });
    }

    println!(
        "{:<44} {:>7} {:>14} {:>14} {:>12} {:>12} {:>10}",
        "subject", "pieces", "area sum", "original", "residual", "vs analytic", "on face"
    );
    println!("{}", "-".repeat(120));

    let mut clean = 0usize;
    let mut problems = 0usize;

    for subject in &subjects {
        match FaceSplitter::split_by_curve(&subject.face, &subject.split, &tol) {
            Ok((pieces, report)) => {
                // **判定はパラメータ面積**（4-76）。閉じた式と突き合わせるのは
                // 3D の面積なので、返った片からここで積む。
                let piece_areas = areas_3d(&pieces);
                let summed: f64 = piece_areas.iter().sum();
                let original_3d = MassCalculator::compute_face_integral(
                    &subject.face,
                    &TessellationParams::default(),
                )
                .0;
                let closest = |expected: f64| {
                    piece_areas
                        .iter()
                        .map(|area| (area - expected).abs() / expected.abs())
                        .fold(f64::INFINITY, f64::min)
                };
                let against = subject
                    .expected_piece
                    .map(|expected| format!("{:.2e}", closest(expected)))
                    .unwrap_or_else(|| "-".to_string());

                let bad = report.area_residual > 1e-6
                    || subject
                        .expected_piece
                        .map(|expected| closest(expected) > 1e-6)
                        .unwrap_or(false);
                if bad {
                    problems += 1;
                } else {
                    clean += 1;
                }

                println!(
                    "{:<44} {:>7} {:>14.6} {:>14.6} {:>12.2e} {:>12} {:>10.2e}",
                    subject.name,
                    pieces.len(),
                    summed,
                    original_3d,
                    report.area_residual,
                    against,
                    report.curve_off_surface
                );
            }
            Err(err) => {
                problems += 1;
                println!(
                    "{:<44} {:>7} {}",
                    subject.name,
                    "-",
                    err.chars().take(66).collect::<String>()
                );
            }
        }
    }

    println!("{}", "-".repeat(120));
    println!(
        "{clean} of {} splits are clean, {problems} with problems",
        subjects.len()
    );

    // 1枚に切り込みを複数入れる。互いに交わらない切り込みなら、1本ずつ
    // 当てていけば足りる。合否はやはり面積の和で見る。
    println!();
    println!(
        "{:<44} {:>7} {:>7} {:>14} {:>14} {:>12}",
        "several cuts on one face", "pieces", "cuts", "area sum", "original", "residual"
    );
    println!("{}", "-".repeat(102));

    let mut multi_clean = 0usize;
    let mut multi_problems = 0usize;

    let multi: Vec<(&str, Face, Vec<Edge>)> = vec![
        (
            "plane 10x10 cut into three by two lines",
            planar_square(10.0),
            vec![
                Edge::line_between(
                    Vertex::from_point(Point3::new(0.0, 3.0, 0.0)),
                    Vertex::from_point(Point3::new(10.0, 3.0, 0.0)),
                )
                .unwrap(),
                Edge::line_between(
                    Vertex::from_point(Point3::new(0.0, 7.0, 0.0)),
                    Vertex::from_point(Point3::new(10.0, 7.0, 0.0)),
                )
                .unwrap(),
            ],
        ),
        (
            "cylinder quarter cut into three by two sections",
            cylinder_quarter(10.0, 40.0).1,
            vec![
                {
                    let curve = tilted_section(10.0, 12.0, 0.4);
                    let (a, b) = curve.param_range();
                    Edge::new(
                        curve.clone(),
                        Vertex::from_point(curve.evaluate(a)),
                        Vertex::from_point(curve.evaluate(b)),
                        1e-6,
                    )
                },
                {
                    let curve = tilted_section(10.0, 28.0, -0.5);
                    let (a, b) = curve.param_range();
                    Edge::new(
                        curve.clone(),
                        Vertex::from_point(curve.evaluate(a)),
                        Vertex::from_point(curve.evaluate(b)),
                        1e-6,
                    )
                },
            ],
        ),
    ];

    for (name, face, cuts) in &multi {
        match FaceSplitter::split_by_curves(face, cuts, &tol) {
            Ok((pieces, report)) => {
                let summed: f64 = areas_3d(&pieces).iter().sum();
                let original_3d =
                    MassCalculator::compute_face_integral(face, &TessellationParams::default()).0;
                let bad = report.area_residual > 1e-6 || report.cuts_refused > 0;
                if bad {
                    multi_problems += 1;
                } else {
                    multi_clean += 1;
                }
                println!(
                    "{:<44} {:>7} {:>7} {:>14.6} {:>14.6} {:>12.2e}",
                    name,
                    pieces.len(),
                    report.cuts_applied,
                    summed,
                    original_3d,
                    report.area_residual
                );
                for reason in report.refusals.iter().take(2) {
                    println!("{:>46}{}", "", reason.chars().take(58).collect::<String>());
                }
            }
            Err(err) => {
                multi_problems += 1;
                println!("{:<44} {}", name, err.chars().take(58).collect::<String>());
            }
        }
    }
    println!("{}", "-".repeat(102));
    println!(
        "{multi_clean} of {} multi-cut faces are clean, {multi_problems} with problems",
        multi.len()
    );

}
