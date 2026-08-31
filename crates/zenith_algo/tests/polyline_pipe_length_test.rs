//! 角丸めポリラインに沿ったパイプの体積が、`断面積 x 経路長` に乗るか。
//!
//! 断面の重心が経路の上にあり経路に垂直なら、経路が曲がっていても
//! `V = A x L` はきっかり成り立つ。角丸めしたパスの長さは閉じた式で出る:
//! 直線の和から各コーナーの `2 r tan(theta/2)` を引き、円弧 `r theta` を足す。
//!
//! 以前はここが 3.9e-4 ずれていました。セグメント列を密に標本して**1次の
//! 折れ線**にしていたので、円弧が弦に落ち、芯線が真のパスより短くなります。
//! 誤差の向きは決まっていて（必ず内側に切る）、掃引をいくら細かくしても
//! 消えません。いまはセグメントをそのまま1本の曲線に繋いでいます。

use std::f64::consts::PI;

use zenith_algo::{MassCalculator, PolylineBuilder};
use zenith_math::{Point3, Tolerance};
use zenith_tess::TessellationParams;

/// 角丸めしたパスの長さ。
fn filleted_path_length(points: &[Point3], corner_radius: f64) -> f64 {
    let mut total = 0.0;
    for index in 0..points.len() - 1 {
        total += (points[index + 1] - points[index]).norm();
    }
    for index in 1..points.len() - 1 {
        let incoming = (points[index] - points[index - 1]).normalize();
        let outgoing = (points[index + 1] - points[index]).normalize();
        let turn = incoming.dot(&outgoing).clamp(-1.0, 1.0).acos();
        if turn < 1e-4 {
            continue;
        }
        total -= 2.0 * corner_radius * (turn / 2.0).tan();
        total += corner_radius * turn;
    }
    total
}

fn volume_of(solid: &zenith_topo::Solid) -> f64 {
    MassCalculator::compute_from_brep(
        solid,
        &TessellationParams {
            u_divisions: 96,
            v_divisions: 96,
        },
    )
    .volume
}

#[test]
fn a_pipe_along_a_filleted_polyline_has_the_volume_its_path_length_implies() {
    let tol = Tolerance::default();

    let cases: Vec<(&str, Vec<Point3>, f64, f64)> = vec![
        (
            "a straight run",
            vec![Point3::new(0.0, 0.0, 0.0), Point3::new(50.0, 0.0, 0.0)],
            3.0,
            0.0,
        ),
        (
            "one right-angle bend",
            vec![
                Point3::new(0.0, 0.0, 0.0),
                Point3::new(50.0, 0.0, 0.0),
                Point3::new(50.0, 40.0, 0.0),
            ],
            3.0,
            10.0,
        ),
        (
            "two bends the same way round",
            vec![
                Point3::new(0.0, 0.0, 0.0),
                Point3::new(40.0, 0.0, 0.0),
                Point3::new(40.0, 30.0, 0.0),
                Point3::new(80.0, 30.0, 0.0),
            ],
            2.5,
            8.0,
        ),
        (
            "a bend out of the plane",
            vec![
                Point3::new(0.0, 0.0, 0.0),
                Point3::new(40.0, 0.0, 0.0),
                Point3::new(40.0, 0.0, 35.0),
            ],
            3.0,
            6.0,
        ),
    ];

    for (name, points, radius, corner) in &cases {
        let solid = PolylineBuilder::sweep_pipe_polyline(points, *radius, *corner, &tol)
            .unwrap_or_else(|err| panic!("{name}: {err}"));

        let report = solid.outer_shell.validate_closed(&tol);
        assert!(
            report.is_valid(),
            "{name} is not a closed shell: {:?}",
            report.errors
        );

        let expected = PI * radius * radius * filleted_path_length(points, *corner);
        let volume = volume_of(&solid);
        assert!(
            (volume - expected).abs() / expected < 1e-5,
            "{name}: volume {volume} against the path length's {expected}"
        );
    }
}

/// 直線だけのパスには近似の余地が無い。
#[test]
fn a_straight_pipe_is_exact() {
    let tol = Tolerance::default();
    let points = vec![Point3::new(0.0, 0.0, 0.0), Point3::new(50.0, 0.0, 0.0)];
    let solid = PolylineBuilder::sweep_pipe_polyline(&points, 3.0, 0.0, &tol).expect("pipe");
    let expected = PI * 9.0 * 50.0;
    let volume = volume_of(&solid);
    assert!(
        (volume - expected).abs() / expected < 1e-11,
        "a straight pipe is {volume}, not {expected}"
    );
}

/// 角丸めの半径を大きくすると、パスは短くなる（角を斜めに切るため）。
/// パイプの体積もそれに従うこと。方向を間違えていれば、ここで気づく。
#[test]
fn a_larger_corner_radius_shortens_the_path_and_the_pipe() {
    let tol = Tolerance::default();
    let points = vec![
        Point3::new(0.0, 0.0, 0.0),
        Point3::new(50.0, 0.0, 0.0),
        Point3::new(50.0, 40.0, 0.0),
    ];

    let mut previous = f64::INFINITY;
    for corner in [2.0, 6.0, 12.0] {
        let solid = PolylineBuilder::sweep_pipe_polyline(&points, 3.0, corner, &tol).expect("pipe");
        let volume = volume_of(&solid);
        let expected = PI * 9.0 * filleted_path_length(&points, corner);
        assert!(
            (volume - expected).abs() / expected < 1e-5,
            "corner {corner}: {volume} against {expected}"
        );
        assert!(
            volume < previous,
            "a larger fillet should shorten the pipe: {volume} after {previous}"
        );
        previous = volume;
    }
}
