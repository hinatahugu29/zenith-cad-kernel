//! 曲面シートに厚みを与えたとき、体積が閉じた式に乗るか。
//!
//! 半径 `r` の円柱の四半パッチ（高さ `h`）を外へ `t` だけ厚くすると、
//! 体積は `(pi/4)((r+t)^2 - r^2) h` になる。
//!
//! 以前はここが**そもそも通りませんでした**。天面は隅1点の法線で全制御点を
//! ずらしており、法線が場所ごとに変わる面ではただの平行移動になります。縁も
//! 4隅を直線で結んでおり、弧の縁からは外れます。シェル検証が「境界点が面から
//! 2.93 外れている」と弾いていたので、誤った立体が出回ることはありません
//! でしたが、曲面シートは作れていませんでした。

use std::f64::consts::{FRAC_1_SQRT_2, FRAC_PI_4};

use zenith_algo::{MassCalculator, ThickenBuilder};
use zenith_geom::{ControlPoint3, KnotVector, NurbsCurve3, NurbsSurface3, PlaneSurface3};
use zenith_math::{Point3, Tolerance, Vec3};
use zenith_tess::TessellationParams;
use zenith_topo::{Edge, Face, FaceGeometry, Orientation, OrientedEdge, Vertex, Wire};

fn cylinder_quarter(r: f64, h: f64) -> Face {
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
    let top_start = Vertex::from_point(Point3::new(r, 0.0, h));
    let top_end = Vertex::from_point(Point3::new(0.0, r, h));
    Face::new(
        FaceGeometry::Nurbs(surface),
        Wire::new(vec![
            OrientedEdge::forward(Edge::new(
                arc(0.0),
                bottom_start.clone(),
                bottom_end.clone(),
                1e-6,
            )),
            OrientedEdge::forward(Edge::line_between(bottom_end.clone(), top_end.clone()).unwrap()),
            OrientedEdge::reversed(Edge::new(arc(h), top_start.clone(), top_end.clone(), 1e-6)),
            OrientedEdge::reversed(
                Edge::line_between(bottom_start.clone(), top_start.clone()).unwrap(),
            ),
        ]),
        Vec::new(),
        Orientation::Forward,
        1e-6,
    )
}

fn planar_square(side: f64) -> Face {
    let plane = PlaneSurface3::new(
        Point3::new(0.0, 0.0, 0.0),
        Vec3::new(1.0, 0.0, 0.0),
        Vec3::new(0.0, 1.0, 0.0),
    )
    .unwrap();
    let corners = [
        Point3::new(0.0, 0.0, 0.0),
        Point3::new(side, 0.0, 0.0),
        Point3::new(side, side, 0.0),
        Point3::new(0.0, side, 0.0),
    ];
    let vertices: Vec<Vertex> = corners.into_iter().map(Vertex::from_point).collect();
    Face::simple(
        FaceGeometry::Plane(plane),
        Wire::new(
            (0..4)
                .map(|index| {
                    OrientedEdge::forward(
                        Edge::line_between(
                            vertices[index].clone(),
                            vertices[(index + 1) % 4].clone(),
                        )
                        .unwrap(),
                    )
                })
                .collect(),
        ),
    )
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
fn a_flat_sheet_thickens_to_exactly_its_area_times_its_thickness() {
    let tol = Tolerance::default();
    for (side, thickness) in [(10.0, 1.0), (10.0, 0.1), (7.0, 2.5)] {
        let solid = ThickenBuilder::thicken_face(&planar_square(side), thickness, &tol)
            .unwrap_or_else(|err| panic!("a flat sheet {side} by {thickness}: {err}"));
        let expected = side * side * thickness;
        let volume = volume_of(&solid);
        assert!(
            (volume - expected).abs() / expected < 1e-12,
            "a flat sheet has no curvature to approximate: {volume} against {expected}"
        );
    }
}

#[test]
fn a_curved_sheet_thickens_to_the_shell_between_two_radii() {
    let tol = Tolerance::default();
    for (radius, height, thickness) in [(10.0, 20.0, 1.0), (10.0, 20.0, 0.2), (6.0, 5.0, 2.0)] {
        let solid =
            ThickenBuilder::thicken_face(&cylinder_quarter(radius, height), thickness, &tol)
                .unwrap_or_else(|err| panic!("a cylinder quarter r{radius} t{thickness}: {err}"));

        let report = solid.outer_shell.validate_closed(&tol);
        assert!(
            report.is_valid(),
            "the thickened sheet is not a closed shell: {:?}",
            report.errors
        );

        // 外へ厚くしたので、半径 r から r + t までの四半シェル。
        let expected = FRAC_PI_4 * ((radius + thickness).powi(2) - radius * radius) * height;
        let volume = volume_of(&solid);
        assert!(
            (volume - expected).abs() / expected < 1e-4,
            "r{radius} t{thickness}: volume {volume} against {expected}"
        );
    }
}

/// 標本を細かくすれば、真の値へ寄ること。
///
/// 厳密なオフセット曲面は NURBS では表せないので、ここは必ず近似になる。
/// 近似であることと、近似が細かさで縮むことは別なので、後者を測る。
#[test]
fn refining_the_offset_sampling_converges_on_the_closed_form() {
    let tol = Tolerance::default();
    let (radius, height, thickness): (f64, f64, f64) = (10.0, 20.0, 1.0);
    let expected = FRAC_PI_4 * ((radius + thickness).powi(2) - radius * radius) * height;

    let error_at = |samples: usize| -> f64 {
        let solid = ThickenBuilder::thicken_face_with_samples(
            &cylinder_quarter(radius, height),
            thickness,
            samples,
            &tol,
        )
        .expect("thickened sheet");
        (volume_of(&solid) - expected).abs() / expected
    };

    let coarse = error_at(8);
    let fine = error_at(16);
    assert!(
        fine < coarse,
        "refining should help: 8 samples {coarse:.3e}, 16 samples {fine:.3e}"
    );
    // 3次の補間なので、細かさを2倍にすればおよそ16分の1になる。
    let ratio = coarse / fine;
    assert!(
        (6.0..40.0).contains(&ratio),
        "doubling the samples should divide the error by about 16, got {ratio:.1}"
    );
}
