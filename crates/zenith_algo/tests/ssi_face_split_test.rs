//! 曲面同士の交線で、実際に面を割るところまでを通しで確かめる。
//!
//! 引継書の 3-1 が言う壁は、交線が取れないことではなく**分割段**だった。
//! この試験はその鎖を端から端まで通す。
//!
//! ```text
//! 交線を辿る -> 1本の曲線に当てはめる -> その曲線で面を割る -> 面積を足す
//! ```
//!
//! 合否は最後の足し算で見る。閉じたワイヤになったことも、曲線が両方の曲面に
//! 乗っていることも、領域を取り違えていないことの証拠にはならない。

use std::f64::consts::FRAC_1_SQRT_2;

use zenith_algo::FaceSplitter;
use zenith_geom::{ControlPoint3, IntersectionMarcher, KnotVector, NurbsCurve3, NurbsSurface3};
use zenith_math::{Point3, Tolerance};
use zenith_topo::{Edge, Face, FaceGeometry, OrientedEdge, Orientation, Vertex, Wire};

/// z 軸まわり、中心 `centre`、半径 `r`、高さ `z_low..z_high` の円柱の
/// 四半パッチと、その境界ワイヤ。
fn cylinder_quarter_face(centre: Point3, r: f64, z_low: f64, z_high: f64) -> (NurbsSurface3, Face) {
    let w = FRAC_1_SQRT_2;
    let ring = [
        (Point3::new(centre.x + r, centre.y, 0.0), 1.0),
        (Point3::new(centre.x + r, centre.y + r, 0.0), w),
        (Point3::new(centre.x, centre.y + r, 0.0), 1.0),
    ];
    let grid: Vec<Vec<ControlPoint3>> = ring
        .iter()
        .map(|(point, weight)| {
            vec![
                ControlPoint3::new(Point3::new(point.x, point.y, z_low), *weight),
                ControlPoint3::new(Point3::new(point.x, point.y, z_high), *weight),
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
                ControlPoint3::unweighted(Point3::new(centre.x + r, centre.y, z)),
                ControlPoint3::new(Point3::new(centre.x + r, centre.y + r, z), w),
                ControlPoint3::unweighted(Point3::new(centre.x, centre.y + r, z)),
            ],
            KnotVector::clamped_uniform(3, 2),
        )
        .unwrap()
    };
    let bottom_start = Vertex::from_point(Point3::new(centre.x + r, centre.y, z_low));
    let bottom_end = Vertex::from_point(Point3::new(centre.x, centre.y + r, z_low));
    let top_start = Vertex::from_point(Point3::new(centre.x + r, centre.y, z_high));
    let top_end = Vertex::from_point(Point3::new(centre.x, centre.y + r, z_high));

    let face = Face::new(
        FaceGeometry::Nurbs(surface.clone()),
        Wire::new(vec![
            OrientedEdge::forward(Edge::new(
                arc(z_low),
                bottom_start.clone(),
                bottom_end.clone(),
                1e-6,
            )),
            OrientedEdge::forward(Edge::line_between(bottom_end.clone(), top_end.clone()).unwrap()),
            OrientedEdge::reversed(Edge::new(
                arc(z_high),
                top_start.clone(),
                top_end.clone(),
                1e-6,
            )),
            OrientedEdge::reversed(
                Edge::line_between(bottom_start.clone(), top_start.clone()).unwrap(),
            ),
        ]),
        Vec::new(),
        Orientation::Forward,
        1e-6,
    );
    (surface, face)
}

/// 原点中心、半径 `r` の球の第1象限。
fn sphere_octant(r: f64) -> NurbsSurface3 {
    let w = FRAC_1_SQRT_2;
    let rows = [(r, 0.0, 1.0), (r, r, w), (0.0, r, 1.0)];
    let grid: Vec<Vec<ControlPoint3>> = rows
        .iter()
        .map(|(radial, height, weight)| {
            vec![
                ControlPoint3::new(Point3::new(*radial, 0.0, *height), *weight),
                ControlPoint3::new(Point3::new(*radial, *radial, *height), weight * w),
                ControlPoint3::new(Point3::new(0.0, *radial, *height), *weight),
            ]
        })
        .collect();
    NurbsSurface3::new(
        2,
        2,
        grid,
        KnotVector::clamped_uniform(3, 2),
        KnotVector::clamped_uniform(3, 2),
    )
    .unwrap()
}

/// 球と、軸を外した円柱の交線で、円柱の四半パッチを割る。
///
/// 交線は `z = sqrt(110 - 30 cos t)` を辿るので、円柱のパラメータ線ではない。
/// 両端は θ = 0 と θ = 90 度、つまりパッチの左右の境界に着く。
#[test]
fn a_cylinder_patch_splits_along_its_intersection_with_a_sphere() {
    let tol = Tolerance::default();
    let sphere = sphere_octant(12.0);
    let (cylinder, face) =
        cylinder_quarter_face(Point3::new(3.0, 0.0, 0.0), 5.0, -20.0, 20.0);

    // 1. 交線を辿り、要求した精度で1本の曲線にする。
    let (curve, marched, deviation) =
        IntersectionMarcher::fit_to_tolerance(&cylinder, &sphere, 2.0, 1e-6, &tol)
            .expect("the sphere and the cylinder do meet");
    assert!(
        deviation <= 1e-6,
        "the fitted curve is {deviation:.3e} off the surfaces"
    );
    assert!(marched.points.len() > 8);

    // 辿った点が本当に両方の上にあるか、閉じた式で確かめる。
    // 円柱: (x - 3)^2 + y^2 = 25、球: x^2 + y^2 + z^2 = 144。
    for sample in &marched.points {
        let p = sample.point;
        let on_cylinder = ((p.x - 3.0).powi(2) + p.y * p.y).sqrt() - 5.0;
        let on_sphere = (p.x * p.x + p.y * p.y + p.z * p.z).sqrt() - 12.0;
        assert!(
            on_cylinder.abs() < 1e-9 && on_sphere.abs() < 1e-9,
            "a marched point is off the closed forms by {on_cylinder:.3e} and {on_sphere:.3e}"
        );
    }

    // 2. その曲線で面を割る。
    let (t0, t1) = curve.param_range();
    let split = Edge::new(
        curve.clone(),
        Vertex::from_point(curve.evaluate(t0)),
        Vertex::from_point(curve.evaluate(t1)),
        1e-6,
    );
    let (pieces, report) = FaceSplitter::split_by_curve(&face, &split, &tol)
        .expect("the intersection curve should split the patch");

    assert_eq!(pieces.len(), 2);
    for piece in &pieces {
        assert!(piece.outer_wire.is_closed(&tol));
    }

    // 3. 面積を足す。ここが合わなければ領域を取り違えている。
    assert!(
        report.area_residual < 1e-6,
        "the pieces do not add up: residual {:.3e} (pieces {:?}, original {})",
        report.area_residual,
        report.piece_areas,
        report.original_area
    );

    // 元の四半パッチの面積には閉じた式がある: r * (pi/2) * 高さ。
    let whole = 5.0 * std::f64::consts::FRAC_PI_2 * 40.0;
    assert!(
        (report.original_area - whole).abs() / whole < 1e-9,
        "the patch area should be {whole}, got {}",
        report.original_area
    );
    // どちらの片も潰れていないこと。片方が 0 でも「和は合う」ので、別に見る。
    for area in &report.piece_areas {
        assert!(
            *area > whole * 0.05,
            "a piece came out empty: {:?}",
            report.piece_areas
        );
    }
}
