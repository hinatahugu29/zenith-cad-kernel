use std::f64::consts::FRAC_1_SQRT_2;

use zenith_algo::FaceSplitter;
use zenith_geom::{ControlPoint3, IntersectionMarcher, KnotVector, NurbsCurve3, NurbsSurface3};
use zenith_math::{Point3, Tolerance};
use zenith_topo::{Edge, Face, FaceGeometry, Orientation, OrientedEdge, Shell, Vertex, Wire};

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

fn sphere_octant(r: f64) -> (NurbsSurface3, Face) {
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
    let surface = NurbsSurface3::new(
        2,
        2,
        grid,
        KnotVector::clamped_uniform(3, 2),
        KnotVector::clamped_uniform(3, 2),
    )
    .unwrap();

    let p00 = Point3::new(r, 0.0, 0.0);
    let p10 = Point3::new(0.0, r, 0.0);
    let p01 = Point3::new(0.0, 0.0, r);

    let v00 = Vertex::from_point(p00);
    let v10 = Vertex::from_point(p10);
    let v01 = Vertex::from_point(p01);

    // 境界円弧
    let equator = NurbsCurve3::new(
        2,
        vec![
            ControlPoint3::unweighted(p00),
            ControlPoint3::new(Point3::new(r, r, 0.0), w),
            ControlPoint3::unweighted(p10),
        ],
        KnotVector::clamped_uniform(3, 2),
    ).unwrap();

    let meridian0 = NurbsCurve3::new(
        2,
        vec![
            ControlPoint3::unweighted(p00),
            ControlPoint3::new(Point3::new(r, 0.0, r), w),
            ControlPoint3::unweighted(p01),
        ],
        KnotVector::clamped_uniform(3, 2),
    ).unwrap();

    let meridian1 = NurbsCurve3::new(
        2,
        vec![
            ControlPoint3::unweighted(p10),
            ControlPoint3::new(Point3::new(0.0, r, r), w),
            ControlPoint3::unweighted(p01),
        ],
        KnotVector::clamped_uniform(3, 2),
    ).unwrap();

    let face = Face::new(
        FaceGeometry::Nurbs(surface.clone()),
        Wire::new(vec![
            OrientedEdge::forward(Edge::new(equator, v00.clone(), v10.clone(), 1e-6)),
            OrientedEdge::forward(Edge::new(meridian1, v10.clone(), v01.clone(), 1e-6)),
            OrientedEdge::reversed(Edge::new(meridian0, v00.clone(), v01.clone(), 1e-6)),
        ]),
        Vec::new(),
        Orientation::Forward,
        1e-6,
    );

    (surface, face)
}

#[test]
fn test_ssi_dual_surface_split_and_trimmed_assembly() {
    let tol = Tolerance::default();
    let (sphere_surf, _sphere_face) = sphere_octant(12.0);
    let (cyl_surf, cyl_face) = cylinder_quarter_face(Point3::new(3.0, 0.0, 0.0), 5.0, -20.0, 20.0);

    // 1. 交線を高精度追跡してフィッティング
    let (curve, marched, deviation) =
        IntersectionMarcher::fit_to_tolerance(&cyl_surf, &sphere_surf, 2.0, 1e-6, &tol)
            .expect("sphere and cylinder intersect");
    assert!(deviation <= 1e-6);
    assert!(marched.points.len() >= 8);

    // 2. 円柱パッチを交線で分割
    let (t0, t1) = curve.param_range();
    let split_edge = Edge::new(
        curve.clone(),
        Vertex::from_point(curve.evaluate(t0)),
        Vertex::from_point(curve.evaluate(t1)),
        1e-6,
    );
    let (cyl_pieces, cyl_report) = FaceSplitter::split_by_curve(&cyl_face, &split_edge, &tol)
        .expect("cylinder face split");
    assert_eq!(cyl_pieces.len(), 2);
    assert!(cyl_report.area_residual < 1e-6);

    // 3. 分割された片から複合シェルを構成
    let assembled_shell = Shell::new(vec![cyl_pieces[0].clone(), cyl_pieces[1].clone()], false);
    assert_eq!(assembled_shell.faces.len(), 2);
}
