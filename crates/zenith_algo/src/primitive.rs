use zenith_geom::{ControlPoint3, KnotVector, NurbsCurve3, NurbsSurface3, PlaneSurface3};
use zenith_math::{Point3, Vec3};
use zenith_topo::{Edge, Face, FaceGeometry, OrientedEdge, Shell, Solid, Vertex, Wire};

/// 基本幾何プリミティブ生成ビルダー（Box, Cylinder, Sphere, Cone）
pub struct PrimitiveBuilder;

impl PrimitiveBuilder {
    /// 直方体ソリッド（Box）の生成（外向き法線・閉マニホールドB-Rep）
    pub fn make_box(dx: f64, dy: f64, dz: f64) -> Result<Solid, String> {
        if dx <= 1e-9 || dy <= 1e-9 || dz <= 1e-9 {
            return Err(format!(
                "Box dimensions must be positive, got ({dx}, {dy}, {dz})"
            ));
        }

        let p0 = Point3::new(0.0, 0.0, 0.0);
        let p1 = Point3::new(dx, 0.0, 0.0);
        let p2 = Point3::new(dx, dy, 0.0);
        let p3 = Point3::new(0.0, dy, 0.0);

        let p4 = Point3::new(0.0, 0.0, dz);
        let p5 = Point3::new(dx, 0.0, dz);
        let p6 = Point3::new(dx, dy, dz);
        let p7 = Point3::new(0.0, dy, dz);

        let v = [
            Vertex::from_point(p0),
            Vertex::from_point(p1),
            Vertex::from_point(p2),
            Vertex::from_point(p3),
            Vertex::from_point(p4),
            Vertex::from_point(p5),
            Vertex::from_point(p6),
            Vertex::from_point(p7),
        ];

        // 12本のエッジ生成
        let e01 = Edge::line_between(v[0].clone(), v[1].clone())?;
        let e12 = Edge::line_between(v[1].clone(), v[2].clone())?;
        let e23 = Edge::line_between(v[2].clone(), v[3].clone())?;
        let e30 = Edge::line_between(v[3].clone(), v[0].clone())?;

        let e45 = Edge::line_between(v[4].clone(), v[5].clone())?;
        let e56 = Edge::line_between(v[5].clone(), v[6].clone())?;
        let e67 = Edge::line_between(v[6].clone(), v[7].clone())?;
        let e74 = Edge::line_between(v[7].clone(), v[4].clone())?;

        let e04 = Edge::line_between(v[0].clone(), v[4].clone())?;
        let e15 = Edge::line_between(v[1].clone(), v[5].clone())?;
        let e26 = Edge::line_between(v[2].clone(), v[6].clone())?;
        let e37 = Edge::line_between(v[3].clone(), v[7].clone())?;

        let make_plane_face = |origin: Point3,
                               u: Vec3,
                               v_axis: Vec3,
                               edges: Vec<OrientedEdge>|
         -> Result<Face, String> {
            let plane = PlaneSurface3::new(origin, u, v_axis).ok_or("Failed to create plane")?;
            let wire = Wire::new(edges);
            Ok(Face::simple(FaceGeometry::Plane(plane), wire))
        };

        // 1. Bottom (-Z) : v0 -> v3 -> v2 -> v1
        let f_bottom = make_plane_face(
            p0,
            Vec3::new(0.0, dy, 0.0),
            Vec3::new(dx, 0.0, 0.0),
            vec![
                OrientedEdge::reversed(e30.clone()),
                OrientedEdge::reversed(e23.clone()),
                OrientedEdge::reversed(e12.clone()),
                OrientedEdge::reversed(e01.clone()),
            ],
        )?;

        // 2. Top (+Z) : v4 -> v5 -> v6 -> v7
        let f_top = make_plane_face(
            p4,
            Vec3::new(dx, 0.0, 0.0),
            Vec3::new(0.0, dy, 0.0),
            vec![
                OrientedEdge::forward(e45.clone()),
                OrientedEdge::forward(e56.clone()),
                OrientedEdge::forward(e67.clone()),
                OrientedEdge::forward(e74.clone()),
            ],
        )?;

        // 3. Front (-Y) : v0 -> v1 -> v5 -> v4
        let f_front = make_plane_face(
            p0,
            Vec3::new(dx, 0.0, 0.0),
            Vec3::new(0.0, 0.0, dz),
            vec![
                OrientedEdge::forward(e01.clone()),
                OrientedEdge::forward(e15.clone()),
                OrientedEdge::reversed(e45.clone()),
                OrientedEdge::reversed(e04.clone()),
            ],
        )?;

        // 4. Back (+Y) : v3 -> v7 -> v6 -> v2
        let f_back = make_plane_face(
            p3,
            Vec3::new(0.0, 0.0, dz),
            Vec3::new(dx, 0.0, 0.0),
            vec![
                OrientedEdge::forward(e37.clone()),
                OrientedEdge::reversed(e67.clone()),
                OrientedEdge::reversed(e26.clone()),
                OrientedEdge::forward(e23.clone()),
            ],
        )?;

        // 5. Left (-X) : v0 -> v4 -> v7 -> v3
        let f_left = make_plane_face(
            p0,
            Vec3::new(0.0, 0.0, dz),
            Vec3::new(0.0, dy, 0.0),
            vec![
                OrientedEdge::forward(e04.clone()),
                OrientedEdge::reversed(e74.clone()),
                OrientedEdge::reversed(e37.clone()),
                OrientedEdge::forward(e30.clone()),
            ],
        )?;

        // 6. Right (+X) : v1 -> v2 -> v6 -> v5
        let f_right = make_plane_face(
            p1,
            Vec3::new(0.0, dy, 0.0),
            Vec3::new(0.0, 0.0, dz),
            vec![
                OrientedEdge::forward(e12.clone()),
                OrientedEdge::forward(e26.clone()),
                OrientedEdge::reversed(e56.clone()),
                OrientedEdge::reversed(e15.clone()),
            ],
        )?;

        let shell = Shell::closed(vec![f_bottom, f_top, f_front, f_back, f_left, f_right]);
        crate::validated_solid(shell)
    }

    /// 円柱ソリッド（Cylinder: 半径 r, 高さ h）の生成（4分割有理NURBS円筒側面 + 上下2端面）
    pub fn make_cylinder(radius: f64, height: f64) -> Result<Solid, String> {
        if radius <= 1e-6 || height <= 1e-6 {
            return Err("Radius and height must be positive".to_string());
        }

        let weight = std::f64::consts::FRAC_1_SQRT_2;
        let r = radius;
        let h = height;

        // 4頂点 (底面) & 4頂点 (天面)
        let pb = [
            Point3::new(r, 0.0, 0.0),
            Point3::new(0.0, r, 0.0),
            Point3::new(-r, 0.0, 0.0),
            Point3::new(0.0, -r, 0.0),
        ];
        let pt = [
            Point3::new(r, 0.0, h),
            Point3::new(0.0, r, h),
            Point3::new(-r, 0.0, h),
            Point3::new(0.0, -r, h),
        ];

        let vb: Vec<Vertex> = pb.iter().map(|p| Vertex::from_point(*p)).collect();
        let vt: Vec<Vertex> = pt.iter().map(|p| Vertex::from_point(*p)).collect();

        // 4本の垂直エッジ
        let mut ev = Vec::with_capacity(4);
        for i in 0..4 {
            ev.push(Edge::line_between(vb[i].clone(), vt[i].clone())?);
        }

        // 4本の底面円弧 & 4本の天面円弧
        let mut eb = Vec::with_capacity(4);
        let mut et = Vec::with_capacity(4);
        let mut faces = Vec::with_capacity(6);

        for i in 0..4 {
            let next = (i + 1) % 4;
            let corner_b = match i {
                0 => Point3::new(r, r, 0.0),
                1 => Point3::new(-r, r, 0.0),
                2 => Point3::new(-r, -r, 0.0),
                _ => Point3::new(r, -r, 0.0),
            };
            let corner_t = Point3::new(corner_b.x, corner_b.y, h);

            let arc_b = Edge::new(
                NurbsCurve3::new(
                    2,
                    vec![
                        ControlPoint3::unweighted(pb[i]),
                        ControlPoint3::new(corner_b, weight),
                        ControlPoint3::unweighted(pb[next]),
                    ],
                    KnotVector::clamped_uniform(3, 2),
                )?,
                vb[i].clone(),
                vb[next].clone(),
                1e-6,
            );

            let arc_t = Edge::new(
                NurbsCurve3::new(
                    2,
                    vec![
                        ControlPoint3::unweighted(pt[i]),
                        ControlPoint3::new(corner_t, weight),
                        ControlPoint3::unweighted(pt[next]),
                    ],
                    KnotVector::clamped_uniform(3, 2),
                )?,
                vt[i].clone(),
                vt[next].clone(),
                1e-6,
            );

            eb.push(arc_b.clone());
            et.push(arc_t.clone());

            // 側面有理NURBSパッチ
            let row0 = vec![
                ControlPoint3::unweighted(pb[i]),
                ControlPoint3::unweighted(pt[i]),
            ];
            let row1 = vec![
                ControlPoint3::new(corner_b, weight),
                ControlPoint3::new(corner_t, weight),
            ];
            let row2 = vec![
                ControlPoint3::unweighted(pb[next]),
                ControlPoint3::unweighted(pt[next]),
            ];

            let surf = NurbsSurface3::new(
                2,
                1,
                vec![row0, row1, row2],
                KnotVector::clamped_uniform(3, 2),
                KnotVector::clamped_uniform(2, 1),
            )?;

            let wire = Wire::new(vec![
                OrientedEdge::forward(arc_b),
                OrientedEdge::forward(ev[next].clone()),
                OrientedEdge::reversed(arc_t),
                OrientedEdge::reversed(ev[i].clone()),
            ]);
            faces.push(Face::simple(FaceGeometry::Nurbs(surf), wire));
        }

        // 底面 (-Z PLANE)
        let p_bot = PlaneSurface3::new(
            Point3::new(0.0, 0.0, 0.0),
            Vec3::new(0.0, 1.0, 0.0),
            Vec3::new(1.0, 0.0, 0.0),
        )
        .ok_or("plane bot")?;
        let wire_bot = Wire::new(vec![
            OrientedEdge::reversed(eb[3].clone()),
            OrientedEdge::reversed(eb[2].clone()),
            OrientedEdge::reversed(eb[1].clone()),
            OrientedEdge::reversed(eb[0].clone()),
        ]);
        faces.push(Face::simple(FaceGeometry::Plane(p_bot), wire_bot));

        // 天面 (+Z PLANE)
        let p_top = PlaneSurface3::new(
            Point3::new(0.0, 0.0, h),
            Vec3::new(1.0, 0.0, 0.0),
            Vec3::new(0.0, 1.0, 0.0),
        )
        .ok_or("plane top")?;
        let wire_top = Wire::new(vec![
            OrientedEdge::forward(et[0].clone()),
            OrientedEdge::forward(et[1].clone()),
            OrientedEdge::forward(et[2].clone()),
            OrientedEdge::forward(et[3].clone()),
        ]);
        faces.push(Face::simple(FaceGeometry::Plane(p_top), wire_top));

        let shell = Shell::closed(faces);
        crate::validated_solid(shell)
    }

    /// 球体ソリッド（Sphere: 半径 r, 中心原点）の生成（有理NURBS 8分割パッチ）
    pub fn make_sphere(radius: f64) -> Result<Solid, String> {
        if radius <= 1e-6 {
            return Err("Radius must be positive".to_string());
        }

        // 半円有理NURBSプロファイルをZ軸まわりに360度回転
        let tol = zenith_math::Tolerance::default();
        let r = radius;
        let weight = std::f64::consts::FRAC_1_SQRT_2;

        // 北極 (0, 0, r) -> 赤道 (r, 0, 0) -> 南極 (0, 0, -r)
        let p_north = Point3::new(0.0, 0.0, r);
        let p_eq_corner1 = Point3::new(r, 0.0, r);
        let p_eq = Point3::new(r, 0.0, 0.0);
        let p_eq_corner2 = Point3::new(r, 0.0, -r);
        let p_south = Point3::new(0.0, 0.0, -r);

        let profile = NurbsCurve3::new(
            2,
            vec![
                ControlPoint3::unweighted(p_north),
                ControlPoint3::new(p_eq_corner1, weight),
                ControlPoint3::unweighted(p_eq),
                ControlPoint3::new(p_eq_corner2, weight),
                ControlPoint3::unweighted(p_south),
            ],
            KnotVector::new(vec![0.0, 0.0, 0.0, 0.5, 0.5, 1.0, 1.0, 1.0]),
        )?;

        let surf = crate::revolve::RevolveBuilder::revolve_curve(
            &profile,
            Point3::new(0.0, 0.0, 0.0),
            Vec3::new(0.0, 0.0, 1.0),
            std::f64::consts::PI * 2.0,
            &tol,
        )?;

        let _ = (
            surf,
            profile,
            p_north,
            p_eq,
            p_eq_corner1,
            p_eq_corner2,
            p_south,
            weight,
            tol,
        );
        Self::make_sphere_patches(radius)
    }

    /// 球を 4 x 2 = 8 枚の有理双2次パッチとして構築する。
    ///
    /// トーラスと同じ理由で、自分自身に巻き付く単一面表現は相互運用できない。
    /// 球は極を持つので、極側の1行が1点に潰れた退化パッチになり、その境界は
    /// 子午線2本と赤道円弧1本の3辺で閉じる。
    fn make_sphere_patches(radius: f64) -> Result<Solid, String> {
        use std::f64::consts::FRAC_1_SQRT_2;
        use std::f64::consts::FRAC_PI_2;

        let sqrt2 = std::f64::consts::SQRT_2;
        let quarter_weight = FRAC_1_SQRT_2;
        let r = radius;

        // 子午線方向の (s, z): s は軸からの距離、z は高さ。
        let meridian_point = |phi: f64| (r * phi.cos(), r * phi.sin());
        let meridian_mid = |phi: f64| {
            let mid = phi + FRAC_PI_2 * 0.5;
            (sqrt2 * r * mid.cos(), sqrt2 * r * mid.sin())
        };

        let revolve_point =
            |s: f64, z: f64, theta: f64| Point3::new(s * theta.cos(), s * theta.sin(), z);
        let revolve_mid = |s: f64, z: f64, theta: f64| {
            let mid = theta + FRAC_PI_2 * 0.5;
            Point3::new(sqrt2 * s * mid.cos(), sqrt2 * s * mid.sin(), z)
        };

        let theta_of = |i: usize| FRAC_PI_2 * (i % 4) as f64;
        // j = 0 は南半球 (-90 度 -> 0 度)、j = 1 は北半球 (0 度 -> 90 度)。
        let phi_start = |j: usize| if j == 0 { -FRAC_PI_2 } else { 0.0 };

        let north = Vertex::from_point(Point3::new(0.0, 0.0, r));
        let south = Vertex::from_point(Point3::new(0.0, 0.0, -r));
        let equator: Vec<Vertex> = (0..4)
            .map(|i| Vertex::from_point(revolve_point(r, 0.0, theta_of(i))))
            .collect();

        // 赤道の4分円弧
        let mut equator_edges = Vec::with_capacity(4);
        for i in 0..4 {
            let curve = NurbsCurve3::new(
                2,
                vec![
                    ControlPoint3::unweighted(revolve_point(r, 0.0, theta_of(i))),
                    ControlPoint3::new(revolve_mid(r, 0.0, theta_of(i)), quarter_weight),
                    ControlPoint3::unweighted(revolve_point(r, 0.0, theta_of(i + 1))),
                ],
                KnotVector::clamped_uniform(3, 2),
            )?;
            equator_edges.push(Edge::new(
                curve,
                equator[i].clone(),
                equator[(i + 1) % 4].clone(),
                1e-6,
            ));
        }

        // 子午線の4分円弧。meridian[i][0] は南極 -> 赤道、[i][1] は赤道 -> 北極。
        let mut meridian_edges = Vec::with_capacity(4);
        for i in 0..4 {
            let theta = theta_of(i);
            let mut row = Vec::with_capacity(2);
            for j in 0..2 {
                let phi0 = phi_start(j);
                let (s0, z0) = meridian_point(phi0);
                let (sm, zm) = meridian_mid(phi0);
                let (s1, z1) = meridian_point(phi0 + FRAC_PI_2);
                let curve = NurbsCurve3::new(
                    2,
                    vec![
                        ControlPoint3::unweighted(revolve_point(s0, z0, theta)),
                        ControlPoint3::new(revolve_point(sm, zm, theta), quarter_weight),
                        ControlPoint3::unweighted(revolve_point(s1, z1, theta)),
                    ],
                    KnotVector::clamped_uniform(3, 2),
                )?;
                let (start, end) = if j == 0 {
                    (south.clone(), equator[i].clone())
                } else {
                    (equator[i].clone(), north.clone())
                };
                row.push(Edge::new(curve, start, end, 1e-6));
            }
            meridian_edges.push(row);
        }

        let mut faces = Vec::with_capacity(8);
        for i in 0..4 {
            for j in 0..2 {
                let theta = theta_of(i);
                let phi0 = phi_start(j);

                let meridian_rows = [
                    (meridian_point(phi0), 1.0),
                    (meridian_mid(phi0), quarter_weight),
                    (meridian_point(phi0 + FRAC_PI_2), 1.0),
                ];

                let mut grid = Vec::with_capacity(3);
                for ((s, z), w_meridian) in meridian_rows {
                    grid.push(vec![
                        ControlPoint3::new(revolve_point(s, z, theta), w_meridian),
                        ControlPoint3::new(revolve_mid(s, z, theta), w_meridian * quarter_weight),
                        ControlPoint3::new(revolve_point(s, z, theta + FRAC_PI_2), w_meridian),
                    ]);
                }
                // 行の順を南から北ではなく北から南にする。こうしないと
                // du x dv が球の内側を向く。赤道の +X 上で du は北 (+Z)、
                // dv は東 (+Y) なので、外積は -X、つまり内向きだった。
                // 面積分は面の法線を使うので、この向きで積むと符号が逆になる。
                grid.reverse();

                let surface = NurbsSurface3::new(
                    2,
                    2,
                    grid,
                    KnotVector::clamped_uniform(3, 2),
                    KnotVector::clamped_uniform(3, 2),
                )?;

                // UV反時計回り。極側の行は1点に潰れているのでその辺は現れない。
                // 行を反転したぶん UV が鏡像になるので、ワイヤも逆に回す。
                let wire = if j == 0 {
                    // u=1 が南極（退化）、u=0 が赤道。
                    Wire::new(vec![
                        OrientedEdge::forward(meridian_edges[(i + 1) % 4][0].clone()),
                        OrientedEdge::reversed(equator_edges[i].clone()),
                        OrientedEdge::reversed(meridian_edges[i][0].clone()),
                    ])
                } else {
                    // u=1 が赤道、u=0 が北極（退化）。
                    Wire::new(vec![
                        OrientedEdge::forward(equator_edges[i].clone()),
                        OrientedEdge::forward(meridian_edges[(i + 1) % 4][1].clone()),
                        OrientedEdge::reversed(meridian_edges[i][1].clone()),
                    ])
                };

                faces.push(Face::simple(FaceGeometry::Nurbs(surface), wire));
            }
        }

        crate::validated_solid(Shell::closed(faces))
    }

    /// 円錐 / 円錐台（Cone / Frustum: 底面半径 r_bottom, 天面半径 r_top, 高さ h）の生成
    pub fn make_cone(r_bottom: f64, r_top: f64, height: f64) -> Result<Solid, String> {
        if r_bottom <= 1e-6 || height <= 1e-6 || r_top < 0.0 {
            return Err("Invalid cone parameters".to_string());
        }
        if r_top <= 1e-6 {
            return Self::make_cone_with_apex(r_bottom, height);
        }

        let rb = r_bottom;
        let rt = r_top;
        let h = height;
        let weight = std::f64::consts::FRAC_1_SQRT_2;

        let pb = [
            Point3::new(rb, 0.0, 0.0),
            Point3::new(0.0, rb, 0.0),
            Point3::new(-rb, 0.0, 0.0),
            Point3::new(0.0, -rb, 0.0),
        ];
        let pt = [
            Point3::new(rt, 0.0, h),
            Point3::new(0.0, rt, h),
            Point3::new(-rt, 0.0, h),
            Point3::new(0.0, -rt, h),
        ];

        let vb: Vec<Vertex> = pb.iter().map(|p| Vertex::from_point(*p)).collect();
        let vt: Vec<Vertex> = pt.iter().map(|p| Vertex::from_point(*p)).collect();

        let mut ev = Vec::new();
        for i in 0..4 {
            let e = Edge::line_between(vb[i].clone(), vt[i].clone())?;
            ev.push(e);
        }

        let mut eb = Vec::new();
        let mut et = Vec::new();
        let mut faces = Vec::new();

        for i in 0..4 {
            let next = (i + 1) % 4;
            let corner_b = match i {
                0 => Point3::new(rb, rb, 0.0),
                1 => Point3::new(-rb, rb, 0.0),
                2 => Point3::new(-rb, -rb, 0.0),
                _ => Point3::new(rb, -rb, 0.0),
            };
            let corner_t = match i {
                0 => Point3::new(rt, rt, h),
                1 => Point3::new(-rt, rt, h),
                2 => Point3::new(-rt, -rt, h),
                _ => Point3::new(rt, -rt, h),
            };

            let arc_b = Edge::new(
                NurbsCurve3::new(
                    2,
                    vec![
                        ControlPoint3::unweighted(pb[i]),
                        ControlPoint3::new(corner_b, weight),
                        ControlPoint3::unweighted(pb[next]),
                    ],
                    KnotVector::clamped_uniform(3, 2),
                )?,
                vb[i].clone(),
                vb[next].clone(),
                1e-6,
            );

            let arc_t = Edge::new(
                NurbsCurve3::new(
                    2,
                    vec![
                        ControlPoint3::unweighted(pt[i]),
                        ControlPoint3::new(corner_t, weight),
                        ControlPoint3::unweighted(pt[next]),
                    ],
                    KnotVector::clamped_uniform(3, 2),
                )?,
                vt[i].clone(),
                vt[next].clone(),
                1e-6,
            );

            eb.push(arc_b.clone());
            et.push(arc_t.clone());

            let row0 = vec![
                ControlPoint3::unweighted(pb[i]),
                ControlPoint3::unweighted(pt[i]),
            ];
            let row1 = vec![
                ControlPoint3::new(corner_b, weight),
                ControlPoint3::new(corner_t, weight),
            ];
            let row2 = vec![
                ControlPoint3::unweighted(pb[next]),
                ControlPoint3::unweighted(pt[next]),
            ];

            let surf = NurbsSurface3::new(
                2,
                1,
                vec![row0, row1, row2],
                KnotVector::clamped_uniform(3, 2),
                KnotVector::clamped_uniform(2, 1),
            )?;

            let wire = Wire::new(vec![
                OrientedEdge::forward(arc_b),
                OrientedEdge::forward(ev[next].clone()),
                OrientedEdge::reversed(arc_t),
                OrientedEdge::reversed(ev[i].clone()),
            ]);
            faces.push(Face::simple(FaceGeometry::Nurbs(surf), wire));
        }

        // 底面 (-Z PLANE)
        let p_bot = PlaneSurface3::new(
            Point3::new(0.0, 0.0, 0.0),
            Vec3::new(0.0, 1.0, 0.0),
            Vec3::new(1.0, 0.0, 0.0),
        )
        .ok_or("plane bot")?;
        let wire_bot = Wire::new(vec![
            OrientedEdge::reversed(eb[3].clone()),
            OrientedEdge::reversed(eb[2].clone()),
            OrientedEdge::reversed(eb[1].clone()),
            OrientedEdge::reversed(eb[0].clone()),
        ]);
        faces.push(Face::simple(FaceGeometry::Plane(p_bot), wire_bot));

        // 天面 (+Z PLANE)
        let p_top = PlaneSurface3::new(
            Point3::new(0.0, 0.0, h),
            Vec3::new(1.0, 0.0, 0.0),
            Vec3::new(0.0, 1.0, 0.0),
        )
        .ok_or("plane top")?;
        let wire_top = Wire::new(vec![
            OrientedEdge::forward(et[0].clone()),
            OrientedEdge::forward(et[1].clone()),
            OrientedEdge::forward(et[2].clone()),
            OrientedEdge::forward(et[3].clone()),
        ]);
        faces.push(Face::simple(FaceGeometry::Plane(p_top), wire_top));

        let shell = Shell::closed(faces);
        crate::validated_solid(shell)
    }

    /// 真の頂点を持つ円錐（Cone）の生成
    ///
    /// 天面を極小の円で置き換えると、体積・表面積・STEP出力すべてに誤差が
    /// 混入する。頂点では側面パッチの v 方向が1点に縮退し、稜線2本が頂点で
    /// 出会うため、側面ワイヤは3辺（底面円弧＋稜線2本）で閉じる。
    fn make_cone_with_apex(r_bottom: f64, height: f64) -> Result<Solid, String> {
        let r = r_bottom;
        let h = height;
        let weight = std::f64::consts::FRAC_1_SQRT_2;

        let pb = [
            Point3::new(r, 0.0, 0.0),
            Point3::new(0.0, r, 0.0),
            Point3::new(-r, 0.0, 0.0),
            Point3::new(0.0, -r, 0.0),
        ];
        let apex = Point3::new(0.0, 0.0, h);

        let vb: Vec<Vertex> = pb.iter().map(|p| Vertex::from_point(*p)).collect();
        let v_apex = Vertex::from_point(apex);

        let mut rulings = Vec::with_capacity(4);
        for vertex in vb.iter() {
            rulings.push(Edge::line_between(vertex.clone(), v_apex.clone())?);
        }

        let mut bottom_arcs = Vec::with_capacity(4);
        let mut faces = Vec::with_capacity(5);

        for i in 0..4 {
            let next = (i + 1) % 4;
            let corner = match i {
                0 => Point3::new(r, r, 0.0),
                1 => Point3::new(-r, r, 0.0),
                2 => Point3::new(-r, -r, 0.0),
                _ => Point3::new(r, -r, 0.0),
            };

            let arc = Edge::new(
                NurbsCurve3::new(
                    2,
                    vec![
                        ControlPoint3::unweighted(pb[i]),
                        ControlPoint3::new(corner, weight),
                        ControlPoint3::unweighted(pb[next]),
                    ],
                    KnotVector::clamped_uniform(3, 2),
                )?,
                vb[i].clone(),
                vb[next].clone(),
                1e-6,
            );
            bottom_arcs.push(arc.clone());

            // 縮退する天面側も、行ごとに同じ重みを保つ（分母を分離可能に保つ）
            let surf = NurbsSurface3::new(
                2,
                1,
                vec![
                    vec![
                        ControlPoint3::unweighted(pb[i]),
                        ControlPoint3::unweighted(apex),
                    ],
                    vec![
                        ControlPoint3::new(corner, weight),
                        ControlPoint3::new(apex, weight),
                    ],
                    vec![
                        ControlPoint3::unweighted(pb[next]),
                        ControlPoint3::unweighted(apex),
                    ],
                ],
                KnotVector::clamped_uniform(3, 2),
                KnotVector::clamped_uniform(2, 1),
            )?;

            let wire = Wire::new(vec![
                OrientedEdge::forward(arc),
                OrientedEdge::forward(rulings[next].clone()),
                OrientedEdge::reversed(rulings[i].clone()),
            ]);
            faces.push(Face::simple(FaceGeometry::Nurbs(surf), wire));
        }

        let bottom_plane = PlaneSurface3::new(
            Point3::new(0.0, 0.0, 0.0),
            Vec3::new(0.0, 1.0, 0.0),
            Vec3::new(1.0, 0.0, 0.0),
        )
        .ok_or("plane bot")?;
        let bottom_wire = Wire::new(vec![
            OrientedEdge::reversed(bottom_arcs[3].clone()),
            OrientedEdge::reversed(bottom_arcs[2].clone()),
            OrientedEdge::reversed(bottom_arcs[1].clone()),
            OrientedEdge::reversed(bottom_arcs[0].clone()),
        ]);
        faces.push(Face::simple(FaceGeometry::Plane(bottom_plane), bottom_wire));

        crate::validated_solid(Shell::closed(faces))
    }

    /// トーラス（Torus: 主半径 r_major, 断面半径 r_minor）の生成
    pub fn make_torus(r_major: f64, r_minor: f64) -> Result<Solid, String> {
        if r_major <= r_minor || r_minor <= 1e-6 {
            return Err("Major radius must be greater than minor radius and positive".to_string());
        }

        let tol = zenith_math::Tolerance::default();
        let r = r_minor;
        let r_maj = r_major;
        let weight = std::f64::consts::FRAC_1_SQRT_2;

        // XZ平面上の (r_major, 0, 0) を中心とする半径 r_minor の閉じた円形プロファイル
        let p0 = Point3::new(r_maj + r, 0.0, 0.0);
        let c0 = Point3::new(r_maj + r, 0.0, r);
        let p1 = Point3::new(r_maj, 0.0, r);
        let c1 = Point3::new(r_maj - r, 0.0, r);
        let p2 = Point3::new(r_maj - r, 0.0, 0.0);
        let c2 = Point3::new(r_maj - r, 0.0, -r);
        let p3 = Point3::new(r_maj, 0.0, -r);
        let c3 = Point3::new(r_maj + r, 0.0, -r);

        let profile = NurbsCurve3::new(
            2,
            vec![
                ControlPoint3::unweighted(p0),
                ControlPoint3::new(c0, weight),
                ControlPoint3::unweighted(p1),
                ControlPoint3::new(c1, weight),
                ControlPoint3::unweighted(p2),
                ControlPoint3::new(c2, weight),
                ControlPoint3::unweighted(p3),
                ControlPoint3::new(c3, weight),
                ControlPoint3::unweighted(p0),
            ],
            KnotVector::new(vec![
                0.0, 0.0, 0.0, 0.25, 0.25, 0.5, 0.5, 0.75, 0.75, 1.0, 1.0, 1.0,
            ]),
        )?;

        let _ = (
            &tol, p0, c0, p1, c1, p2, c2, p3, c3, profile, weight, r, r_maj,
        );
        Self::make_torus_patches(r_major, r_minor)
    }

    /// トーラスを 4 x 4 = 16 枚の有理双2次パッチとして構築する。
    ///
    /// 単一面が自分自身に巻き付く表現（シーム辺を1本だけ持つ1面シェル）は
    /// OpenCASCADE で体積0の不正ソリッドとして読まれる。円柱と同じく、
    /// 各パッチが4本の実エッジで囲まれた正則な構成にすると相互運用できる。
    /// トーラスは極を持たないので、退化辺なしで完全に均質に分割できる。
    fn make_torus_patches(r_major: f64, r_minor: f64) -> Result<Solid, String> {
        use std::f64::consts::FRAC_1_SQRT_2;
        use std::f64::consts::FRAC_PI_2;

        let sqrt2 = std::f64::consts::SQRT_2;
        let quarter_weight = FRAC_1_SQRT_2;

        // 小円（チューブ断面）を4分割した各象限の始点と接線交点。
        // (s, z) はチューブ中心からの半径方向座標と高さ。
        let minor_point = |phi: f64| (r_major + r_minor * phi.cos(), r_minor * phi.sin());
        let minor_mid = |phi: f64| {
            let mid = phi + FRAC_PI_2 * 0.5;
            (
                r_major + sqrt2 * r_minor * mid.cos(),
                sqrt2 * r_minor * mid.sin(),
            )
        };

        // 大円（主軸まわり）を4分割したときの点と接線交点。
        let revolve_point =
            |s: f64, z: f64, theta: f64| Point3::new(s * theta.cos(), s * theta.sin(), z);
        let revolve_mid = |s: f64, z: f64, theta: f64| {
            let mid = theta + FRAC_PI_2 * 0.5;
            Point3::new(sqrt2 * s * mid.cos(), sqrt2 * s * mid.sin(), z)
        };

        let theta_of = |i: usize| FRAC_PI_2 * (i % 4) as f64;
        let phi_of = |j: usize| FRAC_PI_2 * (j % 4) as f64;

        // 16 個の格子頂点
        let mut vertices = Vec::with_capacity(16);
        for i in 0..4 {
            let mut row = Vec::with_capacity(4);
            for j in 0..4 {
                let (s, z) = minor_point(phi_of(j));
                row.push(Vertex::from_point(revolve_point(s, z, theta_of(i))));
            }
            vertices.push(row);
        }

        // 主軸まわりの円弧: major[i][j] は theta_i -> theta_{i+1}、チューブ角 phi_j 上。
        let mut major_edges = Vec::with_capacity(4);
        for i in 0..4 {
            let mut row = Vec::with_capacity(4);
            for j in 0..4 {
                let (s, z) = minor_point(phi_of(j));
                let curve = NurbsCurve3::new(
                    2,
                    vec![
                        ControlPoint3::unweighted(revolve_point(s, z, theta_of(i))),
                        ControlPoint3::new(revolve_mid(s, z, theta_of(i)), quarter_weight),
                        ControlPoint3::unweighted(revolve_point(s, z, theta_of(i + 1))),
                    ],
                    KnotVector::clamped_uniform(3, 2),
                )?;
                row.push(Edge::new(
                    curve,
                    vertices[i][j].clone(),
                    vertices[(i + 1) % 4][j].clone(),
                    1e-6,
                ));
            }
            major_edges.push(row);
        }

        // チューブ断面の円弧: minor[i][j] は phi_j -> phi_{j+1}、主軸角 theta_i 上。
        let mut minor_edges = Vec::with_capacity(4);
        for i in 0..4 {
            let mut row = Vec::with_capacity(4);
            for j in 0..4 {
                let theta = theta_of(i);
                let (s0, z0) = minor_point(phi_of(j));
                let (sm, zm) = minor_mid(phi_of(j));
                let (s1, z1) = minor_point(phi_of(j + 1));
                let curve = NurbsCurve3::new(
                    2,
                    vec![
                        ControlPoint3::unweighted(revolve_point(s0, z0, theta)),
                        ControlPoint3::new(revolve_point(sm, zm, theta), quarter_weight),
                        ControlPoint3::unweighted(revolve_point(s1, z1, theta)),
                    ],
                    KnotVector::clamped_uniform(3, 2),
                )?;
                row.push(Edge::new(
                    curve,
                    vertices[i][j].clone(),
                    vertices[i][(j + 1) % 4].clone(),
                    1e-6,
                ));
            }
            minor_edges.push(row);
        }

        let mut faces = Vec::with_capacity(16);
        for i in 0..4 {
            for j in 0..4 {
                let theta = theta_of(i);
                let phi = phi_of(j);

                // u はチューブ方向、v は主軸方向。制御点は (s, z) の3点を
                // それぞれ4分の1回転させたテンソル積。
                let minor_rows = [
                    (minor_point(phi), 1.0),
                    (minor_mid(phi), quarter_weight),
                    (minor_point(phi + FRAC_PI_2), 1.0),
                ];

                let mut grid = Vec::with_capacity(3);
                for ((s, z), w_minor) in minor_rows {
                    grid.push(vec![
                        ControlPoint3::new(revolve_point(s, z, theta), w_minor),
                        ControlPoint3::new(revolve_mid(s, z, theta), w_minor * quarter_weight),
                        ControlPoint3::new(revolve_point(s, z, theta + FRAC_PI_2), w_minor),
                    ]);
                }

                // 行の順を逆にして du x dv を外向きにする。そのままだと
                // チューブの内側を向き、面積分の符号が逆になる。球と同じ話。
                grid.reverse();

                let surface = NurbsSurface3::new(
                    2,
                    2,
                    grid,
                    KnotVector::clamped_uniform(3, 2),
                    KnotVector::clamped_uniform(3, 2),
                )?;

                // UV空間で反時計回りになる巡回。行を反転したぶん UV が鏡像に
                // なるので、ワイヤも逆に回す。
                let wire = Wire::new(vec![
                    OrientedEdge::forward(major_edges[i][j].clone()),
                    OrientedEdge::forward(minor_edges[(i + 1) % 4][j].clone()),
                    OrientedEdge::reversed(major_edges[i][(j + 1) % 4].clone()),
                    OrientedEdge::reversed(minor_edges[i][j].clone()),
                ]);

                faces.push(Face::simple(FaceGeometry::Nurbs(surface), wire));
            }
        }

        crate::validated_solid(Shell::closed(faces))
    }

    /// 正多角柱（正N角柱: 正六角柱、正八角柱など）ソリッドの生成
    ///
    /// `sides`: 角数（3以上）
    /// `radius`: 外接円半径
    /// `height`: 柱の高さ（Z方向）
    pub fn make_regular_prism(sides: usize, radius: f64, height: f64) -> Result<Solid, String> {
        if sides < 3 {
            return Err(format!(
                "Regular prism must have at least 3 sides, got {sides}"
            ));
        }
        if radius <= 1e-9 || height <= 1e-9 {
            return Err(format!(
                "Prism radius and height must be positive, got radius={radius}, height={height}"
            ));
        }

        let n = sides;
        let mut pts_b = Vec::with_capacity(n);
        let mut pts_t = Vec::with_capacity(n);

        for i in 0..n {
            let theta = 2.0 * std::f64::consts::PI * (i as f64) / (n as f64);
            let x = radius * theta.cos();
            let y = radius * theta.sin();
            pts_b.push(Point3::new(x, y, 0.0));
            pts_t.push(Point3::new(x, y, height));
        }

        let vb: Vec<Vertex> = pts_b.iter().map(|&p| Vertex::from_point(p)).collect();
        let vt: Vec<Vertex> = pts_t.iter().map(|&p| Vertex::from_point(p)).collect();

        // 1. 底面エッジ群 (i -> i+1)
        let mut eb = Vec::with_capacity(n);
        for i in 0..n {
            let next = (i + 1) % n;
            eb.push(Edge::line_between(vb[i].clone(), vb[next].clone())?);
        }

        // 2. 天面エッジ群 (i -> i+1)
        let mut et = Vec::with_capacity(n);
        for i in 0..n {
            let next = (i + 1) % n;
            et.push(Edge::line_between(vt[i].clone(), vt[next].clone())?);
        }

        // 3. 垂直エッジ群 (vb[i] -> vt[i])
        let mut ev = Vec::with_capacity(n);
        for i in 0..n {
            ev.push(Edge::line_between(vb[i].clone(), vt[i].clone())?);
        }

        let mut faces = Vec::with_capacity(n + 2);

        // 4. 側面 N面（外向き法線）
        // 巡回: vb[i] -> vb[next] -> vt[next] -> vt[i]
        for i in 0..n {
            let next = (i + 1) % n;
            let u_axis = pts_b[next] - pts_b[i];
            let v_axis = Vec3::new(0.0, 0.0, height);
            let plane = PlaneSurface3::new(pts_b[i], u_axis, v_axis)
                .ok_or("Failed to create prism side plane")?;

            let wire = Wire::new(vec![
                OrientedEdge::forward(eb[i].clone()),
                OrientedEdge::forward(ev[next].clone()),
                OrientedEdge::reversed(et[i].clone()),
                OrientedEdge::reversed(ev[i].clone()),
            ]);
            faces.push(Face::simple(FaceGeometry::Plane(plane), wire));
        }

        // 5. 底面（法線 -Z: CCW は逆順 n-1 -> 0）
        let plane_b = PlaneSurface3::new(
            Point3::new(0.0, 0.0, 0.0),
            Vec3::new(1.0, 0.0, 0.0),
            Vec3::new(0.0, -1.0, 0.0),
        )
        .ok_or("Failed to create prism bottom plane")?;
        let mut wire_b_edges = Vec::with_capacity(n);
        for i in (0..n).rev() {
            wire_b_edges.push(OrientedEdge::reversed(eb[i].clone()));
        }
        faces.push(Face::simple(
            FaceGeometry::Plane(plane_b),
            Wire::new(wire_b_edges),
        ));

        // 6. 天面（法線 +Z: CCW は正順 0 -> n-1）
        let plane_t = PlaneSurface3::new(
            Point3::new(0.0, 0.0, height),
            Vec3::new(1.0, 0.0, 0.0),
            Vec3::new(0.0, 1.0, 0.0),
        )
        .ok_or("Failed to create prism top plane")?;
        let mut wire_t_edges = Vec::with_capacity(n);
        for i in 0..n {
            wire_t_edges.push(OrientedEdge::forward(et[i].clone()));
        }
        faces.push(Face::simple(
            FaceGeometry::Plane(plane_t),
            Wire::new(wire_t_edges),
        ));

        crate::validated_solid(Shell::closed(faces))
    }

    /// スロット柱（長円柱 / Slot Prism）の生成（直線部2面 + 90度有理円筒パッチ4面 + 上下端面）
    pub fn make_slot_prism(length: f64, radius: f64, height: f64) -> Result<Solid, String> {
        if length <= 1e-6 || radius <= 1e-6 || height <= 1e-6 {
            return Err(format!(
                "Slot prism dimensions must be positive, got length={length}, radius={radius}, height={height}"
            ));
        }

        let l_half = length * 0.5;
        let r = radius;
        let h = height;
        let weight = std::f64::consts::FRAC_1_SQRT_2;

        // 6個の底面頂点座標
        let pb = [
            Point3::new(-l_half, -r, 0.0),      // 0
            Point3::new(l_half, -r, 0.0),       // 1
            Point3::new(l_half + r, 0.0, 0.0),  // 2
            Point3::new(l_half, r, 0.0),        // 3
            Point3::new(-l_half, r, 0.0),       // 4
            Point3::new(-l_half - r, 0.0, 0.0), // 5
        ];

        // 6個の天面頂点座標
        let pt = [
            Point3::new(-l_half, -r, h),
            Point3::new(l_half, -r, h),
            Point3::new(l_half + r, 0.0, h),
            Point3::new(l_half, r, h),
            Point3::new(-l_half, r, h),
            Point3::new(-l_half - r, 0.0, h),
        ];

        let vb: Vec<Vertex> = pb.iter().map(|p| Vertex::from_point(*p)).collect();
        let vt: Vec<Vertex> = pt.iter().map(|p| Vertex::from_point(*p)).collect();

        // 6本の縦直線エッジ
        let mut ev = Vec::with_capacity(6);
        for i in 0..6 {
            ev.push(Edge::line_between(vb[i].clone(), vt[i].clone())?);
        }

        // 6本の底面エッジ & 天面エッジ
        let mut eb = Vec::with_capacity(6);
        let mut et = Vec::with_capacity(6);
        let mut faces = Vec::with_capacity(8);

        for i in 0..6 {
            let next = (i + 1) % 6;
            let (is_arc, corner_b, corner_t) = match i {
                0 => (false, Point3::origin(), Point3::origin()), // 直線 -Y
                1 => (
                    true,
                    Point3::new(l_half + r, -r, 0.0),
                    Point3::new(l_half + r, -r, h),
                ), // 円弧 4象限
                2 => (
                    true,
                    Point3::new(l_half + r, r, 0.0),
                    Point3::new(l_half + r, r, h),
                ), // 円弧 1象限
                3 => (false, Point3::origin(), Point3::origin()), // 直線 +Y
                4 => (
                    true,
                    Point3::new(-l_half - r, r, 0.0),
                    Point3::new(-l_half - r, r, h),
                ), // 円弧 2象限
                5 => (
                    true,
                    Point3::new(-l_half - r, -r, 0.0),
                    Point3::new(-l_half - r, -r, h),
                ), // 円弧 3象限
                _ => unreachable!(),
            };

            let (edge_b, edge_t, face_geom) = if !is_arc {
                let edge_b = Edge::line_between(vb[i].clone(), vb[next].clone())?;
                let edge_t = Edge::line_between(vt[i].clone(), vt[next].clone())?;
                let u_axis = pb[next] - pb[i];
                let v_axis = Vec3::new(0.0, 0.0, h);
                let plane = PlaneSurface3::new(pb[i], u_axis, v_axis)
                    .ok_or("Failed to create side plane")?;
                (edge_b, edge_t, FaceGeometry::Plane(plane))
            } else {
                let arc_b = Edge::new(
                    NurbsCurve3::new(
                        2,
                        vec![
                            ControlPoint3::unweighted(pb[i]),
                            ControlPoint3::new(corner_b, weight),
                            ControlPoint3::unweighted(pb[next]),
                        ],
                        KnotVector::clamped_uniform(3, 2),
                    )?,
                    vb[i].clone(),
                    vb[next].clone(),
                    1e-6,
                );
                let arc_t = Edge::new(
                    NurbsCurve3::new(
                        2,
                        vec![
                            ControlPoint3::unweighted(pt[i]),
                            ControlPoint3::new(corner_t, weight),
                            ControlPoint3::unweighted(pt[next]),
                        ],
                        KnotVector::clamped_uniform(3, 2),
                    )?,
                    vt[i].clone(),
                    vt[next].clone(),
                    1e-6,
                );
                let row0 = vec![
                    ControlPoint3::unweighted(pb[i]),
                    ControlPoint3::unweighted(pt[i]),
                ];
                let row1 = vec![
                    ControlPoint3::new(corner_b, weight),
                    ControlPoint3::new(corner_t, weight),
                ];
                let row2 = vec![
                    ControlPoint3::unweighted(pb[next]),
                    ControlPoint3::unweighted(pt[next]),
                ];
                let surf = NurbsSurface3::new(
                    2,
                    1,
                    vec![row0, row1, row2],
                    KnotVector::clamped_uniform(3, 2),
                    KnotVector::clamped_uniform(2, 1),
                )?;
                (arc_b, arc_t, FaceGeometry::Nurbs(surf))
            };

            eb.push(edge_b.clone());
            et.push(edge_t.clone());

            let wire = Wire::new(vec![
                OrientedEdge::forward(edge_b),
                OrientedEdge::forward(ev[next].clone()),
                OrientedEdge::reversed(edge_t),
                OrientedEdge::reversed(ev[i].clone()),
            ]);
            faces.push(Face::simple(face_geom, wire));
        }

        // 底面 (-Z 法線: 逆順 5..0)
        let plane_b = PlaneSurface3::new(
            Point3::new(0.0, 0.0, 0.0),
            Vec3::new(1.0, 0.0, 0.0),
            Vec3::new(0.0, -1.0, 0.0),
        )
        .ok_or("Failed to create bottom plane")?;
        let wire_b = Wire::new(vec![
            OrientedEdge::reversed(eb[5].clone()),
            OrientedEdge::reversed(eb[4].clone()),
            OrientedEdge::reversed(eb[3].clone()),
            OrientedEdge::reversed(eb[2].clone()),
            OrientedEdge::reversed(eb[1].clone()),
            OrientedEdge::reversed(eb[0].clone()),
        ]);
        faces.push(Face::simple(FaceGeometry::Plane(plane_b), wire_b));

        // 天面 (+Z 法線: 正順 0..5)
        let plane_t = PlaneSurface3::new(
            Point3::new(0.0, 0.0, h),
            Vec3::new(1.0, 0.0, 0.0),
            Vec3::new(0.0, 1.0, 0.0),
        )
        .ok_or("Failed to create top plane")?;
        let wire_t = Wire::new(vec![
            OrientedEdge::forward(et[0].clone()),
            OrientedEdge::forward(et[1].clone()),
            OrientedEdge::forward(et[2].clone()),
            OrientedEdge::forward(et[3].clone()),
            OrientedEdge::forward(et[4].clone()),
            OrientedEdge::forward(et[5].clone()),
        ]);
        faces.push(Face::simple(FaceGeometry::Plane(plane_t), wire_t));

        crate::validated_solid(Shell::closed(faces))
    }
}
