use zenith_geom::{ControlPoint3, KnotVector, NurbsCurve3, NurbsSurface3, PlaneSurface3};
use zenith_math::{Point3, Vec3};
use zenith_topo::{Edge, Face, FaceGeometry, OrientedEdge, Shell, Solid, Vertex, Wire};

/// 基本幾何プリミティブ生成ビルダー（Box, Cylinder, Sphere, Cone）
pub struct PrimitiveBuilder;

impl PrimitiveBuilder {
    /// 直方体ソリッド（Box）の生成（外向き法線・閉マニホールドB-Rep）
    pub fn make_box(dx: f64, dy: f64, dz: f64) -> Result<Solid, String> {
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

        // 閉じた単一面Shellとして構成
        let vn = Vertex::from_point(p_north);
        let vs = Vertex::from_point(p_south);
        let e_seam = Edge::new(profile, vn.clone(), vs.clone(), 1e-6);
        let wire = Wire::new(vec![
            OrientedEdge::forward(e_seam.clone()),
            OrientedEdge::reversed(e_seam),
        ]);
        let face = Face::simple(FaceGeometry::Nurbs(surf), wire);
        let shell = Shell::closed(vec![face]);
        crate::validated_solid(shell)
    }

    /// 円錐 / 円錐台（Cone / Frustum: 底面半径 r_bottom, 天面半径 r_top, 高さ h）の生成
    pub fn make_cone(r_bottom: f64, r_top: f64, height: f64) -> Result<Solid, String> {
        if r_bottom <= 1e-6 || height <= 1e-6 || r_top < 0.0 {
            return Err("Invalid cone parameters".to_string());
        }

        let rb = r_bottom;
        let rt = r_top.max(0.001); // 頂点特異点を防ぐ極小天面
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

        let surf = crate::revolve::RevolveBuilder::revolve_curve(
            &profile,
            Point3::new(0.0, 0.0, 0.0),
            Vec3::new(0.0, 0.0, 1.0),
            std::f64::consts::PI * 2.0,
            &tol,
        )?;

        let v0 = Vertex::from_point(p0);
        let e_seam = Edge::new(profile, v0.clone(), v0.clone(), 1e-6);
        let wire = Wire::new(vec![
            OrientedEdge::forward(e_seam.clone()),
            OrientedEdge::reversed(e_seam),
        ]);
        let face = Face::simple(FaceGeometry::Nurbs(surf), wire);
        let shell = Shell::closed(vec![face]);
        crate::validated_solid(shell)
    }
}
