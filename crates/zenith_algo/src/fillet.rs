use zenith_geom::{ControlPoint3, KnotVector, NurbsCurve3, NurbsSurface3, PlaneSurface3};
use zenith_math::{Point3, Tolerance, Vec3};
use zenith_topo::{Edge, Face, FaceGeometry, OrientedEdge, Shell, Solid, Vertex, Wire};

/// エッジフィレットおよび面取りビルダー
pub struct FilletBuilder;

impl FilletBuilder {
    /// 直方体のZ軸方向の4本のエッジに半径 radius のフィレット（真円角丸め）を適用したソリッドを生成（完全トポロジー共有B-Rep）
    pub fn fillet_box_z_edges(
        dx: f64,
        dy: f64,
        dz: f64,
        radius: f64,
        _tol: &Tolerance,
    ) -> Result<Solid, String> {
        // 要求された半径を黙って詰めない。丸めきれない指定は、それらしい
        // 別形状を返すのではなく理由を返す。
        if radius < 0.0 {
            return Err(format!("Fillet radius must not be negative, got {radius}"));
        }
        let r = radius;
        if r <= 1e-6 {
            return crate::primitive::PrimitiveBuilder::make_box(dx, dy, dz);
        }
        if 2.0 * r >= dx.min(dy) {
            return Err(format!(
                "Fillet radius {r} must be smaller than half the shorter side ({})",
                dx.min(dy) * 0.5
            ));
        }

        // 下面 (z=0) の8頂点
        let p_b0 = Point3::new(r, 0.0, 0.0);
        let p_b1 = Point3::new(dx - r, 0.0, 0.0);
        let p_b2 = Point3::new(dx, r, 0.0);
        let p_b3 = Point3::new(dx, dy - r, 0.0);
        let p_b4 = Point3::new(dx - r, dy, 0.0);
        let p_b5 = Point3::new(r, dy, 0.0);
        let p_b6 = Point3::new(0.0, dy - r, 0.0);
        let p_b7 = Point3::new(0.0, r, 0.0);

        // 上面 (z=dz) の8頂点
        let p_t0 = Point3::new(r, 0.0, dz);
        let p_t1 = Point3::new(dx - r, 0.0, dz);
        let p_t2 = Point3::new(dx, r, dz);
        let p_t3 = Point3::new(dx, dy - r, dz);
        let p_t4 = Point3::new(dx - r, dy, dz);
        let p_t5 = Point3::new(r, dy, dz);
        let p_t6 = Point3::new(0.0, dy - r, dz);
        let p_t7 = Point3::new(0.0, r, dz);

        let vb = [
            Vertex::from_point(p_b0),
            Vertex::from_point(p_b1),
            Vertex::from_point(p_b2),
            Vertex::from_point(p_b3),
            Vertex::from_point(p_b4),
            Vertex::from_point(p_b5),
            Vertex::from_point(p_b6),
            Vertex::from_point(p_b7),
        ];

        let vt = [
            Vertex::from_point(p_t0),
            Vertex::from_point(p_t1),
            Vertex::from_point(p_t2),
            Vertex::from_point(p_t3),
            Vertex::from_point(p_t4),
            Vertex::from_point(p_t5),
            Vertex::from_point(p_t6),
            Vertex::from_point(p_t7),
        ];

        // 1. 直線エッジ群の生成
        // 下面4直線 (z=0)
        let eb01 = Edge::line_between(vb[0].clone(), vb[1].clone())?;
        let eb23 = Edge::line_between(vb[2].clone(), vb[3].clone())?;
        let eb45 = Edge::line_between(vb[4].clone(), vb[5].clone())?;
        let eb67 = Edge::line_between(vb[6].clone(), vb[7].clone())?;

        // 上面4直線 (z=dz)
        let et01 = Edge::line_between(vt[0].clone(), vt[1].clone())?;
        let et23 = Edge::line_between(vt[2].clone(), vt[3].clone())?;
        let et45 = Edge::line_between(vt[4].clone(), vt[5].clone())?;
        let et67 = Edge::line_between(vt[6].clone(), vt[7].clone())?;

        // 8本の垂直エッジ (vb[i] -> vt[i])
        let ev0 = Edge::line_between(vb[0].clone(), vt[0].clone())?;
        let ev1 = Edge::line_between(vb[1].clone(), vt[1].clone())?;
        let ev2 = Edge::line_between(vb[2].clone(), vt[2].clone())?;
        let ev3 = Edge::line_between(vb[3].clone(), vt[3].clone())?;
        let ev4 = Edge::line_between(vb[4].clone(), vt[4].clone())?;
        let ev5 = Edge::line_between(vb[5].clone(), vt[5].clone())?;
        let ev6 = Edge::line_between(vb[6].clone(), vt[6].clone())?;
        let ev7 = Edge::line_between(vb[7].clone(), vt[7].clone())?;

        // 2. 4隅の有理円弧エッジ群の生成 (weight = 1/√2)
        let weight = std::f64::consts::FRAC_1_SQRT_2;

        let make_arc_edge = |p_s: Point3,
                             p_e: Point3,
                             ctr: Point3,
                             v_s: Vertex,
                             v_e: Vertex|
         -> Result<Edge, String> {
            let corner = ctr + (p_s - ctr) + (p_e - ctr);
            let curve = NurbsCurve3::new(
                2,
                vec![
                    ControlPoint3::unweighted(p_s),
                    ControlPoint3::new(corner, weight),
                    ControlPoint3::unweighted(p_e),
                ],
                KnotVector::clamped_uniform(3, 2),
            )?;
            Ok(Edge::new(curve, v_s, v_e, 1e-6))
        };

        // 下面4円弧 (z=0)
        let c1_ctr_b = Point3::new(dx - r, r, 0.0);
        let c2_ctr_b = Point3::new(dx - r, dy - r, 0.0);
        let c3_ctr_b = Point3::new(r, dy - r, 0.0);
        let c4_ctr_b = Point3::new(r, r, 0.0);

        let arc_b1 = make_arc_edge(p_b1, p_b2, c1_ctr_b, vb[1].clone(), vb[2].clone())?;
        let arc_b2 = make_arc_edge(p_b3, p_b4, c2_ctr_b, vb[3].clone(), vb[4].clone())?;
        let arc_b3 = make_arc_edge(p_b5, p_b6, c3_ctr_b, vb[5].clone(), vb[6].clone())?;
        let arc_b4 = make_arc_edge(p_b7, p_b0, c4_ctr_b, vb[7].clone(), vb[0].clone())?;

        // 上面4円弧 (z=dz)
        let c1_ctr_t = Point3::new(dx - r, r, dz);
        let c2_ctr_t = Point3::new(dx - r, dy - r, dz);
        let c3_ctr_t = Point3::new(r, dy - r, dz);
        let c4_ctr_t = Point3::new(r, r, dz);

        let arc_t1 = make_arc_edge(p_t1, p_t2, c1_ctr_t, vt[1].clone(), vt[2].clone())?;
        let arc_t2 = make_arc_edge(p_t3, p_t4, c2_ctr_t, vt[3].clone(), vt[4].clone())?;
        let arc_t3 = make_arc_edge(p_t5, p_t6, c3_ctr_t, vt[5].clone(), vt[6].clone())?;
        let arc_t4 = make_arc_edge(p_t7, p_t0, c4_ctr_t, vt[7].clone(), vt[0].clone())?;

        let mut faces = Vec::new();

        // 3. 4つの平面側面Face (エッジ完全共有)
        // Front Face (-Y): vb0 -> vb1 -> vt1 -> vt0
        let p_front = PlaneSurface3::new(p_b0, Vec3::new(1.0, 0.0, 0.0), Vec3::new(0.0, 0.0, 1.0))
            .ok_or("Failed plane front")?;
        let wire_front = Wire::new(vec![
            OrientedEdge::forward(eb01.clone()),
            OrientedEdge::forward(ev1.clone()),
            OrientedEdge::reversed(et01.clone()),
            OrientedEdge::reversed(ev0.clone()),
        ]);
        faces.push(Face::simple(FaceGeometry::Plane(p_front), wire_front));

        // Right Face (+X): vb2 -> vb3 -> vt3 -> vt2
        let p_right = PlaneSurface3::new(p_b2, Vec3::new(0.0, 1.0, 0.0), Vec3::new(0.0, 0.0, 1.0))
            .ok_or("Failed plane right")?;
        let wire_right = Wire::new(vec![
            OrientedEdge::forward(eb23.clone()),
            OrientedEdge::forward(ev3.clone()),
            OrientedEdge::reversed(et23.clone()),
            OrientedEdge::reversed(ev2.clone()),
        ]);
        faces.push(Face::simple(FaceGeometry::Plane(p_right), wire_right));

        // Back Face (+Y): vb4 -> vb5 -> vt5 -> vt4
        let p_back = PlaneSurface3::new(p_b4, Vec3::new(-1.0, 0.0, 0.0), Vec3::new(0.0, 0.0, 1.0))
            .ok_or("Failed plane back")?;
        let wire_back = Wire::new(vec![
            OrientedEdge::forward(eb45.clone()),
            OrientedEdge::forward(ev5.clone()),
            OrientedEdge::reversed(et45.clone()),
            OrientedEdge::reversed(ev4.clone()),
        ]);
        faces.push(Face::simple(FaceGeometry::Plane(p_back), wire_back));

        // Left Face (-X): vb6 -> vb7 -> vt7 -> vt6
        let p_left = PlaneSurface3::new(p_b6, Vec3::new(0.0, -1.0, 0.0), Vec3::new(0.0, 0.0, 1.0))
            .ok_or("Failed plane left")?;
        let wire_left = Wire::new(vec![
            OrientedEdge::forward(eb67.clone()),
            OrientedEdge::forward(ev7.clone()),
            OrientedEdge::reversed(et67.clone()),
            OrientedEdge::reversed(ev6.clone()),
        ]);
        faces.push(Face::simple(FaceGeometry::Plane(p_left), wire_left));

        // 4. 4つのフィレットNURBS曲面Face
        let make_cyl_surface =
            |p_s: Point3, p_e: Point3, ctr: Point3| -> Result<NurbsSurface3, String> {
                let corner_b = ctr + (p_s - ctr) + (p_e - ctr);
                let corner_t = Point3::new(corner_b.x, corner_b.y, dz);
                let row0 = vec![
                    ControlPoint3::unweighted(p_s),
                    ControlPoint3::unweighted(Point3::new(p_s.x, p_s.y, dz)),
                ];
                let row1 = vec![
                    ControlPoint3::new(corner_b, weight),
                    ControlPoint3::new(corner_t, weight),
                ];
                let row2 = vec![
                    ControlPoint3::unweighted(p_e),
                    ControlPoint3::unweighted(Point3::new(p_e.x, p_e.y, dz)),
                ];
                NurbsSurface3::new(
                    2,
                    1,
                    vec![row0, row1, row2],
                    KnotVector::clamped_uniform(3, 2),
                    KnotVector::clamped_uniform(2, 1),
                )
            };

        // Corner 1: vb1 -> vb2 -> vt2 -> vt1
        let s_c1 = make_cyl_surface(p_b1, p_b2, c1_ctr_b)?;
        let wire_c1 = Wire::new(vec![
            OrientedEdge::forward(arc_b1.clone()),
            OrientedEdge::forward(ev2.clone()),
            OrientedEdge::reversed(arc_t1.clone()),
            OrientedEdge::reversed(ev1.clone()),
        ]);
        faces.push(Face::simple(FaceGeometry::Nurbs(s_c1), wire_c1));

        // Corner 2: vb3 -> vb4 -> vt4 -> vt3
        let s_c2 = make_cyl_surface(p_b3, p_b4, c2_ctr_b)?;
        let wire_c2 = Wire::new(vec![
            OrientedEdge::forward(arc_b2.clone()),
            OrientedEdge::forward(ev4.clone()),
            OrientedEdge::reversed(arc_t2.clone()),
            OrientedEdge::reversed(ev3.clone()),
        ]);
        faces.push(Face::simple(FaceGeometry::Nurbs(s_c2), wire_c2));

        // Corner 3: vb5 -> vb6 -> vt6 -> vt5
        let s_c3 = make_cyl_surface(p_b5, p_b6, c3_ctr_b)?;
        let wire_c3 = Wire::new(vec![
            OrientedEdge::forward(arc_b3.clone()),
            OrientedEdge::forward(ev6.clone()),
            OrientedEdge::reversed(arc_t3.clone()),
            OrientedEdge::reversed(ev5.clone()),
        ]);
        faces.push(Face::simple(FaceGeometry::Nurbs(s_c3), wire_c3));

        // Corner 4: vb7 -> vb0 -> vt0 -> vt7
        let s_c4 = make_cyl_surface(p_b7, p_b0, c4_ctr_b)?;
        let wire_c4 = Wire::new(vec![
            OrientedEdge::forward(arc_b4.clone()),
            OrientedEdge::forward(ev0.clone()),
            OrientedEdge::reversed(arc_t4.clone()),
            OrientedEdge::reversed(ev7.clone()),
        ]);
        faces.push(Face::simple(FaceGeometry::Nurbs(s_c4), wire_c4));

        // 5. Bottom Face (-Z, PLANE): vb0 -> vb7 -> vb6 -> vb5 -> vb4 -> vb3 -> vb2 -> vb1 -> vb0
        // box_solid.stp と同じく原点 (0,0,0)、法線 (0,0,-1)、X軸 (0,1,0)
        let p_bot = PlaneSurface3::new(
            Point3::new(0.0, 0.0, 0.0),
            Vec3::new(0.0, 1.0, 0.0),
            Vec3::new(1.0, 0.0, 0.0),
        )
        .ok_or("Failed plane bottom")?;
        let wire_bot = Wire::new(vec![
            OrientedEdge::reversed(arc_b4.clone()),
            OrientedEdge::reversed(eb67.clone()),
            OrientedEdge::reversed(arc_b3.clone()),
            OrientedEdge::reversed(eb45.clone()),
            OrientedEdge::reversed(arc_b2.clone()),
            OrientedEdge::reversed(eb23.clone()),
            OrientedEdge::reversed(arc_b1.clone()),
            OrientedEdge::reversed(eb01.clone()),
        ]);
        faces.push(Face::simple(FaceGeometry::Plane(p_bot), wire_bot));

        // 6. Top Face (+Z, PLANE): vt0 -> vt1 -> vt2 -> vt3 -> vt4 -> vt5 -> vt6 -> vt7 -> vt0
        // box_solid.stp と同じく原点 (0,0,dz)、法線 (0,0,1)、X軸 (1,0,0)
        let p_top = PlaneSurface3::new(
            Point3::new(0.0, 0.0, dz),
            Vec3::new(1.0, 0.0, 0.0),
            Vec3::new(0.0, 1.0, 0.0),
        )
        .ok_or("Failed plane top")?;
        let wire_top = Wire::new(vec![
            OrientedEdge::forward(et01.clone()),
            OrientedEdge::forward(arc_t1.clone()),
            OrientedEdge::forward(et23.clone()),
            OrientedEdge::forward(arc_t2.clone()),
            OrientedEdge::forward(et45.clone()),
            OrientedEdge::forward(arc_t3.clone()),
            OrientedEdge::forward(et67.clone()),
            OrientedEdge::forward(arc_t4.clone()),
        ]);
        faces.push(Face::simple(FaceGeometry::Plane(p_top), wire_top));

        let shell = Shell::closed(faces);
        crate::validated_solid(shell)
    }
}
