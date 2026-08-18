use zenith_geom::{ControlPoint3, KnotVector, NurbsCurve3, NurbsSurface3, PlaneSurface3};
use zenith_math::{Point3, Vec3};
use zenith_topo::{Edge, Face, FaceGeometry, OrientedEdge, Shell, Solid, Vertex, Wire};

/// 穴あきソリッド（貫通穴・ポケット）ビルダー
pub struct HoleBuilder;

impl HoleBuilder {
    /// 直方体にZ軸方向の貫通円形穴を開けたソリッドを生成（内側穴ループ FACE_BOUND 完全対応）
    pub fn make_drilled_box(dx: f64, dy: f64, dz: f64, hole_radius: f64) -> Result<Solid, String> {
        let r = hole_radius.min(dx * 0.45).min(dy * 0.45);
        if r <= 1e-6 {
            return crate::primitive::PrimitiveBuilder::make_box(dx, dy, dz);
        }

        let cx = dx * 0.5;
        let cy = dy * 0.5;

        // 1. 直方体の8頂点（外側四角形）
        let p_b0 = Point3::new(0.0, 0.0, 0.0);
        let p_b1 = Point3::new(dx, 0.0, 0.0);
        let p_b2 = Point3::new(dx, dy, 0.0);
        let p_b3 = Point3::new(0.0, dy, 0.0);

        let p_t0 = Point3::new(0.0, 0.0, dz);
        let p_t1 = Point3::new(dx, 0.0, dz);
        let p_t2 = Point3::new(dx, dy, dz);
        let p_t3 = Point3::new(0.0, dy, dz);

        let vb = [
            Vertex::from_point(p_b0),
            Vertex::from_point(p_b1),
            Vertex::from_point(p_b2),
            Vertex::from_point(p_b3),
        ];
        let vt = [
            Vertex::from_point(p_t0),
            Vertex::from_point(p_t1),
            Vertex::from_point(p_t2),
            Vertex::from_point(p_t3),
        ];

        // 2. 穴の4頂点（0度, 90度, 180度, 270度）
        let p_hb0 = Point3::new(cx + r, cy, 0.0);
        let p_hb1 = Point3::new(cx, cy + r, 0.0);
        let p_hb2 = Point3::new(cx - r, cy, 0.0);
        let p_hb3 = Point3::new(cx, cy - r, 0.0);

        let p_ht0 = Point3::new(cx + r, cy, dz);
        let p_ht1 = Point3::new(cx, cy + r, dz);
        let p_ht2 = Point3::new(cx - r, cy, dz);
        let p_ht3 = Point3::new(cx, cy - r, dz);

        let v_hb = [
            Vertex::from_point(p_hb0),
            Vertex::from_point(p_hb1),
            Vertex::from_point(p_hb2),
            Vertex::from_point(p_hb3),
        ];
        let v_ht = [
            Vertex::from_point(p_ht0),
            Vertex::from_point(p_ht1),
            Vertex::from_point(p_ht2),
            Vertex::from_point(p_ht3),
        ];

        // 3. 直方体外側エッジ群
        let eb01 = Edge::line_between(vb[0].clone(), vb[1].clone())?;
        let eb12 = Edge::line_between(vb[1].clone(), vb[2].clone())?;
        let eb23 = Edge::line_between(vb[2].clone(), vb[3].clone())?;
        let eb30 = Edge::line_between(vb[3].clone(), vb[0].clone())?;

        let et01 = Edge::line_between(vt[0].clone(), vt[1].clone())?;
        let et12 = Edge::line_between(vt[1].clone(), vt[2].clone())?;
        let et23 = Edge::line_between(vt[2].clone(), vt[3].clone())?;
        let et30 = Edge::line_between(vt[3].clone(), vt[0].clone())?;

        let ev0 = Edge::line_between(vb[0].clone(), vt[0].clone())?;
        let ev1 = Edge::line_between(vb[1].clone(), vt[1].clone())?;
        let ev2 = Edge::line_between(vb[2].clone(), vt[2].clone())?;
        let ev3 = Edge::line_between(vb[3].clone(), vt[3].clone())?;

        // 4. 穴のエッジ群（4つの有理円弧 + 4つの垂直エッジ）
        let weight = std::f64::consts::FRAC_1_SQRT_2;
        let make_hole_arc = |p_s: Point3,
                             p_e: Point3,
                             corner: Point3,
                             v_s: Vertex,
                             v_e: Vertex|
         -> Result<Edge, String> {
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

        // 下面穴円弧 (z=0, 反時計回り: 0->1->2->3->0)
        let arc_hb01 = make_hole_arc(
            p_hb0,
            p_hb1,
            Point3::new(cx + r, cy + r, 0.0),
            v_hb[0].clone(),
            v_hb[1].clone(),
        )?;
        let arc_hb12 = make_hole_arc(
            p_hb1,
            p_hb2,
            Point3::new(cx - r, cy + r, 0.0),
            v_hb[1].clone(),
            v_hb[2].clone(),
        )?;
        let arc_hb23 = make_hole_arc(
            p_hb2,
            p_hb3,
            Point3::new(cx - r, cy - r, 0.0),
            v_hb[2].clone(),
            v_hb[3].clone(),
        )?;
        let arc_hb30 = make_hole_arc(
            p_hb3,
            p_hb0,
            Point3::new(cx + r, cy - r, 0.0),
            v_hb[3].clone(),
            v_hb[0].clone(),
        )?;

        // 上面穴円弧 (z=dz, 反時計回り: 0->1->2->3->0)
        let arc_ht01 = make_hole_arc(
            p_ht0,
            p_ht1,
            Point3::new(cx + r, cy + r, dz),
            v_ht[0].clone(),
            v_ht[1].clone(),
        )?;
        let arc_ht12 = make_hole_arc(
            p_ht1,
            p_ht2,
            Point3::new(cx - r, cy + r, dz),
            v_ht[1].clone(),
            v_ht[2].clone(),
        )?;
        let arc_ht23 = make_hole_arc(
            p_ht2,
            p_ht3,
            Point3::new(cx - r, cy - r, dz),
            v_ht[2].clone(),
            v_ht[3].clone(),
        )?;
        let arc_ht30 = make_hole_arc(
            p_ht3,
            p_ht0,
            Point3::new(cx + r, cy - r, dz),
            v_ht[3].clone(),
            v_ht[0].clone(),
        )?;

        // 穴の垂直エッジ 4本 (v_hb[i] -> v_ht[i])
        let ehv0 = Edge::line_between(v_hb[0].clone(), v_ht[0].clone())?;
        let ehv1 = Edge::line_between(v_hb[1].clone(), v_ht[1].clone())?;
        let ehv2 = Edge::line_between(v_hb[2].clone(), v_ht[2].clone())?;
        let ehv3 = Edge::line_between(v_hb[3].clone(), v_ht[3].clone())?;

        let mut faces = Vec::new();

        // 5. 外側側面4面 (Front, Right, Back, Left)
        // Front Face (-Y): vb0 -> vb1 -> vt1 -> vt0
        let p_front = PlaneSurface3::new(p_b0, Vec3::new(1.0, 0.0, 0.0), Vec3::new(0.0, 0.0, 1.0))
            .ok_or("plane front")?;
        faces.push(Face::simple(
            FaceGeometry::Plane(p_front),
            Wire::new(vec![
                OrientedEdge::forward(eb01.clone()),
                OrientedEdge::forward(ev1.clone()),
                OrientedEdge::reversed(et01.clone()),
                OrientedEdge::reversed(ev0.clone()),
            ]),
        ));

        // Right Face (+X): vb1 -> vb2 -> vt2 -> vt1
        let p_right = PlaneSurface3::new(p_b1, Vec3::new(0.0, 1.0, 0.0), Vec3::new(0.0, 0.0, 1.0))
            .ok_or("plane right")?;
        faces.push(Face::simple(
            FaceGeometry::Plane(p_right),
            Wire::new(vec![
                OrientedEdge::forward(eb12.clone()),
                OrientedEdge::forward(ev2.clone()),
                OrientedEdge::reversed(et12.clone()),
                OrientedEdge::reversed(ev1.clone()),
            ]),
        ));

        // Back Face (+Y): vb2 -> vb3 -> vt3 -> vt2
        let p_back = PlaneSurface3::new(p_b2, Vec3::new(-1.0, 0.0, 0.0), Vec3::new(0.0, 0.0, 1.0))
            .ok_or("plane back")?;
        faces.push(Face::simple(
            FaceGeometry::Plane(p_back),
            Wire::new(vec![
                OrientedEdge::forward(eb23.clone()),
                OrientedEdge::forward(ev3.clone()),
                OrientedEdge::reversed(et23.clone()),
                OrientedEdge::reversed(ev2.clone()),
            ]),
        ));

        // Left Face (-X): vb3 -> vb0 -> vt0 -> vt3
        let p_left = PlaneSurface3::new(p_b3, Vec3::new(0.0, -1.0, 0.0), Vec3::new(0.0, 0.0, 1.0))
            .ok_or("plane left")?;
        faces.push(Face::simple(
            FaceGeometry::Plane(p_left),
            Wire::new(vec![
                OrientedEdge::forward(eb30.clone()),
                OrientedEdge::forward(ev0.clone()),
                OrientedEdge::reversed(et30.clone()),
                OrientedEdge::reversed(ev3.clone()),
            ]),
        ));

        // 6. 内側円筒穴の4曲面Face（法線が内向き＝円筒内側から穴の中心を見る向き）
        let make_hole_cyl_patch = |p_s: Point3,
                                   p_e: Point3,
                                   corner_b: Point3,
                                   arc_b: Edge,
                                   arc_t: Edge,
                                   ev_s: Edge,
                                   ev_e: Edge|
         -> Result<Face, String> {
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
            let s = NurbsSurface3::new(
                2,
                1,
                vec![row0, row1, row2],
                KnotVector::clamped_uniform(3, 2),
                KnotVector::clamped_uniform(2, 1),
            )?;
            // 穴側面ワイヤ: p_s -> p_e -> p_e(top) -> p_s(top)
            let wire = Wire::new(vec![
                OrientedEdge::forward(arc_b),
                OrientedEdge::forward(ev_e),
                OrientedEdge::reversed(arc_t),
                OrientedEdge::reversed(ev_s),
            ]);
            Ok(Face::simple(FaceGeometry::Nurbs(s), wire))
        };

        // Hole Patch 0: v_hb[0] -> v_hb[1]
        faces.push(make_hole_cyl_patch(
            p_hb0,
            p_hb1,
            Point3::new(cx + r, cy + r, 0.0),
            arc_hb01.clone(),
            arc_ht01.clone(),
            ehv0.clone(),
            ehv1.clone(),
        )?);
        // Hole Patch 1: v_hb[1] -> v_hb[2]
        faces.push(make_hole_cyl_patch(
            p_hb1,
            p_hb2,
            Point3::new(cx - r, cy + r, 0.0),
            arc_hb12.clone(),
            arc_ht12.clone(),
            ehv1.clone(),
            ehv2.clone(),
        )?);
        // Hole Patch 2: v_hb[2] -> v_hb[3]
        faces.push(make_hole_cyl_patch(
            p_hb2,
            p_hb3,
            Point3::new(cx - r, cy - r, 0.0),
            arc_hb23.clone(),
            arc_ht23.clone(),
            ehv2.clone(),
            ehv3.clone(),
        )?);
        // Hole Patch 3: v_hb[3] -> v_hb[0]
        faces.push(make_hole_cyl_patch(
            p_hb3,
            p_hb0,
            Point3::new(cx + r, cy - r, 0.0),
            arc_hb30.clone(),
            arc_ht30.clone(),
            ehv3.clone(),
            ehv0.clone(),
        )?);

        // 7. Bottom Face (-Z, PLANE + 穴 FACE_BOUND)
        // 外側ループ（反時計回り）: vb0 -> vb3 -> vb2 -> vb1 -> vb0
        let p_bot = PlaneSurface3::new(
            Point3::new(0.0, 0.0, 0.0),
            Vec3::new(0.0, 1.0, 0.0),
            Vec3::new(1.0, 0.0, 0.0),
        )
        .ok_or("plane bottom")?;
        let outer_wire_bot = Wire::new(vec![
            OrientedEdge::reversed(eb30.clone()),
            OrientedEdge::reversed(eb23.clone()),
            OrientedEdge::reversed(eb12.clone()),
            OrientedEdge::reversed(eb01.clone()),
        ]);
        // 内側穴ループ（外向き法線 -Z に対して時計回り、すなわち反転順）
        let inner_wire_bot = Wire::new(vec![
            OrientedEdge::reversed(arc_hb30.clone()),
            OrientedEdge::reversed(arc_hb23.clone()),
            OrientedEdge::reversed(arc_hb12.clone()),
            OrientedEdge::reversed(arc_hb01.clone()),
        ]);
        faces.push(Face::new(
            FaceGeometry::Plane(p_bot),
            outer_wire_bot,
            vec![inner_wire_bot],
            zenith_topo::Orientation::Forward,
            1e-6,
        ));

        // 8. Top Face (+Z, PLANE + 穴 FACE_BOUND)
        // 外側ループ（反時計回り）: vt0 -> vt1 -> vt2 -> vt3 -> vt0
        let p_top = PlaneSurface3::new(
            Point3::new(0.0, 0.0, dz),
            Vec3::new(1.0, 0.0, 0.0),
            Vec3::new(0.0, 1.0, 0.0),
        )
        .ok_or("plane top")?;
        let outer_wire_top = Wire::new(vec![
            OrientedEdge::forward(et01.clone()),
            OrientedEdge::forward(et12.clone()),
            OrientedEdge::forward(et23.clone()),
            OrientedEdge::forward(et30.clone()),
        ]);
        // 内側穴ループ（外向き法線 +Z に対して時計回り＝穴をくり抜く向き）
        let inner_wire_top = Wire::new(vec![
            OrientedEdge::forward(arc_ht01.clone()),
            OrientedEdge::forward(arc_ht12.clone()),
            OrientedEdge::forward(arc_ht23.clone()),
            OrientedEdge::forward(arc_ht30.clone()),
        ]);
        faces.push(Face::new(
            FaceGeometry::Plane(p_top),
            outer_wire_top,
            vec![inner_wire_top],
            zenith_topo::Orientation::Forward,
            1e-6,
        ));

        let shell = Shell::closed(faces);
        crate::validated_solid(shell)
    }
}
