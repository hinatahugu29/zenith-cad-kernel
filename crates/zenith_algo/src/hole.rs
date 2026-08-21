use zenith_geom::{ControlPoint3, KnotVector, NurbsCurve3, NurbsSurface3};
use zenith_math::Point3;
use zenith_topo::{Edge, Face, FaceGeometry, OrientedEdge, Shell, Solid, Vertex, Wire};

/// 穴あきソリッド（貫通穴・ポケット）ビルダー
pub struct HoleBuilder;

impl HoleBuilder {
    /// 直方体にZ軸方向の貫通円形穴を開けたソリッドを生成（4象限パッチマニホールド方式）
    pub fn make_drilled_box(dx: f64, dy: f64, dz: f64, hole_radius: f64) -> Result<Solid, String> {
        if hole_radius < 0.0 {
            return Err(format!(
                "Hole radius must not be negative, got {hole_radius}"
            ));
        }
        let r = hole_radius;
        if r <= 1e-6 {
            return crate::primitive::PrimitiveBuilder::make_box(dx, dy, dz);
        }
        if 2.0 * r >= dx.min(dy) {
            return Err(format!(
                "Hole radius {r} must be smaller than half the shorter side ({})",
                dx.min(dy) * 0.5
            ));
        }

        let cx = dx * 0.5;
        let cy = dy * 0.5;

        // 1. 直方体の8頂点（外側四角形）
        let p_b = [
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(dx, 0.0, 0.0),
            Point3::new(dx, dy, 0.0),
            Point3::new(0.0, dy, 0.0),
        ];
        let p_t = [
            Point3::new(0.0, 0.0, dz),
            Point3::new(dx, 0.0, dz),
            Point3::new(dx, dy, dz),
            Point3::new(0.0, dy, dz),
        ];

        let vb = [
            Vertex::from_point(p_b[0]),
            Vertex::from_point(p_b[1]),
            Vertex::from_point(p_b[2]),
            Vertex::from_point(p_b[3]),
        ];
        let vt = [
            Vertex::from_point(p_t[0]),
            Vertex::from_point(p_t[1]),
            Vertex::from_point(p_t[2]),
            Vertex::from_point(p_t[3]),
        ];

        // 2. 穴の8頂点（0度: +X, 90度: +Y, 180度: -X, 270度: -Y）
        let p_hb = [
            Point3::new(cx + r, cy, 0.0),
            Point3::new(cx, cy + r, 0.0),
            Point3::new(cx - r, cy, 0.0),
            Point3::new(cx, cy - r, 0.0),
        ];
        let p_ht = [
            Point3::new(cx + r, cy, dz),
            Point3::new(cx, cy + r, dz),
            Point3::new(cx - r, cy, dz),
            Point3::new(cx, cy - r, dz),
        ];

        let v_hb = [
            Vertex::from_point(p_hb[0]),
            Vertex::from_point(p_hb[1]),
            Vertex::from_point(p_hb[2]),
            Vertex::from_point(p_hb[3]),
        ];
        let v_ht = [
            Vertex::from_point(p_ht[0]),
            Vertex::from_point(p_ht[1]),
            Vertex::from_point(p_ht[2]),
            Vertex::from_point(p_ht[3]),
        ];

        // 3. 直方体外側エッジ群
        let eb = [
            Edge::line_between(vb[0].clone(), vb[1].clone())?, // 0->1 (-Y)
            Edge::line_between(vb[1].clone(), vb[2].clone())?, // 1->2 (+X)
            Edge::line_between(vb[2].clone(), vb[3].clone())?, // 2->3 (+Y)
            Edge::line_between(vb[3].clone(), vb[0].clone())?, // 3->0 (-X)
        ];
        let et = [
            Edge::line_between(vt[0].clone(), vt[1].clone())?, // 0->1 (-Y)
            Edge::line_between(vt[1].clone(), vt[2].clone())?, // 1->2 (+X)
            Edge::line_between(vt[2].clone(), vt[3].clone())?, // 2->3 (+Y)
            Edge::line_between(vt[3].clone(), vt[0].clone())?, // 3->0 (-X)
        ];
        let ev = [
            Edge::line_between(vb[0].clone(), vt[0].clone())?,
            Edge::line_between(vb[1].clone(), vt[1].clone())?,
            Edge::line_between(vb[2].clone(), vt[2].clone())?,
            Edge::line_between(vb[3].clone(), vt[3].clone())?,
        ];

        // 4. 穴のエッジ群（4つの有理円弧 + 4つの垂直エッジ）
        let weight = std::f64::consts::FRAC_1_SQRT_2;
        let make_arc = |p_s: Point3,
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

        // 下面穴円弧 (反時計回り CCW: 0->1->2->3->0)
        let arc_hb = [
            make_arc(p_hb[0], p_hb[1], Point3::new(cx + r, cy + r, 0.0), v_hb[0].clone(), v_hb[1].clone())?,
            make_arc(p_hb[1], p_hb[2], Point3::new(cx - r, cy + r, 0.0), v_hb[1].clone(), v_hb[2].clone())?,
            make_arc(p_hb[2], p_hb[3], Point3::new(cx - r, cy - r, 0.0), v_hb[2].clone(), v_hb[3].clone())?,
            make_arc(p_hb[3], p_hb[0], Point3::new(cx + r, cy - r, 0.0), v_hb[3].clone(), v_hb[0].clone())?,
        ];

        // 上面穴円弧 (反時計回り CCW: 0->1->2->3->0)
        let arc_ht = [
            make_arc(p_ht[0], p_ht[1], Point3::new(cx + r, cy + r, dz), v_ht[0].clone(), v_ht[1].clone())?,
            make_arc(p_ht[1], p_ht[2], Point3::new(cx - r, cy + r, dz), v_ht[1].clone(), v_ht[2].clone())?,
            make_arc(p_ht[2], p_ht[3], Point3::new(cx - r, cy - r, dz), v_ht[2].clone(), v_ht[3].clone())?,
            make_arc(p_ht[3], p_ht[0], Point3::new(cx + r, cy - r, dz), v_ht[3].clone(), v_ht[0].clone())?,
        ];

        // 穴の垂直エッジ 4本 (v_hb[i] -> v_ht[i])
        let ehv = [
            Edge::line_between(v_hb[0].clone(), v_ht[0].clone())?,
            Edge::line_between(v_hb[1].clone(), v_ht[1].clone())?,
            Edge::line_between(v_hb[2].clone(), v_ht[2].clone())?,
            Edge::line_between(v_hb[3].clone(), v_ht[3].clone())?,
        ];

        // 5. 底面・天面の斜め境界エッジ (4隅 vb[i] から 穴の点 v_hb[(i+3)%4] への直線)
        let diag_b = [
            Edge::line_between(vb[0].clone(), v_hb[3].clone())?,
            Edge::line_between(vb[1].clone(), v_hb[0].clone())?,
            Edge::line_between(vb[2].clone(), v_hb[1].clone())?,
            Edge::line_between(vb[3].clone(), v_hb[2].clone())?,
        ];

        let diag_t = [
            Edge::line_between(vt[0].clone(), v_ht[3].clone())?,
            Edge::line_between(vt[1].clone(), v_ht[0].clone())?,
            Edge::line_between(vt[2].clone(), v_ht[1].clone())?,
            Edge::line_between(vt[3].clone(), v_ht[2].clone())?,
        ];

        let mut faces = Vec::new();

        // 6. 外側側面4面 (Front, Right, Back, Left) - 法線外向き
        for i in 0..4 {
            let next_i = (i + 1) % 4;
            let p_s = p_b[i];
            let p_e = p_b[next_i];
            let row0 = vec![ControlPoint3::unweighted(p_s), ControlPoint3::unweighted(p_t[i])];
            let row1 = vec![ControlPoint3::unweighted(p_e), ControlPoint3::unweighted(p_t[next_i])];
            let s = NurbsSurface3::new(
                1, 1,
                vec![row0, row1],
                KnotVector::clamped_uniform(2, 1),
                KnotVector::clamped_uniform(2, 1),
            )?;
            let wire = Wire::new(vec![
                OrientedEdge::forward(eb[i].clone()),
                OrientedEdge::forward(ev[next_i].clone()),
                OrientedEdge::reversed(et[i].clone()),
                OrientedEdge::reversed(ev[i].clone()),
            ]);
            faces.push(Face::simple(FaceGeometry::Nurbs(s), wire));
        }

        // 7. 内側円筒穴の4曲面Face（法線が内向き＝軸を向く）
        for i in 0..4 {
            let next_i = (i + 1) % 4;
            let corner_b = match i {
                0 => Point3::new(cx + r, cy + r, 0.0),
                1 => Point3::new(cx - r, cy + r, 0.0),
                2 => Point3::new(cx - r, cy - r, 0.0),
                _ => Point3::new(cx + r, cy - r, 0.0),
            };
            let corner_t = Point3::new(corner_b.x, corner_b.y, dz);

            let row0 = vec![
                ControlPoint3::unweighted(p_hb[next_i]),
                ControlPoint3::unweighted(p_ht[next_i]),
            ];
            let row1 = vec![
                ControlPoint3::new(corner_b, weight),
                ControlPoint3::new(corner_t, weight),
            ];
            let row2 = vec![
                ControlPoint3::unweighted(p_hb[i]),
                ControlPoint3::unweighted(p_ht[i]),
            ];

            let s = NurbsSurface3::new(
                2, 1,
                vec![row0, row1, row2],
                KnotVector::clamped_uniform(3, 2),
                KnotVector::clamped_uniform(2, 1),
            )?;

            // 穴側面ワイヤ: ehv[i] -> arc_ht[i] -> reversed(ehv[next_i]) -> reversed(arc_hb[i])
            let wire = Wire::new(vec![
                OrientedEdge::forward(ehv[i].clone()),
                OrientedEdge::forward(arc_ht[i].clone()),
                OrientedEdge::reversed(ehv[next_i].clone()),
                OrientedEdge::reversed(arc_hb[i].clone()),
            ]);
            faces.push(Face::simple(FaceGeometry::Nurbs(s), wire));
        }

        // 8. 底面 4象限パッチ (法線 -Z: 外向き)
        // U: 外側 -> 内側 (2点), V: 円弧 CW (3点) -> 法線 -Z
        for i in 0..4 {
            let next_i = (i + 1) % 4;
            let prev_i = (i + 3) % 4;
            let corner_b = match prev_i {
                0 => Point3::new(cx + r, cy + r, 0.0),
                1 => Point3::new(cx - r, cy + r, 0.0),
                2 => Point3::new(cx - r, cy - r, 0.0),
                _ => Point3::new(cx + r, cy - r, 0.0),
            };

            let row0 = vec![
                ControlPoint3::unweighted(p_hb[i]),
                ControlPoint3::new(corner_b, weight),
                ControlPoint3::unweighted(p_hb[prev_i]),
            ];
            let row1 = vec![
                ControlPoint3::unweighted(p_b[next_i]),
                ControlPoint3::unweighted(Point3::new((p_b[i].x + p_b[next_i].x)*0.5, (p_b[i].y + p_b[next_i].y)*0.5, 0.0)),
                ControlPoint3::unweighted(p_b[i]),
            ];

            let s = NurbsSurface3::new(
                1, 2,
                vec![row0, row1],
                KnotVector::clamped_uniform(2, 1),
                KnotVector::clamped_uniform(3, 2),
            )?;

            // -Z 外向き法線から見てCCW:
            // p_b[next_i] -> p_b[i] -> p_hb[prev_i] -> p_hb[i] -> p_b[next_i]
            let wire = Wire::new(vec![
                OrientedEdge::reversed(eb[i].clone()),
                OrientedEdge::forward(diag_b[i].clone()),
                OrientedEdge::forward(arc_hb[prev_i].clone()),
                OrientedEdge::reversed(diag_b[next_i].clone()),
            ]);
            faces.push(Face::simple(FaceGeometry::Nurbs(s), wire));
        }

        // 9. 天面 4象限パッチ (法線 +Z: 外向き)
        // U: 外側 -> 内側 (2点), V: 円弧 CCW (3点) -> 法線 +Z
        for i in 0..4 {
            let next_i = (i + 1) % 4;
            let prev_i = (i + 3) % 4;
            let corner_t = match prev_i {
                0 => Point3::new(cx + r, cy + r, dz),
                1 => Point3::new(cx - r, cy + r, dz),
                2 => Point3::new(cx - r, cy - r, dz),
                _ => Point3::new(cx + r, cy - r, dz),
            };

            let row0 = vec![
                ControlPoint3::unweighted(p_ht[prev_i]),
                ControlPoint3::new(corner_t, weight),
                ControlPoint3::unweighted(p_ht[i]),
            ];
            let row1 = vec![
                ControlPoint3::unweighted(p_t[i]),
                ControlPoint3::unweighted(Point3::new((p_t[i].x + p_t[next_i].x)*0.5, (p_t[i].y + p_t[next_i].y)*0.5, dz)),
                ControlPoint3::unweighted(p_t[next_i]),
            ];

            let s = NurbsSurface3::new(
                1, 2,
                vec![row0, row1],
                KnotVector::clamped_uniform(2, 1),
                KnotVector::clamped_uniform(3, 2),
            )?;


            // +Z 外向き法線から見てCCW:
            // p_t[i] -> p_t[next_i] -> p_ht[i] -> p_ht[prev_i] -> p_t[i]
            let wire = Wire::new(vec![
                OrientedEdge::forward(et[i].clone()),
                OrientedEdge::forward(diag_t[next_i].clone()),
                OrientedEdge::reversed(arc_ht[prev_i].clone()),
                OrientedEdge::reversed(diag_t[i].clone()),
            ]);
            faces.push(Face::simple(FaceGeometry::Nurbs(s), wire));
        }

        let shell = Shell::closed(faces);
        crate::validated_solid(shell)
    }

    /// 直方体にザグリ穴（Counterbore: ボルト頭沈め穴）を開けたソリッドを生成
    ///
    /// `dx`, `dy`, `dz`: 箱の寸法
    /// `hole_radius`: 貫通下穴の半径
    /// `cb_radius`: ザグリ部（大径）の半径
    /// `cb_depth`: ザグリ部の深さ（上面から下向き）
    pub fn make_counterbore_hole_box(
        dx: f64,
        dy: f64,
        dz: f64,
        hole_radius: f64,
        cb_radius: f64,
        cb_depth: f64,
    ) -> Result<Solid, String> {
        if cb_radius <= hole_radius {
            return Err("Counterbore radius must be larger than hole radius".to_string());
        }
        if cb_depth <= 0.0 || cb_depth >= dz {
            return Err("Counterbore depth must be between 0 and box thickness".to_string());
        }
        let tol = zenith_math::Tolerance::default();
        let box_solid = crate::PrimitiveBuilder::make_box(dx, dy, dz)?;

        let cx = dx * 0.5;
        let cy = dy * 0.5;

        // 1. 貫通下穴
        let through_drill = crate::PrimitiveBuilder::make_cylinder(hole_radius, dz + 2.0)?;
        let through_drill = crate::BrepTransform::translate_solid(
            &through_drill,
            zenith_math::Vec3::new(cx, cy, -1.0),
        );
        let drilled = crate::BooleanEngine::boolean_solids_exact(
            &box_solid,
            &through_drill,
            crate::BooleanOpType::Difference,
            &tol,
        )?;

        // 2. ザグリ穴（上面 dz から深さ cb_depth）
        let cb_drill = crate::PrimitiveBuilder::make_cylinder(cb_radius, cb_depth + 1.0)?;
        let cb_drill = crate::BrepTransform::translate_solid(
            &cb_drill,
            zenith_math::Vec3::new(cx, cy, dz - cb_depth),
        );
        crate::BooleanEngine::boolean_solids_exact(
            &drilled,
            &cb_drill,
            crate::BooleanOpType::Difference,
            &tol,
        )
    }
}
