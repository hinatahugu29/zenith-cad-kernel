use zenith_geom::{CoonsPatch3, NurbsSurface3, PlaneSurface3};
use zenith_math::Tolerance;
use zenith_topo::{
    Edge, Face, FaceGeometry, Orientation, OrientedEdge, Shell, Solid, Vertex, Wire,
};

/// 自由曲面シート厚み付け（Thicken Sheet to Solid）ビルダー
pub struct ThickenBuilder;

impl ThickenBuilder {
    /// 単一の自由曲面Faceに均一な厚み `thickness` を与えて完全閉B-Repソリッド化
    pub fn thicken_face(face: &Face, thickness: f64, _tol: &Tolerance) -> Result<Solid, String> {
        if thickness.abs() <= 1e-6 {
            return Err("Thickness must be non-zero".to_string());
        }

        match &face.geometry {
            FaceGeometry::Plane(plane) => Self::thicken_planar_face(face, plane, thickness),
            FaceGeometry::Nurbs(nurbs) => Self::thicken_nurbs_face(face, nurbs, thickness),
            FaceGeometry::Coons(coons) => Self::thicken_coons_face(face, coons, thickness),
            _ => Err("Unsupported surface geometry for thicken".to_string()),
        }
    }

    /// Coons パッチ（4境界曲線パッチ）シートの厚み付け。
    ///
    /// 各境界曲線の制御点を、その制御点に対応するパラメータ位置での曲面法線方向へ
    /// `thickness` だけオフセットして天面 Coons パッチを構築する。
    /// 4隅の制御点は隣接する2曲線で必ず同一の法線を用いるため、
    /// オフセット後も `CoonsPatch3::new` のコーナー連続性検証を通過する。
    fn thicken_coons_face(
        _face: &Face,
        coons: &CoonsPatch3,
        thickness: f64,
    ) -> Result<Solid, String> {
        let tol = Tolerance::default();

        // 4隅（Coons パラメータ域は [0,1] x [0,1] 固定）
        let p00_b = coons.evaluate(0.0, 0.0);
        let p10_b = coons.evaluate(1.0, 0.0);
        let p11_b = coons.evaluate(1.0, 1.0);
        let p01_b = coons.evaluate(0.0, 1.0);

        let n00 = coons.normal(0.0, 0.0).ok_or("normal 00 fail")?;
        let n10 = coons.normal(1.0, 0.0).ok_or("normal 10 fail")?;
        let n11 = coons.normal(1.0, 1.0).ok_or("normal 11 fail")?;
        let n01 = coons.normal(0.0, 1.0).ok_or("normal 01 fail")?;

        let p00_t = p00_b + n00 * thickness;
        let p10_t = p10_b + n10 * thickness;
        let p11_t = p11_b + n11 * thickness;
        let p01_t = p01_b + n01 * thickness;

        // 境界曲線を法線方向へオフセット（`along_u` = u方向に走る境界か）
        let offset_boundary = |curve: &zenith_geom::NurbsCurve3, along_u: bool, fixed: f64| {
            let n = curve.control_points.len();
            let mut cps = curve.control_points.clone();
            for (i, cp) in cps.iter_mut().enumerate() {
                let t = if n <= 1 {
                    0.0
                } else {
                    i as f64 / (n - 1) as f64
                };
                let (u, v) = if along_u { (t, fixed) } else { (fixed, t) };
                let nrm = coons.normal(u, v).unwrap_or(n00);
                cp.point += nrm * thickness;
            }
            zenith_geom::NurbsCurve3::new(curve.degree, cps, curve.knots.clone())
        };

        let c0_t = offset_boundary(&coons.c0, true, 0.0)?;
        let c1_t = offset_boundary(&coons.c1, true, 1.0)?;
        let d0_t = offset_boundary(&coons.d0, false, 0.0)?;
        let d1_t = offset_boundary(&coons.d1, false, 1.0)?;

        let top_coons = CoonsPatch3::new(c0_t, c1_t, d0_t, d1_t, &tol)?;

        let v00_b = Vertex::from_point(p00_b);
        let v10_b = Vertex::from_point(p10_b);
        let v11_b = Vertex::from_point(p11_b);
        let v01_b = Vertex::from_point(p01_b);

        let v00_t = Vertex::from_point(p00_t);
        let v10_t = Vertex::from_point(p10_t);
        let v11_t = Vertex::from_point(p11_t);
        let v01_t = Vertex::from_point(p01_t);

        // 底面・天面・垂直エッジ
        let e_b0 = Edge::line_between(v00_b.clone(), v10_b.clone())?;
        let e_b1 = Edge::line_between(v10_b.clone(), v11_b.clone())?;
        let e_b2 = Edge::line_between(v11_b.clone(), v01_b.clone())?;
        let e_b3 = Edge::line_between(v01_b.clone(), v00_b.clone())?;

        let e_t0 = Edge::line_between(v00_t.clone(), v10_t.clone())?;
        let e_t1 = Edge::line_between(v10_t.clone(), v11_t.clone())?;
        let e_t2 = Edge::line_between(v11_t.clone(), v01_t.clone())?;
        let e_t3 = Edge::line_between(v01_t.clone(), v00_t.clone())?;

        let e_v0 = Edge::line_between(v00_b.clone(), v00_t.clone())?;
        let e_v1 = Edge::line_between(v10_b.clone(), v10_t.clone())?;
        let e_v2 = Edge::line_between(v11_b.clone(), v11_t.clone())?;
        let e_v3 = Edge::line_between(v01_b.clone(), v01_t.clone())?;

        let mut faces = Vec::with_capacity(6);

        let p_side0 = PlaneSurface3::new(p00_b, p10_b - p00_b, n00 * thickness).ok_or("side 0")?;
        faces.push(Face::simple(
            FaceGeometry::Plane(p_side0),
            Wire::new(vec![
                OrientedEdge::forward(e_b0.clone()),
                OrientedEdge::forward(e_v1.clone()),
                OrientedEdge::reversed(e_t0.clone()),
                OrientedEdge::reversed(e_v0.clone()),
            ]),
        ));

        let p_side1 = PlaneSurface3::new(p10_b, p11_b - p10_b, n10 * thickness).ok_or("side 1")?;
        faces.push(Face::simple(
            FaceGeometry::Plane(p_side1),
            Wire::new(vec![
                OrientedEdge::forward(e_b1.clone()),
                OrientedEdge::forward(e_v2.clone()),
                OrientedEdge::reversed(e_t1.clone()),
                OrientedEdge::reversed(e_v1.clone()),
            ]),
        ));

        let p_side2 = PlaneSurface3::new(p11_b, p01_b - p11_b, n11 * thickness).ok_or("side 2")?;
        faces.push(Face::simple(
            FaceGeometry::Plane(p_side2),
            Wire::new(vec![
                OrientedEdge::forward(e_b2.clone()),
                OrientedEdge::forward(e_v3.clone()),
                OrientedEdge::reversed(e_t2.clone()),
                OrientedEdge::reversed(e_v2.clone()),
            ]),
        ));

        let p_side3 = PlaneSurface3::new(p01_b, p00_b - p01_b, n01 * thickness).ok_or("side 3")?;
        faces.push(Face::simple(
            FaceGeometry::Plane(p_side3),
            Wire::new(vec![
                OrientedEdge::forward(e_b3.clone()),
                OrientedEdge::forward(e_v0.clone()),
                OrientedEdge::reversed(e_t3.clone()),
                OrientedEdge::reversed(e_v3.clone()),
            ]),
        ));

        // 底面（元シート・法線反転）
        faces.push(Face::new(
            FaceGeometry::Coons(coons.clone()),
            Wire::new(vec![
                OrientedEdge::reversed(e_b3),
                OrientedEdge::reversed(e_b2),
                OrientedEdge::reversed(e_b1),
                OrientedEdge::reversed(e_b0),
            ]),
            vec![],
            Orientation::Reversed,
            1e-6,
        ));

        // 天面（オフセットシート）
        faces.push(Face::simple(
            FaceGeometry::Coons(top_coons),
            Wire::new(vec![
                OrientedEdge::forward(e_t0),
                OrientedEdge::forward(e_t1),
                OrientedEdge::forward(e_t2),
                OrientedEdge::forward(e_t3),
            ]),
        ));

        let shell = Shell::closed(faces);
        crate::validated_solid(shell)
    }

    fn thicken_planar_face(
        face: &Face,
        plane: &PlaneSurface3,
        thickness: f64,
    ) -> Result<Solid, String> {
        let n = plane.normal.normalize();
        let offset_vec = n * thickness;

        // 1. 底面ワイヤ（元のワイヤ）の頂点列を取得
        let mut orig_points = Vec::new();
        for oe in &face.outer_wire.edges {
            orig_points.push(oe.edge.start_vertex.point);
        }
        let num_pts = orig_points.len();
        if num_pts < 3 {
            return Err("Planar face requires at least 3 vertices".to_string());
        }

        // 2. オフセット天面の頂点列
        let mut top_points = Vec::with_capacity(num_pts);
        for p in &orig_points {
            top_points.push(*p + offset_vec);
        }

        let vb: Vec<Vertex> = orig_points.iter().map(|p| Vertex::from_point(*p)).collect();
        let vt: Vec<Vertex> = top_points.iter().map(|p| Vertex::from_point(*p)).collect();

        // 3. 底面エッジ・天面エッジ・垂直エッジの構築
        let mut eb = Vec::with_capacity(num_pts);
        let mut et = Vec::with_capacity(num_pts);
        let mut ev = Vec::with_capacity(num_pts);

        for i in 0..num_pts {
            let next = (i + 1) % num_pts;
            eb.push(Edge::line_between(vb[i].clone(), vb[next].clone())?);
            et.push(Edge::line_between(vt[i].clone(), vt[next].clone())?);
            ev.push(Edge::line_between(vb[i].clone(), vt[i].clone())?);
        }

        let mut faces = Vec::with_capacity(num_pts + 2);

        // 4. 側面Faces
        for i in 0..num_pts {
            let next = (i + 1) % num_pts;
            let p_orig = vb[i].point;
            let u = vb[next].point - vb[i].point;
            let v = offset_vec;
            let side_plane =
                PlaneSurface3::new(p_orig, u, v).ok_or("Side plane creation failed")?;
            let side_wire = Wire::new(vec![
                OrientedEdge::forward(eb[i].clone()),
                OrientedEdge::forward(ev[next].clone()),
                OrientedEdge::reversed(et[i].clone()),
                OrientedEdge::reversed(ev[i].clone()),
            ]);
            faces.push(Face::simple(FaceGeometry::Plane(side_plane), side_wire));
        }

        // 5. 底面 (反時計回り反転)
        let bot_plane =
            PlaneSurface3::new(plane.origin, plane.v_axis, plane.u_axis).ok_or("Bot plane fail")?;
        let mut bot_edges = Vec::with_capacity(num_pts);
        for i in (0..num_pts).rev() {
            bot_edges.push(OrientedEdge::reversed(eb[i].clone()));
        }
        faces.push(Face::simple(
            FaceGeometry::Plane(bot_plane),
            Wire::new(bot_edges),
        ));

        // 6. 天面
        let top_plane = PlaneSurface3::new(plane.origin + offset_vec, plane.u_axis, plane.v_axis)
            .ok_or("Top plane fail")?;
        let mut top_edges = Vec::with_capacity(num_pts);
        for edge in et.iter().take(num_pts) {
            top_edges.push(OrientedEdge::forward(edge.clone()));
        }
        faces.push(Face::simple(
            FaceGeometry::Plane(top_plane),
            Wire::new(top_edges),
        ));

        let shell = Shell::closed(faces);
        crate::validated_solid(shell)
    }

    fn thicken_nurbs_face(
        _face: &Face,
        nurbs: &NurbsSurface3,
        thickness: f64,
    ) -> Result<Solid, String> {
        let ((u_min, u_max), (v_min, v_max)) = nurbs.param_range();

        // 4隅頂点の評価
        let p00_b = nurbs.evaluate(u_min, v_min);
        let p10_b = nurbs.evaluate(u_max, v_min);
        let p11_b = nurbs.evaluate(u_max, v_max);
        let p01_b = nurbs.evaluate(u_min, v_max);

        let n00 = nurbs.normal(u_min, v_min).ok_or("normal 00 fail")?;
        let n10 = nurbs.normal(u_max, v_min).ok_or("normal 10 fail")?;
        let n11 = nurbs.normal(u_max, v_max).ok_or("normal 11 fail")?;
        let n01 = nurbs.normal(u_min, v_max).ok_or("normal 01 fail")?;

        let p00_t = p00_b + n00 * thickness;
        let p10_t = p10_b + n10 * thickness;
        let p11_t = p11_b + n11 * thickness;
        let p01_t = p01_b + n01 * thickness;

        let v00_b = Vertex::from_point(p00_b);
        let v10_b = Vertex::from_point(p10_b);
        let v11_b = Vertex::from_point(p11_b);
        let v01_b = Vertex::from_point(p01_b);

        let v00_t = Vertex::from_point(p00_t);
        let v10_t = Vertex::from_point(p10_t);
        let v11_t = Vertex::from_point(p11_t);
        let v01_t = Vertex::from_point(p01_t);

        // 1. 底面エッジ（4本）
        let e_b0 = Edge::line_between(v00_b.clone(), v10_b.clone())?;
        let e_b1 = Edge::line_between(v10_b.clone(), v11_b.clone())?;
        let e_b2 = Edge::line_between(v11_b.clone(), v01_b.clone())?;
        let e_b3 = Edge::line_between(v01_b.clone(), v00_b.clone())?;

        // 2. 天面エッジ（4本）
        let e_t0 = Edge::line_between(v00_t.clone(), v10_t.clone())?;
        let e_t1 = Edge::line_between(v10_t.clone(), v11_t.clone())?;
        let e_t2 = Edge::line_between(v11_t.clone(), v01_t.clone())?;
        let e_t3 = Edge::line_between(v01_t.clone(), v00_t.clone())?;

        // 3. 垂直エッジ（4本）
        let e_v0 = Edge::line_between(v00_b.clone(), v00_t.clone())?;
        let e_v1 = Edge::line_between(v10_b.clone(), v10_t.clone())?;
        let e_v2 = Edge::line_between(v11_b.clone(), v11_t.clone())?;
        let e_v3 = Edge::line_between(v01_b.clone(), v01_t.clone())?;

        // 4. 天面NURBS曲面（オフセット制御点）
        let mut top_cps = nurbs.control_points.clone();
        for row in &mut top_cps {
            for cp in row {
                cp.point += n00 * thickness;
            }
        }
        let top_nurbs = NurbsSurface3::new(
            nurbs.degree_u,
            nurbs.degree_v,
            top_cps,
            nurbs.knots_u.clone(),
            nurbs.knots_v.clone(),
        )?;

        let mut faces = Vec::with_capacity(6);

        // 側面 0: u_min 側 (v00_b -> v10_b -> v10_t -> v00_t)
        let p_side0 = PlaneSurface3::new(p00_b, p10_b - p00_b, n00 * thickness).ok_or("side 0")?;
        faces.push(Face::simple(
            FaceGeometry::Plane(p_side0),
            Wire::new(vec![
                OrientedEdge::forward(e_b0.clone()),
                OrientedEdge::forward(e_v1.clone()),
                OrientedEdge::reversed(e_t0.clone()),
                OrientedEdge::reversed(e_v0.clone()),
            ]),
        ));

        // 側面 1: u_max 側 (v10_b -> v11_b -> v11_t -> v10_t)
        let p_side1 = PlaneSurface3::new(p10_b, p11_b - p10_b, n10 * thickness).ok_or("side 1")?;
        faces.push(Face::simple(
            FaceGeometry::Plane(p_side1),
            Wire::new(vec![
                OrientedEdge::forward(e_b1.clone()),
                OrientedEdge::forward(e_v2.clone()),
                OrientedEdge::reversed(e_t1.clone()),
                OrientedEdge::reversed(e_v1.clone()),
            ]),
        ));

        // 側面 2: v_max 側 (v11_b -> v01_b -> v01_t -> v11_t)
        let p_side2 = PlaneSurface3::new(p11_b, p01_b - p11_b, n11 * thickness).ok_or("side 2")?;
        faces.push(Face::simple(
            FaceGeometry::Plane(p_side2),
            Wire::new(vec![
                OrientedEdge::forward(e_b2.clone()),
                OrientedEdge::forward(e_v3.clone()),
                OrientedEdge::reversed(e_t2.clone()),
                OrientedEdge::reversed(e_v2.clone()),
            ]),
        ));

        // 側面 3: v_min 側 (v01_b -> v00_b -> v00_t -> v01_t)
        let p_side3 = PlaneSurface3::new(p01_b, p00_b - p01_b, n01 * thickness).ok_or("side 3")?;
        faces.push(Face::simple(
            FaceGeometry::Plane(p_side3),
            Wire::new(vec![
                OrientedEdge::forward(e_b3.clone()),
                OrientedEdge::forward(e_v0.clone()),
                OrientedEdge::reversed(e_t3.clone()),
                OrientedEdge::reversed(e_v3.clone()),
            ]),
        ));

        // 底面
        faces.push(Face::new(
            FaceGeometry::Nurbs(nurbs.clone()),
            Wire::new(vec![
                OrientedEdge::reversed(e_b3),
                OrientedEdge::reversed(e_b2),
                OrientedEdge::reversed(e_b1),
                OrientedEdge::reversed(e_b0),
            ]),
            vec![],
            Orientation::Reversed,
            1e-6,
        ));

        // 天面
        faces.push(Face::simple(
            FaceGeometry::Nurbs(top_nurbs),
            Wire::new(vec![
                OrientedEdge::forward(e_t0),
                OrientedEdge::forward(e_t1),
                OrientedEdge::forward(e_t2),
                OrientedEdge::forward(e_t3),
            ]),
        ));

        let shell = Shell::closed(faces);
        crate::validated_solid(shell)
    }
}
