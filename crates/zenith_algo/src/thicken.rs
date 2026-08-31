use zenith_geom::{CoonsPatch3, NurbsCurve3, NurbsSurface3, PlaneSurface3};
use zenith_math::{Point3, Tolerance};
use zenith_topo::{
    Edge, Face, FaceGeometry, Orientation, OrientedEdge, Shell, Solid, Vertex, Wire,
};

/// 自由曲面シート厚み付け（Thicken Sheet to Solid）ビルダー
pub struct ThickenBuilder;

impl ThickenBuilder {
    /// 曲面をずらすときの標本の細かさの既定値。
    pub const DEFAULT_OFFSET_SAMPLES: usize = 16;
}

impl ThickenBuilder {
    /// 単一の自由曲面Faceに均一な厚み `thickness` を与えて完全閉B-Repソリッド化
    pub fn thicken_face(face: &Face, thickness: f64, tol: &Tolerance) -> Result<Solid, String> {
        Self::thicken_face_with_samples(face, thickness, Self::DEFAULT_OFFSET_SAMPLES, tol)
    }

    /// 開いたシートシェル（複数Face）全体に均一な厚み `thickness` を与えて完全閉B-Repソリッド化
    pub fn thicken_shell(shell: &Shell, thickness: f64, tol: &Tolerance) -> Result<Solid, String> {
        if shell.faces.is_empty() {
            return Err("Cannot thicken an empty shell".to_string());
        }
        if shell.faces.len() == 1 {
            return Self::thicken_face(&shell.faces[0], thickness, tol);
        }

        // 複数面の場合、各パッチを厚み付けして結合（Boolean Union）
        let mut solid = Self::thicken_face(&shell.faces[0], thickness, tol)?;
        for face in shell.faces.iter().skip(1) {
            let next_solid = Self::thicken_face(face, thickness, tol)?;
            solid = crate::BooleanEngine::boolean_solids_exact(
                &solid,
                &next_solid,
                crate::BooleanOpType::Union,
                tol,
            )?;
        }
        Ok(solid)
    }

    /// 曲面をずらすときの標本の細かさを指定して厚みを付ける。
    ///
    /// 厳密なオフセット曲面は一般に NURBS では表せない。ここは曲面を標本して
    /// ずらし、通し直したものなので、細かさがそのまま精度になる。半径10の
    /// 円柱の四半パッチを 1 だけ厚くしたときの、閉じた式との差:
    ///
    /// ```text
    /// samples   8        16       24
    /// rel    8.27e-5  4.49e-6  8.25e-7
    /// ```
    ///
    /// 3次の補間なので、細かさの**4乗**で縮む。
    pub fn thicken_face_with_samples(
        face: &Face,
        thickness: f64,
        samples: usize,
        tol: &Tolerance,
    ) -> Result<Solid, String> {
        if thickness.abs() <= 1e-6 {
            return Err("Thickness must be non-zero".to_string());
        }

        match &face.geometry {
            FaceGeometry::Plane(plane) => Self::thicken_planar_face(face, plane, thickness),
            FaceGeometry::Nurbs(nurbs) => {
                Self::thicken_nurbs_face(face, nurbs, thickness, samples.max(4), tol)
            }
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

    /// 曲面シートに厚みを与える。
    ///
    /// # 以前どうだったか
    ///
    /// 天面は**隅1点の法線**で全制御点をずらしていました。法線が場所によって
    /// 変わる面ではただの平行移動になり、円柱の四半パッチでは天面が横へ
    /// ずれるだけでした。側面と境界も4隅を**直線**で結んでおり、弧を持つ
    /// パッチの縁からは外れます。シェル検証が「境界点が面から 2.93 外れて
    /// いる」と弾いていたので、誤った立体が出回ることはありませんでしたが、
    /// 曲面シートは作れていませんでした。
    ///
    /// # いまどうするか
    ///
    /// 1. 曲面を格子で標本し、**各点の法線**に沿って `thickness` だけずらす。
    /// 2. そのずらした点列を通る曲面を補間して天面にする
    ///    （[`NurbsSurface3::interpolate_points`]）。
    /// 3. 縁は等パラメータ曲線をそのまま使い、側面は下の縁と上の縁を結ぶ
    ///    ルールド曲面にする。直線で結ばないので、弧の縁にも乗る。
    ///
    /// # 何が残っているか
    ///
    /// 厳密なオフセット曲面は一般に NURBS では表せません。ここは標本して
    /// 通し直したものなので、**標本の細かさぶんの近似**です。曲率半径より
    /// 厚みが大きいと面が自分と交わりますが、それは見ていません。
    fn thicken_nurbs_face(
        _face: &Face,
        nurbs: &NurbsSurface3,
        thickness: f64,
        samples: usize,
        tol: &Tolerance,
    ) -> Result<Solid, String> {
        let sample_count = samples;

        let ((u_min, u_max), (v_min, v_max)) = nurbs.param_range();
        let at = |i: usize, j: usize| -> (f64, f64) {
            (
                u_min + (u_max - u_min) * i as f64 / sample_count as f64,
                v_min + (v_max - v_min) * j as f64 / sample_count as f64,
            )
        };

        // 1. 各標本点を、その点の法線に沿ってずらす。
        let mut offset_grid: Vec<Vec<Point3>> = Vec::with_capacity(sample_count + 1);
        for i in 0..=sample_count {
            let mut row = Vec::with_capacity(sample_count + 1);
            for j in 0..=sample_count {
                let (u, v) = at(i, j);
                let point = nurbs.evaluate(u, v);
                let normal = nurbs
                    .normal(u, v)
                    .ok_or("the surface has no normal to offset along")?;
                row.push(point + normal * thickness);
            }
            offset_grid.push(row);
        }

        // 2. ずらした点を通る曲面を起こす。
        let top_nurbs = NurbsSurface3::interpolate_points(
            nurbs.degree_u.max(2),
            nurbs.degree_v.max(2),
            &offset_grid,
        )?;

        // 3. 縁は等パラメータ曲線。直線で結ぶと、弧の縁から外れる。
        //
        //    4本が**巡回になるよう向きを揃える**。等パラメータ曲線は
        //    そのままだと 2本が逆走するので、そこは反転して繋ぐ。揃えないと、
        //    同じ辺が両隣から同じ向きに使われてシェルが閉じない。
        let iso_v = |value: f64| -> Result<NurbsCurve3, String> {
            nurbs
                .iso_curve_v(value)
                .ok_or_else(|| "could not take an iso-curve of the sheet".to_string())
        };
        let iso_u = |value: f64| -> Result<NurbsCurve3, String> {
            nurbs
                .iso_curve_u(value)
                .ok_or_else(|| "could not take an iso-curve of the sheet".to_string())
        };
        let ((tu_min, tu_max), (tv_min, tv_max)) = top_nurbs.param_range();
        let top_iso_v = |value: f64| -> Result<NurbsCurve3, String> {
            top_nurbs
                .iso_curve_v(value)
                .ok_or_else(|| "could not take an iso-curve of the offset sheet".to_string())
        };
        let top_iso_u = |value: f64| -> Result<NurbsCurve3, String> {
            top_nurbs
                .iso_curve_u(value)
                .ok_or_else(|| "could not take an iso-curve of the offset sheet".to_string())
        };

        // c00 -> c10 -> c11 -> c01 -> c00 の順に一周する。
        let bottom_curves = [
            iso_v(v_min)?,
            iso_u(u_max)?,
            iso_v(v_max)?.reversed(),
            iso_u(u_min)?.reversed(),
        ];
        let top_curves = [
            top_iso_v(tv_min)?,
            top_iso_u(tu_max)?,
            top_iso_v(tv_max)?.reversed(),
            top_iso_u(tu_min)?.reversed(),
        ];

        let make_edge = |curve: NurbsCurve3| {
            let (a, b) = curve.param_range();
            let start = Vertex::from_point(curve.evaluate(a));
            let end = Vertex::from_point(curve.evaluate(b));
            Edge::new(curve, start, end, 1e-6)
        };
        let bottom_edges: Vec<Edge> = bottom_curves.iter().cloned().map(make_edge).collect();
        let top_edges: Vec<Edge> = top_curves.iter().cloned().map(make_edge).collect();

        // 縦の辺は隅ごとに1本だけ作り、両隣の側面で分け合う。側面ごとに作ると
        // 別物になり、辺が対にならない。
        let corner_edges: Vec<Edge> = (0..4)
            .map(|index| {
                Edge::line_between(
                    bottom_edges[index].start_vertex.clone(),
                    top_edges[index].start_vertex.clone(),
                )
            })
            .collect::<Result<Vec<_>, _>>()?;

        let mut faces = Vec::with_capacity(6);

        // 4. 側面は4境界の Coons パッチ。下の縁・上の縁・両端の縦線を
        //    **そのまま**境界に持つので、面と境界が食い違わない。
        //
        //    1次のルールド曲面として作るには、下と上を同じ次数・同じノットに
        //    揃える必要がある。`make_compatible` はそれを**再標本**で行うので
        //    曲線の形が変わり、境界が自分の面から 4.95e-3 外れた。次数上げが
        //    要るところを標本で済ませてはいけない。
        for index in 0..4 {
            let rise_start = NurbsCurve3::bspline_from_points(
                1,
                vec![
                    bottom_edges[index].start_vertex.point,
                    top_edges[index].start_vertex.point,
                ],
            )?;
            let rise_end = NurbsCurve3::bspline_from_points(
                1,
                vec![
                    bottom_edges[index].end_vertex.point,
                    top_edges[index].end_vertex.point,
                ],
            )?;
            let wall = CoonsPatch3::new(
                bottom_curves[index].clone(),
                top_curves[index].clone(),
                rise_start,
                rise_end,
                &tol,
            )?;

            faces.push(Face::simple(
                FaceGeometry::Coons(wall),
                Wire::new(vec![
                    OrientedEdge::forward(bottom_edges[index].clone()),
                    OrientedEdge::forward(corner_edges[(index + 1) % 4].clone()),
                    OrientedEdge::reversed(top_edges[index].clone()),
                    OrientedEdge::reversed(corner_edges[index].clone()),
                ]),
            ));
        }

        // 下面は、側面が使ったのと逆向きに一周する。
        faces.push(Face::new(
            FaceGeometry::Nurbs(nurbs.clone()),
            Wire::new(
                (0..4)
                    .rev()
                    .map(|index| OrientedEdge::reversed(bottom_edges[index].clone()))
                    .collect(),
            ),
            vec![],
            Orientation::Reversed,
            1e-6,
        ));

        faces.push(Face::simple(
            FaceGeometry::Nurbs(top_nurbs),
            Wire::new(
                top_edges
                    .iter()
                    .map(|edge| OrientedEdge::forward(edge.clone()))
                    .collect(),
            ),
        ));

        let shell = Shell::closed(faces);
        crate::validated_solid(shell)
    }
}
