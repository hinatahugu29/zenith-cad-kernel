use zenith_geom::{CoonsPatch3, NurbsCurve3, NurbsSurface3, PlaneSurface3};
use zenith_math::{Point3, Tolerance, Vec3, Vec3Ext};
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

        // **輪は、境界曲線そのものから組みます**（4-417）。
        //
        // **2026/09/09 まで、ここは 4 隅から角柱を組むだけ**でした——
        // 底も天も**隅どうしを結んだ直線**で輪を作り、側面は平面。
        // **境界曲線を一度も読んでいません**でした。**縁がまっすぐなら
        // それが正しい答え**ですが、**曲がっていたら中身の違う立体**が
        // 返ります（4-416。閉じた式 210 に対して 203.3）。
        //
        // **側面は、その境界曲線と、そのオフセット曲線のあいだの線織面**
        // です。**オフセットは制御点の数も次数もノットも変えない**ので、
        // **線織面はぴったり組めます**——`v` 方向は次数 1、制御点 2 個。
        // **`v=0` の等パラメータ曲線が下の縁、`v=1` が上の縁**に、
        // 構成上そのまま一致します。
        // **平らなシートだけを受けます**（4-417）。
        //
        // **縁の形は問いません**——**曲がった縁も通ります**（それが
        // この節で足したところです）。**問うのは、面そのものが平らか**
        // どうかです。
        //
        // **なぜか**: 天面は**境界曲線の制御点を法線方向へずらして**
        // 作ります。**平らなら法線はどこでも同じ**なので、これは
        // **ぴったりの平行移動**です。**曲がっていると、ずらした制御点の
        // 曲線は本当のオフセット曲線ではありません**——実測（4-417）:
        // 円弧に近い四半パッチ（長さ 15.527、回転角 1.309 rad）を 1 だけ
        // 厚くすると、**独立に積んだ値 323.633 に対して 320.007**。
        // **1.1% ずれ、しかも刻みを 16 → 128 と上げても縮みません**
        // （3.067e-2 → 2.989e-2）。**幾何のずれで、刻みのずれでは
        // ありません。**
        //
        // **1% 黙ってずれるより、断ります**（4-409 と同じ判断）。
        // **曲がったシートを厚くするには、`thicken_nurbs_face` が
        // やっているように曲面を標本してずらし、通し直す段が要ります。**
        {
            let mut points: Vec<Point3> = Vec::new();
            for curve in [&coons.c0, &coons.c1, &coons.d0, &coons.d1] {
                let (t0, t1) = curve.param_range();
                for step in 0..=16 {
                    points.push(curve.evaluate(t0 + (t1 - t0) * step as f64 / 16.0));
                }
            }
            for i in 0..=8 {
                for j in 0..=8 {
                    points.push(coons.evaluate(i as f64 / 8.0, j as f64 / 8.0));
                }
            }
            let origin = points[0];
            let normal = n00;
            let mut worst: f64 = 0.0;
            for point in &points {
                worst = worst.max((*point - origin).dot(&normal).abs());
            }
            if worst > tol.linear {
                return Err(format!(
                    "the sheet bulges {worst:.3e} out of the plane at its first corner; thicken only handles a flat Coons sheet (offsetting a curved Coons sheet is not implemented, and would be about 1% out)"
                ));
            }
        }

        let ruled = |bottom: &zenith_geom::NurbsCurve3,
                     top: &zenith_geom::NurbsCurve3|
         -> Result<NurbsSurface3, String> {
            if bottom.control_points.len() != top.control_points.len() {
                return Err("the offset boundary changed its control point count".to_string());
            }
            let grid: Vec<Vec<zenith_geom::ControlPoint3>> = bottom
                .control_points
                .iter()
                .zip(top.control_points.iter())
                .map(|(low, high)| vec![*low, *high])
                .collect();
            NurbsSurface3::new(
                bottom.degree,
                1,
                grid,
                bottom.knots.clone(),
                zenith_geom::KnotVector::clamped_uniform(2, 1),
            )
        };

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

        let top_coons = CoonsPatch3::new(c0_t.clone(), c1_t.clone(), d0_t.clone(), d1_t.clone(), &tol)?;

        let v00_b = Vertex::from_point(p00_b);
        let v10_b = Vertex::from_point(p10_b);
        let v11_b = Vertex::from_point(p11_b);
        let v01_b = Vertex::from_point(p01_b);

        let v00_t = Vertex::from_point(p00_t);
        let v10_t = Vertex::from_point(p10_t);
        let v11_t = Vertex::from_point(p11_t);
        let v01_t = Vertex::from_point(p01_t);

        // **縁の稜は、境界曲線そのもの**です。
        // `c0`: p00 -> p10、`c1`: p01 -> p11、`d0`: p00 -> p01、`d1`: p10 -> p11。
        let e_c0: Edge = Edge::new(coons.c0.clone(), v00_b.clone(), v10_b.clone(), tol.linear);
        let e_d1: Edge = Edge::new(coons.d1.clone(), v10_b.clone(), v11_b.clone(), tol.linear);
        let e_c1: Edge = Edge::new(coons.c1.clone(), v01_b.clone(), v11_b.clone(), tol.linear);
        let e_d0: Edge = Edge::new(coons.d0.clone(), v00_b.clone(), v01_b.clone(), tol.linear);

        let e_c0_t: Edge = Edge::new(c0_t.clone(), v00_t.clone(), v10_t.clone(), tol.linear);
        let e_d1_t: Edge = Edge::new(d1_t.clone(), v10_t.clone(), v11_t.clone(), tol.linear);
        let e_c1_t: Edge = Edge::new(c1_t.clone(), v01_t.clone(), v11_t.clone(), tol.linear);
        let e_d0_t: Edge = Edge::new(d0_t.clone(), v00_t.clone(), v01_t.clone(), tol.linear);

        let e_v0 = Edge::line_between(v00_b.clone(), v00_t.clone())?;
        let e_v1 = Edge::line_between(v10_b.clone(), v10_t.clone())?;
        let e_v2 = Edge::line_between(v11_b.clone(), v11_t.clone())?;
        let e_v3 = Edge::line_between(v01_b.clone(), v01_t.clone())?;

        let mut faces = Vec::with_capacity(6);

        // 側面 0（`c0` に沿う。p00 -> p10）
        faces.push(Face::simple(
            FaceGeometry::Nurbs(ruled(&coons.c0, &c0_t)?),
            Wire::new(vec![
                OrientedEdge::forward(e_c0.clone()),
                OrientedEdge::forward(e_v1.clone()),
                OrientedEdge::reversed(e_c0_t.clone()),
                OrientedEdge::reversed(e_v0.clone()),
            ]),
        ));

        // 側面 1（`d1` に沿う。p10 -> p11）
        faces.push(Face::simple(
            FaceGeometry::Nurbs(ruled(&coons.d1, &d1_t)?),
            Wire::new(vec![
                OrientedEdge::forward(e_d1.clone()),
                OrientedEdge::forward(e_v2.clone()),
                OrientedEdge::reversed(e_d1_t.clone()),
                OrientedEdge::reversed(e_v1.clone()),
            ]),
        ));

        // 側面 2（`c1` を逆に辿る。p11 -> p01）
        //
        // **ここだけ、向きの札を裏返します。** `c1` は `c0` と同じ向き
        // （u が増える向き）に走るので、**線織面の法線は 2 枚とも同じ側**
        // を向きます。**外へ向くのは片方だけ**です——実測（4-417）:
        // 裏返さないと `planar p-curve loop is inconsistent with face
        // orientation; oriented area -2.000000e1` で断られます。
        faces.push(Face::new(
            FaceGeometry::Nurbs(ruled(&coons.c1, &c1_t)?),
            Wire::new(vec![
                OrientedEdge::reversed(e_c1.clone()),
                OrientedEdge::forward(e_v3.clone()),
                OrientedEdge::forward(e_c1_t.clone()),
                OrientedEdge::reversed(e_v2.clone()),
            ]),
            vec![],
            Orientation::Reversed,
            1e-6,
        ));

        // 側面 3（`d0` を逆に辿る。p01 -> p00）。**側面 2 と同じ理由で
        // 裏返します。**
        faces.push(Face::new(
            FaceGeometry::Nurbs(ruled(&coons.d0, &d0_t)?),
            Wire::new(vec![
                OrientedEdge::reversed(e_d0.clone()),
                OrientedEdge::forward(e_v0.clone()),
                OrientedEdge::forward(e_d0_t.clone()),
                OrientedEdge::reversed(e_v3.clone()),
            ]),
            vec![],
            Orientation::Reversed,
            1e-6,
        ));

        // 底面（元シート・法線反転）。p00 -> p01 -> p11 -> p10 -> p00
        faces.push(Face::new(
            FaceGeometry::Coons(coons.clone()),
            Wire::new(vec![
                OrientedEdge::forward(e_d0),
                OrientedEdge::forward(e_c1),
                OrientedEdge::reversed(e_d1),
                OrientedEdge::reversed(e_c0),
            ]),
            vec![],
            Orientation::Reversed,
            1e-6,
        ));

        // 天面（オフセットシート）。p00 -> p10 -> p11 -> p01 -> p00
        faces.push(Face::simple(
            FaceGeometry::Coons(top_coons),
            Wire::new(vec![
                OrientedEdge::forward(e_c0_t),
                OrientedEdge::forward(e_d1_t),
                OrientedEdge::reversed(e_c1_t),
                OrientedEdge::reversed(e_d0_t),
            ]),
        ));

        let shell = Shell::closed(faces);
        crate::validated_solid(shell)
    }

    /// 平らなシートに厚みを付ける。
    ///
    /// # 以前どうしていたか
    ///
    /// **外周の稜の始点だけを拾って、直線で結び直して**いました。
    /// 側面も、その頂点どうしを結ぶ平面です。**内側の輪（穴）は
    /// 一度も読んでいませんでした。**
    ///
    /// どちらも**閉じた多様体で返ります**——角柱としては正しく閉じて
    /// いて、非多様体でもなく、内外判定も通ります。**大きさだけが
    /// 違う**ので、形の検査では捕まりません（4-409）。
    ///
    /// | 置き方 | 閉じた式 | 返っていた値 |
    /// | :--- | ---: | ---: |
    /// | 丸板（半径10・厚み3） | 942.477796 | **600.000000**（＝ 2r²t の内接正方形） |
    /// | 40×30 に φ10 の穴（厚み3） | 3364.380551 | **3600.000000**（＝ 穴なし） |
    ///
    /// # いまどうするか
    ///
    /// **平らな面を法線方向へ厚くすることは、その面を押し出すことと
    /// 同じ**です。押し出しの口（[`crate::ExtrudeBuilder::extrude_face_with_holes`]）は
    /// **稜の曲線をそのまま平行移動し**、**内側の輪には内向きの側面を
    /// 立てます**。作り直さずに、そちらへ渡します。
    ///
    /// 向きだけ揃えます。押し出しの口は**外周が押し出す向きから見て
    /// 反時計回り**であることを前提にしているので、面の輪の回り方を
    /// Newell の法線で見て、逆なら輪を反転してから渡します。
    /// **厚みが負のときも同じ道を通ります**——向きが逆になるだけです。
    fn thicken_planar_face(
        face: &Face,
        plane: &PlaneSurface3,
        thickness: f64,
    ) -> Result<Solid, String> {
        let tol = Tolerance::default();
        let normal = plane
            .normal
            .try_normalize_safe(1e-12)
            .ok_or("the sheet's plane has no normal to offset along")?;
        let direction = normal * thickness;

        if face.outer_wire.edges.len() < 3 {
            return Err("Planar face requires at least 3 vertices".to_string());
        }

        // **押し出す向きから見て反時計回りに揃えます。**
        //
        // 揃えないと、押し出しの口が内向きの立体を組み、シェル検証で
        // 落ちます。**黙って裏返った立体を返すよりは良い**のですが、
        // 呼び手から見れば「厚みの符号で断られる」ことになります。
        let flip = wire_turns_against(&face.outer_wire, direction).unwrap_or(false);
        let outer = rebuild_wire_forward(&face.outer_wire, flip);
        // **穴も外周と同じ向きで渡します**——`extrude_face_with_holes`
        // が `is_hole` で内側へ向け直します（`extrude_sketch` と同じ
        // 渡し方です）。
        let inner: Vec<Wire> = face
            .inner_wires
            .iter()
            .map(|wire| {
                let against = wire_turns_against(wire, direction).unwrap_or(false);
                rebuild_wire_forward(wire, against != flip)
            })
            .collect();

        crate::ExtrudeBuilder::extrude_face_with_holes(&outer, &inner, direction, &tol)
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
        face: &Face,
        nurbs: &NurbsSurface3,
        thickness: f64,
        samples: usize,
        tol: &Tolerance,
    ) -> Result<Solid, String> {
        // **面のトリムを読んでいないので、素のパッチだけを受け取ります**
        // （4-409）。
        //
        // ここから下は、曲面のパラメータ域を**端から端まで**厚くします。
        // 面の輪がその端と違う所を走っていても、以前は**黙って素のパッチを
        // 厚くした立体**を返していました。輪を半分で止めた四半円柱で、
        // **閉じた式の 2.000 倍**が返ります（`thicken_trim_probe`）。
        // **閉じた多様体で、形も円柱の一部**なので、形の検査では
        // 捕まりません。
        //
        // **トリムを読んで厚くするのは、まだ実装していません**——
        // オフセットした面を同じ輪で切り直し、側面をトリム曲線に沿って
        // 立てる段が要ります。**半分書いたものを返すより、断ります。**
        if !face.inner_wires.is_empty() {
            return Err(format!(
                "the sheet has {} inner loop(s); thicken only handles an untrimmed NURBS patch (a trimmed curved sheet is not implemented)",
                face.inner_wires.len()
            ));
        }
        if let Some(off) = wire_leaves_patch_boundary(&face.outer_wire, nurbs, tol) {
            return Err(format!(
                "the sheet's boundary runs {off:.3e} inside the patch, so this is a trimmed curved sheet; thicken only handles an untrimmed NURBS patch"
            ));
        }

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

/// 輪が、与えた向きから見て**時計回り**か。
///
/// 稜の始点だけでなく**曲線の途中も標本します**——弧だけでできた輪は
/// 始点が 4 点しかなく、そこだけを見ると内接多角形の回り方になります。
/// 回り方そのものは同じなので符号は変わりませんが、**弧が半周を超える
/// 輪では始点が一直線に並ぶことがあり**、そのとき Newell の法線が
/// 立ちません。
///
/// 判じられないときは `None` を返します。**推測はしません。**
fn wire_turns_against(wire: &Wire, direction: Vec3) -> Option<bool> {
    let mut points: Vec<Point3> = Vec::new();
    for oriented in &wire.edges {
        // 終点は次の稜の始点なので入れません（`include_start` のみ）。
        points.extend(oriented.sample_points(8, true));
    }
    let normal = newell_normal_of(&points)?;
    let dot = normal.dot(&direction);
    if dot.abs() <= 1e-12 {
        return None;
    }
    Some(dot < 0.0)
}

/// 平面多角形の法線（Newell の方法）。
///
/// 3 点だけを見る外積と違って、どの 3 点が一直線に並んでいても落ちません。
fn newell_normal_of(points: &[Point3]) -> Option<Vec3> {
    if points.len() < 3 {
        return None;
    }
    let mut normal = Vec3::zeros();
    for index in 0..points.len() {
        let current = points[index];
        let next = points[(index + 1) % points.len()];
        normal.x += (current.y - next.y) * (current.z + next.z);
        normal.y += (current.z - next.z) * (current.x + next.x);
        normal.z += (current.x - next.x) * (current.y + next.y);
    }
    if normal.norm() <= 1e-12 {
        return None;
    }
    Some(normal)
}

/// 輪を、**曲線そのものが頭から尾へ繋がる**形に組み直す。
///
/// **向きの札を裏返すだけでは足りません。** 押し出しの口
/// （`extrude_loop`）は、稜の**曲線を札を見ずに**取り出して側面を立てる
/// ので、輪の稜は**物理的に**前向きに繋がっていなければなりません。
/// 立体から取り出した面の輪は、`Reversed` の稜を普通に含みます——札の
/// ままで渡すと「外周の輪が開いている」「同じ向きの稜が 2 度使われて
/// いる」と断られます（**実際に 2 本の試験がそれで落ちました**）。
///
/// `reverse` が真なら、同じ組み直しをしたうえで逆回りにします。
fn rebuild_wire_forward(wire: &Wire, reverse: bool) -> Wire {
    let mut edges: Vec<OrientedEdge> = wire
        .edges
        .iter()
        .map(|oriented| {
            // その稜を「実際に走る向き」の曲線に直します。
            let curve = if oriented.orientation.is_forward() {
                oriented.edge.curve.clone()
            } else {
                oriented.edge.curve.reversed()
            };
            let start = oriented.start_vertex().clone();
            let end = oriented.end_vertex().clone();
            OrientedEdge::forward(Edge::new(curve, start, end, 1e-6))
        })
        .collect();

    if reverse {
        edges.reverse();
        edges = edges
            .into_iter()
            .map(|oriented| {
                let curve = oriented.edge.curve.reversed();
                let start = oriented.edge.end_vertex.clone();
                let end = oriented.edge.start_vertex.clone();
                OrientedEdge::forward(Edge::new(curve, start, end, 1e-6))
            })
            .collect();
    }

    Wire::new(edges)
}

/// 輪が、素のパッチの**外周からどれだけ内側へ入っているか**。
///
/// 素のパッチちょうどの面なら `None`。どこかが内側を走っていれば、
/// **いちばん深く入っている距離**を返します。
///
/// 外周は 4 本の等パラメータ曲線です。輪の標本点から、その 4 本を折れ線で
/// 近似したものへの距離を測ります。**標本そのものへの距離ではなく折れ線へ
/// 測ります**——標本への距離は刻みの半分ぶん大きく出るので、長い辺の上に
/// ちょうど乗っている点が「内側にいる」ように見えます。
fn wire_leaves_patch_boundary(
    wire: &Wire,
    nurbs: &NurbsSurface3,
    tol: &Tolerance,
) -> Option<f64> {
    const BOUNDARY_SAMPLES: usize = 96;
    const WIRE_SAMPLES: usize = 12;

    let ((u_min, u_max), (v_min, v_max)) = nurbs.param_range();
    let mut boundary: Vec<Vec<Point3>> = Vec::with_capacity(4);
    for (fixed_u, value) in [(true, u_min), (true, u_max), (false, v_min), (false, v_max)] {
        let mut polyline = Vec::with_capacity(BOUNDARY_SAMPLES + 1);
        for index in 0..=BOUNDARY_SAMPLES {
            let t = index as f64 / BOUNDARY_SAMPLES as f64;
            let (u, v) = if fixed_u {
                (value, v_min + (v_max - v_min) * t)
            } else {
                (u_min + (u_max - u_min) * t, value)
            };
            polyline.push(nurbs.evaluate(u, v));
        }
        boundary.push(polyline);
    }

    let distance_to_boundary = |point: Point3| -> f64 {
        let mut best = f64::INFINITY;
        for polyline in &boundary {
            for pair in polyline.windows(2) {
                let segment = pair[1] - pair[0];
                let length_squared = segment.norm_squared();
                let distance = if length_squared <= f64::EPSILON {
                    (point - pair[0]).norm()
                } else {
                    let s = ((point - pair[0]).dot(&segment) / length_squared).clamp(0.0, 1.0);
                    (point - (pair[0] + segment * s)).norm()
                };
                best = best.min(distance);
            }
        }
        best
    };

    // **許しは、面の大きさに合わせます。** 等パラメータ曲線を折れ線で
    // 近似しているぶん、弧の上の点はわずかに外れます。素のパッチを
    // 断ってしまうほうが困るので、そこは通します。
    let diagonal = (nurbs.evaluate(u_max, v_max) - nurbs.evaluate(u_min, v_min)).norm();
    let allowance = (diagonal * 1e-3).max(tol.linear * 10.0);

    let mut worst: Option<f64> = None;
    for oriented in &wire.edges {
        for point in oriented.sample_points(WIRE_SAMPLES, true) {
            let distance = distance_to_boundary(point);
            if distance > allowance {
                worst = Some(worst.map_or(distance, |best: f64| best.max(distance)));
            }
        }
    }
    worst
}
