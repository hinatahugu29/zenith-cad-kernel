use zenith_geom::{ControlPoint3, KnotVector, NurbsCurve3, NurbsSurface3, PlaneSurface3};
use zenith_math::{Point3, Tolerance, Vec3, Vec3Ext};
use zenith_topo::{Edge, Face, FaceGeometry, OrientedEdge, Shell, Solid, Vertex, Wire};

/// 押し出し（Extrude）モデリングアルゴリズム
pub struct ExtrudeBuilder;

impl ExtrudeBuilder {
    /// 閉じた平坦なワイヤ（底面）と押し出し方向ベクトルから、閉じたSolid（側面Face群 + 底面Face + 天面Face）を構築
    pub fn extrude_wire(bottom_wire: &Wire, dir: Vec3, tol: &Tolerance) -> Result<Solid, String> {
        Self::extrude_face_with_holes(bottom_wire, &[], dir, tol)
    }

    /// ドラフト角度（抜き勾配: draft_angle_rad）付きで閉じたワイヤを押し出し、完全閉Solidを構築
    ///
    /// **勾配は面ごとに測ります。** 抜き勾配とは、側面が引抜方向となす角の
    /// ことなので、天面の輪郭は底面の輪郭を**各辺の外向き法線へ `h·tanα` だけ
    /// 押し出した**もの（オフセット多角形）になります。頂点は隣り合う2辺の
    /// オフセット線の交点へ動くので、動く距離は `h·tanα / sin(θ/2)` で、
    /// **頂点ごとに違います**。
    ///
    /// 以前ここは、各頂点を**ワイヤの重心から放射状に** `h·tanα` だけ動かして
    /// いました。長方形ではこれは相似拡大にしかならず、実測（40 × 25 を高さ
    /// 30、指定 3 度）で
    ///
    /// | | 指定 | 実際 |
    /// | :--- | ---: | ---: |
    /// | 長辺側の面 | 3.0000° | **1.5910°** |
    /// | 短辺側の面 | 3.0000° | **2.5446°** |
    /// | 体積 | 33164.73 | **32044.32**（-3.38%） |
    ///
    /// と、**どちらの面も指定した角度になっていませんでした**。`DraftBuilder::
    /// make_drafted_block` は同じ形を正しく作るので、2つの経路が食い違って
    /// いたことになります。
    ///
    /// いまのところ直線の辺だけを扱います。円弧を含むワイヤは、正しい
    /// オフセットが同心円弧で、有理曲線の中間制御点は `δ/cos(θ/2)` だけ動く
    /// 必要があり、法線方向に一律に動かしても合いません。**近い別の形を
    /// 返さず、理由を返して失敗します。**
    pub fn extrude_wire_with_draft(
        bottom_wire: &Wire,
        dir: Vec3,
        draft_angle_rad: f64,
        tol: &Tolerance,
    ) -> Result<Solid, String> {
        if !bottom_wire.is_closed(tol) {
            return Err("Extrude with draft requires a closed wire".to_string());
        }
        let num_edges = bottom_wire.edges.len();
        if num_edges < 3 {
            return Err("Extrude with draft requires at least 3 edges".to_string());
        }

        let height = dir.norm();
        if height <= 1e-9 {
            return Err("Extrude dir cannot be zero".to_string());
        }

        // 1. 各辺が直線であることを確かめる。曲がった辺のオフセットは
        //    法線方向に一律に動かしても合わない（上のコメント）。
        for (index, oe) in bottom_wire.edges.iter().enumerate() {
            let curve = &oe.edge.curve;
            // 次数だけでは決められない。次数1でも制御点が3つあれば折れ線で、
            // まっすぐとは限らない。**点の並びを見る。**
            let straight = curve.control_points.len() == 2 || {
                // 高次で書かれていても、制御点が始点と終点を結ぶ線分の
                // 上に並んでいれば直線。
                let start = curve.control_points[0].point;
                let end = curve.control_points[curve.control_points.len() - 1].point;
                let axis = end - start;
                let length = axis.norm();
                length > 1e-12
                    && curve
                        .control_points
                        .iter()
                        .all(|cp| (cp.point - start).cross(&axis).norm() / length <= tol.linear)
            };
            if !straight {
                return Err(format!(
                    "Extrude with draft only handles straight edges; edge {index} of the profile is curved"
                ));
            }
        }

        // 2. 輪郭の平面と、その上での回り方を決める。外向きはここから出る。
        let corners: Vec<Point3> = bottom_wire
            .edges
            .iter()
            .map(|oe| oe.start_vertex().point)
            .collect();
        let plane_normal = newell_normal(&corners).ok_or_else(|| {
            "Extrude with draft requires a planar, non-degenerate profile".to_string()
        })?;

        // 3. 各辺の外向き法線。反時計回り（`plane_normal` から見て）の輪郭では
        //    `tangent × normal` が外を向く。
        let mut outward = Vec::with_capacity(num_edges);
        for index in 0..num_edges {
            let start = corners[index];
            let end = corners[(index + 1) % num_edges];
            let tangent = end - start;
            let length = tangent.norm();
            if length <= 1e-12 {
                return Err(format!("Edge {index} of the profile has zero length"));
            }
            outward.push((tangent / length).cross(&plane_normal));
        }

        // 4. 天面の輪郭は、隣り合う2辺のオフセット線の交点。
        //    `(p - v)·n_a = δ` かつ `(p - v)·n_b = δ` を解くと
        //    `p = v + (n_a + n_b)·δ / (1 + n_a·n_b)`。
        let setback = height * draft_angle_rad.tan();
        let mut bottom_vertices = Vec::with_capacity(num_edges);
        let mut top_vertices = Vec::with_capacity(num_edges);
        let mut vertex_offsets = Vec::with_capacity(num_edges);

        for index in 0..num_edges {
            let v_bot = bottom_wire.edges[index].start_vertex();
            bottom_vertices.push(v_bot.clone());

            let previous = outward[(index + num_edges - 1) % num_edges];
            let current = outward[index];
            let bisector = previous + current;
            let denominator = 1.0 + previous.dot(&current);
            if denominator <= 1e-9 {
                return Err(format!(
                    "The profile doubles back on itself at vertex {index}; a draft offset is not defined there"
                ));
            }
            let shift = bisector * (setback / denominator);
            vertex_offsets.push(shift);
            top_vertices.push(Vertex::new(v_bot.point + dir + shift, tol.linear));
        }

        // 5. オフセットが輪郭を裏返していないか。凹んだ角では、後退距離が
        //    その角の逃げより大きいと辺の向きが反転する。**そこで作った立体は
        //    自分と交わる**ので、近い別の形を返さずに断る。
        for index in 0..num_edges {
            let next = (index + 1) % num_edges;
            let before = corners[next] - corners[index];
            let after =
                (corners[next] + vertex_offsets[next]) - (corners[index] + vertex_offsets[index]);
            if before.dot(&after) <= 0.0 {
                return Err(format!(
                    "A draft of {:.4} deg over a height of {height} turns edge {index} of the profile inside out",
                    draft_angle_rad.to_degrees()
                ));
            }
        }

        // 6. 天面エッジ群および天面ワイヤの生成
        let mut top_edges = Vec::with_capacity(num_edges);
        for i in 0..num_edges {
            let next_i = (i + 1) % num_edges;
            let v_start = top_vertices[i].clone();
            let v_end = top_vertices[next_i].clone();

            // 辺は直線なので、制御点は線分上の位置をそのまま持ち越せばよい。
            // 底面の制御点が始点から終点へ何割の所にあるかを測り、天面の
            // 同じ割合の所へ置く。
            let bot_edge = &bottom_wire.edges[i].edge;
            let n_cp = bot_edge.curve.control_points.len();
            let bottom_start = corners[i];
            let axis = corners[next_i] - bottom_start;
            let axis_length_squared = axis.norm_squared();
            let top_start = top_vertices[i].point;
            let top_axis = top_vertices[next_i].point - top_start;
            let mut top_cps = Vec::with_capacity(n_cp);
            for cp in &bot_edge.curve.control_points {
                let fraction = if axis_length_squared > 1e-24 {
                    (cp.point - bottom_start).dot(&axis) / axis_length_squared
                } else {
                    0.0
                };
                let pt = top_start + top_axis * fraction;
                top_cps.push(ControlPoint3::new(pt, cp.weight));
            }
            let top_curve =
                NurbsCurve3::new(bot_edge.curve.degree, top_cps, bot_edge.curve.knots.clone())?;
            let edge = Edge::new(top_curve, v_start, v_end, tol.linear);
            top_edges.push(OrientedEdge::forward(edge));
        }
        let top_wire = Wire::new(top_edges);

        // 4. 側面の柱エッジ群（Pillars）
        let mut pillar_edges = Vec::with_capacity(num_edges);
        for i in 0..num_edges {
            let edge = Edge::line_between(bottom_vertices[i].clone(), top_vertices[i].clone())?;
            pillar_edges.push(edge);
        }

        // 5. 側面Face群の生成
        let mut faces = Vec::with_capacity(num_edges + 2);
        for i in 0..num_edges {
            let next_i = (i + 1) % num_edges;

            let bot_edge = bottom_wire.edges[i].edge.clone();
            let right_pillar = pillar_edges[next_i].clone();
            let top_edge = top_wire.edges[i].edge.clone();
            let left_pillar = pillar_edges[i].clone();

            let side_wire = Wire::new(vec![
                OrientedEdge::forward(bot_edge.clone()),
                OrientedEdge::forward(right_pillar),
                OrientedEdge::reversed(top_edge.clone()),
                OrientedEdge::reversed(left_pillar),
            ]);

            // 底辺カーブと天辺カーブ間のルールドNURBS曲面
            let n_u = bot_edge.curve.control_points.len();
            let mut control_grid = Vec::with_capacity(n_u);
            for u in 0..n_u {
                let cp_bot = bot_edge.curve.control_points[u];
                let cp_top = top_edge.curve.control_points[u];
                control_grid.push(vec![cp_bot, cp_top]);
            }
            let side_surf = NurbsSurface3::new(
                bot_edge.curve.degree,
                1,
                control_grid,
                bot_edge.curve.knots.clone(),
                KnotVector::clamped_uniform(2, 1),
            )?;

            faces.push(Face::simple(FaceGeometry::Nurbs(side_surf), side_wire));
        }

        // 6. 底面キャップ（-dir 法線）
        let bot_normal = -dir
            .try_normalize_safe(1e-12)
            .unwrap_or_else(|| Vec3::new(0.0, 0.0, -1.0));
        let bot_outer_wire = Self::reversed_wire(bottom_wire);
        let bot_face = Self::make_cap_face_with_holes(
            &bot_outer_wire,
            &[],
            bottom_wire.edges[0].start_vertex().point,
            bot_normal,
        )?;
        faces.push(bot_face);

        // 7. 天面キャップ（+dir 法線）
        let top_normal = dir
            .try_normalize_safe(1e-12)
            .unwrap_or_else(|| Vec3::new(0.0, 0.0, 1.0));
        let top_face = Self::make_cap_face_with_holes(
            &top_wire,
            &[],
            top_wire.edges[0].start_vertex().point,
            top_normal,
        )?;
        faces.push(top_face);

        let shell = Shell::closed(faces);
        let report = shell.validate_closed(tol);
        if !report.is_valid() {
            return Err(format!(
                "Draft extrude validation failed: {:?}",
                report.errors
            ));
        }
        crate::validated_solid(shell)
    }

    /// 外側境界ワイヤと複数の内側境界（穴）ワイヤを持つ平坦なプロファイルから中空・穴あき押し出しSolidを構築
    pub fn extrude_face_with_holes(
        outer_wire: &Wire,
        inner_wires: &[Wire],
        dir: Vec3,
        tol: &Tolerance,
    ) -> Result<Solid, String> {
        if !outer_wire.is_closed(tol) {
            return Err("Extrude requires a closed outer wire".to_string());
        }
        if outer_wire.edges.len() < 3 {
            return Err("Extrude requires at least 3 edges in outer wire".to_string());
        }

        for (idx, hole) in inner_wires.iter().enumerate() {
            if !hole.is_closed(tol) {
                return Err(format!("Inner wire {} must be closed", idx));
            }
            if hole.edges.len() < 3 {
                return Err(format!("Inner wire {} must have at least 3 edges", idx));
            }
        }

        let mut faces = Vec::new();

        // 1. 外壁側面Face群（Outer Wall Faces）の生成 (is_hole: false)
        let (top_outer_wire, outer_side_faces) = Self::extrude_loop(outer_wire, dir, false, tol)?;
        faces.extend(outer_side_faces);

        // 2. 内壁側面Face群（Inner Hole Wall Faces）の生成 (is_hole: true)
        let mut top_inner_wires = Vec::with_capacity(inner_wires.len());
        for hole_wire in inner_wires {
            let (top_hole_wire, hole_side_faces) = Self::extrude_loop(hole_wire, dir, true, tol)?;
            faces.extend(hole_side_faces);
            top_inner_wires.push(top_hole_wire);
        }

        // 3. 底面キャップFace（-dir 法線）
        let bot_normal = -dir
            .try_normalize_safe(1e-12)
            .unwrap_or_else(|| Vec3::new(0.0, 0.0, -1.0));
        let top_normal = dir
            .try_normalize_safe(1e-12)
            .unwrap_or_else(|| Vec3::new(0.0, 0.0, 1.0));

        let bot_outer_wire = Self::reversed_wire(outer_wire);
        let bot_inner_wires = inner_wires.to_vec();

        let bot_face = Self::make_cap_face_with_holes(
            &bot_outer_wire,
            &bot_inner_wires,
            outer_wire.edges[0].start_vertex().point,
            bot_normal,
        )?;
        faces.push(bot_face);

        // 4. 天面キャップFace（+dir 法線）
        let top_inner_reversed: Vec<Wire> =
            top_inner_wires.iter().map(Self::reversed_wire).collect();
        let top_face = Self::make_cap_face_with_holes(
            &top_outer_wire,
            &top_inner_reversed,
            top_outer_wire.edges[0].start_vertex().point,
            top_normal,
        )?;
        faces.push(top_face);

        let shell = Shell::closed(faces);
        let report = shell.validate_closed(tol);
        if !report.is_valid() {
            return Err(format!(
                "Extrude hollow validation failed: {:?}",
                report.errors
            ));
        }
        crate::validated_solid(shell)
    }

    /// 単一ループ（外側または穴）から天面ワイヤと側面Face群を構築
    fn extrude_loop(
        wire: &Wire,
        dir: Vec3,
        is_hole: bool,
        tol: &Tolerance,
    ) -> Result<(Wire, Vec<Face>), String> {
        let num_edges = wire.edges.len();

        let mut bottom_vertices = Vec::with_capacity(num_edges);
        let mut top_vertices = Vec::with_capacity(num_edges);

        for oe in &wire.edges {
            let v_bot = oe.start_vertex();
            bottom_vertices.push(v_bot.clone());
            let top_pt = v_bot.point + dir;
            top_vertices.push(Vertex::new(top_pt, tol.linear));
        }

        let mut top_edges = Vec::with_capacity(num_edges);
        for oriented in &wire.edges {
            top_edges.push(OrientedEdge::new(
                crate::BrepTransform::translate_edge(&oriented.edge, dir),
                oriented.orientation,
            ));
        }
        let top_wire = Wire::new(top_edges);

        let mut pillar_edges = Vec::with_capacity(num_edges);
        for i in 0..num_edges {
            let edge = Edge::line_between(bottom_vertices[i].clone(), top_vertices[i].clone())?;
            pillar_edges.push(edge);
        }

        let mut side_faces = Vec::with_capacity(num_edges);
        for i in 0..num_edges {
            let next_i = (i + 1) % num_edges;

            let bot_edge = wire.edges[i].edge.clone();
            let right_pillar = pillar_edges[next_i].clone();
            let top_edge = top_wire.edges[i].edge.clone();
            let left_pillar = pillar_edges[i].clone();

            let (side_wire, surf) = if !is_hole {
                // 外壁: 外向き法線
                let w = Wire::new(vec![
                    OrientedEdge::forward(bot_edge.clone()),
                    OrientedEdge::forward(right_pillar),
                    OrientedEdge::reversed(top_edge),
                    OrientedEdge::reversed(left_pillar),
                ]);
                let s = Self::make_ruled_surface(&bot_edge.curve, dir)?;
                (w, s)
            } else {
                // 穴内壁: 穴の中心向き法線（曲線を反転して法線を内向きにする）
                let w = Wire::new(vec![
                    OrientedEdge::reversed(bot_edge.clone()),
                    OrientedEdge::forward(left_pillar),
                    OrientedEdge::forward(top_edge),
                    OrientedEdge::reversed(right_pillar),
                ]);
                let rev_curve = bot_edge.curve.reversed();
                let s = Self::make_ruled_surface(&rev_curve, dir)?;
                (w, s)
            };

            side_faces.push(Face::simple(FaceGeometry::Nurbs(surf), side_wire));
        }

        Ok((top_wire, side_faces))
    }

    /// 底辺カーブから押し出し方向へのルールドNURBS曲面を生成
    fn make_ruled_surface(curve: &NurbsCurve3, dir: Vec3) -> Result<NurbsSurface3, String> {
        let n_u = curve.control_points.len();
        let degree_u = curve.degree;
        let degree_v = 1;

        let mut control_points = Vec::with_capacity(n_u);
        for cp in &curve.control_points {
            control_points.push(vec![*cp, ControlPoint3::new(cp.point + dir, cp.weight)]);
        }

        let knots_u = curve.knots.clone();
        let knots_v = KnotVector::clamped_uniform(2, 1);

        NurbsSurface3::new(degree_u, degree_v, control_points, knots_u, knots_v)
    }

    /// 平面キャップFace（穴あき対応）の生成
    fn make_cap_face_with_holes(
        outer_wire: &Wire,
        inner_wires: &[Wire],
        origin: Point3,
        normal: Vec3,
    ) -> Result<Face, String> {
        let arb = if normal.x.abs() < 0.9 {
            Vec3::new(1.0, 0.0, 0.0)
        } else {
            Vec3::new(0.0, 1.0, 0.0)
        };
        let u_axis = normal
            .cross(&arb)
            .try_normalize_safe(1e-12)
            .ok_or("Failed u_axis")?;
        let v_axis = normal
            .cross(&u_axis)
            .try_normalize_safe(1e-12)
            .ok_or("Failed v_axis")?;

        let plane = PlaneSurface3::new(origin, u_axis, v_axis).ok_or("Failed to create plane")?;
        Ok(Face::new(
            FaceGeometry::Plane(plane),
            outer_wire.clone(),
            inner_wires.to_vec(),
            zenith_topo::Orientation::Forward,
            1e-6,
        ))
    }

    fn reversed_wire(wire: &Wire) -> Wire {
        let edges = wire
            .edges
            .iter()
            .rev()
            .map(|edge| {
                let mut reversed = edge.clone();
                reversed.orientation = reversed.orientation.reversed();
                reversed
            })
            .collect();
        Wire::new(edges)
    }
}

/// 平面多角形の法線（Newell の方法）。
///
/// 3点だけを見る外積と違って、どの3点が一直線に並んでいても落ちない。
/// 向きは輪郭の回り方に従うので、これを上と見たとき輪郭は反時計回りになる。
fn newell_normal(corners: &[Point3]) -> Option<Vec3> {
    if corners.len() < 3 {
        return None;
    }
    let mut normal = Vec3::zeros();
    for index in 0..corners.len() {
        let current = corners[index];
        let next = corners[(index + 1) % corners.len()];
        normal.x += (current.y - next.y) * (current.z + next.z);
        normal.y += (current.z - next.z) * (current.x + next.x);
        normal.z += (current.x - next.x) * (current.y + next.y);
    }
    let length = normal.norm();
    if length <= 1e-12 {
        return None;
    }
    Some(normal / length)
}
