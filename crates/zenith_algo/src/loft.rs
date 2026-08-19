use zenith_geom::{KnotVector, NurbsCurve3, NurbsSurface3, PlaneSurface3};
use zenith_math::{Point3, Tolerance, Vec3};
use zenith_topo::{Edge, Face, FaceGeometry, OrientedEdge, Shell, Solid, Wire};

/// ロフト（Loft / スキニング）モデリングアルゴリズム
/// 複数の断面プロファイル曲線・ワイヤ群から滑らかなNURBS曲面および完全閉B-Repソリッドを生成
pub struct LoftBuilder;

impl LoftBuilder {
    /// 2本以上のプロファイル曲線からロフト曲面を生成（不揃いな曲線は自動互換化）
    pub fn loft_curves(
        profiles: &[NurbsCurve3],
        degree_v: usize,
        _tol: &Tolerance,
    ) -> Result<NurbsSurface3, String> {
        let m = profiles.len();
        if m < 2 {
            return Err("Loft requires at least 2 profile curves".to_string());
        }

        // 次数や制御点数が異なるプロファイルを自動互換化
        let compatible_profiles = NurbsCurve3::make_compatible(profiles, None)?;
        let num_u = compatible_profiles[0].control_points.len();
        let degree_u = compatible_profiles[0].degree;

        let effective_degree_v = degree_v.min(m - 1).max(1);

        // 制御点グリッドの構築 [row_u][col_v]
        let mut ctrl_pts_grid = vec![Vec::with_capacity(m); num_u];

        for (i, row) in ctrl_pts_grid.iter_mut().enumerate().take(num_u) {
            for profile in compatible_profiles.iter().take(m) {
                row.push(profile.control_points[i]);
            }
        }

        let knots_u = compatible_profiles[0].knots.clone();
        let knots_v = KnotVector::clamped_uniform(m, effective_degree_v);

        NurbsSurface3::new(
            degree_u,
            effective_degree_v,
            ctrl_pts_grid,
            knots_u,
            knots_v,
        )
    }

    /// 2つ以上の閉じた断面ワイヤ群から、側面ロフト曲面群と端面キャップを持つ完全閉B-Repソリッドを生成
    pub fn loft_solid(
        section_wires: &[Wire],
        degree_v: usize,
        tol: &Tolerance,
    ) -> Result<Solid, String> {
        let m = section_wires.len();
        if m < 2 {
            return Err("Loft solid requires at least 2 section wires".to_string());
        }

        // 全ワイヤの閉ループ性検証
        for (idx, wire) in section_wires.iter().enumerate() {
            if !wire.is_closed(tol) {
                return Err(format!("Section wire {} is not closed", idx));
            }
        }

        let k = section_wires[0].edges.len();
        if k < 3 {
            return Err("Section wires must have at least 3 edges".to_string());
        }

        // 全断面のエッジ数が一致しているか確認
        for (idx, wire) in section_wires.iter().enumerate() {
            if wire.edges.len() != k {
                return Err(format!(
                    "Section wire {} has {} edges, expected {}",
                    idx,
                    wire.edges.len(),
                    k
                ));
            }
        }

        // 1. 各断面の各頂点を収集 [m][k]
        let mut vertices_grid = Vec::with_capacity(m);
        for wire in section_wires {
            let mut row = Vec::with_capacity(k);
            for oe in &wire.edges {
                row.push(oe.start_vertex().clone());
            }
            vertices_grid.push(row);
        }

        // 2. 垂直柱（Pillar）エッジ群の生成 [m - 1][k] (断面iの頂点j と 断面i+1の頂点j を結ぶ)
        let mut pillar_edges = Vec::with_capacity(m - 1);
        for i in 0..m - 1 {
            let mut row = Vec::with_capacity(k);
            for j in 0..k {
                let v_start = vertices_grid[i][j].clone();
                let v_end = vertices_grid[i + 1][j].clone();
                let edge = Edge::line_between(v_start, v_end)?;
                row.push(edge);
            }
            pillar_edges.push(row);
        }

        let mut faces = Vec::with_capacity(k * (m - 1) + 2);

        // 3. 各区間 [i, i+1] における側面Face群の構築
        for i in 0..m - 1 {
            for j in 0..k {
                let next_j = (j + 1) % k;

                let bot_edge = section_wires[i].edges[j].edge.clone();
                let top_edge = section_wires[i + 1].edges[j].edge.clone();
                let left_pillar = pillar_edges[i][j].clone();
                let right_pillar = pillar_edges[i][next_j].clone();

                // 側面Faceの4辺ワイヤ: bot(Fwd) -> right(Fwd) -> top(Rev) -> left(Rev)
                let side_wire = Wire::new(vec![
                    OrientedEdge::forward(bot_edge.clone()),
                    OrientedEdge::forward(right_pillar),
                    OrientedEdge::reversed(top_edge.clone()),
                    OrientedEdge::reversed(left_pillar),
                ]);

                // 2つのプロファイル曲線からロフト曲面を生成
                let c_bot = bot_edge.curve.clone();
                let c_top = top_edge.curve.clone();
                let loft_surf = Self::loft_curves(&[c_bot, c_top], degree_v, tol)?;

                let face = Face::simple(FaceGeometry::Nurbs(loft_surf), side_wire);
                faces.push(face);
            }
        }

        // 4. 底面Face（外向き法線 = 下向き）の構築
        let bot_wire = section_wires[0].clone();
        let bot_face = create_cap_face(&bot_wire, true, tol)?;
        faces.push(bot_face);

        // 5. 天面Face（外向き法線 = 上向き）の構築
        let top_wire = section_wires[m - 1].clone();
        let top_face = create_cap_face(&top_wire, false, tol)?;
        faces.push(top_face);

        // 6. 閉シェル化とSolid検証
        let shell = Shell::closed(faces);
        let report = shell.validate_closed(tol);
        if !report.is_valid() {
            let msg = if let Some(first) = report.errors.first() {
                first.chars().take(80).collect::<String>()
            } else {
                "unknown".to_string()
            };
            return Err(format!("LOFT_ERR: {}", msg));
        }


        Solid::try_simple(shell, tol).map_err(|err| format!("Loft solid validation failed: {}", err))
    }

    /// ガイドレール曲線群（Guide Curves）に沿った閉断面ワイヤ群のロフト完全閉B-Repソリッド生成
    pub fn loft_solid_guided(
        section_wires: &[Wire],
        guide_curves: &[NurbsCurve3],
        degree_v: usize,
        tol: &Tolerance,
    ) -> Result<Solid, String> {
        if guide_curves.is_empty() {
            return Self::loft_solid(section_wires, degree_v, tol);
        }

        let m = section_wires.len();
        if m < 2 {
            return Err("Guided loft requires at least 2 section wires".to_string());
        }

        let k = section_wires[0].edges.len();
        for (idx, wire) in section_wires.iter().enumerate() {
            if !wire.is_closed(tol) {
                return Err(format!("Section wire {} is not closed", idx));
            }
            if wire.edges.len() != k {
                return Err(format!(
                    "Section wire {} has {} edges, expected {}",
                    idx,
                    wire.edges.len(),
                    k
                ));
            }
        }

        // 1. 各断面の各頂点を収集 [m][k]
        let mut vertices_grid = Vec::with_capacity(m);
        for wire in section_wires {
            let mut row = Vec::with_capacity(k);
            for oe in &wire.edges {
                row.push(oe.start_vertex().clone());
            }
            vertices_grid.push(row);
        }

        // 2. ガイドレールに基づく柱（Pillar）NURBS曲線エッジ群の生成 [m - 1][k]
        let mut pillar_edges = Vec::with_capacity(m - 1);
        for i in 0..m - 1 {
            let mut row = Vec::with_capacity(k);
            let u_start = i as f64 / (m - 1) as f64;
            let u_end = (i + 1) as f64 / (m - 1) as f64;

            for j in 0..k {
                let v_start = vertices_grid[i][j].clone();
                let v_end = vertices_grid[i + 1][j].clone();

                let guide = &guide_curves[j % guide_curves.len()];
                let p_g0 = guide.evaluate(u_start);
                let p_g1 = guide.evaluate((u_start + u_end) * 0.5);
                let p_g2 = guide.evaluate(u_end);

                let g_mid_offset = p_g1 - (p_g0 + (p_g2 - p_g0) * 0.5);
                let p_mid = v_start.point + (v_end.point - v_start.point) * 0.5 + g_mid_offset;

                let pillar_curve = NurbsCurve3::new(
                    2,
                    vec![
                        zenith_geom::ControlPoint3::unweighted(v_start.point),
                        zenith_geom::ControlPoint3::unweighted(p_mid),
                        zenith_geom::ControlPoint3::unweighted(v_end.point),
                    ],
                    KnotVector::clamped_uniform(3, 2),
                )?;
                let edge = Edge::new(pillar_curve, v_start, v_end, tol.linear);
                row.push(edge);
            }
            pillar_edges.push(row);
        }

        let mut faces = Vec::with_capacity(k * (m - 1) + 2);

        // 3. 各区間 [i, i+1] における側面Face群の構築
        for i in 0..m - 1 {
            let u_start = i as f64 / (m - 1) as f64;
            let u_end = (i + 1) as f64 / (m - 1) as f64;

            for j in 0..k {
                let next_j = (j + 1) % k;

                let bot_edge = section_wires[i].edges[j].edge.clone();
                let top_edge = section_wires[i + 1].edges[j].edge.clone();
                let left_pillar = pillar_edges[i][j].clone();
                let right_pillar = pillar_edges[i][next_j].clone();

                // 側面Faceの4辺ワイヤ: bot(Fwd) -> right(Fwd) -> top(Rev) -> left(Rev)
                let side_wire = Wire::new(vec![
                    OrientedEdge::forward(bot_edge.clone()),
                    OrientedEdge::forward(right_pillar.clone()),
                    OrientedEdge::reversed(top_edge.clone()),
                    OrientedEdge::reversed(left_pillar.clone()),
                ]);

                // 中間プロファイル曲線の生成 (ガイドレールの変位を反映)
                let c0 = bot_edge.curve.clone();
                let c1 = top_edge.curve.clone();
                let mut c_mid_cps = Vec::with_capacity(c0.control_points.len());
                for cp_idx in 0..c0.control_points.len() {
                    let p0 = c0.control_points[cp_idx].point;
                    let p1 = c1.control_points[cp_idx].point;
                    let p_linear = p0 + (p1 - p0) * 0.5;

                    let guide = &guide_curves[j % guide_curves.len()];
                    let p_g0 = guide.evaluate(u_start);
                    let p_g1 = guide.evaluate((u_start + u_end) * 0.5);
                    let p_g2 = guide.evaluate(u_end);
                    let g_offset = p_g1 - (p_g0 + (p_g2 - p_g0) * 0.5);

                    c_mid_cps.push(zenith_geom::ControlPoint3::new(
                        p_linear + g_offset,
                        c0.control_points[cp_idx].weight,
                    ));
                }
                let c_mid = NurbsCurve3::new(c0.degree, c_mid_cps, c0.knots.clone())?;

                let loft_surf = Self::loft_curves(&[c0, c_mid, c1], 2, tol)?;
                let face = Face::simple(FaceGeometry::Nurbs(loft_surf), side_wire);
                faces.push(face);
            }
        }


        // 4. 底面Faceの構築
        let bot_wire = section_wires[0].clone();
        let bot_face = create_cap_face(&bot_wire, true, tol)?;
        faces.push(bot_face);

        // 5. 天面Faceの構築
        let top_wire = section_wires[m - 1].clone();
        let top_face = create_cap_face(&top_wire, false, tol)?;
        faces.push(top_face);

        let shell = Shell::closed(faces);
        let report = shell.validate_closed(tol);
        if !report.is_valid() {
            return Err(format!("Guided loft validation failed: {:?}", report.errors));
        }
        Solid::try_simple(shell, tol).map_err(|err| format!("Guided loft solid failed: {}", err))
    }
}



/// 閉じた平坦ワイヤから端面キャップFaceを生成（is_bottom: true の場合は反転して下向き法線にする）
fn create_cap_face(wire: &Wire, is_bottom: bool, _tol: &Tolerance) -> Result<Face, String> {
    // 平面パラメータの算出（Newellのアルゴリズムによる平均平面法線）
    let pts: Vec<Point3> = wire.edges.iter().map(|oe| oe.start_vertex().point).collect();
    let n_pts = pts.len();
    if n_pts < 3 {
        return Err("Cap wire has fewer than 3 vertices".to_string());
    }

    let mut normal = Vec3::zeros();
    for i in 0..n_pts {
        let curr = pts[i];
        let next = pts[(i + 1) % n_pts];
        normal.x += (curr.y - next.y) * (curr.z + next.z);
        normal.y += (curr.z - next.z) * (curr.x + next.x);
        normal.z += (curr.x - next.x) * (curr.y + next.y);
    }

    let norm_len = normal.norm();
    let base_normal = if norm_len > 1e-9 {
        normal / norm_len
    } else {
        Vec3::new(0.0, 0.0, 1.0)
    };

    let normal = if is_bottom { -base_normal } else { base_normal };

    let p0 = pts[0];
    let arb = if normal.x.abs() < 0.9 {
        Vec3::new(1.0, 0.0, 0.0)
    } else {
        Vec3::new(0.0, 1.0, 0.0)
    };
    let u_axis = normal.cross(&arb).normalize();
    let v_axis = normal.cross(&u_axis).normalize();

    let plane = PlaneSurface3::new(p0, u_axis, v_axis).ok_or("Failed to create cap plane")?;

    let cap_wire = if is_bottom {
        // 底面は法線を外向き（下向き）にするため逆順ワイヤを構築
        let mut rev_edges = Vec::with_capacity(n_pts);
        for oe in wire.edges.iter().rev() {
            rev_edges.push(OrientedEdge::new(oe.edge.clone(), oe.orientation.reversed()));
        }
        Wire::new(rev_edges)
    } else {
        wire.clone()
    };

    Ok(Face::simple(FaceGeometry::Plane(plane), cap_wire))
}
