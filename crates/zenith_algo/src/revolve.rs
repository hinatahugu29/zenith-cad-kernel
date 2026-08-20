use zenith_geom::{ControlPoint3, KnotVector, NurbsCurve3, NurbsSurface3};
use zenith_math::{Point3, Tolerance, Vec3, Vec3Ext};

/// 回転体（Revolve）モデリングアルゴリズム
pub struct RevolveBuilder;

impl RevolveBuilder {
    /// 3次元NURBS曲線を軸まわりに回転させて有理NURBS回転曲面を生成
    /// `axis_origin`: 回転軸上の点, `axis_dir`: 回転軸ベクトル, `angle_rad`: 回転角度 (0 < angle <= 2*PI)
    pub fn revolve_curve(
        curve: &NurbsCurve3,
        axis_origin: Point3,
        axis_dir: Vec3,
        angle_rad: f64,
        _tol: &Tolerance,
    ) -> Result<NurbsSurface3, String> {
        let axis_dir = axis_dir
            .try_normalize_safe(1e-12)
            .ok_or("Axis direction is zero")?;
        let num_u = curve.control_points.len();
        let degree_u = curve.degree;

        // 4セグメント（90度ごと）の有理B-Spline円弧回転（The NURBS Book Algorithm A8.1）
        // 簡易実装として4分割の等価NURBS回転グリッドを構築
        let num_segments = 4;
        let d_theta = angle_rad / num_segments as f64;
        let num_v = 2 * num_segments + 1;
        let degree_v = 2;

        let wm = (d_theta / 2.0).cos(); // 重み係数

        let mut ctrl_pts_grid = vec![Vec::with_capacity(num_v); num_u];

        for (i, cp) in curve.control_points.iter().enumerate() {
            let p = cp.point;
            // 軸への直交射影点
            let v_p = p - axis_origin;
            let proj_len = v_p.dot(&axis_dir);
            let p_center = axis_origin + axis_dir * proj_len;
            let v_radial = p - p_center;
            let radius = v_radial.norm();

            if radius < 1e-12 {
                // 軸上の特異点。位置は回転で動かないが、重みは他の行と同じ
                // 円弧パターン (1, cos(dtheta/2), 1, ...) を保たなければ
                // テンソル積の分母が分離できなくなり、曲面全体が歪む。
                for column in 0..num_v {
                    let arc_weight = if column % 2 == 1 { wm } else { 1.0 };
                    ctrl_pts_grid[i].push(ControlPoint3::new(p, cp.weight * arc_weight));
                }
                continue;
            }

            let x_axis = v_radial / radius;
            let y_axis = axis_dir.cross(&x_axis);

            let mut theta: f64 = 0.0;
            for seg in 0..num_segments {
                let p_start = p_center + (x_axis * theta.cos() + y_axis * theta.sin()) * radius;
                let theta_mid = theta + d_theta / 2.0;
                let p_mid = p_center
                    + (x_axis * theta_mid.cos() + y_axis * theta_mid.sin()) * (radius / wm);
                let theta_end = theta + d_theta;
                let p_end =
                    p_center + (x_axis * theta_end.cos() + y_axis * theta_end.sin()) * radius;

                if seg == 0 {
                    ctrl_pts_grid[i].push(ControlPoint3::new(p_start, cp.weight));
                }
                ctrl_pts_grid[i].push(ControlPoint3::new(p_mid, cp.weight * wm));
                ctrl_pts_grid[i].push(ControlPoint3::new(p_end, cp.weight));

                theta = theta_end;
            }
        }

        // V方向の結び目ベクトル（4セグメント円弧用: [0,0,0, 0.25,0.25, 0.5,0.5, 0.75,0.75, 1,1,1]）
        let mut knots_v = vec![0.0, 0.0, 0.0];
        for s in 1..num_segments {
            let val = s as f64 / num_segments as f64;
            knots_v.push(val);
            knots_v.push(val);
        }
        knots_v.extend_from_slice(&[1.0, 1.0, 1.0]);

        let knots_u = curve.knots.clone();
        let knot_vec_v = KnotVector::new(knots_v);

        NurbsSurface3::new(degree_u, degree_v, ctrl_pts_grid, knots_u, knot_vec_v)
    }


    /// 単一セグメント（角度 d_theta <= PI/2）に対するプロファイル曲線の正確な有理2次NURBS回転曲面
    pub fn revolve_curve_segment(
        curve: &NurbsCurve3,
        axis_origin: Point3,
        axis_dir: Vec3,
        d_theta: f64,
        _tol: &Tolerance,
    ) -> Result<NurbsSurface3, String> {
        let axis_dir = axis_dir
            .try_normalize_safe(1e-12)
            .ok_or("Axis direction is zero")?;
        let num_u = curve.control_points.len();
        let degree_u = curve.degree;
        let degree_v = 2;
        let wm = (d_theta / 2.0).cos();

        let mut ctrl_pts_grid = vec![Vec::with_capacity(3); num_u];

        for (i, cp) in curve.control_points.iter().enumerate() {
            let p = cp.point;
            let v_p = p - axis_origin;
            let proj_len = v_p.dot(&axis_dir);
            let p_center = axis_origin + axis_dir * proj_len;
            let v_radial = p - p_center;
            let radius = v_radial.norm();

            if radius < 1e-12 {
                ctrl_pts_grid[i].push(ControlPoint3::new(p, cp.weight));
                ctrl_pts_grid[i].push(ControlPoint3::new(p, cp.weight * wm));
                ctrl_pts_grid[i].push(ControlPoint3::new(p, cp.weight));
            } else {
                let x_axis = v_radial / radius;
                let y_axis = axis_dir.cross(&x_axis);

                let p0 = p;
                let p_mid = p_center + (x_axis * (d_theta / 2.0).cos() + y_axis * (d_theta / 2.0).sin()) * (radius / wm);
                let p1 = p_center + (x_axis * d_theta.cos() + y_axis * d_theta.sin()) * radius;

                ctrl_pts_grid[i].push(ControlPoint3::new(p0, cp.weight));
                ctrl_pts_grid[i].push(ControlPoint3::new(p_mid, cp.weight * wm));
                ctrl_pts_grid[i].push(ControlPoint3::new(p1, cp.weight));
            }
        }

        let knots_u = curve.knots.clone();
        let knots_v = KnotVector::clamped_uniform(3, degree_v);
        NurbsSurface3::new(degree_u, degree_v, ctrl_pts_grid, knots_u, knots_v)
    }


    /// 閉断面ワイヤを軸まわりに360度回転させ、完全閉B-Repソリッド（Solid）を構築
    pub fn revolve_wire_solid(
        wire: &zenith_topo::Wire,
        axis_origin: Point3,
        axis_dir: Vec3,
        tol: &Tolerance,
    ) -> Result<zenith_topo::Solid, String> {
        if !wire.is_closed(tol) {
            return Err("Revolve solid requires a closed wire".to_string());
        }
        let num_edges = wire.edges.len();
        if num_edges < 3 {
            return Err("Revolve solid requires at least 3 edges".to_string());
        }

        let axis_dir = axis_dir
            .try_normalize_safe(1e-12)
            .ok_or("Axis direction is zero")?;

        // 4セグメント（90度ごと）で360度全周回転
        let num_segments = 4;
        let d_theta = std::f64::consts::TAU / num_segments as f64; // PI/2
        let wm = (d_theta / 2.0).cos();

        // 1. 各頂点の4セグメント回転位置（0, 90, 180, 270, 360度）の頂点マトリクスを生成
        // vertices[seg_idx][vertex_idx]
        let mut rotated_vertices: Vec<Vec<zenith_topo::Vertex>> = Vec::with_capacity(num_segments + 1);

        for seg in 0..=num_segments {
            let theta = seg as f64 * d_theta;
            let mut v_row = Vec::with_capacity(num_edges);
            for oe in &wire.edges {
                let p = oe.start_vertex().point;
                let v_p = p - axis_origin;
                let proj_len = v_p.dot(&axis_dir);
                let p_center = axis_origin + axis_dir * proj_len;
                let v_radial = p - p_center;
                let radius = v_radial.norm();

                let rot_p = if radius < 1e-12 {
                    p
                } else {
                    let x_axis = v_radial / radius;
                    let y_axis = axis_dir.cross(&x_axis);
                    p_center + (x_axis * theta.cos() + y_axis * theta.sin()) * radius
                };
                v_row.push(zenith_topo::Vertex::new(rot_p, tol.linear));
            }
            rotated_vertices.push(v_row);
        }

        // 2. 各セグメント（0..4）における各プロファイルエッジを生成
        // profile_edges[seg_idx][edge_idx]
        let mut profile_edges: Vec<Vec<zenith_topo::Edge>> = Vec::with_capacity(num_segments + 1);

        for seg in 0..=num_segments {
            let theta = seg as f64 * d_theta;
            let mut seg_edges = Vec::with_capacity(num_edges);

            for i in 0..num_edges {
                let next_i = (i + 1) % num_edges;
                let v_start = rotated_vertices[seg][i].clone();
                let v_end = rotated_vertices[seg][next_i].clone();

                let orig_edge = &wire.edges[i].edge;
                let mut rot_cps = Vec::with_capacity(orig_edge.curve.control_points.len());
                for cp in &orig_edge.curve.control_points {
                    let p = cp.point;
                    let v_p = p - axis_origin;
                    let proj_len = v_p.dot(&axis_dir);
                    let p_center = axis_origin + axis_dir * proj_len;
                    let v_radial = p - p_center;
                    let radius = v_radial.norm();
                    let rot_p = if radius < 1e-12 {
                        p
                    } else {
                        let x_axis = v_radial / radius;
                        let y_axis = axis_dir.cross(&x_axis);
                        p_center + (x_axis * theta.cos() + y_axis * theta.sin()) * radius
                    };
                    rot_cps.push(ControlPoint3::new(rot_p, cp.weight));
                }
                let rot_curve = NurbsCurve3::new(
                    orig_edge.curve.degree,
                    rot_cps,
                    orig_edge.curve.knots.clone(),
                )?;
                let edge = zenith_topo::Edge::new(rot_curve, v_start, v_end, tol.linear);
                seg_edges.push(edge);
            }
            profile_edges.push(seg_edges);
        }

        // 3. 各セグメントにおける円弧方向エッジ（90度弧）の生成
        // arc_edges[seg_idx][vertex_idx]
        let mut arc_edges: Vec<Vec<zenith_topo::Edge>> = Vec::with_capacity(num_segments);

        for seg in 0..num_segments {
            let theta_start = seg as f64 * d_theta;
            let theta_mid = theta_start + d_theta / 2.0;
            let theta_end = theta_start + d_theta;

            let mut seg_arcs = Vec::with_capacity(num_edges);
            for i in 0..num_edges {
                let v_start = rotated_vertices[seg][i].clone();
                let v_end = if seg + 1 == num_segments {
                    rotated_vertices[0][i].clone()
                } else {
                    rotated_vertices[seg + 1][i].clone()
                };

                let p = wire.edges[i].start_vertex().point;
                let v_p = p - axis_origin;
                let proj_len = v_p.dot(&axis_dir);
                let p_center = axis_origin + axis_dir * proj_len;
                let v_radial = p - p_center;
                let radius = v_radial.norm();

                if radius < 1e-9 {
                    // 軸上特異点: 直線退化エッジ
                    let line = zenith_topo::Edge::line_between(v_start, v_end)?;
                    seg_arcs.push(line);
                } else {
                    let x_axis = v_radial / radius;
                    let y_axis = axis_dir.cross(&x_axis);

                    let p0 = p_center + (x_axis * theta_start.cos() + y_axis * theta_start.sin()) * radius;
                    let p_mid = p_center + (x_axis * theta_mid.cos() + y_axis * theta_mid.sin()) * (radius / wm);
                    let p1 = p_center + (x_axis * theta_end.cos() + y_axis * theta_end.sin()) * radius;

                    let arc_curve = NurbsCurve3::new(
                        2,
                        vec![
                            ControlPoint3::unweighted(p0),
                            ControlPoint3::new(p_mid, wm),
                            ControlPoint3::unweighted(p1),
                        ],
                        KnotVector::clamped_uniform(3, 2),
                    )?;
                    let edge = zenith_topo::Edge::new(arc_curve, v_start, v_end, tol.linear);
                    seg_arcs.push(edge);
                }
            }
            arc_edges.push(seg_arcs);
        }

        // 4. 各セグメント $\times$ 各エッジの 90 度有理 NURBS 回転曲面 Face を生成
        let mut faces = Vec::with_capacity(num_segments * num_edges);

        for seg in 0..num_segments {
            let next_seg = (seg + 1) % num_segments;

            for i in 0..num_edges {
                let next_i = (i + 1) % num_edges;

                let left_edge = profile_edges[seg][i].clone();
                let right_edge = if next_seg == 0 {
                    profile_edges[0][i].clone()
                } else {
                    profile_edges[seg + 1][i].clone()
                };

                let bot_arc = arc_edges[seg][i].clone();
                let top_arc = arc_edges[seg][next_i].clone();

                // 母線方向を逆に取るぶん、ワイヤも逆に回す。
                // [Bot(arc), Right(prof), Rev(Top)(arc), Rev(Left)(prof)]
                let face_wire = zenith_topo::Wire::new(vec![
                    zenith_topo::OrientedEdge::forward(bot_arc.clone()),
                    zenith_topo::OrientedEdge::forward(right_edge.clone()),
                    zenith_topo::OrientedEdge::reversed(top_arc.clone()),
                    zenith_topo::OrientedEdge::reversed(left_edge.clone()),
                ]);

                // 単一回転セグメント有理NURBS曲面の構築
                let curve = &left_edge.curve;
                let mut surf = Self::revolve_curve_segment(curve, axis_origin, axis_dir, d_theta, tol)?;
                // 母線方向の行の順を逆にして du x dv を外向きにする。
                // そのままだと立体の内側を向き、面積分の符号が逆になる。
                surf.control_points.reverse();

                faces.push(zenith_topo::Face::simple(zenith_topo::FaceGeometry::Nurbs(surf), face_wire));

            }
        }

        let shell = zenith_topo::Shell::closed(faces);
        let report = shell.validate_closed(tol);
        if !report.is_valid() {
            return Err(format!("Revolve solid validation failed: {:?}", report.errors));
        }
        crate::validated_solid(shell)
    }

    /// 任意の回転角度（0 < angle_rad <= 2*PI）で閉断面ワイヤを軸まわりに回転させ、端面キャップ付き完全閉B-Repソリッドを構築
    pub fn revolve_wire_partial_solid(
        wire: &zenith_topo::Wire,
        axis_origin: Point3,
        axis_dir: Vec3,
        angle_rad: f64,
        tol: &Tolerance,
    ) -> Result<zenith_topo::Solid, String> {
        if angle_rad <= 1e-6 {
            return Err("Revolve angle must be positive".to_string());
        }
        if (angle_rad - std::f64::consts::TAU).abs() < 1e-5 || angle_rad >= std::f64::consts::TAU {
            return Self::revolve_wire_solid(wire, axis_origin, axis_dir, tol);
        }

        if !wire.is_closed(tol) {
            return Err("Revolve solid requires a closed wire".to_string());
        }
        let num_edges = wire.edges.len();
        if num_edges < 3 {
            return Err("Revolve solid requires at least 3 edges".to_string());
        }

        let axis_dir = axis_dir
            .try_normalize_safe(1e-12)
            .ok_or("Axis direction is zero")?;

        // 90度以下になるようセグメント数を決定（1〜4）
        let num_segments = ((angle_rad / (std::f64::consts::FRAC_PI_2)).ceil() as usize).max(1);
        let d_theta = angle_rad / num_segments as f64;
        let wm = (d_theta / 2.0).cos();

        // 1. 各頂点の各セグメント回転位置の頂点マトリクス
        let mut rotated_vertices: Vec<Vec<zenith_topo::Vertex>> = Vec::with_capacity(num_segments + 1);

        for seg in 0..=num_segments {
            let theta = seg as f64 * d_theta;
            let mut v_row = Vec::with_capacity(num_edges);
            for oe in &wire.edges {
                let p = oe.start_vertex().point;
                let v_p = p - axis_origin;
                let proj_len = v_p.dot(&axis_dir);
                let p_center = axis_origin + axis_dir * proj_len;
                let v_radial = p - p_center;
                let radius = v_radial.norm();

                let rot_p = if radius < 1e-12 {
                    p
                } else {
                    let x_axis = v_radial / radius;
                    let y_axis = axis_dir.cross(&x_axis);
                    p_center + (x_axis * theta.cos() + y_axis * theta.sin()) * radius
                };
                v_row.push(zenith_topo::Vertex::new(rot_p, tol.linear));
            }
            rotated_vertices.push(v_row);
        }

        // 2. 各セグメント（0..=num_segments）におけるプロファイルエッジ群
        let mut profile_edges: Vec<Vec<zenith_topo::Edge>> = Vec::with_capacity(num_segments + 1);

        for seg in 0..=num_segments {
            let theta = seg as f64 * d_theta;
            let mut seg_edges = Vec::with_capacity(num_edges);

            for i in 0..num_edges {
                let next_i = (i + 1) % num_edges;
                let v_start = rotated_vertices[seg][i].clone();
                let v_end = rotated_vertices[seg][next_i].clone();

                let orig_edge = &wire.edges[i].edge;
                let mut rot_cps = Vec::with_capacity(orig_edge.curve.control_points.len());
                for cp in &orig_edge.curve.control_points {
                    let p = cp.point;
                    let v_p = p - axis_origin;
                    let proj_len = v_p.dot(&axis_dir);
                    let p_center = axis_origin + axis_dir * proj_len;
                    let v_radial = p - p_center;
                    let radius = v_radial.norm();
                    let rot_p = if radius < 1e-12 {
                        p
                    } else {
                        let x_axis = v_radial / radius;
                        let y_axis = axis_dir.cross(&x_axis);
                        p_center + (x_axis * theta.cos() + y_axis * theta.sin()) * radius
                    };
                    rot_cps.push(ControlPoint3::new(rot_p, cp.weight));
                }
                let rot_curve = NurbsCurve3::new(
                    orig_edge.curve.degree,
                    rot_cps,
                    orig_edge.curve.knots.clone(),
                )?;
                let edge = zenith_topo::Edge::new(rot_curve, v_start, v_end, tol.linear);
                seg_edges.push(edge);
            }
            profile_edges.push(seg_edges);
        }

        // 3. 各セグメントにおける円弧方向エッジの生成
        let mut arc_edges: Vec<Vec<zenith_topo::Edge>> = Vec::with_capacity(num_segments);

        for seg in 0..num_segments {
            let theta_start = seg as f64 * d_theta;
            let theta_mid = theta_start + d_theta / 2.0;
            let theta_end = theta_start + d_theta;

            let mut seg_arcs = Vec::with_capacity(num_edges);
            for i in 0..num_edges {
                let v_start = rotated_vertices[seg][i].clone();
                let v_end = rotated_vertices[seg + 1][i].clone();

                let p = wire.edges[i].start_vertex().point;
                let v_p = p - axis_origin;
                let proj_len = v_p.dot(&axis_dir);
                let p_center = axis_origin + axis_dir * proj_len;
                let v_radial = p - p_center;
                let radius = v_radial.norm();

                if radius < 1e-9 {
                    let line = zenith_topo::Edge::line_between(v_start, v_end)?;
                    seg_arcs.push(line);
                } else {
                    let x_axis = v_radial / radius;
                    let y_axis = axis_dir.cross(&x_axis);

                    let p0 = p_center + (x_axis * theta_start.cos() + y_axis * theta_start.sin()) * radius;
                    let p_mid = p_center + (x_axis * theta_mid.cos() + y_axis * theta_mid.sin()) * (radius / wm);
                    let p1 = p_center + (x_axis * theta_end.cos() + y_axis * theta_end.sin()) * radius;

                    let arc_curve = NurbsCurve3::new(
                        2,
                        vec![
                            ControlPoint3::unweighted(p0),
                            ControlPoint3::new(p_mid, wm),
                            ControlPoint3::unweighted(p1),
                        ],
                        KnotVector::clamped_uniform(3, 2),
                    )?;
                    let edge = zenith_topo::Edge::new(arc_curve, v_start, v_end, tol.linear);
                    seg_arcs.push(edge);
                }
            }
            arc_edges.push(seg_arcs);
        }

        // 4. 側面Face群の生成
        let mut faces = Vec::with_capacity(num_segments * num_edges + 2);

        for seg in 0..num_segments {
            for i in 0..num_edges {
                let next_i = (i + 1) % num_edges;

                let left_edge = profile_edges[seg][i].clone();
                let right_edge = profile_edges[seg + 1][i].clone();
                let bot_arc = arc_edges[seg][i].clone();
                let top_arc = arc_edges[seg][next_i].clone();

                let face_wire = zenith_topo::Wire::new(vec![
                    zenith_topo::OrientedEdge::forward(left_edge.clone()),
                    zenith_topo::OrientedEdge::forward(top_arc.clone()),
                    zenith_topo::OrientedEdge::reversed(right_edge.clone()),
                    zenith_topo::OrientedEdge::reversed(bot_arc.clone()),
                ]);

                let curve = &left_edge.curve;
                let surf = Self::revolve_curve_segment(curve, axis_origin, axis_dir, d_theta, tol)?;
                faces.push(zenith_topo::Face::simple(zenith_topo::FaceGeometry::Nurbs(surf), face_wire));

            }
        }

        // 5. 始点断面キャップFace（theta = 0）: ワイヤ逆順共有、外向き法線
        let mut start_raw_edges = Vec::with_capacity(num_edges);
        for i in 0..num_edges {
            start_raw_edges.push(zenith_topo::OrientedEdge::forward(profile_edges[0][i].clone()));
        }
        let start_raw_wire = zenith_topo::Wire::new(start_raw_edges);
        let start_face = create_cap_face(&start_raw_wire, true, tol)?;
        faces.push(start_face);

        // 6. 終点断面キャップFace（theta = angle_rad）: ワイヤ正順共有、外向き法線
        let mut end_raw_edges = Vec::with_capacity(num_edges);
        for i in 0..num_edges {
            end_raw_edges.push(zenith_topo::OrientedEdge::forward(profile_edges[num_segments][i].clone()));
        }
        let end_raw_wire = zenith_topo::Wire::new(end_raw_edges);
        let end_face = create_cap_face(&end_raw_wire, false, tol)?;
        faces.push(end_face);



        let shell = zenith_topo::Shell::closed(faces);
        let report = shell.validate_closed(tol);
        if !report.is_valid() {
            return Err(format!("Partial revolve solid validation failed: {:?}", report.errors));
        }
        crate::validated_solid(shell)
    }
}

/// 閉じた平坦ワイヤから端面キャップFaceを生成（is_bottom: true の場合は反転して逆向き外向き法線にする）
fn create_cap_face(wire: &zenith_topo::Wire, is_bottom: bool, _tol: &Tolerance) -> Result<zenith_topo::Face, String> {
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

    let plane = zenith_geom::PlaneSurface3::new(p0, u_axis, v_axis).ok_or("Failed to create cap plane")?;

    let cap_wire = if is_bottom {
        let mut rev_edges = Vec::with_capacity(n_pts);
        for oe in wire.edges.iter().rev() {
            rev_edges.push(zenith_topo::OrientedEdge::new(oe.edge.clone(), oe.orientation.reversed()));
        }
        zenith_topo::Wire::new(rev_edges)
    } else {
        wire.clone()
    };

    Ok(zenith_topo::Face::simple(zenith_topo::FaceGeometry::Plane(plane), cap_wire))
}




