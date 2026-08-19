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

                // ワイヤループ: [Left(prof), Top(arc), Rev(Right)(prof), Rev(Bot)(arc)]
                let face_wire = zenith_topo::Wire::new(vec![
                    zenith_topo::OrientedEdge::forward(left_edge.clone()),
                    zenith_topo::OrientedEdge::forward(top_arc.clone()),
                    zenith_topo::OrientedEdge::reversed(right_edge.clone()),
                    zenith_topo::OrientedEdge::reversed(bot_arc.clone()),
                ]);

                // 90度回転有理NURBS曲面の構築
                let curve = &left_edge.curve;
                let surf = Self::revolve_curve(curve, axis_origin, axis_dir, d_theta, tol)?;

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
                let surf = Self::revolve_curve(curve, axis_origin, axis_dir, d_theta, tol)?;
                faces.push(zenith_topo::Face::simple(zenith_topo::FaceGeometry::Nurbs(surf), face_wire));
            }
        }

        // 5. 始点断面キャップFace（theta = 0）: ワイヤを反転（逆向き共有）、法線 -Y (外向き)
        let mut start_cap_edges = Vec::with_capacity(num_edges);
        for i in (0..num_edges).rev() {
            start_cap_edges.push(zenith_topo::OrientedEdge::reversed(profile_edges[0][i].clone()));
        }
        let start_wire = zenith_topo::Wire::new(start_cap_edges);
        let start_origin = wire.edges[0].start_vertex().point;
        let v_radial0 = (start_origin - axis_origin) - axis_dir * (start_origin - axis_origin).dot(&axis_dir);
        let r0_norm = v_radial0.try_normalize_safe(1e-12).unwrap_or(Vec3::new(1.0, 0.0, 0.0));
        let start_u_axis = -axis_dir;
        let start_v_axis = r0_norm;
        let start_plane = zenith_geom::PlaneSurface3::new(start_origin, start_u_axis, start_v_axis).ok_or("Failed plane")?;
        let start_face = zenith_topo::Face::new(
            zenith_topo::FaceGeometry::Plane(start_plane),
            start_wire,
            vec![],
            zenith_topo::Orientation::Reversed,
            tol.linear,
        );

        faces.push(start_face);

        // 6. 終点断面キャップFace（theta = angle_rad）: ワイヤを正順で共有、法線 +Y' (外向き)
        let mut end_cap_edges = Vec::with_capacity(num_edges);
        for i in 0..num_edges {
            end_cap_edges.push(zenith_topo::OrientedEdge::forward(profile_edges[num_segments][i].clone()));
        }
        let end_wire = zenith_topo::Wire::new(end_cap_edges);
        let end_origin = rotated_vertices[num_segments][0].point;
        let v_radial_end = (end_origin - axis_origin) - axis_dir * (end_origin - axis_origin).dot(&axis_dir);
        let r_end_norm = v_radial_end.try_normalize_safe(1e-12).unwrap_or(Vec3::new(1.0, 0.0, 0.0));
        let end_u_axis = r_end_norm;
        let end_v_axis = axis_dir;
        let end_plane = zenith_geom::PlaneSurface3::new(end_origin, end_u_axis, end_v_axis).ok_or("Failed plane")?;
        let end_face = zenith_topo::Face::new(
            zenith_topo::FaceGeometry::Plane(end_plane),
            end_wire,
            vec![],
            zenith_topo::Orientation::Forward,
            tol.linear,
        );
        faces.push(end_face);


        let shell = zenith_topo::Shell::closed(faces);
        let report = shell.validate_closed(tol);
        if !report.is_valid() {
            return Err(format!("Partial revolve solid validation failed: {:?}", report.errors));
        }
        crate::validated_solid(shell)
    }
}



