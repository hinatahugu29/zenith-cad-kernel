use zenith_geom::{ControlPoint3, KnotVector, NurbsCurve3, NurbsSurface3, PlaneSurface3};
use zenith_math::{Point3, Tolerance, Vec3};
use zenith_topo::{Edge, Face, FaceGeometry, OrientedEdge, Shell, Solid, Vertex, Wire};

/// 3Dガイド曲線に沿った断面スイープ（パイプ・チューブ・異形断面）ビルダー
pub struct SweepBuilder;

/// 3D パス沿いの RMF 標架データ (中心座標, 接線ベクトル, 法線ベクトル, 従法線ベクトル)
type RmfFrame = (Point3, Vec3, Vec3, Vec3);

/// スイープ方向の補間次数。断面を直線で繋ぐと、パイプは16角柱の連なりに
/// なってしまい、面が各断面で折れる（C0）。3次で補間すると全断面をちょうど
/// 通りながら C2 連続な滑らかな管になる。
const SWEEP_SKIN_DEGREE: usize = 3;

/// 断面列を通る3次B-スプラインの制御点と、全行で共有するノットベクトル。
///
/// テンソル積曲面にするため、どの行も同じパラメータ付けとノットを使う。
/// Piegl & Tiller の大域補間（平均化ノット）で、データ点数と制御点数が
/// 一致するので端条件を足す必要がない。
fn skin_rows(
    rows: &[Vec<ControlPoint3>],
    parameters: &[f64],
) -> Result<(Vec<Vec<ControlPoint3>>, KnotVector), String> {
    let count = parameters.len();
    if rows.iter().any(|row| row.len() != count) {
        return Err("Skinning rows must all have one point per section".to_string());
    }
    if count < SWEEP_SKIN_DEGREE + 1 {
        // 断面が少なすぎて3次補間できないときは折れ線のまま返す。
        return Ok((rows.to_vec(), KnotVector::clamped_uniform(count, 1)));
    }

    let degree = SWEEP_SKIN_DEGREE;
    let knots = averaged_knot_vector(parameters, degree);

    // 補間行列 N[i][j] = B_j(u_i)
    let mut matrix = nalgebra::DMatrix::<f64>::zeros(count, count);
    for (i, u) in parameters.iter().enumerate() {
        let span = knots.find_span(count, degree, *u);
        let basis = knots.basis_functions(span, degree, *u);
        for (offset, value) in basis.iter().enumerate() {
            let column = span + offset - degree;
            if column < count {
                matrix[(i, column)] = *value;
            }
        }
    }

    let decomposition = matrix.lu();

    let mut skinned = Vec::with_capacity(rows.len());
    for row in rows {
        let mut solved = vec![Point3::origin(); count];
        for axis in 0..3 {
            let rhs = nalgebra::DVector::<f64>::from_iterator(
                count,
                row.iter().map(|cp| cp.point[axis]),
            );
            let solution = decomposition
                .solve(&rhs)
                .ok_or_else(|| "Skinning interpolation matrix is singular".to_string())?;
            for (index, value) in solution.iter().enumerate() {
                solved[index][axis] = *value;
            }
        }

        // 重みは行ごとに一定なので、そのまま持ち回れば有理性は保たれる。
        let weight = row.first().map(|cp| cp.weight).unwrap_or(1.0);
        skinned.push(
            solved
                .into_iter()
                .map(|point| ControlPoint3::new(point, weight))
                .collect(),
        );
    }

    Ok((skinned, knots))
}

/// Averaged knot vector for global interpolation (Piegl & Tiller eq. 9.8).
fn averaged_knot_vector(parameters: &[f64], degree: usize) -> KnotVector {
    let count = parameters.len();
    let mut knots = Vec::with_capacity(count + degree + 1);
    for _ in 0..=degree {
        knots.push(0.0);
    }
    for j in 1..count.saturating_sub(degree) {
        let sum: f64 = parameters[j..j + degree].iter().sum();
        knots.push(sum / degree as f64);
    }
    for _ in 0..=degree {
        knots.push(1.0);
    }
    KnotVector::new(knots)
}

/// Chord-length parameters over the section centres, normalised to [0, 1].
fn section_parameters(points: &[Point3]) -> Vec<f64> {
    let count = points.len();
    if count < 2 {
        return vec![0.0; count];
    }

    let mut cumulative = Vec::with_capacity(count);
    cumulative.push(0.0);
    let mut total = 0.0;
    for index in 1..count {
        total += (points[index] - points[index - 1]).norm();
        cumulative.push(total);
    }

    if total <= f64::EPSILON {
        return (0..count)
            .map(|index| index as f64 / (count - 1) as f64)
            .collect();
    }

    cumulative
        .into_iter()
        .map(|distance| distance / total)
        .collect()
}

impl SweepBuilder {
    /// 3D NURBS 軌道曲線に沿った連続最小回転標架 (Parallel Transport Frame / RMF) を算出
    pub fn compute_rmf_frames(path: &NurbsCurve3, num_sections: usize) -> Vec<RmfFrame> {
        let n_sec = num_sections.max(8);
        let (t_min, t_max) = path.param_range();

        // 1. 各断面位置での位置と接線ベクトルのサンプリング
        let mut pts = Vec::with_capacity(n_sec);
        for i in 0..n_sec {
            let u = i as f64 / (n_sec - 1) as f64;
            let t = t_min + u * (t_max - t_min);
            let pt = path.evaluate(t);
            let ders = path.evaluate_derivatives(t, 1);
            let tangent = if ders.len() > 1 && ders[1].norm() > 1e-9 {
                ders[1].normalize()
            } else {
                Vec3::new(0.0, 0.0, 1.0)
            };
            pts.push((pt, tangent));
        }

        // 2. 滑らかな最小回転標架 (RMF) の算出
        let mut frames = Vec::with_capacity(n_sec);

        // 初期標架 (t0, n0, b0)
        let t0 = pts[0].1;
        let n0 = if t0.x.abs() < 0.9 {
            t0.cross(&Vec3::new(1.0, 0.0, 0.0)).normalize()
        } else {
            t0.cross(&Vec3::new(0.0, 1.0, 0.0)).normalize()
        };
        let b0 = t0.cross(&n0).normalize();
        frames.push((pts[0].0, t0, n0, b0));

        // Rodrigues回転による連続標架伝播
        for i in 0..n_sec - 1 {
            let (_, t_curr, n_curr, _) = frames[i];
            let (x_next, t_next) = pts[i + 1];

            let dot = t_curr.dot(&t_next).clamp(-1.0, 1.0);
            let axis = t_curr.cross(&t_next);
            let axis_len = axis.norm();

            let mut n_next = if axis_len > 1e-8 {
                let u_axis = axis / axis_len;
                let angle = dot.acos();
                let c = angle.cos();
                let s = angle.sin();
                n_curr * c + u_axis.cross(&n_curr) * s + u_axis * (u_axis.dot(&n_curr) * (1.0 - c))
            } else if dot < -0.99999 {
                -n_curr
            } else {
                n_curr
            };

            n_next = (n_next - t_next * t_next.dot(&n_next)).normalize();
            let b_next = t_next.cross(&n_next).normalize();
            frames.push((x_next, t_next, n_next, b_next));
        }

        frames
    }

    /// 3D NURBS軌道曲線（パス）に沿って半径 radius の円形断面を掃引したスイープソリッドを生成（Rodrigues回転最小ねじれ標架による滑らかなパイプ）
    pub fn sweep_circle_along_curve(
        path: &NurbsCurve3,
        radius: f64,
        num_sections: usize,
    ) -> Result<Solid, String> {
        let n_sec = num_sections.max(8);
        let frames = Self::compute_rmf_frames(path, n_sec);

        // 1. 各象限 (quad 0..4) の制御点グリッド行を計算
        let weight = std::f64::consts::FRAC_1_SQRT_2;
        let mut quad_rows: Vec<(Vec<ControlPoint3>, Vec<ControlPoint3>, Vec<ControlPoint3>)> = Vec::with_capacity(4);

        for quad in 0..4 {
            let (ang0, ang1) = match quad {
                0 => (0.0, std::f64::consts::FRAC_PI_2),
                1 => (std::f64::consts::FRAC_PI_2, std::f64::consts::PI),
                2 => (std::f64::consts::PI, 3.0 * std::f64::consts::FRAC_PI_2),
                _ => (
                    3.0 * std::f64::consts::FRAC_PI_2,
                    2.0 * std::f64::consts::PI,
                ),
            };

            let mut row0 = Vec::with_capacity(n_sec);
            let mut row1 = Vec::with_capacity(n_sec);
            let mut row2 = Vec::with_capacity(n_sec);

            for &(ctr, _t, normal, binormal) in frames.iter().take(n_sec) {
                let p_s = ctr + normal * (radius * ang0.cos()) + binormal * (radius * ang0.sin());
                let p_e = ctr + normal * (radius * ang1.cos()) + binormal * (radius * ang1.sin());
                let corner = ctr + (p_s - ctr) + (p_e - ctr);

                row0.push(ControlPoint3::unweighted(p_s));
                row1.push(ControlPoint3::new(corner, weight));
                row2.push(ControlPoint3::unweighted(p_e));
            }

            quad_rows.push((row0, row1, row2));
        }

        // 1b. 断面列を3次で補間し、折れのない滑らかな側面にする。
        let section_centres: Vec<Point3> = frames.iter().take(n_sec).map(|frame| frame.0).collect();
        let sweep_parameters = section_parameters(&section_centres);
        let mut skinned_quad_rows = Vec::with_capacity(4);
        let mut sweep_knots = KnotVector::clamped_uniform(n_sec, 1);
        for (row0, row1, row2) in &quad_rows {
            let (skinned, knots) = skin_rows(
                &[row0.clone(), row1.clone(), row2.clone()],
                &sweep_parameters,
            )?;
            sweep_knots = knots;
            skinned_quad_rows.push((skinned[0].clone(), skinned[1].clone(), skinned[2].clone()));
        }
        let sweep_degree = if skinned_quad_rows[0].0.len() == n_sec && n_sec >= SWEEP_SKIN_DEGREE + 1
        {
            SWEEP_SKIN_DEGREE
        } else {
            1
        };

        // 2. 始点・終点リングの共有頂点 (4頂点ずつ)
        let mut vb = Vec::with_capacity(4);
        let mut vt = Vec::with_capacity(4);
        for quad in 0..4 {
            vb.push(Vertex::from_point(quad_rows[quad].0[0].point));
            vt.push(Vertex::from_point(quad_rows[quad].0[n_sec - 1].point));
        }

        // 3. 4本の縦シームエッジ（vb[q] -> vt[q]）。側面と同じ補間を使わないと
        //    エッジが面の境界から浮く。
        let mut ev = Vec::with_capacity(4);
        for quad in 0..4 {
            let curve = NurbsCurve3::new(
                sweep_degree,
                skinned_quad_rows[quad].0.clone(),
                sweep_knots.clone(),
            )?;
            let edge = Edge::new(curve, vb[quad].clone(), vt[quad].clone(), 1e-6);
            ev.push(edge);
        }

        // 4. 始点円弧エッジ (bottom_ring_edges) と 終点円弧エッジ (top_ring_edges)
        let mut bottom_ring_edges = Vec::with_capacity(4);
        let mut top_ring_edges = Vec::with_capacity(4);
        for quad in 0..4 {
            let next_q = (quad + 1) % 4;
            let (ref r0, ref r1, ref r2) = quad_rows[quad];

            let arc_b = Edge::new(
                NurbsCurve3::new(
                    2,
                    vec![r0[0], r1[0], r2[0]],
                    KnotVector::clamped_uniform(3, 2),
                )?,
                vb[quad].clone(),
                vb[next_q].clone(),
                1e-6,
            );
            let arc_t = Edge::new(
                NurbsCurve3::new(
                    2,
                    vec![r0[n_sec - 1], r1[n_sec - 1], r2[n_sec - 1]],
                    KnotVector::clamped_uniform(3, 2),
                )?,
                vt[quad].clone(),
                vt[next_q].clone(),
                1e-6,
            );

            bottom_ring_edges.push(arc_b);
            top_ring_edges.push(arc_t);
        }

        // 5. 4つの側面 NURBS Face の構築
        let mut faces = Vec::with_capacity(6);
        for quad in 0..4 {
            let next_q = (quad + 1) % 4;
            let (ref r0, ref r1, ref r2) = skinned_quad_rows[quad];

            let s = NurbsSurface3::new(
                2,
                sweep_degree,
                vec![r0.clone(), r1.clone(), r2.clone()],
                KnotVector::clamped_uniform(3, 2),
                sweep_knots.clone(),
            )?;

            let wire = Wire::new(vec![
                OrientedEdge::forward(bottom_ring_edges[quad].clone()),
                OrientedEdge::forward(ev[next_q].clone()),
                OrientedEdge::reversed(top_ring_edges[quad].clone()),
                OrientedEdge::reversed(ev[quad].clone()),
            ]);

            faces.push(Face::simple(FaceGeometry::Nurbs(s), wire));
        }

        // 6. 始端面キャップ (4象限 NURBS パッチ)
        let (ctr0, _t0, _n0, _b0) = frames[0];
        let v_bot_center = Vertex::new(ctr0, 1e-6);
        let mut bot_spoke_edges = Vec::with_capacity(4);
        for quad in 0..4 {
            let v_q = &bottom_ring_edges[quad].start_vertex;
            let spoke_line = NurbsCurve3::new(
                1,
                vec![
                    ControlPoint3::unweighted(ctr0),
                    ControlPoint3::unweighted(v_q.point),
                ],
                KnotVector::clamped_uniform(2, 1),
            )?;
            bot_spoke_edges.push(Edge::new(spoke_line, v_bot_center.clone(), v_q.clone(), 1e-6));
        }

        for quad in 0..4 {
            let next_q = (quad + 1) % 4;
            let p_s = bottom_ring_edges[quad].start_vertex.point;
            let p_e = bottom_ring_edges[quad].end_vertex.point;
            let corner = ctr0 + (p_s - ctr0) + (p_e - ctr0);

            // 3x2 有理 NURBS パッチ (U: 2次円弧 p_e -> p_s 3点, V: 1次放射 外周 -> 中心 2点)
            // 法線: (p_e -> p_s) x (外周 -> 中心) = -t0 (外向き法線)
            let row0 = vec![ControlPoint3::new(p_e, 1.0), ControlPoint3::new(ctr0, 1.0)];
            let row1 = vec![ControlPoint3::new(corner, std::f64::consts::FRAC_1_SQRT_2), ControlPoint3::new(ctr0, std::f64::consts::FRAC_1_SQRT_2)];
            let row2 = vec![ControlPoint3::new(p_s, 1.0), ControlPoint3::new(ctr0, 1.0)];

            let surf = NurbsSurface3::new(
                2,
                1,
                vec![row0, row1, row2],
                KnotVector::clamped_uniform(3, 2),
                KnotVector::clamped_uniform(2, 1),
            )?;

            // 3辺ワイヤ (外向き法線 -t0: CCW, v_next_q -> v_q -> v_ctr -> v_next_q)
            let wire = Wire::new(vec![
                OrientedEdge::reversed(bottom_ring_edges[quad].clone()),
                OrientedEdge::reversed(bot_spoke_edges[quad].clone()),
                OrientedEdge::forward(bot_spoke_edges[next_q].clone()),
            ]);
            faces.push(Face::simple(FaceGeometry::Nurbs(surf), wire));
        }


        // 7. 終端面キャップ (4象限 NURBS パッチ)
        let (ctr1, _t1, _n1, _b1) = frames[n_sec - 1];
        let v_top_center = Vertex::new(ctr1, 1e-6);
        let mut top_spoke_edges = Vec::with_capacity(4);
        for quad in 0..4 {
            let v_q = &top_ring_edges[quad].start_vertex;
            let spoke_line = NurbsCurve3::new(
                1,
                vec![
                    ControlPoint3::unweighted(ctr1),
                    ControlPoint3::unweighted(v_q.point),
                ],
                KnotVector::clamped_uniform(2, 1),
            )?;
            top_spoke_edges.push(Edge::new(spoke_line, v_top_center.clone(), v_q.clone(), 1e-6));
        }

        for quad in 0..4 {
            let next_q = (quad + 1) % 4;
            let p_s = top_ring_edges[quad].start_vertex.point;
            let p_e = top_ring_edges[quad].end_vertex.point;
            let corner = ctr1 + (p_s - ctr1) + (p_e - ctr1);

            let row0 = vec![ControlPoint3::new(p_s, 1.0), ControlPoint3::new(ctr1, 1.0)];
            let row1 = vec![ControlPoint3::new(corner, std::f64::consts::FRAC_1_SQRT_2), ControlPoint3::new(ctr1, std::f64::consts::FRAC_1_SQRT_2)];
            let row2 = vec![ControlPoint3::new(p_e, 1.0), ControlPoint3::new(ctr1, 1.0)];

            let surf = NurbsSurface3::new(
                2,
                1,
                vec![row0, row1, row2],
                KnotVector::clamped_uniform(3, 2),
                KnotVector::clamped_uniform(2, 1),
            )?;

            // 3辺ワイヤ (外向き法線 +t1: CCW, v_q -> v_next_q -> v_ctr -> v_q)
            let wire = Wire::new(vec![
                OrientedEdge::forward(top_ring_edges[quad].clone()),
                OrientedEdge::reversed(top_spoke_edges[next_q].clone()),
                OrientedEdge::forward(top_spoke_edges[quad].clone()),
            ]);
            faces.push(Face::simple(FaceGeometry::Nurbs(surf), wire));
        }

        // 8. 閉シェル化とSolid検証
        let shell = Shell::closed(faces);
        crate::validated_solid(shell)
    }







    /// 任意の2D/3D閉断面ワイヤを3D NURBS軌道曲線（パス）に沿って掃引した完全閉B-Repソリッドを生成

    pub fn sweep_wire_along_curve(
        profile_wire: &Wire,
        path: &NurbsCurve3,
        num_sections: usize,
        tol: &Tolerance,
    ) -> Result<Solid, String> {
        if !profile_wire.is_closed(tol) {
            return Err("Profile wire must be closed for sweep solid".to_string());
        }

        let k = profile_wire.edges.len();
        if k < 3 {
            return Err("Profile wire must have at least 3 edges".to_string());
        }

        let n_sec = num_sections.max(8);
        let frames = Self::compute_rmf_frames(path, n_sec);

        // 1. 各フレーム位置での断面頂点マトリクス [n_sec][k] を算出
        //    断面ワイヤの各頂点 (x, y) を RMFフレーム (ctr, t, n, b) に写像: P = ctr + n*x + b*y
        let mut vertex_matrix = Vec::with_capacity(n_sec);
        for frame in &frames {
            let (ctr, _t, n_vec, b_vec) = *frame;
            let mut row = Vec::with_capacity(k);
            for oe in &profile_wire.edges {
                let local_pt = oe.start_vertex().point;
                let world_pt = ctr + n_vec * local_pt.x + b_vec * local_pt.y;
                row.push(Vertex::from_point(world_pt));
            }
            vertex_matrix.push(row);
        }

        // 2. 縦方向の継ぎ目エッジ（Seam/Pillarエッジ）群 [k] を構築。
        //    断面を直線で繋ぐと面が各断面で折れるので、3次で補間する。
        let section_centres: Vec<Point3> = frames.iter().take(n_sec).map(|frame| frame.0).collect();
        let sweep_parameters = section_parameters(&section_centres);
        let sweep_degree = if n_sec >= SWEEP_SKIN_DEGREE + 1 {
            SWEEP_SKIN_DEGREE
        } else {
            1
        };

        let mut seam_edges = Vec::with_capacity(k);
        for j in 0..k {
            let mut ctrl_pts = Vec::with_capacity(n_sec);
            for i in 0..n_sec {
                ctrl_pts.push(ControlPoint3::unweighted(vertex_matrix[i][j].point));
            }
            let (skinned, seam_knots) =
                skin_rows(std::slice::from_ref(&ctrl_pts), &sweep_parameters)?;
            let seam_curve =
                NurbsCurve3::new(sweep_degree, skinned[0].clone(), seam_knots.clone())?;
            let v_start = vertex_matrix[0][j].clone();
            let v_end = vertex_matrix[n_sec - 1][j].clone();
            let edge = Edge::new(seam_curve, v_start, v_end, tol.linear);
            seam_edges.push(edge);
        }

        // 3. 各フレームでのリングエッジ群 [n_sec][k] を構築
        let mut ring_edges_matrix = Vec::with_capacity(n_sec);
        for i in 0..n_sec {
            let (ctr, _t, n_vec, b_vec) = frames[i];
            let mut row = Vec::with_capacity(k);
            for j in 0..k {
                let next_j = (j + 1) % k;
                let orig_curve = &profile_wire.edges[j].edge.curve;

                // 制御点をフレームに写像
                let mapped_cps: Vec<ControlPoint3> = orig_curve
                    .control_points
                    .iter()
                    .map(|cp| {
                        let lp = cp.point;
                        let wp = ctr + n_vec * lp.x + b_vec * lp.y;
                        ControlPoint3::new(wp, cp.weight)
                    })
                    .collect();

                let mapped_curve = NurbsCurve3::new(
                    orig_curve.degree,
                    mapped_cps,
                    orig_curve.knots.clone(),
                )?;

                let v_s = vertex_matrix[i][j].clone();
                let v_e = vertex_matrix[i][next_j].clone();
                let edge = Edge::new(mapped_curve, v_s, v_e, tol.linear);
                row.push(edge);
            }
            ring_edges_matrix.push(row);
        }

        let mut faces = Vec::with_capacity(k + 2);

        // 4. 各エッジ j の側面 NURBS Face を構築
        for j in 0..k {
            let next_j = (j + 1) % k;
            let orig_curve = &profile_wire.edges[j].edge.curve;
            let num_u = orig_curve.control_points.len();
            let degree_u = orig_curve.degree;

            // 制御点グリッド [row_u][col_v]
            let mut ctrl_pts_grid = vec![Vec::with_capacity(n_sec); num_u];
            for frame in frames.iter().take(n_sec) {
                let (ctr, _t, n_vec, b_vec) = *frame;
                for (l, cp) in orig_curve.control_points.iter().enumerate() {
                    let lp = cp.point;
                    let wp = ctr + n_vec * lp.x + b_vec * lp.y;
                    ctrl_pts_grid[l].push(ControlPoint3::new(wp, cp.weight));
                }
            }

            let knots_u = orig_curve.knots.clone();
            let (skinned_grid, knots_v) = skin_rows(&ctrl_pts_grid, &sweep_parameters)?;

            let side_surf =
                NurbsSurface3::new(degree_u, sweep_degree, skinned_grid, knots_u, knots_v)?;

            let bot_edge = ring_edges_matrix[0][j].clone();
            let top_edge = ring_edges_matrix[n_sec - 1][j].clone();
            let left_seam = seam_edges[j].clone();
            let right_seam = seam_edges[next_j].clone();

            let side_wire = Wire::new(vec![
                OrientedEdge::forward(bot_edge),
                OrientedEdge::forward(right_seam),
                OrientedEdge::reversed(top_edge),
                OrientedEdge::reversed(left_seam),
            ]);

            faces.push(Face::simple(FaceGeometry::Nurbs(side_surf), side_wire));
        }

        // 5. 始端面キャップ (Start Cap: PLANE, 外向き法線 -t0)
        let (ctr0, _t0, n0, b0) = frames[0];
        let p_start_cap = PlaneSurface3::new(ctr0, b0, n0).ok_or("Failed to create start cap plane")?;
        let mut start_cap_edges = Vec::with_capacity(k);
        for j in (0..k).rev() {
            start_cap_edges.push(OrientedEdge::reversed(ring_edges_matrix[0][j].clone()));
        }
        faces.push(Face::simple(FaceGeometry::Plane(p_start_cap), Wire::new(start_cap_edges)));

        // 6. 終端面キャップ (End Cap: PLANE, 外向き法線 +t1)
        let (ctr1, _t1, n1, b1) = frames[n_sec - 1];
        let p_end_cap = PlaneSurface3::new(ctr1, n1, b1).ok_or("Failed to create end cap plane")?;
        let mut end_cap_edges = Vec::with_capacity(k);
        for j in 0..k {
            end_cap_edges.push(OrientedEdge::forward(ring_edges_matrix[n_sec - 1][j].clone()));
        }
        faces.push(Face::simple(FaceGeometry::Plane(p_end_cap), Wire::new(end_cap_edges)));

        // 7. 閉シェル化とSolid検証
        let shell = Shell::closed(faces);
        crate::validated_solid(shell)
    }
}
