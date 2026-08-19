use zenith_geom::{ControlPoint3, KnotVector, NurbsCurve3};
use zenith_math::{Point3, Tolerance};
use zenith_topo::{OrientedEdge, Solid, Vertex, Wire};


/// 3D ポリラインおよび角丸め（フィレット）ポリラインモデリングビルダー
pub struct PolylineBuilder;

/// ポリラインパスのセグメント種別
pub enum PathSegment {
    /// 直線セグメント（始点・終点）
    Line { start: Point3, end: Point3 },
    /// 有理2次NURBS円弧セグメント（始点・制御点・終点・重み・中心・半径）
    Arc {
        start: Point3,
        mid_cp: Point3,
        end: Point3,
        weight: f64,
        center: Point3,
        radius: f64,
    },
}

impl PolylineBuilder {
    /// 3D点列を結ぶ角丸め（フィレット）ポリラインセグメント列を計算
    pub fn build_filleted_path(
        points: &[Point3],
        corner_radius: f64,
    ) -> Result<Vec<PathSegment>, String> {
        if points.len() < 2 {
            return Err("Polyline requires at least 2 points".to_string());
        }

        if points.len() == 2 || corner_radius <= 1e-6 {
            let mut segs = Vec::with_capacity(points.len() - 1);
            for i in 0..points.len() - 1 {
                segs.push(PathSegment::Line {
                    start: points[i],
                    end: points[i + 1],
                });
            }
            return Ok(segs);
        }

        let mut segs = Vec::new();
        let mut curr_start = points[0];

        for i in 1..points.len() - 1 {
            let p_prev = points[i - 1];
            let p_curr = points[i];
            let p_next = points[i + 1];

            let v1 = p_curr - p_prev;
            let v2 = p_next - p_curr;
            let len1 = v1.norm();
            let len2 = v2.norm();

            if len1 < 1e-9 || len2 < 1e-9 {
                continue;
            }

            let d1 = v1 / len1;
            let d2 = v2 / len2;

            let dot = d1.dot(&d2).clamp(-1.0, 1.0);
            let theta = dot.acos(); // 方向変化角 (0: 直進, PI: 反転)

            // 直進または180度反転の場合はフィレット不要
            if theta < 1e-4 || (theta - std::f64::consts::PI).abs() < 1e-4 {
                segs.push(PathSegment::Line {
                    start: curr_start,
                    end: p_curr,
                });
                curr_start = p_curr;
                continue;
            }

            let half_angle = theta / 2.0;
            let tangent_dist = corner_radius * (half_angle).tan();

            // 直線セグメントの長さ上限チェック
            let max_t = (len1 * 0.45).min(len2 * 0.45);
            let actual_t = tangent_dist.min(max_t);
            let actual_r = actual_t / (half_angle).tan();

            let t1 = p_curr - d1 * actual_t;
            let t2 = p_curr + d2 * actual_t;

            // 1. 直線区間 (curr_start -> t1)
            if (t1 - curr_start).norm() > 1e-6 {
                segs.push(PathSegment::Line {
                    start: curr_start,
                    end: t1,
                });
            }

            // 2. 円弧区間 (t1 -> t2, 制御点 p_curr)
            let wm = (half_angle).cos();
            let normal = d1.cross(&d2).normalize();
            let binormal = d1.cross(&normal).normalize(); // 中心方向
            let center = t1 + binormal * actual_r;

            segs.push(PathSegment::Arc {
                start: t1,
                mid_cp: p_curr,
                end: t2,
                weight: wm,
                center,
                radius: actual_r,
            });

            curr_start = t2;
        }

        // 最後の直線区間
        let p_last = *points.last().unwrap();
        if (p_last - curr_start).norm() > 1e-6 {
            segs.push(PathSegment::Line {
                start: curr_start,
                end: p_last,
            });
        }

        Ok(segs)
    }

    /// 角丸めポリラインに沿って円形パイプ閉ソリッド（Solid）を構築
    pub fn sweep_pipe_polyline(
        points: &[Point3],
        radius: f64,
        corner_radius: f64,
        _tol: &Tolerance,
    ) -> Result<Solid, String> {

        if radius <= 1e-6 {
            return Err("Pipe radius must be positive".to_string());
        }

        let segments = Self::build_filleted_path(points, corner_radius)?;
        if segments.is_empty() {
            return Err("No path segments generated".to_string());
        }

        // セグメント列から密なパス点列をサンプリング
        let mut dense_path = Vec::new();
        for seg in &segments {
            match seg {
                PathSegment::Line { start, end } => {
                    let len = (*end - *start).norm();
                    let n = ((len / (radius * 0.5)).ceil() as usize).max(4);
                    for step in 0..n {
                        let t = step as f64 / n as f64;
                        dense_path.push(*start + (*end - *start) * t);
                    }
                }
                PathSegment::Arc {
                    start,
                    mid_cp,
                    end,
                    weight,
                    ..
                } => {
                    let n = 12;
                    let curve = NurbsCurve3::new(
                        2,
                        vec![
                            ControlPoint3::unweighted(*start),
                            ControlPoint3::new(*mid_cp, *weight),
                            ControlPoint3::unweighted(*end),
                        ],
                        KnotVector::clamped_uniform(3, 2),
                    )?;
                    for step in 0..n {
                        let u = step as f64 / n as f64;
                        dense_path.push(curve.evaluate(u));
                    }
                }
            }
        }
        dense_path.push(*points.last().unwrap());

        // 密なパス点列から1次B-Spline（折れ線）曲線を生成
        let n_dense = dense_path.len();
        let cps: Vec<_> = dense_path.iter().map(|&p| ControlPoint3::unweighted(p)).collect();
        let path_curve = NurbsCurve3::new(1, cps, KnotVector::clamped_uniform(n_dense, 1))?;

        crate::sweep::SweepBuilder::sweep_circle_along_curve(&path_curve, radius, n_dense)
    }

    /// 角丸めポリラインに沿って任意閉断面ワイヤを掃引した閉ソリッドを構築
    pub fn sweep_wire_polyline(
        profile_pts: &[Point3],
        path_points: &[Point3],
        corner_radius: f64,
        tol: &Tolerance,
    ) -> Result<Solid, String> {
        let segments = Self::build_filleted_path(path_points, corner_radius)?;
        if segments.is_empty() {
            return Err("No path segments generated".to_string());
        }

        let mut dense_path = Vec::new();
        for seg in &segments {
            match seg {
                PathSegment::Line { start, end } => {
                    let len = (*end - *start).norm();
                    let n = ((len / 5.0).ceil() as usize).max(4);
                    for step in 0..n {
                        let t = step as f64 / n as f64;
                        dense_path.push(*start + (*end - *start) * t);
                    }
                }
                PathSegment::Arc {
                    start,
                    mid_cp,
                    end,
                    weight,
                    ..
                } => {
                    let n = 12;
                    let curve = NurbsCurve3::new(
                        2,
                        vec![
                            ControlPoint3::unweighted(*start),
                            ControlPoint3::new(*mid_cp, *weight),
                            ControlPoint3::unweighted(*end),
                        ],
                        KnotVector::clamped_uniform(3, 2),
                    )?;
                    for step in 0..n {
                        let u = step as f64 / n as f64;
                        dense_path.push(curve.evaluate(u));
                    }
                }
            }
        }
        dense_path.push(*path_points.last().unwrap());

        // 閉断面ワイヤの構築
        let num_p = profile_pts.len();
        let mut edges = Vec::with_capacity(num_p);
        let verts: Vec<_> = profile_pts.iter().map(|&p| Vertex::from_point(p)).collect();
        for i in 0..num_p {
            let next_i = (i + 1) % num_p;
            edges.push(OrientedEdge::forward(zenith_topo::Edge::line_between(
                verts[i].clone(),
                verts[next_i].clone(),
            )?));
        }
        let profile_wire = Wire::new(edges);

        let n_dense = dense_path.len();
        let cps: Vec<_> = dense_path.iter().map(|&p| ControlPoint3::unweighted(p)).collect();
        let path_curve = NurbsCurve3::new(1, cps, KnotVector::clamped_uniform(n_dense, 1))?;

        crate::sweep::SweepBuilder::sweep_wire_along_curve(&profile_wire, &path_curve, n_dense, tol)
    }
}

