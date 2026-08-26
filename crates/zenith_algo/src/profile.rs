//! 2D スケッチプロファイル構築ビルダー（ProfileBuilder）
//!
//! 機械設計で頻出する各種断面（長方形、角丸長方形、円、スロット長円、正多角形、穴あき複合断面）
//! を正確な有理2次NURBSエッジおよび直線エッジで構築し、押し出し・回転・ロフト等へ供給します。

use std::f64::consts::{FRAC_1_SQRT_2, PI};
use zenith_geom::{ControlPoint3, KnotVector, NurbsCurve3};
use zenith_math::{Point3, Vec3, Vec3Ext};
use zenith_topo::{Edge, OrientedEdge, Vertex, Wire};

/// 2D スケッチプロファイルビルダー
pub struct ProfileBuilder;

impl ProfileBuilder {
    /// 3D 空間内の任意の平面上に配置される長方形ワイヤを生成
    pub fn make_rectangle(
        width: f64,
        height: f64,
        center: Point3,
        normal: Vec3,
        x_axis: Vec3,
    ) -> Result<Wire, String> {
        let x_axis = x_axis
            .try_normalize_safe(1e-12)
            .ok_or("x_axis cannot be zero")?;
        let normal = normal
            .try_normalize_safe(1e-12)
            .ok_or("normal cannot be zero")?;
        let y_axis = normal.cross(&x_axis).try_normalize_safe(1e-12).ok_or("x_axis and normal must not be parallel")?;

        let half_w = width * 0.5;
        let half_h = height * 0.5;

        let p0 = center - x_axis * half_w - y_axis * half_h;
        let p1 = center + x_axis * half_w - y_axis * half_h;
        let p2 = center + x_axis * half_w + y_axis * half_h;
        let p3 = center - x_axis * half_w + y_axis * half_h;

        let v0 = Vertex::from_point(p0);
        let v1 = Vertex::from_point(p1);
        let v2 = Vertex::from_point(p2);
        let v3 = Vertex::from_point(p3);

        let e0 = Edge::line_between(v0.clone(), v1.clone())?;
        let e1 = Edge::line_between(v1.clone(), v2.clone())?;
        let e2 = Edge::line_between(v2.clone(), v3.clone())?;
        let e3 = Edge::line_between(v3.clone(), v0.clone())?;

        Ok(Wire::new(vec![
            OrientedEdge::forward(e0),
            OrientedEdge::forward(e1),
            OrientedEdge::forward(e2),
            OrientedEdge::forward(e3),
        ]))
    }

    /// 四隅に真円フィレット（corner_radius）を持つ角丸長方形ワイヤを生成（4直線＋4有理2次円弧）
    pub fn make_rounded_rectangle(
        width: f64,
        height: f64,
        corner_radius: f64,
        center: Point3,
        normal: Vec3,
        x_axis: Vec3,
    ) -> Result<Wire, String> {
        let r = corner_radius;
        let half_w = width * 0.5;
        let half_h = height * 0.5;

        if r <= 1e-6 {
            return Self::make_rectangle(width, height, center, normal, x_axis);
        }
        if r >= half_w || r >= half_h {
            return Err("Corner radius cannot exceed half of width or height".to_string());
        }

        let x_axis = x_axis
            .try_normalize_safe(1e-12)
            .ok_or("x_axis cannot be zero")?;
        let normal = normal
            .try_normalize_safe(1e-12)
            .ok_or("normal cannot be zero")?;
        let y_axis = normal.cross(&x_axis).try_normalize_safe(1e-12).ok_or("x_axis and normal must not be parallel")?;

        let dx = half_w - r;
        let dy = half_h - r;

        // 8頂点 (2D ローカル: 4直線の端点)
        let pts_2d = [
            (-dx, -half_h),
            (dx, -half_h),
            (half_w, -dy),
            (half_w, dy),
            (dx, half_h),
            (-dx, half_h),
            (-half_w, dy),
            (-half_w, -dy),
        ];

        let pts_3d: Vec<Point3> = pts_2d
            .iter()
            .map(|&(x, y)| center + x_axis * x + y_axis * y)
            .collect();
        let verts: Vec<Vertex> = pts_3d.iter().map(|&p| Vertex::from_point(p)).collect();

        let corners_2d = [
            (half_w, -half_h),
            (half_w, half_h),
            (-half_w, half_h),
            (-half_w, -half_h),
        ];

        let mut edges = Vec::with_capacity(8);

        // 4本の直線と4本の有理2次円弧を交互に配置
        // 0: 下辺直線 (-dx,-half_h) -> (dx,-half_h)
        edges.push(OrientedEdge::forward(Edge::line_between(verts[0].clone(), verts[1].clone())?));

        // 1: 右下円弧 (dx,-half_h) -> (half_w,-dy)
        let c0 = center + x_axis * corners_2d[0].0 + y_axis * corners_2d[0].1;
        let arc0 = Edge::new(
            NurbsCurve3::new(
                2,
                vec![
                    ControlPoint3::unweighted(pts_3d[1]),
                    ControlPoint3::new(c0, FRAC_1_SQRT_2),
                    ControlPoint3::unweighted(pts_3d[2]),
                ],
                KnotVector::clamped_uniform(3, 2),
            )?,
            verts[1].clone(),
            verts[2].clone(),
            1e-6,
        );
        edges.push(OrientedEdge::forward(arc0));

        // 2: 右辺直線 (half_w,-dy) -> (half_w,dy)
        edges.push(OrientedEdge::forward(Edge::line_between(verts[2].clone(), verts[3].clone())?));

        // 3: 右上円弧 (half_w,dy) -> (dx,half_h)
        let c1 = center + x_axis * corners_2d[1].0 + y_axis * corners_2d[1].1;
        let arc1 = Edge::new(
            NurbsCurve3::new(
                2,
                vec![
                    ControlPoint3::unweighted(pts_3d[3]),
                    ControlPoint3::new(c1, FRAC_1_SQRT_2),
                    ControlPoint3::unweighted(pts_3d[4]),
                ],
                KnotVector::clamped_uniform(3, 2),
            )?,
            verts[3].clone(),
            verts[4].clone(),
            1e-6,
        );
        edges.push(OrientedEdge::forward(arc1));

        // 4: 上辺直線 (dx,half_h) -> (-dx,half_h)
        edges.push(OrientedEdge::forward(Edge::line_between(verts[4].clone(), verts[5].clone())?));

        // 5: 左上円弧 (-dx,half_h) -> (-half_w,dy)
        let c2 = center + x_axis * corners_2d[2].0 + y_axis * corners_2d[2].1;
        let arc2 = Edge::new(
            NurbsCurve3::new(
                2,
                vec![
                    ControlPoint3::unweighted(pts_3d[5]),
                    ControlPoint3::new(c2, FRAC_1_SQRT_2),
                    ControlPoint3::unweighted(pts_3d[6]),
                ],
                KnotVector::clamped_uniform(3, 2),
            )?,
            verts[5].clone(),
            verts[6].clone(),
            1e-6,
        );
        edges.push(OrientedEdge::forward(arc2));

        // 6: 左辺直線 (-half_w,dy) -> (-half_w,-dy)
        edges.push(OrientedEdge::forward(Edge::line_between(verts[6].clone(), verts[7].clone())?));

        // 7: 左下円弧 (-half_w,-dy) -> (-dx,-half_h)
        let c3 = center + x_axis * corners_2d[3].0 + y_axis * corners_2d[3].1;
        let arc3 = Edge::new(
            NurbsCurve3::new(
                2,
                vec![
                    ControlPoint3::unweighted(pts_3d[7]),
                    ControlPoint3::new(c3, FRAC_1_SQRT_2),
                    ControlPoint3::unweighted(pts_3d[0]),
                ],
                KnotVector::clamped_uniform(3, 2),
            )?,
            verts[7].clone(),
            verts[0].clone(),
            1e-6,
        );
        edges.push(OrientedEdge::forward(arc3));

        Ok(Wire::new(edges))
    }

    /// 4分割有理2次NURBSによる真円ワイヤを生成
    pub fn make_circle(
        radius: f64,
        center: Point3,
        normal: Vec3,
        x_axis: Vec3,
    ) -> Result<Wire, String> {
        let x_axis = x_axis
            .try_normalize_safe(1e-12)
            .ok_or("x_axis cannot be zero")?;
        let normal = normal
            .try_normalize_safe(1e-12)
            .ok_or("normal cannot be zero")?;
        let y_axis = normal.cross(&x_axis).try_normalize_safe(1e-12).ok_or("x_axis and normal must not be parallel")?;

        let p_pts = [
            center + x_axis * radius,
            center + y_axis * radius,
            center - x_axis * radius,
            center - y_axis * radius,
        ];
        let v_pts: Vec<Vertex> = p_pts.iter().map(|&p| Vertex::from_point(p)).collect();

        let corners = [
            center + (x_axis + y_axis) * radius,
            center + (-x_axis + y_axis) * radius,
            center + (-x_axis - y_axis) * radius,
            center + (x_axis - y_axis) * radius,
        ];

        let mut edges = Vec::with_capacity(4);
        for i in 0..4 {
            let next = (i + 1) % 4;
            let arc = Edge::new(
                NurbsCurve3::new(
                    2,
                    vec![
                        ControlPoint3::unweighted(p_pts[i]),
                        ControlPoint3::new(corners[i], FRAC_1_SQRT_2),
                        ControlPoint3::unweighted(p_pts[next]),
                    ],
                    KnotVector::clamped_uniform(3, 2),
                )?,
                v_pts[i].clone(),
                v_pts[next].clone(),
                1e-6,
            );
            edges.push(OrientedEdge::forward(arc));
        }

        Ok(Wire::new(edges))
    }

    /// 2直線＋4有理2次円弧によるスロット（長円）ワイヤを生成
    pub fn make_slot(
        length: f64,
        radius: f64,
        center: Point3,
        normal: Vec3,
        x_axis: Vec3,
    ) -> Result<Wire, String> {
        let x_axis = x_axis
            .try_normalize_safe(1e-12)
            .ok_or("x_axis cannot be zero")?;
        let normal = normal
            .try_normalize_safe(1e-12)
            .ok_or("normal cannot be zero")?;
        let y_axis = normal.cross(&x_axis).try_normalize_safe(1e-12).ok_or("x_axis and normal must not be parallel")?;

        let l_half = length * 0.5;
        let loc = [
            (-l_half, -radius),
            (l_half, -radius),
            (l_half + radius, 0.0),
            (l_half, radius),
            (-l_half, radius),
            (-l_half - radius, 0.0),
        ];

        let pts_3d: Vec<Point3> = loc
            .iter()
            .map(|&(x, y)| center + x_axis * x + y_axis * y)
            .collect();
        let verts: Vec<Vertex> = pts_3d.iter().map(|&p| Vertex::from_point(p)).collect();

        let mut edges = Vec::with_capacity(6);

        // 0: 下辺直線
        edges.push(OrientedEdge::forward(Edge::line_between(verts[0].clone(), verts[1].clone())?));

        // 1: 右下円弧
        let c1 = center + x_axis * (l_half + radius) - y_axis * radius;
        let arc1 = Edge::new(
            NurbsCurve3::new(
                2,
                vec![
                    ControlPoint3::unweighted(pts_3d[1]),
                    ControlPoint3::new(c1, FRAC_1_SQRT_2),
                    ControlPoint3::unweighted(pts_3d[2]),
                ],
                KnotVector::clamped_uniform(3, 2),
            )?,
            verts[1].clone(),
            verts[2].clone(),
            1e-6,
        );
        edges.push(OrientedEdge::forward(arc1));

        // 2: 右上円弧
        let c2 = center + x_axis * (l_half + radius) + y_axis * radius;
        let arc2 = Edge::new(
            NurbsCurve3::new(
                2,
                vec![
                    ControlPoint3::unweighted(pts_3d[2]),
                    ControlPoint3::new(c2, FRAC_1_SQRT_2),
                    ControlPoint3::unweighted(pts_3d[3]),
                ],
                KnotVector::clamped_uniform(3, 2),
            )?,
            verts[2].clone(),
            verts[3].clone(),
            1e-6,
        );
        edges.push(OrientedEdge::forward(arc2));

        // 3: 上辺直線
        edges.push(OrientedEdge::forward(Edge::line_between(verts[3].clone(), verts[4].clone())?));

        // 4: 左上円弧
        let c4 = center - x_axis * (l_half + radius) + y_axis * radius;
        let arc4 = Edge::new(
            NurbsCurve3::new(
                2,
                vec![
                    ControlPoint3::unweighted(pts_3d[4]),
                    ControlPoint3::new(c4, FRAC_1_SQRT_2),
                    ControlPoint3::unweighted(pts_3d[5]),
                ],
                KnotVector::clamped_uniform(3, 2),
            )?,
            verts[4].clone(),
            verts[5].clone(),
            1e-6,
        );
        edges.push(OrientedEdge::forward(arc4));

        // 5: 左下円弧
        let c5 = center - x_axis * (l_half + radius) - y_axis * radius;
        let arc5 = Edge::new(
            NurbsCurve3::new(
                2,
                vec![
                    ControlPoint3::unweighted(pts_3d[5]),
                    ControlPoint3::new(c5, FRAC_1_SQRT_2),
                    ControlPoint3::unweighted(pts_3d[0]),
                ],
                KnotVector::clamped_uniform(3, 2),
            )?,
            verts[5].clone(),
            verts[0].clone(),
            1e-6,
        );
        edges.push(OrientedEdge::forward(arc5));

        Ok(Wire::new(edges))
    }

    /// 正N角形ワイヤを生成
    pub fn make_regular_polygon(
        num_sides: usize,
        radius: f64,
        center: Point3,
        normal: Vec3,
        x_axis: Vec3,
    ) -> Result<Wire, String> {
        if num_sides < 3 {
            return Err("Polygon must have at least 3 sides".to_string());
        }

        let x_axis = x_axis
            .try_normalize_safe(1e-12)
            .ok_or("x_axis cannot be zero")?;
        let normal = normal
            .try_normalize_safe(1e-12)
            .ok_or("normal cannot be zero")?;
        let y_axis = normal.cross(&x_axis).try_normalize_safe(1e-12).ok_or("x_axis and normal must not be parallel")?;

        let d_theta = (2.0 * PI) / (num_sides as f64);
        let mut pts = Vec::with_capacity(num_sides);

        for i in 0..num_sides {
            let theta = i as f64 * d_theta;
            let p = center + (x_axis * theta.cos() + y_axis * theta.sin()) * radius;
            pts.push(p);
        }

        let verts: Vec<Vertex> = pts.iter().map(|&p| Vertex::from_point(p)).collect();
        let mut edges = Vec::with_capacity(num_sides);

        for i in 0..num_sides {
            let next = (i + 1) % num_sides;
            let line = Edge::line_between(verts[i].clone(), verts[next].clone())?;
            edges.push(OrientedEdge::forward(line));
        }

        Ok(Wire::new(edges))
    }
}
