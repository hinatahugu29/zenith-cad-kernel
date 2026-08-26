use crate::nurbs_curve::NurbsCurve3;
use serde::{Deserialize, Serialize};
use zenith_math::{Point3, Vec3, Vec3Ext};

/// 汎用3次元曲線トレイト
pub trait Curve3: std::fmt::Debug + Send + Sync {
    /// パラメータ範囲 [t_min, t_max]
    fn param_range(&self) -> (f64, f64);
    /// 座標評価
    fn evaluate(&self, t: f64) -> Point3;
    /// 接線ベクトル（正規化）
    fn tangent(&self, t: f64) -> Option<Vec3>;
    /// NURBS曲線表現への変換（可能な場合）
    fn to_nurbs(&self) -> Option<NurbsCurve3>;
}

/// 3次元線分（直線）
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct Line3 {
    pub start: Point3,
    pub end: Point3,
}

impl Line3 {
    pub fn new(start: Point3, end: Point3) -> Self {
        Self { start, end }
    }

    pub fn length(&self) -> f64 {
        (self.end - self.start).norm()
    }

    pub fn direction(&self) -> Option<Vec3> {
        (self.end - self.start).try_normalize_safe(1e-12)
    }
}

impl Curve3 for Line3 {
    fn param_range(&self) -> (f64, f64) {
        (0.0, 1.0)
    }

    fn evaluate(&self, t: f64) -> Point3 {
        Point3::from(self.start.coords * (1.0 - t) + self.end.coords * t)
    }

    fn tangent(&self, _t: f64) -> Option<Vec3> {
        self.direction()
    }

    fn to_nurbs(&self) -> Option<NurbsCurve3> {
        NurbsCurve3::bspline_from_points(1, vec![self.start, self.end]).ok()
    }
}

/// 3次元円弧
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct Circle3 {
    pub center: Point3,
    pub radius: f64,
    pub normal: Vec3,
    pub x_axis: Vec3,
    pub start_angle: f64,
    pub end_angle: f64,
}

impl Circle3 {
    pub fn new(
        center: Point3,
        radius: f64,
        normal: Vec3,
        start_angle: f64,
        end_angle: f64,
    ) -> Option<Self> {
        let normal = normal.try_normalize_safe(1e-12)?;
        // 任意の直交ベクトル
        let arb = if normal.x.abs() < 0.9 {
            Vec3::new(1.0, 0.0, 0.0)
        } else {
            Vec3::new(0.0, 1.0, 0.0)
        };
        let x_axis = normal.cross(&arb).try_normalize_safe(1e-12)?;
        Some(Self {
            center,
            radius,
            normal,
            x_axis,
            start_angle,
            end_angle,
        })
    }
}

impl Curve3 for Circle3 {
    fn param_range(&self) -> (f64, f64) {
        (self.start_angle, self.end_angle)
    }

    fn evaluate(&self, t: f64) -> Point3 {
        let y_axis = self.normal.cross(&self.x_axis);
        let cos = t.cos();
        let sin = t.sin();
        self.center + (self.x_axis * cos + y_axis * sin) * self.radius
    }

    fn tangent(&self, t: f64) -> Option<Vec3> {
        let y_axis = self.normal.cross(&self.x_axis);
        let cos = t.cos();
        let sin = t.sin();
        (-self.x_axis * sin + y_axis * cos).try_normalize_safe(1e-12)
    }

    fn to_nurbs(&self) -> Option<NurbsCurve3> {
        let delta = self.end_angle - self.start_angle;
        if delta.abs() < 1e-12 {
            return None;
        }

        let y_axis = self.normal.cross(&self.x_axis);
        let num_segments = ((delta.abs() / std::f64::consts::FRAC_PI_2 - 1e-9).ceil() as usize).max(1);
        let d_theta = delta / num_segments as f64;
        let wm = (d_theta / 2.0).cos();

        let mut control_points = Vec::with_capacity(2 * num_segments + 1);

        // 始点制御点
        let p0 = self.center + (self.x_axis * self.start_angle.cos() + y_axis * self.start_angle.sin()) * self.radius;
        control_points.push(crate::nurbs_curve::ControlPoint3::unweighted(p0));

        for seg in 0..num_segments {
            let theta_start = self.start_angle + seg as f64 * d_theta;
            let theta_mid = theta_start + d_theta / 2.0;
            let theta_end = theta_start + d_theta;

            // 中間制御点 (重み wm)
            let p_mid = self.center + (self.x_axis * theta_mid.cos() + y_axis * theta_mid.sin()) * (self.radius / wm);
            control_points.push(crate::nurbs_curve::ControlPoint3::new(p_mid, wm));

            // 終点制御点 (重み 1.0)
            let p_end = self.center + (self.x_axis * theta_end.cos() + y_axis * theta_end.sin()) * self.radius;
            control_points.push(crate::nurbs_curve::ControlPoint3::unweighted(p_end));
        }

        // ノットベクトルの構築: [0, 0, 0, 1, 1, 2, 2, ..., N, N, N]
        let mut knots = Vec::with_capacity(control_points.len() + 3);
        knots.push(0.0);
        knots.push(0.0);
        knots.push(0.0);
        for i in 1..num_segments {
            knots.push(i as f64);
            knots.push(i as f64);
        }
        let end_k = num_segments as f64;
        knots.push(end_k);
        knots.push(end_k);
        knots.push(end_k);

        NurbsCurve3::new(2, control_points, crate::bspline_basis::KnotVector::new(knots)).ok()
    }
}

/// 3次元楕円弧
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct Ellipse3 {
    pub center: Point3,
    pub major_radius: f64,
    pub minor_radius: f64,
    pub normal: Vec3,
    pub major_axis: Vec3,
    pub start_angle: f64,
    pub end_angle: f64,
}

impl Ellipse3 {
    pub fn new(
        center: Point3,
        major_radius: f64,
        minor_radius: f64,
        normal: Vec3,
        major_axis: Vec3,
        start_angle: f64,
        end_angle: f64,
    ) -> Option<Self> {
        let normal = normal.try_normalize_safe(1e-12)?;
        let major_axis = major_axis.try_normalize_safe(1e-12)?;
        if major_radius <= 0.0 || minor_radius <= 0.0 {
            return None;
        }
        Some(Self {
            center,
            major_radius,
            minor_radius,
            normal,
            major_axis,
            start_angle,
            end_angle,
        })
    }

    /// 短軸方向の単位ベクトル
    pub fn minor_axis(&self) -> Vec3 {
        self.normal.cross(&self.major_axis)
    }
}

impl Curve3 for Ellipse3 {
    fn param_range(&self) -> (f64, f64) {
        (self.start_angle, self.end_angle)
    }

    fn evaluate(&self, t: f64) -> Point3 {
        let minor_axis = self.minor_axis();
        let cos = t.cos();
        let sin = t.sin();
        self.center + self.major_axis * (self.major_radius * cos) + minor_axis * (self.minor_radius * sin)
    }

    fn tangent(&self, t: f64) -> Option<Vec3> {
        let minor_axis = self.minor_axis();
        let cos = t.cos();
        let sin = t.sin();
        (-self.major_axis * (self.major_radius * sin) + minor_axis * (self.minor_radius * cos)).try_normalize_safe(1e-12)
    }

    fn to_nurbs(&self) -> Option<NurbsCurve3> {
        let delta = self.end_angle - self.start_angle;
        if delta.abs() < 1e-12 {
            return None;
        }

        let minor_axis = self.minor_axis();
        let num_segments = ((delta.abs() / std::f64::consts::FRAC_PI_2 - 1e-9).ceil() as usize).max(1);
        let d_theta = delta / num_segments as f64;
        let wm = (d_theta / 2.0).cos();

        let mut control_points = Vec::with_capacity(2 * num_segments + 1);

        // 始点制御点
        let p0 = self.center
            + self.major_axis * (self.major_radius * self.start_angle.cos())
            + minor_axis * (self.minor_radius * self.start_angle.sin());
        control_points.push(crate::nurbs_curve::ControlPoint3::unweighted(p0));

        for seg in 0..num_segments {
            let theta_start = self.start_angle + seg as f64 * d_theta;
            let theta_mid = theta_start + d_theta / 2.0;
            let theta_end = theta_start + d_theta;

            // 中間制御点 (重み wm)
            let p_mid = self.center
                + self.major_axis * ((self.major_radius / wm) * theta_mid.cos())
                + minor_axis * ((self.minor_radius / wm) * theta_mid.sin());
            control_points.push(crate::nurbs_curve::ControlPoint3::new(p_mid, wm));

            // 終点制御点 (重み 1.0)
            let p_end = self.center
                + self.major_axis * (self.major_radius * theta_end.cos())
                + minor_axis * (self.minor_radius * theta_end.sin());
            control_points.push(crate::nurbs_curve::ControlPoint3::unweighted(p_end));
        }

        let mut knots = Vec::with_capacity(control_points.len() + 3);
        knots.push(0.0);
        knots.push(0.0);
        knots.push(0.0);
        for i in 1..num_segments {
            knots.push(i as f64);
            knots.push(i as f64);
        }
        let end_k = num_segments as f64;
        knots.push(end_k);
        knots.push(end_k);
        knots.push(end_k);

        NurbsCurve3::new(2, control_points, crate::bspline_basis::KnotVector::new(knots)).ok()
    }
}

