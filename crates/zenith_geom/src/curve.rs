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
        // 円弧の正確な9制御点NURBS表現（有理B-Spline）などを今後追加可能
        None
    }
}
