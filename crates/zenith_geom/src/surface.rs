use crate::coons_patch::CoonsPatch3;
use crate::nurbs_surface::NurbsSurface3;
use serde::{Deserialize, Serialize};
use zenith_math::{Point3, Vec3, Vec3Ext};

/// 汎用3次元曲面トレイト
pub trait Surface3: std::fmt::Debug + Send + Sync {
    /// UVパラメータ範囲 [u_min, u_max] x [v_min, v_max]
    fn param_range(&self) -> ((f64, f64), (f64, f64));
    /// UV座標における3次元座標の評価
    fn evaluate(&self, u: f64, v: f64) -> Point3;
    /// UV座標における法線ベクトルの評価（正規化）
    fn normal(&self, u: f64, v: f64) -> Option<Vec3>;
}

/// 3次元平面曲面
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct PlaneSurface3 {
    pub origin: Point3,
    pub u_axis: Vec3,
    pub v_axis: Vec3,
    pub normal: Vec3,
}

impl PlaneSurface3 {
    pub fn new(origin: Point3, u_axis: Vec3, v_axis: Vec3) -> Option<Self> {
        let u_axis = u_axis.try_normalize_safe(1e-12)?;
        let v_axis = v_axis.try_normalize_safe(1e-12)?;
        let normal = u_axis.cross(&v_axis).try_normalize_safe(1e-12)?;
        Some(Self {
            origin,
            u_axis,
            v_axis,
            normal,
        })
    }
}

impl Surface3 for PlaneSurface3 {
    fn param_range(&self) -> ((f64, f64), (f64, f64)) {
        (
            (-f64::INFINITY, f64::INFINITY),
            (-f64::INFINITY, f64::INFINITY),
        )
    }

    fn evaluate(&self, u: f64, v: f64) -> Point3 {
        self.origin + self.u_axis * u + self.v_axis * v
    }

    fn normal(&self, _u: f64, _v: f64) -> Option<Vec3> {
        Some(self.normal)
    }
}

impl Surface3 for NurbsSurface3 {
    fn param_range(&self) -> ((f64, f64), (f64, f64)) {
        self.param_range()
    }

    fn evaluate(&self, u: f64, v: f64) -> Point3 {
        self.evaluate(u, v)
    }

    fn normal(&self, u: f64, v: f64) -> Option<Vec3> {
        self.normal(u, v)
    }
}

impl Surface3 for CoonsPatch3 {
    fn param_range(&self) -> ((f64, f64), (f64, f64)) {
        ((0.0, 1.0), (0.0, 1.0))
    }

    fn evaluate(&self, u: f64, v: f64) -> Point3 {
        self.evaluate(u, v)
    }

    fn normal(&self, u: f64, v: f64) -> Option<Vec3> {
        self.normal(u, v)
    }
}
