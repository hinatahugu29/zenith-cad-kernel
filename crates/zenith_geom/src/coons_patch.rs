use crate::nurbs_curve::NurbsCurve3;
use serde::{Deserialize, Serialize};
use zenith_math::{Point3, Point3Ext, Tolerance, Vec3, Vec3Ext};

/// 4つの境界曲線から定義される双線形 Coons パッチ
/// - `c0(u)`: v = 0 の境界
/// - `c1(u)`: v = 1 の境界
/// - `d0(v)`: u = 0 の境界
/// - `d1(v)`: u = 1 の境界
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct CoonsPatch3 {
    pub c0: NurbsCurve3,
    pub c1: NurbsCurve3,
    pub d0: NurbsCurve3,
    pub d1: NurbsCurve3,
    // 4隅のコーナー点
    pub p00: Point3, // c0(0) == d0(0)
    pub p10: Point3, // c0(1) == d1(0)
    pub p01: Point3, // c1(0) == d0(1)
    pub p11: Point3, // c1(1) == d1(1)
}

impl CoonsPatch3 {
    /// 4本の境界曲線からCoonsパッチを作成（4隅のトポロジー連続性を自動検証）
    pub fn new(
        c0: NurbsCurve3,
        c1: NurbsCurve3,
        d0: NurbsCurve3,
        d1: NurbsCurve3,
        tol: &Tolerance,
    ) -> Result<Self, String> {
        let (u0_min, u0_max) = c0.param_range();
        let (u1_min, u1_max) = c1.param_range();
        let (v0_min, v0_max) = d0.param_range();
        let (v1_min, v1_max) = d1.param_range();

        let p_c0_0 = c0.evaluate(u0_min);
        let p_c0_1 = c0.evaluate(u0_max);
        let p_c1_0 = c1.evaluate(u1_min);
        let p_c1_1 = c1.evaluate(u1_max);

        let p_d0_0 = d0.evaluate(v0_min);
        let p_d0_1 = d0.evaluate(v0_max);
        let p_d1_0 = d1.evaluate(v1_min);
        let p_d1_1 = d1.evaluate(v1_max);

        // コーナー連続性の確認
        if !p_c0_0.is_coincident_with(&p_d0_0, tol.linear) {
            return Err(format!(
                "Corner P00 mismatch: c0(0)={:?} != d0(0)={:?} (dist={})",
                p_c0_0,
                p_d0_0,
                p_c0_0.distance_to(&p_d0_0)
            ));
        }
        if !p_c0_1.is_coincident_with(&p_d1_0, tol.linear) {
            return Err(format!(
                "Corner P10 mismatch: c0(1)={:?} != d1(0)={:?} (dist={})",
                p_c0_1,
                p_d1_0,
                p_c0_1.distance_to(&p_d1_0)
            ));
        }
        if !p_c1_0.is_coincident_with(&p_d0_1, tol.linear) {
            return Err(format!(
                "Corner P01 mismatch: c1(0)={:?} != d0(1)={:?} (dist={})",
                p_c1_0,
                p_d0_1,
                p_c1_0.distance_to(&p_d0_1)
            ));
        }
        if !p_c1_1.is_coincident_with(&p_d1_1, tol.linear) {
            return Err(format!(
                "Corner P11 mismatch: c1(1)={:?} != d1(1)={:?} (dist={})",
                p_c1_1,
                p_d1_1,
                p_c1_1.distance_to(&p_d1_1)
            ));
        }

        let p00 = Point3::from((p_c0_0.coords + p_d0_0.coords) * 0.5);
        let p10 = Point3::from((p_c0_1.coords + p_d1_0.coords) * 0.5);
        let p01 = Point3::from((p_c1_0.coords + p_d0_1.coords) * 0.5);
        let p11 = Point3::from((p_c1_1.coords + p_d1_1.coords) * 0.5);

        Ok(Self {
            c0,
            c1,
            d0,
            d1,
            p00,
            p10,
            p01,
            p11,
        })
    }

    /// パラメータ (u, v) in [0, 1] x [0, 1] での3次元座標の補間計算
    pub fn evaluate(&self, u: f64, v: f64) -> Point3 {
        let u = u.clamp(0.0, 1.0);
        let v = v.clamp(0.0, 1.0);

        let (u0_min, u0_max) = self.c0.param_range();
        let (u1_min, u1_max) = self.c1.param_range();
        let (v0_min, v0_max) = self.d0.param_range();
        let (v1_min, v1_max) = self.d1.param_range();

        let pt_c0 = self.c0.evaluate(u0_min + u * (u0_max - u0_min));
        let pt_c1 = self.c1.evaluate(u1_min + u * (u1_max - u1_min));
        let pt_d0 = self.d0.evaluate(v0_min + v * (v0_max - v0_min));
        let pt_d1 = self.d1.evaluate(v1_min + v * (v1_max - v1_min));

        // ルールド曲面 1: Sc(u, v) = (1-v) * c0(u) + v * c1(u)
        let sc = pt_c0.coords * (1.0 - v) + pt_c1.coords * v;

        // ルールド曲面 2: Sd(u, v) = (1-u) * d0(v) + u * d1(v)
        let sd = pt_d0.coords * (1.0 - u) + pt_d1.coords * u;

        // コーナー補正曲面: Scd(u, v)
        let scd = self.p00.coords * ((1.0 - u) * (1.0 - v))
            + self.p10.coords * (u * (1.0 - v))
            + self.p01.coords * ((1.0 - u) * v)
            + self.p11.coords * (u * v);

        Point3::from(sc + sd - scd)
    }

    /// 数値微分による法線ベクトルの計算
    pub fn normal(&self, u: f64, v: f64) -> Option<Vec3> {
        let eps = 1e-5;
        let u_plus = (u + eps).min(1.0);
        let u_minus = (u - eps).max(0.0);
        let du = (self.evaluate(u_plus, v) - self.evaluate(u_minus, v)) / (u_plus - u_minus);

        let v_plus = (v + eps).min(1.0);
        let v_minus = (v - eps).max(0.0);
        let dv = (self.evaluate(u, v_plus) - self.evaluate(u, v_minus)) / (v_plus - v_minus);

        du.cross(&dv).try_normalize_safe(1e-12)
    }
}
