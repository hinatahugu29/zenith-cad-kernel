use crate::nurbs_curve::NurbsCurve3;
use crate::surface::Surface3;
use serde::{Deserialize, Serialize};
use zenith_math::{Point3, Point3Ext, Tolerance, Vec3, Vec3Ext};

/// カーブネットワーク補間 Gordon 曲面 (Gordon Surface)
/// 複数のU方向カーブ群とV方向カーブ群の格子ネットワークから滑らかな曲面を生成
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct GordonSurface3 {
    /// U方向パラメータ位置 [u_0, u_1, ..., u_n]
    pub u_params: Vec<f64>,
    /// V方向パラメータ位置 [v_0, v_1, ..., v_m]
    pub v_params: Vec<f64>,
    /// U方向のプロファイル曲線群 (各曲線は v in [0, 1] で評価)
    pub u_curves: Vec<NurbsCurve3>,
    /// V方向のプロファイル曲線群 (各曲線は u in [0, 1] で評価)
    pub v_curves: Vec<NurbsCurve3>,
    /// 交差点マトリクス P_{i,j} = u_i(v_j) == v_j(u_i)
    pub intersection_points: Vec<Vec<Point3>>,
}

impl GordonSurface3 {
    /// カーブネットワークからGordon曲面を作成
    pub fn new(
        u_curves: Vec<NurbsCurve3>,
        v_curves: Vec<NurbsCurve3>,
        tol: &Tolerance,
    ) -> Result<Self, String> {
        let n = u_curves.len();
        let m = v_curves.len();

        if n < 2 || m < 2 {
            return Err("Gordon surface requires at least 2 U-curves and 2 V-curves".to_string());
        }

        // パラメータ割り当て [0.0, ..., 1.0]
        let mut u_params = Vec::with_capacity(n);
        for i in 0..n {
            u_params.push(i as f64 / (n - 1) as f64);
        }

        let mut v_params = Vec::with_capacity(m);
        for j in 0..m {
            v_params.push(j as f64 / (m - 1) as f64);
        }

        // 交差点の検証と収集
        let mut intersection_points = vec![vec![Point3::origin(); m]; n];

        for i in 0..n {
            let (v_min, v_max) = u_curves[i].param_range();
            for j in 0..m {
                let (u_min, u_max) = v_curves[j].param_range();

                let v_val = v_min + v_params[j] * (v_max - v_min);
                let u_val = u_min + u_params[i] * (u_max - u_min);

                let p_u = u_curves[i].evaluate(v_val);
                let p_v = v_curves[j].evaluate(u_val);

                if !p_u.is_coincident_with(&p_v, tol.linear) {
                    return Err(format!(
                        "Curve network intersection mismatch at ({}, {}): u-curve={:?} != v-curve={:?} (dist={})",
                        i, j, p_u, p_v, p_u.distance_to(&p_v)
                    ));
                }

                intersection_points[i][j] = Point3::from((p_u.coords + p_v.coords) * 0.5);
            }
        }

        Ok(Self {
            u_params,
            v_params,
            u_curves,
            v_curves,
            intersection_points,
        })
    }

    /// Lagrange基底多項式 L_i(t)
    fn lagrange_basis(params: &[f64], i: usize, t: f64) -> f64 {
        let mut l = 1.0;
        let ti = params[i];
        for (k, &tk) in params.iter().enumerate() {
            if k != i {
                let denom = ti - tk;
                if denom.abs() > 1e-15 {
                    l *= (t - tk) / denom;
                }
            }
        }
        l
    }

    /// パラメータ (u, v) in [0, 1] x [0, 1] での3次元座標の評価
    /// S(u, v) = S_u(u, v) + S_v(u, v) - S_uv(u, v)
    pub fn evaluate(&self, u: f64, v: f64) -> Point3 {
        let u = u.clamp(0.0, 1.0);
        let v = v.clamp(0.0, 1.0);

        let n = self.u_curves.len();
        let m = self.v_curves.len();

        // 1. S_u(u, v) = sum_{i=0}^{n-1} L_i^u(u) * u_i(v)
        let mut s_u = Vec3::zeros();
        for i in 0..n {
            let l_u = Self::lagrange_basis(&self.u_params, i, u);
            let (v_min, v_max) = self.u_curves[i].param_range();
            let pt = self.u_curves[i].evaluate(v_min + v * (v_max - v_min));
            s_u += pt.coords * l_u;
        }

        // 2. S_v(u, v) = sum_{j=0}^{m-1} L_j^v(v) * v_j(u)
        let mut s_v = Vec3::zeros();
        for j in 0..m {
            let l_v = Self::lagrange_basis(&self.v_params, j, v);
            let (u_min, u_max) = self.v_curves[j].param_range();
            let pt = self.v_curves[j].evaluate(u_min + u * (u_max - u_min));
            s_v += pt.coords * l_v;
        }

        // 3. S_uv(u, v) = sum_{i=0}^{n-1} sum_{j=0}^{m-1} L_i^u(u) * L_j^v(v) * P_{i,j}
        let mut s_uv = Vec3::zeros();
        for i in 0..n {
            let l_u = Self::lagrange_basis(&self.u_params, i, u);
            for j in 0..m {
                let l_v = Self::lagrange_basis(&self.v_params, j, v);
                s_uv += self.intersection_points[i][j].coords * (l_u * l_v);
            }
        }

        Point3::from(s_u + s_v - s_uv)
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

impl Surface3 for GordonSurface3 {
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
