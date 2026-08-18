use crate::nurbs_surface::NurbsSurface3;
use serde::{Deserialize, Serialize};
use zenith_math::{Point3, Vec3, Vec3Ext};

/// 微分幾何曲率解析結果
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct SurfaceCurvature {
    /// 第一基本形式係数 (E = Du·Du, F = Du·Dv, G = Dv·Dv)
    pub e: f64,
    pub f: f64,
    pub g: f64,
    /// 第二基本形式係数 (L = Duu·n, M = Duv·n, N = Dvv·n)
    pub l: f64,
    pub m: f64,
    pub n: f64,
    /// ガウス曲率 K = (LN - M^2) / (EG - F^2)
    pub gaussian_curvature: f64,
    /// 平均曲率 H = (EN + GL - 2FM) / (2(EG - F^2))
    pub mean_curvature: f64,
    /// 最大主曲率 kappa_1
    pub principal_curvature_1: f64,
    /// 最小主曲率 kappa_2
    pub principal_curvature_2: f64,
}

impl NurbsSurface3 {
    /// 2階偏導関数 (Du, Dv, Duu, Dvv, Duv) の評価
    pub fn evaluate_derivatives_2nd(
        &self,
        u: f64,
        v: f64,
    ) -> (Point3, Vec3, Vec3, Vec3, Vec3, Vec3) {
        let num_u = self.control_points.len();
        let num_v = self.control_points[0].len();

        let span_u = self.knots_u.find_span(num_u, self.degree_u, u);
        let span_v = self.knots_v.find_span(num_v, self.degree_v, v);

        let ders_u = self
            .knots_u
            .ders_basis_functions(span_u, self.degree_u, 2, u);
        let ders_v = self
            .knots_v
            .ders_basis_functions(span_v, self.degree_v, 2, v);

        // a_{k, l} (k in 0..=2, l in 0..=2)
        let mut a = vec![vec![nalgebra::Vector4::zeros(); 3]; 3];

        for k in 0..=2 {
            for l in 0..=2 {
                for j in 0..=self.degree_v {
                    let v_idx = span_v - self.degree_v + j;
                    let mut temp = nalgebra::Vector4::zeros();
                    for i in 0..=self.degree_u {
                        let u_idx = span_u - self.degree_u + i;
                        let pw = self.control_points[u_idx][v_idx].to_homogeneous();
                        let d_u = if k < ders_u.len() { ders_u[k][i] } else { 0.0 };
                        temp += pw * d_u;
                    }
                    let d_v = if l < ders_v.len() { ders_v[l][j] } else { 0.0 };
                    a[k][l] += temp * d_v;
                }
            }
        }

        let p = crate::nurbs_curve::ControlPoint3::from_homogeneous(&a[0][0]).point;
        let w = a[0][0].w;

        let to_vec3 = |v4: &nalgebra::Vector4<f64>| Vec3::new(v4.x, v4.y, v4.z);

        let a00 = to_vec3(&a[0][0]);
        let a10 = to_vec3(&a[1][0]);
        let a01 = to_vec3(&a[0][1]);
        let a20 = to_vec3(&a[2][0]);
        let a02 = to_vec3(&a[0][2]);
        let a11 = to_vec3(&a[1][1]);

        let w00 = w;
        let w10 = a[1][0].w;
        let w01 = a[0][1].w;
        let w20 = a[2][0].w;
        let w02 = a[0][2].w;
        let w11 = a[1][1].w;

        // 1階導関数
        let du = (a10 - a00 * (w10 / w00)) / w00;
        let dv = (a01 - a00 * (w01 / w00)) / w00;

        // 2階導関数 (有理微分の商の微分公式)
        let duu = (a20 - du * (2.0 * w10) - a00 * (w20 / w00)) / w00;
        let dvv = (a02 - dv * (2.0 * w01) - a00 * (w02 / w00)) / w00;
        let duv = (a11 - du * w01 - dv * w10 - a00 * (w11 / w00)) / w00;

        (p, du, dv, duu, dvv, duv)
    }

    /// 曲面上の微分幾何・主曲率・ガウス曲率・平均曲率の厳密評価
    pub fn evaluate_curvature(&self, u: f64, v: f64) -> Option<SurfaceCurvature> {
        let (_p, du, dv, duu, dvv, duv) = self.evaluate_derivatives_2nd(u, v);
        let normal = du.cross(&dv).try_normalize_safe(1e-12)?;

        // 第一基本形式 (First Fundamental Form)
        let e = du.dot(&du);
        let f = du.dot(&dv);
        let g = dv.dot(&dv);

        // 第二基本形式 (Second Fundamental Form)
        let l = duu.dot(&normal);
        let m = duv.dot(&normal);
        let n = dvv.dot(&normal);

        let denom = e * g - f * f;
        if denom.abs() < 1e-14 {
            return None;
        }

        // ガウス曲率 K & 平均曲率 H
        let gaussian_curvature = (l * n - m * m) / denom;
        let mean_curvature = (e * n + g * l - 2.0 * f * m) / (2.0 * denom);

        // 主曲率 kappa_1, kappa_2 (H +/- sqrt(H^2 - K))
        let discr = (mean_curvature * mean_curvature - gaussian_curvature).max(0.0);
        let sqrt_discr = discr.sqrt();
        let principal_curvature_1 = mean_curvature + sqrt_discr;
        let principal_curvature_2 = mean_curvature - sqrt_discr;

        Some(SurfaceCurvature {
            e,
            f,
            g,
            l,
            m,
            n,
            gaussian_curvature,
            mean_curvature,
            principal_curvature_1,
            principal_curvature_2,
        })
    }
}
