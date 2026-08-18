use crate::nurbs_curve::NurbsCurve3;
use crate::nurbs_surface::NurbsSurface3;
use zenith_math::{Point3, Vec3};

/// 点と曲面・曲線の最近傍点・最短距離（Extremum & Distance）探索エンジン
pub struct ExtremumEngine;

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct PointCurveProjection {
    pub parameter: f64,
    pub closest_point: Point3,
    pub distance: f64,
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct PointSurfaceProjection {
    pub u: f64,
    pub v: f64,
    pub closest_point: Point3,
    pub distance: f64,
}

impl ExtremumEngine {
    /// 3次元点から NURBS 曲線への最短距離・最近傍パラメータ t をニュートン法で探索
    pub fn point_to_curve(
        point: Point3,
        curve: &NurbsCurve3,
        max_iterations: usize,
        tolerance: f64,
    ) -> Result<PointCurveProjection, String> {
        let (t_min, t_max) = curve.param_range();

        // 1. 粗いサンプリングで最良の初期値を探索
        let num_samples = 32;
        let mut best_t = t_min;
        let mut min_dist_sq = f64::INFINITY;

        for i in 0..=num_samples {
            let t = t_min + (i as f64 / num_samples as f64) * (t_max - t_min);
            let pt = curve.evaluate(t);
            let dist_sq = (pt - point).norm_squared();
            if dist_sq < min_dist_sq {
                min_dist_sq = dist_sq;
                best_t = t;
            }
        }

        // 2. ニュートン・ラフソン法でパラメータ t を精密反復改善
        // 目的関数 f(t) = (C(t) - P) . C'(t) = 0
        let mut current_t = best_t;

        for _ in 0..max_iterations {
            let ders = curve.evaluate_derivatives(current_t, 2);
            let c_t = curve.evaluate(current_t);
            let diff = c_t - point;

            let c_prime = if ders.len() > 1 {
                ders[1]
            } else {
                Vec3::new(1.0, 0.0, 0.0)
            };
            let f = diff.dot(&c_prime);

            if f.abs() < tolerance {
                break;
            }

            let c_prime_prime = if ders.len() > 2 {
                ders[2]
            } else {
                Vec3::new(0.0, 0.0, 0.0)
            };
            let f_prime = c_prime.norm_squared() + diff.dot(&c_prime_prime);

            if f_prime.abs() < 1e-12 {
                break;
            }

            let delta_t = f / f_prime;
            current_t = (current_t - delta_t).clamp(t_min, t_max);

            if delta_t.abs() < tolerance {
                break;
            }
        }

        let closest_point = curve.evaluate(current_t);
        let distance = (closest_point - point).norm();

        Ok(PointCurveProjection {
            parameter: current_t,
            closest_point,
            distance,
        })
    }

    /// 3次元点から NURBS 曲面への最短距離・最近傍パラメータ (u, v) を2変数ニュートン法で探索
    pub fn point_to_surface(
        point: Point3,
        surface: &NurbsSurface3,
        max_iterations: usize,
        tolerance: f64,
    ) -> Result<PointSurfaceProjection, String> {
        let ((u_min, u_max), (v_min, v_max)) = surface.param_range();

        // 1. 粗いサンプリングで初期パラメータ (u, v) を決定
        let samples = 16;
        let mut best_u = u_min;
        let mut best_v = v_min;
        let mut min_dist_sq = f64::INFINITY;

        for i in 0..=samples {
            let u = u_min + (i as f64 / samples as f64) * (u_max - u_min);
            for j in 0..=samples {
                let v = v_min + (j as f64 / samples as f64) * (v_max - v_min);
                let pt = surface.evaluate(u, v);
                let dist_sq = (pt - point).norm_squared();
                if dist_sq < min_dist_sq {
                    min_dist_sq = dist_sq;
                    best_u = u;
                    best_v = v;
                }
            }
        }

        // 2. 2変数ニュートン・ラフソン法による精密収束
        let mut cur_u = best_u;
        let mut cur_v = best_v;

        for _ in 0..max_iterations {
            let (pt, su, sv) = surface.evaluate_derivatives_1st(cur_u, cur_v);
            let diff = pt - point;

            let f = diff.dot(&su);
            let g = diff.dot(&sv);

            if f.abs() < tolerance && g.abs() < tolerance {
                break;
            }

            let e = su.norm_squared();
            let f_coeff = su.dot(&sv);
            let g_coeff = sv.norm_squared();

            let det = e * g_coeff - f_coeff * f_coeff;
            if det.abs() < 1e-12 {
                break;
            }

            let delta_u = (f * g_coeff - g * f_coeff) / det;
            let delta_v = (g * e - f * f_coeff) / det;

            cur_u = (cur_u - delta_u).clamp(u_min, u_max);
            cur_v = (cur_v - delta_v).clamp(v_min, v_max);

            if delta_u.abs() < tolerance && delta_v.abs() < tolerance {
                break;
            }
        }

        let closest_point = surface.evaluate(cur_u, cur_v);
        let distance = (closest_point - point).norm();

        Ok(PointSurfaceProjection {
            u: cur_u,
            v: cur_v,
            closest_point,
            distance,
        })
    }
}
