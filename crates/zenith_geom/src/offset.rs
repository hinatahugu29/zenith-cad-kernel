use crate::nurbs_curve::NurbsCurve3;
use crate::nurbs_surface::NurbsSurface3;
use zenith_math::Vec3;

/// 曲面・曲線のオフセット（Offset Surface & Offset Curve）エンジン
pub struct OffsetEngine;

impl OffsetEngine {
    /// 自由曲面 NURBS を法線方向に距離 distance だけオフセットした NURBS 曲面を生成
    /// S_off(u, v) = S(u, v) + distance * N(u, v)
    pub fn offset_surface(surface: &NurbsSurface3, distance: f64) -> Result<NurbsSurface3, String> {
        if distance.abs() < 1e-9 {
            return Ok(surface.clone());
        }

        let num_u = surface.control_points.len();
        let num_v = if num_u > 0 {
            surface.control_points[0].len()
        } else {
            0
        };

        if num_u < 2 || num_v < 2 {
            return Err("Surface control points grid too small for offset".to_string());
        }

        let ((u_min, u_max), (v_min, v_max)) = surface.param_range();
        let mut new_cps = surface.control_points.clone();

        for (i, row) in new_cps.iter_mut().enumerate().take(num_u) {
            let u_t = i as f64 / (num_u - 1) as f64;
            let u = u_min + u_t * (u_max - u_min);

            for (j, cp) in row.iter_mut().enumerate().take(num_v) {
                let v_t = j as f64 / (num_v - 1) as f64;
                let v = v_min + v_t * (v_max - v_min);

                let normal = surface
                    .normal(u, v)
                    .unwrap_or_else(|| Vec3::new(0.0, 0.0, 1.0));
                let offset_vec = normal * distance;

                cp.point += offset_vec;
            }
        }

        NurbsSurface3::new(
            surface.degree_u,
            surface.degree_v,
            new_cps,
            surface.knots_u.clone(),
            surface.knots_v.clone(),
        )
    }

    /// 3D NURBS 曲線を指定法線ベクトル方向 normal_dir に距離 distance だけオフセットした NURBS 曲線を生成
    pub fn offset_curve(
        curve: &NurbsCurve3,
        normal_dir: Vec3,
        distance: f64,
    ) -> Result<NurbsCurve3, String> {
        if distance.abs() < 1e-9 {
            return Ok(curve.clone());
        }

        let n = normal_dir.normalize();
        let offset_vec = n * distance;
        let mut new_cps = curve.control_points.clone();

        for cp in &mut new_cps {
            cp.point += offset_vec;
        }

        NurbsCurve3::new(curve.degree, new_cps, curve.knots.clone())
    }
}
