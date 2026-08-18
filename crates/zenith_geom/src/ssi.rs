use crate::nurbs_surface::NurbsSurface3;
use serde::{Deserialize, Serialize};
use zenith_math::{Point3, Tolerance, Vec3Ext};

/// 2つの曲面の交差点データ（3D座標 + 各曲面上のUVパラメータ）
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct SurfaceIntersectionPoint {
    pub point: Point3,
    pub uv1: (f64, f64),
    pub uv2: (f64, f64),
}

/// 曲面同士の幾何交差（Surface-Surface Intersection, SSI）エンジン
pub struct SurfaceIntersection;

impl SurfaceIntersection {
    /// ニュートン・ラフソン法により、初期推定値から厳密な交差点 (u, v, s, t) を収束計算
    pub fn refine_intersection_point(
        s1: &NurbsSurface3,
        s2: &NurbsSurface3,
        mut u: f64,
        mut v: f64,
        mut s: f64,
        mut t: f64,
        tol: &Tolerance,
    ) -> Option<SurfaceIntersectionPoint> {
        let max_iters = 25;
        let ((u_min, u_max), (v_min, v_max)) = s1.param_range();
        let ((s_min, s_max), (t_min, t_max)) = s2.param_range();

        for _ in 0..max_iters {
            let (p1, du1, dv1) = s1.evaluate_derivatives_1st(u, v);
            let (p2, du2, dv2) = s2.evaluate_derivatives_1st(s, t);

            let res = p1 - p2; // 残差ベクトル (3D)
            if res.norm() <= tol.linear {
                let mid_pt = Point3::from((p1.coords + p2.coords) * 0.5);
                return Some(SurfaceIntersectionPoint {
                    point: mid_pt,
                    uv1: (u, v),
                    uv2: (s, t),
                });
            }

            // 法線ベクトル
            let n1 = du1.cross(&dv1).try_normalize_safe(1e-12)?;
            let n2 = du2.cross(&dv2).try_normalize_safe(1e-12)?;

            // 探索方向の接線ベクトル T = n1 x n2
            let tangent = n1.cross(&n2);
            if tangent.norm_squared() < 1e-12 {
                // 2曲面が平行・接している特異ケース
                break;
            }

            // 射影逆問題: ニュートンステップ
            // 簡略化された局所直交射影ステップ
            let step_u = -res.dot(&du1) / (du1.norm_squared() + 1e-10);
            let step_v = -res.dot(&dv1) / (dv1.norm_squared() + 1e-10);
            let step_s = res.dot(&du2) / (du2.norm_squared() + 1e-10);
            let step_t = res.dot(&dv2) / (dv2.norm_squared() + 1e-10);

            u = (u + step_u * 0.5).clamp(u_min, u_max);
            v = (v + step_v * 0.5).clamp(v_min, v_max);
            s = (s + step_s * 0.5).clamp(s_min, s_max);
            t = (t + step_t * 0.5).clamp(t_min, t_max);
        }

        let p1 = s1.evaluate(u, v);
        let p2 = s2.evaluate(s, t);
        if (p1 - p2).norm() <= tol.linear * 10.0 {
            Some(SurfaceIntersectionPoint {
                point: Point3::from((p1.coords + p2.coords) * 0.5),
                uv1: (u, v),
                uv2: (s, t),
            })
        } else {
            None
        }
    }

    /// 2つのNURBS曲面の交差点群（交差曲線サンプリング列）を計算
    pub fn intersect_surfaces(
        s1: &NurbsSurface3,
        s2: &NurbsSurface3,
        tol: &Tolerance,
    ) -> Vec<SurfaceIntersectionPoint> {
        let mut results = Vec::new();
        let grid_res = 10;

        let ((u_min, u_max), (v_min, v_max)) = s1.param_range();
        let ((s_min, s_max), (t_min, t_max)) = s2.param_range();

        // 粗グリッドサンプリングによるシード探索
        for i in 0..=grid_res {
            let u = u_min + (u_max - u_min) * (i as f64 / grid_res as f64);
            for j in 0..=grid_res {
                let v = v_min + (v_max - v_min) * (j as f64 / grid_res as f64);
                let p1 = s1.evaluate(u, v);

                // s2上の最近傍シード点探索
                let mut best_dist = f64::INFINITY;
                let mut best_st = (0.5, 0.5);

                for k in 0..=grid_res {
                    let s = s_min + (s_max - s_min) * (k as f64 / grid_res as f64);
                    for l in 0..=grid_res {
                        let t = t_min + (t_max - t_min) * (l as f64 / grid_res as f64);
                        let p2 = s2.evaluate(s, t);
                        let d = (p1 - p2).norm();
                        if d < best_dist {
                            best_dist = d;
                            best_st = (s, t);
                        }
                    }
                }

                // 初期距離が十分近い場合にニュートン収束を実行
                if best_dist < (u_max - u_min).max(v_max - v_min) * 0.5 {
                    if let Some(ipt) =
                        Self::refine_intersection_point(s1, s2, u, v, best_st.0, best_st.1, tol)
                    {
                        // 重複点除外
                        let is_duplicate =
                            results.iter().any(|existing: &SurfaceIntersectionPoint| {
                                (existing.point - ipt.point).norm() < tol.linear * 5.0
                            });
                        if !is_duplicate {
                            results.push(ipt);
                        }
                    }
                }
            }
        }

        results
    }
}
