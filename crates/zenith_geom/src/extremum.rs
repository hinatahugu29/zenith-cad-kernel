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
        crate::work_counter::count_point_surface_projection();
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

        // 粗い格子のまま渡すと、ニュートン法が動けなかったときに格子間隔
        // そのものが答えとして残る。半径10の球を16分割した格子では、
        // 赤道上の点が 1.8 ずれたまま通っていた。当たりの周りを数回詰めてから
        // 渡せば、出発点は最初から近い。
        let closed = SurfaceClosure::of(surface);
        let mut cell_u = (u_max - u_min) / samples as f64;
        let mut cell_v = (v_max - v_min) / samples as f64;
        for _ in 0..8 {
            cell_u *= 0.5;
            cell_v *= 0.5;
            for i in -1..=1 {
                for j in -1..=1 {
                    if i == 0 && j == 0 {
                        continue;
                    }
                    let u = closed.settle_u(best_u + cell_u * i as f64, u_min, u_max);
                    let v = closed.settle_v(best_v + cell_v * j as f64, v_min, v_max);
                    let dist_sq = (surface.evaluate(u, v) - point).norm_squared();
                    if dist_sq < min_dist_sq {
                        min_dist_sq = dist_sq;
                        best_u = u;
                        best_v = v;
                    }
                }
            }
        }

        Self::refine_surface_projection(point, surface, best_u, best_v, min_dist_sq, max_iterations, tolerance)
    }

    /// 最近傍点の探索を、与えられた (u, v) から始める。
    ///
    /// 曲線に沿って少しずつ進むときは、隣の結果がそのまま良い出発点になる。
    /// 毎回そこら中を粗くサンプリングし直す必要は無い。
    pub fn point_to_surface_seeded(
        point: Point3,
        surface: &NurbsSurface3,
        seed_u: f64,
        seed_v: f64,
        max_iterations: usize,
        tolerance: f64,
    ) -> Result<PointSurfaceProjection, String> {
        crate::work_counter::count_point_surface_projection();
        let ((u_min, u_max), (v_min, v_max)) = surface.param_range();
        let start_u = seed_u.clamp(u_min, u_max);
        let start_v = seed_v.clamp(v_min, v_max);
        let start_dist_sq = (surface.evaluate(start_u, start_v) - point).norm_squared();
        Self::refine_surface_projection(
            point,
            surface,
            start_u,
            start_v,
            start_dist_sq,
            max_iterations,
            tolerance,
        )
    }

    fn refine_surface_projection(
        point: Point3,
        surface: &NurbsSurface3,
        start_u: f64,
        start_v: f64,
        start_dist_sq: f64,
        max_iterations: usize,
        tolerance: f64,
    ) -> Result<PointSurfaceProjection, String> {
        let ((u_min, u_max), (v_min, v_max)) = surface.param_range();
        let (best_u, best_v, min_dist_sq) = (start_u, start_v, start_dist_sq);
        let closed = SurfaceClosure::of(surface);

        // 2. 2変数ニュートン・ラフソン法による精密収束
        //
        // ニュートン法は距離の停留点に向かうだけで、それが最小とは限らない。
        // 素直に反復して最後の位置を返していたときは、退化行（球の極など）で
        // ヤコビアンが特異になった時点で打ち切り、極そのものを答えとして
        // 返していた。半径10の球で 0.446 ずれる。
        //
        // そこで、これまでで最も近かった位置を常に持ち、**それより悪い位置は
        // 決して返さない**。歩幅は距離が縮むまで半分にする。縮む向きが無ければ
        // そこが極小なので止める。こうすると、結果は粗サンプリングの当たり以上
        // であることが保証される。
        let mut cur_u = best_u;
        let mut cur_v = best_v;
        let mut best_dist_sq = min_dist_sq;

        let distance_sq_at = |u: f64, v: f64| (surface.evaluate(u, v) - point).norm_squared();

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
            // 特異なら降下方向へ退く。極の上でも進めるようにするため、
            // ここで打ち切らない。
            let (step_u, step_v) = if det.abs() < 1e-12 {
                let scale = (e + g_coeff).max(1e-12);
                (f / scale, g / scale)
            } else {
                (
                    (f * g_coeff - g * f_coeff) / det,
                    (g * e - f * f_coeff) / det,
                )
            };

            if !step_u.is_finite() || !step_v.is_finite() {
                break;
            }

            // 距離が縮む歩幅が見つかるまで半分にする。
            let mut damping = 1.0;
            let mut moved = false;
            for _ in 0..24 {
                let next_u = closed.settle_u(cur_u - step_u * damping, u_min, u_max);
                let next_v = closed.settle_v(cur_v - step_v * damping, v_min, v_max);
                let next = distance_sq_at(next_u, next_v);
                if next < best_dist_sq {
                    let settled = (next_u - cur_u).abs() < tolerance
                        && (next_v - cur_v).abs() < tolerance;
                    cur_u = next_u;
                    cur_v = next_v;
                    best_dist_sq = next;
                    moved = true;
                    if settled {
                        return Ok(Self::surface_projection(point, surface, cur_u, cur_v));
                    }
                    break;
                }
                damping *= 0.5;
            }

            if !moved {
                break;
            }
        }

        Ok(Self::surface_projection(point, surface, cur_u, cur_v))
    }

    fn surface_projection(
        point: Point3,
        surface: &NurbsSurface3,
        u: f64,
        v: f64,
    ) -> PointSurfaceProjection {
        let closest_point = surface.evaluate(u, v);
        PointSurfaceProjection {
            u,
            v,
            closest_point,
            distance: (closest_point - point).norm(),
        }
    }
}

/// Which way round a surface joins up with itself.
///
/// A parameter that runs off one end of a closed direction has not left the
/// surface, it has come back at the other end. Clamping it instead pins the
/// search against the seam: a point just short of longitude zero on a sphere
/// would be answered with the seam itself, 1.83 out on a radius of ten, and no
/// amount of iterating could cross back because every step was clamped away.
struct SurfaceClosure {
    in_u: bool,
    in_v: bool,
}

impl SurfaceClosure {
    fn of(surface: &NurbsSurface3) -> Self {
        let ((u_min, u_max), (v_min, v_max)) = surface.param_range();
        // 端どうしが同じ点を指しているかを、何本かの線で確かめる。
        let extent = (surface.evaluate(u_max, v_max) - surface.evaluate(u_min, v_min))
            .norm()
            .max(1.0);
        let limit = extent * 1e-9;

        let mut in_u = u_max > u_min;
        let mut in_v = v_max > v_min;
        for step in 0..=4 {
            let fraction = step as f64 / 4.0;
            let v = v_min + (v_max - v_min) * fraction;
            if in_u && (surface.evaluate(u_min, v) - surface.evaluate(u_max, v)).norm() > limit {
                in_u = false;
            }
            let u = u_min + (u_max - u_min) * fraction;
            if in_v && (surface.evaluate(u, v_min) - surface.evaluate(u, v_max)).norm() > limit {
                in_v = false;
            }
        }

        Self { in_u, in_v }
    }

    fn settle_u(&self, value: f64, min: f64, max: f64) -> f64 {
        Self::settle(value, min, max, self.in_u)
    }

    fn settle_v(&self, value: f64, min: f64, max: f64) -> f64 {
        Self::settle(value, min, max, self.in_v)
    }

    fn settle(value: f64, min: f64, max: f64, wraps: bool) -> f64 {
        if !wraps || !(max > min) {
            return value.clamp(min, max);
        }
        let span = max - min;
        let wrapped = min + (value - min).rem_euclid(span);
        if wrapped.is_finite() {
            wrapped
        } else {
            value.clamp(min, max)
        }
    }
}
