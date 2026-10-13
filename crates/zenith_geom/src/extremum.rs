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

        // **走査と詰めを別々に測ります**（4-735。`ZENITH_BOOLEAN_TIME` のみ、
        // 既定オフ）。
        //
        // 4-734 で**この関数が 285,225 回、1 回 約 67 µs**だと測りました。
        // **走査だけで 32 点の曲線の評価**が走ります——**913 万回**です。
        // **走査と詰めのどちらが重いかは、割るまで分かりません。**
        let cost_on = point_curve_cost::on();
        let scan_began = cost_on.then(std::time::Instant::now);
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
        if let Some(began) = scan_began {
            point_curve_cost::add_scan(began.elapsed().as_secs_f64());
        }
        let newton_began = cost_on.then(std::time::Instant::now);
        let mut newton_steps = 0u64;
        // **上限で打ち切られたか、break で抜けたか**（4-736）。
        //
        // **`current_t` は範囲内に clamp されます。** **最近点が稜の端に
        // あるとき、ニュートンは外へ押し出して clamp され、`Δt` が
        // 小さくなりません**——**上限まで回り切る**はずです。
        // **打ち切りを 5 桁緩めても歩数が 3.5% しか減らなかった**のは、
        // それで説明が付きます（4-735）。**数えて確かめます。**
        let mut broke_out = false;
        let mut current_t = best_t;

        for _ in 0..max_iterations {
            newton_steps += 1;
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
                broke_out = true;
                break;
            }

            let c_prime_prime = if ders.len() > 2 {
                ders[2]
            } else {
                Vec3::new(0.0, 0.0, 0.0)
            };
            let f_prime = c_prime.norm_squared() + diff.dot(&c_prime_prime);

            if f_prime.abs() < 1e-12 {
                broke_out = true;
                break;
            }

            let delta_t = f / f_prime;
            let moved_from = current_t;
            current_t = (current_t - delta_t).clamp(t_min, t_max);
            // **動かなくなったら、そこが答えです**（4-737 の測り。
            // `ZENITH_CURVE_CLAMP_STOP`。**4-750 で既定オンに昇格**）。
            //
            // 4-736 の実測: **7 割の射影が 64 歩を使い切って抜けています。**
            // **最近点が稜の端にあるとき、ニュートンは外へ押し出し、clamp され、
            // `current_t` は動きません**——**`delta_t` は大きいまま**なので、
            // **下の break が働きません。**
            //
            // **返すのは clamp 後の `current_t`** なので、**動かないなら 64 歩
            // まわしても同じ値**です。**ここで止めても同じ値を返します**
            // ——**「同じ答えを安く出す」口**です（4-732 の分け方）。
            if clamp_stop_enabled() && current_t == moved_from {
                broke_out = true;
                break;
            }

            if delta_t.abs() < tolerance {
                broke_out = true;
                break;
            }
        }

        let closest_point = curve.evaluate(current_t);
        let distance = (closest_point - point).norm();

        if let Some(began) = newton_began {
            point_curve_cost::add_newton(began.elapsed().as_secs_f64(), newton_steps);
            if !broke_out {
                point_curve_cost::add_capped();
            }
        }
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
        crate::work_counter::count_point_surface_coarse_search();
        let ((u_min, u_max), (v_min, v_max)) = surface.param_range();

        // 1. 粗いサンプリングで初期パラメータ (u, v) を決定
        //
        // **ここの数は動かさないでください**（4-165 で測って戻しました）。
        // 17x17 = 289 回は高くつくので、格子を粗くして下の詰めを増やす、を
        // 試しました。**仕事は 19% 減りましたが、`contact_placement_probe`
        // が 81 ok → 78 ok になりました。**
        //
        // **粗さのせいではありません。** 格子を 16 のままにして詰めだけ
        // 2段増やしても 78 になります。**詰めを増やすほうが悪くなります。**
        // 出発点がわずかに動くと、そのあとのニュートンが別の解へ降りて、
        // 分類が変わるためです。
        //
        // **分類が射影の答えの細部に依存している**、というのが本当の所見です。
        // そこを直さずに値段だけ下げると、通っていた演算が断られます。
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

        // **当たりが極（退化した行）だったら、そこからは降りられません。**
        //
        // 極では u をいくら動かしても 3D の点が変わらないので、上の詰めは
        // 「u を選び直す」ことができません。v を1目盛動かすと、**当たった
        // ときの u の経線**に沿って動くだけで、たいてい遠くなります。そこで
        // 8回とも改善が無く、**極そのものが答えとして残ります**。
        //
        // 実測: 半径10の球を (20,10,10) に置き、球面上の点 (20, 9.5542, 0.0099)
        // を射影すると **0.446 ずれた極**が返っていました（点は面の上にあるので
        // 正しい答えは 0）。原点に置いた同じ球では起きません——粗格子の当たりが
        // たまたま極にならないからです。**置き方で答えが変わっていました。**
        //
        // ここに落ちたら、**隣の v で u を全域から選び直します**。
        // 経線さえ決まれば、あとはニュートンが降ります。
        // **近いほうを選んではいけません。** 極のすぐ隣にある点では、極そのもの
        // のほうが「隣の行」より近いことがあります（実測: 極から 0.14、隣の行は
        // 0.33）。近さで選ぶと極が残り、また降りられません。
        //
        // **両方から降ろして、着いた先で選びます。** 降りる先が本物の最近点なら、
        // 出発点がどちらでも同じところへ行きます。
        let from_grid = Self::refine_surface_projection(
            point,
            surface,
            best_u,
            best_v,
            min_dist_sq,
            max_iterations,
            tolerance,
        );
        let Some((seed_u, seed_v, seed_dist_sq)) =
            Self::escape_degenerate_seed(point, surface, best_u, best_v)
        else {
            return from_grid;
        };

        let from_meridian = Self::refine_surface_projection(
            point,
            surface,
            seed_u,
            seed_v,
            seed_dist_sq,
            max_iterations,
            tolerance,
        );
        match (from_grid, from_meridian) {
            (Ok(grid), Ok(meridian)) => Ok(if meridian.distance < grid.distance {
                meridian
            } else {
                grid
            }),
            (Ok(only), Err(_)) | (Err(_), Ok(only)) => Ok(only),
            (Err(err), Err(_)) => Err(err),
        }
    }

    /// 当たりが退化した位置（極）なら、**隣の行で経線を選び直した種**を返す。
    ///
    /// 退化していないなら `None`。**近いかどうかでは選びません**——ここは
    /// 「どの経線を降りるか」を決めるためだけの種で、近さは降りた先で比べます。
    fn escape_degenerate_seed(
        point: Point3,
        surface: &NurbsSurface3,
        best_u: f64,
        best_v: f64,
    ) -> Option<(f64, f64, f64)> {
        let ((u_min, u_max), (v_min, v_max)) = surface.param_range();
        let (_, su, sv) = surface.evaluate_derivatives_1st(best_u, best_v);

        // 退化の判定は、もう片方の向きとの比で見ます。大きさの単位に依りません。
        let u_collapsed = su.norm() <= sv.norm() * 1e-6;
        let v_collapsed = sv.norm() <= su.norm() * 1e-6;
        if !u_collapsed && !v_collapsed {
            return None;
        }

        // 潰れているほうを全域から選び直し、潰れていないほうは1目盛だけ動かす。
        // 動かす向きは、範囲の内側へ。
        let samples = 32;
        let mut found: Option<(f64, f64, f64)> = None;
        let consider = |u: f64, v: f64, found: &mut Option<(f64, f64, f64)>| {
            let distance_sq = (surface.evaluate(u, v) - point).norm_squared();
            if distance_sq < found.map(|(_, _, d)| d).unwrap_or(f64::INFINITY) {
                *found = Some((u, v, distance_sq));
            }
        };

        if u_collapsed {
            let step = (v_max - v_min) / samples as f64;
            let toward_inside = if (best_v - v_min).abs() <= (v_max - best_v).abs() {
                step
            } else {
                -step
            };
            let v = (best_v + toward_inside).clamp(v_min, v_max);
            for index in 0..=samples {
                let u = u_min + (u_max - u_min) * index as f64 / samples as f64;
                consider(u, v, &mut found);
            }
        }
        if v_collapsed {
            let step = (u_max - u_min) / samples as f64;
            let toward_inside = if (best_u - u_min).abs() <= (u_max - best_u).abs() {
                step
            } else {
                -step
            };
            let u = (best_u + toward_inside).clamp(u_min, u_max);
            for index in 0..=samples {
                let v = v_min + (v_max - v_min) * index as f64 / samples as f64;
                consider(u, v, &mut found);
            }
        }

        found
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
            crate::work_counter::count_projection_newton_iteration();
            let (pt, su, sv) = surface.evaluate_derivatives_1st(cur_u, cur_v);
            let diff = pt - point;

            let f = diff.dot(&su);
            let g = diff.dot(&sv);

            // **判定の量には次元があります**（4-259）。
            //
            // `f = 残差 · ∂S/∂u` なので、単位は「長さ × 長さ/パラメータ」です。
            // これを**絶対の 1e-7** と比べると、**到達できる残差は
            // `tolerance / |∂S/∂u|`**——**パッチが小さいほど大きく**なります。
            //
            // 実測（トーラスの四半パッチ）:
            //
            // | 桁 | `|∂S/∂u|` | 予想される残差 | 実測 |
            // | ---: | ---: | ---: | ---: |
            // | 1 | 6.28 | 1.6e-8 | **1.741e-8** |
            // | 0.01 | 0.0628 | 1.6e-6 | **1.546e-6** |
            //
            // **点は曲面の上にある**（厳密な距離 1.830e-8）のに、射影は
            // 1.546e-6 離れた点を返していました。これが H6 の破れの出どころ
            // です（4-258）。
            //
            // **微分の大きさで割って、長さの次元に戻します。** ただし
            // `min(1.0)` を掛けてあるので、**大きいパッチでは判定が変わり
            // ません**——いま通っているものの要求は動きません。
            let scale_u = su.norm().min(1.0);
            let scale_v = sv.norm().min(1.0);
            if f.abs() < tolerance * scale_u && g.abs() < tolerance * scale_v {
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
                crate::work_counter::count_projection_damping_trial();
                let next_u = closed.settle_u(cur_u - step_u * damping, u_min, u_max);
                let next_v = closed.settle_v(cur_v - step_v * damping, v_min, v_max);
                let next = distance_sq_at(next_u, next_v);
                if next < best_dist_sq {
                    let settled =
                        (next_u - cur_u).abs() < tolerance && (next_v - cur_v).abs() < tolerance;
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

/// **点を曲線へ落とす 1 回の内訳**（4-735。`ZENITH_BOOLEAN_TIME` のみ、既定オフ）。
///
/// 4-734 で**285,225 回、1 回 約 67 µs**だと測りました。
/// **走査（32 点）と詰め（ニュートン、最大 64 回）**に割ります。
/// **clamp で動かなくなったら止める口**（4-737 の測り。
/// `ZENITH_CURVE_CLAMP_STOP`）。**4-750 で既定オンに昇格**——
/// `=0` で旧既定（64 歩を使い切るまで止めない）に戻せます。
fn clamp_stop_enabled() -> bool {
    std::env::var("ZENITH_CURVE_CLAMP_STOP")
        .map(|value| value != "0")
        .unwrap_or(true)
}

pub mod point_curve_cost {
    use std::cell::Cell;

    thread_local! {
        static SCAN_SECS: Cell<f64> = const { Cell::new(0.0) };
        static NEWTON_SECS: Cell<f64> = const { Cell::new(0.0) };
        static CALLS: Cell<u64> = const { Cell::new(0) };
        static STEPS: Cell<u64> = const { Cell::new(0) };
        static CAPPED: Cell<u64> = const { Cell::new(0) };
    }

    pub fn on() -> bool {
        std::env::var_os("ZENITH_BOOLEAN_TIME").is_some()
    }

    pub fn add_scan(secs: f64) {
        SCAN_SECS.with(|c| c.set(c.get() + secs));
        CALLS.with(|c| c.set(c.get() + 1));
    }

    /// **上限まで回り切った回数**（4-736）。
    pub fn add_capped() {
        CAPPED.with(|c| c.set(c.get() + 1));
    }

    pub fn add_newton(secs: f64, steps: u64) {
        NEWTON_SECS.with(|c| c.set(c.get() + secs));
        STEPS.with(|c| c.set(c.get() + steps));
    }

    /// 溜まったものを出して、ゼロに戻します。
    pub fn take() -> (f64, f64, u64, u64, u64) {
        (
            SCAN_SECS.with(|c| c.replace(0.0)),
            NEWTON_SECS.with(|c| c.replace(0.0)),
            CALLS.with(|c| c.replace(0)),
            STEPS.with(|c| c.replace(0)),
            CAPPED.with(|c| c.replace(0)),
        )
    }
}
