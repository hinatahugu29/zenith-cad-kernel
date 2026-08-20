//! 曲面同士の交線を、点を1つずつ確かめながら辿る。
//!
//! # なぜ作り直すか
//!
//! `ssi::SurfaceIntersection::refine_intersection_point` は、残差を各接ベクトルへ
//! 射影して半分ずつ動かす反復である。ところが `p1(u,v) = p2(s,t)` は**3式4未知数**で、
//! 解は点ではなく1次元の族——交線そのもの——になる。拘束を置かずに反復すると、
//! どこへ寄るかは初期値まかせで、収束したように見えても交線上の別の点へ
//! 滑っている。曲線を辿るには**4つ目の式**が要る。
//!
//! ここでは進む向き `T = n1 x n2` を法線とする平面を4つ目の式に置く。
//!
//! ```text
//! F1..F3 :  p1(u,v) - p2(s,t) = 0
//! F4     : (p1(u,v) - anchor) . T = step
//! ```
//!
//! 4式4未知数なのでヤコビ行列は正方で、ニュートン法が2次で収束する。
//! `step = 0` なら「いまの位置から動かずに交線へ落とす」、`step > 0` なら
//! 「交線に沿って `step` だけ進む」になる。同じ式で両方が書ける。
//!
//! # 何を測るか
//!
//! 辿った点が**両方の曲面の上にある**ことを、点ごとに測って持ち帰る。
//! 交線らしい点列が出たことは、それが交線である証拠にならない。

use crate::extremum::ExtremumEngine;
use crate::nurbs_surface::NurbsSurface3;
use crate::ssi::SurfaceIntersectionPoint;
use zenith_math::{Point3, Tolerance, Vec3, Vec3Ext};

/// 辿った交線と、その確からしさ。
#[derive(Debug, Clone, PartialEq)]
pub struct MarchedIntersection {
    /// 交線上の点列。進んだ順に並ぶ。
    pub points: Vec<SurfaceIntersectionPoint>,
    /// 端が始点に戻ったか（閉じた交線か）。
    pub closed: bool,
    /// どの点も、2つの曲面からこれ以上は離れていない。
    pub worst_off_surface: f64,
    /// 端がパラメータ領域の縁で止まったか。
    pub stopped_at_boundary: bool,
    /// 端が接点の手前で止まったか。
    ///
    /// 2つの法線が平行になるところでは、「両方の曲面の上にある」が位置を
    /// 決めない。曲面同士がその近傍で寄り添うので、残差を 1e-11 まで詰めても
    /// 交線から 3e-5 離れた点が通ってしまう（等半径の直交円柱で実測）。
    /// 進む向き自体も外積なので定まらない。手前で止めて、そう言う。
    pub stopped_at_tangency: bool,
}

/// 交線を辿る係。
pub struct IntersectionMarcher;

/// 2つの法線のなす角の正弦がこれを下回ったら、接していると見なして止める。
///
/// 値は測って決めた。等半径の直交円柱では、接点まで進むと交線から 2.99e-5
/// 離れた点が残差 2.24e-11 で「収束」する。手前で止めれば、残る点はすべて
/// 交線の上に 1e-12 で乗る。
pub const TANGENCY_SINE_LIMIT: f64 = 1e-4;

impl IntersectionMarcher {
    /// `s1` の `(u, v)` あたりから交線に乗り、両方向へ辿る。
    ///
    /// `step` は1歩の長さ（3D の距離）。`max_points` は打ち切り。
    pub fn march(
        s1: &NurbsSurface3,
        s2: &NurbsSurface3,
        seed_u: f64,
        seed_v: f64,
        step: f64,
        max_points: usize,
        tol: &Tolerance,
    ) -> Option<MarchedIntersection> {
        let start = Self::land_on_curve(s1, s2, seed_u, seed_v, tol)?;

        let mut forward = Self::run(s1, s2, start, step, max_points, tol);
        let closed = forward.closed;
        if closed {
            let worst = Self::worst_distance(s1, s2, &forward.points);
            return Some(MarchedIntersection {
                points: forward.points,
                closed: true,
                worst_off_surface: worst,
                stopped_at_boundary: false,
                stopped_at_tangency: false,
            });
        }

        // 閉じなかったら、反対向きにも辿って前に継ぐ。
        let backward = Self::run(s1, s2, start, -step, max_points, tol);
        let mut points = backward.points;
        points.reverse();
        points.pop(); // 始点が二重にならないように
        points.append(&mut forward.points);

        let worst = Self::worst_distance(s1, s2, &points);
        Some(MarchedIntersection {
            points,
            closed: false,
            worst_off_surface: worst,
            stopped_at_boundary: forward.hit_boundary || backward.hit_boundary,
            stopped_at_tangency: forward.hit_tangency || backward.hit_tangency,
        })
    }

    /// 交線に近く、かつ接していない種を探して辿る。
    ///
    /// 種は自分で選ぶ。渡された `(u, v)` から落とすと、そこから見て交線の
    /// どこへ落ちるかは拘束平面まかせで、実際に接点へ落ちる配置がある
    /// （等半径の直交円柱で、四半パッチの真ん中から落とすとそうなる）。
    /// `s1` の格子を走らせ、`s2` までの距離が最小で、法線が平行でない点を選ぶ。
    pub fn march_from_best_seed(
        s1: &NurbsSurface3,
        s2: &NurbsSurface3,
        grid: usize,
        step: f64,
        max_points: usize,
        tol: &Tolerance,
    ) -> Option<MarchedIntersection> {
        let mut best: Option<MarchedIntersection> = None;
        for (seed_u, seed_v) in Self::find_seeds(s1, s2, grid, 8) {
            let Some(curve) = Self::march(s1, s2, seed_u, seed_v, step, max_points, tol) else {
                continue;
            };
            if curve.closed {
                return Some(curve);
            }
            // 長く辿れたほうを採る。種が交線の端に近いと数歩で終わる。
            if best
                .as_ref()
                .map(|found| curve.points.len() > found.points.len())
                .unwrap_or(true)
            {
                best = Some(curve);
            }
        }
        best
    }

    /// 交線に乗りそうな種を、近い順に集める。
    ///
    /// 1つに絞ろうとしない。近さで選ぶと、2面が触れている配置では接点が
    /// いちばん近く、そこから始めても進む向きが定まらない。条件の良さで
    /// 選ぶと、交線から遠い点を掴む。**どちらの当て方も外したので、
    /// 候補を並べて順に試し、結果を測って採ることにした。**
    ///
    /// 格子の各点から最近傍射影を回すと、1点あたり粗サンプリングとニュートンが
    /// 走って高くつく（ブーリアンで面の組ごとに呼ぶと 45 ケースの走査が
    /// 4分47秒になった）。両方の格子を並べて近い組を選び、そこだけ詰める。
    pub fn find_seeds(
        s1: &NurbsSurface3,
        s2: &NurbsSurface3,
        grid: usize,
        limit: usize,
    ) -> Vec<(f64, f64)> {
        let steps = grid.max(4);
        let ((u_min, u_max), (v_min, v_max)) = s1.param_range();
        let ((s_min, s_max), (t_min, t_max)) = s2.param_range();

        let mut grid_a = Vec::with_capacity((steps + 1) * (steps + 1));
        for i in 0..=steps {
            let u = u_min + (u_max - u_min) * i as f64 / steps as f64;
            for j in 0..=steps {
                let v = v_min + (v_max - v_min) * j as f64 / steps as f64;
                grid_a.push((u, v, s1.evaluate(u, v)));
            }
        }
        let mut grid_b = Vec::with_capacity((steps + 1) * (steps + 1));
        for i in 0..=steps {
            let s = s_min + (s_max - s_min) * i as f64 / steps as f64;
            for j in 0..=steps {
                let t = t_min + (t_max - t_min) * j as f64 / steps as f64;
                grid_b.push((s, t, s2.evaluate(s, t)));
            }
        }

        let mut pairs: Vec<(f64, usize, usize)> = Vec::with_capacity(grid_a.len());
        for (index_a, (_, _, point_a)) in grid_a.iter().enumerate() {
            let mut best = (f64::INFINITY, 0usize);
            for (index_b, (_, _, point_b)) in grid_b.iter().enumerate() {
                let distance = (point_a - point_b).norm();
                if distance < best.0 {
                    best = (distance, index_b);
                }
            }
            pairs.push((best.0, index_a, best.1));
        }
        pairs.sort_by(|a, b| a.0.partial_cmp(&b.0).unwrap());

        // 触れてすらいない組は、ここで打ち切る。格子の目より遠いところに
        // しか近づかないなら、この2枚は交わっていない。ブーリアンは面の組
        // ごとにここを通るので、交わらない組で辿りに入らないことが効く。
        let mut spacing = 0.0f64;
        for index in 1..grid_a.len().min(steps + 2) {
            spacing = spacing.max((grid_a[index].2 - grid_a[index - 1].2).norm());
        }
        if pairs.first().map(|(distance, _, _)| *distance).unwrap_or(f64::INFINITY)
            > spacing * 2.0
        {
            return Vec::new();
        }

        let mut seeds = Vec::new();
        for (_, index_a, index_b) in pairs.iter().take(limit * 8) {
            if seeds.len() >= limit {
                break;
            }
            let (u, v, point) = grid_a[*index_a];
            let (s, t, _) = grid_b[*index_b];
            let Ok(projection) =
                ExtremumEngine::point_to_surface_seeded(point, s2, s, t, 64, 1e-13)
            else {
                continue;
            };
            let state = [u, v, projection.u, projection.v];
            let Some((_, sine)) = Self::tangent(s1, s2, &state) else {
                continue;
            };
            if sine < TANGENCY_SINE_LIMIT * 10.0 {
                continue;
            }
            seeds.push((u, v));
        }
        seeds
    }

    /// 種の位置から、動かずに交線へ落とす。
    ///
    /// `s1` の点を `s2` へ射影して種を作り、`step = 0` の拘束で落とす。
    pub fn land_on_curve(
        s1: &NurbsSurface3,
        s2: &NurbsSurface3,
        seed_u: f64,
        seed_v: f64,
        tol: &Tolerance,
    ) -> Option<[f64; 4]> {
        let point = s1.evaluate(seed_u, seed_v);
        let projection = ExtremumEngine::point_to_surface(point, s2, 64, 1e-13).ok()?;
        let mut state = [seed_u, seed_v, projection.u, projection.v];

        // 拘束の向きは、いまの位置での接線でよい。落とす間に多少ずれても、
        // 平面が交線と横断していれば解は1つに決まる。
        let (direction, sine) = Self::tangent(s1, s2, &state)?;
        if sine < TANGENCY_SINE_LIMIT {
            // 接点の上から始めても、交線のどこにいるのか決まらない。
            return None;
        }
        let anchor = s1.evaluate(state[0], state[1]);
        Self::newton(s1, s2, &mut state, anchor, direction, 0.0, tol)?;

        // 落ちた先でも確かめる。始める前が良くても、拘束平面がちょうど接点を
        // 選ぶ配置がある（等半径の直交円柱の四半パッチの真ん中がそれ）。
        let (_, settled_sine) = Self::tangent(s1, s2, &state)?;
        if settled_sine < TANGENCY_SINE_LIMIT {
            return None;
        }
        Some(state)
    }

    /// 4式4未知数のニュートン法。収束したら `Some(残差)`。
    fn newton(
        s1: &NurbsSurface3,
        s2: &NurbsSurface3,
        state: &mut [f64; 4],
        anchor: Point3,
        direction: Vec3,
        step: f64,
        tol: &Tolerance,
    ) -> Option<f64> {
        let ((u_min, u_max), (v_min, v_max)) = s1.param_range();
        let ((s_min, s_max), (t_min, t_max)) = s2.param_range();
        let limit = tol.linear.min(1e-10);

        for _ in 0..40 {
            let (p1, du1, dv1) = s1.evaluate_derivatives_1st(state[0], state[1]);
            let (p2, du2, dv2) = s2.evaluate_derivatives_1st(state[2], state[3]);

            let gap = p1 - p2;
            let along = (p1 - anchor).dot(&direction) - step;
            let residual = gap.norm().max(along.abs());
            if residual <= limit {
                return Some(residual);
            }

            let jacobian = nalgebra::Matrix4::new(
                du1.x,
                dv1.x,
                -du2.x,
                -dv2.x,
                du1.y,
                dv1.y,
                -du2.y,
                -dv2.y,
                du1.z,
                dv1.z,
                -du2.z,
                -dv2.z,
                du1.dot(&direction),
                dv1.dot(&direction),
                0.0,
                0.0,
            );
            let rhs = nalgebra::Vector4::new(-gap.x, -gap.y, -gap.z, -along);
            let delta = jacobian.lu().solve(&rhs)?;
            if !delta.iter().all(|value| value.is_finite()) {
                return None;
            }

            state[0] = (state[0] + delta[0]).clamp(u_min, u_max);
            state[1] = (state[1] + delta[1]).clamp(v_min, v_max);
            state[2] = (state[2] + delta[2]).clamp(s_min, s_max);
            state[3] = (state[3] + delta[3]).clamp(t_min, t_max);
        }

        let gap = s1.evaluate(state[0], state[1]) - s2.evaluate(state[2], state[3]);
        if gap.norm() <= tol.linear {
            Some(gap.norm())
        } else {
            None
        }
    }

    /// いまの位置から、進む向きの先にあるパラメータ領域の縁へ着地する。
    ///
    /// 8つの縁（2曲面 x 2方向 x 上下）を順に試し、いまより先に進んだものの
    /// うち**いちばん近い**ものを採る。遠いほうを採ると、交線を通り越した
    /// 位置に着いてしまう。
    fn land_on_nearest_bound(
        s1: &NurbsSurface3,
        s2: &NurbsSurface3,
        state: &[f64; 4],
        direction: Vec3,
        reach: f64,
        tol: &Tolerance,
    ) -> Option<[f64; 4]> {
        let ((u_min, u_max), (v_min, v_max)) = s1.param_range();
        let ((s_min, s_max), (t_min, t_max)) = s2.param_range();
        let bounds = [
            (u_min, u_max),
            (v_min, v_max),
            (s_min, s_max),
            (t_min, t_max),
        ];
        let here = s1.evaluate(state[0], state[1]);

        let mut best: Option<(f64, [f64; 4])> = None;
        for (index, (low, high)) in bounds.iter().enumerate() {
            for target in [*low, *high] {
                if (state[index] - target).abs() <= 1e-12 {
                    continue;
                }
                let mut trial = *state;
                if Self::newton_to_bound(s1, s2, &mut trial, index, target, tol).is_none() {
                    continue;
                }
                let landed = s1.evaluate(trial[0], trial[1]);
                let travel = (landed - here).dot(&direction);
                // 進む向きの先にあることを確かめる。後ろに着いたら、それは
                // いま来た道である。
                if travel <= tol.linear {
                    continue;
                }
                // **1歩ぶんより遠くへは着地しない。** 遠い縁を目がけると、
                // ニュートンは交線の別の場所にある解へ落ちることがある。
                // 式は満たされているので、そのままだと点列に飛びが混ざる。
                if (landed - here).norm() > reach {
                    continue;
                }
                if best.as_ref().map(|(d, _)| travel < *d).unwrap_or(true) {
                    best = Some((travel, trial));
                }
            }
        }

        best.map(|(_, trial)| trial)
    }

    /// 4つ目の式を「`which` 番目のパラメータが `value` に等しい」に差し替えた
    /// ニュートン法。パッチの縁ちょうどに着地させるために使う。
    fn newton_to_bound(
        s1: &NurbsSurface3,
        s2: &NurbsSurface3,
        state: &mut [f64; 4],
        which: usize,
        value: f64,
        tol: &Tolerance,
    ) -> Option<f64> {
        let ((u_min, u_max), (v_min, v_max)) = s1.param_range();
        let ((s_min, s_max), (t_min, t_max)) = s2.param_range();
        let limit = tol.linear.min(1e-10);
        state[which] = value;

        for _ in 0..40 {
            let (p1, du1, dv1) = s1.evaluate_derivatives_1st(state[0], state[1]);
            let (p2, du2, dv2) = s2.evaluate_derivatives_1st(state[2], state[3]);
            let gap = p1 - p2;
            let off = state[which] - value;
            if gap.norm().max(off.abs()) <= limit {
                return Some(gap.norm());
            }

            let mut row = [0.0f64; 4];
            row[which] = 1.0;
            let jacobian = nalgebra::Matrix4::new(
                du1.x, dv1.x, -du2.x, -dv2.x, du1.y, dv1.y, -du2.y, -dv2.y, du1.z, dv1.z, -du2.z,
                -dv2.z, row[0], row[1], row[2], row[3],
            );
            let rhs = nalgebra::Vector4::new(-gap.x, -gap.y, -gap.z, -off);
            let delta = jacobian.lu().solve(&rhs)?;
            if !delta.iter().all(|value| value.is_finite()) {
                return None;
            }
            state[0] = (state[0] + delta[0]).clamp(u_min, u_max);
            state[1] = (state[1] + delta[1]).clamp(v_min, v_max);
            state[2] = (state[2] + delta[2]).clamp(s_min, s_max);
            state[3] = (state[3] + delta[3]).clamp(t_min, t_max);
        }

        let gap = s1.evaluate(state[0], state[1]) - s2.evaluate(state[2], state[3]);
        if gap.norm() <= tol.linear && (state[which] - value).abs() <= tol.parametric {
            Some(gap.norm())
        } else {
            None
        }
    }

    /// 交線の進む向きと、2つの法線のなす角の正弦。
    ///
    /// 正弦は「そこで交線の位置がどれだけ決まるか」を表す。小さいほど、
    /// 残差を詰めても位置が定まらない。
    fn tangent(s1: &NurbsSurface3, s2: &NurbsSurface3, state: &[f64; 4]) -> Option<(Vec3, f64)> {
        let (_, du1, dv1) = s1.evaluate_derivatives_1st(state[0], state[1]);
        let (_, du2, dv2) = s2.evaluate_derivatives_1st(state[2], state[3]);
        let n1 = du1.cross(&dv1).try_normalize_safe(1e-12)?;
        let n2 = du2.cross(&dv2).try_normalize_safe(1e-12)?;
        let cross = n1.cross(&n2);
        let sine = cross.norm();
        Some((cross.try_normalize_safe(1e-12)?, sine))
    }

    fn run(
        s1: &NurbsSurface3,
        s2: &NurbsSurface3,
        start: [f64; 4],
        step: f64,
        max_points: usize,
        tol: &Tolerance,
    ) -> Run {
        let ((u_min, u_max), (v_min, v_max)) = s1.param_range();
        let ((s_min, s_max), (t_min, t_max)) = s2.param_range();
        let inside = |state: &[f64; 4]| {
            let margin = 1e-12;
            state[0] > u_min - margin
                && state[0] < u_max + margin
                && state[1] > v_min - margin
                && state[1] < v_max + margin
                && state[2] > s_min - margin
                && state[2] < s_max + margin
                && state[3] > t_min - margin
                && state[3] < t_max + margin
        };
        let at_edge = |state: &[f64; 4]| {
            let margin = 1e-9;
            (state[0] - u_min).abs() < margin
                || (state[0] - u_max).abs() < margin
                || (state[1] - v_min).abs() < margin
                || (state[1] - v_max).abs() < margin
                || (state[2] - s_min).abs() < margin
                || (state[2] - s_max).abs() < margin
                || (state[3] - t_min).abs() < margin
                || (state[3] - t_max).abs() < margin
        };

        let mut state = start;
        let start_point = s1.evaluate(state[0], state[1]);
        let mut points = vec![Self::sample(s1, s2, &state)];
        let mut hit_boundary = false;
        let mut hit_tangency = false;
        let mut closed = false;
        let mut travelled = 0.0f64;

        for index in 0..max_points {
            let Some((direction, sine)) = Self::tangent(s1, s2, &state) else {
                break;
            };
            if sine < TANGENCY_SINE_LIMIT {
                hit_tangency = true;
                break;
            }
            let anchor = s1.evaluate(state[0], state[1]);

            // 予測: 接線ぶんだけパラメータを進める。両曲面それぞれで
            // 「その向きに動くにはどれだけ (u, v) を動かすか」を最小二乗で解く。
            let mut next = state;
            if let Some((a, b)) = Self::parameter_velocity(s1, state[0], state[1], direction) {
                next[0] += a * step;
                next[1] += b * step;
            }
            if let Some((c, d)) = Self::parameter_velocity(s2, state[2], state[3], direction) {
                next[2] += c * step;
                next[3] += d * step;
            }

            // 修正: 進んだ先で交線へ落とす。歩幅が大きすぎて落ちなければ縮める。
            let mut taken = step;
            let mut settled = None;
            for _ in 0..8 {
                let mut trial = next;
                if Self::newton(s1, s2, &mut trial, anchor, direction, taken, tol).is_some() {
                    settled = Some(trial);
                    break;
                }
                taken *= 0.5;
                trial = state;
                if let Some((a, b)) = Self::parameter_velocity(s1, state[0], state[1], direction) {
                    trial[0] += a * taken;
                    trial[1] += b * taken;
                }
                if let Some((c, d)) = Self::parameter_velocity(s2, state[2], state[3], direction) {
                    trial[2] += c * taken;
                    trial[3] += d * taken;
                }
                next = trial;
            }
            // 歩けなかったときも、そこで終わりにしない。パラメータは毎回
            // 範囲に丸められるので、縁の向こうへ出ようとした歩は必ず失敗する。
            // 失敗は「縁に着いた」ことの合図でもあるので、縁への着地を試す。
            let settled = match settled {
                Some(state) => state,
                None => {
                    if let Some(edge_state) = Self::land_on_nearest_bound(
                        s1,
                        s2,
                        &state,
                        direction,
                        step.abs() * 2.0,
                        tol,
                    ) {
                        points.push(Self::sample(s1, s2, &edge_state));
                    }
                    hit_boundary = true;
                    break;
                }
            };

            if !inside(&settled) {
                // 縁を越えたら、越えた手前で止めるのではなく**縁ちょうど**に
                // 着地させる。手前で止めると、辿った曲線の端が面の境界に
                // 届かず、その曲線では面を割れない（実測で 7.07e-4 足りず、
                // 分割が断られた）。4つ目の式を平面から「そのパラメータが
                // 境界値に等しい」に差し替えるだけでよい。
                let bounds = [
                    (u_min, u_max),
                    (v_min, v_max),
                    (s_min, s_max),
                    (t_min, t_max),
                ];
                let mut landed = None;
                for (index, (low, high)) in bounds.iter().enumerate() {
                    let target = if settled[index] < *low {
                        *low
                    } else if settled[index] > *high {
                        *high
                    } else {
                        continue;
                    };
                    let mut trial = state;
                    if Self::newton_to_bound(s1, s2, &mut trial, index, target, tol).is_some() {
                        landed = Some(trial);
                        break;
                    }
                }
                if let Some(edge_state) = landed {
                    // 直前の点と同じ位置なら足さない。
                    let here = s1.evaluate(edge_state[0], edge_state[1]);
                    let previous = s1.evaluate(state[0], state[1]);
                    if (here - previous).norm() > tol.linear {
                        points.push(Self::sample(s1, s2, &edge_state));
                    }
                }
                hit_boundary = true;
                break;
            }
            state = settled;
            travelled += taken.abs();
            points.push(Self::sample(s1, s2, &state));

            let here = s1.evaluate(state[0], state[1]);
            if index > 2 && travelled > step.abs() * 3.0 && (here - start_point).norm() <= step.abs()
            {
                closed = true;
                break;
            }
            if at_edge(&state) {
                hit_boundary = true;
                break;
            }
        }

        Run {
            points,
            closed,
            hit_boundary,
            hit_tangency,
        }
    }

    /// `direction` の向きに 3D で1進むには、`(u, v)` をどれだけ動かすか。
    fn parameter_velocity(
        surface: &NurbsSurface3,
        u: f64,
        v: f64,
        direction: Vec3,
    ) -> Option<(f64, f64)> {
        let (_, du, dv) = surface.evaluate_derivatives_1st(u, v);
        let a11 = du.dot(&du);
        let a12 = du.dot(&dv);
        let a22 = dv.dot(&dv);
        let b1 = direction.dot(&du);
        let b2 = direction.dot(&dv);
        let determinant = a11 * a22 - a12 * a12;
        if determinant.abs() <= 1e-18 {
            return None;
        }
        Some((
            (b1 * a22 - b2 * a12) / determinant,
            (b2 * a11 - b1 * a12) / determinant,
        ))
    }

    fn sample(
        s1: &NurbsSurface3,
        s2: &NurbsSurface3,
        state: &[f64; 4],
    ) -> SurfaceIntersectionPoint {
        let p1 = s1.evaluate(state[0], state[1]);
        let p2 = s2.evaluate(state[2], state[3]);
        SurfaceIntersectionPoint {
            point: Point3::from((p1.coords + p2.coords) * 0.5),
            uv1: (state[0], state[1]),
            uv2: (state[2], state[3]),
        }
    }

    /// 辿った点列を、両方の曲面の上に乗る1本の曲線に当てはめる。
    ///
    /// **当てはめた点で測っても意味がない。** そこは補間の定義から通るので、
    /// どんな曲線でも 0 が出る。曲線を、補間に使った位置と**互いに素な**
    /// 位置で標本し、そこから両曲面への距離を測って返す。
    ///
    /// 返すのは `(曲線, 最悪の距離)`。距離が要求に足りなければ、歩幅を
    /// 縮めて辿り直すのは呼び手の仕事である。
    pub fn fit_curve(
        s1: &NurbsSurface3,
        s2: &NurbsSurface3,
        marched: &MarchedIntersection,
        degree: usize,
    ) -> Option<(crate::nurbs_curve::NurbsCurve3, f64)> {
        let points: Vec<Point3> = marched.points.iter().map(|sample| sample.point).collect();
        if points.len() < 2 {
            return None;
        }
        let curve = crate::nurbs_curve::NurbsCurve3::interpolate_points(degree, &points).ok()?;

        let (t0, t1) = curve.param_range();
        // 補間に使ったのは点の数ぶんの位置なので、標本数はそれと互いに素に取る。
        let samples = points.len() * 4 + 1;
        let mut worst: f64 = 0.0;
        for step in 0..=samples {
            let fraction = step as f64 / samples as f64;
            let t = t0 + (t1 - t0) * fraction;
            let point = curve.evaluate(t);
            // 射影の出発点には、いちばん近い辿り点の (u, v) を渡す。曲線は
            // その点列を通るので、答えはすぐ隣にある。粗サンプリングから
            // 始めると1回あたり 16 x 16 の評価が余分に走り、面の組ごとに
            // これを回すブーリアンでは効いてくる。
            let nearest =
                ((fraction * (marched.points.len() - 1) as f64).round() as usize)
                    .min(marched.points.len() - 1);
            let seed = &marched.points[nearest];
            for (surface, uv) in [(s1, seed.uv1), (s2, seed.uv2)] {
                let projection = ExtremumEngine::point_to_surface_seeded(
                    point, surface, uv.0, uv.1, 64, 1e-13,
                )
                .ok()?;
                worst = worst.max(projection.distance);
            }
        }
        Some((curve, worst))
    }

    /// 交線を、要求した精度で1本の曲線にする。
    ///
    /// 歩幅を決め打ちにすると、曲率の高い交線では足りず、緩い交線では
    /// 無駄に細かくなる。**当てはめてから測り、足りなければ歩幅を半分にして
    /// やり直す。** 測ってから決めるので、形を問わずに要求が満たせる。
    pub fn fit_to_tolerance(
        s1: &NurbsSurface3,
        s2: &NurbsSurface3,
        first_step: f64,
        deviation_limit: f64,
        tol: &Tolerance,
    ) -> Option<(crate::nurbs_curve::NurbsCurve3, MarchedIntersection, f64)> {
        // 種は一度だけ集める。歩幅を縮めるたびに探し直すと、同じ答えを何度も
        // 計算することになる。
        let seeds = Self::find_seeds(s1, s2, 16, 4);
        for (seed_u, seed_v) in seeds {
            let mut step = first_step;
            let mut previous: Option<f64> = None;
            for _ in 0..6 {
                if let Some(marched) = Self::march(s1, s2, seed_u, seed_v, step, 2048, tol) {
                    if marched.points.len() >= 4 {
                        if let Some((curve, deviation)) = Self::fit_curve(s1, s2, &marched, 3) {
                            if deviation <= deviation_limit {
                                return Some((curve, marched, deviation));
                            }
                            // 3次の補間なので、歩幅を半分にすればずれは 8 分の1
                            // 前後まで減るはずである。減り方がそれよりずっと
                            // 悪いなら、足りないのは歩幅ではない。刻み続けても
                            // 届かないので、次の種へ移る。
                            if let Some(before) = previous {
                                if deviation > before * 0.5 {
                                    break;
                                }
                            }
                            previous = Some(deviation);
                        }
                    }
                }
                step *= 0.5;
            }
        }
        None
    }

    /// 点列が2つの曲面からどれだけ離れているか。
    ///
    /// **辿るのに使った (u, v) では測らない。** そこは構成上ぴったりなので、
    /// 何も分からない。改めて曲面へ射影して測る。
    pub fn worst_distance(
        s1: &NurbsSurface3,
        s2: &NurbsSurface3,
        points: &[SurfaceIntersectionPoint],
    ) -> f64 {
        let mut worst: f64 = 0.0;
        for sample in points {
            for surface in [s1, s2] {
                if let Ok(projection) =
                    ExtremumEngine::point_to_surface(sample.point, surface, 64, 1e-13)
                {
                    worst = worst.max(projection.distance);
                }
            }
        }
        worst
    }
}

struct Run {
    points: Vec<SurfaceIntersectionPoint>,
    closed: bool,
    hit_boundary: bool,
    hit_tangency: bool,
}

#[cfg(test)]
mod tests {
    use super::{IntersectionMarcher, TANGENCY_SINE_LIMIT};
    use crate::bspline_basis::KnotVector;
    use crate::nurbs_curve::ControlPoint3;
    use crate::nurbs_surface::NurbsSurface3;
    use std::f64::consts::FRAC_1_SQRT_2;
    use zenith_math::{Point3, Tolerance, Vec3};

    /// 軸 `axis` まわり、半径 `r` の円柱の四半パッチ。
    fn cylinder_quarter(r: f64, length: f64, axis: Vec3, x_axis: Vec3, origin: Point3) -> NurbsSurface3 {
        let w = FRAC_1_SQRT_2;
        let y_axis = axis.cross(&x_axis).normalize();
        let ring = [
            (origin + x_axis * r, 1.0),
            (origin + (x_axis + y_axis) * r, w),
            (origin + y_axis * r, 1.0),
        ];
        let grid: Vec<Vec<ControlPoint3>> = ring
            .iter()
            .map(|(point, weight)| {
                vec![
                    ControlPoint3::new(*point - axis * (length * 0.5), *weight),
                    ControlPoint3::new(*point + axis * (length * 0.5), *weight),
                ]
            })
            .collect();
        NurbsSurface3::new(
            2,
            1,
            grid,
            KnotVector::clamped_uniform(3, 2),
            KnotVector::clamped_uniform(2, 1),
        )
        .unwrap()
    }

    /// 球の第1象限（経度 0..90 度、赤道から極まで）。
    fn sphere_octant(r: f64) -> NurbsSurface3 {
        let w = FRAC_1_SQRT_2;
        let rows = [(r, 0.0, 1.0), (r, r, w), (0.0, r, 1.0)];
        let grid: Vec<Vec<ControlPoint3>> = rows
            .iter()
            .map(|(radial, height, weight)| {
                vec![
                    ControlPoint3::new(Point3::new(*radial, 0.0, *height), *weight),
                    ControlPoint3::new(Point3::new(*radial, *radial, *height), weight * w),
                    ControlPoint3::new(Point3::new(0.0, *radial, *height), *weight),
                ]
            })
            .collect();
        NurbsSurface3::new(
            2,
            2,
            grid,
            KnotVector::clamped_uniform(3, 2),
            KnotVector::clamped_uniform(3, 2),
        )
        .unwrap()
    }

    /// 半径の等しい直交円柱の交線は、`z = x` の平面上の楕円ちょうど。
    ///
    /// **辿るのに使った (u, v) では測らない。** そこは構成上ぴったりなので、
    /// どんな点列を作っても 0 が出る。3D の座標を閉じた式に当てる。
    #[test]
    fn equal_radius_perpendicular_cylinders_meet_on_the_plane_z_equals_x() {
        let tol = Tolerance::default();
        let radius = 10.0;
        let a = cylinder_quarter(
            radius,
            60.0,
            Vec3::new(0.0, 0.0, 1.0),
            Vec3::new(1.0, 0.0, 0.0),
            Point3::new(0.0, 0.0, 0.0),
        );
        let b = cylinder_quarter(
            radius,
            60.0,
            Vec3::new(1.0, 0.0, 0.0),
            Vec3::new(0.0, 1.0, 0.0),
            Point3::new(0.0, 0.0, 0.0),
        );

        let curve = IntersectionMarcher::march_from_best_seed(&a, &b, 16, 0.5, 4096, &tol)
            .expect("the two cylinders do meet");
        assert!(
            curve.points.len() > 20,
            "only {} points came back",
            curve.points.len()
        );

        let mut worst_plane: f64 = 0.0;
        let mut worst_first: f64 = 0.0;
        let mut worst_second: f64 = 0.0;
        for sample in &curve.points {
            let p = sample.point;
            worst_plane = worst_plane.max((p.z - p.x).abs());
            worst_first = worst_first.max(((p.x * p.x + p.y * p.y).sqrt() - radius).abs());
            worst_second = worst_second.max(((p.y * p.y + p.z * p.z).sqrt() - radius).abs());
        }
        assert!(worst_plane < 1e-9, "the curve left the plane z = x by {worst_plane:.3e}");
        assert!(worst_first < 1e-9, "the curve left the first cylinder by {worst_first:.3e}");
        assert!(worst_second < 1e-9, "the curve left the second cylinder by {worst_second:.3e}");
        assert!(curve.worst_off_surface < 1e-9);
    }

    /// 球と、その中心を通る軸の円柱の交線は、半径 `r`、高さ
    /// `sqrt(R^2 - r^2)` の真円。
    #[test]
    fn a_sphere_and_a_cylinder_through_its_centre_meet_on_a_circle() {
        let tol = Tolerance::default();
        let (sphere_radius, cylinder_radius): (f64, f64) = (12.0, 5.0);
        let height = (sphere_radius * sphere_radius - cylinder_radius * cylinder_radius).sqrt();

        let sphere = sphere_octant(sphere_radius);
        let cylinder = cylinder_quarter(
            cylinder_radius,
            40.0,
            Vec3::new(0.0, 0.0, 1.0),
            Vec3::new(1.0, 0.0, 0.0),
            Point3::new(0.0, 0.0, 0.0),
        );

        let curve = IntersectionMarcher::march_from_best_seed(&sphere, &cylinder, 16, 0.3, 4096, &tol)
            .expect("the sphere and the cylinder do meet");

        let mut worst_height: f64 = 0.0;
        let mut worst_radius: f64 = 0.0;
        for sample in &curve.points {
            let p = sample.point;
            worst_height = worst_height.max((p.z - height).abs());
            worst_radius =
                worst_radius.max(((p.x * p.x + p.y * p.y).sqrt() - cylinder_radius).abs());
        }
        assert!(
            worst_height < 1e-9,
            "the curve should sit at z = {height}, off by {worst_height:.3e}"
        );
        assert!(
            worst_radius < 1e-9,
            "the curve should have radius {cylinder_radius}, off by {worst_radius:.3e}"
        );
    }

    /// 接点の上から始めることは断る。
    ///
    /// そこでは「両方の曲面の上にある」が位置を決めない。等半径の直交円柱の
    /// 四半パッチの真ん中から落とすと、拘束平面がちょうど接点を選び、
    /// 交線から 2.99e-5 離れた点が残差 2.24e-11 で通ってしまう。
    #[test]
    fn a_seed_that_lands_on_a_tangency_is_refused_rather_than_answered() {
        let tol = Tolerance::default();
        let a = cylinder_quarter(
            10.0,
            60.0,
            Vec3::new(0.0, 0.0, 1.0),
            Vec3::new(1.0, 0.0, 0.0),
            Point3::new(0.0, 0.0, 0.0),
        );
        let b = cylinder_quarter(
            10.0,
            60.0,
            Vec3::new(1.0, 0.0, 0.0),
            Vec3::new(0.0, 1.0, 0.0),
            Point3::new(0.0, 0.0, 0.0),
        );

        // パッチの真ん中から落とすと接点に着く配置。
        assert!(
            IntersectionMarcher::land_on_curve(&a, &b, 0.5, 0.5, &tol).is_none(),
            "landing on a tangency must be refused, not answered with a plausible point"
        );

        // 種を選ばせれば通る。
        assert!(IntersectionMarcher::march_from_best_seed(&a, &b, 16, 0.5, 4096, &tol).is_some());
        assert!(TANGENCY_SINE_LIMIT > 0.0);
    }

    /// 交わらない2面からは、交線を作ってはならない。
    #[test]
    fn surfaces_that_do_not_meet_produce_nothing() {
        let tol = Tolerance::default();
        let a = cylinder_quarter(
            10.0,
            60.0,
            Vec3::new(0.0, 0.0, 1.0),
            Vec3::new(1.0, 0.0, 0.0),
            Point3::new(0.0, 0.0, 0.0),
        );
        let far = cylinder_quarter(
            10.0,
            60.0,
            Vec3::new(0.0, 0.0, 1.0),
            Vec3::new(1.0, 0.0, 0.0),
            Point3::new(500.0, 0.0, 0.0),
        );
        let outcome = IntersectionMarcher::march_from_best_seed(&a, &far, 12, 0.5, 512, &tol);
        assert!(
            outcome.map(|curve| curve.points.len() < 3).unwrap_or(true),
            "two cylinders 500 apart must not produce an intersection curve"
        );
    }
}
