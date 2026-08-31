//! 仕事の量を数える。
//!
//! # なぜ時間ではなく数か
//!
//! 「速くなったか」を壁時計で判定するには、同じ仕事が同じ時間で終わる必要が
//! ある。この環境ではそうならない。同じバイナリで同じ45ケースを走らせて、
//! 合計が 6分13秒 から 6分39秒 まで振れ、1ケースでは**やる仕事を減らした側が
//! 14秒遅く**出た。その幅の中では、消した重複が効いたのかどうかを言えない。
//!
//! 数え上げは決定的である。曲面を何回評価したか、ニュートン法を何回回したかは
//! 走らせるたびに同じ値になり、機械の忙しさに左右されない。重複を消したなら
//! 数は必ず減り、形を変えていないなら結果の表は動かない。**この2つは壁時計
//! なしで確かめられる。**
//!
//! 数が減っても時間が減るとは限らない（数えている単位の重さが一定とは
//! 限らない）。時間を主張したいなら時間を測る必要がある。ここで言えるのは
//! 「同じ答えに、より少ない仕事で到達した」までである。

use std::sync::atomic::{AtomicU64, Ordering};

static SURFACE_EVALUATIONS: AtomicU64 = AtomicU64::new(0);
static MARCHING_NEWTON_ITERATIONS: AtomicU64 = AtomicU64::new(0);
static MARCHING_CALLS: AtomicU64 = AtomicU64::new(0);
static SEED_SEARCHES: AtomicU64 = AtomicU64::new(0);
static POINT_SURFACE_PROJECTIONS: AtomicU64 = AtomicU64::new(0);
static POINT_SURFACE_COARSE_SEARCHES: AtomicU64 = AtomicU64::new(0);
static PROJECTION_NEWTON_ITERATIONS: AtomicU64 = AtomicU64::new(0);
static PROJECTION_DAMPING_TRIALS: AtomicU64 = AtomicU64::new(0);
static FACE_INTEGRALS: AtomicU64 = AtomicU64::new(0);
static UV_TRIANGULATIONS: AtomicU64 = AtomicU64::new(0);
static UV_TRIANGLES: AtomicU64 = AtomicU64::new(0);
static UV_BOUNDARY_POINTS: AtomicU64 = AtomicU64::new(0);
static UV_WORST_BOUNDARY: AtomicU64 = AtomicU64::new(0);
static SOLID_TESSELLATIONS: AtomicU64 = AtomicU64::new(0);
static GRID_PATCHES: AtomicU64 = AtomicU64::new(0);
static EARCUT_PATCHES: AtomicU64 = AtomicU64::new(0);

/// ある時点までに積み上がった仕事の量。
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct WorkCounters {
    /// NURBS 曲面の点・微分の評価回数。
    pub surface_evaluations: u64,
    /// 交線を辿るニュートン法の反復回数。
    pub marching_newton_iterations: u64,
    /// 交線を辿り始めた回数。
    pub marching_calls: u64,
    /// 種を探すために格子を走らせた回数。
    pub seed_searches: u64,
    /// 点から曲面への最近傍射影の回数。
    pub point_surface_projections: u64,
    /// そのうち、出発点を渡されずに全域を粗く見た回数。
    ///
    /// 1回につき 17x17 の格子と8段の詰めで 353 回の曲面評価を払う。
    /// ニュートン法本体はその後の数回なので、**費用のほとんどが
    /// 「どこから始めるか」に消える**。ここが減れば全体が減る。
    pub point_surface_coarse_searches: u64,
    /// 射影のニュートン反復の回数。
    pub projection_newton_iterations: u64,
    /// そのうち、歩幅を半分にして試した回数。1回につき曲面評価1回。
    pub projection_damping_trials: u64,
    /// 面を1枚まるごと積分した回数。
    ///
    /// ブーリアンは分割の正しさを面積の和で確かめるので、演算の途中で
    /// ここが何度も回る。1回につき三角形の数 x 6 点の評価を払う。
    pub face_integrals: u64,
    /// 面のパラメータ領域を三角形に割った回数。
    pub uv_triangulations: u64,
    /// そこで出来た三角形の総数。
    pub uv_triangles: u64,
    /// トリム境界の折れ線が持っていた点の総数。
    pub uv_boundary_points: u64,
    /// そのうち、いちばん多かった1ループぶん。
    pub uv_worst_boundary: u64,
    /// 立体をまるごとテッセレートした回数。
    ///
    /// 内外判定はメッシュに対して行われるので、同じ立体を何度も刻んで
    /// いないかがここに出る。
    pub solid_tessellations: u64,
    /// 面を**構造格子**で張った回数。境界がパラメータ矩形の縁を1周している
    /// パッチはこちらを通り、頼んだ分割数どおりの規則的な格子になる。
    pub grid_patches: u64,
    /// 面を **earcut ＋ 適応細分**で張った回数。
    ///
    /// 構造格子が使えなかった面はこちらへ落ちる。境界の多角形から三角形を
    /// 起こし、たわみが目標を下回るまで最長辺を割り続けるので、**頼んだ
    /// 分割数が上限として効かない**。
    ///
    /// ブーリアンの結果はここを通る面が多い。同じ形をビルダーで作ったものと
    /// 比べて重くなる原因はここで、`SamplePlan` の稜の刻みではない（そちらは
    /// 実測で両者とも頼んだ分割数ちょうどだった）。
    pub earcut_patches: u64,
}

impl WorkCounters {
    /// 2つの時点の差。区間の仕事量になる。
    pub fn since(&self, earlier: &WorkCounters) -> WorkCounters {
        WorkCounters {
            surface_evaluations: self
                .surface_evaluations
                .saturating_sub(earlier.surface_evaluations),
            marching_newton_iterations: self
                .marching_newton_iterations
                .saturating_sub(earlier.marching_newton_iterations),
            marching_calls: self.marching_calls.saturating_sub(earlier.marching_calls),
            seed_searches: self.seed_searches.saturating_sub(earlier.seed_searches),
            point_surface_projections: self
                .point_surface_projections
                .saturating_sub(earlier.point_surface_projections),
            point_surface_coarse_searches: self
                .point_surface_coarse_searches
                .saturating_sub(earlier.point_surface_coarse_searches),
            projection_newton_iterations: self
                .projection_newton_iterations
                .saturating_sub(earlier.projection_newton_iterations),
            projection_damping_trials: self
                .projection_damping_trials
                .saturating_sub(earlier.projection_damping_trials),
            face_integrals: self.face_integrals.saturating_sub(earlier.face_integrals),
            uv_triangulations: self
                .uv_triangulations
                .saturating_sub(earlier.uv_triangulations),
            uv_triangles: self.uv_triangles.saturating_sub(earlier.uv_triangles),
            uv_boundary_points: self
                .uv_boundary_points
                .saturating_sub(earlier.uv_boundary_points),
            uv_worst_boundary: self.uv_worst_boundary.max(earlier.uv_worst_boundary),
            solid_tessellations: self
                .solid_tessellations
                .saturating_sub(earlier.solid_tessellations),
            grid_patches: self.grid_patches.saturating_sub(earlier.grid_patches),
            earcut_patches: self.earcut_patches.saturating_sub(earlier.earcut_patches),
        }
    }
}

/// いまの積算値を読む。
pub fn snapshot() -> WorkCounters {
    WorkCounters {
        surface_evaluations: SURFACE_EVALUATIONS.load(Ordering::Relaxed),
        marching_newton_iterations: MARCHING_NEWTON_ITERATIONS.load(Ordering::Relaxed),
        marching_calls: MARCHING_CALLS.load(Ordering::Relaxed),
        seed_searches: SEED_SEARCHES.load(Ordering::Relaxed),
        point_surface_projections: POINT_SURFACE_PROJECTIONS.load(Ordering::Relaxed),
        point_surface_coarse_searches: POINT_SURFACE_COARSE_SEARCHES.load(Ordering::Relaxed),
        projection_newton_iterations: PROJECTION_NEWTON_ITERATIONS.load(Ordering::Relaxed),
        projection_damping_trials: PROJECTION_DAMPING_TRIALS.load(Ordering::Relaxed),
        face_integrals: FACE_INTEGRALS.load(Ordering::Relaxed),
        uv_triangulations: UV_TRIANGULATIONS.load(Ordering::Relaxed),
        uv_triangles: UV_TRIANGLES.load(Ordering::Relaxed),
        uv_boundary_points: UV_BOUNDARY_POINTS.load(Ordering::Relaxed),
        uv_worst_boundary: UV_WORST_BOUNDARY.load(Ordering::Relaxed),
        solid_tessellations: SOLID_TESSELLATIONS.load(Ordering::Relaxed),
        grid_patches: GRID_PATCHES.load(Ordering::Relaxed),
        earcut_patches: EARCUT_PATCHES.load(Ordering::Relaxed),
    }
}

/// 積算値を 0 に戻す。
pub fn reset() {
    SURFACE_EVALUATIONS.store(0, Ordering::Relaxed);
    MARCHING_NEWTON_ITERATIONS.store(0, Ordering::Relaxed);
    MARCHING_CALLS.store(0, Ordering::Relaxed);
    SEED_SEARCHES.store(0, Ordering::Relaxed);
    POINT_SURFACE_PROJECTIONS.store(0, Ordering::Relaxed);
    POINT_SURFACE_COARSE_SEARCHES.store(0, Ordering::Relaxed);
    PROJECTION_NEWTON_ITERATIONS.store(0, Ordering::Relaxed);
    PROJECTION_DAMPING_TRIALS.store(0, Ordering::Relaxed);
    FACE_INTEGRALS.store(0, Ordering::Relaxed);
    UV_TRIANGULATIONS.store(0, Ordering::Relaxed);
    UV_TRIANGLES.store(0, Ordering::Relaxed);
    UV_BOUNDARY_POINTS.store(0, Ordering::Relaxed);
    UV_WORST_BOUNDARY.store(0, Ordering::Relaxed);
    SOLID_TESSELLATIONS.store(0, Ordering::Relaxed);
    GRID_PATCHES.store(0, Ordering::Relaxed);
    EARCUT_PATCHES.store(0, Ordering::Relaxed);
}

#[inline]
pub fn count_surface_evaluation() {
    SURFACE_EVALUATIONS.fetch_add(1, Ordering::Relaxed);
}

#[inline]
pub fn count_marching_newton_iteration() {
    MARCHING_NEWTON_ITERATIONS.fetch_add(1, Ordering::Relaxed);
}

#[inline]
pub fn count_marching_call() {
    MARCHING_CALLS.fetch_add(1, Ordering::Relaxed);
}

#[inline]
pub fn count_seed_search() {
    SEED_SEARCHES.fetch_add(1, Ordering::Relaxed);
}

#[inline]
pub fn count_point_surface_projection() {
    POINT_SURFACE_PROJECTIONS.fetch_add(1, Ordering::Relaxed);
}

#[inline]
pub fn count_solid_tessellation() {
    SOLID_TESSELLATIONS.fetch_add(1, Ordering::Relaxed);
}

#[inline]
pub fn count_point_surface_coarse_search() {
    POINT_SURFACE_COARSE_SEARCHES.fetch_add(1, Ordering::Relaxed);
}

#[inline]
pub fn count_projection_newton_iteration() {
    PROJECTION_NEWTON_ITERATIONS.fetch_add(1, Ordering::Relaxed);
}

#[inline]
pub fn count_projection_damping_trial() {
    PROJECTION_DAMPING_TRIALS.fetch_add(1, Ordering::Relaxed);
}

#[inline]
pub fn count_face_integral() {
    FACE_INTEGRALS.fetch_add(1, Ordering::Relaxed);
}

#[inline]
pub fn count_uv_triangulation(triangles: usize) {
    UV_TRIANGULATIONS.fetch_add(1, Ordering::Relaxed);
    UV_TRIANGLES.fetch_add(triangles as u64, Ordering::Relaxed);
}

#[inline]
pub fn count_uv_boundary(points: usize) {
    UV_BOUNDARY_POINTS.fetch_add(points as u64, Ordering::Relaxed);
    UV_WORST_BOUNDARY.fetch_max(points as u64, Ordering::Relaxed);
}

/// 面を構造格子で張った。
#[inline]
pub fn count_grid_patch() {
    GRID_PATCHES.fetch_add(1, Ordering::Relaxed);
}

/// 面を earcut ＋ 適応細分で張った。
#[inline]
pub fn count_earcut_patch() {
    EARCUT_PATCHES.fetch_add(1, Ordering::Relaxed);
}
