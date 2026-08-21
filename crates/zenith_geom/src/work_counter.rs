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

/// ある時点までに積み上がった仕事の量。
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct WorkCounters {
    /// NURBS 曲面の点・微分の評価回数。
    pub surface_evaluations: u64,
    /// 交線を辿るニュートン法の反復回数。
    pub marching_newton_iterations: u64,
    /// 交線を辿り始めた回数。
    pub marching_calls: u64,
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
        }
    }
}

/// いまの積算値を読む。
pub fn snapshot() -> WorkCounters {
    WorkCounters {
        surface_evaluations: SURFACE_EVALUATIONS.load(Ordering::Relaxed),
        marching_newton_iterations: MARCHING_NEWTON_ITERATIONS.load(Ordering::Relaxed),
        marching_calls: MARCHING_CALLS.load(Ordering::Relaxed),
    }
}

/// 積算値を 0 に戻す。
pub fn reset() {
    SURFACE_EVALUATIONS.store(0, Ordering::Relaxed);
    MARCHING_NEWTON_ITERATIONS.store(0, Ordering::Relaxed);
    MARCHING_CALLS.store(0, Ordering::Relaxed);
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
