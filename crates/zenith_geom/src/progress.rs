//! **どれだけ経ったか**と、**その間に進んだか**を出す。
//!
//! # なぜ要るのか
//!
//! 計算が返ってこないとき、知りたいのは2つです。
//!
//! - **どの計算で止まっているのか**
//! - それは**遅いだけ**なのか、**進んでいない**のか
//!
//! 後者は時間だけでは分かりません。10 分返らなくても、仕事量が増え続けて
//! いるなら進んでいます。増えていないなら、**収束しない輪の中にいます**。
//!
//! そこで、経過時間と [`work_counter`](crate::work_counter) の差分を**一緒に**
//! 出します。実際にこれで見つけた例:
//!
//! - `linkrods.step` の和が 2時間20分 返らなかった件（HANDOVER 4-269）。
//!   仕事量は増え続けていた——止まっていたのではなく、**切り手が実物の
//!   10万倍の大きさ**でした
//! - 射影の 9割が p-curve の導出だと分かった件（4-271）
//!
//! # 数え上げと壁時計の使い分け
//!
//! [`work_counter`](crate::work_counter) の冒頭に書いてあるとおり、
//! **「速くなったか」を壁時計で判定してはいけません**（同じ仕事が同じ時間で
//! 終わらないので）。ここで壁時計を使うのは**別の目的**です——
//! **「終わらないものを見つける」**ためで、比較のためではありません。
//!
//! 速さを主張したいときは、いまでも数え上げを見てください。
//!
//! # 使い方
//!
//! ```no_run
//! use zenith_geom::progress::Heartbeat;
//!
//! // 5秒ごとに「経過 / その間に増えた仕事量」を出す。落ちるときに合計も出す。
//! let beat = Heartbeat::start("linkrods union");
//! // ... 重い計算 ...
//! drop(beat);
//! ```
//!
//! **既定では黙っています。** `ZENITH_PROGRESS=1`（または秒数）で開きます。
//! 黙っているときは糸も立てません。

use std::sync::{Arc, Condvar, Mutex};
use std::time::{Duration, Instant};

/// 拍を打つ間隔を決める。`ZENITH_PROGRESS` が無ければ `None`。
///
/// - `ZENITH_PROGRESS=1` → 5秒ごと（既定）
/// - `ZENITH_PROGRESS=30` → 30秒ごと
fn interval_from_env() -> Option<Duration> {
    let value = std::env::var("ZENITH_PROGRESS").ok()?;
    let trimmed = value.trim();
    if trimmed.is_empty() || trimmed == "0" {
        return None;
    }
    let seconds: f64 = trimmed.parse().unwrap_or(1.0);
    // `=1` は「開ける」の意味で書かれることが多いので、既定の 5 秒に寄せます。
    let seconds = if seconds <= 1.0 { 5.0 } else { seconds };
    Some(Duration::from_secs_f64(seconds))
}

/// 一区切りの計算に付ける拍動。
///
/// 落ちる（`drop`）ときに、**掛かった時間と、その間の仕事量**を出します。
/// 開いていれば、途中でも一定の間隔で経過を出します。
pub struct Heartbeat {
    label: String,
    started: Instant,
    before: crate::work_counter::WorkCounters,
    /// 止めの合図。**眠っている糸をその場で起こします**——`sleep` で刻むと、
    /// 落ちるたびに最大1刻みぶん待たされます（実測で 1演算あたり 0.2 秒。
    /// 141演算なら 28 秒になります）。
    stop: Option<Arc<(Mutex<bool>, Condvar)>>,
    handle: Option<std::thread::JoinHandle<()>>,
}

impl Heartbeat {
    /// 拍動を始める。`ZENITH_PROGRESS` が無ければ、時間だけ控えて黙ります。
    pub fn start(label: impl Into<String>) -> Self {
        let label = label.into();
        let started = Instant::now();
        let before = crate::work_counter::snapshot();

        let Some(interval) = interval_from_env() else {
            return Self {
                label,
                started,
                before,
                stop: None,
                handle: None,
            };
        };

        eprintln!("PROGRESS [{label}] 開始");
        let stop = Arc::new((Mutex::new(false), Condvar::new()));
        let handle = {
            let stop = Arc::clone(&stop);
            let label = label.clone();
            let before = before;
            std::thread::spawn(move || {
                let mut previous = before;
                let (lock, condvar) = &*stop;
                loop {
                    // **間隔ぶん眠り、止められたらその場で起きます。**
                    let guard = lock.lock().expect("止めの合図");
                    let (guard, timeout) = condvar
                        .wait_timeout(guard, interval)
                        .expect("止めの合図を待つ");
                    if *guard {
                        break;
                    }
                    drop(guard);
                    if !timeout.timed_out() {
                        continue;
                    }
                    let now = crate::work_counter::snapshot();
                    let step = now.since(&previous);
                    previous = now;
                    // **増えていないなら、進んでいません。**
                    let moving = step.surface_evaluations > 0
                        || step.point_surface_projections > 0
                        || step.marching_newton_iterations > 0
                        || step.face_integrals > 0
                        || step.uv_triangulations > 0;
                    eprintln!(
                        "PROGRESS [{label}] 経過 {:.0} 秒: 曲面評価 +{}、射影 +{}（うち p-curve +{}）、辿り +{}、面の積分 +{}、p-curve 作り直し +{}（持っていた +{}）  {}",
                        started.elapsed().as_secs_f64(),
                        step.surface_evaluations,
                        step.point_surface_projections,
                        step.pcurve_projections,
                        step.marching_newton_iterations,
                        step.face_integrals,
                        step.pcurve_derivations,
                        step.pcurve_cache_hits,
                        if moving {
                            "進んでいます"
                        } else {
                            "**進んでいません**（収束しない輪の疑い）"
                        }
                    );
                }
            })
        };

        Self {
            label,
            started,
            before,
            stop: Some(stop),
            handle: Some(handle),
        }
    }

    /// ここまでの経過。
    pub fn elapsed(&self) -> Duration {
        self.started.elapsed()
    }

    /// ここまでに増えた仕事量。
    pub fn work(&self) -> crate::work_counter::WorkCounters {
        crate::work_counter::snapshot().since(&self.before)
    }
}

impl Drop for Heartbeat {
    fn drop(&mut self) {
        if let Some(stop) = self.stop.take() {
            let (lock, condvar) = &*stop;
            *lock.lock().expect("止めの合図") = true;
            condvar.notify_all();
        }
        if let Some(handle) = self.handle.take() {
            let _ = handle.join();
        }
        // **終わりの1行は、開いていなくても出しません。** 黙っているときに
        // 出力の形を変えると、既存の口の読み取りが壊れます。開いているときだけ。
        if interval_from_env().is_some() {
            let step = self.work();
            eprintln!(
                "PROGRESS [{}] 終了 {:.2} 秒（曲面評価 {}、射影 {}、面の積分 {}）",
                self.label,
                self.started.elapsed().as_secs_f64(),
                step.surface_evaluations,
                step.point_surface_projections,
                step.face_integrals
            );
        }
    }
}

/// 一区切りを測って、掛かった秒数と一緒に返す。
///
/// **返り値の秒数は、比較のためではありません**（work_counter の冒頭を
/// 読んでください）。**どこで時間が消えたかを見る**ためのものです。
pub fn timed<T>(label: &str, body: impl FnOnce() -> T) -> (T, f64) {
    let beat = Heartbeat::start(label);
    let value = body();
    let seconds = beat.elapsed().as_secs_f64();
    (value, seconds)
}
