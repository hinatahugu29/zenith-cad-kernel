use serde::{Deserialize, Serialize};

/// CAD演算における幾何公差（Tolerance）設定
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct Tolerance {
    /// 空間内の距離公差（デフォルト: 1e-6）
    pub linear: f64,
    /// 角度公差（ラジアン）（デフォルト: 1e-5 rad ≒ 0.00057度）
    pub angular: f64,
    /// パラメトリック公差（UV空間など）（デフォルト: 1e-7）
    pub parametric: f64,
}

impl Default for Tolerance {
    fn default() -> Self {
        Self {
            linear: 1e-6,
            angular: 1e-5,
            parametric: 1e-7,
        }
    }
}

impl Tolerance {
    /// 新規公差設定
    pub const fn new(linear: f64, angular: f64, parametric: f64) -> Self {
        Self {
            linear,
            angular,
            parametric,
        }
    }

    /// 高精度（ファイン）公差
    pub const fn fine() -> Self {
        Self {
            linear: 1e-8,
            angular: 1e-6,
            parametric: 1e-9,
        }
    }

    /// 粗め（コース）公差（テッセレーションや大まかなバウンディング判定用）
    pub const fn coarse() -> Self {
        Self {
            linear: 1e-3,
            angular: 1e-3,
            parametric: 1e-4,
        }
    }

    /// 2つの実数値が線形公差内で等しいか
    pub fn approx_eq(&self, a: f64, b: f64) -> bool {
        (a - b).abs() <= self.linear
    }

    /// パラメータ値がパラメトリック公差内で等しいか
    pub fn approx_eq_param(&self, u1: f64, u2: f64) -> bool {
        (u1 - u2).abs() <= self.parametric
    }
}
