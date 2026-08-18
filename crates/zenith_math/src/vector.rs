/// 2次元ベクトル
pub type Vec2 = nalgebra::Vector2<f64>;

/// 3次元ベクトル
pub type Vec3 = nalgebra::Vector3<f64>;

/// ベクトルの拡張トレイト
pub trait Vec3Ext {
    /// ゼロベクトル判定
    fn is_zero(&self, tol: f64) -> bool;
    /// 2つのベクトルが平行かどうか
    fn is_parallel_to(&self, other: &Self, tol: f64) -> bool;
    /// 2つのベクトルが直交しているかどうか
    fn is_perpendicular_to(&self, other: &Self, tol: f64) -> bool;
    /// 正規化（長さが0近傍ならNoneを返す安全な正規化）
    fn try_normalize_safe(&self, tol: f64) -> Option<Vec3>;
}

impl Vec3Ext for Vec3 {
    fn is_zero(&self, tol: f64) -> bool {
        self.norm_squared() <= tol * tol
    }

    fn is_parallel_to(&self, other: &Self, tol: f64) -> bool {
        if let (Some(n1), Some(n2)) = (self.try_normalize_safe(tol), other.try_normalize_safe(tol))
        {
            let dot = n1.dot(&n2).abs();
            (dot - 1.0).abs() <= tol
        } else {
            false
        }
    }

    fn is_perpendicular_to(&self, other: &Self, tol: f64) -> bool {
        if let (Some(n1), Some(n2)) = (self.try_normalize_safe(tol), other.try_normalize_safe(tol))
        {
            n1.dot(&n2).abs() <= tol
        } else {
            false
        }
    }

    fn try_normalize_safe(&self, tol: f64) -> Option<Vec3> {
        let norm = self.norm();
        if norm > tol {
            Some(self / norm)
        } else {
            None
        }
    }
}
