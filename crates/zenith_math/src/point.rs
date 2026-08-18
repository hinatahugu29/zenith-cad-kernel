/// 2次元座標点
pub type Point2 = nalgebra::Point2<f64>;

/// 3次元座標点
pub type Point3 = nalgebra::Point3<f64>;

/// 点の拡張トレイト
pub trait Point3Ext {
    /// 2点間のユークリッド距離
    fn distance_to(&self, other: &Self) -> f64;
    /// 許容誤差範囲内で同一点かどうか
    fn is_coincident_with(&self, other: &Self, tol: f64) -> bool;
    /// 重み付き内分点
    fn lerp(&self, other: &Self, t: f64) -> Point3;
}

impl Point3Ext for Point3 {
    fn distance_to(&self, other: &Self) -> f64 {
        nalgebra::distance(self, other)
    }

    fn is_coincident_with(&self, other: &Self, tol: f64) -> bool {
        self.distance_to(other) <= tol
    }

    fn lerp(&self, other: &Self, t: f64) -> Point3 {
        Point3::from(self.coords * (1.0 - t) + other.coords * t)
    }
}
