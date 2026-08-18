use crate::point::Point3;
use crate::vector::Vec3;
use serde::{Deserialize, Serialize};

/// 3次元軸平行バウンディングボックス (AABB)
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct BoundingBox3 {
    pub min: Point3,
    pub max: Point3,
}

impl BoundingBox3 {
    /// 空のバウンディングボックス
    pub fn empty() -> Self {
        Self {
            min: Point3::new(f64::INFINITY, f64::INFINITY, f64::INFINITY),
            max: Point3::new(f64::NEG_INFINITY, f64::NEG_INFINITY, f64::NEG_INFINITY),
        }
    }

    /// 単一点から初期化
    pub fn from_point(p: Point3) -> Self {
        Self { min: p, max: p }
    }

    /// 最小点と最大点から作成
    pub fn from_min_max(min: Point3, max: Point3) -> Self {
        Self { min, max }
    }

    /// 点を追加して拡大
    pub fn extend_point(&mut self, p: Point3) {
        self.min.x = self.min.x.min(p.x);
        self.min.y = self.min.y.min(p.y);
        self.min.z = self.min.z.min(p.z);

        self.max.x = self.max.x.max(p.x);
        self.max.y = self.max.y.max(p.y);
        self.max.z = self.max.z.max(p.z);
    }

    /// 別のBBoxを追加して拡大
    pub fn extend_bbox(&mut self, other: &BoundingBox3) {
        if other.is_valid() {
            self.extend_point(other.min);
            self.extend_point(other.max);
        }
    }

    /// 有効なBBoxかどうか
    pub fn is_valid(&self) -> bool {
        self.min.x <= self.max.x && self.min.y <= self.max.y && self.min.z <= self.max.z
    }

    /// 中心点
    pub fn center(&self) -> Point3 {
        Point3::from((self.min.coords + self.max.coords) * 0.5)
    }

    /// 各軸のサイズ（幅・奥行き・高さ）
    pub fn size(&self) -> Vec3 {
        self.max - self.min
    }

    /// 対角線の長さ
    pub fn diagonal(&self) -> f64 {
        self.size().norm()
    }

    /// 点を包含しているか
    pub fn contains_point(&self, p: Point3, tol: f64) -> bool {
        p.x >= self.min.x - tol
            && p.x <= self.max.x + tol
            && p.y >= self.min.y - tol
            && p.y <= self.max.y + tol
            && p.z >= self.min.z - tol
            && p.z <= self.max.z + tol
    }

    /// 別のBBoxと交差しているか
    pub fn intersects(&self, other: &BoundingBox3, tol: f64) -> bool {
        self.min.x - tol <= other.max.x
            && self.max.x + tol >= other.min.x
            && self.min.y - tol <= other.max.y
            && self.max.y + tol >= other.min.y
            && self.min.z - tol <= other.max.z
            && self.max.z + tol >= other.min.z
    }
}
