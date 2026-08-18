use serde::{Deserialize, Serialize};
use std::sync::atomic::{AtomicU64, Ordering};
use zenith_math::{Point3, Point3Ext, Tolerance};

static VERTEX_ID_GEN: AtomicU64 = AtomicU64::new(1);

/// B-Rep 頂点（Vertex）
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct Vertex {
    pub id: u64,
    pub point: Point3,
    pub tolerance: f64,
}

impl Vertex {
    pub fn new(point: Point3, tolerance: f64) -> Self {
        Self {
            id: VERTEX_ID_GEN.fetch_add(1, Ordering::Relaxed),
            point,
            tolerance,
        }
    }

    /// デフォルト公差で作成
    pub fn from_point(point: Point3) -> Self {
        Self::new(point, 1e-6)
    }

    /// 別の頂点と公差内で一致しているか
    pub fn is_same_as(&self, other: &Self, tol: &Tolerance) -> bool {
        let max_tol = self.tolerance.max(other.tolerance).max(tol.linear);
        self.point.is_coincident_with(&other.point, max_tol)
    }
}
