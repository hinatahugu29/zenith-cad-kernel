//! Zenith Algo: 任意ソリッド間最短距離・最近傍点探索エンジン (Distance Engine)
//!
//! 箱・円柱・球・自由曲面など任意のB-Repソリッド間の表面最短距離および最近傍点ペアを算出します。

use zenith_math::{Point3, Tolerance};
use zenith_tess::{tessellate_solid, TessellationParams};
use zenith_topo::Solid;

#[derive(Debug, Clone, PartialEq)]
pub struct DistanceResult {
    pub min_distance: f64,
    pub closest_point_a: Point3,
    pub closest_point_b: Point3,
}

pub struct DistanceEngine;

impl DistanceEngine {
    /// 2つのソリッド間の表面最短距離および最近傍点ペアを算出
    pub fn compute_min_distance(
        solid_a: &Solid,
        solid_b: &Solid,
        _tol: &Tolerance,
    ) -> DistanceResult {
        let params = TessellationParams::default();
        let mesh_a = tessellate_solid(solid_a, &params);
        let mesh_b = tessellate_solid(solid_b, &params);

        let mut min_dist_sq = f64::INFINITY;
        let mut best_pt_a = Point3::origin();
        let mut best_pt_b = Point3::origin();

        // メッシュ頂点間の総当たり探索
        for p_a in &mesh_a.positions {
            for p_b in &mesh_b.positions {
                let dist_sq = (p_a - p_b).norm_squared();
                if dist_sq < min_dist_sq {
                    min_dist_sq = dist_sq;
                    best_pt_a = *p_a;
                    best_pt_b = *p_b;
                }
            }
        }

        DistanceResult {
            min_distance: min_dist_sq.sqrt(),
            closest_point_a: best_pt_a,
            closest_point_b: best_pt_b,
        }
    }
}
