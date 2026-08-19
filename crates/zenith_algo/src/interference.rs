//! Zenith Algo: アセンブリ干渉・衝突判定エンジン (Interference / Clash Detection)
//! 2つのB-Repソリッド間の空間干渉、めり込み、接触、最小クリアランスの判定。

use zenith_math::{Point3, Tolerance};
use zenith_topo::Solid;

/// 干渉判定の結果種別
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ClashStatus {
    /// 完全に離れている（干渉なし）
    Clearance,
    /// 表面同士が接触している（公差内で接している）
    Touching,
    /// 立体同士がめり込んでいる（干渉あり）
    Clash,
}

/// 干渉判定の詳細レポート
#[derive(Debug, Clone, PartialEq)]
pub struct InterferenceReport {
    /// 干渉状態
    pub status: ClashStatus,
    /// 2ソリッド間の最小距離 (mm) (Clashの場合は 0.0)
    pub min_distance: f64,
    /// 重複AABBバウンディングボックスの体積 (mm^3) (概算干渉ボリューム)
    pub overlap_volume: f64,
    /// 交差判定メッセージ
    pub message: String,
}

pub struct InterferenceChecker;

impl InterferenceChecker {
    /// 2つのソリッド間の干渉・クリアランスを判定
    pub fn check(solid_a: &Solid, solid_b: &Solid, tol: &Tolerance) -> InterferenceReport {
        let (min_a, max_a) = Self::compute_solid_bbox(solid_a);
        let (min_b, max_b) = Self::compute_solid_bbox(solid_b);

        // 1. AABB 重複判定 (Broad Phase)
        let overlap_min_x = min_a.x.max(min_b.x);
        let overlap_max_x = max_a.x.min(max_b.x);
        let overlap_min_y = min_a.y.max(min_b.y);
        let overlap_max_y = max_a.y.min(max_b.y);
        let overlap_min_z = min_a.z.max(min_b.z);
        let overlap_max_z = max_a.z.min(max_b.z);

        let dx = overlap_max_x - overlap_min_x;
        let dy = overlap_max_y - overlap_min_y;
        let dz = overlap_max_z - overlap_min_z;

        if dx < -tol.linear || dy < -tol.linear || dz < -tol.linear {
            let dist_x = (min_b.x - max_a.x).max(min_a.x - max_b.x).max(0.0);
            let dist_y = (min_b.y - max_a.y).max(min_a.y - max_b.y).max(0.0);
            let dist_z = (min_b.z - max_a.z).max(min_a.z - max_b.z).max(0.0);
            let min_dist = (dist_x * dist_x + dist_y * dist_y + dist_z * dist_z).sqrt();

            return InterferenceReport {
                status: ClashStatus::Clearance,
                min_distance: min_dist,
                overlap_volume: 0.0,
                message: format!("Solids are separated by at least {min_dist:.3} mm"),
            };
        }

        if dx.abs() <= tol.linear || dy.abs() <= tol.linear || dz.abs() <= tol.linear {
            return InterferenceReport {
                status: ClashStatus::Touching,
                min_distance: 0.0,
                overlap_volume: 0.0,
                message: "Solids are touching at bounding box boundaries".to_string(),
            };
        }

        let overlap_volume = (dx.max(0.0)) * (dy.max(0.0)) * (dz.max(0.0));

        InterferenceReport {
            status: ClashStatus::Clash,
            min_distance: 0.0,
            overlap_volume,
            message: format!("Solids are clashing with overlap bounding volume ~{overlap_volume:.2} mm^3"),
        }
    }

    fn compute_solid_bbox(solid: &Solid) -> (Point3, Point3) {
        let mut min_pt = Point3::new(f64::INFINITY, f64::INFINITY, f64::INFINITY);
        let mut max_pt = Point3::new(f64::NEG_INFINITY, f64::NEG_INFINITY, f64::NEG_INFINITY);

        let all_faces = solid.outer_shell.faces.iter().chain(
            solid.inner_shells.iter().flat_map(|s| s.faces.iter())
        );

        for face in all_faces {
            for oe in &face.outer_wire.edges {
                for pt in [&oe.start_vertex().point, &oe.end_vertex().point] {
                    min_pt.x = min_pt.x.min(pt.x);
                    min_pt.y = min_pt.y.min(pt.y);
                    min_pt.z = min_pt.z.min(pt.z);
                    max_pt.x = max_pt.x.max(pt.x);
                    max_pt.y = max_pt.y.max(pt.y);
                    max_pt.z = max_pt.z.max(pt.z);
                }
            }
        }

        if min_pt.x.is_infinite() {
            (Point3::origin(), Point3::origin())
        } else {
            (min_pt, max_pt)
        }
    }
}
