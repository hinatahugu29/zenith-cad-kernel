//! Zenith Algo: フランジモデリングビルダー (Flange Builder)
//!
//! 配管継手・モータマウント・軸受ハウジング等で使用されるPCD等配ボルト穴付き円形フランジを生成します。

use std::f64::consts::PI;
use zenith_math::{Tolerance, Vec3};
use zenith_topo::Solid;

pub struct FlangeBuilder;

impl FlangeBuilder {
    /// PCD等配ボルト穴付き円形フランジソリッドの生成
    ///
    /// `outer_radius`: フランジ外径半径 R_outer
    /// `thickness`: フランジ厚み T
    /// `center_hole_radius`: 中心貫通穴の半径 R_center
    /// `pcd_radius`: ボルト穴ピッチ円半径 R_pcd
    /// `num_bolt_holes`: ボルト穴の個数 N (N >= 1)
    /// `bolt_hole_radius`: ボルト穴の半径 r_bolt
    pub fn make_circular_flange(
        outer_radius: f64,
        thickness: f64,
        center_hole_radius: f64,
        pcd_radius: f64,
        num_bolt_holes: usize,
        bolt_hole_radius: f64,
    ) -> Result<Solid, String> {
        if outer_radius <= 1e-9 || thickness <= 1e-9 || center_hole_radius <= 1e-9 {
            return Err("Flange dimensions must be positive".to_string());
        }
        if center_hole_radius >= outer_radius {
            return Err("Center hole radius must be smaller than outer radius".to_string());
        }
        if num_bolt_holes > 0 {
            if bolt_hole_radius <= 1e-9 || pcd_radius <= 1e-9 {
                return Err("Bolt hole dimensions must be positive".to_string());
            }
            if pcd_radius - bolt_hole_radius <= center_hole_radius {
                return Err("Bolt holes intersect center hole".to_string());
            }
            if pcd_radius + bolt_hole_radius >= outer_radius {
                return Err("Bolt holes intersect outer boundary".to_string());
            }
        }

        let tol = Tolerance::default();

        // 1. ベース外径円柱
        let outer_cyl = crate::PrimitiveBuilder::make_cylinder(outer_radius, thickness)?;

        // 2. 中心穴の差分切削
        let center_drill = crate::PrimitiveBuilder::make_cylinder(center_hole_radius, thickness + 2.0)?;
        let center_drill = crate::BrepTransform::translate_solid(
            &center_drill,
            Vec3::new(0.0, 0.0, -1.0),
        );
        let base_ring = crate::BooleanEngine::boolean_solids_exact(
            &outer_cyl,
            &center_drill,
            crate::BooleanOpType::Difference,
            &tol,
        )?;

        if num_bolt_holes == 0 {
            return Ok(base_ring);
        }

        // 3. PCD等配ボルト穴ツールの生成
        let mut bolt_cutters = Vec::with_capacity(num_bolt_holes);
        let d_theta = 2.0 * PI / num_bolt_holes as f64;

        for i in 0..num_bolt_holes {
            let theta = i as f64 * d_theta;
            let cx = pcd_radius * theta.cos();
            let cy = pcd_radius * theta.sin();

            let bolt_cyl = crate::PrimitiveBuilder::make_cylinder(bolt_hole_radius, thickness + 2.0)?;
            let bolt_cyl = crate::BrepTransform::translate_solid(
                &bolt_cyl,
                Vec3::new(cx, cy, -1.0),
            );
            bolt_cutters.push(bolt_cyl);
        }

        // 4. 一括バッチブーリアン差分
        crate::BooleanEngine::boolean_solids_batch(
            &base_ring,
            &bolt_cutters,
            crate::BooleanOpType::Difference,
            &tol,
        )
    }
}
