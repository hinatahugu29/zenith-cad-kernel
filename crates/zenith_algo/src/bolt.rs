//! Zenith Algo: ボルト・締結要素モデリングビルダー (Bolt Builder)
//!
//! 機械設計で最も頻出する標準六角ボルト・六角穴付きボルトの完全閉多様体B-Repソリッドを生成します。

use zenith_math::{Tolerance, Vec3};
use zenith_topo::Solid;

pub struct BoltBuilder;

impl BoltBuilder {
    /// 六角ボルト（Hex Bolt）ソリッドの生成
    ///
    /// `across_flats`: 頭部二面幅 S（対辺距離、ミリメートル）
    /// `head_thickness`: 頭部の厚み k（高さ、ミリメートル）
    /// `shank_radius`: 軸部の半径 r（呼び径 d の半分、ミリメートル）
    /// `shank_length`: 軸部の長さ L（首下長さ、ミリメートル）
    pub fn make_hex_bolt(
        across_flats: f64,
        head_thickness: f64,
        shank_radius: f64,
        shank_length: f64,
    ) -> Result<Solid, String> {
        if across_flats <= 1e-9
            || head_thickness <= 1e-9
            || shank_radius <= 1e-9
            || shank_length <= 1e-9
        {
            return Err(format!(
                "Bolt dimensions must be positive, got across_flats={across_flats}, head_thickness={head_thickness}, shank_radius={shank_radius}, shank_length={shank_length}"
            ));
        }

        let tol = Tolerance::default();
        // 二面幅 S に対する外接円半径 R = S / sqrt(3)
        let circum_radius = across_flats / 3.0f64.sqrt();

        // 1. 六角ボルト頭部 (z = 0 から z = head_thickness)
        let head = crate::PrimitiveBuilder::make_regular_prism(6, circum_radius, head_thickness)?;

        // 2. 軸部円柱 (z = head_thickness - 0.1 から z = head_thickness + shank_length)
        // 境界接触を排除し真の交差とするため、わずかに頭部内部へ食い込ませて結合
        let shank = crate::PrimitiveBuilder::make_cylinder(shank_radius, shank_length + 0.1)?;
        let shank = crate::BrepTransform::translate_solid(
            &shank,
            Vec3::new(0.0, 0.0, head_thickness - 0.1),
        );

        crate::BooleanEngine::boolean_solids_exact(&head, &shank, crate::BooleanOpType::Union, &tol)
    }
}
