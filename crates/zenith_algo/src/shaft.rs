//! Zenith Algo: 段付きシャフト・キー溝モデリングビルダー (Shaft & Keyway Builder)
//!
//! モータ軸・減速機シャフト・スピンドルなどの多段軸ソリッドおよびキー溝加工を提供します。

use zenith_math::{Tolerance, Vec3};
use zenith_topo::Solid;

pub struct ShaftBuilder;

impl ShaftBuilder {
    /// 段付きシャフト（Stepped Shaft）ソリッドの生成
    ///
    /// `sections`: 各段の寸法リスト `&[(radius, length)]` (下から順に積み上げ)
    pub fn make_stepped_shaft(sections: &[(f64, f64)]) -> Result<Solid, String> {
        if sections.is_empty() {
            return Err("Shaft requires at least one section".to_string());
        }
        for (idx, &(r, l)) in sections.iter().enumerate() {
            if r <= 1e-9 || l <= 1e-9 {
                return Err(format!(
                    "Section {idx} dimensions must be positive, got radius={r}, length={l}"
                ));
            }
        }

        let tol = Tolerance::default();

        // 各段の円柱ソリッドを作成
        // 隣接する段同士の境界面で、半径が小さい側の段を大きい側の段の内部へ 0.1mm 食い込ませることで、
        // 外周へのはみ出しをゼロに抑えつつ境界接触を排除して真の交差として安定にブーリアン結合する。
        let n = sections.len();
        let mut cyls: Vec<Solid> = Vec::with_capacity(n);

        let mut current_z = 0.0;
        for i in 0..n {
            let (r, l) = sections[i];

            let extend_bottom = if i > 0 && r <= sections[i - 1].0 { 0.1 } else { 0.0 };
            let extend_top = if i + 1 < n && r < sections[i + 1].0 { 0.1 } else { 0.0 };

            let actual_len = l + extend_bottom + extend_top;
            let cyl = crate::PrimitiveBuilder::make_cylinder(r, actual_len)?;
            let cyl = crate::BrepTransform::translate_solid(
                &cyl,
                Vec3::new(0.0, 0.0, current_z - extend_bottom),
            );

            cyls.push(cyl);
            current_z += l;
        }

        // 全段を順次ブーリアン結合
        let mut result = cyls[0].clone();
        for next_cyl in &cyls[1..] {
            result = crate::BooleanEngine::boolean_solids_exact(
                &result,
                next_cyl,
                crate::BooleanOpType::Union,
                &tol,
            )?;
        }

        Ok(result)
    }

    /// 軸に対する平行キー溝（Keyway）の差分加工
    ///
    /// `shaft`: 対象の軸ソリッド
    /// `shaft_radius`: キー溝加工位置の軸半径
    /// `key_width`: キー溝の幅 W
    /// `key_depth`: キー溝の深さ T（外周面から半径方向内向き）
    /// `key_length`: キー溝の長さ L
    /// `key_z_pos`: キー溝の始点 Z 座標
    pub fn make_shaft_with_keyway(
        shaft: &Solid,
        shaft_radius: f64,
        key_width: f64,
        key_depth: f64,
        key_length: f64,
        key_z_pos: f64,
    ) -> Result<Solid, String> {
        if key_width <= 1e-9 || key_depth <= 1e-9 || key_length <= 1e-9 {
            return Err(format!(
                "Keyway dimensions must be positive, got width={key_width}, depth={key_depth}, length={key_length}"
            ));
        }
        if key_depth >= shaft_radius {
            return Err(format!(
                "Keyway depth ({key_depth}) must be less than shaft radius ({shaft_radius})"
            ));
        }

        let tol = Tolerance::default();

        // キー溝カッター直方体
        // 外周面 y = shaft_radius から半径方向内向きに key_depth 切削
        // カッターを外側（y方向）へ突き出させて確実に抜く
        let cutter_y_len = key_depth + 2.0;
        let cutter = crate::PrimitiveBuilder::make_box(key_width, cutter_y_len, key_length)?;
        let cutter = crate::BrepTransform::translate_solid(
            &cutter,
            Vec3::new(
                -key_width * 0.5,
                shaft_radius - key_depth,
                key_z_pos,
            ),
        );

        crate::BooleanEngine::boolean_solids_exact(
            shaft,
            &cutter,
            crate::BooleanOpType::Difference,
            &tol,
        )
    }
}
