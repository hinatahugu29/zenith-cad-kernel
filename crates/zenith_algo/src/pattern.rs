use zenith_math::{Point3, Vec3, Vec3Ext};
use zenith_topo::{Shape, Solid};

/// 配列複写（Pattern）モデリングアルゴリズム
pub struct PatternBuilder;

impl PatternBuilder {
    /// 直線パターン複写（Linear Pattern）
    /// `solid`: 原本ソリッド, `dir`: 配列方向ベクトル, `spacing`: インスタンス間隔, `count`: 複製総数（原本含む >= 1）
    pub fn linear_pattern(
        solid: &Solid,
        dir: Vec3,
        spacing: f64,
        count: usize,
    ) -> Result<Vec<Solid>, String> {
        if count == 0 {
            return Err("Pattern count must be at least 1".to_string());
        }
        let dir_norm = dir
            .try_normalize_safe(1e-12)
            .ok_or("Pattern direction is zero")?;

        let mut instances = Vec::with_capacity(count);
        for i in 0..count {
            let offset = dir_norm * (spacing * i as f64);
            let transformed = crate::BrepTransform::translate_solid(solid, offset);
            instances.push(transformed);
        }
        Ok(instances)
    }

    /// 円形パターン複写（Circular Pattern）
    /// `solid`: 原本ソリッド, `axis_origin`: 回転軸上の点, `axis_dir`: 回転軸ベクトル, `total_angle_rad`: 配列角度範囲, `count`: 複製総数
    pub fn circular_pattern(
        solid: &Solid,
        axis_origin: Point3,
        axis_dir: Vec3,
        total_angle_rad: f64,
        count: usize,
    ) -> Result<Vec<Solid>, String> {
        if count == 0 {
            return Err("Pattern count must be at least 1".to_string());
        }
        let axis_dir_norm = axis_dir
            .try_normalize_safe(1e-12)
            .ok_or("Axis direction is zero")?;

        let d_theta = if count == 1 {
            0.0
        } else if (total_angle_rad - std::f64::consts::TAU).abs() < 1e-6 {
            total_angle_rad / count as f64
        } else {
            total_angle_rad / (count - 1) as f64
        };

        let mut instances = Vec::with_capacity(count);
        for i in 0..count {
            let angle = d_theta * i as f64;
            if angle.abs() < 1e-12 {
                instances.push(solid.clone());
            } else {
                let t_neg = zenith_math::Transform3::from_translation(-axis_origin.coords);
                let rot = zenith_math::Transform3::from_axis_angle(&axis_dir_norm, angle);
                let t_pos = zenith_math::Transform3::from_translation(axis_origin.coords);
                let combined = zenith_math::Transform3 {
                    matrix: t_pos.matrix * rot.matrix * t_neg.matrix,
                };
                let transformed = crate::BrepTransform::transform_solid(solid, &combined)?;
                instances.push(transformed);
            }
        }
        Ok(instances)
    }

    /// 直線パターンを複合Shape（Compound）として生成
    pub fn linear_pattern_shape(
        solid: &Solid,
        dir: Vec3,
        spacing: f64,
        count: usize,
    ) -> Result<Shape, String> {
        let solids = Self::linear_pattern(solid, dir, spacing, count)?;
        Ok(Shape::compound_solids(solids))
    }

    /// 円形パターンを複合Shape（Compound）として生成
    pub fn circular_pattern_shape(
        solid: &Solid,
        axis_origin: Point3,
        axis_dir: Vec3,
        total_angle_rad: f64,
        count: usize,
    ) -> Result<Shape, String> {
        let solids = Self::circular_pattern(solid, axis_origin, axis_dir, total_angle_rad, count)?;
        Ok(Shape::compound_solids(solids))
    }
}
