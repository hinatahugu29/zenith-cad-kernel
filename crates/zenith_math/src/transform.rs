use crate::point::Point3;
use crate::vector::Vec3;
use serde::{Deserialize, Serialize};

/// 3次元アフィン変換（平行移動・回転・スケール）
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct Transform3 {
    pub matrix: nalgebra::Matrix4<f64>,
}

impl Default for Transform3 {
    fn default() -> Self {
        Self::identity()
    }
}

impl Transform3 {
    /// 単位変換
    pub fn identity() -> Self {
        Self {
            matrix: nalgebra::Matrix4::identity(),
        }
    }

    /// 平行移動
    pub fn from_translation(v: Vec3) -> Self {
        let mut mat = nalgebra::Matrix4::identity();
        mat[(0, 3)] = v.x;
        mat[(1, 3)] = v.y;
        mat[(2, 3)] = v.z;
        Self { matrix: mat }
    }

    /// 均等スケール
    pub fn from_scale(s: f64) -> Self {
        let mut mat = nalgebra::Matrix4::identity();
        mat[(0, 0)] = s;
        mat[(1, 1)] = s;
        mat[(2, 2)] = s;
        Self { matrix: mat }
    }

    /// 軸まわりの回転
    pub fn from_axis_angle(axis: &Vec3, angle_rad: f64) -> Self {
        let axis_unit = nalgebra::Unit::new_normalize(*axis);
        let rot = nalgebra::Rotation3::from_axis_angle(&axis_unit, angle_rad);
        Self {
            matrix: rot.to_homogeneous(),
        }
    }

    /// 点の変換
    pub fn transform_point(&self, p: &Point3) -> Point3 {
        let p4 = self.matrix * p.to_homogeneous();
        Point3::from_homogeneous(p4).unwrap_or(*p)
    }

    /// ベクトルの変換（平行移動は適用しない）
    pub fn transform_vector(&self, v: &Vec3) -> Vec3 {
        let v4 = self.matrix * nalgebra::Vector4::new(v.x, v.y, v.z, 0.0);
        Vec3::new(v4.x, v4.y, v4.z)
    }

    /// 逆変換
    pub fn inverse(&self) -> Option<Self> {
        self.matrix.try_inverse().map(|matrix| Self { matrix })
    }

    /// 変換の合成
    pub fn compose(&self, other: &Self) -> Self {
        Self {
            matrix: self.matrix * other.matrix,
        }
    }
}
