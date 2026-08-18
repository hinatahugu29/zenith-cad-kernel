use crate::Solid;
use serde::{Deserialize, Serialize};
use std::sync::atomic::{AtomicU64, Ordering};
use zenith_math::{Point3, Vec3};

static INSTANCE_ID_GEN: AtomicU64 = AtomicU64::new(1);
static ASSEMBLY_ID_GEN: AtomicU64 = AtomicU64::new(1);

/// 4x4 3次元アフィン変換行列
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct Transform3 {
    pub matrix: [[f64; 4]; 4],
}

impl Transform3 {
    /// 単位行列（恒等変換）
    pub fn identity() -> Self {
        Self {
            matrix: [
                [1.0, 0.0, 0.0, 0.0],
                [0.0, 1.0, 0.0, 0.0],
                [0.0, 0.0, 1.0, 0.0],
                [0.0, 0.0, 0.0, 1.0],
            ],
        }
    }

    /// 平行移動
    pub fn translation(dx: f64, dy: f64, dz: f64) -> Self {
        Self {
            matrix: [
                [1.0, 0.0, 0.0, dx],
                [0.0, 1.0, 0.0, dy],
                [0.0, 0.0, 1.0, dz],
                [0.0, 0.0, 0.0, 1.0],
            ],
        }
    }

    /// 3D点の変換
    pub fn transform_point(&self, p: Point3) -> Point3 {
        let x = self.matrix[0][0] * p.x
            + self.matrix[0][1] * p.y
            + self.matrix[0][2] * p.z
            + self.matrix[0][3];
        let y = self.matrix[1][0] * p.x
            + self.matrix[1][1] * p.y
            + self.matrix[1][2] * p.z
            + self.matrix[1][3];
        let z = self.matrix[2][0] * p.x
            + self.matrix[2][1] * p.y
            + self.matrix[2][2] * p.z
            + self.matrix[2][3];
        Point3::new(x, y, z)
    }

    /// 3Dベクトルの変換（平行移動なし）
    pub fn transform_vector(&self, v: Vec3) -> Vec3 {
        let x = self.matrix[0][0] * v.x + self.matrix[0][1] * v.y + self.matrix[0][2] * v.z;
        let y = self.matrix[1][0] * v.x + self.matrix[1][1] * v.y + self.matrix[1][2] * v.z;
        let z = self.matrix[2][0] * v.x + self.matrix[2][1] * v.y + self.matrix[2][2] * v.z;
        Vec3::new(x, y, z)
    }
}

/// アセンブリ内のソリッドインスタンス
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct ComponentInstance {
    pub id: u64,
    pub name: String,
    pub solid: Solid,
    pub transform: Transform3,
}

impl ComponentInstance {
    pub fn new(name: &str, solid: Solid, transform: Transform3) -> Self {
        Self {
            id: INSTANCE_ID_GEN.fetch_add(1, Ordering::Relaxed),
            name: name.to_string(),
            solid,
            transform,
        }
    }
}

/// CAD アセンブリ（マルチボディ・階層構造）
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct Assembly {
    pub id: u64,
    pub name: String,
    pub instances: Vec<ComponentInstance>,
    pub sub_assemblies: Vec<Assembly>,
}

impl Assembly {
    pub fn new(name: &str) -> Self {
        Self {
            id: ASSEMBLY_ID_GEN.fetch_add(1, Ordering::Relaxed),
            name: name.to_string(),
            instances: Vec::new(),
            sub_assemblies: Vec::new(),
        }
    }

    pub fn add_instance(&mut self, instance: ComponentInstance) {
        self.instances.push(instance);
    }

    pub fn add_sub_assembly(&mut self, sub: Assembly) {
        self.sub_assemblies.push(sub);
    }

    /// アセンブリ内の全ソリッドインスタンス数を再帰カウント
    pub fn total_instance_count(&self) -> usize {
        let mut count = self.instances.len();
        for sub in &self.sub_assemblies {
            count += sub.total_instance_count();
        }
        count
    }
}
