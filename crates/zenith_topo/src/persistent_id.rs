use crate::{Face, FaceGeometry};
use serde::{Deserialize, Serialize};
use zenith_math::{Point3, Vec3};

/// トポロジー要素（Face/Edge）の生成元セマンティックタグ
#[derive(Debug, Clone, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub enum SemanticTag {
    /// 押し出し・直方体の天面 (+Z)
    CapTop,
    /// 押し出し・直方体の底面 (-Z)
    CapBottom,
    /// 押し出し・直方体の側面（インデックスまたはプロファイル由来）
    SideFace(usize),
    /// エッジ間のフィレットブレンド面（隣接する2つのFace名）
    FilletBlend(String, String),
    /// エッジ間の面取り斜面（隣接する2つのFace名）
    ChamferFace(String, String),
    /// 穴の内壁面
    HoleInterior(String),
    /// 厚み付け側面
    ThickenSide(usize),
    /// 自由曲面パッチ
    FreeformSurface,
    /// 汎用カスタムタグ
    Custom(String),
}

/// 永続的トポロジー識別子（TNP解決用）
#[derive(Debug, Clone, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub struct PersistentId {
    /// この要素を生成したフィーチャー名（例: "Extrude_1", "Box_1"）
    pub feature_id: String,
    /// フィーチャー内でのセマンティックな役割
    pub tag: SemanticTag,
    /// 世代・派生番号
    pub generation: u32,
}

impl PersistentId {
    pub fn new(feature_id: &str, tag: SemanticTag) -> Self {
        Self {
            feature_id: feature_id.to_string(),
            tag,
            generation: 0,
        }
    }

    pub fn to_string_id(&self) -> String {
        format!("{}.{:?}_{}", self.feature_id, self.tag, self.generation)
    }
}

/// トポロジー要素の幾何シグネチャ（自己修復・類似度マッチング用）
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct GeometricSignature {
    pub surface_type: String,
    pub normal: Vec3,
    pub centroid: Point3,
    pub area_hint: f64,
}

impl GeometricSignature {
    /// Faceから幾何シグネチャを抽出
    pub fn from_face(face: &Face) -> Self {
        let (surface_type, normal, origin) = match &face.geometry {
            FaceGeometry::Plane(plane) => {
                ("Plane".to_string(), plane.normal.normalize(), plane.origin)
            }
            FaceGeometry::Nurbs(nurbs) => (
                "Nurbs".to_string(),
                nurbs.normal(0.5, 0.5).unwrap_or(Vec3::new(0.0, 0.0, 1.0)),
                nurbs.evaluate(0.5, 0.5),
            ),
            FaceGeometry::Coons(_) => (
                "Coons".to_string(),
                Vec3::new(0.0, 0.0, 1.0),
                Point3::new(0.0, 0.0, 0.0),
            ),
            FaceGeometry::Gordon(_) => (
                "Gordon".to_string(),
                Vec3::new(0.0, 0.0, 1.0),
                Point3::new(0.0, 0.0, 0.0),
            ),
            FaceGeometry::Triangular(_) => (
                "Triangular".to_string(),
                Vec3::new(0.0, 0.0, 1.0),
                Point3::new(0.0, 0.0, 0.0),
            ),
        };

        Self {
            surface_type,
            normal,
            centroid: origin,
            area_hint: 1.0,
        }
    }

    /// 2つのシグネチャ間の類似度スコア（0.0〜1.0、1.0が完全一致）
    pub fn similarity(&self, other: &GeometricSignature) -> f64 {
        if self.surface_type != other.surface_type {
            return 0.0;
        }

        // 法線の一致度 (dot product: -1..1 -> 0..1)
        let dot = self.normal.dot(&other.normal).clamp(-1.0, 1.0);
        let normal_score = (dot + 1.0) * 0.5;

        // 重心距離の近さスコア
        let dist = (self.centroid - other.centroid).norm();
        let dist_score = (-dist * 0.01).exp();

        normal_score * 0.7 + dist_score * 0.3
    }
}

/// 幾何シグネチャに基づく自己修復マッチャー
pub struct GeometricMatcher;

impl GeometricMatcher {
    /// 候補Face群の中から、ターゲットシグネチャに最も一致するFaceのインデックスを探索
    pub fn find_best_matching_face(
        target: &GeometricSignature,
        candidate_faces: &[Face],
    ) -> Option<(usize, f64)> {
        let mut best_idx = None;
        let mut highest_score = 0.0;

        for (idx, face) in candidate_faces.iter().enumerate() {
            let sig = GeometricSignature::from_face(face);
            let score = target.similarity(&sig);
            if score > highest_score && score > 0.5 {
                highest_score = score;
                best_idx = Some(idx);
            }
        }

        best_idx.map(|idx| (idx, highest_score))
    }
}
