use crate::vertex::Vertex;
use serde::{Deserialize, Serialize};
use std::sync::atomic::{AtomicU64, Ordering};
use zenith_geom::NurbsCurve3;
use zenith_math::{BoundingBox3, Point3};

static EDGE_ID_GEN: AtomicU64 = AtomicU64::new(1);

/// トポロジー要素の向き
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum Orientation {
    Forward,
    Reversed,
}

impl Orientation {
    pub fn is_forward(&self) -> bool {
        matches!(self, Self::Forward)
    }

    pub fn reversed(&self) -> Self {
        match self {
            Self::Forward => Self::Reversed,
            Self::Reversed => Self::Forward,
        }
    }
}

/// B-Rep エッジ（Edge）
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct Edge {
    pub id: u64,
    pub curve: NurbsCurve3,
    pub start_vertex: Vertex,
    pub end_vertex: Vertex,
    pub tolerance: f64,
}

impl Edge {
    pub fn new(
        curve: NurbsCurve3,
        start_vertex: Vertex,
        end_vertex: Vertex,
        tolerance: f64,
    ) -> Self {
        Self {
            id: EDGE_ID_GEN.fetch_add(1, Ordering::Relaxed),
            curve,
            start_vertex,
            end_vertex,
            tolerance,
        }
    }

    /// 直線エッジを2つの頂点から簡易生成
    pub fn line_between(start_vertex: Vertex, end_vertex: Vertex) -> Result<Self, String> {
        let curve =
            NurbsCurve3::bspline_from_points(1, vec![start_vertex.point, end_vertex.point])?;
        Ok(Self::new(curve, start_vertex, end_vertex, 1e-6))
    }

    /// パラメータ t における3次元座標
    pub fn evaluate(&self, t: f64) -> Point3 {
        self.curve.evaluate(t)
    }

    /// エッジの軸平行バウンディングボックス (AABB) を計算
    pub fn bounding_box(&self) -> BoundingBox3 {
        let mut bbox = BoundingBox3::from_point(self.start_vertex.point);
        bbox.extend_point(self.end_vertex.point);
        for cp in &self.curve.control_points {
            bbox.extend_point(cp.point);
        }
        bbox
    }
}

/// 向き情報付きエッジ参照
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct OrientedEdge {
    pub edge: Edge,
    pub orientation: Orientation,
}

impl OrientedEdge {
    pub fn new(edge: Edge, orientation: Orientation) -> Self {
        Self { edge, orientation }
    }

    pub fn forward(edge: Edge) -> Self {
        Self::new(edge, Orientation::Forward)
    }

    pub fn reversed(edge: Edge) -> Self {
        Self::new(edge, Orientation::Reversed)
    }

    /// バウンディングボックス（向きによらずエッジ自体のAABB）
    pub fn bounding_box(&self) -> BoundingBox3 {
        self.edge.bounding_box()
    }

    /// 向きを考慮した始点頂点
    pub fn start_vertex(&self) -> &Vertex {
        if self.orientation.is_forward() {
            &self.edge.start_vertex
        } else {
            &self.edge.end_vertex
        }
    }

    /// 向きを考慮した終点頂点
    pub fn end_vertex(&self) -> &Vertex {
        if self.orientation.is_forward() {
            &self.edge.end_vertex
        } else {
            &self.edge.start_vertex
        }
    }

    /// 向きを考慮して、0.0=start, 1.0=end として曲線上の点を評価
    pub fn evaluate_normalized(&self, t: f64) -> Point3 {
        let clamped_t = t.clamp(0.0, 1.0);
        let (u_min, u_max) = self.edge.curve.param_range();
        let directed_t = if self.orientation.is_forward() {
            clamped_t
        } else {
            1.0 - clamped_t
        };
        let u = u_min + directed_t * (u_max - u_min);
        self.edge.evaluate(u)
    }

    /// 表示・メッシュ化向けに、向きを考慮した点列を生成する。
    pub fn sample_points(&self, curve_segments: usize, include_start: bool) -> Vec<Point3> {
        let segments = if self.is_linear() {
            1
        } else {
            curve_segments.max(2)
        };

        let start_i = if include_start { 0 } else { 1 };
        let mut points = Vec::with_capacity(segments + 1 - start_i);
        for i in start_i..=segments {
            let t = i as f64 / segments as f64;
            points.push(self.evaluate_normalized(t));
        }
        points
    }

    fn is_linear(&self) -> bool {
        self.edge.curve.degree == 1 && self.edge.curve.control_points.len() == 2
    }
}
