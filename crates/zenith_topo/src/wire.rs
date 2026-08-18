use crate::edge::OrientedEdge;
use crate::vertex::Vertex;
use serde::{Deserialize, Serialize};
use std::sync::atomic::{AtomicU64, Ordering};
use zenith_math::{Point3, Tolerance};

static WIRE_ID_GEN: AtomicU64 = AtomicU64::new(1);

/// B-Rep ワイヤ（Wire: エッジの連続した列）
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct Wire {
    pub id: u64,
    pub edges: Vec<OrientedEdge>,
}

impl Wire {
    pub fn new(edges: Vec<OrientedEdge>) -> Self {
        Self {
            id: WIRE_ID_GEN.fetch_add(1, Ordering::Relaxed),
            edges,
        }
    }

    /// ワイヤが閉じているか検証（各エッジの終点が次のエッジの始点と一致し、最後の終点が最初の始点と一致するか）
    pub fn is_closed(&self, tol: &Tolerance) -> bool {
        if self.edges.is_empty() {
            return false;
        }
        for i in 0..self.edges.len() {
            let next_idx = (i + 1) % self.edges.len();
            let current_end = self.edges[i].end_vertex();
            let next_start = self.edges[next_idx].start_vertex();
            if !current_end.is_same_as(next_start, tol) {
                return false;
            }
        }
        true
    }

    /// ワイヤ内の全頂点リスト（重複なし）を取得
    pub fn vertices(&self) -> Vec<Vertex> {
        let mut verts = Vec::new();
        for e in &self.edges {
            verts.push(e.start_vertex().clone());
        }
        verts
    }

    /// メッシュ化向けに、各エッジの曲線形状を反映した境界点列を取得
    pub fn sample_points(&self, curve_segments: usize) -> Vec<Point3> {
        let mut points = Vec::new();

        for (idx, edge) in self.edges.iter().enumerate() {
            for point in edge.sample_points(curve_segments, idx == 0) {
                let is_duplicate = points
                    .last()
                    .map(|last: &Point3| (point - *last).norm() <= 1e-9)
                    .unwrap_or(false);
                if !is_duplicate {
                    points.push(point);
                }
            }
        }

        if points.len() > 1 {
            let first = points[0];
            let last = *points.last().unwrap();
            if (last - first).norm() <= 1e-9 {
                points.pop();
            }
        }

        points
    }
}
