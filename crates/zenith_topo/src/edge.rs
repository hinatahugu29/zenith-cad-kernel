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
        let mut curve = curve;
        snap_curve_ends_to_vertices(&mut curve, &start_vertex, &end_vertex);
        Self {
            id: EDGE_ID_GEN.fetch_add(1, Ordering::Relaxed),
            curve,
            start_vertex,
            end_vertex,
            tolerance,
        }
    }

    /// **番号と公差はそのままに、端点の頂点だけ差し替える。**
    ///
    /// 縫うときに、離れた頂点を1つに束ねてから稜へ入れ直す経路があります
    /// （`sew`）。そこで**頂点だけが動いて曲線が置き去りになる**と、曲線の端が
    /// 頂点から離れます——実測 1.1〜1.3e-7（4-208）。ここを通せば、束ねた
    /// 位置へ曲線の端も一緒に動きます。
    pub fn with_vertices(&self, start_vertex: Vertex, end_vertex: Vertex) -> Self {
        let mut curve = self.curve.clone();
        snap_curve_ends_to_vertices(&mut curve, &start_vertex, &end_vertex);
        Self {
            id: self.id,
            curve,
            start_vertex,
            end_vertex,
            tolerance: self.tolerance,
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

/// 曲線の端が頂点から離れられる距離。**これを超えたら寄せません**——
/// 別の曲線を渡された可能性のほうが高いので、黙って形を変えるほうが危険です。
const END_SNAP_LIMIT: f64 = 1e-5;

/// **曲線の端を、頂点の位置に合わせる**（4-208）。
///
/// 交線から作った稜では、曲線の端が頂点と **1.1〜1.3e-7** ずれることが
/// あります（実測。`edge_end_gap_probe` で掃くと、稜の端 5,292 箇所のうち
/// 54 箇所。`box × cylinder` や `cylinder × cone` にも出ます）。
///
/// **B-Rep としては多様体のままで、恒等式も破れません。** 効くのは
/// 「稜の曲線から点を取る」側です——境界の標本を曲線から取るテッセレーション
/// では、隣り合う2本の稜が継ぎ目に「同じはずの点」を2つ作り、溶接の距離
/// (1e-7) より大きいと束ねられず、そこが穴になりました（4-208）。
///
/// 端を止めた（clamped）曲線では、`evaluate` の端はそのまま最初・最後の
/// 制御点なので、そこを頂点へ動かせば端が一致します。**端が制御点で
/// 決まっていない曲線には触れません**——確かめてから動かします。
fn snap_curve_ends_to_vertices(curve: &mut NurbsCurve3, start: &Vertex, end: &Vertex) {
    let (t_min, t_max) = curve.param_range();
    for (t, target, index) in [
        (t_min, start.point, 0),
        (
            t_max,
            end.point,
            curve.control_points.len().saturating_sub(1),
        ),
    ] {
        let Some(control) = curve.control_points.get(index) else {
            continue;
        };
        let here = curve.evaluate(t);
        let gap = (here - target).norm();
        if gap == 0.0 || gap > END_SNAP_LIMIT {
            continue;
        }
        // **端が制御点そのものか**を確かめてから動かします。
        if (here - control.point).norm() > 1e-12 {
            continue;
        }
        curve.control_points[index].point = target;
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
