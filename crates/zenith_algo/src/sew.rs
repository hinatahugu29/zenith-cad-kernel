//! 面の集まりを、稜を実体として共有した B-Rep に縫い合わせる。
//!
//! ## なぜ要るのか
//!
//! シェルが「閉じている」ことは、隣り合う面の境界が**座標として**一致して
//! いれば成り立ちます。ブーリアンの面片組み立ては面を1枚ずつ作るので、
//! 同じ位置に2本の別々の `Edge`（別の ID、別の `Vertex`）が並んだ状態でも
//! 閉性の検査は通ります。
//!
//! 通ってしまうと、そこから先が全部使えません。
//!
//! - 稜を選んでフィレット・面取りする演算子は「この稜を共有する2面」を
//!   引けない（どの稜も1面からしか参照されていない）
//! - 稜 ID に紐づく履歴（TNP・永続 ID）が次の演算で切れる
//! - STEP に書くとき、同じ稜に2つの実体が出る
//!
//! ここはその後片付けを1回で行います。座標が公差内で一致する頂点を1つに
//! 束ね、**形も一致する**稜を1本に束ね、各面のワイヤをその共有稜で
//! 張り直します。向きが逆に格納されていた側は、参照の向きを反転させて
//! 同じ実体を指すようにします。
//!
//! ## 束ねない場合
//!
//! 端点が同じでも途中の形が違う稜（同じ2点を結ぶ円弧と直線など）は
//! 別物なので束ねません。束ねた結果それでも2面から参照されない稜が
//! 残ったときは、黙って直さずレポートに数を残します。

use std::collections::{BTreeMap, HashMap};

use zenith_math::{Point3, Tolerance};
use zenith_topo::{Edge, Face, OrientedEdge, Shell, Solid, Vertex, Wire};

/// 縫い合わせで何が起きたか
#[derive(Debug, Clone, Default, PartialEq, Eq)]
pub struct SewReport {
    /// 束ねる前の稜の実体数
    pub edges_before: usize,
    /// 束ねた後の稜の実体数
    pub edges_after: usize,
    /// 束ねる前の頂点の実体数
    pub vertices_before: usize,
    /// 束ねた後の頂点の実体数
    pub vertices_after: usize,
    /// ちょうど2つの面から参照されている稜の数
    pub shared_edges: usize,
    /// 1面からしか参照されていない稜の数（0 でなければ縫い残し）
    pub boundary_edges: usize,
    /// 3面以上から参照されている稜の数
    pub non_manifold_edges: usize,
}

impl SewReport {
    /// すべての稜がちょうど2面に共有されているか
    pub fn is_watertight(&self) -> bool {
        self.boundary_edges == 0 && self.non_manifold_edges == 0
    }

    pub fn summary(&self) -> String {
        format!(
            "edges {} -> {} (shared {}, boundary {}, non-manifold {}), vertices {} -> {}",
            self.edges_before,
            self.edges_after,
            self.shared_edges,
            self.boundary_edges,
            self.non_manifold_edges,
            self.vertices_before,
            self.vertices_after
        )
    }
}

/// 面の集まりを共有稜の B-Rep に縫い合わせる
pub struct Sewer;

impl Sewer {
    /// ソリッドの外殻と空洞シェルをそれぞれ縫い合わせる
    pub fn sew_solid(solid: &Solid, tol: &Tolerance) -> Result<(Solid, SewReport), String> {
        let (outer, mut report) = Self::sew_shell(&solid.outer_shell, tol);
        let mut inners = Vec::with_capacity(solid.inner_shells.len());
        for shell in &solid.inner_shells {
            let (sewn, inner_report) = Self::sew_shell(shell, tol);
            report.merge(&inner_report);
            inners.push(sewn);
        }

        let solid = Solid::try_new(outer, inners, tol).map_err(|err| err.to_string())?;
        Ok((solid, report))
    }

    /// 1つのシェルを縫い合わせる。閉性の検査はしないので、開いたシェルにも掛かる。
    pub fn sew_shell(shell: &Shell, tol: &Tolerance) -> (Shell, SewReport) {
        let mut vertices = VertexPool::new(tol.linear);
        let mut edges: Vec<CanonicalEdge> = Vec::new();
        // 元の稜 ID -> (共有稜の添字, 向きが元と同じか)
        let mut mapping: BTreeMap<u64, (usize, bool)> = BTreeMap::new();
        let mut before_edges: BTreeMap<u64, ()> = BTreeMap::new();
        let mut before_vertices: BTreeMap<u64, ()> = BTreeMap::new();

        for face in &shell.faces {
            for wire in std::iter::once(&face.outer_wire).chain(face.inner_wires.iter()) {
                for oriented in &wire.edges {
                    let edge = &oriented.edge;
                    before_edges.insert(edge.id, ());
                    before_vertices.insert(edge.start_vertex.id, ());
                    before_vertices.insert(edge.end_vertex.id, ());

                    if mapping.contains_key(&edge.id) {
                        continue;
                    }

                    let start = vertices.intern(&edge.start_vertex);
                    let end = vertices.intern(&edge.end_vertex);
                    let samples = interior_samples(edge);

                    let mut matched = None;
                    for (index, candidate) in edges.iter().enumerate() {
                        if let Some(same_direction) =
                            candidate.matches(start, end, &samples, tol.linear)
                        {
                            matched = Some((index, same_direction));
                            break;
                        }
                    }

                    match matched {
                        Some((index, same_direction)) => {
                            mapping.insert(edge.id, (index, same_direction));
                        }
                        None => {
                            edges.push(CanonicalEdge {
                                start,
                                end,
                                samples,
                                edge: edge.clone(),
                                use_count: 0,
                            });
                            mapping.insert(edge.id, (edges.len() - 1, true));
                        }
                    }
                }
            }
        }

        // 共有稜の端点を、束ねた後の頂点実体で作り直す
        for canonical in edges.iter_mut() {
            let start = vertices.representative(canonical.start);
            let end = vertices.representative(canonical.end);
            // **曲線の端も、束ねた頂点へ一緒に動かします**（4-208）。
            // 頂点だけ差し替えると、曲線の端がそこから 1e-7 の桁で離れ、
            // 境界の標本を曲線から取る側（テッセレーション）が継ぎ目に
            // 「同じはずの点」を2つ作ります。
            canonical.edge = canonical.edge.with_vertices(start, end);
        }

        // 面のワイヤを共有稜で張り直す
        let mut faces = Vec::with_capacity(shell.faces.len());
        for face in &shell.faces {
            let rebuild = |wire: &Wire, edges: &mut Vec<CanonicalEdge>| -> Wire {
                let rebuilt = wire
                    .edges
                    .iter()
                    .map(|oriented| {
                        let (index, same_direction) = mapping[&oriented.edge.id];
                        edges[index].use_count += 1;
                        let orientation = if same_direction {
                            oriented.orientation
                        } else {
                            oriented.orientation.reversed()
                        };
                        OrientedEdge::new(edges[index].edge.clone(), orientation)
                    })
                    .collect();
                Wire::new(rebuilt)
            };

            let outer = rebuild(&face.outer_wire, &mut edges);
            let inners: Vec<Wire> = face
                .inner_wires
                .iter()
                .map(|wire| rebuild(wire, &mut edges))
                .collect();

            faces.push(Face::new(
                face.geometry.clone(),
                outer,
                inners,
                face.orientation,
                face.tolerance,
            ));
        }

        let report = SewReport {
            edges_before: before_edges.len(),
            edges_after: edges.len(),
            vertices_before: before_vertices.len(),
            vertices_after: vertices.count(),
            shared_edges: edges.iter().filter(|e| e.use_count == 2).count(),
            boundary_edges: edges.iter().filter(|e| e.use_count < 2).count(),
            non_manifold_edges: edges.iter().filter(|e| e.use_count > 2).count(),
        };

        (Shell::new(faces, shell.is_closed), report)
    }
}

impl SewReport {
    fn merge(&mut self, other: &SewReport) {
        self.edges_before += other.edges_before;
        self.edges_after += other.edges_after;
        self.vertices_before += other.vertices_before;
        self.vertices_after += other.vertices_after;
        self.shared_edges += other.shared_edges;
        self.boundary_edges += other.boundary_edges;
        self.non_manifold_edges += other.non_manifold_edges;
    }
}

/// 束ねた稜1本ぶん
struct CanonicalEdge {
    start: usize,
    end: usize,
    /// 始点から終点へ向かう向きで取った内部標本
    samples: Vec<Point3>,
    edge: Edge,
    use_count: usize,
}

impl CanonicalEdge {
    /// 同じ稜なら「向きが揃っているか」を返す。別の稜なら `None`。
    fn matches(&self, start: usize, end: usize, samples: &[Point3], tol: f64) -> Option<bool> {
        if samples.len() != self.samples.len() {
            return None;
        }
        if self.start == start && self.end == end {
            let same = self
                .samples
                .iter()
                .zip(samples.iter())
                .all(|(a, b)| (a - b).norm() <= tol);
            if same {
                return Some(true);
            }
        }
        if self.start == end && self.end == start {
            let same = self
                .samples
                .iter()
                .zip(samples.iter().rev())
                .all(|(a, b)| (a - b).norm() <= tol);
            if same {
                return Some(false);
            }
        }
        None
    }
}

/// 位置が公差内で一致する頂点を1つに束ねる
struct VertexPool {
    tolerance: f64,
    cell: f64,
    buckets: HashMap<(i64, i64, i64), Vec<usize>>,
    points: Vec<Point3>,
    representatives: Vec<Vertex>,
}

impl VertexPool {
    fn new(tolerance: f64) -> Self {
        let tolerance = tolerance.max(1e-12);
        Self {
            tolerance,
            cell: tolerance * 2.0,
            buckets: HashMap::new(),
            points: Vec::new(),
            representatives: Vec::new(),
        }
    }

    fn key(&self, point: Point3) -> (i64, i64, i64) {
        (
            (point.x / self.cell).floor() as i64,
            (point.y / self.cell).floor() as i64,
            (point.z / self.cell).floor() as i64,
        )
    }

    fn intern(&mut self, vertex: &Vertex) -> usize {
        let point = vertex.point;
        let (kx, ky, kz) = self.key(point);
        for dx in -1..=1 {
            for dy in -1..=1 {
                for dz in -1..=1 {
                    if let Some(indices) = self.buckets.get(&(kx + dx, ky + dy, kz + dz)) {
                        for index in indices {
                            if (self.points[*index] - point).norm() <= self.tolerance {
                                return *index;
                            }
                        }
                    }
                }
            }
        }

        let index = self.points.len();
        self.points.push(point);
        self.representatives.push(vertex.clone());
        self.buckets.entry((kx, ky, kz)).or_default().push(index);
        index
    }

    fn representative(&self, index: usize) -> Vertex {
        self.representatives[index].clone()
    }

    fn count(&self) -> usize {
        self.points.len()
    }
}

/// 稜の形を比べるための内部標本。端点は既に束ねてあるので取らない。
fn interior_samples(edge: &Edge) -> Vec<Point3> {
    const STEPS: usize = 5;
    (1..STEPS)
        .map(|step| edge.curve.evaluate(step as f64 / STEPS as f64))
        .collect()
}
