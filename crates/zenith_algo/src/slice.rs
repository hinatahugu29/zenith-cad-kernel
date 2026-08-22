//! Zenith Algo: 断面スライス＆2D輪郭抽出エンジン (Section Slicing)
//! 任意の3D切断平面でB-Repソリッドを切断し、閉じた断面ワイヤループ群と断面特性を抽出。
//!
//! The section is taken against the solid's triangulated faces rather than
//! against face edges alone. Chording between edge crossings turned a circular
//! section into the square inscribed in it; going through the triangles keeps
//! planar faces exact and makes curved faces converge with the tessellation
//! density instead of collapsing to the seam points.

use zenith_math::{Point3, Tolerance, Vec3};
use zenith_tess::{tessellate_solid, TessellationParams, TriangleMesh};
use zenith_topo::{Edge, OrientedEdge, Solid, Vertex, Wire};

/// 断面スライスの結果データ構造
#[derive(Debug, Clone)]
pub struct SectionSliceResult {
    /// 抽出された閉じた断面ワイヤループ群
    pub section_wires: Vec<Wire>,
    /// 総断面積 (mm^2)。穴ループは符号付きで差し引かれる。
    pub total_area: f64,
    /// 断面外周の総周長 (mm)。穴ループの周長も含む。
    pub total_perimeter: f64,
    /// 各ループの符号付き面積 (mm^2)。外周は正、穴は負。
    pub signed_loop_areas: Vec<f64>,
    /// 断面の抽出に使われた三角形分割の細かさ。
    pub tessellation: TessellationParams,
    /// 断面上の点を測れて2次で積んだ弦の本数。
    pub refined_chord_count: usize,
    /// 測れず、弦のまま積んだ本数。ここが 0 でないぶんだけ、面積は
    /// 内側に削れたままになる。
    pub unrefined_chord_count: usize,
    /// 三角形の辺の上に落ちていたのを、断面の上へ載せ直した点の数。
    pub settled_point_count: usize,
}

/// 曲面を含むソリッドで断面精度を上げるための既定の三角形分割。
/// 円柱・球のような有理NURBS面はこの分割数で内接多角形近似されるため、
/// 断面積の相対誤差はおおよそ (pi^2 / 3) / n^2 のオーダーで縮む。
pub const DEFAULT_SECTION_TESSELLATION: TessellationParams = TessellationParams {
    u_divisions: 128,
    v_divisions: 128,
};

pub struct SectionSlicer;

impl SectionSlicer {
    /// 任意の切断平面（原点 origin, 法線 normal）でソリッドを切断し、断面ループ群を抽出
    pub fn slice_solid(
        solid: &Solid,
        plane_origin: Point3,
        plane_normal: Vec3,
        tol: &Tolerance,
    ) -> Result<SectionSliceResult, String> {
        Self::slice_solid_with_tessellation(
            solid,
            plane_origin,
            plane_normal,
            tol,
            &DEFAULT_SECTION_TESSELLATION,
        )
    }

    /// 三角形分割の細かさを指定して断面を抽出する。
    ///
    /// 平面のみで構成されたソリッドでは分割数によらず厳密。曲面を含む場合は
    /// 分割数を上げるほど解析解に収束する。
    pub fn slice_solid_with_tessellation(
        solid: &Solid,
        plane_origin: Point3,
        plane_normal: Vec3,
        tol: &Tolerance,
        tessellation: &TessellationParams,
    ) -> Result<SectionSliceResult, String> {
        let Some(normal) = normalize_or_none(plane_normal) else {
            return Err("Section plane normal must not be zero".to_string());
        };

        let mesh = tessellate_solid(solid, tessellation);
        if mesh.indices.is_empty() {
            return Err("Section slicing requires a tessellatable solid".to_string());
        }

        // 1. 三角形ごとに平面との交線を求め、向き付き線分として集める。
        //    向きは plane_normal x triangle_normal に揃えるので、外殻の外向き
        //    法線からは反時計回りの外周ループが、穴や空洞からは時計回りの
        //    ループが得られる。
        let segments = collect_directed_segments(&mesh, plane_origin, normal, tol);

        if segments.is_empty() {
            let (min_pt, max_pt) = mesh_bounds(&mesh);
            let min_distance = (min_pt - plane_origin).dot(&normal);
            let max_distance = (max_pt - plane_origin).dot(&normal);
            let straddles = min_distance.min(max_distance) < -tol.linear
                && max_distance.max(min_distance) > tol.linear;

            if straddles {
                return Err(
                    "Section plane crosses the solid but produced no intersection segments"
                        .to_string(),
                );
            }

            return Ok(SectionSliceResult {
                section_wires: Vec::new(),
                total_area: 0.0,
                total_perimeter: 0.0,
                signed_loop_areas: Vec::new(),
                tessellation: *tessellation,
                refined_chord_count: 0,
                unrefined_chord_count: 0,
                settled_point_count: 0,
            });
        }

        // 2. 向きを保ったまま端点で連結し、閉じたループだけを採用する。
        let weld_tolerance = weld_tolerance_for(&mesh, tol);
        let chained = chain_directed_segments(&segments, weld_tolerance);

        if !chained.open_chains.is_empty() {
            let longest = chained
                .open_chains
                .iter()
                .map(|chain| chain.len())
                .max()
                .unwrap_or(0);
            return Err(format!(
                "Section slicing produced {} unclosed chain(s) (longest {longest} points) alongside {} closed loop(s); the section outline is incomplete",
                chained.open_chains.len(),
                chained.loops.len()
            ));
        }

        if chained.loops.is_empty() {
            return Err(
                "Section slicing found intersection segments but could not close any loop"
                    .to_string(),
            );
        }

        // 3. 弦の中点が本当に断面の上にあるかを、B-Rep に当てて測る。
        //    メッシュから拾った輪郭は弦の多角形なので、曲面の断面では必ず
        //    内側に削れる。分割数を上げれば減るが、**向きが一方向に決まった**
        //    誤差なので、収束を待つ性質のものではない（面のトリムループで
        //    同じことを分割数から切り離したのと同じ話）。
        let mut refiner = SectionRefiner::new(solid);

        // 4. 平面内の正規直交フレームに射影して符号付き面積を積む。
        let (axis_u, axis_v) = plane_frame(normal);

        let mut section_wires = Vec::new();
        let mut signed_loop_areas = Vec::new();
        let mut total_area = 0.0;
        let mut total_perimeter = 0.0;
        let mut refined_chord_count = 0usize;
        let mut unrefined_chord_count = 0usize;
        let mut settled_point_count = 0usize;

        for loop_points in &chained.loops {
            if loop_points.len() < 3 {
                continue;
            }

            // 輪郭の点は三角形の**辺の上**で平面と交わった位置なので、曲面の
            // 内側に入っている。半径6の穴を96分割で切ると、点の半径が
            // 5.99981 まで落ちる。弦の間を直す前に、点そのものを断面へ載せる。
            let mut owned = loop_points.clone();
            settled_point_count +=
                refiner.settle_loop_points(&mut owned, plane_origin, normal, tol);
            let points = &owned;

            let mut signed_area = signed_area_on_plane(points, plane_origin, axis_u, axis_v);

            let mut wire_edges = Vec::with_capacity(points.len());
            let mut perimeter = 0.0;
            for index in 0..points.len() {
                let start = points[index];
                let end = points[(index + 1) % points.len()];
                let length = (end - start).norm();
                if length <= tol.linear {
                    continue;
                }

                // 弦の中点に対応する断面上の点を測り、あれば二次で積む。
                // 平面だけでできた断面ではここが必ず弦の上に来るので、
                // 補正は 0 になり、箱の断面は今までどおり厳密なままになる。
                match refiner.section_point_between(start, end, plane_origin, normal, tol) {
                    Some(middle) => {
                        signed_area += quadratic_area_gain(
                            start,
                            middle,
                            end,
                            plane_origin,
                            axis_u,
                            axis_v,
                        );
                        perimeter += quadratic_arc_length(start, middle, end);
                        refined_chord_count += 1;
                    }
                    None => {
                        perimeter += length;
                        unrefined_chord_count += 1;
                    }
                }

                let edge = Edge::line_between(Vertex::from_point(start), Vertex::from_point(end))?;
                wire_edges.push(OrientedEdge::forward(edge));
            }

            if wire_edges.len() < 3 {
                continue;
            }

            total_area += signed_area;
            total_perimeter += perimeter;
            signed_loop_areas.push(signed_area);
            section_wires.push(Wire::new(wire_edges));
        }

        if section_wires.is_empty() {
            return Err("Section slicing produced no usable section loop".to_string());
        }

        // 外周が時計回りに出た場合（法線の向き次第）は全体の符号だけ反転する。
        if total_area < 0.0 {
            total_area = -total_area;
            for area in &mut signed_loop_areas {
                *area = -*area;
            }
        }

        if total_area <= 0.0 {
            return Err(format!(
                "Section slicing produced a non-positive area {total_area} across {} loop(s)",
                section_wires.len()
            ));
        }

        Ok(SectionSliceResult {
            section_wires,
            total_area,
            total_perimeter,
            signed_loop_areas,
            tessellation: *tessellation,
            refined_chord_count,
            unrefined_chord_count,
            settled_point_count,
        })
    }
}

fn normalize_or_none(v: Vec3) -> Option<Vec3> {
    let norm = v.norm();
    if norm < 1e-12 {
        return None;
    }
    Some(v / norm)
}

/// Builds an orthonormal in-plane frame whose cross product is the plane
/// normal, so a counter-clockwise loop seen from the normal side has a
/// positive shoelace area.
fn plane_frame(normal: Vec3) -> (Vec3, Vec3) {
    let seed = if normal.x.abs() < 0.9 {
        Vec3::new(1.0, 0.0, 0.0)
    } else {
        Vec3::new(0.0, 1.0, 0.0)
    };
    let axis_u = (seed - normal * seed.dot(&normal)).normalize();
    let axis_v = normal.cross(&axis_u);
    (axis_u, axis_v)
}

fn signed_area_on_plane(points: &[Point3], origin: Point3, axis_u: Vec3, axis_v: Vec3) -> f64 {
    let mut sum = 0.0;
    for index in 0..points.len() {
        let a = points[index] - origin;
        let b = points[(index + 1) % points.len()] - origin;
        let (au, av) = (a.dot(&axis_u), a.dot(&axis_v));
        let (bu, bv) = (b.dot(&axis_u), b.dot(&axis_v));
        sum += au * bv - bu * av;
    }
    sum * 0.5
}

fn mesh_bounds(mesh: &TriangleMesh) -> (Point3, Point3) {
    let mut min_pt = Point3::new(f64::INFINITY, f64::INFINITY, f64::INFINITY);
    let mut max_pt = Point3::new(f64::NEG_INFINITY, f64::NEG_INFINITY, f64::NEG_INFINITY);
    for p in &mesh.positions {
        min_pt.x = min_pt.x.min(p.x);
        min_pt.y = min_pt.y.min(p.y);
        min_pt.z = min_pt.z.min(p.z);
        max_pt.x = max_pt.x.max(p.x);
        max_pt.y = max_pt.y.max(p.y);
        max_pt.z = max_pt.z.max(p.z);
    }
    (min_pt, max_pt)
}

/// Endpoint matching has to tolerate the gap between two adjacent faces that
/// were tessellated independently, so the weld distance is scaled to the model
/// rather than left at the raw linear tolerance.
fn weld_tolerance_for(mesh: &TriangleMesh, tol: &Tolerance) -> f64 {
    let (min_pt, max_pt) = mesh_bounds(mesh);
    let diagonal = (max_pt - min_pt).norm();
    if !diagonal.is_finite() || diagonal <= 0.0 {
        return tol.linear.max(1e-9);
    }
    (diagonal * 1e-6).max(tol.linear)
}

/// One directed intersection segment, oriented so that chained loops come out
/// counter-clockwise around the plane normal for outward-facing material.
#[derive(Debug, Clone, Copy)]
struct DirectedSegment {
    start: Point3,
    end: Point3,
}

fn collect_directed_segments(
    mesh: &TriangleMesh,
    plane_origin: Point3,
    normal: Vec3,
    tol: &Tolerance,
) -> Vec<DirectedSegment> {
    let mut segments = Vec::new();

    for tri in &mesh.indices {
        let p = [
            mesh.positions[tri[0] as usize],
            mesh.positions[tri[1] as usize],
            mesh.positions[tri[2] as usize],
        ];
        // 公差内の距離は 0 に丸めてから符号で分類する。丸めておかないと、
        // 平面がちょうど頂点行を通ったときに上下の三角形が同じ輪郭を
        // 二重に出力してしまう。
        let distance = [
            snap_distance((p[0] - plane_origin).dot(&normal), tol.linear),
            snap_distance((p[1] - plane_origin).dot(&normal), tol.linear),
            snap_distance((p[2] - plane_origin).dot(&normal), tol.linear),
        ];

        let Some(crossing) = triangle_plane_crossing(&p, &distance) else {
            continue;
        };
        let (first, second) = crossing;
        if (second - first).norm() <= tol.linear {
            continue;
        }

        let facet_normal = (p[1] - p[0]).cross(&(p[2] - p[0]));
        let Some(facet_normal) = normalize_or_none(facet_normal) else {
            continue;
        };

        // 断面輪郭の進行方向は plane_normal x facet_normal。外向き法線の面から
        // 反時計回りの外周が得られ、穴の内壁からは時計回りのループになる。
        let expected_direction = normal.cross(&facet_normal);
        let candidate = second - first;
        if candidate.dot(&expected_direction) >= 0.0 {
            segments.push(DirectedSegment {
                start: first,
                end: second,
            });
        } else {
            segments.push(DirectedSegment {
                start: second,
                end: first,
            });
        }
    }

    segments
}

fn snap_distance(distance: f64, linear_tolerance: f64) -> f64 {
    if distance.abs() <= linear_tolerance {
        0.0
    } else {
        distance
    }
}

/// Returns the two points where the plane cuts a triangle.
///
/// Vertices lying on the plane are the awkward part: counted naively, a plane
/// that lands exactly on a tessellation row picks up the same outline from the
/// triangles on both sides and reports twice the area. Each configuration is
/// therefore classified explicitly, and a triangle with one edge in the plane
/// contributes that edge only from its positive side so the segment is emitted
/// exactly once.
fn triangle_plane_crossing(points: &[Point3; 3], distance: &[f64; 3]) -> Option<(Point3, Point3)> {
    let zero_count = distance.iter().filter(|d| **d == 0.0).count();
    let positive_count = distance.iter().filter(|d| **d > 0.0).count();
    let negative_count = distance.iter().filter(|d| **d < 0.0).count();

    match zero_count {
        // 平面上に寝ている三角形は輪郭を定義しない。
        3 => None,

        // 1辺が平面に乗っている。正側の三角形からのみ1回出力する。
        2 => {
            if positive_count != 1 {
                return None;
            }
            let on_plane: Vec<Point3> = (0..3)
                .filter(|index| distance[*index] == 0.0)
                .map(|index| points[index])
                .collect();
            Some((on_plane[0], on_plane[1]))
        }

        // 1頂点が平面上にあり、残り2点が平面を挟む場合のみ輪郭になる。
        1 => {
            if positive_count != 1 || negative_count != 1 {
                return None;
            }
            let vertex_index = (0..3).find(|index| distance[*index] == 0.0)?;
            let other = [(vertex_index + 1) % 3, (vertex_index + 2) % 3];
            let (d0, d1) = (distance[other[0]], distance[other[1]]);
            let t = d0 / (d0 - d1);
            let crossing = points[other[0]] + (points[other[1]] - points[other[0]]) * t;
            Some((points[vertex_index], crossing))
        }

        // 頂点が平面上になければ、素直に2辺を横切る。
        _ => {
            if positive_count == 0 || negative_count == 0 {
                return None;
            }
            let mut hits = Vec::with_capacity(2);
            for index in 0..3 {
                let next = (index + 1) % 3;
                let (d0, d1) = (distance[index], distance[next]);
                if (d0 > 0.0) == (d1 > 0.0) {
                    continue;
                }
                let t = d0 / (d0 - d1);
                hits.push(points[index] + (points[next] - points[index]) * t);
            }
            if hits.len() < 2 {
                return None;
            }
            Some((hits[0], hits[1]))
        }
    }
}

struct ChainedSegments {
    loops: Vec<Vec<Point3>>,
    open_chains: Vec<Vec<Point3>>,
}

/// Walks the directed segments head-to-tail so the loop winding survives the
/// chaining, and keeps closed loops apart from chains that never came back to
/// their start.
fn chain_directed_segments(segments: &[DirectedSegment], weld: f64) -> ChainedSegments {
    let mut used = vec![false; segments.len()];
    let mut loops = Vec::new();
    let mut open_chains = Vec::new();

    for seed_index in 0..segments.len() {
        if used[seed_index] {
            continue;
        }
        used[seed_index] = true;

        let seed = segments[seed_index];
        let mut chain = vec![seed.start, seed.end];
        let mut closed = false;

        loop {
            let current_end = *chain.last().unwrap();

            if chain.len() >= 3 && (current_end - chain[0]).norm() <= weld {
                chain.pop();
                closed = true;
                break;
            }

            let mut next_index = None;
            let mut best_distance = weld;
            for (index, segment) in segments.iter().enumerate() {
                if used[index] {
                    continue;
                }
                let distance = (segment.start - current_end).norm();
                if distance <= best_distance {
                    best_distance = distance;
                    next_index = Some(index);
                }
            }

            let Some(index) = next_index else {
                if chain.len() >= 3 && (current_end - chain[0]).norm() <= weld {
                    chain.pop();
                    closed = true;
                }
                break;
            };

            used[index] = true;
            chain.push(segments[index].end);
        }

        if closed {
            loops.push(chain);
        } else {
            open_chains.push(chain);
        }
    }

    ChainedSegments { loops, open_chains }
}

/// 弦 `a`-`b` と、その間で測った断面上の点 `m` を通る2次曲線が、弦より外に
/// 抱える符号付き面積。
///
/// 2次ベジエ `A, Q, B`（`Q = 2m - (A + B) / 2` なら t=1/2 で `m` を通る）が
/// 弦との間に囲む面積は、三角形 `A Q B` のちょうど 2/3 である。`Q` の弦から
/// の高さは `m` の2倍なので、結局 **三角形 `A m B` の 4/3** になる。
///
/// 半径 `r`、半角 `θ` の円弧では、真の弓形面積が
/// `r²((2/3)θ³ - (2/15)θ⁵)`、この式が `r²((2/3)θ³ - (1/6)θ⁵)` なので、
/// 1弦あたり `r²θ⁵/30` だけ過剰になる。全周では相対 `θ⁴/30` で、既定の
/// 分割ではおよそ 1e-10 である。弦のままだと `θ²` のオーダーで残る。
fn quadratic_area_gain(
    a: Point3,
    m: Point3,
    b: Point3,
    origin: Point3,
    axis_u: Vec3,
    axis_v: Vec3,
) -> f64 {
    let to_uv = |p: Point3| {
        let d = p - origin;
        (d.dot(&axis_u), d.dot(&axis_v))
    };
    let (au, av) = to_uv(a);
    let (mu, mv) = to_uv(m);
    let (bu, bv) = to_uv(b);
    // 三角形 A m B の符号付き面積
    let triangle = 0.5 * ((mu - au) * (bv - av) - (bu - au) * (mv - av));
    triangle * 4.0 / 3.0
}

/// 同じ2次曲線の弧長。5点ガウス・ルジャンドルで、放物線そのものについては
/// 機械精度で積める。
fn quadratic_arc_length(a: Point3, m: Point3, b: Point3) -> f64 {
    // t=1/2 で m を通る2次ベジエの制御点
    let q = Point3::from((m.coords * 2.0) - (a.coords + b.coords) * 0.5);
    // C'(t) = 2(1-t)(Q-A) + 2t(B-Q)
    let d0 = q - a;
    let d1 = b - q;
    const NODES: [(f64, f64); 5] = [
        (0.0, 0.568_888_888_888_888_9),
        (-0.538_469_310_105_683_1, 0.478_628_670_499_366_5),
        (0.538_469_310_105_683_1, 0.478_628_670_499_366_5),
        (-0.906_179_845_938_664_0, 0.236_926_885_056_189_1),
        (0.906_179_845_938_664_0, 0.236_926_885_056_189_1),
    ];
    let mut length = 0.0;
    for (node, weight) in NODES {
        let t = 0.5 * (node + 1.0);
        let derivative = d0 * (2.0 * (1.0 - t)) + d1 * (2.0 * t);
        length += weight * derivative.norm();
    }
    length * 0.5
}

/// 弦の中点のそばで、**本当に断面の上にある点**を探す係。
///
/// 断面上の点は、ソリッドのある面の上にあり、かつ切断平面の上にある。その2つを
/// 交互に満たしにいく（面へ最近傍射影 → 平面へ落とす、の繰り返し）。面と平面が
/// 直交に近いほど速く、実測では数回で 1e-13 まで詰まる。
///
/// 平面の面は最初から見ない。平面 × 平面の断面は直線なので、弦の中点は
/// すでに断面の上にあり、探す意味がないうえ、探せば丸め分だけ動いてしまう。
struct SectionRefiner<'a> {
    surfaces: Vec<&'a zenith_geom::NurbsSurface3>,
    /// 面ごとの前回の (u, v)。断面は面の上を連続的に進むので、次の弦の
    /// 探索はここから始めれば粗いサンプリングを繰り返さずに済む。
    seeds: Vec<Option<(f64, f64)>>,
}

impl<'a> SectionRefiner<'a> {
    fn new(solid: &'a Solid) -> Self {
        let mut surfaces = Vec::new();
        for shell in std::iter::once(&solid.outer_shell).chain(solid.inner_shells.iter()) {
            for face in &shell.faces {
                if let zenith_topo::FaceGeometry::Nurbs(surface) = &face.geometry {
                    surfaces.push(surface);
                }
            }
        }
        let seeds = vec![None; surfaces.len()];
        Self { surfaces, seeds }
    }

    /// 輪郭の各点を断面の上へ載せ直し、動かした点の数を返す。
    ///
    /// 動かしてよい量は隣の弦の長さに対して小さいはずである。それを超えたら
    /// 別の面へ飛んでいるので、その点は動かさない。
    fn settle_loop_points(
        &mut self,
        points: &mut [Point3],
        origin: Point3,
        normal: Vec3,
        tol: &Tolerance,
    ) -> usize {
        if self.surfaces.is_empty() || points.len() < 3 {
            return 0;
        }
        let count = points.len();
        let mut settled = Vec::with_capacity(count);
        let mut moved_count = 0usize;

        for index in 0..count {
            let point = points[index];
            let previous = points[(index + count - 1) % count];
            let next = points[(index + 1) % count];
            let neighbourhood = (point - previous).norm().max((next - point).norm());
            let limit = (neighbourhood * 0.25).max(tol.linear);

            let mut best: Option<(f64, Point3)> = None;
            for surface_index in 0..self.surfaces.len() {
                let Some((candidate, residual)) =
                    self.settle_on(surface_index, point, origin, normal, tol)
                else {
                    continue;
                };
                let moved = (candidate - point).norm();
                if moved > limit || moved <= residual * 8.0 {
                    continue;
                }
                if best.as_ref().map(|(d, _)| moved < *d).unwrap_or(true) {
                    best = Some((moved, candidate));
                }
            }

            match best {
                Some((_, candidate)) => {
                    settled.push(candidate);
                    moved_count += 1;
                }
                None => settled.push(point),
            }
        }

        points.copy_from_slice(&settled);
        moved_count
    }

    /// `a` と `b` の間の断面上の点。見つからなければ `None`（呼び手は弦のまま
    /// 積む）。
    fn section_point_between(
        &mut self,
        a: Point3,
        b: Point3,
        origin: Point3,
        normal: Vec3,
        tol: &Tolerance,
    ) -> Option<Point3> {
        if self.surfaces.is_empty() {
            return None;
        }
        let chord = (b - a).norm();
        if chord <= tol.linear {
            return None;
        }
        let middle = Point3::from((a.coords + b.coords) * 0.5);

        // 補正は弦のたわみの大きさしか出ないはずである。それより大きく動いたら、
        // 隣の面や面の外へ飛んでいる。受け取らない。
        let limit = chord * 0.25;

        let mut best: Option<(f64, Point3)> = None;
        for index in 0..self.surfaces.len() {
            let Some((point, residual)) = self.settle_on(index, middle, origin, normal, tol) else {
                continue;
            };
            let moved = (point - middle).norm();
            if moved > limit {
                continue;
            }
            // **探索の粗さを補正値として採用しない。** 平面のパッチでは断面は
            // 直線なので、弦の中点は既に断面の上にあり、正しい補正は 0 である。
            // それでも射影は自分の残差ぶんだけ点を動かすので、動いた量がその
            // 残差と同じ桁なら、動いたのは幾何ではなく探索のほうである。
            if moved <= residual * 8.0 {
                continue;
            }
            if best.as_ref().map(|(d, _)| moved < *d).unwrap_or(true) {
                best = Some((moved, point));
            }
        }

        best.map(|(_, point)| point)
    }

    /// 1つの面に対して、面と平面の両方を満たす点まで詰める。
    /// 詰めた点と、**その詰め方自身の残差**を返す。残差は呼び手が
    /// 「動いた量が本物か」を判断するために要る。
    fn settle_on(
        &mut self,
        index: usize,
        start: Point3,
        origin: Point3,
        normal: Vec3,
        tol: &Tolerance,
    ) -> Option<(Point3, f64)> {
        let surface = self.surfaces[index];
        // 反復の打ち切りは、比べたい補正量よりずっと細かくないと意味がない。
        // 既定の parametric (1e-7) は、辺の長さ 40 の面では 4e-6 に当たり、
        // 求めたい補正（1e-4 オーダー）と桁が近すぎる。
        let parametric = tol.parametric.min(1e-13);
        let mut point = start;
        let mut uv = self.seeds[index];

        for _ in 0..16 {
            let projection = match uv {
                Some((u, v)) => zenith_geom::ExtremumEngine::point_to_surface_seeded(
                    point, surface, u, v, 64, parametric,
                ),
                None => zenith_geom::ExtremumEngine::point_to_surface(point, surface, 64, parametric),
            }
            .ok()?;

            uv = Some((projection.u, projection.v));
            let on_surface = surface.evaluate(projection.u, projection.v);
            // 面の上に来たら、法線方向に落として平面の上へ戻す。
            let offset = (on_surface - origin).dot(&normal);
            let next = on_surface - normal * offset;
            let step = (next - point).norm();
            point = next;
            if step <= 1e-14 {
                break;
            }
        }

        let (u, v) = uv?;
        // 残差は「詰めた先が2つの条件からどれだけ外れているか」。面までの距離は
        // 最後の uv ではなく、詰めた点をもう一度射影して測る。
        let final_projection = zenith_geom::ExtremumEngine::point_to_surface_seeded(
            point, surface, u, v, 64, parametric,
        )
        .ok()?;
        let off_surface = final_projection.distance;
        let off_plane = (point - origin).dot(&normal).abs();
        if off_surface > tol.linear || off_plane > tol.linear {
            return None;
        }

        self.seeds[index] = Some((final_projection.u, final_projection.v));
        Some((point, off_surface.max(off_plane).max(f64::EPSILON)))
    }
}
