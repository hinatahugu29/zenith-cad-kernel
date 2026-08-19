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
}

/// 曲面を含むソリッドで断面精度を上げるための既定の三角形分割。
/// 円柱・球のような有理NURBS面はこの分割数で内接多角形近似されるため、
/// 断面積の相対誤差はおおよそ (pi^2 / 3) / n^2 のオーダーで縮む。
pub const DEFAULT_SECTION_TESSELLATION: TessellationParams = TessellationParams {
    u_divisions: 96,
    v_divisions: 96,
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

        // 3. 平面内の正規直交フレームに射影して符号付き面積を積む。
        let (axis_u, axis_v) = plane_frame(normal);

        let mut section_wires = Vec::new();
        let mut signed_loop_areas = Vec::new();
        let mut total_area = 0.0;
        let mut total_perimeter = 0.0;

        for points in &chained.loops {
            if points.len() < 3 {
                continue;
            }

            let signed_area = signed_area_on_plane(points, plane_origin, axis_u, axis_v);

            let mut wire_edges = Vec::with_capacity(points.len());
            let mut perimeter = 0.0;
            for index in 0..points.len() {
                let start = points[index];
                let end = points[(index + 1) % points.len()];
                let length = (end - start).norm();
                if length <= tol.linear {
                    continue;
                }
                perimeter += length;
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
