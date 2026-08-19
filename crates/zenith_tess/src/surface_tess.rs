use crate::mesh::TriangleMesh;
use rayon::prelude::*;
use std::collections::{HashMap, HashSet};
use zenith_geom::Surface3;
use zenith_math::{Point2, Point3, Vec2, Vec3};
use zenith_topo::{Face, FaceGeometry, FacePcurveLoop, Orientation, Shell, Solid};

/// 曲面テッセレーション設定パラメータ
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct TessellationParams {
    pub u_divisions: usize,
    pub v_divisions: usize,
}

impl Default for TessellationParams {
    fn default() -> Self {
        Self {
            u_divisions: 24,
            v_divisions: 24,
        }
    }
}

/// A face's parameter domain, triangulated inside its trim loops.
///
/// This is the tessellator's intermediate result, exposed because exact surface
/// integration (area, volume, centroid) needs the same trimmed domain that the
/// display mesh is built from, but must evaluate the surface itself rather than
/// reuse the linearized triangle vertices.
#[derive(Debug, Clone, Default, PartialEq)]
pub struct UvTriangulation {
    pub uvs: Vec<Point2>,
    pub triangles: Vec<[usize; 3]>,
}

impl UvTriangulation {
    pub fn is_empty(&self) -> bool {
        self.triangles.is_empty()
    }
}

/// Triangulates a face's trimmed parameter domain.
///
/// Planar faces are triangulated exactly by their trim loops. NURBS faces use
/// the same loops and are then refined for curvature. Faces whose trim loops
/// cannot be used, and surface classes without p-curve support, fall back to a
/// uniform grid over the whole parameter rectangle.
pub fn face_uv_triangulation(face: &Face, params: &TessellationParams) -> UvTriangulation {
    match &face.geometry {
        FaceGeometry::Plane(_) => {
            let trimmed = planar_uv_triangulation(face, params);
            if trimmed.is_empty() {
                UvTriangulation::default()
            } else {
                trimmed
            }
        }
        FaceGeometry::Nurbs(nurbs) => {
            let trimmed = trimmed_uv_triangulation(face, nurbs, params);
            if trimmed.is_empty() {
                grid_uv_triangulation(nurbs, params)
            } else {
                trimmed
            }
        }
        FaceGeometry::Coons(coons) => grid_uv_triangulation(coons, params),
        FaceGeometry::Gordon(gordon) => grid_uv_triangulation(gordon, params),
        FaceGeometry::Triangular(triangular) => grid_uv_triangulation(triangular, params),
    }
}

fn grid_uv_triangulation(surface: &impl Surface3, params: &TessellationParams) -> UvTriangulation {
    let ((u_min, u_max), (v_min, v_max)) = surface.param_range();
    let num_u = params.u_divisions.max(2);
    let num_v = params.v_divisions.max(2);

    let mut uvs = Vec::with_capacity((num_u + 1) * (num_v + 1));
    for j in 0..=num_v {
        let v = v_min + (v_max - v_min) * (j as f64 / num_v as f64);
        for i in 0..=num_u {
            let u = u_min + (u_max - u_min) * (i as f64 / num_u as f64);
            uvs.push(Point2::new(u, v));
        }
    }

    let mut triangles = Vec::with_capacity(num_u * num_v * 2);
    for j in 0..num_v {
        for i in 0..num_u {
            let stride = num_u + 1;
            let i0 = j * stride + i;
            triangles.push([i0, i0 + 1, i0 + stride + 1]);
            triangles.push([i0, i0 + stride + 1, i0 + stride]);
        }
    }

    UvTriangulation { uvs, triangles }
}

/// Triangulates a planar face's trim loops in its own UV basis.
fn planar_uv_triangulation(face: &Face, params: &TessellationParams) -> UvTriangulation {
    let Ok(pcurves) = face.plane_pcurves() else {
        return UvTriangulation::default();
    };
    let outer_uvs = sample_pcurve_loop_uv(&pcurves.outer_loop, params);
    if outer_uvs.len() < 3 {
        return UvTriangulation::default();
    }

    let mut flat_coords = Vec::new();
    let mut uvs = Vec::new();
    let mut hole_indices = Vec::new();
    for uv in &outer_uvs {
        flat_coords.push(uv.x);
        flat_coords.push(uv.y);
        uvs.push(*uv);
    }
    for pcurve_loop in &pcurves.inner_loops {
        let hole_uvs = sample_pcurve_loop_uv(pcurve_loop, params);
        if hole_uvs.len() >= 3 {
            hole_indices.push(uvs.len());
            for uv in &hole_uvs {
                flat_coords.push(uv.x);
                flat_coords.push(uv.y);
                uvs.push(*uv);
            }
        }
    }

    let triangles: Vec<[usize; 3]> = earcutr::earcut(&flat_coords, &hole_indices, 2)
        .unwrap_or_default()
        .chunks_exact(3)
        .map(|chunk| [chunk[0], chunk[1], chunk[2]])
        .collect();

    UvTriangulation { uvs, triangles }
}

/// 汎用 Surface3 トレイト実装からのグリッドテッセレーション
pub fn tessellate_surface<S: Surface3>(
    surface: &S,
    params: &TessellationParams,
    orientation: Orientation,
) -> TriangleMesh {
    let (u_range, v_range) = surface.param_range();
    tessellate_surface_range(surface, params, orientation, u_range, v_range)
}

/// UVパラメータ部分範囲に限定したグリッドテッセレーション
pub fn tessellate_surface_range<S: Surface3>(
    surface: &S,
    params: &TessellationParams,
    orientation: Orientation,
    (u_min, u_max): (f64, f64),
    (v_min, v_max): (f64, f64),
) -> TriangleMesh {
    let num_u = params.u_divisions.max(2);
    let num_v = params.v_divisions.max(2);

    let mut mesh = TriangleMesh::new();

    // 頂点・法線・UVの生成
    for j in 0..=num_v {
        let v_t = j as f64 / num_v as f64;
        let v = v_min + v_t * (v_max - v_min);

        for i in 0..=num_u {
            let u_t = i as f64 / num_u as f64;
            let u = u_min + u_t * (u_max - u_min);

            let pt = surface.evaluate(u, v);
            let mut norm = surface
                .normal(u, v)
                .unwrap_or_else(|| Vec3::new(0.0, 0.0, 1.0));
            if !orientation.is_forward() {
                norm = -norm;
            }

            mesh.positions.push(pt);
            mesh.normals.push(norm);
            mesh.uvs.push(Vec2::new(u_t, v_t));
        }
    }

    // 三角形インデックスの生成 (各グリッドセルを2つの三角形に分割)
    for j in 0..num_v {
        for i in 0..num_u {
            let row_stride = (num_u + 1) as u32;
            let i0 = (j as u32) * row_stride + (i as u32);
            let i1 = i0 + 1;
            let i2 = i0 + row_stride;
            let i3 = i2 + 1;

            if orientation.is_forward() {
                // 三角形 1: (i0, i1, i3)
                mesh.indices.push([i0, i1, i3]);
                // 三角形 2: (i0, i3, i2)
                mesh.indices.push([i0, i3, i2]);
            } else {
                // 逆向き
                mesh.indices.push([i0, i3, i1]);
                mesh.indices.push([i0, i2, i3]);
            }
        }
    }

    mesh
}

/// B-Rep Face のテッセレーション（Earcut による穴あき・非凸多角形の完全三角形分割）
pub fn tessellate_face(face: &Face, params: &TessellationParams) -> TriangleMesh {
    match &face.geometry {
        FaceGeometry::Nurbs(nurbs) => {
            // トリムループが使えるならそれに従い、扱えない面（球の極など）は
            // 従来どおりパラメータ矩形全体の一様グリッドに落とす
            let trimmed = trimmed_uv_triangulation(face, nurbs, params);
            if trimmed.is_empty() {
                tessellate_surface(nurbs, params, face.orientation)
            } else {
                build_trimmed_mesh(face, nurbs, &trimmed.uvs, &trimmed.triangles)
            }
        }
        FaceGeometry::Coons(coons) => tessellate_surface(coons, params, face.orientation),
        FaceGeometry::Gordon(gordon) => tessellate_surface(gordon, params, face.orientation),
        FaceGeometry::Triangular(tri) => tessellate_surface(tri, params, face.orientation),
        FaceGeometry::Plane(plane) => {
            let Ok(pcurves) = face.plane_pcurves() else {
                return TriangleMesh::new();
            };
            let outer_uvs = sample_pcurve_loop_uv(&pcurves.outer_loop, params);
            if outer_uvs.len() < 3 {
                return TriangleMesh::new();
            }

            // 三角形の向きは面の実効法線で決める。トリムループの周り方にも
            // 三角化ライブラリの出力順にも依存させない。
            let norm = if face.orientation.is_forward() {
                plane.normal
            } else {
                -plane.normal
            };

            // 穴がない単純凸多角形（3〜4頂点）の場合は最速ファン三角化
            if face.inner_wires.is_empty() && outer_uvs.len() <= 4 {
                let mut mesh = TriangleMesh::new();
                for uv in &outer_uvs {
                    mesh.positions.push(plane.evaluate(uv.x, uv.y));
                    mesh.normals.push(norm);
                    mesh.uvs.push(Vec2::new(uv.x, uv.y));
                }
                for i in 1..outer_uvs.len() - 1 {
                    push_oriented_triangle(&mut mesh, [0, i as u32, (i + 1) as u32], norm);
                }
                return mesh;
            }

            // 穴あき多角形または多角形の場合：Earcut アルゴリズムによるロバスト三角化
            let mut flat_coords = Vec::new();
            let mut all_positions = Vec::new();
            let mut hole_indices = Vec::new();

            // 外側ループ
            for uv in &outer_uvs {
                flat_coords.push(uv.x);
                flat_coords.push(uv.y);
                all_positions.push(plane.evaluate(uv.x, uv.y));
            }

            // 内側穴ループ
            for pcurve_loop in &pcurves.inner_loops {
                let hole_uvs = sample_pcurve_loop_uv(pcurve_loop, params);
                if hole_uvs.len() >= 3 {
                    hole_indices.push(all_positions.len());
                    for uv in &hole_uvs {
                        flat_coords.push(uv.x);
                        flat_coords.push(uv.y);
                        all_positions.push(plane.evaluate(uv.x, uv.y));
                    }
                }
            }

            let triangle_indices =
                earcutr::earcut(&flat_coords, &hole_indices, 2).unwrap_or_default();

            let mut mesh = TriangleMesh::new();
            for pt in all_positions {
                mesh.positions.push(pt);
                mesh.normals.push(norm);
                mesh.uvs.push(Vec2::new(0.0, 0.0));
            }

            for chunk in triangle_indices.chunks_exact(3) {
                push_oriented_triangle(
                    &mut mesh,
                    [chunk[0] as u32, chunk[1] as u32, chunk[2] as u32],
                    norm,
                );
            }

            mesh
        }
    }
}

/// Tessellates a NURBS face inside its p-curve trim loops.
///
/// The trim loops are triangulated in UV, then the triangulation is refined
/// until every triangle is no coarser than the requested parameter grid and its
/// 3D chord stays within the deflection target. Refinement splits shared edges
/// through one midpoint table, so the mesh never develops T-junction cracks.
fn trimmed_uv_triangulation(
    face: &Face,
    surface: &impl Surface3,
    params: &TessellationParams,
) -> UvTriangulation {
    let Some(pcurves) = &face.pcurves else {
        return UvTriangulation::default();
    };
    let outer_uvs = sample_pcurve_loop_uv(&pcurves.outer_loop, params);
    if outer_uvs.len() < 3 {
        return UvTriangulation::default();
    }

    let mut flat_coords = Vec::new();
    let mut uvs = Vec::new();
    let mut hole_indices = Vec::new();

    for uv in &outer_uvs {
        flat_coords.push(uv.x);
        flat_coords.push(uv.y);
        uvs.push(*uv);
    }

    for pcurve_loop in &pcurves.inner_loops {
        let hole_uvs = sample_pcurve_loop_uv(pcurve_loop, params);
        if hole_uvs.len() >= 3 {
            hole_indices.push(uvs.len());
            for uv in &hole_uvs {
                flat_coords.push(uv.x);
                flat_coords.push(uv.y);
                uvs.push(*uv);
            }
        }
    }

    let triangle_indices = earcutr::earcut(&flat_coords, &hole_indices, 2).unwrap_or_default();
    if triangle_indices.is_empty() {
        return UvTriangulation::default();
    }
    let mut triangles: Vec<[usize; 3]> = triangle_indices
        .chunks_exact(3)
        .map(|chunk| [chunk[0], chunk[1], chunk[2]])
        .collect();

    refine_uv_triangulation(surface, params, &mut uvs, &mut triangles);
    UvTriangulation { uvs, triangles }
}

/// Upper bound on triangles produced by trimmed refinement, so a pathological
/// surface degrades into a coarse mesh instead of exhausting memory.
const MAX_REFINED_TRIANGLES: usize = 200_000;
const MAX_REFINEMENT_PASSES: usize = 24;

fn refine_uv_triangulation(
    surface: &impl Surface3,
    params: &TessellationParams,
    uvs: &mut Vec<Point2>,
    triangles: &mut Vec<[usize; 3]>,
) {
    let ((u_min, u_max), (v_min, v_max)) = surface.param_range();
    let cell_u = (u_max - u_min) / params.u_divisions.max(2) as f64;
    let cell_v = (v_max - v_min) / params.v_divisions.max(2) as f64;
    let deflection = surface_deflection_target(surface, params);

    // 一度基準を満たした三角形は、隣が辺を割らない限り再評価しない
    let mut settled = vec![false; triangles.len()];

    for _ in 0..MAX_REFINEMENT_PASSES {
        if triangles.len() * 2 > MAX_REFINED_TRIANGLES {
            return;
        }

        // 最長辺だけを割る（Rivara の最長辺二分）。四分割にすると異方な
        // グリッド指定でも等方に細かくなり、必要のない方向まで倍々に増える。
        let mut split_edges: HashSet<(usize, usize)> = HashSet::new();
        for (index, triangle) in triangles.iter().enumerate() {
            if settled[index] {
                continue;
            }
            if !triangle_needs_refinement(surface, uvs, triangle, cell_u, cell_v, deflection) {
                settled[index] = true;
                continue;
            }
            let longest = (0..3)
                .max_by(|left, right| {
                    scaled_edge_length(uvs, triangle, *left, cell_u, cell_v)
                        .total_cmp(&scaled_edge_length(uvs, triangle, *right, cell_u, cell_v))
                })
                .unwrap_or(0);
            split_edges.insert(edge_key(triangle[longest], triangle[(longest + 1) % 3]));
        }
        if split_edges.is_empty() {
            return;
        }

        let mut midpoints: HashMap<(usize, usize), usize> = HashMap::new();
        for edge in split_edges {
            let midpoint = Point2::from((uvs[edge.0].coords + uvs[edge.1].coords) * 0.5);
            midpoints.insert(edge, uvs.len());
            uvs.push(midpoint);
        }

        let mut refined = Vec::with_capacity(triangles.len());
        let mut refined_settled = Vec::with_capacity(triangles.len());
        for (index, triangle) in triangles.iter().enumerate() {
            let pieces = subdivide_triangle(triangle, &midpoints);
            let unchanged = pieces.len() == 1;
            refined_settled.extend(std::iter::repeat_n(
                unchanged && settled[index],
                pieces.len(),
            ));
            refined.extend(pieces);
        }
        *triangles = refined;
        settled = refined_settled;
    }
}

fn scaled_edge_length(
    uvs: &[Point2],
    triangle: &[usize; 3],
    corner: usize,
    cell_u: f64,
    cell_v: f64,
) -> f64 {
    let offset = uvs[triangle[(corner + 1) % 3]] - uvs[triangle[corner]];
    ((offset.x / cell_u).powi(2) + (offset.y / cell_v).powi(2)).sqrt()
}

fn edge_key(a: usize, b: usize) -> (usize, usize) {
    if a <= b {
        (a, b)
    } else {
        (b, a)
    }
}

fn triangle_needs_refinement(
    surface: &impl Surface3,
    uvs: &[Point2],
    triangle: &[usize; 3],
    cell_u: f64,
    cell_v: f64,
    deflection: f64,
) -> bool {
    let corners = [uvs[triangle[0]], uvs[triangle[1]], uvs[triangle[2]]];
    let u_extent = corners
        .iter()
        .fold(f64::NEG_INFINITY, |acc, uv| acc.max(uv.x))
        - corners.iter().fold(f64::INFINITY, |acc, uv| acc.min(uv.x));
    let v_extent = corners
        .iter()
        .fold(f64::NEG_INFINITY, |acc, uv| acc.max(uv.y))
        - corners.iter().fold(f64::INFINITY, |acc, uv| acc.min(uv.y));
    if u_extent > cell_u || v_extent > cell_v {
        return true;
    }

    let positions = corners.map(|uv| surface.evaluate(uv.x, uv.y));
    (0..3).any(|corner| {
        let next = (corner + 1) % 3;
        let mid_uv = Point2::from((corners[corner].coords + corners[next].coords) * 0.5);
        let chord = Point3::from((positions[corner].coords + positions[next].coords) * 0.5);
        (surface.evaluate(mid_uv.x, mid_uv.y) - chord).norm() > deflection
    })
}

/// Splits one triangle according to which of its edges carry a midpoint.
///
/// Handling the one and two edge cases, not only the full four-way split, is
/// what keeps a refined triangle from leaving a T-junction against a neighbour
/// that did not need refining.
fn subdivide_triangle(
    triangle: &[usize; 3],
    midpoints: &HashMap<(usize, usize), usize>,
) -> Vec<[usize; 3]> {
    let splits: Vec<Option<usize>> = (0..3)
        .map(|corner| {
            midpoints
                .get(&edge_key(triangle[corner], triangle[(corner + 1) % 3]))
                .copied()
        })
        .collect();
    let split_count = splits.iter().filter(|split| split.is_some()).count();

    match split_count {
        0 => vec![*triangle],
        3 => {
            let (a, b, c) = (triangle[0], triangle[1], triangle[2]);
            let (ab, bc, ca) = (splits[0].unwrap(), splits[1].unwrap(), splits[2].unwrap());
            vec![[a, ab, ca], [ab, b, bc], [ca, bc, c], [ab, bc, ca]]
        }
        1 => {
            let corner = splits.iter().position(|split| split.is_some()).unwrap();
            let a = triangle[corner];
            let b = triangle[(corner + 1) % 3];
            let c = triangle[(corner + 2) % 3];
            let mid = splits[corner].unwrap();
            vec![[a, mid, c], [mid, b, c]]
        }
        _ => {
            // 分割されていない辺を (c, a) に回して正規形にする
            let unsplit = splits.iter().position(|split| split.is_none()).unwrap();
            let corner = (unsplit + 1) % 3;
            let a = triangle[corner];
            let b = triangle[(corner + 1) % 3];
            let c = triangle[(corner + 2) % 3];
            let ab = splits[corner].unwrap();
            let bc = splits[(corner + 1) % 3].unwrap();
            vec![[a, ab, bc], [ab, b, bc], [a, bc, c]]
        }
    }
}

/// Chord deflection target in model units, derived from the requested grid
/// density over the patch's own 3D size.
fn surface_deflection_target(surface: &impl Surface3, params: &TessellationParams) -> f64 {
    let ((u_min, u_max), (v_min, v_max)) = surface.param_range();
    let mut min = Point3::new(f64::INFINITY, f64::INFINITY, f64::INFINITY);
    let mut max = Point3::new(f64::NEG_INFINITY, f64::NEG_INFINITY, f64::NEG_INFINITY);
    for i in 0..=4 {
        for j in 0..=4 {
            let point = surface.evaluate(
                u_min + (u_max - u_min) * (i as f64 / 4.0),
                v_min + (v_max - v_min) * (j as f64 / 4.0),
            );
            min = Point3::new(min.x.min(point.x), min.y.min(point.y), min.z.min(point.z));
            max = Point3::new(max.x.max(point.x), max.y.max(point.y), max.z.max(point.z));
        }
    }

    let diagonal = (max - min).norm();
    if !diagonal.is_finite() || diagonal <= 1e-9 {
        return 1e-3;
    }

    let divisions = params.u_divisions.max(params.v_divisions).max(2) as f64;
    (diagonal / (divisions * 8.0)).max(1e-6)
}

/// Emits the refined UV triangulation, orienting every triangle by the face's
/// effective surface normal rather than by the trim loop winding.
fn build_trimmed_mesh(
    face: &Face,
    surface: &impl Surface3,
    uvs: &[Point2],
    triangles: &[[usize; 3]],
) -> TriangleMesh {
    let mut mesh = TriangleMesh::new();
    for uv in uvs {
        let mut normal = surface
            .normal(uv.x, uv.y)
            .unwrap_or_else(|| Vec3::new(0.0, 0.0, 1.0));
        if !face.orientation.is_forward() {
            normal = -normal;
        }
        mesh.positions.push(surface.evaluate(uv.x, uv.y));
        mesh.normals.push(normal);
        mesh.uvs.push(Vec2::new(uv.x, uv.y));
    }

    for triangle in triangles {
        let a = mesh.positions[triangle[0]];
        let b = mesh.positions[triangle[1]];
        let c = mesh.positions[triangle[2]];
        let facet = (b - a).cross(&(c - a));
        if facet.norm() <= 1e-18 {
            continue;
        }

        let centroid = Point2::from(
            (uvs[triangle[0]].coords + uvs[triangle[1]].coords + uvs[triangle[2]].coords) / 3.0,
        );
        let mut expected = surface
            .normal(centroid.x, centroid.y)
            .unwrap_or_else(|| mesh.normals[triangle[0]]);
        if !face.orientation.is_forward() {
            expected = -expected;
        }

        if facet.dot(&expected) >= 0.0 {
            mesh.indices
                .push([triangle[0] as u32, triangle[1] as u32, triangle[2] as u32]);
        } else {
            mesh.indices
                .push([triangle[0] as u32, triangle[2] as u32, triangle[1] as u32]);
        }
    }

    mesh
}

fn sample_pcurve_loop_uv(pcurve_loop: &FacePcurveLoop, params: &TessellationParams) -> Vec<Point2> {
    let mut points = Vec::new();
    let deflection = loop_deflection_target(pcurve_loop, params);

    for (segment_index, segment) in pcurve_loop.segments.iter().enumerate() {
        let mut segment_points =
            if segment.curve.degree == 1 && segment.curve.control_points.len() == 2 {
                segment.curve.sample_points(2)
            } else {
                sample_pcurve_segment_adaptive(segment, deflection)
            };

        if segment_index > 0 && !segment_points.is_empty() {
            segment_points.remove(0);
        }

        for uv in segment_points {
            let is_duplicate = points
                .last()
                .map(|last: &Point2| (uv - *last).norm() <= 1e-9)
                .unwrap_or(false);
            if !is_duplicate {
                points.push(uv);
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

fn loop_deflection_target(pcurve_loop: &FacePcurveLoop, params: &TessellationParams) -> f64 {
    let mut min = Point2::new(f64::INFINITY, f64::INFINITY);
    let mut max = Point2::new(f64::NEG_INFINITY, f64::NEG_INFINITY);

    for segment in &pcurve_loop.segments {
        for point in segment.curve.sample_points(5) {
            min.x = min.x.min(point.x);
            min.y = min.y.min(point.y);
            max.x = max.x.max(point.x);
            max.y = max.y.max(point.y);
        }
    }

    let diagonal = (max - min).norm();
    if !diagonal.is_finite() || diagonal <= 1e-9 {
        return 1e-3;
    }

    let divisions = params.u_divisions.max(params.v_divisions).max(8) as f64;
    (diagonal / (divisions * 4.0)).max(1e-4)
}

fn sample_pcurve_segment_adaptive(
    segment: &zenith_topo::FacePcurveSegment,
    deflection: f64,
) -> Vec<Point2> {
    let (t_min, t_max) = segment.curve.param_range();
    let start = segment.curve.evaluate(t_min);
    let end = segment.curve.evaluate(t_max);
    let mut points = vec![start];

    sample_pcurve_interval(
        segment,
        t_min,
        t_max,
        start,
        end,
        deflection,
        0,
        &mut points,
    );
    points.push(end);
    points
}

fn sample_pcurve_interval(
    segment: &zenith_topo::FacePcurveSegment,
    t0: f64,
    t1: f64,
    p0: Point2,
    p1: Point2,
    deflection: f64,
    depth: usize,
    points: &mut Vec<Point2>,
) {
    const MAX_DEPTH: usize = 10;

    let tm = (t0 + t1) * 0.5;
    let pm = segment.curve.evaluate(tm);
    let chord_mid = p0 + (p1 - p0) * 0.5;
    let sagitta = (pm - chord_mid).norm();

    if sagitta <= deflection || depth >= MAX_DEPTH {
        return;
    }

    sample_pcurve_interval(segment, t0, tm, p0, pm, deflection, depth + 1, points);
    let is_duplicate = points
        .last()
        .map(|last| (pm - *last).norm() <= 1e-9)
        .unwrap_or(false);
    if !is_duplicate {
        points.push(pm);
    }
    sample_pcurve_interval(segment, tm, t1, pm, p1, deflection, depth + 1, points);
}

/// B-Rep Shell のテッセレーション（Rayon によるマルチコア超並列処理）
pub fn tessellate_shell(shell: &Shell, params: &TessellationParams) -> TriangleMesh {
    shell
        .faces
        .par_iter()
        .map(|face| tessellate_face(face, params))
        .reduce(TriangleMesh::new, |mut acc, next| {
            acc.merge(&next);
            acc
        })
}

/// B-Rep Solid のテッセレーション（Rayon によるマルチコア超並列処理）
pub fn tessellate_solid(solid: &Solid, params: &TessellationParams) -> TriangleMesh {
    let mut total_mesh = tessellate_shell(&solid.outer_shell, params);
    // 空洞シェルは通常のソリッド外殻と同じ向きで保持されるため、ここで反転する
    for inner in &solid.inner_shells {
        let mut inner_mesh = tessellate_shell(inner, params);
        flip_mesh_orientation(&mut inner_mesh);
        total_mesh.merge(&inner_mesh);
    }
    total_mesh
}

fn flip_mesh_orientation(mesh: &mut TriangleMesh) {
    for normal in &mut mesh.normals {
        *normal = -*normal;
    }
    for tri in &mut mesh.indices {
        tri.swap(1, 2);
    }
}

/// Appends a triangle wound so its facet normal agrees with `expected`.
///
/// Triangulation libraries are free to emit whatever winding they like, and a
/// reversed face keeps its surface normal while flipping its trim loop, so the
/// winding has to be decided against the face's effective normal rather than
/// inherited from either of them.
fn push_oriented_triangle(mesh: &mut TriangleMesh, triangle: [u32; 3], expected: Vec3) {
    let a = mesh.positions[triangle[0] as usize];
    let b = mesh.positions[triangle[1] as usize];
    let c = mesh.positions[triangle[2] as usize];
    if (b - a).cross(&(c - a)).dot(&expected) >= 0.0 {
        mesh.indices.push(triangle);
    } else {
        mesh.indices.push([triangle[0], triangle[2], triangle[1]]);
    }
}
