use crate::mesh::TriangleMesh;
use rayon::prelude::*;
use std::collections::{HashMap, HashSet};
use zenith_geom::Surface3;
use zenith_math::{Point2, Point3, Tolerance, Vec2, Vec3};
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
    let result = face_uv_triangulation_inner(face, params);
    zenith_geom::work_counter::count_uv_triangulation(result.triangles.len());
    result
}

fn face_uv_triangulation_inner(face: &Face, params: &TessellationParams) -> UvTriangulation {
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
            // p-curve を保持していない面は、**その場で導出してから**使う。
            // 以前はここで諦めて全矩形のグリッドに落ちていた。トリムされた面が
            // 黙ってトリム前の面として積まれることになり、穴の壁が3倍の面積で
            // 返っていた（`pcurve_derivation_probe` が並べる）。平面側は
            // `plane_pcurves()` が同じことを既にしている。
            let derived_holder;
            let face = if face.pcurves.is_some() {
                face
            } else {
                match face.pcurves(&Tolerance::default()) {
                    Ok(pcurves) => {
                        let mut with = face.clone();
                        with.pcurves = Some(pcurves);
                        derived_holder = with;
                        &derived_holder
                    }
                    Err(_) => face,
                }
            };

            // 境界がパラメータ矩形そのものなら、ノット線に整合したグリッドを
            // 使う。B-spline は各ノット区間の内側でだけ滑らかなので、区間を
            // またぐ三角形の上で求積すると、いくら細分しても誤差が減らない。
            if let Some(aligned) = knot_aligned_uv_triangulation(face, nurbs, params) {
                return aligned;
            }
            // 面積を積む側なので、境界の折れは1つも落とさない。
            let trimmed = trimmed_uv_triangulation(face, nurbs, params, LoopFidelity::Exact);
            if trimmed.is_empty() {
                // ここに来るのは p-curve が導出もできなかった面だけ。全矩形を
                // 積むので、トリムされた面ならこの値は小さすぎず大きすぎる。
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

/// Builds a grid whose lines include every interior knot, for a face whose trim
/// loop is the whole parameter rectangle.
///
/// A B-spline is only smooth inside a knot span. Integrating over cells that
/// straddle a span leaves an error that refinement cannot remove, which is what
/// made a swept pipe's area wander at the fourth decimal no matter how many
/// triangles it was given. Snapping the grid to the spans restores convergence,
/// and it costs nothing for surfaces without interior knots.
///
/// Returns `None` when the face is genuinely trimmed, so the trimmed path keeps
/// handling it.
fn knot_aligned_uv_triangulation(
    face: &Face,
    surface: &zenith_geom::NurbsSurface3,
    params: &TessellationParams,
) -> Option<UvTriangulation> {
    if !face.inner_wires.is_empty() {
        return None;
    }

    let ((u_min, u_max), (v_min, v_max)) = surface.param_range();
    if !(u_max > u_min && v_max > v_min) {
        return None;
    }

    let pcurves = face.pcurves.as_ref()?;
    if !pcurves.inner_loops.is_empty() {
        return None;
    }

    // 縫い目だけの境界は UV 上で全域を囲むが、p-curve からはそう読めない。
    // 縫い目上の点は領域の両端どちらにも写るので、辿った符号付き面積が
    // 全域と一致しない。位相のほうが確かなので、そちらを先に見る。
    if !face.has_seam_only_boundary(Tolerance::default().linear) {
        let outer_uvs = sample_pcurve_loop_uv(&pcurves.outer_loop, params, LoopFidelity::Display);
        if outer_uvs.len() < 3 {
            return None;
        }
        if !loop_covers_full_domain(&outer_uvs, u_min, u_max, v_min, v_max) {
            return None;
        }
    }

    let u_lines = span_aligned_lines(&surface.knots_u.knots, surface.degree_u, u_min, u_max, params.u_divisions);
    let v_lines = span_aligned_lines(&surface.knots_v.knots, surface.degree_v, v_min, v_max, params.v_divisions);

    let mut uvs = Vec::with_capacity(u_lines.len() * v_lines.len());
    for v in &v_lines {
        for u in &u_lines {
            uvs.push(Point2::new(*u, *v));
        }
    }

    let stride = u_lines.len();
    let mut triangles = Vec::with_capacity((u_lines.len() - 1) * (v_lines.len() - 1) * 2);
    for j in 0..v_lines.len() - 1 {
        for i in 0..u_lines.len() - 1 {
            let i0 = j * stride + i;
            triangles.push([i0, i0 + 1, i0 + stride + 1]);
            triangles.push([i0, i0 + stride + 1, i0 + stride]);
        }
    }

    Some(UvTriangulation { uvs, triangles })
}

/// True when the sampled trim loop is the parameter rectangle itself.
fn loop_covers_full_domain(
    uvs: &[Point2],
    u_min: f64,
    u_max: f64,
    v_min: f64,
    v_max: f64,
) -> bool {
    let domain = (u_max - u_min) * (v_max - v_min);
    if domain <= 0.0 {
        return false;
    }

    let scale = (u_max - u_min).max(v_max - v_min);
    let tolerance = scale * 1e-9;

    for uv in uvs {
        if uv.x < u_min - tolerance
            || uv.x > u_max + tolerance
            || uv.y < v_min - tolerance
            || uv.y > v_max + tolerance
        {
            return false;
        }
        // 矩形の辺の上に乗っていない点があれば、それは本当のトリム境界。
        let on_u_edge = (uv.x - u_min).abs() <= tolerance || (uv.x - u_max).abs() <= tolerance;
        let on_v_edge = (uv.y - v_min).abs() <= tolerance || (uv.y - v_max).abs() <= tolerance;
        if !on_u_edge && !on_v_edge {
            return false;
        }
    }

    let mut signed_area = 0.0;
    for index in 0..uvs.len() {
        let a = uvs[index];
        let b = uvs[(index + 1) % uvs.len()];
        signed_area += a.x * b.y - b.x * a.y;
    }
    (signed_area.abs() * 0.5 - domain).abs() <= domain * 1e-9
}

/// Grid lines covering `[min, max]`: every distinct interior knot, plus uniform
/// subdivision inside each span so the requested density is still met.
fn span_aligned_lines(
    knots: &[f64],
    degree: usize,
    min: f64,
    max: f64,
    divisions: usize,
) -> Vec<f64> {
    let mut breaks: Vec<f64> = Vec::new();
    let interior = knots
        .iter()
        .skip(degree + 1)
        .take(knots.len().saturating_sub(2 * (degree + 1)));
    for knot in interior {
        if *knot > min + f64::EPSILON && *knot < max - f64::EPSILON {
            breaks.push(*knot);
        }
    }
    breaks.push(min);
    breaks.push(max);
    breaks.sort_by(f64::total_cmp);
    breaks.dedup_by(|a, b| (*a - *b).abs() <= (max - min) * 1e-12);

    let span_count = breaks.len() - 1;
    let per_span = divisions.max(2).div_ceil(span_count).max(1);

    let mut lines = Vec::with_capacity(span_count * per_span + 1);
    for window in breaks.windows(2) {
        let (start, end) = (window[0], window[1]);
        for step in 0..per_span {
            lines.push(start + (end - start) * (step as f64 / per_span as f64));
        }
    }
    lines.push(max);
    lines
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
    let outer_uvs = sample_pcurve_loop_uv(&pcurves.outer_loop, params, LoopFidelity::Display);
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
        let hole_uvs = sample_pcurve_loop_uv(pcurve_loop, params, LoopFidelity::Display);
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
            // 表示用。境界に何千点も置くと三角形が破綻するので、たわみの
            // 目標までの適応標本で足りる。
            let trimmed = trimmed_uv_triangulation(face, nurbs, params, LoopFidelity::Display);
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
            let outer_uvs = sample_pcurve_loop_uv(&pcurves.outer_loop, params, LoopFidelity::Display);
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
                let hole_uvs = sample_pcurve_loop_uv(pcurve_loop, params, LoopFidelity::Display);
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
    fidelity: LoopFidelity,
) -> UvTriangulation {
    let Some(pcurves) = &face.pcurves else {
        return UvTriangulation::default();
    };
    let outer_uvs = sample_pcurve_loop_uv(&pcurves.outer_loop, params, fidelity);
    zenith_geom::work_counter::count_uv_boundary(outer_uvs.len());
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
        let hole_uvs = sample_pcurve_loop_uv(pcurve_loop, params, fidelity);
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

    refine_uv_triangulation_protected(
        surface,
        params,
        &mut uvs,
        &mut triangles,
        &HashSet::new(),
    );
    UvTriangulation { uvs, triangles }
}

/// Upper bound on triangles produced by trimmed refinement, so a pathological
/// surface degrades into a coarse mesh instead of exhausting memory.
const MAX_REFINED_TRIANGLES: usize = 200_000;
const MAX_REFINEMENT_PASSES: usize = 24;

/// 最長辺二分でトリム領域を細かくする。
///
/// `protected` に入っている辺は割らない。隣の面と共有している境界を割ると、
/// 相手側に対応する点が無く、そこでメッシュが開くため。
pub(crate) fn refine_uv_triangulation_protected(
    surface: &impl Surface3,
    params: &TessellationParams,
    uvs: &mut Vec<Point2>,
    triangles: &mut Vec<[usize; 3]>,
    protected: &HashSet<(usize, usize)>,
) {
    let ((u_min, u_max), (v_min, v_max)) = surface.param_range();
    let cell_u = (u_max - u_min) / params.u_divisions.max(2) as f64;
    let cell_v = (v_max - v_min) / params.v_divisions.max(2) as f64;
    let deflection = surface_deflection_target(surface, params);

    // 一度基準を満たした三角形は、隣が辺を割らない限り再評価しない
    let mut settled = vec![false; triangles.len()];
    let mut cache = EvaluatedPositions::new(uvs.len());

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
            if !triangle_needs_refinement(
                surface,
                uvs,
                triangle,
                cell_u,
                cell_v,
                deflection,
                &mut cache,
            ) {
                settled[index] = true;
                continue;
            }
            let longest = (0..3)
                .filter(|corner| {
                    !protected.contains(&edge_key(
                        triangle[*corner],
                        triangle[(*corner + 1) % 3],
                    ))
                })
                .max_by(|left, right| {
                    scaled_edge_length(uvs, triangle, *left, cell_u, cell_v)
                        .total_cmp(&scaled_edge_length(uvs, triangle, *right, cell_u, cell_v))
                });
            let Some(longest) = longest else {
                // 3辺とも共有境界。これ以上は割れない。
                settled[index] = true;
                continue;
            };
            split_edges.insert(edge_key(triangle[longest], triangle[(longest + 1) % 3]));
        }
        if split_edges.is_empty() {
            return;
        }

        let mut midpoints: HashMap<(usize, usize), usize> = HashMap::new();
        for edge in split_edges {
            let midpoint = Point2::from((uvs[edge.0].coords + uvs[edge.1].coords) * 0.5);
            let index = uvs.len();
            midpoints.insert(edge, index);
            uvs.push(midpoint);
            // 判定で既に評価した点である。頂点になっても評価し直さない。
            cache.adopt_midpoint(edge.0, edge.1, index, uvs);
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

/// 細分の判定で評価した位置を覚えておく係。
///
/// 判定は三角形1つにつき3隅と3辺の中点を評価します。**隅は隣り合う三角形と
/// 平均6枚で、辺の中点は2枚で共有されている**ので、同じ `(u, v)` を何度も
/// 評価していました。円柱×円柱のブーリアン1回で三角形は 260万個できるので、
/// ここは効きます。
///
/// 覚えるのは**同じ引数に対する同じ戻り値**だけなので、結果はビット単位で
/// 変わりません。減るのは回数だけです。
struct EvaluatedPositions {
    /// `uvs` と同じ添字で引ける、評価済みの位置。
    corners: Vec<Option<Point3>>,
    /// 辺の中点。同じ辺は両隣から一度ずつ問われる。
    ///
    /// パスの終わりに捨てても正しさは変わりませんが、**捨てると 3.6% 遅く
    /// なります**（円柱×円柱の和で 16,111,483 → 16,692,654）。隣が辺を
    /// 割ると、基準を満たしていた三角形も作り直されて再判定になり、そのとき
    /// 別の辺の中点が既に記憶されているからです。パスをまたいだ再利用は
    /// 実在します。1面ぶんで数MB、分割が終われば解放されるので、回数を
    /// 取ります。
    midpoints: HashMap<(usize, usize), Point3>,
}

impl EvaluatedPositions {
    fn new(count: usize) -> Self {
        Self {
            corners: vec![None; count],
            midpoints: HashMap::new(),
        }
    }

    fn grow_to(&mut self, count: usize) {
        if self.corners.len() < count {
            self.corners.resize(count, None);
        }
    }

    fn corner(&mut self, surface: &impl Surface3, uvs: &[Point2], index: usize) -> Point3 {
        self.grow_to(uvs.len());
        if let Some(point) = self.corners[index] {
            return point;
        }
        let uv = uvs[index];
        let point = surface.evaluate(uv.x, uv.y);
        self.corners[index] = Some(point);
        point
    }

    fn midpoint(
        &mut self,
        surface: &impl Surface3,
        uvs: &[Point2],
        a: usize,
        b: usize,
    ) -> Point3 {
        let key = edge_key(a, b);
        if let Some(point) = self.midpoints.get(&key) {
            return *point;
        }
        let uv = Point2::from((uvs[a].coords + uvs[b].coords) * 0.5);
        let point = surface.evaluate(uv.x, uv.y);
        self.midpoints.insert(key, point);
        point
    }

    /// 割った辺の中点は、次のパスでは頂点になる。覚えていた値をそのまま渡す。
    fn adopt_midpoint(&mut self, a: usize, b: usize, index: usize, uvs: &[Point2]) {
        let Some(point) = self.midpoints.get(&edge_key(a, b)).copied() else {
            return;
        };
        self.grow_to(uvs.len());
        self.corners[index] = Some(point);
    }
}

fn triangle_needs_refinement(
    surface: &impl Surface3,
    uvs: &[Point2],
    triangle: &[usize; 3],
    cell_u: f64,
    cell_v: f64,
    deflection: f64,
    cache: &mut EvaluatedPositions,
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

    let positions = [
        cache.corner(surface, uvs, triangle[0]),
        cache.corner(surface, uvs, triangle[1]),
        cache.corner(surface, uvs, triangle[2]),
    ];
    (0..3).any(|corner| {
        let next = (corner + 1) % 3;
        let chord = Point3::from((positions[corner].coords + positions[next].coords) * 0.5);
        let middle = cache.midpoint(surface, uvs, triangle[corner], triangle[next]);
        (middle - chord).norm() > deflection
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

/// トリムループを UV 上の折れ線にするときの細かさ。
///
/// 積分と表示では要るものが違う。面積を積むなら境界の折れを1つも落とせない
/// ——落ちたぶんは必ず内側に削れる一方向の誤差になる——が、表示用の三角形は
/// 目に見える細かさで足り、境界に何千点も置くと三角形が破綻する。
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum LoopFidelity {
    /// 表示用。たわみの目標まで適応的に標本を取る。
    Display,
    /// 積分用。1次の p-curve は制御点がそのまま折れ線の頂点なので、
    /// 取り直さずに全部使う。
    Exact,
}

fn sample_pcurve_loop_uv(
    pcurve_loop: &FacePcurveLoop,
    params: &TessellationParams,
    fidelity: LoopFidelity,
) -> Vec<Point2> {
    let mut points = Vec::new();
    let deflection = loop_deflection_target(pcurve_loop, params);

    for segment in pcurve_loop.segments.iter() {
        // 先頭点は「前の区間の終点と一致するときだけ」落とす。縮退エッジを
        // 持つ面（円錐の頂点など）では UV 上に正当な跳びがあり、無条件に
        // 落とすとトリム領域が欠ける。
        // 1次の p-curve は折れ線そのものなので、制御点がそのまま頂点である。
        // 取り直すと角が落ちる。適応標本は深さ 10 までの二分で区間は最大
        // 1024 本なので、投影で作られた p-curve がそれより多くの折れを持つと
        // 必ず削れる。実測では、円柱を傾いた平面で切った境界（p-curve は
        // 制御点 1566 個）で UV 面積が 3.85e-6 欠け、ヤコビアン 628 を掛けて
        // 3D の面積が 2.42e-3 足りなかった。
        let segment_points = if segment.curve.degree == 1
            && (fidelity == LoopFidelity::Exact || segment.curve.control_points.len() == 2)
        {
            segment.curve.control_points.iter().map(|cp| cp.point).collect()
        } else {
            sample_pcurve_segment_adaptive(segment, deflection)
        };

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

    // 境界の折れは面積を必ず内側に削る一方向の誤差で、しかも分割数に比例して
    // しか減らない。分割数に紐づけていたときは、512分割でも円形キャップの面積が
    // 1.1e-3 足りなかった。境界は1次元なので、面の格子よりずっと細かく取っても
    // 費用は線形にしか増えない。分割数に紐づけず、形の大きさに対する比で決める。
    let divisions = params.u_divisions.max(params.v_divisions).max(8) as f64;
    let from_divisions = diagonal / (divisions * 4.0);
    (diagonal * 1e-5).min(from_divisions).max(1e-9)
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

/// B-Rep Solid のテッセレーション（稜を共有した完全な閉多様体メッシュを出力）
///
/// 全分割数（4〜32分割）において開いたエッジ（open edge）・非多様体エッジ・退化三角形が
/// 0件の完全密閉メッシュ（Watertight STL/OBJ対応）を生成する。
pub fn tessellate_solid(solid: &Solid, params: &TessellationParams) -> TriangleMesh {
    zenith_geom::work_counter::count_solid_tessellation();
    crate::stitched::tessellate_solid_stitched(solid, params)
}

#[allow(dead_code)]
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
