use crate::mesh::TriangleMesh;
use rayon::prelude::*;
use zenith_geom::Surface3;
use zenith_math::{Point2, Vec2, Vec3};
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

/// 汎用 Surface3 トレイト実装からのグリッドテッセレーション
pub fn tessellate_surface<S: Surface3>(
    surface: &S,
    params: &TessellationParams,
    orientation: Orientation,
) -> TriangleMesh {
    let ((u_min, u_max), (v_min, v_max)) = surface.param_range();
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
        FaceGeometry::Nurbs(nurbs) => tessellate_surface(nurbs, params, face.orientation),
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

            // 穴がない単純凸多角形（3〜4頂点）の場合は最速ファン三角化
            if face.inner_wires.is_empty() && outer_uvs.len() <= 4 {
                let mut mesh = TriangleMesh::new();
                let norm = if face.orientation.is_forward() {
                    plane.normal
                } else {
                    -plane.normal
                };
                for uv in &outer_uvs {
                    mesh.positions.push(plane.evaluate(uv.x, uv.y));
                    mesh.normals.push(norm);
                    mesh.uvs.push(Vec2::new(uv.x, uv.y));
                }
                for i in 1..outer_uvs.len() - 1 {
                    if face.orientation.is_forward() {
                        mesh.indices.push([0, i as u32, (i + 1) as u32]);
                    } else {
                        mesh.indices.push([0, (i + 1) as u32, i as u32]);
                    }
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
            let norm = if face.orientation.is_forward() {
                plane.normal
            } else {
                -plane.normal
            };

            for pt in all_positions {
                mesh.positions.push(pt);
                mesh.normals.push(norm);
                mesh.uvs.push(Vec2::new(0.0, 0.0));
            }

            for chunk in triangle_indices.chunks_exact(3) {
                if face.orientation.is_forward() {
                    mesh.indices
                        .push([chunk[0] as u32, chunk[1] as u32, chunk[2] as u32]);
                } else {
                    mesh.indices
                        .push([chunk[0] as u32, chunk[2] as u32, chunk[1] as u32]);
                }
            }

            mesh
        }
    }
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
