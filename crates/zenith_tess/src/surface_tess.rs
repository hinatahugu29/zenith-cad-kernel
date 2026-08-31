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

/// 面のトリム領域の**パラメータ面積**（外周 − 穴）。
///
/// # 何のためにあるか
///
/// 面を割ったとき、「片の面積の和が元に戻るか」で領域の取り違え（重複・
/// 取りこぼし）を見ています。**その検算に 3D の面積は要りません。**
/// 割る前と割ったあとで**同じ曲面の同じパラメータ領域**を見ているので、
/// パラメータ面積が合えば領域は合っています。
///
/// 3D の面積を積むほうは、トリム境界（実測で 4000 点級）を earcut に通し、
/// たわみで 30 倍に細分し、三角形1枚につき6点を評価します。**ブーリアン
/// 1回の仕事の 90% がそこでした**（HANDOVER 4-75）。こちらは折れ線の
/// 多角形面積そのもので、曲面を1回も評価しません。
///
/// 境界の標本は積分側と**同じ取り方**（`LoopFidelity::Exact`）にしてあります。
/// 元と片で取り方が違うと、合うはずのものが合いません。
///
/// p-curve が無く導出もできない面では `None` を返します（そのときは呼ぶ側が
/// 3D の面積に落ちてください）。
pub fn face_parameter_area(face: &Face) -> Option<f64> {
    let derived_holder;
    let face = if face.pcurves.is_some() {
        face
    } else {
        let pcurves = match &face.geometry {
            FaceGeometry::Plane(_) => face.plane_pcurves().ok()?,
            _ => face.pcurves(&Tolerance::default()).ok()?,
        };
        let mut with = face.clone();
        with.pcurves = Some(pcurves);
        derived_holder = with;
        &derived_holder
    };
    let pcurves = face.pcurves.as_ref()?;
    let params = TessellationParams::default();

    let loop_area = |pcurve_loop: &FacePcurveLoop| {
        let uvs = sample_pcurve_loop_uv(pcurve_loop, &params, LoopFidelity::Exact);
        if uvs.len() < 3 {
            return 0.0;
        }
        // 巻き方は面ごとに違うので、大きさだけ使います。穴は外周から引きます。
        let mut twice_area = 0.0;
        for index in 0..uvs.len() {
            let here = uvs[index];
            let next = uvs[(index + 1) % uvs.len()];
            twice_area += here.x * next.y - next.x * here.y;
        }
        (twice_area * 0.5).abs()
    };

    let outer = loop_area(&pcurves.outer_loop);
    let holes: f64 = pcurves.inner_loops.iter().map(loop_area).sum();
    Some(outer - holes)
}

/// Triangulates a face's trimmed parameter domain.
///
/// Planar faces are triangulated exactly by their trim loops. NURBS faces use
/// the same loops and are then refined for curvature. Faces whose trim loops
/// cannot be used, and surface classes without p-curve support, fall back to a
/// uniform grid over the whole parameter rectangle.
pub fn face_uv_triangulation(face: &Face, params: &TessellationParams) -> UvTriangulation {
    let result = face_uv_triangulation_inner(face, params, true);
    zenith_geom::work_counter::count_uv_triangulation(result.triangles.len());
    if std::env::var_os("ZENITH_UVWHY").is_some() {
        eprintln!("UVWHY 細分あり {} u{} v{}", result.triangles.len(), params.u_divisions, params.v_divisions);
    }
    result
}

/// **面の中の点を選ぶための**トリム領域の三角化。
///
/// 細分を掛けず、境界も表示用の精度で標本します。**点を1つ選ぶだけなら
/// どちらも要りません**——earcut が出した三角形の重心は、もう領域の中に
/// あります。平面側の `planar_point_clear_of_holes` も、前から境界を
/// 8点で標本しています。
///
/// **面積や体積を積むのに使ってはいけません。** 細分を外すと曲面の面積が
/// 相対 6e-5 動きます（`ZENITH_NO_TRIM_REFINE` の注記）。境界を粗く取れば
/// 領域そのものが変わります。
///
/// 実測（4-160）: 45ケースの uv 三角形 8,994,423 枚のうち **8,745,591 枚
/// （97%）が、点を1つ選ぶために作られていました**
/// （`representative_face_point` と `spread_face_points`）。**重いのは
/// 細分ではなく earcut のほう**で、1回あたり約2万枚——境界を「1点も
/// 落とさない」精度で標本していたためです。
pub fn face_uv_triangulation_for_point_picking(
    face: &Face,
    params: &TessellationParams,
) -> UvTriangulation {
    let result = face_uv_triangulation_inner(face, params, false);
    zenith_geom::work_counter::count_uv_triangulation(result.triangles.len());
    if std::env::var_os("ZENITH_UVWHY").is_some() {
        eprintln!("UVWHY 点を選ぶ {} u{} v{}", result.triangles.len(), params.u_divisions, params.v_divisions);
    }
    result
}

fn face_uv_triangulation_inner(
    face: &Face,
    params: &TessellationParams,
    refine: bool,
) -> UvTriangulation {
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
            let fidelity = if refine {
                // 面積を積む側なので、境界の折れは1つも落とさない。
                LoopFidelity::Exact
            } else {
                // 点を選ぶだけなので、表示用の適応標本で足りる。
                LoopFidelity::Display
            };
            let trimmed = trimmed_uv_triangulation_with(face, nurbs, params, fidelity, refine);
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
    trimmed_uv_triangulation_with(face, surface, params, fidelity, true)
}

fn trimmed_uv_triangulation_with(
    face: &Face,
    surface: &impl Surface3,
    params: &TessellationParams,
    fidelity: LoopFidelity,
    refine: bool,
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

    // **earcut の出したものを、面積で検算します。**
    //
    // ここは**体積を積む経路**です（`MassCalculator::compute_face_integral`
    // はこの三角形分割を使います）。earcut が多角形より広いものを返すと、
    // **黙って体積が過大になります**。
    //
    // 実測（4-143、他カーネルの円柱 × 楕円柱）: 切り欠きのある4分の1面で、
    // 標本した外周は 0.219583860 なのに earcut の出力は 0.223069042 でした。
    // その 0.003485 が 3D で 9.07、立体の体積で 30.23 の過大として出ます。
    // **恒等式でしか見えませんでした。**
    //
    // 直し方は「別の始点で引き直す」です。earcut の出力は多角形をどの点から
    // 辿るかで変わるので、**回してやり直すと通ることがあります**。回す量は
    // 決め打ちの列なので、答えは実行ごとに変わりません（4-132 で測定の
    // 非決定性を直したばかりなので、ここで戻さないようにしています）。
    let polygon_area = {
        let mut twice = 0.0;
        for index in 0..outer_uvs.len() {
            let a = outer_uvs[index];
            let b = outer_uvs[(index + 1) % outer_uvs.len()];
            twice += a.x * b.y - b.x * a.y;
        }
        (twice * 0.5).abs()
    };
    let hole_area: f64 = {
        let mut sum = 0.0;
        for start in &hole_indices {
            let end = uvs.len();
            // 穴は外周のあとに続けて入っている。次の穴の手前まで。
            let stop = hole_indices
                .iter()
                .copied()
                .find(|next| next > start)
                .unwrap_or(end);
            let mut twice = 0.0;
            for index in *start..stop {
                let a = uvs[index];
                let b = uvs[if index + 1 < stop { index + 1 } else { *start }];
                twice += a.x * b.y - b.x * a.y;
            }
            sum += (twice * 0.5).abs();
        }
        sum
    };
    let wanted_area = polygon_area - hole_area;

    let triangulated_area = |indices: &[usize], points: &[Point2]| {
        let mut sum = 0.0;
        for chunk in indices.chunks_exact(3) {
            let (a, b, c) = (points[chunk[0]], points[chunk[1]], points[chunk[2]]);
            sum += ((b.x - a.x) * (c.y - a.y) - (b.y - a.y) * (c.x - a.x)) * 0.5;
        }
        sum.abs()
    };

    let why = std::env::var_os("ZENITH_TRIM_AREA_WHY").is_some();
    let mut triangle_indices =
        earcutr::earcut(&flat_coords, &hole_indices, 2).unwrap_or_default();
    let tolerance = wanted_area.abs().max(1e-12) * 1e-9;

    if !triangle_indices.is_empty()
        && (triangulated_area(&triangle_indices, &uvs) - wanted_area).abs() > tolerance
    {
        // **合いません。** 外周を回して引き直します。穴は動かしません。
        let count = outer_uvs.len();
        let mut best: Option<(f64, Vec<usize>, Vec<Point2>)> = None;
        for shift_step in 1..8 {
            let shift = count * shift_step / 8;
            if shift == 0 {
                continue;
            }
            let mut rotated_flat = Vec::with_capacity(flat_coords.len());
            let mut rotated_uvs = Vec::with_capacity(uvs.len());
            for offset in 0..count {
                let uv = outer_uvs[(offset + shift) % count];
                rotated_flat.push(uv.x);
                rotated_flat.push(uv.y);
                rotated_uvs.push(uv);
            }
            for index in count..uvs.len() {
                rotated_flat.push(uvs[index].x);
                rotated_flat.push(uvs[index].y);
                rotated_uvs.push(uvs[index]);
            }
            let candidate =
                earcutr::earcut(&rotated_flat, &hole_indices, 2).unwrap_or_default();
            if candidate.is_empty() {
                continue;
            }
            let error = (triangulated_area(&candidate, &rotated_uvs) - wanted_area).abs();
            if error <= tolerance {
                if why {
                    eprintln!(
                        "TRIMAREA earcut が多角形より広いものを返したので、始点を {shift} 回して引き直しました（残差 {error:.3e}）"
                    );
                }
                triangle_indices = candidate;
                uvs = rotated_uvs;
                break;
            }
            let better = match &best {
                Some((previous, _, _)) => error < *previous,
                None => true,
            };
            if better {
                best = Some((error, candidate, rotated_uvs));
            }
        }
        if (triangulated_area(&triangle_indices, &uvs) - wanted_area).abs() > tolerance {
            if let Some((error, candidate, candidate_uvs)) = best {
                if why {
                    eprintln!(
                        "TRIMAREA どの始点でも合わないので、いちばん近いものを採りました（残差 {error:.3e}）"
                    );
                }
                triangle_indices = candidate;
                uvs = candidate_uvs;
            }
        }
    }

    if why {
        eprintln!(
            "TRIMAREA 標本した外周 {polygon_area:.9}、穴 {hole_area:.9}、三角形 {:.9}（点 {}）",
            triangulated_area(&triangle_indices, &uvs),
            outer_uvs.len()
        );
    }
    if triangle_indices.is_empty() {
        return UvTriangulation::default();
    }
    let mut triangles: Vec<[usize; 3]> = triangle_indices
        .chunks_exact(3)
        .map(|chunk| [chunk[0], chunk[1], chunk[2]])
        .collect();

    // **ブーリアン1回の仕事の大半がここです**（実測: 交差した円柱で、曲面
    // 評価 1500万回のうち 1350万回。HANDOVER 4-75）。トリムされた面の面積を
    // 積むたびに、境界の折れ線（実測で 4115 点）を earcut に通し、そこから
    // 出た細長い三角形を細分するので、1枚が 4 万〜16 万枚になります。
    //
    // `ZENITH_TRIM_WHY=1` で、1回ぶんの枚数が出ます。
    //
    // `ZENITH_NO_TRIM_REFINE=1` は**細分を止めます。答えが変わります**——
    // 「細分は表示のためで、積分には要らないのでは」を確かめるための口です。
    // 実測では**要りました**（曲面の面積が相対 6e-5 動き、面分割が 1e-6 の
    // 関門で断られるようになります）。速くするために外す口ではありません。
    let before_refinement = triangles.len();
    let skip_refinement = !refine || std::env::var_os("ZENITH_NO_TRIM_REFINE").is_some();
    if skip_refinement && refine {
        eprintln!(
            "ZENITH_NO_TRIM_REFINE is set: trimmed faces are integrated on the unrefined \
             triangulation and their areas are WRONG (measured: 6e-5 relative on a cylinder)"
        );
    } else {
        // ここは面を1枚で刻む経路（稜を共有しない）。守る境界が無いので 0。
        refine_uv_triangulation_protected(
            surface,
            params,
            &mut uvs,
            &mut triangles,
            &HashSet::new(),
            0,
            &[],
            // ここは面を1枚で刻む経路。**積む側も、面ごとの表示も掛けます。**
            true,
        );
    }
    if std::env::var_os("ZENITH_TRIM_WHY").is_some() {
        eprintln!(
            "TRIMWHY boundary {} pts, earcut {} tris -> refined {} tris (x{:.1})",
            outer_uvs.len(),
            before_refinement,
            triangles.len(),
            triangles.len() as f64 / before_refinement.max(1) as f64
        );
    }
    UvTriangulation { uvs, triangles }
}

/// Upper bound on triangles produced by trimmed refinement, so a pathological
/// surface degrades into a coarse mesh instead of exhausting memory.
/// メッシュを溶接するときの距離（`stitched::tessellate_solid_stitched`）。
///
/// **ここより細かい三角形は、溶接で頂点が束ねられて潰れ、`weld` が外します。
/// 外した跡はそのまま穴になります**——実測（4-117、傾けたトーラス × 箱の差、
/// 24分割）: 1枚の面で **622枚が潰れて消え**、非多様体の稜が 121本
/// 残っていました。潰れているのは分割した辺ではなく、**uv では離れているのに
/// 3D では溶接距離の中に来る頂点対**なので、「短い辺を割らない」歯止めでは
/// 1枚も減りませんでした（測って戻しました）。
pub(crate) const WELD_TOLERANCE: f64 = 1e-7;

/// 細分が**新しい点を作らない**距離。溶接の距離より広く取ります。
///
/// 溶接そのものは緩めません（上の注記のとおり、緩めると別のところが
/// 潰れます）。ここは「割らずに置く」だけの歯止めで、**割らなくても形は
/// 変わりません**（細分は品質のための段です）。
///
/// **なぜ溶接の距離ちょうどでは足りないのか。** 束ねられて潰れる対だけが
/// 問題ではありません。**束ねられないまま、すぐ隣に居る対**も、面積が
/// ほぼ 0 の薄片を作ります。実測（4-131、トーラス × 半径 9 の円柱の差、
/// 24分割）: 面の中を割って作った点が、境界の点から **1.7e-7** のところに
/// 落ち、面積 1.06e-8 の薄片が残って稜が4回使われていました。溶接の距離
/// (1e-7) のすぐ外側なので、そこちょうどの歯止めでは掛かりません。
///
/// 境界の点は稜から取った 3D 点で上書きされるので、面の中の評価とは
/// この桁でずれます。**そのずれより広く取る**のが要ります。
pub(crate) const REFINEMENT_CLEARANCE: f64 = WELD_TOLERANCE * 8.0;

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
    boundary_vertex_count: usize,
    boundary_rings: &[std::ops::Range<usize>],
    // パラメータ格子の条項を掛けるか。**答えを積む側だけ `true`**。
    // 表示側で掛けると、弦誤差が足りているのに割り続けて枚数が
    // 17〜23 倍になります（4-150）。
    enforce_parametric_cells: bool,
) {
    // **境界の点どうしを結ぶ辺は、連続していなくても割ってはいけません。**
    //
    // `protected` に入っているのはリングの**連続する対**だけです。ところが
    // earcut は、境界の点を飛ばして結ぶ辺（弦）を作ります。境界が uv で
    // 直線なら——球のパッチの縁がまさにそうです——その弦は**ぴったり境界の
    // 上に乗ります**。割ると、境界の上に新しい点ができ、隣の面にはその点が
    // 無いので、そこでメッシュが開きます。
    //
    // 実測（4-84）: 球を45度回して切った結果の大円の上に、リングから来る
    // 96点のはずが **1203点**ありました。対を持たない稜が 311 本、すべて
    // 「1回だけ」で、205本は両端の間に別の頂点があります。
    //
    // **「両端が境界の点」だけでは守りすぎです。** 面の内側を通る弦まで
    // 割らなくなり、境界だけで囲まれた曲面（斜めに切った円柱の楕円面など）は
    // 細分が一度も掛かりません。実測で体積が 4712.39 に対して 3335.78
    // ——**29% 狂いました**（`modeling_test` が止めました）。
    //
    // **「領域の縁に乗っているか」だけでも足りません。** 球を貫く大円は
    // 経線なので uv では直線ですが、継ぎ目が別の経度にあるので**領域の縁
    // ではありません**。実測でそこは守られず、247本のまま戻りました。
    //
    // 見るのは「**その辺が境界の折れ線の上を走っているか**」です。中点が
    // 折れ線に乗っていれば走っており、内側を通る弦は乗りません。境界が
    // uv で曲がっていれば弦の中点は膨らみの分だけ外れるので、そこも
    // 守りません（4-84）。
    let mut boundary_span = 0.0f64;
    for range in boundary_rings {
        for offset in 0..range.len() {
            let a = uvs[range.start + offset];
            let b = uvs[range.start + (offset + 1) % range.len()];
            boundary_span = boundary_span.max((b - a).norm());
        }
    }
    let on_polyline_eps = boundary_span.max(1e-12) * 1e-6;
    let runs_along_the_boundary = |a: usize, b: usize, uvs: &[Point2]| {
        if !(a < boundary_vertex_count && b < boundary_vertex_count) {
            return false;
        }
        let middle = Point2::new((uvs[a].x + uvs[b].x) * 0.5, (uvs[a].y + uvs[b].y) * 0.5);
        for range in boundary_rings {
            for offset in 0..range.len() {
                let p = uvs[range.start + offset];
                let q = uvs[range.start + (offset + 1) % range.len()];
                let span = q - p;
                let length_squared = span.norm_squared();
                let t = if length_squared <= f64::EPSILON {
                    0.0
                } else {
                    ((middle - p).dot(&span) / length_squared).clamp(0.0, 1.0)
                };
                if ((p + span * t) - middle).norm() <= on_polyline_eps {
                    return true;
                }
            }
        }
        false
    };

    let ((u_min, u_max), (v_min, v_max)) = surface.param_range();
    let cell_u = (u_max - u_min) / params.u_divisions.max(2) as f64;
    let cell_v = (v_max - v_min) / params.v_divisions.max(2) as f64;
    let deflection = surface_deflection_target(surface, params);

    // **その向きに曲面が直線なら、その向きには割りません。**
    //
    // パラメータ格子の条項は「三角形の uv 範囲が1マスを超えたら、弦誤差に
    // 関係なく割る」というものです（4-150）。**曲面がその向きに直線なら、
    // 割っても何も良くなりません**——求積の則はそこで厳密ですし、弦誤差も
    // ちょうど 0 です。
    //
    // 円柱・円錐の四半パッチは母線が直線なので、**v 方向がこれに当たります**。
    // 45ケースの面積分の 46.1% がその形です（4-154）。
    //
    // **表現ではなく、形で見ます。** 中点が両端の平均に一致するかを数点で
    // 測ります。NURBS の重みを覗くより、これが効く条件そのものです。
    let straight_along = |axis: usize| {
        let scale = {
            let a = surface.evaluate(u_min, v_min);
            let b = surface.evaluate(u_max, v_max);
            (b - a).norm().max(1.0)
        };
        [0.15, 0.5, 0.85].iter().all(|fraction| {
            let other = if axis == 0 {
                v_min + (v_max - v_min) * fraction
            } else {
                u_min + (u_max - u_min) * fraction
            };
            let at = |t: f64| {
                if axis == 0 {
                    surface.evaluate(u_min + (u_max - u_min) * t, other)
                } else {
                    surface.evaluate(other, v_min + (v_max - v_min) * t)
                }
            };
            let (low, middle, high) = (at(0.0), at(0.5), at(1.0));
            let average = Point3::from((low.coords + high.coords) * 0.5);
            (middle - average).norm() <= scale * 1e-12
        })
    };
    let straight_u = enforce_parametric_cells && straight_along(0);
    let straight_v = enforce_parametric_cells && straight_along(1);

    // 一度基準を満たした三角形は、隣が辺を割らない限り再評価しない
    let mut settled = vec![false; triangles.len()];
    let mut cache = EvaluatedPositions::new(uvs.len());

    // **溶接で既にある頂点と1点になる中点は、作らない。**
    //
    // 作ると `weld` がそれを束ね、両方を使っている三角形が潰れ、外した跡が
    // 穴になります（4-117）。実測（4-123、傾けたトーラス × 箱の積、24分割）:
    // 束ねられていた対は**すべて細分が作った頂点どうし**で、境界の点は1組も
    // 関与していませんでした（uv の隔たり 2e-9〜1.5e-8）。
    //
    // **使い回してはいけません。** その頂点は辺の中点ではないので、細分の
    // 三角形が壊れます（実測で 1本 → 34本に悪化。4-123）。**割らずに置く**
    // のが正しい手当てです。細分は品質のための段で、割らなくても形は
    // 変わりません。
    //
    // 格子はパスをまたいで持ち越します——衝突する中点は**別のパス**で
    // 作られていました（同じパスの中だけで見ても1組も減りませんでした）。
    // **最初からある頂点（境界の点と earcut の点）も入れます。**
    //
    // 実測で束ねられていた対は「細分どうし」だけでしたが、**入れないほうを
    // 測ったら悪くなりました**（メッシュが非多様体の演算が 6 → 7）。仕事量も
    // ほとんど変わりません（58,923,183 → 58,769,054）。**測って良かった
    // ほうを採ります。**
    let cell = |v: f64| (v / REFINEMENT_CLEARANCE).floor() as i64;
    let mut weld_grid: HashMap<(i64, i64, i64), (Vec<usize>, Vec<Point3>)> = HashMap::new();
    for index in 0..uvs.len() {
        let position = cache.corner(surface, uvs, index);
        let bucket = weld_grid
            .entry((cell(position.x), cell(position.y), cell(position.z)))
            .or_default();
        bucket.0.push(index);
        bucket.1.push(position);
    }
    let mut skipped_for_weld = 0usize;

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
                enforce_parametric_cells && !straight_u,
                enforce_parametric_cells && !straight_v,
                &mut cache,
            ) {
                settled[index] = true;
                continue;
            }
            let longest = (0..3)
                .filter(|corner| {
                    let (a, b) = (triangle[*corner], triangle[(*corner + 1) % 3]);
                    !protected.contains(&edge_key(a, b)) && !runs_along_the_boundary(a, b, uvs)
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
        // **順序を決めます。**
        //
        // この下のループは、中点を1つ作るたびに `weld_grid` を更新し、
        // 次の中点はその格子と突き合わせて「作るか、割らずに置くか」を
        // 決めます。**つまり結果は順序に依存します。**
        //
        // `HashSet` の反復順は実行ごとに変わります（Rust の既定のハッシャは
        // プロセスごとに種が違います）。実測（4-132）: **同じバイナリ・同じ
        // 入力で `sphere × cylinder (eccentric)` の和が、ある実行では
        // メッシュ非多様体 6本、別の実行では 0本**になりました。ほかの
        // 89演算はすべて一致していたので、揺れているのはここだけです。
        //
        // 添字で並べれば、実行をまたいで同じ答えになります。**この
        // リポジトリは仕事量を実行間で突き合わせる建て付けなので
        // （`boolean_envelope` の「deterministic; compare these across
        // runs」）、揺れるものを残してはいけません。**
        let mut split_edges: Vec<(usize, usize)> = split_edges.into_iter().collect();
        split_edges.sort_unstable();
        if split_edges.is_empty() {
            return;
        }

        let mut midpoints: HashMap<(usize, usize), usize> = HashMap::new();
        for edge in split_edges {
            let midpoint = Point2::from((uvs[edge.0].coords + uvs[edge.1].coords) * 0.5);
            let position = cache.midpoint(surface, uvs, edge.0, edge.1);
            let key = (cell(position.x), cell(position.y), cell(position.z));

            let mut collides = false;
            'search: for dx in -1..=1 {
                for dy in -1..=1 {
                    for dz in -1..=1 {
                        let Some(bucket) = weld_grid.get(&(key.0 + dx, key.1 + dy, key.2 + dz))
                        else {
                            continue;
                        };
                        for existing in &bucket.1 {
                            if (existing - position).norm() <= REFINEMENT_CLEARANCE {
                                collides = true;
                                break 'search;
                            }
                        }
                    }
                }
            }
            if collides {
                // この辺は割らない。中点を入れないので `subdivide_triangle`
                // はここを割りません。
                skipped_for_weld += 1;
                continue;
            }

            // **この中点を作ったのは誰か**（`ZENITH_SPLIT_WATCH="x,y,z"`）。
            //
            // 面をまたいだ継ぎ目が開くとき、片方の面だけが割っていることが
            // あります。**割った現場を、座標で名指しして捕まえる**ための口です
            // （4-209）。境界の点どうしなら、そこは守りの穴です。
            if let Some(watch) = std::env::var("ZENITH_SPLIT_WATCH").ok().and_then(|value| {
                let parts: Vec<f64> = value
                    .split(',')
                    .filter_map(|part| part.trim().parse().ok())
                    .collect();
                (parts.len() == 3).then(|| Point3::new(parts[0], parts[1], parts[2]))
            }) {
                if (position - watch).norm() <= 1e-6 {
                    eprintln!(
                        "SPLITWATCH 中点 ({:.9},{:.9},{:.9}) を作った: 辺 [{}]-[{}]（境界の点か: {} / {}、境界の点は {} 個）",
                        position.x,
                        position.y,
                        position.z,
                        edge.0,
                        edge.1,
                        edge.0 < boundary_vertex_count,
                        edge.1 < boundary_vertex_count,
                        boundary_vertex_count
                    );
                    eprintln!(
                        "SPLITWATCH   uv ({:.9},{:.9})-({:.9},{:.9})、protected か: {}",
                        uvs[edge.0].x,
                        uvs[edge.0].y,
                        uvs[edge.1].x,
                        uvs[edge.1].y,
                        protected.contains(&edge_key(edge.0, edge.1))
                    );
                }
            }

            let index = uvs.len();
            midpoints.insert(edge, index);
            uvs.push(midpoint);
            // 判定で既に評価した点である。頂点になっても評価し直さない。
            cache.adopt_midpoint(edge.0, edge.1, index, uvs);
            let bucket = weld_grid.entry(key).or_default();
            bucket.0.push(index);
            bucket.1.push(position);
        }
        if midpoints.is_empty() {
            // 割れる辺が1本も無い。これ以上進んでも変わらない。
            break;
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

    if skipped_for_weld > 0 && std::env::var_os("ZENITH_REFINE_WHY").is_some() {
        eprintln!("REFINEWHY 溶接で1点になる中点 {skipped_for_weld} 個を、作らずに置いた");
    }

    if std::env::var_os("ZENITH_REFINE_WHY").is_some() {
        // **溶接で束ねられる対が、細分のあとに何組できているか。**
        //
        // 束ねられると三角形が潰れ、`weld` がそれを外し、跡が穴になります
        // （4-117）。対の**両方が境界の点**なのか、**片方が細分の点**なのか、
        // **両方が細分の点**なのかで、直す場所が変わります。
        let mut cache = EvaluatedPositions::new(uvs.len());
        let mut both_boundary = 0usize;
        let mut mixed = 0usize;
        let mut both_interior = 0usize;
        let mut worst_uv = 0.0f64;
        let points: Vec<Point3> = (0..uvs.len())
            .map(|index| cache.corner(surface, uvs, index))
            .collect();
        for left in 0..points.len() {
            for right in (left + 1)..points.len() {
                let gap = (points[right] - points[left]).norm();
                if gap > WELD_TOLERANCE {
                    // **溶接距離のすぐ外に居る対**も数えます。束ねられない
                    // ぶん、そのまま2つの点として残り、同じ稜が2本になります。
                    if gap <= REFINEMENT_CLEARANCE {
                        eprintln!(
                            "REFINEWHY   溶接距離のすぐ外の対 {gap:.3e}（{} と {}）uv 隔たり {:.3e}",
                            if left < boundary_vertex_count { "境界" } else { "細分" },
                            if right < boundary_vertex_count { "境界" } else { "細分" },
                            (uvs[right] - uvs[left]).norm()
                        );
                    }
                    continue;
                }
                let left_boundary = left < boundary_vertex_count;
                let right_boundary = right < boundary_vertex_count;
                match (left_boundary, right_boundary) {
                    (true, true) => both_boundary += 1,
                    (false, false) => both_interior += 1,
                    _ => mixed += 1,
                }
                worst_uv = worst_uv.max((uvs[right] - uvs[left]).norm());
            }
        }
        if both_boundary + mixed + both_interior > 0 {
            eprintln!(
                "REFINEWHY 溶接で束ねられる対 {}（境界どうし {both_boundary}、境界と細分 {mixed}、細分どうし {both_interior}）、uv の隔たりは最大 {worst_uv:.3e}",
                both_boundary + mixed + both_interior
            );
        }
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
    enforce_cell_u: bool,
    enforce_cell_v: bool,
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
    // **パラメータ格子の条項。答えを積む側だけで掛けます。**
    //
    // 三角形の uv 範囲が格子1マスを超えたら、**弦誤差に関係なく**割ります。
    // これは幾何の基準ではなく、パラメータの細かさを揃えるためのものです。
    //
    // 積分では要ります。外して測ったら、板に穴を1つ開けた体積が
    // 94429.2036732051 → 94429.2035749507 と**相対 1.04e-9 動きました**
    // （`boolean_chained_test`。4-150）。
    //
    // **表示では要りません。** 弦誤差の基準はそのまま掛かるので形は保たれ、
    // 枚数だけが落ちます。実測（傾けたトーラス × 箱の差、4-150）:
    // 縫合メッシュ 124,804 → **7,428 枚**（24分割）、1,294,274 → **56,592 枚**
    // （64分割）。1枚の面の最大は 71,250 → **3,572 枚**。
    if (enforce_cell_u && u_extent > cell_u) || (enforce_cell_v && v_extent > cell_v) {
        return true;
    }

    let positions = [
        cache.corner(surface, uvs, triangle[0]),
        cache.corner(surface, uvs, triangle[1]),
        cache.corner(surface, uvs, triangle[2]),
    ];
    // **重心も見る、を試して外しました**（4-150）。3辺の中点が弦に乗って
    // いても内側が膨らむことはある、という理屈は立ちますが、**測ったら
    // 枚数が1枚も変わりませんでした**（縫合メッシュ 7,428 枚で前後同じ）。
    // 効果の測れないものは入れません。
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
