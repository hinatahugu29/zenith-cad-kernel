//! 稜を共有したまま三角形分割する、出力用のテッセレーション。
//!
//! ## なぜ要るのか
//!
//! 面を1枚ずつ三角形にすると、隣り合う面が境界を**別々に刻みます**。平面の
//! キャップはたわみを見て適応的に、曲面のパッチは指定分割数で刻むので、同じ
//! 円の上で片方が 512 点、もう片方が 16 点になります。出来上がったメッシュは
//! 稜に沿って開いていて、見た目には分かりません。
//!
//! 実測（`mesh_watertight_probe`、4分割）:
//!
//! | 立体 | 三角形 | 開いた辺 |
//! | :--- | --: | --: |
//! | 直方体 | 12 | 0 |
//! | 円柱 | 1148 | **1056** |
//! | 円錐 | 1148 | **1056** |
//! | 穴あき直方体 | 1168 | **1056** |
//! | 球 | 256 | 非多様体 18・退化 32 |
//!
//! 効くのは出力先です。STL は三角形しか持たないので、稜で開いていればスライサ
//! が「閉じていない」と言い、3Dプリンタに送れません。
//!
//! ## やること
//!
//! 稜ごとに**1本の刻み方を先に決め**、その稜を使うすべての面が同じ点を使います。
//! 面はその境界を動かせません。
//!
//! - 稜の刻み数は、隣接する面と稜自身の性質から決める。両側が平面で稜が直線
//!   なら2点（直方体が 12 三角形のままでいられる）。曲がっているか、片側でも
//!   曲面パッチなら指定分割数。
//! - 各面は、自分の p-curve をその稜の**同じパラメータ**で評価して uv を得る。
//!   p-curve は辺の正規化パラメータで張られているので、両側が同じ 3D 点に乗る。
//! - パラメータ矩形が丸ごと境界になっている曲面パッチは、4辺の点列から
//!   transfinite 補間で内部を張る。境界は共有点そのものになる。
//!
//! 最後に座標で頂点を束ね、面積0の三角形を落とします（球の極など）。

use std::collections::BTreeMap;


use zenith_math::{Point2, Point3, Tolerance, Vec3};
use zenith_topo::{Face, FaceGeometry, Orientation, OrientedEdge, Shell, Solid, Wire};

use crate::mesh::TriangleMesh;
use crate::surface_tess::TessellationParams;

/// 稜を共有したまま閉じたメッシュを作る
pub fn tessellate_solid_stitched(solid: &Solid, params: &TessellationParams) -> TriangleMesh {
    let plan = SamplePlan::for_solid(solid, params);

    let mut mesh = tessellate_shell_stitched(&solid.outer_shell, params, &plan);
    for inner in &solid.inner_shells {
        let mut inner_mesh = tessellate_shell_stitched(inner, params, &plan);
        for normal in &mut inner_mesh.normals {
            *normal = -*normal;
        }
        for triangle in &mut inner_mesh.indices {
            triangle.swap(1, 2);
        }
        mesh.merge(&inner_mesh);
    }

    weld(&mut mesh, 1e-9);
    mesh
}

fn tessellate_shell_stitched(
    shell: &Shell,
    params: &TessellationParams,
    plan: &SamplePlan,
) -> TriangleMesh {
    let mut mesh = TriangleMesh::new();
    for face in &shell.faces {
        mesh.merge(&tessellate_face_stitched(face, params, plan));
    }
    mesh
}

/// 稜ごとの刻み数。立体全体で1つだけ決めるので、どの面から見ても同じ。
struct SamplePlan {
    counts: BTreeMap<u64, usize>,
    fallback: usize,
}

impl SamplePlan {
    fn for_solid(solid: &Solid, params: &TessellationParams) -> Self {
        let fine = params.u_divisions.max(params.v_divisions).max(2);
        let deflection = deflection_target(solid, fine);
        let mut counts: BTreeMap<u64, usize> = BTreeMap::new();

        let visit = |shell: &Shell, counts: &mut BTreeMap<u64, usize>| {
            for face in &shell.faces {
                let patch = !matches!(face.geometry, FaceGeometry::Plane(_));
                for wire in std::iter::once(&face.outer_wire).chain(face.inner_wires.iter()) {
                    for oriented in &wire.edges {
                        // 直線で、両側が平面の稜は端点だけでよい（直方体が 12
                        // 三角形のままでいられる）。それ以外は**その稜自身の
                        // たわみ**が目標を下回るまで刻む。分割数を一律に使うと、
                        // 螺旋の掃引のように長く曲がった稜が全く足りず、パッチ
                        // の境界が巻きをまたいで切り取ってしまう（丸線ばねの
                        // 体積が 8分割で 7.2% 不足していた）。
                        let wanted = if !patch && is_straight(oriented) {
                            1
                        } else {
                            segments_for_edge(oriented, deflection, fine)
                        };
                        let slot = counts.entry(oriented.edge.id).or_insert(1);
                        *slot = (*slot).max(wanted);
                    }
                }
            }
        };

        visit(&solid.outer_shell, &mut counts);
        for inner in &solid.inner_shells {
            visit(inner, &mut counts);
        }

        Self {
            counts,
            fallback: fine,
        }
    }

    fn segments_for(&self, edge_id: u64) -> usize {
        *self.counts.get(&edge_id).unwrap_or(&self.fallback)
    }
}

fn is_straight(oriented: &OrientedEdge) -> bool {
    let start = oriented.edge.start_vertex.point;
    let end = oriented.edge.end_vertex.point;
    let span = end - start;
    let length = span.norm();
    if length <= 1e-12 {
        return true;
    }
    let direction = span / length;
    for step in 1..8 {
        let point = oriented.edge.curve.evaluate(
            oriented.edge.curve.param_range().0
                + (oriented.edge.curve.param_range().1 - oriented.edge.curve.param_range().0)
                    * step as f64
                    / 8.0,
        );
        let offset = point - start;
        let along = offset.dot(&direction).clamp(0.0, length);
        if (offset - direction * along).norm() > 1e-9 * length.max(1.0) {
            return false;
        }
    }
    true
}

/// 面の境界を、稜ごとに決めた刻み方で表したもの。
///
/// `uv` はその面のパラメータ、`points` は**稜そのものを評価した 3D 点**。
/// 位置を p-curve 経由で作ると、面ごとに p-curve の忠実度（実測 1e-7 台）の
/// ぶんだけずれる。そのずれは公差より小さいのに、メッシュとしては別の点に
/// なるので、穴の口の稜が「4枚の三角形に共有される」状態になっていた。
/// 位置は稜から取り、uv は三角形分割にだけ使う。
struct BoundaryRing {
    uv: Vec<Point2>,
    points: Vec<Point3>,
    /// このループの稜ごとの刻み本数。構造格子の分割数を決めるのに使う。
    segments: Vec<usize>,
}

fn tessellate_face_stitched(
    face: &Face,
    params: &TessellationParams,
    plan: &SamplePlan,
) -> TriangleMesh {
    // p-curve が無ければ、この面は境界を共有できない。従来の経路に落とす。
    let derived;
    let face = if face.pcurves.is_some() {
        face
    } else {
        match face.pcurves(&Tolerance::default()) {
            Ok(pcurves) => {
                let mut with = face.clone();
                with.pcurves = Some(pcurves);
                derived = with;
                &derived
            }
            Err(_) => return crate::surface_tess::tessellate_face(face, params),
        }
    };

    let Some(rings) = boundary_rings(face, plan) else {
        return crate::surface_tess::tessellate_face(face, params);
    };
    if rings.is_empty() || rings[0].uv.len() < 3 {
        return crate::surface_tess::tessellate_face(face, params);
    }

    match &face.geometry {
        FaceGeometry::Plane(_) => patch_mesh(&rings, None, face.orientation, params),
        FaceGeometry::Nurbs(surface) => {
            patch_mesh(&rings, Some(surface), face.orientation, params)
        }
        _ => crate::surface_tess::tessellate_face(face, params),
    }
}

/// 各ループを、稜ごとに決めた刻み方で uv と 3D の点列にする
fn boundary_rings(face: &Face, plan: &SamplePlan) -> Option<Vec<BoundaryRing>> {
    let pcurves = face.pcurves.as_ref()?;
    let wires: Vec<&Wire> = std::iter::once(&face.outer_wire)
        .chain(face.inner_wires.iter())
        .collect();
    let loops = std::iter::once(&pcurves.outer_loop).chain(pcurves.inner_loops.iter());

    let mut out = Vec::new();
    for (wire, pcurve_loop) in wires.iter().zip(loops) {
        if pcurve_loop.segments.len() != wire.edges.len() {
            return None;
        }
        let mut uv: Vec<Point2> = Vec::new();
        let mut points: Vec<Point3> = Vec::new();
        let mut segment_counts: Vec<usize> = Vec::new();

        for (segment, oriented) in pcurve_loop.segments.iter().zip(wire.edges.iter()) {
            if segment.edge_id != oriented.edge.id {
                return None;
            }
            let segments = plan.segments_for(segment.edge_id).max(1);
            segment_counts.push(segments);
            let (t_min, t_max) = segment.curve.param_range();
            for step in 0..=segments {
                let fraction = step as f64 / segments as f64;
                let t = t_min + (t_max - t_min) * fraction;
                let here = segment.curve.evaluate(t);
                // 位置は稜そのものから。p-curve はパラメータを取るためだけに使う。
                let point = oriented.evaluate_normalized(fraction);
                let duplicate = points
                    .last()
                    .map(|last: &Point3| (point - *last).norm() <= 1e-12)
                    .unwrap_or(false);
                if !duplicate {
                    uv.push(here);
                    points.push(point);
                }
            }
        }

        if points.len() > 1 && (points[points.len() - 1] - points[0]).norm() <= 1e-12 {
            uv.pop();
            points.pop();
        }
        out.push(BoundaryRing {
            uv,
            points,
            segments: segment_counts,
        });
    }
    Some(out)
}

/// 境界の点で三角形にし、共有境界を割らない条件で細分してからメッシュにする。
///
/// `surface` が `None` なら平面（細分不要、位置は境界の点だけで決まる）。
fn patch_mesh(
    rings: &[BoundaryRing],
    surface: Option<&zenith_geom::NurbsSurface3>,
    orientation: Orientation,
    params: &TessellationParams,
) -> TriangleMesh {
    // 境界がパラメータ矩形の縁を1周しているパッチは、構造格子で張る。
    // 境界の多角形から earcut で始めて細分に任せると、細長い三角形を大量に
    // 割ることになり、円柱1本で 2172 -> 16828 三角形に膨らむ。
    if let Some(surface) = surface {
        if rings.len() == 1 {
            if let Some(mesh) = grid_patch(rings, surface, orientation, params) {
                return mesh;
            }
        }
    }

    let mut flat = Vec::new();
    let mut uvs: Vec<Point2> = Vec::new();
    // 境界の点は動かせない。細分で足した点だけが曲面から作られる。
    let mut fixed: Vec<Option<Point3>> = Vec::new();
    let mut hole_indices = Vec::new();
    let mut protected: std::collections::HashSet<(usize, usize)> = Default::default();

    for (index, ring) in rings.iter().enumerate() {
        if ring.uv.len() < 3 {
            continue;
        }
        if index > 0 {
            hole_indices.push(uvs.len());
        }
        let first = uvs.len();
        for (uv, point) in ring.uv.iter().zip(ring.points.iter()) {
            flat.push(uv.x);
            flat.push(uv.y);
            uvs.push(*uv);
            fixed.push(Some(*point));
        }
        for offset in 0..ring.uv.len() {
            let a = first + offset;
            let b = first + (offset + 1) % ring.uv.len();
            protected.insert(if a < b { (a, b) } else { (b, a) });
        }
    }

    let mut triangles: Vec<[usize; 3]> = earcutr::earcut(&flat, &hole_indices, 2)
        .unwrap_or_default()
        .chunks_exact(3)
        .map(|chunk| [chunk[0], chunk[1], chunk[2]])
        .collect();
    if triangles.is_empty() {
        return TriangleMesh::new();
    }

    if let Some(surface) = surface {
        crate::surface_tess::refine_uv_triangulation_protected(
            surface,
            params,
            &mut uvs,
            &mut triangles,
            &protected,
        );
    }

    let mut mesh = TriangleMesh::new();
    for (index, uv) in uvs.iter().enumerate() {
        let position = match fixed.get(index).copied().flatten() {
            Some(point) => point,
            None => match surface {
                Some(surface) => surface.evaluate(uv.x, uv.y),
                None => continue,
            },
        };
        mesh.positions.push(position);
        mesh.normals.push(match surface {
            Some(surface) => oriented_normal(surface, *uv, orientation),
            None => Vec3::new(0.0, 0.0, 1.0),
        });
        mesh.uvs.push(uv.coords);
    }

    let forward = orientation.is_forward();
    for triangle in triangles {
        push_with_uv_winding(
            &mut mesh,
            [triangle[0] as u32, triangle[1] as u32, triangle[2] as u32],
            &uvs,
            forward,
        );
    }
    mesh
}

fn push_with_uv_winding(
    mesh: &mut TriangleMesh,
    triangle: [u32; 3],
    uvs: &[Point2],
    forward: bool,
) {
    let p0 = mesh.positions[triangle[0] as usize];
    let p1 = mesh.positions[triangle[1] as usize];
    let p2 = mesh.positions[triangle[2] as usize];
    if (p1 - p0).cross(&(p2 - p0)).norm() <= 1e-18 {
        return;
    }

    let a = uvs[triangle[0] as usize];
    let b = uvs[triangle[1] as usize];
    let c = uvs[triangle[2] as usize];
    let signed = (b.x - a.x) * (c.y - a.y) - (c.x - a.x) * (b.y - a.y);
    if signed == 0.0 {
        return;
    }

    let counter_clockwise = signed > 0.0;
    if counter_clockwise == forward {
        mesh.indices.push(triangle);
    } else {
        mesh.indices.push([triangle[0], triangle[2], triangle[1]]);
    }
}


fn oriented_normal(
    surface: &zenith_geom::NurbsSurface3,
    uv: Point2,
    orientation: Orientation,
) -> Vec3 {
    let normal = surface
        .normal(uv.x, uv.y)
        .unwrap_or_else(|| Vec3::new(0.0, 0.0, 1.0));
    if orientation.is_forward() {
        normal
    } else {
        -normal
    }
}

/// 座標が一致する頂点を1つに束ね、面積0の三角形を落とす
fn weld(mesh: &mut TriangleMesh, tolerance: f64) {
    let cell = tolerance.max(1e-12) * 2.0;
    let key = |p: Point3| {
        (
            (p.x / cell).round() as i64,
            (p.y / cell).round() as i64,
            (p.z / cell).round() as i64,
        )
    };

    let mut lookup: BTreeMap<(i64, i64, i64), u32> = BTreeMap::new();
    let mut remap = vec![0u32; mesh.positions.len()];
    let mut positions = Vec::new();
    let mut normals = Vec::new();
    let mut uvs = Vec::new();

    for (index, position) in mesh.positions.iter().enumerate() {
        match lookup.get(&key(*position)) {
            Some(existing) => remap[index] = *existing,
            None => {
                let slot = positions.len() as u32;
                lookup.insert(key(*position), slot);
                positions.push(*position);
                normals.push(
                    mesh.normals
                        .get(index)
                        .copied()
                        .unwrap_or_else(|| Vec3::new(0.0, 0.0, 1.0)),
                );
                uvs.push(mesh.uvs.get(index).copied().unwrap_or_default());
                remap[index] = slot;
            }
        }
    }

    let mut indices = Vec::with_capacity(mesh.indices.len());
    for triangle in &mesh.indices {
        let mapped = [
            remap[triangle[0] as usize],
            remap[triangle[1] as usize],
            remap[triangle[2] as usize],
        ];
        if mapped[0] == mapped[1] || mapped[1] == mapped[2] || mapped[2] == mapped[0] {
            continue;
        }
        let p0 = positions[mapped[0] as usize];
        let p1 = positions[mapped[1] as usize];
        let p2 = positions[mapped[2] as usize];
        if (p1 - p0).cross(&(p2 - p0)).norm() <= 1e-18 {
            continue;
        }
        indices.push(mapped);
    }

    mesh.positions = positions;
    mesh.normals = normals;
    mesh.uvs = uvs;
    mesh.indices = indices;
}

/// この立体で許すたわみ。寸法と指定分割数から決める。
fn deflection_target(solid: &Solid, divisions: usize) -> f64 {
    let mut low: Option<Point3> = None;
    let mut high: Option<Point3> = None;
    let mut consider = |point: Point3| {
        low = Some(match low {
            Some(l) => Point3::new(l.x.min(point.x), l.y.min(point.y), l.z.min(point.z)),
            None => point,
        });
        high = Some(match high {
            Some(h) => Point3::new(h.x.max(point.x), h.y.max(point.y), h.z.max(point.z)),
            None => point,
        });
    };
    for shell in std::iter::once(&solid.outer_shell).chain(solid.inner_shells.iter()) {
        for face in &shell.faces {
            for wire in std::iter::once(&face.outer_wire).chain(face.inner_wires.iter()) {
                for oriented in &wire.edges {
                    consider(oriented.edge.start_vertex.point);
                    consider(oriented.edge.end_vertex.point);
                }
            }
        }
    }
    let extent = match (low, high) {
        (Some(l), Some(h)) => (h - l).norm(),
        _ => 1.0,
    };
    let divisions = divisions.max(2) as f64;
    (extent / (8.0 * divisions * divisions)).max(1e-9)
}

/// この稜を、自身のたわみが目標を下回るまで何本に刻むか
fn segments_for_edge(oriented: &OrientedEdge, deflection: f64, minimum: usize) -> usize {
    const MAX_SEGMENTS: usize = 4096;
    let mut segments = minimum.max(2);
    while segments < MAX_SEGMENTS {
        let mut worst: f64 = 0.0;
        for step in 0..segments {
            let t0 = step as f64 / segments as f64;
            let t1 = (step + 1) as f64 / segments as f64;
            let a = oriented.evaluate_normalized(t0);
            let b = oriented.evaluate_normalized(t1);
            let middle = oriented.evaluate_normalized((t0 + t1) * 0.5);
            let chord = Point3::from((a.coords + b.coords) * 0.5);
            worst = worst.max((middle - chord).norm());
            if worst > deflection {
                break;
            }
        }
        if worst <= deflection {
            return segments;
        }
        segments *= 2;
    }
    MAX_SEGMENTS
}

/// パラメータ矩形の縁が境界になっているパッチを、構造格子で張る。
///
/// 共有点がその格子の縁にちょうど乗ることを確かめてから使い、乗った点の
/// **位置は稜から取った 3D 点で上書き**する。乗っていなければ使わない。
fn grid_patch(
    rings: &[BoundaryRing],
    surface: &zenith_geom::NurbsSurface3,
    orientation: Orientation,
    params: &TessellationParams,
) -> Option<TriangleMesh> {
    let ring = &rings[0];
    let ((u_min, u_max), (v_min, v_max)) = surface.param_range();
    let (u_span, v_span) = (u_max - u_min, v_max - v_min);
    if !(u_span > 0.0 && v_span > 0.0) {
        return None;
    }

    let on_border = |uv: &Point2| {
        let du = (uv.x - u_min).abs().min((uv.x - u_max).abs());
        let dv = (uv.y - v_min).abs().min((uv.y - v_max).abs());
        du <= 1e-7 * u_span || dv <= 1e-7 * v_span
    };
    if !ring.uv.iter().all(on_border) {
        return None;
    }

    // 格子の分割数は、このループの稜が刻まれた本数そのもの。稜ごとに刻み数が
    // 違う（円弧と真っすぐな継ぎ目が同じパッチにある）ことがあるので、
    // **方向ごとに**取る。1つの数で代表すると、粗いほうの辺の点が細かい格子
    // の縁に「たまたま乗る」ので検査を通り抜け、間の格子点に相手がいない
    // T字の継ぎ目ができる。実測で 6・10・12・20・24 分割のときにメッシュが
    // 開いていた（4・8・16・32 では偶然そろっていた）。
    let counts = &ring.segments;
    let (across, along) = if counts.len() == 4 && counts[0] == counts[2] && counts[1] == counts[3] {
        (counts[0].max(1), counts[1].max(1))
    } else if !counts.is_empty() && counts.iter().all(|c| *c == counts[0]) {
        (counts[0].max(1), counts[0].max(1))
    } else {
        return None;
    };
    // どちらの向きが u かは、格子を作って共有点が全部乗るほうを採る。
    let tolerance = 1e-7 * u_span.max(v_span);
    for (columns_minus, rows_minus) in [(across, along), (along, across)] {
        let columns = columns_minus + 1;
        let rows = rows_minus + 1;
        let mut uvs: Vec<Point2> = Vec::with_capacity(rows * columns);
        for row in 0..rows {
            for column in 0..columns {
                uvs.push(Point2::new(
                    u_min + u_span * column as f64 / columns_minus as f64,
                    v_min + v_span * row as f64 / rows_minus as f64,
                ));
            }
        }

        let mut fixed: Vec<Option<Point3>> = vec![None; uvs.len()];
        let mut matched = true;
        for (uv, point) in ring.uv.iter().zip(ring.points.iter()) {
            let mut best: Option<(f64, usize)> = None;
            for row in 0..rows {
                for column in 0..columns {
                    if row != 0 && row != rows - 1 && column != 0 && column != columns - 1 {
                        continue;
                    }
                    let index = row * columns + column;
                    let distance = (uvs[index] - uv).norm();
                    if best.map(|(d, _)| distance < d).unwrap_or(true) {
                        best = Some((distance, index));
                    }
                }
            }
            match best {
                Some((distance, index)) if distance <= tolerance => fixed[index] = Some(*point),
                _ => {
                    matched = false;
                    break;
                }
            }
        }
        if !matched {
            continue;
        }

        // **格子の縁の頂点にも、すべて相手がいなければならない**。片側だけ
        // 確かめると、相手のいない格子点が残って T 字の継ぎ目になる。
        // ただし潰れた縁（球の極）は稜を持たないので、その行/列がまるごと
        // 1点に潰れている場合だけ許す。
        let mut border_ok = true;
        for row in 0..rows {
            for column in 0..columns {
                if row != 0 && row != rows - 1 && column != 0 && column != columns - 1 {
                    continue;
                }
                let index = row * columns + column;
                if fixed[index].is_some() {
                    continue;
                }
                let uv = uvs[index];
                let here = surface.evaluate(uv.x, uv.y);
                let degenerate = (0..columns).all(|other| {
                    let probe = uvs[row * columns + other];
                    (surface.evaluate(probe.x, probe.y) - here).norm() <= 1e-9
                }) || (0..rows).all(|other| {
                    let probe = uvs[other * columns + column];
                    (surface.evaluate(probe.x, probe.y) - here).norm() <= 1e-9
                });
                if !degenerate {
                    border_ok = false;
                    break;
                }
            }
            if !border_ok {
                break;
            }
        }
        if !border_ok {
            continue;
        }

        return Some(build_grid_mesh(
            surface, orientation, params, uvs, fixed, rows, columns,
        ));
    }
    None
}

#[allow(clippy::too_many_arguments)]
fn build_grid_mesh(
    surface: &zenith_geom::NurbsSurface3,
    orientation: Orientation,
    params: &TessellationParams,
    mut uvs: Vec<Point2>,
    fixed: Vec<Option<Point3>>,
    rows: usize,
    columns: usize,
) -> TriangleMesh {

    let mut triangles = Vec::with_capacity((rows - 1) * (columns - 1) * 2);
    let mut protected: std::collections::HashSet<(usize, usize)> = Default::default();
    let mark = |a: usize, b: usize, set: &mut std::collections::HashSet<(usize, usize)>| {
        set.insert(if a < b { (a, b) } else { (b, a) });
    };
    for row in 0..(rows - 1) {
        for column in 0..(columns - 1) {
            let a = row * columns + column;
            let b = a + 1;
            let c = (row + 1) * columns + column;
            let d = c + 1;
            triangles.push([a, b, d]);
            triangles.push([a, d, c]);
        }
    }
    for column in 0..(columns - 1) {
        mark(column, column + 1, &mut protected);
        let base = (rows - 1) * columns;
        mark(base + column, base + column + 1, &mut protected);
    }
    for row in 0..(rows - 1) {
        mark(row * columns, (row + 1) * columns, &mut protected);
        mark(
            row * columns + columns - 1,
            (row + 1) * columns + columns - 1,
            &mut protected,
        );
    }

    crate::surface_tess::refine_uv_triangulation_protected(
        surface,
        params,
        &mut uvs,
        &mut triangles,
        &protected,
    );

    let mut mesh = TriangleMesh::new();
    for (index, uv) in uvs.iter().enumerate() {
        mesh.positions.push(match fixed.get(index).copied().flatten() {
            Some(point) => point,
            None => surface.evaluate(uv.x, uv.y),
        });
        mesh.normals.push(oriented_normal(surface, *uv, orientation));
        mesh.uvs.push(uv.coords);
    }
    let forward = orientation.is_forward();
    for triangle in triangles {
        push_with_uv_winding(
            &mut mesh,
            [triangle[0] as u32, triangle[1] as u32, triangle[2] as u32],
            &uvs,
            forward,
        );
    }
    mesh
}
