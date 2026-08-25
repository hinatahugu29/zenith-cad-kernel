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

        balance_opposite_edges(solid, &mut counts);

        Self {
            counts,
            fallback: fine,
        }
    }

    fn segments_for(&self, edge_id: u64) -> usize {
        *self.counts.get(&edge_id).unwrap_or(&self.fallback)
    }
}

/// 4辺の面の、向かい合う稜の刻み数を揃える。
///
/// 構造格子は「対辺の刻み数が一致」することを求めます。刻み数は稜ごとに
/// **その稜自身のたわみ**で決めるので、半径の違う2つの円は違う数になります。
/// 読んだ円錐台がそれで、下の円 256・上の円 128 となり、格子から落ちて
/// earcut ＋ 適応細分へ行っていました（32分割で 100,286 三角形）。
///
/// 揃えるときは**多いほうへ**上げます。減らすと、その稜を使う別の面の弦が
/// 粗くなるからです。刻み数は稜ごとに1つだけ持つので、上げた結果は両側の面が
/// そのまま見ます——**継ぎ目は開きません**。
///
/// 1回上げると別の面の対が崩れることがあるので、変わらなくなるまで繰り返し
/// ます。上げる方向にしか動かないので必ず止まります。
fn balance_opposite_edges(solid: &Solid, counts: &mut BTreeMap<u64, usize>) {
    let faces = || {
        std::iter::once(&solid.outer_shell)
            .chain(solid.inner_shells.iter())
            .flat_map(|shell| shell.faces.iter())
    };

    for _round in 0..8 {
        let mut changed = false;
        for face in faces() {
            // 平面は格子を張らないので、揃える必要がありません。
            if matches!(face.geometry, FaceGeometry::Plane(_)) {
                continue;
            }
            for wire in std::iter::once(&face.outer_wire).chain(face.inner_wires.iter()) {
                if wire.edges.len() != 4 {
                    continue;
                }
                for (a, b) in [(0usize, 2usize), (1, 3)] {
                    let (id_a, id_b) = (wire.edges[a].edge.id, wire.edges[b].edge.id);
                    if id_a == id_b {
                        continue;
                    }
                    let want = counts
                        .get(&id_a)
                        .copied()
                        .unwrap_or(1)
                        .max(counts.get(&id_b).copied().unwrap_or(1));
                    for id in [id_a, id_b] {
                        let slot = counts.entry(id).or_insert(1);
                        if *slot < want {
                            *slot = want;
                            changed = true;
                        }
                    }
                }
            }
        }
        if !changed {
            break;
        }
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

    let explain = std::env::var_os("ZENITH_TESS_WHY").is_some();
    let Some(rings) = boundary_rings(face, plan) else {
        if explain {
            eprintln!(
                "TESSWHY face {}: boundary_rings が None → 共有しない経路",
                face.id
            );
        }
        return crate::surface_tess::tessellate_face(face, params);
    };
    if rings.is_empty() || rings[0].uv.len() < 3 {
        if explain {
            eprintln!(
                "TESSWHY face {}: ring が小さすぎる → 共有しない経路",
                face.id
            );
        }
        return crate::surface_tess::tessellate_face(face, params);
    }
    if explain {
        eprintln!(
            "TESSWHY face {}: ring 点数 {:?}, 稜ごとの刻み {:?}",
            face.id,
            rings.iter().map(|r| r.uv.len()).collect::<Vec<_>>(),
            rings
                .first()
                .map(|r| r.segments.clone())
                .unwrap_or_default()
        );
        for (index, ring) in rings.iter().enumerate() {
            let signed_area = (0..ring.uv.len())
                .map(|offset| {
                    let a = ring.uv[offset];
                    let b = ring.uv[(offset + 1) % ring.uv.len()];
                    a.x * b.y - a.y * b.x
                })
                .sum::<f64>()
                * 0.5;
            let (mut u_min, mut u_max) = (f64::INFINITY, f64::NEG_INFINITY);
            let (mut v_min, mut v_max) = (f64::INFINITY, f64::NEG_INFINITY);
            for uv in &ring.uv {
                u_min = u_min.min(uv.x);
                u_max = u_max.max(uv.x);
                v_min = v_min.min(uv.y);
                v_max = v_max.max(uv.y);
            }
            eprintln!(
                "TESSWHY   ring {index}: signed area {signed_area:.9}, bbox ({u_min:.9},{v_min:.9})-({u_max:.9},{v_max:.9}), segments {:?}",
                ring.segments
            );
        }
    }

    match &face.geometry {
        FaceGeometry::Plane(_) => patch_mesh(&rings, None, face.orientation, params),
        FaceGeometry::Nurbs(surface) => patch_mesh(&rings, Some(surface), face.orientation, params),
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
                zenith_geom::work_counter::count_grid_patch();
                return mesh;
            }
        }
        // 構造格子が使えなかった面。ここを通る枚数は `tess_density_probe` が
        // 数えており、ブーリアンの結果が重くなる原因はこの分岐である。
        zenith_geom::work_counter::count_earcut_patch();
    }

    let mut flat = Vec::new();
    let mut uvs: Vec<Point2> = Vec::new();
    // 境界の点は動かせない。細分で足した点だけが曲面から作られる。
    let mut fixed: Vec<Option<Point3>> = Vec::new();
    let mut hole_indices = Vec::new();
    let mut protected: std::collections::HashSet<(usize, usize)> = Default::default();

    let mut ring_ranges: Vec<std::ops::Range<usize>> = Vec::new();
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
        ring_ranges.push(first..uvs.len());
    }

    let mut triangles = earcut_boundary_rings(&uvs, &ring_ranges, &flat, &hole_indices);
    if triangles.is_empty() {
        return TriangleMesh::new();
    }

    let explain_flat = |stage: &str, triangles: &[[usize; 3]], uvs: &[Point2]| {
        if std::env::var_os("ZENITH_TESS_WHY").is_none() {
            return;
        }
        let flat = triangles
            .iter()
            .filter(|triangle| {
                let (a, b, c) = (uvs[triangle[0]], uvs[triangle[1]], uvs[triangle[2]]);
                ((b.x - a.x) * (c.y - a.y) - (b.y - a.y) * (c.x - a.x)).abs() <= 2e-14
            })
            .count();
        eprintln!(
            "TESSWHY   {stage}: 三角形 {}、uv 極小 {flat}",
            triangles.len()
        );
    };
    explain_flat("earcut 直後", &triangles, &uvs);

    // **earcut は、一直線に並んだ境界の点を自分で落とします。**
    // `filterPoints` が共線・重複の点を除くためで、そのぶん三角形は綺麗に
    // なりますが、**落とされた点は隣の面では使われている**ので、そこは
    // 相手のいない稜になります。
    //
    // 実測（4-86）: 箱の面が境界 125 点を持つのに三角形が 2 枚だけになり、
    // 125 の連続対のうち **122 が辺になっていません**でした。
    //
    // **自分で共線を判定して外すのではなく、落とされたものを測って挿し
    // 戻します。** 判定を二重に持つと、earcut の基準と食い違ったときに
    // また壊れます（4-85 で1度やって悪化しました）。
    reinsert_dropped_boundary_points(&uvs, &ring_ranges, &mut triangles);
    explain_flat("境界点挿し戻し後", &triangles, &uvs);
    repair_boundary_ears(
        &uvs,
        rings,
        &ring_ranges,
        surface.is_some(),
        &protected,
        &mut triangles,
    );
    explain_flat("極小 ear 修復後", &triangles, &uvs);

    if let Some(surface) = surface {
        // 境界の点は先頭に固めて入っている。その数を渡して、**境界の点
        // どうしを結ぶ辺は連続していなくても割らない**ようにする（4-84）。
        let boundary_vertex_count = uvs.len();
        crate::surface_tess::refine_uv_triangulation_protected(
            surface,
            params,
            &mut uvs,
            &mut triangles,
            &protected,
            boundary_vertex_count,
            &ring_ranges,
        );
        explain_flat("適応細分後", &triangles, &uvs);
    }

    // **前提を測ります**（4-85 で推論のまま2回外したので）。uv では面積 0 なのに
    // 3D では面積を持つ三角形が、本当に出ているのか。`ZENITH_TESS_WHY=1` で出ます。
    if std::env::var_os("ZENITH_TESS_WHY").is_some() {
        let uv_area = |t: &[usize; 3]| {
            let (a, b, c) = (uvs[t[0]], uvs[t[1]], uvs[t[2]]);
            ((b.x - a.x) * (c.y - a.y) - (b.y - a.y) * (c.x - a.x)).abs() * 0.5
        };
        let mut flat_in_uv = 0usize;
        let mut area_in_3d = 0.0f64;
        let mut total_uv = 0.0f64;
        for triangle in &triangles {
            let a = uv_area(triangle);
            total_uv += a;
            if a <= 1e-14 {
                flat_in_uv += 1;
                if let (Some(pa), Some(pb), Some(pc)) = (
                    fixed.get(triangle[0]).copied().flatten(),
                    fixed.get(triangle[1]).copied().flatten(),
                    fixed.get(triangle[2]).copied().flatten(),
                ) {
                    area_in_3d += (pb - pa).cross(&(pc - pa)).norm() * 0.5;
                }
            }
        }
        // **境界の連続する対が、すべて三角形の辺になっているか。** なって
        // いなければ、そこは相手のいない稜になります（4-86）。
        let mut edges: std::collections::HashSet<(usize, usize)> = Default::default();
        for triangle in &triangles {
            for corner in 0..3 {
                let (a, b) = (triangle[corner], triangle[(corner + 1) % 3]);
                edges.insert(if a < b { (a, b) } else { (b, a) });
            }
        }
        let mut missing = 0usize;
        let mut boundary_pairs = 0usize;
        for range in &ring_ranges {
            let n = range.len();
            for offset in 0..n {
                let a = range.start + offset;
                let b = range.start + (offset + 1) % n;
                boundary_pairs += 1;
                if !edges.contains(&if a < b { (a, b) } else { (b, a) }) {
                    missing += 1;
                }
            }
        }
        eprintln!(
            "TESSWHY   三角形 {}、uv で面積 0 のもの {flat_in_uv} 枚（3D 面積 {area_in_3d:.6}）、uv 面積 {total_uv:.6}、境界の連続対 {boundary_pairs} のうち辺になっていないもの {missing}",
            triangles.len()
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
    if std::env::var_os("ZENITH_TESS_WHY").is_some() {
        let mut emitted_edges: std::collections::HashSet<(usize, usize)> = Default::default();
        for triangle in &mesh.indices {
            for corner in 0..3 {
                let (a, b) = (
                    triangle[corner] as usize,
                    triangle[(corner + 1) % 3] as usize,
                );
                emitted_edges.insert(if a < b { (a, b) } else { (b, a) });
            }
        }
        let mut missing_after_emit = 0usize;
        let mut boundary_pairs = 0usize;
        for range in &ring_ranges {
            for offset in 0..range.len() {
                let a = range.start + offset;
                let b = range.start + (offset + 1) % range.len();
                boundary_pairs += 1;
                let key = if a < b { (a, b) } else { (b, a) };
                if !emitted_edges.contains(&key) {
                    missing_after_emit += 1;
                }
            }
        }
        eprintln!(
            "TESSWHY   mesh emit 後: 三角形 {}、境界の連続対 {boundary_pairs} のうち辺になっていないもの {missing_after_emit}",
            mesh.indices.len()
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
///
/// 寸法は**頂点**から測ります。曲がった形では、頂点の箱は形そのものより
/// 小さく出ます（螺旋ばねは両端が同じ角度に来るので、頂点だけ見ると
/// 幅 2R が消える）。その控えめさは**効いています**——刻みが細かい側に
/// 倒れるので、丸線ばねのメッシュ体積は 16〜256 分割で 3.98e-4 から
/// 1.55e-6 まで単調に落ちます。
///
/// **稜の上の点まで見て「正しく」測ると、これが壊れます。** 実測（同じ
/// `helix_volume_probe`、稜を5点ずつ見るようにしたもの）:
///
/// | 分割 | 頂点で測る | 稜の上まで測る |
/// | --: | --: | --: |
/// | 16 | 3.98e-4 | 1.19e-3 |
/// | 64 | 2.49e-5 | **5.85e-1** |
/// | 128 | 6.22e-6 | **-4.86e0** |
///
/// 目標が緩むとパッチの境界が巻きをまたいで切り取られ、収束すらしなく
/// なります。ここは「正しい寸法」ではなく「安全側の寸法」が要る場所です。
///
/// 潰れたときだけが問題です。全周で閉じた1枚の面（他カーネルが書いた
/// トーラス）は4本の継ぎ目がすべて同じ1点で始まって同じ点で終わるので、
/// **頂点の箱は1点に潰れて範囲が 0 になります**。目標が下限 1e-9 に落ち、
/// どの稜も上限 4096 まで刻まれ、16384 点の多角形を渡された earcut は
/// 事実上返ってきません（実測: 4分割でも 120 秒で返らず）。
///
/// なので、**測り方は変えず、潰れだけを見張ります**。頂点の箱が稜の箱に
/// 比べて桁違いに小さいときだけ、稜の箱に切り替えます。
fn deflection_target(solid: &Solid, divisions: usize) -> f64 {
    let vertex_extent = solid_extent(solid, 0);
    let edge_extent = solid_extent(solid, 4);
    // 潰れの判定。ばねは 0.42 倍なので触りません。トーラスは 0 です。
    let extent = if vertex_extent > edge_extent * 1e-3 {
        vertex_extent
    } else {
        edge_extent
    };
    let divisions = divisions.max(2) as f64;
    (extent / (8.0 * divisions * divisions)).max(1e-9)
}

/// 立体の境界箱の対角長。`samples` が 0 なら稜の端点だけ、正なら稜の途中も見る。
fn solid_extent(solid: &Solid, samples: usize) -> f64 {
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
                    if samples == 0 {
                        consider(oriented.edge.start_vertex.point);
                        consider(oriented.edge.end_vertex.point);
                    } else {
                        for step in 0..=samples {
                            consider(oriented.evaluate_normalized(step as f64 / samples as f64));
                        }
                    }
                }
            }
        }
    }
    match (low, high) {
        (Some(l), Some(h)) => (h - l).norm(),
        _ => 1.0,
    }
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

/// 極を持つ面の、境界3辺から格子の刻み数を読む。
///
/// 潰れた側に稜が無いので、残るのは「向かい合う2辺」と「1辺」です。
/// 向かい合う2辺は同じ刻み数でなければならず、そうでなければ格子は張れません。
fn degenerate_side_counts(counts: &[usize]) -> Option<(usize, usize)> {
    if counts.len() != 3 {
        return None;
    }
    for (a, b, odd) in [(0, 1, 2), (0, 2, 1), (1, 2, 0)] {
        if counts[a] == counts[b] {
            return Some((counts[odd].max(1), counts[a].max(1)));
        }
    }
    None
}

/// earcut が落とした境界の点を、三角形の辺に挿し戻す。
///
/// 使われなかった境界の点の連なりを見つけ、その両端を結ぶ辺を持つ三角形を
/// 扇状に割り直します。**共有する点を1つも失いません。**
///
/// 自分で「共線かどうか」を判定しないのが要点です。earcut が何を落とすかは
/// earcut の基準で決まるので、こちらで別の基準を持つと食い違います
/// （4-85 で1度やって悪化しました）。**落ちた結果のほうを見ます。**
fn reinsert_dropped_boundary_points(
    uvs: &[Point2],
    ring_ranges: &[std::ops::Range<usize>],
    triangles: &mut Vec<[usize; 3]>,
) {
    let _ = uvs;
    // 1回の走査で1つ直して数え直します。**上限は多めに**——8 回では足りず、
    // 125 点の境界で 120 対が残りました（実測）。
    for _round in 0..4096 {
        let mut used: std::collections::HashSet<usize> = Default::default();
        let mut edges: std::collections::HashSet<(usize, usize)> = Default::default();
        for triangle in triangles.iter() {
            for corner in 0..3 {
                used.insert(triangle[corner]);
                let (a, b) = (triangle[corner], triangle[(corner + 1) % 3]);
                edges.insert(if a < b { (a, b) } else { (b, a) });
            }
        }

        let mut repaired = false;
        for range in ring_ranges {
            let n = range.len();
            if n < 3 {
                continue;
            }
            let mut offset = 0usize;
            while offset < n {
                let index = range.start + offset;
                if used.contains(&index) {
                    offset += 1;
                    continue;
                }
                // 使われていない点の連なり。両側の、使われている点を探す。
                let mut run = vec![index];
                let mut ahead = offset + 1;
                while ahead < offset + n {
                    let next = range.start + (ahead % n);
                    if used.contains(&next) {
                        break;
                    }
                    run.push(next);
                    ahead += 1;
                }
                let before = range.start + (offset + n - 1) % n;
                let after = range.start + (ahead % n);
                offset = ahead + 1;

                if !used.contains(&before) || !used.contains(&after) {
                    continue;
                }
                let key = if before < after {
                    (before, after)
                } else {
                    (after, before)
                };
                if !edges.contains(&key) {
                    continue;
                }
                let Some(position) = triangles.iter().position(|t| {
                    (0..3).any(|corner| {
                        let (a, b) = (t[corner], t[(corner + 1) % 3]);
                        (a == before && b == after) || (a == after && b == before)
                    })
                }) else {
                    continue;
                };
                let triangle = triangles.swap_remove(position);
                let Some(apex) = triangle
                    .iter()
                    .find(|v| **v != before && **v != after)
                    .copied()
                else {
                    continue;
                };
                let forward = (0..3).any(|corner| {
                    triangle[corner] == before && triangle[(corner + 1) % 3] == after
                });

                let mut chain = Vec::with_capacity(run.len() + 2);
                chain.push(before);
                chain.extend(run.iter().copied());
                chain.push(after);
                for pair in chain.windows(2) {
                    let (a, b) = (pair[0], pair[1]);
                    triangles.push(if forward { [a, b, apex] } else { [b, a, apex] });
                }
                repaired = true;
                break;
            }
            if repaired {
                break;
            }
        }
        if !repaired {
            return;
        }
    }
}

/// earcutへ渡す境界を作る。外周に接する穴は、接点で外周へ縫い込んだ単一ring
/// として試し、通常のhole入力より境界辺を多く保てた場合だけ採用する。
fn earcut_boundary_rings(
    uvs: &[Point2],
    ring_ranges: &[std::ops::Range<usize>],
    flat: &[f64],
    hole_indices: &[usize],
) -> Vec<[usize; 3]> {
    let decode = |indices: Vec<usize>, mapping: &[usize]| {
        indices
            .chunks_exact(3)
            .map(|chunk| [mapping[chunk[0]], mapping[chunk[1]], mapping[chunk[2]]])
            .collect::<Vec<_>>()
    };
    let identity = (0..uvs.len()).collect::<Vec<_>>();
    let mut best = decode(
        earcutr::earcut(flat, hole_indices, 2).unwrap_or_default(),
        &identity,
    );
    if ring_ranges.len() != 2 {
        return best;
    }

    let outer = &ring_ranges[0];
    let hole = &ring_ranges[1];
    if outer.is_empty() || hole.is_empty() {
        return best;
    }
    let mut span = 0.0f64;
    for range in ring_ranges {
        for index in range.clone() {
            let next = if index + 1 == range.end {
                range.start
            } else {
                index + 1
            };
            span = span.max((uvs[next] - uvs[index]).norm());
        }
    }
    let touch_eps = span.max(1.0) * 1e-9;
    let touching = outer.clone().find_map(|outer_index| {
        hole.clone()
            .find(|hole_index| (uvs[outer_index] - uvs[*hole_index]).norm() <= touch_eps)
            .map(|hole_index| (outer_index, hole_index))
    });
    let Some((outer_touch, hole_touch)) = touching else {
        return best;
    };

    let mut mapping = Vec::with_capacity(outer.len() + hole.len() + 1);
    mapping.extend(outer.start..=outer_touch);
    for step in 1..=hole.len() {
        mapping.push(hole.start + ((hole_touch - hole.start + step) % hole.len()));
    }
    mapping.extend((outer_touch + 1)..outer.end);

    let mut merged_flat = Vec::with_capacity(mapping.len() * 2);
    for index in &mapping {
        merged_flat.push(uvs[*index].x);
        merged_flat.push(uvs[*index].y);
    }
    let candidate = decode(
        earcutr::earcut(&merged_flat, &[], 2).unwrap_or_default(),
        &mapping,
    );

    let missing_boundary_edges = |triangles: &[[usize; 3]]| {
        let mut edges: std::collections::HashSet<(usize, usize)> = Default::default();
        for triangle in triangles {
            for corner in 0..3 {
                let (a, b) = (triangle[corner], triangle[(corner + 1) % 3]);
                edges.insert(if a < b { (a, b) } else { (b, a) });
            }
        }
        ring_ranges
            .iter()
            .map(|range| {
                (0..range.len())
                    .filter(|offset| {
                        let a = range.start + *offset;
                        let b = range.start + (*offset + 1) % range.len();
                        let key = if a < b { (a, b) } else { (b, a) };
                        !edges.contains(&key)
                    })
                    .count()
            })
            .sum::<usize>()
    };
    if !candidate.is_empty() && missing_boundary_edges(&candidate) < missing_boundary_edges(&best) {
        best = candidate;
    }
    best
}

/// earcut が境界上に残した、uv 面積がほぼ 0 の ear を内部 edge flip で直す。
///
/// 球を大円で切った面では、earcut が大円上の3点だけからなる細長い三角形を
/// 返すことがある。uv ではほぼ潰れていても、3Dでは3点が円板を張るため、隣の
/// 平面 cap と重なって同じ mesh edge が4回使われる。
///
/// 三角形を捨てると共有境界が開き、境界を細分すると隣の面に無い頂点が増える。
/// そこで外周の頂点・辺には触れず、隣接する正常な三角形との共有対角線だけを
/// flipする。新旧のuv面積が一致し、2枚とも十分な面積を持ち、新しい対角線が
/// 既存辺と重複しない場合だけ採用する。
fn repair_boundary_ears(
    uvs: &[Point2],
    rings: &[BoundaryRing],
    ring_ranges: &[std::ops::Range<usize>],
    curved_surface: bool,
    protected: &std::collections::HashSet<(usize, usize)>,
    triangles: &mut Vec<[usize; 3]>,
) {
    if triangles.len() < 2 || uvs.len() < 4 {
        return;
    }

    let mut u_min = f64::INFINITY;
    let mut u_max = f64::NEG_INFINITY;
    let mut v_min = f64::INFINITY;
    let mut v_max = f64::NEG_INFINITY;
    for uv in uvs {
        u_min = u_min.min(uv.x);
        u_max = u_max.max(uv.x);
        v_min = v_min.min(uv.y);
        v_max = v_max.max(uv.y);
    }
    let span = (u_max - u_min).abs().max((v_max - v_min).abs()).max(1.0);
    let flat_eps = span * span * 2e-14;

    let area2 = |triangle: [usize; 3]| {
        let (a, b, c) = (uvs[triangle[0]], uvs[triangle[1]], uvs[triangle[2]]);
        (b.x - a.x) * (c.y - a.y) - (b.y - a.y) * (c.x - a.x)
    };
    let orient = |mut triangle: [usize; 3], sign: f64| {
        if area2(triangle).signum() != sign.signum() {
            triangle.swap(1, 2);
        }
        triangle
    };
    let lies_on_one_boundary_edge = |triangle: [usize; 3]| {
        for (ring, range) in rings.iter().zip(ring_ranges) {
            if !triangle.iter().all(|vertex| range.contains(vertex)) {
                continue;
            }
            let n = range.len();
            if n == 0 || ring.segments.iter().sum::<usize>() != n {
                continue;
            }
            let offsets = triangle.map(|vertex| vertex - range.start);
            let mut start = 0usize;
            for segments in &ring.segments {
                let end = start + *segments;
                let on_edge = |offset: usize| {
                    if end < n {
                        offset >= start && offset <= end
                    } else {
                        offset >= start || offset <= end - n
                    }
                };
                if offsets.iter().copied().all(on_edge) {
                    return true;
                }
                start = end;
            }
        }
        false
    };
    let needs_repair = |triangle: [usize; 3]| {
        area2(triangle).abs() <= flat_eps || (curved_surface && lies_on_one_boundary_edge(triangle))
    };

    let mut skipped: std::collections::HashSet<usize> = Default::default();
    for _round in 0..triangles.len() * 4 {
        let Some(flat_index) = triangles
            .iter()
            .enumerate()
            .find(|(index, triangle)| !skipped.contains(index) && needs_repair(**triangle))
            .map(|(index, _)| index)
        else {
            break;
        };

        let mut edge_uses: std::collections::HashMap<(usize, usize), Vec<usize>> =
            Default::default();
        for (index, triangle) in triangles.iter().enumerate() {
            for corner in 0..3 {
                let (a, b) = (triangle[corner], triangle[(corner + 1) % 3]);
                let key = if a < b { (a, b) } else { (b, a) };
                edge_uses.entry(key).or_default().push(index);
            }
        }

        let flat = triangles[flat_index];
        let mut repaired = false;
        for corner in 0..3 {
            let (u, v) = (flat[corner], flat[(corner + 1) % 3]);
            let shared = if u < v { (u, v) } else { (v, u) };
            if protected.contains(&shared) {
                continue;
            }
            let Some(uses) = edge_uses.get(&shared) else {
                continue;
            };
            if uses.len() != 2 {
                continue;
            }
            let neighbour_index = if uses[0] == flat_index {
                uses[1]
            } else if uses[1] == flat_index {
                uses[0]
            } else {
                continue;
            };
            let neighbour = triangles[neighbour_index];
            let Some(a) = flat
                .iter()
                .find(|vertex| **vertex != u && **vertex != v)
                .copied()
            else {
                continue;
            };
            let Some(d) = neighbour
                .iter()
                .find(|vertex| **vertex != u && **vertex != v)
                .copied()
            else {
                continue;
            };
            if a == d {
                continue;
            }
            let diagonal = if a < d { (a, d) } else { (d, a) };
            if edge_uses.contains_key(&diagonal) {
                continue;
            }

            let sign = area2(neighbour);
            if sign.abs() <= flat_eps {
                continue;
            }
            let first = orient([a, u, d], sign);
            let second = orient([a, d, v], sign);
            let first_area = area2(first).abs();
            let second_area = area2(second).abs();
            // 極小earが数枚連なっていると、隣も同じく極小なため「2枚とも
            // 一度で正常になる」条件では入口で止まる。悪い三角形の枚数が
            // 必ず減るflipなら1枚を残して先へ進め、次のroundでその1枚を
            // 直す。単なる横移動（1 -> 1）は許さないので循環しない。
            let old_bad = usize::from(needs_repair(flat)) + usize::from(needs_repair(neighbour));
            let new_bad = usize::from(needs_repair(first)) + usize::from(needs_repair(second));
            if new_bad >= old_bad {
                continue;
            }
            let old_area = area2(flat).abs() + area2(neighbour).abs();
            let new_area = first_area + second_area;
            let area_tolerance = flat_eps * 16.0 + old_area * 1e-9;
            if (new_area - old_area).abs() > area_tolerance {
                continue;
            }

            triangles[flat_index] = first;
            triangles[neighbour_index] = second;
            skipped.clear();
            repaired = true;
            break;
        }

        if !repaired {
            skipped.insert(flat_index);
        }
    }

    if std::env::var_os("ZENITH_TESS_WHY").is_some() {
        let mut edge_uses: std::collections::HashMap<(usize, usize), usize> = Default::default();
        for triangle in triangles.iter() {
            for corner in 0..3 {
                let (a, b) = (triangle[corner], triangle[(corner + 1) % 3]);
                let key = if a < b { (a, b) } else { (b, a) };
                *edge_uses.entry(key).or_default() += 1;
            }
        }
        for (index, triangle) in triangles.iter().enumerate() {
            if !needs_repair(*triangle) {
                continue;
            }
            let edge_state = (0..3)
                .map(|corner| {
                    let (a, b) = (triangle[corner], triangle[(corner + 1) % 3]);
                    let key = if a < b { (a, b) } else { (b, a) };
                    format!(
                        "{}-{}:uses{}{}",
                        a,
                        b,
                        edge_uses.get(&key).copied().unwrap_or(0),
                        if protected.contains(&key) {
                            ":protected"
                        } else {
                            ""
                        }
                    )
                })
                .collect::<Vec<_>>();
            eprintln!(
                "TESSWHY   BOUNDARYEAR unresolved triangle {index} {:?}, area2 {:.3e}, same edge {}, edges {:?}",
                triangle,
                area2(*triangle),
                lies_on_one_boundary_edge(*triangle),
                edge_state
            );
        }
    }
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
    let ((domain_u_min, domain_u_max), (domain_v_min, domain_v_max)) = surface.param_range();
    if !(domain_u_max > domain_u_min && domain_v_max > domain_v_min) {
        return None;
    }

    // 格子を張る矩形は、曲面のパラメータ領域全体ではなく **境界が実際に囲んで
    // いる矩形** で取る。
    //
    // 面の境界が領域の縁に乗っているとは限らない。ブーリアンの結果では、支持
    // 曲面が元の立体のまま（例えば全高 40 の円柱）で、面はその一部（箱の高さ
    // 20）しか使っていないことがある。そのとき境界は領域の**内部**の等パラ
    // メータ線上にあり、領域全体で縁を判定すると弾かれる。実測で、穴あき箱を
    // ブーリアンで作ったときのボア壁4枚は境界 64 点のうち 30 点が「縁の外」と
    // 判定され、4枚とも earcut ＋ 適応細分の経路に落ちていた。同じ形をビルダー
    // で作ると 4 枚とも構造格子を通る。三角形数の比は最大 17.5 倍だった
    // （`tess_density_probe`）。
    //
    // 境界が囲む矩形で見れば、内部の等パラメータ線も「縁」になる。矩形の外に
    // 出る境界（斜めに切られた面など）は、これでも弾かれる。
    let (mut u_min, mut u_max) = (f64::INFINITY, f64::NEG_INFINITY);
    let (mut v_min, mut v_max) = (f64::INFINITY, f64::NEG_INFINITY);
    for uv in &ring.uv {
        u_min = u_min.min(uv.x);
        u_max = u_max.max(uv.x);
        v_min = v_min.min(uv.y);
        v_max = v_max.max(uv.y);
    }
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
        if std::env::var("ZENITH_GRID_WHY").is_ok() {
            let off = ring.uv.iter().filter(|uv| !on_border(uv)).count();
            eprintln!("GRIDWHY off_border {} of {}", off, ring.uv.len());
        }
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
    } else if counts.len() == 3 && degenerate_side_counts(counts).is_some() {
        // **極を持つ面は、境界が3辺しかありません。** 円錐の側面や、他カーネル
        // から読んだ回転面がそれで、潰れた側（頂点）には稜がありません。
        // 4辺の規則にも「全部同じ」にも当てはまらないので、格子を張れる形
        // なのに毎回断られ、earcut ＋ 適応細分へ落ちていました。読んだ円錐は
        // それで、自前の円錐の 5〜10 倍の三角形になり（16分割で 16,874 対
        // 2,046）、しかも粗密が偏るので断面の輪郭が乱れていました。
        //
        // 潰れた行/列がまるごと1点に潰れていることは、この下の border_ok が
        // 曲面そのものを見て確かめます。ここで通しても、そこが崩れていれば
        // 格子は採用されません。
        degenerate_side_counts(counts).expect("checked just above")
    } else {
        if std::env::var("ZENITH_GRID_WHY").is_ok() {
            eprintln!("GRIDWHY segment_counts {counts:?}");
        }
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
            surface,
            orientation,
            params,
            uvs,
            fixed,
            rows,
            columns,
        ));
    }
    if std::env::var("ZENITH_GRID_WHY").is_ok() {
        eprintln!("GRIDWHY no_orientation_matched across={across} along={along}");
    }
    None
}

#[allow(clippy::too_many_arguments)]
fn build_grid_mesh(
    surface: &zenith_geom::NurbsSurface3,
    orientation: Orientation,
    _params: &TessellationParams,
    uvs: Vec<Point2>,
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

    // 構造格子は既に境界エッジと整合した解像度を持っているため、
    // 格子内部の不規則な最長辺二分細分はスキップし、規則的トポロジーを保つ。

    let mut mesh = TriangleMesh::new();
    for (index, uv) in uvs.iter().enumerate() {
        mesh.positions
            .push(match fixed.get(index).copied().flatten() {
                Some(point) => point,
                None => surface.evaluate(uv.x, uv.y),
            });
        mesh.normals
            .push(oriented_normal(surface, *uv, orientation));
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

#[cfg(test)]
mod tests {
    use super::*;

    fn boundary_missing(triangles: &[[usize; 3]], ranges: &[std::ops::Range<usize>]) -> usize {
        let mut edges: std::collections::HashSet<(usize, usize)> = Default::default();
        for triangle in triangles {
            for corner in 0..3 {
                let (a, b) = (triangle[corner], triangle[(corner + 1) % 3]);
                edges.insert(if a < b { (a, b) } else { (b, a) });
            }
        }
        ranges
            .iter()
            .map(|range| {
                (0..range.len())
                    .filter(|offset| {
                        let a = range.start + *offset;
                        let b = range.start + (*offset + 1) % range.len();
                        let key = if a < b { (a, b) } else { (b, a) };
                        !edges.contains(&key)
                    })
                    .count()
            })
            .sum()
    }

    #[test]
    fn flat_boundary_ear_is_flipped_without_losing_the_boundary() {
        let uvs = vec![
            Point2::new(0.0, 0.0),
            Point2::new(0.5, 0.0),
            Point2::new(1.0, 0.0),
            Point2::new(0.5, 1.0),
        ];
        let rings = vec![BoundaryRing {
            uv: uvs.clone(),
            points: vec![Point3::origin(); 4],
            segments: vec![1, 1, 1, 1],
        }];
        let ranges = vec![0..4];
        let protected = [(0, 1), (1, 2), (2, 3), (0, 3)].into_iter().collect();
        let mut triangles = vec![[0, 1, 2], [0, 2, 3]];

        repair_boundary_ears(&uvs, &rings, &ranges, false, &protected, &mut triangles);

        let area2 = |triangle: [usize; 3]| {
            let (a, b, c) = (uvs[triangle[0]], uvs[triangle[1]], uvs[triangle[2]]);
            (b.x - a.x) * (c.y - a.y) - (b.y - a.y) * (c.x - a.x)
        };
        assert!(triangles
            .iter()
            .all(|triangle| area2(*triangle).abs() > 1e-12));
        assert_eq!(boundary_missing(&triangles, &ranges), 0);
    }

    #[test]
    fn a_hole_touching_the_outer_ring_is_bridged_into_the_earcut_input() {
        let uvs = vec![
            Point2::new(0.0, 0.0),
            Point2::new(4.0, 0.0),
            Point2::new(4.0, 4.0),
            Point2::new(0.0, 4.0),
            Point2::new(0.0, 2.0),
            Point2::new(0.0, 2.0),
            Point2::new(1.0, 3.0),
            Point2::new(2.0, 2.0),
            Point2::new(1.0, 1.0),
        ];
        let ranges = vec![0..5, 5..9];
        let flat = uvs.iter().flat_map(|uv| [uv.x, uv.y]).collect::<Vec<_>>();

        let mut triangles = earcut_boundary_rings(&uvs, &ranges, &flat, &[5]);
        reinsert_dropped_boundary_points(&uvs, &ranges, &mut triangles);

        assert!(!triangles.is_empty());
        // 接点は外周側と穴側で別indexのままなので、weld前は接点の両隣2辺だけ
        // canonical indexが一致しない。座標は同じで、最終weldで1頂点になる。
        assert!(boundary_missing(&triangles, &ranges) <= 2);
        let area = triangles
            .iter()
            .map(|triangle| {
                let (a, b, c) = (uvs[triangle[0]], uvs[triangle[1]], uvs[triangle[2]]);
                ((b.x - a.x) * (c.y - a.y) - (b.y - a.y) * (c.x - a.x)).abs() * 0.5
            })
            .sum::<f64>();
        assert!((area - 14.0).abs() < 1e-12, "area was {area}");
    }
}
