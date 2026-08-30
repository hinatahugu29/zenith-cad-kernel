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

    // `ZENITH_FACE_OWNER_WHY=1` のときだけ、三角形がどの面から来たかを覚えて
    // おきます。溶接して1つのメッシュになったあとでは、非多様体の稜を見ても
    // **どの面とどの面の間で開いているのかが分かりません**。稜の刻みが両側で
    // 揃っているかを疑うには、その2枚を名指しできる必要があります。
    let mut owners: Vec<(u64, std::ops::Range<usize>)> = Vec::new();
    let attribute = std::env::var_os("ZENITH_FACE_OWNER_WHY").is_some();

    let mut mesh = tessellate_shell_stitched_owned(
        &solid.outer_shell,
        params,
        &plan,
        attribute.then_some(&mut owners),
    );
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

    weld(&mut mesh, crate::surface_tess::WELD_TOLERANCE);
    if attribute {
        explain_face_owners(&mesh, &owners);
    }
    mesh
}

/// 溶接前の三角形添字の範囲を面ごとに控えながら、殻をメッシュにする。
fn tessellate_shell_stitched_owned(
    shell: &Shell,
    params: &TessellationParams,
    plan: &SamplePlan,
    mut owners: Option<&mut Vec<(u64, std::ops::Range<usize>)>>,
) -> TriangleMesh {
    let mut mesh = TriangleMesh::new();
    for face in &shell.faces {
        let start = mesh.indices.len();
        mesh.merge(&tessellate_face_stitched(face, params, plan));
        if let Some(owners) = owners.as_mut() {
            owners.push((face.id, start..mesh.indices.len()));
        }
    }
    mesh
}

/// 溶接後のメッシュで、ちょうど2枚に共有されていない稜を、**それを出した面の
/// 番号つきで**並べる。
///
/// 溶接は頂点を潰すだけで三角形の並び順を変えないので、溶接前に控えた範囲が
/// そのまま使えます。
fn explain_face_owners(mesh: &TriangleMesh, owners: &[(u64, std::ops::Range<usize>)]) {
    let face_of = |triangle: usize| -> Option<u64> {
        owners
            .iter()
            .find(|(_, range)| range.contains(&triangle))
            .map(|(id, _)| *id)
    };
    let mut uses: BTreeMap<(u32, u32), Vec<usize>> = BTreeMap::new();
    for (index, triangle) in mesh.indices.iter().enumerate() {
        for step in 0..3 {
            let (a, b) = (triangle[step], triangle[(step + 1) % 3]);
            if a == b {
                continue;
            }
            uses.entry(if a < b { (a, b) } else { (b, a) })
                .or_default()
                .push(index);
        }
    }
    let bad: Vec<_> = uses.iter().filter(|(_, tris)| tris.len() != 2).collect();
    eprintln!("OWNERWHY 非多様体の稜 {} 本", bad.len());
    for (id, range) in owners {
        eprintln!("OWNERWHY   面 {id} の三角形は {}..{}", range.start, range.end);
    }
    let mut pairs: BTreeMap<Vec<Option<u64>>, usize> = BTreeMap::new();
    for (_, tris) in &bad {
        let mut faces: Vec<Option<u64>> = tris.iter().map(|t| face_of(*t)).collect();
        faces.sort();
        faces.dedup();
        *pairs.entry(faces).or_insert(0) += 1;
    }
    for (faces, count) in &pairs {
        eprintln!("OWNERWHY   面 {faces:?} が出した稜 {count} 本");
    }
    for ((a, b), tris) in bad.iter().take(6) {
        let (pa, pb) = (mesh.positions[*a as usize], mesh.positions[*b as usize]);
        eprintln!(
            "OWNERWHY   ({:.6},{:.6},{:.6})-({:.6},{:.6},{:.6}) 使用 {}、面 {:?}",
            pa.x,
            pa.y,
            pa.z,
            pb.x,
            pb.y,
            pb.z,
            tris.len(),
            tris.iter().map(|t| face_of(*t)).collect::<Vec<_>>()
        );
    }
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
    if std::env::var_os("ZENITH_FACE_EDGES_WHY").is_some() {
        for (index, wire) in std::iter::once(&face.outer_wire)
            .chain(face.inner_wires.iter())
            .enumerate()
        {
            for oriented in &wire.edges {
                let (a, b) = (
                    oriented.edge.start_vertex.point,
                    oriented.edge.end_vertex.point,
                );
                eprintln!(
                    "FACEEDGE face {} wire {} edge {} seg {} ({:.6},{:.6},{:.6})-({:.6},{:.6},{:.6}) 長さ {:.6}",
                    face.id,
                    index,
                    oriented.edge.id,
                    plan.segments_for(oriented.edge.id),
                    a.x,
                    a.y,
                    a.z,
                    b.x,
                    b.y,
                    b.z,
                    (b - a).norm()
                );
            }
        }
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

    let mesh = match &face.geometry {
        FaceGeometry::Plane(_) => patch_mesh(&rings, None, face.orientation, params),
        FaceGeometry::Nurbs(surface) => patch_mesh(&rings, Some(surface), face.orientation, params),
        _ => crate::surface_tess::tessellate_face(face, params),
    };

    // **画面に出る枚数を、面ごとに数える口**（9-G の G3。4-151）。
    //
    // G3 は長く `ZENITH_TRIM_WHY=1` の値で測っていましたが、あれは
    // `trimmed_uv_triangulation`——**積分と、面を1枚だけ刻む経路**の値です。
    // 画面に出るのはこちら（縫合）なので、狙っているものを測るならここで
    // 数えます（4-150 で取り違えに気づきました）。
    if std::env::var_os("ZENITH_PATCH_WHY").is_some() {
        eprintln!(
            "PATCHWHY 面 {} 枚（境界のリング {}、{}）",
            mesh.num_triangles(),
            rings.len(),
            match &face.geometry {
                FaceGeometry::Plane(_) => "平面",
                FaceGeometry::Nurbs(_) => "曲面",
                _ => "その他",
            }
        );
    }
    mesh
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
            let mut dropped_here = 0usize;
            let (t_min, t_max) = segment.curve.param_range();
            for step in 0..=segments {
                let fraction = step as f64 / segments as f64;
                let t = t_min + (t_max - t_min) * fraction;
                let here = segment.curve.evaluate(t);
                // 位置は稜そのものから。p-curve はパラメータを取るためだけに使う。
                let point = oriented.evaluate_normalized(fraction);
                // **落とす基準は、溶接の距離に合わせます。**
                //
                // ここが 1e-12 で、あとから掛かる `weld` が 1e-7 だと、その
                // 隙間に入った標本は「境界の点としては別、溶接では同じ」に
                // なります。すると、両方を使っている三角形は溶接で潰れ、
                // `weld` がそれを外し、外した跡が穴になります——実測
                // （4-117、傾けたトーラス × 箱の差、24分割）で1枚の面から
                // 622枚が消え、非多様体の稜が 121本残っていました。
                //
                // **落としても継ぎ目は開きません。** どの点を落とすかは稜の
                // 幾何と刻み方だけで決まり、隣の面も同じ稜を同じ割合で標本
                // するので、同じ点を落とします。
                let duplicate = points
                    .last()
                    .map(|last: &Point3| {
                        (point - *last).norm() <= crate::surface_tess::WELD_TOLERANCE
                    })
                    .unwrap_or(false);
                if !duplicate {
                    uv.push(here);
                    points.push(point);
                } else {
                    dropped_here += 1;
                    if std::env::var_os("ZENITH_DROP_WHY").is_some() {
                        eprintln!(
                            "DROPSTEP face {} edge {} step {step}/{segments}",
                            face.id, segment.edge_id
                        );
                    }
                }
            }
            if std::env::var_os("ZENITH_DROP_WHY").is_some() && dropped_here > 0 {
                eprintln!(
                    "DROPWHY face {} edge {} 刻み {segments} のうち {dropped_here} 点を落とした",
                    face.id, segment.edge_id
                );
            }
        }

        if points.len() > 1
            && (points[points.len() - 1] - points[0]).norm()
                <= crate::surface_tess::WELD_TOLERANCE
        {
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

    if std::env::var_os("ZENITH_TESS_WHY").is_some() {
        // 同じ uv に頂点が2つ以上あると、溶接でそれらが1点になり、両方を
        // 使っている三角形が潰れて外されます（4-117）。境界を作った時点で
        // 重複があるのかを、推測せずに数えます。
        // 見るのは **3D で溶接距離の中に来る対**です。uv で重なっているかでは
        // ありません——潰すかどうかを決めるのは `weld` で、`weld` は 3D の
        // 距離で束ねます（4-118）。
        let mut duplicates = 0usize;
        let mut worst = 0.0f64;
        let mut pairs: Vec<(usize, usize)> = Vec::new();
        for left in 0..uvs.len() {
            for right in (left + 1)..uvs.len() {
                let (Some(a), Some(b)) = (fixed[left], fixed[right]) else {
                    continue;
                };
                let gap = (b - a).norm();
                if gap <= crate::surface_tess::WELD_TOLERANCE {
                    duplicates += 1;
                    worst = worst.max(gap);
                    pairs.push((left, right));
                }
            }
        }
        if duplicates > 0 {
            eprintln!(
                "TESSWHY   境界の点 {} 個のうち、同じ uv に重なっている対 {duplicates}（最大の隔たり {worst:.3e}）",
                uvs.len()
            );
            {
                for (left, right) in pairs.iter().copied() {
                    let place = |index: usize| {
                        for (ring, range) in ring_ranges.iter().enumerate() {
                            if range.contains(&index) {
                                let offset = index - range.start;
                                let mut start = 0usize;
                                for (edge, segments) in
                                    rings[ring].segments.iter().enumerate()
                                {
                                    if offset < start + *segments {
                                        return format!(
                                            "ring {ring} 位置 {offset}/{}（{edge} 本目の稜の {} 点目）",
                                            range.len(),
                                            offset - start
                                        );
                                    }
                                    start += *segments;
                                }
                                return format!("ring {ring} 位置 {offset}/{}（稜の外）", range.len());
                            }
                        }
                        "不明".to_string()
                    };
                    eprintln!(
                        "TESSWHY     重複: {} と {}",
                        place(left),
                        place(right)
                    );
                }
            }
        }
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

    // 修復で直らなかった耳を外し、空いた境界を張り直す。
    //
    // `repair_boundary_ears` は内部の対角線を入れ替えるだけなので、**境界の
    // 点が3点より多く一直線に並ぶと手が出ません**——入れ替えた先も同じ直線の
    // 上なので、潰れた枚数が減らないからです。そこで潰れたままの耳を外して
    // 点を「使われていない」状態へ戻し、同じ受け皿に扇で張り直させます。
    let dropped = drop_flat_boundary_triangles(&uvs, &ring_ranges, &mut triangles);
    if dropped > 0 {
        reinsert_dropped_boundary_points(&uvs, &ring_ranges, &mut triangles);
        explain_flat("潰れた耳を外して張り直した後", &triangles, &uvs);
    }

    // **境界の辺が抜けている穴を埋めます**（4-207）。挿し戻しと耳の修理を
    // 通っても、「点は使われているのに辺だけ無い」形が残ります。
    fill_missing_boundary_edges(&uvs, &ring_ranges, &mut triangles);

    // **境界の辺が全部そろっているかを数えます**（`ZENITH_EARCUT_WHY=1`）。
    //
    // リングの連続する2点を結ぶ辺は、三角形分割に必ず現れなければなりません。
    // 現れないと、そこは**隣の面から見て相手のいない稜**になり、メッシュに
    // 穴が開きます。earcut と、そのあとの挿し戻し・耳の修理（4-85、4-86）が
    // どこまで効いたかを、推測せずに見るための口です。
    if std::env::var_os("ZENITH_EARCUT_WHY").is_some() {
        let mut present: std::collections::HashSet<(usize, usize)> = Default::default();
        for triangle in &triangles {
            for corner in 0..3 {
                let (a, b) = (triangle[corner], triangle[(corner + 1) % 3]);
                present.insert(if a < b { (a, b) } else { (b, a) });
            }
        }
        let mut missing = 0usize;
        let mut total = 0usize;
        for range in &ring_ranges {
            let count = range.len();
            for offset in 0..count {
                let a = range.start + offset;
                let b = range.start + (offset + 1) % count;
                total += 1;
                if !present.contains(&if a < b { (a, b) } else { (b, a) }) {
                    missing += 1;
                }
            }
        }
        eprintln!(
            "EARCUTWHY   境界の辺 {total} 本のうち {missing} 本が三角形分割に無い（三角形 {}）",
            triangles.len()
        );
        // **本数だけでは直せません。** どの辺かを名指しします。
        for range in &ring_ranges {
            let count = range.len();
            for offset in 0..count {
                let a = range.start + offset;
                let b = range.start + (offset + 1) % count;
                if present.contains(&if a < b { (a, b) } else { (b, a) }) {
                    continue;
                }
                let previous = range.start + (offset + count - 1) % count;
                let following = range.start + (offset + 2) % count;
                eprintln!(
                    "EARCUTWHY     欠けた辺 [{a}]->[{b}]  uv ({:.9},{:.9})->({:.9},{:.9})  長さ {:.3e}",
                    uvs[a].x,
                    uvs[a].y,
                    uvs[b].x,
                    uvs[b].y,
                    (uvs[b] - uvs[a]).norm()
                );
                // 共線かどうかは、その場で見ておきます（earcut が落とすのは
                // 共線・重複の点なので）。
                let cross = |p: Point2, q: Point2, r: Point2| {
                    (q.x - p.x) * (r.y - p.y) - (q.y - p.y) * (r.x - p.x)
                };
                eprintln!(
                    "EARCUTWHY       前後との外積: 前 {:.3e}、後 {:.3e}",
                    cross(uvs[previous], uvs[a], uvs[b]),
                    cross(uvs[a], uvs[b], uvs[following])
                );
                if let (Some(pa), Some(pb)) = (fixed[a], fixed[b]) {
                    eprintln!(
                        "EARCUTWHY       3D ({:.6},{:.6},{:.6})->({:.6},{:.6},{:.6}) 長さ {:.3e}",
                        pa.x, pa.y, pa.z, pb.x, pb.y, pb.z, (pb - pa).norm()
                    );
                }
                // **その辺を「またいでいる」三角形を出します。**
                // 直す側は、この三角形を割ることになります。まず現物を
                // 見てから設計してください（4-85 で推測して悪化させました）。
                for (index, triangle) in triangles.iter().enumerate() {
                    let touches_a = triangle.contains(&a);
                    let touches_b = triangle.contains(&b);
                    if !touches_a && !touches_b {
                        continue;
                    }
                    eprintln!(
                        "EARCUTWHY       三角形 {index}: [{}] [{}] [{}]{}{}",
                        triangle[0],
                        triangle[1],
                        triangle[2],
                        if touches_a { "  <-a" } else { "" },
                        if touches_b { "  <-b" } else { "" }
                    );
                }
            }
        }
    }

    if let Some(surface) = surface {
        // 境界の点は先頭に固めて入っている。その数を渡して、**境界の点
        // どうしを結ぶ辺は連続していなくても割らない**ようにする（4-84）。
        let boundary_vertex_count = uvs.len();
        // **細分の前後を数える口**（9-H の H5）。earcut が返した枚数と、
        // たわみを満たすまでに何倍になったかが分かります。膨らむ倍率が
        // 大きいなら、悪いのは細分ではなく**渡している三角形の形**です。
        let before_refinement = triangles.len();
        crate::surface_tess::refine_uv_triangulation_protected(
            surface,
            params,
            &mut uvs,
            &mut triangles,
            &protected,
            boundary_vertex_count,
            &ring_ranges,
            // **ここは表示・書き出し用のメッシュ**なので、パラメータ格子の
            // 条項は掛けません（弦誤差の基準はそのまま掛かります。4-150）。
            false,
        );
        explain_flat("適応細分後", &triangles, &uvs);
        if std::env::var_os("ZENITH_PATCH_WHY").is_some() {
            eprintln!(
                "PATCHWHY   境界 {boundary_vertex_count} 点、earcut {before_refinement} 枚 -> 細分後 {} 枚（{:.1}倍）",
                triangles.len(),
                triangles.len() as f64 / before_refinement.max(1) as f64
            );
        }
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
        if std::env::var_os("ZENITH_TESS_WHY").is_some() {
            eprintln!("TESSWHY   EMITDROP 3d-zero");
        }
        return;
    }

    let a = uvs[triangle[0] as usize];
    let b = uvs[triangle[1] as usize];
    let c = uvs[triangle[2] as usize];
    let signed = (b.x - a.x) * (c.y - a.y) - (c.x - a.x) * (b.y - a.y);
    if signed == 0.0 {
        if std::env::var_os("ZENITH_TESS_WHY").is_some() {
            eprintln!("TESSWHY   EMITDROP uv-zero");
        }
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

/// 座標が一致する頂点を1つに束ね、面積0の三角形を落とす（27近傍空間ハッシュ）
fn weld(mesh: &mut TriangleMesh, tolerance: f64) {
    let tol = tolerance.max(1e-12);
    let tol_sq = tol * tol;
    let cell_size = tol;
    let cell_coord = |v: f64| (v / cell_size).floor() as i64;
    let cell_key = |p: Point3| (cell_coord(p.x), cell_coord(p.y), cell_coord(p.z));

    let mut grid: std::collections::HashMap<(i64, i64, i64), Vec<u32>> = std::collections::HashMap::new();
    let mut remap = vec![0u32; mesh.positions.len()];
    let mut positions = Vec::new();
    let mut normals = Vec::new();
    let mut uvs = Vec::new();

    for (index, position) in mesh.positions.iter().enumerate() {
        let (cx, cy, cz) = cell_key(*position);
        let mut matched = None;

        'search: for dx in -1..=1 {
            for dy in -1..=1 {
                for dz in -1..=1 {
                    if let Some(candidates) = grid.get(&(cx + dx, cy + dy, cz + dz)) {
                        for &cand_idx in candidates {
                            let cand_pos = positions[cand_idx as usize];
                            let diff: Vec3 = cand_pos - *position;
                            if diff.norm_squared() <= tol_sq {
                                matched = Some(cand_idx);
                                break 'search;
                            }
                        }
                    }
                }
            }
        }

        match matched {
            Some(existing) => remap[index] = existing,
            None => {
                let slot = positions.len() as u32;
                grid.entry((cx, cy, cz)).or_default().push(slot);
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

    let sorted_triangle = |t: [u32; 3]| {
        let mut arr = t;
        arr.sort_unstable();
        arr
    };

    let mut seen: std::collections::HashMap<[u32; 3], usize> = Default::default();
    let mut indices = Vec::with_capacity(mesh.indices.len());
    let (mut collapsed, mut flat, mut duplicated) = (0usize, 0usize, 0usize);
    let collapse_why = std::env::var_os("ZENITH_COLLAPSE_WHY").is_some();
    for (triangle_index, triangle) in mesh.indices.iter().enumerate() {
        let mapped = [
            remap[triangle[0] as usize],
            remap[triangle[1] as usize],
            remap[triangle[2] as usize],
        ];
        if mapped[0] == mapped[1] || mapped[1] == mapped[2] || mapped[2] == mapped[0] {
            collapsed += 1;
            if collapse_why {
                // 束ねられた対を、**uv でどれだけ離れていたか**と一緒に出す。
                // 溶接は 3D の距離で束ねるので、uv で離れた対が束ねられて
                // いるなら、支持曲面のパラメータ付けがそこで縮退している
                // （極や継ぎ目）ということになる。
                for corner in 0..3 {
                    let (a, b) = (corner, (corner + 1) % 3);
                    if mapped[a] != mapped[b] {
                        continue;
                    }
                    let (ua, ub) = (
                        mesh.uvs
                            .get(triangle[a] as usize)
                            .copied()
                            .unwrap_or_default(),
                        mesh.uvs
                            .get(triangle[b] as usize)
                            .copied()
                            .unwrap_or_default(),
                    );
                    let position = positions[mapped[a] as usize];
                    eprintln!(
                        "COLLAPSEWHY uv ({:.9},{:.9})-({:.9},{:.9}) uv 距離 {:.3e} → 3D ({:.6},{:.6},{:.6})",
                        ua.x,
                        ua.y,
                        ub.x,
                        ub.y,
                        (ub - ua).norm(),
                        position.x,
                        position.y,
                        position.z
                    );
                }
            }
            continue;
        }
        let p0 = positions[mapped[0] as usize];
        let p1 = positions[mapped[1] as usize];
        let p2 = positions[mapped[2] as usize];
        if (p1 - p0).cross(&(p2 - p0)).norm() <= 1e-18 {
            flat += 1;
            continue;
        }
        let key = sorted_triangle(mapped);
        if let Some(first) = seen.get(&key).copied() {
            duplicated += 1;
            if collapse_why {
                // **同じ3頂点の三角形を2枚出したのはどこか。**
                // 溶接前の三角形の添字で言う。面ごとの範囲と突き合わせれば、
                // 1枚の面の中なのか、2枚の面にまたがるのかが分かる。
                let (p0, p1, p2) = (
                    positions[mapped[0] as usize],
                    positions[mapped[1] as usize],
                    positions[mapped[2] as usize],
                );
                eprintln!(
                    "DUPWHY 三角形 {} は {} と同じ3頂点 ({:.4} {:.4} {:.4}) ({:.4} {:.4} {:.4}) ({:.4} {:.4} {:.4})",
                    triangle_index,
                    first,
                    p0.x, p0.y, p0.z,
                    p1.x, p1.y, p1.z,
                    p2.x, p2.y, p2.z
                );
            }
        } else {
            seen.insert(key, triangle_index);
            indices.push(mapped);
        }
    }
    let before_flaps = indices.len();
    remove_redundant_flap_triangles(&mut indices);
    if std::env::var_os("ZENITH_TESS_WHY").is_some()
        && (collapsed + flat + duplicated + (before_flaps - indices.len())) > 0
    {
        eprintln!(
            "TESSWHY weld が外した三角形: 頂点が潰れた {collapsed}、面積 0 {flat}、同じ3頂点の重複 {duplicated}、flap {}",
            before_flaps - indices.len()
        );
    }

    mesh.positions = positions;
    mesh.normals = normals;
    mesh.uvs = uvs;
    mesh.indices = indices;
}

/// 閉じた面に1枚だけ重なって付いたflapを除く。
///
/// 外周へ一点で接する穴をearcut用の単一ringへ縫い込むと、接点の直後に
/// `[3, 3, 1]` の辺使用数を持つ三角形が1枚だけ残ることがある。この1枚を
/// 外すと、過剰な2辺は3回から2回へ戻り、1回だけの対角線は消える。
/// それ以外の使用数パターンには触れない。
fn remove_redundant_flap_triangles(indices: &mut Vec<[u32; 3]>) {
    loop {
        let mut edge_uses: BTreeMap<(u32, u32), usize> = BTreeMap::new();
        for triangle in indices.iter() {
            for corner in 0..3 {
                let (a, b) = (triangle[corner], triangle[(corner + 1) % 3]);
                let key = if a < b { (a, b) } else { (b, a) };
                *edge_uses.entry(key).or_default() += 1;
            }
        }
        let redundant = indices.iter().position(|triangle| {
            let mut uses = [0usize; 3];
            for corner in 0..3 {
                let (a, b) = (triangle[corner], triangle[(corner + 1) % 3]);
                let key = if a < b { (a, b) } else { (b, a) };
                uses[corner] = edge_uses.get(&key).copied().unwrap_or(0);
            }
            uses.sort_unstable();
            uses == [1, 3, 3]
        });
        let Some(index) = redundant else {
            return;
        };
        indices.remove(index);
    }
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
/// 境界の上で潰れたままの三角形を外す。
///
/// **これは「捨てる」段ではありません。** 外したあとに
/// [`reinsert_dropped_boundary_points`] を通すための下ごしらえです。
///
/// earcut は、境界の点が一直線に並ぶところに面積 0 の耳を作ります
/// （円錐を斜めに切った面で実測: 共有稜の格子点が 4 点以上そのまま
/// 一直線に乗る）。耳の中の点は**三角形に使われてはいる**ので、
/// `reinsert_dropped_boundary_points` の「使われていない点」には入りません。
/// かといって耳自身は 3D で面積 0 なので、`push_with_uv_winding` が黙って
/// 落とします。**落ちた先で、保護していた境界辺が誰にも使われない稜として
/// 残ります**——隣の面はその辺を持っているので、溶接しても閉じません
/// （4-116）。
///
/// そこで、**同じリングの上だけで潰れている三角形**を先に外し、点を
/// 「使われていない」状態に戻してから受け皿へ渡します。外すのは
///
/// - uv で面積が 0 とみなせる
/// - 3頂点が同じ境界リングに乗っている
///
/// の両方を満たすものだけです。内部の頂点を含む三角形には触れません。
fn drop_flat_boundary_triangles(
    uvs: &[Point2],
    ring_ranges: &[std::ops::Range<usize>],
    triangles: &mut Vec<[usize; 3]>,
) -> usize {
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

    let before = triangles.len();
    triangles.retain(|triangle| {
        let (a, b, c) = (uvs[triangle[0]], uvs[triangle[1]], uvs[triangle[2]]);
        let area2 = (b.x - a.x) * (c.y - a.y) - (b.y - a.y) * (c.x - a.x);
        if area2.abs() > flat_eps {
            return true;
        }
        !ring_ranges
            .iter()
            .any(|range| triangle.iter().all(|vertex| range.contains(vertex)))
    });
    before - triangles.len()
}

/// 境界の辺が抜けている穴を、共通の隣で埋める。
///
/// # 何が起きているのか
///
/// earcut は共線・重複の点を自分で落とします（`filterPoints`）。落とされた
/// 点を挿し戻す段は既にありますが（4-86）、あれは**どの三角形にも使われて
/// いない点**を探します。**点は使われていて、無いのは辺のほう**という形は
/// そこを素通りします。
///
/// 実測（4-207、`cone × torus` を持ち上げた和、24分割）:
///
/// ```text
/// 欠けた辺 [110]->[111]   前の点との外積 -4.733e-29（共線）
///   三角形 42:  [109] [110] [121]   <- a を使う唯一の三角形
///   三角形 141: [120] [121] [111]   <- b を使う
/// ```
///
/// `a` と `b` は**同じ頂点 `[121]` と辺を共有**していて、その間の三角形
/// `[110][111][121]` だけが無い。境界の辺が1本欠け、隣の面から見て相手の
/// いない稜になります。
///
/// # 何をするのか
///
/// 欠けた境界の辺 `(a, b)` について、**`a` とも `b` とも辺を共有している
/// 頂点 `c`** を探し、三角形 `(a, b, c)` を足します。向きは既にある三角形に
/// 合わせます。
///
/// **点は1つも増えません。** 覆えていなかったところを覆うだけです。
/// 見つからなければ何もしません（もっともらしい三角形を作るより、
/// 埋まらないほうが良い）。
fn fill_missing_boundary_edges(
    uvs: &[Point2],
    ring_ranges: &[std::ops::Range<usize>],
    triangles: &mut Vec<[usize; 3]>,
) {
    let signed_area = |t: &[usize; 3]| {
        let (a, b, c) = (uvs[t[0]], uvs[t[1]], uvs[t[2]]);
        (b.x - a.x) * (c.y - a.y) - (b.y - a.y) * (c.x - a.x)
    };

    for _round in 0..8 {
        let mut edges: std::collections::HashSet<(usize, usize)> = Default::default();
        let mut neighbours: std::collections::HashMap<usize, std::collections::HashSet<usize>> =
            std::collections::HashMap::new();
        for triangle in triangles.iter() {
            for corner in 0..3 {
                let (a, b) = (triangle[corner], triangle[(corner + 1) % 3]);
                edges.insert(if a < b { (a, b) } else { (b, a) });
                neighbours.entry(a).or_default().insert(b);
                neighbours.entry(b).or_default().insert(a);
            }
        }

        let mut added = false;
        for range in ring_ranges {
            let count = range.len();
            if count < 3 {
                continue;
            }
            for offset in 0..count {
                let a = range.start + offset;
                let b = range.start + (offset + 1) % count;
                if edges.contains(&if a < b { (a, b) } else { (b, a) }) {
                    continue;
                }
                let (Some(from), Some(to)) = (neighbours.get(&a), neighbours.get(&b)) else {
                    continue;
                };
                // 両方と辺を共有している頂点。複数あれば、面積のいちばん
                // 小さいものを採ります——覆えていない隙間はいちばん狭い
                // ところなので。
                let mut best: Option<(f64, [usize; 3])> = None;
                for candidate in from.intersection(to) {
                    let candidate = *candidate;
                    if candidate == a || candidate == b {
                        continue;
                    }
                    let mut triangle = [a, b, candidate];
                    let area = signed_area(&triangle);
                    if area == 0.0 {
                        continue;
                    }
                    // 向きは、既にある三角形に合わせます。
                    let reference = triangles
                        .iter()
                        .find(|existing| existing.contains(&a))
                        .map(|existing| signed_area(existing))
                        .unwrap_or(area);
                    if area * reference < 0.0 {
                        triangle = [b, a, candidate];
                    }
                    let size = area.abs();
                    if best.as_ref().map(|(worst, _)| size < *worst).unwrap_or(true) {
                        best = Some((size, triangle));
                    }
                }
                if let Some((_, triangle)) = best {
                    triangles.push(triangle);
                    added = true;
                }
            }
        }
        if !added {
            return;
        }
    }
}

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

    // 面の向き。**隣の三角形からは取れません。**
    //
    // 境界の点が一直線に並ぶところでは、耳もその隣も潰れています（円錐を斜めに
    // 切った面で実測: 共有稜の格子点 8・9・10・11 が同一直線に乗る）。隣の符号で
    // 向きを決めていると、そこで「向きが取れない」という理由だけで修復を諦め、
    // 直らなかった耳は後段の 3D 面積 0 で捨てられます。**捨てると、保護して
    // いたはずの境界辺が誰にも使われないまま開きます**——隣の面はその2本を
    // 持っているので、溶接後に T 字が残ります（4-116）。
    //
    // 向きはリング1つにつき1つなので、外周リングの符号を使います。
    let ring_sign = {
        let mut twice_area = 0.0;
        if let Some(range) = ring_ranges.first() {
            for index in range.clone() {
                let a = uvs[index];
                let b = uvs[if index + 1 < range.end {
                    index + 1
                } else {
                    range.start
                }];
                twice_area += a.x * b.y - b.x * a.y;
            }
        }
        if twice_area >= 0.0 {
            1.0
        } else {
            -1.0
        }
    };

    let why = std::env::var_os("ZENITH_TESS_WHY").is_some();
    let mut skipped: std::collections::HashSet<usize> = Default::default();
    // **横移動（2 → 2）で一度でも触った辺**。二度と触りません。
    //
    // 端の無い塊（悪い三角形が閉じた輪になっている）では、どこを選んでも悪い
    // 枚数が減らないので、`new_bad < old_bad` だけでは一歩も動けません
    // （4-125 実測: 断った 364 件が全部これ）。そこで**端が1つも無いときに
    // 限って** 2 → 2 の入れ替えを許します。
    //
    // ただし条件を緩めるだけでは**循環します**（入れ替えて、戻して、また
    // 入れ替える）。歯止めは、横移動で消した辺と作った辺を覚えて二度と触ら
    // ないことです。横移動は1回につき辺を恒久的に使い切るので、回数は辺の
    // 本数で頭打ちになり、循環しません。
    let mut lateral_taboo: std::collections::HashSet<(usize, usize)> = Default::default();
    for _round in 0..triangles.len() * 4 {
        let mut edge_uses: std::collections::HashMap<(usize, usize), Vec<usize>> =
            Default::default();
        for (index, triangle) in triangles.iter().enumerate() {
            for corner in 0..3 {
                let (a, b) = (triangle[corner], triangle[(corner + 1) % 3]);
                let key = if a < b { (a, b) } else { (b, a) };
                edge_uses.entry(key).or_default().push(index);
            }
        }

        // **鎖の端から解きます。**
        //
        // 悪い三角形は1枚ずつ独立には出ません。境界に沿って**鎖**になります
        // （4-117）。鎖の途中を選ぶと、入れ替える相手も同じく悪いので、
        // 悪い枚数は 2 → 2 のままで、`new_bad >= old_bad` で必ず止まります。
        //
        // 実測（4-124、`box × cylinder (both turned)` の積、24分割）: 断った
        // 理由は **3113 件すべて「悪い枚数が減らない 2 → 2」**でした。
        // 「対角が既にある」も「面積が変わる」も1件も出ません。**つまり
        // 一度も端に当たっていませんでした。**
        //
        // 端（相手が良い三角形であるもの）を先に選べば 2 → 1 になるので、
        // そこから鎖をほどけます。端が無ければ従来どおり最初の1枚を選びます。
        let is_bad = |index: usize| needs_repair(triangles[index]);
        let touches_a_good_neighbour = |index: usize| {
            let triangle = triangles[index];
            (0..3).any(|corner| {
                let (u, v) = (triangle[corner], triangle[(corner + 1) % 3]);
                let shared = if u < v { (u, v) } else { (v, u) };
                if protected.contains(&shared) {
                    return false;
                }
                let Some(uses) = edge_uses.get(&shared) else {
                    return false;
                };
                if uses.len() != 2 {
                    return false;
                }
                let neighbour = if uses[0] == index { uses[1] } else { uses[0] };
                !is_bad(neighbour)
            })
        };
        let candidates = || {
            (0..triangles.len())
                .filter(|index| !skipped.contains(index) && is_bad(*index))
        };
        let end_of_a_chain = candidates().find(|index| touches_a_good_neighbour(*index));
        if end_of_a_chain.is_none() && why && candidates().next().is_some() {
            // **端が1つも無い。** 悪い三角形が閉じた輪になっていると
            // こうなります。輪には端が無いので、どこを選んでも 2 → 2 です。
            eprintln!(
                "TESSWHY   FLIPWHY 端の無い悪い三角形が {} 枚（閉じた輪か）",
                candidates().count()
            );
        }
        // 端が1つも無いときだけ、横移動を許します。端があるなら「必ず減る」
        // 入れ替えだけで鎖はほどけるので、緩める必要がありません。
        let allow_lateral = end_of_a_chain.is_none();
        let Some(flat_index) = end_of_a_chain.or_else(|| candidates().next()) else {
            break;
        };

        let flat = triangles[flat_index];
        let mut repaired = false;
        for corner in 0..3 {
            let (u, v) = (flat[corner], flat[(corner + 1) % 3]);
            let shared = if u < v { (u, v) } else { (v, u) };
            if protected.contains(&shared) || lateral_taboo.contains(&shared) {
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
                if why {
                    eprintln!("TESSWHY   FLIPWHY {flat:?} 角 {corner}: 対角 {diagonal:?} は既にある");
                }
                continue;
            }

            // 潰れた隣とも入れ替えられるように、向きはリングから取ります。
            let neighbour_area2 = area2(neighbour);
            let sign = if neighbour_area2.abs() > flat_eps {
                neighbour_area2
            } else {
                ring_sign
            };
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
            let lateral = allow_lateral
                && old_bad == 2
                && new_bad == old_bad
                && !lateral_taboo.contains(&diagonal);
            if new_bad >= old_bad && !lateral {
                if why {
                    eprintln!("TESSWHY   FLIPWHY {flat:?} 角 {corner}: 悪い枚数が減らない {old_bad} -> {new_bad}");
                }
                continue;
            }
            let old_area = area2(flat).abs() + area2(neighbour).abs();
            let new_area = first_area + second_area;
            let area_tolerance = flat_eps * 16.0 + old_area * 1e-9;
            if (new_area - old_area).abs() > area_tolerance {
                if why {
                    eprintln!("TESSWHY   FLIPWHY {flat:?} 角 {corner}: 面積が変わる {old_area:.3e} -> {new_area:.3e}");
                }
                continue;
            }

            if lateral {
                // 消した辺と作った辺の両方を封じます。戻す道が無くなるので、
                // 循環しません。
                lateral_taboo.insert(shared);
                lateral_taboo.insert(diagonal);
                if why {
                    eprintln!(
                        "TESSWHY   FLIPWHY {flat:?} 角 {corner}: 端が無いので横移動 {shared:?} -> {diagonal:?}"
                    );
                }
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
    fn a_single_flap_with_two_overused_edges_is_removed() {
        let mut triangles = vec![
            [0, 1, 3],
            [1, 0, 4],
            [1, 2, 5],
            [2, 1, 6],
            [0, 1, 2],
        ];

        remove_redundant_flap_triangles(&mut triangles);

        assert_eq!(triangles.len(), 4);
        assert!(!triangles.contains(&[0, 1, 2]));
        let count = |left: u32, right: u32| {
            triangles
                .iter()
                .flat_map(|triangle| {
                    (0..3).map(move |corner| {
                        let (a, b) = (triangle[corner], triangle[(corner + 1) % 3]);
                        if a < b { (a, b) } else { (b, a) }
                    })
                })
                .filter(|edge| *edge == (left, right))
                .count()
        };
        assert_eq!(count(0, 1), 2);
        assert_eq!(count(1, 2), 2);
        assert_eq!(count(0, 2), 0);
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
