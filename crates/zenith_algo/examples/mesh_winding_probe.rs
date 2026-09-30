//! **三角形の巻き方が揃っているかを数える**（4-569）。
//!
//! # なぜ要るのか
//!
//! 4-568 で、和と差の答えに「**同じ向きで 2 回以上使われた稜 55 本**」が
//! 出ました（**折り返し 10 本、別の 2 枚が同じ向き 45 本**）。
//! **和は面 49 枚、差は 37 枚なのに、数が同じ**です。
//!
//! **差は、検証を通って返り、OCC と 1.58e-5 で合っている立体**です。
//! **そこに 45 本あるなら、ブーリアンが作ったものではありません。**
//!
//! **読んだ立体そのものを、同じ物差しで数えます。**
//! **入口で既に 45 本あれば、これは刻み方の性質**で、
//! **ブーリアンの欠陥ではありません。**
//!
//! # 4-573 で足したもの
//!
//! **「巻き付いた面（全周を1つで持つ面）が怪しい」という仮説を測って
//! 外しました**——巻き付いた面と、巻き方が食い違う面は重なりがゼロです。
//! **代わりに見えたのは、次数 2×2 の球パッチ（`sphere_patch_for_boundary`
//! の署名）が、読んだファイルの自由曲面と接する所に集中している**という形
//! です。**まだ数値で法線を突き合わせるところまでは進んでいません。**

use zenith_io::StepImporter;
use zenith_tess::{face_triangle_counts, tessellate_solid, TessellationParams};

type CrossedExample = std::collections::BTreeMap<(u64, u64), (zenith_math::Point3, zenith_math::Point3)>;

/// **読んだファイルを、刻みを振って数えます**（4-581）。
///
/// # なぜ要るのか
///
/// 4-576 の巻き方の直しは、**`linkrods.step` の 24 分割だけで測っていました**。
/// **`uv-zero` の受け皿は、そもそも `screw.step` のために在ります**
/// （4-286。**uv で潰れた三角形 41 枚。捨てたら穴が 14 本**）——
/// **直しが触るのは、まさにその受け皿**です。**測っていない所で
/// 壊していないか、確かめます。**
///
/// **1 つの刻みだけ見るのは測り方の穴**です（4-296。**`screw.step` は
/// 24 が緑でも 16・20・32・48 で壊れていたことがあります**）。
fn sweep_foreign(name: &str, path: &str) {
    let solids = match StepImporter::import_solids_from_file(path) {
        Ok(solids) => solids,
        Err(reason) => {
            println!("  {name}: **読めません**（{reason}）");
            return;
        }
    };
    let Some(solid) = solids
        .into_iter()
        .max_by_key(|solid| solid.outer_shell.faces.len())
    else {
        println!("  {name}: **立体が 0 個**");
        return;
    };
    let cell = |point: zenith_math::Point3| {
        (
            (point.x / 1e-6).round() as i64,
            (point.y / 1e-6).round() as i64,
            (point.z / 1e-6).round() as i64,
        )
    };
    for divisions in [8usize, 12, 16, 20, 24, 32, 48] {
        let params = TessellationParams {
            u_divisions: divisions,
            v_divisions: divisions,
        };
        let mesh = tessellate_solid(&solid, &params);
        let counts = face_triangle_counts(&solid, &params);
        let mut undirected: std::collections::BTreeMap<
            ((i64, i64, i64), (i64, i64, i64)),
            usize,
        > = std::collections::BTreeMap::new();
        for triangle in &mesh.indices {
            for pair in [
                (triangle[0] as usize, triangle[1] as usize),
                (triangle[1] as usize, triangle[2] as usize),
                (triangle[2] as usize, triangle[0] as usize),
            ] {
                let (a, b) = (cell(mesh.positions[pair.0]), cell(mesh.positions[pair.1]));
                let (lo, hi) = if a <= b { (a, b) } else { (b, a) };
                *undirected.entry((lo, hi)).or_insert(0) += 1;
            }
        }
        let holes = undirected.values().filter(|count| **count == 1).count();
        let overlaps = undirected.values().filter(|count| **count > 2).count();
        let (folded, crossed) = count_mismatches(&mesh.positions, &mesh.indices, &counts);
        println!(
            "  {name:<10} 刻み {divisions:>2}  三角形 {:>7}  穴 {holes:>3}  重なり {overlaps:>3}  折り返し {folded:>3}  別の 2 枚が同じ向き {crossed:>3}",
            mesh.indices.len()
        );
        // **折り返しが出た刻みでは、持っている面の姿も書きます**（4-584）。
        if folded > 0 && std::env::var_os("ZENITH_FOLDED_FACES").is_some() {
            let ids = folded_face_ids(&mesh.positions, &mesh.indices, &counts);
            describe_faces(&solid, &ids);
            let folds = folded_edges(&mesh.positions, &mesh.indices, &counts);
            where_are_folds(&solid, &folds);
            folds_in_uv(&solid, &mesh, &counts);
            for id in &ids {
                folds_in_uv_before_weld(&solid, *id, &params);
            }
        }
    }
}

/// **`count()` と同じ物差しで、折り返し・別の2枚の数だけを返します**（4-574）。
/// **三角形を裏返す実験のために、頂点順を差し替えた `indices` を渡せる**
/// よう、`count()` から折り返し／数え方の芯だけを取り出したものです。
fn count_mismatches(
    positions: &[zenith_math::Point3],
    indices: &[[u32; 3]],
    counts: &[(u64, usize)],
) -> (usize, usize) {
    let cell = |point: zenith_math::Point3| {
        (
            (point.x / 1e-6).round() as i64,
            (point.y / 1e-6).round() as i64,
            (point.z / 1e-6).round() as i64,
        )
    };
    let mut directed: std::collections::BTreeMap<((i64, i64, i64), (i64, i64, i64)), usize> =
        std::collections::BTreeMap::new();
    for triangle in indices {
        for pair in [
            (triangle[0] as usize, triangle[1] as usize),
            (triangle[1] as usize, triangle[2] as usize),
            (triangle[2] as usize, triangle[0] as usize),
        ] {
            *directed
                .entry((cell(positions[pair.0]), cell(positions[pair.1])))
                .or_insert(0) += 1;
        }
    }
    let mut owner: std::collections::BTreeMap<((i64, i64, i64), (i64, i64, i64)), Vec<u64>> =
        std::collections::BTreeMap::new();
    let mut at = 0usize;
    for (face_id, triangle_count) in counts {
        for index in at..(at + triangle_count).min(indices.len()) {
            let triangle = indices[index];
            for pair in [
                (triangle[0] as usize, triangle[1] as usize),
                (triangle[1] as usize, triangle[2] as usize),
                (triangle[2] as usize, triangle[0] as usize),
            ] {
                let key = (cell(positions[pair.0]), cell(positions[pair.1]));
                if directed.get(&key).copied().unwrap_or(0) > 1 {
                    owner.entry(key).or_default().push(*face_id);
                }
            }
        }
        at += triangle_count;
    }
    let (mut folded, mut crossed) = (0usize, 0usize);
    // **折り返しを持っている面の番号も控えます**（4-584）。**折り返しは
    // 「同じ面が、同じ辺を同じ向きに 2 回使う」**——**裏返しでは直せない**
    // （4-582）ので、**どの面なのかを名指しできないと次へ進めません。**
    let mut folded_faces: std::collections::BTreeMap<u64, usize> = Default::default();
    for users in owner.values() {
        let mut unique = users.clone();
        unique.sort_unstable();
        unique.dedup();
        if unique.len() <= 1 {
            folded += 1;
            if let Some(id) = unique.first() {
                *folded_faces.entry(*id).or_insert(0) += 1;
            }
        } else {
            crossed += 1;
        }
    }
    if folded > 0 && std::env::var_os("ZENITH_FOLDED_FACES").is_some() {
        println!("      折り返しを持つ面: {folded_faces:?}");
    }
    (folded, crossed)
}

/// **溶接前のパッチメッシュで、同じ面の折り返しを uv で見ます**（4-587）。
///
/// **溶接後の `uv` は共有頂点では隣の面のものが入っています**（4-586）。
/// **`face_patch_mesh` は本番と同じ刻みで、溶接せずに 1 面だけ返す**ので、
/// **`uv` は必ずその面のもの**です。
fn folds_in_uv_before_weld(
    solid: &zenith_topo::Solid,
    face_id: u64,
    params: &TessellationParams,
) {
    let Some(patch) = zenith_tess::face_patch_mesh(solid, face_id, params) else {
        println!("      面 {face_id}: パッチメッシュが取れません");
        return;
    };
    let mut seen: std::collections::HashMap<(u32, u32), usize> = Default::default();
    for triangle in &patch.indices {
        for corner in 0..3 {
            *seen
                .entry((triangle[corner], triangle[(corner + 1) % 3]))
                .or_insert(0) += 1;
        }
    }
    let mut folds: Vec<(u32, u32)> = seen
        .iter()
        .filter(|(_, count)| **count > 1)
        .map(|(edge, _)| *edge)
        .collect();
    folds.sort_unstable();
    println!(
        "      面 {face_id}（**溶接前**、三角形 {}）の折り返し {} 本:",
        patch.indices.len(),
        folds.len()
    );
    let mut on_line: std::collections::BTreeSet<i64> = Default::default();
    for (a, b) in &folds {
        let (ua, ub) = (patch.uvs[*a as usize], patch.uvs[*b as usize]);
        let (pa, pb) = (patch.positions[*a as usize], patch.positions[*b as usize]);
        println!(
            "        uv ({:.6}, {:.6}) → ({:.6}, {:.6})   長さ {:.3e}",
            ua.x,
            ua.y,
            ub.x,
            ub.y,
            (pb - pa).norm()
        );
        for uv in [ua, ub] {
            if (uv.y - 0.5).abs() < 1e-9 {
                on_line.insert((uv.x * 1e6).round() as i64);
            }
        }
    }
    // **`v = 0.5` の上に並んだ u を、順に並べて隣との差も出します**（4-587）。
    // **ほとんど同じ u が何本も並んでいれば、境界がそこで折り返している
    // （細い切れ込み）**ということです。
    if !on_line.is_empty() {
        let values: Vec<f64> = on_line.iter().map(|u| *u as f64 * 1e-6).collect();
        println!("        v = 0.5 の上の u（{} 個）と隣との差:", values.len());
        for pair in values.windows(2) {
            println!("          {:.6}  （次との差 {:.6}）", pair[0], pair[1] - pair[0]);
        }
        if let Some(last) = values.last() {
            println!("          {last:.6}");
        }
    }
}

/// **折り返した辺を、その面の uv で書き出します**（4-586）。
///
/// **12 と 24 分割でだけ起きて、8・16・20・32・48 では起きない**（4-584）
/// ので、**uv の刻み方と絡んでいるはず**です。**どの u・どの v に来るか**
/// を見ます。**節（knot）も並べて書きます**——**節の上に来ていれば、
/// 刻みが節と噛み合っていない話**になります。
fn folds_in_uv(solid: &zenith_topo::Solid, mesh: &zenith_tess::TriangleMesh, counts: &[(u64, usize)]) {
    let mut at = 0usize;
    for (face_id, triangle_count) in counts {
        let end = (at + triangle_count).min(mesh.indices.len());
        // **この面の中だけで、同じ向きに 2 回使われた有向辺**を探します。
        let mut seen: std::collections::HashMap<(u32, u32), usize> = Default::default();
        for index in at..end {
            let triangle = mesh.indices[index];
            for corner in 0..3 {
                *seen
                    .entry((triangle[corner], triangle[(corner + 1) % 3]))
                    .or_insert(0) += 1;
            }
        }
        let mut folds: Vec<(u32, u32)> = seen
            .iter()
            .filter(|(_, count)| **count > 1)
            .map(|(edge, _)| *edge)
            .collect();
        if folds.is_empty() {
            at = end;
            continue;
        }
        folds.sort_unstable();
        println!("      面 {face_id} の折り返し（同じ面が同じ向きに 2 回使った辺）{} 本:", folds.len());
        if let Some(face) = solid.outer_shell.faces.iter().find(|f| f.id == *face_id) {
            if let zenith_topo::FaceGeometry::Nurbs(surface) = &face.geometry {
                println!(
                    "        u の節: {:?}",
                    surface.knots_u.knots.iter().map(|k| (k * 1e4).round() / 1e4).collect::<Vec<_>>()
                );
                println!(
                    "        v の節: {:?}",
                    surface.knots_v.knots.iter().map(|k| (k * 1e4).round() / 1e4).collect::<Vec<_>>()
                );
            }
        }
        for (a, b) in folds {
            let (ua, ub) = (mesh.uvs[a as usize], mesh.uvs[b as usize]);
            println!(
                "        uv ({:.6}, {:.6}) → ({:.6}, {:.6})",
                ua.x, ua.y, ub.x, ub.y
            );
        }
        at = end;
    }
}

/// **折り返しが、面の境界の上にあるのか内側にあるのかを測ります**（4-585）。
///
/// **これが次の分かれ道**です（4-584）——**境界の上なら耳の橋渡しの話、
/// 内側なら三角形分割そのものが自分と重なっている話**です。
///
/// **面の外輪を細かく折れ線にして、折り返した辺の中点からの距離**を見ます。
fn where_are_folds(solid: &zenith_topo::Solid, folds: &[(u64, zenith_math::Point3, zenith_math::Point3)]) {
    for (id, p0, p1) in folds {
        let Some(face) = solid.outer_shell.faces.iter().find(|f| f.id == *id) else {
            continue;
        };
        // **外輪を折れ線にします**（稜ごとに 64 点）。
        let mut wire: Vec<zenith_math::Point3> = Vec::new();
        for oriented in &face.outer_wire.edges {
            let curve = &oriented.edge.curve;
            let (lo, hi) = curve.param_range();
            for step in 0..=64 {
                let t = lo + (hi - lo) * (step as f64) / 64.0;
                wire.push(curve.evaluate(t));
            }
        }
        let middle = zenith_math::Point3::from((p0.coords + p1.coords) * 0.5);
        let nearest = wire
            .iter()
            .map(|point| (point - middle).norm())
            .fold(f64::INFINITY, f64::min);
        // **「近い」は、面の大きさに対して言わないと意味がありません**
        // （4-585。**最初は辺の長さと比べて書いてしまい、長い辺が
        // どれも「境界の上」に見えていました**——**直しました**）。
        let mut low = wire[0];
        let mut high = wire[0];
        for point in &wire {
            low = zenith_math::Point3::new(low.x.min(point.x), low.y.min(point.y), low.z.min(point.z));
            high =
                zenith_math::Point3::new(high.x.max(point.x), high.y.max(point.y), high.z.max(point.z));
        }
        let span = (high - low).norm();
        let ratio = if span > 0.0 { nearest / span } else { 0.0 };
        let length = (p1 - p0).norm();
        println!(
            "        面 {id} 折り返した辺（長さ {length:.3e}）の中点から外輪まで {nearest:.3e}（面の差し渡し {span:.3e} の {:.1}%）  → {}",
            ratio * 100.0,
            if ratio <= 0.01 {
                "**境界の上**"
            } else {
                "**面の内側**"
            }
        );
    }
}

/// **折り返した辺そのもの（面の番号と両端）を返します**（4-585）。
fn folded_edges(
    positions: &[zenith_math::Point3],
    indices: &[[u32; 3]],
    counts: &[(u64, usize)],
) -> Vec<(u64, zenith_math::Point3, zenith_math::Point3)> {
    let cell = |point: zenith_math::Point3| {
        (
            (point.x / 1e-6).round() as i64,
            (point.y / 1e-6).round() as i64,
            (point.z / 1e-6).round() as i64,
        )
    };
    let mut directed: std::collections::BTreeMap<((i64, i64, i64), (i64, i64, i64)), usize> =
        std::collections::BTreeMap::new();
    for triangle in indices {
        for pair in [
            (triangle[0] as usize, triangle[1] as usize),
            (triangle[1] as usize, triangle[2] as usize),
            (triangle[2] as usize, triangle[0] as usize),
        ] {
            *directed
                .entry((cell(positions[pair.0]), cell(positions[pair.1])))
                .or_insert(0) += 1;
        }
    }
    let mut owner: std::collections::BTreeMap<((i64, i64, i64), (i64, i64, i64)), Vec<u64>> =
        std::collections::BTreeMap::new();
    let mut at = 0usize;
    for (face_id, triangle_count) in counts {
        for index in at..(at + triangle_count).min(indices.len()) {
            let triangle = indices[index];
            for pair in [
                (triangle[0] as usize, triangle[1] as usize),
                (triangle[1] as usize, triangle[2] as usize),
                (triangle[2] as usize, triangle[0] as usize),
            ] {
                let key = (cell(positions[pair.0]), cell(positions[pair.1]));
                if directed.get(&key).copied().unwrap_or(0) > 1 {
                    owner.entry(key).or_default().push(*face_id);
                }
            }
        }
        at += triangle_count;
    }
    let mut out = Vec::new();
    for (key, users) in &owner {
        let mut unique = users.clone();
        unique.sort_unstable();
        unique.dedup();
        if unique.len() <= 1 {
            if let Some(id) = unique.first() {
                out.push((
                    *id,
                    zenith_math::Point3::new(
                        key.0 .0 as f64 * 1e-6,
                        key.0 .1 as f64 * 1e-6,
                        key.0 .2 as f64 * 1e-6,
                    ),
                    zenith_math::Point3::new(
                        key.1 .0 as f64 * 1e-6,
                        key.1 .1 as f64 * 1e-6,
                        key.1 .2 as f64 * 1e-6,
                    ),
                ));
            }
        }
    }
    out
}

/// **折り返しを持っている面の番号を返します**（4-584）。
fn folded_face_ids(
    positions: &[zenith_math::Point3],
    indices: &[[u32; 3]],
    counts: &[(u64, usize)],
) -> Vec<u64> {
    let cell = |point: zenith_math::Point3| {
        (
            (point.x / 1e-6).round() as i64,
            (point.y / 1e-6).round() as i64,
            (point.z / 1e-6).round() as i64,
        )
    };
    let mut directed: std::collections::BTreeMap<((i64, i64, i64), (i64, i64, i64)), usize> =
        std::collections::BTreeMap::new();
    for triangle in indices {
        for pair in [
            (triangle[0] as usize, triangle[1] as usize),
            (triangle[1] as usize, triangle[2] as usize),
            (triangle[2] as usize, triangle[0] as usize),
        ] {
            *directed
                .entry((cell(positions[pair.0]), cell(positions[pair.1])))
                .or_insert(0) += 1;
        }
    }
    let mut owner: std::collections::BTreeMap<((i64, i64, i64), (i64, i64, i64)), Vec<u64>> =
        std::collections::BTreeMap::new();
    let mut at = 0usize;
    for (face_id, triangle_count) in counts {
        for index in at..(at + triangle_count).min(indices.len()) {
            let triangle = indices[index];
            for pair in [
                (triangle[0] as usize, triangle[1] as usize),
                (triangle[1] as usize, triangle[2] as usize),
                (triangle[2] as usize, triangle[0] as usize),
            ] {
                let key = (cell(positions[pair.0]), cell(positions[pair.1]));
                if directed.get(&key).copied().unwrap_or(0) > 1 {
                    owner.entry(key).or_default().push(*face_id);
                }
            }
        }
        at += triangle_count;
    }
    let mut ids: Vec<u64> = Vec::new();
    for users in owner.values() {
        let mut unique = users.clone();
        unique.sort_unstable();
        unique.dedup();
        if unique.len() <= 1 {
            if let Some(id) = unique.first() {
                ids.push(*id);
            }
        }
    }
    ids.sort_unstable();
    ids.dedup();
    ids
}

/// **折り返しを持っていた面の姿を書き出します**（4-584）。
fn describe_faces(solid: &zenith_topo::Solid, ids: &[u64]) {
    for id in ids {
        let Some(face) = solid.outer_shell.faces.iter().find(|f| f.id == *id) else {
            println!("        面 {id}: 外殻に見つかりません");
            continue;
        };
        let shape = match &face.geometry {
            zenith_topo::FaceGeometry::Nurbs(surface) => format!(
                "NURBS 次数 {}×{}、制御点 {}×{}",
                surface.degree_u,
                surface.degree_v,
                surface.control_points.len(),
                surface.control_points.first().map_or(0, |row| row.len())
            ),
            zenith_topo::FaceGeometry::Plane(_) => "平面".to_string(),
            zenith_topo::FaceGeometry::Coons(_) => "Coons".to_string(),
            zenith_topo::FaceGeometry::Gordon(_) => "Gordon".to_string(),
            zenith_topo::FaceGeometry::Triangular(_) => "三角パッチ".to_string(),
        };
        println!(
            "        面 {id}: {shape}、向き {:?}、外輪の稜 {}、内輪 {}（稜 {}）、申告する粗さ {:.3e}、p-curve {}",
            face.orientation,
            face.outer_wire.edges.len(),
            face.inner_wires.len(),
            face.inner_wires
                .iter()
                .map(|wire| wire.edges.len())
                .sum::<usize>(),
            face.tolerance,
            face.pcurves.is_some()
        );
    }
}

fn count(
    name: &str,
    solid: &zenith_topo::Solid,
) -> (std::collections::BTreeSet<u64>, CrossedExample) {
    let params = TessellationParams::default();
    let mesh = tessellate_solid(solid, &params);
    let counts = face_triangle_counts(solid, &params);
    let cell = |point: zenith_math::Point3| {
        (
            (point.x / 1e-6).round() as i64,
            (point.y / 1e-6).round() as i64,
            (point.z / 1e-6).round() as i64,
        )
    };
    let mut directed: std::collections::BTreeMap<((i64, i64, i64), (i64, i64, i64)), usize> =
        std::collections::BTreeMap::new();
    for triangle in &mesh.indices {
        for pair in [
            (triangle[0] as usize, triangle[1] as usize),
            (triangle[1] as usize, triangle[2] as usize),
            (triangle[2] as usize, triangle[0] as usize),
        ] {
            *directed
                .entry((cell(mesh.positions[pair.0]), cell(mesh.positions[pair.1])))
                .or_insert(0) += 1;
        }
    }
    let mut owner: std::collections::BTreeMap<((i64, i64, i64), (i64, i64, i64)), Vec<u64>> =
        std::collections::BTreeMap::new();
    // **面ごとの割り当ては、合計が合うときだけ当てになります**（4-569）。
    // **実測（球）: `face_triangle_counts` の合計が、三角形の数を超えます**
    // ——**そこで落ちました**。**守りを入れ、合うかどうかを出します。**
    let total: usize = counts.iter().map(|(_, count)| *count).sum();
    let attributable = total == mesh.indices.len();
    let mut at = 0usize;
    for (face_id, triangle_count) in &counts {
        for index in at..(at + triangle_count).min(mesh.indices.len()) {
            let triangle = mesh.indices[index];
            for pair in [
                (triangle[0] as usize, triangle[1] as usize),
                (triangle[1] as usize, triangle[2] as usize),
                (triangle[2] as usize, triangle[0] as usize),
            ] {
                let key = (cell(mesh.positions[pair.0]), cell(mesh.positions[pair.1]));
                if directed.get(&key).copied().unwrap_or(0) > 1 {
                    owner.entry(key).or_default().push(*face_id);
                }
            }
        }
        at += triangle_count;
    }
    let (mut folded, mut crossed) = (0usize, 0usize);
    let mut crossed_face_ids: std::collections::BTreeSet<u64> = std::collections::BTreeSet::new();
    // **組ごとに、実例の座標を 1 つ持っておきます**（4-573）。**面 id は
    // 走行ごとに変わり得る**ので、番号を決め打ちせず、この組から
    // その場で座標を引けるようにします——**次に見る人が、同じ手で
    // 別の組を確かめられる**ようにするためです。
    let mut crossed_examples: CrossedExample = std::collections::BTreeMap::new();
    for (key, users) in &owner {
        let mut unique = users.clone();
        unique.sort_unstable();
        unique.dedup();
        if unique.len() <= 1 {
            folded += 1;
        } else {
            crossed += 1;
            crossed_face_ids.extend(&unique);
            let point0 = zenith_math::Point3::new(
                key.0 .0 as f64 * 1e-6,
                key.0 .1 as f64 * 1e-6,
                key.0 .2 as f64 * 1e-6,
            );
            let point1 = zenith_math::Point3::new(
                key.1 .0 as f64 * 1e-6,
                key.1 .1 as f64 * 1e-6,
                key.1 .2 as f64 * 1e-6,
            );
            for i in 0..unique.len() {
                for j in (i + 1)..unique.len() {
                    crossed_examples
                        .entry((unique[i], unique[j]))
                        .or_insert((point0, point1));
                }
            }
        }
    }
    if name.contains("linkrods") && !name.contains("正規化") {
        for (pair, points) in &crossed_examples {
            println!("    組 {:?} の実例座標: {:?}", pair, points);
        }
        println!(
            "    面の組（{} 組）: {:?}",
            crossed_examples.len(),
            crossed_examples.keys().collect::<Vec<_>>()
        );
    }
    println!(
        "  {name:<28} 面 {:<3} 三角形 {:<6} 折り返し {folded:<4} 別の 2 枚が同じ向き {crossed}{}",
        solid.outer_shell.faces.len(),
        mesh.indices.len(),
        if attributable {
            ""
        } else {
            "  **面への割り当ては当てになりません**（合計が合いません）"
        }
    );
    (crossed_face_ids, crossed_examples)
}

fn main() {
    // **自作の立体でも起きるか**（4-569）。**起きなければ、読んだ面に固有**です。
    println!("自作の立体:");
    if let Ok(solid) = zenith_algo::PrimitiveBuilder::make_box(20.0, 10.0, 6.0) {
        count("箱", &solid);
    }
    if let Ok(solid) = zenith_algo::PrimitiveBuilder::make_cylinder(5.0, 12.0) {
        count("円柱", &solid);
    }
    if let Ok(solid) = zenith_algo::PrimitiveBuilder::make_sphere(6.0) {
        count("球", &solid);
    }
    if let Ok(solid) = zenith_algo::PrimitiveBuilder::make_torus(10.0, 3.0) {
        count("トーラス", &solid);
    }
    // **ブーリアンの答えも見ます**（4-583）。**4-568 は「和と差の答えに
    // 同じ向きの稜が 55 本（折り返し 10・別の 2 枚 45）」と測りました**
    // ——**素形状は継ぎ目が素直でも、割った面はそうではありません。**
    // **溶接のあとで直す口（4-582）が、ここにも効くかを見ます。**
    println!();
    println!("ブーリアンの答え（4-583）:");
    let tol = zenith_math::Tolerance::default();
    let torus = zenith_algo::PrimitiveBuilder::make_torus(12.0, 4.0);
    let cylinder = zenith_algo::PrimitiveBuilder::make_cylinder(9.0, 40.0);
    if let (Ok(torus), Ok(cylinder)) = (&torus, &cylinder) {
        let rod = zenith_algo::BrepTransform::translate_solid(
            cylinder,
            zenith_math::Vec3::new(0.0, 0.0, -20.0),
        );
        for (label, op) in [
            ("トーラス − 棒", zenith_algo::BooleanOpType::Difference),
            ("トーラス ＋ 棒", zenith_algo::BooleanOpType::Union),
        ] {
            match zenith_algo::BooleanEngine::boolean_solids_exact_result(torus, &rod, op, &tol) {
                Ok(result) => {
                    for solid in &result.solids {
                        count(label, solid);
                    }
                }
                Err(reason) => println!("  {label}: **断られました**（{reason}）"),
            }
        }
    }
    println!();

    let path = "reference/OCCT/data/step/linkrods.step";
    let solids = match StepImporter::import_solids_from_file(path) {
        Ok(solids) => solids,
        Err(reason) => {
            println!("**読めません**: {reason}");
            println!("（`reference/OCCT/data/step/linkrods.step` に置いてください）");
            return;
        }
    };
    let Some(solid) = solids.into_iter().max_by_key(|solid| solid.outer_shell.faces.len()) else {
        println!("**立体が 0 個**");
        return;
    };
    // **読んだ立体も、同じ守り付きの関数に通します**（4-569）。
    // **向きを見ない数え方（穴・重なり）も、ここで出します。**
    let params = TessellationParams::default();
    let mesh = tessellate_solid(&solid, &params);
    let cell = |point: zenith_math::Point3| {
        (
            (point.x / 1e-6).round() as i64,
            (point.y / 1e-6).round() as i64,
            (point.z / 1e-6).round() as i64,
        )
    };
    let mut undirected: std::collections::BTreeMap<((i64, i64, i64), (i64, i64, i64)), usize> =
        std::collections::BTreeMap::new();
    for triangle in &mesh.indices {
        for pair in [
            (triangle[0] as usize, triangle[1] as usize),
            (triangle[1] as usize, triangle[2] as usize),
            (triangle[2] as usize, triangle[0] as usize),
        ] {
            let (a, b) = (cell(mesh.positions[pair.0]), cell(mesh.positions[pair.1]));
            let (lo, hi) = if a <= b { (a, b) } else { (b, a) };
            *undirected.entry((lo, hi)).or_insert(0) += 1;
        }
    }
    println!("読んだ立体（linkrods.step）:");
    println!(
        "  メッシュの穴 {} 本、重なり {} 本（**向きを見ない数え方**）",
        undirected.values().filter(|count| **count == 1).count(),
        undirected.values().filter(|count| **count > 2).count()
    );
    let (crossed_ids, crossed_examples) = count("linkrods", &solid);

    // **巻き付いた面（全周を1つで持つ NURBS 面）の id を集めます**（4-573）。
    let tol0 = zenith_math::Tolerance::default();
    let wrapped_ids: std::collections::BTreeSet<u64> = solid
        .outer_shell
        .faces
        .iter()
        .filter(|face| match &face.geometry {
            zenith_topo::FaceGeometry::Nurbs(surface) => {
                zenith_algo::Regularizer::grid_closes_in_u(surface, &tol0)
                    || zenith_algo::Regularizer::grid_closes_in_v(surface, &tol0)
            }
            _ => false,
        })
        .map(|face| face.id)
        .collect();
    println!(
        "  巻き付いた面 {} 枚: {:?}",
        wrapped_ids.len(),
        wrapped_ids
    );
    println!(
        "  巻き方が食い違う面 {} 枚: {:?}",
        crossed_ids.len(),
        crossed_ids
    );
    let overlap: std::collections::BTreeSet<u64> =
        wrapped_ids.intersection(&crossed_ids).copied().collect();
    println!(
        "  **重なり {} 枚**（巻き付いていて、かつ食い違う）: {:?}",
        overlap.len(),
        overlap
    );
    println!("  食い違う面の中身:");
    for face in &solid.outer_shell.faces {
        if !crossed_ids.contains(&face.id) {
            continue;
        }
        let kind = match &face.geometry {
            zenith_topo::FaceGeometry::Plane(_) => "平面".to_string(),
            zenith_topo::FaceGeometry::Nurbs(s) => {
                format!("NURBS 次数{}x{} 制御点{}x{}", s.degree_u, s.degree_v, s.control_points.len(), s.control_points[0].len())
            }
            _ => "その他".to_string(),
        };
        println!(
            "    id {}: {kind}  向き {:?}  稜 {} 内輪 {}",
            face.id,
            face.orientation,
            face.outer_wire.edges.len(),
            face.inner_wires.len()
        );
        // **退化した行（同じ点が並ぶ）が無いか**——円錐の頂点・球の極では
        // 制御点の1行が1点に潰れ、そこで法線が定義できません（4-573）。
        if let zenith_topo::FaceGeometry::Nurbs(s) = &face.geometry {
            for (row_index, row) in s.control_points.iter().enumerate() {
                let first = row[0].point;
                if row.iter().all(|cp| (cp.point - first).norm() <= 1e-9) {
                    println!("      行 {row_index} が 1 点に潰れています（退化）: {first:?}");
                }
            }
        }
    }

    // **正規化（巻き付いた面を割る）を通すと減るか**（4-573）。
    //
    // `Regularizer::split_wrapped_face` は、全周を1つで持つ面（円柱・
    // トーラスなど、制御格子の最初と最後の行／列が重なる面）を、
    // ノットの位置で割ります。**ブーリアンの入口
    // （`hold_like_our_own`）だけがこれを通し**、この掃き出しの素の
    // 読み込みは通していません。**巻き付いた面の周りで巻き方が
    // 崩れているなら、通すと 43 本が減るはず**です。
    let tol = zenith_math::Tolerance::default();
    let (regularized, report) = zenith_algo::Regularizer::regularize_solid(&solid, &tol);
    println!(
        "  正規化: 割った巻き付き面 {} 枚、割らずに残した巻き付き面 {} 枚",
        report.wrapped_faces_split, report.wrapped_faces_left_alone
    );
    for reason in &report.left_alone_reasons {
        println!("    残した理由: {reason}");
    }
    count("linkrods（正規化後）", &regularized);
    println!();

    // **`surface.normal() × orientation` が、本当に外を向いているか**
    // （4-573）。**食い違う 7 枚（球パッチ 3・自由曲面 4）と、対照に
    // 円柱面 1 枚を、答えの材料の外へ出るかで直に確かめます**——
    // **4-567 で蓋の向きを決めたのと同じやり方**です。
    println!("法線が、本当に外を向いているか（4-573）:");
    let mesh = tessellate_solid(&solid, &params);
    let step = {
        let bbox = solid.bounding_box();
        (bbox.max - bbox.min).norm() * 1e-4
    };
    let check_ids: Vec<u64> = crossed_ids
        .iter()
        .copied()
        .chain(std::iter::once(147)) // 面36。対照用（申告のある円柱っぽい平面ではなく、
        // 巻いていない普通の面）。
        .collect();
    for face in &solid.outer_shell.faces {
        if !check_ids.contains(&face.id) {
            continue;
        }
        let zenith_topo::FaceGeometry::Nurbs(surface) = &face.geometry else {
            continue;
        };
        let ((u0, u1), (v0, v1)) = surface.param_range();
        let (um, vm) = ((u0 + u1) * 0.5, (v0 + v1) * 0.5);
        let Some(mut normal) = surface.normal(um, vm) else {
            println!("    id {}: 中央で法線が取れません", face.id);
            continue;
        };
        if !face.orientation.is_forward() {
            normal = -normal;
        }
        let point = surface.evaluate(um, vm);
        let probe = point + normal * step;
        let inside = zenith_algo::BooleanEngine::is_point_inside_mesh(probe, &mesh);
        println!(
            "    id {}: 中央から外向きへ踏み出した点は{}",
            face.id,
            if inside { "**中**（向きが逆！）" } else { "外（正しい）" }
        );
    }
    println!();

    // **境界そのものの点で、両方の面の法線を突き合わせます**（4-573）。
    // **中央では両方とも「正しい」でした**——**問題は境界の近くにあるかも
    // しれません。** `crossed_examples` は、食い違う組ごとに実際の
    // 境界の座標を持っています——**id を決め打ちしません**。
    //
    // 実測（2026/09/28。この検体）: **面 78 と 84 の境界で、両方の法線は
    // 桁まで一致**（(-0.5565 0.0263 -0.8304)）。**法線は揃っており、
    // どちらも正しく外を向きます**——**4-573 の結論（`build_trimmed_mesh`
    // の巻き方補正が、接する面どうしで境界稜の向きを取り違える）は、
    // ここから来ています。**
    println!("境界の点で、両方の面の向きが揃っているか（4-573）:");
    let tol = zenith_math::Tolerance::default();
    for (&(id_a, id_b), &(boundary_point, _)) in &crossed_examples {
        for id in [id_a, id_b] {
            let Some(face) = solid.outer_shell.faces.iter().find(|f| f.id == id) else {
                continue;
            };
            let zenith_topo::FaceGeometry::Nurbs(surface) = &face.geometry else {
                continue;
            };
            match zenith_geom::ExtremumEngine::point_to_surface(
                boundary_point,
                surface,
                64,
                tol.parametric,
            ) {
                Ok(projection) => {
                    let Some(mut normal) = surface.normal(projection.u, projection.v) else {
                        println!("    組 ({id_a},{id_b}) の id {id}: 境界で法線が取れません");
                        continue;
                    };
                    if !face.orientation.is_forward() {
                        normal = -normal;
                    }
                    let probe = boundary_point + normal * step;
                    let inside = zenith_algo::BooleanEngine::is_point_inside_mesh(probe, &mesh);
                    println!(
                        "    組 ({id_a},{id_b}) の id {id}: 距離 {:.3e}  法線 ({:.4} {:.4} {:.4})  踏み出した点は{}",
                        projection.distance,
                        normal.x, normal.y, normal.z,
                        if inside { "**中**（向きが逆！）" } else { "外（正しい）" }
                    );
                }
                Err(reason) => println!("    組 ({id_a},{id_b}) の id {id}: 射影できません: {reason}"),
            }
        }
    }

    // **辺の両端で絞り込んで、両面それぞれの三角形を名指しします**（4-573）。
    // **`tessellate_solid`（縫い合わせる版。実際に使われるほう）の頂点で
    // 探します**——**`tessellate_face` とは別の頂点位置を持つ**ので、
    // 単独では刻み直しません。
    println!("辺の両端で名指しした三角形（4-573）:");
    let mut stitched_mesh = tessellate_solid(&solid, &params);
    let stitched_counts = face_triangle_counts(&solid, &params);
    // **捨てずに、曖昧な方だけ巻き方を裏返す（4-574 の続き）**。
    // **捨てる手は穴を増やしました**（4-574）。**ここでは、組ごとに
    // 「揃い具合が桁違いに小さい方」だけを見つけ、その 1 枚の頂点順を
    // 裏返して**（三角形自体は残したまま）、食い違いが本当に消えるかを
    // その場でシミュレートします。**まだ `stitched.rs` は直していません**
    // ——ここは診断だけです。
    let mut flip_candidates: Vec<usize> = Vec::new();
    for (&(id_a, id_b), &(end0, end1)) in &crossed_examples {
        for id in [id_a, id_b] {
            let mut at = 0usize;
            let mut found = false;
            for (face_id, triangle_count) in &stitched_counts {
                if *face_id != id {
                    at += triangle_count;
                    continue;
                }
                for index in at..(at + triangle_count).min(stitched_mesh.indices.len()) {
                    let tri = stitched_mesh.indices[index];
                    let pts = [
                        stitched_mesh.positions[tri[0] as usize],
                        stitched_mesh.positions[tri[1] as usize],
                        stitched_mesh.positions[tri[2] as usize],
                    ];
                    let has_end0 = pts.iter().any(|p| (p - end0).norm() <= 1e-6);
                    let has_end1 = pts.iter().any(|p| (p - end1).norm() <= 1e-6);
                    if has_end0 && has_end1 {
                        let facet = (pts[1] - pts[0]).cross(&(pts[2] - pts[0]));
                        // **この面の、この三角形の重心での「正しい」外向き
                        // 法線**——`surface.normal()` に `orientation` を
                        // 掛けたもの。facet との内積が正なら、この三角形は
                        // 正しく外向きです。
                        let verdict = 'verdict: {
                            let Some(face) = solid.outer_shell.faces.iter().find(|f| f.id == id)
                            else {
                                break 'verdict "面が見つかりません".to_string();
                            };
                            let zenith_topo::FaceGeometry::Nurbs(surface) = &face.geometry else {
                                break 'verdict "NURBS ではありません".to_string();
                            };
                            let centroid = zenith_math::Point3::from(
                                (pts[0].coords + pts[1].coords + pts[2].coords) / 3.0,
                            );
                            match zenith_geom::ExtremumEngine::point_to_surface(
                                centroid, surface, 64, tol.parametric,
                            ) {
                                Ok(projection) => {
                                    let Some(mut normal) =
                                        surface.normal(projection.u, projection.v)
                                    else {
                                        break 'verdict "法線が取れません".to_string();
                                    };
                                    if !face.orientation.is_forward() {
                                        normal = -normal;
                                    }
                                    let dot = facet.dot(&normal);
                                    let normal_len = normal.norm();
                                    let ratio = if normal_len > 1e-18 && facet.norm() > 1e-18 {
                                        (dot / (facet.norm() * normal_len)).abs()
                                    } else {
                                        1.0
                                    };
                                    if ratio < 1e-6 {
                                        flip_candidates.push(index);
                                    }
                                    format!(
                                        "facet長さ={:.3e}  facet・正しい法線 = {dot:+.3e}（{}、揃い具合 {ratio:.3e}）",
                                        facet.norm(),
                                        if dot >= 0.0 { "正しく外向き" } else { "**内向き（誤り）**" }
                                    )
                                }
                                Err(reason) => format!("射影できません: {reason}"),
                            }
                        };
                        println!(
                            "    組 ({id_a},{id_b}) の id {id}: ({:.6} {:.6} {:.6})-({:.6} {:.6} {:.6})-({:.6} {:.6} {:.6})  {verdict}",
                            pts[0].x, pts[0].y, pts[0].z,
                            pts[1].x, pts[1].y, pts[1].z,
                            pts[2].x, pts[2].y, pts[2].z
                        );
                        found = true;
                    }
                }
                break;
            }
            if !found {
                println!("    組 ({id_a},{id_b}) の id {id}: 両端を持つ三角形が見つかりません");
            }
        }
    }
    println!(
        "揃い具合が桁違いに小さい三角形（裏返す候補）: {} 枚 {:?}",
        flip_candidates.len(),
        flip_candidates
    );
    let before_total = stitched_mesh.indices.len();
    for &index in &flip_candidates {
        stitched_mesh.indices[index] = [
            stitched_mesh.indices[index][0],
            stitched_mesh.indices[index][2],
            stitched_mesh.indices[index][1],
        ];
    }
    let (flipped_folded, flipped_crossed) =
        count_mismatches(&stitched_mesh.positions, &stitched_mesh.indices, &stitched_counts);
    println!(
        "裏返した後（捨てていない。三角形数 {}→{}）: 折り返し {}  別の 2 枚が同じ向き {}",
        before_total,
        stitched_mesh.indices.len(),
        flipped_folded,
        flipped_crossed
    );

    // **読んだファイル 2 つを、刻みを振って数えます**（4-581）。
    // **`screw.step` は `uv-zero` の受け皿が在る理由そのもの**なので、
    // **巻き方の直しを入れたら、必ずここも見てください。**
    println!();
    println!("読んだファイルを、刻みを振って（4-581）:");
    sweep_foreign("screw", "reference/OCCT/data/step/screw.step");
    sweep_foreign("linkrods", "reference/OCCT/data/step/linkrods.step");

    report_boundary_on_surface(&solid);
    println!();
    println!("**自作の立体は 0 本、読んだ立体だけが持ちます。**");
    println!("**ブーリアンの答えは 45 本**（4-568）——**足しているのは 2 本だけ**です。");
    println!();
    report_face_areas(&solid);
}

/// **A（読んだ立体そのもの）の面ごとの面積と体積への寄与**（4-572）。
///
/// 4-559 は「**積の 1.68e-4 のうち 1.234e-4 は入口に在る**——**ブーリアンを
/// 1 回もかけていない段階で、`V(A)` が OCC より小さい**」と測りました。
/// **どの面が、その差を持っているか**を見ます。**OCC 側は
/// `ZENITH_OCC_FACES=A`**（`tools/occ_h8_reference.py`）で、
/// **同じ並び（面積の大きい順）**にしてあります。
fn report_face_areas(solid: &zenith_topo::Solid) {
    let params = TessellationParams::default();
    println!("A の面ごとの面積（大きい順）:");
    let mut rows: Vec<(f64, f64, zenith_math::Point3, usize)> = Vec::new();
    for face in &solid.outer_shell.faces {
        let (area, volume) =
            zenith_algo::MassCalculator::compute_face_integral(face, &params);
        let mesh = zenith_tess::tessellate_face(face, &params);
        let mut centre = zenith_math::Vec3::zeros();
        for point in &mesh.positions {
            centre += point.coords;
        }
        if !mesh.positions.is_empty() {
            centre /= mesh.positions.len() as f64;
        }
        rows.push((
            area,
            volume,
            zenith_math::Point3::from(centre),
            face.outer_wire.edges.len(),
        ));
    }
    let total_area: f64 = rows.iter().map(|row| row.0).sum();
    let total_volume: f64 = rows.iter().map(|row| row.1).sum();
    rows.sort_by(|left, right| right.0.partial_cmp(&left.0).unwrap());
    for (area, volume, centre, edges) in &rows {
        println!(
            "  面積 {area:.6}  重心 ({:.4} {:.4} {:.4})  稜 {edges}  寄与 {volume:+.6}",
            centre.x, centre.y, centre.z
        );
    }
    println!("  面積の合計 {total_area:.6}、体積（寄与の合計） {total_volume:.6}");
}

/// **読んだ面の境界が、自分の曲面の上に在るか**（4-570）。
///
/// 4-562 は「**和の最後の 2 本は、3D の稜が曲面から 4.000839e-5 浮いている**」
/// **面36 が申告する粗さ 3.901828e-5 を 2.5% 超える**と測りました。
///
/// **同じ問いを、入口で立てます**——**ブーリアンの前から浮いているのか。**
/// **浮いているなら、原因は刻む段ではなく、読む段**です
/// （**OCC は円柱とトーラスのまま持ち、こちらは 13 枚を NURBS に直しています**。4-560）。
fn report_boundary_on_surface(solid: &zenith_topo::Solid) {
    let tol = zenith_math::Tolerance::default();
    println!("読んだ面の境界が、自分の曲面の上に在るか（**37 点で測ります**）:");
    let mut rows: Vec<(usize, u64, f64, f64, usize)> = Vec::new();
    for (at, face) in solid.outer_shell.faces.iter().enumerate() {
        let report = face.validate_boundary_on_surface(&tol, 37);
        let allowed = face.tolerance + face.pcurve_tolerance;
        rows.push((
            at,
            face.id,
            report.max_distance,
            allowed,
            report.off_surface_point_count,
        ));
    }
    rows.sort_by(|left, right| right.2.partial_cmp(&left.2).unwrap());
    for (at, id, distance, allowed, off) in rows.iter() {
        println!(
            "  面{at:<3}（id {id}）外れ {distance:.6e}  申告 {allowed:.6e}  比 {:.3}  外れた点 {off}",
            if *allowed > 0.0 { distance / allowed } else { f64::INFINITY }
        );
    }
    let worst = rows.first().map(|row| row.2).unwrap_or(0.0);
    let over = rows.iter().filter(|row| row.2 > row.3).count();
    println!("  いちばん外れた面 {worst:.6e}、**申告を超えている面 {over} 枚**");
}
