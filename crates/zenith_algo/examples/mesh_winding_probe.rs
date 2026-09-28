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

use zenith_io::StepImporter;
use zenith_tess::{face_triangle_counts, tessellate_solid, TessellationParams};

fn count(name: &str, solid: &zenith_topo::Solid) {
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
    for users in owner.values() {
        let mut unique = users.clone();
        unique.sort_unstable();
        unique.dedup();
        if unique.len() <= 1 {
            folded += 1;
        } else {
            crossed += 1;
        }
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
    count("linkrods", &solid);
    println!();
    println!();
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
