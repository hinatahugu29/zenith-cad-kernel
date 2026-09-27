//! **H8 の 1 演算を、OCC の数と並べます**（4-557、4-558 で 3 演算に）。
//!
//! 使い方: `h8_difference_probe [difference|union|intersection]`
//!
//! 4-555 で `linkrods` の差が返るようになりました。**体積は合っています**
//! が、**体積だけでは「合っている」とは言えません**——**面の数**も
//! **閉じているか**も見ます（**正しさは OCC の数で見ること**。4-511）。
//!
//! **`read_and_cut_probe` は 2 検体 × 3 演算で 13 分**かかります。
//! **ここは差 1 本だけ**なので、**4 分**で返ります。
use std::time::Instant;
use zenith_algo::{
    BooleanEngine, BooleanOpType, BrepTransform, MassCalculator, PrimitiveBuilder,
};
use zenith_io::StepImporter;
use zenith_math::{Point3, Tolerance, Vec3};
use zenith_tess::{tessellate_solid, TessellationParams};

fn main() {
    let sample = "reference/OCCT/data/step/linkrods.step";
    let Ok(solids) = StepImporter::import_solids_from_file(sample) else {
        println!("{sample} が読めません");
        return;
    };
    let Some(read) = solids.into_iter().max_by_key(|s| s.outer_shell.faces.len()) else {
        return;
    };
    let params = TessellationParams::default();
    let mesh = tessellate_solid(&read, &params);
    let (mut low, mut high) = (mesh.positions[0], mesh.positions[0]);
    for vertex in &mesh.positions {
        low = zenith_math::Point3::new(low.x.min(vertex.x), low.y.min(vertex.y), low.z.min(vertex.z));
        high =
            zenith_math::Point3::new(high.x.max(vertex.x), high.y.max(vertex.y), high.z.max(vertex.z));
    }
    let span = Vec3::new(high.x - low.x, high.y - low.y, high.z - low.z);
    let (inset, height) = (0.03, 0.47);
    let cutter = BrepTransform::translate_solid(
        &PrimitiveBuilder::make_box(
            span.x * (1.0 - inset * 2.0),
            span.y * (1.0 - inset * 2.0),
            span.z * height,
        )
        .expect("箱"),
        Vec3::new(
            low.x + span.x * inset,
            low.y + span.y * inset,
            low.z + span.z * (height * 0.5),
        ),
    );

    let which = std::env::args().nth(1).unwrap_or_else(|| "difference".to_string());
    // **OCC の数**（4-511）。**合わせる桁は 1e-4。**
    let (op, occ_volume, occ_faces) = match which.as_str() {
        "union" => (BooleanOpType::Union, 7.805669, 49usize),
        "intersection" => (BooleanOpType::Intersection, 2.295916, 37),
        _ => (BooleanOpType::Difference, 1.551124, 37),
    };
    let tol = Tolerance::default();
    if std::env::var_os("ZENITH_H8_OPERANDS").is_some() {
        println!(
            "|A| = {:.6}、|B| = {:.6}",
            MassCalculator::compute_volume_from_brep(&read, &params),
            MassCalculator::compute_volume_from_brep(&cutter, &params)
        );
    }
    let started = Instant::now();
    // **検証を通さずに中身を見る口**（4-559。`ZENITH_H8_UNVERIFIED=1`）。
    //
    // **断られた演算でも、体積は出ます。** **こちらの 3 つが互いに
    // 辻褄が合っているか**（|A∪B| + |A∩B| = |A| + |B|）は、
    // **OCC が無くても確かめられます**——**合っていなければ、
    // ずれているのはこちらの内側**です。
    let unverified = std::env::var_os("ZENITH_H8_UNVERIFIED").is_some();
    let result = if unverified {
        BooleanEngine::boolean_solids_exact_result_unverified(&read, &cutter, op, &tol)
    } else {
        BooleanEngine::boolean_solids_exact_result(&read, &cutter, op, &tol)
    };
    let seconds = started.elapsed().as_secs_f64();

    match result {
        Ok(result) => {
            let solids = result.solids;
            println!("{which}: 返りました（{seconds:.1} 秒）、立体 {} 個", solids.len());
            let (occ_volume, occ_faces) = (occ_volume, occ_faces);
            const BAND: f64 = 1e-4;
            for (index, solid) in solids.iter().enumerate() {
                let volume = MassCalculator::compute_volume_from_brep(solid, &params);
                let faces = solid.outer_shell.faces.len();
                let mesh = tessellate_solid(solid, &params);
                // **閉じているか**——1 回しか使われない稜があれば穴です。
                let mut uses: std::collections::BTreeMap<(i64, i64, i64, i64, i64, i64), usize> =
                    std::collections::BTreeMap::new();
                let grid = 1e-6f64;
                for triangle in &mesh.indices {
                    for pair in [
                        (triangle[0] as usize, triangle[1] as usize),
                        (triangle[1] as usize, triangle[2] as usize),
                        (triangle[2] as usize, triangle[0] as usize),
                    ] {
                        let cell = |point: zenith_math::Point3| {
                            (
                                (point.x / grid).round() as i64,
                                (point.y / grid).round() as i64,
                                (point.z / grid).round() as i64,
                            )
                        };
                        let (mut a, mut b) = (cell(mesh.positions[pair.0]), cell(mesh.positions[pair.1]));
                        if a > b {
                            std::mem::swap(&mut a, &mut b);
                        }
                        *uses.entry((a.0, a.1, a.2, b.0, b.1, b.2)).or_insert(0) += 1;
                    }
                }
                let holes = uses.values().filter(|count| **count == 1).count();
                let overlaps = uses.values().filter(|count| **count > 2).count();
                println!("  立体{index}: 体積 {volume:.6}、面 {faces} 枚、三角形 {}", mesh.indices.len());
                // **絶対と相対を、両方出します**（4-559）。
                //
                // **「合わせる桁は 1e-4」の単位が、どこにも書いてありません。**
                // 4-511 の `V(A)` の行は **3.2e-5 で一致**と書いていますが、
                // **それは相対**です（**絶対は 1.234e-4**）。**こちらの
                // `h8_difference_probe`（4-557）は絶対で比べていました。**
                // **積は、絶対なら外れ、相対なら内側**——**都合のよい
                // ほうを選ばないために、両方出します。**
                let absolute = (volume - occ_volume).abs();
                let relative = absolute / occ_volume;
                let verdict = |value: f64| {
                    if value <= BAND {
                        "内側"
                    } else {
                        "外"
                    }
                };
                println!(
                    "    OCC {occ_volume:.6} との差: 絶対 {absolute:.3e}（{}）／相対 {relative:.3e}（{}）　※桁 {BAND:.0e}",
                    verdict(absolute),
                    verdict(relative)
                );
                println!(
                    "    OCC の面 {occ_faces} 枚との差 {}、メッシュの穴 {holes} 本、重なり {overlaps} 本",
                    faces as i64 - occ_faces as i64
                );
                // **面ごとの体積への寄与**（4-566。`ZENITH_H8_FACE_VOLUMES=1`）。
                //
                // **和は 0.311（4%）大きい**のに、**面は 49 枚ちょうどで
                // メッシュも閉じています**（4-564）。**余っているのは、どの面か。**
                // **`compute_face_integral` は (面積, 体積) を返す**ので、
                // **寄与を 1 枚ずつ並べれば、0.311 を出している面が見えます。**
                if std::env::var_os("ZENITH_H8_FACE_VOLUMES").is_some() {
                    println!("    面ごとの寄与（大きい順）:");
                    // **重心も出します**（4-566）。**OCC 側は面積と重心で並びます**
                    // （`tools/occ_h8_reference.py` の `ZENITH_OCC_FACES`）ので、
                    // **同じ物差しでなければ 1 対 1 に並べられません**。
                    // **面積の大きい順**——**OCC 側と同じ並び**にします。
                    let mut rows: Vec<(usize, f64, f64, Point3, usize, usize)> = solid
                        .outer_shell
                        .faces
                        .iter()
                        .enumerate()
                        .map(|(at, face)| {
                            let (area, contribution) =
                                MassCalculator::compute_face_integral(face, &params);
                            let mesh = zenith_tess::tessellate_face(face, &params);
                            let mut centre = zenith_math::Vec3::zeros();
                            for point in &mesh.positions {
                                centre += point.coords;
                            }
                            if !mesh.positions.is_empty() {
                                centre /= mesh.positions.len() as f64;
                            }
                            (
                                at,
                                contribution,
                                area,
                                Point3::from(centre),
                                face.outer_wire.edges.len(),
                                face.inner_wires.len(),
                            )
                        })
                        .collect();
                    let sum: f64 = rows.iter().map(|row| row.1).sum();
                    rows.sort_by(|left, right| right.2.partial_cmp(&left.2).unwrap());
                    for (at, contribution, area, centre, edges, inner) in rows.iter() {
                        println!(
                            "      面{at:<3} 面積 {area:.6}  重心 ({:.4} {:.4} {:.4})  稜 {edges} 内輪 {inner}  寄与 {contribution:+.6}",
                            centre.x, centre.y, centre.z
                        );
                    }
                    println!("    寄与の合計 {sum:.6}");
                    // **巻き方が食い違っている面を数えます**（4-566）。
                    //
                    // **面積の合計は OCC と 3e-5 で一致するのに、体積が 4%
                    // 多い**（4-564）——**面は正しく、向きが裏返っている**
                    // という形です。**メッシュの穴・重なりは 0** ですが、
                    // **あの数え方は稜が何回使われたかだけ**で、
                    // **向きを見ていません**。
                    //
                    // **閉じて向きの揃った殻では、向き付きの稜は 1 回ずつ**
                    // です（逆向きが 1 回、必ず相手にいる）。**同じ向きで
                    // 2 回出てくる稜**は、そこで巻き方が食い違っています。
                    {
                        let counts = zenith_tess::face_triangle_counts(solid, &params);
                        let cell = |point: zenith_math::Point3| {
                            (
                                (point.x / 1e-6).round() as i64,
                                (point.y / 1e-6).round() as i64,
                                (point.z / 1e-6).round() as i64,
                            )
                        };
                        let mut directed: std::collections::BTreeMap<
                            ((i64, i64, i64), (i64, i64, i64)),
                            usize,
                        > = std::collections::BTreeMap::new();
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
                        // **どの面の三角形か**を、面ごとの三角形数から引きます。
                        let mut at = 0usize;
                        let mut bad_by_face: Vec<(u64, usize)> = Vec::new();
                        for (face_id, triangle_count) in &counts {
                            let mut bad = 0usize;
                            for index in at..(at + triangle_count) {
                                let triangle = mesh.indices[index];
                                for pair in [
                                    (triangle[0] as usize, triangle[1] as usize),
                                    (triangle[1] as usize, triangle[2] as usize),
                                    (triangle[2] as usize, triangle[0] as usize),
                                ] {
                                    let key = (
                                        cell(mesh.positions[pair.0]),
                                        cell(mesh.positions[pair.1]),
                                    );
                                    if directed.get(&key).copied().unwrap_or(0) > 1 {
                                        bad += 1;
                                    }
                                }
                            }
                            if bad > 0 {
                                bad_by_face.push((*face_id, bad));
                            }
                            at += triangle_count;
                        }
                        let same: usize = directed.values().filter(|count| **count > 1).count();
                        // **長さ 0 の辺を分けて数えます**（4-568）。**両端が同じ枡に
                        // 落ちる辺は、辺ではありません**——**それを「向きが食い違う」と
                        // 数えていたら、この口は嘘をつきます。
                        let degenerate: usize = directed
                            .iter()
                            .filter(|(key, count)| **count > 1 && key.0 == key.1)
                            .count();
                        println!(
                            "    同じ向きで 2 回以上使われた稜 {same} 本（うち長さ 0 が {degenerate} 本、本物は {} 本）",
                            same - degenerate
                        );
                        // **具体例を 3 つ名指しします**（4-568）。**どの 2 枚が、どの辺を
                        // 同じ向きに使っているか**——**それが分からないと、
                        // 数え方の嘘か、本物の欠陥かが決まりません。**
                        {
                            let mut shown = 0usize;
                            let mut at = 0usize;
                            let mut owner: std::collections::BTreeMap<
                                ((i64, i64, i64), (i64, i64, i64)),
                                Vec<u64>,
                            > = std::collections::BTreeMap::new();
                            for (face_id, triangle_count) in &counts {
                                for index in at..(at + triangle_count) {
                                    let triangle = mesh.indices[index];
                                    for pair in [
                                        (triangle[0] as usize, triangle[1] as usize),
                                        (triangle[1] as usize, triangle[2] as usize),
                                        (triangle[2] as usize, triangle[0] as usize),
                                    ] {
                                        let key = (
                                            cell(mesh.positions[pair.0]),
                                            cell(mesh.positions[pair.1]),
                                        );
                                        if directed.get(&key).copied().unwrap_or(0) > 1 {
                                            owner.entry(key).or_default().push(*face_id);
                                        }
                                    }
                                }
                                at += triangle_count;
                            }
                            for (key, faces) in owner.iter() {
                                if shown >= 3 {
                                    break;
                                }
                                println!(
                                    "      例: ({:.6} {:.6} {:.6}) -> ({:.6} {:.6} {:.6}) を面 {:?} が同じ向きに使っています",
                                    key.0 .0 as f64 * 1e-6,
                                    key.0 .1 as f64 * 1e-6,
                                    key.0 .2 as f64 * 1e-6,
                                    key.1 .0 as f64 * 1e-6,
                                    key.1 .1 as f64 * 1e-6,
                                    key.1 .2 as f64 * 1e-6,
                                    faces
                                );
                                shown += 1;
                            }
                        }
                        bad_by_face.sort_by_key(|row| std::cmp::Reverse(row.1));
                        // **面積と寄与も出します**（4-568）。**巻き方の食い違いが
                        // 体積に効いているかは、その面の寄与の大きさで決まります**
                        // ——**寄与が 1e-5 より小さければ、体積には出ません**。
                        for (face_id, bad) in bad_by_face.iter().take(12) {
                            let found = solid
                                .outer_shell
                                .faces
                                .iter()
                                .find(|face| face.id == *face_id);
                            match found {
                                Some(face) => {
                                    let (area, contribution) =
                                        MassCalculator::compute_face_integral(face, &params);
                                    println!(
                                        "      面(id {face_id}) の三角形の辺 {bad} 本が食い違い——面積 {area:.6}、寄与 {contribution:+.6}、稜 {} 内輪 {}",
                                        face.outer_wire.edges.len(),
                                        face.inner_wires.len()
                                    );
                                }
                                None => println!("      面(id {face_id}) の三角形の辺 {bad} 本が食い違い"),
                            }
                        }
                    }
                }
            }
        }
        Err(message) => println!("{which}: 断られました（{seconds:.1} 秒）: {message}"),
    }
}
