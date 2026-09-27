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
use zenith_math::{Tolerance, Vec3};
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
            }
        }
        Err(message) => println!("{which}: 断られました（{seconds:.1} 秒）: {message}"),
    }
}
