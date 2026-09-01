//! **読んだ立体（STEP）を切る**——掃き出しの4本目の軸（9-H の H8）。
//!
//! 常設の検体（`tests/fixtures/occ_reference_*.step`）は、**このカーネルが
//! 書いたものを OCCT が読み直した**ものか、素直な形です。ここで読むのは
//! **OCCT が配っている実物**（`reference/OCCT/data/step/`）——面の多い、
//! 誰かの設計データです。
//!
//! ## なぜ要るのか
//!
//! **実用でいちばん多い使われ方が、他人のデータを読んで切ることです。**
//! 4-142 の誤答も、他カーネルの立体を入れて初めて見えました。
//!
//! ## 何を見るか
//!
//! - **読めるか**（面の数、体積、閉じているか）
//! - 切ったときに**恒等式が閉じるか**（`|A＼B| + |A∩B| = |A|`、
//!   `|A∪B| + |A∩B| = |A| + |B|`）。閉じた式が要らないので、どんな形でも
//!   採点できます
//! - **非多様体を返していないか**
//!
//! **読めないこと自体は、ここでは赤にしません。** 読めなければ、それは
//! 「まだ届いていない」という事実で、次にやることが決まります。
use std::path::PathBuf;
use zenith_algo::{
    BooleanEngine, BooleanOpType, BrepTransform, MassCalculator, PrimitiveBuilder, Regularizer,
};
use zenith_io::StepImporter;
use zenith_math::{Tolerance, Vec3};
use zenith_tess::TessellationParams;
use zenith_topo::Solid;

fn params() -> TessellationParams {
    TessellationParams {
        u_divisions: 32,
        v_divisions: 32,
    }
}

fn volume(solids: &[Solid]) -> f64 {
    solids
        .iter()
        .map(|solid| MassCalculator::compute_from_brep(solid, &params()).volume)
        .sum()
}

fn face_count(solid: &Solid) -> usize {
    std::iter::once(&solid.outer_shell)
        .chain(solid.inner_shells.iter())
        .map(|shell| shell.faces.len())
        .sum()
}

/// メッシュの稜のうち、ちょうど2枚に共有されていない本数。
fn non_manifold_edges(solid: &Solid) -> usize {
    let mesh = zenith_tess::tessellate_solid(
        solid,
        &TessellationParams {
            u_divisions: 16,
            v_divisions: 16,
        },
    );
    let mut uses: std::collections::HashMap<(u32, u32), usize> = std::collections::HashMap::new();
    for triangle in &mesh.indices {
        for step in 0..3 {
            let (a, b) = (triangle[step], triangle[(step + 1) % 3]);
            if a == b {
                continue;
            }
            let key = if a < b { (a, b) } else { (b, a) };
            *uses.entry(key).or_insert(0) += 1;
        }
    }
    uses.values().filter(|count| **count != 2).count()
}

fn occt_sample(name: &str) -> PathBuf {
    PathBuf::from(concat!(env!("CARGO_MANIFEST_DIR"), "/../../reference/OCCT/data/step"))
        .join(name)
}

fn main() {
    let tol = Tolerance::default();
    let samples = ["screw.step", "linkrods.step"];

    println!("読んだ立体（OCCT の配布データ）を切る（9-H の H8）");
    println!();

    for name in samples {
        let path = occt_sample(name);
        let solids = match StepImporter::import_solids_from_file(&path) {
            Ok(solids) => solids,
            Err(reason) => {
                println!("{name:<16} **読めません**: {reason}");
                continue;
            }
        };
        if solids.is_empty() {
            println!("{name:<16} **立体が 0 個**（面や殻だけのファイルかもしれません）");
            continue;
        }
        println!(
            "{name:<16} 立体 {} 個。いちばん面の多いものを切ります",
            solids.len()
        );

        let Some(subject) = solids
            .iter()
            .max_by_key(|solid| face_count(solid))
            .map(|solid| Regularizer::hold_like_our_own(solid, &tol))
        else {
            continue;
        };
        let bbox = subject.bounding_box();
        let span = Vec3::new(
            bbox.max.x - bbox.min.x,
            bbox.max.y - bbox.min.y,
            bbox.max.z - bbox.min.z,
        );
        let va = volume(std::slice::from_ref(&subject));
        println!(
            "  面 {}、体積 {va:.6}、差し渡し ({:.3}, {:.3}, {:.3})",
            face_count(&subject),
            span.x,
            span.y,
            span.z
        );

        // **半分に食い込む箱**で切ります。中心を通す置き方は、面をいちばん
        // 多く割ります。
        let cutter = BrepTransform::translate_solid(
            &PrimitiveBuilder::make_box(span.x, span.y, span.z * 0.5)
                .expect("cutter"),
            Vec3::new(bbox.min.x, bbox.min.y, bbox.min.z + span.z * 0.25),
        );
        let vb = volume(std::slice::from_ref(&cutter));

        let mut volumes = [None; 3];
        for (index, (label, op)) in [
            ("union", BooleanOpType::Union),
            ("difference", BooleanOpType::Difference),
            ("intersection", BooleanOpType::Intersection),
        ]
        .into_iter()
        .enumerate()
        {
            match BooleanEngine::boolean_solids_exact_result(&subject, &cutter, op, &tol) {
                Ok(result) => {
                    let bad: usize = result.solids.iter().map(non_manifold_edges).sum();
                    let value = volume(&result.solids);
                    volumes[index] = Some(value);
                    println!(
                        "  {label:<13} ok  立体 {}、体積 {value:.6}、メッシュ非多様体 {bad} 本",
                        result.solids.len()
                    );
                }
                Err(reason) => {
                    let short: String = reason.chars().take(90).collect();
                    println!("  {label:<13} 断られた: {short}");
                }
            }
        }

        if let [Some(union), Some(difference), Some(intersection)] = volumes {
            let scale = (va + vb).abs().max(1.0);
            let first = ((union + intersection) - (va + vb)).abs() / scale;
            let second = ((difference + intersection) - va).abs() / scale;
            println!(
                "  **恒等式**: |A∪B|+|A∩B|-(|A|+|B|) = {first:.3e}、|A＼B|+|A∩B|-|A| = {second:.3e}"
            );
        } else {
            println!("  **恒等式**: 3演算そろわないので測れません");
        }
        println!();
    }

    println!("**読めないこと・断ることは、ここでは赤にしません。** 次にやることが");
    println!("決まる、という事実として置きます。赤にするのは「返ってきたのに");
    println!("答えが合わない」ほうです。");
}
