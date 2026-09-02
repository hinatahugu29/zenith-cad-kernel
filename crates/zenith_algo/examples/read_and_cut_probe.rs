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
    PathBuf::from(concat!(
        env!("CARGO_MANIFEST_DIR"),
        "/../../reference/OCCT/data/step"
    ))
    .join(name)
}

/// 立体の**境界（ワイヤ）だけ**の囲み箱。
///
/// `Solid::bounding_box()` は、トリムされた NURBS では曲面の制御点まで含み
/// ます。**面がどこにあるか**を知りたいときは、境界を見ます（4-269）。
fn boundary_bounding_box(solid: &zenith_topo::Solid) -> Option<zenith_math::BoundingBox3> {
    let mut bbox: Option<zenith_math::BoundingBox3> = None;
    for shell in std::iter::once(&solid.outer_shell).chain(solid.inner_shells.iter()) {
        for face in &shell.faces {
            for wire in std::iter::once(&face.outer_wire).chain(face.inner_wires.iter()) {
                for point in wire.sample_points(12) {
                    match &mut bbox {
                        Some(box3) => box3.extend_point(point),
                        None => bbox = Some(zenith_math::BoundingBox3::from_point(point)),
                    }
                }
            }
        }
    }
    bbox
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
        // **`Solid::bounding_box()` は、立体の広がりではありません**（4-269）。
        //
        // トリムされた NURBS の面では、**曲面の制御点まで**入ります。読んだ
        // ファイルは自由曲面（次数 6x10）を持つので、そこが大きく効きます。
        // 実測（`linkrods.step`）:
        //
        // | | x | y | z |
        // | :--- | :--- | :--- | :--- |
        // | `bounding_box()` | [-3.273, 9.468] | [-14.383, 19.751] | **[-13.604, 14.922]** |
        // | **境界（ワイヤ）だけ** | [3.125, 8.145] | [2.500, 4.000] | **[0.000, 2.000]** |
        //
        // **z の差し渡しが 14 倍**違います。前はここから切り手を作っていた
        // ので、**箱が部品を丸ごと飲み込み**、差が空・積が A 丸ごとになって
        // いました（4-267 で「分類が間違っている」と書いたのは**誤診**です）。
        let bbox = boundary_bounding_box(&subject).unwrap_or_else(|| subject.bounding_box());
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
            &PrimitiveBuilder::make_box(span.x, span.y, span.z * 0.5).expect("cutter"),
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
            // **走る前に、走ることを言います**（4-269）。
            //
            // 切り手を実寸にしたら、`linkrods.step` の演算が **2時間20分
            // 回っても返りませんでした**。出力は溜め込まれるので、**画面には
            // 1行も出ません**——止まっているのか進んでいるのかが分かりません。
            //
            // 掃き出しは**数秒から十数秒で判定が付く**のが決まりです
            // （4-252）。ここが返らないこと自体が H8 の壁なので、**どの演算で
            // 止まったかが残る**ようにします。
            print!("  {label:<13} 走らせています…");
            use std::io::Write;
            let _ = std::io::stdout().flush();
            let started = std::time::Instant::now();

            // **返らない演算があります**（4-269）。実測: `linkrods.step` の
            // 和は **2時間20分回っても返りません**（自由曲面 次数 6x10 を
            // 箱で切る）。**掃き出しが止まったら、掃き出しではありません。**
            //
            // 別の糸で走らせて、待つのをやめます。**止めることはできない**
            // ので糸は走り続けますが、プロセスが終われば消えます。上限は
            // `ZENITH_READ_CUT_BUDGET`（秒）で変えられます。既定は 120 秒。
            let budget = std::env::var("ZENITH_READ_CUT_BUDGET")
                .ok()
                .and_then(|v| v.parse::<u64>().ok())
                .unwrap_or(120);
            // **返らないときに、進んでいるかどうかを出します**（4-272）。
            // `ZENITH_PROGRESS=15` なら 15 秒ごとに「経過と、その間に増えた
            // 仕事量」が出ます。**仕事量が増えていなければ、収束しない輪の
            // 中にいます。** 待つのをやめたあとも、この行だけが手掛かりです。
            let _beat = zenith_geom::progress::Heartbeat::start(format!(
                "{name} {label}"
            ));
            let (sender, receiver) = std::sync::mpsc::channel();
            let (a, b) = (subject.clone(), cutter.clone());
            std::thread::spawn(move || {
                let tol = Tolerance::default();
                let _ = sender.send(BooleanEngine::boolean_solids_exact_result(&a, &b, op, &tol));
            });
            let outcome = match receiver.recv_timeout(std::time::Duration::from_secs(budget)) {
                Ok(result) => {
                    println!(" {:.1} 秒", started.elapsed().as_secs_f64());
                    result
                }
                Err(_) => {
                    println!(" **{budget} 秒で返りません**（待つのをやめました）");
                    continue;
                }
            };
            match outcome {
                Ok(result) => {
                    let bad: usize = result.solids.iter().map(non_manifold_edges).sum();
                    let value = volume(&result.solids);
                    volumes[index] = Some(value);
                    println!(
                        "  {label:<13} ok  立体 {}、体積 {value:.6}、メッシュ非多様体 {bad} 本",
                        result.solids.len()
                    );
                }
                Err(reason) if reason.contains("inside-out or collapsed") => {
                    // **裏返った立体を名指しします**（4-275）。断り文には
                    // 番号と体積しか出ないので、**それがどこにある何なのか**
                    // が分かりません。空洞（内側の殻）を別の立体として
                    // 返していないかを、その場で見られるようにします。
                    println!("  {label:<13} 断られた（裏返り）:");
                    for line in reason.split("; ") {
                        println!("    {line}");
                    }
                    let tol = Tolerance::default();
                    if let Ok(again) =
                        // **検証を通さない口で取り直します**——通すと同じ
                        // ところで断られて、中身が見られません。
                        BooleanEngine::boolean_solids_exact_result_unverified(
                            &subject, &cutter, op, &tol,
                        )
                    {
                        // **答えの体積を、ブーリアンに依らずに決めます**（4-276）。
                        //
                        // 符号付きの和（579.89）と、絶対値の和（1056.07）の
                        // どちらが正しいのかは、**A と B だけを見れば決まります**。
                        // A の範囲に点をばらまき、**A の中にあって B の外**に
                        // ある割合を数えれば、それが |A＼B| です。
                        // **決定的にするため、種を固定した格子＋ずらし**で撒きます。
                        {
                            let params24 = zenith_tess::TessellationParams {
                                u_divisions: 24,
                                v_divisions: 24,
                            };
                            let mesh_a = zenith_tess::tessellate_solid(&subject, &params24);
                            let mesh_b = zenith_tess::tessellate_solid(&cutter, &params24);
                            let bbox = subject.bounding_box();
                            let span = bbox.max - bbox.min;
                            let steps = 60usize;
                            let (mut in_a, mut in_a_not_b) = (0u64, 0u64);
                            for ix in 0..steps {
                                for iy in 0..steps {
                                    for iz in 0..steps {
                                        // 格子の真ん中を採る（面に乗りにくい）。
                                        let point = zenith_math::Point3::new(
                                            bbox.min.x + span.x * (ix as f64 + 0.5) / steps as f64,
                                            bbox.min.y + span.y * (iy as f64 + 0.5) / steps as f64,
                                            bbox.min.z + span.z * (iz as f64 + 0.5) / steps as f64,
                                        );
                                        if !BooleanEngine::is_point_inside_mesh(point, &mesh_a) {
                                            continue;
                                        }
                                        in_a += 1;
                                        if !BooleanEngine::is_point_inside_mesh(point, &mesh_b) {
                                            in_a_not_b += 1;
                                        }
                                    }
                                }
                            }
                            let cell = span.x * span.y * span.z / (steps * steps * steps) as f64;
                            // **この口が信用できるかを、先に確かめます。**
                            // 点の数え方は光線の交差回数で決めるので、
                            // **メッシュに穴があると当てになりません**。
                            // A の体積は面積分で分かっているので、突き合わせます。
                            let sampled_a = in_a as f64 * cell;
                            let exact_a =
                                zenith_algo::MassCalculator::compute_from_brep(&subject, &params24)
                                    .volume;
                            let gap = (sampled_a - exact_a).abs() / exact_a.abs().max(1e-30);
                            println!(
                                "      **点で数えた体積**: A {sampled_a:.3}（面積分では {exact_a:.3}、ずれ {:.1}%）、A＼B {:.3}",
                                gap * 100.0,
                                in_a_not_b as f64 * cell
                            );
                            if gap > 0.02 {
                                println!(
                                    "      **この数え方は使えません**——メッシュに穴があると光線の数え方が狂います。**ブーリアンの検証も同じ数え方です**"
                                );
                            }
                        }

                        // **同じ面が2つの殻に入っていないか**（4-276）。
                        // 入っていれば**二重被覆**です（4-137 で一度見た形）。
                        // 面の番号は割った断片にも引き継がれるので、番号で
                        // 突き合わせれば分かります。
                        {
                            let ids: Vec<Vec<u64>> = again
                                .solids
                                .iter()
                                .map(|solid| {
                                    solid
                                        .outer_shell
                                        .faces
                                        .iter()
                                        .map(|face| face.id)
                                        .collect()
                                })
                                .collect();
                            for left in 0..ids.len() {
                                for right in (left + 1)..ids.len() {
                                    let shared: Vec<u64> = ids[left]
                                        .iter()
                                        .filter(|id| ids[right].contains(id))
                                        .copied()
                                        .collect();
                                    println!(
                                        "      立体 {left} と {right} で**同じ番号の面** {} 枚{}",
                                        shared.len(),
                                        if shared.is_empty() {
                                            String::new()
                                        } else {
                                            format!("（{shared:?}）")
                                        }
                                    );
                                }
                            }
                        }
                        for (index, solid) in again.solids.iter().enumerate() {
                            let bbox = solid.bounding_box();
                            let params = zenith_tess::TessellationParams {
                                u_divisions: 24,
                                v_divisions: 24,
                            };
                            let volume =
                                zenith_algo::MassCalculator::compute_from_brep(solid, &params)
                                    .volume;
                            // **その殻が、ほかの立体の中にあるか**（4-275）。
                            // 空洞なら、別の立体ではなく**内側の殻**として
                            // 返すべきものです。範囲だけでは決まらないので、
                            // 殻の上の点を使って包含を見ます。
                            let mut inside_of = String::from("-");
                            // **重心を使います**（4-275）。稜の上の点は境界
                            // そのものなので、包含の判定が当てになりません。
                            let properties =
                                zenith_algo::MassCalculator::compute_from_brep(solid, &params);
                            let probe_point = Some(properties.center_of_mass);
                            if let Some(point) = probe_point {
                                for (other_index, other) in again.solids.iter().enumerate() {
                                    if other_index == index {
                                        continue;
                                    }
                                    let mesh = zenith_tess::tessellate_solid(
                                        other,
                                        &zenith_tess::TessellationParams {
                                            u_divisions: 24,
                                            v_divisions: 24,
                                        },
                                    );
                                    if BooleanEngine::is_point_inside_mesh(point, &mesh) {
                                        inside_of = format!("立体 {other_index} の中");
                                    }
                                }
                            }
                            println!(
                                "      立体 {index}: 体積 {volume:+.6}、面 {}、内側の殻 {}、{inside_of}、範囲 ({:.3}, {:.3}, {:.3})〜({:.3}, {:.3}, {:.3})",
                                solid.outer_shell.faces.len(),
                                solid.inner_shells.len(),
                                bbox.min.x, bbox.min.y, bbox.min.z,
                                bbox.max.x, bbox.max.y, bbox.max.z
                            );
                        }
                    }
                }
                Err(reason) => {
                    // **断り文は切り詰めません**（4-275）。90 字で切っていた
                    // ので、検証が何を見て断ったのかが読めませんでした——
                    // `volume A=..., B=839` で切れており、肝心の**どの検査が
                    // 落ちたか**が消えていました。**直す側が読むのは末尾です。**
                    println!("  {label:<13} 断られた:");
                    for line in reason.split("; ") {
                        println!("    {line}");
                    }
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
            // **中身の無い恒等式を、緑と数えないこと**（4-267）。
            //
            // 差が空で積が A 丸ごとなら、`0 + |A| - |A| = 0` は**必ず**閉じます。
            // 何も測っていません。実測: `linkrods.step` がこれで、**部品は
            // 切り手の z 範囲からはみ出しているのに**積が部品全体を返して
            // いました（分類の誤り）。4-211 と同じ形の落とし穴です。
            let empty_difference = difference.abs() <= va.abs() * 1e-9;
            let whole_intersection = (intersection - va).abs() <= va.abs() * 1e-9;
            if empty_difference && whole_intersection {
                println!("  **この恒等式は中身がありません**——差が空で積が A 丸ごとなので、必ず閉じます。");
                println!("  **切り手が部品を丸ごと含んでいるか、分類が間違っています。**");
            }
        } else {
            println!("  **恒等式**: 3演算そろわないので測れません");
        }
        println!();
    }

    println!("**読めないこと・断ることは、ここでは赤にしません。** 次にやることが");
    println!("決まる、という事実として置きます。赤にするのは「返ってきたのに");
    println!("答えが合わない」ほうです。");
}
