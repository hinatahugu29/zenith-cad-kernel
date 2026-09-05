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

/// **表示メッシュと検査で共通に使う刻み**（4-290）。
///
/// ここを1か所にまとめてあります。**同じ立体を別の刻みで測って数字を並べる
/// と、片方だけを見た人が読み違えます。**
fn display_params() -> TessellationParams {
    TessellationParams {
        u_divisions: 24,
        v_divisions: 24,
    }
}

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
/// **表示メッシュと同じ刻みで測ります**（4-290）。
///
/// ここは長く 16 分割で、表示メッシュのほうは 24 分割でした。同じ立体に
/// ついて「穴 0 本」と「非多様体 45 本」が並び、**ブーリアンが壊したように
/// 読めます**——実際は刻みの違いで、45 本は**読んだ立体そのもの**に出て
/// いました（和・差・積はそれを引き継いでいるだけ）。**私はそう読み違え
/// ました。**
///
/// **物差しは1つにします。** 刻みを変えて測りたいときは、変えたことを
/// 書いてください。
fn non_manifold_edges(solid: &Solid) -> usize {
    let mesh = zenith_tess::tessellate_solid(solid, &display_params());
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
    // **1ファイルだけ回す口**（4-304）。`linkrods` を1回追うのに
    // `screw` の 8〜48 分割まで回すと、毎回 2 分よけいに待ちます。
    // 診断を何度も振るときは `ZENITH_READ_CUT_ONLY=linkrods` のように
    // 名前の一部を渡してください。**門としては既定（全部）で回します。**
    let only = std::env::var("ZENITH_READ_CUT_ONLY").unwrap_or_default();

    println!("読んだ立体（OCCT の配布データ）を切る（9-H の H8）");
    println!();

    for name in samples {
        if !only.is_empty() && !name.contains(only.as_str()) {
            continue;
        }
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
        // **読んだ立体そのものが水密か**（4-277）。切る前に見ます——ここが
        // 開いていると、内外判定を使う検査が全部あてになりません（4-276）。
        {
            let params24 = display_params();
            let mesh = zenith_tess::tessellate_solid(&subject, &params24);
            // **刻みを振って測ります**（4-296）。24 で 0 でも、16 では
            // 45 本残っていました（4-290）。**「見えなくなった」と「直った」は
            // 別**なので、範囲で見ます。
            // **刻みを1つに絞る口**（4-329。`ZENITH_READ_CUT_DIVISIONS=16`）。
            // 壊れる刻みを診断つきで追うとき、7 通り全部の出力が混ざると
            // 読めません。**門としては既定（7 通り）で回します。**
            let only_divisions: Option<usize> = std::env::var("ZENITH_READ_CUT_DIVISIONS")
                .ok()
                .and_then(|value| value.parse().ok());
            for divisions in [8usize, 12, 16, 20, 24, 32, 48] {
                if let Some(want) = only_divisions {
                    if divisions != want {
                        continue;
                    }
                }
                let params = zenith_tess::TessellationParams {
                    u_divisions: divisions,
                    v_divisions: divisions,
                };
                let probe = zenith_tess::tessellate_solid(&subject, &params);
                let mut uses: std::collections::HashMap<(u32, u32), usize> =
                    std::collections::HashMap::new();
                for triangle in &probe.indices {
                    for step in 0..3 {
                        let (a, b) = (triangle[step], triangle[(step + 1) % 3]);
                        if a == b {
                            continue;
                        }
                        let key = if a < b { (a, b) } else { (b, a) };
                        *uses.entry(key).or_insert(0) += 1;
                    }
                }
                let open = uses.values().filter(|count| **count == 1).count();
                let over = uses.values().filter(|count| **count > 2).count();
                println!(
                    "  {divisions:>3} 分割: 三角形 {:>6}、穴 {open:>4} 本、重なり {over:>4} 本{}",
                    probe.indices.len(),
                    if open + over > 0 { "  **水密ではありません**" } else { "" }
                );
                // **壊れている場所を座標で出します**（`ZENITH_SEAM_WHY=1`。4-298）。
                //
                // 本数だけでは直せません。**継ぎ目の上か、面の内側か**は座標を
                // 見ないと決まらず、そこから `ZENITH_RING_WATCH` へ渡せます。
                if std::env::var_os("ZENITH_SEAM_WHY").is_some() && open + over > 0 {
                    let mut listed: Vec<((u32, u32), usize)> = uses
                        .iter()
                        .filter(|(_, count)| **count != 2)
                        .map(|(key, count)| (*key, *count))
                        .collect();
                    listed.sort_by_key(|((a, b), _)| (*a, *b));
                    // **何点から出ているか**を先に言います（4-302）。
                    // 1点なら、その頂点のまわりだけの話です。
                    let mut ends: std::collections::HashSet<(i64, i64, i64)> = Default::default();
                    let key_of = |p: zenith_math::Point3| {
                        (
                            (p.x * 1e6).round() as i64,
                            (p.y * 1e6).round() as i64,
                            (p.z * 1e6).round() as i64,
                        )
                    };
                    for ((a, b), _) in listed.iter() {
                        ends.insert(key_of(probe.positions[*a as usize]));
                        ends.insert(key_of(probe.positions[*b as usize]));
                    }
                    println!(
                        "      壊れた稜 {} 本は、**{} 個の点**につながっています",
                        listed.len(),
                        ends.len()
                    );
                    for ((a, b), count) in listed.iter().take(4) {
                        let (pa, pb) = (probe.positions[*a as usize], probe.positions[*b as usize]);
                        println!(
                            "      使用 {count} 回: ({:.6},{:.6},{:.6}) -> ({:.6},{:.6},{:.6}) 長さ {:.3e}",
                            pa.x, pa.y, pa.z, pb.x, pb.y, pb.z, (pb - pa).norm()
                        );
                    }
                }
            }
            let mut uses: std::collections::HashMap<(u32, u32), usize> =
                std::collections::HashMap::new();
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
            let open = uses.values().filter(|count| **count == 1).count();
            let over = uses.values().filter(|count| **count > 2).count();
            println!(
                "  表示メッシュ: 三角形 {}、**穴（1回しか使われない稜） {open} 本**、重なり（3回以上） {over} 本",
                mesh.indices.len()
            );
            // **穴の場所を1つ名指しします**（4-277）。座標が分かれば
            // `ZENITH_RING_WATCH` で待ち伏せられます。
            if let Some(((a, b), _)) = uses
                .iter()
                .filter(|(_, count)| **count == 1)
                .min_by_key(|((a, b), _)| (*a, *b))
            {
                let (pa, pb) = (mesh.positions[*a as usize], mesh.positions[*b as usize]);
                println!(
                    "    穴の一例: ({:.9},{:.9},{:.9}) -> ({:.9},{:.9},{:.9}) 長さ {:.3e}",
                    pa.x,
                    pa.y,
                    pa.z,
                    pb.x,
                    pb.y,
                    pb.z,
                    (pb - pa).norm()
                );
            }
            // **稜は、本当に面どうしで共有されているか**（4-277）。
            //
            // 縫合は「同じ稜の番号なら同じ点を出す」ことで継ぎ目を閉じます。
            // 番号が面ごとに別なら、**同じ場所を別々に標本する**ので穴が
            // 開きます。使われ方を数えれば分かります——閉じた立体では
            // **どの稜もちょうど2回**使われるはずです。
            let mut edge_uses: std::collections::HashMap<u64, usize> =
                std::collections::HashMap::new();
            for face in &subject.outer_shell.faces {
                for wire in std::iter::once(&face.outer_wire).chain(face.inner_wires.iter()) {
                    for oriented in &wire.edges {
                        *edge_uses.entry(oriented.edge.id).or_insert(0) += 1;
                    }
                }
            }
            let once = edge_uses.values().filter(|count| **count == 1).count();
            let twice = edge_uses.values().filter(|count| **count == 2).count();
            let more = edge_uses.values().filter(|count| **count > 2).count();
            println!(
                "  B-Rep の稜: {} 本（**1面からしか使われない {once} 本**、2面 {twice} 本、3面以上 {more} 本）",
                edge_uses.len()
            );
            // **面の種類の内訳**（4-277）。縫合の経路は平面と NURBS だけを
            // 扱い、**それ以外は別の道**（`tessellate_face`）に落ちます。
            // その道は面ごとに独立に標本するので、**継ぎ目が合いません**。
            let mut kinds: std::collections::BTreeMap<&str, usize> = Default::default();
            for face in &subject.outer_shell.faces {
                let kind = match &face.geometry {
                    zenith_topo::FaceGeometry::Plane(_) => "平面",
                    zenith_topo::FaceGeometry::Nurbs(_) => "NURBS",
                    zenith_topo::FaceGeometry::Coons(_) => "**Coons**",
                    zenith_topo::FaceGeometry::Gordon(_) => "**Gordon**",
                    zenith_topo::FaceGeometry::Triangular(_) => "**三角パッチ**",
                };
                *kinds.entry(kind).or_insert(0) += 1;
            }
            let listed: Vec<String> = kinds
                .iter()
                .map(|(kind, count)| format!("{kind} {count}"))
                .collect();
            println!("  面の種類: {}", listed.join("、"));
            // **番号と種類を並べます**（4-296）。壊れる面を名指しできたとき、
            // それが何の面かをすぐ引けるように。
            if std::env::var_os("ZENITH_FACE_KIND_WHY").is_some() {
                for face in &subject.outer_shell.faces {
                    let kind = match &face.geometry {
                        zenith_topo::FaceGeometry::Plane(_) => "平面".to_string(),
                        zenith_topo::FaceGeometry::Nurbs(surface) => format!(
                            "NURBS {}x{}",
                            surface.control_points.len(),
                            surface.control_points.first().map(|r| r.len()).unwrap_or(0)
                        ),
                        _ => "その他".to_string(),
                    };
                    println!(
                        "    面 {}: {kind}、稜 {}、内側の輪 {}、粗さ {:.3e}",
                        face.id,
                        face.outer_wire.edges.len(),
                        face.inner_wires.len(),
                        face.tolerance
                    );
                }
            }
            // **平面の輪の巻き方向**（4-279）。面の向きと合っているか。
            // 合っていないと、外向き法線が材料の反対を向きます。
            {
                let tol = Tolerance::default();
                let mut bad = 0usize;
                let mut listed = Vec::new();
                for face in &subject.outer_shell.faces {
                    let zenith_topo::FaceGeometry::Plane(_) = &face.geometry else {
                        continue;
                    };
                    let Ok(pcurves) = face.pcurves(&tol) else {
                        continue;
                    };
                    // 符号付き面積（多角形近似）。
                    let mut area = 0.0;
                    for segment in &pcurves.outer_loop.segments {
                        let (t0, t1) = segment.curve.param_range();
                        for step in 0..8 {
                            let a = segment.curve.evaluate(t0 + (t1 - t0) * step as f64 / 8.0);
                            let b = segment
                                .curve
                                .evaluate(t0 + (t1 - t0) * (step + 1) as f64 / 8.0);
                            area += a.x * b.y - a.y * b.x;
                        }
                    }
                    area *= 0.5;
                    let oriented = if face.orientation.is_forward() {
                        area
                    } else {
                        -area
                    };
                    if oriented <= 0.0 {
                        bad += 1;
                        if listed.len() < 4 {
                            listed.push(format!(
                                "面 {}（{}、符号付き面積 {oriented:.3e}）",
                                face.id,
                                if face.orientation.is_forward() {
                                    "正"
                                } else {
                                    "逆"
                                }
                            ));
                        }
                    }
                }
                println!(
                    "  平面の輪: **向きが合わないもの {bad} 枚**{}",
                    if listed.is_empty() {
                        String::new()
                    } else {
                        format!("——{}", listed.join("、"))
                    }
                );
            }
            // **稜ごとに、使っている面を並べます**（4-277）。継ぎ目が開いて
            // いるとき、**その稜を持つ2枚が同じ刻みで取っているか**を見ます。
            {
                let mut users: std::collections::BTreeMap<u64, Vec<u64>> = Default::default();
                for face in &subject.outer_shell.faces {
                    for wire in std::iter::once(&face.outer_wire).chain(face.inner_wires.iter()) {
                        for oriented in &wire.edges {
                            users.entry(oriented.edge.id).or_default().push(face.id);
                        }
                    }
                }
                for (edge, faces) in users.iter().take(6) {
                    println!("    稜 {edge}: 面 {faces:?}");
                }
            }
        }

        println!(
            "  面 {}、体積 {va:.6}、差し渡し ({:.3}, {:.3}, {:.3})",
            face_count(&subject),
            span.x,
            span.y,
            span.z
        );

        // **半分に食い込む箱**で切ります。中心を通す置き方は、面をいちばん
        // 多く割ります。
        //
        // **既定は「普通の置き方」です**（4-294）。それまでは切り手の側面を
        // **立体の境界面そのもの**（`bbox.min.x` と `bbox.max.x`）に置いて
        // いました。**そこは必ず端の特徴に接します**——実測（`linkrods.step`）:
        // 切り手の面 `x = 8.145285` が、穴の口の円（中心 7.75、半径 0.395285）に
        // **ちょうど接して**いました（`7.75 + 0.395285 = 8.145285`）。
        // **接触はこのカーネルがいちばん手を焼く配置**で（3-1、3-N）、
        // H8 は**いちばん難しい置き方だけ**を測っていたことになります。
        //
        // 側面を内側へ 3% 寄せ、切る高さも半分ちょうどから外します。
        // **難しい置き方も測れるように、`ZENITH_CUT_TANGENT=1` で戻せます。**
        let tangent = std::env::var_os("ZENITH_CUT_TANGENT").is_some();
        let (inset, height) = if tangent { (0.0, 0.5) } else { (0.03, 0.47) };
        let cutter = BrepTransform::translate_solid(
            &PrimitiveBuilder::make_box(
                span.x * (1.0 - inset * 2.0),
                span.y * (1.0 - inset * 2.0),
                span.z * height,
            )
            .expect("cutter"),
            Vec3::new(
                bbox.min.x + span.x * inset,
                bbox.min.y + span.y * inset,
                bbox.min.z + span.z * (height * 0.5),
            ),
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
            // `ZENITH_READ_CUT_BUDGET`（秒）で変えられます。既定は 300 秒。
            //
            // **120 秒では足りませんでした**（4-305）。`linkrods` は
            // **131.7〜144.4 秒**で返って断りを返しますが、既定が 120 秒
            // だったので**「返りません」と表示され、断りの中身が見えて
            // いませんでした**。引継書の「3演算とも 80 秒で返る」も、
            // ここで測ったものなので**古い数字です**。
            //
            // **上限は、いま掛かる時間の 2 倍くらいに置いてください。**
            // 詰めすぎると、遅いだけのものが「止まった」に化けます。
            let budget = std::env::var("ZENITH_READ_CUT_BUDGET")
                .ok()
                .and_then(|v| v.parse::<u64>().ok())
                .unwrap_or(300);
            // **返らないときに、進んでいるかどうかを出します**（4-272）。
            // `ZENITH_PROGRESS=15` なら 15 秒ごとに「経過と、その間に増えた
            // 仕事量」が出ます。**仕事量が増えていなければ、収束しない輪の
            // 中にいます。** 待つのをやめたあとも、この行だけが手掛かりです。
            let _beat = zenith_geom::progress::Heartbeat::start(format!("{name} {label}"));
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
                                    solid.outer_shell.faces.iter().map(|face| face.id).collect()
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
