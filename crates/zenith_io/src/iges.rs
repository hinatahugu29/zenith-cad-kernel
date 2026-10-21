//! Zenith IO: IGES 5.3 エクスポーター
//!
//! 各 Face の支持曲面を Entity 128（有理Bスプライン曲面）として書き出す。
//!
//! **4-769 でトリムの実体は書けるようになりました。** 面が `pcurves`
//! （境界の2D パラメータ空間曲線。自前ビルダーが持ち、
//! `pcurve_fidelity_probe` が全標本 3e-12 以内を見張っているもの）を
//! 持っていれば、Entity 128（曲面）＋ Entity 126（2D 境界曲線）＋
//! Entity 142（曲面上の曲線）＋ Entity 102（複数区間の輪を束ねる複合
//! 曲線）＋ Entity 144（トリムされた曲面）を書く。**`pcurves` を持たない
//! 面（STEP から読んだ他カーネルの面など）は、以前と同じ Entity 128
//! だけ**——導出した p-curve は書かない（導出の精度に頼ると境界が
//! 元よりずれる。`step.rs` の `plan_surfaces_and_pcurves` と同じ判断）。
//!
//! **4-771 で見つけたこと: 書いた Entity 144/142/126 は構文として正しく
//! 読めますが、実際にトリムが効いているかは未確認のまま信じていました。**
//! 面ごとの境界箱の合算では `grow 0` に見えましたが、**面ごとの面積を
//! 測ると、トリムされていない全体の面積のままでした**
//! （FreeCAD / OCCT 7.8。実測:箱の隅の四辺形が 450——矩形全体のまま、
//! 正しい 205.365 まで切り取られていない。円柱の蓋も円の面積 314.16
//! ではなく外接正方形の 400 のまま）。**最小の再現**（平面1枚を単純な
//! 内側の正方形にトリムするだけの手作りの検体）**でも同じでした**——
//! 自分のパイプライン固有の問題ではなく、**この書き方そのものが
//! まだ OCCT にトリムを認めさせられていません**。パラメータ域の正規化・
//! 複合曲線（102）の有無を変えても変わらず、根はまだ見つけていません。
//! **境界箱が揃うことと、トリムが実際に効いていることは別**だという
//! ことが、この検証不足の教訓です。
//!
//! 立体としてやり取りするなら STEP を使うこと——IGES は面の集まりで、
//! シェルや立体そのものの実体（514/186 等）はまだ書いていない。
//!
//! ここは以前、引数の `solid` を一度も読まずに、固定の文字列と中身の無い
//! Entity 186 を1つ書くだけだった（`export_solid_to_string(_solid, ..)` の
//! 先頭のアンダースコアがその証拠）。どんな立体を渡しても同じ6行が出る。
//! それでも仕様書には「Entity 128 (NURBS Surface), 102 (Composite Curve),
//! 124 (Transformation Matrix) 出力」と書かれており、テストは `"S0000001"`
//! のような文字列が含まれるかだけを見ていた。

use std::fs::File;
use std::io::Write;
use std::path::Path;
use zenith_geom::{ControlPoint3, KnotVector, NurbsCurve2, NurbsSurface3, Surface3};
use zenith_math::Point3;
use zenith_topo::{Face, FaceGeometry, FacePcurveLoop, Solid};

/// IGES 5.3 CADファイルエクスポーター
pub struct IgesExporter;

/// 1レコードは80桁。72桁までが中身で、73桁目が区分、74〜80桁が連番。
fn record(body: &str, section: char, sequence: usize) -> String {
    format!("{body:<72}{section}{sequence:>7}\n")
}

/// IGES のホレリス文字列（`<長さ>H<中身>`）。
fn hollerith(text: &str) -> String {
    format!("{}H{}", text.len(), text)
}

/// IGES の実数表記。小数点を必ず持たせる。
fn real(value: f64) -> String {
    let text = format!("{value:.10}");
    if text.contains('.') {
        text
    } else {
        format!("{text}.")
    }
}

/// 書き出す実体を1つ集める。**DE ポインタは、これを積んだ順に
/// `1, 3, 5, ...` と決まる**（各実体が DE を2行使うため）。
struct EntityWriter {
    /// (型番, フォーム番号, パラメータ本体の行)
    entities: Vec<(u32, u32, Vec<String>)>,
}

impl EntityWriter {
    fn new() -> Self {
        Self {
            entities: Vec::new(),
        }
    }

    /// 実体を積み、その DE ポインタ（1 始まりの奇数）を返す。
    fn push(&mut self, entity_type: u32, form: u32, body: Vec<String>) -> usize {
        let de_pointer = self.entities.len() * 2 + 1;
        self.entities.push((entity_type, form, body));
        de_pointer
    }
}

/// 72桁に収まるように詰める。最後のフィールドだけ `;` で閉じ、残りは `,`。
fn pack_fields(fields: &[String]) -> Vec<String> {
    let mut lines = Vec::new();
    let mut current = String::new();
    for (index, field) in fields.iter().enumerate() {
        let terminator = if index + 1 == fields.len() { ';' } else { ',' };
        let piece = format!("{field}{terminator}");
        if current.len() + piece.len() > 64 {
            lines.push(std::mem::take(&mut current));
        }
        current.push_str(&piece);
    }
    if !current.is_empty() {
        lines.push(current);
    }
    lines
}

impl IgesExporter {
    /// Solid を IGES 5.3 フォーマット (.igs / .iges) としてエクスポート
    pub fn export_solid_to_file<P: AsRef<Path>>(
        solid: &Solid,
        path: P,
        product_name: &str,
    ) -> Result<(), String> {
        let content = Self::export_solid_to_string(solid, product_name)?;
        if let Some(parent) = path.as_ref().parent() {
            std::fs::create_dir_all(parent)
                .map_err(|e| format!("Failed to create IGES directory: {}", e))?;
        }
        let mut file =
            File::create(path).map_err(|e| format!("Failed to create IGES file: {}", e))?;
        file.write_all(content.as_bytes())
            .map_err(|e| format!("Failed to write IGES file: {}", e))?;
        Ok(())
    }

    /// Solid から IGES 5.3 テキスト文字列を生成
    pub fn export_solid_to_string(solid: &Solid, product_name: &str) -> Result<String, String> {
        let mut writer = EntityWriter::new();
        let mut extent = 0.0f64;
        let mut face_count = 0usize;

        for shell in std::iter::once(&solid.outer_shell).chain(solid.inner_shells.iter()) {
            for (face_index, face) in shell.faces.iter().enumerate() {
                let surface = Self::surface_of(face)?;
                extent = extent.max(Self::surface_extent(&surface));
                let surface_de = writer.push(128, 0, Self::entity_128_parameters(&surface));
                if std::env::var_os("ZENITH_IGES_WHY").is_some() {
                    eprintln!("IGESWHY face{face_index} surface_de={surface_de}");
                }

                if let Some(pcurves) = &face.pcurves {
                    let before = writer.entities.len();
                    let edge_curves: std::collections::HashMap<u64, zenith_geom::NurbsCurve3> =
                        std::iter::once(&face.outer_wire)
                            .chain(face.inner_wires.iter())
                            .flat_map(|wire| wire.edges.iter())
                            .map(|oriented| (oriented.edge.id, oriented.edge.curve.clone()))
                            .collect();
                    Self::write_trimmed_face(&mut writer, surface_de, pcurves, &edge_curves);
                    if std::env::var_os("ZENITH_IGES_WHY").is_some() {
                        eprintln!(
                            "IGESWHY face{face_index} trim entities {}..{}",
                            before * 2 + 1,
                            writer.entities.len() * 2 - 1
                        );
                    }
                } else {
                    // `pcurves` が無い面（他カーネルから読んだ面など）は、
                    // 以前と同じ曲面だけを書く。導出した p-curve は書かない
                    // （`step.rs` の `plan_surfaces_and_pcurves` と同じ判断——
                    // 導出は射影のやり直しで、精度が元の境界より落ちる）。
                }
                face_count += 1;
            }
        }
        if face_count == 0 {
            return Err("IGES export needs at least one face".to_string());
        }

        let mut directory = String::new();
        let mut parameters = String::new();
        let mut directory_sequence = 1usize;
        let mut parameter_sequence = 1usize;

        for (index, (entity_type, form, body)) in writer.entities.iter().enumerate() {
            let directory_line = directory_sequence;
            let parameter_line = parameter_sequence;

            directory.push_str(&record(
                &format!(
                    "{:>8}{:>8}{:>8}{:>8}{:>8}{:>8}{:>8}{:>8}{:>8}",
                    entity_type, parameter_line, 0, 0, 0, 0, 0, 0, "00000000"
                ),
                'D',
                directory_sequence,
            ));
            directory_sequence += 1;

            directory.push_str(&record(
                &format!(
                    "{:>8}{:>8}{:>8}{:>8}{:>8}{:>8}{:>8}{:>8}{:>8}",
                    entity_type,
                    0,
                    0,
                    body.len(),
                    form,
                    "",
                    "",
                    format!("F{index:07}"),
                    0
                ),
                'D',
                directory_sequence,
            ));
            directory_sequence += 1;

            for line in body {
                // 65〜72桁は、この P レコードが属する DE の行番号。
                parameters.push_str(&record(
                    &format!("{line:<64}{directory_line:>8}"),
                    'P',
                    parameter_sequence,
                ));
                parameter_sequence += 1;
            }
        }

        let start = record(
            &format!("Zenith CAD Kernel IGES 5.3 export of {product_name}"),
            'S',
            1,
        );

        // Global セクションは 26 個のフィールドを持つ。
        //
        // **1番目と2番目（区切り記号の宣言）は空にする。** ここに `1H,` と
        // `1H;` を書いていたときは、OpenCASCADE が 20x30x40 の箱を
        // 508x762x1016 として読んだ。ちょうど 25.4 倍、つまり単位をインチと
        // 解釈している。区切り記号の宣言そのものが**カンマとセミコロンを値と
        // して含む**ので、フィールドを先にカンマで割る読み手はここで数を
        // 取り違え、14番目にあるはずの単位フラグを見失う。空にしておけば
        // 既定（`,` と `;`）が使われる。OpenCASCADE 自身もそう書いている。
        let file_name = format!("{product_name}.igs");
        let fields: Vec<String> = vec![
            String::new(),                        // 1  パラメータ区切り（既定 ,）
            String::new(),                        // 2  レコード区切り（既定 ;）
            hollerith(product_name),              // 3  送り手の製品ID
            hollerith(&file_name),                // 4  ファイル名
            hollerith("Zenith CAD Kernel"),       // 5  ネイティブシステムID
            hollerith("Zenith CAD Kernel 0.1.0"), // 6  プリプロセッサ版
            "32".to_string(),                     // 7  整数のビット数
            "308".to_string(),                    // 8  単精度の指数上限
            "15".to_string(),                     // 9  単精度の有効桁
            "308".to_string(),                    // 10 倍精度の指数上限
            "15".to_string(),                     // 11 倍精度の有効桁
            String::new(),                        // 12 受け手の製品ID
            "1.".to_string(),                     // 13 モデル空間の倍率
            "2".to_string(),                      // 14 単位フラグ（2 = ミリメートル）
            "2HMM".to_string(),                   // 15 単位名
            "1".to_string(),                      // 16 線幅の段階数
            "0.01".to_string(),                   // 17 最大線幅
            hollerith("20260822.120000"),         // 18 ファイル生成日時
            "1E-07".to_string(),                  // 19 最小分解能
            real(extent.max(1.0)),                // 20 座標の最大値
            String::new(),                        // 21 作成者
            String::new(),                        // 22 組織
            "11".to_string(),                     // 23 仕様の版（11 = IGES 5.3）
            "0".to_string(),                      // 24 製図規格
            hollerith("20260822.120000"),         // 25 モデル最終更新日時
            String::new(),                        // 26 プロトコル識別子
        ];

        let mut global = String::new();
        let mut global_sequence = 1usize;
        let mut current = String::new();
        for (index, field) in fields.iter().enumerate() {
            let terminator = if index + 1 == fields.len() { ';' } else { ',' };
            let piece = format!("{field}{terminator}");
            // レコードの途中でフィールドを割らない。割ると、繋ぎ直さない読み手が
            // 数値やホレリス文字列を切られたまま受け取る。
            if current.len() + piece.len() > 72 {
                global.push_str(&record(&current, 'G', global_sequence));
                global_sequence += 1;
                current.clear();
            }
            current.push_str(&piece);
        }
        if !current.is_empty() {
            global.push_str(&record(&current, 'G', global_sequence));
            global_sequence += 1;
        }

        let terminate = record(
            &format!(
                "S{:>7}G{:>7}D{:>7}P{:>7}",
                1,
                global_sequence - 1,
                directory_sequence - 1,
                parameter_sequence - 1
            ),
            'T',
            1,
        );

        Ok(format!("{start}{global}{directory}{parameters}{terminate}"))
    }

    /// 面のトリム境界（外周＋内側の輪）を Entity 144 として書く。
    ///
    /// **輪が1区間だけなら Entity 142 を直接境界として渡し、2区間以上なら
    /// Entity 102（複合曲線）で束ねる**——どちらも IGES 5.3 の Entity 144 が
    /// 受け取れる形だが、1本で済む所に複合曲線を被せると読み手によっては
    /// 退化した複合曲線を嫌う。
    fn write_trimmed_face(
        writer: &mut EntityWriter,
        surface_de: usize,
        pcurves: &zenith_topo::FacePcurves,
        edge_curves: &std::collections::HashMap<u64, zenith_geom::NurbsCurve3>,
    ) {
        let Some(outer_de) =
            Self::write_boundary_loop(writer, surface_de, &pcurves.outer_loop, edge_curves)
        else {
            return;
        };
        let inner_des: Vec<usize> = pcurves
            .inner_loops
            .iter()
            .filter_map(|loop_| {
                Self::write_boundary_loop(writer, surface_de, loop_, edge_curves)
            })
            .collect();

        let mut fields = vec!["144".to_string(), format!("{surface_de}"), "1".to_string()];
        fields.push(inner_des.len().to_string());
        fields.push(format!("{outer_de}"));
        for id in &inner_des {
            fields.push(format!("{id}"));
        }
        writer.push(144, 0, pack_fields(&fields));
    }

    /// 1つの輪（外周または内側の穴）を、曲面上の曲線（142）の並びとして書き、
    /// 2区間以上なら複合曲線（102）で束ねた DE ポインタを返す。
    fn write_boundary_loop(
        writer: &mut EntityWriter,
        surface_de: usize,
        loop_: &FacePcurveLoop,
        edge_curves: &std::collections::HashMap<u64, zenith_geom::NurbsCurve3>,
    ) -> Option<usize> {
        if loop_.segments.is_empty() {
            return None;
        }
        // **4-787 の次の一手②**: OCCT 自身が書く参照ファイルは BPTR=0・
        // CPTR優先・PREF=2（CRTN も含めて5パラメータ）で書いている
        // ——こちらの既定（BPTR優先・PREF=1・4パラメータ）とは逆の戦略。
        // `ZENITH_IGES_CPTR_PRIMARY`（既定オフ・診断用）はその逆の戦略を
        // 試すための口で、3D側の composite curve（実稜線そのもの）を
        // CPTR に渡す。既定の書き方は変えていない。
        let cptr_primary = std::env::var_os("ZENITH_IGES_CPTR_PRIMARY").is_some();
        let cptr_de = if cptr_primary {
            Self::write_boundary_loop_3d(writer, loop_, edge_curves)
        } else {
            None
        };

        let segment_des: Vec<usize> = loop_
            .segments
            .iter()
            .map(|segment| {
                let curve_de = writer.push(126, 0, Self::entity_126_parameters(&segment.curve));
                if let Some(cptr_de) = cptr_de {
                    // CRTN=0、SPTR=surface_de、BPTR=0（パラメータ空間側は
                    // 渡さない）、CPTR=cptr_de（3D composite curve 優先）、
                    // PREF=2（CPTR 側の表現を優先する、と宣言する）。
                    writer.push(
                        142,
                        0,
                        pack_fields(&[
                            "142".to_string(),
                            "0".to_string(),
                            format!("{surface_de}"),
                            "0".to_string(),
                            format!("{cptr_de}"),
                            "2".to_string(),
                        ]),
                    )
                } else {
                    // CRTN=0（未指定）、CPTR=0（3Dの複合曲線は渡さない）、
                    // PREF=1（パラメータ空間側の表現を優先する、と宣言する）。
                    writer.push(
                        142,
                        0,
                        pack_fields(&[
                            "142".to_string(),
                            format!("{surface_de}"),
                            format!("{curve_de}"),
                            "0".to_string(),
                            "1".to_string(),
                        ]),
                    )
                }
            })
            .collect();

        if segment_des.len() == 1 {
            return Some(segment_des[0]);
        }
        let mut fields = vec!["102".to_string(), segment_des.len().to_string()];
        for id in &segment_des {
            fields.push(format!("{id}"));
        }
        Some(writer.push(102, 0, pack_fields(&fields)))
    }

    /// `ZENITH_IGES_CPTR_PRIMARY` 用: 1つの輪の実稜線（3D）を Entity 126
    /// （3D曲線）の並びとして書き、2区間以上なら Entity 102 で束ねた
    /// DE ポインタを返す。`edge_id` が見つからない区間はこの輪全体を
    /// 諦める（CPTR を半端に渡すよりは、BPTR 側の既定経路に任せる）。
    fn write_boundary_loop_3d(
        writer: &mut EntityWriter,
        loop_: &FacePcurveLoop,
        edge_curves: &std::collections::HashMap<u64, zenith_geom::NurbsCurve3>,
    ) -> Option<usize> {
        let mut segment_des = Vec::with_capacity(loop_.segments.len());
        for segment in &loop_.segments {
            let curve = edge_curves.get(&segment.edge_id)?;
            segment_des.push(writer.push(126, 0, Self::entity_126_parameters_3d(curve)));
        }
        if segment_des.len() == 1 {
            return Some(segment_des[0]);
        }
        let mut fields = vec!["102".to_string(), segment_des.len().to_string()];
        for id in &segment_des {
            fields.push(format!("{id}"));
        }
        Some(writer.push(102, 0, pack_fields(&fields)))
    }

    /// Face の支持曲面を NURBS として取り出す。
    ///
    /// 平面は制御点の格子を持たないので、その面の境界がちょうど収まる矩形を
    /// 張って返す。トリムを書いていない呼び手（`pcurves` が無い面）では、
    /// ここを無限平面にすると受け側で際限なく広がってしまう。
    fn surface_of(face: &Face) -> Result<NurbsSurface3, String> {
        match &face.geometry {
            FaceGeometry::Nurbs(nurbs) => Ok(nurbs.clone()),
            FaceGeometry::Plane(plane) => {
                let mut min_u = f64::INFINITY;
                let mut max_u = f64::NEG_INFINITY;
                let mut min_v = f64::INFINITY;
                let mut max_v = f64::NEG_INFINITY;
                let mut consider = |point: Point3| {
                    let offset = point - plane.origin;
                    let u = offset.dot(&plane.u_axis);
                    let v = offset.dot(&plane.v_axis);
                    min_u = min_u.min(u);
                    max_u = max_u.max(u);
                    min_v = min_v.min(v);
                    max_v = max_v.max(v);
                };
                for wire in std::iter::once(&face.outer_wire).chain(face.inner_wires.iter()) {
                    for oriented in &wire.edges {
                        // **端点だけでは足りません。** 円を1本の稜（始点と終点が
                        // 同じ点）で持つ面（他カーネルから読んだ円柱の蓋など）では、
                        // 端点が同じ1点に重なり、円そのものの広がりが消える。
                        // 曲線に沿って標本を取る（4-770）。
                        let (t_min, t_max) = oriented.edge.curve.param_range();
                        const SAMPLES: usize = 16;
                        for i in 0..=SAMPLES {
                            let t = t_min + (t_max - t_min) * (i as f64 / SAMPLES as f64);
                            consider(oriented.edge.curve.evaluate(t));
                        }
                    }
                }
                if !(max_u > min_u && max_v > min_v) {
                    return Err("planar face has no measurable extent".to_string());
                }

                let corner = |u: f64, v: f64| {
                    ControlPoint3::unweighted(plane.origin + plane.u_axis * u + plane.v_axis * v)
                };
                // **パラメータ域を `[0,1]` に正規化せず、面の実際の (u, v)
                // 幅（`min_u..max_u`）そのものにします**（4-771）。
                //
                // `face.pcurves` の2D境界曲線は、面の支持平面の生の
                // (u, v)（`plane.origin` からの距離。0〜1 に縮めていない）
                // で持っています。曲面（Entity 128）のパラメータ域を
                // `clamped_uniform` の `[0,1]` にすると、境界曲線
                // （Entity 126）の値と**単位が食い違う**ので、ここは
                // 面の実際の幅に合わせます。**ただし、これだけでは
                // トリムが効きません**——実測すると、個々の面の面積は
                // この直しの前後で変わらず、トリム前の全体の面積の
                // ままでした（4-771）。この直しは「単位を合わせる」
                // という意味では必要ですが、**十分ではありません**。
                // （実測:
                // 30×15 の箱の隅の四辺形が、トリム後も 450——矩形全体
                // のまま、205.365 まで切り取られていない）。
                NurbsSurface3::new(
                    1,
                    1,
                    vec![
                        vec![corner(min_u, min_v), corner(min_u, max_v)],
                        vec![corner(max_u, min_v), corner(max_u, max_v)],
                    ],
                    KnotVector::new(vec![min_u, min_u, max_u, max_u]),
                    KnotVector::new(vec![min_v, min_v, max_v, max_v]),
                )
            }
            FaceGeometry::Coons(coons) => Self::sampled(coons),
            FaceGeometry::Gordon(gordon) => Self::sampled(gordon),
            FaceGeometry::Triangular(patch) => Self::sampled(patch),
        }
    }

    fn sampled(surface: &dyn Surface3) -> Result<NurbsSurface3, String> {
        NurbsSurface3::approximate_surface(surface, 12, 12)
    }

    fn surface_extent(surface: &NurbsSurface3) -> f64 {
        surface
            .control_points
            .iter()
            .flatten()
            .fold(0.0f64, |worst, cp| {
                worst
                    .max(cp.point.x.abs())
                    .max(cp.point.y.abs())
                    .max(cp.point.z.abs())
            })
    }

    /// Entity 128（有理Bスプライン曲面）のパラメータデータ。
    ///
    /// 並びは IGES 5.3 の定義どおり:
    /// `128, K1, K2, M1, M2, PROP1..PROP5, ノットU, ノットV, 重み, 制御点, U0,U1,V0,V1`
    fn entity_128_parameters(surface: &NurbsSurface3) -> Vec<String> {
        let rows = surface.control_points.len();
        let columns = surface.control_points[0].len();

        let closed_u = surface.control_points[0]
            .iter()
            .zip(surface.control_points[rows - 1].iter())
            .all(|(a, b)| (a.point - b.point).norm() <= 1e-9) as i32;
        let closed_v = surface
            .control_points
            .iter()
            .all(|row| (row[0].point - row[columns - 1].point).norm() <= 1e-9)
            as i32;
        let polynomial = surface
            .control_points
            .iter()
            .flatten()
            .all(|cp| (cp.weight - 1.0).abs() <= 1e-12) as i32;

        let mut fields: Vec<String> = vec!["128".to_string()];
        for value in [
            rows as i32 - 1,
            columns as i32 - 1,
            surface.degree_u as i32,
            surface.degree_v as i32,
            closed_u,
            closed_v,
            polynomial,
            0,
            0,
        ] {
            fields.push(value.to_string());
        }
        for knot in &surface.knots_u.knots {
            fields.push(real(*knot));
        }
        for knot in &surface.knots_v.knots {
            fields.push(real(*knot));
        }
        for row in &surface.control_points {
            for cp in row {
                fields.push(real(cp.weight));
            }
        }
        for row in &surface.control_points {
            for cp in row {
                fields.push(real(cp.point.x));
                fields.push(real(cp.point.y));
                fields.push(real(cp.point.z));
            }
        }
        let ((u_min, u_max), (v_min, v_max)) = surface.param_range();
        for value in [u_min, u_max, v_min, v_max] {
            fields.push(real(value));
        }

        pack_fields(&fields)
    }

    /// Entity 126（有理Bスプライン曲線）のパラメータデータ。
    ///
    /// 曲面のパラメータ空間に乗る2D境界曲線を、Z=0 の3D曲線として書く
    /// （IGES 126 は常に3D座標を持つ。XY がそのまま (u, v)、Z は常に 0）。
    /// PROP1（平面フラグ）を立て、法線 (0,0,1) を末尾に添える。
    ///
    /// 並びは `128` と同じ骨組み:
    /// `126, K, M, PROP1..PROP4, ノット, 重み, 制御点, V0, V1, [法線]`
    fn entity_126_parameters(curve: &NurbsCurve2) -> Vec<String> {
        let k = curve.control_points.len();

        let closed = (curve.control_points[0].point - curve.control_points[k - 1].point).norm()
            <= 1e-9;
        let polynomial = curve
            .control_points
            .iter()
            .all(|cp| (cp.weight - 1.0).abs() <= 1e-12);

        let mut fields: Vec<String> = vec!["126".to_string()];
        for value in [
            k as i32 - 1,
            curve.degree as i32,
            1, // PROP1: 平面（パラメータ空間の2D曲線なので常に真）
            closed as i32,
            polynomial as i32,
            0, // PROP4: 周期的ではない
        ] {
            fields.push(value.to_string());
        }
        for knot in &curve.knots.knots {
            fields.push(real(*knot));
        }
        for cp in &curve.control_points {
            fields.push(real(cp.weight));
        }
        for cp in &curve.control_points {
            fields.push(real(cp.point.x));
            fields.push(real(cp.point.y));
            fields.push(real(0.0));
        }
        let (t_min, t_max) = curve.param_range();
        fields.push(real(t_min));
        fields.push(real(t_max));
        // PROP1 = 1（平面）なので、単位法線を書く。
        fields.push(real(0.0));
        fields.push(real(0.0));
        fields.push(real(1.0));

        pack_fields(&fields)
    }

    /// Entity 126（有理Bスプライン曲線）のパラメータデータ。実稜線（3D）版。
    ///
    /// `ZENITH_IGES_CPTR_PRIMARY` 専用: 法線は添えない（平面とは限らない
    /// ので PROP1=0）。並びは2D版と同じ骨組み。
    fn entity_126_parameters_3d(curve: &zenith_geom::NurbsCurve3) -> Vec<String> {
        let k = curve.control_points.len();

        let closed = (curve.control_points[0].point - curve.control_points[k - 1].point).norm()
            <= 1e-9;
        let polynomial = curve
            .control_points
            .iter()
            .all(|cp| (cp.weight - 1.0).abs() <= 1e-12);

        let mut fields: Vec<String> = vec!["126".to_string()];
        for value in [
            k as i32 - 1,
            curve.degree as i32,
            0, // PROP1: 平面とは限らない
            closed as i32,
            polynomial as i32,
            0, // PROP4: 周期的ではない
        ] {
            fields.push(value.to_string());
        }
        for knot in &curve.knots.knots {
            fields.push(real(*knot));
        }
        for cp in &curve.control_points {
            fields.push(real(cp.weight));
        }
        for cp in &curve.control_points {
            fields.push(real(cp.point.x));
            fields.push(real(cp.point.y));
            fields.push(real(cp.point.z));
        }
        let (t_min, t_max) = curve.param_range();
        fields.push(real(t_min));
        fields.push(real(t_max));

        pack_fields(&fields)
    }
}
