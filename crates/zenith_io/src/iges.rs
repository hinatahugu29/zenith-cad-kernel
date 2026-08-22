//! Zenith IO: IGES 5.3 エクスポーター
//!
//! 各 Face の支持曲面を Entity 128（有理Bスプライン曲面）として書き出す。
//!
//! **トリムは出力しない。** 出る形は「面の土台になっている曲面の集まり」で
//! あって、B-Rep ソリッドではない。境界で切り取る Entity 144 / 142 / 126 は
//! まだ書いていない。立体としてやり取りするなら STEP を使うこと。
//!
//! ただし実測では、自前ビルダーの立体についてはこれで足りている。曲面が
//! 元から面ごとのパッチに割れており、パラメータ矩形の縁がそのまま面の境界に
//! なっているからである。`tools/verify_iges.py` で OpenCASCADE に読ませると、
//! 5検体すべてで**曲面の枚数が一致し、境界箱のはみ出しも欠けも 0**（他カーネル
//! から読んだ全周1枚の面のように、パラメータ域が面より広い曲面を持つ立体では
//! はみ出す。そこはトリムを書くまでの制限として残る）。
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
use zenith_geom::{ControlPoint3, KnotVector, NurbsSurface3, Surface3};
use zenith_math::Point3;
use zenith_topo::{Face, FaceGeometry, Solid};

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
        let mut surfaces = Vec::new();
        for shell in std::iter::once(&solid.outer_shell).chain(solid.inner_shells.iter()) {
            for face in &shell.faces {
                surfaces.push(Self::surface_of(face)?);
            }
        }
        if surfaces.is_empty() {
            return Err("IGES export needs at least one face".to_string());
        }

        let extent = surfaces
            .iter()
            .flat_map(|surface| surface.control_points.iter().flatten())
            .fold(0.0f64, |worst, cp| {
                worst
                    .max(cp.point.x.abs())
                    .max(cp.point.y.abs())
                    .max(cp.point.z.abs())
            });

        // Directory Entry は1エンティティにつき2行、Parameter Data は可変行数。
        // DE から PD の先頭行を指すので、PD の中身を先に組んで行数を数える。
        let parameter_bodies: Vec<Vec<String>> = surfaces
            .iter()
            .map(Self::entity_128_parameters)
            .collect();

        let mut directory = String::new();
        let mut parameters = String::new();
        let mut directory_sequence = 1usize;
        let mut parameter_sequence = 1usize;

        for (index, body) in parameter_bodies.iter().enumerate() {
            let directory_line = directory_sequence;
            let parameter_line = parameter_sequence;

            // 1行目: 型番, PDポインタ, 構造, 線種, レベル, ビュー, 変換行列,
            //         ラベル表示, ステータス
            directory.push_str(&record(
                &format!(
                    "{:>8}{:>8}{:>8}{:>8}{:>8}{:>8}{:>8}{:>8}{:>8}",
                    128, parameter_line, 0, 0, 0, 0, 0, 0, "00000000"
                ),
                'D',
                directory_sequence,
            ));
            directory_sequence += 1;

            // 2行目: 型番, 線幅, 色, PD行数, フォーム番号, 予約, 予約,
            //         ラベル, 添字
            directory.push_str(&record(
                &format!(
                    "{:>8}{:>8}{:>8}{:>8}{:>8}{:>8}{:>8}{:>8}{:>8}",
                    128,
                    0,
                    0,
                    body.len(),
                    0,
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
            String::new(),                                    // 1  パラメータ区切り（既定 ,）
            String::new(),                                    // 2  レコード区切り（既定 ;）
            hollerith(product_name),                          // 3  送り手の製品ID
            hollerith(&file_name),                            // 4  ファイル名
            hollerith("Zenith CAD Kernel"),                   // 5  ネイティブシステムID
            hollerith("Zenith CAD Kernel 0.1.0"),             // 6  プリプロセッサ版
            "32".to_string(),                                 // 7  整数のビット数
            "308".to_string(),                                // 8  単精度の指数上限
            "15".to_string(),                                 // 9  単精度の有効桁
            "308".to_string(),                                // 10 倍精度の指数上限
            "15".to_string(),                                 // 11 倍精度の有効桁
            String::new(),                                    // 12 受け手の製品ID
            "1.".to_string(),                                 // 13 モデル空間の倍率
            "2".to_string(),                                  // 14 単位フラグ（2 = ミリメートル）
            "2HMM".to_string(),                               // 15 単位名
            "1".to_string(),                                  // 16 線幅の段階数
            "0.01".to_string(),                               // 17 最大線幅
            hollerith("20260822.120000"),                     // 18 ファイル生成日時
            "1E-07".to_string(),                              // 19 最小分解能
            real(extent.max(1.0)),                            // 20 座標の最大値
            String::new(),                                    // 21 作成者
            String::new(),                                    // 22 組織
            "11".to_string(),                                 // 23 仕様の版（11 = IGES 5.3）
            "0".to_string(),                                  // 24 製図規格
            hollerith("20260822.120000"),                     // 25 モデル最終更新日時
            String::new(),                                    // 26 プロトコル識別子
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

    /// Face の支持曲面を NURBS として取り出す。
    ///
    /// 平面は制御点の格子を持たないので、その面の境界がちょうど収まる矩形を
    /// 張って返す。トリムを書いていない以上、ここを無限平面にすると受け側で
    /// 際限なく広がってしまう。
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
                        consider(oriented.edge.start_vertex.point);
                        consider(oriented.edge.end_vertex.point);
                    }
                }
                if !(max_u > min_u && max_v > min_v) {
                    return Err("planar face has no measurable extent".to_string());
                }

                let corner = |u: f64, v: f64| {
                    ControlPoint3::unweighted(plane.origin + plane.u_axis * u + plane.v_axis * v)
                };
                NurbsSurface3::new(
                    1,
                    1,
                    vec![
                        vec![corner(min_u, min_v), corner(min_u, max_v)],
                        vec![corner(max_u, min_v), corner(max_u, max_v)],
                    ],
                    KnotVector::clamped_uniform(2, 1),
                    KnotVector::clamped_uniform(2, 1),
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

        // 64桁に収まるように詰める。最後のフィールドだけ ';' で閉じる。
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
}
