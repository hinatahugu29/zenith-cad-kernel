//! **面積は、分割を細かくすれば OCC に寄っていくのか**（4-572）。
//!
//! # なぜ要るのか
//!
//! 4-560 は「読んだ `linkrods.step` の円柱面（面積 7.048879628）が、OCC より
//! 8.71e-5 大きい」と測りましたが、**それが求積の粗さ（分割数）のせいなのか、
//! それとも面そのものが違うのか**を区別していませんでした。
//!
//! **区別する方法は 1 つ**——**分割を上げて、差が縮むかを見る**ことです。
//! 縮んで 0 に近づけば求積の問題、**縮まずに張り付けば、面そのものが
//! （ほんの少し）違う**ということになります。
//!
//! # 実測（2026/09/28）
//!
//! ```text
//! 面の種類: NURBS 次数2x1 制御点9x2
//!   分割 8    面積 7.048967260  OCC との差 +8.763181e-5
//!   分割 16   面積 7.048966711  OCC との差 +8.708307e-5
//!   分割 24   面積 7.048966718  OCC との差 +8.709010e-5
//!   分割 256  面積 7.048966750  OCC との差 +8.712180e-5
//! ```
//!
//! **分割 8 から 256 まで、差は 9 桁のうち 5 桁目までびくともしません。**
//! **求積はとっくに収束しています。** 差は面そのものにあります。
//!
//! # そして、これは近似のせいではありません
//!
//! この面は**制御点 9×2**（有理2次×1次）——`step_import.rs` の
//! `rational_elliptic_arc` が組む形です。**この構成は数学的に厳密な円弧**
//! です（重み `cos(半区間角 / 2)` を持つ有理2次ベジエは、教科書どおり円弧を
//! 厳密に表します）。**4-560 が「近似です」と書いたのは誤りでした**——
//! **コードを読まずに、面積の差だけから逆算した推測**だったからです。
//!
//! **では、差はどこから来るか。** 半径を STEP の申告どおり 0.75 として、
//! 円柱の高さを逆算すると——
//!
//! ```text
//! h_occ  = 1.4958187147412805
//! h_ours = 1.4958372026038833
//! diff   = 1.85e-5（相対 1.2e-5）
//! ```
//!
//! **高さ（＝軸方向の trim の範囲）が、ほんのわずかに違います。** これは
//! 面の形の近似ではなく、**トリム境界を読む段の、桁の端での小さな不一致**
//! だと考えられます——**両方のカーネルが、別々に曲線を評価しているので、
//! 端点の位置がその桁で完全には揃いません。**
//!
//! **この桁（相対 1e-5）は、OCC 自身の恒等式が閉じる桁（3.8e-5〜6.9e-5。
//! 4-45）より小さいか同じ**です。**これ以上追う値打ちは薄いと判断し、
//! ここで止めています。**

use zenith_algo::MassCalculator;
use zenith_io::StepImporter;
use zenith_tess::TessellationParams;

/// OCC が報告する、この面の面積（`tools/occ_h8_reference.py` の
/// `ZENITH_OCC_FACES=A`。4-560、4-572）。
const OCC_AREA: f64 = 7.048879628;

fn main() {
    let path = "reference/OCCT/data/step/linkrods.step";
    let solids = StepImporter::import_solids_from_file(path).expect("読めません");
    let solid = solids
        .into_iter()
        .max_by_key(|s| s.outer_shell.faces.len())
        .expect("立体が 0 個");

    // OCC の面積に最も近い面を探す（4-560 の突き合わせと同じ、面積で照合）。
    let mut best: Option<(f64, &zenith_topo::Face)> = None;
    for face in &solid.outer_shell.faces {
        let (area, _) =
            MassCalculator::compute_face_integral(face, &TessellationParams::default());
        if (area - OCC_AREA).abs() < 0.01 {
            best = Some((area, face));
        }
    }
    let (_, face) = best.expect("OCC の面積に近い面が見つかりません");

    println!(
        "面の種類: {}",
        match &face.geometry {
            zenith_topo::FaceGeometry::Nurbs(s) => format!(
                "NURBS 次数{}x{} 制御点{}x{}",
                s.degree_u,
                s.degree_v,
                s.control_points.len(),
                s.control_points[0].len()
            ),
            _ => "NURBS ではありません".to_string(),
        }
    );

    for divs in [8usize, 16, 24, 32, 48, 64, 96, 128, 256] {
        let params = TessellationParams {
            u_divisions: divs,
            v_divisions: divs,
        };
        let (area, volume) = MassCalculator::compute_face_integral(face, &params);
        println!(
            "  分割 {divs:<4} 面積 {area:.9}  OCC との差 {:+.6e}  寄与 {volume:.9}",
            area - OCC_AREA
        );
    }
}
