//! **境界箱の外の点を「中」と言わないこと**（4-507）。
//!
//! `exact_inside` は**最近点の外向き法線**で内外を決めます。
//! **頂点では法線が定まりません**——実測（4-506。読んだ `cone_full`、面 2 枚）:
//!
//! ```text
//! 点 (4,4,23)、最近 6.403、同着 1 枚
//!   足 (0,0,20)＝頂点、法線の長さ 1.000、外向き −2.827  → 「中」
//! ```
//!
//! **天辺は z=20** です。**3 も上の点を「中」**と答えていました。
//!
//! **境界箱で先に落とします。** `Face::bounding_box` は**輪と、トリムする
//! 前の制御点の両方**を包むので、**立体を必ず含みます**——**その外なら、
//! 確実に外**です。**推し量りではありません。**
//!
//! **頂点の問題そのものは残ります**（箱の中で頂点が最近点になる点）。
//! **安く確実に取れる分だけを取った、という試験**です。

use std::path::PathBuf;

use zenith_algo::boolean_validation::exact_inside;
use zenith_io::StepImporter;
use zenith_math::{Point3, Tolerance};

#[test]
fn a_point_above_a_cone_apex_is_outside() {
    let tol = Tolerance::default();
    // **読んだ立体で見ます。** 自作の円錐は頂点が 4 枚に分かれるので、
    // **最近点が頂点になっても、他の 3 枚が「外」と言えます**——
    // **この欠陥は出ません**（書いた最初の試験がそれで、変更の有無に
    // 関わらず通りました）。**読んだ `cone_full` は側面 1 枚**です。
    let path = PathBuf::from(concat!(env!("CARGO_MANIFEST_DIR"), "/tests/fixtures"))
        .join("occ_reference_cone_full.step");
    let cone = StepImporter::import_solids_from_file(&path)
        .expect("読めるはずの検体")
        .into_iter()
        .next()
        .expect("立体が 1 つ");
    assert_eq!(cone.outer_shell.faces.len(), 2, "側面 1 枚と底 1 枚のはずです");

    // **頂点の真上と、斜め上。** どちらも外です。
    for point in [
        Point3::new(0.0, 0.0, 23.0),
        Point3::new(4.0, 4.0, 23.0),
        Point3::new(0.0, 0.0, 20.5),
    ] {
        assert_eq!(
            exact_inside(point, &cone, &tol),
            Some(false),
            "{point:?} は円錐の外です"
        );
    }

    // **中の点は、中のまま。** 門を足して内外が入れ替わっていないこと。
    for point in [Point3::new(0.0, 0.0, 10.0), Point3::new(4.0, 0.0, 1.0)] {
        assert_eq!(
            exact_inside(point, &cone, &tol),
            Some(true),
            "{point:?} は円錐の中です"
        );
    }
}
