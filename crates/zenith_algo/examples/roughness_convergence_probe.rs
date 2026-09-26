//! **申告した粗さは、標本を増やすと動くのか**（4-553）。
//!
//! # なぜ要るか
//!
//! 取り込みは面の粗さを**稜あたり 8 点**で測って `face.tolerance` に
//! 書きます（4-266）。検証も**稜あたり 8 点**で見ます——**同じ数**です。
//! ところが検証が見るのは**割った片の境界**で、**親の稜は小片に割れて
//! います**。**小片 1 本につき 8 点なら、同じ曲線をより細かく見る**ことに
//! なり、**親が見なかった点で、親より悪い値が出ます。**
//!
//! **それなら、標本を増やすだけで申告値は上がるはず**です。**測ります。**
//!
//! 使い方: `roughness_convergence_probe [検体パス]`
use zenith_io::StepImporter;
use zenith_math::Tolerance;

fn main() {
    let path = std::env::args()
        .nth(1)
        .unwrap_or_else(|| "reference/OCCT/data/step/linkrods.step".to_string());
    let Ok(solids) = StepImporter::import_solids_from_file(&path) else {
        println!("{path} が読めません");
        return;
    };
    let Some(read) = solids.into_iter().max_by_key(|s| s.outer_shell.faces.len()) else {
        return;
    };
    let tol = Tolerance::default();
    let counts = [8usize, 16, 32, 64, 128];

    println!("{path}: 面 {} 枚", read.outer_shell.faces.len());
    println!();
    println!("面   申告        8 点        16 点       32 点       64 点       128 点      128/8");
    let mut worst_ratio = 0.0f64;
    let mut worst_face = usize::MAX;
    for (index, face) in read.outer_shell.faces.iter().enumerate() {
        let measured: Vec<f64> = counts
            .iter()
            .map(|n| face.validate_boundary_on_surface(&tol, *n).max_distance)
            .collect();
        let ratio = if measured[0] > 0.0 {
            measured[measured.len() - 1] / measured[0]
        } else {
            1.0
        };
        if ratio > worst_ratio {
            worst_ratio = ratio;
            worst_face = index;
        }
        // **動く面だけ出します。** 全部出すと、動かない面に埋もれます。
        if ratio > 1.001 {
            println!(
                "{index:<4} {:.4e}  {:.4e}  {:.4e}  {:.4e}  {:.4e}  {:.4e}  {ratio:.3}",
                face.tolerance,
                measured[0],
                measured[1],
                measured[2],
                measured[3],
                measured[4],
            );
        }
    }
    println!();
    println!("いちばん動く面: 面{worst_face}（128 点 / 8 点 = {worst_ratio:.3}）");

    // **p-curve の粗さも、同じ問いにかけます**（4-556）。
    //
    // 取り込みは `validate_pcurves(tol, 37)` で測ります。**検証も 37 点**
    // ですが、**検証が見るのは割った片**で、**親の稜は小片に割れて
    // います**——4-553 と同じ理屈なら、**ここも足りていないはず**です。
    println!();
    println!("p-curve の粗さ（面が申告した値と、標本を増やしたとき）:");
    println!("面   申告        37 点       74 点       148 点      296 点      296/37");
    for (index, face) in read.outer_shell.faces.iter().enumerate() {
        let at = |n: usize| {
            face.validate_pcurves(&tol, n)
                .map(|report| report.max_distance)
                .unwrap_or(f64::NAN)
        };
        let (a, b, c, d) = (at(37), at(74), at(148), at(296));
        let ratio = if a > 0.0 { d / a } else { 1.0 };
        if ratio > 1.001 {
            println!(
                "{index:<4} {:.4e}  {a:.4e}  {b:.4e}  {c:.4e}  {d:.4e}  {ratio:.3}",
                face.pcurve_tolerance
            );
        }
    }
    println!();
    println!("**8 点で測った値を申告しているのに、検証は割った片を 8 点ずつ見ます**");
    println!("**——同じ曲線を、より細かく。ここが 1 を超えるなら、申告値は足りません。**");
}
