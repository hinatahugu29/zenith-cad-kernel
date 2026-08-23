//! 同じ形を違う書き方で書いたファイルを、同じ答えで読めるか。
//!
//! # なぜこれが要るか
//!
//! 他カーネルの検体は、**書き方が全部同じ**でした。ミリで、解析曲面のまま、
//! 1ファイルに1立体。そこを変えたら、単位を1行も読んでいないことが出ました
//! （4-44、インチのファイルが 16387 分の1）。同じ理由で、他の軸も変えます。
//!
//! - **1ファイルに複数の立体。** 実務のファイルは1個ではありません。合計だけ
//!   見ていると、片方を落としても気づけません。だから**個数も**見ます。
//! - **解析曲面か B-spline か。** 多くの書き出しが全部 B-spline に落とします。
//!   形は同じでも、読み手が通る経路は別です。
//!
//! # 書き手は増やせなくても、書き方は増やせる
//!
//! 有償 CAD のファイルは手元にありません。FreeCAD は OpenCASCADE そのものなので、
//! **書き手の多様性は得られません**。得られるのは表現の多様性だけです。
//! それでも、上の2つは実務で最初に当たる差です。
//!
//! なお FreeCAD の headless 書き出しは、**スキーマ（AP203/214/242）と単位の
//! 設定を無視します**（実測: どちらを指定しても AUTOMOTIVE_DESIGN の
//! ミリで出る）。だからスキーマ違いはここでは測れていません。単位は
//! `tools/make_unit_step.py` が別の作り方で用意しています。
//!
//! ```bash
//! cargo run --release -p zenith_algo --example step_representation_probe
//! ```

use std::path::{Path, PathBuf};

use zenith_algo::MassCalculator;
use zenith_io::StepImporter;
use zenith_tess::TessellationParams;

fn params() -> TessellationParams {
    TessellationParams {
        u_divisions: 48,
        v_divisions: 48,
    }
}

fn fixture(name: &str) -> PathBuf {
    Path::new(concat!(
        env!("CARGO_MANIFEST_DIR"),
        "/tests/fixtures/representation"
    ))
    .join(format!("{name}.step"))
}

struct Case {
    name: &'static str,
    what: &'static str,
    /// ミリでの解析解。
    expected_volume: f64,
    /// ファイルが持っている立体の数。
    expected_solids: usize,
    /// 許容。**すべて解析解に対して**見ます（他カーネルの測定値ではなく）。
    tolerance: f64,
}

fn main() {
    let drilled = 13500.0 - std::f64::consts::PI * 25.0 * 15.0;

    let cases = [
        Case {
            name: "block_default",
            what: "1 solid, analytic",
            expected_volume: 24000.0,
            expected_solids: 1,
            tolerance: 1e-9,
        },
        Case {
            name: "two_solids",
            what: "2 solids in one file",
            // 10x10x10 と 20x5x5。**合計だけでは片方を落としても気づけない**
            // ので、個数も見る。
            expected_volume: 1000.0 + 500.0,
            expected_solids: 2,
            tolerance: 1e-9,
        },
        Case {
            name: "drilled_analytic",
            what: "cylinder + planes",
            expected_volume: drilled,
            expected_solids: 1,
            tolerance: 1e-9,
        },
        Case {
            name: "drilled_bspline",
            // 穴は degree 2・重み (1, 0.5, 1, 0.5, ...) の**厳密な有理円柱**です
            // （120度ごとの円弧の標準形）。B-spline 化しても形は近似されて
            // いないので、真値は解析解のままです。
            //
            // **ここで一度、期待値の置き方を間違えました。** OpenCASCADE が
            // このファイルを読んで測った 12312.350278 を「正解」にしたところ、
            // こちらの 12321.902755 が WRONG と出ました。ファイルの中身
            // （重み）を見て分かったのは、**外れているのは OCC の求積のほう**
            // だということです（解析解に対して 0.078% 低い）。
            //
            // 外の物差しは万能ではありません。有理 B-spline の求積では、
            // OCC の数字は解析解より緩みます。**落ちたらまず自分の期待値を
            // 疑う**（5章）。
            what: "the same, all B-spline",
            expected_volume: drilled,
            expected_solids: 1,
            tolerance: 1e-9,
        },
    ];

    println!(
        "{:<20} {:<24} {:>7} {:>16} {:>16} {:>10}  {}",
        "subject", "how it is written", "solids", "volume read", "want", "rel", "verdict"
    );
    println!("{}", "-".repeat(108));

    let mut ok = 0usize;
    let mut wrong = 0usize;
    let mut missing = 0usize;

    for case in &cases {
        let path = fixture(case.name);
        if !path.exists() {
            println!("{:<20} {:<24} file not found", case.name, case.what);
            missing += 1;
            continue;
        }

        let solids = match StepImporter::import_solids_from_file(&path) {
            Ok(solids) => solids,
            Err(err) => {
                println!(
                    "{:<20} {:<24} refused: {}",
                    case.name,
                    case.what,
                    err.chars().take(40).collect::<String>()
                );
                wrong += 1;
                continue;
            }
        };

        let volume: f64 = solids
            .iter()
            .map(|solid| MassCalculator::compute_from_brep(solid, &params()).volume)
            .sum();
        let relative = (volume - case.expected_volume).abs() / case.expected_volume;

        let mut notes: Vec<String> = Vec::new();
        if solids.len() != case.expected_solids {
            notes.push(format!(
                "WRONG COUNT: {} solid(s), want {}",
                solids.len(),
                case.expected_solids
            ));
        }
        if relative > case.tolerance {
            notes.push("WRONG VOLUME".to_string());
        }

        let verdict = if notes.is_empty() {
            ok += 1;
            "ok".to_string()
        } else {
            wrong += 1;
            notes.join("; ")
        };

        println!(
            "{:<20} {:<24} {:>7} {volume:>16.6} {:>16.6} {relative:>10.2e}  {verdict}",
            case.name,
            case.what,
            solids.len(),
            case.expected_volume
        );
    }

    println!("{}", "-".repeat(108));
    println!("ok {ok}   WRONG {wrong}   missing {missing}");
    println!();
    println!("The count matters as much as the total. A file holding two solids");
    println!("read as one, or as one of the two, still returns a plausible number.");
    println!();
    println!("Every expected volume here is a closed form, not another kernel's");
    println!("measurement. OpenCASCADE measures its own drilled_bspline file at");
    println!("12312.350278 - 0.078% below the analytic value - because integrating");
    println!("over a rational B-spline is where its quadrature loosens. Taking that");
    println!("number as the truth is what made this probe report a defect that was");
    println!("not there.");

    if wrong > 0 {
        std::process::exit(1);
    }
}
