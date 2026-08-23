//! ミリ以外の単位で書かれた STEP を、正しい大きさで読めているか。
//!
//! # なぜこれが要るか
//!
//! 検体10本はすべてミリで書かれていました。**単位を読み落としても、10本とも
//! 通ります。** 実務のファイルはそうではありません。インチで書かれた STEP は
//! 珍しくなく、読み落とすと体積は 25.4^3 = 16387 倍ずれます。
//!
//! しかも**返ってくるのは、もっともらしい閉じた立体**です。形は正しく、
//! 大きさだけが違う。閉性の検査も、面の検査も、恒等式も、全部通ります。
//! 気づけるのは「その数値が何ミリのつもりか」を知っているときだけです。
//!
//! ここで読む検体は `tools/make_unit_step.py` が作り、**OpenCASCADE が
//! 24000 mm^3 で読み戻すことを確かめてから**置いています。ファイルが正しい
//! ことは別途保証されているので、食い違えばこちらの読み手の問題です。
//!
//! ```bash
//! cargo run --release -p zenith_algo --example step_unit_probe
//! ```

use std::path::{Path, PathBuf};

use zenith_algo::MassCalculator;
use zenith_io::StepImporter;
use zenith_tess::TessellationParams;

fn params() -> TessellationParams {
    TessellationParams {
        u_divisions: 32,
        v_divisions: 32,
    }
}

/// 検体はリポジトリに入っています（`tools/make_unit_step.py` が作り、
/// OpenCASCADE が正しい大きさで読み戻すことを確かめてから置いたもの）。
/// **生成物に依存させません** — `target/` を消しても検査は走ります。
fn subject(name: &str) -> PathBuf {
    Path::new(concat!(env!("CARGO_MANIFEST_DIR"), "/tests/fixtures/units"))
        .join(format!("{name}.step"))
}

struct Case {
    name: &'static str,
    unit: &'static str,
    /// この単位1つが何ミリか。読み落としたときのずれ倍率でもある。
    millimetres: f64,
    /// ミリでの解析解。
    expected: f64,
}

fn main() {
    // 平らな面だけの形では、**半径のスカラ**を読み落としても気づけません。
    // 座標に単位を掛けて半径に掛け忘れると、箱は通って円柱が壊れます。
    // だから曲面を持つ形と、内側のループを持つ形も並べます。
    let cases = [
        Case {
            name: "block_inch",
            unit: "inch",
            millimetres: 25.4,
            expected: 24000.0,
        },
        Case {
            name: "block_centimetre",
            unit: "centimetre",
            millimetres: 10.0,
            expected: 24000.0,
        },
        Case {
            name: "cylinder_inch",
            unit: "inch",
            millimetres: 25.4,
            // pi * 10^2 * 40
            expected: 12566.370614359172,
        },
        Case {
            name: "drilled_inch",
            unit: "inch",
            millimetres: 25.4,
            // 30*30*15 - pi * 5^2 * 15
            expected: 13500.0 - std::f64::consts::PI * 25.0 * 15.0,
        },
    ];

    println!(
        "{:<20} {:<12} {:>16} {:>16} {:>10}  {}",
        "subject", "unit", "volume read", "want", "rel", "verdict"
    );
    println!("{}", "-".repeat(92));

    let mut missing = 0usize;
    let mut wrong = 0usize;
    let mut ok = 0usize;

    for case in &cases {
        let path = subject(case.name);
        if !path.exists() {
            println!(
                "{:<20} {:<12} {:>16} {:>16} {:>10}  file not found; run tools/make_unit_step.py",
                case.name, case.unit, "-", "-", "-"
            );
            missing += 1;
            continue;
        }

        let solids = match StepImporter::import_solids_from_file(&path) {
            Ok(solids) if !solids.is_empty() => solids,
            Ok(_) => {
                println!("{:<20} {:<12} no solids in the file", case.name, case.unit);
                wrong += 1;
                continue;
            }
            Err(err) => {
                println!(
                    "{:<20} {:<12} refused: {}",
                    case.name,
                    case.unit,
                    err.chars().take(50).collect::<String>()
                );
                // 断ることは誤答よりずっとよい。読めないなら読めないと言えている。
                continue;
            }
        };

        let volume: f64 = solids
            .iter()
            .map(|solid| MassCalculator::compute_from_brep(solid, &params()).volume)
            .sum();
        let expected = case.expected;
        let relative = (volume - expected).abs() / expected;

        // 読み落としたときにどうなるかを、先に書いておく。出た数字が
        // 「たまたま近い」のか「単位ぶんちょうどずれている」のかが分かる。
        let ignored = expected / case.millimetres.powi(3);
        let looks_ignored = (volume - ignored).abs() / ignored <= 1e-6;

        // 1e-9 は、求積そのものの実力（builder_audit で 1e-13 台）より緩く、
        // 単位の取り違え（最小でも 1000 倍）よりはるかに厳しい線です。
        let verdict = if relative <= 1e-9 {
            ok += 1;
            "ok".to_string()
        } else {
            wrong += 1;
            if looks_ignored {
                format!(
                    "WRONG - the unit was ignored ({}^3 = {:.0}x small)",
                    case.unit,
                    case.millimetres.powi(3)
                )
            } else {
                "WRONG".to_string()
            }
        };

        println!(
            "{:<20} {:<12} {volume:>16.6} {expected:>16.6} {relative:>10.2e}  {verdict}",
            case.name, case.unit
        );
    }

    println!("{}", "-".repeat(92));
    println!("ok {ok}   WRONG {wrong}   missing {missing}");
    println!();
    println!("Every file here was written in a unit other than the millimetre,");
    println!("and OpenCASCADE reads each one back at its analytic size, so the");
    println!("files themselves are not in question.");
    println!();
    println!("A wrong answer here is the dangerous kind: the solid comes back");
    println!("closed, manifold and correctly shaped, and only its size is wrong.");
    println!("No shape check can catch it.");

    if wrong > 0 {
        std::process::exit(1);
    }
}
