//! 同じ形を違う書き方で書いたファイルを、同じ答えで読むこと。
//!
//! 検体10本は書き方が全部同じ（ミリ・解析曲面・1立体）でした。軸を変えたら
//! 単位の欠陥が出ています（4-44）。残る軸をここで押さえます。
//!
//! **期待値はすべて閉じた式です。** 他カーネルの測定値ではありません。
//! OpenCASCADE は `drilled_bspline` を自分で書いておきながら 12312.350278 と
//! 測ります（解析解に対して 0.078% 低い）。有理 B-spline の求積は OCC の
//! 弱いところで、そこを正解に据えると**無い欠陥を報告します**（実際にやりました）。

use std::path::PathBuf;

use zenith_algo::MassCalculator;
use zenith_io::StepImporter;
use zenith_tess::TessellationParams;

fn fixture(name: &str) -> PathBuf {
    PathBuf::from(concat!(
        env!("CARGO_MANIFEST_DIR"),
        "/tests/fixtures/representation"
    ))
    .join(format!("{name}.step"))
}

fn read(name: &str) -> Vec<zenith_topo::Solid> {
    StepImporter::import_solids_from_file(fixture(name))
        .unwrap_or_else(|err| panic!("{name} could not be read: {err}"))
}

fn total_volume(solids: &[zenith_topo::Solid]) -> f64 {
    solids
        .iter()
        .map(|solid| {
            MassCalculator::compute_from_brep(
                solid,
                &TessellationParams {
                    u_divisions: 48,
                    v_divisions: 48,
                },
            )
            .volume
        })
        .sum()
}

fn assert_volume(name: &str, solids: &[zenith_topo::Solid], expected: f64) {
    let volume = total_volume(solids);
    let relative = (volume - expected).abs() / expected;
    assert!(
        relative <= 1e-9,
        "{name}: read {volume} mm3, expected the analytic {expected} mm3 (relative {relative:.3e})"
    );
}

/// **個数は合計と同じくらい大事です。** 2立体のファイルを1立体として読んでも、
/// 片方だけ読んでも、返るのは「もっともらしい数」です。
#[test]
fn reads_both_solids_from_one_file() {
    let solids = read("two_solids");
    assert_eq!(
        solids.len(),
        2,
        "a file holding two solids came back as {} solid(s)",
        solids.len()
    );
    // 10x10x10 と 20x5x5。
    assert_volume("two_solids", &solids, 1500.0);
}

/// 解析曲面で書かれた穴あき箱。
#[test]
fn reads_an_analytic_drilled_block() {
    let solids = read("drilled_analytic");
    assert_eq!(solids.len(), 1);
    assert_volume(
        "drilled_analytic",
        &solids,
        13500.0 - std::f64::consts::PI * 25.0 * 15.0,
    );
}

/// 同じ形を全部 B-spline で書いたもの。穴は重み `(1, 0.5, 1, 0.5, ...)` の
/// **厳密な有理円柱**なので、答えは解析解のままでなければなりません。
#[test]
fn a_rational_bspline_hole_is_still_exact() {
    let solids = read("drilled_bspline");
    assert_eq!(solids.len(), 1);
    assert_volume(
        "drilled_bspline",
        &solids,
        13500.0 - std::f64::consts::PI * 25.0 * 15.0,
    );
}

/// 解析曲面で書いた版と B-spline で書いた版が、**同じ答え**になること。
/// どちらかだけ正しくても意味がありません。
#[test]
fn the_two_writings_of_the_same_block_agree() {
    let analytic = total_volume(&read("drilled_analytic"));
    let bspline = total_volume(&read("drilled_bspline"));
    let relative = (analytic - bspline).abs() / analytic;
    assert!(
        relative <= 1e-9,
        "the same block written two ways disagreed: {analytic} vs {bspline} (relative {relative:.3e})"
    );
}
