//! ミリ以外の単位で書かれた STEP を、正しい大きさで読むこと。
//!
//! この検査が無いあいだ、インポーターは**単位を一行も読んでいませんでした**。
//! インチのファイルは体積 1.46 mm^3（解析解 24000、25.4^3 = 16387 倍小さい）、
//! センチのファイルは 24.0 mm^3 で返っていました。**どちらも閉じた多様体で、
//! 形も正しい**ので、閉性の検査も面の検査も恒等式も全部通ります。
//!
//! 検体は `tools/make_unit_step.py` が作り、OpenCASCADE が解析解どおりの
//! 大きさで読み戻すことを確かめてから置いてあります。ファイルの正しさは
//! 別に担保されているので、ここで食い違えば読み手の問題です。

use std::path::PathBuf;

use zenith_algo::MassCalculator;
use zenith_io::StepImporter;
use zenith_tess::TessellationParams;

fn fixture(name: &str) -> PathBuf {
    PathBuf::from(concat!(
        env!("CARGO_MANIFEST_DIR"),
        "/tests/fixtures/units"
    ))
    .join(format!("{name}.step"))
}

fn volume_of(name: &str) -> f64 {
    let solids = StepImporter::import_solids_from_file(fixture(name))
        .unwrap_or_else(|err| panic!("{name} could not be read: {err}"));
    assert!(!solids.is_empty(), "{name} produced no solids");
    solids
        .iter()
        .map(|solid| {
            MassCalculator::compute_from_brep(
                solid,
                &TessellationParams {
                    u_divisions: 32,
                    v_divisions: 32,
                },
            )
            .volume
        })
        .sum()
}

/// 単位を読み落としたときのずれは、最小でも 1000 倍です。1e-9 はそれより
/// はるかに厳しく、求積そのものの実力（`builder_audit` で 1e-13 台）よりは
/// 緩い線です。
fn assert_size(name: &str, expected: f64, unit_millimetres: f64) {
    let volume = volume_of(name);
    let relative = (volume - expected).abs() / expected;
    let ignored = expected / unit_millimetres.powi(3);
    assert!(
        relative <= 1e-9,
        "{name}: read {volume} mm3, expected {expected} mm3 (relative {relative:.3e}). \
         Ignoring the unit would give {ignored} mm3."
    );
}

#[test]
fn reads_an_inch_file_at_the_right_size() {
    assert_size("block_inch", 24000.0, 25.4);
}

#[test]
fn reads_a_centimetre_file_at_the_right_size() {
    assert_size("block_centimetre", 24000.0, 10.0);
}

/// 平らな面だけでは足りません。**半径はスカラで、座標とは別の場所で読まれます。**
/// 座標に単位を掛けて半径に掛け忘れると、箱は通って円柱だけが壊れます。
#[test]
fn scales_the_radius_of_a_curved_surface_too() {
    assert_size(
        "cylinder_inch",
        std::f64::consts::PI * 100.0 * 40.0,
        25.4,
    );
}

/// 内側のループ（穴）を持つ面も、同じ倍率で動かなければ穴の大きさが変わります。
#[test]
fn scales_a_solid_with_an_inner_loop() {
    assert_size(
        "drilled_inch",
        13500.0 - std::f64::consts::PI * 25.0 * 15.0,
        25.4,
    );
}

/// ミリのファイルが動かないこと。単位を掛ける口を足したので、既定の経路に
/// 1 以外が紛れ込んでいないかを、既存の検体で押さえます。
#[test]
fn a_millimetre_file_is_left_alone() {
    let path = PathBuf::from(concat!(
        env!("CARGO_MANIFEST_DIR"),
        "/tests/fixtures/occ_reference_cylinder.step"
    ));
    let solids = StepImporter::import_solids_from_file(&path).expect("read the millimetre cylinder");
    let volume: f64 = solids
        .iter()
        .map(|solid| {
            MassCalculator::compute_from_brep(
                solid,
                &TessellationParams {
                    u_divisions: 32,
                    v_divisions: 32,
                },
            )
            .volume
        })
        .sum();
    let expected = std::f64::consts::PI * 100.0 * 40.0;
    let relative = (volume - expected).abs() / expected;
    assert!(
        relative <= 1e-9,
        "a millimetre file moved: read {volume}, expected {expected} (relative {relative:.3e})"
    );
}
