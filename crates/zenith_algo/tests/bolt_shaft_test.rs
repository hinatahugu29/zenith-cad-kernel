use std::f64::consts::PI;
use zenith_algo::{BoltBuilder, MassCalculator, ShaftBuilder};
use zenith_math::Tolerance;
use zenith_tess::TessellationParams;

#[test]
fn test_make_hex_bolt_matches_analytic_volume() {
    let tol = Tolerance::default();
    let across_flats = 16.0;   // 二面幅 S = 16mm (M10ボルト頭相当)
    let head_thickness = 6.4;  // 頭部厚み k = 6.4mm
    let shank_radius = 5.0;    // 軸部半径 r = 5.0mm (直径 10mm)
    let shank_length = 30.0;   // 軸部長 L = 30mm

    let bolt = BoltBuilder::make_hex_bolt(across_flats, head_thickness, shank_radius, shank_length)
        .expect("make_hex_bolt");

    assert!(bolt.is_topologically_valid(&tol), "bolt must be valid closed solid");

    let params = TessellationParams::default();
    let mass = MassCalculator::compute_from_brep(&bolt, &params);

    // 解析体積: 六角柱頭部体積 + 軸部円柱体積
    let hex_area = (3.0f64.sqrt() / 2.0) * across_flats * across_flats;
    let head_vol = hex_area * head_thickness;
    let shank_vol = PI * shank_radius * shank_radius * shank_length;
    let expected_vol = head_vol + shank_vol;

    let diff = (mass.volume - expected_vol).abs();
    assert!(
        diff / expected_vol < 1e-4,
        "bolt volume {} vs expected {}, diff {}",
        mass.volume,
        expected_vol,
        diff
    );
}

#[test]
fn test_make_stepped_shaft_matches_analytic_volume() {
    let tol = Tolerance::default();
    // 3段シャフト: 段1 (r=10, L=20), 段2 (r=15, L=30), 段3 (r=8, L=25)
    let sections = [(10.0, 20.0), (15.0, 30.0), (8.0, 25.0)];

    let shaft = ShaftBuilder::make_stepped_shaft(&sections)
        .expect("make_stepped_shaft");

    assert!(shaft.is_topologically_valid(&tol), "stepped shaft must be valid closed solid");

    let params = TessellationParams::default();
    let mass = MassCalculator::compute_from_brep(&shaft, &params);

    // 解析体積: 各段の円柱体積の総和
    let mut expected_vol = 0.0;
    for &(r, l) in &sections {
        expected_vol += PI * r * r * l;
    }

    let diff = (mass.volume - expected_vol).abs();
    assert!(
        diff / expected_vol < 1e-4,
        "stepped shaft volume {} vs expected {}, diff {}",
        mass.volume,
        expected_vol,
        diff
    );
}

#[test]
fn test_make_shaft_with_keyway_matches_analytic_volume() {
    let tol = Tolerance::default();
    let radius = 12.0;
    let length = 50.0;

    let base_shaft = ShaftBuilder::make_stepped_shaft(&[(radius, length)])
        .expect("base shaft");

    let key_width = 6.0;
    let key_depth = 3.5;
    let key_length = 20.0;
    let key_z_pos = 15.0;

    let shaft_with_key = ShaftBuilder::make_shaft_with_keyway(
        &base_shaft,
        radius,
        key_width,
        key_depth,
        key_length,
        key_z_pos,
    )
    .expect("make_shaft_with_keyway");

    assert!(shaft_with_key.is_topologically_valid(&tol), "shaft with keyway must be valid closed solid");

    let params = TessellationParams::default();
    let mass = MassCalculator::compute_from_brep(&shaft_with_key, &params);

    // 解析体積: 円柱体積 - キー溝直方体切削体積 (キー溝体積 = W * T * L)
    let shaft_vol = PI * radius * radius * length;
    let keyway_vol = key_width * key_depth * key_length;
    let expected_vol = shaft_vol - keyway_vol;

    let diff = (mass.volume - expected_vol).abs();
    assert!(
        diff / expected_vol < 1e-3,
        "shaft with keyway volume {} vs expected {}, diff {}",
        mass.volume,
        expected_vol,
        diff
    );
}
