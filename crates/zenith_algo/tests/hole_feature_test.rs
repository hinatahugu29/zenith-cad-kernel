use std::f64::consts::PI;
use zenith_algo::{HoleBuilder, MassCalculator};
use zenith_math::Tolerance;
use zenith_tess::TessellationParams;

#[test]
fn test_counterbore_hole_box_matches_analytic_volume() {
    let tol = Tolerance::default();
    let dx = 40.0;
    let dy = 40.0;
    let dz = 20.0;
    let hole_radius = 4.0;
    let cb_radius = 8.0;
    let cb_depth = 5.0;

    let cb_box =
        HoleBuilder::make_counterbore_hole_box(dx, dy, dz, hole_radius, cb_radius, cb_depth)
            .expect("make_counterbore_hole_box");

    assert!(
        cb_box.is_topologically_valid(&tol),
        "cb_box must be valid closed solid"
    );

    let params = TessellationParams::default();
    let mass = MassCalculator::compute_from_brep(&cb_box, &params);

    // 解析体積: 直方体体積 - 貫通下穴体積 - ザグリ追加切削体積
    let base_vol = dx * dy * dz;
    let through_vol = PI * hole_radius * hole_radius * dz;
    let cb_extra_vol = PI * (cb_radius * cb_radius - hole_radius * hole_radius) * cb_depth;
    let expected_vol = base_vol - through_vol - cb_extra_vol;

    let diff = (mass.volume - expected_vol).abs();
    assert!(
        diff / expected_vol < 1e-4,
        "counterbore volume {} differed from expected {} by diff {}",
        mass.volume,
        expected_vol,
        diff
    );
}

#[test]
fn test_counterbore_hole_box_shallow_and_deep_cases() {
    let tol = Tolerance::default();
    let dx = 50.0;
    let dy = 50.0;
    let dz = 25.0;

    for (hole_radius, cb_radius, cb_depth) in [(3.0, 6.0, 3.0), (5.0, 10.0, 8.0), (2.0, 5.0, 15.0)]
    {
        let cb_box =
            HoleBuilder::make_counterbore_hole_box(dx, dy, dz, hole_radius, cb_radius, cb_depth)
                .expect("make_counterbore_hole_box");

        assert!(
            cb_box.is_topologically_valid(&tol),
            "cb_box must be valid closed solid"
        );

        let params = TessellationParams::default();
        let mass = MassCalculator::compute_from_brep(&cb_box, &params);

        let base_vol = dx * dy * dz;
        let through_vol = PI * hole_radius * hole_radius * dz;
        let cb_extra_vol = PI * (cb_radius * cb_radius - hole_radius * hole_radius) * cb_depth;
        let expected_vol = base_vol - through_vol - cb_extra_vol;

        let diff = (mass.volume - expected_vol).abs();
        assert!(
            diff / expected_vol < 1e-4,
            "counterbore volume {} differed from expected {} by diff {}",
            mass.volume,
            expected_vol,
            diff
        );
    }
}

#[test]
fn test_make_hex_nut_matches_analytic_volume() {
    let tol = Tolerance::default();
    let across_flats = 16.0; // 二面幅 S = 16mm (M10ナット相当)
    let hole_radius = 4.25; // 下穴半径 r = 4.25mm
    let thickness = 8.0; // 厚み H = 8mm

    let nut =
        HoleBuilder::make_hex_nut(across_flats, hole_radius, thickness).expect("make_hex_nut");

    assert!(
        nut.is_topologically_valid(&tol),
        "hex nut must be valid closed solid"
    );

    let params = TessellationParams::default();
    let mass = MassCalculator::compute_from_brep(&nut, &params);

    let hex_area = (3.0f64.sqrt() / 2.0) * across_flats * across_flats;
    let hole_area = PI * hole_radius * hole_radius;
    let expected_vol = (hex_area - hole_area) * thickness;

    let diff = (mass.volume - expected_vol).abs();
    assert!(
        diff / expected_vol < 1e-4,
        "hex nut volume {} vs expected {}, diff {}",
        mass.volume,
        expected_vol,
        diff
    );
}
