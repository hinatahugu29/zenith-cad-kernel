use std::f64::consts::PI;
use zenith_algo::{FlangeBuilder, HoleBuilder, MassCalculator, ShaftBuilder};
use zenith_math::Tolerance;
use zenith_tess::TessellationParams;

#[test]
fn test_make_countersink_hole_box_matches_analytic_volume() {
    let tol = Tolerance::default();
    let (w, d, h) = (40.0, 40.0, 20.0);
    let hole_r = 3.0; // M6下穴 (半径3mm)
    let cs_r = 6.0;   // 皿モミ上面半径 6mm
    let cs_angle_deg = 90.0;
    let (cx, cy) = (20.0, 20.0);

    let solid = HoleBuilder::make_countersink_hole_box(w, d, h, hole_r, cs_r, cs_angle_deg, cx, cy)
        .expect("make_countersink_hole_box");

    assert!(solid.is_topologically_valid(&tol), "countersink hole box must be valid closed solid");

    let params = TessellationParams::default();
    let mass = MassCalculator::compute_from_brep(&solid, &params);

    // 解析体積:
    // 直方体体積 - 貫通下穴体積 - (円錐台体積 - 下穴体積の重複分)
    // 皿モミ深さ H_cs = (cs_r - hole_r) / tan(45°) = cs_r - hole_r
    let cs_depth = cs_r - hole_r; // 3.0mm
    let box_vol = w * d * h;
    let through_hole_vol = PI * hole_r * hole_r * h;
    // 円錐台体積 V_frustum = (PI * H_cs / 3) * (cs_r^2 + cs_r * hole_r + hole_r^2)
    let frustum_vol = (PI * cs_depth / 3.0) * (cs_r * cs_r + cs_r * hole_r + hole_r * hole_r);
    // 円錐台領域内の下穴円柱体積 V_cyl = PI * hole_r^2 * cs_depth
    let frustum_excess = frustum_vol - (PI * hole_r * hole_r * cs_depth);
    let expected_vol = box_vol - through_hole_vol - frustum_excess;

    let diff = (mass.volume - expected_vol).abs();
    assert!(
        diff / expected_vol < 1e-4,
        "countersink volume {} vs expected {}, diff {}",
        mass.volume,
        expected_vol,
        diff
    );
}

#[test]
fn test_make_circular_flange_matches_analytic_volume() {
    let tol = Tolerance::default();
    let outer_r = 40.0;
    let thickness = 10.0;
    let center_r = 15.0;
    let pcd_r = 28.0;
    let num_holes = 4;
    let bolt_r = 3.5;

    let flange = FlangeBuilder::make_circular_flange(
        outer_r,
        thickness,
        center_r,
        pcd_r,
        num_holes,
        bolt_r,
    )
    .expect("make_circular_flange");

    assert!(flange.is_topologically_valid(&tol), "circular flange must be valid closed solid");

    let params = TessellationParams::default();
    let mass = MassCalculator::compute_from_brep(&flange, &params);

    // 解析体積: PI * (outer_r^2 - center_r^2 - num_holes * bolt_r^2) * thickness
    let area = PI * (outer_r * outer_r - center_r * center_r - (num_holes as f64) * bolt_r * bolt_r);
    let expected_vol = area * thickness;

    let diff = (mass.volume - expected_vol).abs();
    assert!(
        diff / expected_vol < 1e-4,
        "flange volume {} vs expected {}, diff {}",
        mass.volume,
        expected_vol,
        diff
    );
}

#[test]
fn test_make_shaft_with_annular_groove_matches_analytic_volume() {
    let tol = Tolerance::default();
    let shaft_r = 15.0;
    let shaft_l = 60.0;
    let groove_w = 4.0;
    let groove_depth = 2.5;
    let groove_z = 25.0;

    let base_shaft = ShaftBuilder::make_stepped_shaft(&[(shaft_r, shaft_l)])
        .expect("base shaft");

    let grooved_shaft = ShaftBuilder::make_shaft_with_annular_groove(
        &base_shaft,
        shaft_r,
        groove_w,
        groove_depth,
        groove_z,
    )
    .expect("make_shaft_with_annular_groove");

    assert!(grooved_shaft.is_topologically_valid(&tol), "grooved shaft must be valid closed solid");

    let params = TessellationParams::default();
    let mass = MassCalculator::compute_from_brep(&grooved_shaft, &params);

    // 解析体積: 軸体積 - 溝リング切削体積 (溝体積 = PI * (shaft_r^2 - (shaft_r - groove_depth)^2) * groove_w)
    let shaft_vol = PI * shaft_r * shaft_r * shaft_l;
    let inner_r = shaft_r - groove_depth;
    let groove_vol = PI * (shaft_r * shaft_r - inner_r * inner_r) * groove_w;
    let expected_vol = shaft_vol - groove_vol;

    let diff = (mass.volume - expected_vol).abs();
    assert!(
        diff / expected_vol < 1e-4,
        "grooved shaft volume {} vs expected {}, diff {}",
        mass.volume,
        expected_vol,
        diff
    );
}
