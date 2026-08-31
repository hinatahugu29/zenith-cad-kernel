use std::f64::consts::PI;
use zenith_algo::{BooleanEngine, BooleanOpType, BrepTransform, MassCalculator, PrimitiveBuilder};
use zenith_math::{Tolerance, Vec3};
use zenith_tess::TessellationParams;

#[test]
fn test_batch_boolean_four_bolt_holes_difference() {
    let tol = Tolerance::default();
    let params = TessellationParams::default();

    // 1. ベースプレート: 100 x 100 x 10 mm
    let plate = PrimitiveBuilder::make_box(100.0, 100.0, 10.0).expect("make_box");

    // 2. 4隅のボルト穴ツール (半径 4mm, 高さ 12mm)
    let drill_prototype = PrimitiveBuilder::make_cylinder(4.0, 12.0).expect("drill");
    let hole_positions = [
        Vec3::new(20.0, 20.0, -1.0),
        Vec3::new(80.0, 20.0, -1.0),
        Vec3::new(80.0, 80.0, -1.0),
        Vec3::new(20.0, 80.0, -1.0),
    ];

    let drills: Vec<_> = hole_positions
        .iter()
        .map(|&pos| BrepTransform::translate_solid(&drill_prototype, pos))
        .collect();

    // 3. バッチブーリアン差分
    let drilled_plate =
        BooleanEngine::boolean_solids_batch(&plate, &drills, BooleanOpType::Difference, &tol)
            .expect("boolean_solids_batch 4 holes");

    assert!(
        drilled_plate.is_topologically_valid(&tol),
        "drilled plate must be valid closed solid"
    );

    let mass = MassCalculator::compute_from_brep(&drilled_plate, &params);

    // 解析体積: プレート体積 - 4 * 穴体積
    let base_vol = 100.0 * 100.0 * 10.0;
    let four_holes_vol = 4.0 * (PI * 16.0 * 10.0);
    let expected_vol = base_vol - four_holes_vol;

    let diff = (mass.volume - expected_vol).abs();
    assert!(
        diff < 1e-4,
        "drilled plate volume {} vs expected {}, diff {}",
        mass.volume,
        expected_vol,
        diff
    );
}

#[test]
fn test_batch_boolean_multiple_ribs_union() {
    let tol = Tolerance::default();
    let params = TessellationParams::default();

    // 1. ベースブロック: 80 x 40 x 10 mm
    let base = PrimitiveBuilder::make_box(80.0, 40.0, 10.0).expect("make_box");

    // 2. 2本のリブブロック: 10 x 40 x 15 mm (z=10からz=25へ突出)
    let rib1 = PrimitiveBuilder::make_box(10.0, 40.0, 15.0).expect("rib1");
    let rib1 = BrepTransform::translate_solid(&rib1, Vec3::new(15.0, 0.0, 10.0));

    let rib2 = PrimitiveBuilder::make_box(10.0, 40.0, 15.0).expect("rib2");
    let rib2 = BrepTransform::translate_solid(&rib2, Vec3::new(55.0, 0.0, 10.0));

    let ribs = vec![rib1, rib2];

    // 3. バッチブーリアン結合
    let ribbed_part = BooleanEngine::boolean_solids_batch(&base, &ribs, BooleanOpType::Union, &tol)
        .expect("boolean_solids_batch ribs union");

    assert!(
        ribbed_part.is_topologically_valid(&tol),
        "ribbed part must be valid closed solid"
    );

    let mass = MassCalculator::compute_from_brep(&ribbed_part, &params);

    // 解析体積: ベース体積 + 2 * リブ体積
    let base_vol = 80.0 * 40.0 * 10.0;
    let ribs_vol = 2.0 * (10.0 * 40.0 * 15.0);
    let expected_vol = base_vol + ribs_vol;

    let diff = (mass.volume - expected_vol).abs();
    assert!(
        diff < 1e-6,
        "ribbed part volume {} vs expected {}, diff {}",
        mass.volume,
        expected_vol,
        diff
    );
}
