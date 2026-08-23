use std::f64::consts::PI;
use zenith_algo::{MassCalculator, PrimitiveBuilder};
use zenith_math::Tolerance;
use zenith_tess::TessellationParams;

#[test]
fn test_regular_prisms_match_analytic_volume() {
    let tol = Tolerance::default();
    let params = TessellationParams::default();

    // 1. 正六角柱 (sides = 6, r = 10, h = 30)
    // 解析体積: (6 / 2) * R^2 * sin(60°) * H = 3 * 100 * (sqrt(3)/2) * 30 = 4500 * sqrt(3) ≈ 7794.228634
    let hex_prism = PrimitiveBuilder::make_regular_prism(6, 10.0, 30.0).expect("make_regular_prism 6");
    assert!(hex_prism.is_topologically_valid(&tol), "hex prism must be valid closed solid");
    let mass_hex = MassCalculator::compute_from_brep(&hex_prism, &params);
    let expected_hex = (6.0 / 2.0) * 100.0 * (2.0 * PI / 6.0).sin() * 30.0;
    assert!(
        (mass_hex.volume - expected_hex).abs() < 1e-6,
        "hex prism volume {} vs expected {}",
        mass_hex.volume,
        expected_hex
    );

    // 2. 正八角柱 (sides = 8, r = 12, h = 20)
    // 解析体積: (8 / 2) * 144 * sin(45°) * 20 = 4 * 144 * (1/sqrt(2)) * 20 ≈ 8145.88706
    let oct_prism = PrimitiveBuilder::make_regular_prism(8, 12.0, 20.0).expect("make_regular_prism 8");
    assert!(oct_prism.is_topologically_valid(&tol), "oct prism must be valid closed solid");
    let mass_oct = MassCalculator::compute_from_brep(&oct_prism, &params);
    let expected_oct = (8.0 / 2.0) * 144.0 * (2.0 * PI / 8.0).sin() * 20.0;
    assert!(
        (mass_oct.volume - expected_oct).abs() < 1e-6,
        "oct prism volume {} vs expected {}",
        mass_oct.volume,
        expected_oct
    );

    // 3. 正三角柱 (sides = 3, r = 15, h = 25)
    // 解析体積: (3 / 2) * 225 * sin(120°) * 25
    let tri_prism = PrimitiveBuilder::make_regular_prism(3, 15.0, 25.0).expect("make_regular_prism 3");
    assert!(tri_prism.is_topologically_valid(&tol), "tri prism must be valid closed solid");
    let mass_tri = MassCalculator::compute_from_brep(&tri_prism, &params);
    let expected_tri = (3.0 / 2.0) * 225.0 * (2.0 * PI / 3.0).sin() * 25.0;
    assert!(
        (mass_tri.volume - expected_tri).abs() < 1e-6,
        "tri prism volume {} vs expected {}",
        mass_tri.volume,
        expected_tri
    );
}

#[test]
fn test_hex_bolt_modeling_via_boolean_union() {
    use zenith_algo::{BooleanEngine, BooleanOpType, BrepTransform};

    let tol = Tolerance::default();
    let params = TessellationParams::default();

    // 1. 六角ボルト頭: 外接半径 8mm, 高さ 6mm
    let hex_head = PrimitiveBuilder::make_regular_prism(6, 8.0, 6.0).expect("hex_head");
    // 2. ボルト軸円柱: 半径 4mm, 長さ 24mm (z=6 から z=30)
    let shaft = PrimitiveBuilder::make_cylinder(4.0, 24.0).expect("shaft");
    let shaft = BrepTransform::translate_solid(&shaft, zenith_math::Vec3::new(0.0, 0.0, 6.0));

    // ブーリアン結合
    let bolt = BooleanEngine::boolean_solids_exact(&hex_head, &shaft, BooleanOpType::Union, &tol)
        .expect("boolean union hex_head + shaft");

    assert!(bolt.is_topologically_valid(&tol), "bolt must be valid solid");
    let mass_bolt = MassCalculator::compute_from_brep(&bolt, &params);

    // 解析体積: 頭部体積 + 軸体積
    let head_vol = (6.0 / 2.0) * 64.0 * (2.0 * PI / 6.0).sin() * 6.0;
    let shaft_vol = PI * 16.0 * 24.0;
    let expected_bolt = head_vol + shaft_vol;

    let diff = (mass_bolt.volume - expected_bolt).abs();
    assert!(
        diff < 1e-6,
        "bolt volume {} vs expected {}, diff {}",
        mass_bolt.volume,
        expected_bolt,
        diff
    );
}
