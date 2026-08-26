use zenith_algo::{MassCalculator, ShaftBuilder};
use zenith_io::{StepExporter, StepImporter};
use zenith_math::Tolerance;
use zenith_tess::TessellationParams;

#[test]
fn test_stepped_shaft_with_keyway() {
    let tol = Tolerance::default();

    // 2段シャフト (段1: 半径15, 長さ40 / 段2: 半径10, 長さ30)
    let shaft = ShaftBuilder::make_stepped_shaft(&[(15.0, 40.0), (10.0, 30.0)])
        .expect("stepped shaft");

    // 段2 (半径10) に幅4, 深さ2.5, 長さ20のキー溝加工 (z=45〜65)
    let shaft_with_key = ShaftBuilder::make_shaft_with_keyway(
        &shaft,
        10.0,
        4.0,
        2.5,
        20.0,
        45.0,
    )
    .expect("shaft with keyway");

    // 1. B-Rep 閉多様体検証
    assert!(
        shaft_with_key.outer_shell.validate_closed(&tol).is_valid(),
        "Shaft with keyway must be valid closed manifold"
    );

    // 2. テッセレーション＆体積検証
    let params = TessellationParams {
        u_divisions: 32,
        v_divisions: 32,
    };
    let mass = MassCalculator::compute_from_brep(&shaft_with_key, &params);
    assert!(mass.volume > 0.0, "Volume must be positive, got {}", mass.volume);

    // 3. STEP 往復検証
    let step_str = StepExporter::export_solid_to_string(&shaft_with_key, "KeywayShaft");
    let reimported = StepImporter::import_solid_from_str(&step_str).expect("import STEP");
    assert!(
        reimported.outer_shell.validate_closed(&tol).is_valid(),
        "Reimported keyway shaft must be valid closed"
    );
}
