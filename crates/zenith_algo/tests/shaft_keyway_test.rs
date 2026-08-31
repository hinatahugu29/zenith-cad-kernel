use zenith_algo::{MassCalculator, ShaftBuilder};
use zenith_io::{StepExporter, StepImporter};
use zenith_math::Tolerance;
use zenith_tess::TessellationParams;

#[test]
fn test_stepped_shaft_with_keyway() {
    let tol = Tolerance::default();

    // 2段シャフト (段1: 半径15, 長さ40 / 段2: 半径10, 長さ30)
    let shaft =
        ShaftBuilder::make_stepped_shaft(&[(15.0, 40.0), (10.0, 30.0)]).expect("stepped shaft");

    // 段2 (半径10) に幅4, 深さ2.5, 長さ20のキー溝加工 (z=45〜65)
    let shaft_with_key = ShaftBuilder::make_shaft_with_keyway(&shaft, 10.0, 4.0, 2.5, 20.0, 45.0)
        .expect("shaft with keyway");

    // 1. B-Rep 閉多様体検証
    assert!(
        shaft_with_key.outer_shell.validate_closed(&tol).is_valid(),
        "Shaft with keyway must be valid closed manifold"
    );

    // 2. 閉形式体積一致検証
    //
    // **以前ここは `volume > 0` だけだった。** それでは溝が丸ごと切れていなく
    // ても、深さが倍でも通る。キー溝は「半径 R の円柱から、幅 w・深さ d の
    // 角溝を長さ L だけ抜いた」形なので、断面は円弧と弦で囲まれた領域になり、
    // 閉じた式で書ける。
    //
    //   除去断面 = ∫_{-w/2}^{w/2} ( sqrt(R^2 - x^2) - (R - d) ) dx
    //            = [ x/2 * sqrt(R^2-x^2) + (R^2/2) * asin(x/R) ]_{-w/2}^{w/2} - w(R-d)
    let params = TessellationParams {
        u_divisions: 32,
        v_divisions: 32,
    };
    let mass = MassCalculator::compute_from_brep(&shaft_with_key, &params);

    let pi = std::f64::consts::PI;
    let shaft_volume = pi * 15.0 * 15.0 * 40.0 + pi * 10.0 * 10.0 * 30.0;

    let r = 10.0_f64;
    let half = 4.0 / 2.0;
    let flat = r - 2.5;
    let antiderivative = |x: f64| x * 0.5 * (r * r - x * x).sqrt() + (r * r * 0.5) * (x / r).asin();
    let removed_section = (antiderivative(half) - antiderivative(-half)) - 2.0 * half * flat;
    let expected = shaft_volume - removed_section * 20.0;

    let error = (mass.volume - expected).abs() / expected;
    assert!(
        error < 1e-6,
        "Keyway shaft volume {} does not match the closed form {expected} (relative {error:.3e})",
        mass.volume
    );

    // 3. STEP 往復検証
    let step_str = StepExporter::export_solid_to_string(&shaft_with_key, "KeywayShaft");
    let reimported = StepImporter::import_solid_from_str(&step_str).expect("import STEP");
    assert!(
        reimported.outer_shell.validate_closed(&tol).is_valid(),
        "Reimported keyway shaft must be valid closed"
    );
}
