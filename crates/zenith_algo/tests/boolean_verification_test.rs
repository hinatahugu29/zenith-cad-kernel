//! Locks in the boolean correctness gate.
//!
//! A closed manifold shell does not prove a boolean is right - returning one
//! operand untouched is manifold and wrong. These tests pin both directions:
//! results that are genuinely correct must still get through, and the known
//! silent-wrong curved-surface cases must now surface as errors instead of
//! plausible-looking solids.

use zenith_algo::{
    BooleanEngine, BooleanOpType, BooleanResultVerifier, BrepTransform, MassCalculator,
    PrimitiveBuilder,
};
use zenith_math::{Tolerance, Vec3};
use zenith_tess::TessellationParams;

fn fine_tessellation() -> TessellationParams {
    TessellationParams {
        u_divisions: 32,
        v_divisions: 32,
    }
}

#[test]
fn test_gate_lets_correct_axis_aligned_box_booleans_through() {
    let tol = Tolerance::default();
    let a = PrimitiveBuilder::make_box(20.0, 20.0, 20.0).unwrap();
    let b = BrepTransform::translate_solid(&a, Vec3::new(10.0, 10.0, 10.0));

    // 重なり 10^3 の角重なり配置。和 15000 / 差 7000 / 積 1000。
    for (op, expected) in [
        (BooleanOpType::Union, 15000.0),
        (BooleanOpType::Difference, 7000.0),
        (BooleanOpType::Intersection, 1000.0),
    ] {
        let result = BooleanEngine::boolean_solids_exact_result(&a, &b, op, &tol)
            .unwrap_or_else(|err| panic!("{op:?} should succeed but was rejected: {err}"));

        let volume: f64 = result
            .solids
            .iter()
            .map(|s| MassCalculator::compute_from_brep(s, &fine_tessellation()).volume)
            .sum();

        assert!(
            (volume - expected).abs() / expected < 1e-9,
            "{op:?} volume {volume} does not match the analytic {expected}"
        );
    }
}

/// 2つの重なる球のブーリアン。
///
/// この試験はもともと「成功したと言ってはならない」を主張していた。曲面同士の
/// 交差が無く、通れば片方の球をそのまま返すしかなかったからである。交線を辿って
/// 面を割れるようになったので、いまは**答えそのもの**を見る。主張が
/// 「できないこと」から「正しいこと」に変わっただけで、守っている当のものは
/// 同じ——片方の球（4188.790）を答えにしてはならない、である。
///
/// 半径 r の球2つ、中心間距離 d の重なり（レンズ）の体積は
/// `(pi/12)(4r + d)(2r - d)^2` で閉じている。
#[test]
fn test_two_overlapping_spheres_give_the_closed_form_or_an_error_but_never_one_operand() {
    let tol = Tolerance::default();
    let radius = 10.0f64;
    let distance = 10.0f64;
    let a = PrimitiveBuilder::make_sphere(radius).unwrap();
    let b = BrepTransform::translate_solid(&a, Vec3::new(distance, 0.0, 0.0));

    let one_sphere = 4.0 / 3.0 * std::f64::consts::PI * radius.powi(3);
    let lens = std::f64::consts::PI / 12.0
        * (4.0 * radius + distance)
        * (2.0 * radius - distance).powi(2);

    for (op, expected) in [
        (BooleanOpType::Union, 2.0 * one_sphere - lens),
        (BooleanOpType::Intersection, lens),
        (BooleanOpType::Difference, one_sphere - lens),
    ] {
        let Ok(result) = BooleanEngine::boolean_solids_exact_result(&a, &b, op, &tol) else {
            // 対応範囲外をエラーで返すのは仕様。誤答でなければよい。
            continue;
        };

        let volume: f64 = result
            .solids
            .iter()
            .map(|s| MassCalculator::compute_from_brep(s, &fine_tessellation()).volume)
            .sum();

        assert!(
            (volume - one_sphere).abs() / one_sphere > 1e-3,
            "{op:?} returned one sphere untouched ({volume})"
        );
        assert!(
            (volume - expected).abs() / expected < 1e-3,
            "{op:?} volume {volume} does not match the closed form {expected}"
        );
    }
}

#[test]
fn test_an_empty_result_is_judged_by_whether_it_should_be_empty() {
    let tol = Tolerance::default();
    let a = PrimitiveBuilder::make_box(20.0, 20.0, 20.0).unwrap();

    // 交わらない2箱の積は空。空であることは失敗ではなく答えなので、
    // ゲートは通さなければならない。
    let far = BrepTransform::translate_solid(&a, Vec3::new(60.0, 0.0, 0.0));
    assert!(
        BooleanResultVerifier::verify(&a, &far, &[], BooleanOpType::Intersection, &tol).is_valid(),
        "an intersection that really is empty must be accepted"
    );

    // ただし「空」を万能の逃げ道にはさせない。重なっている2箱の積が空だと
    // 言えば、内外一貫性が食い違うので弾かれる。
    let overlapping = BrepTransform::translate_solid(&a, Vec3::new(10.0, 10.0, 10.0));
    assert!(
        !BooleanResultVerifier::verify(&a, &overlapping, &[], BooleanOpType::Intersection, &tol)
            .is_valid(),
        "an intersection that is not empty must not be reported as empty"
    );

    // 離れていても、和は空ではない。
    assert!(
        !BooleanResultVerifier::verify(&a, &far, &[], BooleanOpType::Union, &tol).is_valid(),
        "a union is never empty when its operands are not"
    );
}

#[test]
fn test_cone_box_difference_and_intersection_now_split_the_cone_correctly() {
    let tol = Tolerance::default();
    let cone = PrimitiveBuilder::make_cone(10.0, 4.0, 20.0).unwrap();
    let cutter = BrepTransform::translate_solid(
        &PrimitiveBuilder::make_box(20.0, 20.0, 20.0).unwrap(),
        Vec3::new(-10.0, -10.0, 10.0),
    );

    // かつては差も積も同じ 3141.593 を返していた。円柱向けの近道が円錐を
    // 円柱と読み違え、半径10・高さ10の円柱として作り直していたためで、
    // V(A-B) + V(A*B) = V(A) を満たさない。ゲートがそれを弾いていたので
    // 誤答は外に出ていなかった。
    //
    // いまは両方とも正しく求まる。ゲートが弾くことではなく、正しい値が
    // 出ることを固定する。これは以前より強い主張で、3141.593 が戻れば
    // やはり落ちる。
    let frustum = |r0: f64, r1: f64, h: f64| {
        std::f64::consts::PI * h / 3.0 * (r0 * r0 + r0 * r1 + r1 * r1)
    };
    let whole = frustum(10.0, 4.0, 20.0);
    let overlap = frustum(7.0, 4.0, 10.0);

    let expected = [
        (BooleanOpType::Difference, whole - overlap),
        (BooleanOpType::Intersection, overlap),
    ];

    let params = TessellationParams {
        u_divisions: 64,
        v_divisions: 64,
    };
    for (op, want) in expected {
        let result = BooleanEngine::boolean_solids_exact_result(&cone, &cutter, op, &tol)
            .unwrap_or_else(|err| panic!("{op:?} of cone and box should succeed: {err}"));
        assert_eq!(result.solids.len(), 1);
        let got = MassCalculator::compute_from_brep(&result.solids[0], &params).volume;
        let relative = (got - want).abs() / want;
        assert!(
            relative < 1e-6,
            "{op:?} gave {got:.4}, closed form {want:.4} (relative {relative:.2e})"
        );
    }
}

#[test]
fn test_gate_rejects_torus_box_union_smaller_than_an_operand() {
    let tol = Tolerance::default();
    let torus = PrimitiveBuilder::make_torus(12.0, 4.0).unwrap();
    let boxed = BrepTransform::translate_solid(
        &PrimitiveBuilder::make_box(20.0, 20.0, 20.0).unwrap(),
        Vec3::new(-10.0, -10.0, -2.0),
    );

    // 和集合が 4210.072 で、ボックス単体の 8000 を下回っていた。
    assert!(
        BooleanEngine::boolean_solids_exact_result(&torus, &boxed, BooleanOpType::Union, &tol)
            .is_err(),
        "a union smaller than one of its operands must not report success"
    );
}

#[test]
fn test_verifier_rejects_an_operand_passed_through_as_the_union() {
    let tol = Tolerance::default();
    let a = PrimitiveBuilder::make_box(20.0, 20.0, 20.0).unwrap();
    let b = BrepTransform::translate_solid(&a, Vec3::new(10.0, 10.0, 10.0));

    // 検証器そのものを、エンジンを介さずに直接試す。
    let report = BooleanResultVerifier::verify(
        &a,
        &b,
        std::slice::from_ref(&a),
        BooleanOpType::Union,
        &tol,
    );

    assert!(
        !report.is_valid(),
        "handing operand A back as the union must be rejected: {}",
        report.summary()
    );
    assert!(
        report.membership_mismatch_count > 0,
        "the point membership check should be what catches it: {}",
        report.summary()
    );
}

#[test]
fn test_verifier_accepts_a_genuine_union_result() {
    let tol = Tolerance::default();
    let a = PrimitiveBuilder::make_box(20.0, 20.0, 20.0).unwrap();
    let b = BrepTransform::translate_solid(&a, Vec3::new(10.0, 10.0, 10.0));

    let union = BooleanEngine::boolean_solids_exact_result(&a, &b, BooleanOpType::Union, &tol)
        .expect("axis-aligned box union should succeed");

    let report =
        BooleanResultVerifier::verify(&a, &b, &union.solids, BooleanOpType::Union, &tol);

    assert!(
        report.is_valid(),
        "a correct union must not be flagged: {}",
        report.summary()
    );
    assert_eq!(report.membership_mismatch_count, 0);
}
