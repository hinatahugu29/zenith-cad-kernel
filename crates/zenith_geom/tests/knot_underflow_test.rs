//! **制御点が次数より少ないノットベクトルで、プロセスごと落ちないこと**（4-415）。
//!
//! # 何が起きていたか
//!
//! `KnotVector::clamped_uniform(num_ctrl_pts, degree)` は
//! `num_ctrl_pts - degree - 1` を `usize` で計算していました。
//! **`num_ctrl_pts <= degree` だと桁溢れします**——**debug では panic、
//! release では黙って 1.8e19 に化けます。** そのまま
//! `for i in 1..=num_inner` へ入るので、**128 GiB を確保しようとして
//! プロセスごと落ちます**（実測:
//! `memory allocation of 137438953472 bytes failed`）。
//!
//! **Python から届きました。** `thicken_surface_patch` は渡された点を
//! そのまま次数 3 の B-spline にするので、**点を 2 個渡すだけ**です。
//! **例外ではなく abort** なので、**Blender に載せれば Blender ごと
//! 落ちます。**
//!
//! # なぜ断りに変わるのか
//!
//! **断りは、もともとありました**——`NurbsCurve3::new` は
//! 「制御点が次数 + 1 個より少ない」を返します。**そこへ届く前に
//! 落ちていた**だけです。飽和させると、ノットの本数が
//! `2 * (degree + 1)` になり、その断りに届きます。
//!
//! # 読み方
//!
//! **この試験は release でこそ効きます**——**debug なら桁溢れは
//! panic で捕まりますが、配るのは release** です。

use zenith_geom::{KnotVector, NurbsCurve3};
use zenith_math::Point3;

/// **制御点が次数以下でも、ノットの本数が有限であること。**
#[test]
fn clamped_uniform_does_not_overflow_when_there_are_too_few_control_points() {
    for degree in 1..=5 {
        for count in 0..=degree {
            let knots = KnotVector::clamped_uniform(count, degree);
            // **飽和させたので、内側のノットは 0 本**です。
            assert_eq!(
                knots.knots.len(),
                2 * (degree + 1),
                "clamped_uniform({count}, {degree}) should stay at the two clamped ends"
            );
        }
    }
}

/// **点が足りない曲線は、落ちずに断られること。**
#[test]
fn a_cubic_spline_through_two_points_is_refused_not_a_crash() {
    let points = vec![Point3::new(0.0, 0.0, 0.0), Point3::new(10.0, 0.0, 0.0)];
    let result = NurbsCurve3::bspline_from_points(3, points);
    let message = result.expect_err("two points cannot carry a cubic").to_string();
    assert!(
        message.contains("degree"),
        "the refusal should name the degree, got {message}"
    );
}

/// **足りている側は、これまでどおり通ること。**
///
/// **飽和は、壊れていた場合にしか効きません**——**正しい入力の
/// ノットが 1 本でも変わっていないこと**を、ここで押さえます。
#[test]
fn enough_control_points_still_build_the_same_knots() {
    for degree in 1..=5 {
        for count in (degree + 1)..=(degree + 6) {
            let knots = KnotVector::clamped_uniform(count, degree);
            assert_eq!(
                knots.knots.len(),
                count + degree + 1,
                "clamped_uniform({count}, {degree}) must stay at n + p + 1"
            );
        }
    }
    let points: Vec<Point3> = (0..6)
        .map(|index| Point3::new(index as f64, 0.0, 0.0))
        .collect();
    NurbsCurve3::bspline_from_points(3, points).expect("six points carry a cubic");
}
