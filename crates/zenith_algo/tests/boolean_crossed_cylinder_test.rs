//! 直交する2本の円柱のブーリアン。
//!
//! これは曲面同士が交わるブーリアンの、球より一段難しい配置である。交線は
//! 相手のパッチの境界で**細切れ**になって届き、どの片も面の内側で終わるので、
//! 1本ずつ当てても割れない。端で繋いで初めて、境界から境界へ届く切り込みに
//! なる。
//!
//! 半径 `R`, `r` の直交する2円柱（軸は交わる）の交わりには閉じた式がある。
//!
//! ```text
//! V = (8/3) R^3 [(1 + k^2) E(k) - (1 - k^2) K(k)],  k = r / R
//! ```
//!
//! 半径が等しいときは Steinmetz の `16 R^3 / 3` に戻る。**測定値を期待値に
//! 使わない**ために、`K` と `E` はここで算術幾何平均から求める。

use std::f64::consts::PI;

use zenith_algo::{
    BooleanEngine, BooleanOpType, BooleanResultVerifier, BrepTransform, MassCalculator,
    PrimitiveBuilder,
};
use zenith_math::{Tolerance, Transform3, Vec3};
use zenith_tess::TessellationParams;

/// 第1種・第2種の完全楕円積分を算術幾何平均で求める。
fn complete_elliptic_k_e(k: f64) -> (f64, f64) {
    let mut a = 1.0f64;
    let mut b = (1.0 - k * k).sqrt();
    let mut c = k;
    let mut sum = c * c * 0.5;
    let mut power = 1.0f64;
    for _ in 0..40 {
        if c.abs() < 1e-17 {
            break;
        }
        let next_a = (a + b) * 0.5;
        let next_b = (a * b).sqrt();
        c = (a - b) * 0.5;
        a = next_a;
        b = next_b;
        power *= 2.0;
        sum += power * c * c * 0.5;
    }
    let k_value = PI / (2.0 * a);
    (k_value, k_value * (1.0 - sum))
}

fn bicylinder_intersection_volume(big: f64, small: f64) -> f64 {
    let k = small / big;
    let (k_value, e_value) = complete_elliptic_k_e(k);
    8.0 / 3.0 * big.powi(3) * ((1.0 + k * k) * e_value - (1.0 - k * k) * k_value)
}

/// 算術幾何平均の実装そのものを、独立に分かっている値で確かめる。
/// 期待値を出す道具が狂っていれば、その先の一致には意味がない。
#[test]
fn the_elliptic_integrals_match_their_known_values() {
    let (k0, e0) = complete_elliptic_k_e(0.0);
    assert!(
        (k0 - PI / 2.0).abs() < 1e-14,
        "K(0) should be pi/2, got {k0}"
    );
    assert!(
        (e0 - PI / 2.0).abs() < 1e-14,
        "E(0) should be pi/2, got {e0}"
    );

    // 半径が等しいときの Steinmetz 立体は 16 R^3 / 3。
    let steinmetz = bicylinder_intersection_volume(10.0, 10.0 - 1e-12);
    assert!(
        (steinmetz - 16.0 * 1000.0 / 3.0).abs() / (16.0 * 1000.0 / 3.0) < 1e-6,
        "equal radii should give 16 R^3 / 3, got {steinmetz}"
    );
}

#[test]
fn crossed_cylinders_match_the_closed_form_for_all_three_operations() {
    let tol = Tolerance::default();
    let big_radius = 10.0f64;
    let small_radius = 6.0f64;
    let height = 40.0f64;

    let a = PrimitiveBuilder::make_cylinder(big_radius, height).unwrap();
    let rotation =
        Transform3::from_axis_angle(&Vec3::new(0.0, 1.0, 0.0), std::f64::consts::FRAC_PI_2);
    let along_x = BrepTransform::transform_solid(
        &PrimitiveBuilder::make_cylinder(small_radius, height).unwrap(),
        &rotation,
    )
    .unwrap();
    // 大きい円柱の中ほど (z = 20) を貫くように置く。
    let b = BrepTransform::translate_solid(&along_x, Vec3::new(-20.0, 0.0, 20.0));

    let volume_a = PI * big_radius * big_radius * height;
    let volume_b = PI * small_radius * small_radius * height;
    let lens = bicylinder_intersection_volume(big_radius, small_radius);

    let params = TessellationParams {
        u_divisions: 48,
        v_divisions: 48,
    };

    for (op, expected) in [
        (BooleanOpType::Union, volume_a + volume_b - lens),
        (BooleanOpType::Difference, volume_a - lens),
        (BooleanOpType::Intersection, lens),
    ] {
        let result = BooleanEngine::boolean_solids_exact_result(&a, &b, op, &tol)
            .unwrap_or_else(|err| panic!("{op:?} of two crossed cylinders failed: {err}"));

        for solid in &result.solids {
            let report = solid.outer_shell.validate_closed(&tol);
            assert!(
                report.is_valid(),
                "{op:?} produced a shell that is not closed: {:?}",
                report.errors
            );
        }

        // 384点の内外一貫性も見る。体積が合っていても形が違えば落ちる。
        assert!(
            BooleanResultVerifier::verify(&a, &b, &result.solids, op, &tol).is_valid(),
            "{op:?} did not pass the verification gate"
        );

        let volume: f64 = result
            .solids
            .iter()
            .map(|solid| MassCalculator::compute_from_brep(solid, &params).volume)
            .sum();

        // どちらかのオペランドをそのまま返していないこと。閉じた形になって
        // いても、それは答えではない。
        for (name, operand) in [("A", volume_a), ("B", volume_b)] {
            if (expected - operand).abs() / operand > 1e-3 {
                assert!(
                    (volume - operand).abs() / operand > 1e-3,
                    "{op:?} returned operand {name} untouched ({volume})"
                );
            }
        }

        assert!(
            (volume - expected).abs() / expected < 1e-6,
            "{op:?} volume {volume} does not match the closed form {expected}"
        );
    }
}

/// **半径が等しい**直交2円柱。上のテスト（半径 10 と 6）との違いは1つだけ
/// ですが、難しさが変わります。
///
/// 半径が違うと交線は滑らかな閉じた輪1本ですが、**等しいと2本の楕円になり、
/// それが2点で交わります**。その交点では両曲面の法線が平行——つまり接して
/// いるので、交線の向きが決まりません。辿りはそこで止まりますが、**止まる
/// 場所が A 側と B 側で食い違います**（実測 2e-5。残差では位置が決まらない、
/// 4-81 と同じ機構）。端が合わないので縫合が 16本あぶれ、3演算とも
/// 「未実装」として断られていました（4-128 で解決）。
///
/// 等半径のときの交わりは Steinmetz 立体で、体積は `16 R^3 / 3` です。
#[test]
fn equal_radius_crossed_cylinders_give_the_steinmetz_solid() {
    let tol = Tolerance::default();
    let radius = 6.0f64;
    let height = 40.0f64;

    let a = BrepTransform::translate_solid(
        &PrimitiveBuilder::make_cylinder(radius, height).unwrap(),
        Vec3::new(0.0, 0.0, -20.0),
    );
    let rotation =
        Transform3::from_axis_angle(&Vec3::new(0.0, 1.0, 0.0), std::f64::consts::FRAC_PI_2);
    let b = BrepTransform::translate_solid(
        &BrepTransform::transform_solid(
            &PrimitiveBuilder::make_cylinder(radius, height).unwrap(),
            &rotation,
        )
        .unwrap(),
        Vec3::new(-20.0, 0.0, 0.0),
    );

    let one = PI * radius * radius * height;
    let steinmetz = 16.0 * radius.powi(3) / 3.0;
    // 閉じた式を出す道具そのものと突き合わせる。**測定値は期待値に使いません。**
    let from_elliptic = bicylinder_intersection_volume(radius, radius - 1e-12);
    assert!(
        (from_elliptic - steinmetz).abs() / steinmetz < 1e-6,
        "the elliptic form and 16 R^3 / 3 should agree, got {from_elliptic} and {steinmetz}"
    );

    let params = TessellationParams {
        u_divisions: 96,
        v_divisions: 96,
    };

    for (op, expected, solids) in [
        (BooleanOpType::Union, 2.0 * one - steinmetz, 1usize),
        // 切り手の半径が同じなので、A は帯をまるごと失って**2つに割れます**。
        (BooleanOpType::Difference, one - steinmetz, 2),
        (BooleanOpType::Intersection, steinmetz, 1),
    ] {
        let result = BooleanEngine::boolean_solids_exact_result(&a, &b, op, &tol)
            .unwrap_or_else(|err| panic!("{op:?} of two equal cylinders failed: {err}"));

        assert_eq!(
            result.solids.len(),
            solids,
            "{op:?} should return {solids} solid(s)"
        );

        for solid in &result.solids {
            let report = solid.outer_shell.validate_closed(&tol);
            assert!(
                report.is_valid(),
                "{op:?} produced a shell that is not closed: {:?}",
                report.errors
            );
        }

        let volume: f64 = result
            .solids
            .iter()
            .map(|solid| MassCalculator::compute_from_brep(solid, &params).volume)
            .sum();
        assert!(
            (volume - expected).abs() / expected < 1e-6,
            "{op:?} volume {volume} does not match the closed form {expected}"
        );
    }
}
