//! トーラスと箱のブーリアン。
//!
//! 曲面同士が交わるブーリアンのうち、いちばん噛み合わせが多い配置である。
//! 箱の**底面**にはトーラスの断面（入れ子の2つの円）が来るが、外側の円は
//! 面からはみ出すので、面のトリム境界で切ってからでないと使えない。箱の
//! **側面**にはパラメータ線でない交線が細切れで届き、端で繋いで初めて
//! 境界から境界へ届く切り込みになる。
//!
//! 期待値は閉じた式では書けないので、**別のやり方で積んだ値**を使う。
//! トーラスの断面は各 (x, y) について z 方向の区間として陽に書けるので、
//! 正方形の上で2次元求積すれば体積が出る。カーネルの答えとは無関係な経路
//! なので、外の物差しになる。

use std::f64::consts::PI;

use zenith_algo::{
    BooleanEngine, BooleanOpType, BooleanResultVerifier, BrepTransform, MassCalculator,
    PrimitiveBuilder,
};
use zenith_math::{Tolerance, Vec3};
use zenith_tess::TessellationParams;

/// 主半径 `major`、副半径 `minor` のトーラスと、
/// `|x| <= half`, `|y| <= half`, `z >= floor` の箱の共通部分の体積。
///
/// 各 (x, y) でトーラスは `|z| <= sqrt(minor^2 - (rho - major)^2)` を占める。
/// その区間を `z >= floor` で切り、正方形の上で足す。
fn torus_box_intersection_volume(
    major: f64,
    minor: f64,
    half: f64,
    floor: f64,
    steps: usize,
) -> f64 {
    let step = 2.0 * half / steps as f64;
    let mut total = 0.0;
    for i in 0..steps {
        let x = -half + (i as f64 + 0.5) * step;
        for j in 0..steps {
            let y = -half + (j as f64 + 0.5) * step;
            let rho = (x * x + y * y).sqrt();
            let inside = minor * minor - (rho - major) * (rho - major);
            if inside <= 0.0 {
                continue;
            }
            let top = inside.sqrt();
            let bottom = (-top).max(floor);
            if top > bottom {
                total += top - bottom;
            }
        }
    }
    total * step * step
}

/// 求積そのものを、閉じた式で答えの分かる場合に当てて確かめる。
/// 期待値を出す道具が狂っていれば、その先の一致に意味はない。
#[test]
fn the_quadrature_reproduces_the_whole_torus_when_nothing_is_cut_away() {
    // 正方形を十分大きく、床を十分下に取れば、トーラス全体になる。
    let whole = torus_box_intersection_volume(12.0, 4.0, 20.0, -10.0, 2000);
    let closed_form = 2.0 * PI * PI * 12.0 * 16.0;
    let error = (whole - closed_form).abs() / closed_form;
    assert!(
        error < 1e-4,
        "the quadrature gives {whole} where the torus is {closed_form} ({error:.3e})"
    );
}

#[test]
fn torus_and_box_agree_with_an_independent_quadrature() {
    let tol = Tolerance::default();
    let torus = PrimitiveBuilder::make_torus(12.0, 4.0).unwrap();
    // 20 立方の箱を (-10, -10, -2) へ。|x|, |y| <= 10、z は -2 から 18。
    let cutter = BrepTransform::translate_solid(
        &PrimitiveBuilder::make_box(20.0, 20.0, 20.0).unwrap(),
        Vec3::new(-10.0, -10.0, -2.0),
    );

    let torus_volume = 2.0 * PI * PI * 12.0 * 16.0;
    let box_volume = 8000.0;
    let lens = torus_box_intersection_volume(12.0, 4.0, 10.0, -2.0, 2000);

    let params = TessellationParams {
        u_divisions: 48,
        v_divisions: 48,
    };

    for (op, expected) in [
        (BooleanOpType::Union, torus_volume + box_volume - lens),
        (BooleanOpType::Difference, torus_volume - lens),
        (BooleanOpType::Intersection, lens),
    ] {
        let result = BooleanEngine::boolean_solids_exact_result(&torus, &cutter, op, &tol)
            .unwrap_or_else(|err| panic!("{op:?} of a torus and a box failed: {err}"));

        for solid in &result.solids {
            let report = solid.outer_shell.validate_closed(&tol);
            assert!(
                report.is_valid(),
                "{op:?} produced a shell that is not closed: {:?}",
                report.errors
            );
        }
        assert!(
            BooleanResultVerifier::verify(&torus, &cutter, &result.solids, op, &tol).is_valid(),
            "{op:?} did not pass the verification gate"
        );

        let volume: f64 = result
            .solids
            .iter()
            .map(|solid| MassCalculator::compute_from_brep(solid, &params).volume)
            .sum();

        // どちらかのオペランドをそのまま返していないこと。
        for (name, operand) in [("the torus", torus_volume), ("the box", box_volume)] {
            if (expected - operand).abs() / operand > 1e-3 {
                assert!(
                    (volume - operand).abs() / operand > 1e-3,
                    "{op:?} returned {name} untouched ({volume})"
                );
            }
        }

        // 求積は 2000 分割で 1e-6 台まで収束している。1e-4 で見る。
        assert!(
            (volume - expected).abs() / expected < 1e-4,
            "{op:?} volume {volume} does not match the quadrature {expected}"
        );
    }
}
