//! 皿モミ穴が、1組の寸法だけでなく寸法を振っても作れるか。
//!
//! 既存のテストは下穴 r3 / 皿 r6 / 90 度の1組だけを見ていました。比を 2.0 から
//! 1.8 に変えるだけで落ちる組があり、しかも落ち方が「寸法が範囲外」ではなく
//! p-curve のずれを72件並べた文字列だったので、呼び出し側には何を直せばよいか
//! 分かりませんでした。
//!
//! 原因は p-curve の細分が、詰まらない1区間に予算を食われて残りの区間を
//! 最初の8分割のまま残していたことです（`crates/zenith_topo/src/face.rs`）。
//!
//! ここでは寸法を格子状に振り、**全部作れること**と、**体積が閉じた式に
//! 乗ること**を測ります。作れただけでは足りません。

use std::f64::consts::PI;

use zenith_algo::{HoleBuilder, MassCalculator};
use zenith_math::Tolerance;
use zenith_tess::TessellationParams;

/// 皿モミ穴を空けた直方体の体積（解析解）
///
/// 直方体から、貫通下穴の円柱と、皿の円錐台のうち下穴の外側にある分を引く。
fn expected_volume(w: f64, d: f64, h: f64, hole_r: f64, cs_r: f64, cs_angle_deg: f64) -> f64 {
    let tan_half = (cs_angle_deg * 0.5).to_radians().tan();
    let cs_depth = (cs_r - hole_r) / tan_half;
    let frustum = PI / 3.0 * cs_depth * (hole_r * hole_r + hole_r * cs_r + cs_r * cs_r);
    let already_drilled = PI * hole_r * hole_r * cs_depth;
    w * d * h - PI * hole_r * hole_r * h - (frustum - already_drilled)
}

#[test]
fn a_countersink_is_built_across_the_dimensions_it_claims_to_take() {
    let tol = Tolerance::default();
    let (w, d, h) = (40.0, 40.0, 20.0);
    let (cx, cy) = (20.0, 20.0);
    let params = TessellationParams {
        u_divisions: 32,
        v_divisions: 32,
    };

    // 先頭の7組は、細分の予算配分を直すまで落ちていた寸法そのもの。
    // 残りは通っていた側から、下穴・比・角度の端を拾ったもの。
    // 格子64組すべてを回すと4分近くかかるので、ここは代表を測る
    // （全数は `cargo run -p zenith_algo --example countersink_range_probe`）。
    let cases: [(f64, f64, f64); 12] = [
        (2.0, 3.0, 60.0),
        (2.0, 3.6, 82.0),
        (2.0, 5.0, 60.0),
        (5.0, 7.5, 60.0),
        (5.0, 9.0, 90.0),
        (5.0, 10.0, 90.0),
        (5.0, 10.0, 120.0),
        (2.0, 3.0, 120.0),
        (3.0, 6.0, 90.0),
        (3.0, 7.5, 60.0),
        (4.0, 10.0, 82.0),
        (5.0, 12.5, 120.0),
    ];

    for (hole_r, cs_r, angle) in cases {
        let solid = HoleBuilder::make_countersink_hole_box(w, d, h, hole_r, cs_r, angle, cx, cy)
            .unwrap_or_else(|err| {
                panic!(
                    "countersink hole_r={hole_r} cs_r={cs_r} angle={angle}: {}",
                    &err[..err.len().min(160)]
                )
            });

        let report = solid.outer_shell.validate_closed(&tol);
        assert!(
            report.is_valid(),
            "hole_r={hole_r} cs_r={cs_r} angle={angle}: {:?}",
            report.errors.first()
        );

        let volume = MassCalculator::compute_from_brep(&solid, &params).volume;
        let expected = expected_volume(w, d, h, hole_r, cs_r, angle);
        assert!(
            (volume - expected).abs() / expected < 1e-6,
            "hole_r={hole_r} cs_r={cs_r} angle={angle}: measured {volume} against {expected}"
        );
    }
}

#[test]
fn dimensions_outside_the_range_are_refused_with_a_reason() {
    let (w, d, h) = (40.0, 40.0, 20.0);
    let (cx, cy) = (20.0, 20.0);

    // 皿が下穴より小さい / 角度が範囲外は、内部の数値ではなく寸法として断る
    for (hole_r, cs_r, angle) in [(5.0, 4.0, 90.0), (5.0, 9.0, 0.0), (5.0, 9.0, 180.0)] {
        let error = HoleBuilder::make_countersink_hole_box(w, d, h, hole_r, cs_r, angle, cx, cy)
            .expect_err("out of range dimensions must be refused");
        assert!(
            error.contains("Invalid countersink dimensions"),
            "the refusal should name the dimensions: {error}"
        );
    }
}
