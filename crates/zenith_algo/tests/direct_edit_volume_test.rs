//! ダイレクト編集の結果が、閉じた式に乗るか。
//!
//! これらの操作は「体積がいくつ変わるか」が幾何から一意に決まります。
//! 面を押し引きすれば `面積 x 距離`、1本の縦稜を半径 r で丸めれば断面が
//! `(1 - pi/4) r^2` だけ減り、`c` で面取りすれば `c^2 / 2` だけ減ります。
//!
//! 出来上がりが閉じたソリッドであることは、これまでも見られていました。
//! **値が正しいか**は見られていなかったので、ここで測ります。

use std::f64::consts::FRAC_PI_4;

use zenith_algo::{DirectModeling, MassCalculator, PrimitiveBuilder};
use zenith_math::Tolerance;
use zenith_tess::TessellationParams;

fn volume_of(solid: &zenith_topo::Solid) -> f64 {
    MassCalculator::compute_from_brep(
        solid,
        &TessellationParams {
            u_divisions: 48,
            v_divisions: 48,
        },
    )
    .volume
}

#[test]
fn pushing_a_face_changes_the_volume_by_its_area_times_the_distance() {
    let tol = Tolerance::default();
    let boxed = PrimitiveBuilder::make_box(20.0, 30.0, 40.0).unwrap();
    let base = volume_of(&boxed);
    assert!((base - 24000.0).abs() < 1e-9);

    // 面 0 は 20 x 30 = 600 の面。押しても引いても、その面積ぶん変わる。
    for distance in [5.0, -5.0, 12.5, -1.0] {
        let moved = DirectModeling::push_pull_face(&boxed, 0, distance)
            .unwrap_or_else(|err| panic!("pushing by {distance}: {err}"));

        let report = moved.outer_shell.validate_closed(&tol);
        assert!(
            report.is_valid(),
            "pushing by {distance} left an invalid shell: {:?}",
            report.errors
        );

        let expected = base + 600.0 * distance;
        let volume = volume_of(&moved);
        assert!(
            (volume - expected).abs() / expected < 1e-12,
            "pushing by {distance}: {volume} against {expected}"
        );
    }
}

#[test]
fn filleting_one_edge_removes_the_corner_the_closed_form_says() {
    let tol = Tolerance::default();
    for radius in [1.0f64, 2.0, 4.0, 7.5] {
        let solid = DirectModeling::fillet_box_single_edge(20.0, 30.0, 40.0, 0, radius)
            .unwrap_or_else(|err| panic!("filleting by {radius}: {err}"));

        let report = solid.outer_shell.validate_closed(&tol);
        assert!(report.is_valid(), "{:?}", report.errors);
        assert_eq!(
            solid.outer_shell.faces.len(),
            7,
            "rounding one edge turns six faces into seven"
        );

        // 縦稜を1本だけ丸めると、断面から (1 - pi/4) r^2 が落ちる。
        let expected = (20.0 * 30.0 - (1.0 - FRAC_PI_4) * radius * radius) * 40.0;
        let volume = volume_of(&solid);
        assert!(
            (volume - expected).abs() / expected < 1e-12,
            "fillet r{radius}: {volume} against {expected}"
        );
    }
}

#[test]
fn chamfering_one_edge_removes_the_triangle_the_closed_form_says() {
    let tol = Tolerance::default();
    for distance in [1.0f64, 2.0, 4.0, 9.0] {
        let solid = DirectModeling::chamfer_box_single_edge(20.0, 30.0, 40.0, 0, distance)
            .unwrap_or_else(|err| panic!("chamfering by {distance}: {err}"));

        let report = solid.outer_shell.validate_closed(&tol);
        assert!(report.is_valid(), "{:?}", report.errors);

        // 断面から一辺 c の直角二等辺三角形が落ちる。
        let expected = (20.0 * 30.0 - 0.5 * distance * distance) * 40.0;
        let volume = volume_of(&solid);
        assert!(
            (volume - expected).abs() / expected < 1e-12,
            "chamfer c{distance}: {volume} against {expected}"
        );
    }
}

/// 丸めのほうが面取りより残る。同じ寸法なら、弧は弦より外にあるからである。
/// 大きさだけでなく**向き**を見ておくと、符号を取り違えたときに気づく。
#[test]
fn a_fillet_leaves_more_material_than_a_chamfer_of_the_same_size() {
    for size in [2.0f64, 4.0, 6.0] {
        let filleted = volume_of(
            &DirectModeling::fillet_box_single_edge(20.0, 30.0, 40.0, 0, size).expect("fillet"),
        );
        let chamfered = volume_of(
            &DirectModeling::chamfer_box_single_edge(20.0, 30.0, 40.0, 0, size).expect("chamfer"),
        );
        assert!(
            filleted > chamfered,
            "at size {size} the fillet {filleted} should keep more than the chamfer {chamfered}"
        );
    }
}
