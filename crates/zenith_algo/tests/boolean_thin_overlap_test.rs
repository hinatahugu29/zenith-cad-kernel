//! 重なりが小さくても、削ること。
//!
//! 差には近道がありました——「中身が重なっていなければ A をそのまま返す」。
//! その判定は共通の境界箱に 512 点を撒いて、両方の内側に入る点を探すだけです。
//!
//! **標本は「重なっている」ことは示せますが、「重なっていない」ことは
//! 示せません。** 円錐の角を箱で削る配置では、重なりは 0.003239 mm^3、
//! 共通の箱は 216 mm^3。当たる確率は 1.5e-5 で、512点の期待値は 0.008 点です。
//! そのため「重なっていない」と判断し、**削らずに A を返していました**。
//!
//! 汎用経路は正しい答え（9面、3267.253121）を作れていたのに、そこへ届く前に
//! 打ち切られていました。近道は最後の受け皿に移してあります。
//!
//! 期待値は OpenCASCADE に同じ配置を計算させたものです
//! （`tools/occ_cut_reference.py cone corner`）。閉じた式が書けない形なので、
//! ここは外の物差しを使います。

use zenith_algo::{BooleanEngine, BooleanOpType, BrepTransform, MassCalculator, PrimitiveBuilder};
use zenith_math::{Tolerance, Vec3};
use zenith_tess::TessellationParams;
use zenith_topo::Solid;

fn params() -> TessellationParams {
    TessellationParams {
        u_divisions: 64,
        v_divisions: 64,
    }
}

fn volume(solids: &[Solid]) -> f64 {
    solids
        .iter()
        .map(|solid| MassCalculator::compute_from_brep(solid, &params()).volume)
        .sum()
}

/// 円錐 r10/r4 h20 と、角に置いた 9x9x9 の箱。重なりは 0.003239 mm^3 しか
/// ありません。
fn cone_and_corner_block() -> (Solid, Solid) {
    let cone = PrimitiveBuilder::make_cone(10.0, 4.0, 20.0).expect("cone");
    let block = BrepTransform::translate_solid(
        &PrimitiveBuilder::make_box(9.0, 9.0, 9.0).expect("block"),
        Vec3::new(4.0, 4.0, 14.0),
    );
    (cone, block)
}

/// OpenCASCADE の値（`tools/occ_cut_reference.py cone corner`）。
const OCC_DIFFERENCE: f64 = 3267.253121;
const OCC_INTERSECTION: f64 = 0.003239;
const CONE_VOLUME: f64 = 3267.256360;

#[test]
fn a_sliver_of_overlap_is_still_removed() {
    let tol = Tolerance::default();
    let (cone, block) = cone_and_corner_block();
    let result =
        BooleanEngine::boolean_solids_exact_result(&cone, &block, BooleanOpType::Difference, &tol)
            .expect("the difference must be produced");
    let got = volume(&result.solids);

    // **元のままでないこと**を先に見ます。これが近道の症状でした。
    assert!(
        (got - CONE_VOLUME).abs() > 1e-6,
        "the difference came back as the untouched cone ({got}); nothing was removed"
    );

    let relative = (got - OCC_DIFFERENCE).abs() / OCC_DIFFERENCE;
    assert!(
        relative <= 1e-6,
        "difference {got} against OpenCASCADE's {OCC_DIFFERENCE} (relative {relative:.3e})"
    );
}

/// 同じ配置の積。こちらは近道を通らないので、前から合っていました。
/// **両方を見るのは、片方だけ直しても恒等式が破れたままだからです。**
#[test]
fn the_matching_intersection_is_the_sliver() {
    let tol = Tolerance::default();
    let (cone, block) = cone_and_corner_block();
    let result =
        BooleanEngine::boolean_solids_exact_result(&cone, &block, BooleanOpType::Intersection, &tol)
            .expect("the intersection must be produced");
    let got = volume(&result.solids);
    let relative = (got - OCC_INTERSECTION).abs() / OCC_INTERSECTION;
    assert!(
        relative <= 1e-3,
        "intersection {got} against OpenCASCADE's {OCC_INTERSECTION} (relative {relative:.3e})"
    );
}

/// 差と積が足して元に戻ること。**形に依らない恒等式**で、切った形の閉じた式を
/// 書かずに済みます。近道が効いていたときは、ここが 9.91e-7 で破れていました。
#[test]
fn the_two_halves_add_back_up_to_the_cone() {
    let tol = Tolerance::default();
    let (cone, block) = cone_and_corner_block();
    let difference =
        BooleanEngine::boolean_solids_exact_result(&cone, &block, BooleanOpType::Difference, &tol)
            .expect("difference");
    let intersection =
        BooleanEngine::boolean_solids_exact_result(&cone, &block, BooleanOpType::Intersection, &tol)
            .expect("intersection");

    let total = volume(&difference.solids) + volume(&intersection.solids);
    let relative = (total - CONE_VOLUME).abs() / CONE_VOLUME;
    assert!(
        relative <= 1e-9,
        "V(A-B) + V(A^B) = {total} against V(A) = {CONE_VOLUME} (relative {relative:.3e})"
    );
}

/// 本当に触れていない相手は、いままでどおり A をそのまま返すこと。
/// **近道を後ろに下げたことで、その働きまで失っていないか**を押さえます。
#[test]
fn a_block_that_misses_entirely_leaves_the_cone_alone() {
    let tol = Tolerance::default();
    let (cone, _) = cone_and_corner_block();
    let far = BrepTransform::translate_solid(
        &PrimitiveBuilder::make_box(9.0, 9.0, 9.0).expect("block"),
        Vec3::new(50.0, 50.0, 50.0),
    );
    let result = BooleanEngine::boolean_solids_exact_result(
        &cone,
        &far,
        BooleanOpType::Difference,
        &tol,
    )
    .expect("a difference with a solid that is nowhere near must succeed");
    let got = volume(&result.solids);
    let relative = (got - CONE_VOLUME).abs() / CONE_VOLUME;
    assert!(
        relative <= 1e-9,
        "a block that misses the cone changed it: {got} against {CONE_VOLUME}"
    );
}
