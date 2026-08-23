//! 全周トーラスを半分のスラブで切ること。
//!
//! **恒等式が見られない誤答**でした。答えはトーラスそのまま（切り手を無視）
//! で、そのとき V(A-B) = V(A)、V(A^B) = 0、V(AuB) = V(A)+V(B) となり、
//! 2つの恒等式は**どちらも成り立ちます**。プローブは残差 2.44e-12 の
//! 「ok」と報告していました。
//!
//! 外の物差しが決めました。OpenCASCADE は同じ箱で 1862.79 削ります
//! （`tools/occ_cut_reference.py torus slab --box -16 -16 -4 16 16 4`）。
//!
//! **箱を指定するのは、OCC の `BoundBox` が密着していないからです。**
//! 真の境界箱は (-16,-16,-4)-(16,16,4) ですが、OCC は (-17.3183, ...) と
//! 返します。切り手は箱に対する比で置くので、そのままだと別の配置を測る
//! ことになります（`cutter_placement_probe` が使う箱を出します）。
//!
//! 原因は平面×トーラス面の交線が1本も出ていなかったこと。当てはめの刻みを
//! 6回で打ち切っていたためで、`plane_cone_section_test` と同じ機構です。

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

/// 主半径 12・管半径 4 の全周トーラス。境界箱は (-16,-16,-4)-(16,16,4)。
fn torus() -> Solid {
    let path = std::path::PathBuf::from(concat!(
        env!("CARGO_MANIFEST_DIR"),
        "/tests/fixtures/occ_reference_torus.step"
    ));
    zenith_io::StepImporter::import_solids_from_file(&path)
        .expect("the torus fixture must be readable")
        .into_iter()
        .next()
        .expect("one torus")
}

/// `foreign_boolean_probe` と同じスラブ。切断面は x = -0.32 に来ます。
fn half_slab() -> Solid {
    BrepTransform::translate_solid(
        &PrimitiveBuilder::make_box(19.2, 64.0, 16.0).expect("slab"),
        Vec3::new(-19.52, -32.0, -8.0),
    )
}

const TORUS_VOLUME: f64 = 3789.928090;
/// OpenCASCADE（`occ_cut_reference.py torus slab --box -16 -16 -4 16 16 4`）。
const OCC_DIFFERENCE: f64 = 1927.137910;
const OCC_INTERSECTION: f64 = 1862.789475;

fn run(op: BooleanOpType) -> Vec<Solid> {
    let tol = Tolerance::default();
    BooleanEngine::boolean_solids_exact_result(&torus(), &half_slab(), op, &tol)
        .unwrap_or_else(|err| {
            panic!(
                "{op:?} refused: {}",
                err.chars().take(140).collect::<String>()
            )
        })
        .solids
}

#[test]
fn the_slab_really_removes_material() {
    let got = volume(&run(BooleanOpType::Difference));
    // **これが症状でした。** 恒等式では捕まりません。
    assert!(
        (got - TORUS_VOLUME).abs() > 1e-6,
        "the difference came back as the untouched torus ({got}); nothing was removed"
    );
    let relative = (got - OCC_DIFFERENCE).abs() / OCC_DIFFERENCE;
    assert!(
        relative <= 1e-5,
        "difference {got} against OpenCASCADE's {OCC_DIFFERENCE} (relative {relative:.3e})"
    );
}

#[test]
fn the_matching_intersection_is_the_other_side() {
    let got = volume(&run(BooleanOpType::Intersection));
    // 0 でないことを先に見ます。0 だと下の恒等式は切り手を無視しても
    // 成り立ってしまいます。
    assert!(
        got > TORUS_VOLUME * 1e-3,
        "the intersection came out empty ({got})"
    );
    let relative = (got - OCC_INTERSECTION).abs() / OCC_INTERSECTION;
    assert!(
        relative <= 1e-5,
        "intersection {got} against OpenCASCADE's {OCC_INTERSECTION} (relative {relative:.3e})"
    );
}

#[test]
fn the_two_sides_add_back_up_to_the_torus() {
    let total = volume(&run(BooleanOpType::Difference)) + volume(&run(BooleanOpType::Intersection));
    let relative = (total - TORUS_VOLUME).abs() / TORUS_VOLUME;
    assert!(
        relative <= 1e-9,
        "V(A-B) + V(A^B) = {total} against V(A) = {TORUS_VOLUME} (relative {relative:.3e})"
    );
}
