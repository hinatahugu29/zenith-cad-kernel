//! 切り込みが**穴を横切る**平面の面を割れること。
//!
//! 輪をスラブで半分に切る配置がこれです。上下面は円環で、切り込みは外周から
//! 穴まで走るので、片方の端が**内側のワイヤ**に乗ります。それまでの分割は
//! 端を外側のワイヤにしか探さず、「Split edge end does not lie on the outer
//! boundary」で断っていました。
//!
//! 穴が片方の片に丸ごと入る配置は 4-54 で扱えるようになりましたが、
//! **穴自体が割れる**配置はそこでも扱えません。uv の平面アレンジメントを
//! 最後の受け皿として組み、外側・内側すべてのワイヤを着地点で細分してから
//! 面を辿るようにしました。
//!
//! そこまで進めると、今度は**穴の壁の断片が裏返っている**ことが見えました
//! （同じ向きに2度使われた稜が16本）。円柱側面の分割器が断片の輪を決め打ちの
//! 順で組んでいて、元の面がどちら巻きかを見ていませんでした。輪の穴の壁は
//! 裏向きの面なので、そこだけ裏返ります。
//!
//! 期待値は OpenCASCADE です
//! （`occ_cut_reference.py revolved_ring slab --box -10 -10 0 10 10 6`）。

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

/// 外周 10・穴 4・高さ 6 の輪。上下面は**穴あきの円環**です。
fn ring() -> Solid {
    let path = std::path::PathBuf::from(concat!(
        env!("CARGO_MANIFEST_DIR"),
        "/tests/fixtures/occ_reference_revolved_ring.step"
    ));
    zenith_io::StepImporter::import_solids_from_file(&path)
        .expect("the ring fixture must be readable")
        .into_iter()
        .next()
        .expect("one ring")
}

/// `foreign_boolean_probe` と同じスラブ。切断面は x = -0.2 で、**穴を
/// 横切ります**。
fn half_slab() -> Solid {
    BrepTransform::translate_solid(
        &PrimitiveBuilder::make_box(12.0, 40.0, 12.0).expect("slab"),
        Vec3::new(-12.2, -20.0, -3.0),
    )
}

const RING_VOLUME: f64 = 1583.362697;
/// OpenCASCADE。
const OCC_DIFFERENCE: f64 = 806.083750;
const OCC_INTERSECTION: f64 = 777.278947;

fn run(op: BooleanOpType) -> Vec<Solid> {
    let tol = Tolerance::default();
    BooleanEngine::boolean_solids_exact_result(&ring(), &half_slab(), op, &tol)
        .unwrap_or_else(|err| {
            panic!(
                "{op:?} refused: {}",
                err.chars().take(140).collect::<String>()
            )
        })
        .solids
}

#[test]
fn the_slab_takes_half_the_ring() {
    let got = volume(&run(BooleanOpType::Difference));
    assert!(
        (got - RING_VOLUME).abs() > 1e-6,
        "the difference came back as the untouched ring ({got})"
    );
    let relative = (got - OCC_DIFFERENCE).abs() / OCC_DIFFERENCE;
    assert!(
        relative <= 1e-6,
        "difference {got} against OpenCASCADE's {OCC_DIFFERENCE} (relative {relative:.3e})"
    );
}

#[test]
fn the_matching_intersection_is_the_other_half() {
    let got = volume(&run(BooleanOpType::Intersection));
    assert!(
        got > RING_VOLUME * 1e-3,
        "the intersection came out empty ({got})"
    );
    let relative = (got - OCC_INTERSECTION).abs() / OCC_INTERSECTION;
    assert!(
        relative <= 1e-6,
        "intersection {got} against OpenCASCADE's {OCC_INTERSECTION} (relative {relative:.3e})"
    );
}

#[test]
fn the_two_halves_add_back_up_to_the_ring() {
    let total = volume(&run(BooleanOpType::Difference)) + volume(&run(BooleanOpType::Intersection));
    let relative = (total - RING_VOLUME).abs() / RING_VOLUME;
    assert!(
        relative <= 1e-9,
        "V(A-B) + V(A^B) = {total} against V(A) = {RING_VOLUME} (relative {relative:.3e})"
    );
}

/// **穴の壁が残っていること。** 円環を割ったあとも、内側の面は結果の一部です。
/// 失われれば体積は穴のぶん（半分だけ π·16·6/2 = 150.80）増えます。
#[test]
fn the_bore_wall_survives_the_cut() {
    let got = volume(&run(BooleanOpType::Difference));
    let without_the_bore = std::f64::consts::PI * 100.0 * 6.0 * 0.5;
    assert!(
        got < without_the_bore * 0.95,
        "the bore was lost: {got} is close to the solid half disc {without_the_bore}"
    );
}
