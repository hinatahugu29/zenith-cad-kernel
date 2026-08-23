//! 平面が円錐を斜めに切る交線を、要求した精度まで当てはめること。
//!
//! 全周円錐をスラブで半分に切ると、面の組は3つ見つかるのに**交線は1本**
//! しか出ていませんでした。出ていたのは底面との直線だけで、**側面との
//! 交線が両方とも空**です。
//!
//! 辿るほうは合っていました。辿った点は両方の曲面から 4e-11 しか離れて
//! いません。落ちていたのは**当てはめ**です。刻みを半分にするのを 6 回と
//! 決め打ちにしていて、ずれは
//!
//!   1.26e-1 → 8.38e-2 → 5.09e-2 → 6.53e-3 → 6.20e-4 → 3.53e-5
//!
//! で打ち切られ、要求の 1e-6 に届かないまま「交わらない」と報告されて
//! いました。平面×円錐は双曲線で、3次補間でこの精度を出すには刻みが
//! もっと要ります。**回数ではなく、ずれの減り方で止める**ようにしました。
//!
//! 期待値は OpenCASCADE に同じ配置を計算させたものです
//! （`tools/occ_cut_reference.py cone_full slab`。この検体は OCC の
//! `BoundBox` と実際の境界箱が一致するので、箱の指定は要りません）。

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

/// 底面 r10・高さ 20 の全周円錐。頂点まで閉じているので側面は1枚です。
fn cone() -> Solid {
    let path = std::path::PathBuf::from(concat!(
        env!("CARGO_MANIFEST_DIR"),
        "/tests/fixtures/occ_reference_cone_full.step"
    ));
    zenith_io::StepImporter::import_solids_from_file(&path)
        .expect("the cone fixture must be readable")
        .into_iter()
        .next()
        .expect("one cone")
}

/// `foreign_boolean_probe` と同じスラブ。境界箱 (-10,-10,0)-(10,10,20) に
/// 対する比で置くので、切断面は x = -0.2 に来ます。
fn half_slab() -> Solid {
    BrepTransform::translate_solid(
        &PrimitiveBuilder::make_box(12.0, 40.0, 40.0).expect("slab"),
        Vec3::new(-12.2, -20.0, -10.0),
    )
}

const CONE_VOLUME: f64 = 2094.395102;
/// OpenCASCADE（`tools/occ_cut_reference.py cone_full slab`）。
const OCC_DIFFERENCE: f64 = 1087.168546;
const OCC_INTERSECTION: f64 = 1007.226556;

fn run(op: BooleanOpType) -> Vec<Solid> {
    let tol = Tolerance::default();
    BooleanEngine::boolean_solids_exact_result(&cone(), &half_slab(), op, &tol)
        .unwrap_or_else(|err| {
            panic!(
                "{op:?} refused: {}",
                err.chars().take(140).collect::<String>()
            )
        })
        .solids
}

#[test]
fn the_slab_takes_half_of_the_cone() {
    let got = volume(&run(BooleanOpType::Difference));
    // **元のままでないこと**を先に見ます。交線が出ないと A がそのまま返ります。
    assert!(
        (got - CONE_VOLUME).abs() > 1e-6,
        "the difference came back as the untouched cone ({got})"
    );
    let relative = (got - OCC_DIFFERENCE).abs() / OCC_DIFFERENCE;
    assert!(
        relative <= 1e-5,
        "difference {got} against OpenCASCADE's {OCC_DIFFERENCE} (relative {relative:.3e})"
    );
}

#[test]
fn the_matching_intersection_is_the_other_half() {
    let got = volume(&run(BooleanOpType::Intersection));
    let relative = (got - OCC_INTERSECTION).abs() / OCC_INTERSECTION;
    assert!(
        relative <= 1e-5,
        "intersection {got} against OpenCASCADE's {OCC_INTERSECTION} (relative {relative:.3e})"
    );
}

/// 形に依らない恒等式。**積が 0 でないことも見ます**——0 なら恒等式は
/// 切り手を無視しても成り立ってしまい、何も確かめたことになりません。
#[test]
fn the_two_halves_add_back_up_to_the_cone() {
    let difference = volume(&run(BooleanOpType::Difference));
    let intersection = volume(&run(BooleanOpType::Intersection));
    assert!(
        intersection > CONE_VOLUME * 1e-3,
        "the intersection came out empty ({intersection}); the identity below would hold anyway"
    );
    let total = difference + intersection;
    let relative = (total - CONE_VOLUME).abs() / CONE_VOLUME;
    assert!(
        relative <= 1e-9,
        "V(A-B) + V(A^B) = {total} against V(A) = {CONE_VOLUME} (relative {relative:.3e})"
    );
}
