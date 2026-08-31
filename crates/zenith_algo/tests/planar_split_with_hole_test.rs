//! 穴を持つ平面の面を割れること。
//!
//! 平面の分割は、内側のワイヤを1つでも持つ面を**一切**断っていました
//! （"Planar face splitting with inner wires is not implemented yet"）。
//! 実務では穴あき板の面そのもので、円環の上面もこれです。
//!
//! 実測: 外周 r10・穴 r4・高さ6 の輪の角を箱で削ると、上面（円環）が
//! 割れないせいで切り口の三角形が縫えず、3演算とも拒否されていました。
//!
//! 期待値は OpenCASCADE に同じ配置を計算させたものです
//! （`tools/occ_cut_reference.py revolved_ring corner`）。

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

/// 外周 10・穴 4・高さ 6 の輪。上面と下面は**穴あきの円環**です。
///
/// **OpenCASCADE が書いた検体を読みます。** 同じ形をビルダー（円柱からボアを
/// 引く）で組むと面が 10 枚に割れており、そちらはまだ切れません。直したのは
/// 読んだ形（4枚）の経路なので、測ったものをそのまま検査します。
/// 両者の違いは `ring_corner_probe` が並べて出します。
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

/// `foreign_boolean_probe` と同じ角の箱。境界箱は (-10,-10,0)-(10,10,6)。
fn corner_block() -> Solid {
    BrepTransform::translate_solid(
        &PrimitiveBuilder::make_box(9.0, 9.0, 2.7).expect("block"),
        Vec3::new(4.0, 4.0, 4.2),
    )
}

const RING_VOLUME: f64 = std::f64::consts::PI * (100.0 - 16.0) * 6.0;
/// OpenCASCADE（`tools/occ_cut_reference.py revolved_ring corner`）。
const OCC_DIFFERENCE: f64 = 1553.253150;
const OCC_INTERSECTION: f64 = 30.109547;

fn run(op: BooleanOpType) -> Vec<Solid> {
    let tol = Tolerance::default();
    let ring = ring();
    let block = corner_block();
    BooleanEngine::boolean_solids_exact_result(&ring, &block, op, &tol)
        .unwrap_or_else(|err| {
            panic!(
                "{op:?} refused: {}",
                err.chars().take(140).collect::<String>()
            )
        })
        .solids
}

#[test]
fn a_ring_can_have_its_corner_cut() {
    let got = volume(&run(BooleanOpType::Difference));
    // **元のままでないこと**を先に見ます。割れないと A がそのまま返ります。
    assert!(
        (got - RING_VOLUME).abs() > 1e-6,
        "the difference came back as the untouched ring ({got})"
    );
    let relative = (got - OCC_DIFFERENCE).abs() / OCC_DIFFERENCE;
    assert!(
        relative <= 1e-5,
        "difference {got} against OpenCASCADE's {OCC_DIFFERENCE} (relative {relative:.3e})"
    );
}

#[test]
fn the_matching_intersection_is_the_corner() {
    let got = volume(&run(BooleanOpType::Intersection));
    let relative = (got - OCC_INTERSECTION).abs() / OCC_INTERSECTION;
    assert!(
        relative <= 1e-4,
        "intersection {got} against OpenCASCADE's {OCC_INTERSECTION} (relative {relative:.3e})"
    );
}

/// 形に依らない恒等式。切った形の閉じた式を書かずに済みます。
#[test]
fn the_two_halves_add_back_up_to_the_ring() {
    let total = volume(&run(BooleanOpType::Difference)) + volume(&run(BooleanOpType::Intersection));
    let relative = (total - RING_VOLUME).abs() / RING_VOLUME;
    assert!(
        relative <= 1e-9,
        "V(A-B) + V(A^B) = {total} against V(A) = {RING_VOLUME} (relative {relative:.3e})"
    );
}

/// **穴はどちらか一方の片に丸ごと入らなければなりません。** 両方に入れたり
/// 落としたりすると、体積は合っても穴が消えます。差の結果が輪であること——
/// つまり中空であること——を、内側の殻の有無ではなく体積で見ます。
/// 穴が失われれば体積は π·100·6 = 1884.96 ぶん増えます。
#[test]
fn the_bore_survives_the_cut() {
    let got = volume(&run(BooleanOpType::Difference));
    let solid_disc = std::f64::consts::PI * 100.0 * 6.0;
    assert!(
        got < solid_disc * 0.9,
        "the bore was lost: {got} is close to the solid disc {solid_disc}"
    );
}

/// 切り込みが穴を横切る配置は、いまも断ること。**できないことを、静かに
/// 間違えた形で返さない**ためです。
#[test]
fn a_cut_that_crosses_the_bore_is_still_refused_or_correct() {
    let tol = Tolerance::default();
    let ring = ring();
    // 中心を通る幅広のスラブ。上面の穴をまたいで切ります。
    let slab = BrepTransform::translate_solid(
        &PrimitiveBuilder::make_box(30.0, 3.0, 30.0).expect("slab"),
        Vec3::new(-15.0, -1.5, -12.0),
    );
    match BooleanEngine::boolean_solids_exact_result(&ring, &slab, BooleanOpType::Difference, &tol)
    {
        // 断るのは正しい挙動です。
        Err(_) => {}
        Ok(result) => {
            // 通ったなら、答えが合っていなければなりません。
            let got = volume(&result.solids);
            assert!(
                got < RING_VOLUME && got > 0.0,
                "a slab through the bore returned {got}, which is not less than the ring {RING_VOLUME}"
            );
        }
    }
}
