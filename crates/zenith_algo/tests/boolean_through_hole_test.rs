//! 穴を素通りするだけの相手を、交わっているものとして扱わないこと。
//!
//! 面のトリムを外周ループだけで見ていたあいだ、**穴の中も面の一部**でした。
//! 円環の平らなキャップに、穴を通り抜ける丸棒の側面が作る円は、外周の内側に
//! あるので「面の上の交線」として通ります。実体としては触れていないのに、
//! 交線が8本立ちました。
//!
//! そうなると答えは「もっともらしい拒否」になります。積は空、和は2立体、
//! 差は元のまま——**どれも切る仕事の要らない自明な答え**なのに、返せません。
//!
//! ここは自前のビルダーで組みます。同じ配置を OpenCASCADE にも計算させて
//! ありますが（`tools/occ_cut_reference.py`）、期待値は閉じた式です。

use zenith_algo::{BooleanEngine, BooleanOpType, BrepTransform, MassCalculator, PrimitiveBuilder};
use zenith_math::{Tolerance, Vec3};
use zenith_tess::TessellationParams;
use zenith_topo::Solid;

fn params() -> TessellationParams {
    TessellationParams {
        u_divisions: 48,
        v_divisions: 48,
    }
}

fn volume(solids: &[Solid]) -> f64 {
    solids
        .iter()
        .map(|solid| MassCalculator::compute_from_brep(solid, &params()).volume)
        .sum()
}

/// 外半径 10・内半径 4・高さ 6 の円環と、それより細い丸棒を同軸に置く。
/// 棒は穴を素通りするので、**どこにも触れていません**。
fn ring_and_rod() -> (Solid, Solid, f64, f64) {
    let tol = Tolerance::default();
    let outer = PrimitiveBuilder::make_cylinder(10.0, 6.0).expect("outer cylinder");
    let bore = BrepTransform::translate_solid(
        &PrimitiveBuilder::make_cylinder(4.0, 30.0).expect("bore"),
        Vec3::new(0.0, 0.0, -12.0),
    );
    let ring =
        BooleanEngine::boolean_solids_exact_result(&outer, &bore, BooleanOpType::Difference, &tol)
            .expect("the ring itself must build")
            .solids
            .into_iter()
            .next()
            .expect("one ring");

    // 半径 3.6 < 4 なので、穴の壁にも届きません。
    let rod = BrepTransform::translate_solid(
        &PrimitiveBuilder::make_cylinder(3.6, 30.0).expect("rod"),
        Vec3::new(0.0, 0.0, -12.0),
    );

    let ring_volume = std::f64::consts::PI * (100.0 - 16.0) * 6.0;
    let rod_volume = std::f64::consts::PI * 3.6 * 3.6 * 30.0;
    (ring, rod, ring_volume, rod_volume)
}

fn run(a: &Solid, b: &Solid, op: BooleanOpType) -> Vec<Solid> {
    BooleanEngine::boolean_solids_exact_result(a, b, op, &Tolerance::default())
        .unwrap_or_else(|err| {
            panic!(
                "{op:?} refused: {}",
                err.chars().take(120).collect::<String>()
            )
        })
        .solids
}

#[test]
fn a_rod_through_the_bore_removes_nothing() {
    let (ring, rod, ring_volume, _) = ring_and_rod();
    let solids = run(&ring, &rod, BooleanOpType::Difference);
    let got = volume(&solids);
    let relative = (got - ring_volume).abs() / ring_volume;
    assert!(
        relative <= 1e-9,
        "difference moved the ring: {got} against {ring_volume} (relative {relative:.3e})"
    );
}

#[test]
fn a_rod_through_the_bore_meets_nothing() {
    let (ring, rod, _, _) = ring_and_rod();
    let solids = run(&ring, &rod, BooleanOpType::Intersection);
    let got = volume(&solids);
    assert!(
        got.abs() <= 1e-9,
        "intersection of two solids that do not touch came back as {got}"
    );
}

/// 和は**2つの立体**です。合計だけ見ていると、1つに融合してしまっても
/// 気づけません。
#[test]
fn the_union_is_two_separate_solids() {
    let (ring, rod, ring_volume, rod_volume) = ring_and_rod();
    let solids = run(&ring, &rod, BooleanOpType::Union);
    assert_eq!(
        solids.len(),
        2,
        "two solids that do not touch fused into {} solid(s)",
        solids.len()
    );
    let got = volume(&solids);
    let want = ring_volume + rod_volume;
    let relative = (got - want).abs() / want;
    assert!(
        relative <= 1e-9,
        "union volume {got} against {want} (relative {relative:.3e})"
    );
}

/// 穴の壁に本当に届く棒は、いままでどおり切れること。**穴を外したことで
/// 「穴のそばは全部無視」になっていないか**を押さえます。
#[test]
fn a_rod_that_does_reach_the_bore_wall_still_cuts() {
    let tol = Tolerance::default();
    let (ring, _, ring_volume, _) = ring_and_rod();
    // 半径 6 は内半径 4 を越えて外半径 10 には届かないので、輪の内側を削ります。
    let fat = BrepTransform::translate_solid(
        &PrimitiveBuilder::make_cylinder(6.0, 30.0).expect("fat rod"),
        Vec3::new(0.0, 0.0, -12.0),
    );
    let result =
        BooleanEngine::boolean_solids_exact_result(&ring, &fat, BooleanOpType::Difference, &tol);
    let Ok(result) = result else {
        // 断ることは誤答よりましなので、ここは通らなくても赤にしません。
        // 見張っているのは「静かに何も削らない」ことです。
        return;
    };
    let got = volume(&result.solids);
    let want = std::f64::consts::PI * (100.0 - 36.0) * 6.0;
    let relative = (got - want).abs() / want;
    assert!(
        relative <= 1e-6,
        "a rod that reaches the wall should have cut: {got} against {want} \
         (relative {relative:.3e}); the untouched ring is {ring_volume}"
    );
}
