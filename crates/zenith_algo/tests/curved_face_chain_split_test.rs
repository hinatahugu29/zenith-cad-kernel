//! 曲面に来た交線が**2本の鎖**でも、面を割れること。
//!
//! トーラス片をドリルで抜く配置は、交線が12本出て、A 側の4枚はどれも
//! 割れるのに、**ドリルの側面が1枚も割れません**でした。
//!
//! 曲面同士の交線は相手のパッチの境界で細切れになって届くので、1本ずつでは
//! どれも面の内側で終わります。そのための受け皿（鎖にまとめて当て直す）は
//! ありましたが、**来た稜を丸ごと1本の鎖として**渡していました。
//! ドリルの側面1枚には**出入り2箇所ぶん**の稜が届くので、
//!
//! ```text
//! B1  4 edge(s) -> a cut made of 4 curves has 4 loose ends, not two
//! ```
//!
//! と断られ、面はそのまま残っていました。端の繋がりで鎖に分け、1本ずつ順に
//! 当てるようにしました（平面の経路は前からそうしています）。
//!
//! 期待値は OpenCASCADE です
//! （`occ_cut_reference.py torus_segment drill --box 0 0 -4 16 16 4`）。
//! **ここは桁いっぱいでは合いません。** 切り口が曲面どうしの交線で囲まれる
//! ので、こちらの体積はメッシュから、OCC のは有理 B-spline 上の求積から
//! 出ており、OCC 自身も V(A^B) と removed を 7e-4 違えて報告します。
//! 見たいのは「本当に抜けているか」なので、緩い帯で受けます。
//! 形に依らない恒等式のほうは 4.04e-9 で閉じています。

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

/// 4分の1トーラス。境界箱は (0,0,-4)-(16,16,4)。
fn torus_segment() -> Solid {
    let path = std::path::PathBuf::from(concat!(
        env!("CARGO_MANIFEST_DIR"),
        "/tests/fixtures/occ_reference_torus_segment.step"
    ));
    zenith_io::StepImporter::import_solids_from_file(&path)
        .expect("the fixture must be readable")
        .into_iter()
        .next()
        .expect("one solid")
}

/// `foreign_boolean_probe` と同じドリル。中心 (8, 8) を軸に立てます。
fn centre_drill() -> Solid {
    BrepTransform::translate_solid(
        &PrimitiveBuilder::make_cylinder(2.88, 24.0).expect("drill"),
        Vec3::new(8.0, 8.0, -12.0),
    )
}

const SEGMENT_VOLUME: f64 = 947.482023;
/// OpenCASCADE。上の注のとおり、ここは桁いっぱいでは合いません。
const OCC_DIFFERENCE: f64 = 756.636359;
const OCC_INTERSECTION: f64 = 190.846360;

fn run(op: BooleanOpType) -> Vec<Solid> {
    let tol = Tolerance::default();
    BooleanEngine::boolean_solids_exact_result(&torus_segment(), &centre_drill(), op, &tol)
        .unwrap_or_else(|err| {
            panic!(
                "{op:?} refused: {}",
                err.chars().take(140).collect::<String>()
            )
        })
        .solids
}

#[test]
fn the_drill_goes_through_the_segment() {
    let got = volume(&run(BooleanOpType::Difference));
    assert!(
        (got - SEGMENT_VOLUME).abs() > 1e-6,
        "the difference came back as the untouched segment ({got})"
    );
    let relative = (got - OCC_DIFFERENCE).abs() / OCC_DIFFERENCE;
    assert!(
        relative <= 1e-4,
        "difference {got} against OpenCASCADE's {OCC_DIFFERENCE} (relative {relative:.3e})"
    );
}

#[test]
fn the_matching_intersection_is_the_plug() {
    let got = volume(&run(BooleanOpType::Intersection));
    assert!(
        got > SEGMENT_VOLUME * 1e-3,
        "the intersection came out empty ({got})"
    );
    let relative = (got - OCC_INTERSECTION).abs() / OCC_INTERSECTION;
    assert!(
        relative <= 1e-3,
        "intersection {got} against OpenCASCADE's {OCC_INTERSECTION} (relative {relative:.3e})"
    );
}

/// 形にも物差しにも依らない恒等式。外の値が緩いぶん、ここは厳しく見ます。
#[test]
fn the_plug_and_the_rest_add_back_up() {
    let total = volume(&run(BooleanOpType::Difference)) + volume(&run(BooleanOpType::Intersection));
    let relative = (total - SEGMENT_VOLUME).abs() / SEGMENT_VOLUME;
    assert!(
        relative <= 1e-7,
        "V(A-B) + V(A^B) = {total} against V(A) = {SEGMENT_VOLUME} (relative {relative:.3e})"
    );
}
