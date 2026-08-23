//! 境界が多スパンのスプラインでも、着地点で分けられること。
//!
//! 押し出したスプラインをスラブで半分に切ると、上下の蓋が**割れません**
//! でした。交線は4本とも正しく出ていて、輪も閉じているのに、蓋の片は
//! スプラインを端から端まで（(0,0,0) から (24,6,0) まで）抱えたままです。
//!
//! 断り文はそのまま原因を言っていました。
//!
//! ```text
//! A4 -> Boundary edge is not a single splittable Bezier span
//! ```
//!
//! 境界の稜を着地点で分ける口が `split_bezier_at` しか使っておらず、
//! **内部ノットを持つ曲線を断っていました**。押し出したスプラインの輪郭は
//! まさにそれです。`split_at`（ノット挿入）を後ろに置いて解決しました。
//!
//! 期待値は OpenCASCADE に同じ配置を計算させたものです
//! （`occ_cut_reference.py extruded_spline slab --box 0 -2.2542287165 0 24 20 12`）。

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

/// 断面がスプラインの押し出し。上下の蓋の境界に、多スパンの3次が入ります。
fn extruded_spline() -> Solid {
    let path = std::path::PathBuf::from(concat!(
        env!("CARGO_MANIFEST_DIR"),
        "/tests/fixtures/occ_reference_extruded_spline.step"
    ));
    zenith_io::StepImporter::import_solids_from_file(&path)
        .expect("the fixture must be readable")
        .into_iter()
        .next()
        .expect("one solid")
}

/// `foreign_boolean_probe` と同じスラブ。切断面は x = 11.76 に来ます。
fn half_slab() -> Solid {
    BrepTransform::translate_solid(
        &PrimitiveBuilder::make_box(14.4, 44.5084574330, 24.0).expect("slab"),
        Vec3::new(-2.64, -24.5084574330, -6.0),
    )
}

const SOLID_VOLUME: f64 = 5220.435297;
/// OpenCASCADE。
const OCC_DIFFERENCE: f64 = 2950.632975;
const OCC_INTERSECTION: f64 = 2269.802321;

fn run(op: BooleanOpType) -> Vec<Solid> {
    let tol = Tolerance::default();
    BooleanEngine::boolean_solids_exact_result(&extruded_spline(), &half_slab(), op, &tol)
        .unwrap_or_else(|err| {
            panic!(
                "{op:?} refused: {}",
                err.chars().take(140).collect::<String>()
            )
        })
        .solids
}

#[test]
fn the_slab_cuts_the_extrusion_in_two() {
    let got = volume(&run(BooleanOpType::Difference));
    assert!(
        (got - SOLID_VOLUME).abs() > 1e-6,
        "the difference came back as the untouched solid ({got})"
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
    assert!(
        got > SOLID_VOLUME * 1e-3,
        "the intersection came out empty ({got})"
    );
    let relative = (got - OCC_INTERSECTION).abs() / OCC_INTERSECTION;
    assert!(
        relative <= 1e-5,
        "intersection {got} against OpenCASCADE's {OCC_INTERSECTION} (relative {relative:.3e})"
    );
}

#[test]
fn the_two_halves_add_back_up() {
    let total = volume(&run(BooleanOpType::Difference)) + volume(&run(BooleanOpType::Intersection));
    let relative = (total - SOLID_VOLUME).abs() / SOLID_VOLUME;
    assert!(
        relative <= 1e-9,
        "V(A-B) + V(A^B) = {total} against V(A) = {SOLID_VOLUME} (relative {relative:.3e})"
    );
}
