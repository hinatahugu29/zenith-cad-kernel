//! **ROADMAP柱③の周期1面化実験を、恒久的なテストとして固定する**
//! （HANDOVER 4-807〜4-817）。
//!
//! これまでの検証（`periodic_*_probe`の一連の例）は`cargo run --example`
//! でしか動かず、`cargo test --workspace`では1本も走らない——今後
//! `zenith_geom`・`zenith_topo`・`zenith_algo`の内部が変わったときに、
//! この等価性が壊れても誰にも気づかれない。**ここで、確かめたことの
//! 核だけを`#[test]`として固定する。**
//!
//! `PrimitiveBuilder::make_cylinder_periodic`・`make_torus_periodic`・
//! `make_cone_periodic`は**まだ既定の`make_cylinder`等を置き換えていない**
//! ——この門は「周期1面化の選択肢が、既定の4分割と等価であり続けるか」
//! だけを見張る。既定を切り替えるかどうかの判断とは別。

use zenith_algo::{BooleanEngine, BooleanOpType, BrepTransform, EdgeBlender, MassCalculator, PrimitiveBuilder};
use zenith_math::{Tolerance, Vec3};
use zenith_tess::TessellationParams;

fn params() -> TessellationParams {
    TessellationParams { u_divisions: 48, v_divisions: 48 }
}

#[test]
fn cylinder_periodic_volume_matches_closed_form() {
    let radius = 10.0;
    let height = 20.0;
    let solid = PrimitiveBuilder::make_cylinder_periodic(radius, height).unwrap();
    let exact = std::f64::consts::PI * radius * radius * height;
    let measured = MassCalculator::compute_volume_from_brep(&solid, &params());
    assert!(
        (measured - exact).abs() / exact < 1e-9,
        "measured {measured} vs exact {exact}"
    );
}

#[test]
fn torus_periodic_volume_matches_closed_form() {
    let r_major = 12.0;
    let r_minor = 4.0;
    let solid = PrimitiveBuilder::make_torus_periodic(r_major, r_minor).unwrap();
    let exact = 2.0 * std::f64::consts::PI.powi(2) * r_major * r_minor * r_minor;
    let measured = MassCalculator::compute_volume_from_brep(&solid, &params());
    assert!(
        (measured - exact).abs() / exact < 1e-9,
        "measured {measured} vs exact {exact}"
    );
}

#[test]
fn cone_periodic_volume_matches_closed_form() {
    let r_bottom = 10.0;
    let height = 20.0;
    let solid = PrimitiveBuilder::make_cone_periodic(r_bottom, height).unwrap();
    let exact = std::f64::consts::PI * r_bottom * r_bottom * height / 3.0;
    let measured = MassCalculator::compute_volume_from_brep(&solid, &params());
    assert!(
        (measured - exact).abs() / exact < 1e-9,
        "measured {measured} vs exact {exact}"
    );
}

/// 円柱の周期1面版は、`validate_closed`に受理される
/// （4-808。`is_valid()`がfalseを返したことがある——エラーの中身を
/// `errors`で見られるようにしておく）。
#[test]
fn cylinder_periodic_is_topologically_valid() {
    let solid = PrimitiveBuilder::make_cylinder_periodic(10.0, 20.0).unwrap();
    let tol = Tolerance::default();
    let report = solid.outer_shell.validate_closed(&tol);
    assert!(report.is_valid(), "errors: {:?}", report.errors);
}

#[test]
fn torus_periodic_is_topologically_valid() {
    let solid = PrimitiveBuilder::make_torus_periodic(12.0, 4.0).unwrap();
    let tol = Tolerance::default();
    let report = solid.outer_shell.validate_closed(&tol);
    assert!(report.is_valid(), "errors: {:?}", report.errors);
}

#[test]
fn cone_periodic_is_topologically_valid() {
    let solid = PrimitiveBuilder::make_cone_periodic(10.0, 20.0).unwrap();
    let tol = Tolerance::default();
    let report = solid.outer_shell.validate_closed(&tol);
    assert!(report.is_valid(), "errors: {:?}", report.errors);
}

/// ブーリアン（箱×円柱の差）が、既定の4分割版と体積・面数まで一致する
/// （4-815）。
#[test]
fn cylinder_periodic_boolean_matches_standard() {
    let tol = Tolerance::default();
    let radius = 10.0;
    let height = 20.0;
    let standard = PrimitiveBuilder::make_cylinder(radius, height).unwrap();
    let periodic = PrimitiveBuilder::make_cylinder_periodic(radius, height).unwrap();
    let box_solid = PrimitiveBuilder::make_box(30.0, 30.0, 30.0).unwrap();
    let box_centered = BrepTransform::translate_solid(&box_solid, Vec3::new(-15.0, -15.0, 5.0));

    let a = BooleanEngine::boolean_solids_exact(&box_centered, &standard, BooleanOpType::Difference, &tol)
        .unwrap();
    let b = BooleanEngine::boolean_solids_exact(&box_centered, &periodic, BooleanOpType::Difference, &tol)
        .unwrap();

    assert_eq!(a.outer_shell.faces.len(), b.outer_shell.faces.len());
    let va = MassCalculator::compute_volume_from_brep(&a, &params());
    let vb = MassCalculator::compute_volume_from_brep(&b, &params());
    assert!(
        (va - vb).abs() / va.abs() < 1e-9,
        "standard {va} vs periodic {vb}"
    );
}

/// フィレット（底縁、半径1.0）が、既定の4分割版と除去体積まで一致する
/// （4-814）。
#[test]
fn cylinder_periodic_fillet_matches_standard() {
    let radius = 10.0;
    let height = 20.0;
    let standard = PrimitiveBuilder::make_cylinder(radius, height).unwrap();
    let periodic = PrimitiveBuilder::make_cylinder_periodic(radius, height).unwrap();

    let standard_edge = EdgeBlender::blendable_edges(&standard)
        .first()
        .expect("standard cylinder should have a blendable rim")
        .edge_id;
    let periodic_edge = EdgeBlender::blendable_edges(&periodic)
        .first()
        .expect("periodic cylinder should have a blendable rim")
        .edge_id;

    let a = EdgeBlender::fillet_edge(&standard, standard_edge, 1.0).unwrap();
    let b = EdgeBlender::fillet_edge(&periodic, periodic_edge, 1.0).unwrap();

    let va = MassCalculator::compute_volume_from_brep(&a, &params());
    let vb = MassCalculator::compute_volume_from_brep(&b, &params());
    assert!(
        (va - vb).abs() / va.abs() < 1e-9,
        "standard {va} vs periodic {vb}"
    );
}
