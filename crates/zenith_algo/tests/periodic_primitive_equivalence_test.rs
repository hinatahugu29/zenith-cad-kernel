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

/// トーラスのブーリアン等価性（4-816は`example`だけで確かめていた）。
/// 箱がトーラスの一部を貫く配置で、既定の4分割版と体積・面数まで
/// 一致するか。
#[test]
fn torus_periodic_boolean_matches_standard() {
    let tol = Tolerance::default();
    let r_major = 12.0;
    let r_minor = 4.0;
    let standard = PrimitiveBuilder::make_torus(r_major, r_minor).unwrap();
    let periodic = PrimitiveBuilder::make_torus_periodic(r_major, r_minor).unwrap();
    let box_solid = PrimitiveBuilder::make_box(10.0, 10.0, 10.0).unwrap();
    let box_placed =
        BrepTransform::translate_solid(&box_solid, Vec3::new(r_major - 5.0, -5.0, -5.0));

    let a = BooleanEngine::boolean_solids_exact(&box_placed, &standard, BooleanOpType::Difference, &tol)
        .unwrap();
    let b = BooleanEngine::boolean_solids_exact(&box_placed, &periodic, BooleanOpType::Difference, &tol)
        .unwrap();

    assert_eq!(a.outer_shell.faces.len(), b.outer_shell.faces.len());
    let va = MassCalculator::compute_volume_from_brep(&a, &params());
    let vb = MassCalculator::compute_volume_from_brep(&b, &params());
    assert!(
        (va - vb).abs() / va.abs() < 1e-8,
        "standard {va} vs periodic {vb}"
    );
}

/// 円錐のブーリアン等価性（4-816は`example`だけで確かめていた）。
#[test]
fn cone_periodic_boolean_matches_standard() {
    let tol = Tolerance::default();
    let r_bottom = 10.0;
    let height = 20.0;
    let standard = PrimitiveBuilder::make_cone(r_bottom, 0.0, height).unwrap();
    let periodic = PrimitiveBuilder::make_cone_periodic(r_bottom, height).unwrap();
    let box_solid = PrimitiveBuilder::make_box(30.0, 30.0, 30.0).unwrap();
    let box_placed = BrepTransform::translate_solid(&box_solid, Vec3::new(-15.0, -15.0, 5.0));

    let a = BooleanEngine::boolean_solids_exact(&box_placed, &standard, BooleanOpType::Difference, &tol)
        .unwrap();
    let b = BooleanEngine::boolean_solids_exact(&box_placed, &periodic, BooleanOpType::Difference, &tol)
        .unwrap();

    assert_eq!(a.outer_shell.faces.len(), b.outer_shell.faces.len());
    let va = MassCalculator::compute_volume_from_brep(&a, &params());
    let vb = MassCalculator::compute_volume_from_brep(&b, &params());
    assert!(
        (va - vb).abs() / va.abs() < 1e-8,
        "standard {va} vs periodic {vb}"
    );
}

/// 円錐のフィレット等価性（4-816は`example`だけで確かめていた）。
#[test]
fn cone_periodic_fillet_matches_standard() {
    let r_bottom = 10.0;
    let height = 20.0;
    let standard = PrimitiveBuilder::make_cone(r_bottom, 0.0, height).unwrap();
    let periodic = PrimitiveBuilder::make_cone_periodic(r_bottom, height).unwrap();

    let standard_edge = EdgeBlender::blendable_edges(&standard)
        .first()
        .expect("standard cone should have a blendable rim")
        .edge_id;
    let periodic_edge = EdgeBlender::blendable_edges(&periodic)
        .first()
        .expect("periodic cone should have a blendable rim")
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

/// 球（v方向の両端で退化。4-819〜4-820）。自動導出p-curveを手で直した
/// `make_sphere_periodic`が、単独で正しい体積を返すか。
#[test]
fn sphere_periodic_volume_matches_closed_form() {
    let radius = 10.0;
    let solid = PrimitiveBuilder::make_sphere_periodic(radius).unwrap();
    let exact = 4.0 / 3.0 * std::f64::consts::PI * radius.powi(3);
    let measured = MassCalculator::compute_volume_from_brep(&solid, &params());
    assert!(
        (measured - exact).abs() / exact < 1e-9,
        "measured {measured} vs exact {exact}"
    );
}

#[test]
fn sphere_periodic_is_topologically_valid() {
    let solid = PrimitiveBuilder::make_sphere_periodic(10.0).unwrap();
    let tol = Tolerance::default();
    let report = solid.outer_shell.validate_closed(&tol);
    assert!(report.is_valid(), "errors: {:?}", report.errors);
}

/// 表示メッシュが水密になるか（4-820の直し——これが無いと
/// `tessellate_solid`は0三角形を返していた）。
#[test]
fn sphere_periodic_mesh_is_watertight() {
    let solid = PrimitiveBuilder::make_sphere_periodic(10.0).unwrap();
    let mesh = zenith_tess::tessellate_solid(&solid, &params());
    assert!(!mesh.indices.is_empty(), "mesh must not be empty");

    let mut counts: std::collections::BTreeMap<(i64, i64, i64, i64, i64, i64), usize> =
        std::collections::BTreeMap::new();
    let quantize = |p: zenith_math::Point3| {
        let scale = 1e6;
        ((p.x * scale).round() as i64, (p.y * scale).round() as i64, (p.z * scale).round() as i64)
    };
    for triangle in &mesh.indices {
        let points: Vec<_> = triangle.iter().map(|i| quantize(mesh.positions[*i as usize])).collect();
        for corner in 0..3 {
            let a = points[corner];
            let b = points[(corner + 1) % 3];
            let key = if a <= b { (a.0, a.1, a.2, b.0, b.1, b.2) } else { (b.0, b.1, b.2, a.0, a.1, a.2) };
            *counts.entry(key).or_insert(0) += 1;
        }
    }
    let open = counts.values().filter(|c| **c == 1).count();
    let non_manifold = counts.values().filter(|c| **c > 2).count();
    assert_eq!(open, 0, "open edges found");
    assert_eq!(non_manifold, 0, "non-manifold edges found");
}

/// **既知の限界（4-820）**: 球の周期1面版は、ブーリアンではまだ既定の
/// 4分割版と等価にならない——交線で割った面片の縫い直しが「25個の
/// 相手のいない辺」で断られる。この門は、その限界が直ったらテストが
/// 「想定外に通った」ことで気づけるように、**現在の既知の失敗**を
/// 明示的に固定する（1ビットも動かない等価性ではなく、失敗することを
/// 確認する側のテスト）。
#[test]
fn sphere_periodic_boolean_known_limitation() {
    let tol = Tolerance::default();
    let periodic = PrimitiveBuilder::make_sphere_periodic(10.0).unwrap();
    let box_solid = PrimitiveBuilder::make_box(12.0, 12.0, 12.0).unwrap();
    let box_centered = BrepTransform::translate_solid(&box_solid, Vec3::new(-6.0, -6.0, -6.0));
    let result = BooleanEngine::boolean_solids_exact(
        &box_centered,
        &periodic,
        BooleanOpType::Union,
        &tol,
    );
    assert!(
        result.is_err(),
        "sphere_periodic boolean started working — update ROADMAP/HANDOVER and this test"
    );
}
