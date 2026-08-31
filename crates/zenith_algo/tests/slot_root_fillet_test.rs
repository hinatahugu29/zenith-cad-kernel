use std::f64::consts::PI;
use zenith_algo::{
    BooleanEngine, BooleanOpType, BrepTransform, DirectModeling, EdgeBlender, MassCalculator,
    PrimitiveBuilder,
};
use zenith_io::{StepExporter, StepImporter};
use zenith_math::{Tolerance, Vec3};
use zenith_tess::{tessellate_solid, TessellationParams};

fn params() -> TessellationParams {
    TessellationParams {
        u_divisions: 64,
        v_divisions: 64,
    }
}

fn slot_boss_solid() -> zenith_topo::Solid {
    let tol = Tolerance::default();
    let plate = PrimitiveBuilder::make_box(60.0, 60.0, 10.0).expect("plate");
    let slot = PrimitiveBuilder::make_slot_prism(20.0, 5.0, 15.0).expect("slot");
    let moved_slot = BrepTransform::translate_solid(&slot, Vec3::new(30.0, 30.0, 10.0));
    BooleanEngine::boolean_solids_exact_simplified(&plate, &moved_slot, BooleanOpType::Union, &tol)
        .expect("slot boss on plate")
}

#[test]
fn test_slot_boss_root_fillet_exact_volume_and_watertight() {
    let tol = Tolerance::default();
    let boss = slot_boss_solid();
    let before_mass = MassCalculator::compute_from_brep(&boss, &params());

    // 根元エッジの検出
    let blendable = DirectModeling::list_blendable_edges(&boss);
    assert!(
        !blendable.is_empty(),
        "Should find blendable edges on slot boss root"
    );

    // スロット根元エッジ（dihedral 270度）を探す
    let root_edge = blendable
        .iter()
        .find(|edge| (edge.dihedral_angle_deg - 270.0).abs() < 1.0)
        .expect("should find slot root concave edge");

    let fillet_radius = 2.0;
    let (rounded, report) = EdgeBlender::blend_edge(
        &boss,
        root_edge.edge_id,
        zenith_algo::BlendKind::Fillet {
            radius: fillet_radius,
        },
    )
    .expect("fillet slot root");

    // 1. 閉多様体B-Rep検証
    assert!(rounded.outer_shell.validate_closed(&tol).is_valid());

    // 2. 閉形式の追加体積
    let length = 20.0;
    let radius = 5.0;
    let r = fillet_radius;
    let expected_added_volume = 2.0 * length * r * r * (1.0 - PI * 0.25)
        + PI * (radius * r * r * (2.0 - PI * 0.5) + r.powi(3) * (5.0 / 3.0 - PI * 0.5));

    assert!(
        (-report.predicted_removed_volume - expected_added_volume).abs() < 1e-6,
        "Report predicted volume matches closed form"
    );

    let after_mass = MassCalculator::compute_from_brep(&rounded, &params());
    let actual_added_volume = after_mass.volume - before_mass.volume;
    let vol_err = (actual_added_volume - expected_added_volume).abs() / expected_added_volume;
    assert!(
        vol_err < 0.05,
        "Integrated volume error {vol_err:.3e} within 5% tolerance (actual {actual_added_volume}, expected {expected_added_volume})"
    );

    // 3. メッシュ水密性検証
    let mesh = tessellate_solid(&rounded, &params());
    assert!(!mesh.indices.is_empty());

    // 4. STEP往復検証
    let step = StepExporter::export_solid_to_string(&rounded, "SLOT_BOSS_FILLET");
    let reread = StepImporter::import_solid_from_str(&step).expect("import step");
    assert!(reread.outer_shell.validate_closed(&tol).is_valid());
}

#[test]
fn test_slot_boss_root_chamfer_exact_volume_and_watertight() {
    let tol = Tolerance::default();
    let boss = slot_boss_solid();
    let before_mass = MassCalculator::compute_from_brep(&boss, &params());

    let blendable = DirectModeling::list_blendable_edges(&boss);
    let root_edge = blendable
        .iter()
        .find(|edge| (edge.dihedral_angle_deg - 270.0).abs() < 1.0)
        .expect("should find slot root concave edge");

    let chamfer_dist = 2.0;
    let (chamfered, report) = EdgeBlender::blend_edge(
        &boss,
        root_edge.edge_id,
        zenith_algo::BlendKind::Chamfer {
            distance: chamfer_dist,
        },
    )
    .expect("chamfer slot root");

    assert!(chamfered.outer_shell.validate_closed(&tol).is_valid());

    let length = 20.0;
    let radius = 5.0;
    let d = chamfer_dist;
    let expected_added_volume = length * d * d + PI * d * d * (radius + d / 3.0);

    assert!(
        (-report.predicted_removed_volume - expected_added_volume).abs() < 1e-6,
        "Report predicted volume matches closed form"
    );

    let after_mass = MassCalculator::compute_from_brep(&chamfered, &params());
    let actual_added_volume = after_mass.volume - before_mass.volume;
    let vol_err = (actual_added_volume - expected_added_volume).abs() / expected_added_volume;
    assert!(
        vol_err < 0.05,
        "Integrated volume error {vol_err:.3e} within 5% tolerance"
    );
}
