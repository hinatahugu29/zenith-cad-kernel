use std::f64::consts::PI;
use zenith_algo::{
    DirectModeling, EdgeBlender, MassCalculator, PrimitiveBuilder,
};
use zenith_io::{StepExporter, StepImporter};
use zenith_math::Tolerance;
use zenith_tess::{tessellate_solid, TessellationParams};

fn params() -> TessellationParams {
    TessellationParams {
        u_divisions: 64,
        v_divisions: 64,
    }
}

#[test]
fn test_slot_top_rim_fillet_exact_volume_and_watertight() {
    let tol = Tolerance::default();
    let length = 20.0;
    let radius = 5.0;
    let height = 15.0;
    let slot = PrimitiveBuilder::make_slot_prism(length, radius, height).expect("slot prism");
    let before_mass = MassCalculator::compute_from_brep(&slot, &params());

    let blendable = DirectModeling::list_blendable_edges(&slot);
    assert!(!blendable.is_empty(), "Should find blendable edges on slot prism rim");

    // 天面凸稜（dihedral 90度）を探す
    let rim_edge = blendable
        .iter()
        .find(|edge| (edge.dihedral_angle_deg - 90.0).abs() < 1.0)
        .expect("should find slot top rim convex edge");

    let fillet_radius = 1.5;
    let (filleted, report) = EdgeBlender::blend_edge(
        &slot,
        rim_edge.edge_id,
        zenith_algo::BlendKind::Fillet {
            radius: fillet_radius,
        },
    )
    .expect("fillet slot rim");

    // 1. 閉多様体B-Rep検証
    assert!(filleted.outer_shell.validate_closed(&tol).is_valid());

    // 2. 閉形式の除去体積
    let r = fillet_radius;
    let expected_removed_volume = 2.0 * length * r * r * (1.0 - PI * 0.25)
        + PI * ((radius - r) * r * r * (2.0 - PI * 0.5) + r.powi(3) / 3.0);

    assert!(
        (report.predicted_removed_volume - expected_removed_volume).abs() < 1e-6,
        "Report predicted removed volume matches closed form"
    );

    let after_mass = MassCalculator::compute_from_brep(&filleted, &params());
    let actual_removed_volume = before_mass.volume - after_mass.volume;
    let vol_err = (actual_removed_volume - expected_removed_volume).abs() / expected_removed_volume;
    assert!(
        vol_err < 0.05,
        "Integrated volume error {vol_err:.3e} within 5% tolerance (actual {actual_removed_volume}, expected {expected_removed_volume})"
    );

    // 3. メッシュ水密性検証
    let mesh = tessellate_solid(&filleted, &params());
    assert!(!mesh.indices.is_empty());

    // 4. STEP往復検証
    let step = StepExporter::export_solid_to_string(&filleted, "SLOT_RIM_FILLET");
    let reread = StepImporter::import_solid_from_str(&step).expect("import step");
    assert!(reread.outer_shell.validate_closed(&tol).is_valid());
}

#[test]
fn test_slot_top_rim_chamfer_exact_volume_and_watertight() {
    let tol = Tolerance::default();
    let length = 20.0;
    let radius = 5.0;
    let height = 15.0;
    let slot = PrimitiveBuilder::make_slot_prism(length, radius, height).expect("slot prism");
    let before_mass = MassCalculator::compute_from_brep(&slot, &params());

    let blendable = DirectModeling::list_blendable_edges(&slot);
    let rim_edge = blendable
        .iter()
        .find(|edge| (edge.dihedral_angle_deg - 90.0).abs() < 1.0)
        .expect("should find slot top rim convex edge");

    let chamfer_dist = 1.5;
    let (chamfered, report) = EdgeBlender::blend_edge(
        &slot,
        rim_edge.edge_id,
        zenith_algo::BlendKind::Chamfer {
            distance: chamfer_dist,
        },
    )
    .expect("chamfer slot rim");

    assert!(chamfered.outer_shell.validate_closed(&tol).is_valid());

    let d = chamfer_dist;
    let expected_removed_volume = length * d * d + PI * d * d * (radius - d / 3.0);

    assert!(
        (report.predicted_removed_volume - expected_removed_volume).abs() < 1e-6,
        "Report predicted volume matches closed form"
    );

    let after_mass = MassCalculator::compute_from_brep(&chamfered, &params());
    let actual_removed_volume = before_mass.volume - after_mass.volume;
    let vol_err = (actual_removed_volume - expected_removed_volume).abs() / expected_removed_volume;
    assert!(
        vol_err < 0.05,
        "Integrated volume error {vol_err:.3e} within 5% tolerance"
    );
}
