use std::f64::consts::PI;

use zenith_algo::{
    BooleanEngine, BooleanOpType, BrepTransform, EdgeBlender, MassCalculator, PrimitiveBuilder,
};
use zenith_io::{StepExporter, StepImporter};
use zenith_math::{Tolerance, Vec3};
use zenith_tess::{tessellate_solid, TessellationParams};

fn make_slotted_plate(
    plate_w: f64,
    plate_d: f64,
    plate_h: f64,
    slot_l: f64,
    slot_r: f64,
) -> (zenith_topo::Solid, u64) {
    let tol = Tolerance::default();
    let plate = PrimitiveBuilder::make_box(plate_w, plate_d, plate_h).expect("plate");
    let slot_tool = BrepTransform::translate_solid(
        &PrimitiveBuilder::make_slot_prism(slot_l, slot_r, plate_h + 20.0).expect("slot tool"),
        Vec3::new(plate_w * 0.5, plate_d * 0.5, -10.0),
    );
    let slotted = BooleanEngine::boolean_solids_exact_result(
        &plate,
        &slot_tool,
        BooleanOpType::Difference,
        &tol,
    )
    .expect("boolean slot cut")
    .solids
    .remove(0);

    // 穴口（z = plate_h の面にある内側ワイヤのエッジ）を探す
    let top_face = slotted
        .outer_shell
        .faces
        .iter()
        .find(|face| {
            if let zenith_topo::FaceGeometry::Plane(p) = &face.geometry {
                (p.origin.z - plate_h).abs() < 1e-6 && !face.inner_wires.is_empty()
            } else {
                false
            }
        })
        .expect("slotted plate top face");

    let mouth_edge_id = top_face.inner_wires[0].edges[0].edge.id;
    (slotted, mouth_edge_id)
}

#[test]
fn test_slot_hole_mouth_chamfer() {
    let tol = Tolerance::default();
    let plate_w = 80.0;
    let plate_d = 60.0;
    let plate_h = 20.0;
    let slot_l = 25.0;
    let slot_r = 8.0;
    let chamfer_d = 2.0;

    let (plate, mouth_edge_id) = make_slotted_plate(plate_w, plate_d, plate_h, slot_l, slot_r);

    let (chamfered, report) = EdgeBlender::blend_edge(
        &plate,
        mouth_edge_id,
        zenith_algo::BlendKind::Chamfer {
            distance: chamfer_d,
        },
    )
    .expect("chamfer slot hole mouth");

    // 1. 体積検証
    let expected_removed =
        slot_l * chamfer_d * chamfer_d + PI * chamfer_d * chamfer_d * (slot_r + chamfer_d / 3.0);
    let initial_vol =
        plate_w * plate_d * plate_h - (2.0 * slot_l * slot_r + PI * slot_r * slot_r) * plate_h;
    let expected_vol = initial_vol - expected_removed;

    let mass = MassCalculator::compute_from_brep(
        &chamfered,
        &TessellationParams {
            u_divisions: 48,
            v_divisions: 48,
        },
    );
    let rel_err = (mass.volume - expected_vol).abs() / expected_vol;
    assert!(
        rel_err < 1e-3,
        "Volume mismatch: mass={}, expected={}, rel_err={rel_err}",
        mass.volume,
        expected_vol
    );
    assert!((report.predicted_removed_volume - expected_removed).abs() < 1e-6);

    // 2. B-Rep 閉多様体・メッシュ検証
    assert!(
        chamfered.outer_shell.validate_closed(&tol).is_valid(),
        "Chamfered slotted plate must be closed manifold"
    );
    let mesh = tessellate_solid(&chamfered, &TessellationParams::default());
    assert!(
        mesh.num_triangles() > 0,
        "Chamfered slotted plate mesh must have triangles"
    );

    // 3. STEP往復検証
    let step_str = StepExporter::export_solid_to_string(&chamfered, "slot_hole_chamfer");
    let reimported = StepImporter::import_solid_from_str(&step_str).expect("import STEP");
    let reimport_mass = MassCalculator::compute_from_brep(
        &reimported,
        &TessellationParams {
            u_divisions: 48,
            v_divisions: 48,
        },
    );
    assert!((reimport_mass.volume - mass.volume).abs() / mass.volume < 1e-4);
}

#[test]
fn test_slot_hole_mouth_fillet() {
    let tol = Tolerance::default();
    let plate_w = 80.0;
    let plate_d = 60.0;
    let plate_h = 20.0;
    let slot_l = 25.0;
    let slot_r = 8.0;
    let fillet_r = 2.5;

    let (plate, mouth_edge_id) = make_slotted_plate(plate_w, plate_d, plate_h, slot_l, slot_r);

    let (filleted, report) = EdgeBlender::blend_edge(
        &plate,
        mouth_edge_id,
        zenith_algo::BlendKind::Fillet { radius: fillet_r },
    )
    .expect("fillet slot hole mouth");

    // 1. 体積検証
    let expected_removed = 2.0 * slot_l * fillet_r * fillet_r * (1.0 - PI * 0.25)
        + PI * (slot_r * fillet_r * fillet_r * (2.0 - PI * 0.5)
            + fillet_r.powi(3) * (5.0 / 3.0 - PI * 0.5));
    let initial_vol =
        plate_w * plate_d * plate_h - (2.0 * slot_l * slot_r + PI * slot_r * slot_r) * plate_h;
    let expected_vol = initial_vol - expected_removed;

    let mass = MassCalculator::compute_from_brep(
        &filleted,
        &TessellationParams {
            u_divisions: 48,
            v_divisions: 48,
        },
    );
    let rel_err = (mass.volume - expected_vol).abs() / expected_vol;
    assert!(
        rel_err < 1e-3,
        "Volume mismatch: mass={}, expected={}, rel_err={rel_err}",
        mass.volume,
        expected_vol
    );
    assert!((report.predicted_removed_volume - expected_removed).abs() < 1e-6);

    // 2. B-Rep 閉多様体・メッシュ検証
    assert!(
        filleted.outer_shell.validate_closed(&tol).is_valid(),
        "Filleted slotted plate must be closed manifold"
    );
    let mesh = tessellate_solid(&filleted, &TessellationParams::default());
    assert!(
        mesh.num_triangles() > 0,
        "Filleted slotted plate mesh must have triangles"
    );

    // 3. STEP往復検証
    let step_str = StepExporter::export_solid_to_string(&filleted, "slot_hole_fillet");
    let reimported = StepImporter::import_solid_from_str(&step_str).expect("import STEP");
    let reimport_mass = MassCalculator::compute_from_brep(
        &reimported,
        &TessellationParams {
            u_divisions: 48,
            v_divisions: 48,
        },
    );
    assert!((reimport_mass.volume - mass.volume).abs() / mass.volume < 1e-4);
}
