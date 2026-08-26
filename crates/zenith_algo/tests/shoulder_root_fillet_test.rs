use std::collections::HashMap;
use std::f64::consts::PI;

use zenith_algo::{
    BooleanEngine, BooleanOpType, BrepTransform, EdgeBlender, MassCalculator, PrimitiveBuilder,
    ShaftBuilder,
};
use zenith_io::{StepExporter, StepImporter};
use zenith_math::{Tolerance, Transform3, Vec3};
use zenith_tess::{tessellate_solid, TessellationParams};
use zenith_topo::FaceGeometry;

fn params() -> TessellationParams {
    TessellationParams {
        u_divisions: 96,
        v_divisions: 96,
    }
}

fn added_volume(shaft_radius: f64, fillet: f64) -> f64 {
    PI * (shaft_radius * fillet * fillet * (2.0 - PI * 0.5)
        + fillet.powi(3) * (5.0 / 3.0 - PI * 0.5))
}

fn conical_root_added_volume(root_radius: f64, slope: f64, fillet: f64) -> f64 {
    let norm = slope.hypot(1.0);
    let centre_radius = root_radius + fillet * (norm + slope);
    let contact_z = fillet * (1.0 + slope / norm);
    let arc_primitive = |z: f64| {
        let shifted = z - fillet;
        let root = (fillet * fillet - shifted * shifted).max(0.0).sqrt();
        let integral_root =
            0.5 * (shifted * root + fillet * fillet * (shifted / fillet).clamp(-1.0, 1.0).asin());
        centre_radius * centre_radius * z - 2.0 * centre_radius * integral_root
            + fillet * fillet * z
            - shifted.powi(3) / 3.0
    };
    let cone_primitive = |z: f64| {
        root_radius * root_radius * z
            + root_radius * slope * z * z
            + slope * slope * z.powi(3) / 3.0
    };
    PI * ((arc_primitive(contact_z) - arc_primitive(0.0))
        - (cone_primitive(contact_z) - cone_primitive(0.0)))
}

fn chamfer_added_volume(shaft_radius: f64, distance: f64) -> f64 {
    PI * distance * distance * (shaft_radius + distance / 3.0)
}

fn expected_area(
    base_radius: f64,
    base_height: f64,
    shaft_radius: f64,
    shaft_height: f64,
    fillet: f64,
) -> f64 {
    let bottom = PI * base_radius * base_radius;
    let base_side = 2.0 * PI * base_radius * base_height;
    let shoulder = PI * (base_radius * base_radius - (shaft_radius + fillet).powi(2));
    let shaft_side = 2.0 * PI * shaft_radius * (shaft_height - fillet);
    let top = PI * shaft_radius * shaft_radius;
    let torus = PI * PI * fillet * (shaft_radius + fillet) - 2.0 * PI * fillet * fillet;
    bottom + base_side + shoulder + shaft_side + top + torus
}

fn expected_chamfer_area(
    base_radius: f64,
    base_height: f64,
    shaft_radius: f64,
    shaft_height: f64,
    distance: f64,
) -> f64 {
    let bottom = PI * base_radius * base_radius;
    let base_side = 2.0 * PI * base_radius * base_height;
    let shoulder = PI * (base_radius * base_radius - (shaft_radius + distance).powi(2));
    let shaft_side = 2.0 * PI * shaft_radius * (shaft_height - distance);
    let top = PI * shaft_radius * shaft_radius;
    let chamfer = PI * (2.0 * shaft_radius + distance) * distance * 2.0_f64.sqrt();
    bottom + base_side + shoulder + shaft_side + top + chamfer
}

fn close(actual: f64, expected: f64, relative: f64) {
    let scale = expected.abs().max(1.0);
    assert!(
        (actual - expected).abs() <= relative * scale,
        "actual={actual:.15}, expected={expected:.15}, diff={:.3e}",
        actual - expected
    );
}

#[test]
fn one_root_arc_locally_fillets_the_complete_stepped_shaft_shoulder() {
    let shaft =
        ShaftBuilder::make_stepped_shaft(&[(10.0, 12.0), (7.0, 10.0)]).expect("two-step shaft");
    let all_candidates = EdgeBlender::blendable_edges(&shaft);
    let candidates: Vec<_> = all_candidates
        .iter()
        .cloned()
        .into_iter()
        .filter(|edge| (edge.length - 2.0 * PI * 7.0).abs() < 1e-6)
        .collect();
    assert_eq!(
        candidates.len(),
        4,
        "all four root arcs must be selectable; listed={all_candidates:?}"
    );
    for candidate in &candidates {
        close(candidate.dihedral_angle_deg, 270.0, 1e-13);
        close(candidate.max_fillet_radius, 3.0 * 0.999, 1e-12);
        close(candidate.max_chamfer_distance, 3.0 * 0.999, 1e-12);
    }

    let before = MassCalculator::compute_from_brep(&shaft, &params());
    let old_ids: Vec<u64> = shaft.outer_shell.faces.iter().map(|face| face.id).collect();
    let fillet = 1.25;
    let (rounded, report) = EdgeBlender::blend_edge(
        &shaft,
        candidates[2].edge_id,
        zenith_algo::BlendKind::Fillet { radius: fillet },
    )
    .expect("local shoulder-root fillet");
    assert!(rounded
        .outer_shell
        .validate_closed(&Tolerance::default())
        .is_valid());
    assert_eq!(rounded.outer_shell.faces.len(), 15);
    assert_eq!(
        old_ids
            .iter()
            .filter(|id| rounded.outer_shell.faces.iter().any(|face| face.id == **id))
            .count(),
        7,
        "only the four shortened shaft-side patches may lose their Face IDs"
    );
    let after = MassCalculator::compute_from_brep(&rounded, &params());
    let added = added_volume(7.0, fillet);
    close(after.volume - before.volume, added, 3e-10);
    close(report.predicted_removed_volume, -added, 1e-13);
    close(report.edge_length, 2.0 * PI * 7.0, 1e-13);
    close(report.dihedral_angle_deg, 270.0, 1e-13);
    close(
        after.surface_area,
        expected_area(10.0, 12.0, 7.0, 10.0, fillet),
        4e-10,
    );
}

#[test]
fn one_root_arc_locally_chamfers_the_complete_stepped_shaft_shoulder() {
    let shaft =
        ShaftBuilder::make_stepped_shaft(&[(10.0, 12.0), (7.0, 10.0)]).expect("two-step shaft");
    let candidates: Vec<_> = EdgeBlender::blendable_edges(&shaft)
        .into_iter()
        .filter(|edge| (edge.length - 2.0 * PI * 7.0).abs() < 1e-6)
        .collect();
    assert_eq!(candidates.len(), 4);

    let before = MassCalculator::compute_from_brep(&shaft, &params());
    let old_ids: Vec<u64> = shaft.outer_shell.faces.iter().map(|face| face.id).collect();
    let distance = 1.25;
    let (chamfered, report) = EdgeBlender::blend_edge(
        &shaft,
        candidates[1].edge_id,
        zenith_algo::BlendKind::Chamfer { distance },
    )
    .expect("local shoulder-root chamfer");
    assert!(chamfered
        .outer_shell
        .validate_closed(&Tolerance::default())
        .is_valid());
    assert_eq!(chamfered.outer_shell.faces.len(), 15);
    assert_eq!(
        old_ids
            .iter()
            .filter(|id| chamfered
                .outer_shell
                .faces
                .iter()
                .any(|face| face.id == **id))
            .count(),
        7,
        "only the four shortened shaft-side patches may lose their Face IDs"
    );
    let after = MassCalculator::compute_from_brep(&chamfered, &params());
    let added = chamfer_added_volume(7.0, distance);
    close(after.volume - before.volume, added, 3e-10);
    close(report.predicted_removed_volume, -added, 1e-13);
    close(report.setback, distance, 1e-13);
    close(report.edge_length, 2.0 * PI * 7.0, 1e-13);
    close(report.dihedral_angle_deg, 270.0, 1e-13);
    close(
        after.surface_area,
        expected_chamfer_area(10.0, 12.0, 7.0, 10.0, distance),
        4e-10,
    );
}

#[test]
fn shoulder_torus_is_tangent_to_the_shaft_and_plane() {
    let shaft = ShaftBuilder::make_stepped_shaft(&[(10.0, 12.0), (7.0, 10.0)]).unwrap();
    let candidate = EdgeBlender::blendable_edges(&shaft)
        .into_iter()
        .find(|edge| (edge.length - 2.0 * PI * 7.0).abs() < 1e-6)
        .unwrap();
    let rounded = EdgeBlender::fillet_edge(&shaft, candidate.edge_id, 1.0).unwrap();

    for face in &rounded.outer_shell.faces[11..15] {
        let FaceGeometry::Nurbs(surface) = &face.geometry else {
            panic!("shoulder blend face is not NURBS");
        };
        let at_shaft_point = surface.evaluate(0.0, 0.5);
        let radial = Vec3::new(at_shaft_point.x, at_shaft_point.y, 0.0).normalize();
        let at_shaft = surface.normal(0.0, 0.5).unwrap();
        let at_plane = surface.normal(1.0, 0.5).unwrap();
        assert!(at_shaft.dot(&radial) > 1.0 - 1e-12);
        assert!(at_plane.dot(&Vec3::new(0.0, 0.0, 1.0)) > 1.0 - 1e-12);
    }
}

#[test]
fn external_three_step_shaft_keeps_the_other_root_and_survives_rigid_placement() {
    let path = concat!(
        env!("CARGO_MANIFEST_DIR"),
        "/tests/fixtures/occ_reference_stepped_shaft.step"
    );
    let imported = StepImporter::import_solids_from_file(path)
        .unwrap()
        .into_iter()
        .next()
        .unwrap();
    let turn = Transform3::from_axis_angle(&Vec3::new(1.0, -2.0, 0.75), 29f64.to_radians());
    let moved = BrepTransform::translate_solid(
        &BrepTransform::transform_solid(&imported, &turn).unwrap(),
        Vec3::new(17.0, -11.0, 8.0),
    );
    let before = MassCalculator::compute_from_brep(&moved, &params()).volume;
    let old_ids: Vec<u64> = moved.outer_shell.faces.iter().map(|face| face.id).collect();
    let first = EdgeBlender::blendable_edges(&moved)
        .into_iter()
        .find(|edge| (edge.length - 2.0 * PI * 6.5).abs() < 1e-6)
        .expect("the first OCC full-circle shoulder root");
    let once = EdgeBlender::fillet_edge(&moved, first.edge_id, 1.0).unwrap();
    close(
        MassCalculator::compute_from_brep(&once, &params()).volume - before,
        added_volume(6.5, 1.0),
        5e-10,
    );
    assert_eq!(
        old_ids
            .iter()
            .filter(|id| once.outer_shell.faces.iter().any(|face| face.id == **id))
            .count(),
        6,
        "only the selected full-cylinder side face may lose its ID"
    );

    let second = EdgeBlender::blendable_edges(&once)
        .into_iter()
        .find(|edge| (edge.length - 2.0 * PI * 4.0).abs() < 1e-6)
        .expect("the unselected second shoulder root must remain selectable");
    let twice = EdgeBlender::fillet_edge(&once, second.edge_id, 0.75).unwrap();
    close(
        MassCalculator::compute_from_brep(&twice, &params()).volume - before,
        added_volume(6.5, 1.0) + added_volume(4.0, 0.75),
        5e-10,
    );
}

#[test]
fn impossible_shoulder_radius_is_refused_without_partial_output() {
    let shaft = ShaftBuilder::make_stepped_shaft(&[(10.0, 12.0), (7.0, 10.0)]).unwrap();
    let candidate = EdgeBlender::blendable_edges(&shaft)
        .into_iter()
        .find(|edge| (edge.length - 2.0 * PI * 7.0).abs() < 1e-6)
        .unwrap();
    assert!(EdgeBlender::fillet_edge(&shaft, candidate.edge_id, 0.0).is_err());
    assert!(EdgeBlender::fillet_edge(&shaft, candidate.edge_id, -1.0).is_err());
    assert!(EdgeBlender::fillet_edge(&shaft, candidate.edge_id, f64::NAN).is_err());
    assert!(EdgeBlender::fillet_edge(&shaft, candidate.edge_id, 3.0).is_err());
    assert!(EdgeBlender::fillet_edge(&shaft, candidate.edge_id, 5.0).is_err());
}

#[test]
fn impossible_shoulder_chamfer_distance_is_refused_without_partial_output() {
    let shaft = ShaftBuilder::make_stepped_shaft(&[(10.0, 12.0), (7.0, 10.0)]).unwrap();
    let candidate = EdgeBlender::blendable_edges(&shaft)
        .into_iter()
        .find(|edge| (edge.length - 2.0 * PI * 7.0).abs() < 1e-6)
        .unwrap();
    assert!(EdgeBlender::chamfer_edge(&shaft, candidate.edge_id, 0.0).is_err());
    assert!(EdgeBlender::chamfer_edge(&shaft, candidate.edge_id, -1.0).is_err());
    assert!(EdgeBlender::chamfer_edge(&shaft, candidate.edge_id, f64::NAN).is_err());
    assert!(EdgeBlender::chamfer_edge(&shaft, candidate.edge_id, 3.0).is_err());
    assert!(EdgeBlender::chamfer_edge(&shaft, candidate.edge_id, 5.0).is_err());
}

#[test]
fn external_three_step_shaft_chamfers_twice_after_rigid_placement() {
    let path = concat!(
        env!("CARGO_MANIFEST_DIR"),
        "/tests/fixtures/occ_reference_stepped_shaft.step"
    );
    let imported = StepImporter::import_solids_from_file(path)
        .unwrap()
        .into_iter()
        .next()
        .unwrap();
    let turn = Transform3::from_axis_angle(&Vec3::new(1.0, -2.0, 0.75), 29f64.to_radians());
    let moved = BrepTransform::translate_solid(
        &BrepTransform::transform_solid(&imported, &turn).unwrap(),
        Vec3::new(17.0, -11.0, 8.0),
    );
    let before = MassCalculator::compute_from_brep(&moved, &params()).volume;
    let first = EdgeBlender::blendable_edges(&moved)
        .into_iter()
        .find(|edge| (edge.length - 2.0 * PI * 6.5).abs() < 1e-6)
        .expect("the first OCC full-circle shoulder root");
    let once = EdgeBlender::chamfer_edge(&moved, first.edge_id, 1.0).unwrap();
    let second = EdgeBlender::blendable_edges(&once)
        .into_iter()
        .find(|edge| (edge.length - 2.0 * PI * 4.0).abs() < 1e-6)
        .expect("the unselected second shoulder root must remain selectable");
    let twice = EdgeBlender::chamfer_edge(&once, second.edge_id, 0.75).unwrap();
    close(
        MassCalculator::compute_from_brep(&twice, &params()).volume - before,
        chamfer_added_volume(6.5, 1.0) + chamfer_added_volume(4.0, 0.75),
        6e-10,
    );
}

#[test]
fn boolean_boss_root_is_locally_filleted_without_rebuilding_the_block() {
    let tol = Tolerance::default();
    let block = PrimitiveBuilder::make_box(40.0, 40.0, 20.0).unwrap();
    let boss = BrepTransform::translate_solid(
        &PrimitiveBuilder::make_cylinder(6.0, 30.0).unwrap(),
        Vec3::new(20.0, 20.0, 20.0),
    );
    let joined = BooleanEngine::boolean_solids_exact(&block, &boss, BooleanOpType::Union, &tol)
        .expect("block union cylindrical boss");
    let candidates: Vec<_> = EdgeBlender::blendable_edges(&joined)
        .into_iter()
        .filter(|edge| {
            (edge.length - 2.0 * PI * 6.0).abs() < 1e-6
                && (edge.dihedral_angle_deg - 270.0).abs() < 1e-10
        })
        .collect();
    assert_eq!(
        candidates.len(),
        4,
        "all four boss-root arcs must select one ring"
    );
    let old_block_faces: Vec<u64> = joined
        .outer_shell
        .faces
        .iter()
        .filter(|face| matches!(face.geometry, FaceGeometry::Plane(_)))
        .map(|face| face.id)
        .collect();
    let before = MassCalculator::compute_from_brep(&joined, &params());
    let fillet = 2.0;
    let rounded = EdgeBlender::fillet_edge(&joined, candidates[1].edge_id, fillet).unwrap();
    let after = MassCalculator::compute_from_brep(&rounded, &params());
    close(
        after.volume - before.volume,
        added_volume(6.0, fillet),
        1e-9,
    );
    assert!(rounded.outer_shell.validate_closed(&tol).is_valid());
    assert!(old_block_faces.iter().all(|id| rounded
        .outer_shell
        .faces
        .iter()
        .any(|face| face.id == *id)));
}

#[test]
fn boolean_conical_boss_root_is_recognized_and_filleted_exactly() {
    let tol = Tolerance::default();
    let block = PrimitiveBuilder::make_box(40.0, 40.0, 20.0).unwrap();
    let cone = BrepTransform::translate_solid(
        &PrimitiveBuilder::make_cone(8.0, 5.0, 12.0).unwrap(),
        Vec3::new(20.0, 20.0, 20.0),
    );
    let joined = BooleanEngine::boolean_solids_exact(&block, &cone, BooleanOpType::Union, &tol)
        .expect("block union conical boss");
    let candidates: Vec<_> = EdgeBlender::blendable_edges(&joined)
        .into_iter()
        .filter(|edge| (edge.length - 2.0 * PI * 8.0).abs() < 1e-6)
        .collect();
    assert_eq!(
        candidates.len(),
        4,
        "all four conical root arcs must select one ring"
    );
    let slope = (5.0_f64 - 8.0) / 12.0;
    for candidate in &candidates {
        close(
            candidate.dihedral_angle_deg,
            270.0 + slope.atan().to_degrees(),
            1e-12,
        );
        assert!(candidate.max_fillet_radius > 10.0);
        assert_eq!(candidate.max_chamfer_distance, 0.0);
    }
    assert!(EdgeBlender::chamfer_edge(&joined, candidates[0].edge_id, 1.0).is_err());

    let before = MassCalculator::compute_from_brep(&joined, &params());
    let old_planar_ids: Vec<u64> = joined
        .outer_shell
        .faces
        .iter()
        .filter(|face| matches!(face.geometry, FaceGeometry::Plane(_)))
        .map(|face| face.id)
        .collect();
    let fillet = 2.0;
    let (rounded, report) = EdgeBlender::blend_edge(
        &joined,
        candidates[2].edge_id,
        zenith_algo::BlendKind::Fillet { radius: fillet },
    )
    .expect("local conical boss-root fillet");
    assert!(rounded.outer_shell.validate_closed(&tol).is_valid());
    assert!(old_planar_ids.iter().all(|id| rounded
        .outer_shell
        .faces
        .iter()
        .any(|face| face.id == *id)));
    let after = MassCalculator::compute_from_brep(&rounded, &params());
    let added = conical_root_added_volume(8.0, slope, fillet);
    close(after.volume - before.volume, added, 2e-9);
    close(report.predicted_removed_volume, -added, 1e-13);
    close(report.edge_length, 2.0 * PI * 8.0, 1e-13);

    let blend_start = rounded.outer_shell.faces.len() - 4;
    for face in &rounded.outer_shell.faces[blend_start..] {
        let FaceGeometry::Nurbs(surface) = &face.geometry else {
            panic!("conical root blend face is not NURBS");
        };
        let at_cone_point = surface.evaluate(0.0, 0.5);
        let radial = Vec3::new(at_cone_point.x - 20.0, at_cone_point.y - 20.0, 0.0).normalize();
        let expected_cone_normal = (radial - Vec3::new(0.0, 0.0, slope)).normalize();
        assert!(surface.normal(0.0, 0.5).unwrap().dot(&expected_cone_normal) > 1.0 - 1e-12);
        assert!(
            surface
                .normal(1.0, 0.5)
                .unwrap()
                .dot(&Vec3::new(0.0, 0.0, 1.0))
                > 1.0 - 1e-12
        );
    }

    let step = StepExporter::export_solid_to_string(&rounded, "CONICAL_BOSS_ROOT_FILLET");
    let reread = StepImporter::import_solids_from_str(&step)
        .unwrap()
        .into_iter()
        .next()
        .unwrap();
    assert!(reread.outer_shell.validate_closed(&tol).is_valid());
    close(
        MassCalculator::compute_from_brep(&reread, &params()).volume - before.volume,
        added,
        3e-9,
    );
    for divisions in [4, 8, 16, 32] {
        let mesh = tessellate_solid(
            &reread,
            &TessellationParams {
                u_divisions: divisions,
                v_divisions: divisions,
            },
        );
        let mut uses: HashMap<(u32, u32), usize> = HashMap::new();
        for triangle in &mesh.indices {
            let a = mesh.positions[triangle[0] as usize];
            let b = mesh.positions[triangle[1] as usize];
            let c = mesh.positions[triangle[2] as usize];
            assert!((b - a).cross(&(c - a)).norm() > 1e-12);
            for step in 0..3 {
                let (a, b) = (triangle[step], triangle[(step + 1) % 3]);
                *uses.entry(if a < b { (a, b) } else { (b, a) }).or_insert(0) += 1;
            }
        }
        assert_eq!(uses.values().filter(|count| **count != 2).count(), 0);
    }
}

#[test]
fn conical_boss_root_fillet_survives_rigid_placement() {
    let tol = Tolerance::default();
    let block = PrimitiveBuilder::make_box(40.0, 40.0, 20.0).unwrap();
    let cone = BrepTransform::translate_solid(
        &PrimitiveBuilder::make_cone(8.0, 5.0, 12.0).unwrap(),
        Vec3::new(20.0, 20.0, 20.0),
    );
    let joined = BooleanEngine::boolean_solids_exact(&block, &cone, BooleanOpType::Union, &tol)
        .expect("block union conical boss");
    let turn = Transform3::from_axis_angle(&Vec3::new(1.0, -2.0, 0.75), 31f64.to_radians());
    let moved = BrepTransform::translate_solid(
        &BrepTransform::transform_solid(&joined, &turn).unwrap(),
        Vec3::new(17.0, -11.0, 8.0),
    );
    let candidate = EdgeBlender::blendable_edges(&moved)
        .into_iter()
        .find(|edge| (edge.length - 2.0 * PI * 8.0).abs() < 1e-6)
        .expect("placed conical root remains selectable");
    let before = MassCalculator::compute_from_brep(&moved, &params()).volume;
    let rounded = EdgeBlender::fillet_edge(&moved, candidate.edge_id, 1.5).unwrap();
    assert!(rounded.outer_shell.validate_closed(&tol).is_valid());
    close(
        MassCalculator::compute_from_brep(&rounded, &params()).volume - before,
        conical_root_added_volume(8.0, -0.25, 1.5),
        3e-9,
    );
}

#[test]
fn boolean_boss_root_is_locally_chamfered_without_rebuilding_the_block() {
    let tol = Tolerance::default();
    let block = PrimitiveBuilder::make_box(40.0, 40.0, 20.0).unwrap();
    let boss = BrepTransform::translate_solid(
        &PrimitiveBuilder::make_cylinder(6.0, 30.0).unwrap(),
        Vec3::new(20.0, 20.0, 20.0),
    );
    let joined = BooleanEngine::boolean_solids_exact(&block, &boss, BooleanOpType::Union, &tol)
        .expect("block union cylindrical boss");
    let candidate = EdgeBlender::blendable_edges(&joined)
        .into_iter()
        .find(|edge| {
            (edge.length - 2.0 * PI * 6.0).abs() < 1e-6
                && (edge.dihedral_angle_deg - 270.0).abs() < 1e-10
        })
        .expect("boss-root chamfer candidate");
    let old_block_faces: Vec<u64> = joined
        .outer_shell
        .faces
        .iter()
        .filter(|face| matches!(face.geometry, FaceGeometry::Plane(_)))
        .map(|face| face.id)
        .collect();
    let before = MassCalculator::compute_from_brep(&joined, &params());
    let distance = 2.0;
    let chamfered = EdgeBlender::chamfer_edge(&joined, candidate.edge_id, distance).unwrap();
    let after = MassCalculator::compute_from_brep(&chamfered, &params());
    close(
        after.volume - before.volume,
        chamfer_added_volume(6.0, distance),
        1e-9,
    );
    assert!(chamfered.outer_shell.validate_closed(&tol).is_valid());
    assert!(old_block_faces.iter().all(|id| chamfered
        .outer_shell
        .faces
        .iter()
        .any(|face| face.id == *id)));
}

#[test]
fn local_shoulder_fillet_survives_step_and_coarse_to_fine_meshes() {
    let shaft = ShaftBuilder::make_stepped_shaft(&[(10.0, 12.0), (7.0, 10.0)]).unwrap();
    let candidate = EdgeBlender::blendable_edges(&shaft)
        .into_iter()
        .find(|edge| (edge.length - 2.0 * PI * 7.0).abs() < 1e-6)
        .unwrap();
    let before = MassCalculator::compute_from_brep(&shaft, &params()).volume;
    let rounded = EdgeBlender::fillet_edge(&shaft, candidate.edge_id, 1.5).unwrap();
    let step = StepExporter::export_solid_to_string(&rounded, "LOCAL_SHOULDER_ROOT_FILLET");
    let reread = StepImporter::import_solids_from_str(&step)
        .unwrap()
        .into_iter()
        .next()
        .unwrap();
    assert!(reread
        .outer_shell
        .validate_closed(&Tolerance::default())
        .is_valid());
    close(
        MassCalculator::compute_from_brep(&reread, &params()).volume - before,
        added_volume(7.0, 1.5),
        6e-10,
    );

    for divisions in [4, 6, 8, 12, 16, 24, 32] {
        let mesh = tessellate_solid(
            &reread,
            &TessellationParams {
                u_divisions: divisions,
                v_divisions: divisions,
            },
        );
        let mut uses: HashMap<(u32, u32), usize> = HashMap::new();
        let mut degenerate = 0usize;
        for triangle in &mesh.indices {
            let a = mesh.positions[triangle[0] as usize];
            let b = mesh.positions[triangle[1] as usize];
            let c = mesh.positions[triangle[2] as usize];
            if (b - a).cross(&(c - a)).norm() <= 1e-12 {
                degenerate += 1;
            }
            for step in 0..3 {
                let (a, b) = (triangle[step], triangle[(step + 1) % 3]);
                let key = if a < b { (a, b) } else { (b, a) };
                *uses.entry(key).or_insert(0) += 1;
            }
        }
        assert_eq!(
            uses.values().filter(|count| **count != 2).count(),
            0,
            "{divisions} divisions left non-manifold mesh edges"
        );
        assert_eq!(
            degenerate, 0,
            "{divisions} divisions made zero-area triangles"
        );
    }
}

#[test]
fn local_shoulder_chamfer_survives_step_and_coarse_to_fine_meshes() {
    let shaft = ShaftBuilder::make_stepped_shaft(&[(10.0, 12.0), (7.0, 10.0)]).unwrap();
    let candidate = EdgeBlender::blendable_edges(&shaft)
        .into_iter()
        .find(|edge| (edge.length - 2.0 * PI * 7.0).abs() < 1e-6)
        .unwrap();
    let before = MassCalculator::compute_from_brep(&shaft, &params()).volume;
    let chamfered = EdgeBlender::chamfer_edge(&shaft, candidate.edge_id, 1.5).unwrap();
    let step = StepExporter::export_solid_to_string(&chamfered, "LOCAL_SHOULDER_ROOT_CHAMFER");
    let reread = StepImporter::import_solids_from_str(&step)
        .unwrap()
        .into_iter()
        .next()
        .unwrap();
    assert!(reread
        .outer_shell
        .validate_closed(&Tolerance::default())
        .is_valid());
    close(
        MassCalculator::compute_from_brep(&reread, &params()).volume - before,
        chamfer_added_volume(7.0, 1.5),
        6e-10,
    );

    for divisions in [4, 6, 8, 12, 16, 24, 32] {
        let mesh = tessellate_solid(
            &reread,
            &TessellationParams {
                u_divisions: divisions,
                v_divisions: divisions,
            },
        );
        let mut uses: HashMap<(u32, u32), usize> = HashMap::new();
        let mut degenerate = 0usize;
        for triangle in &mesh.indices {
            let a = mesh.positions[triangle[0] as usize];
            let b = mesh.positions[triangle[1] as usize];
            let c = mesh.positions[triangle[2] as usize];
            if (b - a).cross(&(c - a)).norm() <= 1e-12 {
                degenerate += 1;
            }
            for step in 0..3 {
                let (a, b) = (triangle[step], triangle[(step + 1) % 3]);
                let key = if a < b { (a, b) } else { (b, a) };
                *uses.entry(key).or_insert(0) += 1;
            }
        }
        assert_eq!(
            uses.values().filter(|count| **count != 2).count(),
            0,
            "{divisions} divisions left non-manifold mesh edges"
        );
        assert_eq!(
            degenerate, 0,
            "{divisions} divisions made zero-area triangles"
        );
    }
}
