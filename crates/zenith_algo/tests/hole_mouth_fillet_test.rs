use std::collections::HashMap;
use std::f64::consts::PI;

use zenith_algo::{BlendKind, BrepTransform, EdgeBlender, FaceMerger, HoleBuilder, MassCalculator};
use zenith_io::{StepExporter, StepImporter};
use zenith_math::{Tolerance, Transform3, Vec3};
use zenith_tess::{tessellate_solid, TessellationParams};
use zenith_topo::{FaceGeometry, Solid};

fn params() -> TessellationParams {
    TessellationParams {
        u_divisions: 64,
        v_divisions: 64,
    }
}

fn close(actual: f64, expected: f64, relative: f64) {
    let scale = actual.abs().max(expected.abs()).max(1.0);
    assert!(
        (actual - expected).abs() / scale <= relative,
        "actual {actual:.12}, expected {expected:.12}, relative error {:.3e}",
        (actual - expected).abs() / scale
    );
}

fn prepared_box() -> Solid {
    let raw = HoleBuilder::make_drilled_box(40.0, 40.0, 20.0, 8.0).unwrap();
    FaceMerger::simplify_solid(&raw, &Tolerance::default())
        .unwrap()
        .0
}

fn mouth_ids(solid: &Solid, direction: Vec3) -> Vec<u64> {
    let mut ids = Vec::new();
    for face in &solid.outer_shell.faces {
        let FaceGeometry::Plane(plane) = &face.geometry else {
            continue;
        };
        let normal = if face.orientation.is_forward() {
            plane.normal
        } else {
            -plane.normal
        };
        if normal.dot(&direction) < 0.99 {
            continue;
        }
        for wire in &face.inner_wires {
            for edge in &wire.edges {
                ids.push(edge.edge.id);
            }
        }
    }
    ids
}

fn removed_volume(hole: f64, fillet: f64) -> f64 {
    PI * (hole * fillet * fillet * (2.0 - PI * 0.5) + fillet.powi(3) * (5.0 / 3.0 - PI * 0.5))
}

fn expected_area(width: f64, depth: f64, height: f64, hole: f64, fillet: f64) -> f64 {
    let outer_sides = 2.0 * (width + depth) * height;
    let bottom = width * depth - PI * hole * hole;
    let top = width * depth - PI * (hole + fillet).powi(2);
    let bore = 2.0 * PI * hole * (height - fillet);
    let torus = PI * PI * (hole + fillet) * fillet - 2.0 * PI * fillet * fillet;
    outer_sides + bottom + top + bore + torus
}

#[test]
fn one_hole_arc_locally_fillets_the_complete_top_mouth() {
    let drilled = prepared_box();
    assert_eq!(drilled.outer_shell.faces.len(), 10);
    let top = mouth_ids(&drilled, Vec3::new(0.0, 0.0, 1.0));
    assert_eq!(top.len(), 4);

    let candidates = EdgeBlender::blendable_edges(&drilled);
    for id in &top {
        let candidate = candidates
            .iter()
            .find(|candidate| candidate.edge_id == *id)
            .expect("every smooth arc of the mouth must select the same fillet");
        close(candidate.length, 2.0 * PI * 8.0, 1e-13);
        assert_eq!(candidate.max_chamfer_distance, 0.0);
    }

    let before = MassCalculator::compute_from_brep(&drilled, &params());
    let (rounded, report) =
        EdgeBlender::blend_edge(&drilled, top[2], BlendKind::Fillet { radius: 1.5 })
            .expect("local through-hole mouth fillet");
    let validation = rounded.outer_shell.validate_closed(&Tolerance::default());
    assert!(validation.is_valid(), "{:#?}", validation.errors);
    assert_eq!(rounded.outer_shell.faces.len(), 14);

    let after = MassCalculator::compute_from_brep(&rounded, &params());
    let removed = removed_volume(8.0, 1.5);
    close(before.volume - after.volume, removed, 3e-10);
    close(report.predicted_removed_volume, removed, 1e-13);
    close(report.edge_length, 2.0 * PI * 8.0, 1e-13);
    close(
        after.surface_area,
        expected_area(40.0, 40.0, 20.0, 8.0, 1.5),
        3e-10,
    );
}

#[test]
fn local_hole_fillet_preserves_every_unrelated_side_face() {
    let drilled = prepared_box();
    let top = mouth_ids(&drilled, Vec3::new(0.0, 0.0, 1.0));
    let side_ids: Vec<u64> = drilled
        .outer_shell
        .faces
        .iter()
        .filter_map(|face| match &face.geometry {
            FaceGeometry::Plane(plane) if plane.normal.z.abs() < 0.5 => Some(face.id),
            _ => None,
        })
        .collect();
    assert_eq!(side_ids.len(), 4);

    let rounded = EdgeBlender::fillet_edge(&drilled, top[0], 1.0).unwrap();
    for id in side_ids {
        assert!(
            rounded.outer_shell.faces.iter().any(|face| face.id == id),
            "unrelated outer face {id} was rebuilt"
        );
    }
    assert_eq!(mouth_ids(&rounded, Vec3::new(0.0, 0.0, -1.0)).len(), 4);
}

#[test]
fn hole_mouth_recognition_survives_rigid_placement() {
    let drilled = prepared_box();
    let turn = Transform3::from_axis_angle(&Vec3::new(1.0, -2.0, 0.75), 39f64.to_radians());
    let moved = BrepTransform::translate_solid(
        &BrepTransform::transform_solid(&drilled, &turn).unwrap(),
        Vec3::new(17.0, -23.0, 9.0),
    );
    let axis = turn.transform_vector(&Vec3::new(0.0, 0.0, 1.0));
    let top = mouth_ids(&moved, axis);
    assert_eq!(top.len(), 4);
    let rounded = EdgeBlender::fillet_edge(&moved, top[1], 1.25).unwrap();
    let expected = 40.0 * 40.0 * 20.0 - PI * 8.0 * 8.0 * 20.0 - removed_volume(8.0, 1.25);
    close(
        MassCalculator::compute_from_brep(&rounded, &params()).volume,
        expected,
        3e-10,
    );
}

#[test]
fn sectorised_and_stepped_holes_are_not_silently_approximated() {
    let raw = HoleBuilder::make_drilled_box(40.0, 40.0, 20.0, 8.0).unwrap();
    assert_eq!(EdgeBlender::blendable_edges(&raw).len(), 0);

    let counterbore = HoleBuilder::make_counterbore_hole_box(40.0, 40.0, 20.0, 5.0, 9.0, 6.0)
        .expect("counterbore fixture");
    let curved_candidates = EdgeBlender::blendable_edges(&counterbore)
        .into_iter()
        .filter(|edge| edge.max_chamfer_distance == 0.0)
        .count();
    assert_eq!(
        curved_candidates, 0,
        "a stepped bore must not be presented as an unbroken through hole"
    );
}

#[test]
fn either_end_of_the_through_hole_can_be_selected() {
    let drilled = prepared_box();
    for direction in [Vec3::new(0.0, 0.0, 1.0), Vec3::new(0.0, 0.0, -1.0)] {
        let ids = mouth_ids(&drilled, direction);
        assert_eq!(ids.len(), 4);
        let rounded = EdgeBlender::fillet_edge(&drilled, ids[3], 1.0).unwrap();
        let expected = 40.0 * 40.0 * 20.0 - PI * 8.0 * 8.0 * 20.0 - removed_volume(8.0, 1.0);
        close(
            MassCalculator::compute_from_brep(&rounded, &params()).volume,
            expected,
            3e-10,
        );
    }
}

#[test]
fn torus_is_tangent_to_the_cap_and_bore() {
    let drilled = prepared_box();
    let top = mouth_ids(&drilled, Vec3::new(0.0, 0.0, 1.0));
    let rounded = EdgeBlender::fillet_edge(&drilled, top[0], 1.0).unwrap();

    // The four locally inserted torus patches are appended after four shortened
    // bore patches. Their u direction runs from bore contact to cap contact.
    for face in &rounded.outer_shell.faces[10..14] {
        let FaceGeometry::Nurbs(surface) = &face.geometry else {
            panic!("hole blend face is not NURBS");
        };
        let at_bore_point = surface.evaluate(0.0, 0.5);
        let radial = Vec3::new(at_bore_point.x - 20.0, at_bore_point.y - 20.0, 0.0);
        let inward_bore_normal = -radial.normalize();
        let at_bore = surface.normal(0.0, 0.5).unwrap();
        let at_cap = surface.normal(1.0, 0.5).unwrap();
        assert!(at_bore.dot(&inward_bore_normal) > 1.0 - 1e-12);
        assert!(at_cap.dot(&Vec3::new(0.0, 0.0, 1.0)) > 1.0 - 1e-12);
    }
}

#[test]
fn impossible_hole_mouth_radius_is_refused_without_partial_output() {
    let drilled = prepared_box();
    let top = mouth_ids(&drilled, Vec3::new(0.0, 0.0, 1.0));
    // The hole centre is 20 mm from each side and its radius is 8 mm.
    assert!(EdgeBlender::fillet_edge(&drilled, top[0], 12.0).is_err());
    assert!(EdgeBlender::fillet_edge(&drilled, top[0], 20.0).is_err());
}

#[test]
fn local_hole_fillet_survives_step_and_coarse_to_fine_meshes() {
    let drilled = prepared_box();
    let top = mouth_ids(&drilled, Vec3::new(0.0, 0.0, 1.0));
    let rounded = EdgeBlender::fillet_edge(&drilled, top[1], 1.5).unwrap();
    let step = StepExporter::export_solid_to_string(&rounded, "LOCAL_HOLE_MOUTH_FILLET");
    let reread = StepImporter::import_solids_from_str(&step)
        .unwrap()
        .into_iter()
        .next()
        .unwrap();
    assert!(reread
        .outer_shell
        .validate_closed(&Tolerance::default())
        .is_valid());
    let expected = 40.0 * 40.0 * 20.0 - PI * 8.0 * 8.0 * 20.0 - removed_volume(8.0, 1.5);
    close(
        MassCalculator::compute_from_brep(&reread, &params()).volume,
        expected,
        5e-10,
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
fn an_external_full_circle_through_hole_is_locally_filleted() {
    let path = concat!(
        env!("CARGO_MANIFEST_DIR"),
        "/tests/fixtures/representation/drilled_analytic.step"
    );
    let solid = StepImporter::import_solids_from_file(path)
        .unwrap()
        .into_iter()
        .next()
        .unwrap();
    let top = mouth_ids(&solid, Vec3::new(0.0, 0.0, 1.0));
    assert_eq!(top.len(), 1, "the STEP fixture uses one full-circle edge");
    let candidate = EdgeBlender::blendable_edges(&solid)
        .into_iter()
        .find(|candidate| candidate.edge_id == top[0])
        .expect("external full circle should be selectable");
    close(candidate.length, 2.0 * PI * 5.0, 1e-12);

    let rounded = EdgeBlender::fillet_edge(&solid, top[0], 1.0).unwrap();
    assert!(rounded
        .outer_shell
        .validate_closed(&Tolerance::default())
        .is_valid());
    assert_eq!(rounded.outer_shell.faces.len(), 14);
    let expected = 30.0 * 30.0 * 15.0 - PI * 5.0 * 5.0 * 15.0 - removed_volume(5.0, 1.0);
    close(
        MassCalculator::compute_from_brep(&rounded, &params()).volume,
        expected,
        3e-10,
    );
    close(
        MassCalculator::compute_from_brep(&rounded, &params()).surface_area,
        expected_area(30.0, 30.0, 15.0, 5.0, 1.0),
        3e-10,
    );
}

#[test]
fn an_all_bspline_step_hole_can_be_planarized_then_filleted() {
    let path = concat!(
        env!("CARGO_MANIFEST_DIR"),
        "/tests/fixtures/representation/drilled_bspline.step"
    );
    let imported = StepImporter::import_solids_from_file(path)
        .unwrap()
        .into_iter()
        .next()
        .unwrap();
    let (solid, _) = FaceMerger::simplify_solid(&imported, &Tolerance::default()).unwrap();
    let top = mouth_ids(&solid, Vec3::new(0.0, 0.0, 1.0));
    assert!(!top.is_empty());
    let rounded = EdgeBlender::fillet_edge(&solid, top[0], 1.0).unwrap();
    let expected = 30.0 * 30.0 * 15.0 - PI * 5.0 * 5.0 * 15.0 - removed_volume(5.0, 1.0);
    close(
        MassCalculator::compute_from_brep(&rounded, &params()).volume,
        expected,
        3e-10,
    );
}

#[test]
fn one_of_three_external_holes_is_edited_without_rebuilding_the_other_two() {
    let path = concat!(
        env!("CARGO_MANIFEST_DIR"),
        "/tests/fixtures/occ_reference_plate_with_holes.step"
    );
    let solid = StepImporter::import_solids_from_file(path)
        .unwrap()
        .into_iter()
        .next()
        .unwrap();
    assert_eq!(solid.outer_shell.faces.len(), 9);
    let top = mouth_ids(&solid, Vec3::new(0.0, 0.0, 1.0));
    assert_eq!(top.len(), 3);
    let candidates = EdgeBlender::blendable_edges(&solid);
    let selected = candidates
        .iter()
        .filter(|candidate| top.contains(&candidate.edge_id))
        .max_by(|left, right| left.length.total_cmp(&right.length))
        .expect("the three imported hole mouths should be selectable");
    close(selected.length, 2.0 * PI * 4.0, 1e-12);

    let old_bore_ids: Vec<u64> = solid
        .outer_shell
        .faces
        .iter()
        .filter(|face| matches!(face.geometry, FaceGeometry::Nurbs(_)))
        .map(|face| face.id)
        .collect();
    assert_eq!(old_bore_ids.len(), 3);
    let before = MassCalculator::compute_from_brep(&solid, &params());
    let rounded = EdgeBlender::fillet_edge(&solid, selected.edge_id, 1.0).unwrap();
    let after = MassCalculator::compute_from_brep(&rounded, &params());
    close(
        before.volume - after.volume,
        removed_volume(4.0, 1.0),
        3e-10,
    );

    let retained = old_bore_ids
        .iter()
        .filter(|id| rounded.outer_shell.faces.iter().any(|face| face.id == **id))
        .count();
    assert_eq!(
        retained, 2,
        "the two unselected bore faces must keep their IDs"
    );
    let top_face = rounded
        .outer_shell
        .faces
        .iter()
        .find(|face| matches!(&face.geometry, FaceGeometry::Plane(plane) if plane.normal.z > 0.9))
        .unwrap();
    assert_eq!(top_face.inner_wires.len(), 3);

    let torus = PI * PI * 5.0 - 2.0 * PI;
    let removed_cap = PI * (5.0f64.powi(2) - 4.0f64.powi(2));
    let removed_bore = 2.0 * PI * 4.0;
    close(
        after.surface_area - before.surface_area,
        torus - removed_cap - removed_bore,
        5e-10,
    );
}
