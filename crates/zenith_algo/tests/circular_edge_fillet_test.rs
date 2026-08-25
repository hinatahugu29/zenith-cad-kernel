use std::collections::HashMap;
use std::f64::consts::PI;

use zenith_algo::{
    BrepTransform, ChamferBuilder, EdgeBlender, FilletBuilder, HoleBuilder, MassCalculator,
    PrimitiveBuilder,
};
use zenith_io::{StepExporter, StepImporter};
use zenith_math::{Tolerance, Transform3, Vec3};
use zenith_tess::{tessellate_solid, TessellationParams};
use zenith_topo::FaceGeometry;

fn params() -> TessellationParams {
    TessellationParams {
        u_divisions: 64,
        v_divisions: 64,
    }
}

fn expected_removed_volume(radius: f64, fillet: f64) -> f64 {
    let major = radius - fillet;
    PI * (major * fillet * fillet * (2.0 - PI * 0.5) + fillet.powi(3) / 3.0)
}

fn expected_area(radius: f64, height: f64, fillet: f64) -> f64 {
    let major = radius - fillet;
    PI * radius * radius
        + 2.0 * PI * radius * (height - fillet)
        + PI * major * major
        + PI * PI * major * fillet
        + 2.0 * PI * fillet * fillet
}

fn expected_chamfer_removed_volume(radius: f64, distance: f64) -> f64 {
    PI * distance * distance * (radius - distance / 3.0)
}

fn expected_chamfer_area(radius: f64, height: f64, distance: f64) -> f64 {
    let top_radius = radius - distance;
    PI * radius * radius
        + 2.0 * PI * radius * (height - distance)
        + PI * top_radius * top_radius
        + PI * (radius + top_radius) * distance * 2.0_f64.sqrt()
}

fn close(actual: f64, expected: f64, relative: f64) {
    let scale = actual.abs().max(expected.abs()).max(1.0);
    assert!(
        (actual - expected).abs() / scale <= relative,
        "actual {actual:.12}, expected {expected:.12}, relative error {:.3e}",
        (actual - expected).abs() / scale
    );
}

#[test]
fn the_cylinder_top_rim_matches_the_closed_form_across_sizes() {
    let tol = Tolerance::default();
    for (radius, height, fillet) in [(10.0, 40.0, 2.0), (7.5, 12.0, 0.5), (20.0, 8.0, 3.0)] {
        let solid = FilletBuilder::fillet_cylinder_top_edge(radius, height, fillet, &tol)
            .unwrap_or_else(|error| panic!("r={radius}, h={height}, f={fillet}: {error}"));
        let report = solid.outer_shell.validate_closed(&tol);
        assert!(report.is_valid(), "{:#?}", report.errors);
        assert_eq!(solid.outer_shell.faces.len(), 10);

        let measured = MassCalculator::compute_from_brep(&solid, &params());
        let cylinder_volume = PI * radius * radius * height;
        close(
            measured.volume,
            cylinder_volume - expected_removed_volume(radius, fillet),
            2e-11,
        );
        close(
            measured.surface_area,
            expected_area(radius, height, fillet),
            2e-11,
        );
    }
}

#[test]
fn the_reference_case_agrees_with_opencascade() {
    let solid = FilletBuilder::fillet_cylinder_top_edge(10.0, 40.0, 2.0, &Tolerance::default())
        .expect("the documented reference case");
    let measured = MassCalculator::compute_from_brep(&solid, &params());

    // FreeCAD 1.1.1 / OpenCASCADE, Part.makeCylinder(10, 40), top circular
    // edge, makeFillet(2): valid closed solid, four faces and five edges.
    close(measured.volume, 12514.844774537281, 2e-11);
    close(measured.surface_area, 3085.878023563117, 2e-11);
}

#[test]
fn unusable_radii_are_refused_instead_of_clamped() {
    let tol = Tolerance::default();
    assert!(FilletBuilder::fillet_cylinder_top_edge(10.0, 40.0, -1.0, &tol).is_err());
    assert!(FilletBuilder::fillet_cylinder_top_edge(10.0, 40.0, 10.0, &tol).is_err());
    assert!(FilletBuilder::fillet_cylinder_top_edge(10.0, 2.0, 2.0, &tol).is_err());

    let plain = FilletBuilder::fillet_cylinder_top_edge(10.0, 40.0, 0.0, &tol)
        .expect("zero radius keeps the existing builder contract");
    let expected = PrimitiveBuilder::make_cylinder(10.0, 40.0).unwrap();
    close(
        MassCalculator::compute_from_brep(&plain, &params()).volume,
        MassCalculator::compute_from_brep(&expected, &params()).volume,
        1e-13,
    );
}

fn cap_edge_ids(solid: &zenith_topo::Solid, direction: Vec3) -> Vec<u64> {
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
        for edge in &face.outer_wire.edges {
            if !ids.contains(&edge.edge.id) {
                ids.push(edge.edge.id);
            }
        }
    }
    ids
}

#[test]
fn selecting_one_arc_fillets_the_complete_smooth_rim() {
    let cylinder = PrimitiveBuilder::make_cylinder(10.0, 40.0).unwrap();
    let top = cap_edge_ids(&cylinder, Vec3::new(0.0, 0.0, 1.0));
    assert_eq!(top.len(), 4, "our cylinder stores a rim as four exact arcs");

    let listed = EdgeBlender::blendable_edges(&cylinder);
    assert_eq!(
        listed.len(),
        8,
        "both four-arc cap rims should be selectable"
    );
    for edge_id in &top {
        assert!(listed.iter().any(|edge| edge.edge_id == *edge_id));
    }

    let (rounded, report) = EdgeBlender::blend_edge(
        &cylinder,
        top[2],
        zenith_algo::BlendKind::Fillet { radius: 2.0 },
    )
    .expect("one selected arc should propagate over its smooth circular chain");
    close(report.edge_length, 2.0 * PI * 10.0, 1e-13);
    close(
        report.predicted_removed_volume,
        expected_removed_volume(10.0, 2.0),
        1e-13,
    );
    let before = MassCalculator::compute_from_brep(&cylinder, &params()).volume;
    let after = MassCalculator::compute_from_brep(&rounded, &params()).volume;
    close(
        after,
        PI * 10.0 * 10.0 * 40.0 - report.predicted_removed_volume,
        2e-11,
    );
    close(before - after, report.predicted_removed_volume, 3e-10);
}

#[test]
fn a_rigidly_placed_cylinder_uses_the_same_selected_edge_path() {
    let cylinder = PrimitiveBuilder::make_cylinder(10.0, 40.0).unwrap();
    let turn = Transform3::from_axis_angle(&Vec3::new(1.0, 2.0, -0.5), 41f64.to_radians());
    let moved = BrepTransform::translate_solid(
        &BrepTransform::transform_solid(&cylinder, &turn).unwrap(),
        Vec3::new(31.0, -17.0, 9.0),
    );
    let axis = turn.transform_vector(&Vec3::new(0.0, 0.0, 1.0));
    let top = cap_edge_ids(&moved, axis);
    assert_eq!(top.len(), 4);

    let rounded = EdgeBlender::fillet_edge(&moved, top[0], 2.0)
        .expect("recognition must not depend on the world axes or origin");
    let expected = PI * 10.0 * 10.0 * 40.0 - expected_removed_volume(10.0, 2.0);
    close(
        MassCalculator::compute_from_brep(&rounded, &params()).volume,
        expected,
        2e-11,
    );
}

#[test]
fn either_convex_cap_rim_can_be_selected() {
    let cylinder = PrimitiveBuilder::make_cylinder(10.0, 40.0).unwrap();
    let bottom = cap_edge_ids(&cylinder, Vec3::new(0.0, 0.0, -1.0));
    assert_eq!(bottom.len(), 4);
    let rounded = EdgeBlender::fillet_edge(&cylinder, bottom[1], 2.0)
        .expect("the lower convex rim is the same local configuration");
    let expected = PI * 10.0 * 10.0 * 40.0 - expected_removed_volume(10.0, 2.0);
    close(
        MassCalculator::compute_from_brep(&rounded, &params()).volume,
        expected,
        2e-11,
    );
}

#[test]
fn a_full_circle_edge_imported_from_opencascade_is_selectable() {
    let path = concat!(
        env!("CARGO_MANIFEST_DIR"),
        "/tests/fixtures/occ_reference_cylinder.step"
    );
    let cylinder = StepImporter::import_solids_from_file(path)
        .expect("read OCC cylinder")
        .into_iter()
        .next()
        .expect("one cylinder");
    let listed = EdgeBlender::blendable_edges(&cylinder);
    assert_eq!(
        listed.len(),
        2,
        "OCC stores each rim as one full circle edge"
    );

    let rounded = EdgeBlender::fillet_edge(&cylinder, listed[0].edge_id, 2.0)
        .expect("a full-circle imported rim should use the same exact construction");
    let before = MassCalculator::compute_from_brep(&cylinder, &params()).volume;
    let after = MassCalculator::compute_from_brep(&rounded, &params()).volume;
    close(before - after, expected_removed_volume(10.0, 2.0), 3e-10);
}

#[test]
fn the_torus_patches_survive_a_step_round_trip() {
    let rounded = FilletBuilder::fillet_cylinder_top_edge(10.0, 40.0, 2.0, &Tolerance::default())
        .expect("rounded cylinder");
    let step = StepExporter::export_solid_to_string(&rounded, "CIRCULAR_EDGE_FILLET");
    let reread = StepImporter::import_solids_from_str(&step)
        .expect("read our STEP back")
        .into_iter()
        .next()
        .expect("one rounded cylinder");
    let report = reread.outer_shell.validate_closed(&Tolerance::default());
    assert!(report.is_valid(), "{:#?}", report.errors);

    let expected = PI * 10.0 * 10.0 * 40.0 - expected_removed_volume(10.0, 2.0);
    close(
        MassCalculator::compute_from_brep(&reread, &params()).volume,
        expected,
        2e-10,
    );
}

#[test]
fn the_blend_is_tangent_to_both_supporting_faces() {
    let rounded = FilletBuilder::fillet_cylinder_top_edge(10.0, 40.0, 2.0, &Tolerance::default())
        .expect("rounded cylinder");

    // Construction order is four cylindrical patches, four torus patches,
    // then the two caps. Reversed profile rows put u=0 on the top cap and u=1
    // on the cylindrical side.
    for (quadrant, face) in rounded.outer_shell.faces[4..8].iter().enumerate() {
        let FaceGeometry::Nurbs(surface) = &face.geometry else {
            panic!("blend face {quadrant} is not an exact NURBS torus patch");
        };
        let top = surface.normal(0.0, 0.5).expect("top contact normal");
        assert!(
            top.dot(&Vec3::new(0.0, 0.0, 1.0)) > 1.0 - 1e-12,
            "quadrant {quadrant} is not tangent to the top cap: {top:?}"
        );

        let side = surface.normal(1.0, 0.5).expect("side contact normal");
        assert!(
            side.z.abs() < 1e-12,
            "quadrant {quadrant} is not tangent to the cylinder: {side:?}"
        );
        assert!((side.x * side.x + side.y * side.y).sqrt() > 1.0 - 1e-12);
    }
}

#[test]
fn the_output_mesh_is_closed_across_requested_densities() {
    let rounded = FilletBuilder::fillet_cylinder_top_edge(10.0, 40.0, 2.0, &Tolerance::default())
        .expect("rounded cylinder");

    for divisions in [4, 6, 8, 12, 16, 24, 32] {
        let mesh = tessellate_solid(
            &rounded,
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
        let non_manifold = uses.values().filter(|count| **count != 2).count();
        assert_eq!(
            non_manifold, 0,
            "{divisions} divisions left non-manifold mesh edges"
        );
        assert_eq!(
            degenerate, 0,
            "{divisions} divisions made zero-area triangles"
        );
    }
}

#[test]
fn a_circular_hole_edge_is_not_misread_as_a_pure_cylinder_rim() {
    let drilled = HoleBuilder::make_drilled_box(30.0, 30.0, 15.0, 5.0).unwrap();
    let listed = EdgeBlender::blendable_edges(&drilled);
    let mut curved_ids = Vec::new();
    for face in &drilled.outer_shell.faces {
        for wire in std::iter::once(&face.outer_wire).chain(face.inner_wires.iter()) {
            for oriented in &wire.edges {
                let edge = &oriented.edge;
                let chord = edge.end_vertex.point - edge.start_vertex.point;
                let middle = edge.curve.evaluate(0.5);
                let deviation = if chord.norm() <= 1e-12 {
                    (middle - edge.start_vertex.point).norm()
                } else {
                    let direction = chord.normalize();
                    let offset = middle - edge.start_vertex.point;
                    (offset - direction * offset.dot(&direction)).norm()
                };
                if deviation > 1e-6 && !curved_ids.contains(&edge.id) {
                    curved_ids.push(edge.id);
                }
            }
        }
    }
    assert!(
        !curved_ids.is_empty(),
        "fixture must contain its circular bore rims"
    );
    for edge_id in curved_ids {
        assert!(
            !listed.iter().any(|edge| edge.edge_id == edge_id),
            "a bore rim was advertised as the rim of a pure cylinder"
        );
        assert!(
            EdgeBlender::fillet_edge(&drilled, edge_id, 1.0).is_err(),
            "unsupported boss/bore topology must be refused, not rebuilt as a cylinder"
        );
        assert!(
            EdgeBlender::chamfer_edge(&drilled, edge_id, 1.0).is_err(),
            "unsupported boss/bore topology must not be rebuilt by circular chamfering"
        );
    }
}

#[test]
fn the_cylinder_top_chamfer_matches_closed_volume_and_area() {
    let tol = Tolerance::default();
    for (radius, height, distance) in [(10.0, 40.0, 2.0), (7.5, 12.0, 0.5), (20.0, 8.0, 3.0)] {
        let solid = ChamferBuilder::chamfer_cylinder_top_edge(radius, height, distance, &tol)
            .unwrap_or_else(|error| panic!("r={radius}, h={height}, c={distance}: {error}"));
        let report = solid.outer_shell.validate_closed(&tol);
        assert!(report.is_valid(), "{:#?}", report.errors);
        assert_eq!(solid.outer_shell.faces.len(), 10);

        let measured = MassCalculator::compute_from_brep(&solid, &params());
        close(
            measured.volume,
            PI * radius * radius * height - expected_chamfer_removed_volume(radius, distance),
            2e-11,
        );
        close(
            measured.surface_area,
            expected_chamfer_area(radius, height, distance),
            2e-11,
        );
    }
}

#[test]
fn the_circular_chamfer_reference_case_agrees_with_opencascade() {
    let solid = ChamferBuilder::chamfer_cylinder_top_edge(10.0, 40.0, 2.0, &Tolerance::default())
        .expect("the documented reference case");
    let measured = MassCalculator::compute_from_brep(&solid, &params());

    // FreeCAD 1.1.1 / OpenCASCADE, Part.makeCylinder(10, 40), circular cap
    // edge, makeChamfer(2): valid closed solid, four faces and five edges.
    close(measured.volume, 12449.084488625153, 2e-11);
    close(measured.surface_area, 3062.7753976906697, 2e-11);
}

#[test]
fn selecting_one_arc_chamfers_the_complete_circular_rim() {
    let cylinder = PrimitiveBuilder::make_cylinder(10.0, 40.0).unwrap();
    let top = cap_edge_ids(&cylinder, Vec3::new(0.0, 0.0, 1.0));
    let listed = EdgeBlender::blendable_edges(&cylinder);
    for edge_id in &top {
        let candidate = listed
            .iter()
            .find(|edge| edge.edge_id == *edge_id)
            .expect("every exact cap arc should be advertised");
        assert!(candidate.max_chamfer_distance > 9.9);
    }

    let (chamfered, report) = EdgeBlender::blend_edge(
        &cylinder,
        top[1],
        zenith_algo::BlendKind::Chamfer { distance: 2.0 },
    )
    .expect("one selected arc should chamfer its complete smooth chain");
    close(report.edge_length, 2.0 * PI * 10.0, 1e-13);
    close(
        report.predicted_removed_volume,
        expected_chamfer_removed_volume(10.0, 2.0),
        1e-13,
    );
    close(
        MassCalculator::compute_from_brep(&chamfered, &params()).volume,
        PI * 100.0 * 40.0 - report.predicted_removed_volume,
        2e-11,
    );
}

#[test]
fn circular_chamfer_handles_either_cap_and_rigid_placement() {
    let cylinder = PrimitiveBuilder::make_cylinder(10.0, 40.0).unwrap();
    let turn = Transform3::from_axis_angle(&Vec3::new(-1.0, 0.5, 2.0), 37f64.to_radians());
    let moved = BrepTransform::translate_solid(
        &BrepTransform::transform_solid(&cylinder, &turn).unwrap(),
        Vec3::new(-23.0, 14.0, 31.0),
    );
    let axis = turn.transform_vector(&Vec3::new(0.0, 0.0, 1.0));
    let expected = PI * 100.0 * 40.0 - expected_chamfer_removed_volume(10.0, 2.0);

    for direction in [axis, -axis] {
        let ids = cap_edge_ids(&moved, direction);
        assert_eq!(ids.len(), 4);
        let chamfered = EdgeBlender::chamfer_edge(&moved, ids[2], 2.0)
            .expect("either placed convex cap should use the exact circular path");
        close(
            MassCalculator::compute_from_brep(&chamfered, &params()).volume,
            expected,
            2e-11,
        );
    }
}

#[test]
fn an_opencascade_full_circle_can_be_chamfered() {
    let path = concat!(
        env!("CARGO_MANIFEST_DIR"),
        "/tests/fixtures/occ_reference_cylinder.step"
    );
    let cylinder = StepImporter::import_solids_from_file(path)
        .expect("read OCC cylinder")
        .into_iter()
        .next()
        .expect("one cylinder");
    let listed = EdgeBlender::blendable_edges(&cylinder);
    assert_eq!(listed.len(), 2);
    assert!(listed.iter().all(|edge| edge.max_chamfer_distance > 9.9));

    let chamfered = EdgeBlender::chamfer_edge(&cylinder, listed[1].edge_id, 2.0)
        .expect("the imported full-circle rim should be selectable");
    let before = MassCalculator::compute_from_brep(&cylinder, &params()).volume;
    let after = MassCalculator::compute_from_brep(&chamfered, &params()).volume;
    close(
        before - after,
        expected_chamfer_removed_volume(10.0, 2.0),
        3e-10,
    );
}

#[test]
fn circular_chamfer_survives_step_and_mesh_generation() {
    let chamfered =
        ChamferBuilder::chamfer_cylinder_top_edge(10.0, 40.0, 2.0, &Tolerance::default())
            .expect("chamfered cylinder");
    let step = StepExporter::export_solid_to_string(&chamfered, "CIRCULAR_EDGE_CHAMFER");
    let reread = StepImporter::import_solids_from_str(&step)
        .expect("read our STEP back")
        .into_iter()
        .next()
        .expect("one chamfered cylinder");
    assert!(reread
        .outer_shell
        .validate_closed(&Tolerance::default())
        .is_valid());

    for divisions in [4, 8, 16, 32] {
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
fn unusable_circular_chamfer_distances_are_refused() {
    let tol = Tolerance::default();
    assert!(ChamferBuilder::chamfer_cylinder_top_edge(10.0, 40.0, -1.0, &tol).is_err());
    assert!(ChamferBuilder::chamfer_cylinder_top_edge(10.0, 40.0, 10.0, &tol).is_err());
    assert!(ChamferBuilder::chamfer_cylinder_top_edge(10.0, 2.0, 2.0, &tol).is_err());
}
