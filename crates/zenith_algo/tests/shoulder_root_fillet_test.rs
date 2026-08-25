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
        assert_eq!(candidate.max_chamfer_distance, 0.0);
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
