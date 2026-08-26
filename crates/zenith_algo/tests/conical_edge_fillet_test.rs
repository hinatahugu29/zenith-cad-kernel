use std::collections::HashMap;
use std::f64::consts::{FRAC_PI_2, PI};

use zenith_algo::{
    BrepTransform, ChamferBuilder, EdgeBlender, FilletBuilder, MassCalculator, PrimitiveBuilder,
};
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

#[derive(Clone, Copy)]
struct Expected {
    volume: f64,
    area: f64,
    removed: f64,
    dihedral: f64,
    setback: f64,
}

/// Independent meridian integration for a fillet on the `r_top` cap.
fn expected(r_bottom: f64, r_top: f64, height: f64, fillet: f64) -> Expected {
    let slope = (r_top - r_bottom) / height;
    let norm = slope.hypot(1.0);
    let centre_z = height - fillet;
    let centre_radius = r_top - fillet * (norm + slope);
    let side_z = centre_z - fillet * slope / norm;
    let side_radius = centre_radius + fillet / norm;
    let side_angle = (-slope).atan();

    let lower_volume =
        PI * side_z * (r_bottom * r_bottom + r_bottom * side_radius + side_radius * side_radius)
            / 3.0;
    let primitive = |angle: f64| {
        let sine = angle.sin();
        let cosine = angle.cos();
        fillet * centre_radius * centre_radius * sine
            + fillet * fillet * centre_radius * (angle + sine * cosine)
            + fillet.powi(3) * (sine - sine.powi(3) / 3.0)
    };
    let upper_volume = PI * (primitive(FRAC_PI_2) - primitive(side_angle));
    let original = PI * height * (r_bottom * r_bottom + r_bottom * r_top + r_top * r_top) / 3.0;

    let side_slant = (side_z * side_z + (side_radius - r_bottom).powi(2)).sqrt();
    let side_area = PI * (r_bottom + side_radius) * side_slant;
    let bottom_area = PI * r_bottom * r_bottom;
    let top_area = PI * centre_radius * centre_radius;
    let torus_area = 2.0
        * PI
        * fillet
        * (centre_radius * (FRAC_PI_2 - side_angle) + fillet * (1.0 - side_angle.sin()));

    Expected {
        volume: lower_volume + upper_volume,
        area: bottom_area + side_area + top_area + torus_area,
        removed: original - lower_volume - upper_volume,
        dihedral: FRAC_PI_2 - slope.atan(),
        setback: fillet * (norm + slope),
    }
}

fn expected_chamfer(r_bottom: f64, r_top: f64, height: f64, distance: f64) -> Expected {
    let slope = (r_top - r_bottom) / height;
    let norm = slope.hypot(1.0);
    let side_z = height - distance / norm;
    let side_radius = r_top - slope * distance / norm;
    let cap_radius = r_top - distance;
    let upper_height = height - side_z;
    let lower_volume =
        PI * side_z * (r_bottom * r_bottom + r_bottom * side_radius + side_radius * side_radius)
            / 3.0;
    let upper_volume = PI
        * upper_height
        * (side_radius * side_radius + side_radius * cap_radius + cap_radius * cap_radius)
        / 3.0;
    let original = PI * height * (r_bottom * r_bottom + r_bottom * r_top + r_top * r_top) / 3.0;
    let side_slant = (side_z * side_z + (side_radius - r_bottom).powi(2)).sqrt();
    let chamfer_slant = (upper_height * upper_height + (side_radius - cap_radius).powi(2)).sqrt();
    let area = PI * r_bottom * r_bottom
        + PI * (r_bottom + side_radius) * side_slant
        + PI * (side_radius + cap_radius) * chamfer_slant
        + PI * cap_radius * cap_radius;
    Expected {
        volume: lower_volume + upper_volume,
        area,
        removed: original - lower_volume - upper_volume,
        dihedral: FRAC_PI_2 - slope.atan(),
        setback: distance,
    }
}

fn cap_edge_ids(solid: &Solid, direction: Vec3) -> Vec<u64> {
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
fn exact_conical_rim_matches_independent_meridian_integrals() {
    let tol = Tolerance::default();
    for (bottom, top, height, fillet) in [
        (10.0, 4.0, 20.0, 1.0),
        (4.0, 10.0, 20.0, 1.0),
        (0.0, 10.0, 20.0, 1.0),
        (12.0, 7.0, 8.0, 0.5),
    ] {
        let solid = FilletBuilder::fillet_cone_top_edge(bottom, top, height, fillet, &tol)
            .unwrap_or_else(|error| {
                panic!("bottom={bottom}, top={top}, h={height}, r={fillet}: {error}")
            });
        let report = solid.outer_shell.validate_closed(&tol);
        assert!(report.is_valid(), "{:#?}", report.errors);
        let measured = MassCalculator::compute_from_brep(&solid, &params());
        let expected = expected(bottom, top, height, fillet);
        close(measured.volume, expected.volume, 3e-11);
        close(measured.surface_area, expected.area, 3e-11);
    }
}

#[test]
fn exact_conical_chamfer_matches_independent_meridian_integrals() {
    let tol = Tolerance::default();
    for (bottom, top, height, distance) in [
        (10.0, 4.0, 20.0, 1.0),
        (4.0, 10.0, 20.0, 1.0),
        (0.0, 10.0, 20.0, 1.0),
        (12.0, 7.0, 8.0, 0.5),
    ] {
        let solid = ChamferBuilder::chamfer_cone_top_edge(bottom, top, height, distance, &tol)
            .unwrap_or_else(|error| {
                panic!("bottom={bottom}, top={top}, h={height}, c={distance}: {error}")
            });
        let report = solid.outer_shell.validate_closed(&tol);
        assert!(report.is_valid(), "{:#?}", report.errors);
        let measured = MassCalculator::compute_from_brep(&solid, &params());
        let expected = expected_chamfer(bottom, top, height, distance);
        close(measured.volume, expected.volume, 3e-11);
        close(measured.surface_area, expected.area, 3e-11);
    }
}

#[test]
fn both_frustum_rims_match_opencascade_reference_values() {
    let cone = PrimitiveBuilder::make_cone(10.0, 4.0, 20.0).unwrap();
    let top = cap_edge_ids(&cone, Vec3::new(0.0, 0.0, 1.0));
    let bottom = cap_edge_ids(&cone, Vec3::new(0.0, 0.0, -1.0));
    assert_eq!(top.len(), 4);
    assert_eq!(bottom.len(), 4);

    let rounded_top = EdgeBlender::fillet_edge(&cone, top[0], 1.0).expect("top cone rim");
    let top_props = MassCalculator::compute_from_brep(&rounded_top, &params());
    close(top_props.volume, 3264.7082284293483, 3e-11);
    close(top_props.surface_area, 1277.2926805751288, 3e-11);

    let rounded_bottom = EdgeBlender::fillet_edge(&cone, bottom[0], 1.0).expect("bottom cone rim");
    let bottom_props = MassCalculator::compute_from_brep(&rounded_bottom, &params());
    close(bottom_props.volume, 3242.364474159573, 3e-11);
    close(bottom_props.surface_area, 1230.5830568214212, 3e-11);

    for (edge, want) in [
        (top[1], expected_chamfer(10.0, 4.0, 20.0, 1.0)),
        (bottom[1], expected_chamfer(4.0, 10.0, 20.0, 1.0)),
    ] {
        let chamfered = EdgeBlender::chamfer_edge(&cone, edge, 1.0).unwrap();
        let measured = MassCalculator::compute_from_brep(&chamfered, &params());
        close(measured.volume, want.volume, 3e-11);
        close(measured.surface_area, want.area, 3e-11);
    }
}

#[test]
fn one_selected_arc_propagates_around_the_non_right_angle_rim() {
    let cone = PrimitiveBuilder::make_cone(10.0, 4.0, 20.0).unwrap();
    let top = cap_edge_ids(&cone, Vec3::new(0.0, 0.0, 1.0));
    let candidates = EdgeBlender::blendable_edges(&cone);
    assert_eq!(candidates.len(), 8, "both four-arc rims are blendable");
    let candidate = candidates
        .iter()
        .find(|candidate| candidate.edge_id == top[2])
        .unwrap();
    let expected = expected(10.0, 4.0, 20.0, 1.0);
    close(
        candidate.dihedral_angle_deg.to_radians(),
        expected.dihedral,
        1e-13,
    );
    assert!(candidate.max_chamfer_distance > 3.0);

    let (_, report) = EdgeBlender::blend_edge(
        &cone,
        top[2],
        zenith_algo::BlendKind::Fillet { radius: 1.0 },
    )
    .unwrap();
    close(report.edge_length, 2.0 * PI * 4.0, 1e-13);
    close(report.setback, expected.setback, 1e-13);
    close(report.predicted_removed_volume, expected.removed, 3e-13);

    let (_, report) = EdgeBlender::blend_edge(
        &cone,
        top[1],
        zenith_algo::BlendKind::Chamfer { distance: 1.0 },
    )
    .unwrap();
    let expected = expected_chamfer(10.0, 4.0, 20.0, 1.0);
    close(report.edge_length, 2.0 * PI * 4.0, 1e-13);
    close(report.setback, 1.0, 1e-13);
    close(report.predicted_removed_volume, expected.removed, 3e-13);
}

#[test]
fn a_true_cone_base_rim_is_supported_without_an_artificial_apex_cap() {
    let cone = PrimitiveBuilder::make_cone(10.0, 0.0, 20.0).unwrap();
    let base = cap_edge_ids(&cone, Vec3::new(0.0, 0.0, -1.0));
    assert_eq!(base.len(), 4);
    let candidates = EdgeBlender::blendable_edges(&cone);
    assert_eq!(candidates.len(), 4);

    let rounded = EdgeBlender::fillet_edge(&cone, base[1], 1.0).expect("true cone base fillet");
    assert_eq!(rounded.outer_shell.faces.len(), 9);
    assert_eq!(
        rounded
            .outer_shell
            .faces
            .iter()
            .filter(|face| matches!(face.geometry, FaceGeometry::Plane(_)))
            .count(),
        1,
        "the apex must stay a point, not become a tiny planar cap"
    );
    let measured = MassCalculator::compute_from_brep(&rounded, &params());
    let want = expected(0.0, 10.0, 20.0, 1.0);
    close(measured.volume, want.volume, 3e-11);
    close(measured.surface_area, want.area, 3e-11);

    let chamfered = EdgeBlender::chamfer_edge(&cone, base[2], 1.0).unwrap();
    assert_eq!(
        chamfered
            .outer_shell
            .faces
            .iter()
            .filter(|face| matches!(face.geometry, FaceGeometry::Plane(_)))
            .count(),
        1
    );
    let measured = MassCalculator::compute_from_brep(&chamfered, &params());
    let want = expected_chamfer(0.0, 10.0, 20.0, 1.0);
    close(measured.volume, want.volume, 3e-11);
    close(measured.surface_area, want.area, 3e-11);
}

#[test]
fn an_occ_full_circle_cone_rim_is_selectable() {
    let path = concat!(
        env!("CARGO_MANIFEST_DIR"),
        "/tests/fixtures/occ_reference_cone.step"
    );
    let cone = StepImporter::import_solids_from_file(path)
        .unwrap()
        .into_iter()
        .next()
        .unwrap();
    let candidates = EdgeBlender::blendable_edges(&cone);
    assert_eq!(candidates.len(), 2);
    let small = candidates
        .iter()
        .min_by(|left, right| left.length.total_cmp(&right.length))
        .unwrap();
    let rounded = EdgeBlender::fillet_edge(&cone, small.edge_id, 1.0).unwrap();
    let measured = MassCalculator::compute_from_brep(&rounded, &params());
    close(measured.volume, 3264.7082284293483, 3e-10);
    close(measured.surface_area, 1277.2926805751288, 3e-10);
    let chamfered = EdgeBlender::chamfer_edge(&cone, small.edge_id, 1.0).unwrap();
    let measured = MassCalculator::compute_from_brep(&chamfered, &params());
    let want = expected_chamfer(10.0, 4.0, 20.0, 1.0);
    close(measured.volume, want.volume, 3e-10);
    close(measured.surface_area, want.area, 3e-10);
}

#[test]
fn an_occ_true_cone_full_circle_keeps_its_real_apex() {
    let path = concat!(
        env!("CARGO_MANIFEST_DIR"),
        "/tests/fixtures/occ_reference_cone_full.step"
    );
    let cone = StepImporter::import_solids_from_file(path)
        .unwrap()
        .into_iter()
        .next()
        .unwrap();
    let candidates = EdgeBlender::blendable_edges(&cone);
    assert_eq!(candidates.len(), 1);
    let rounded = EdgeBlender::fillet_edge(&cone, candidates[0].edge_id, 1.0).unwrap();
    assert_eq!(
        rounded
            .outer_shell
            .faces
            .iter()
            .filter(|face| matches!(face.geometry, FaceGeometry::Plane(_)))
            .count(),
        1
    );
    let measured = MassCalculator::compute_from_brep(&rounded, &params());
    let want = expected(0.0, 10.0, 20.0, 1.0);
    close(measured.volume, want.volume, 3e-10);
    close(measured.surface_area, want.area, 3e-10);
    let chamfered = EdgeBlender::chamfer_edge(&cone, candidates[0].edge_id, 1.0).unwrap();
    assert_eq!(
        chamfered
            .outer_shell
            .faces
            .iter()
            .filter(|face| matches!(face.geometry, FaceGeometry::Plane(_)))
            .count(),
        1
    );
    let measured = MassCalculator::compute_from_brep(&chamfered, &params());
    let want = expected_chamfer(0.0, 10.0, 20.0, 1.0);
    close(measured.volume, want.volume, 3e-10);
    close(measured.surface_area, want.area, 3e-10);
}

#[test]
fn conical_rim_recognition_is_invariant_under_rigid_placement() {
    let cone = PrimitiveBuilder::make_cone(10.0, 4.0, 20.0).unwrap();
    let turn = Transform3::from_axis_angle(&Vec3::new(2.0, -1.0, 0.5), 33f64.to_radians());
    let moved = BrepTransform::translate_solid(
        &BrepTransform::transform_solid(&cone, &turn).unwrap(),
        Vec3::new(28.0, -11.0, 17.0),
    );
    let axis = turn.transform_vector(&Vec3::new(0.0, 0.0, 1.0));
    let top = cap_edge_ids(&moved, axis);
    assert_eq!(top.len(), 4);
    let rounded = EdgeBlender::fillet_edge(&moved, top[3], 1.0).unwrap();
    close(
        MassCalculator::compute_from_brep(&rounded, &params()).volume,
        expected(10.0, 4.0, 20.0, 1.0).volume,
        3e-11,
    );
    let chamfered = EdgeBlender::chamfer_edge(&moved, top[2], 1.0).unwrap();
    close(
        MassCalculator::compute_from_brep(&chamfered, &params()).volume,
        expected_chamfer(10.0, 4.0, 20.0, 1.0).volume,
        3e-11,
    );
}

#[test]
fn torus_patch_is_tangent_to_the_cap_and_conical_side() {
    let rounded =
        FilletBuilder::fillet_cone_top_edge(10.0, 4.0, 20.0, 1.0, &Tolerance::default()).unwrap();
    let slope: f64 = (4.0 - 10.0) / 20.0;
    for (quadrant, face) in rounded.outer_shell.faces[4..8].iter().enumerate() {
        let FaceGeometry::Nurbs(surface) = &face.geometry else {
            panic!("blend face is not NURBS");
        };
        let top = surface.normal(0.0, 0.5).unwrap();
        assert!(top.dot(&Vec3::new(0.0, 0.0, 1.0)) > 1.0 - 1e-12);
        let angle = (quadrant as f64 + 0.5) * FRAC_PI_2;
        let expected_side = Vec3::new(angle.cos(), angle.sin(), -slope).normalize();
        let side = surface.normal(1.0, 0.5).unwrap();
        assert!(side.dot(&expected_side) > 1.0 - 1e-12);
    }
}

#[test]
fn conical_blends_survive_step_and_coarse_to_fine_meshes() {
    let tol = Tolerance::default();
    for (name, blended) in [
        (
            "CONICAL_EDGE_FILLET",
            FilletBuilder::fillet_cone_top_edge(10.0, 4.0, 20.0, 1.0, &tol).unwrap(),
        ),
        (
            "CONICAL_EDGE_CHAMFER",
            ChamferBuilder::chamfer_cone_top_edge(10.0, 4.0, 20.0, 1.0, &tol).unwrap(),
        ),
    ] {
        let step = StepExporter::export_solid_to_string(&blended, name);
        let reread = StepImporter::import_solids_from_str(&step)
            .unwrap()
            .into_iter()
            .next()
            .unwrap();
        assert!(reread.outer_shell.validate_closed(&tol).is_valid());

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
            assert_eq!(uses.values().filter(|count| **count != 2).count(), 0);
            assert_eq!(degenerate, 0);
        }
    }
}

#[test]
fn stepped_shaft_roots_are_not_misrecognized_as_one_pure_cone() {
    let shaft = zenith_algo::ShaftBuilder::make_stepped_shaft(&[(10.0, 12.0), (7.0, 10.0)])
        .expect("stepped shaft");
    let candidates = EdgeBlender::blendable_edges(&shaft);
    assert_eq!(
        candidates.len(),
        4,
        "the four root arcs must propagate as one site"
    );
    assert!(candidates.iter().all(|edge| {
        (edge.dihedral_angle_deg - 270.0).abs() < 1e-12
            && edge.max_fillet_radius > 0.0
            && edge.max_chamfer_distance > 0.0
    }));
}

#[test]
fn impossible_conical_fillet_radii_are_refused() {
    let tol = Tolerance::default();
    assert!(FilletBuilder::fillet_cone_top_edge(10.0, 4.0, 20.0, -1.0, &tol).is_err());
    assert!(FilletBuilder::fillet_cone_top_edge(10.0, 4.0, 20.0, 6.0, &tol).is_err());
    assert!(FilletBuilder::fillet_cone_top_edge(10.0, 10.0, 20.0, 1.0, &tol).is_err());
    assert!(ChamferBuilder::chamfer_cone_top_edge(10.0, 4.0, 20.0, -1.0, &tol).is_err());
    assert!(ChamferBuilder::chamfer_cone_top_edge(10.0, 4.0, 20.0, 4.0, &tol).is_err());
    assert!(ChamferBuilder::chamfer_cone_top_edge(10.0, 10.0, 20.0, 1.0, &tol).is_err());
}
