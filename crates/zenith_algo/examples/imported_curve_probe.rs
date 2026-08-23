//! Inspects a curve as the importer reconstructs it.
//!
//! A cap read from an OpenCASCADE file comes out 10% small. The weights are
//! parsed, so the next suspect is the knot vector: OpenCASCADE writes closed
//! curves unclamped, with end multiplicity 1 rather than degree+1, and an
//! unclamped curve is only valid over part of its knot range.
//!
//! Run with: cargo run --release -p zenith_algo --example imported_curve_probe

use std::path::Path;

use zenith_io::StepImporter;

fn main() {
    let path = Path::new("target/validation/occ_reference_cylinder_nurbs.step");
    let Ok(content) = std::fs::read_to_string(path) else {
        println!("missing {}; run tools/occ_reference_export.py", path.display());
        return;
    };

    // 平面の面を拾って、面積とp-curve導出の成否を見る。
    for id in 1..200u64 {
        let Ok(face) = StepImporter::import_face_from_str(&content, id) else {
            continue;
        };
        if !matches!(face.geometry, zenith_topo::FaceGeometry::Plane(_)) {
            continue;
        }

        let (area, _) = zenith_algo::MassCalculator::compute_face_integral(
            &face,
            &zenith_tess::TessellationParams {
                u_divisions: 32,
                v_divisions: 32,
            },
        );
        let pcurves = face.plane_pcurves();
        println!(
            "planar face #{id}: area {area:.4}, boundary edges {}, p-curves {}",
            face.outer_wire.edges.len(),
            match &pcurves {
                Ok(loops) => format!("ok ({} segment(s))", loops.outer_loop.segments.len()),
                Err(err) => format!("FAILED: {err}"),
            }
        );
    }

    // ファイル内の EDGE_CURVE を総当たりで拾う。
    for id in 1..200u64 {
        let Ok(edge) = StepImporter::import_edge_from_str(&content, id) else {
            continue;
        };
        let curve = &edge.curve;
        let (t_min, t_max) = curve.param_range();

        let samples = 512;
        let mut length = 0.0;
        let mut previous = curve.evaluate(t_min);
        let mut min_radius = f64::INFINITY;
        let mut max_radius: f64 = 0.0;
        for index in 1..=samples {
            let t = t_min + (t_max - t_min) * (index as f64 / samples as f64);
            let point = curve.evaluate(t);
            length += (point - previous).norm();
            previous = point;
            let radius = (point.x * point.x + point.y * point.y).sqrt();
            min_radius = min_radius.min(radius);
            max_radius = max_radius.max(radius);
        }

        println!(
            "edge #{id}: degree {}, {} control point(s), param [{t_min}, {t_max}]",
            curve.degree,
            curve.control_points.len()
        );
        println!(
            "    knots {:?}",
            curve.knots.knots.iter().map(|k| (k * 1000.0).round() / 1000.0).collect::<Vec<_>>()
        );
        println!(
            "    weights {:?}",
            curve
                .control_points
                .iter()
                .map(|cp| (cp.weight * 1000.0).round() / 1000.0)
                .collect::<Vec<_>>()
        );
        println!(
            "    length {length:.4}, radius from axis {min_radius:.4} to {max_radius:.4}"
        );
    }
}
