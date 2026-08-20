//! How far a face's p-curve really is from its 3D edge.
//!
//! Shell validation asks `validate_pcurves(tol, 8)`, and a NURBS face's
//! p-curves are built by projecting each edge at 8 evenly spaced parameters.
//! The check therefore lands on the very points the p-curve was built from,
//! where it is exact by construction, and reports a deviation of zero for
//! curves that are nowhere near their edges in between.
//!
//! This probe measures the same distance at several sample counts. A p-curve
//! that is genuinely on its edge reads small at every count. One that is only
//! pinned at its construction points reads zero at 8 and large everywhere else,
//! and the gap between those two columns is the size of the illusion.
//!
//! Run with: cargo run --release -p zenith_algo --example pcurve_fidelity_probe

use std::fs;
use std::path::Path;

use zenith_geom::Surface3;
use zenith_io::StepImporter;
use zenith_math::{Point3, Tolerance};
use zenith_topo::{Face, FaceGeometry, Solid};

/// The worst distance between a face's p-curves and its 3D edges, measured at
/// `samples` evenly spaced parameters per edge - the same way validation does.
fn worst_pcurve_distance(face: &Face, samples: usize) -> Option<f64> {
    let tol = Tolerance::default();
    let pcurves = face.pcurves(&tol).ok()?;
    let evaluate = |u: f64, v: f64| -> Option<Point3> {
        match &face.geometry {
            FaceGeometry::Plane(plane) => Some(plane.evaluate(u, v)),
            FaceGeometry::Nurbs(surface) => Some(surface.evaluate(u, v)),
            _ => None,
        }
    };

    let mut worst: f64 = 0.0;
    for (edge, segment) in face
        .outer_wire
        .edges
        .iter()
        .zip(pcurves.outer_loop.segments.iter())
    {
        let (t_min, t_max) = segment.curve.param_range();
        for step in 0..=samples {
            let fraction = step as f64 / samples as f64;
            let uv = segment.curve.evaluate(t_min + (t_max - t_min) * fraction);
            let Some(from_pcurve) = evaluate(uv.x, uv.y) else {
                return None;
            };
            let from_edge = edge.evaluate_normalized(fraction);
            worst = worst.max((from_pcurve - from_edge).norm());
        }
    }
    Some(worst)
}

fn report(name: &str, solids: &[Solid]) {
    let counts = [8usize, 9, 16, 37, 64];
    for solid in solids {
        for (index, face) in solid.outer_shell.faces.iter().enumerate() {
            let kind = match &face.geometry {
                FaceGeometry::Plane(_) => "plane",
                FaceGeometry::Nurbs(_) => "nurbs",
                _ => continue,
            };
            let measured: Vec<String> = counts
                .iter()
                .map(|count| match worst_pcurve_distance(face, *count) {
                    Some(distance) => format!("{distance:>10.3e}"),
                    None => "         -".to_string(),
                })
                .collect();
            println!(
                "{:<38} face {index:>2} {kind:<6} {}",
                name,
                measured.join(" ")
            );
        }
    }
}

fn main() {
    println!("worst p-curve to 3D edge distance, by how many samples the check takes");
    println!(
        "{:<38} {:>8} {}",
        "file",
        "face",
        [8usize, 9, 16, 37, 64]
            .iter()
            .map(|count| format!("{count:>10}"))
            .collect::<Vec<_>>()
            .join(" ")
    );
    println!("{}", "-".repeat(112));
    println!("validation asks for 8, and the p-curves are built from 8; the other columns");
    println!("are the same curves measured anywhere else.");
    println!();

    let validation = Path::new("target/validation");
    if !validation.is_dir() {
        println!("target/validation is missing; run the export_validation_suite example first");
        return;
    }

    let mut paths: Vec<_> = fs::read_dir(validation)
        .unwrap()
        .filter_map(|entry| entry.ok())
        .map(|entry| entry.path())
        .filter(|path| path.extension().map(|ext| ext == "step").unwrap_or(false))
        .filter(|path| {
            path.file_name()
                .map(|name| name.to_string_lossy().starts_with("occ_reference"))
                .unwrap_or(false)
        })
        .collect();
    paths.sort();

    for path in paths {
        let name = path.file_name().unwrap().to_string_lossy().to_string();
        match StepImporter::import_solids_from_file(&path) {
            Ok(solids) => report(&name, &solids),
            Err(err) => println!(
                "{name:<38} refused: {}",
                err.chars().take(90).collect::<String>()
            ),
        }
    }
}
