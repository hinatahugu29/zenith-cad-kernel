//! Measures what the STEP importer actually reads back.
//!
//! Export has been verified against OpenCASCADE; import has not been measured
//! at all. Two questions matter for an addon that opens other people's files:
//! does a solid survive a round trip through our own writer and reader, and can
//! the reader open a file written by a different kernel?
//!
//! Run with: cargo run --release -p zenith_algo --example step_import_audit

use std::f64::consts::PI;
use std::fs;
use std::path::Path;

use zenith_algo::{MassCalculator, PrimitiveBuilder};
use zenith_io::{StepExporter, StepImporter};
use zenith_math::Tolerance;
use zenith_tess::TessellationParams;
use zenith_topo::Solid;

fn volume(solid: &Solid) -> f64 {
    MassCalculator::compute_from_brep(
        solid,
        &TessellationParams {
            u_divisions: 32,
            v_divisions: 32,
        },
    )
    .volume
}

fn round_trip(name: &str, solid: &Solid, analytic: Option<f64>) {
    let tol = Tolerance::default();
    let original_volume = volume(solid);
    let original_faces = solid.outer_shell.faces.len();

    let step = StepExporter::export_solid_to_string(solid, name);

    match StepImporter::import_solid_from_str(&step) {
        Ok(imported) => {
            let imported_volume = volume(&imported);
            let relative =
                (imported_volume - original_volume).abs() / original_volume.abs().max(1e-12);
            let shell_ok = imported.outer_shell.validate_closed(&tol).is_valid();

            let analytic_note = analytic
                .map(|expected| {
                    format!(
                        " vs analytic {:.2e}",
                        (imported_volume - expected).abs() / expected.abs()
                    )
                })
                .unwrap_or_default();

            println!(
                "{name:<28} {original_faces:>3} -> {:>3} faces  volume {original_volume:>13.4} -> {imported_volume:>13.4}  rel {relative:.2e}  shell {}{analytic_note}",
                imported.outer_shell.faces.len(),
                if shell_ok { "valid" } else { "INVALID" }
            );
        }
        Err(err) => println!("{name:<28} IMPORT FAILED: {err}"),
    }
}

fn read_foreign(path: &Path) {
    let name = path.file_name().unwrap().to_string_lossy().to_string();
    match StepImporter::import_solids_from_file(path) {
        Ok(solids) => {
            if solids.is_empty() {
                println!("{name:<44} read 0 solids");
                return;
            }
            let total: f64 = solids.iter().map(volume).sum();
            let faces: usize = solids
                .iter()
                .map(|solid| solid.outer_shell.faces.len())
                .sum();
            println!(
                "{name:<44} {} solid(s), {faces} face(s), volume {total:.4}",
                solids.len()
            );
            for solid in &solids {
                let tol = Tolerance::default();
                let report = solid.outer_shell.validate_closed(&tol);
                if !report.is_valid() {
                    println!(
                        "        shell invalid: {}",
                        report.errors.first().cloned().unwrap_or_default()
                    );
                }
                for (index, face) in solid.outer_shell.faces.iter().enumerate() {
                    let (area, contribution) = MassCalculator::compute_face_integral(
                        face,
                        &TessellationParams {
                            u_divisions: 64,
                            v_divisions: 64,
                        },
                    );
                    println!(
                        "        face {index}: area {area:.4}, volume share {contribution:.4}"
                    );
                }
            }
        }
        Err(err) => println!(
            "{name:<44} FAILED: {}",
            err.chars().take(400).collect::<String>()
        ),
    }
}

fn main() {
    println!("=== round trip through our own writer and reader");
    round_trip(
        "box",
        &PrimitiveBuilder::make_box(20.0, 30.0, 40.0).unwrap(),
        Some(24000.0),
    );
    round_trip(
        "cylinder",
        &PrimitiveBuilder::make_cylinder(10.0, 40.0).unwrap(),
        Some(PI * 100.0 * 40.0),
    );
    round_trip(
        "sphere",
        &PrimitiveBuilder::make_sphere(10.0).unwrap(),
        Some(4.0 / 3.0 * PI * 1000.0),
    );
    round_trip(
        "cone",
        &PrimitiveBuilder::make_cone(10.0, 4.0, 20.0).unwrap(),
        Some(PI * 20.0 / 3.0 * (100.0 + 40.0 + 16.0)),
    );
    round_trip(
        "torus",
        &PrimitiveBuilder::make_torus(12.0, 4.0).unwrap(),
        Some(2.0 * PI * PI * 12.0 * 16.0),
    );

    println!();
    println!("=== reading the showcase files back");
    let showcase = Path::new("target/showcase");
    if showcase.is_dir() {
        let mut names: Vec<_> = fs::read_dir(showcase)
            .unwrap()
            .filter_map(|entry| entry.ok())
            .map(|entry| entry.path())
            .filter(|path| path.extension().map(|ext| ext == "step").unwrap_or(false))
            .collect();
        names.sort();
        for path in names {
            read_foreign(&path);
        }
    } else {
        println!("    target/showcase is missing; run the export_showcase example first");
    }

    println!();
    println!("=== reading files written by OpenCASCADE");
    let validation = Path::new("target/validation");
    if validation.is_dir() {
        let mut names: Vec<_> = fs::read_dir(validation)
            .unwrap()
            .filter_map(|entry| entry.ok())
            .map(|entry| entry.path())
            .filter(|path| {
                path.file_name()
                    .map(|name| name.to_string_lossy().starts_with("occ_reference"))
                    .unwrap_or(false)
            })
            .collect();
        names.sort();
        if names.is_empty() {
            println!("    no OpenCASCADE reference files; run tools/occ_reference_export.py");
        }
        for path in names {
            read_foreign(&path);
        }
    }
}
