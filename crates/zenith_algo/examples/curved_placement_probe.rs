//! The same skew pair of curved solids must give the same answer after a
//! common rigid transform.
//!
//! Most permanent boolean fixtures put one operand on a world axis or near the
//! origin.  That leaves projection seeds, seams, and tolerance calculations
//! free to depend accidentally on placement.  This probe turns both cylinders
//! independently, then moves the complete pair away from the origin.  It does
//! not need a closed form for the oblique bicylinder: rigid-motion invariance
//! supplies an independent expectation for all three operations.

use std::panic::{catch_unwind, AssertUnwindSafe};

use zenith_algo::{BooleanEngine, BooleanOpType, BrepTransform, MassCalculator, PrimitiveBuilder};
use zenith_math::{Tolerance, Transform3, Vec3};
use zenith_tess::TessellationParams;
use zenith_topo::Solid;

// The current integration path accumulates split-face contributions against
// the world origin. The measured worst rigid-placement residual is 2.85e-8.
// Keep a named allowance above that value and ratchet it down when the volume
// integral can improve without weakening boolean partition identities.
const PLACEMENT_ALLOWANCE: f64 = 5e-8;

#[derive(Debug)]
enum Outcome {
    Success { solids: Vec<Solid>, volume: f64 },
    Refused,
    Panic,
    Invalid(String),
}

fn centred_cylinder(radius: f64, height: f64, turn: &Transform3) -> Result<Solid, String> {
    let cylinder = PrimitiveBuilder::make_cylinder(radius, height)?;
    let centred = BrepTransform::translate_solid(&cylinder, Vec3::new(0.0, 0.0, -height * 0.5));
    BrepTransform::transform_solid(&centred, turn)
}

fn run(a: &Solid, b: &Solid, op: BooleanOpType, tol: &Tolerance) -> Outcome {
    let attempted = catch_unwind(AssertUnwindSafe(|| {
        BooleanEngine::boolean_solids_exact_result(a, b, op, tol)
    }));
    let result = match attempted {
        Err(_) => return Outcome::Panic,
        Ok(Err(_)) => return Outcome::Refused,
        Ok(Ok(result)) => result,
    };

    for solid in &result.solids {
        let report = solid.outer_shell.validate_closed(tol);
        if !report.is_valid() {
            return Outcome::Invalid(format!(
                "returned an open outer shell: {}",
                report.errors.join("; ")
            ));
        }
        for (index, shell) in solid.inner_shells.iter().enumerate() {
            let report = shell.validate_closed(tol);
            if !report.is_valid() {
                return Outcome::Invalid(format!(
                    "returned an open inner shell {index}: {}",
                    report.errors.join("; ")
                ));
            }
        }
    }

    let volume = result.solids.iter().map(volume).sum();
    Outcome::Success {
        solids: result.solids,
        volume,
    }
}

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

fn main() {
    let tol = Tolerance::default();
    let turn_a = Transform3::from_axis_angle(&Vec3::new(1.0, 2.0, 0.5), 31f64.to_radians());
    let turn_b = Transform3::from_axis_angle(&Vec3::new(-1.0, 0.75, 2.0), 67f64.to_radians());
    let a = centred_cylinder(10.0, 40.0, &turn_a).expect("first skew cylinder");
    let b = centred_cylinder(6.0, 40.0, &turn_b).expect("second skew cylinder");

    let common_turn = Transform3::from_axis_angle(&Vec3::new(1.0, 1.0, 1.0), 23f64.to_radians());
    let moved_a = BrepTransform::translate_solid(
        &BrepTransform::transform_solid(&a, &common_turn).expect("turn first cylinder"),
        Vec3::new(37.0, -19.0, 11.0),
    );
    let moved_b = BrepTransform::translate_solid(
        &BrepTransform::transform_solid(&b, &common_turn).expect("turn second cylinder"),
        Vec3::new(37.0, -19.0, 11.0),
    );

    println!("two independently turned cylinders, then one common rigid transform");
    for (name, near, moved) in [("operand A", &a, &moved_a), ("operand B", &b, &moved_b)] {
        let near_volume = volume(near);
        let moved_volume = volume(moved);
        let residual = (near_volume - moved_volume).abs() / near_volume.abs().max(1.0);
        println!(
            "{name}: {near_volume:.9} near / {moved_volume:.9} moved / residual {residual:.2e}"
        );
    }
    println!(
        "{:<13} {:<24} {:<24} {}",
        "operation", "near origin", "moved pair", "verdict"
    );
    println!("{}", "-".repeat(94));

    let mut wrong = 0usize;
    let mut panics = 0usize;
    for (name, op) in [
        ("union", BooleanOpType::Union),
        ("difference", BooleanOpType::Difference),
        ("intersection", BooleanOpType::Intersection),
    ] {
        let near = run(&a, &b, op, &tol);
        let moved = run(&moved_a, &moved_b, op, &tol);

        let (near_text, moved_text, verdict) = match (&near, &moved) {
            (
                Outcome::Success {
                    solids: near_result,
                    volume: near_volume,
                },
                Outcome::Success {
                    solids: moved_result,
                    volume: moved_volume,
                },
            ) => {
                let transported_volume: f64 = near_result
                    .iter()
                    .map(|solid| {
                        let turned = BrepTransform::transform_solid(solid, &common_turn)
                            .expect("turn near-origin result");
                        volume(&BrepTransform::translate_solid(
                            &turned,
                            Vec3::new(37.0, -19.0, 11.0),
                        ))
                    })
                    .sum();
                let scale = near_volume.abs().max(moved_volume.abs()).max(1.0);
                let residual = (near_volume - moved_volume).abs() / scale;
                let transport_residual = (transported_volume - moved_volume).abs() / scale;
                if near_result.len() != moved_result.len() || residual > PLACEMENT_ALLOWANCE {
                    wrong += 1;
                    (
                        format!("{} solid, {near_volume:.6}", near_result.len()),
                        format!("{} solid, {moved_volume:.6}", moved_result.len()),
                        format!(
                            "WRONG: rigid residual {residual:.2e}, transported {transport_residual:.2e}"
                        ),
                    )
                } else {
                    (
                        format!("{} solid, {near_volume:.6}", near_result.len()),
                        format!("{} solid, {moved_volume:.6}", moved_result.len()),
                        format!(
                            "ok, residual {residual:.2e}, transported {transport_residual:.2e}"
                        ),
                    )
                }
            }
            (Outcome::Refused, Outcome::Refused) => (
                "REFUSED".to_string(),
                "REFUSED".to_string(),
                "clean refusal in both placements".to_string(),
            ),
            (Outcome::Panic, _) | (_, Outcome::Panic) => {
                panics += 1;
                (
                    format!("{near:?}"),
                    format!("{moved:?}"),
                    "PANIC".to_string(),
                )
            }
            (Outcome::Invalid(reason), _) | (_, Outcome::Invalid(reason)) => {
                wrong += 1;
                (
                    format!("{near:?}"),
                    format!("{moved:?}"),
                    format!("WRONG: {reason}"),
                )
            }
            _ => {
                wrong += 1;
                (
                    format!("{near:?}"),
                    format!("{moved:?}"),
                    "WRONG: support depends on world placement".to_string(),
                )
            }
        };

        println!("{name:<13} {near_text:<24} {moved_text:<24} {verdict}");
    }

    println!("{}", "-".repeat(94));
    println!("WRONG {wrong}   PANIC {panics}");
    if wrong > 0 || panics > 0 {
        std::process::exit(1);
    }
}
