//! Checks whether boolean results can be fed back into the boolean engine.
//!
//! A plate with four bolt holes is four subtractions in a row, each operating
//! on the result of the last. Drilling once works; whether the result is a
//! usable operand for the next cut is a different question, and it is the one
//! that decides if the capability is practical.
//!
//! Run with: cargo run --release -p zenith_algo --example chained_boolean_probe

use std::f64::consts::PI;

use zenith_algo::{
    BooleanEngine, BooleanOpType, BrepTransform, MassCalculator, PrimitiveBuilder,
};
use zenith_math::{Tolerance, Vec3};
use zenith_tess::TessellationParams;
use zenith_topo::Solid;

fn volume(solid: &Solid) -> f64 {
    MassCalculator::compute_from_brep(
        solid,
        &TessellationParams {
            u_divisions: 48,
            v_divisions: 48,
        },
    )
    .volume
}

fn main() {
    let tol = Tolerance::default();

    // 80 x 60 x 20 の板に、四隅へ半径5の穴を4つ開ける。
    let plate = PrimitiveBuilder::make_box(80.0, 60.0, 20.0).unwrap();
    let hole_radius = 5.0;
    let centres = [
        (15.0, 15.0),
        (65.0, 15.0),
        (65.0, 45.0),
        (15.0, 45.0),
    ];

    let mut current = plate;
    let plate_volume = 80.0 * 60.0 * 20.0;
    let hole_volume = PI * hole_radius * hole_radius * 20.0;

    println!("plate 80x60x20, four bolt holes of radius {hole_radius}");
    println!("    start volume {:.4}", volume(&current));

    for (index, (x, y)) in centres.iter().enumerate() {
        let drill = BrepTransform::translate_solid(
            &PrimitiveBuilder::make_cylinder(hole_radius, 60.0).unwrap(),
            Vec3::new(*x, *y, -20.0),
        );

        match BooleanEngine::boolean_solids_exact_result(
            &current,
            &drill,
            BooleanOpType::Difference,
            &tol,
        ) {
            Ok(result) => {
                if result.solids.len() != 1 {
                    println!(
                        "    hole {} produced {} solids, stopping",
                        index + 1,
                        result.solids.len()
                    );
                    break;
                }
                current = result.solids.into_iter().next().unwrap();
                let expected = plate_volume - hole_volume * (index + 1) as f64;
                let actual = volume(&current);
                println!(
                    "    hole {} ok: volume {:.4}, expected {:.4}, relative {:.2e}, faces {}",
                    index + 1,
                    actual,
                    expected,
                    (actual - expected).abs() / expected,
                    current.outer_shell.faces.len()
                );
            }
            Err(err) => {
                println!(
                    "    hole {} FAILED: {}",
                    index + 1,
                    err.chars().take(100).collect::<String>()
                );
                break;
            }
        }
    }

    println!();

    // 座ぐり: 同軸の細い貫通穴と太い浅い穴。
    println!("counterbore: a through hole plus a wider shallow one on the same axis");
    let block = PrimitiveBuilder::make_box(40.0, 40.0, 20.0).unwrap();
    let pilot = BrepTransform::translate_solid(
        &PrimitiveBuilder::make_cylinder(5.0, 60.0).unwrap(),
        Vec3::new(20.0, 20.0, -20.0),
    );

    match BooleanEngine::boolean_solids_exact_result(
        &block,
        &pilot,
        BooleanOpType::Difference,
        &tol,
    ) {
        Ok(first) => {
            let drilled = first.solids.into_iter().next().unwrap();
            println!("    pilot hole ok: volume {:.4}", volume(&drilled));

            let counterbore = BrepTransform::translate_solid(
                &PrimitiveBuilder::make_cylinder(9.0, 40.0).unwrap(),
                Vec3::new(20.0, 20.0, 14.0),
            );
            match BooleanEngine::boolean_solids_exact_result(
                &drilled,
                &counterbore,
                BooleanOpType::Difference,
                &tol,
            ) {
                Ok(second) => {
                    let solid = &second.solids[0];
                    let expected =
                        40.0 * 40.0 * 20.0 - PI * 25.0 * 20.0 - (PI * 81.0 - PI * 25.0) * 6.0;
                    let actual = volume(solid);
                    println!(
                        "    counterbore ok: volume {:.4}, expected {:.4}, relative {:.2e}",
                        actual,
                        expected,
                        (actual - expected).abs() / expected
                    );
                }
                Err(err) => println!(
                    "    counterbore FAILED: {}",
                    err.chars().take(100).collect::<String>()
                ),
            }
        }
        Err(err) => println!("    pilot hole FAILED: {err}"),
    }
}
