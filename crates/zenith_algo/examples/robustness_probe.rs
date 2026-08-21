//! 端の配置で、カーネルが何をするかを並べる。
//!
//! # なぜこれが要るか
//!
//! `boolean_envelope` の45ケースは、どれも**素直な配置**である。半径も距離も
//! 桁が揃っていて、面はきれいに交わるか、きれいに離れている。実務のデータは
//! そうならない。隙間 1e-9 で触れている面、厚み 1e-6 の板、桁が6つ違う立体、
//! 自分自身との演算——そういうものが来る。
//!
//! **答えられないこと自体は欠陥ではない。** このカーネルはエラーを返すことを
//! 選んでおり、それは正しい設計である。危険なのは次の3つで、ここではそれを
//! 見分けるために走らせる。
//!
//! - **誤答**: 閉じた立体を返すが、体積が明らかに違う。
//! - **パニック**: 呼び出し側が受け止められない。
//! - **戻ってこない**: 実用上はパニックより悪い。
//!
//! 出力の verdict はその3つと「clean error（断った）」「ok（通った）」を
//! 分ける。**clean error が並ぶのは健全な結果である。**

use std::panic::{catch_unwind, AssertUnwindSafe};

use zenith_algo::{
    BooleanEngine, BooleanOpType, BooleanResultVerifier, BrepTransform, MassCalculator,
    PrimitiveBuilder,
};
use zenith_math::{Tolerance, Vec3};
use zenith_tess::TessellationParams;
use zenith_topo::Solid;

struct Case {
    name: &'static str,
    /// 立体を作るところから壊れうるので、ここもクロージャにする。
    build: Box<dyn Fn() -> Result<(Solid, Solid), String>>,
    /// 体積が分かっているなら書く。誤答を見分けるにはこれが要る。
    expected: [Option<f64>; 3],
}

fn shifted(solid: &Solid, x: f64, y: f64, z: f64) -> Solid {
    BrepTransform::translate_solid(solid, Vec3::new(x, y, z))
}

fn cases() -> Vec<Case> {
    vec![
        Case {
            name: "boxes overlapping by 1e-9",
            build: Box::new(|| {
                let a = PrimitiveBuilder::make_box(20.0, 20.0, 20.0)?;
                let b = shifted(&a, 20.0 - 1e-9, 0.0, 0.0);
                Ok((a, b))
            }),
            expected: [None, None, None],
        },
        Case {
            name: "boxes separated by 1e-9",
            build: Box::new(|| {
                let a = PrimitiveBuilder::make_box(20.0, 20.0, 20.0)?;
                let b = shifted(&a, 20.0 + 1e-9, 0.0, 0.0);
                Ok((a, b))
            }),
            expected: [Some(16000.0), Some(8000.0), Some(0.0)],
        },
        Case {
            name: "a box against itself",
            build: Box::new(|| {
                let a = PrimitiveBuilder::make_box(20.0, 20.0, 20.0)?;
                Ok((a.clone(), a))
            }),
            expected: [Some(8000.0), Some(0.0), Some(8000.0)],
        },
        Case {
            name: "a box against itself moved 1e-12",
            build: Box::new(|| {
                let a = PrimitiveBuilder::make_box(20.0, 20.0, 20.0)?;
                let b = shifted(&a, 1e-12, 0.0, 0.0);
                Ok((a, b))
            }),
            expected: [None, None, None],
        },
        Case {
            name: "a 1e-6 thin plate cut by a box",
            build: Box::new(|| {
                let a = PrimitiveBuilder::make_box(20.0, 20.0, 1e-6)?;
                let b = shifted(&PrimitiveBuilder::make_box(10.0, 10.0, 10.0)?, 5.0, 5.0, -5.0);
                Ok((a, b))
            }),
            expected: [None, None, None],
        },
        Case {
            name: "a drill of radius 1e-6",
            build: Box::new(|| {
                let a = PrimitiveBuilder::make_box(20.0, 20.0, 20.0)?;
                let drill = PrimitiveBuilder::make_cylinder(1e-6, 60.0)?;
                Ok((a, shifted(&drill, 10.0, 10.0, -20.0)))
            }),
            expected: [None, None, None],
        },
        Case {
            name: "scales six orders apart",
            build: Box::new(|| {
                let a = PrimitiveBuilder::make_box(1.0e6, 1.0e6, 1.0e6)?;
                let b = PrimitiveBuilder::make_box(1.0, 1.0, 1.0)?;
                Ok((a, b))
            }),
            expected: [None, None, None],
        },
        Case {
            name: "everything at 1e-6 scale",
            build: Box::new(|| {
                let a = PrimitiveBuilder::make_box(2e-6, 2e-6, 2e-6)?;
                let b = shifted(&a, 1e-6, 1e-6, 1e-6);
                Ok((a, b))
            }),
            expected: [None, None, None],
        },
        Case {
            name: "a cylinder of zero height",
            build: Box::new(|| {
                let a = PrimitiveBuilder::make_box(20.0, 20.0, 20.0)?;
                let b = PrimitiveBuilder::make_cylinder(5.0, 0.0)?;
                Ok((a, b))
            }),
            expected: [None, None, None],
        },
        Case {
            name: "a cylinder of zero radius",
            build: Box::new(|| {
                let a = PrimitiveBuilder::make_box(20.0, 20.0, 20.0)?;
                let b = PrimitiveBuilder::make_cylinder(0.0, 40.0)?;
                Ok((a, b))
            }),
            expected: [None, None, None],
        },
        Case {
            name: "a sphere clear of a face by 1e-9",
            build: Box::new(|| {
                let a = PrimitiveBuilder::make_box(20.0, 20.0, 20.0)?;
                let sphere = PrimitiveBuilder::make_sphere(5.0)?;
                Ok((a, shifted(&sphere, 0.0, 0.0, 15.0 + 1e-9)))
            }),
            expected: [None, None, None],
        },
        Case {
            name: "a sphere into a face by 1e-9",
            build: Box::new(|| {
                let a = PrimitiveBuilder::make_box(20.0, 20.0, 20.0)?;
                let sphere = PrimitiveBuilder::make_sphere(5.0)?;
                Ok((a, shifted(&sphere, 0.0, 0.0, 15.0 - 1e-9)))
            }),
            expected: [None, None, None],
        },
    ]
}

fn main() {
    let tol = Tolerance::default();
    let params = TessellationParams {
        u_divisions: 24,
        v_divisions: 24,
    };

    let ops = [
        ("union", BooleanOpType::Union),
        ("difference", BooleanOpType::Difference),
        ("intersection", BooleanOpType::Intersection),
    ];

    println!(
        "{:<40} {:<13} {:<14} {:>16}  {}",
        "case", "op", "verdict", "volume", "note"
    );
    println!("{}", "-".repeat(120));

    // ok / clean error / wrong / panic / build refused
    let mut tally = [0usize; 5];

    for case in cases() {
        let built = catch_unwind(AssertUnwindSafe(|| (case.build)()));
        let (solid_a, solid_b) = match built {
            Err(_) => {
                println!(
                    "{:<40} {:<13} {:<14} {:>16}  {}",
                    case.name, "-", "PANIC", "-", "panicked while building the inputs"
                );
                tally[3] += 1;
                continue;
            }
            Ok(Err(err)) => {
                println!(
                    "{:<40} {:<13} {:<14} {:>16}  {}",
                    case.name,
                    "-",
                    "build refused",
                    "-",
                    err.chars().take(56).collect::<String>()
                );
                tally[4] += 1;
                continue;
            }
            Ok(Ok(pair)) => pair,
        };

        for (op_index, (op_name, op)) in ops.iter().enumerate() {
            let outcome = catch_unwind(AssertUnwindSafe(|| {
                BooleanEngine::boolean_solids_exact_result_unverified(&solid_a, &solid_b, *op, &tol)
            }));

            match outcome {
                Err(_) => {
                    println!(
                        "{:<40} {:<13} {:<14} {:>16}  {}",
                        case.name, op_name, "PANIC", "-", "the operation panicked"
                    );
                    tally[3] += 1;
                }
                Ok(Err(err)) => {
                    let short = err.split(';').next().unwrap_or(&err);
                    println!(
                        "{:<40} {:<13} {:<14} {:>16}  {}",
                        case.name,
                        op_name,
                        "clean error",
                        "-",
                        short.chars().take(56).collect::<String>()
                    );
                    tally[1] += 1;
                }
                Ok(Ok(result)) => {
                    let volume: f64 = result
                        .solids
                        .iter()
                        .map(|s| MassCalculator::compute_from_brep(s, &params).volume)
                        .sum();
                    let closed = result
                        .solids
                        .iter()
                        .all(|s| s.outer_shell.validate_closed(&tol).is_valid());
                    let gate = BooleanResultVerifier::verify(
                        &solid_a,
                        &solid_b,
                        &result.solids,
                        *op,
                        &tol,
                    );

                    let mut notes = vec![format!("{} solid(s)", result.solids.len())];
                    let mut wrong = !closed || !gate.is_valid();
                    if !closed {
                        notes.push("SHELL NOT CLOSED".to_string());
                    }
                    if !gate.is_valid() {
                        notes.push(format!(
                            "GATE REJECT: {}",
                            gate.errors
                                .first()
                                .map(|error| error.chars().take(40).collect::<String>())
                                .unwrap_or_default()
                        ));
                    }
                    if let Some(expected) = case.expected[op_index] {
                        let error = (volume - expected).abs() / expected.abs().max(1e-9);
                        if error > 1e-6 {
                            notes.push(format!("EXPECTED {expected:.6}"));
                            wrong = true;
                        } else {
                            notes.push(format!("analytic {error:.2e}"));
                        }
                    }

                    if wrong {
                        tally[2] += 1;
                    } else {
                        tally[0] += 1;
                    }
                    println!(
                        "{:<40} {:<13} {:<14} {:>16.6}  {}",
                        case.name,
                        op_name,
                        if wrong { "WRONG" } else { "ok" },
                        volume,
                        notes.join(", ")
                    );
                }
            }
        }
    }

    println!("{}", "-".repeat(120));
    println!(
        "ok {}   clean error {}   WRONG {}   PANIC {}   build refused {}",
        tally[0], tally[1], tally[2], tally[3], tally[4]
    );
    println!();
    println!("clean error is a healthy outcome. What matters is WRONG and PANIC,");
    println!("because a caller cannot tell either of them apart from an answer.");
}
