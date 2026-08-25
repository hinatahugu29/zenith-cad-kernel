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
        Case {
            // 角と角だけが触れている。共有するのは1点で、面でも辺でもない。
            // 和は非多様体になり、積は点しか持たない。古典的に落ちる配置。
            name: "boxes touching at one corner",
            build: Box::new(|| {
                let a = PrimitiveBuilder::make_box(20.0, 20.0, 20.0)?;
                let b = shifted(&a, 20.0, 20.0, 20.0);
                Ok((a, b))
            }),
            expected: [Some(16000.0), Some(8000.0), Some(0.0)],
        },
        Case {
            // 辺だけを共有する。積は線分で、体積は0。
            name: "boxes touching along one edge",
            build: Box::new(|| {
                let a = PrimitiveBuilder::make_box(20.0, 20.0, 20.0)?;
                let b = shifted(&a, 20.0, 20.0, 0.0);
                Ok((a, b))
            }),
            expected: [Some(16000.0), Some(8000.0), Some(0.0)],
        },
        Case {
            // 小さい箱が大きい箱の中に完全に入っている。実務では「中にある
            // 部品」そのもので、差は空洞を作るはず。
            name: "a box fully inside another",
            build: Box::new(|| {
                let outer = PrimitiveBuilder::make_box(20.0, 20.0, 20.0)?;
                let inner = shifted(&PrimitiveBuilder::make_box(4.0, 4.0, 4.0)?, 8.0, 8.0, 8.0);
                Ok((outer, inner))
            }),
            expected: [Some(8000.0), Some(8000.0 - 64.0), Some(64.0)],
        },
        Case {
            // 上と逆。A が B に完全に含まれるので、差は空。
            name: "a box fully containing the other",
            build: Box::new(|| {
                let outer = PrimitiveBuilder::make_box(20.0, 20.0, 20.0)?;
                let inner = shifted(&PrimitiveBuilder::make_box(4.0, 4.0, 4.0)?, 8.0, 8.0, 8.0);
                Ok((inner, outer))
            }),
            expected: [Some(8000.0), Some(0.0), Some(64.0)],
        },
        Case {
            // 1e-9 ラジアンだけ回した箱。軸平行の近道が「軸平行だ」と
            // 誤認すると、返るのは**回していない答え**になる。誤認しても
            // 体積はほぼ同じなので、体積だけ見ていると気づけない。
            name: "a box rotated by 1e-9 radians",
            build: Box::new(|| {
                let a = PrimitiveBuilder::make_box(20.0, 20.0, 20.0)?;
                let rotation =
                    zenith_math::Transform3::from_axis_angle(&Vec3::new(0.0, 0.0, 1.0), 1e-9);
                let b = BrepTransform::transform_solid(&shifted(&a, 10.0, 0.0, 0.0), &rotation)?;
                Ok((a, b))
            }),
            expected: [None, None, None],
        },
        Case {
            // 縦横比 1e4。細長い棒が板を貫く。
            name: "a needle through a plate",
            build: Box::new(|| {
                let plate = PrimitiveBuilder::make_box(200.0, 200.0, 2.0)?;
                let needle = shifted(&PrimitiveBuilder::make_box(0.02, 0.02, 20.0)?, 100.0, 100.0, -9.0);
                Ok((plate, needle))
            }),
            expected: [None, None, None],
        },
        Case {
            // 面をぴったり共有する。同一平面の扱いがそのまま出る。
            name: "boxes sharing a whole face",
            build: Box::new(|| {
                let a = PrimitiveBuilder::make_box(20.0, 20.0, 20.0)?;
                let b = shifted(&a, 20.0, 0.0, 0.0);
                Ok((a, b))
            }),
            expected: [Some(16000.0), Some(8000.0), Some(0.0)],
        },
        Case {
            // **自分の出力を入力に戻す。** 実務のモデリングは逐次的で、
            // ブーリアンの結果にさらにブーリアンをかける。結果の立体は
            // プリミティブと面構成が違うので、通るかどうかは別の問いになる。
            // 箱の連鎖はショーケースにあるが、**曲面ブーリアンの結果を
            // 入力に戻す検体は1つも無い。**
            name: "a drilled block, drilled again",
            build: Box::new(|| {
                let block = PrimitiveBuilder::make_box(40.0, 40.0, 20.0)?;
                let drill = PrimitiveBuilder::make_cylinder(5.0, 60.0)?;
                let once = BooleanEngine::boolean_solids_exact(
                    &block,
                    &shifted(&drill, 12.0, 20.0, -20.0),
                    BooleanOpType::Difference,
                    &Tolerance::default(),
                )?;
                Ok((once, shifted(&drill, 28.0, 20.0, -20.0)))
            }),
            expected: [None, None, None],
        },
        Case {
            // 曲面同士の結果を入力に戻す。円柱を円柱で貫いたものに、
            // さらに箱を当てる。
            name: "crossed cylinders, then cut by a box",
            build: Box::new(|| {
                let tol = Tolerance::default();
                let upright = PrimitiveBuilder::make_cylinder(10.0, 40.0)?;
                let rotation = zenith_math::Transform3::from_axis_angle(
                    &Vec3::new(0.0, 1.0, 0.0),
                    std::f64::consts::FRAC_PI_2,
                );
                let along_x = BrepTransform::transform_solid(
                    &PrimitiveBuilder::make_cylinder(6.0, 40.0)?,
                    &rotation,
                )?;
                let crossed = BooleanEngine::boolean_solids_exact(
                    &upright,
                    &shifted(&along_x, -20.0, 0.0, 20.0),
                    BooleanOpType::Union,
                    &tol,
                )?;
                let slab = shifted(&PrimitiveBuilder::make_box(60.0, 60.0, 10.0)?, -30.0, -30.0, 30.0);
                Ok((crossed, slab))
            }),
            expected: [None, None, None],
        },
        Case {
            // 空洞を持つ立体（内側シェル）を入力にする。差で出来た空洞入りの
            // 立体は、次の演算で内側シェルを正しく扱えないと壊れる。
            name: "a solid with a cavity, cut again",
            build: Box::new(|| {
                let tol = Tolerance::default();
                let outer = PrimitiveBuilder::make_box(40.0, 40.0, 40.0)?;
                let inner = shifted(&PrimitiveBuilder::make_box(10.0, 10.0, 10.0)?, 15.0, 15.0, 15.0);
                let hollow =
                    BooleanEngine::boolean_solids_exact(&outer, &inner, BooleanOpType::Difference, &tol)?;
                let knife = shifted(&PrimitiveBuilder::make_box(60.0, 60.0, 10.0)?, -10.0, -10.0, 35.0);
                Ok((hollow, knife))
            }),
            expected: [None, None, None],
        },
        Case {
            // 45度回した箱どうし。軸平行の近道を確実に外して一般経路へ流す。
            name: "boxes both rotated 45 degrees",
            build: Box::new(|| {
                let rotation = zenith_math::Transform3::from_axis_angle(
                    &Vec3::new(0.0, 0.0, 1.0),
                    std::f64::consts::FRAC_PI_4,
                );
                let a = BrepTransform::transform_solid(
                    &PrimitiveBuilder::make_box(20.0, 20.0, 20.0)?,
                    &rotation,
                )?;
                let b = shifted(&a, 10.0, 10.0, 0.0);
                Ok((a, b))
            }),
            expected: [None, None, None],
        },
        Case {
            // 曲面どうしで桁が離れている。上で直したゲートの件が、曲面でも
            // 通るかを見る。
            name: "a big cylinder against a tiny one",
            build: Box::new(|| {
                let big = PrimitiveBuilder::make_cylinder(1000.0, 2000.0)?;
                let small = PrimitiveBuilder::make_cylinder(0.5, 4000.0)?;
                Ok((big, shifted(&small, 0.0, 0.0, -1000.0)))
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

    // Keep the probe's process status aligned with the verdict it prints.
    // CI also scans this summary, but local runners such as `fast_test.sh`
    // intentionally trust each probe's exit status.  Returning success after
    // reporting a wrong answer or a panic would therefore make the two gates
    // enforce different contracts.
    if tally[2] > 0 || tally[3] > 0 {
        std::process::exit(1);
    }
}
