//! Empirically measures which exact B-Rep boolean cases the kernel actually
//! supports, instead of inferring the envelope from the dispatch code.
//!
//! Run with: cargo run -p zenith_algo --example boolean_envelope

use zenith_algo::{
    BooleanEngine, BooleanOpType, BooleanResultVerifier, BrepTransform, MassCalculator,
    PrimitiveBuilder,
};
use zenith_math::{Tolerance, Vec3};
use zenith_tess::TessellationParams;
use zenith_topo::Solid;

struct Case {
    name: &'static str,
    a: Solid,
    b: Solid,
    /// Analytic volume per op, when it is known in closed form.
    expected: [Option<f64>; 3],
}

fn shifted(solid: &Solid, x: f64, y: f64, z: f64) -> Solid {
    BrepTransform::translate_solid(solid, Vec3::new(x, y, z))
}

/// 第1種・第2種の完全楕円積分 `K(k)`, `E(k)` を算術幾何平均で求める。
///
/// 半径の違う2本の直交円柱の交わりには閉じた式があり、そこにこれが要る。
/// 期待値の側に特殊関数が要るからといって、期待値を測定値で置き換えては
/// ならない。それをすると、この行は何も確かめなくなる。
fn complete_elliptic_k_e(k: f64) -> (f64, f64) {
    let mut a = 1.0f64;
    let mut b = (1.0 - k * k).sqrt();
    let mut c = k;
    let mut sum = c * c * 0.5;
    let mut power = 1.0f64;
    for _ in 0..40 {
        if c.abs() < 1e-17 {
            break;
        }
        let next_a = (a + b) * 0.5;
        let next_b = (a * b).sqrt();
        c = (a - b) * 0.5;
        a = next_a;
        b = next_b;
        power *= 2.0;
        sum += power * c * c * 0.5;
    }
    let k_value = std::f64::consts::PI / (2.0 * a);
    (k_value, k_value * (1.0 - sum))
}

/// 半径 `big` と `small` の直交する2円柱の交わりの体積。軸は交わっているとする。
///
/// `V = (8/3) R^3 [(1 + k^2) E(k) - (1 - k^2) K(k)]`, `k = r / R`。
/// 半径が等しいときは Steinmetz の `16 R^3 / 3` に戻る。
fn bicylinder_intersection_volume(big: f64, small: f64) -> f64 {
    let k = small / big;
    let (k_value, e_value) = complete_elliptic_k_e(k);
    8.0 / 3.0 * big.powi(3) * ((1.0 + k * k) * e_value - (1.0 - k * k) * k_value)
}

fn main() {
    let tol = Tolerance::default();
    let params = TessellationParams {
        u_divisions: 24,
        v_divisions: 24,
    };

    let boxa = PrimitiveBuilder::make_box(20.0, 20.0, 20.0).unwrap();
    let cyl = PrimitiveBuilder::make_cylinder(6.0, 40.0).unwrap();
    let sphere = PrimitiveBuilder::make_sphere(10.0).unwrap();
    let cone = PrimitiveBuilder::make_cone(10.0, 4.0, 20.0).unwrap();
    let torus = PrimitiveBuilder::make_torus(12.0, 4.0).unwrap();

    let cases = vec![
        Case {
            name: "box x box (corner overlap)",
            a: boxa.clone(),
            b: shifted(&boxa, 10.0, 10.0, 10.0),
            // union = 2*8000 - 1000, diff = 8000 - 1000, isect = 10^3
            expected: [Some(15000.0), Some(7000.0), Some(1000.0)],
        },
        Case {
            name: "box x box (face flush, no overlap)",
            a: boxa.clone(),
            b: shifted(&boxa, 20.0, 0.0, 0.0),
            expected: [Some(16000.0), Some(8000.0), None],
        },
        Case {
            name: "box x box (fully disjoint)",
            a: boxa.clone(),
            b: shifted(&boxa, 100.0, 0.0, 0.0),
            expected: [Some(16000.0), Some(8000.0), None],
        },
        Case {
            name: "box x box (rotated 45deg, also offset in Z)",
            a: boxa.clone(),
            b: {
                // Z にもずらすと、どの面も相手と同一平面にならない。
                let rotation = zenith_math::Transform3::from_axis_angle(
                    &Vec3::new(0.0, 0.0, 1.0),
                    std::f64::consts::FRAC_PI_4,
                );
                let rotated =
                    BrepTransform::transform_solid(&shifted(&boxa, 10.0, 10.0, 0.0), &rotation)
                        .unwrap();
                shifted(&rotated, 0.0, 0.0, 7.0)
            },
            expected: [None, None, None],
        },
        Case {
            name: "box x box (rotated 45deg about Z)",
            a: boxa.clone(),
            b: {
                let t = zenith_math::Transform3::from_axis_angle(
                    &Vec3::new(0.0, 0.0, 1.0),
                    std::f64::consts::FRAC_PI_4,
                );
                BrepTransform::transform_solid(&shifted(&boxa, 10.0, 10.0, 0.0), &t).unwrap()
            },
            expected: [None, None, None],
        },
        Case {
            name: "box x cylinder (axis-aligned through hole)",
            a: boxa.clone(),
            b: shifted(&cyl, 10.0, 10.0, -10.0),
            // 20^3 の箱を半径6の円柱が貫通する。穴の体積は pi*36*20。
            // 円柱は高さ40なので、箱の外に出ている分は pi*36*40 - pi*36*20。
            expected: [
                Some(8000.0 + std::f64::consts::PI * 36.0 * 40.0
                    - std::f64::consts::PI * 36.0 * 20.0),
                Some(8000.0 - std::f64::consts::PI * 36.0 * 20.0),
                Some(std::f64::consts::PI * 36.0 * 20.0),
            ],
        },
        Case {
            name: "box x cylinder (blind hole from the top)",
            a: boxa.clone(),
            b: {
                // 半径6・高さ25の円柱を z=10..35 に置く。下端が箱の内部
                // (z=10) で止まるので、天面から深さ10の止まり穴になる。
                let drill = PrimitiveBuilder::make_cylinder(6.0, 25.0).unwrap();
                shifted(&drill, 10.0, 10.0, 10.0)
            },
            // 差 = 8000 - pi*36*10、積 = pi*36*10
            expected: [
                Some(8000.0 + std::f64::consts::PI * 36.0 * 25.0
                    - std::f64::consts::PI * 36.0 * 10.0),
                Some(8000.0 - std::f64::consts::PI * 36.0 * 10.0),
                Some(std::f64::consts::PI * 36.0 * 10.0),
            ],
        },
        Case {
            name: "box x cylinder (through hole along X)",
            a: boxa.clone(),
            b: {
                let rotation = zenith_math::Transform3::from_axis_angle(
                    &Vec3::new(0.0, 1.0, 0.0),
                    std::f64::consts::FRAC_PI_2,
                );
                let along_x = BrepTransform::transform_solid(
                    &PrimitiveBuilder::make_cylinder(5.0, 40.0).unwrap(),
                    &rotation,
                )
                .unwrap();
                shifted(&along_x, -10.0, 10.0, 10.0)
            },
            expected: [
                Some(8000.0 + std::f64::consts::PI * 25.0 * 40.0
                    - std::f64::consts::PI * 25.0 * 20.0),
                Some(8000.0 - std::f64::consts::PI * 25.0 * 20.0),
                Some(std::f64::consts::PI * 25.0 * 20.0),
            ],
        },
        Case {
            name: "box x cylinder (off-centre through hole)",
            a: boxa.clone(),
            b: {
                // 中心 (8, 12)、半径5。箱の面には触れず完全に内側を通る。
                let drill = PrimitiveBuilder::make_cylinder(5.0, 40.0).unwrap();
                shifted(&drill, 8.0, 12.0, -10.0)
            },
            expected: [
                Some(8000.0 + std::f64::consts::PI * 25.0 * 40.0
                    - std::f64::consts::PI * 25.0 * 20.0),
                Some(8000.0 - std::f64::consts::PI * 25.0 * 20.0),
                Some(std::f64::consts::PI * 25.0 * 20.0),
            ],
        },
        Case {
            name: "box x cylinder (tangent to a side face)",
            a: boxa.clone(),
            b: {
                // 半径6を中心 (6, 10) に置くと、x=0 面にちょうど接する。
                let drill = PrimitiveBuilder::make_cylinder(6.0, 40.0).unwrap();
                shifted(&drill, 6.0, 10.0, -10.0)
            },
            expected: [None, None, None],
        },
        Case {
            name: "box x sphere",
            a: boxa.clone(),
            b: shifted(&sphere, 20.0, 10.0, 10.0),
            expected: [None, None, None],
        },
        Case {
            name: "cylinder x cylinder (perpendicular cross)",
            a: PrimitiveBuilder::make_cylinder(10.0, 40.0).unwrap(),
            b: {
                // +Z 向きの円柱を Y 軸まわりに90度回して +X 向きにし、
                // 相手の中ほど (z = 20) を貫くように置く。
                let rotation = zenith_math::Transform3::from_axis_angle(
                    &Vec3::new(0.0, 1.0, 0.0),
                    std::f64::consts::FRAC_PI_2,
                );
                let along_x = BrepTransform::transform_solid(
                    &PrimitiveBuilder::make_cylinder(6.0, 40.0).unwrap(),
                    &rotation,
                )
                .unwrap();
                shifted(&along_x, -20.0, 0.0, 20.0)
            },
            expected: {
                let big = std::f64::consts::PI * 100.0 * 40.0;
                let small = std::f64::consts::PI * 36.0 * 40.0;
                let lens = bicylinder_intersection_volume(10.0, 6.0);
                [Some(big + small - lens), Some(big - lens), Some(lens)]
            },
        },
        Case {
            name: "sphere x sphere",
            a: sphere.clone(),
            b: shifted(&sphere, 10.0, 0.0, 0.0),
            expected: {
                // 半径 r の球2つ、中心間距離 d のレンズは (pi/12)(4r+d)(2r-d)^2。
                let one = 4.0 / 3.0 * std::f64::consts::PI * 1000.0;
                let lens = std::f64::consts::PI / 12.0 * 50.0 * 100.0;
                [Some(2.0 * one - lens), Some(one - lens), Some(lens)]
            },
        },
        Case {
            name: "cone x box",
            a: cone.clone(),
            b: shifted(&boxa, -10.0, -10.0, 10.0),
            expected: [None, None, None],
        },
        Case {
            name: "torus x box",
            a: torus.clone(),
            b: shifted(&boxa, -10.0, -10.0, -2.0),
            expected: [None, None, None],
        },
    ];

    let ops = [
        ("union", BooleanOpType::Union),
        ("difference", BooleanOpType::Difference),
        ("intersection", BooleanOpType::Intersection),
    ];

    let mut ok = 0usize;
    let mut failed = 0usize;
    let mut wrong = 0usize;

    println!(
        "{:<42} {:<13} {:<9} {:>12}  {}",
        "case", "op", "result", "volume", "note"
    );
    println!("{}", "-".repeat(110));

    for case in &cases {
        for (op_index, (op_name, op)) in ops.iter().enumerate() {
            match BooleanEngine::boolean_solids_exact_result(&case.a, &case.b, *op, &tol) {
                Ok(result) => {
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
                        &case.a,
                        &case.b,
                        &result.solids,
                        *op,
                        &tol,
                    );

                    let mut notes = Vec::new();
                    notes.push(format!("{} solid(s)", result.solids.len()));
                    notes.push(format!(
                        "gate {}",
                        if gate.is_valid() { "pass" } else { "REJECT" }
                    ));
                    if !gate.is_valid() {
                        notes.push(
                            gate.errors[0].chars().take(58).collect::<String>(),
                        );
                    }
                    if !closed {
                        notes.push("SHELL NOT VALID".to_string());
                    }

                    let mut is_wrong = !closed;
                    if let Some(expected) = case.expected[op_index] {
                        let error = (volume - expected).abs();
                        let relative = error / expected.max(1e-9);
                        if relative > 1e-6 {
                            notes.push(format!("EXPECTED {expected:.3}"));
                            is_wrong = true;
                        } else {
                            notes.push("volume matches analytic".to_string());
                        }
                    }

                    if is_wrong {
                        wrong += 1;
                    } else {
                        ok += 1;
                    }

                    println!(
                        "{:<42} {:<13} {:<9} {:>12.3}  {}",
                        case.name,
                        op_name,
                        if is_wrong { "WRONG" } else { "ok" },
                        volume,
                        notes.join(", ")
                    );
                }
                Err(err) => {
                    failed += 1;
                    let short = err.split(';').next().unwrap_or(&err);
                    let short = short.chars().take(60).collect::<String>();
                    println!(
                        "{:<42} {:<13} {:<9} {:>12}  {}",
                        case.name, op_name, "ERROR", "-", short
                    );
                }
            }
        }
    }

    println!("{}", "-".repeat(110));
    println!(
        "supported: {ok}   wrong-result: {wrong}   unsupported/error: {failed}   (total {})",
        ok + wrong + failed
    );
}
