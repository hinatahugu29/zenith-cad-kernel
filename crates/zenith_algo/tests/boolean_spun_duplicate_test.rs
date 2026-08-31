//! **同じ立体を、自分の軸まわりに回してから重ねる。**
//!
//! 回しても**形は完全に同一**なので、和も積も元の立体、差は空です。
//! 変わるのはパッチの境（継ぎ目）の位置だけです。
//!
//! ## なぜ要るのか
//!
//! 2026/08/28 まで、球とトーラスを 30 度回して和を取ると、**同じ立体を
//! 2枚重ねた「二重被覆」**が返っていました（体積は2倍。4-137）。
//! 90 度・180 度ではパッチの境がちょうど重なるので起きません。
//! **「1つの角度で通る」は「通っている」ではありません。**
//!
//! 二重被覆は当時の検査を3つとも通りました——稜はどれもちょうど2回
//! 使われ、内外判定は全点一致し、体積は和の上限 `va + vb` にちょうど
//! 収まります。**ここは「体積が元の立体と一致するか」で見ます。**

use std::f64::consts::PI;

use zenith_algo::{BooleanEngine, BooleanOpType, BrepTransform, MassCalculator, PrimitiveBuilder};
use zenith_math::{Tolerance, Transform3, Vec3};
use zenith_tess::TessellationParams;
use zenith_topo::Solid;

fn params() -> TessellationParams {
    TessellationParams {
        u_divisions: 48,
        v_divisions: 48,
    }
}

fn volume(solids: &[Solid]) -> f64 {
    solids
        .iter()
        .map(|solid| MassCalculator::compute_from_brep(solid, &params()).volume)
        .sum()
}

/// **返って、合っていること。** 4-139 で断りも無くなりました。
///
/// 「同じ場所を占めるなら、面を刻み直さなくても答えは決まる」を入れた
/// ので、パッチの境に揃わない角度（7°・30°・45°）でも通ります。
/// **ここが断られるようになったら、その道が塞がったということです。**
#[test]
fn spinning_a_duplicate_about_its_own_axis_gives_back_the_same_solid() {
    let tol = Tolerance::default();
    let mut failures: Vec<String> = Vec::new();

    let shapes: Vec<(&str, Solid, f64)> = vec![
        (
            "cylinder(r5,h10)",
            PrimitiveBuilder::make_cylinder(5.0, 10.0).unwrap(),
            PI * 250.0,
        ),
        (
            "sphere(r5)",
            PrimitiveBuilder::make_sphere(5.0).unwrap(),
            4.0 / 3.0 * PI * 125.0,
        ),
        (
            "cone(r5,h10)",
            PrimitiveBuilder::make_cone(5.0, 0.0, 10.0).unwrap(),
            PI * 250.0 / 3.0,
        ),
        (
            "torus(R8,r3)",
            PrimitiveBuilder::make_torus(8.0, 3.0).unwrap(),
            2.0 * PI * PI * 8.0 * 9.0,
        ),
    ];

    for (name, solid, whole) in shapes {
        for angle in [7.0_f64, 30.0, 45.0, 90.0, 180.0] {
            let turn = Transform3::from_axis_angle(&Vec3::new(0.0, 0.0, 1.0), angle.to_radians());
            let spun = BrepTransform::transform_solid(&solid, &turn).expect("spin");
            for (label, op, expected) in [
                ("union", BooleanOpType::Union, whole),
                ("difference", BooleanOpType::Difference, 0.0),
                ("intersection", BooleanOpType::Intersection, whole),
            ] {
                let result =
                    match BooleanEngine::boolean_solids_exact_result(&solid, &spun, op, &tol) {
                        Ok(result) => result,
                        Err(err) => {
                            failures.push(format!("{name} / {angle}deg / {label}: refused: {err}"));
                            continue;
                        }
                    };
                let measured = volume(&result.solids);
                if (measured - expected).abs() > whole * 2e-3 {
                    failures.push(format!(
                        "{name} / {angle}deg / {label}: volume {measured} is not {expected}"
                    ));
                }
            }
        }
    }

    assert!(
        failures.is_empty(),
        "{} case(s) returned a wrong answer:\n{}",
        failures.len(),
        failures.join("\n")
    );
}
