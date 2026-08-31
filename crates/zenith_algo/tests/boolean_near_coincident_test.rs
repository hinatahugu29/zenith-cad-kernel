//! **同じ立体を、公差の前後だけずらして重ねる。**
//!
//! ずらし幅を `1e-9 / 1e-7 / 1e-5 / 1e-3 / 1e-1` と振ります。真の答えは
//! 全部閉じた式です——はみ出す薄片の体積は `断面積 × ずらし幅`。
//!
//! ## なぜ要るのか
//!
//! 2026/08/28 まで、円柱を軸方向に `1e-5` ずらした差が**同じ薄片を2つ**
//! 返していました。片方は `+7.854e-4`、もう片方は**裏返しの** `-7.854e-4`
//! で、**合計すると打ち消し合います**（4-135）。検査が結果の体積を
//! **合計**でしか見ていなかったので、そのまま通っていました。
//!
//! **公差ぎりぎりの重なりは実務にいくらでもあります**（履歴の丸め、
//! 嵌合、面取りのかかり）。ここは通る／断るの両方を許しますが、
//! **返ってきたものが間違っていることは許しません。**

use std::f64::consts::PI;

use zenith_algo::{BooleanEngine, BooleanOpType, BrepTransform, MassCalculator, PrimitiveBuilder};
use zenith_math::{Tolerance, Vec3};
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

/// **返ったら合っていること。断るのは赤にしません。**
///
/// 薄片が公差に埋もれる幅（`1e-9`、`1e-7`）では、差が空・積が元のまま
/// なのが正しい答えです。そこは閉じた式のほうを丸めて比べます。
#[test]
fn shifting_a_duplicate_by_a_hair_never_returns_a_wrong_answer() {
    let tol = Tolerance::default();
    let mut failures: Vec<String> = Vec::new();

    let cases: Vec<(&str, Solid, Vec3, f64, f64)> = vec![
        (
            "box(10) along x",
            PrimitiveBuilder::make_box(10.0, 10.0, 10.0).unwrap(),
            Vec3::new(1.0, 0.0, 0.0),
            1000.0,
            100.0,
        ),
        (
            "cylinder(r5,h10) along its axis",
            PrimitiveBuilder::make_cylinder(5.0, 10.0).unwrap(),
            Vec3::new(0.0, 0.0, 1.0),
            PI * 250.0,
            PI * 25.0,
        ),
    ];

    for (name, solid, direction, whole, cross_section) in cases {
        for delta in [1e-9_f64, 1e-7, 1e-5, 1e-3, 1e-1] {
            let moved = BrepTransform::translate_solid(&solid, direction * delta);
            let sliver = cross_section * delta;
            for (label, op, expected) in [
                ("union", BooleanOpType::Union, whole + sliver),
                ("difference", BooleanOpType::Difference, sliver),
                ("intersection", BooleanOpType::Intersection, whole - sliver),
            ] {
                // **断りは赤にしません。** まだ返せない配置があります。
                let Ok(result) =
                    BooleanEngine::boolean_solids_exact_result(&solid, &moved, op, &tol)
                else {
                    continue;
                };

                // 裏返しの立体は、それ自体が誤りです。
                for (index, piece) in result.solids.iter().enumerate() {
                    let piece_volume = MassCalculator::compute_from_brep(piece, &params()).volume;
                    if piece_volume <= 0.0 {
                        failures.push(format!(
                            "{name} / {delta:e} / {label}: solid {index} encloses {piece_volume:e}, which is not positive"
                        ));
                    }
                }

                let measured = volume(&result.solids);
                // 物差しは**立体そのものの大きさ**です。薄片は公差より
                // 細いことがあるので、薄片で正規化すると比べられません。
                if (measured - expected).abs() > whole * 2e-3 {
                    failures.push(format!(
                        "{name} / {delta:e} / {label}: volume {measured} is not {expected}"
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
