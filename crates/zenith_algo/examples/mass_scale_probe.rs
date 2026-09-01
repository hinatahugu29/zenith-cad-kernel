//! **同じ立体を縮めても、体積が `s³` どおりか**（9-H の H6。4-231）。
//!
//! # なぜ要るのか
//!
//! `scale_sweep_probe` の恒等式は**体積で**測ります。だから破れの原因は
//! 2つに分かれます——**形が違う**のか、**体積の測り方が桁で崩れる**のか。
//!
//! 4-212 から 4-230 まで、**5回とも「形（交線の精度）」を疑って5回とも
//! 外しました**。**測り方のほうは、まだ一度も疑っていません。**
//!
//! # どう分けるか
//!
//! 測り方は2つあります。**どちらも見ます。**
//!
//! - **桁ごとに作る**（寸法を `s` 倍して作る。厳密に相似）
//! - **作った立体を縮める**（`BrepTransform::scale_solid`。4-232 で通るように
//!   しました。それまでは剛体変換しか受け付けませんでした）
//!
//! **どちらも体積は `s³` 倍になるはず**です。ならなければ、崩れているのは
//! **積分の側**か、**縮め方**です。
use zenith_algo::{BrepTransform, MassCalculator, PrimitiveBuilder};
use zenith_tess::TessellationParams;
use zenith_topo::Solid;

fn params() -> TessellationParams {
    TessellationParams {
        u_divisions: 32,
        v_divisions: 32,
    }
}

fn volume(solid: &Solid) -> f64 {
    MassCalculator::compute_from_brep(solid, &params()).volume
}

struct Subject {
    name: &'static str,
    /// 桁を渡すと、その大きさで作ります。**厳密に相似**です。
    build: fn(f64) -> Option<Solid>,
}

fn subjects() -> Vec<Subject> {
    vec![
        Subject {
            name: "torus",
            build: |s| PrimitiveBuilder::make_torus(12.0 * s, 4.0 * s).ok(),
        },
        Subject {
            name: "cylinder",
            build: |s| PrimitiveBuilder::make_cylinder(9.0 * s, 40.0 * s).ok(),
        },
        Subject {
            name: "sphere",
            build: |s| PrimitiveBuilder::make_sphere(10.0 * s).ok(),
        },
        Subject {
            name: "cone",
            build: |s| PrimitiveBuilder::make_cone(10.0 * s, 0.0, 20.0 * s).ok(),
        },
        Subject {
            name: "box",
            build: |s| PrimitiveBuilder::make_box(20.0 * s, 30.0 * s, 40.0 * s).ok(),
        },
    ]
}

fn main() {
    let scales = [1.0_f64, 0.1, 0.02, 0.01, 0.005];

    println!("同じ立体を縮めても、体積が s^3 どおりか（形は変えていません）");
    println!();
    println!(
        "{:<34}{:>8}{:>20}{:>16}{:>16}  {}",
        "subject", "scale", "volume / s^3", "作り直し", "縮めた", "verdict"
    );
    println!("{}", "-".repeat(112));

    let mut worst = 0.0f64;
    let mut worst_where = String::new();

    for subject in subjects() {
        let Some(reference_solid) = (subject.build)(1.0) else {
            println!("{:<34} 作れません", subject.name);
            continue;
        };
        let reference = volume(&reference_solid);
        if reference.abs() <= 0.0 {
            println!("{:<34} 体積が 0 なので測れません", subject.name);
            continue;
        }
        for scale in scales {
            let Some(scaled) = (subject.build)(scale) else {
                println!("{:<34}{:>8}  作れません", subject.name, scale);
                continue;
            };
            let measured = volume(&scaled) / scale.powi(3);
            let relative = ((measured - reference) / reference).abs();
            if relative > worst {
                worst = relative;
                worst_where = format!("{} / scale {scale}（作り直し）", subject.name);
            }
            // **縮めたほうも測ります**（4-232）。同じ立体を縮めるので、
            // ブーリアンの結果でも使えます。
            let shrunk_relative = BrepTransform::scale_solid(&reference_solid, scale)
                .ok()
                .map(|solid| {
                    let value = volume(&solid) / scale.powi(3);
                    ((value - reference) / reference).abs()
                });
            if let Some(value) = shrunk_relative {
                if value > worst {
                    worst = value;
                    worst_where = format!("{} / scale {scale}（縮めた）", subject.name);
                }
            }
            println!(
                "{:<34}{:>8}{:>20.9}{:>16.3e}{:>16}  {}",
                subject.name,
                scale,
                measured,
                relative,
                match shrunk_relative {
                    Some(value) => format!("{value:.3e}"),
                    None => "縮められない".to_string(),
                },
                if relative <= 1e-9 && shrunk_relative.map(|v| v <= 1e-9).unwrap_or(false) {
                    "ok"
                } else {
                    "**ずれ**"
                }
            );
        }
        println!();
    }

    println!("{}", "-".repeat(96));
    println!("いちばん大きいずれ {worst:.3e}（{worst_where}）。");
    println!();
    println!("**寸法だけを変えた、厳密に相似な形です。** ずれるなら、崩れているのは");
    println!("**体積の測り方**です。ずれないなら、`scale_sweep_probe` の破れは");
    println!("**形のほう**——桁ごとに作り直したときに、別の立体になっている——です。");
}
