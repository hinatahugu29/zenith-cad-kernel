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
//! **1つの立体を作って縮める**のがいちばん綺麗ですが、**このカーネルは
//! B-Rep を縮められません**——`BrepTransform::transform_solid` は剛体変換しか
//! 受け付けません（`B-Rep transform must be rigid`）。**これ自体が実用の
//! 欠落です**（模型を拡大縮小するのは基本の操作です。4-231 に記録）。
//!
//! そこで**素形状は桁ごとに作ります**。円柱もトーラスも、寸法を `s` 倍して
//! 作れば**厳密に相似**なので、混ざりません。**体積は `s³` 倍になるはず**で、
//! ならなければ崩れているのは**積分の側**です。
use zenith_algo::{MassCalculator, PrimitiveBuilder};
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
        "{:<34}{:>8}{:>20}{:>16}  {}",
        "subject", "scale", "volume / s^3", "relative", "verdict"
    );
    println!("{}", "-".repeat(96));

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
                worst_where = format!("{} / scale {scale}", subject.name);
            }
            println!(
                "{:<34}{:>8}{:>20.9}{:>16.3e}  {}",
                subject.name,
                scale,
                measured,
                relative,
                if relative <= 1e-9 { "ok" } else { "**ずれ**" }
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
