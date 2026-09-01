//! **大きさの桁を変えて、同じ形を測る。**
//!
//! # なぜこの軸か
//!
//! このカーネルの公差は**絶対値**です——`Tolerance::linear` が 1e-6、
//! メッシュの溶接が 1e-7。ところが実務のモデルは mm だけではありません。
//! 時計の部品なら 0.1 mm 級、建築なら 10 m 級です。**同じ形でも、桁が
//! 変われば公差との関係が変わります。**
//!
//! 掃き出しは長く「置き方を変える」でやってきました（4-205、4-207、4-211）。
//! **4-211 で初めて欠陥 0 になった**ので、軸を変えます。ここは
//! **大きさの桁**です。
//!
//! # 何を測るか
//!
//! 桁が変わっても**相対的な答えは同じはず**です。だから相対で見ます。
//!
//! | 見るもの | 赤にする条件 |
//! | :--- | :--- |
//! | 恒等式 `V(A-B) + V(A∩B) = V(A)` | **相対**の破れが 1e-6 を超えたら |
//! | B-Rep の非多様体 | 1本でもあれば |
//! | 演算が返るか | **返らないこと自体は赤にしません**（桁で断るなら、それは分かったほうが良い事実です） |
//!
//! **体積そのものは比べません。** 桁が違えば体積も `s^3` で変わるので、
//! そのまま比べても意味がありません。比べるのは**無次元の残差**です。
//!
//! ```bash
//! cargo run --release -p zenith_algo --example scale_sweep_probe
//! ```

use zenith_algo::{BooleanEngine, BooleanOpType, BrepTransform, MassCalculator, PrimitiveBuilder};
use zenith_math::{Tolerance, Transform3, Vec3};
use zenith_tess::TessellationParams;
use zenith_topo::Solid;

/// 桁ごとの検体。`s` を掛けた寸法で組みます。
struct Placement {
    name: &'static str,
    build: fn(f64) -> (Solid, Solid),
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

fn total_volume(solids: &[Solid]) -> f64 {
    solids.iter().map(volume).sum()
}

/// B-Rep の稜のうち、ちょうど2つの面ループに使われていない本数。
///
/// 位置で突き合わせます（同じ弧でも面ごとに別の実体を持つことがあるため。
/// 4-80）。**量子化の刻みは大きさに比例させます**——絶対の刻みで丸めると、
/// 小さい模型では全部の点が同じ格子に落ちて「非多様体 0」に見えてしまい、
/// 何も測れません。
fn non_manifold_brep_edges(solid: &Solid, scale: f64) -> usize {
    let grid = (1e-7 * scale).max(f64::MIN_POSITIVE);
    let quantise = |p: zenith_math::Point3| {
        let q = |v: f64| (v / grid).round() as i64;
        (q(p.x), q(p.y), q(p.z))
    };
    let mut uses: std::collections::HashMap<((i64, i64, i64), (i64, i64, i64)), usize> =
        std::collections::HashMap::new();
    for shell in std::iter::once(&solid.outer_shell).chain(solid.inner_shells.iter()) {
        for face in &shell.faces {
            for wire in std::iter::once(&face.outer_wire).chain(face.inner_wires.iter()) {
                for oriented in &wire.edges {
                    let a = quantise(oriented.edge.start_vertex.point);
                    let b = quantise(oriented.edge.end_vertex.point);
                    let key = if a <= b { (a, b) } else { (b, a) };
                    *uses.entry(key).or_insert(0) += 1;
                }
            }
        }
    }
    uses.values().filter(|count| **count != 2).count()
}

fn placements() -> Vec<Placement> {
    vec![
        Placement {
            name: "box x cylinder (through drill)",
            build: |s| {
                let block = PrimitiveBuilder::make_box(40.0 * s, 40.0 * s, 20.0 * s).unwrap();
                let drill = BrepTransform::translate_solid(
                    &PrimitiveBuilder::make_cylinder(6.0 * s, 60.0 * s).unwrap(),
                    Vec3::new(20.0 * s, 20.0 * s, -20.0 * s),
                );
                (block, drill)
            },
        },
        Placement {
            name: "cylinder x cylinder (orthogonal)",
            build: |s| {
                let upright = PrimitiveBuilder::make_cylinder(10.0 * s, 40.0 * s).unwrap();
                let rotation = Transform3::from_axis_angle(
                    &Vec3::new(0.0, 1.0, 0.0),
                    std::f64::consts::FRAC_PI_2,
                );
                let lying = BrepTransform::translate_solid(
                    &BrepTransform::transform_solid(
                        &PrimitiveBuilder::make_cylinder(6.0 * s, 40.0 * s).unwrap(),
                        &rotation,
                    )
                    .unwrap(),
                    Vec3::new(-20.0 * s, 0.0, 20.0 * s),
                );
                (upright, lying)
            },
        },
        Placement {
            name: "sphere x sphere (overlapping)",
            build: |s| {
                let a = PrimitiveBuilder::make_sphere(10.0 * s).unwrap();
                let b = BrepTransform::translate_solid(
                    &PrimitiveBuilder::make_sphere(8.0 * s).unwrap(),
                    Vec3::new(9.0 * s, 0.0, 0.0),
                );
                (a, b)
            },
        },
        Placement {
            name: "cone x sphere (biting the side)",
            build: |s| {
                let cone = PrimitiveBuilder::make_cone(10.0 * s, 0.0, 20.0 * s).unwrap();
                let sphere = BrepTransform::translate_solid(
                    &PrimitiveBuilder::make_sphere(6.0 * s).unwrap(),
                    Vec3::new(6.0 * s, 0.0, 8.0 * s),
                );
                (cone, sphere)
            },
        },
        Placement {
            name: "torus x cylinder (rod through the hole)",
            build: |s| {
                let torus = PrimitiveBuilder::make_torus(12.0 * s, 4.0 * s).unwrap();
                let rod = BrepTransform::translate_solid(
                    &PrimitiveBuilder::make_cylinder(9.0 * s, 40.0 * s).unwrap(),
                    Vec3::new(0.0, 0.0, -20.0 * s),
                );
                (torus, rod)
            },
        },
    ]
}

fn main() {
    let tol = Tolerance::default();
    // 4桁ぶん振ります。1 が mm 級、100 が建築級。
    //
    // **0.02 より小さいところは門にしません。** 測ったら、恒等式の残差が
    // **0.02 と 0.01 の間で 6桁跳びます**（2.074e-12 → 1.368e-6、さらに
    // 0.005 で 8.912e-6）。公差の床なら滑らかに上がるはずなので、**そこには
    // まだ名前の付いていない欠陥があります**（4-212）。見えるところには
    // 置きますが、赤にはしません——**直っていないものを赤で常設すると、
    // 赤に慣れてしまいます**。
    // **門を 0.005 まで下げました**（4-259。9-H の H6 を達成）。
    //
    // 0.02 以下を門の外に置いていたのは、`torus × cylinder` の恒等式が
    // 0.01 で 1.367e-6、0.005 で 9.038e-6 まで崩れていたからです（4-212）。
    // 原因は**射影の収束判定に次元があった**ことで（`残差 · ∂S/∂u` を絶対の
    // 1e-7 と比べていた）、直すと **3.959e-13 / 1.660e-12** になりました。
    let scales = [100.0_f64, 10.0, 1.0, 0.1, 0.02, 0.01, 0.005];
    let watched: [f64; 0] = [];

    println!("大きさの桁を振って、同じ形を測る（恒等式は相対で見ます）");
    println!();
    println!(
        "{:<40}{:>8}{:>10}{:>16}{:>10}  {}",
        "placement", "scale", "returned", "identity (rel)", "n-manifold", "verdict"
    );
    println!("{}", "-".repeat(104));

    let mut broken = 0usize;
    let mut non_manifold = 0usize;
    let mut refused = 0usize;
    let mut worst_residual = 0.0f64;
    let mut worst_where = String::new();

    // **1つの配置だけを測る口**（`ZENITH_SCALE_FILTER`）。桁の小さいところは
    // 1回が重いので、追っている組だけを回せるようにしておきます。
    let filter = std::env::var("ZENITH_SCALE_FILTER").ok();
    for placement in placements().into_iter().filter(|placement| {
        filter
            .as_deref()
            .map(|needle| placement.name.contains(needle))
            .unwrap_or(true)
    }) {
        for scale in scales.into_iter().chain(watched) {
            let gated = scales.contains(&scale);
            let (a, b) = (placement.build)(scale);
            let mut returned = 0usize;
            let mut volumes = [0.0f64; 3];
            let mut all_returned = true;
            let mut bad_edges = 0usize;

            for (index, op) in [
                BooleanOpType::Difference,
                BooleanOpType::Intersection,
                BooleanOpType::Union,
            ]
            .into_iter()
            .enumerate()
            {
                match BooleanEngine::boolean_solids_exact_result(&a, &b, op, &tol) {
                    Ok(result) => {
                        returned += 1;
                        volumes[index] = total_volume(&result.solids);
                        for solid in &result.solids {
                            bad_edges += non_manifold_brep_edges(solid, scale);
                        }
                    }
                    Err(_) => {
                        all_returned = false;
                        refused += 1;
                    }
                }
            }

            // 恒等式は **A の体積で割って** 見ます。桁に依らない数になります。
            let residual = if all_returned {
                let whole = volume(&a);
                ((volumes[0] + volumes[1]) - whole).abs() / whole.abs().max(f64::MIN_POSITIVE)
            } else {
                f64::NAN
            };
            if residual.is_finite() && residual > worst_residual {
                worst_residual = residual;
                worst_where = format!("{} / scale {scale}", placement.name);
            }

            let identity_bad = residual.is_finite() && residual > 1e-6;
            if identity_bad && gated {
                broken += 1;
            }
            if bad_edges > 0 && gated {
                non_manifold += 1;
            }

            println!(
                "{:<40}{:>8}{:>10}{:>16}{:>10}  {}",
                placement.name,
                scale,
                format!("{returned}/3"),
                if residual.is_finite() {
                    format!("{residual:.3e}")
                } else {
                    "-".to_string()
                },
                bad_edges,
                match (identity_bad, bad_edges > 0, all_returned, gated) {
                    (true, _, _, true) => "**恒等式が破れた**",
                    (_, true, _, true) => "**非多様体を返した**",
                    (true, _, _, false) => "破れているが門の外（4-212 の既知）",
                    (_, true, _, false) => "非多様体だが門の外（4-212 の既知）",
                    (_, _, false, _) => "断った（赤にはしません）",
                    _ => "ok",
                }
            );
        }
    }

    println!("{}", "-".repeat(104));
    println!(
        "恒等式の破れ {broken} 件、非多様体を返したもの {non_manifold} 件、断り {refused} 件。\
         残差の最悪 {worst_residual:.3e}（{worst_where}）。"
    );
    println!();
    println!("**0.02 以下は門の外です**（4-212）。恒等式の残差が 0.02 と 0.01 の間で");
    println!("6桁跳びます。公差の床なら滑らかに上がるはずなので、そこにはまだ名前の");
    println!("付いていない欠陥があります。**見せますが、赤にはしません。**");
    println!();
    println!("**断ることは赤にしません。** 桁の端で断るなら、それは分かったほうが良い事実です。");
    println!(
        "赤にするのは **返ってきたのに答えが合わない** ほうと、**非多様体を返した** ほうです。"
    );

    if broken > 0 || non_manifold > 0 {
        std::process::exit(1);
    }
}
