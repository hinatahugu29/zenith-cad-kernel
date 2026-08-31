//! **自分の出力を、もう一度入力に戻す。**
//!
//! # なぜこの軸か
//!
//! 掃き出しは「置き方を変える」で始まり（4-205、4-207）、**4-211 で 0 件**に
//! なりました。「大きさの桁」に変えたら**また出ました**（4-212）。**枯れたのは
//! 軸であって、カーネルではありません。**
//!
//! ここは3本目の軸です。実務のモデルは、**ブーリアンの結果をもう一度切り**
//! ます。ところが常設の検体は、ほとんどが**ビルダーが作ったばかりの立体**
//! どうしです。ブーリアンの出力は、
//!
//! - 面が分割されていて、境界がパラメータ矩形に沿っていない
//! - 稜が交線の当てはめで、真円でも直線でもない
//! - 同じ弧に別々の `Edge` の実体が並ぶことがある（4-80）
//!
//! ——つまり**ビルダーの出力とは別物**です。`chained_boolean_probe` は
//! 既にありますが、そちらは**平面だけの穴あき板**で、曲面が交わる結果を
//! 戻していません。
//!
//! # 何を測るか
//!
//! 2段目の演算について、**恒等式**（`V(R-C) + V(R∩C) = V(R)`）を相対で
//! 見ます。閉じた式が要らないので、どんな形でも採点できます。
//!
//! | 見るもの | 赤にする条件 |
//! | :--- | :--- |
//! | 2段目の恒等式 | 相対の破れが 1e-6 を超えたら |
//! | B-Rep の非多様体 | 1本でもあれば |
//! | 演算が返るか | **返らないこと自体は赤にしません** |
//!
//! ```bash
//! cargo run --release -p zenith_algo --example rechained_boolean_probe
//! ```

use zenith_algo::{BooleanEngine, BooleanOpType, BrepTransform, MassCalculator, PrimitiveBuilder};
use zenith_math::{Tolerance, Transform3, Vec3};
use zenith_tess::TessellationParams;
use zenith_topo::Solid;

struct Chain {
    name: &'static str,
    /// 1段目。曲面が交わるものを選びます。
    first: fn() -> (Solid, Solid, BooleanOpType),
    /// 2段目に当てる立体。
    second: fn() -> Solid,
}

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

fn total(solids: &[Solid]) -> f64 {
    solids.iter().map(volume).sum()
}

/// B-Rep の稜のうち、ちょうど2つの面ループに使われていない本数（位置で照合）。
fn non_manifold_brep_edges(solid: &Solid) -> usize {
    let quantise = |p: zenith_math::Point3| {
        let q = |v: f64| (v * 1e7).round() as i64;
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

fn chains() -> Vec<Chain> {
    vec![
        Chain {
            name: "(cylinder - cylinder) then cut by a box",
            first: || {
                let upright = PrimitiveBuilder::make_cylinder(10.0, 40.0).unwrap();
                let rotation =
                    Transform3::from_axis_angle(&Vec3::new(0.0, 1.0, 0.0), std::f64::consts::FRAC_PI_2);
                let lying = BrepTransform::translate_solid(
                    &BrepTransform::transform_solid(
                        &PrimitiveBuilder::make_cylinder(6.0, 40.0).unwrap(),
                        &rotation,
                    )
                    .unwrap(),
                    Vec3::new(-20.0, 0.0, 20.0),
                );
                (upright, lying, BooleanOpType::Difference)
            },
            second: || {
                BrepTransform::translate_solid(
                    &PrimitiveBuilder::make_box(30.0, 30.0, 30.0).unwrap(),
                    Vec3::new(-5.0, -15.0, 25.0),
                )
            },
        },
        Chain {
            name: "(sphere - cylinder) then cut by a sphere",
            first: || {
                let ball = PrimitiveBuilder::make_sphere(12.0).unwrap();
                let drill = BrepTransform::translate_solid(
                    &PrimitiveBuilder::make_cylinder(5.0, 40.0).unwrap(),
                    Vec3::new(0.0, 0.0, -20.0),
                );
                (ball, drill, BooleanOpType::Difference)
            },
            second: || {
                BrepTransform::translate_solid(
                    &PrimitiveBuilder::make_sphere(8.0).unwrap(),
                    Vec3::new(10.0, 0.0, 0.0),
                )
            },
        },
        Chain {
            name: "(torus - cylinder) then cut by a box",
            first: || {
                let torus = PrimitiveBuilder::make_torus(12.0, 4.0).unwrap();
                let rod = BrepTransform::translate_solid(
                    &PrimitiveBuilder::make_cylinder(9.0, 40.0).unwrap(),
                    Vec3::new(0.0, 0.0, -20.0),
                );
                (torus, rod, BooleanOpType::Difference)
            },
            second: || {
                BrepTransform::translate_solid(
                    &PrimitiveBuilder::make_box(40.0, 40.0, 40.0).unwrap(),
                    Vec3::new(-20.0, 0.0, -20.0),
                )
            },
        },
        Chain {
            name: "(cone + sphere) then cut by a cylinder",
            first: || {
                let cone = PrimitiveBuilder::make_cone(10.0, 0.0, 20.0).unwrap();
                let ball = BrepTransform::translate_solid(
                    &PrimitiveBuilder::make_sphere(6.0).unwrap(),
                    Vec3::new(0.0, 0.0, 14.0),
                );
                (cone, ball, BooleanOpType::Union)
            },
            second: || {
                BrepTransform::translate_solid(
                    &PrimitiveBuilder::make_cylinder(4.0, 40.0).unwrap(),
                    Vec3::new(0.0, 0.0, -10.0),
                )
            },
        },
        Chain {
            name: "(box - cylinder) then cut by a torus",
            first: || {
                let block = PrimitiveBuilder::make_box(40.0, 40.0, 20.0).unwrap();
                let drill = BrepTransform::translate_solid(
                    &PrimitiveBuilder::make_cylinder(6.0, 60.0).unwrap(),
                    Vec3::new(20.0, 20.0, -20.0),
                );
                (block, drill, BooleanOpType::Difference)
            },
            second: || {
                BrepTransform::translate_solid(
                    &PrimitiveBuilder::make_torus(14.0, 5.0).unwrap(),
                    Vec3::new(20.0, 20.0, 10.0),
                )
            },
        },
    ]
}

fn main() {
    let tol = Tolerance::default();

    println!("ブーリアンの結果を、もう一度切る（2段目の恒等式を相対で見ます）");
    println!();
    println!(
        "{:<44}{:>8}{:>10}{:>16}{:>12}  {}",
        "chain", "faces", "returned", "identity (rel)", "n-manifold", "verdict"
    );
    println!("{}", "-".repeat(108));

    let mut broken = 0usize;
    let mut non_manifold = 0usize;
    let mut refused_first = 0usize;
    let mut refused_second = 0usize;
    let mut worst = 0.0f64;
    let mut worst_where = String::new();

    for chain in chains() {
        let (a, b, op) = (chain.first)();
        let Ok(stage_one) = BooleanEngine::boolean_solids_exact_result(&a, &b, op, &tol) else {
            println!("{:<44}{:>8}  1段目が断られた", chain.name, "-");
            refused_first += 1;
            continue;
        };
        if stage_one.solids.len() != 1 {
            println!(
                "{:<44}{:>8}  1段目が {} 個の立体を返した（この掃き出しは1個のときだけ測ります）",
                chain.name,
                "-",
                stage_one.solids.len()
            );
            continue;
        }
        let result = &stage_one.solids[0];
        let faces = result.outer_shell.faces.len();
        let whole = volume(result);
        let cutter = (chain.second)();

        let mut returned = 0usize;
        let mut volumes = [0.0f64; 2];
        let mut bad_edges = 0usize;
        let mut all_returned = true;
        for (index, second_op) in [BooleanOpType::Difference, BooleanOpType::Intersection]
            .into_iter()
            .enumerate()
        {
            match BooleanEngine::boolean_solids_exact_result(result, &cutter, second_op, &tol) {
                Ok(out) => {
                    returned += 1;
                    volumes[index] = total(&out.solids);
                    for solid in &out.solids {
                        bad_edges += non_manifold_brep_edges(solid);
                    }
                }
                Err(_) => {
                    all_returned = false;
                    refused_second += 1;
                }
            }
        }
        // 和も返るかは見ますが、恒等式には要りません。
        if BooleanEngine::boolean_solids_exact_result(result, &cutter, BooleanOpType::Union, &tol)
            .is_ok()
        {
            returned += 1;
        } else {
            refused_second += 1;
            all_returned = false;
        }

        let residual = if volumes[0] > 0.0 || volumes[1] > 0.0 {
            ((volumes[0] + volumes[1]) - whole).abs() / whole.abs().max(f64::MIN_POSITIVE)
        } else {
            f64::NAN
        };
        if residual.is_finite() && residual > worst {
            worst = residual;
            worst_where = chain.name.to_string();
        }
        let identity_bad = residual.is_finite() && residual > 1e-6;
        if identity_bad {
            broken += 1;
        }
        if bad_edges > 0 {
            non_manifold += 1;
        }

        println!(
            "{:<44}{:>8}{:>10}{:>16}{:>12}  {}",
            chain.name,
            faces,
            format!("{returned}/3"),
            if residual.is_finite() {
                format!("{residual:.3e}")
            } else {
                "-".to_string()
            },
            bad_edges,
            if identity_bad {
                "**恒等式が破れた**"
            } else if bad_edges > 0 {
                "**非多様体を返した**"
            } else if !all_returned {
                "一部を断った（赤にはしません）"
            } else {
                "ok"
            }
        );
    }

    println!("{}", "-".repeat(108));
    println!(
        "恒等式の破れ {broken} 件、非多様体を返したもの {non_manifold} 件、\
         1段目の断り {refused_first} 件、2段目の断り {refused_second} 件。\
         残差の最悪 {worst:.3e}（{worst_where}）。"
    );
    println!();
    println!("**1段目の出力は、ビルダーの出力とは別物です**——面が分割され、稜が交線の");
    println!("当てはめで、同じ弧に別々の実体が並ぶこともあります。そこを2段目に渡すのが");
    println!("この掃き出しの狙いです。");

    if broken > 0 || non_manifold > 0 {
        std::process::exit(1);
    }
}
