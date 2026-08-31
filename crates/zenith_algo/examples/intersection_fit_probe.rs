//! **交線が、乗っているはずの曲面からどれだけ浮いているか。**
//!
//! # なぜ測るか
//!
//! 4-214 で、正しく割れた片が落とされていました。切り込みの稜が支持曲面から
//! **1.895788e-5** 浮いていて `Face::pcurves` が断り、面積の検算が受け皿へ
//! 落ちて**トリムを知らない素のパッチ**を測っていた、というものです。
//!
//! そこで「交線そのものの当てはめが悪いのではないか」と疑いました。
//! **ここで測った結果、その疑いは外れました**（4-215）——交線は 1段目・2段目
//! あわせて **101 本すべてが 5e-7 以下**で合っています。浮かせていたのは
//! `split_by_chain` が切り込みの端を境界へ**吸着**させる段でした
//! （端の制御点を動かすので、**動かした量ちょうど浮きます**）。
//!
//! この掃き出しは、その**反証**として残してあります。断りの入口は
//! `project_edge_to_nurbs_pcurve` の
//! `on_surface_limit = tol.linear.max(1e-6) * 10.0`——**1e-5 の絶対値**です。
//! **1e-5 を超えた曲線は、それを境界に持つ面の p-curve を取れなくします。**
//! いつか交線の当てはめが悪くなったら、ここが先に鳴ります。
//!
//! **門にはしていません**——まだ数を見るためのものです。
//!
//! ```bash
//! cargo run --release -p zenith_algo --example intersection_fit_probe
//! ```

use zenith_algo::{
    BooleanOpType, BrepIntersectionBuilder, BrepTransform, PrimitiveBuilder, Regularizer,
};
use zenith_geom::ExtremumEngine;
use zenith_math::{Point3, Tolerance, Transform3, Vec3};
use zenith_topo::{Face, FaceGeometry, Solid};

/// `project_edge_to_nurbs_pcurve` が使っている絶対の閾値。
const ON_SURFACE_LIMIT: f64 = 1e-5;

struct Placement {
    name: &'static str,
    build: fn() -> (Solid, Solid),
}

/// 点が面の曲面からどれだけ離れているか。曲面の種類ごとに厳密に測ります。
fn distance_to_surface(face: &Face, point: Point3) -> Option<f64> {
    match &face.geometry {
        FaceGeometry::Plane(plane) => Some((point - plane.origin).dot(&plane.normal).abs()),
        FaceGeometry::Nurbs(surface) => ExtremumEngine::point_to_surface(point, surface, 48, 1e-13)
            .ok()
            .map(|result| result.distance),
        _ => None,
    }
}

fn placements() -> Vec<Placement> {
    vec![
        Placement {
            name: "box x cylinder (through drill)",
            build: || {
                let block = PrimitiveBuilder::make_box(40.0, 40.0, 20.0).unwrap();
                let drill = BrepTransform::translate_solid(
                    &PrimitiveBuilder::make_cylinder(6.0, 60.0).unwrap(),
                    Vec3::new(20.0, 20.0, -20.0),
                );
                (block, drill)
            },
        },
        Placement {
            name: "cylinder x cylinder (orthogonal)",
            build: || {
                let upright = PrimitiveBuilder::make_cylinder(10.0, 40.0).unwrap();
                let rotation = Transform3::from_axis_angle(
                    &Vec3::new(0.0, 1.0, 0.0),
                    std::f64::consts::FRAC_PI_2,
                );
                let lying = BrepTransform::translate_solid(
                    &BrepTransform::transform_solid(
                        &PrimitiveBuilder::make_cylinder(6.0, 40.0).unwrap(),
                        &rotation,
                    )
                    .unwrap(),
                    Vec3::new(-20.0, 0.0, 20.0),
                );
                (upright, lying)
            },
        },
        Placement {
            name: "sphere x sphere (overlapping)",
            build: || {
                let a = PrimitiveBuilder::make_sphere(10.0).unwrap();
                let b = BrepTransform::translate_solid(
                    &PrimitiveBuilder::make_sphere(8.0).unwrap(),
                    Vec3::new(9.0, 0.0, 0.0),
                );
                (a, b)
            },
        },
        Placement {
            name: "sphere x cylinder (drilled)",
            build: || {
                let ball = PrimitiveBuilder::make_sphere(12.0).unwrap();
                let drill = BrepTransform::translate_solid(
                    &PrimitiveBuilder::make_cylinder(5.0, 40.0).unwrap(),
                    Vec3::new(0.0, 0.0, -20.0),
                );
                (ball, drill)
            },
        },
        Placement {
            name: "cone x sphere (biting the side)",
            build: || {
                let cone = PrimitiveBuilder::make_cone(10.0, 0.0, 20.0).unwrap();
                let sphere = BrepTransform::translate_solid(
                    &PrimitiveBuilder::make_sphere(6.0).unwrap(),
                    Vec3::new(6.0, 0.0, 8.0),
                );
                (cone, sphere)
            },
        },
        Placement {
            name: "torus x cylinder (rod through the hole)",
            build: || {
                let torus = PrimitiveBuilder::make_torus(12.0, 4.0).unwrap();
                let rod = BrepTransform::translate_solid(
                    &PrimitiveBuilder::make_cylinder(9.0, 40.0).unwrap(),
                    Vec3::new(0.0, 0.0, -20.0),
                );
                (torus, rod)
            },
        },
        Placement {
            name: "torus x sphere (ball in the tube)",
            build: || {
                let torus = PrimitiveBuilder::make_torus(12.0, 4.0).unwrap();
                let ball = BrepTransform::translate_solid(
                    &PrimitiveBuilder::make_sphere(6.0).unwrap(),
                    Vec3::new(12.0, 0.0, 0.0),
                );
                (torus, ball)
            },
        },
        Placement {
            name: "cylinder x cone (side by side)",
            build: || {
                let cylinder = PrimitiveBuilder::make_cylinder(8.0, 30.0).unwrap();
                let cone = BrepTransform::translate_solid(
                    &PrimitiveBuilder::make_cone(9.0, 0.0, 24.0).unwrap(),
                    Vec3::new(10.0, 0.0, 2.0),
                );
                (cylinder, cone)
            },
        },
    ]
}

fn main() {
    let tol = Tolerance::default();

    println!("交線が、乗っているはずの曲面からどれだけ浮いているか");
    println!();
    println!(
        "{:<42}{:>8}{:>14}{:>14}{:>10}  {}",
        "placement", "edges", "worst A", "worst B", "over 1e-5", "where"
    );
    println!("{}", "-".repeat(112));

    let mut worst_overall = 0.0f64;
    let mut worst_overall_where = String::new();
    let mut total_edges = 0usize;
    let mut total_over = 0usize;

    for placement in placements() {
        let (a, b) = (placement.build)();
        let held_a = Regularizer::hold_like_our_own(&a, &tol);
        let held_b = Regularizer::hold_like_our_own(&b, &tol);
        let assembly = BrepIntersectionBuilder::collect_boolean_shell_assembly(
            &held_a,
            &held_b,
            BooleanOpType::Difference,
            &tol,
        );

        let mut worst_a = 0.0f64;
        let mut worst_b = 0.0f64;
        let mut over = 0usize;
        let mut over_where = String::new();
        let edges = assembly.edge_candidates.len();

        for candidate in &assembly.edge_candidates {
            let face_a = &held_a.outer_shell.faces[candidate.face_a_index];
            let face_b = &held_b.outer_shell.faces[candidate.face_b_index];
            let (t0, t1) = candidate.edge.curve.param_range();
            let samples = 33;
            let mut here_a = 0.0f64;
            let mut here_b = 0.0f64;
            for step in 0..=samples {
                let t = t0 + (t1 - t0) * step as f64 / samples as f64;
                let point = candidate.edge.curve.evaluate(t);
                if let Some(distance) = distance_to_surface(face_a, point) {
                    here_a = here_a.max(distance);
                }
                if let Some(distance) = distance_to_surface(face_b, point) {
                    here_b = here_b.max(distance);
                }
            }
            worst_a = worst_a.max(here_a);
            worst_b = worst_b.max(here_b);
            let here = here_a.max(here_b);
            if here > ON_SURFACE_LIMIT {
                over += 1;
                if over_where.is_empty() {
                    let start = candidate.edge.start_vertex.point;
                    over_where = format!(
                        "A{} x B{} from ({:.2} {:.2} {:.2})",
                        candidate.face_a_index, candidate.face_b_index, start.x, start.y, start.z
                    );
                }
            }
            if here > worst_overall {
                worst_overall = here;
                worst_overall_where = placement.name.to_string();
            }
        }

        total_edges += edges;
        total_over += over;
        println!(
            "{:<42}{:>8}{:>14}{:>14}{:>10}  {}",
            placement.name,
            edges,
            format!("{worst_a:.3e}"),
            format!("{worst_b:.3e}"),
            over,
            over_where
        );
    }

    println!("{}", "-".repeat(112));
    println!(
        "交線 {total_edges} 本のうち、**{total_over} 本**が 1e-5 を超えています。\
         最悪 {worst_overall:.3e}（{worst_overall_where}）。"
    );
    println!();
    println!("**1e-5 を超えた交線は、それを境界に持つ面の p-curve を取れなくします**");
    println!("（`project_edge_to_nurbs_pcurve` の `on_surface_limit`）。そこから先は");
    println!("面積の検算が受け皿へ落ち、**トリムを知らない素のパッチ**を測ります（4-214）。");
    println!();
    println!("**門にはしていません。** まず数を見るためのものです。");

    second_stage(&tol);
}

/// **2段目も同じ物差しで測ります。**
///
/// 1段目（ビルダーの出力どうし）は上の表のとおりよく合っています。ところが
/// 4-214 が拾った 1.895788e-5 は、**ブーリアンの結果をもう一度切ったとき**の
/// 交線でした。そこが違うなら、違いは「面が割れていること」にあります。
fn second_stage(tol: &Tolerance) {
    println!();
    println!("{}", "=".repeat(112));
    println!("2段目——ブーリアンの結果を、もう一度切ったときの交線");
    println!();
    println!(
        "{:<42}{:>8}{:>14}{:>14}{:>10}  {}",
        "chain", "edges", "worst A", "worst B", "over 1e-5", "where"
    );
    println!("{}", "-".repeat(112));

    let chains: Vec<(&str, fn() -> (Solid, Solid))> = vec![
        ("(cyl - cyl) then a box", || {
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
            let first = zenith_algo::BooleanEngine::boolean_solids_exact_result(
                &upright,
                &lying,
                BooleanOpType::Difference,
                &Tolerance::default(),
            )
            .unwrap();
            let cutter = BrepTransform::translate_solid(
                &PrimitiveBuilder::make_box(30.0, 30.0, 30.0).unwrap(),
                Vec3::new(-5.0, -15.0, 25.0),
            );
            (first.solids[0].clone(), cutter)
        }),
        ("(sphere - cyl) then a sphere", || {
            let ball = PrimitiveBuilder::make_sphere(12.0).unwrap();
            let drill = BrepTransform::translate_solid(
                &PrimitiveBuilder::make_cylinder(5.0, 40.0).unwrap(),
                Vec3::new(0.0, 0.0, -20.0),
            );
            let first = zenith_algo::BooleanEngine::boolean_solids_exact_result(
                &ball,
                &drill,
                BooleanOpType::Difference,
                &Tolerance::default(),
            )
            .unwrap();
            let cutter = BrepTransform::translate_solid(
                &PrimitiveBuilder::make_sphere(8.0).unwrap(),
                Vec3::new(10.0, 0.0, 0.0),
            );
            (first.solids[0].clone(), cutter)
        }),
        ("(torus - cyl) then a box", || {
            let torus = PrimitiveBuilder::make_torus(12.0, 4.0).unwrap();
            let rod = BrepTransform::translate_solid(
                &PrimitiveBuilder::make_cylinder(9.0, 40.0).unwrap(),
                Vec3::new(0.0, 0.0, -20.0),
            );
            let first = zenith_algo::BooleanEngine::boolean_solids_exact_result(
                &torus,
                &rod,
                BooleanOpType::Difference,
                &Tolerance::default(),
            )
            .unwrap();
            let cutter = BrepTransform::translate_solid(
                &PrimitiveBuilder::make_box(40.0, 40.0, 40.0).unwrap(),
                Vec3::new(-20.0, 0.0, -20.0),
            );
            (first.solids[0].clone(), cutter)
        }),
    ];

    let mut total_edges = 0usize;
    let mut total_over = 0usize;
    let mut worst_overall = 0.0f64;
    let mut worst_overall_where = String::new();

    for (name, build) in chains {
        let (a, b) = build();
        let held_a = Regularizer::hold_like_our_own(&a, tol);
        let held_b = Regularizer::hold_like_our_own(&b, tol);
        let assembly = BrepIntersectionBuilder::collect_boolean_shell_assembly(
            &held_a,
            &held_b,
            BooleanOpType::Difference,
            tol,
        );

        let mut worst_a = 0.0f64;
        let mut worst_b = 0.0f64;
        let mut over = 0usize;
        let mut over_where = String::new();

        for candidate in &assembly.edge_candidates {
            let face_a = &held_a.outer_shell.faces[candidate.face_a_index];
            let face_b = &held_b.outer_shell.faces[candidate.face_b_index];
            let (t0, t1) = candidate.edge.curve.param_range();
            let samples = 33;
            let mut here_a = 0.0f64;
            let mut here_b = 0.0f64;
            for step in 0..=samples {
                let t = t0 + (t1 - t0) * step as f64 / samples as f64;
                let point = candidate.edge.curve.evaluate(t);
                if let Some(distance) = distance_to_surface(face_a, point) {
                    here_a = here_a.max(distance);
                }
                if let Some(distance) = distance_to_surface(face_b, point) {
                    here_b = here_b.max(distance);
                }
            }
            worst_a = worst_a.max(here_a);
            worst_b = worst_b.max(here_b);
            let here = here_a.max(here_b);
            if here > ON_SURFACE_LIMIT {
                over += 1;
                if over_where.is_empty() {
                    let start = candidate.edge.start_vertex.point;
                    over_where = format!(
                        "A{} x B{} from ({:.2} {:.2} {:.2})",
                        candidate.face_a_index, candidate.face_b_index, start.x, start.y, start.z
                    );
                }
            }
            if here > worst_overall {
                worst_overall = here;
                worst_overall_where = name.to_string();
            }
        }

        total_edges += assembly.edge_candidates.len();
        total_over += over;
        println!(
            "{:<42}{:>8}{:>14}{:>14}{:>10}  {}",
            name,
            assembly.edge_candidates.len(),
            format!("{worst_a:.3e}"),
            format!("{worst_b:.3e}"),
            over,
            over_where
        );
    }

    println!("{}", "-".repeat(112));
    println!(
        "2段目の交線 {total_edges} 本のうち、**{total_over} 本**が 1e-5 を超えています。\
         最悪 {worst_overall:.3e}（{worst_overall_where}）。"
    );
}
