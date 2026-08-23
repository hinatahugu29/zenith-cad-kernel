//! どこまで浅い食い込みを、食い込みとして検出できるか。
//!
//! 干渉判定で危ないのは**見落とし**です。離れているものを干渉と言えば設計者が
//! 確かめて終わりますが、食い込んでいるものを「隙間あり」と言えばそのまま
//! 製造に流れます。
//!
//! 食い込み量を段階的に減らして、どこで検出できなくなるかを測ります。
//! 併せて、報告される最短距離が閉じた式に乗るかも見ます。

use zenith_algo::{
    BrepTransform, ClashStatus, DistanceEngine, InterferenceChecker, PrimitiveBuilder,
};
use zenith_math::{Tolerance, Vec3};
use zenith_topo::Solid;

fn label(status: ClashStatus) -> &'static str {
    match status {
        ClashStatus::Clearance => "clearance",
        ClashStatus::Touching => "touching",
        ClashStatus::Clash => "CLASH",
    }
}

fn main() {
    let tol = Tolerance::default();
    let depths = [5.0f64, 1.0, 0.5, 0.1, 0.05, 0.01, 0.001];

    println!("食い込み量を減らしていったときの判定");
    println!(
        "{:<34}{:>10}{:>12}{:>14}{:>14}",
        "case", "overlap", "status", "reported gap", "true gap"
    );
    println!("{}", "-".repeat(84));

    let mut missed = 0;
    let mut wrong_gap = 0;

    let bodies: Vec<(&str, Box<dyn Fn(f64) -> (Solid, Solid)>)> = vec![
        (
            "two boxes, face into face",
            Box::new(|depth: f64| {
                let a = PrimitiveBuilder::make_box(20.0, 20.0, 20.0).unwrap();
                let b = BrepTransform::translate_solid(
                    &PrimitiveBuilder::make_box(20.0, 20.0, 20.0).unwrap(),
                    Vec3::new(20.0 - depth, 0.0, 0.0),
                );
                (a, b)
            }),
        ),
        (
            "a pin pressed into a plate",
            Box::new(|depth: f64| {
                let plate = PrimitiveBuilder::make_box(60.0, 60.0, 10.0).unwrap();
                let pin = BrepTransform::translate_solid(
                    &PrimitiveBuilder::make_cylinder(2.0, 20.0).unwrap(),
                    Vec3::new(30.0, 30.0, 10.0 - depth),
                );
                (plate, pin)
            }),
        ),
        (
            "a ball pressed into a plate",
            Box::new(|depth: f64| {
                let plate = PrimitiveBuilder::make_box(60.0, 60.0, 10.0).unwrap();
                let ball = BrepTransform::translate_solid(
                    &PrimitiveBuilder::make_sphere(5.0).unwrap(),
                    Vec3::new(30.0, 30.0, 15.0 - depth),
                );
                (plate, ball)
            }),
        ),
    ];

    for (name, make) in &bodies {
        for depth in depths {
            let (a, b) = make(depth);
            let report = InterferenceChecker::check(&a, &b, &tol);
            let measured = DistanceEngine::compute_min_distance(&a, &b, &tol).min_distance;
            println!(
                "{name:<34}{depth:>10.3}{:>12}{:>14.6}{:>14.6}",
                label(report.status),
                report.min_distance,
                measured
            );
            if report.status != ClashStatus::Clash {
                missed += 1;
            }
        }
        println!();
    }

    // 箱が離れている配置。速い経路を通っても距離が閉じた式に乗るか。
    println!("箱が離れている配置での報告距離");
    println!("{:<34}{:>10}{:>12}{:>14}{:>14}", "case", "gap", "status", "reported", "expected");
    println!("{}", "-".repeat(84));
    for (name, a, b, expected) in [
        (
            "two spheres side by side",
            PrimitiveBuilder::make_sphere(10.0).unwrap(),
            BrepTransform::translate_solid(
                &PrimitiveBuilder::make_sphere(10.0).unwrap(),
                Vec3::new(40.0, 0.0, 0.0),
            ),
            20.0f64,
        ),
        (
            "two parallel cylinders",
            PrimitiveBuilder::make_cylinder(5.0, 20.0).unwrap(),
            BrepTransform::translate_solid(
                &PrimitiveBuilder::make_cylinder(5.0, 20.0).unwrap(),
                Vec3::new(30.0, 0.0, 0.0),
            ),
            20.0,
        ),
    ] {
        let report = InterferenceChecker::check(&a, &b, &tol);
        println!(
            "{name:<34}{expected:>10.3}{:>12}{:>14.6}{expected:>14.6}",
            label(report.status),
            report.min_distance
        );
        if (report.min_distance - expected).abs() / expected > 1e-6 {
            wrong_gap += 1;
        }
    }
    println!();

    // 離れている側。報告される距離が閉じた式に乗るか。
    println!("離れている配置での報告距離");
    println!("{:<34}{:>10}{:>12}{:>14}{:>14}", "case", "gap", "status", "reported", "expected");
    println!("{}", "-".repeat(84));
    for gap in [10.0f64, 1.0, 0.1] {
        let plate = PrimitiveBuilder::make_box(200.0, 200.0, 2.0).unwrap();
        let ball = BrepTransform::translate_solid(
            &PrimitiveBuilder::make_sphere(5.0).unwrap(),
            Vec3::new(100.0, 100.0, 7.0 + gap),
        );
        let report = InterferenceChecker::check(&plate, &ball, &tol);
        println!(
            "{:<34}{gap:>10.3}{:>12}{:>14.6}{:>14.6}",
            "ball above a big plate",
            label(report.status),
            report.min_distance,
            gap
        );
        if (report.min_distance - gap).abs() / gap > 1e-3 {
            wrong_gap += 1;
        }
    }

    println!("{}", "-".repeat(84));
    println!("食い込みを見落とした回数        : {missed}");
    println!("報告距離が閉じた式から外れた回数: {wrong_gap}");
    if missed > 0 || wrong_gap > 0 {
        println!();
        println!("A missed clash reaches manufacturing. A reported gap that is not the gap");
        println!("is worse than no gap at all.");
        std::process::exit(1);
    }
    println!("every overlap was reported as a clash, and every gap matched the closed form");
}
