//! 2つの立体の最短距離が、閉じた式に乗るか。
//!
//! 距離はクリアランス検証に使う値です。「隙間 0.2 mm」を答えるつもりの関数が
//! 別の値を返していれば、干渉していない設計が干渉していることになります。
//!
//! ここでは答えが手で書ける配置だけを並べます。平面どうし、球どうし、平板と
//! 球のように**最近点が頂点に来ない**配置を意図的に入れてあります。

use zenith_algo::{BrepTransform, DistanceEngine, PrimitiveBuilder};
use zenith_math::{Tolerance, Vec3};
use zenith_topo::Solid;

struct Case {
    name: &'static str,
    a: Solid,
    b: Solid,
    expected: f64,
}

fn main() {
    let tol = Tolerance::default();
    let mut cases: Vec<Case> = Vec::new();

    // 1. 面と面が正対する。最近点は面の内側どこでもよい。
    cases.push(Case {
        name: "two boxes, faces 10 apart",
        a: PrimitiveBuilder::make_box(10.0, 10.0, 10.0).unwrap(),
        b: BrepTransform::translate_solid(
            &PrimitiveBuilder::make_box(10.0, 10.0, 10.0).unwrap(),
            Vec3::new(20.0, 0.0, 0.0),
        ),
        expected: 10.0,
    });

    // 2. 大きな平板の真ん中の上に小さな球。最近点は板の**面の中央**で、
    //    板の頂点からは遠い。頂点どうしを見ていると答えが桁で外れる。
    let plate = PrimitiveBuilder::make_box(200.0, 200.0, 2.0).unwrap();
    let ball = BrepTransform::translate_solid(
        &PrimitiveBuilder::make_sphere(5.0).unwrap(),
        Vec3::new(100.0, 100.0, 2.0 + 5.0 + 3.0),
    );
    cases.push(Case {
        name: "small ball 3 above a big plate",
        a: plate.clone(),
        b: ball,
        expected: 3.0,
    });

    // 3. 球どうし。中心間 40、半径 10 ずつ。
    cases.push(Case {
        name: "two spheres, 20 apart",
        a: PrimitiveBuilder::make_sphere(10.0).unwrap(),
        b: BrepTransform::translate_solid(
            &PrimitiveBuilder::make_sphere(10.0).unwrap(),
            Vec3::new(40.0, 0.0, 0.0),
        ),
        expected: 20.0,
    });

    // 4. 平行な円柱。軸間 30、半径 5 ずつ。
    cases.push(Case {
        name: "two parallel cylinders, 20 apart",
        a: PrimitiveBuilder::make_cylinder(5.0, 20.0).unwrap(),
        b: BrepTransform::translate_solid(
            &PrimitiveBuilder::make_cylinder(5.0, 20.0).unwrap(),
            Vec3::new(30.0, 0.0, 0.0),
        ),
        expected: 20.0,
    });

    // 5. 板の面の上に、辺で向き合う細い角材。最近点は辺と面。
    cases.push(Case {
        name: "thin bar 0.5 above a plate",
        a: plate.clone(),
        b: BrepTransform::translate_solid(
            &PrimitiveBuilder::make_box(4.0, 4.0, 4.0).unwrap(),
            Vec3::new(98.0, 98.0, 2.5),
        ),
        expected: 0.5,
    });

    // 6. 接している。
    cases.push(Case {
        name: "two boxes touching",
        a: PrimitiveBuilder::make_box(10.0, 10.0, 10.0).unwrap(),
        b: BrepTransform::translate_solid(
            &PrimitiveBuilder::make_box(10.0, 10.0, 10.0).unwrap(),
            Vec3::new(10.0, 0.0, 0.0),
        ),
        expected: 0.0,
    });

    // 7. めり込んでいる。距離は 0。
    cases.push(Case {
        name: "two boxes overlapping",
        a: PrimitiveBuilder::make_box(10.0, 10.0, 10.0).unwrap(),
        b: BrepTransform::translate_solid(
            &PrimitiveBuilder::make_box(10.0, 10.0, 10.0).unwrap(),
            Vec3::new(5.0, 0.0, 0.0),
        ),
        expected: 0.0,
    });

    println!(
        "{:<34}{:>14}{:>14}{:>14}  {}",
        "case", "expected", "measured", "error", "closest points on both surfaces?"
    );
    println!("{}", "-".repeat(110));

    let mut wrong = 0;
    for case in &cases {
        let result = DistanceEngine::compute_min_distance(&case.a, &case.b, &tol);
        let error = (result.min_distance - case.expected).abs();
        let relative = error / case.expected.abs().max(1.0);
        let separation = (result.closest_point_b - result.closest_point_a).norm();
        let consistent = (separation - result.min_distance).abs() < 1e-9;

        println!(
            "{:<34}{:>14.6}{:>14.6}{:>14.3e}  {}",
            case.name,
            case.expected,
            result.min_distance,
            error,
            if consistent { "consistent" } else { "INCONSISTENT" }
        );
        if relative > 1e-3 {
            wrong += 1;
        }
    }

    println!("{}", "-".repeat(110));
    println!("{wrong} of {} cases miss the closed form by more than 0.1%", cases.len());

    // 刻みの細かさは探索の出発点にしか効かないはず。答えが刻みで動くなら、
    // B-Rep へ詰める段が効いていない。
    println!();
    println!("{:<34}{:>12}{:>12}{:>12}{:>12}", "case", "8x8", "16x16", "32x32", "spread");
    println!("{}", "-".repeat(82));
    let mut unstable = 0;
    for case in &cases {
        let values: Vec<f64> = [8usize, 16, 32]
            .into_iter()
            .map(|d| {
                DistanceEngine::compute_min_distance_with_tessellation(
                    &case.a,
                    &case.b,
                    &tol,
                    &zenith_tess::TessellationParams {
                        u_divisions: d,
                        v_divisions: d,
                    },
                )
                .min_distance
            })
            .collect();
        let spread = values.iter().cloned().fold(f64::NEG_INFINITY, f64::max)
            - values.iter().cloned().fold(f64::INFINITY, f64::min);
        println!(
            "{:<34}{:>12.6}{:>12.6}{:>12.6}{:>12.2e}",
            case.name, values[0], values[1], values[2], spread
        );
        if spread > 1e-9 {
            unstable += 1;
        }
    }
    println!("{}", "-".repeat(82));
    println!("{unstable} of {} cases move when the tessellation changes", cases.len());

    if wrong > 0 || unstable > 0 {
        std::process::exit(1);
    }
    println!("every distance lands on the closed form and does not move with the tessellation");
}
