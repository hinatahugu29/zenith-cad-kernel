//! 干渉判定が、答えの分かっている配置で何と言うか。
//!
//! 以前は軸並行の箱だけで判定しており、箱が重なれば `Clash` と答えていた。
//! 離れている立体でも箱が重なれば干渉と報告し、`min_distance` は箱同士の
//! 距離、`overlap_volume` は箱の重なりの体積だった。
//!
//! ここは**答えの分かる配置**を並べて、状態・距離・体積の3つを突き合わせる。
//! 距離と体積は三角形に割った表面の上で測るので、曲面を含む配置では分割の
//! 細かさぶんだけずれる。ずれる向きは決まっていて、距離は大きめ、体積は
//! 小さめに出る（内接多角形になるため）。
//!
//! 走らせ方: cargo run --release -p zenith_algo --example interference_probe

use zenith_algo::{BrepTransform, ClashStatus, InterferenceChecker, PrimitiveBuilder};
use zenith_math::{Tolerance, Vec3};
use zenith_topo::Solid;

struct Subject {
    name: &'static str,
    a: Solid,
    b: Solid,
    status: ClashStatus,
    /// 分かっているなら、離隔距離。
    distance: Option<f64>,
    /// 分かっているなら、重なりの体積。
    overlap: Option<f64>,
    /// 曲面を含む配置では分割の粗さぶんだけずれる。その許容。
    tolerance: f64,
}

fn main() {
    let tol = Tolerance::default();
    let cube = |size: f64| PrimitiveBuilder::make_box(size, size, size).unwrap();
    let shift = |solid: &Solid, x: f64, y: f64, z: f64| {
        BrepTransform::translate_solid(solid, Vec3::new(x, y, z))
    };

    let subjects = vec![
        Subject {
            name: "two cubes far apart",
            a: cube(20.0),
            b: shift(&cube(20.0), 50.0, 0.0, 0.0),
            status: ClashStatus::Clearance,
            distance: Some(30.0),
            overlap: Some(0.0),
            tolerance: 1e-9,
        },
        Subject {
            name: "two cubes face to face",
            a: cube(20.0),
            b: shift(&cube(20.0), 20.0, 0.0, 0.0),
            status: ClashStatus::Touching,
            distance: Some(0.0),
            overlap: Some(0.0),
            tolerance: 1e-9,
        },
        Subject {
            name: "two cubes overlapping at a corner",
            a: cube(20.0),
            b: shift(&cube(20.0), 10.0, 10.0, 10.0),
            status: ClashStatus::Clash,
            distance: Some(0.0),
            overlap: Some(1000.0),
            tolerance: 1e-9,
        },
        Subject {
            name: "a small cube inside a big one",
            a: cube(20.0),
            b: shift(&cube(4.0), 8.0, 8.0, 8.0),
            status: ClashStatus::Clash,
            distance: Some(0.0),
            overlap: Some(64.0),
            tolerance: 1e-9,
        },
        Subject {
            // 球の表面と箱のいちばん近い隅 (3,3,3) の距離は sqrt(27) - 5。
            // 箱は重なるので、箱だけで見ていた頃はここを Clash と答えていた。
            name: "sphere and a box whose corner misses it",
            a: PrimitiveBuilder::make_sphere(5.0).unwrap(),
            b: shift(&cube(7.0), 3.0, 3.0, 3.0),
            status: ClashStatus::Clearance,
            distance: Some(27.0f64.sqrt() - 5.0),
            overlap: Some(0.0),
            tolerance: 2e-2,
        },
        Subject {
            name: "two rods crossing without a shared vertex",
            a: BrepTransform::translate_solid(
                &PrimitiveBuilder::make_box(40.0, 2.0, 2.0).unwrap(),
                Vec3::new(-20.0, -1.0, -1.0),
            ),
            b: BrepTransform::translate_solid(
                &PrimitiveBuilder::make_box(2.0, 40.0, 2.0).unwrap(),
                Vec3::new(-1.0, -20.0, -1.0),
            ),
            status: ClashStatus::Clash,
            distance: Some(0.0),
            overlap: Some(8.0),
            tolerance: 1e-9,
        },
    ];

    println!(
        "{:<44} {:>10} {:>10} {:>12} {:>12} {:>12} {:>12}",
        "subject", "status", "expected", "distance", "expected", "overlap", "expected"
    );
    println!("{}", "-".repeat(118));

    let mut clean = 0usize;
    let mut problems = 0usize;

    for subject in &subjects {
        let report = InterferenceChecker::check(&subject.a, &subject.b, &tol);
        let mut bad = report.status != subject.status;

        let distance_note = match subject.distance {
            Some(expected) => {
                let error = (report.min_distance - expected).abs();
                if error > subject.tolerance.max(expected.abs() * subject.tolerance) {
                    bad = true;
                }
                format!("{expected:.6}")
            }
            None => "-".to_string(),
        };
        let overlap_note = match subject.overlap {
            Some(expected) => {
                let error = (report.overlap_volume - expected).abs();
                let allowed = subject.tolerance.max(expected.abs() * 0.05);
                if error > allowed {
                    bad = true;
                }
                format!("{expected:.6}")
            }
            None => "-".to_string(),
        };

        if bad {
            problems += 1;
        } else {
            clean += 1;
        }

        println!(
            "{:<44} {:>10} {:>10} {:>12.6} {:>12} {:>12.6} {:>12}",
            subject.name,
            format!("{:?}", report.status),
            format!("{:?}", subject.status),
            report.min_distance,
            distance_note,
            report.overlap_volume,
            overlap_note
        );
    }

    println!("{}", "-".repeat(118));
    println!(
        "{clean} of {} interference cases agree, {problems} with problems",
        subjects.len()
    );
    println!();
    println!("distance and overlap are measured on the tessellated surfaces at");
    println!(
        "{} divisions. A curved surface becomes an inscribed polygon, so the",
        InterferenceChecker::DEFAULT_DIVISIONS
    );
    println!("distance reads slightly long and the overlap slightly short.");
}
