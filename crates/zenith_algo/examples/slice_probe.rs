//! Measures whether section slicing actually produces closed loops, and how
//! its reported area compares with the analytic cross-section.
//!
//! Run with: cargo run -p zenith_algo --example slice_probe

use zenith_algo::{BrepTransform, HoleBuilder, PrimitiveBuilder, SectionSlicer};
use zenith_math::{Point3, Tolerance, Vec3};
use zenith_topo::Solid;

/// **1 件測って、健全なら `true`**（4-640）。
///
/// **これまでは何も返していませんでした。** **断られても `ERROR` と
/// 印字するだけ**で、**門が見ている語**（`WRONG` / `PANIC` /
/// `miss(es)` / `over the allowance`）**に 1 つも当たりません**。
/// **終了コードも 0** でした——**つまり、この門は赤くなれません。**
fn probe(
    name: &str,
    solid: &Solid,
    origin: Point3,
    normal: Vec3,
    expected_area: Option<f64>,
) -> bool {
    let tol = Tolerance::default();
    match SectionSlicer::slice_solid(solid, origin, normal, &tol) {
        Ok(result) => {
            let closed_flags: Vec<bool> = result
                .section_wires
                .iter()
                .map(|wire| {
                    let Some(first) = wire.edges.first() else {
                        return false;
                    };
                    let Some(last) = wire.edges.last() else {
                        return false;
                    };
                    (last.end_vertex().point - first.start_vertex().point).norm() <= 1e-6
                })
                .collect();

            // **閉じた式があるものは、合っているかを返します**（4-640）。
            // **許容は相対 1e-6**——**いまの 5 件は、どれも 0.00%** です。
            let mut good = !closed_flags.is_empty() && closed_flags.iter().all(|c| *c);
            let note = match expected_area {
                Some(expected) => {
                    let error = (result.total_area - expected).abs();
                    if error > expected.abs() * 1e-6 {
                        good = false;
                    }
                    format!(
                        "expected area {expected:.4}, error {error:.4} ({:.2}%)",
                        100.0 * error / expected
                    )
                }
                None => String::new(),
            };

            println!(
                "{name:<44} loops={:<3} closed={:<3} area={:<12.4} perim={:<11.4} {note}",
                result.section_wires.len(),
                closed_flags.iter().filter(|c| **c).count(),
                result.total_area,
                result.total_perimeter
            );
            good
        }
        Err(err) => {
            println!("{name:<44} ERROR {err}");
            false
        }
    }
}

fn main() {
    let boxa = PrimitiveBuilder::make_box(20.0, 30.0, 40.0).unwrap();
    let cyl = PrimitiveBuilder::make_cylinder(10.0, 40.0).unwrap();
    let sphere = PrimitiveBuilder::make_sphere(10.0).unwrap();
    let drilled = HoleBuilder::make_drilled_box(30.0, 30.0, 15.0, 5.0).unwrap();
    let tube = BrepTransform::translate_solid(&cyl, Vec3::new(0.0, 0.0, 0.0));

    println!("{:<44} {}", "case", "result");
    println!("{}", "-".repeat(120));

    let mut ran = 0usize;
    let mut ok = 0usize;
    let mut tally = |good: bool| {
        ran += 1;
        if good {
            ok += 1;
        }
    };

    tally(probe(
        "box 20x30x40, z=20 plane",
        &boxa,
        Point3::new(0.0, 0.0, 20.0),
        Vec3::new(0.0, 0.0, 1.0),
        Some(600.0),
    ));
    tally(probe(
        "box 20x30x40, x=10 plane",
        &boxa,
        Point3::new(10.0, 0.0, 0.0),
        Vec3::new(1.0, 0.0, 0.0),
        Some(1200.0),
    ));
    tally(probe(
        "box 20x30x40, diagonal plane",
        &boxa,
        Point3::new(10.0, 15.0, 20.0),
        Vec3::new(1.0, 1.0, 1.0),
        None,
    ));
    tally(probe(
        "cylinder r10 h40, z=20 plane",
        &tube,
        Point3::new(0.0, 0.0, 20.0),
        Vec3::new(0.0, 0.0, 1.0),
        Some(std::f64::consts::PI * 100.0),
    ));
    tally(probe(
        "sphere r10, z=0 plane",
        &sphere,
        Point3::new(0.0, 0.0, 0.0),
        Vec3::new(0.0, 0.0, 1.0),
        Some(std::f64::consts::PI * 100.0),
    ));
    tally(probe(
        "drilled box 30x30x15 r5, z=7.5 plane",
        &drilled,
        Point3::new(0.0, 0.0, 7.5),
        Vec3::new(0.0, 0.0, 1.0),
        Some(900.0 - std::f64::consts::PI * 25.0),
    ));

    // **数の床**（4-640。4-619／4-622／4-639 と同じ型）。
    //
    // **この門は、どの場面でも赤くなれませんでした。** 断られても
    // `ERROR` と印字するだけで、**門の grep にも終了コードにも出ません**。
    // **面積が閉じた式から外れても、誤差を印字するだけ**でした。
    //
    // **場面は固定の 6 件**なので、**数が減ったら赤**、**増えたら
    // 「床を上げてください」**で構いません。
    const EXPECTED_CASES: usize = 6;
    println!();
    println!("{ok} of {ran} cases closed and matched the closed form");
    if ran != EXPECTED_CASES {
        println!(
            "**場面の数が変わりました**: {ran} 件（記録は {EXPECTED_CASES} 件）——**床を直してください**（4-640）。"
        );
        std::process::exit(1);
    }
    if ok != EXPECTED_CASES {
        println!(
            "**合わない場面があります**: {ok} / {EXPECTED_CASES} 件（4-640）。"
        );
        std::process::exit(1);
    }
}
