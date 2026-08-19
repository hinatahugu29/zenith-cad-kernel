//! Measures whether section slicing actually produces closed loops, and how
//! its reported area compares with the analytic cross-section.
//!
//! Run with: cargo run -p zenith_algo --example slice_probe

use zenith_algo::{BrepTransform, HoleBuilder, PrimitiveBuilder, SectionSlicer};
use zenith_math::{Point3, Tolerance, Vec3};
use zenith_topo::Solid;

fn probe(name: &str, solid: &Solid, origin: Point3, normal: Vec3, expected_area: Option<f64>) {
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

            let note = match expected_area {
                Some(expected) => {
                    let error = (result.total_area - expected).abs();
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
        }
        Err(err) => println!("{name:<44} ERROR {err}"),
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

    probe(
        "box 20x30x40, z=20 plane",
        &boxa,
        Point3::new(0.0, 0.0, 20.0),
        Vec3::new(0.0, 0.0, 1.0),
        Some(600.0),
    );
    probe(
        "box 20x30x40, x=10 plane",
        &boxa,
        Point3::new(10.0, 0.0, 0.0),
        Vec3::new(1.0, 0.0, 0.0),
        Some(1200.0),
    );
    probe(
        "box 20x30x40, diagonal plane",
        &boxa,
        Point3::new(10.0, 15.0, 20.0),
        Vec3::new(1.0, 1.0, 1.0),
        None,
    );
    probe(
        "cylinder r10 h40, z=20 plane",
        &tube,
        Point3::new(0.0, 0.0, 20.0),
        Vec3::new(0.0, 0.0, 1.0),
        Some(std::f64::consts::PI * 100.0),
    );
    probe(
        "sphere r10, z=0 plane",
        &sphere,
        Point3::new(0.0, 0.0, 0.0),
        Vec3::new(0.0, 0.0, 1.0),
        Some(std::f64::consts::PI * 100.0),
    );
    probe(
        "drilled box 30x30x15 r5, z=7.5 plane",
        &drilled,
        Point3::new(0.0, 0.0, 7.5),
        Vec3::new(0.0, 0.0, 1.0),
        Some(900.0 - std::f64::consts::PI * 25.0),
    );
}
