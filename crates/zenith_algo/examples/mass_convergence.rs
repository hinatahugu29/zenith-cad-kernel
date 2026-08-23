//! Checks whether the kernel's mass integration has converged, by refining the
//! integration domain and watching the value settle.
//!
//! Run with: cargo run --release -p zenith_algo --example mass_convergence

use std::f64::consts::PI;

use zenith_algo::{HelixBuilder, MassCalculator, PrimitiveBuilder, SweepBuilder};
use zenith_geom::NurbsCurve3;
use zenith_math::{Point3, Tolerance, Vec3};
use zenith_tess::TessellationParams;
use zenith_topo::{Edge, OrientedEdge, Solid, Vertex, Wire};

fn report(name: &str, solid: &Solid, reference: Option<f64>) {
    println!("=== {name}");
    let mut previous: Option<f64> = None;
    for divisions in [12usize, 24, 48, 96, 192] {
        let params = TessellationParams {
            u_divisions: divisions,
            v_divisions: divisions,
        };
        let mass = MassCalculator::compute_from_brep(solid, &params);
        let delta = previous
            .map(|p: f64| format!("{:+.3e}", mass.volume - p))
            .unwrap_or_else(|| "-".to_string());
        let against = reference
            .map(|r| format!("{:.3e}", (mass.volume - r).abs() / r.abs()))
            .unwrap_or_else(|| "-".to_string());
        println!(
            "    {divisions:>4} divisions: volume {:>14.6}  area {:>14.6}  step {delta:>11}  vs reference {against}",
            mass.volume, mass.surface_area
        );
        previous = Some(mass.volume);
    }
    println!();
}

fn square_profile(cx: f64, half: f64) -> Wire {
    let points = [
        Point3::new(cx - half, -half, 0.0),
        Point3::new(cx + half, -half, 0.0),
        Point3::new(cx + half, half, 0.0),
        Point3::new(cx - half, half, 0.0),
    ];
    let vertices: Vec<Vertex> = points.into_iter().map(Vertex::from_point).collect();
    let edges = (0..4)
        .map(|index| {
            let edge =
                Edge::line_between(vertices[index].clone(), vertices[(index + 1) % 4].clone())
                    .unwrap();
            OrientedEdge::forward(edge)
        })
        .collect();
    Wire::new(edges)
}

fn main() {
    let tol = Tolerance::default();

    report(
        "cylinder r10 h40 (analytic 12566.370614)",
        &PrimitiveBuilder::make_cylinder(10.0, 40.0).unwrap(),
        Some(PI * 100.0 * 40.0),
    );

    let path = NurbsCurve3::bspline_from_points(
        3,
        vec![
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(10.0, 0.0, 10.0),
            Point3::new(20.0, 20.0, 25.0),
            Point3::new(30.0, 20.0, 40.0),
        ],
    )
    .unwrap();
    // 他カーネルの読みをここに写さないこと。かつて「OCC reads 1275.086556
    // for the area」と書いてありましたが、実際の OCC は 1276.598256 で、
    // 自前の 1276.600032 と 1.39e-6 で一致します。写した数字は、対象が
    // 変わっても、カーネルが直っても、そのまま残ります。**存在しない
    // 1.19e-3 の食い違いを報告し続けていました。** 突き合わせは
    // `tools/freecad_cross_validate.py` にあり、そこは実際に両方を測ります。
    report(
        "swept pipe (cross-checked in freecad_cross_validate: area 1.39e-6)",
        &SweepBuilder::sweep_circle_along_curve(&path, 3.5, 16).unwrap(),
        None,
    );

    let helix = HelixBuilder::sweep_wire_along_helix(
        &square_profile(10.0, 1.0),
        10.0,
        6.0,
        2.0,
        Point3::new(0.0, 0.0, 0.0),
        Vec3::new(0.0, 0.0, 1.0),
        64,
        &tol,
    )
    .unwrap();
    // 同じく写した数字でした。「OCC reads 768.397883」に対し、実際の OCC は
    // 770.039001732、自前は 770.039017238 で 2.01e-8 の一致です。0.2% の
    // 食い違いは幻で、それを信じて原因を探すと何日でも溶けます。
    //
    // この断面は重心が経路から外れている（x=10 が中心）ので `V = A x L` は
    // 成り立ちません。閉じた式が要るなら `builder_audit` の螺旋のほうを
    // 見てください。あちらは原点中心の断面で、2.20e-7 で式に乗ります。
    report(
        "helix spring (cross-checked in freecad_cross_validate: volume 2.01e-8)",
        &helix,
        None,
    );
}
