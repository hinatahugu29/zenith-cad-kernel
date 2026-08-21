//! 螺旋掃引の体積で、自前と OpenCASCADE が 2.1e-3 食い違う件を詰める。
//!
//! `mass_convergence` の螺旋ばねは、自前が 770.039017、OCC が 768.397883。
//! 0.2% は、このカーネルで他に例のない大きさである。どちらが外れているかは
//! 突き合わせだけでは決まらない（4-4 と同じで、噛み合わないとき直すべきが
//! どちらかは、噛み合わせを見ても分からない）。
//!
//! ここでは3つ用意する。
//!
//! 1. **閉じた式**。断面の重心が経路の上にあるときだけ `V = A x L` が厳密に
//!    成り立つ。`builder_audit` の螺旋はそちらで、2.20e-7 で乗っている。
//! 2. **B-Rep 積分**（`compute_from_brep`）。面をパラメータ領域で積む。
//! 3. **メッシュ積分**（`compute_from_mesh`）。三角形の発散定理で積む。
//!
//! 2 と 3 は別の経路なので、両方が同じ値に寄るなら、自前の答えは自分の中で
//! 一貫している。そこまで来て初めて「OCC のほうが外れている」と言える。

use zenith_algo::{HelixBuilder, MassCalculator};
use zenith_math::{Point3, Tolerance, Vec3};
use zenith_tess::{tessellate_solid, TessellationParams};
use zenith_topo::{Edge, OrientedEdge, Vertex, Wire};

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
    let radius = 10.0;
    let pitch = 6.0;
    let turns = 2.0;
    let length = turns * ((std::f64::consts::TAU * radius).powi(2) + pitch * pitch).sqrt();

    println!("helix: radius {radius}, pitch {pitch}, turns {turns}, path length {length:.9}");
    println!();

    for (label, cx) in [("centred on the spine", 0.0), ("offset to x = 10", 10.0)] {
        let profile = square_profile(cx, 1.0);
        let area = 4.0;
        let solid = HelixBuilder::sweep_wire_along_helix(
            &profile,
            radius,
            pitch,
            turns,
            Point3::new(0.0, 0.0, 0.0),
            Vec3::new(0.0, 0.0, 1.0),
            64,
            &tol,
        )
        .expect("helix sweep");

        println!("=== profile {label} (area {area}) ===");
        if cx == 0.0 {
            println!("  closed form V = A x L = {:.9}", area * length);
        } else {
            println!("  no closed form: the centroid is off the spine, so V != A x L");
        }
        println!(
            "  {:>6}  {:>18}  {:>18}  {:>12}",
            "div", "brep integral", "mesh integral", "brep - mesh"
        );
        for divisions in [16usize, 32, 64, 128, 256] {
            let params = TessellationParams {
                u_divisions: divisions,
                v_divisions: divisions,
            };
            let brep = MassCalculator::compute_from_brep(&solid, &params).volume;
            let mesh = MassCalculator::compute_from_mesh(&tessellate_solid(&solid, &params)).volume;
            println!(
                "  {divisions:>6}  {brep:>18.9}  {mesh:>18.9}  {:>12.2e}",
                (brep - mesh) / brep
            );
        }
        println!();
    }
}
