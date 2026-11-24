//! 円錐×円柱の全周の稜（皿もみの底）のブレンド（HANDOVER 4-975、`ZENITH_REVOLVED_CORNER=1`）。
//!
//! 皿もみの貫通穴（r4、口 r8→r4 の円錐、深さ 4）の底の円 ρ=4, z=36。断面 (ρ, z) で
//! 円錐の向き (1,1)/√2、円柱の向き (0,−1)、材料の角 135°。予測を閉じた式の Pappus と、
//! ブレンド後の体積を厳密な体積 − 予測と照合する。

use std::f64::consts::{FRAC_1_SQRT_2, PI, TAU};

use zenith_algo::{BlendKind, BooleanEngine, BooleanOpType, BrepTransform, EdgeBlender, FaceMerger, MassCalculator, PrimitiveBuilder};
use zenith_math::{Tolerance, Vec3};
use zenith_tess::TessellationParams;
use zenith_topo::Solid;

fn diff(a: Solid, b: Solid) -> Solid {
    let tol = Tolerance::default();
    let r = BooleanEngine::boolean_solids_exact_result(&a, &b, BooleanOpType::Difference, &tol).unwrap().solids.remove(0);
    FaceMerger::simplify_solid(&r, &tol).unwrap().0
}

fn countersink() -> Solid {
    let base = PrimitiveBuilder::make_box(20.0, 30.0, 40.0).unwrap();
    let h = BrepTransform::translate_solid(&PrimitiveBuilder::make_cylinder(4.0, 60.0).unwrap(), Vec3::new(10.0, 15.0, -10.0));
    let c = BrepTransform::translate_solid(&PrimitiveBuilder::make_cone(4.0, 8.0, 4.0).unwrap(), Vec3::new(10.0, 15.0, 36.0));
    diff(diff(base, h), c)
}

fn bottom_edge(solid: &Solid) -> u64 {
    for face in &solid.outer_shell.faces {
        for w in std::iter::once(&face.outer_wire).chain(face.inner_wires.iter()) {
            for o in &w.edges {
                let (p, q) = (o.edge.start_vertex.point, o.edge.end_vertex.point);
                let on = |p: zenith_math::Point3| (p.z - 36.0).abs() < 1e-6 && (((p.x - 10.0).powi(2) + (p.y - 15.0).powi(2)).sqrt() - 4.0).abs() < 1e-6;
                if on(p) && on(q) {
                    return o.edge.id;
                }
            }
        }
    }
    panic!("bottom edge not found");
}

/// 線分 p→q の ∫ρ² dz
fn line(p: (f64, f64), q: (f64, f64)) -> f64 {
    (q.1 - p.1) * (p.0 * p.0 + p.0 * q.0 + q.0 * q.0) / 3.0
}

/// 中心 o・半径 r の円弧（角 a0→a1）の ∫ρ² dz。ρ = oρ + r cos a、dz = r cos a da
fn arc(o: (f64, f64), r: f64, a0: f64, a1: f64) -> f64 {
    let g = |a: f64| {
        let s = a.sin();
        r * o.0 * o.0 * s + 2.0 * o.0 * r * r * (a / 2.0 + (2.0 * a).sin() / 4.0) + r.powi(3) * (s - s.powi(3) / 3.0)
    };
    g(a1) - g(a0)
}

fn check(solid: &Solid, edge: u64, kind: BlendKind, exact_before: f64, expected: f64) {
    std::env::set_var("ZENITH_REVOLVED_CORNER", "1");
    let (after, report) = EdgeBlender::blend_edge(solid, edge, kind).unwrap();
    let v = MassCalculator::compute_volume_from_brep(&after, &TessellationParams { u_divisions: 64, v_divisions: 64 });
    let target = exact_before - expected;
    eprintln!("{kind:?}: 予測 {} 独立 {expected} 後 {v} 期待 {target} 相対 {:.3e}", report.predicted_removed_volume, (v - target) / expected);
    assert!((report.predicted_removed_volume - expected).abs() <= 1e-9 * expected);
    assert!((v - target).abs() <= 1e-7 * expected);
}

#[test]
fn countersink_bottom() {
    let s = countersink();
    let e = bottom_edge(&s);
    let exact = 24000.0 - 640.0 * PI - (4.0 / 3.0 * 112.0 - 64.0) * PI;
    let h = FRAC_1_SQRT_2;
    let p0 = (4.0, 36.0);
    for d in [0.5, 1.0] {
        let (a, b) = ((4.0 + d * h, 36.0 + d * h), (4.0, 36.0 - d));
        check(&s, e, BlendKind::Chamfer { distance: d }, exact, TAU * (0.5 * (line(p0, a) + line(a, b) + line(b, p0))).abs());
    }
    for r in [0.5, 1.0] {
        let t = r / 67.5f64.to_radians().tan();
        let (a, b) = ((4.0 + t * h, 36.0 + t * h), (4.0, 36.0 - t));
        let o = (4.0 + r, 36.0 - t);
        // B は O から角 π、A は O から角 3π/4（円錐の内向き法線 (1,−1)/√2 の逆）
        let m = line(p0, a) + arc(o, r, 0.75 * PI, PI) + line(b, p0);
        check(&s, e, BlendKind::Fillet { radius: r }, exact, TAU * (0.5 * m).abs());
    }
}
