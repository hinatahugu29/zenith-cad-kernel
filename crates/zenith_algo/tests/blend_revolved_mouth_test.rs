//! 平面×回転面（円錐・凹の球）の全周の口のブレンド（HANDOVER 4-973、`ZENITH_REVOLVED_MOUTH=1`）。
//!
//! 皿もみの貫通穴（r4、口 r8→r4 の円錐、深さ 4）と、上面の球のくぼみ（中心 (10,15,44)、r6）。
//! 予測を、断面の閉じた式（線分と円弧の ∫ρ² dd の原始関数）による Pappus と照合し、
//! ブレンド後の立体の体積を、厳密な体積から予測を引いた値と照合する。
//!
//! 削れた体積を「前 − 後」で測らないのは、元のくぼみの体積の積分（面より広い球の面を
//! p-curve で切ったもの）が厳密値から 2.7e-7 ずれるため（作り直した面には無いずれ）。

use std::f64::consts::{FRAC_PI_2, PI, TAU};

use zenith_algo::{
    BlendKind, BooleanEngine, BooleanOpType, BrepTransform, EdgeBlender, FaceMerger, MassCalculator,
    PrimitiveBuilder,
};
use zenith_math::{Tolerance, Vec3};
use zenith_tess::TessellationParams;
use zenith_topo::Solid;

fn diff(a: Solid, b: Solid) -> Solid {
    let tol = Tolerance::default();
    let r = BooleanEngine::boolean_solids_exact_result(&a, &b, BooleanOpType::Difference, &tol)
        .unwrap()
        .solids
        .remove(0);
    FaceMerger::simplify_solid(&r, &tol).unwrap().0
}

fn dimple() -> Solid {
    let base = PrimitiveBuilder::make_box(20.0, 30.0, 40.0).unwrap();
    diff(base, BrepTransform::translate_solid(&PrimitiveBuilder::make_sphere(6.0).unwrap(), Vec3::new(10.0, 15.0, 44.0)))
}

fn countersink() -> Solid {
    let base = PrimitiveBuilder::make_box(20.0, 30.0, 40.0).unwrap();
    let h = BrepTransform::translate_solid(&PrimitiveBuilder::make_cylinder(4.0, 60.0).unwrap(), Vec3::new(10.0, 15.0, -10.0));
    let c = BrepTransform::translate_solid(&PrimitiveBuilder::make_cone(4.0, 8.0, 4.0).unwrap(), Vec3::new(10.0, 15.0, 36.0));
    diff(diff(base, h), c)
}

/// 上面 z=40 の口の円の稜（半径 rho）
fn mouth_edge(solid: &Solid, rho: f64) -> u64 {
    for face in &solid.outer_shell.faces {
        for w in &face.inner_wires {
            for o in &w.edges {
                let p = o.edge.start_vertex.point;
                if (p.z - 40.0).abs() < 1e-6
                    && (((p.x - 10.0).powi(2) + (p.y - 15.0).powi(2)).sqrt() - rho).abs() < 1e-6
                {
                    return o.edge.id;
                }
            }
        }
    }
    panic!("mouth edge not found");
}

fn volume(solid: &Solid) -> f64 {
    MassCalculator::compute_volume_from_brep(solid, &TessellationParams { u_divisions: 64, v_divisions: 64 })
}

/// 線分 p→q の ∫ρ² dd
fn line(p: (f64, f64), q: (f64, f64)) -> f64 {
    (q.1 - p.1) * (p.0 * p.0 + p.0 * q.0 + q.0 * q.0) / 3.0
}

/// 中心 o・半径 r の円弧（角 a0→a1）の ∫ρ² dd。ρ = oρ + r cos a、dd = r cos a da の原始関数。
fn arc(o: (f64, f64), r: f64, a0: f64, a1: f64) -> f64 {
    let g = |a: f64| {
        let s = a.sin();
        r * o.0 * o.0 * s + 2.0 * o.0 * r * r * (a / 2.0 + (2.0 * a).sin() / 4.0) + r.powi(3) * (s - s.powi(3) / 3.0)
    };
    g(a1) - g(a0)
}

fn pappus(moment: f64) -> f64 {
    TAU * (0.5 * moment).abs()
}

fn check(solid: &Solid, edge: u64, kind: BlendKind, exact_before: f64, expected: f64) {
    std::env::set_var("ZENITH_REVOLVED_MOUTH", "1");
    let (after, report) = EdgeBlender::blend_edge(solid, edge, kind).unwrap();
    let after_volume = volume(&after);
    let target = exact_before - expected;
    eprintln!(
        "{kind:?}: 予測 {} 独立 {expected} 後の体積 {after_volume} 期待 {target} 相対 {:.3e}",
        report.predicted_removed_volume,
        (after_volume - target) / expected
    );
    assert!(
        (report.predicted_removed_volume - expected).abs() <= 1e-9 * expected,
        "{kind:?}: 予測 {} 独立 {expected}",
        report.predicted_removed_volume
    );
    assert!((after_volume - target).abs() <= 1e-7 * expected, "{kind:?}: 後の体積 {after_volume} 期待 {target}");
}

#[test]
fn countersink_mouth() {
    // 断面（ρ, d）: 口 P0=(8,0)、円錐の向き (−1,1)/√2、材料の角 135°
    let s = countersink();
    let e = mouth_edge(&s, 8.0);
    // 箱 − 貫通穴 r4 − 皿もみの円錐台（r8→r4、高さ 4）のうち穴の外の分
    let exact = 24000.0 - 640.0 * PI - (4.0 / 3.0 * 112.0 - 64.0) * PI;
    let h = std::f64::consts::FRAC_1_SQRT_2;
    let (p0, a, b) = ((8.0, 0.0), (9.0, 0.0), (8.0 - h, h));
    check(&s, e, BlendKind::Chamfer { distance: 1.0 }, exact, pappus(line(p0, a) + line(a, b) + line(b, p0)));
    // フィレット 1: 接点は P0 から t = 1/tan(67.5°)、中心 O=(8+t, 1)。A は O から −90°、B は −135°
    let t = 1.0 / 67.5f64.to_radians().tan();
    let o = (8.0 + t, 1.0);
    let (a, b) = ((8.0 + t, 0.0), (o.0 - h, o.1 - h));
    let m = line(p0, a) + arc(o, 1.0, -FRAC_PI_2, -0.75 * PI) + line(b, p0);
    check(&s, e, BlendKind::Fillet { radius: 1.0 }, exact, pappus(m));
}

#[test]
fn dimple_mouth() {
    // 断面: 球の中心 C=(0, −4)、半径 6、口 P0=(√20, 0)
    let s = dimple();
    let rho0 = 20f64.sqrt();
    let e = mouth_edge(&s, rho0);
    let exact = 24000.0 - PI * 64.0 / 3.0; // 高さ 2 の球冠
    let c = (0.0, -4.0);
    let a_p0 = 4.0f64.atan2(rho0);
    // 面取り 1: A=(ρ0+1, 0)、B は弦 1 だけ極の側（角 +2 asin(1/12)）、B→P0 は球の弧
    let a_b = a_p0 + 2.0 * (1.0f64 / 12.0).asin();
    let b = (6.0 * a_b.cos(), -4.0 + 6.0 * a_b.sin());
    let m = line((rho0, 0.0), (rho0 + 1.0, 0.0)) + line((rho0 + 1.0, 0.0), b) + arc(c, 6.0, a_b, a_p0);
    check(&s, e, BlendKind::Chamfer { distance: 1.0 }, exact, pappus(m));
    // フィレット 1: 中心 O=(√(7²−5²), 1)=(√24, 1)、B は O から C の向きに 1
    let o = (24f64.sqrt(), 1.0);
    let ang_b = (c.1 - o.1).atan2(c.0 - o.0);
    let b = (o.0 + ang_b.cos(), o.1 + ang_b.sin());
    let a_b = (b.1 - c.1).atan2(b.0 - c.0);
    let m = line((rho0, 0.0), (o.0, 0.0)) + arc(o, 1.0, -FRAC_PI_2, ang_b) + arc(c, 6.0, a_b, a_p0);
    check(&s, e, BlendKind::Fillet { radius: 1.0 }, exact, pappus(m));
}
