//! 円柱の縁の部分円弧の面取り（HANDOVER 4-966、`ZENITH_ARC_RIM_CHAMFER=1`）。
//!
//! D字ブロック（直方体 20x30x40 と、軸 z・中心 (0,15)・r18 の円柱の積）の縦の稜
//! (√99,0,z)。断面は平面 y=0 と円の交わりで、高さ 40 の角柱。

use zenith_algo::{
    BlendKind, BooleanEngine, BooleanOpType, BrepTransform, EdgeBlender, FaceMerger, MassCalculator,
    PrimitiveBuilder,
};
use zenith_math::{Tolerance, Vec3};
use zenith_tess::TessellationParams;
use zenith_topo::Solid;

fn d_block() -> Solid {
    let tol = Tolerance::default();
    let block = PrimitiveBuilder::make_box(20.0, 30.0, 40.0).unwrap();
    let cylinder = BrepTransform::translate_solid(
        &PrimitiveBuilder::make_cylinder(18.0, 60.0).unwrap(),
        Vec3::new(0.0, 15.0, -10.0),
    );
    let raw = BooleanEngine::boolean_solids_exact_result(&block, &cylinder, BooleanOpType::Intersection, &tol)
        .unwrap()
        .solids
        .remove(0);
    FaceMerger::simplify_solid(&raw, &tol).unwrap().0
}


fn top_arc(solid: &Solid) -> u64 {
    for face in &solid.outer_shell.faces {
        for oriented in &face.outer_wire.edges {
            let e = &oriented.edge;
            let (t0, t1) = e.curve.param_range();
            let m = e.curve.evaluate((t0 + t1) * 0.5);
            let (a, b) = (e.start_vertex.point, e.end_vertex.point);
            if (a.z - 40.0).abs() < 1e-6 && (b.z - 40.0).abs() < 1e-6 && (m.z - 40.0).abs() < 1e-6 && m.x > 10.0 && (a.y.abs() < 1e-6 || b.y.abs() < 1e-6) {
                return e.id;
            }
        }
    }
    panic!("the top arc was not found");
}

fn volume(solid: &Solid) -> f64 {
    MassCalculator::compute_volume_from_brep(solid, &TessellationParams { u_divisions: 64, v_divisions: 64 })
}

/// ∫_{R−s}^{R} (s−R+r)·r·2·asin(15/r) dr（テスト側で Simpson）
fn expected(s: f64) -> f64 {
    let r_big = 18.0;
    let n = 4000;
    let (a, b) = (r_big - s, r_big);
    let h = (b - a) / n as f64;
    let f = |r: f64| (s - r_big + r) * r * 2.0 * (15.0f64 / r).asin();
    let mut sum = f(a) + f(b);
    for i in 1..n {
        sum += f(a + h * i as f64) * if i % 2 == 1 { 4.0 } else { 2.0 };
    }
    sum * h / 3.0
}

#[test]
fn chamfers_a_partial_rim() {
    std::env::set_var("ZENITH_ARC_RIM_CHAMFER", "1");
    let solid = d_block();
    let edge = top_arc(&solid);
    let (after, report) = EdgeBlender::blend_edge(&solid, edge, BlendKind::Chamfer { distance: 1.0 }).unwrap();
    let removed = volume(&solid) - volume(&after);
    let want = expected(1.0);
    eprintln!("予測 {} 実測 {removed} 独立 {want}", report.predicted_removed_volume);
    assert!((report.predicted_removed_volume - want).abs() <= 1e-9 * want);
    assert!((removed - want).abs() <= 1e-7 * want, "削れた {removed} 期待 {want}");
}

fn bottom_arc(solid: &Solid) -> u64 {
    for face in &solid.outer_shell.faces {
        for oriented in &face.outer_wire.edges {
            let e = &oriented.edge;
            let (t0, t1) = e.curve.param_range();
            let m = e.curve.evaluate((t0 + t1) * 0.5);
            let (a, b) = (e.start_vertex.point, e.end_vertex.point);
            if a.z.abs() < 1e-6 && b.z.abs() < 1e-6 && m.z.abs() < 1e-6 && m.x > 10.0 && (a.y.abs() < 1e-6 || b.y.abs() < 1e-6) {
                return e.id;
            }
        }
    }
    panic!("the bottom arc was not found");
}

#[test]
fn chamfers_a_partial_rim_from_below() {
    std::env::set_var("ZENITH_ARC_RIM_CHAMFER", "1");
    let solid = d_block();
    let edge = bottom_arc(&solid);
    let (after, report) = EdgeBlender::blend_edge(&solid, edge, BlendKind::Chamfer { distance: 1.0 }).unwrap();
    let removed = volume(&solid) - volume(&after);
    let want = expected(1.0);
    eprintln!("下: 予測 {} 実測 {removed} 独立 {want}", report.predicted_removed_volume);
    assert!((report.predicted_removed_volume - want).abs() <= 1e-9 * want);
    assert!((removed - want).abs() <= 1e-7 * want, "削れた {removed} 期待 {want}");
}
