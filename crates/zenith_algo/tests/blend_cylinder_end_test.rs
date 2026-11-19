//! 片端が円柱の側面に当たる直線の稜の面取り（HANDOVER 4-962、`ZENITH_CYL_CAP_CHAMFER=1`）。
//!
//! 直方体 20x30x40 と、軸 z・中心 (0,15)・r18 の円柱の積（D字ブロック）。上の縁
//! (0,0,40)–(√99,0,40) は x=√99 で円柱に当たる。面取りの端は面取りの平面と円柱の
//! 楕円弧、削れる体積は (s²/2)·√99 + ∫₀ˢ (s−y)(√(324−(y−15)²) − √99) dy。

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

fn top_front_edge(solid: &Solid) -> u64 {
    let root99 = 99f64.sqrt();
    for face in &solid.outer_shell.faces {
        for oriented in &face.outer_wire.edges {
            let (a, b) = (oriented.edge.start_vertex.point, oriented.edge.end_vertex.point);
            let on = |p: zenith_math::Point3, x: f64| (p.x - x).abs() < 1e-6 && p.y.abs() < 1e-6 && (p.z - 40.0).abs() < 1e-6;
            if (on(a, 0.0) && on(b, root99)) || (on(b, 0.0) && on(a, root99)) {
                return oriented.edge.id;
            }
        }
    }
    panic!("the top front edge was not found");
}

fn volume(solid: &Solid) -> f64 {
    MassCalculator::compute_volume_from_brep(solid, &TessellationParams { u_divisions: 64, v_divisions: 64 })
}

/// ∫₀ˢ (s−y)(√(R²−(y−15)²) − √99) dy の閉じた式（テスト側で独立に書く）
fn expected_removed(s: f64) -> f64 {
    let r: f64 = 18.0;
    let root99 = 99f64.sqrt();
    let f = |v: f64| 0.5 * (v * (r * r - v * v).sqrt() + r * r * (v / r).asin());
    let g = |v: f64| (r * r - v * v).powf(1.5) / 3.0;
    let at = |v: f64| (s - 15.0) * f(v) + g(v);
    let j = at(s - 15.0) - at(-15.0);
    0.5 * s * s * root99 + j - root99 * s * s * 0.5
}

#[test]
fn chamfer_ending_on_a_cylinder() {
    std::env::set_var("ZENITH_CYL_CAP_CHAMFER", "1");
    let solid = d_block();
    let edge = top_front_edge(&solid);
    let (after, report) = EdgeBlender::blend_edge(&solid, edge, BlendKind::Chamfer { distance: 1.0 }).unwrap();
    let removed = volume(&solid) - volume(&after);
    let expected = expected_removed(1.0);
    assert!((report.predicted_removed_volume - expected).abs() <= 1e-9 * expected, "予測 {} 期待 {expected}", report.predicted_removed_volume);
    assert!((removed - expected).abs() <= 1e-7 * expected, "削れた {removed} 期待 {expected}");
    // フィレットは端が有理曲線にならないので断る
    let fillet = EdgeBlender::blend_edge(&solid, edge, BlendKind::Fillet { radius: 1.0 });
    assert!(fillet.is_err());
}
