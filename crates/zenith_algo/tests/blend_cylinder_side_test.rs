//! 平面と、稜を母線に持つ凸の円柱の間の直線の稜のブレンド（HANDOVER 4-964、`ZENITH_CYL_SIDE_BLEND=1`）。
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

fn vertical_front_edge(solid: &Solid) -> u64 {
    let root99 = 99f64.sqrt();
    for face in &solid.outer_shell.faces {
        for oriented in &face.outer_wire.edges {
            let (a, b) = (oriented.edge.start_vertex.point, oriented.edge.end_vertex.point);
            let on = |p: zenith_math::Point3| (p.x - root99).abs() < 1e-6 && p.y.abs() < 1e-6;
            if on(a) && on(b) && (a.z - b.z).abs() > 39.0 {
                return oriented.edge.id;
            }
        }
    }
    panic!("the vertical front edge was not found");
}

fn volume(solid: &Solid) -> f64 {
    MassCalculator::compute_volume_from_brep(solid, &TessellationParams { u_divisions: 64, v_divisions: 64 })
}

/// 断面積（テスト側で独立に: 円の弓形 + 三角形）
fn chamfer_area(s: f64) -> f64 {
    let (r, root99) = (18.0f64, 99f64.sqrt());
    let phi = 2.0 * (s / (2.0 * r)).asin();
    // 頂点 V=(√99,0) から中心 (0,15) への角から φ 回した点が P_b（y が増える側）
    let theta_v = (0.0f64 - 15.0).atan2(root99);
    let pb = (r * (theta_v + phi).cos(), 15.0 + r * (theta_v + phi).sin());
    let pa = (root99 - s, 0.0);
    let v = (root99, 0.0);
    let tri = 0.5 * ((pa.0 - v.0) * (pb.1 - v.1) - (pa.1 - v.1) * (pb.0 - v.0)).abs();
    tri + 0.5 * r * r * (phi - phi.sin())
}

#[test]
fn blends_beside_a_cylinder() {
    std::env::set_var("ZENITH_CYL_SIDE_BLEND", "1");
    let solid = d_block();
    let edge = vertical_front_edge(&solid);
    let before = volume(&solid);
    for kind in [BlendKind::Chamfer { distance: 1.0 }, BlendKind::Fillet { radius: 1.0 }] {
        let (after, report) = EdgeBlender::blend_edge(&solid, edge, kind).unwrap();
        let removed = before - volume(&after);
        eprintln!("{kind:?}: 予測 {} 実測 {removed}", report.predicted_removed_volume);
        assert!(
            (removed - report.predicted_removed_volume).abs() <= 1e-7 * report.predicted_removed_volume,
            "{kind:?}: 削れた {removed} 予測 {}",
            report.predicted_removed_volume
        );
        if let BlendKind::Chamfer { distance } = kind {
            let expected = chamfer_area(distance) * 40.0;
            assert!((report.predicted_removed_volume - expected).abs() <= 1e-9 * expected);
        }
    }
}
