//! 平面と、稜を母線に持つ凹の円柱の間の直線の稜のブレンド（HANDOVER 4-968、`ZENITH_CYL_CONCAVE_SIDE=1`）。
//!
//! 直方体 20x30x40 から、軸 z・中心 (20,30)・r8 の円柱を抜いた差の縦の稜 (12,30,z)。
//! 断面は平面 y=30（立体は y<30）と、立体が外側にある円の交わりで、高さ 40 の角柱。

use zenith_algo::{
    BlendKind, BooleanEngine, BooleanOpType, BrepTransform, EdgeBlender, FaceMerger, MassCalculator,
    PrimitiveBuilder,
};
use zenith_math::{Tolerance, Vec3};
use zenith_tess::TessellationParams;
use zenith_topo::Solid;

const R: f64 = 8.0;

fn notched_box() -> Solid {
    let tol = Tolerance::default();
    let block = PrimitiveBuilder::make_box(20.0, 30.0, 40.0).unwrap();
    let cylinder = BrepTransform::translate_solid(
        &PrimitiveBuilder::make_cylinder(R, 60.0).unwrap(),
        Vec3::new(20.0, 30.0, -10.0),
    );
    let raw = BooleanEngine::boolean_solids_exact_result(&block, &cylinder, BooleanOpType::Difference, &tol)
        .unwrap()
        .solids
        .remove(0);
    FaceMerger::simplify_solid(&raw, &tol).unwrap().0
}

fn vertical_edge(solid: &Solid) -> u64 {
    for face in &solid.outer_shell.faces {
        for oriented in &face.outer_wire.edges {
            let (a, b) = (oriented.edge.start_vertex.point, oriented.edge.end_vertex.point);
            let on = |p: zenith_math::Point3| (p.x - 12.0).abs() < 1e-6 && (p.y - 30.0).abs() < 1e-6;
            if on(a) && on(b) && (a.z - b.z).abs() > 39.0 {
                return oriented.edge.id;
            }
        }
    }
    panic!("the vertical edge was not found");
}

fn volume(solid: &Solid) -> f64 {
    MassCalculator::compute_volume_from_brep(solid, &TessellationParams { u_divisions: 64, v_divisions: 64 })
}

/// 円の弓形（弦の中心角 θ）
fn segment(theta: f64) -> f64 {
    0.5 * R * R * (theta - theta.sin())
}

/// 面取りの断面積（テスト側で独立に）: 三角形 V, P_a, P_b から、円が三角形に食い込む弓形を引く。
fn chamfer_area(s: f64) -> f64 {
    let alpha = s * s / (2.0 * R);
    let beta = (s * s - alpha * alpha).sqrt();
    0.5 * s * beta - segment(2.0 * (s / (2.0 * R)).asin())
}

/// フィレットの断面積（テスト側で独立に）: 中心 O=(20−√((R+r)²−r²), 30−r)。
/// 四角形 V, P_a, O, P_b から、O の扇形と、円が食い込む弓形を引く。
fn fillet_area(r: f64) -> f64 {
    let ox = 20.0 - ((R + r) * (R + r) - r * r).sqrt();
    let (o, v, pa) = ((ox, 30.0 - r), (12.0, 30.0), (ox, 30.0));
    let k = R / (R + r);
    let pb = (20.0 + (o.0 - 20.0) * k, 30.0 + (o.1 - 30.0) * k);
    let poly = [v, pa, o, pb];
    let mut quad = 0.0;
    for i in 0..4 {
        let (p, q) = (poly[i], poly[(i + 1) % 4]);
        quad += p.0 * q.1 - p.1 * q.0;
    }
    let quad = 0.5 * quad.abs();
    // O の扇形: OP_a は +y、OP_b は C の向き
    let uy = (30.0 - o.1) / (R + r);
    let phi = uy.clamp(-1.0, 1.0).acos();
    let sector = 0.5 * r * r * phi;
    let chord = ((pb.0 - v.0).powi(2) + (pb.1 - v.1).powi(2)).sqrt();
    quad - sector - segment(2.0 * (chord / (2.0 * R)).asin())
}

#[test]
fn blends_beside_a_concave_cylinder() {
    std::env::set_var("ZENITH_CYL_CONCAVE_SIDE", "1");
    let solid = notched_box();
    let edge = vertical_edge(&solid);
    let before = volume(&solid);
    for kind in [BlendKind::Chamfer { distance: 1.0 }, BlendKind::Fillet { radius: 1.0 }, BlendKind::Fillet { radius: 3.0 }] {
        let (after, report) = EdgeBlender::blend_edge(&solid, edge, kind).unwrap();
        let removed = before - volume(&after);
        let expected = 40.0
            * match kind {
                BlendKind::Chamfer { distance } => chamfer_area(distance),
                BlendKind::Fillet { radius } => fillet_area(radius),
            };
        eprintln!("{kind:?}: 予測 {} 独立 {expected} 実測 {removed}", report.predicted_removed_volume);
        assert!((report.predicted_removed_volume - expected).abs() <= 1e-9 * expected, "{kind:?}: 予測が独立の式と違う");
        assert!((removed - expected).abs() <= 1e-7 * expected, "{kind:?}: 削れた {removed} 期待 {expected}");
    }
}
