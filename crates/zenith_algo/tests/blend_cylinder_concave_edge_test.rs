//! 平面と、稜を母線に持つ凸の円柱の間の凹の稜（入隅）のブレンド（HANDOVER 4-970、
//! `ZENITH_CYL_SIDE_CONCAVE_EDGE=1`）。
//!
//! 直方体 20x30x40 の側面 x=20 に、軸 z・中心 (20,15)・r6・高さ 40 の円柱を半分埋めた和の
//! 縦の入隅 (20,9,z)。材料を足すので、削れる体積は負。

use zenith_algo::{
    BlendKind, BooleanEngine, BooleanOpType, BrepTransform, EdgeBlender, FaceMerger, MassCalculator,
    PrimitiveBuilder,
};
use zenith_math::{Tolerance, Vec3};
use zenith_tess::TessellationParams;
use zenith_topo::Solid;

const R: f64 = 6.0;

fn bossed_box() -> Solid {
    let tol = Tolerance::default();
    let block = PrimitiveBuilder::make_box(20.0, 30.0, 40.0).unwrap();
    let cylinder = BrepTransform::translate_solid(
        &PrimitiveBuilder::make_cylinder(R, 40.0).unwrap(),
        Vec3::new(20.0, 15.0, 0.0),
    );
    let raw = BooleanEngine::boolean_solids_exact_result(&block, &cylinder, BooleanOpType::Union, &tol)
        .unwrap()
        .solids
        .remove(0);
    FaceMerger::simplify_solid(&raw, &tol).unwrap().0
}

fn inner_corner(solid: &Solid) -> u64 {
    for face in &solid.outer_shell.faces {
        for oriented in &face.outer_wire.edges {
            let (a, b) = (oriented.edge.start_vertex.point, oriented.edge.end_vertex.point);
            let on = |p: zenith_math::Point3| (p.x - 20.0).abs() < 1e-6 && (p.y - 9.0).abs() < 1e-6;
            if on(a) && on(b) && (a.z - b.z).abs() > 39.0 {
                return oriented.edge.id;
            }
        }
    }
    panic!("the inner corner was not found");
}

fn volume(solid: &Solid) -> f64 {
    MassCalculator::compute_volume_from_brep(solid, &TessellationParams { u_divisions: 64, v_divisions: 64 })
}

fn segment(theta: f64) -> f64 {
    0.5 * R * R * (theta - theta.sin())
}

/// 面取りで足す断面積（独立に）: 三角形 V, P_a, P_b から円の弓形を引く。
fn chamfer_area(s: f64) -> f64 {
    let alpha = s * s / (2.0 * R);
    let beta = (s * s - alpha * alpha).sqrt();
    0.5 * s * beta - segment(2.0 * (s / (2.0 * R)).asin())
}

/// フィレットで足す断面積（独立に）: 中心 O=(20+r, 15−√((R+r)²−r²))。
fn fillet_area(r: f64) -> f64 {
    let o = (20.0 + r, 15.0 - ((R + r) * (R + r) - r * r).sqrt());
    let (v, pa) = ((20.0, 9.0), (20.0, o.1));
    let k = R / (R + r);
    let pb = (20.0 + (o.0 - 20.0) * k, 15.0 + (o.1 - 15.0) * k);
    let poly = [v, pa, o, pb];
    let mut quad = 0.0;
    for i in 0..4 {
        let (p, q) = (poly[i], poly[(i + 1) % 4]);
        quad += p.0 * q.1 - p.1 * q.0;
    }
    let quad = 0.5 * quad.abs();
    // O の扇形: OP_a は −x、OP_b は中心 (20,15) の向き
    let ux = (20.0 - o.0) / (R + r);
    let phi = (-ux).clamp(-1.0, 1.0).acos();
    let sector = 0.5 * r * r * phi;
    let chord = ((pb.0 - v.0).powi(2) + (pb.1 - v.1).powi(2)).sqrt();
    quad - sector - segment(2.0 * (chord / (2.0 * R)).asin())
}

#[test]
fn blends_an_inner_corner_beside_a_cylinder() {
    std::env::set_var("ZENITH_CYL_SIDE_CONCAVE_EDGE", "1");
    let solid = bossed_box();
    let edge = inner_corner(&solid);
    let before = volume(&solid);
    for kind in [BlendKind::Chamfer { distance: 1.0 }, BlendKind::Fillet { radius: 1.0 }, BlendKind::Fillet { radius: 3.0 }] {
        let (after, report) = EdgeBlender::blend_edge(&solid, edge, kind).unwrap();
        let removed = before - volume(&after);
        let expected = -40.0
            * match kind {
                BlendKind::Chamfer { distance } => chamfer_area(distance),
                BlendKind::Fillet { radius } => fillet_area(radius),
            };
        eprintln!("{kind:?}: 予測 {} 独立 {expected} 実測 {removed}", report.predicted_removed_volume);
        assert!((report.predicted_removed_volume - expected).abs() <= 1e-9 * expected.abs(), "{kind:?}: 予測が独立の式と違う");
        assert!((removed - expected).abs() <= 1e-7 * expected.abs(), "{kind:?}: 削れた {removed} 期待 {expected}");
    }
}
