//! 端が円柱の埋もれた側に退く直線の稜の面取り（HANDOVER 4-979、`ZENITH_CYL_BURIED_END=1`）。
//!
//! 直方体（20×30×40）の側面 x=20 に、軸 z・中心 (20,15)・r6 の円柱を半分埋めた和。
//! 上下の面と側面の稜（y 0→9、21→30）は、端で円柱に当たる。円の面（上下の面）の弧は
//! 退く向き（−x）と逆の +x へ走るので、退いた点は円柱の埋もれた半分に乗る。
//!
//! 削れる体積: 断面の三角形 (u = 20−x, z) ∈ {u, z ≥ 0, u + z ≤ d} を、y=0（または 30）から
//! 円柱まで（y の長さ 15 − √(36 − u²)）。V = ∫₀ᵈ (d−u)(15 − √(36−u²)) du を閉じた式で。

use std::f64::consts::PI;

use zenith_algo::{BlendKind, BooleanEngine, BooleanOpType, BrepTransform, EdgeBlender, FaceMerger, MassCalculator, PrimitiveBuilder};
use zenith_math::{Tolerance, Vec3};
use zenith_tess::TessellationParams;
use zenith_topo::Solid;

fn boss() -> Solid {
    let tol = Tolerance::default();
    let base = PrimitiveBuilder::make_box(20.0, 30.0, 40.0).unwrap();
    let c = BrepTransform::translate_solid(&PrimitiveBuilder::make_cylinder(6.0, 40.0).unwrap(), Vec3::new(20.0, 15.0, 0.0));
    let r = BooleanEngine::boolean_solids_exact_result(&base, &c, BooleanOpType::Union, &tol).unwrap().solids.remove(0);
    FaceMerger::simplify_solid(&r, &tol).unwrap().0
}

/// 側面 x=20 の上下の稜（端が円柱に当たる 4 本）
fn side_edges(solid: &Solid) -> Vec<u64> {
    let mut out = Vec::new();
    for f in &solid.outer_shell.faces {
        for w in std::iter::once(&f.outer_wire).chain(f.inner_wires.iter()) {
            for o in &w.edges {
                let (p, q) = (o.edge.start_vertex.point, o.edge.end_vertex.point);
                let flat = |z: f64| (z.abs() < 1e-6 || (z - 40.0).abs() < 1e-6);
                if (p.x - 20.0).abs() < 1e-6 && (q.x - 20.0).abs() < 1e-6 && flat(p.z) && (p.z - q.z).abs() < 1e-6
                    && ((p.y - q.y).abs() - 9.0).abs() < 1e-6 && !out.contains(&o.edge.id)
                {
                    out.push(o.edge.id);
                }
            }
        }
    }
    out
}

fn removed(d: f64) -> f64 {
    let f = |u: f64| 0.5 * (u * (36.0 - u * u).sqrt() + 36.0 * (u / 6.0).asin());
    7.5 * d * d - d * f(d) - ((36.0 - d * d).powf(1.5) - 216.0) / 3.0
}

#[test]
fn chamfer_retreats_onto_the_buried_half() {
    std::env::set_var("ZENITH_CYL_BURIED_END", "1");
    let s = boss();
    let edges = side_edges(&s);
    assert_eq!(edges.len(), 4);
    let exact_before = 24000.0 + 720.0 * PI;
    for id in edges {
        let listed = EdgeBlender::blendability(&s, id).unwrap();
        assert_eq!(listed.max_fillet_radius, 0.0, "only a chamfer is built at a cylinder end");
        assert!(EdgeBlender::fillet_edge(&s, id, 1.0).is_err());
        for d in [0.5, 1.0, 2.0] {
            let (after, report) = EdgeBlender::blend_edge(&s, id, BlendKind::Chamfer { distance: d }).unwrap();
            let v = MassCalculator::compute_volume_from_brep(&after, &TessellationParams { u_divisions: 64, v_divisions: 64 });
            let expected = removed(d);
            let target = exact_before - expected;
            eprintln!("edge {id} d={d}: 予測 {} 閉じた式 {expected} 後 {v} 期待 {target} 相対 {:.3e}", report.predicted_removed_volume, (v - target) / expected);
            assert!((report.predicted_removed_volume - expected).abs() <= 1e-9 * expected, "prediction");
            assert!((v - target).abs() <= 1e-7 * expected, "after volume");
        }
    }
}
