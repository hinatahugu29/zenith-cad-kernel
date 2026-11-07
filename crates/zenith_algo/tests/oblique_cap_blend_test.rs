//! 斜めの平面で終わる稜のブレンド（ROADMAP 柱⑦、HANDOVER 4-936、`ZENITH_OBLIQUE_CAP`）。
//!
//! 縦稜を面取りした直方体の上の稜は、端が面取り面（稜と45度）にぶつかる。
//! 丸めた面は面取り面で斜めに切られ、端の曲線は楕円の一部になる。
//! 削れる体積は「断面積 × 断面の重心を通る線の長さ」。90度のフィレットの
//! 断面の重心は、角から各面に沿って `(10 - 3π) r / (3 (4 - π))`。

use std::f64::consts::PI;

use zenith_algo::{BlendKind, EdgeBlender, MassCalculator, PrimitiveBuilder};
use zenith_math::Tolerance;
use zenith_tess::{tessellate_solid, TessellationParams};
use zenith_topo::Solid;

fn volume(solid: &Solid) -> f64 {
    MassCalculator::compute_volume_from_brep(
        solid,
        &TessellationParams {
            u_divisions: 64,
            v_divisions: 64,
        },
    )
}

fn find_edge(solid: &Solid, test: impl Fn([f64; 6]) -> bool) -> u64 {
    solid
        .outer_shell
        .faces
        .iter()
        .flat_map(|face| face.outer_wire.edges.iter())
        .find(|oriented| {
            let a = oriented.edge.start_vertex.point;
            let b = oriented.edge.end_vertex.point;
            test([a.x, a.y, a.z, b.x, b.y, b.z]) || test([b.x, b.y, b.z, a.x, a.y, a.z])
        })
        .map(|oriented| oriented.edge.id)
        .expect("edge")
}

/// 20×30×40 の直方体の、原点の縦稜を距離 3 で面取り（面取り面は x + y = 3）。
fn chamfered_box() -> Solid {
    let solid = PrimitiveBuilder::make_box(20.0, 30.0, 40.0).expect("box");
    let near = |v: f64, w: f64| (v - w).abs() < 1e-9;
    let vertical = find_edge(&solid, |p| near(p[0], 0.0) && near(p[1], 0.0) && near(p[3], 0.0) && near(p[4], 0.0));
    EdgeBlender::chamfer_edge(&solid, vertical, 3.0).expect("chamfer")
}

/// 上の稜（y = 0, z = 40）で、x = 3 から 20 まで。
fn top_front_edge(solid: &Solid) -> u64 {
    let near = |v: f64, w: f64| (v - w).abs() < 1e-9;
    find_edge(solid, |p| near(p[1], 0.0) && near(p[2], 40.0) && near(p[4], 0.0) && near(p[5], 40.0) && near(p[0], 3.0) && near(p[3], 20.0))
}

fn assert_closed(solid: &Solid, what: &str) {
    assert!(solid.outer_shell.validate_closed(&Tolerance::default()).is_valid(), "{what}: open B-Rep");
    for divisions in [8, 16, 32] {
        let mesh = tessellate_solid(solid, &TessellationParams { u_divisions: divisions, v_divisions: divisions });
        let mut uses: std::collections::HashMap<(u32, u32), usize> = Default::default();
        for t in &mesh.indices {
            for k in 0..3 {
                let (a, b) = (t[k], t[(k + 1) % 3]);
                if a != b {
                    *uses.entry((a.min(b), a.max(b))).or_insert(0) += 1;
                }
            }
        }
        assert_eq!(uses.values().filter(|c| **c != 2).count(), 0, "{what}: open mesh at {divisions}");
    }
}

#[test]
fn an_edge_ending_on_a_chamfer_face_fillets_with_the_centroid_formula() {
    std::env::set_var("ZENITH_OBLIQUE_CAP", "1");
    let solid = chamfered_box();
    let before = volume(&solid);
    let edge = top_front_edge(&solid);
    for r in [0.5, 2.0] {
        let (result, report) = EdgeBlender::blend_edge(&solid, edge, BlendKind::Fillet { radius: r }).expect("fillet");
        let area = r * r * (1.0 - PI / 4.0);
        let centroid = (10.0 - 3.0 * PI) * r / (3.0 * (4.0 - PI));
        // 重心を通る線は y = centroid で面取り面 x + y = 3 に当たる
        let expected = area * (20.0 - (3.0 - centroid));
        let removed = before - volume(&result);
        assert!((report.predicted_removed_volume - expected).abs() < 1e-9 * expected, "r={r}: report {} vs {expected}", report.predicted_removed_volume);
        assert!((removed - expected).abs() < 1e-8 * before, "r={r}: removed {removed} vs {expected}");
        assert_closed(&result, &format!("fillet r={r}"));
    }
}

#[test]
fn an_edge_ending_on_a_chamfer_face_chamfers_with_the_centroid_formula() {
    std::env::set_var("ZENITH_OBLIQUE_CAP", "1");
    let solid = chamfered_box();
    let before = volume(&solid);
    let edge = top_front_edge(&solid);
    let c = 2.0;
    let (result, _) = EdgeBlender::blend_edge(&solid, edge, BlendKind::Chamfer { distance: c }).expect("chamfer");
    // 三角形の重心は角から各面に沿って c/3
    let expected = 0.5 * c * c * (20.0 - (3.0 - c / 3.0));
    let removed = before - volume(&result);
    assert!((removed - expected).abs() < 1e-8 * before, "removed {removed} vs {expected}");
    assert_closed(&result, "chamfer");
}
