//! 同じ平面で割れた面を持つ立体の稜を、面をまとめてからもう一度試す（HANDOVER 4-977、
//! `ZENITH_BLEND_MERGE_RETRY=1`）。
//!
//! `make_drilled_box` は継ぎ目で割れた面のまま返すので、箱の稜も穴の縁も直接は断られる。
//! まとめ直しの後で全部の稜が通り、削れた体積が予測と合うことを見る。

use zenith_algo::{BlendKind, EdgeBlender, HoleBuilder, MassCalculator};
use zenith_tess::TessellationParams;
use zenith_topo::Solid;

fn volume(solid: &Solid) -> f64 {
    MassCalculator::compute_volume_from_brep(solid, &TessellationParams { u_divisions: 64, v_divisions: 64 })
}

#[test]
fn drilled_box_edges_pass_after_merge() {
    std::env::set_var("ZENITH_BLEND_MERGE_RETRY", "1");
    let solid = HoleBuilder::make_drilled_box(20.0, 30.0, 40.0, 5.0).unwrap();
    let before = volume(&solid);
    let mut ids: Vec<u64> = Vec::new();
    for f in &solid.outer_shell.faces {
        for w in std::iter::once(&f.outer_wire).chain(f.inner_wires.iter()) {
            for o in &w.edges {
                if !ids.contains(&o.edge.id) {
                    ids.push(o.edge.id);
                }
            }
        }
    }
    let mut passed = 0;
    for id in ids {
        let Ok(edge) = EdgeBlender::blendability(&solid, id) else { continue };
        assert_eq!(edge.edge_id, id);
        for kind in [BlendKind::Fillet { radius: 1.5 }, BlendKind::Chamfer { distance: 1.5 }] {
            let (after, report) = EdgeBlender::blend_edge(&solid, id, kind).unwrap();
            let removed = before - volume(&after);
            let rel = (removed - report.predicted_removed_volume).abs() / report.predicted_removed_volume.abs();
            assert!(rel <= 1e-6, "edge {id} {kind:?}: 削れた {removed} 予測 {} 相対 {rel:.3e}", report.predicted_removed_volume);
        }
        passed += 1;
    }
    // 箱の 12 本と穴の上下の縁 8 本
    assert_eq!(passed, 20);
}
