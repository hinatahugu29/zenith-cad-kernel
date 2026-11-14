//! 平面×平面の丸めが、面の穴に食い込まない（HANDOVER 4-954）。
//!
//! x=0 の側面から肉厚 1 の所に r4 の貫通穴を開けた直方体で、上面の x=0 の稜を
//! 丸める。以前は r 1.2 や r 2 でも、穴が無いときの閉形式どおりの体積で
//! 「閉じた」立体を返していた（上面の穴のワイヤが縮めた上面の外にはみ出す誤答）。

use zenith_algo::{
    BlendKind, BooleanEngine, BooleanOpType, BrepTransform, EdgeBlender, FaceMerger, MassCalculator,
    PrimitiveBuilder,
};
use zenith_math::{Tolerance, Vec3};
use zenith_topo::Solid;

fn drilled_near_side() -> (Solid, u64) {
    let tol = Tolerance::default();
    let block = PrimitiveBuilder::make_box(20.0, 30.0, 40.0).unwrap();
    let drill = BrepTransform::translate_solid(&PrimitiveBuilder::make_cylinder(4.0, 60.0).unwrap(), Vec3::new(5.0, 15.0, -10.0));
    let raw = BooleanEngine::boolean_solids_exact_result(&block, &drill, BooleanOpType::Difference, &tol)
        .unwrap()
        .solids
        .remove(0);
    let (solid, _) = FaceMerger::simplify_solid(&raw, &tol).unwrap();
    let edge = solid
        .outer_shell
        .faces
        .iter()
        .flat_map(|face| face.outer_wire.edges.iter())
        .find(|o| {
            let (p, q) = (o.edge.start_vertex.point, o.edge.end_vertex.point);
            p.x.abs() < 1e-9 && q.x.abs() < 1e-9 && (p.z - 40.0).abs() < 1e-9 && (q.z - 40.0).abs() < 1e-9
        })
        .unwrap()
        .edge
        .id;
    (solid, edge)
}

fn volume(solid: &Solid) -> f64 {
    MassCalculator::compute_from_brep(solid, &zenith_tess::TessellationParams { u_divisions: 64, v_divisions: 64 }).volume
}

#[test]
fn a_blend_narrower_than_the_wall_passes_with_the_closed_form_volume() {
    let (solid, edge) = drilled_near_side();
    let (result, report) = EdgeBlender::blend_edge(&solid, edge, BlendKind::Fillet { radius: 0.9 }).unwrap();
    let removed = volume(&solid) - volume(&result);
    assert!((removed - report.predicted_removed_volume).abs() < 1e-8 * removed, "{removed} vs {}", report.predicted_removed_volume);
}

#[test]
fn a_blend_reaching_the_hole_is_refused_including_exact_contact() {
    let (solid, edge) = drilled_near_side();
    for radius in [1.0, 1.2, 2.0] {
        let err = EdgeBlender::fillet_edge(&solid, edge, radius).expect_err("reaches the hole");
        assert!(err.contains("hole"), "r{radius}: {err}");
        let err = EdgeBlender::chamfer_edge(&solid, edge, radius).expect_err("reaches the hole");
        assert!(err.contains("hole"), "c{radius}: {err}");
    }
    let limits = EdgeBlender::blendability(&solid, edge).unwrap();
    assert!(limits.max_chamfer_distance < 1.0, "{}", limits.max_chamfer_distance);
}
