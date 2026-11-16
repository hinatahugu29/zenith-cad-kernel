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

fn simplified_difference(block: &Solid, tool: &Solid) -> Solid {
    let tol = Tolerance::default();
    let raw = BooleanEngine::boolean_solids_exact_result(block, tool, BooleanOpType::Difference, &tol)
        .unwrap()
        .solids
        .remove(0);
    FaceMerger::simplify_solid(&raw, &tol).unwrap().0
}

fn edge_where(solid: &Solid, keep: impl Fn(zenith_math::Point3, zenith_math::Point3) -> bool) -> u64 {
    solid
        .outer_shell
        .faces
        .iter()
        .flat_map(|face| std::iter::once(&face.outer_wire).chain(face.inner_wires.iter()))
        .flat_map(|wire| wire.edges.iter())
        .find(|o| keep(o.edge.start_vertex.point, o.edge.end_vertex.point))
        .unwrap()
        .edge
        .id
}

/// ポケットの口の稜を丸めると、帯は上面の外側ワイヤのほうへ広がる。4-954 は
/// 稜の乗るワイヤを飛ばして穴だけを測り、外側ワイヤとの隔たりを見ていなかった
/// （4-956。`blendability` の上限 7.992 の 0.95 倍で、上面の穴のワイヤが外へはみ出した）。
#[test]
fn a_pocket_mouth_blend_may_not_reach_the_outer_boundary_of_the_face() {
    let block = PrimitiveBuilder::make_box(20.0, 30.0, 40.0).unwrap();
    let pocket = BrepTransform::translate_solid(&PrimitiveBuilder::make_box(10.0, 14.0, 20.0).unwrap(), Vec3::new(5.0, 8.0, 32.0));
    let solid = simplified_difference(&block, &pocket);
    // 口の稜 x=5, z=40（y に沿う）。外側の縁 x=0 まで 5。
    let edge = edge_where(&solid, |p, q| {
        (p.x - 5.0).abs() < 1e-9 && (q.x - 5.0).abs() < 1e-9 && (p.z - 40.0).abs() < 1e-9 && (q.z - 40.0).abs() < 1e-9
    });
    let (result, report) = EdgeBlender::blend_edge(&solid, edge, BlendKind::Fillet { radius: 4.0 }).unwrap();
    let removed = volume(&solid) - volume(&result);
    assert!((removed - report.predicted_removed_volume).abs() < 1e-8 * removed.abs(), "{removed} vs {}", report.predicted_removed_volume);
    for size in [5.0, 6.0, 7.5] {
        // 4-958 からは、口の端（上面の凹の角）で延ばす壁の上限が先に効くこともある。
        let reason = |err: &str| err.contains("hole") || err.contains("neighbouring edge");
        let err = EdgeBlender::fillet_edge(&solid, edge, size).expect_err("reaches the outer boundary");
        assert!(reason(&err), "r{size}: {err}");
        let err = EdgeBlender::chamfer_edge(&solid, edge, size).expect_err("reaches the outer boundary");
        assert!(reason(&err), "c{size}: {err}");
    }
    assert!(EdgeBlender::blendability(&solid, edge).unwrap().max_chamfer_distance < 5.0);
}

/// 縦の稜を面取りすると、上下の端面から三角形が削られる。端面の穴にその三角形が
/// 掛かるのに立体を返していた（4-956）。穴は中心 (10,15)・r5 なので、角 (0,0) の
/// 面取り x+y=s が穴に触れるのは s ≥ 25 − 5√2 ≈ 17.93。
#[test]
fn a_vertical_chamfer_may_not_cut_into_a_hole_in_the_end_faces() {
    let block = PrimitiveBuilder::make_box(20.0, 30.0, 40.0).unwrap();
    let drill = BrepTransform::translate_solid(&PrimitiveBuilder::make_cylinder(5.0, 60.0).unwrap(), Vec3::new(10.0, 15.0, -10.0));
    let solid = simplified_difference(&block, &drill);
    let edge = edge_where(&solid, |p, q| p.x.abs() < 1e-9 && p.y.abs() < 1e-9 && q.x.abs() < 1e-9 && q.y.abs() < 1e-9);
    let (result, report) = EdgeBlender::blend_edge(&solid, edge, BlendKind::Chamfer { distance: 17.0 }).unwrap();
    let removed = volume(&solid) - volume(&result);
    assert!((removed - report.predicted_removed_volume).abs() < 1e-8 * removed, "{removed} vs {}", report.predicted_removed_volume);
    let threshold = 25.0 - 5.0 * 2f64.sqrt();
    for size in [threshold, 18.5, 19.0] {
        let err = EdgeBlender::chamfer_edge(&solid, edge, size).expect_err("cuts into the hole");
        assert!(err.contains("hole"), "c{size}: {err}");
    }
    let limit = EdgeBlender::blendability(&solid, edge).unwrap().max_chamfer_distance;
    assert!(limit < threshold && limit > 17.0, "{limit}");
}
