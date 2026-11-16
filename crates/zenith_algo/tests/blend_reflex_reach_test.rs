//! 凹の角で終わる稜の丸めで、延ばす隣の壁が面の外へ出ない（HANDOVER 4-958）。
//!
//! 直方体 20x30x40 の角を、z まわりに 30 度回した箱で切り欠いた。切り欠きの
//! 底の稜 (20,24.309,0)–(16,22,0) は、(16,22) で凹の角に終わる。そこで延ばす
//! 隣の壁は、底面の中を (0.5,−0.866) の向きへ進み、x=20 の縁に長さ 8 で届く。
//! 以前は延ばす量に上限が無く、r 20 で頂点が x≈26（立体の外）に在る形を、
//! 体積が閉じた式どおりの「閉じた」立体として返していた。

use zenith_algo::{
    BlendKind, BooleanEngine, BooleanOpType, BrepTransform, EdgeBlender, FaceMerger, MassCalculator,
    PrimitiveBuilder,
};
use zenith_math::{Tolerance, Transform3, Vec3};
use zenith_topo::Solid;

fn notched() -> (Solid, u64) {
    let tol = Tolerance::default();
    let block = PrimitiveBuilder::make_box(20.0, 30.0, 40.0).unwrap();
    let cutter = PrimitiveBuilder::make_box(20.0, 20.0, 60.0).unwrap();
    let turn = Transform3::from_axis_angle(&Vec3::new(0.0, 0.0, 1.0), 30f64.to_radians());
    let cutter = BrepTransform::translate_solid(&BrepTransform::transform_solid(&cutter, &turn).unwrap(), Vec3::new(16.0, 22.0, -10.0));
    let raw = BooleanEngine::boolean_solids_exact_result(&block, &cutter, BooleanOpType::Difference, &tol)
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
            let at = |p: zenith_math::Point3, x: f64, y: f64| (p.x - x).abs() < 1e-6 && (p.y - y).abs() < 1e-6 && p.z.abs() < 1e-9;
            (at(p, 16.0, 22.0) && (q.x - 20.0).abs() < 1e-9 && q.z.abs() < 1e-9)
                || (at(q, 16.0, 22.0) && (p.x - 20.0).abs() < 1e-9 && p.z.abs() < 1e-9)
        })
        .unwrap()
        .edge
        .id;
    (solid, edge)
}

fn volume(solid: &Solid) -> f64 {
    MassCalculator::compute_from_brep(solid, &zenith_tess::TessellationParams { u_divisions: 64, v_divisions: 64 }).volume
}

fn max_x(solid: &Solid) -> f64 {
    solid
        .outer_shell
        .faces
        .iter()
        .flat_map(|face| face.outer_wire.edges.iter())
        .flat_map(|o| [o.edge.start_vertex.point.x, o.edge.end_vertex.point.x])
        .fold(f64::NEG_INFINITY, f64::max)
}

#[test]
fn a_small_blend_at_the_reflex_corner_passes_inside_the_solid() {
    let (solid, edge) = notched();
    for kind in [BlendKind::Fillet { radius: 3.0 }, BlendKind::Chamfer { distance: 3.0 }] {
        let (result, report) = EdgeBlender::blend_edge(&solid, edge, kind).unwrap();
        let removed = volume(&solid) - volume(&result);
        assert!((removed - report.predicted_removed_volume).abs() < 1e-8 * removed, "{removed} vs {}", report.predicted_removed_volume);
        assert!(max_x(&result) <= 20.0 + 1e-9, "{}", max_x(&result));
    }
}

#[test]
fn the_extended_wall_may_not_leave_the_face() {
    let (solid, edge) = notched();
    for size in [8.5, 12.0, 20.0] {
        assert!(EdgeBlender::fillet_edge(&solid, edge, size).is_err(), "r{size}");
        assert!(EdgeBlender::chamfer_edge(&solid, edge, size).is_err(), "c{size}");
    }
    let limits = EdgeBlender::blendability(&solid, edge).unwrap();
    assert!(limits.max_fillet_radius < 8.0 && limits.max_chamfer_distance < 8.0, "{} {}", limits.max_fillet_radius, limits.max_chamfer_distance);
}
