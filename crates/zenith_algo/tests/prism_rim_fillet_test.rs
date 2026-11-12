//! 角丸の角柱の上縁を全周で丸める（ROADMAP 柱⑦、HANDOVER 4-934）。
//!
//! 縦稜4本を丸めた直方体の上縁は、直線4本と円弧4本が接線で繋がったループ。
//! 直線の稜を1本選ぶと全周へ伝わり、直線は円柱、円弧はトーラスの一部で丸める。
//! 削れる体積はパップス・ギュルダンの閉形式と照合する:
//!
//! - 直線 長さ L: `L r^2 (1 - π/4)`（面取りは `L d^2 / 2`）
//! - 円弧 半径 R・中心角 θ: `θ ((R - r) r^2 (1 - π/4) + r^3 / 6)`
//!   （面取りは `θ (R d^2 / 2 - d^3 / 6)`）
//!
//! この経路は `ZENITH_PRISM_RIM_BLEND`（既定オン）の下にある。このファイルの
//! テストは念のため口を立てて走らせる（同じ値を書くだけなので、並列でも食い違わない）。

use std::f64::consts::{FRAC_PI_2, PI};

use zenith_algo::{BlendKind, EdgeBlender, MassCalculator, PrimitiveBuilder};
use zenith_math::Tolerance;
use zenith_tess::{tessellate_solid, TessellationParams};
use zenith_topo::Solid;

fn enable() {
    std::env::set_var("ZENITH_PRISM_RIM_BLEND", "1");
}

fn volume(solid: &Solid) -> f64 {
    MassCalculator::compute_volume_from_brep(
        solid,
        &TessellationParams {
            u_divisions: 64,
            v_divisions: 64,
        },
    )
}

/// 20×30×40 の直方体の縦稜4本を半径 3 で丸めた立体。
fn rounded_prism() -> Solid {
    let solid = PrimitiveBuilder::make_box(20.0, 30.0, 40.0).expect("box");
    let mut verticals = Vec::new();
    for face in &solid.outer_shell.faces {
        for oriented in &face.outer_wire.edges {
            let a = oriented.edge.start_vertex.point;
            let b = oriented.edge.end_vertex.point;
            if (a.x - b.x).abs() < 1e-9
                && (a.y - b.y).abs() < 1e-9
                && !verticals.contains(&oriented.edge.id)
            {
                verticals.push(oriented.edge.id);
            }
        }
    }
    assert_eq!(verticals.len(), 4);
    let edges: Vec<(u64, f64)> = verticals.into_iter().map(|id| (id, 3.0)).collect();
    EdgeBlender::fillet_edges(&solid, &edges).expect("vertical fillets")
}

/// z = `z` の高さにある直線の稜を1本。
fn straight_rim_edge(solid: &Solid, z: f64) -> u64 {
    solid
        .outer_shell
        .faces
        .iter()
        .flat_map(|face| face.outer_wire.edges.iter())
        .find(|oriented| {
            let a = oriented.edge.start_vertex.point;
            let b = oriented.edge.end_vertex.point;
            (a.z - z).abs() < 1e-9 && (b.z - z).abs() < 1e-9 && ((a.x - b.x).abs() < 1e-9 || (a.y - b.y).abs() < 1e-9)
        })
        .map(|oriented| oriented.edge.id)
        .expect("a straight rim edge")
}

/// 角丸の矩形（20×30、角の半径 3）の上縁を半径 r で丸めたときの削れ量。
fn expected_fillet(r: f64) -> f64 {
    let straight = 2.0 * (20.0 - 6.0) + 2.0 * (30.0 - 6.0);
    straight * r * r * (1.0 - PI / 4.0) + 4.0 * FRAC_PI_2 * ((3.0 - r) * r * r * (1.0 - PI / 4.0) + r.powi(3) / 6.0)
}

fn expected_chamfer(d: f64) -> f64 {
    let straight = 2.0 * (20.0 - 6.0) + 2.0 * (30.0 - 6.0);
    straight * d * d * 0.5 + 4.0 * FRAC_PI_2 * (3.0 * d * d * 0.5 - d.powi(3) / 6.0)
}

fn assert_closed_mesh(solid: &Solid, what: &str) {
    assert!(
        solid.outer_shell.validate_closed(&Tolerance::default()).is_valid(),
        "{what}: B-Rep shell is open"
    );
    for divisions in [8, 16, 32] {
        let mesh = tessellate_solid(
            solid,
            &TessellationParams {
                u_divisions: divisions,
                v_divisions: divisions,
            },
        );
        let mut uses: std::collections::HashMap<(u32, u32), usize> = Default::default();
        for triangle in &mesh.indices {
            for step in 0..3 {
                let (a, b) = (triangle[step], triangle[(step + 1) % 3]);
                if a != b {
                    *uses.entry(if a < b { (a, b) } else { (b, a) }).or_insert(0) += 1;
                }
            }
        }
        let bad = uses.values().filter(|count| **count != 2).count();
        assert_eq!(bad, 0, "{what}: {bad} non-manifold mesh edges at {divisions} divisions");
    }
}

#[test]
fn the_rounded_prism_top_rim_fillets_all_around_with_the_closed_form_volume() {
    enable();
    let prism = rounded_prism();
    let before = volume(&prism);
    let edge = straight_rim_edge(&prism, 40.0);
    for r in [0.5, 2.0, 2.99] {
        let (result, report) =
            EdgeBlender::blend_edge(&prism, edge, BlendKind::Fillet { radius: r }).expect("fillet");
        let removed = before - volume(&result);
        let expected = expected_fillet(r);
        assert!((report.predicted_removed_volume - expected).abs() < 1e-9 * expected);
        assert!(
            (removed - expected).abs() < 1e-9 * before,
            "r={r}: removed {removed}, expected {expected}"
        );
        assert_closed_mesh(&result, &format!("fillet r={r}"));
    }
}

#[test]
fn the_rounded_prism_top_rim_chamfers_all_around_with_the_closed_form_volume() {
    enable();
    let prism = rounded_prism();
    let before = volume(&prism);
    let edge = straight_rim_edge(&prism, 40.0);
    for d in [0.5, 2.0, 2.99] {
        let (result, _) =
            EdgeBlender::blend_edge(&prism, edge, BlendKind::Chamfer { distance: d }).expect("chamfer");
        let removed = before - volume(&result);
        let expected = expected_chamfer(d);
        assert!(
            (removed - expected).abs() < 1e-9 * before,
            "d={d}: removed {removed}, expected {expected}"
        );
        assert_closed_mesh(&result, &format!("chamfer d={d}"));
    }
}

#[test]
fn top_then_bottom_rim_fillets_chain() {
    enable();
    let prism = rounded_prism();
    let before = volume(&prism);
    let top = EdgeBlender::fillet_edge(&prism, straight_rim_edge(&prism, 40.0), 2.0).expect("top");
    let both = EdgeBlender::fillet_edge(&top, straight_rim_edge(&top, 0.0), 1.5).expect("bottom");
    let removed = before - volume(&both);
    let expected = expected_fillet(2.0) + expected_fillet(1.5);
    assert!((removed - expected).abs() < 1e-9 * before, "removed {removed}, expected {expected}");
    assert_closed_mesh(&both, "top then bottom");
}

#[test]
fn a_rim_fillet_at_or_above_the_corner_radius_is_refused() {
    enable();
    let prism = rounded_prism();
    let edge = straight_rim_edge(&prism, 40.0);
    assert!(EdgeBlender::fillet_edge(&prism, edge, 3.0).is_err());
    assert!(EdgeBlender::fillet_edge(&prism, edge, 5.0).is_err());
    // 断っても元の立体は変わらない
    assert!(prism.outer_shell.validate_closed(&Tolerance::default()).is_valid());
}

#[test]
fn a_sharp_cornered_box_rim_is_left_to_the_per_edge_path() {
    enable();
    // 角のある矩形の上縁は対象外。既存の経路が稜を1本だけ丸める（全周へは伝わらない）。
    let solid = PrimitiveBuilder::make_box(20.0, 30.0, 40.0).expect("box");
    let before = volume(&solid);
    let edge = straight_rim_edge(&solid, 40.0);
    let (result, report) =
        EdgeBlender::blend_edge(&solid, edge, BlendKind::Fillet { radius: 2.0 }).expect("fillet");
    let removed = before - volume(&result);
    assert!((removed - report.predicted_removed_volume).abs() < 1e-9 * before);
    assert!(report.edge_length == 20.0 || report.edge_length == 30.0);
}

/// 角丸の角柱の中心に半径 `hole` の通し穴を開けた立体。
fn holed_prism(hole: f64) -> Solid {
    let drill = zenith_algo::BrepTransform::translate_solid(
        &PrimitiveBuilder::make_cylinder(hole, 60.0).expect("cylinder"),
        zenith_math::Vec3::new(10.0, 15.0, -10.0),
    );
    zenith_algo::BooleanEngine::boolean_solids_exact_result(
        &rounded_prism(),
        &drill,
        zenith_algo::BooleanOpType::Difference,
        &Tolerance::default(),
    )
    .expect("drill")
    .solids[0]
        .clone()
}

#[test]
fn a_holed_top_face_keeps_its_hole_and_fillets_all_around() {
    enable();
    let prism = holed_prism(4.0);
    let before = volume(&prism);
    let edge = straight_rim_edge(&prism, 40.0);
    let (result, _) =
        EdgeBlender::blend_edge(&prism, edge, BlendKind::Fillet { radius: 2.0 }).expect("fillet");
    let removed = before - volume(&result);
    let expected = expected_fillet(2.0);
    assert!((removed - expected).abs() < 1e-9 * before, "removed {removed}, expected {expected}");
    assert_closed_mesh(&result, "holed fillet");
}

#[test]
fn a_hole_close_to_the_rim_caps_the_radius() {
    enable();
    // 半径 8 の穴は x の側面まで 2。半径 2 以上は帯が穴に掛かるので断る。
    let prism = holed_prism(8.0);
    let before = volume(&prism);
    let edge = straight_rim_edge(&prism, 40.0);
    assert!(EdgeBlender::fillet_edge(&prism, edge, 2.5).is_err());
    let (result, _) =
        EdgeBlender::blend_edge(&prism, edge, BlendKind::Chamfer { distance: 1.5 }).expect("chamfer");
    let removed = before - volume(&result);
    let expected = expected_chamfer(1.5);
    assert!((removed - expected).abs() < 1e-9 * before, "removed {removed}, expected {expected}");
    assert_closed_mesh(&result, "holed chamfer");
}

/// 縦稜を1本だけ半径 3 で丸めた直方体。上縁は「直線・円弧・直線」の開いた鎖で、
/// 両端は角（蓋は鎖の端と直交する側面）。
fn one_corner_rounded() -> Solid {
    let solid = PrimitiveBuilder::make_box(20.0, 30.0, 40.0).expect("box");
    let vertical = solid
        .outer_shell
        .faces
        .iter()
        .flat_map(|face| face.outer_wire.edges.iter())
        .find(|oriented| {
            let a = oriented.edge.start_vertex.point;
            let b = oriented.edge.end_vertex.point;
            (a.x - b.x).abs() < 1e-9 && (a.y - b.y).abs() < 1e-9
        })
        .map(|oriented| oriented.edge.id)
        .expect("vertical");
    EdgeBlender::fillet_edge(&solid, vertical, 3.0).expect("one fillet")
}

#[test]
fn an_open_chain_stopping_at_square_corners_fillets_and_chamfers() {
    enable();
    std::env::set_var("ZENITH_PRISM_OPEN_CHAIN", "1");
    let solid = one_corner_rounded();
    let before = volume(&solid);
    // 円弧の隣の直線（長さ 17 か 27）を選ぶ。鎖は 17 + 円弧 + 27。
    let edge = solid
        .outer_shell
        .faces
        .iter()
        .flat_map(|face| face.outer_wire.edges.iter())
        .find(|oriented| {
            let a = oriented.edge.start_vertex.point;
            let b = oriented.edge.end_vertex.point;
            let length = (b - a).norm();
            (a.z - 40.0).abs() < 1e-9 && (b.z - 40.0).abs() < 1e-9 && ((length - 17.0).abs() < 1e-9 || (length - 27.0).abs() < 1e-9)
        })
        .map(|oriented| oriented.edge.id)
        .expect("chain edge");
    for r in [0.5, 2.0, 2.9] {
        let (result, _) =
            EdgeBlender::blend_edge(&solid, edge, BlendKind::Fillet { radius: r }).expect("fillet");
        let expected = 44.0 * r * r * (1.0 - PI / 4.0)
            + FRAC_PI_2 * ((3.0 - r) * r * r * (1.0 - PI / 4.0) + r.powi(3) / 6.0);
        let removed = before - volume(&result);
        assert!((removed - expected).abs() < 1e-9 * before, "r={r}: removed {removed}, expected {expected}");
        assert_closed_mesh(&result, &format!("open fillet r={r}"));

        let (result, _) =
            EdgeBlender::blend_edge(&solid, edge, BlendKind::Chamfer { distance: r }).expect("chamfer");
        let expected = 44.0 * r * r * 0.5 + FRAC_PI_2 * (3.0 * r * r * 0.5 - r.powi(3) / 6.0);
        let removed = before - volume(&result);
        assert!((removed - expected).abs() < 1e-9 * before, "d={r}: removed {removed}, expected {expected}");
        assert_closed_mesh(&result, &format!("open chamfer d={r}"));
    }
    assert!(EdgeBlender::fillet_edge(&solid, edge, 3.0).is_err());
}

#[test]
fn an_open_chain_whose_two_ends_share_one_cap_face() {
    enable();
    std::env::set_var("ZENITH_PRISM_OPEN_CHAIN", "1");
    // 縦稜を2本（x = 0 側の2本）丸める。上縁は「直線・円弧・直線・円弧・直線」で、
    // 鎖の前後が同じ1本（y = 30 の稜）になり、蓋も同じ1枚。
    let solid = PrimitiveBuilder::make_box(20.0, 30.0, 40.0).expect("box");
    let mut verticals = Vec::new();
    for oriented in solid.outer_shell.faces.iter().flat_map(|face| face.outer_wire.edges.iter()) {
        let a = oriented.edge.start_vertex.point;
        let b = oriented.edge.end_vertex.point;
        if (a.x - b.x).abs() < 1e-9 && (a.y - b.y).abs() < 1e-9 && a.y.abs() < 1e-9 && !verticals.contains(&oriented.edge.id) {
            verticals.push(oriented.edge.id);
        }
    }
    assert_eq!(verticals.len(), 2);
    let solid = EdgeBlender::fillet_edges(&solid, &[(verticals[0], 3.0), (verticals[1], 3.0)]).expect("two fillets");
    let before = volume(&solid);
    let edge = straight_rim_edge(&solid, 40.0);
    let r = 2.0;
    let (result, _) = EdgeBlender::blend_edge(&solid, edge, BlendKind::Fillet { radius: r }).expect("fillet");
    // 鎖: 27 + 円弧 + 14 + 円弧 + 27
    let expected = 68.0 * r * r * (1.0 - PI / 4.0)
        + 2.0 * FRAC_PI_2 * ((3.0 - r) * r * r * (1.0 - PI / 4.0) + r.powi(3) / 6.0);
    let removed = before - volume(&result);
    assert!((removed - expected).abs() < 1e-9 * before, "removed {removed}, expected {expected}");
    assert_closed_mesh(&result, "shared cap");
}

/// 縦稜1本（r3）と、その上の開いた鎖（r2）を丸めた直方体（4-941）。
/// 鎖の端の側壁では「縦稜・端の円弧・上の稜」が接線で繋がる開いた鎖になり、
/// 上の稜の側面は大きな上面（4本の帯ではない一般の平面）。
fn vertical_then_top_chain() -> Solid {
    let solid = PrimitiveBuilder::make_box(20.0, 30.0, 40.0).expect("box");
    let vertical = solid
        .outer_shell
        .faces
        .iter()
        .flat_map(|face| face.outer_wire.edges.iter())
        .find(|oriented| {
            let a = oriented.edge.start_vertex.point;
            let b = oriented.edge.end_vertex.point;
            (a.x - 20.0).abs() < 1e-9 && (b.x - 20.0).abs() < 1e-9 && a.y.abs() < 1e-9 && b.y.abs() < 1e-9
        })
        .map(|oriented| oriented.edge.id)
        .expect("vertical edge at (20, 0)");
    let rounded = EdgeBlender::fillet_edge(&solid, vertical, 3.0).expect("vertical fillet");
    let arc = rounded
        .outer_shell
        .faces
        .iter()
        .flat_map(|face| face.outer_wire.edges.iter())
        .find(|oriented| {
            let a = oriented.edge.start_vertex.point;
            let b = oriented.edge.end_vertex.point;
            (a.z - 40.0).abs() < 1e-9 && (b.z - 40.0).abs() < 1e-9 && (b - a).norm() < 5.0
        })
        .map(|oriented| oriented.edge.id)
        .expect("top arc");
    EdgeBlender::fillet_edge(&rounded, arc, 2.0).expect("top chain fillet")
}

#[test]
fn chains_on_the_side_walls_at_the_ends_of_a_top_chain_blend() {
    enable();
    std::env::set_var("ZENITH_PRISM_OPEN_CHAIN", "1");
    std::env::set_var("ZENITH_PRISM_GENERAL_SIDE", "1");
    let solid = vertical_then_top_chain();
    let before = volume(&solid);
    let mut ids: Vec<u64> = Vec::new();
    for face in &solid.outer_shell.faces {
        for oriented in &face.outer_wire.edges {
            if !ids.contains(&oriented.edge.id) {
                ids.push(oriented.edge.id);
            }
        }
    }
    let mut passed = 0;
    for id in ids {
        for kind in [BlendKind::Fillet { radius: 1.5 }, BlendKind::Chamfer { distance: 1.5 }] {
            let Ok((result, report)) = EdgeBlender::blend_edge(&solid, id, kind) else {
                continue;
            };
            let removed = before - volume(&result);
            let predicted = report.predicted_removed_volume;
            assert!(
                (removed - predicted).abs() <= 1e-6 * predicted.abs(),
                "edge {id} {kind:?}: removed {removed}, predicted {predicted}"
            );
            assert_closed_mesh(&result, &format!("edge {id} {kind:?}"));
            passed += 1;
        }
    }
    // 角のある稜 12 本（接していて角の無い 10 本は数えない）を、フィレットと面取りで
    assert_eq!(passed, 24, "every cornered edge blends both ways");
}

/// L字（20×30 から x 10..20, y 15..30 を欠く、高さ 40）の凹の縦稜を r3 で丸めた立体（4-942）。
/// 上下の縁は「直線・凹の円弧・直線」の開いた鎖。
fn l_with_concave_fillet() -> Solid {
    use zenith_algo::ExtrudeBuilder;
    use zenith_math::{Point3, Vec3};
    use zenith_topo::{Edge, OrientedEdge, Vertex, Wire};
    let corners = [(0.0, 0.0), (20.0, 0.0), (20.0, 15.0), (10.0, 15.0), (10.0, 30.0), (0.0, 30.0)];
    let vertices: Vec<Vertex> = corners
        .iter()
        .map(|(x, y)| Vertex::from_point(Point3::new(*x, *y, 0.0)))
        .collect();
    let edges = (0..vertices.len())
        .map(|i| {
            OrientedEdge::forward(
                Edge::line_between(vertices[i].clone(), vertices[(i + 1) % vertices.len()].clone()).unwrap(),
            )
        })
        .collect();
    let solid = ExtrudeBuilder::extrude_wire(&Wire::new(edges), Vec3::new(0.0, 0.0, 40.0), &Tolerance::default())
        .expect("L prism");
    let concave = solid
        .outer_shell
        .faces
        .iter()
        .flat_map(|face| face.outer_wire.edges.iter())
        .find(|oriented| {
            let a = oriented.edge.start_vertex.point;
            let b = oriented.edge.end_vertex.point;
            (a.x - 10.0).abs() < 1e-9 && (a.y - 15.0).abs() < 1e-9 && (b.x - 10.0).abs() < 1e-9 && (b.y - 15.0).abs() < 1e-9
        })
        .map(|oriented| oriented.edge.id)
        .expect("concave vertical");
    EdgeBlender::fillet_edge(&solid, concave, 3.0).expect("concave fillet")
}

#[test]
fn a_rim_chain_through_a_concave_arc_blends_with_the_closed_form_volume() {
    enable();
    std::env::set_var("ZENITH_PRISM_OPEN_CHAIN", "1");
    std::env::set_var("ZENITH_PRISM_GENERAL_SIDE", "1");
    std::env::set_var("ZENITH_PRISM_CONCAVE_ARC", "1");
    let solid = l_with_concave_fillet();
    let before = volume(&solid);
    // 上の縁の直線 (20,15,40)→(13,15,40) を選ぶと、鎖は 直線7・凹の円弧（R3、90度）・直線12
    let rim = solid
        .outer_shell
        .faces
        .iter()
        .flat_map(|face| face.outer_wire.edges.iter())
        .find(|oriented| {
            let a = oriented.edge.start_vertex.point;
            let b = oriented.edge.end_vertex.point;
            (a.z - 40.0).abs() < 1e-9
                && (b.z - 40.0).abs() < 1e-9
                && (a.y - 15.0).abs() < 1e-9
                && (b.y - 15.0).abs() < 1e-9
        })
        .map(|oriented| oriented.edge.id)
        .expect("rim line");
    let r: f64 = 1.5;
    let straight = 7.0 + 12.0;
    let expected = straight * r * r * (1.0 - PI / 4.0)
        + FRAC_PI_2 * ((3.0 + r) * r * r * (1.0 - PI / 4.0) - r.powi(3) / 6.0);
    let (result, report) = EdgeBlender::blend_edge(&solid, rim, BlendKind::Fillet { radius: r }).expect("fillet");
    assert!((report.predicted_removed_volume - expected).abs() < 1e-9 * expected);
    let removed = before - volume(&result);
    assert!((removed - expected).abs() < 1e-9 * before, "removed {removed}, expected {expected}");
    assert_closed_mesh(&result, "concave-arc chain fillet");

    // ほかの角のある稜もすべて、フィレット・面取りとも閉じて体積が合う
    let mut ids: Vec<u64> = Vec::new();
    for face in &solid.outer_shell.faces {
        for oriented in &face.outer_wire.edges {
            if !ids.contains(&oriented.edge.id) {
                ids.push(oriented.edge.id);
            }
        }
    }
    let mut passed = 0;
    for id in ids {
        for kind in [BlendKind::Fillet { radius: r }, BlendKind::Chamfer { distance: r }] {
            let Ok((result, report)) = EdgeBlender::blend_edge(&solid, id, kind) else {
                continue;
            };
            let removed = before - volume(&result);
            let predicted = report.predicted_removed_volume;
            assert!(
                (removed - predicted).abs() <= 1e-6 * predicted.abs(),
                "edge {id} {kind:?}: removed {removed}, predicted {predicted}"
            );
            assert_closed_mesh(&result, &format!("edge {id} {kind:?}"));
            passed += 1;
        }
    }
    assert_eq!(passed, 38, "19 cornered edges, fillet and chamfer");
}

/// 平座金（内径5・外径12・厚さ3）の穴の縁（上面の内側ワイヤの円、4-943）。
#[test]
fn the_hole_rim_of_a_washer_blends_with_the_closed_form_volume() {
    enable();
    std::env::set_var("ZENITH_PRISM_CONCAVE_ARC", "1");
    std::env::set_var("ZENITH_PRISM_HOLE_RIM", "1");
    let solid = zenith_algo::FastenerBuilder::make_plain_washer(5.0, 12.0, 3.0, &Tolerance::default()).expect("washer");
    let before = volume(&solid);
    // 上面（z=3）の内側ワイヤにある稜を1本選ぶ
    let rim = solid
        .outer_shell
        .faces
        .iter()
        .flat_map(|face| face.inner_wires.iter())
        .flat_map(|wire| wire.edges.iter())
        .find(|oriented| (oriented.edge.start_vertex.point.z - 3.0).abs() < 1e-9)
        .map(|oriented| oriented.edge.id)
        .expect("hole rim");
    let (big_r, r, d): (f64, f64, f64) = (5.0, 0.8, 0.8);
    let fillet = 2.0 * PI * ((big_r + r) * r * r * (1.0 - PI / 4.0) - r.powi(3) / 6.0);
    let chamfer = 2.0 * PI * (big_r * d * d * 0.5 + d.powi(3) / 6.0);
    for (kind, expected) in [(BlendKind::Fillet { radius: r }, fillet), (BlendKind::Chamfer { distance: d }, chamfer)] {
        let (result, report) = EdgeBlender::blend_edge(&solid, rim, kind).expect("hole rim blend");
        assert!(
            (report.predicted_removed_volume - expected).abs() < 1e-9 * expected,
            "{kind:?}: predicted {}, expected {expected}",
            report.predicted_removed_volume
        );
        let removed = before - volume(&result);
        assert!((removed - expected).abs() < 1e-6 * expected, "{kind:?}: removed {removed}, expected {expected}");
        assert_closed_mesh(&result, &format!("washer hole rim {kind:?}"));
    }
}

/// 座ぐりの段の根元（段の面と座ぐりの壁の凹の円、R9）。側面が上面から上へ
/// 立つので、凸と同じ断面を同じ場所に足す（4-944、`ZENITH_PRISM_RAISED_SIDE`）。
#[test]
fn the_root_of_a_counterbore_step_blends_with_the_closed_form_volume() {
    enable();
    std::env::set_var("ZENITH_PRISM_CONCAVE_ARC", "1");
    std::env::set_var("ZENITH_PRISM_HOLE_RIM", "1");
    std::env::set_var("ZENITH_PRISM_RAISED_SIDE", "1");
    let solid = zenith_algo::HoleBuilder::make_counterbore_hole_box(40.0, 40.0, 20.0, 5.0, 9.0, 6.0)
        .expect("counterbore");
    let before = volume(&solid);
    // 段の面は z = 14。根元の円は中心 (20,20) から 9
    let root = solid
        .outer_shell
        .faces
        .iter()
        .flat_map(|face| std::iter::once(&face.outer_wire).chain(face.inner_wires.iter()))
        .flat_map(|wire| wire.edges.iter())
        .find(|oriented| {
            let a = oriented.edge.start_vertex.point;
            let b = oriented.edge.end_vertex.point;
            (a.z - 14.0).abs() < 1e-9
                && (b.z - 14.0).abs() < 1e-9
                && (((a.x - 20.0).powi(2) + (a.y - 20.0).powi(2)).sqrt() - 9.0).abs() < 1e-9
        })
        .map(|oriented| oriented.edge.id)
        .expect("step root");
    let (big_r, r): (f64, f64) = (9.0, 1.0);
    let fillet = -2.0 * PI * ((big_r - r) * r * r * (1.0 - PI / 4.0) + r.powi(3) / 6.0);
    let chamfer = -2.0 * PI * (big_r * r * r * 0.5 - r.powi(3) / 6.0);
    for (kind, expected) in [(BlendKind::Fillet { radius: r }, fillet), (BlendKind::Chamfer { distance: r }, chamfer)] {
        let (result, report) = EdgeBlender::blend_edge(&solid, root, kind).expect("step root blend");
        assert!(
            (report.predicted_removed_volume - expected).abs() < 1e-9 * expected.abs(),
            "{kind:?}: predicted {}, expected {expected}",
            report.predicted_removed_volume
        );
        let removed = before - volume(&result);
        assert!(
            (removed - expected).abs() < 1e-6 * expected.abs(),
            "{kind:?}: removed {removed}, expected {expected}"
        );
        assert_closed_mesh(&result, &format!("counterbore step root {kind:?}"));
    }
}

/// 直方体の角に集まる2本を順に丸める（4-945）。2本目の鎖には1本目の端の円弧が入る。
/// 同じ半径だと円弧の内側が潰れるので断る（上限が丸めの誤差で 1 ulp 大きく出ても
/// 通さない）。違う半径なら角の先の稜まで鎖で丸まり、体積は予測どおり。
#[test]
fn a_second_blend_at_a_corner_refuses_the_same_radius_and_takes_a_smaller_one() {
    enable();
    std::env::set_var("ZENITH_PRISM_OPEN_CHAIN", "1");
    std::env::set_var("ZENITH_PRISM_GENERAL_SIDE", "1");
    let solid = PrimitiveBuilder::make_box(20.0, 30.0, 40.0).expect("box");
    let at_origin = |solid: &Solid, along: zenith_math::Vec3| {
        solid
            .outer_shell
            .faces
            .iter()
            .flat_map(|face| face.outer_wire.edges.iter())
            .find(|oriented| {
                let a = oriented.edge.start_vertex.point;
                let b = oriented.edge.end_vertex.point;
                let span = (b - a).normalize();
                (a.coords.norm() < 1e-9 || b.coords.norm() < 1e-9 || a.coords.dot(&along).abs() > 0.0)
                    && span.dot(&along).abs() > 1.0 - 1e-9
                    && a.coords.cross(&along).norm() < 1e-9
                    && b.coords.cross(&along).norm() < 1e-9
            })
            .map(|oriented| oriented.edge.id)
    };
    let along_y = zenith_math::Vec3::new(0.0, 1.0, 0.0);
    let along_x = zenith_math::Vec3::new(1.0, 0.0, 0.0);
    for first in [1.0, 2.0, 3.0] {
        let rounded = EdgeBlender::fillet_edge(&solid, at_origin(&solid, along_y).expect("y edge"), first)
            .expect("first fillet");
        let second = at_origin(&rounded, along_x).expect("x edge");
        let error = EdgeBlender::fillet_edge(&rounded, second, first).expect_err("same radius must be refused");
        assert!(error.contains("must be smaller"), "r{first}: {error}");

        let smaller = first * 0.5;
        let before = volume(&rounded);
        let (result, report) = EdgeBlender::blend_edge(&rounded, second, BlendKind::Fillet { radius: smaller })
            .expect("smaller second fillet");
        let removed = before - volume(&result);
        let predicted = report.predicted_removed_volume;
        assert!(
            (removed - predicted).abs() <= 1e-6 * predicted,
            "r{first} then r{smaller}: removed {removed}, predicted {predicted}"
        );
        assert_closed_mesh(&result, &format!("corner r{first} then r{smaller}"));
    }
}
