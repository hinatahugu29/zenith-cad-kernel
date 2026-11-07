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
