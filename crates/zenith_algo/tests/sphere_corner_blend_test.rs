//! 直方体の角に同じ半径で集まる稜を丸める（ROADMAP 柱⑦ 手順3、HANDOVER 4-946）。
//!
//! 1本目を半径 r で丸めると、2本目の鎖に半径 r の端の円弧が入る。同じ r で丸めると
//! 内側の円弧が1点（円弧の中心）に潰れ、トーラスの面は球の面になる。削れる体積は
//! 円弧の閉形式 `θ((R - r) r^2 (1 - π/4) + r^3 / 6)` で R = r としたもの。
//! 口 `ZENITH_PRISM_SPHERE_CORNER` はこのテストバイナリの中だけで立てる。

use std::f64::consts::{FRAC_PI_2, PI};

use zenith_algo::{BlendKind, EdgeBlender, MassCalculator, PrimitiveBuilder};
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

/// 原点の角から軸 `axis` の向きに走る稜
fn corner_edge(solid: &Solid, axis: usize) -> u64 {
    solid
        .outer_shell
        .faces
        .iter()
        .flat_map(|face| face.outer_wire.edges.iter())
        .find(|oriented| {
            let a = oriented.edge.start_vertex.point;
            let b = oriented.edge.end_vertex.point;
            (0..3).all(|k| {
                if k == axis {
                    (b[k] - a[k]).abs() > 1e-9
                } else {
                    a[k].abs() < 1e-9 && b[k].abs() < 1e-9
                }
            })
        })
        .map(|oriented| oriented.edge.id)
        .expect("corner edge")
}

fn assert_closed(solid: &Solid, what: &str) {
    assert!(
        solid.outer_shell.validate_closed(&zenith_math::Tolerance::default()).is_valid(),
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
fn three_edges_at_a_box_corner_close_with_a_sphere_octant() {
    std::env::set_var("ZENITH_PRISM_SPHERE_CORNER", "1");
    let solid = PrimitiveBuilder::make_box(20.0, 30.0, 40.0).expect("box");
    for r in [1.0f64, 2.0, 3.0] {
        let one = EdgeBlender::fillet_edge(&solid, corner_edge(&solid, 1), r).expect("first fillet");
        let before = volume(&one);
        // 鎖: x 向きの稜（長さ 20 - r）、1本目の端の円弧（R = r、90度）、z 向きの稜（長さ 40 - r）
        let straight = (20.0 - r) + (40.0 - r);
        let expected = straight * r * r * (1.0 - PI / 4.0) + FRAC_PI_2 * r.powi(3) / 6.0;
        let (result, report) = EdgeBlender::blend_edge(&one, corner_edge(&one, 0), BlendKind::Fillet { radius: r })
            .expect("same-radius corner");
        assert!(
            (report.predicted_removed_volume - expected).abs() < 1e-9 * expected,
            "r{r}: predicted {}, expected {expected}",
            report.predicted_removed_volume
        );
        let removed = before - volume(&result);
        assert!((removed - expected).abs() < 1e-8 * expected, "r{r}: removed {removed}, expected {expected}");
        assert_closed(&result, &format!("corner r{r}"));
    }
}
