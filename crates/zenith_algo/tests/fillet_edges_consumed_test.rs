//! `fillet_edges` に角の3本をまとめて渡す（HANDOVER 4-947）。
//!
//! 2本目の鎖が3本目を同じ半径で丸めて消すので、3本目は飛ばす。違う半径を頼まれて
//! いたら黙って飛ばさず断る。口 `ZENITH_FILLET_EDGES_SKIP_CONSUMED` はこのテスト
//! バイナリの中だけで立てる。

use zenith_algo::{EdgeBlender, MassCalculator, PrimitiveBuilder};
use zenith_tess::TessellationParams;
use zenith_topo::Solid;

fn volume(solid: &Solid) -> f64 {
    MassCalculator::compute_volume_from_brep(solid, &TessellationParams { u_divisions: 64, v_divisions: 64 })
}

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

#[test]
fn three_corner_edges_at_once_match_two_sequential_fillets() {
    std::env::set_var("ZENITH_FILLET_EDGES_SKIP_CONSUMED", "1");
    let solid = PrimitiveBuilder::make_box(20.0, 30.0, 40.0).expect("box");
    let ids = [corner_edge(&solid, 1), corner_edge(&solid, 0), corner_edge(&solid, 2)];
    for r in [1.0, 2.0, 3.0] {
        let all = EdgeBlender::fillet_edges(&solid, &ids.map(|id| (id, r))).expect("three at once");
        let one = EdgeBlender::fillet_edge(&solid, ids[0], r).unwrap();
        let two = EdgeBlender::fillet_edge(&one, corner_edge(&one, 0), r).unwrap();
        let (a, b) = (volume(&all), volume(&two));
        assert!((a - b).abs() < 1e-12 * b, "r{r}: {a} vs {b}");
        assert!(all.outer_shell.validate_closed(&zenith_math::Tolerance::default()).is_valid());
    }
}

#[test]
fn a_consumed_edge_asked_with_another_radius_is_refused() {
    std::env::set_var("ZENITH_FILLET_EDGES_SKIP_CONSUMED", "1");
    let solid = PrimitiveBuilder::make_box(20.0, 30.0, 40.0).expect("box");
    let ids = [corner_edge(&solid, 1), corner_edge(&solid, 0), corner_edge(&solid, 2)];
    let err = EdgeBlender::fillet_edges(&solid, &[(ids[0], 2.0), (ids[1], 2.0), (ids[2], 1.0)])
        .expect_err("different radius");
    assert!(err.contains("already rounded"), "{err}");
}
