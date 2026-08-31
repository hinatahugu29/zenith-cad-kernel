//! 履歴ツリーが本当にパラメトリックか。
//!
//! これまでの `recompute` は、ほとんどの操作が前段の結果を**捨てて**
//! 作り直していました。前段を受け取るのは Push-Pull と厚み付けだけで、
//! ブーリアン演算子はツリーに存在しませんでした。つまり
//! 「作る → 穴をあける → 角を丸める」という形は履歴で表せず、
//! 上流の寸法を変えて下流を追従させることもできませんでした。
//!
//! ここで測るのは
//!
//! 1. ブーリアンを含む列が一続きに評価されること
//! 2. 上流の寸法だけを差し替えて再計算すると、下流のフィレットが
//!    **同じ稜に**付き直すこと
//! 3. 選び直せないときは、別の稜を黙って丸めずに失敗すること

use std::f64::consts::FRAC_PI_4;

use zenith_algo::{
    edge_signature, BooleanKind, EdgeBlender, FeatureOp, FeatureTree, MassCalculator,
    PrimitiveBuilder,
};
use zenith_math::Point3;
use zenith_tess::TessellationParams;
use zenith_topo::{EdgeSignature, Solid};

fn volume_of(solid: &Solid) -> f64 {
    MassCalculator::compute_from_brep(
        solid,
        &TessellationParams {
            u_divisions: 48,
            v_divisions: 48,
        },
    )
    .volume
}

/// 直方体の (x=dx, y=0) にある縦稜のシグネチャ
fn upright_signature(dx: f64, dy: f64, dz: f64) -> EdgeSignature {
    let boxed = PrimitiveBuilder::make_box(dx, dy, dz).unwrap();
    let target = Point3::new(dx, 0.0, dz * 0.5);

    let mut best: Option<(f64, u64)> = None;
    for edge in EdgeBlender::blendable_edges(&boxed) {
        for face in &boxed.outer_shell.faces {
            for wire in std::iter::once(&face.outer_wire).chain(face.inner_wires.iter()) {
                for oriented in &wire.edges {
                    if oriented.edge.id != edge.edge_id {
                        continue;
                    }
                    let mid = Point3::from(
                        (oriented.edge.start_vertex.point.coords
                            + oriented.edge.end_vertex.point.coords)
                            * 0.5,
                    );
                    let distance = (mid - target).norm();
                    if best.map(|(d, _)| distance < d).unwrap_or(true) {
                        best = Some((distance, edge.edge_id));
                    }
                }
            }
        }
    }
    edge_signature(&boxed, best.expect("an upright edge").1).expect("signature")
}

#[test]
fn a_history_with_a_boolean_in_it_evaluates_as_one_chain() {
    let mut tree = FeatureTree::new();
    tree.add_feature(
        "block",
        FeatureOp::CreateBox {
            dx: 40.0,
            dy: 40.0,
            dz: 20.0,
        },
    );
    tree.add_feature(
        "bore",
        FeatureOp::Boolean {
            op: BooleanKind::Difference,
            tool: vec![
                FeatureOp::CreateCylinder {
                    radius: 6.0,
                    height: 20.0,
                },
                FeatureOp::Translate {
                    offset: [20.0, 20.0, 0.0],
                },
            ],
        },
    );

    let solid = tree.recompute().expect("a boolean in the history");
    let expected = 40.0 * 40.0 * 20.0 - std::f64::consts::PI * 36.0 * 20.0;
    let volume = volume_of(&solid);
    assert!(
        (volume - expected).abs() / expected < 1e-9,
        "the bored block measures {volume} against {expected}"
    );
}

#[test]
fn changing_an_upstream_dimension_moves_the_fillet_to_the_same_edge() {
    let mut tree = FeatureTree::new();
    let block = tree.add_feature(
        "block",
        FeatureOp::CreateBox {
            dx: 20.0,
            dy: 30.0,
            dz: 40.0,
        },
    );
    tree.add_feature(
        "round the corner",
        FeatureOp::FilletSolidEdge {
            target: upright_signature(20.0, 30.0, 40.0),
            radius: 3.0,
        },
    );

    let first = tree.recompute().expect("first build");
    let expected = (20.0 * 30.0 - (1.0 - FRAC_PI_4) * 9.0) * 40.0;
    assert!(
        (volume_of(&first) - expected).abs() / expected < 1e-11,
        "{} against {expected}",
        volume_of(&first)
    );
    assert_eq!(first.outer_shell.faces.len(), 7);

    // 上流の寸法だけを差し替える。下流のフィレットは同じ稜に付き直すはず。
    tree.update_feature_op(
        &block,
        FeatureOp::CreateBox {
            dx: 26.0,
            dy: 30.0,
            dz: 40.0,
        },
    )
    .expect("update the block");

    let second = tree.recompute().expect("rebuild after the change");
    assert_eq!(
        second.outer_shell.faces.len(),
        7,
        "the fillet should still be one extra face, not a different feature"
    );
    let expected = (26.0 * 30.0 - (1.0 - FRAC_PI_4) * 9.0) * 40.0;
    assert!(
        (volume_of(&second) - expected).abs() / expected < 1e-11,
        "after widening: {} against {expected}",
        volume_of(&second)
    );
}

#[test]
fn a_fillet_whose_edge_is_gone_fails_rather_than_rounding_a_different_one() {
    let mut tree = FeatureTree::new();
    tree.add_feature(
        "cylinder",
        FeatureOp::CreateCylinder {
            radius: 10.0,
            height: 20.0,
        },
    );
    // 直方体の縦稜として取ったシグネチャは、円柱には当たらない。
    tree.add_feature(
        "round a corner that is not there",
        FeatureOp::FilletSolidEdge {
            target: upright_signature(20.0, 30.0, 40.0),
            radius: 3.0,
        },
    );

    let error = tree
        .recompute()
        .expect_err("there is no such edge on a cylinder");
    assert!(
        error.contains("No edge matches"),
        "the refusal should say why: {error}"
    );
}

#[test]
fn a_chamfer_can_follow_a_boolean_in_the_same_history() {
    let mut tree = FeatureTree::new();
    tree.add_feature(
        "block",
        FeatureOp::CreateBox {
            dx: 40.0,
            dy: 40.0,
            dz: 20.0,
        },
    );
    tree.add_feature(
        "notch",
        FeatureOp::Boolean {
            op: BooleanKind::Difference,
            tool: vec![
                FeatureOp::CreateBox {
                    dx: 20.0,
                    dy: 20.0,
                    dz: 20.0,
                },
                FeatureOp::Translate {
                    offset: [20.0, 20.0, 0.0],
                },
            ],
        },
    );

    let notched = tree.recompute().expect("the notched block");
    let start = volume_of(&notched);

    // 切り欠きで初めて生まれた凸の縦稜 (40, 20)
    let target = Point3::new(40.0, 20.0, 10.0);
    let mut chosen: Option<u64> = None;
    let mut best = f64::INFINITY;
    for edge in EdgeBlender::blendable_edges(&notched) {
        for face in &notched.outer_shell.faces {
            for wire in std::iter::once(&face.outer_wire).chain(face.inner_wires.iter()) {
                for oriented in &wire.edges {
                    if oriented.edge.id != edge.edge_id {
                        continue;
                    }
                    let mid = Point3::from(
                        (oriented.edge.start_vertex.point.coords
                            + oriented.edge.end_vertex.point.coords)
                            * 0.5,
                    );
                    let distance = (mid - target).norm();
                    if distance < best {
                        best = distance;
                        chosen = Some(edge.edge_id);
                    }
                }
            }
        }
    }
    let chosen = chosen.expect("an upright next to the notch");
    let signature = edge_signature(&notched, chosen).expect("signature");

    tree.add_feature(
        "break the corner",
        FeatureOp::ChamferSolidEdge {
            target: signature,
            distance: 2.0,
        },
    );

    let chamfered = tree.recompute().expect("chamfer after a boolean");
    let expected = start - 20.0 * 2.0 * 2.0 * 0.5;
    let volume = volume_of(&chamfered);
    assert!(
        (volume - expected).abs() / expected < 1e-10,
        "chamfer after a boolean: {volume} against {expected}"
    );
}
