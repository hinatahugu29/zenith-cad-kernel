use zenith_algo::{BooleanEngine, BooleanOpType, BrepTransform, DirectModeling, PrimitiveBuilder};
use zenith_math::{Tolerance, Vec3};

#[test]
fn test_direct_modeling_blend_edges_on_boolean_solid() {
    let tol = Tolerance::default();
    let block = PrimitiveBuilder::make_box(40.0, 40.0, 20.0).unwrap();
    let corner = BrepTransform::translate_solid(
        &PrimitiveBuilder::make_box(20.0, 20.0, 20.0).unwrap(),
        Vec3::new(20.0, 20.0, 0.0),
    );
    let l_prism = BooleanEngine::boolean_solids_exact_simplified(
        &block,
        &corner,
        BooleanOpType::Difference,
        &tol,
    )
    .expect("simplified L prism");

    // ブレンド可能な稜を自動検出
    let blendable = DirectModeling::list_blendable_edges(&l_prism);
    assert!(
        !blendable.is_empty(),
        "L-prism should have blendable vertical convex edges"
    );

    // 最初のブレンド可能凸稜にフィレットを適用
    let target_edge = blendable[0];
    let filleted = DirectModeling::fillet_solid_edge(&l_prism, target_edge.edge_id, 3.0)
        .expect("fillet should succeed on L prism edge");

    assert!(filleted.outer_shell.validate_closed(&tol).is_valid());

    // 別のブレンド可能凸稜に面取りを適用
    if blendable.len() > 1 {
        let chamfered = DirectModeling::chamfer_solid_edge(&l_prism, blendable[1].edge_id, 2.0)
            .expect("chamfer should succeed on L prism edge");
        assert!(chamfered.outer_shell.validate_closed(&tol).is_valid());
    }
}
