use zenith_algo::{FeatureOp, FeatureTree, MassCalculator};
use zenith_math::Tolerance;
use zenith_tess::TessellationParams;

#[test]
fn test_feature_tree_extended_features() {
    let tol = Tolerance::default();
    let params = TessellationParams {
        u_divisions: 32,
        v_divisions: 32,
    };

    // 1. 金型抜き勾配ブロックのフィーチャーツリー評価
    let mut tree = FeatureTree::new();
    tree.add_feature("draft_block", FeatureOp::DraftBlock {
        dx: 40.0,
        dy: 30.0,
        dz: 20.0,
        draft_angle_deg: 5.0,
    });
    let solid = tree.recompute().expect("evaluate draft block tree");
    assert!(solid.outer_shell.validate_closed(&tol).is_valid());
    let mass = MassCalculator::compute_from_brep(&solid, &params);
    assert!(mass.volume > 0.0);

    // 2. 三角柱ガセットリブ
    let mut tree_rib = FeatureTree::new();
    tree_rib.add_feature("rib", FeatureOp::TriangularRib {
        length: 40.0,
        height: 30.0,
        thickness: 8.0,
    });
    let solid_rib = tree_rib.recompute().expect("evaluate rib tree");
    assert!(solid_rib.outer_shell.validate_closed(&tol).is_valid());
    let mass_rib = MassCalculator::compute_from_brep(&solid_rib, &params);
    let expected_rib_vol = 0.5 * 40.0 * 30.0 * 8.0;
    assert!((mass_rib.volume - expected_rib_vol).abs() < 1e-6);

    // 3. 六角穴付きボルト
    let mut tree_cap = FeatureTree::new();
    tree_cap.add_feature("cap_screw", FeatureOp::SocketHeadCapScrew {
        shank_radius: 4.0,
        shank_length: 25.0,
        head_radius: 6.5,
        head_height: 8.0,
        socket_across_flats: 6.0,
        socket_depth: 4.0,
    });
    let solid_cap = tree_cap.recompute().expect("evaluate cap screw tree");
    assert!(solid_cap.outer_shell.validate_closed(&tol).is_valid());

    // 4. フランジ付き六角ボルト
    let mut tree_fl = FeatureTree::new();
    tree_fl.add_feature("flanged_bolt", FeatureOp::FlangedHexBolt {
        shank_radius: 4.0,
        shank_length: 25.0,
        flange_radius: 8.5,
        flange_height: 2.0,
        hex_across_flats: 12.0,
        hex_head_height: 6.0,
    });
    let solid_fl = tree_fl.recompute().expect("evaluate flanged bolt tree");
    assert!(solid_fl.outer_shell.validate_closed(&tol).is_valid());

    // 5. 座ぐり長穴
    let mut tree_slot = FeatureTree::new();
    tree_slot.add_feature("cb_slot", FeatureOp::CounterboredSlot {
        box_w: 80.0,
        box_d: 60.0,
        box_h: 20.0,
        slot_length: 20.0,
        slot_radius: 5.0,
        cb_length: 20.0,
        cb_radius: 8.0,
        cb_depth: 6.0,
        center_x: 40.0,
        center_y: 30.0,
    });
    let solid_slot = tree_slot.recompute().expect("evaluate slot tree");
    assert!(solid_slot.outer_shell.validate_closed(&tol).is_valid());

    // 6. JSON シリアライズ / デシリアライズ検証
    let json = serde_json::to_string(&tree_slot).expect("serialize tree");
    let deserialized_tree: FeatureTree = serde_json::from_str(&json).expect("deserialize tree");
    let solid_recomputed = deserialized_tree.recompute().expect("recompute from json tree");
    assert!(solid_recomputed.outer_shell.validate_closed(&tol).is_valid());
}
