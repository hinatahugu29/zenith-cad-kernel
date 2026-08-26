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
    // 抜き勾配ブロックの閉じた式。**以前ここは `volume > 0` だった。**
    // 勾配が指定と違っていても通るので、フィーチャーツリー経由で来たときに
    // 角度が届いているかを見られなかった。
    let t = 5.0_f64.to_radians().tan();
    let expected =
        40.0 * 30.0 * 20.0 + t * 20.0 * 20.0 * (40.0 + 30.0) + (4.0 / 3.0) * t * t * 20.0f64.powi(3);
    let error = (mass.volume - expected).abs() / expected;
    assert!(
        error < 1e-12,
        "DraftBlock through the feature tree is {} not {expected} (relative {error:.3e})",
        mass.volume
    );

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

    // 6. スプリングワッシャー
    let mut tree_sp = FeatureTree::new();
    tree_sp.add_feature("spring_washer", FeatureOp::SpringWasher {
        inner_radius: 4.25,
        outer_radius: 7.4,
        thickness: 2.0,
        free_height: 3.5,
        gap_deg: 20.0,
    });
    let solid_sp = tree_sp.recompute().expect("evaluate spring washer tree");
    assert!(solid_sp.outer_shell.validate_closed(&tol).is_valid());

    // 7. C形止め輪
    let mut tree_rr = FeatureTree::new();
    tree_rr.add_feature("retaining_ring", FeatureOp::RetainingRing {
        inner_radius: 4.8,
        outer_radius: 6.2,
        thickness: 1.0,
        gap_angle_deg: 45.0,
    });
    let solid_rr = tree_rr.recompute().expect("evaluate retaining ring tree");
    assert!(solid_rr.outer_shell.validate_closed(&tol).is_valid());

    // 8. 皿頭六角穴付きボルト
    let mut tree_cs = FeatureTree::new();
    tree_cs.add_feature("countersunk_screw", FeatureOp::CountersunkSocketScrew {
        shank_radius: 4.0,
        shank_length: 20.0,
        head_radius: 8.0,
        head_height: 4.4,
        socket_across_flats: 5.0,
        socket_depth: 2.8,
    });
    let solid_cs = tree_cs.recompute().expect("evaluate countersunk screw tree");
    assert!(solid_cs.outer_shell.validate_closed(&tol).is_valid());

    // 9. 溶接ネック配管フランジ
    let mut tree_fl = FeatureTree::new();
    tree_fl.add_feature("weld_neck_flange", FeatureOp::WeldNeckFlange {
        flange_radius: 25.0,
        flange_thickness: 10.0,
        hub_radius: 15.0,
        hub_height: 15.0,
        pipe_radius: 8.0,
        pcd_radius: 19.0,
        bolt_hole_radius: 3.0,
        num_bolt_holes: 4,
    });
    let solid_flange = tree_fl.recompute().expect("evaluate flange tree");
    assert!(solid_flange.outer_shell.validate_closed(&tol).is_valid());

    // 10. 六角穴付き管用テーパプラグ
    let mut tree_tp = FeatureTree::new();
    tree_tp.add_feature("pipe_plug", FeatureOp::TaperPipePlug {
        small_radius: 6.0,
        large_radius: 6.6,
        height: 10.0,
        socket_across_flats: 6.0,
        socket_depth: 5.0,
    });
    let solid_plug = tree_tp.recompute().expect("evaluate pipe plug tree");
    assert!(solid_plug.outer_shell.validate_closed(&tol).is_valid());

    // 11. 六角胴スタッドボルト
    let mut tree_sb = FeatureTree::new();
    tree_sb.add_feature("stud_bolt", FeatureOp::StudBolt {
        bottom_shank_radius: 4.0,
        bottom_shank_length: 15.0,
        hex_across_flats: 13.0,
        hex_height: 6.0,
        top_shank_radius: 4.0,
        top_shank_length: 20.0,
    });
    let solid_sb = tree_sb.recompute().expect("evaluate stud bolt tree");
    assert!(solid_sb.outer_shell.validate_closed(&tol).is_valid());

    // 12. 皿ばね
    let mut tree_bs = FeatureTree::new();
    tree_bs.add_feature("belleville_spring", FeatureOp::BellevilleSpring {
        inner_radius: 8.2,
        outer_radius: 16.0,
        thickness: 0.9,
        cone_height: 1.25,
    });
    let solid_bs = tree_bs.recompute().expect("evaluate belleville spring tree");
    assert!(solid_bs.outer_shell.validate_closed(&tol).is_valid());

    // 13. JSON シリアライズ / デシリアライズ検証
    let json = serde_json::to_string(&tree_bs).expect("serialize tree");
    let deserialized_tree: FeatureTree = serde_json::from_str(&json).expect("deserialize tree");
    let solid_recomputed = deserialized_tree.recompute().expect("recompute from json tree");
    assert!(solid_recomputed.outer_shell.validate_closed(&tol).is_valid());
}
