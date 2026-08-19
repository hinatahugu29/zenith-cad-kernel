use zenith_algo::{FeatureOp, FeatureTree, MassCalculator};
use zenith_math::Tolerance;
use zenith_tess::{tessellate_solid, TessellationParams};

#[test]
fn test_feature_tree_parametric_recompute_extended() {
    let tol = Tolerance::default();
    let mut tree = FeatureTree::new();

    // 1. 中空角パイプ押し出しフィーチャーの追加
    let outer_pts = vec![
        [-15.0, -10.0, 0.0],
        [15.0, -10.0, 0.0],
        [15.0, 10.0, 0.0],
        [-15.0, 10.0, 0.0],
    ];
    let inner_pts = vec![vec![
        [-8.0, -5.0, 0.0],
        [8.0, -5.0, 0.0],
        [8.0, 5.0, 0.0],
        [-8.0, 5.0, 0.0],
    ]];
    let dir = [0.0, 0.0, 25.0];

    let feat_id = tree.add_feature(
        "HollowTube",
        FeatureOp::ExtrudeHollow {
            outer_points: outer_pts.clone(),
            inner_points: inner_pts.clone(),
            dir,
        },
    );

    // 初回再計算
    let solid1 = tree.recompute().expect("initial recompute");
    assert!(solid1.outer_shell.validate_closed(&tol).is_valid());

    let mesh1 = tessellate_solid(&solid1, &TessellationParams::default());
    let mass1 = MassCalculator::compute_from_mesh(&mesh1);
    assert!((mass1.volume - 11000.0).abs() < 1.0);

    // 2. パラメータ変更: 高さを 25 -> 40 に更新
    let new_dir = [0.0, 0.0, 40.0];
    tree.update_feature_op(
        &feat_id,
        FeatureOp::ExtrudeHollow {
            outer_points: outer_pts,
            inner_points: inner_pts,
            dir: new_dir,
        },
    )
    .expect("update feature op");

    // パラメトリック自動再計算
    let solid2 = tree.recompute().expect("parametric recompute after update");
    assert!(solid2.outer_shell.validate_closed(&tol).is_valid());

    let mesh2 = tessellate_solid(&solid2, &TessellationParams::default());
    let mass2 = MassCalculator::compute_from_mesh(&mesh2);
    let expected_new_vol = (30.0 * 20.0 - 16.0 * 10.0) * 40.0; // 440 * 40 = 17600
    assert!(
        (mass2.volume - expected_new_vol).abs() < 1.0,
        "Updated volume error: got {}, expected {}",
        mass2.volume,
        expected_new_vol
    );
}

#[test]
fn test_feature_tree_chamfer_single_edge() {
    let tol = Tolerance::default();
    let mut tree = FeatureTree::new();

    tree.add_feature(
        "CornerChamfer",
        FeatureOp::ChamferEdge {
            dx: 30.0,
            dy: 20.0,
            dz: 15.0,
            edge_index: 0,
            distance: 4.0,
        },
    );

    let solid = tree.recompute().expect("chamfer recompute");
    assert!(solid.outer_shell.validate_closed(&tol).is_valid());

    let mesh = tessellate_solid(&solid, &TessellationParams::default());
    let mass = MassCalculator::compute_from_mesh(&mesh);
    let expected = 30.0 * 20.0 * 15.0 - 0.5 * 4.0 * 4.0 * 15.0; // 8880.0
    assert!((mass.volume - expected).abs() < 1.0);
}
