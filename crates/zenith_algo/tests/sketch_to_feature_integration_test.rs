use zenith_algo::{
    Constraint, FeatureOp, FeatureTree, MassCalculator, SketchSolver,
};
use zenith_math::Tolerance;
use zenith_tess::TessellationParams;

#[test]
fn test_sketch_solver_to_feature_tree_pipeline() {
    let tol = Tolerance::default();
    let params = TessellationParams {
        u_divisions: 32,
        v_divisions: 32,
    };

    // 1. スケッチ拘束ソルバーで長方形断面を定義・解く
    let mut solver = SketchSolver::new();
    let p0 = solver.add_fixed_point(0.0, 0.0);
    let p1 = solver.add_point(35.0, 2.0);  // 初期値（不正確）
    let p2 = solver.add_point(38.0, 22.0); // 初期値
    let p3 = solver.add_point(2.0, 18.0);  // 初期値

    solver.add_line(p0, p1);
    solver.add_line(p1, p2);
    solver.add_line(p2, p3);
    solver.add_line(p3, p0);

    // 水平・垂直・寸法拘束（幅 40.0, 高さ 25.0 の完全長方形）
    solver.add_constraint(Constraint::Horizontal(p0, p1));
    solver.add_constraint(Constraint::Vertical(p1, p2));
    solver.add_constraint(Constraint::Horizontal(p3, p2));
    solver.add_constraint(Constraint::Vertical(p0, p3));
    solver.add_constraint(Constraint::Distance(p0, p1, 40.0));
    solver.add_constraint(Constraint::Distance(p1, p2, 25.0));

    let iters = solver.solve(50, 1e-7).expect("solve sketch");
    assert!(iters < 50, "Solver should converge within 50 iterations");

    // 2. 解かれた2D点を抽出して3D点列（Z=0平面）を構築
    let pt0 = solver.get_point(p0).unwrap();
    let pt1 = solver.get_point(p1).unwrap();
    let pt2 = solver.get_point(p2).unwrap();
    let pt3 = solver.get_point(p3).unwrap();

    // 座標精度検証
    assert!((pt1[0] - 40.0).abs() < 1e-5);
    assert!((pt1[1] - 0.0).abs() < 1e-5);
    assert!((pt2[0] - 40.0).abs() < 1e-5);
    assert!((pt2[1] - 25.0).abs() < 1e-5);
    assert!((pt3[0] - 0.0).abs() < 1e-5);
    assert!((pt3[1] - 25.0).abs() < 1e-5);

    let sketch_profile_3d = vec![
        [pt0[0], pt0[1], 0.0],
        [pt1[0], pt1[1], 0.0],
        [pt2[0], pt2[1], 0.0],
        [pt3[0], pt3[1], 0.0],
    ];

    // 3. フィーチャーツリーに投入してドラフト付き押し出しソリッド（高さ30, 抜き勾配3度）を構築
    let mut tree = FeatureTree::new();
    tree.add_feature("draft_extrude_from_sketch", FeatureOp::ExtrudeDraft {
        points: sketch_profile_3d.clone(),
        dir: [0.0, 0.0, 30.0],
        draft_angle_rad: 3.0_f64.to_radians(),
    });

    let solid = tree.recompute().expect("evaluate feature tree from sketch");

    // 4. B-Rep 閉多様体検証
    assert!(
        solid.outer_shell.validate_closed(&tol).is_valid(),
        "Solid created from solved sketch must be valid closed manifold"
    );

    // 5. 質量物性値計算
    let mass = MassCalculator::compute_from_brep(&solid, &params);
    assert!(mass.volume > 0.0, "Volume must be positive, got {}", mass.volume);

    // 6. 回転体ソリッド（360度）への投入テスト
    let mut tree_revolve = FeatureTree::new();
    // X軸から離れた位置のプロファイル
    let revolved_profile = vec![
        [10.0, 0.0, 0.0],
        [30.0, 0.0, 0.0],
        [30.0, 0.0, 20.0],
        [10.0, 0.0, 20.0],
    ];
    tree_revolve.add_feature("revolve_from_sketch", FeatureOp::RevolveSolid {
        profile_points: revolved_profile,
        axis_origin: [0.0, 0.0, 0.0],
        axis_dir: [0.0, 0.0, 1.0],
    });
    let solid_revolve = tree_revolve.recompute().expect("evaluate revolve from sketch");
    assert!(solid_revolve.outer_shell.validate_closed(&tol).is_valid());
    let mass_revolve = MassCalculator::compute_from_brep(&solid_revolve, &params);
    let expected_revolve_vol = std::f64::consts::PI * (30.0 * 30.0 - 10.0 * 10.0) * 20.0;
    let diff = (mass_revolve.volume - expected_revolve_vol).abs() / expected_revolve_vol;
    assert!(diff < 1e-4, "Revolve volume error too high: {diff}");
}
