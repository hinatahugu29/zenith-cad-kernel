use zenith_algo::{Constraint, SketchConstraintStatus, SketchSolver};

#[test]
fn test_sketch_degrees_of_freedom_unconstrained() {
    let mut solver = SketchSolver::new();
    let _p0 = solver.add_point(0.0, 0.0);
    let _p1 = solver.add_point(10.0, 0.0);
    let (total_dof, rank, rem_dof) = solver.degrees_of_freedom();
    assert_eq!(total_dof, 4);
    assert_eq!(rank, 0);
    assert_eq!(rem_dof, 4);
    assert_eq!(
        solver.constraint_status(),
        SketchConstraintStatus::UnderConstrained { remaining_dof: 4 }
    );
}

#[test]
fn test_sketch_fully_constrained_rectangle() {
    let mut solver = SketchSolver::new();
    let p0 = solver.add_fixed_point(0.0, 0.0);
    let p1 = solver.add_point(10.0, 0.5);
    let p2 = solver.add_point(9.5, 5.5);
    let p3 = solver.add_point(0.5, 5.0);

    let l0 = solver.add_line(p0, p1);
    let l1 = solver.add_line(p1, p2);
    let l2 = solver.add_line(p2, p3);
    let l3 = solver.add_line(p3, p0);

    solver.add_constraint(Constraint::Horizontal(p0, p1));
    solver.add_constraint(Constraint::Vertical(p0, p3));
    solver.add_constraint(Constraint::Parallel(l0, l2));
    solver.add_constraint(Constraint::Parallel(l1, l3));
    solver.add_constraint(Constraint::Distance(p0, p1, 10.0));
    solver.add_constraint(Constraint::Distance(p0, p3, 5.0));

    let (_, _, rem_dof) = solver.degrees_of_freedom();
    assert_eq!(rem_dof, 0);
    assert_eq!(
        solver.constraint_status(),
        SketchConstraintStatus::FullyConstrained
    );

    let iters = solver.solve(50, 1e-6).expect("solve should converge");
    assert!(iters < 20);

    let pt1 = solver.get_point(p1).unwrap();
    let pt2 = solver.get_point(p2).unwrap();
    let pt3 = solver.get_point(p3).unwrap();

    assert!((pt1[0] - 10.0).abs() < 1e-5);
    assert!((pt1[1] - 0.0).abs() < 1e-5);
    assert!((pt2[0] - 10.0).abs() < 1e-5);
    assert!((pt2[1] - 5.0).abs() < 1e-5);
    assert!((pt3[0] - 0.0).abs() < 1e-5);
    assert!((pt3[1] - 5.0).abs() < 1e-5);
}

#[test]
fn test_sketch_over_constrained_detection() {
    let mut solver = SketchSolver::new();
    let p0 = solver.add_fixed_point(0.0, 0.0);
    let p1 = solver.add_point(10.0, 0.0);
    solver.add_constraint(Constraint::Horizontal(p0, p1));
    solver.add_constraint(Constraint::Distance(p0, p1, 10.0));
    solver.add_constraint(Constraint::HorizontalDistance(p0, p1, 10.0)); // 冗長

    let status = solver.constraint_status();
    match status {
        SketchConstraintStatus::OverConstrained {
            redundant_constraints,
        } => {
            assert_eq!(redundant_constraints, 1);
        }
        _ => panic!("Expected OverConstrained status, got {:?}", status),
    }
}

/// 階数は、自由変数の本数を超えられない。
///
/// ヤコビアンは固定点の列も持って組まれるので、そのまま特異値を数えると
/// 階数が自由度より大きくなる。`remaining_dof` は `saturating_sub` で 0 に
/// 丸められ、まだ動けるスケッチが `FullyConstrained` として報告される。
/// 幾何学的にありえない大小関係なので、不等式そのものを押さえておく。
#[test]
fn test_rank_never_exceeds_the_free_degrees_of_freedom() {
    // 中心と直線の一端を固定した、直線-円の接線拘束。固定点が2つあるので
    // 全体の変数は 6、自由なのは 2 だけになる。
    let mut solver = SketchSolver::new();
    let centre = solver.add_fixed_point(5.0, 5.0);
    let circle = solver.add_circle(centre, 3.0);
    let anchored = solver.add_fixed_point(0.0, 0.0);
    let free_end = solver.add_point(10.0, 2.0);
    let line = solver.add_line(anchored, free_end);
    solver.add_constraint(Constraint::TangentLineCircle(line, circle));

    let (total_dof, rank, remaining) = solver.degrees_of_freedom();
    assert_eq!(total_dof, 2, "only the free end moves");
    assert!(
        rank <= total_dof,
        "rank {rank} cannot exceed the {total_dof} free variables"
    );
    assert_eq!(
        remaining,
        total_dof - rank,
        "the remainder must be a real subtraction, not a saturated one"
    );
}

/// 固定していない点が残っていれば、完全拘束と言ってはいけない。
#[test]
fn test_a_sketch_with_a_loose_point_is_not_reported_fully_constrained() {
    let mut solver = SketchSolver::new();
    let a = solver.add_fixed_point(0.0, 0.0);
    let b = solver.add_point(10.0, 0.0);
    // b は距離1本しか受けていないので、円周上を自由に動ける（残余1）。
    solver.add_constraint(Constraint::Distance(a, b, 10.0));
    // どこからも拘束されていない点をもう1つ足す（残余2）。
    let _loose = solver.add_point(50.0, 50.0);

    let (total_dof, rank, remaining) = solver.degrees_of_freedom();
    assert!(rank <= total_dof, "rank {rank} vs dof {total_dof}");
    assert!(
        remaining >= 3,
        "one circle freedom plus two for the loose point, got {remaining} \
         (dof {total_dof}, rank {rank})"
    );
    assert!(
        !matches!(
            solver.constraint_status(),
            SketchConstraintStatus::FullyConstrained
        ),
        "a sketch with a completely loose point cannot be fully constrained"
    );
}

// ---- 過剰拘束の中身を見分ける（4-393）----
//
// `OverConstrained` は「式が階数より多い」ことしか言いません。**それは
// 2 つの、まるで違う状態を 1 つの名前で呼んで**います。
//
// | | 使う人がすること |
// | :--- | :--- |
// | **冗長** | **消してよい**。形は変わらない |
// | **矛盾** | **どちらかを直す**。形が決まらない |

/// 三角形の 3 辺に長さを与えて、**そのうち 1 本を 2 回**言う。
fn triangle_with_a_repeated_length() -> zenith_algo::SketchSolver {
    let mut solver = zenith_algo::SketchSolver::new();
    let a = solver.add_fixed_point(0.0, 0.0);
    let b = solver.add_point(30.0, 1.0);
    let c = solver.add_point(12.0, 20.0);
    // **点を1つ固定しただけでは足りません**——**まわりに回れます**。
    // それだと自由度が 1 残り、`UnderConstrained` になります（最初に
    // これを書いて落ちました）。**回転も止めます。**
    solver.add_constraint(zenith_algo::Constraint::Horizontal(a, b));
    solver.add_constraint(zenith_algo::Constraint::Distance(a, b, 30.0));
    solver.add_constraint(zenith_algo::Constraint::Distance(b, c, 25.0));
    solver.add_constraint(zenith_algo::Constraint::Distance(c, a, 20.0));
    // **同じことを 2 回**。
    solver.add_constraint(zenith_algo::Constraint::Distance(a, b, 30.0));
    solver
}

#[test]
fn a_repeated_constraint_is_reported_as_redundant_not_conflicting() {
    let solver = triangle_with_a_repeated_length();
    assert!(
        matches!(
            solver.constraint_status(),
            zenith_algo::SketchConstraintStatus::OverConstrained { .. }
        ),
        "過剰拘束として出ていません: {:?}",
        solver.constraint_status()
    );

    match solver.diagnose_over_constraint(400, 1e-9) {
        Some(zenith_algo::OverConstraintKind::Redundant { worst_residual, .. }) => {
            assert!(
                worst_residual <= 1e-9,
                "冗長なのに残差が {worst_residual:.3e} 残っています"
            );
        }
        other => panic!("冗長と出ませんでした: {other:?}"),
    }
}

#[test]
fn two_different_lengths_for_the_same_pair_are_reported_as_conflicting() {
    // **両立しません。** 30 でも 20 でもある辺は引けません。
    let mut solver = zenith_algo::SketchSolver::new();
    let a = solver.add_fixed_point(0.0, 0.0);
    let b = solver.add_point(25.0, 0.0);
    solver.add_constraint(zenith_algo::Constraint::Horizontal(a, b));
    solver.add_constraint(zenith_algo::Constraint::Distance(a, b, 30.0));
    solver.add_constraint(zenith_algo::Constraint::Distance(a, b, 20.0));

    match solver.diagnose_over_constraint(400, 1e-9) {
        Some(zenith_algo::OverConstraintKind::Conflicting { worst_residual, .. }) => {
            // **半分ずつ譲る**ので、どちらの式にも 5 前後が残ります。
            assert!(
                worst_residual > 1e-9,
                "矛盾なのに残差が {worst_residual:.3e} しかありません"
            );
        }
        other => panic!("矛盾と出ませんでした: {other:?}"),
    }
}

#[test]
fn a_sketch_that_is_not_over_constrained_reports_nothing() {
    let mut solver = zenith_algo::SketchSolver::new();
    let a = solver.add_fixed_point(0.0, 0.0);
    let b = solver.add_point(25.0, 0.0);
    solver.add_constraint(zenith_algo::Constraint::Distance(a, b, 30.0));

    assert!(
        solver.diagnose_over_constraint(400, 1e-9).is_none(),
        "過剰拘束でないのに何か返りました"
    );
}

#[test]
fn diagnosing_does_not_move_the_sketch() {
    // **複製して解きます。** 呼んだだけで形が変わってはいけません。
    let solver = triangle_with_a_repeated_length();
    let before: Vec<[f64; 2]> = solver
        .points
        .iter()
        .map(|point| [point.x, point.y])
        .collect();
    let _ = solver.diagnose_over_constraint(400, 1e-9);
    let after: Vec<[f64; 2]> = solver
        .points
        .iter()
        .map(|point| [point.x, point.y])
        .collect();
    assert_eq!(before, after, "見ただけで点が動きました");
}
