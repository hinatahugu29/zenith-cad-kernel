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
