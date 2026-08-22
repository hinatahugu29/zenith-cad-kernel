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
        SketchConstraintStatus::OverConstrained { redundant_constraints } => {
            assert_eq!(redundant_constraints, 1);
        }
        _ => panic!("Expected OverConstrained status, got {:?}", status),
    }
}
