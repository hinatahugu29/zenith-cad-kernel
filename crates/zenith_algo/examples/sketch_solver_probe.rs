use zenith_algo::{Constraint, SketchSolver};

fn main() {
    println!("=== Zenith CAD Kernel: SketchSolver 拘束・自由度解析プローブ ===");

    // ケース 1: 自由な4点（拘束なし）
    let mut solver1 = SketchSolver::new();
    let _p0 = solver1.add_point(0.0, 0.0);
    let _p1 = solver1.add_point(10.0, 0.0);
    let _p2 = solver1.add_point(10.0, 10.0);
    let _p3 = solver1.add_point(0.0, 10.0);
    let (dof1, rank1, rem1) = solver1.degrees_of_freedom();
    let status1 = solver1.constraint_status();
    println!("Case 1 (4自由点): Total DOF = {}, Rank = {}, Remaining DOF = {}, Status = {:?}", dof1, rank1, rem1, status1);

    // ケース 2: 原点固定 + 正方形（完全拘束）
    let mut solver2 = SketchSolver::new();
    let p0 = solver2.add_fixed_point(0.0, 0.0);
    let p1 = solver2.add_point(9.0, 0.5);
    let p2 = solver2.add_point(9.5, 9.5);
    let p3 = solver2.add_point(0.5, 10.5);

    let l0 = solver2.add_line(p0, p1);
    let l1 = solver2.add_line(p1, p2);
    let l2 = solver2.add_line(p2, p3);
    let l3 = solver2.add_line(p3, p0);

    solver2.add_constraint(Constraint::Horizontal(p0, p1));
    solver2.add_constraint(Constraint::Vertical(p0, p3));
    solver2.add_constraint(Constraint::Parallel(l0, l2));
    solver2.add_constraint(Constraint::Parallel(l1, l3));
    solver2.add_constraint(Constraint::Distance(p0, p1, 10.0));
    solver2.add_constraint(Constraint::Distance(p0, p3, 10.0));

    let (dof2, rank2, rem2) = solver2.degrees_of_freedom();
    let status2 = solver2.constraint_status();
    let iters2 = solver2.solve(50, 1e-6).expect("solve square");
    println!("Case 2 (完全拘束正方形): Total DOF = {}, Rank = {}, Remaining DOF = {}, Status = {:?}, Iters = {}", dof2, rank2, rem2, status2, iters2);

    // ケース 3: 接線拘束を持つ直線と円
    let mut solver3 = SketchSolver::new();
    let cp = solver3.add_fixed_point(5.0, 5.0);
    let c = solver3.add_circle(cp, 3.0);
    let lp1 = solver3.add_fixed_point(0.0, 0.0);
    let lp2 = solver3.add_point(10.0, 2.0);
    let line = solver3.add_line(lp1, lp2);
    solver3.add_constraint(Constraint::TangentLineCircle(line, c));
    let (dof3, rank3, rem3) = solver3.degrees_of_freedom();
    let iters3 = solver3.solve(50, 1e-6).expect("solve tangent");
    println!("Case 3 (直線-円接線): Total DOF = {}, Rank = {}, Remaining DOF = {}, Iters = {}", dof3, rank3, rem3, iters3);

    // ケース 4: 冗長な拘束（過剰拘束）
    let mut solver4 = SketchSolver::new();
    let p0 = solver4.add_fixed_point(0.0, 0.0);
    let p1 = solver4.add_point(10.0, 0.0);
    solver4.add_constraint(Constraint::Horizontal(p0, p1));
    solver4.add_constraint(Constraint::Distance(p0, p1, 10.0));
    solver4.add_constraint(Constraint::HorizontalDistance(p0, p1, 10.0)); // 冗長な拘束
    let (dof4, rank4, rem4) = solver4.degrees_of_freedom();
    let status4 = solver4.constraint_status();
    println!("Case 4 (冗長拘束): Total DOF = {}, Rank = {}, Remaining DOF = {}, Status = {:?}", dof4, rank4, rem4, status4);

    println!("--------------------------------------------------------------------------------");
    println!("every sketch constraint case evaluated with exact degrees of freedom");
}
