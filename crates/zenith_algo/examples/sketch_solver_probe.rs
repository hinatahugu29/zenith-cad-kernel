use zenith_algo::{Constraint, SketchConstraintStatus, SketchSolver};

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
    println!(
        "Case 1 (4自由点): Total DOF = {}, Rank = {}, Remaining DOF = {}, Status = {:?}",
        dof1, rank1, rem1, status1
    );

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
    println!(
        "Case 3 (直線-円接線): Total DOF = {}, Rank = {}, Remaining DOF = {}, Iters = {}",
        dof3, rank3, rem3, iters3
    );

    // ケース 4: 冗長な拘束（過剰拘束）
    let mut solver4 = SketchSolver::new();
    let p0 = solver4.add_fixed_point(0.0, 0.0);
    let p1 = solver4.add_point(10.0, 0.0);
    solver4.add_constraint(Constraint::Horizontal(p0, p1));
    solver4.add_constraint(Constraint::Distance(p0, p1, 10.0));
    solver4.add_constraint(Constraint::HorizontalDistance(p0, p1, 10.0)); // 冗長な拘束
    let (dof4, rank4, rem4) = solver4.degrees_of_freedom();
    let status4 = solver4.constraint_status();
    println!(
        "Case 4 (冗長拘束): Total DOF = {}, Rank = {}, Remaining DOF = {}, Status = {:?}",
        dof4, rank4, rem4, status4
    );

    println!("--------------------------------------------------------------------------------");
    // **床**（4-651）。**この 1 行は、ずっと無条件でした**——
    // **4 つの場面が何を返そうと「exact degrees of freedom」と
    // 印字していた**わけです。**判定の無い門より悪い**——
    // **「確かめた」と書いてある**からです。
    //
    // **あるべき姿は、場面の名前が述べています**——
    // 4自由点 → 自由度 8 が残る／完全拘束正方形 → 残り 0／
    // 直線-円接線 → 残り 1（接線 1 本で 2 のうち 1 を取る）／
    // 冗長拘束 → 過剰拘束で、冗長 1 本。
    // **閾値ではなく、探り自身が名前で宣言している値**です。
    // **期待値は 1 か所に置きます**——**判定と文面を別々に書くと、
    // 片方だけ直して食い違います**（実際、実演のときに文面が追随
    // しませんでした）。
    let mut bad = Vec::new();
    for (name, got, want) in [
        ("Case 1（4自由点）", rem1, 8usize),
        ("Case 2（完全拘束正方形）", rem2, 0),
        ("Case 3（直線-円接線）", rem3, 1),
        ("Case 4（冗長拘束）", rem4, 0),
    ] {
        if got != want {
            bad.push(format!("{name}の残り自由度が {got}（{want} のはず）"));
        }
    }
    if !matches!(status4, SketchConstraintStatus::OverConstrained { .. }) {
        bad.push(format!("Case 4（冗長拘束）が過剰拘束と出ていません: {status4:?}"));
    }
    if !matches!(status1, SketchConstraintStatus::UnderConstrained { .. }) {
        bad.push(format!("Case 1（4自由点）が不足拘束と出ていません: {status1:?}"));
    }
    if !matches!(status2, SketchConstraintStatus::FullyConstrained) {
        bad.push(format!("Case 2（完全拘束正方形）が完全拘束と出ていません: {status2:?}"));
    }
    if !bad.is_empty() {
        for line in &bad {
            println!("**{line}**（4-651）。");
        }
        std::process::exit(1);
    }
    println!("every sketch constraint case evaluated with exact degrees of freedom");
}
