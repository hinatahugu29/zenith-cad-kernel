//! **穴を横切る割りを、残りの再挑戦にも回す**（4-449）。
//!
//! **この 3 本が留めるのは 2 つ**です。
//!
//! 1. **接する置き方の和と差が返る**——**前は「未実装」で断っていました。**
//! 2. **積は、いまも断る**——**接している点で 2 つに割れる**ので、
//!    **1 つの立体では持てません**（規約 3-1）。**返すほうが困ります。**
//!
//! **2 が要ります。** 4-449 を入れた最初の形では、**積まで返りました**
//! ——**体積は正しい**（閉じた式と一致）**のに、形は 2 つに割れている**
//! ものです。**「体積が合う」は、非多様体を見つけません。**

use zenith_algo::{extrude_sketch, BooleanOpType, BooleanEngine, MassCalculator, SketchSolver, WorkPlane};
use zenith_math::{Tolerance, Vec3};
use zenith_tess::TessellationParams;
use zenith_topo::Solid;

fn rectangle(x0: f64, y0: f64, width: f64, height: f64) -> SketchSolver {
    let mut solver = SketchSolver::new();
    let a = solver.add_point(x0, y0);
    let b = solver.add_point(x0 + width, y0);
    let c = solver.add_point(x0 + width, y0 + height);
    let d = solver.add_point(x0, y0 + height);
    solver.add_line(a, b);
    solver.add_line(b, c);
    solver.add_line(c, d);
    solver.add_line(d, a);
    solver
}

fn add_circle(solver: &mut SketchSolver, cx: f64, cy: f64, r: f64) {
    let centre = solver.add_point(cx, cy);
    let east = solver.add_point(cx + r, cy);
    let north = solver.add_point(cx, cy + r);
    let west = solver.add_point(cx - r, cy);
    let south = solver.add_point(cx, cy - r);
    solver.add_arc(centre, east, north, true);
    solver.add_arc(centre, north, west, true);
    solver.add_arc(centre, west, south, true);
    solver.add_arc(centre, south, east, true);
}

/// **穴のある板と、その穴に接する帯**（`sketch_boolean_probe` の 6 番目）。
fn specimen(tol: &Tolerance) -> (Solid, Solid) {
    let plane = WorkPlane::xy();
    let mut holed = rectangle(0.0, 0.0, 30.0, 30.0);
    add_circle(&mut holed, 15.0, 15.0, 5.0);
    let bar = rectangle(10.0, -20.0, 8.0, 70.0);
    let a = extrude_sketch(&holed, &plane, 8.0, tol).expect("穴のある板");
    let b = zenith_algo::BrepTransform::translate_solid(
        &extrude_sketch(&bar, &plane, 8.0, tol).expect("帯"),
        Vec3::new(0.0, 0.0, -2.0),
    );
    (a, b)
}

fn volume(solids: &[Solid]) -> f64 {
    solids
        .iter()
        .map(|solid| MassCalculator::compute_from_brep(solid, &TessellationParams::default()).volume)
        .sum()
}

#[test]
fn the_union_of_a_plate_and_a_bar_tangent_to_its_hole_closes() {
    let tol = Tolerance::default();
    let (a, b) = specimen(&tol);
    let result = BooleanEngine::boolean_solids_exact_result(&a, &b, BooleanOpType::Union, &tol)
        .expect("和が返ること");
    // **板 6571.6815 ＋ 帯 4480 − 積 1035.8556**。
    //
    // 板 = 30·30·8 − π·5²·8、積は下の試験の註のとおりです。
    let want = (30.0 * 30.0 * 8.0 - std::f64::consts::PI * 25.0 * 8.0) + 4480.0 - 1035.8556;
    let got = volume(&result.solids);
    assert!(
        (got - want).abs() / want <= 1e-5,
        "和の体積が合いません: {got} 対 {want}"
    );
    for solid in &result.solids {
        assert!(
            solid.outer_shell.validate_closed(&tol).errors.is_empty(),
            "和の殻が閉じていません"
        );
    }
}

#[test]
fn the_difference_of_a_plate_and_a_bar_tangent_to_its_hole_closes() {
    let tol = Tolerance::default();
    let (a, b) = specimen(&tol);
    let result = BooleanEngine::boolean_solids_exact_result(&a, &b, BooleanOpType::Difference, &tol)
        .expect("差が返ること");
    let want = (30.0 * 30.0 * 8.0 - std::f64::consts::PI * 25.0 * 8.0) - 1035.8556;
    let got = volume(&result.solids);
    assert!(
        (got - want).abs() / want <= 1e-5,
        "差の体積が合いません: {got} 対 {want}"
    );
    for solid in &result.solids {
        assert!(
            solid.outer_shell.validate_closed(&tol).errors.is_empty(),
            "差の殻が閉じていません"
        );
    }
}

#[test]
fn the_intersection_is_still_refused_by_name() {
    let tol = Tolerance::default();
    let (a, b) = specimen(&tol);
    // **積は (10, 15) の線で 2 つに割れます。**
    //
    // 帯の西の面（x = 10）が、穴（中心 (15, 15)・半径 5）に**接する**ので、
    // **残る材料は、その線でしか繋がっていません**。
    //
    // **体積は出せます**——240 − 67.3574 を高さ 6 倍で 1035.8556。
    // **出せることと、立体として持てることは別**です。
    let reason = BooleanEngine::boolean_solids_exact_result(&a, &b, BooleanOpType::Intersection, &tol)
        .err()
        .expect("積は断られること");
    assert!(
        reason.contains("non-manifold"),
        "断り文が場所を名指ししていません: {reason}"
    );
    assert!(
        !reason.contains("not implemented"),
        "「未実装」で断ってはいけません: {reason}"
    );
}
