//! **スケッチから立体までの門**（HANDOVER 9-H の H3）。
//!
//! # 測り方
//!
//! スケッチは「何を編集させるか」で形が決まるので、そこはまだ作りません
//! （9-G）。**ここで測るのは、閉じた式で答え合わせできる部分だけ**です。
//!
//! ```text
//! 輪の面積      = 多角形の公式（靴紐）
//! 押し出した体積 = 面積 × 高さ
//! ```
//!
//! **「解けた」で終わらせません。** ソルバーが成功を返したことではなく、
//! 出てきた形のほうを測ります。

use std::f64::consts::PI;
use zenith_algo::{
    extract_loops, extrude_sketch, Constraint, MassCalculator, SketchSolver, WorkPlane,
};
use zenith_math::{Point3, Tolerance, Vec3};
use zenith_tess::TessellationParams;

/// 立体の体積。面積分は解析的なので、刻みに依りません。
fn volume(solid: &zenith_topo::Solid) -> f64 {
    MassCalculator::compute_from_brep(
        solid,
        &TessellationParams {
            u_divisions: 32,
            v_divisions: 32,
        },
    )
    .volume
}

/// 与えた点をそのまま四角形として組む。**拘束は掛けません**——ここで
/// 測りたいのは輪の取り出しであって、ソルバーではありません。
fn rectangle(width: f64, height: f64) -> SketchSolver {
    let mut solver = SketchSolver::new();
    let a = solver.add_point(0.0, 0.0);
    let b = solver.add_point(width, 0.0);
    let c = solver.add_point(width, height);
    let d = solver.add_point(0.0, height);
    solver.add_line(a, b);
    solver.add_line(b, c);
    solver.add_line(c, d);
    solver.add_line(d, a);
    solver
}

#[test]
fn a_rectangle_gives_one_loop_with_the_right_area() {
    let tol = Tolerance::default();
    let solver = rectangle(30.0, 20.0);
    let loops = extract_loops(&solver, &tol).expect("輪が取り出せません");
    assert_eq!(loops.len(), 1, "輪が {} 本ありました", loops.len());
    let area = loops[0].area();
    assert!(
        (area - 600.0).abs() <= 1e-9,
        "面積が {area:.9} で、閉じた式の 600 と違います"
    );
}

#[test]
fn the_loop_is_found_whatever_order_the_lines_were_added_in() {
    // **輪の取り出しは、線を足した順に依りません。** 端点の一致で辿ります。
    let tol = Tolerance::default();
    let mut solver = SketchSolver::new();
    let a = solver.add_point(0.0, 0.0);
    let b = solver.add_point(10.0, 0.0);
    let c = solver.add_point(10.0, 10.0);
    let d = solver.add_point(0.0, 10.0);
    // わざとばらばらに、向きも揃えずに足します。
    solver.add_line(c, b);
    solver.add_line(d, a);
    solver.add_line(a, b);
    solver.add_line(c, d);

    let loops = extract_loops(&solver, &tol).expect("輪が取り出せません");
    assert_eq!(loops.len(), 1, "輪が {} 本ありました", loops.len());
    assert!(
        (loops[0].area() - 100.0).abs() <= 1e-9,
        "面積が {:.9} で、100 と違います",
        loops[0].area()
    );
}

#[test]
fn points_joined_by_a_constraint_do_not_break_the_loop() {
    // **番号ではなく座標で辿ります。**
    //
    // 拘束で重ねた2点は、番号が違うまま同じ場所に来ます。番号で辿ると
    // そこで輪が切れます。
    let tol = Tolerance::default();
    let mut solver = SketchSolver::new();
    let a = solver.add_fixed_point(0.0, 0.0);
    let b = solver.add_point(10.0, 0.0);
    let c = solver.add_point(10.0, 10.0);
    let d = solver.add_point(0.0, 10.0);
    // 最後の辺の終点を、`a` とは**別の点**にしておいて、拘束で重ねます。
    let d_end = solver.add_point(0.5, -0.5);
    solver.add_line(a, b);
    solver.add_line(b, c);
    solver.add_line(c, d);
    solver.add_line(d, d_end);
    solver.add_constraint(Constraint::Coincident(d_end, a));
    solver.solve(200, 1e-12).expect("解けません");

    let loops = extract_loops(&solver, &tol).expect("重ねた点で輪が切れています");
    assert_eq!(loops.len(), 1, "輪が {} 本ありました", loops.len());
    assert!(
        (loops[0].area() - 100.0).abs() <= 1e-6,
        "面積が {:.9} で、100 と違います",
        loops[0].area()
    );
}

#[test]
fn a_branching_sketch_is_refused_rather_than_guessed() {
    // **もっともらしい輪を返してはいけません。** 1つの点に3本集まったら、
    // どちらへ進むかは決められません。
    let tol = Tolerance::default();
    let mut solver = rectangle(10.0, 10.0);
    let extra = solver.add_point(5.0, 5.0);
    let corner = zenith_algo::PointId(0);
    solver.add_line(corner, extra);

    assert!(
        extract_loops(&solver, &tol).is_none(),
        "分岐しているのに輪を返しました"
    );
}

#[test]
fn an_open_chain_is_refused() {
    let tol = Tolerance::default();
    let mut solver = SketchSolver::new();
    let a = solver.add_point(0.0, 0.0);
    let b = solver.add_point(10.0, 0.0);
    let c = solver.add_point(10.0, 10.0);
    solver.add_line(a, b);
    solver.add_line(b, c);

    assert!(
        extract_loops(&solver, &tol).is_none(),
        "開いた鎖なのに輪を返しました"
    );
}

#[test]
fn extruding_a_sketch_gives_area_times_height() {
    // **これが H3 の本体です。**
    let tol = Tolerance::default();
    let solver = rectangle(30.0, 20.0);
    let plane = WorkPlane::xy();
    let solid = extrude_sketch(&solver, &plane, 7.0, &tol).expect("押し出せません");

    let measured = volume(&solid);
    let expected = 600.0 * 7.0;
    let residual = (measured - expected).abs() / expected;
    assert!(
        residual <= 1e-9,
        "体積が {measured:.9} で、面積 × 高さ の {expected:.9} と相対 {residual:.3e} 違います"
    );
}

#[test]
fn the_answer_does_not_depend_on_where_the_workplane_is() {
    // **作業平面を動かしても回しても、体積は変わりません。**
    //
    // 4-159 で、原点から離すと体積が 6.67 倍になる誤りを見つけています。
    // 平面の写像でも同じ形の誤りが起こりえます。
    let tol = Tolerance::default();
    let solver = rectangle(12.0, 9.0);
    let expected = 12.0 * 9.0 * 5.0;

    let flat = extrude_sketch(&solver, &WorkPlane::xy(), 5.0, &tol).expect("押し出せません");
    let here = volume(&flat);
    assert!(
        (here - expected).abs() / expected <= 1e-9,
        "原点で {here:.9}、期待 {expected:.9}"
    );

    // 斜めに傾けて、原点からも離します。
    let tilted = WorkPlane::from_normal(Point3::new(137.0, -91.0, 53.0), Vec3::new(1.0, 2.0, 3.0))
        .expect("作業平面が作れません");
    let solid = extrude_sketch(&solver, &tilted, 5.0, &tol).expect("押し出せません");
    let there = volume(&solid);
    let residual = (there - expected).abs() / expected;
    assert!(
        residual <= 1e-9,
        "傾けて離すと体積が {there:.9} になりました（期待 {expected:.9}、相対 {residual:.3e}）"
    );
}

#[test]
fn a_solved_sketch_extrudes_to_the_size_the_constraints_asked_for() {
    // **端から端まで通します**——拘束を解いて、その形を押し出し、
    // **指定した寸法どおりの体積になるか**を測ります。
    let tol = Tolerance::default();
    let mut solver = SketchSolver::new();
    let a = solver.add_fixed_point(0.0, 0.0);
    let b = solver.add_point(7.0, 3.0);
    let c = solver.add_point(9.0, 11.0);
    let d = solver.add_point(-2.0, 8.0);
    solver.add_line(a, b);
    solver.add_line(b, c);
    solver.add_line(c, d);
    solver.add_line(d, a);

    // 40 × 25 の長方形になるように縛ります。
    solver.add_constraint(Constraint::Horizontal(a, b));
    solver.add_constraint(Constraint::Distance(a, b, 40.0));
    solver.add_constraint(Constraint::Vertical(b, c));
    solver.add_constraint(Constraint::Distance(b, c, 25.0));
    solver.add_constraint(Constraint::Horizontal(c, d));
    solver.add_constraint(Constraint::Distance(c, d, 40.0));
    solver.add_constraint(Constraint::Vertical(d, a));
    solver.solve(400, 1e-12).expect("解けません");

    let loops = extract_loops(&solver, &tol).expect("輪が取り出せません");
    assert_eq!(loops.len(), 1);
    let area = loops[0].area();
    assert!(
        (area - 1000.0).abs() <= 1e-6,
        "面積が {area:.9} で、40 × 25 = 1000 と違います"
    );

    let solid = extrude_sketch(&solver, &WorkPlane::xy(), 3.0, &tol).expect("押し出せません");
    let measured = volume(&solid);
    let expected = 1000.0 * 3.0;
    let residual = (measured - expected).abs() / expected;
    assert!(
        residual <= 1e-6,
        "体積が {measured:.9} で、期待の {expected:.9} と相対 {residual:.3e} 違います"
    );
}

#[test]
fn a_half_disc_has_the_area_of_half_a_circle() {
    // **円弧の切片を、閉じた式で照らします。**
    //
    // 半径 10 の半円と、その直径。面積は `πr²/2 = 50π`。
    let tol = Tolerance::default();
    let mut solver = SketchSolver::new();
    let centre = solver.add_point(0.0, 0.0);
    let right = solver.add_point(10.0, 0.0);
    let left = solver.add_point(-10.0, 0.0);
    // 直径（左から右へ）と、上を回る弧（右から左へ、反時計回り）。
    solver.add_line(left, right);
    solver.add_arc(centre, right, left, true);

    let loops = extract_loops(&solver, &tol).expect("輪が取り出せません");
    assert_eq!(loops.len(), 1, "輪が {} 本ありました", loops.len());
    let area = loops[0].area();
    let expected = PI * 100.0 / 2.0;
    assert!(
        (area - expected).abs() / expected <= 1e-12,
        "面積が {area:.9} で、πr²/2 の {expected:.9} と違います"
    );
}

#[test]
fn a_rounded_slot_has_the_area_of_a_rectangle_plus_a_circle() {
    // **長円（スタジアム形）**。直線2本＋半円2つ。
    //
    // 面積 = 中央の長方形 + 両端の半円 = `2 r L + π r²`。
    let tol = Tolerance::default();
    let (length, radius) = (30.0, 6.0);
    let mut solver = SketchSolver::new();
    let right_centre = solver.add_point(length / 2.0, 0.0);
    let left_centre = solver.add_point(-length / 2.0, 0.0);
    let a = solver.add_point(length / 2.0, -radius);
    let b = solver.add_point(length / 2.0, radius);
    let c = solver.add_point(-length / 2.0, radius);
    let d = solver.add_point(-length / 2.0, -radius);
    solver.add_arc(right_centre, a, b, true); // 右端の半円
    solver.add_line(b, c); // 上辺
    solver.add_arc(left_centre, c, d, true); // 左端の半円
    solver.add_line(d, a); // 下辺

    let loops = extract_loops(&solver, &tol).expect("輪が取り出せません");
    assert_eq!(loops.len(), 1, "輪が {} 本ありました", loops.len());
    let area = loops[0].area();
    let expected = 2.0 * radius * length + PI * radius * radius;
    assert!(
        (area - expected).abs() / expected <= 1e-12,
        "面積が {area:.9} で、2rL + πr² の {expected:.9} と違います"
    );
}

#[test]
fn extruding_a_rounded_slot_gives_area_times_height() {
    // **H3 の本体、円弧つき。**
    let tol = Tolerance::default();
    let (length, radius, height) = (30.0, 6.0, 4.0);
    let mut solver = SketchSolver::new();
    let right_centre = solver.add_point(length / 2.0, 0.0);
    let left_centre = solver.add_point(-length / 2.0, 0.0);
    let a = solver.add_point(length / 2.0, -radius);
    let b = solver.add_point(length / 2.0, radius);
    let c = solver.add_point(-length / 2.0, radius);
    let d = solver.add_point(-length / 2.0, -radius);
    solver.add_arc(right_centre, a, b, true);
    solver.add_line(b, c);
    solver.add_arc(left_centre, c, d, true);
    solver.add_line(d, a);

    let solid = extrude_sketch(&solver, &WorkPlane::xy(), height, &tol).expect("押し出せません");
    let measured = volume(&solid);
    let expected = (2.0 * radius * length + PI * radius * radius) * height;
    let residual = (measured - expected).abs() / expected;
    assert!(
        residual <= 1e-9,
        "体積が {measured:.9} で、面積 × 高さ の {expected:.9} と相対 {residual:.3e} 違います"
    );
}

#[test]
fn an_arc_whose_ends_are_not_the_same_distance_from_the_centre_is_refused() {
    // **推測しません。** 半径が合っていなければ、それは弧ではありません。
    let tol = Tolerance::default();
    let mut solver = SketchSolver::new();
    let centre = solver.add_point(0.0, 0.0);
    let right = solver.add_point(10.0, 0.0);
    let wrong = solver.add_point(-7.0, 0.0); // 半径が 7 で合っていない
    solver.add_line(wrong, right);
    solver.add_arc(centre, right, wrong, true);

    assert!(
        extract_loops(&solver, &tol).is_none(),
        "半径が合っていないのに弧として通しました"
    );
}

#[test]
fn a_workplane_normal_is_perpendicular_to_its_axes() {
    // 作業平面が直交していないと、写した形が歪みます。
    for normal in [
        Vec3::new(0.0, 0.0, 1.0),
        Vec3::new(1.0, 0.0, 0.0),
        Vec3::new(1.0, 2.0, 3.0),
        Vec3::new(-5.0, 0.3, 0.0),
    ] {
        let plane = WorkPlane::from_normal(Point3::origin(), normal).expect("作れません");
        assert!(
            plane.x_axis.dot(&plane.y_axis).abs() <= 1e-12,
            "軸が直交していません: {:.3e}",
            plane.x_axis.dot(&plane.y_axis).abs()
        );
        assert!(
            (plane.x_axis.norm() - 1.0).abs() <= 1e-12
                && (plane.y_axis.norm() - 1.0).abs() <= 1e-12,
            "軸の長さが 1 ではありません"
        );
        let made = plane.normal();
        let wanted = normal / normal.norm();
        let sine = made.cross(&wanted).norm();
        assert!(sine <= 1e-12, "法線が指定と違います（正弦 {sine:.3e}）");
    }
    // 0 ベクトルは断ります。
    assert!(
        WorkPlane::from_normal(Point3::origin(), Vec3::new(0.0, 0.0, 0.0)).is_none(),
        "0 の法線なのに平面を返しました"
    );
    let _ = PI;
}

#[test]
fn revolving_a_sketch_obeys_pappus() {
    // **パップスの定理で照らします。**
    //
    // ```text
    // 体積 = 2π × (重心から軸までの距離) × 面積
    // ```
    //
    // **閉じた式です。** 断面が長方形なら重心も面積も手で書けます。
    // 軸から離した長方形を1周させると、**角のある輪（リング）**になります。
    let tol = Tolerance::default();
    let (width, height, offset) = (4.0, 6.0, 10.0);

    // x ∈ [offset, offset + width]、y ∈ [0, height] の長方形。
    let mut solver = SketchSolver::new();
    let a = solver.add_point(offset, 0.0);
    let b = solver.add_point(offset + width, 0.0);
    let c = solver.add_point(offset + width, height);
    let d = solver.add_point(offset, height);
    solver.add_line(a, b);
    solver.add_line(b, c);
    solver.add_line(c, d);
    solver.add_line(d, a);

    // 軸は原点を通る y 方向（スケッチの座標）。
    let solid = zenith_algo::revolve_sketch(
        &solver,
        &WorkPlane::xy(),
        zenith_math::Point2::new(0.0, 0.0),
        zenith_math::Point2::new(0.0, 1.0),
        &tol,
    )
    .expect("回せません");

    let area = width * height;
    let centroid = offset + width / 2.0;
    let expected = 2.0 * PI * centroid * area;
    let measured = volume(&solid);
    let residual = (measured - expected).abs() / expected;
    // **実測は 1e-13 より良い**です（掃くのが有理2次の四半弧で厳密、
    // 面積分も解析的なので、刻みに依りません）。**関門は 1e-12** に
    // 置いて、少しだけ余裕を持たせています。
    assert!(
        residual <= 1e-12,
        "体積が {measured:.9} で、パップスの 2π·{centroid}·{area} = {expected:.9} と \
         相対 {residual:.3e} 違います"
    );
}

#[test]
fn a_loop_that_crosses_the_axis_is_refused() {
    // **またぐ輪を回すと、自分を突き抜けます。** 推測せずに断ります。
    let tol = Tolerance::default();
    let mut solver = SketchSolver::new();
    let a = solver.add_point(-5.0, 0.0);
    let b = solver.add_point(5.0, 0.0);
    let c = solver.add_point(5.0, 6.0);
    let d = solver.add_point(-5.0, 6.0);
    solver.add_line(a, b);
    solver.add_line(b, c);
    solver.add_line(c, d);
    solver.add_line(d, a);

    assert!(
        zenith_algo::revolve_sketch(
            &solver,
            &WorkPlane::xy(),
            zenith_math::Point2::new(0.0, 0.0),
            zenith_math::Point2::new(0.0, 1.0),
            &tol,
        )
        .is_err(),
        "軸をまたいでいるのに回しました"
    );
}

#[test]
fn revolving_does_not_depend_on_where_the_workplane_is() {
    // **作業平面を傾けて原点から離しても、体積は変わりません**（4-159）。
    let tol = Tolerance::default();
    let (width, height, offset) = (3.0, 5.0, 8.0);
    let mut solver = SketchSolver::new();
    let a = solver.add_point(offset, 0.0);
    let b = solver.add_point(offset + width, 0.0);
    let c = solver.add_point(offset + width, height);
    let d = solver.add_point(offset, height);
    solver.add_line(a, b);
    solver.add_line(b, c);
    solver.add_line(c, d);
    solver.add_line(d, a);

    let expected = 2.0 * PI * (offset + width / 2.0) * width * height;
    let axis_point = zenith_math::Point2::new(0.0, 0.0);
    let axis_dir = zenith_math::Point2::new(0.0, 1.0);

    let here = volume(
        &zenith_algo::revolve_sketch(&solver, &WorkPlane::xy(), axis_point, axis_dir, &tol)
            .expect("回せません"),
    );
    assert!(
        (here - expected).abs() / expected <= 1e-12,
        "原点で {here:.9}、期待 {expected:.9}"
    );

    let tilted = WorkPlane::from_normal(Point3::new(137.0, -91.0, 53.0), Vec3::new(1.0, 2.0, 3.0))
        .expect("作業平面が作れません");
    let there = volume(
        &zenith_algo::revolve_sketch(&solver, &tilted, axis_point, axis_dir, &tol)
            .expect("回せません"),
    );
    let residual = (there - expected).abs() / expected;
    assert!(
        residual <= 1e-12,
        "傾けて離すと体積が {there:.9} になりました（期待 {expected:.9}、相対 {residual:.3e}）"
    );
}

/// 中心 (cx, cy)、半径 r の円を、四半弧 4 本でスケッチに足す。
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

#[test]
fn a_sketch_with_a_round_hole_extrudes_to_area_minus_hole_times_height() {
    // **実務でいちばん多い形です**——外形の中に穴。
    //
    // ```text
    // 体積 = (W·H − πr²) × 高さ
    // ```
    let tol = Tolerance::default();
    let (w, h, r, height) = (40.0, 30.0, 5.0, 7.0);

    let mut solver = SketchSolver::new();
    let a = solver.add_point(0.0, 0.0);
    let b = solver.add_point(w, 0.0);
    let c = solver.add_point(w, h);
    let d = solver.add_point(0.0, h);
    solver.add_line(a, b);
    solver.add_line(b, c);
    solver.add_line(c, d);
    solver.add_line(d, a);
    add_circle(&mut solver, w / 2.0, h / 2.0, r);

    let loops = extract_loops(&solver, &tol).expect("輪が取り出せません");
    assert_eq!(loops.len(), 2, "輪が {} 本ありました", loops.len());

    let solid = extrude_sketch(&solver, &WorkPlane::xy(), height, &tol).expect("押し出せません");
    let expected = (w * h - PI * r * r) * height;
    let measured = volume(&solid);
    let residual = (measured - expected).abs() / expected;
    assert!(
        residual <= 1e-9,
        "体積が {measured:.9} で、(W·H − πr²)·高さ = {expected:.9} と \
         相対 {residual:.3e} 違います"
    );
}

#[test]
fn two_holes_are_both_subtracted() {
    let tol = Tolerance::default();
    let (w, h, r, height) = (60.0, 30.0, 4.0, 5.0);
    let mut solver = SketchSolver::new();
    let a = solver.add_point(0.0, 0.0);
    let b = solver.add_point(w, 0.0);
    let c = solver.add_point(w, h);
    let d = solver.add_point(0.0, h);
    solver.add_line(a, b);
    solver.add_line(b, c);
    solver.add_line(c, d);
    solver.add_line(d, a);
    add_circle(&mut solver, 15.0, 15.0, r);
    add_circle(&mut solver, 45.0, 15.0, r);

    let solid = extrude_sketch(&solver, &WorkPlane::xy(), height, &tol).expect("押し出せません");
    let expected = (w * h - 2.0 * PI * r * r) * height;
    let measured = volume(&solid);
    let residual = (measured - expected).abs() / expected;
    assert!(
        residual <= 1e-9,
        "体積が {measured:.9} で、期待の {expected:.9} と相対 {residual:.3e} 違います"
    );
}

#[test]
fn two_separate_outlines_are_refused_rather_than_guessed() {
    // **外形が2つある図は、立体が2つに分かれます。** どちらを返すかを
    // 決められないので、断ります。
    let tol = Tolerance::default();
    let mut solver = SketchSolver::new();
    for offset in [0.0, 50.0] {
        let a = solver.add_point(offset, 0.0);
        let b = solver.add_point(offset + 20.0, 0.0);
        let c = solver.add_point(offset + 20.0, 20.0);
        let d = solver.add_point(offset, 20.0);
        solver.add_line(a, b);
        solver.add_line(b, c);
        solver.add_line(c, d);
        solver.add_line(d, a);
    }
    assert!(
        extrude_sketch(&solver, &WorkPlane::xy(), 5.0, &tol).is_err(),
        "外形が2つあるのに立体を返しました"
    );
}

#[test]
fn an_island_inside_a_hole_is_refused() {
    // **入れ子が二段**（穴の中の島）は受け取れません。
    let tol = Tolerance::default();
    let mut solver = SketchSolver::new();
    let a = solver.add_point(0.0, 0.0);
    let b = solver.add_point(40.0, 0.0);
    let c = solver.add_point(40.0, 40.0);
    let d = solver.add_point(0.0, 40.0);
    solver.add_line(a, b);
    solver.add_line(b, c);
    solver.add_line(c, d);
    solver.add_line(d, a);
    add_circle(&mut solver, 20.0, 20.0, 15.0);
    add_circle(&mut solver, 20.0, 20.0, 5.0);

    assert!(
        extrude_sketch(&solver, &WorkPlane::xy(), 5.0, &tol).is_err(),
        "穴の中の島があるのに立体を返しました"
    );
}
