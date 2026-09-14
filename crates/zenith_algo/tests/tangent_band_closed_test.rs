//! **接する所のすぐ先で、3 演算が通るか**（4-457）。
//!
//! # なぜ要るのか
//!
//! 4-451 が、**接していないのに 3 演算とも「未実装」で断られる
//! 幅 0.02 の帯**を見つけました。**体積は誰も間違えていません**
//! ——**返ってこない**のが欠陥でした。
//!
//! 4-457 で塞ぎました（`point_inside_pcurve_loop` の「境界の上」を、
//! **光線の媒介変数ではなく長さ**で測るようにした）。**塞いだものは、
//! 開いたときに鳴るようにしておきます。**
//!
//! # 何を測るか
//!
//! **穴のある板（中心 (15, 15)・半径 5）に、西の縁を `x` に置いた帯**を
//! 当てます。**`x = 10` でちょうど接する**ので、**そのすぐ先**を見ます。
//!
//! * **帯の中にあった 3 つの離れ**（0.01、0.02、0.0235）——**4-456 で
//!   「繋がらない隙間」が 1.0033e-6 〜 1.537e-6 と測れた所**です
//! * **帯の外の 1 つ**（0.03）——**前から通っていた所**。ここが赤に
//!   なったら、直しではなく壊しです
//!
//! **見るのは 2 つだけ**——**3 演算とも返ること**と、**積が閉じた式に
//! 合うこと**。
//!
//! **接する所そのもの（離れ 0）は入れていません。** **そこは積が
//! 名指しで断るのが正しい**からです（規約 3-1）。

use zenith_algo::{
    extrude_sketch, BooleanEngine, BooleanOpType, MassCalculator, SketchSolver, WorkPlane,
};
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

fn volume_of(solids: &[Solid]) -> f64 {
    solids
        .iter()
        .map(|solid| MassCalculator::compute_from_brep(solid, &TessellationParams::default()).volume)
        .sum()
}

/// **積の、閉じた式**（`tangent_sweep_probe` と同じ式）。
fn closed_form_intersection(x0: f64, height: f64) -> f64 {
    let r: f64 = 5.0;
    let disc = std::f64::consts::PI * r * r;
    let segment = |d: f64| {
        if d >= r {
            0.0
        } else {
            r * r * (d / r).acos() - d * (r * r - d * d).sqrt()
        }
    };
    let hole_in_band = disc - segment(3.0) - segment((15.0 - x0).max(0.0));
    let band = (18.0 - x0) * 30.0;
    (band - hole_in_band) * (height - 2.0)
}

#[test]
fn the_band_just_past_the_tangency_returns_all_three_operations() {
    let tol = Tolerance::default();
    let height = 8.0;
    let plane = WorkPlane::xy();

    // **前の 3 つは、4-451〜4-456 で断られていた離れ**。最後の 1 つは、
    // **前から通っていた離れ**（壊していないことの見張り）。
    for offset in [0.01_f64, 0.02, 0.0235, 0.03] {
        let x = 10.0 + offset;
        let mut holed = rectangle(0.0, 0.0, 30.0, 30.0);
        add_circle(&mut holed, 15.0, 15.0, 5.0);
        let bar = rectangle(x, -20.0, 18.0 - x, 70.0);
        let a = extrude_sketch(&holed, &plane, height, &tol).expect("穴のある板");
        let b = zenith_algo::BrepTransform::translate_solid(
            &extrude_sketch(&bar, &plane, height, &tol).expect("帯"),
            Vec3::new(0.0, 0.0, -2.0),
        );

        for op in [
            BooleanOpType::Union,
            BooleanOpType::Intersection,
            BooleanOpType::Difference,
        ] {
            let result = BooleanEngine::boolean_solids_exact_result(&a, &b, op, &tol);
            let solids = result.map(|outcome| outcome.solids).unwrap_or_else(|error| {
                panic!("離れ {offset} の {op:?} が断られました: {error}")
            });
            assert!(
                !solids.is_empty(),
                "離れ {offset} の {op:?} が、立体を 1 つも返しませんでした"
            );
        }

        // **返ってきただけでは足りません。** 積は閉じた式に乗るべきです。
        let product = BooleanEngine::boolean_solids_exact_result(&a, &b, BooleanOpType::Intersection, &tol)
            .expect("積")
            .solids;
        let measured = volume_of(&product);
        let expected = closed_form_intersection(x, height);
        let relative = (measured - expected).abs() / expected.abs().max(1.0);
        assert!(
            relative <= 1e-5,
            "離れ {offset} の積が {measured:.6}、閉じた式は {expected:.6}（相対差 {relative:.3e}）"
        );
    }
}
