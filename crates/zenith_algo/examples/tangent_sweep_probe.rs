//! **接する寸前・ちょうど・過ぎた後**を掃く（4-451）。
//!
//! # なぜ要るのか
//!
//! 4-449 で、**体積は小数第 4 位まで合っているのに、形は割れている**
//! 答えを 2 件踏みました。**どちらも、接している置き方**です。
//!
//! **接触は、点の置き方です。** **1e-9 動かせば、接していません。**
//! **そこで答えが静かに入れ替わると、使う人は気づけません。**
//!
//! # 何を測るか
//!
//! **帯を、穴の縁をまたいで少しずつ動かします。**
//!
//! * **手前**（食い込んでいない）——**3 演算とも返り、恒等式が閉じる**
//! * **ちょうど**（接している）——**積は断られる**（規約 3-1）
//! * **奥**（食い込んでいる）——**3 演算とも返り、恒等式が閉じる**
//!
//! **見たいのは、境目のまわりで答えが連続しているか**です。
//! **体積が、接する所を挟んで滑らかに繋がっていなければ、どちらかが
//! 誤り**です。
//!
//! # 読み方
//!
//! **断りは誤りではありません**（3-1）。**返ってきたのに恒等式が
//! 破れているほう**と、**接する所を挟んで体積が跳ぶほう**だけを赤に
//! します。
//!
//! # 使い方
//!
//! ```bash
//! cargo run --release -p zenith_algo --example tangent_sweep_probe
//! ```

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

/// **穴のある板 × 帯**。帯の西の縁を `x` に置く。
///
/// **穴は中心 (15, 15)・半径 5** なので、**`x = 10` でちょうど接します**。
/// **`x > 10` なら食い込み、`x < 10` なら離れています**（穴の縁からは）。
fn placement(x: f64, height: f64, tol: &Tolerance) -> Option<(Solid, Solid)> {
    let plane = WorkPlane::xy();
    let mut holed = rectangle(0.0, 0.0, 30.0, 30.0);
    add_circle(&mut holed, 15.0, 15.0, 5.0);
    // 帯の東の縁は 18 で止めます（穴の東側には届きません）。
    let bar = rectangle(x, -20.0, 18.0 - x, 70.0);
    let a = extrude_sketch(&holed, &plane, height, tol).ok()?;
    let b = extrude_sketch(&bar, &plane, height, tol).ok()?;
    Some((
        a,
        zenith_algo::BrepTransform::translate_solid(&b, Vec3::new(0.0, 0.0, -2.0)),
    ))
}

/// **積の、閉じた式**。
///
/// 交わりは **x ∈ [x0, 18]、y ∈ [0, 30]、z ∈ [0, 6]** から、**穴のうち
/// その帯に入る分**を抜いたものです。
///
/// 穴（中心 (15, 15)・半径 5）のうち **x ≤ 18 の分**は、**円から
/// x > 18 の弓形を抜いたもの**——
/// `πr² − (r²·acos(d/r) − d·√(r²−d²))`、`d = 3`。
///
/// **x0 > 10 なら、さらに x < x0 の弓形も抜けます。**
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
    // 東側（x > 18）の弓形。中心から 3。
    let east = segment(3.0);
    // 西側（x < x0）の弓形。中心から 15 − x0。
    let west = segment((15.0 - x0).max(0.0));
    let hole_in_band = disc - east - west;
    let band = (18.0 - x0) * 30.0;
    // 交わる高さは、板 [0, height] と帯 [-2, height-2] の重なり。
    (band - hole_in_band) * (height - 2.0)
}

fn main() {
    let tol = Tolerance::default();

    println!("接する寸前・ちょうど・過ぎた後を掃く（4-451）");
    println!();
    println!("**穴は中心 (15, 15)・半径 5。帯の西の縁を動かします。**");
    println!("**x = 10 でちょうど接します。**");
    println!();
    println!("**高さを 2 通りで掃きます。**——**接触の検出器は、交線の長さの");
    println!("1/100 を輪の半径**に取ります。**高さを変えると半径が変わる**ので、");
    println!("**断りの帯の幅が半径で決まっているなら、一緒に動きます。**");

    let mut broken = 0usize;
    let mut widths: Vec<(f64, f64, f64)> = Vec::new();

    for height in [8.0_f64, 16.0] {
        let contact_length = height - 2.0;
        println!();
        println!(
            "== 高さ {height}（接する線の長さ {contact_length}、輪の半径 {:.4}） ==",
            contact_length * 1e-2
        );
        println!();
        println!(
            "{:>12}{:>8}{:>15}{:>15}{:>12}{:>13}  {}",
            "帯の西の縁", "離れ", "|A∩B|", "閉じた式", "相対差", "恒等式の残差", "3 演算（名=名指し 未=未実装）"
        );
        println!("{}", "-".repeat(100));

        let offsets = [
            -1.0_f64, -0.1, -0.01, -1e-4, 0.0, 1e-4, 0.01, 0.02, 0.03, 0.05, 0.07,
            0.1, 0.2, 0.3, 1.0,
        ];
        let mut first_refused: Option<f64> = None;
        let mut last_refused: Option<f64> = None;

        // **1 つだけ回す口**（`ZENITH_SWEEP_ONLY=10.01`）。
        //
        // **`ZENITH_BATCH_WHY=1` は面の番号しか言いません。** 15 通りぶんの
        // 出力が混ざると、**どの置き方のものか分かりません。**
        let only: Option<f64> = std::env::var("ZENITH_SWEEP_ONLY")
            .ok()
            .and_then(|text| text.parse().ok());
        for offset in offsets {
            let x = 10.0 + offset;
            if let Some(only) = only {
                if (x - only).abs() > 1e-9 {
                    continue;
                }
            }
            let Some((a, b)) = placement(x, height, &tol) else {
                println!("{x:>12.6}  **立体が作れません**");
                continue;
            };

            let mut volumes: [Option<f64>; 3] = [None; 3];
            // **どの演算が断ったか**を、そのまま並べます。
            // **3 つまとめて「断り」と書くと、どれが断ったか消えます。**
            let mut per_op = ["和 ○", "積 ○", "差 ○"];
            for (index, op) in [
                BooleanOpType::Union,
                BooleanOpType::Intersection,
                BooleanOpType::Difference,
            ]
            .into_iter()
            .enumerate()
            {
                match BooleanEngine::boolean_solids_exact_result(&a, &b, op, &tol) {
                    Ok(result) => volumes[index] = Some(volume_of(&result.solids)),
                    Err(reason) => {
                        // **断り方で分けます。** **「非多様体だと名指し」と
                        // 「未実装」は、まるで違うもの**です——前者は
                        // **実装しても返せない**、後者は**返せるはずのものを
                        // 返せていない**。**まとめて「×」と書くと、
                        // 帯の正体が消えます**（4-451）。
                        //
                        // **「未実装」を先に見ます。** **未実装の文面にも
                        // `non-manifold edge uses` が入っています**——縫合の
                        // 報告がそう書くからです。**`non-manifold` を先に
                        // 見ると、未実装まで「名指し」に数えます**
                        // （2026/09/13 に一度そう数えました）。
                        per_op[index] = if reason.contains("not implemented") {
                            ["**和 未**", "**積 未**", "**差 未**"][index]
                        } else if reason.contains("refuses this placement") {
                            ["和 名", "積 名", "差 名"][index]
                        } else {
                            ["和 ×", "積 ×", "差 ×"][index]
                        };
                        // **断り文の頭を控えます。** **どの仕掛けが断ったか**が
                        // 分からないと、帯の正体が決まりません（4-451）。
                        if std::env::var_os("ZENITH_SWEEP_WHY").is_some() {
                            eprintln!(
                                "SWEEPWHY x={x:.6} {} — {}",
                                ["和", "積", "差"][index],
                                reason.chars().take(900).collect::<String>()
                            );
                        }
                    }
                }
            }
            // **接する所そのもの（離れ 0）は、積が断られて当然**です。
            // **数えたいのは、その先で和と差まで断られる帯**です。
            if offset > 0.0 && per_op[0].contains('未') {
                first_refused.get_or_insert(offset);
                last_refused = Some(offset);
            }

            // **通る置き方と通らない置き方を、同じ物差しで並べます**（4-451）。
            //
            // **断り文は、断られたときにしか出ません。** **通ったほうの
            // 縫合の数字が見えないと、「何が違うのか」が決まりません。**
            if std::env::var_os("ZENITH_SWEEP_WHY").is_some() {
                let assembly = zenith_algo::BrepIntersectionBuilder::collect_boolean_shell_assembly(
                    &a,
                    &b,
                    BooleanOpType::Union,
                    &tol,
                );
                let report = &assembly.selection.stitch_report;
                eprintln!(
                    "SWEEPSTITCH x={x:.6} 交線 {} 本、面片 {}、稜の使用 {}、合わない {}、非多様体 {}、同じ向き {}",
                    assembly.edge_candidates.len(),
                    report.face_piece_count,
                    report.edge_use_count,
                    report.unmatched_edge_use_count,
                    report.non_manifold_edge_use_count,
                    report.same_direction_edge_use_count
                );
            }

            let want = closed_form_intersection(x, height);
            let (shown, residual_text) = match (volumes[0], volumes[1], volumes[2]) {
                (Some(union), Some(meet), Some(minus)) => {
                    let va = volume_of(std::slice::from_ref(&a));
                    let vb = volume_of(std::slice::from_ref(&b));
                    let worst = (((union + meet) - (va + vb)).abs() / (va + vb).max(1.0))
                        .max((minus + meet - va).abs() / va.max(1.0));
                    if worst > 1e-6 {
                        broken += 1;
                    }
                    (Some(meet), format!("{worst:.3e}"))
                }
                (_, Some(meet), _) => (Some(meet), "-".to_string()),
                _ => (None, "-".to_string()),
            };

            match shown {
                Some(got) => {
                    let relative = (got - want).abs() / want.abs().max(1.0);
                    if relative > 1e-5 {
                        broken += 1;
                    }
                    println!(
                        "{x:>12.6}{offset:>8.0e}{got:>15.4}{want:>15.4}{relative:>12.1e}{residual_text:>13}  {}",
                        per_op.join(" ")
                    );
                }
                None => println!(
                    "{x:>12.6}{offset:>8.0e}{:>15}{want:>15.4}{:>12}{:>13}  {}",
                    "断り",
                    "-",
                    "-",
                    per_op.join(" ")
                ),
            }
        }

        widths.push((
            height,
            first_refused.unwrap_or(f64::NAN),
            last_refused.unwrap_or(f64::NAN),
        ));
    }

    println!();
    println!("**和が「未実装」で断られる帯**（接する所より先）");
    println!();
    for (height, first, last) in &widths {
        println!("  高さ {height:>5}: 離れ {first:.0e} 〜 {last:.0e}（輪の半径 {:.4}）",
            (height - 2.0) * 1e-2);
    }

    println!();
    if broken > 0 {
        println!("**返ってきたのに答えが合わないものが {broken} 件あります。**");
        std::process::exit(1);
    }
    println!("**返ったものは全部、閉じた式と 1e-5 以内で合っています。**");
    println!();
    println!("**断りは誤りではありません**（3-1）。**ここは赤にしません。**");
}
