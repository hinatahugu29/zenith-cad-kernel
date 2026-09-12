//! **接する置き方で、浮いた端が「相手の面の境界」に乗っているか**（4-446）。
//!
//! # なぜ要るのか
//!
//! 4-444 は「**A の外周の稜を、B 側の分割点で割る**」のが要ると見立て、
//! 4-446 で測ったら**外れていました**——**割る先が無い**のです
//! （いちばん近い境界まで、最小でも 0.5。受け入れの 5 万倍）。
//!
//! **代わりに出てきた形**が、これです——**拒否点の約半分が、板の穴の円
//! （中心 (15, 15)・半径 5）の上**にありました。**交線は、相手の面の
//! 境界に当たったところで終わっている**、という形です。
//!
//! **半径を数えるのは、この検体だけで通る当て推量**です（穴が円だと
//! 知っているから測れます）。**ここでは、面の境界そのものに当てます**
//! ——**円かどうかを知らなくても測れる形**にします。
//!
//! # 何を測るか
//!
//! **浮いた端ひとつずつについて、相手方の立体のどの面の、どの境界の稜に、
//! どれだけ近いか**です。
//!
//! * **稜の上にいる**なら——**続きは、その稜に沿ってある**。
//!   **交線の段で拾えば閉じます。**
//! * **どの稜からも遠い**なら——**別の話**です。
//!
//! # 使い方
//!
//! ```bash
//! cargo run --release -p zenith_algo --example tangent_boundary_probe
//! ```
//!
//! **これは診断です。赤にはしません。**

use std::collections::BTreeMap;
use zenith_algo::{extrude_sketch, BrepIntersectionBuilder, SketchSolver, WorkPlane};
use zenith_math::{Point3, Tolerance, Vec3};
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

/// その点から、立体のどの面の・どの境界の稜までが、いちばん近いか。
///
/// **内側の輪も見ます。** 穴はそこに入っています——**外周だけ見ると、
/// この検体の肝を落とします。**
fn nearest_boundary(point: Point3, solid: &Solid) -> Option<(usize, String, f64)> {
    const SAMPLES: usize = 256;
    let mut best: Option<(usize, String, f64)> = None;
    for (face_index, face) in solid.outer_shell.faces.iter().enumerate() {
        let wires = std::iter::once((&face.outer_wire, "外"))
            .chain(face.inner_wires.iter().map(|wire| (wire, "内")));
        for (wire, kind) in wires {
            for (edge_index, edge) in wire.edges.iter().enumerate() {
                let mut near = f64::INFINITY;
                for step in 0..=SAMPLES {
                    let distance =
                        (edge.evaluate_normalized(step as f64 / SAMPLES as f64) - point).norm();
                    near = near.min(distance);
                }
                if best.as_ref().is_none_or(|(_, _, d)| near < *d) {
                    best = Some((face_index, format!("{kind}の稜{edge_index}"), near));
                }
            }
        }
    }
    best
}

fn main() {
    let tol = Tolerance::default();
    let plane = WorkPlane::xy();

    // **`sketch_boolean_probe` の 6 番目と、同じ 2 つ**です。
    // 値を変えたら、あちらも変えてください。
    let mut holed = rectangle(0.0, 0.0, 30.0, 30.0);
    add_circle(&mut holed, 15.0, 15.0, 5.0);
    let bar = rectangle(10.0, -20.0, 8.0, 70.0);
    let a = extrude_sketch(&holed, &plane, 8.0, &tol).expect("穴のある板");
    let b = zenith_algo::BrepTransform::translate_solid(
        &extrude_sketch(&bar, &plane, 8.0, &tol).expect("帯"),
        Vec3::new(0.0, 0.0, -2.0),
    );

    println!("接する置き方で、浮いた端が相手の面の境界に乗っているか（4-446）");
    println!();

    let candidates = BrepIntersectionBuilder::collect_intersection_edge_candidates(
        &a.outer_shell.faces,
        &b.outer_shell.faces,
        &tol,
    );
    let key = |point: Point3| {
        (
            (point.x * 1e7).round() as i64,
            (point.y * 1e7).round() as i64,
            (point.z * 1e7).round() as i64,
        )
    };
    let mut uses: BTreeMap<_, (usize, Point3)> = BTreeMap::new();
    for candidate in &candidates {
        for point in [
            candidate.edge.start_vertex.point,
            candidate.edge.end_vertex.point,
        ] {
            let entry = uses.entry(key(point)).or_insert((0, point));
            entry.0 += 1;
        }
    }
    let loose: Vec<(usize, Point3)> = uses
        .values()
        .filter(|(count, _)| *count != 2)
        .map(|(count, point)| (*count, *point))
        .collect();

    println!("交線 {} 本、浮いた端 {} 個", candidates.len(), loose.len());
    println!();
    println!(
        "{:<30}{:>5}  {:<22}{:<22}",
        "浮いた端", "使用", "A のいちばん近い境界", "B のいちばん近い境界"
    );
    println!("{}", "-".repeat(92));

    let mut on_boundary = 0usize;
    for (count, point) in &loose {
        let near_a = nearest_boundary(*point, &a);
        let near_b = nearest_boundary(*point, &b);
        let show = |near: &Option<(usize, String, f64)>| match near {
            Some((face, which, distance)) => {
                format!("面{face} {which} {distance:.2e}")
            }
            None => "境界がありません".to_string(),
        };
        // **どちらかの境界の上にいれば、続きはそこにあります。**
        let sits = [&near_a, &near_b]
            .iter()
            .any(|near| near.as_ref().is_some_and(|(_, _, d)| *d <= tol.linear));
        if sits {
            on_boundary += 1;
        }
        println!(
            "({:14.10},{:14.10},{:14.10}){:>5}  {:<22}{:<22}",
            point.x,
            point.y,
            point.z,
            count,
            show(&near_a),
            show(&near_b)
        );
    }

    println!();
    println!(
        "**浮いた端 {} 個のうち、{} 個は相手か自分の面の境界の上**（{:.3e} 以内）",
        loose.len(),
        on_boundary,
        tol.linear
    );
    // ---- **公差で束ねたら、いくつ残るか** ----
    //
    // **1e-7 に丸めて数えると、1.2e-7 離れた 2 点は別**になります。
    // **線形公差は 1e-6** です——**公差の下で別々に数えている**なら、
    // それは「交線が足りない」ではなく「**同じ点を 2 つに数えている**」。
    // **直す場所が、まるで違います。**
    let mut welded: Vec<(Point3, usize)> = Vec::new();
    for (count, point) in &loose {
        match welded
            .iter_mut()
            .find(|(seat, _)| (*seat - *point).norm() <= tol.linear)
        {
            Some((_, total)) => *total += count,
            None => welded.push((*point, *count)),
        }
    }
    let still_loose: Vec<&(Point3, usize)> =
        welded.iter().filter(|(_, count)| *count != 2).collect();

    println!();
    println!("**公差（{:.3e}）で束ねると**", tol.linear);
    println!();
    for (point, count) in &welded {
        println!(
            "  ({:.6}, {:.6}, {:.6})  使用 {}{}",
            point.x,
            point.y,
            point.z,
            count,
            if *count == 2 { "  **閉じます**" } else { "" }
        );
    }
    println!();
    println!(
        "  束ねる前 {} 個 → 束ねたあと、まだ浮いているのは {} 個",
        loose.len(),
        still_loose.len()
    );

    println!();
    println!("**これは診断です。何も直していません。**");
}
