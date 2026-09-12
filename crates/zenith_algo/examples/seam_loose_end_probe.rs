//! **浮いた端から、交線を歩かせてみる**（4-441）。
//!
//! # なぜ要るのか
//!
//! 4-436 で、**交線を出す側は 5 つとも潰れました**——上限・種・
//! 重複判定・`march`・濾過。**それでも輪は閉じません**（浮いた端 4 つ）。
//!
//! **断り文にも、同じ点が出ます。**
//!
//! ```text
//! the cut is not a closed loop: (11.6325 -2.9470 -4.0000)
//!   is used 1 time(s), not 2
//! ```
//!
//! **繋ぐ段まで来て、そこで開いている**のです。
//!
//! # 何を測るか
//!
//! **浮いた端から、その組の 2 枚のパッチの上を歩かせます。**
//!
//! * **歩けるなら**——**交線はそこにあり、種が置かれていない**だけ
//! * **歩けないなら**——**その組には本当に交線が無く**、続きは
//!   **隣の組**にあります（＝**継ぎ目を渡る段**が要る）
//!
//! **4-412 は「両方の面に乗っている」までしか見ていません。**
//! **乗っていることと、そこに交線が通っていることは別**です。
//!
//! # 使い方
//!
//! ```bash
//! cargo run --release -p zenith_algo --example seam_loose_end_probe
//! ```
//!
//! **これは診断です。赤にはしません。**

use zenith_algo::{BrepIntersectionBuilder, BrepTransform, PrimitiveBuilder};
use zenith_geom::{ExtremumEngine, IntersectionMarcher};
use zenith_math::{Point3, Tolerance, Transform3, Vec3};
use zenith_topo::{FaceGeometry, Solid};

/// その点が、面のどのパッチに乗っているか（1e-6 以内）。
fn faces_carrying(point: Point3, solid: &Solid, tol: &Tolerance) -> Vec<usize> {
    let mut out = Vec::new();
    for (index, face) in solid.outer_shell.faces.iter().enumerate() {
        if let FaceGeometry::Nurbs(nurbs) = &face.geometry {
            if let Ok(projection) =
                ExtremumEngine::point_to_surface(point, nurbs, 32, tol.parametric)
            {
                if projection.distance <= 1e-6 {
                    out.push(index);
                }
            }
        }
    }
    out
}

fn main() {
    let tol = Tolerance::default();

    // **`seam_torus_wall_probe` と、同じ 2 つ**です。値を変えたら、
    // あちらも変えてください。
    let torus = PrimitiveBuilder::make_torus(12.0, 4.0).expect("torus");
    let spin = Transform3::from_axis_angle(&Vec3::new(0.0, 0.0, 1.0), 29f64.to_radians());
    let tip = Transform3::from_axis_angle(&Vec3::new(1.0, 0.0, 0.0), 61f64.to_radians());
    let a = BrepTransform::transform_solid(&torus, &spin).expect("turn");
    let b = BrepTransform::translate_solid(
        &BrepTransform::transform_solid(&torus, &tip).expect("tip"),
        Vec3::new(18.0, 0.0, 0.0),
    );

    println!("浮いた端から、交線を歩かせてみる（4-441）");
    println!();

    // ---- 浮いた端を、その場で数える ----
    use std::collections::BTreeMap;
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
    let loose: Vec<Point3> = uses
        .values()
        .filter(|(count, _)| *count != 2)
        .map(|(_, point)| *point)
        .collect();

    println!("交線 {} 本、浮いた端 {} 個", candidates.len(), loose.len());
    println!();

    for point in &loose {
        let on_a = faces_carrying(*point, &a, &tol);
        let on_b = faces_carrying(*point, &b, &tol);
        println!(
            "端 ({:.4}, {:.4}, {:.4})   A面 {:?} / B面 {:?}",
            point.x, point.y, point.z, on_a, on_b
        );

        // **その端が乗っている A と B の、すべての組で歩かせます。**
        for &ai in &on_a {
            for &bi in &on_b {
                let (FaceGeometry::Nurbs(sa), FaceGeometry::Nurbs(sb)) = (
                    &a.outer_shell.faces[ai].geometry,
                    &b.outer_shell.faces[bi].geometry,
                ) else {
                    continue;
                };
                // **端をその面へ射影して、そこから歩き出します。**
                let Ok(projection) = ExtremumEngine::point_to_surface(*point, sa, 64, 1e-13) else {
                    println!("    A面{ai} × B面{bi}: 射影できません");
                    continue;
                };
                let extent = 22.6_f64;
                match IntersectionMarcher::march(sa, sb, projection.u, projection.v, extent * 0.02, 2048, &tol) {
                    Some(marched) => {
                        let first = marched.points.first().map(|s| s.point);
                        let last = marched.points.last().map(|s| s.point);
                        let (Some(first), Some(last)) = (first, last) else {
                            println!("    A面{ai} × B面{bi}: 点が足りません");
                            continue;
                        };
                        let length: f64 = marched
                            .points
                            .windows(2)
                            .map(|pair| (pair[1].point - pair[0].point).norm())
                            .sum();
                        println!(
                            "    A面{ai} × B面{bi}: **歩けました** {} 点、長さ {:.4}   ({:.3},{:.3},{:.3}) -> ({:.3},{:.3},{:.3})",
                            marched.points.len(), length,
                            first.x, first.y, first.z, last.x, last.y, last.z
                        );
                    }
                    None => println!("    A面{ai} × B面{bi}: **歩けません**"),
                }
            }
        }
        println!();
    }

    println!("**歩けたなら、交線はそこにあり、種が置かれていないだけ**です。");
    println!("**歩けないなら、その組には本当に交線が無く、続きは隣の組**にあります。");
    println!();

    // ---- **閉じた輪という不変量で、拾い直す**（4-441）----
    //
    // **種を増やすのではありません**（4-436 で尽きました）。
    // **「どの端点もちょうど 2 回」が破れている所だけ**、
    // **その端から歩いて、足りない交線を拾います。**
    //
    // **費用は、破れているときだけ**かかります。
    println!("**閉じた輪という不変量で、拾い直す**");
    println!();

    let mut added = 0usize;
    let mut known: Vec<(Point3, Point3)> = candidates
        .iter()
        .map(|c| (c.edge.start_vertex.point, c.edge.end_vertex.point))
        .collect();

    for round in 1..=4 {
        // いまの浮いた端
        let mut uses: BTreeMap<_, (usize, Point3)> = BTreeMap::new();
        for (start, end) in &known {
            for point in [*start, *end] {
                let entry = uses.entry(key(point)).or_insert((0, point));
                entry.0 += 1;
            }
        }
        let loose: Vec<Point3> = uses
            .values()
            .filter(|(count, _)| *count != 2)
            .map(|(_, point)| *point)
            .collect();
        if loose.is_empty() {
            println!("  {round} 回目: **浮いた端 0。閉じました。**");
            break;
        }
        println!("  {round} 回目: 浮いた端 {} 個", loose.len());

        let mut gained = 0usize;
        for point in &loose {
            for &ai in &faces_carrying(*point, &a, &tol) {
                for &bi in &faces_carrying(*point, &b, &tol) {
                    let (FaceGeometry::Nurbs(sa), FaceGeometry::Nurbs(sb)) = (
                        &a.outer_shell.faces[ai].geometry,
                        &b.outer_shell.faces[bi].geometry,
                    ) else {
                        continue;
                    };
                    let Ok(projection) = ExtremumEngine::point_to_surface(*point, sa, 64, 1e-13)
                    else {
                        continue;
                    };
                    let Some(marched) = IntersectionMarcher::march(
                        sa, sb, projection.u, projection.v, 22.6 * 0.02, 2048, &tol,
                    ) else {
                        continue;
                    };
                    if marched.points.len() < 4 {
                        continue;
                    }
                    let first = marched.points.first().unwrap().point;
                    let last = marched.points.last().unwrap().point;
                    if (last - first).norm() <= tol.linear {
                        continue;
                    }
                    // **もう持っているか**——端の組で見ます（向きは問いません）。
                    let same = |x: Point3, y: Point3| (x - y).norm() <= 1e-5;
                    if known.iter().any(|(s, e)| {
                        (same(*s, first) && same(*e, last)) || (same(*s, last) && same(*e, first))
                    }) {
                        continue;
                    }
                    known.push((first, last));
                    gained += 1;
                    println!(
                        "      足しました A面{ai} × B面{bi}: ({:.3},{:.3},{:.3}) -> ({:.3},{:.3},{:.3})",
                        first.x, first.y, first.z, last.x, last.y, last.z
                    );
                }
            }
        }
        added += gained;
        if gained == 0 {
            println!("      **1 本も増えません。ここで止まります。**");
            break;
        }
    }
    let repaired = known.len();
    println!();
    println!("交線 {} 本 -> **{repaired} 本**（{added} 本 足した）", candidates.len());
}
