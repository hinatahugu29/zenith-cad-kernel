//! 同じ向きに2度使われている辺を名指しする。
//!
//! 穴あきの板を箱で削ると、シェルは閉じて多様体なのに **16 の同方向辺使用**
//! が残って組み立てが止まります。数だけでは何が起きているか分かりません。
//! 引継書 4-4 は「向きの誤り」がどう見えるかを教えてくれますが、**どの面が
//! 裏返っているか**は、辺ごとに使い手を並べないと分かりません。
//!
//! 同じ稜は分割の過程で別々の `Edge` として作られるので、実体ではなく
//! **位置と向き**で集計します。

use std::collections::BTreeMap;

use zenith_algo::{
    BooleanOpType, BrepIntersectionBuilder, BrepTransform, HoleBuilder, MassCalculator,
    PrimitiveBuilder,
};
use zenith_math::{Point3, Tolerance, Vec3};
use zenith_tess::TessellationParams;
use zenith_topo::FaceGeometry;

fn main() {
    let tol = Tolerance::default();
    let grid = tol.linear.max(1e-9);
    let key = |point: Point3| {
        (
            (point.x / grid).round() as i64,
            (point.y / grid).round() as i64,
            (point.z / grid).round() as i64,
        )
    };

    let drilled = HoleBuilder::make_drilled_box(30.0, 30.0, 15.0, 5.0).expect("drilled");
    let slab = BrepTransform::translate_solid(
        &PrimitiveBuilder::make_box(60.0, 60.0, 10.0).expect("slab"),
        Vec3::new(-15.0, -15.0, 12.0),
    );

    // まず、**入力そのもの**を同じ物差しで見る。ブーリアンが揃えられないのか、
    // 元から揃っていないのかは、ここでしか分かれません。
    {
        let mut uses: BTreeMap<((i64, i64, i64), (i64, i64, i64)), Vec<usize>> = BTreeMap::new();
        for (index, face) in drilled.outer_shell.faces.iter().enumerate() {
            for wire in std::iter::once(&face.outer_wire).chain(face.inner_wires.iter()) {
                for edge in &wire.edges {
                    let (start, end) = edge.edge.curve.param_range();
                    let (mut a, mut b) = (
                        key(edge.edge.curve.evaluate(start)),
                        key(edge.edge.curve.evaluate(end)),
                    );
                    if !edge.orientation.is_forward() {
                        std::mem::swap(&mut a, &mut b);
                    }
                    uses.entry((a, b)).or_default().push(index);
                }
            }
        }
        let same: usize = uses.values().filter(|users| users.len() > 1).map(|u| u.len()).sum();
        let unmatched = uses
            .iter()
            .filter(|((a, b), users)| users.len() == 1 && !uses.contains_key(&(*b, *a)))
            .count();
        println!(
            "=== the drilled box as built: {} faces, {} same-direction edge uses, {} unmatched",
            drilled.outer_shell.faces.len(),
            same,
            unmatched
        );
        println!(
            "    shell validates closed: {}",
            drilled.outer_shell.validate_closed(&tol).is_valid()
        );
        println!();
    }

    // 分割の前後で、同じ円弧の向きが変わっていないか。変わっていれば、
    // 隣の無傷な面と噛み合わなくなる。
    {
        println!("=== the z = 0 arcs, before and after the split");
        let arc_direction = |faces: &[zenith_topo::Face], reversed: Option<&[bool]>| {
            let mut found = Vec::new();
            for (index, face) in faces.iter().enumerate() {
                for wire in std::iter::once(&face.outer_wire).chain(face.inner_wires.iter()) {
                    for edge in &wire.edges {
                        let (start, end) = edge.edge.curve.param_range();
                        let p0 = edge.edge.curve.evaluate(start);
                        let p1 = edge.edge.curve.evaluate(end);
                        if p0.z.abs() > 1e-6 || p1.z.abs() > 1e-6 {
                            continue;
                        }
                        let radius = ((p0.x - 15.0).powi(2) + (p0.y - 15.0).powi(2)).sqrt();
                        if (radius - 5.0).abs() > 1e-6 {
                            continue;
                        }
                        let mut forward = edge.orientation.is_forward();
                        if let Some(flags) = reversed {
                            if flags[index] {
                                forward = !forward;
                            }
                        }
                        let (a, b) = if forward { (p0, p1) } else { (p1, p0) };
                        found.push((index, a, b));
                    }
                }
            }
            found
        };

        for (index, a, b) in arc_direction(&drilled.outer_shell.faces, None) {
            println!(
                "  input  face {index:<3} arc ({:.2},{:.2}) -> ({:.2},{:.2})",
                a.x, a.y, b.x, b.y
            );
        }
        println!();
    }

    let assembly = BrepIntersectionBuilder::collect_boolean_shell_assembly(
        &drilled,
        &slab,
        BooleanOpType::Difference,
        &tol,
    );

    for (label, pieces, report) in [
        (
            "selection",
            &assembly.selection.selected_face_pieces,
            &assembly.selection.stitch_report,
        ),
        (
            "with caps",
            &assembly.assembly.selected_face_pieces,
            &assembly.assembly.stitch_report,
        ),
    ] {
        println!(
            "=== {label}: {} pieces, {} unmatched, {} non-manifold, {} same-direction",
            pieces.len(),
            report.unmatched_edge_use_count,
            report.non_manifold_edge_use_count,
            report.same_direction_edge_use_count
        );

        let params = TessellationParams {
            u_divisions: 16,
            v_divisions: 16,
        };

        // 辺の使用を、位置と向きの組で集計する。
        let mut uses: BTreeMap<((i64, i64, i64), (i64, i64, i64)), Vec<usize>> = BTreeMap::new();
        for (index, piece) in pieces.iter().enumerate() {
            let face = &piece.face;
            for wire in std::iter::once(&face.outer_wire).chain(face.inner_wires.iter()) {
                for edge in &wire.edges {
                    let (start, end) = edge.edge.curve.param_range();
                    let (mut a, mut b) = (
                        key(edge.edge.curve.evaluate(start)),
                        key(edge.edge.curve.evaluate(end)),
                    );
                    // ワイヤの巡回向きと、面の表裏の指定を両方反映する。
                    let mut forward = edge.orientation.is_forward();
                    if piece.reverse_orientation {
                        forward = !forward;
                    }
                    if !forward {
                        std::mem::swap(&mut a, &mut b);
                    }
                    uses.entry((a, b)).or_default().push(index);
                }
            }
        }

        let mut offenders: BTreeMap<usize, usize> = BTreeMap::new();
        let mut shown = 0;
        for ((a, b), users) in &uses {
            if users.len() < 2 {
                continue;
            }
            for user in users {
                *offenders.entry(*user).or_default() += 1;
            }
            if shown < 12 {
                let midpoint = Point3::new(
                    (a.0 + b.0) as f64 * grid * 0.5,
                    (a.1 + b.1) as f64 * grid * 0.5,
                    (a.2 + b.2) as f64 * grid * 0.5,
                );
                println!(
                    "  edge near ({:.3}, {:.3}, {:.3}) used {} times in the same direction by {:?}",
                    midpoint.x,
                    midpoint.y,
                    midpoint.z,
                    users.len(),
                    users
                );
                shown += 1;
            }
        }

        if offenders.is_empty() {
            println!("  no edge is used twice in the same direction");
        } else {
            println!("  pieces involved:");
            for (index, count) in &offenders {
                let piece = &pieces[*index];
                let kind = match &piece.face.geometry {
                    FaceGeometry::Plane(_) => "plane",
                    FaceGeometry::Nurbs(_) => "nurbs",
                    _ => "other",
                };
                let (area, _) = MassCalculator::compute_face_integral(&piece.face, &params);
                println!(
                    "    piece {index:<3} {:?} {kind:<6} area {area:>10.4} reversed {:<5} in {count} bad edge(s)",
                    piece.operand, piece.reverse_orientation
                );
            }
        }
        println!();
    }

    {
        println!("=== the same arcs after selection");
        let pieces = &assembly.selection.selected_face_pieces;
        for (index, piece) in pieces.iter().enumerate() {
            let face = &piece.face;
            for wire in std::iter::once(&face.outer_wire).chain(face.inner_wires.iter()) {
                for edge in &wire.edges {
                    let (start, end) = edge.edge.curve.param_range();
                    let p0 = edge.edge.curve.evaluate(start);
                    let p1 = edge.edge.curve.evaluate(end);
                    if p0.z.abs() > 1e-6 || p1.z.abs() > 1e-6 {
                        continue;
                    }
                    let radius = ((p0.x - 15.0).powi(2) + (p0.y - 15.0).powi(2)).sqrt();
                    if (radius - 5.0).abs() > 1e-6 {
                        continue;
                    }
                    let mut forward = edge.orientation.is_forward();
                    if piece.reverse_orientation {
                        forward = !forward;
                    }
                    let (a, b) = if forward { (p0, p1) } else { (p1, p0) };
                    println!(
                        "  piece  {index:<3} arc ({:.2},{:.2}) -> ({:.2},{:.2})",
                        a.x, a.y, b.x, b.y
                    );
                }
            }
        }
    }
}
