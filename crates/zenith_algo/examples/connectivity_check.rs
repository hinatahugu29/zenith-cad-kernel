//! ブーリアンが返した立体が、本当に1つの塊か。
//!
//! 穴あきの板をスロットで分断すると、`ExactBooleanResult` は **1 solid** を
//! 返しました。体積は正しいのですが、その切り方は板を2つに割ります。
//! 体積は発散定理で両方を足すので、**非連結なものを1つとして返しても
//! 正しい値が出ます**。位相のほうを測らないと分かりません。
//!
//! ここでは面を辺で繋いで塊を数えます。

use std::collections::{HashMap, HashSet};

use zenith_algo::{
    BooleanEngine, BooleanOpType, BrepTransform, HoleBuilder, MassCalculator, PrimitiveBuilder,
};
use zenith_math::{Point3, Tolerance, Vec3};
use zenith_tess::TessellationParams;
use zenith_topo::{Face, Solid};

fn params() -> TessellationParams {
    TessellationParams {
        u_divisions: 24,
        v_divisions: 24,
    }
}

/// 点を格子に丸めた鍵。辺の同一性を座標で見るため。
fn key(point: Point3, tol: f64) -> (i64, i64, i64) {
    let round = |value: f64| (value / tol).round() as i64;
    (round(point.x), round(point.y), round(point.z))
}

/// 面が辺を共有しているかで繋いで、塊の数を数える。
fn connected_components(faces: &[Face], tol: f64) -> usize {
    // 辺の両端の鍵の組を、その辺を使っている面の一覧に写す。
    let mut edge_users: HashMap<((i64, i64, i64), (i64, i64, i64)), Vec<usize>> = HashMap::new();
    for (index, face) in faces.iter().enumerate() {
        for wire in std::iter::once(&face.outer_wire).chain(face.inner_wires.iter()) {
            for edge in &wire.edges {
                let (start, end) = edge.edge.curve.param_range();
                let a = key(edge.edge.curve.evaluate(start), tol);
                let b = key(edge.edge.curve.evaluate(end), tol);
                let pair = if a <= b { (a, b) } else { (b, a) };
                edge_users.entry(pair).or_default().push(index);
            }
        }
    }

    let mut neighbours: Vec<Vec<usize>> = vec![Vec::new(); faces.len()];
    for users in edge_users.values() {
        for (position, left) in users.iter().enumerate() {
            for right in users.iter().skip(position + 1) {
                if left != right {
                    neighbours[*left].push(*right);
                    neighbours[*right].push(*left);
                }
            }
        }
    }

    let mut seen: HashSet<usize> = HashSet::new();
    let mut components = 0;
    for start in 0..faces.len() {
        if seen.contains(&start) {
            continue;
        }
        components += 1;
        let mut stack = vec![start];
        seen.insert(start);
        while let Some(index) = stack.pop() {
            for next in &neighbours[index] {
                if seen.insert(*next) {
                    stack.push(*next);
                }
            }
        }
    }
    components
}

fn report(name: &str, solid: &Solid, tol: f64) {
    let components = connected_components(&solid.outer_shell.faces, tol);
    let volume = MassCalculator::compute_from_brep(solid, &params()).volume;
    println!(
        "  {name:<34} {} face(s), {} connected piece(s), volume {volume:.4}{}",
        solid.outer_shell.faces.len(),
        components,
        if components > 1 {
            "   <- one Solid holding several bodies"
        } else {
            ""
        }
    );
}

fn main() {
    let tol = Tolerance::default();
    let grid = tol.linear.max(1e-9);

    // 参照。どれも1つの塊のはず。
    println!("solids that should be one piece:");
    report(
        "a plain box",
        &PrimitiveBuilder::make_box(30.0, 30.0, 15.0).expect("box"),
        grid,
    );
    report(
        "a drilled box",
        &HoleBuilder::make_drilled_box(30.0, 30.0, 15.0, 5.0).expect("drilled"),
        grid,
    );

    println!();
    println!("a slot that cuts the plate in two:");
    let drilled = HoleBuilder::make_drilled_box(30.0, 30.0, 15.0, 5.0).expect("drilled");
    let slot = BrepTransform::translate_solid(
        &PrimitiveBuilder::make_box(60.0, 6.0, 40.0).expect("slot"),
        Vec3::new(-15.0, 12.0, -10.0),
    );
    match BooleanEngine::boolean_solids_exact_result(
        &drilled,
        &slot,
        BooleanOpType::Difference,
        &tol,
    ) {
        Ok(result) => {
            println!("  the result carries {} solid(s)", result.solids.len());
            for (index, solid) in result.solids.iter().enumerate() {
                report(&format!("solid {index}"), solid, grid);
            }
            let total: f64 = result
                .solids
                .iter()
                .map(|s| MassCalculator::compute_from_brep(s, &params()).volume)
                .sum();
            println!("  total volume {total:.4}  (expected 10464.54)");
        }
        Err(err) => println!("  ERROR {err}"),
    }

    println!();
    println!("A Solid holding two bodies measures correctly, because the divergence");
    println!("theorem adds both. Only the topology says otherwise.");
}
