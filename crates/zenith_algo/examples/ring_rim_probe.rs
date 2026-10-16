//! `tools/blender_workflow_test.py` の8段目と同じワッシャー形
//! （スケッチ回転）を作り、EdgeBlender::blendability を稜ごとに試して
//! 通る/断られる理由を印字する（4-759 の続き、まだ直していない段階の
//! 調べもの）。

use zenith_algo::{revolve_sketch, EdgeBlender, SketchSolver, WorkPlane};
use zenith_math::{Point2, Tolerance};

fn main() {
    let tol = Tolerance::default();
    let mut solver = SketchSolver::new();
    let p0 = solver.add_point(5.0, 0.0);
    let p1 = solver.add_point(8.0, 0.0);
    let p2 = solver.add_point(8.0, 3.0);
    let p3 = solver.add_point(5.0, 3.0);
    solver.add_line(p0, p1);
    solver.add_line(p1, p2);
    solver.add_line(p2, p3);
    solver.add_line(p3, p0);

    let solid = revolve_sketch(
        &solver,
        &WorkPlane::xy(),
        Point2::new(0.0, 0.0),
        Point2::new(0.0, 1.0),
        &tol,
    )
    .expect("revolve washer");

    println!("面 {} 枚", solid.outer_shell.faces.len());
    let mut ids: Vec<u64> = Vec::new();
    for face in &solid.outer_shell.faces {
        for w in std::iter::once(&face.outer_wire).chain(face.inner_wires.iter()) {
            for oe in &w.edges {
                if !ids.contains(&oe.edge.id) {
                    ids.push(oe.edge.id);
                }
            }
        }
    }
    println!("稜 {} 本", ids.len());
    let mut first_ok: Option<u64> = None;
    for id in &ids {
        match EdgeBlender::blendability(&solid, *id) {
            Ok(b) => {
                println!(
                    "  稜{id}: ok 半径上限={:.4} 面取り上限={:.4} 二面角={:.1}",
                    b.max_fillet_radius, b.max_chamfer_distance, b.dihedral_angle_deg
                );
                if first_ok.is_none() {
                    first_ok = Some(*id);
                }
            }
            Err(reason) => println!("  稜{id}: 断り — {reason}"),
        }
    }

    if let Some(id) = first_ok {
        match EdgeBlender::fillet_edge(&solid, id, 1.0) {
            Ok(filleted) => {
                println!(
                    "稜{id} を半径1.0で実際にフィレット: ok, 面 {} 枚",
                    filleted.outer_shell.faces.len()
                );
            }
            Err(reason) => println!("稜{id} のフィレット実行: 断り — {reason}"),
        }
    }
}
