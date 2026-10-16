//! `tools/blender_workflow_test.py` の8段目と同じワッシャー形
//! （スケッチ回転）を作り、EdgeBlender::blendability を稜ごとに試して
//! 通る/断られる理由を印字する（4-759 の続き）。
//!
//! 4-762 で天面・底面の併合によりフィレットが通るようになった後、
//! **フィレット済みの回転体を他の立体と和を取ると断られる**ことが
//! `tools/blender_workflow_test.py` の 9 段目で見つかった。ここでは
//! その断りを Blender を介さずに Rust だけで再現する（穴あき板＋
//! 円柱ドリルの板と、フィレット済みワッシャーの和）。

use zenith_algo::{
    revolve_sketch, BooleanEngine, BooleanOpType, BrepTransform, EdgeBlender, ExtrudeBuilder,
    PrimitiveBuilder, SketchSolver, WorkPlane,
};
use zenith_math::{Point2, Point3, Tolerance, Vec3};
use zenith_topo::{Edge, OrientedEdge, Vertex, Wire};

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

    let mut filleted_washer = None;
    if let Some(id) = first_ok {
        match EdgeBlender::fillet_edge(&solid, id, 0.3) {
            Ok(filleted) => {
                println!(
                    "稜{id} を半径0.3で実際にフィレット: ok, 面 {} 枚",
                    filleted.outer_shell.faces.len()
                );
                filleted_washer = Some(filleted);
            }
            Err(reason) => println!("稜{id} のフィレット実行: 断り — {reason}"),
        }
    }

    let Some(filleted_washer) = filleted_washer else {
        return;
    };

    // tools/blender_workflow_test.py と同じ、穴あき板 + 円柱ドリル。
    let plate_pts = [
        Point3::new(0.0, 0.0, 0.0),
        Point3::new(20.0, 0.0, 0.0),
        Point3::new(20.0, 10.0, 0.0),
        Point3::new(0.0, 10.0, 0.0),
    ];
    let hole_pts = [
        Point3::new(7.0, 2.0, 0.0),
        Point3::new(13.0, 2.0, 0.0),
        Point3::new(13.0, 8.0, 0.0),
        Point3::new(7.0, 8.0, 0.0),
    ];
    let make_wire = |pts: &[Point3]| {
        let vertices: Vec<Vertex> = pts.iter().map(|p| Vertex::from_point(*p)).collect();
        let n = vertices.len();
        let edges = (0..n)
            .map(|i| {
                OrientedEdge::forward(
                    Edge::line_between(vertices[i].clone(), vertices[(i + 1) % n].clone())
                        .unwrap(),
                )
            })
            .collect();
        Wire::new(edges)
    };
    let outer = make_wire(&plate_pts);
    let inner = make_wire(&hole_pts);
    let plate = ExtrudeBuilder::extrude_face_with_holes(&outer, &[inner], Vec3::new(0.0, 0.0, 5.0), &tol)
        .expect("extrude plate with hole");
    let drill = zenith_algo::BrepTransform::translate_solid(
        &PrimitiveBuilder::make_cylinder(2.0, 10.0).expect("cylinder"),
        Vec3::new(10.0, 5.0, -2.5),
    );
    let drilled = BooleanEngine::boolean_solids_exact(&plate, &drill, BooleanOpType::Difference, &tol)
        .expect("drill plate");

    let washer_moved = BrepTransform::translate_solid(&filleted_washer, Vec3::new(0.0, 0.0, 5.0));
    match BooleanEngine::boolean_solids_exact(&drilled, &washer_moved, BooleanOpType::Union, &tol) {
        Ok(stacked) => println!("積み重ねたブーリアン: ok、面 {} 枚", stacked.outer_shell.faces.len()),
        Err(reason) => println!("積み重ねたブーリアン: 断り — {reason}"),
    }
}
