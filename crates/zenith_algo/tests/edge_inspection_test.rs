//! 稜の凸凹と二面角が、面の並び順や稜の格納向きに依らず同じ答えを出すか。
//!
//! 判定に「2面の法線の外積と稜の接線の向き」を使っていたときは、同じ形でも
//! 面が列挙される順番が入れ替わるだけで凸と凹が反転していました。ここは
//! 立体の形だけで決まる答えが返ることを測ります。
//!
//! 併せて、二面角を**材料の側から**測っているかも見ます。法線どうしの角度
//! では、直方体の外角 (90 度) と切り欠きの内角 (270 度) が区別できません。

use zenith_algo::{
    BooleanEngine, BooleanOpType, BrepTransform, DirectModeling, EdgeKind, HoleBuilder,
    PrimitiveBuilder,
};
use zenith_math::{Tolerance, Vec3};
use zenith_topo::Solid;

fn all_edge_ids(solid: &Solid) -> Vec<u64> {
    let mut ids = Vec::new();
    for face in &solid.outer_shell.faces {
        for wire in std::iter::once(&face.outer_wire).chain(face.inner_wires.iter()) {
            for oriented in &wire.edges {
                if !ids.contains(&oriented.edge.id) {
                    ids.push(oriented.edge.id);
                }
            }
        }
    }
    ids
}

#[test]
fn every_edge_of_a_box_is_convex_at_ninety_degrees() {
    let boxed = PrimitiveBuilder::make_box(10.0, 20.0, 30.0).unwrap();
    let ids = all_edge_ids(&boxed);
    assert_eq!(ids.len(), 12);

    for id in ids {
        let inspection = DirectModeling::inspect_solid_edge(&boxed, id).unwrap();
        assert_eq!(
            inspection.kind,
            EdgeKind::Convex,
            "edge {id} came back as {:?}",
            inspection.kind
        );
        let angle = inspection
            .dihedral_angle_deg
            .unwrap_or_else(|| panic!("edge {id} reported no dihedral angle"));
        assert!(
            (angle - 90.0).abs() < 1e-9,
            "edge {id} reported {angle} deg"
        );
    }
}

#[test]
fn reversing_the_order_the_faces_are_listed_in_does_not_flip_the_answer() {
    let boxed = PrimitiveBuilder::make_box(10.0, 20.0, 30.0).unwrap();
    let ids = all_edge_ids(&boxed);

    let mut flipped = boxed.clone();
    flipped.outer_shell.faces.reverse();

    for id in ids {
        let before = DirectModeling::inspect_solid_edge(&boxed, id).unwrap();
        let after = DirectModeling::inspect_solid_edge(&flipped, id).unwrap();
        assert_eq!(before.kind, after.kind, "edge {id} changed kind");
        assert!(
            (before.dihedral_angle_deg.unwrap() - after.dihedral_angle_deg.unwrap()).abs() < 1e-12,
            "edge {id} changed angle"
        );
    }
}

#[test]
fn the_inside_corner_of_a_notch_reads_as_concave_at_two_hundred_seventy_degrees() {
    let tol = Tolerance::default();
    let base = PrimitiveBuilder::make_box(40.0, 40.0, 20.0).unwrap();
    let cutter = BrepTransform::translate_solid(
        &PrimitiveBuilder::make_box(20.0, 20.0, 20.0).unwrap(),
        Vec3::new(20.0, 20.0, 0.0),
    );
    let notched =
        BooleanEngine::boolean_solids_exact(&base, &cutter, BooleanOpType::Difference, &tol)
            .expect("notched difference");

    // 切り欠きの内側の縦稜 (x=20, y=20)
    let mut inside = None;
    for id in all_edge_ids(&notched) {
        let inspection = DirectModeling::inspect_solid_edge(&notched, id).unwrap();
        let start = inspection.start_point;
        let end = inspection.end_point;
        let upright = (end.x - start.x).abs() < 1e-9 && (end.y - start.y).abs() < 1e-9;
        if upright && (start.x - 20.0).abs() < 1e-9 && (start.y - 20.0).abs() < 1e-9 {
            inside = Some(inspection);
        }
    }

    let inside = inside.expect("the notch has an inside upright edge");
    assert_eq!(inside.kind, EdgeKind::Concave, "{:?}", inside.kind);
    let angle = inside.dihedral_angle_deg.expect("angle");
    assert!(
        (angle - 270.0).abs() < 1e-9,
        "the inside corner reported {angle} deg"
    );
}

#[test]
fn the_mouth_of_a_drilled_hole_is_convex_and_its_seams_are_smooth() {
    // 穴の口（円筒と平面の交わり）は凸の 90 度。円筒を4分割している縦の
    // 継ぎ目は、同じ円筒の続きなので 180 度でなければならない。曲面の
    // 法線を面の真ん中で測っていると、この2つが区別できない。
    let drilled = HoleBuilder::make_drilled_box(40.0, 40.0, 20.0, 8.0).unwrap();

    let radius_from_axis = |point: zenith_math::Point3| {
        ((point.x - 20.0).powi(2) + (point.y - 20.0).powi(2)).sqrt()
    };

    let mut mouth_arcs = 0;
    let mut seams = 0;
    for face in &drilled.outer_shell.faces {
        for wire in std::iter::once(&face.outer_wire).chain(face.inner_wires.iter()) {
            for oriented in &wire.edges {
                let edge = &oriented.edge;
                let start = edge.start_vertex.point;
                let end = edge.end_vertex.point;
                let middle = edge.curve.evaluate(0.5);
                let on_bore = [start, end, middle]
                    .iter()
                    .all(|point| (radius_from_axis(*point) - 8.0).abs() < 1e-6);
                if !on_bore {
                    continue;
                }

                let inspection = DirectModeling::inspect_solid_edge(&drilled, edge.id).unwrap();
                let angle = inspection
                    .dihedral_angle_deg
                    .unwrap_or_else(|| panic!("edge {} on the bore got no angle", edge.id));

                if (start.z - end.z).abs() < 1e-9 {
                    // 口の円弧
                    assert_eq!(
                        inspection.kind,
                        EdgeKind::Convex,
                        "the hole mouth read {:?} at {angle} deg",
                        inspection.kind
                    );
                    assert!(
                        (angle - 90.0).abs() < 1e-6,
                        "the hole mouth reported {angle} deg"
                    );
                    mouth_arcs += 1;
                } else {
                    // 円筒の継ぎ目
                    assert_eq!(
                        inspection.kind,
                        EdgeKind::Smooth,
                        "a seam inside the bore read {:?} at {angle} deg",
                        inspection.kind
                    );
                    assert!(
                        (angle - 180.0).abs() < 1e-6,
                        "a seam inside the bore reported {angle} deg"
                    );
                    seams += 1;
                }
            }
        }
    }

    assert!(
        mouth_arcs >= 8 && seams >= 4,
        "expected both mouths and the seams to be measured, saw {mouth_arcs} arcs and {seams} seams"
    );
}
