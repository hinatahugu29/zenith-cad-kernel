//! 面と稜を、実形状の数まで減らせるか。
//!
//! 穴あけやブーリアンは平面を扇形・短冊に割ってから組み直すので、割った跡が
//! そのまま残ります。実形状に無い面と稜が選択肢に並び、STEP のエンティティが
//! 倍になり、平面しか受け付けない演算に無関係な候補が混じります。
//!
//! ここで測るのは「減ったか」だけではありません。**体積が動いていないこと**、
//! **閉じたままであること**、そして減った結果 **本当に使えるようになったこと**
//! （フィレットが掛かるようになる）を同時に見ます。

use zenith_algo::{
    BooleanEngine, BooleanOpType, BrepTransform, EdgeBlender, FaceMerger, HoleBuilder,
    MassCalculator, PrimitiveBuilder,
};
use zenith_math::{Tolerance, Vec3};
use zenith_tess::TessellationParams;
use zenith_topo::{FaceGeometry, Solid};

fn volume_of(solid: &Solid) -> f64 {
    MassCalculator::compute_from_brep(
        solid,
        &TessellationParams {
            u_divisions: 64,
            v_divisions: 64,
        },
    )
    .volume
}

fn edge_count(solid: &Solid) -> usize {
    let mut ids: Vec<u64> = Vec::new();
    for face in &solid.outer_shell.faces {
        for wire in std::iter::once(&face.outer_wire).chain(face.inner_wires.iter()) {
            for oriented in &wire.edges {
                if !ids.contains(&oriented.edge.id) {
                    ids.push(oriented.edge.id);
                }
            }
        }
    }
    ids.len()
}

#[test]
fn a_solid_with_nothing_to_merge_comes_back_untouched() {
    let tol = Tolerance::default();
    for solid in [
        PrimitiveBuilder::make_box(20.0, 30.0, 40.0).unwrap(),
        PrimitiveBuilder::make_regular_prism(6, 10.0, 25.0).unwrap(),
    ] {
        let before = volume_of(&solid);
        let (simplified, report) = FaceMerger::simplify_solid(&solid, &tol).expect("simplify");

        assert_eq!(
            report.faces_before, report.faces_after,
            "{}",
            report.summary()
        );
        assert_eq!(report.merged_groups, 0, "{}", report.summary());
        assert!(
            (volume_of(&simplified) - before).abs() / before < 1e-15,
            "simplifying moved the volume"
        );
    }
}

#[test]
fn the_split_up_faces_of_a_boolean_result_come_back_as_one_each() {
    let tol = Tolerance::default();
    let block = PrimitiveBuilder::make_box(40.0, 40.0, 20.0).unwrap();
    let corner = BrepTransform::translate_solid(
        &PrimitiveBuilder::make_box(20.0, 20.0, 20.0).unwrap(),
        Vec3::new(20.0, 20.0, 0.0),
    );
    let l_prism =
        BooleanEngine::boolean_solids_exact(&block, &corner, BooleanOpType::Difference, &tol)
            .expect("difference");

    assert_eq!(
        l_prism.outer_shell.faces.len(),
        14,
        "this test needs the boolean to still hand back split faces"
    );
    let before = volume_of(&l_prism);

    let (simplified, report) = FaceMerger::simplify_solid(&l_prism, &tol).expect("simplify");

    // L 字角柱は側面6枚 + 上下面2枚、稜は 6 x 3 = 18 本
    assert_eq!(
        simplified.outer_shell.faces.len(),
        8,
        "an L shaped prism has eight faces: {}",
        report.summary()
    );
    assert_eq!(edge_count(&simplified), 18, "{}", report.summary());
    assert!(
        simplified
            .outer_shell
            .validate_closed(&tol)
            .is_valid(),
        "simplifying left an invalid shell"
    );
    assert!(
        (volume_of(&simplified) - before).abs() / before < 1e-13,
        "simplifying moved the volume: {} against {before}",
        volume_of(&simplified)
    );
}

#[test]
fn a_drilled_box_is_filletable_and_simplifies_to_ten_faces() {
    // `make_drilled_box` は 16 面すべてを NURBS で持っていた。平面しか受け
    // 付けない演算はどれも掛からず、フィレットの候補は 0 本だった。いまは
    // ビルダーの共通出口（`validated_solid`）が平面を平面として持ち直す。
    let tol = Tolerance::default();
    let drilled = HoleBuilder::make_drilled_box(40.0, 40.0, 20.0, 8.0).unwrap();

    assert_eq!(drilled.outer_shell.faces.len(), 16);
    let planes = drilled
        .outer_shell
        .faces
        .iter()
        .filter(|face| matches!(face.geometry, FaceGeometry::Plane(_)))
        .count();
    assert_eq!(
        planes, 12,
        "the twelve flat faces must be held as planes, not as NURBS"
    );
    // 平面として持つだけでは足りない。上下面が扇形に割れたままなので、
    // 稜の端の頂点に3枚目の面が1枚に定まらず、まだ1本も丸められない。
    assert_eq!(
        EdgeBlender::blendable_edges(&drilled).len(),
        0,
        "split up faces still block every edge"
    );
    let before = volume_of(&drilled);

    let (simplified, report) = FaceMerger::simplify_solid(&drilled, &tol).expect("simplify");

    // 側面4 + 環状の上下面2 + 円筒の4分割4
    assert_eq!(
        simplified.outer_shell.faces.len(),
        10,
        "{}",
        report.summary()
    );
    assert!(
        simplified
            .outer_shell
            .validate_closed(&tol)
            .is_valid(),
        "simplifying left an invalid shell"
    );
    assert!(
        (volume_of(&simplified) - before).abs() / before < 1e-12,
        "simplifying moved the volume: {} against {before}",
        volume_of(&simplified)
    );

    // 上下面は穴を内側ワイヤとして持つ1枚になっている
    let with_holes = simplified
        .outer_shell
        .faces
        .iter()
        .filter(|face| !face.inner_wires.is_empty())
        .count();
    assert_eq!(with_holes, 2, "the two mouths become one face each with a hole");

    // そして、丸められるようになっている
    let blendable = EdgeBlender::blendable_edges(&simplified);
    assert_eq!(
        blendable.len(),
        12,
        "the twelve outer edges of the block should be blendable now"
    );

    let radius = 2.0;
    let filleted = EdgeBlender::fillet_edge(&simplified, blendable[0].edge_id, radius)
        .expect("filleting a simplified drilled box");
    assert!(
        filleted
            .outer_shell
            .validate_closed(&tol)
            .is_valid(),
        "the fillet left an invalid shell"
    );
    let removed = before - volume_of(&filleted);
    let expected =
        blendable[0].length * radius * radius * (1.0 - std::f64::consts::FRAC_PI_4);
    assert!(
        (removed - expected).abs() / expected < 1e-9,
        "the fillet removed {removed} against {expected}"
    );
}

#[test]
fn a_planar_face_kept_as_nurbs_is_recognised_without_moving_it() {
    // ビルダーの出口で直すようになったので、素材は手で作る。ここで測るのは
    // 「制御点が同一平面に乗る NURBS 面を平面として見抜けるか」と、
    // 「曲面を平面と取り違えないか」の2つ。
    let tol = Tolerance::default();

    // 直方体の6面を、すべて 1 x 1 次の NURBS パッチとして持ち直したもの
    let boxed = PrimitiveBuilder::make_box(20.0, 30.0, 40.0).unwrap();
    let disguised_faces: Vec<zenith_topo::Face> = boxed
        .outer_shell
        .faces
        .iter()
        .map(|face| {
            let zenith_topo::FaceGeometry::Plane(plane) = &face.geometry else {
                panic!("a box should be made of planes");
            };
            let corner = |u: f64, v: f64| plane.origin + plane.u_axis * u + plane.v_axis * v;
            let rows = vec![
                vec![
                    zenith_geom::ControlPoint3::unweighted(corner(-100.0, -100.0)),
                    zenith_geom::ControlPoint3::unweighted(corner(-100.0, 100.0)),
                ],
                vec![
                    zenith_geom::ControlPoint3::unweighted(corner(100.0, -100.0)),
                    zenith_geom::ControlPoint3::unweighted(corner(100.0, 100.0)),
                ],
            ];
            let surface = zenith_geom::NurbsSurface3::new(
                1,
                1,
                rows,
                zenith_geom::KnotVector::clamped_uniform(2, 1),
                zenith_geom::KnotVector::clamped_uniform(2, 1),
            )
            .expect("a flat NURBS patch");
            zenith_topo::Face::new(
                zenith_topo::FaceGeometry::Nurbs(surface),
                face.outer_wire.clone(),
                face.inner_wires.clone(),
                face.orientation,
                face.tolerance,
            )
        })
        .collect();

    let disguised = zenith_topo::Solid::new(zenith_topo::Shell::closed(disguised_faces), vec![]);
    assert!(
        disguised
            .outer_shell
            .faces
            .iter()
            .all(|face| matches!(face.geometry, FaceGeometry::Nurbs(_))),
        "the subject must start out entirely as NURBS"
    );
    let before = volume_of(&disguised);

    let (flat, converted) = FaceMerger::planarize(&disguised, &tol).expect("planarize");

    assert_eq!(converted, 6, "all six flat patches should be recognised");
    assert_eq!(
        flat.outer_shell.faces.len(),
        6,
        "planarizing changes how a face is held, not how many there are"
    );
    assert!(
        (volume_of(&flat) - before).abs() / before.abs() < 1e-12,
        "planarizing moved the volume: {} against {before}",
        volume_of(&flat)
    );
    assert!(
        flat.outer_shell.validate_closed(&tol).is_valid(),
        "planarizing left an invalid shell"
    );

    // 曲面は取り違えない
    let cylinder = PrimitiveBuilder::make_cylinder(6.0, 15.0).unwrap();
    let curved_before = cylinder
        .outer_shell
        .faces
        .iter()
        .filter(|face| matches!(face.geometry, FaceGeometry::Nurbs(_)))
        .count();
    let (untouched, converted) = FaceMerger::planarize(&cylinder, &tol).expect("planarize");
    assert_eq!(converted, 0, "a cylinder wall is not a plane");
    assert_eq!(
        untouched
            .outer_shell
            .faces
            .iter()
            .filter(|face| matches!(face.geometry, FaceGeometry::Nurbs(_)))
            .count(),
        curved_before
    );
}
