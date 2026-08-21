//! 任意ソリッドの稜フィレット / 面取りが、閉じた式の体積に乗るか。
//!
//! これまでのフィレットは「寸法から丸めた直方体を作り直す」ビルダーで、
//! ブーリアンやプリズムの結果には掛けられませんでした。ここで測るのは
//! **既にある立体の稜を編集した結果**です。
//!
//! 二面角 θ、稜長 L に対して削れる体積は
//!
//! - フィレット: `L r^2 (cot(θ/2) - (π - θ)/2)`
//! - 面取り    : `L c^2 sin(θ) / 2`
//!
//! で決まるので、直方体 (θ=90°) でも六角柱 (θ=120°) でも
//! ブーリアンで出来た L 字 (θ=90°) でも、同じ式で照合できます。

use std::f64::consts::{FRAC_PI_4, PI};

use zenith_algo::{
    BlendKind, BooleanEngine, BooleanOpType, EdgeBlender, MassCalculator, PrimitiveBuilder,
};
use zenith_math::{Point3, Tolerance, Vec3};
use zenith_tess::TessellationParams;
use zenith_topo::Solid;

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

fn assert_closed(solid: &Solid, what: &str) {
    let report = solid.outer_shell.validate_closed(&Tolerance::default());
    assert!(report.is_valid(), "{what} left an open shell: {:?}", report.errors);
}

/// 稜の中点が `point` に最も近いブレンド可能な稜を選ぶ
fn edge_nearest(solid: &Solid, point: Point3) -> u64 {
    let candidates = EdgeBlender::blendable_edges(solid);
    assert!(!candidates.is_empty(), "no blendable edge on this solid");

    let mut best: Option<(f64, u64)> = None;
    for candidate in &candidates {
        for face in &solid.outer_shell.faces {
            for wire in std::iter::once(&face.outer_wire).chain(face.inner_wires.iter()) {
                for oriented in &wire.edges {
                    if oriented.edge.id != candidate.edge_id {
                        continue;
                    }
                    let mid = Point3::from(
                        (oriented.edge.start_vertex.point.coords
                            + oriented.edge.end_vertex.point.coords)
                            * 0.5,
                    );
                    let distance = (mid - point).norm();
                    if best.map(|(d, _)| distance < d).unwrap_or(true) {
                        best = Some((distance, candidate.edge_id));
                    }
                }
            }
        }
    }
    best.expect("no blendable edge matched").1
}

#[test]
fn every_edge_of_a_box_is_blendable_and_reports_a_right_angle() {
    let boxed = PrimitiveBuilder::make_box(20.0, 30.0, 40.0).unwrap();
    let edges = EdgeBlender::blendable_edges(&boxed);

    assert_eq!(edges.len(), 12, "a box has twelve blendable edges");
    for edge in &edges {
        assert!(
            (edge.dihedral_angle_deg - 90.0).abs() < 1e-9,
            "edge {} reported {} deg",
            edge.edge_id,
            edge.dihedral_angle_deg
        );
    }
}

#[test]
fn filleting_a_box_edge_removes_the_volume_the_closed_form_says() {
    let boxed = PrimitiveBuilder::make_box(20.0, 30.0, 40.0).unwrap();
    let base = volume_of(&boxed);
    let edge_id = edge_nearest(&boxed, Point3::new(0.0, 0.0, 20.0));

    for radius in [0.5f64, 1.0, 2.0, 5.0, 9.0] {
        let (solid, report) =
            EdgeBlender::blend_edge(&boxed, edge_id, BlendKind::Fillet { radius })
                .unwrap_or_else(|err| panic!("fillet r{radius}: {err}"));

        assert_closed(&solid, &format!("fillet r{radius}"));
        assert_eq!(solid.outer_shell.faces.len(), 7, "one edge becomes one face");

        let expected_removed = 40.0 * radius * radius * (1.0 - FRAC_PI_4);
        assert!(
            (report.predicted_removed_volume - expected_removed).abs() < 1e-12 * expected_removed,
            "r{radius}: the report says {} but the right angle form says {expected_removed}",
            report.predicted_removed_volume
        );

        let volume = volume_of(&solid);
        let expected = base - expected_removed;
        assert!(
            (volume - expected).abs() / expected < 1e-12,
            "fillet r{radius}: measured {volume} against {expected}"
        );
    }
}

#[test]
fn chamfering_a_box_edge_removes_the_triangle_the_closed_form_says() {
    let boxed = PrimitiveBuilder::make_box(20.0, 30.0, 40.0).unwrap();
    let base = volume_of(&boxed);
    let edge_id = edge_nearest(&boxed, Point3::new(0.0, 0.0, 20.0));

    for distance in [0.5f64, 1.0, 3.0, 7.5] {
        let solid = EdgeBlender::chamfer_edge(&boxed, edge_id, distance)
            .unwrap_or_else(|err| panic!("chamfer c{distance}: {err}"));

        assert_closed(&solid, &format!("chamfer c{distance}"));
        assert_eq!(solid.outer_shell.faces.len(), 7);

        let expected = base - 40.0 * distance * distance * 0.5;
        let volume = volume_of(&solid);
        assert!(
            (volume - expected).abs() / expected < 1e-12,
            "chamfer c{distance}: measured {volume} against {expected}"
        );
    }
}

#[test]
fn a_hexagonal_prism_edge_is_blended_at_its_own_dihedral_angle() {
    // 正六角柱の縦稜は 120 度。直角に固めた式では合わない。
    let prism = PrimitiveBuilder::make_regular_prism(6, 10.0, 25.0).unwrap();
    let base = volume_of(&prism);

    let edges = EdgeBlender::blendable_edges(&prism);
    assert_eq!(edges.len(), 6, "only the six upright edges qualify");
    for edge in &edges {
        assert!(
            (edge.dihedral_angle_deg - 120.0).abs() < 1e-9,
            "reported {} deg",
            edge.dihedral_angle_deg
        );
    }

    let theta = 120.0f64.to_radians();
    for radius in [0.5f64, 1.5, 3.0] {
        let (solid, report) =
            EdgeBlender::blend_edge(&prism, edges[0].edge_id, BlendKind::Fillet { radius })
                .unwrap_or_else(|err| panic!("fillet r{radius}: {err}"));
        assert_closed(&solid, &format!("hex fillet r{radius}"));

        let expected_removed =
            25.0 * radius * radius * (1.0 / (theta * 0.5).tan() - 0.5 * (PI - theta));
        assert!(
            (report.predicted_removed_volume - expected_removed).abs()
                < 1e-12 * expected_removed.abs(),
            "r{radius}: report {} against form {expected_removed}",
            report.predicted_removed_volume
        );

        let expected = base - expected_removed;
        let volume = volume_of(&solid);
        assert!(
            (volume - expected).abs() / expected < 1e-11,
            "hex fillet r{radius}: measured {volume} against {expected}"
        );
    }

    for distance in [0.5f64, 2.0, 4.0] {
        let solid = EdgeBlender::chamfer_edge(&prism, edges[0].edge_id, distance)
            .unwrap_or_else(|err| panic!("chamfer c{distance}: {err}"));
        assert_closed(&solid, &format!("hex chamfer c{distance}"));

        let expected = base - 25.0 * distance * distance * 0.5 * theta.sin();
        let volume = volume_of(&solid);
        assert!(
            (volume - expected).abs() / expected < 1e-11,
            "hex chamfer c{distance}: measured {volume} against {expected}"
        );
    }
}

#[test]
fn an_edge_created_by_a_boolean_can_be_filleted() {
    // L 字断面をブーリアンで作り、そこで初めて生まれた縦稜を丸める。
    // 旧来の「寸法から作り直す」ビルダーには渡す寸法が存在しない形。
    let tol = Tolerance::default();
    let base = PrimitiveBuilder::make_box(40.0, 40.0, 20.0).unwrap();
    let cutter = zenith_algo::BrepTransform::translate_solid(
        &PrimitiveBuilder::make_box(20.0, 20.0, 20.0).unwrap(),
        Vec3::new(20.0, 20.0, 0.0),
    );
    let l_shape =
        BooleanEngine::boolean_solids_exact(&base, &cutter, BooleanOpType::Difference, &tol)
            .expect("L shaped difference");
    assert_closed(&l_shape, "boolean difference");

    let start_volume = volume_of(&l_shape);
    assert!(
        (start_volume - (40.0 * 40.0 - 20.0 * 20.0) * 20.0).abs() < 1e-9,
        "the L shape itself is off: {start_volume}"
    );

    // 切り欠きの外側の凸な縦稜 (x=40, y=20) を丸める
    let edge_id = edge_nearest(&l_shape, Point3::new(40.0, 20.0, 10.0));
    let radius = 3.0;
    let (filleted, report) =
        EdgeBlender::blend_edge(&l_shape, edge_id, BlendKind::Fillet { radius })
            .unwrap_or_else(|err| panic!("filleting a boolean edge: {err}"));

    assert_closed(&filleted, "fillet on a boolean result");
    assert!(
        (report.dihedral_angle_deg - 90.0).abs() < 1e-9,
        "the boolean edge measured {} deg",
        report.dihedral_angle_deg
    );

    let expected = start_volume - 20.0 * radius * radius * (1.0 - FRAC_PI_4);
    let volume = volume_of(&filleted);
    assert!(
        (volume - expected).abs() / expected < 1e-11,
        "boolean fillet: measured {volume} against {expected}"
    );
}

#[test]
fn several_edges_can_be_blended_one_after_another() {
    let boxed = PrimitiveBuilder::make_box(20.0, 30.0, 40.0).unwrap();
    let base = volume_of(&boxed);

    // 頂点を共有しない4本の縦稜
    let uprights: Vec<u64> = [
        Point3::new(0.0, 0.0, 20.0),
        Point3::new(20.0, 0.0, 20.0),
        Point3::new(20.0, 30.0, 20.0),
        Point3::new(0.0, 30.0, 20.0),
    ]
    .iter()
    .map(|point| edge_nearest(&boxed, *point))
    .collect();
    assert_eq!(
        uprights.iter().collect::<std::collections::BTreeSet<_>>().len(),
        4,
        "the four uprights must be four distinct edges"
    );

    let radius = 2.5;
    let requests: Vec<(u64, f64)> = uprights.iter().map(|id| (*id, radius)).collect();
    let filleted = EdgeBlender::fillet_edges(&boxed, &requests).expect("four fillets in a row");

    assert_closed(&filleted, "four fillets");
    assert_eq!(filleted.outer_shell.faces.len(), 10, "6 + 4 new faces");

    let expected = base - 4.0 * 40.0 * radius * radius * (1.0 - FRAC_PI_4);
    let volume = volume_of(&filleted);
    assert!(
        (volume - expected).abs() / expected < 1e-11,
        "four fillets: measured {volume} against {expected}"
    );
}

#[test]
fn a_concave_edge_is_refused_rather_than_blended_the_wrong_way() {
    let tol = Tolerance::default();
    let base = PrimitiveBuilder::make_box(40.0, 40.0, 20.0).unwrap();
    let cutter = zenith_algo::BrepTransform::translate_solid(
        &PrimitiveBuilder::make_box(20.0, 20.0, 20.0).unwrap(),
        Vec3::new(20.0, 20.0, 0.0),
    );
    let l_shape =
        BooleanEngine::boolean_solids_exact(&base, &cutter, BooleanOpType::Difference, &tol)
            .expect("L shaped difference");

    // 切り欠きの内側の縦稜 (x=20, y=20) は凹。凸専用の演算子は断るべき。
    let mut concave_ids = Vec::new();
    for face in &l_shape.outer_shell.faces {
        for wire in std::iter::once(&face.outer_wire).chain(face.inner_wires.iter()) {
            for oriented in &wire.edges {
                let start = oriented.edge.start_vertex.point;
                let end = oriented.edge.end_vertex.point;
                let vertical = (end.x - start.x).abs() < 1e-9 && (end.y - start.y).abs() < 1e-9;
                let at_notch = (start.x - 20.0).abs() < 1e-9 && (start.y - 20.0).abs() < 1e-9;
                if vertical && at_notch && !concave_ids.contains(&oriented.edge.id) {
                    concave_ids.push(oriented.edge.id);
                }
            }
        }
    }
    assert_eq!(concave_ids.len(), 1, "there is one inside upright at the notch");

    let error = EdgeBlender::fillet_edge(&l_shape, concave_ids[0], 2.0)
        .expect_err("a concave edge must be refused");
    assert!(
        error.contains("convex"),
        "the refusal should say why: {error}"
    );

    // 列挙にも出てこない
    assert!(
        !EdgeBlender::blendable_edges(&l_shape)
            .iter()
            .any(|edge| edge.edge_id == concave_ids[0]),
        "a concave edge must not be listed as blendable"
    );
}

#[test]
fn a_blend_larger_than_the_neighbouring_edges_is_refused() {
    let boxed = PrimitiveBuilder::make_box(20.0, 30.0, 40.0).unwrap();
    let edge_id = edge_nearest(&boxed, Point3::new(0.0, 0.0, 20.0));

    // 20 の辺を越える後退距離は取れない
    let error =
        EdgeBlender::chamfer_edge(&boxed, edge_id, 25.0).expect_err("too large must be refused");
    assert!(error.contains("setback"), "{error}");

    let listed = EdgeBlender::blendable_edges(&boxed)
        .into_iter()
        .find(|edge| edge.edge_id == edge_id)
        .expect("the edge is listed");
    assert!(
        listed.max_chamfer_distance < 20.0 && listed.max_chamfer_distance > 19.9,
        "the stated ceiling was {}",
        listed.max_chamfer_distance
    );
    assert!(
        EdgeBlender::chamfer_edge(&boxed, edge_id, listed.max_chamfer_distance * 0.99).is_ok(),
        "just under the stated ceiling must work"
    );
}

#[test]
fn a_filleted_solid_still_writes_and_reads_back_as_a_step_solid() {
    let boxed = PrimitiveBuilder::make_box(20.0, 30.0, 40.0).unwrap();
    let edge_id = edge_nearest(&boxed, Point3::new(0.0, 0.0, 20.0));
    let filleted = EdgeBlender::fillet_edge(&boxed, edge_id, 4.0).unwrap();
    let expected = volume_of(&filleted);

    let (text, _report) = zenith_algo::StepInterop::export_solid_to_string(
        &filleted,
        "edge_blend_probe",
        &Tolerance::default(),
    );
    let reimported = zenith_io::StepImporter::import_solid_from_str(&text).expect("re-import");

    let volume = volume_of(&reimported);
    assert!(
        (volume - expected).abs() / expected < 1e-9,
        "after a STEP round trip: {volume} against {expected}"
    );
}
