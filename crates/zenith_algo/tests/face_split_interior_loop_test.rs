//! 面の内側で閉じたループによる分割。
//!
//! 境界から境界へ届く切り込みとは別の形です。球の八分片を角の箱の3面が切ると、
//! 交線は3本の弧になり、**3本で球面上の閉じたループ**を作ります。どの弧も
//! 八分片の境界に着かないので、それまでの口はすべて「境界に届かない」と
//! 断っていました。
//!
//! 検査は**面積の和**で行います。割り方が正しければ、内側の片と外側の片の
//! 面積の和は元の面に戻ります。戻らなければ、穴が抜けていないか、内側を
//! 二重に数えています。

use zenith_algo::{FaceSplitter, MassCalculator};
use zenith_geom::{ControlPoint3, KnotVector, NurbsCurve3, PlaneSurface3};
use zenith_math::{Point3, Tolerance, Vec3};
use zenith_tess::TessellationParams;
use zenith_topo::{Edge, Face, FaceGeometry, OrientedEdge, Vertex, Wire};

fn segment(from: Point3, to: Point3) -> Edge {
    let curve = NurbsCurve3::new(
        1,
        vec![
            ControlPoint3::unweighted(from),
            ControlPoint3::unweighted(to),
        ],
        KnotVector::new(vec![0.0, 0.0, 1.0, 1.0]),
    )
    .expect("a straight edge");
    Edge::new(
        curve,
        Vertex::from_point(from),
        Vertex::from_point(to),
        1e-9,
    )
}

fn rectangle(half: f64, z: f64) -> Vec<Edge> {
    let corners = [
        Point3::new(-half, -half, z),
        Point3::new(half, -half, z),
        Point3::new(half, half, z),
        Point3::new(-half, half, z),
    ];
    (0..4)
        .map(|index| segment(corners[index], corners[(index + 1) % 4]))
        .collect()
}

fn planar_face(half: f64) -> Face {
    let plane = PlaneSurface3::new(
        Point3::new(0.0, 0.0, 0.0),
        Vec3::new(1.0, 0.0, 0.0),
        Vec3::new(0.0, 1.0, 0.0),
    )
    .expect("a plane");
    let wire = Wire::new(
        rectangle(half, 0.0)
            .into_iter()
            .map(OrientedEdge::forward)
            .collect(),
    );
    Face::new(
        FaceGeometry::Plane(plane),
        wire,
        Vec::new(),
        zenith_topo::Orientation::Forward,
        1e-9,
    )
}

/// 20 x 20 の正方形の真ん中に、6 x 6 の正方形のループを入れて割る。
#[test]
fn the_two_pieces_add_back_up_to_the_original() {
    let tol = Tolerance::default();
    let face = planar_face(10.0);
    let cut = rectangle(3.0, 0.0);

    let (pieces, report) = FaceSplitter::split_by_interior_loop(&face, &cut, &tol)
        .expect("an interior loop must split the face");

    assert_eq!(pieces.len(), 2, "an interior loop makes exactly two pieces");
    assert!(
        report.area_residual <= 1e-9,
        "the pieces do not add back up: {:.3e} (parameter areas: original {:.6}, pieces {:?})",
        report.area_residual,
        report.original_parameter_area,
        report.piece_parameter_areas
    );

    // 内側の片は 6x6 = 36、外側は 400 - 36 = 364。**どちらがどちらかも
    // 見ます** — 合計だけでは、内側と外側を取り違えても通ります。
    // 判定はパラメータ面積でしますが（4-76）、閉じた式と突き合わせるのは
    // **3D の面積**です。ここで自分で積みます。
    let params = TessellationParams::default();
    let mut areas: Vec<f64> = pieces
        .iter()
        .map(|piece| MassCalculator::compute_face_integral(piece, &params).0)
        .collect();
    areas.sort_by(f64::total_cmp);
    assert!(
        (areas[0] - 36.0).abs() <= 1e-9,
        "the inner piece should be 36, got {}",
        areas[0]
    );
    assert!(
        (areas[1] - 364.0).abs() <= 1e-9,
        "the outer piece should be 364, got {}",
        areas[1]
    );
}

/// 外側の片は、ループを**穴**として持たなければなりません。持っていなければ
/// 面積は合っていても、後段でその稜に相手がいません。
#[test]
fn the_outer_piece_carries_the_loop_as_a_hole() {
    let tol = Tolerance::default();
    let face = planar_face(10.0);
    let cut = rectangle(3.0, 0.0);
    let (pieces, _) = FaceSplitter::split_by_interior_loop(&face, &cut, &tol).expect("split");

    let with_hole = pieces
        .iter()
        .find(|piece| !piece.inner_wires.is_empty())
        .expect("one piece must carry the loop as a hole");
    assert_eq!(with_hole.inner_wires.len(), 1);
    assert_eq!(with_hole.inner_wires[0].edges.len(), 4);
    assert_eq!(with_hole.outer_wire.edges.len(), 4);
}

/// 閉じていない切り込みは断ること。**境界から境界への切り込みを、うっかり
/// この口に流し込まない**ためです。
#[test]
fn an_open_chain_is_refused() {
    let tol = Tolerance::default();
    let face = planar_face(10.0);
    let open: Vec<Edge> = rectangle(3.0, 0.0).into_iter().take(3).collect();
    let error = FaceSplitter::split_by_interior_loop(&face, &open, &tol)
        .expect_err("an open chain is not an interior loop");
    assert!(
        error.contains("closed loop"),
        "the reason should say the cut is not closed, got: {error}"
    );
}

/// 面から離れたループは断ること。
#[test]
fn a_loop_off_the_face_is_refused() {
    let tol = Tolerance::default();
    let face = planar_face(10.0);
    let floating = rectangle(3.0, 5.0);
    let error = FaceSplitter::split_by_interior_loop(&face, &floating, &tol)
        .expect_err("a loop that is not on the face must be refused");
    assert!(
        error.contains("leaves the face"),
        "the reason should say the loop is off the face, got: {error}"
    );
}

/// **もとから穴のある面**を、内側のループで割る（4-491）。
///
/// それまで「**穴のある面は未実装**」と断っていました。実測（`linkrods.step`）:
/// **穴を 1 つ持つ面に、閉じた輪が 4 本届いていて、1 つも割れません**でした。
///
/// 割り方は素直です——**もとの穴を、内側の片と外側の片に配り分ける**。
/// **またいでいる穴は断ります**（本当に交わっているので、ここでは扱えません）。
#[test]
fn a_face_that_already_has_a_hole_still_splits() {
    let tol = Tolerance::default();

    // 20 × 20 の板の**隅**に 2 × 2 の穴を開けます（中心から (6,6) の所）。
    let hole_edges: Vec<Edge> = {
        let corners = [
            Point3::new(5.0, 5.0, 0.0),
            Point3::new(7.0, 5.0, 0.0),
            Point3::new(7.0, 7.0, 0.0),
            Point3::new(5.0, 7.0, 0.0),
        ];
        // **穴は外周と逆に巻きます。**
        (0..4)
            .map(|index| segment(corners[(4 - index) % 4], corners[(3 - index) % 4]))
            .collect()
    };
    let plane = PlaneSurface3::new(
        Point3::new(0.0, 0.0, 0.0),
        Vec3::new(1.0, 0.0, 0.0),
        Vec3::new(0.0, 1.0, 0.0),
    )
    .expect("a plane");
    let outer = Wire::new(
        rectangle(10.0, 0.0)
            .into_iter()
            .map(OrientedEdge::forward)
            .collect(),
    );
    let hole = Wire::new(hole_edges.into_iter().map(OrientedEdge::forward).collect());
    let face = Face::new(
        FaceGeometry::Plane(plane),
        outer,
        vec![hole],
        zenith_topo::Orientation::Forward,
        1e-9,
    );

    // 真ん中に 6 × 6 のループ。**穴（隅）はこの中に入りません。**
    let cut = rectangle(3.0, 0.0);
    let (pieces, report) = FaceSplitter::split_by_interior_loop(&face, &cut, &tol)
        .expect("穴のある面も、内側のループで割れること");

    assert_eq!(pieces.len(), 2, "内側のループは 2 枚にします");
    assert!(
        report.area_residual <= 1e-9,
        "片の面積が元に戻りません: {:.3e}",
        report.area_residual
    );

    let params = TessellationParams::default();
    let mut areas: Vec<f64> = pieces
        .iter()
        .map(|piece| MassCalculator::compute_face_integral(piece, &params).0)
        .collect();
    areas.sort_by(f64::total_cmp);
    // 内側の片 = 36、外側の片 = 400 - 36 - 4（もとの穴）= 360。
    assert!(
        (areas[0] - 36.0).abs() <= 1e-9,
        "内側の片は 36 のはず、実際は {}",
        areas[0]
    );
    assert!(
        (areas[1] - 360.0).abs() <= 1e-9,
        "外側の片は 360 のはず（もとの穴 4 を抜いた分）、実際は {}",
        areas[1]
    );

    // **もとの穴は、外側の片が持ちます**（内側のループの外にあるので）。
    let outer_piece = pieces
        .iter()
        .max_by(|a, b| {
            MassCalculator::compute_face_integral(a, &params)
                .0
                .total_cmp(&MassCalculator::compute_face_integral(b, &params).0)
        })
        .expect("片");
    assert_eq!(
        outer_piece.inner_wires.len(),
        2,
        "外側の片は、もとの穴と新しいループの 2 つを穴として持ちます"
    );
}
