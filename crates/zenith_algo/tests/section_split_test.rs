//! Sectioning and splitting a patch, whatever surface it sits on.
//!
//! A plane square to the axis of a surface of revolution meets it along one of
//! the surface's own parameter lines. That is true of a cylinder, a cone and a
//! torus alike, and it is worth having as one path rather than three: the line
//! comes out of the control net exactly and runs from one edge of the patch to
//! the other, which is what splitting needs.
//!
//! Nothing here recognizes a shape. The section is found by asking the patch
//! where its distance along the plane's normal is zero, and then checking the
//! line really lies in the plane. The split reads the face's own boundary to
//! find which two edges are sections and which two carry it between them.
//!
//! The torus is the case that motivated it. Its parameter lines run the other
//! way round from a cylinder's - the axial direction is u, not v - and its
//! sides are arcs rather than straight rulings, so both had to stop being
//! assumed.

use zenith_algo::{BrepIntersectionBuilder, BrepTransform, FaceIntersectionKind, PrimitiveBuilder};
use zenith_math::{Point3, Tolerance, Vec3};
use zenith_topo::Solid;

fn section_curves(a: &Solid, b: &Solid, tol: &Tolerance) -> Vec<(usize, zenith_topo::Edge)> {
    BrepIntersectionBuilder::collect_face_pair_candidates(
        &a.outer_shell.faces,
        &b.outer_shell.faces,
        tol,
    )
    .into_iter()
    .filter_map(|candidate| match candidate.kind {
        FaceIntersectionKind::Curve { edge } => Some((candidate.face_a_index, edge)),
        _ => None,
    })
    .collect()
}

fn axis_distance(point: Point3) -> f64 {
    (point.x * point.x + point.y * point.y).sqrt()
}

#[test]
fn test_a_plane_sections_a_torus_on_the_two_circles_it_should() {
    let tol = Tolerance::default();
    // 主半径12・副半径4のトーラス。z = -2 の平面で切ると、芯円からの距離が
    // sqrt(16 - 4) = 3.4641 のところで交わるので、半径 12 +- 3.4641 の
    // 2本の円になる。
    let torus = PrimitiveBuilder::make_torus(12.0, 4.0).expect("torus");
    let cutter = BrepTransform::translate_solid(
        &PrimitiveBuilder::make_box(20.0, 20.0, 20.0).expect("box"),
        Vec3::new(-10.0, -10.0, -2.0),
    );

    // 箱の**底面**が作る断面だけを取る。箱は 20 立方で |x|, |y| <= 10 なので、
    // 主半径12・副半径4のトーラス（軸からの距離は 8 から 16）は箱の側面も
    // 突き抜けており、そちらにも本物の交線が出る。平面が軸に垂直でない切り方は
    // 以前は取れておらず、この試験はそれが 0 本だった頃に書かれている。
    let sections: Vec<_> = section_curves(&torus, &cutter, &tol)
        .into_iter()
        .filter(|(_, edge)| {
            (0..=8).all(|step| (edge.evaluate(step as f64 / 8.0).z + 2.0).abs() < 1e-9)
        })
        .collect();
    // 16枚のうち z = -2 を含むのは下半分の8枚だが、**面に載るのは4本だけ**で
    // ある。外側の円は半径 12 + 3.4641 = 15.4641 で、箱の底面（20 角、隅まで
    // sqrt(200) = 14.1421）の外を通る。交線は面のトリム境界で切られるので、
    // 内側の円の4本だけが残る。切らずに渡していた頃はここが8本で、その4本は
    // どの面にも載らないまま分割を邪魔していた。
    assert_eq!(
        sections.len(),
        4,
        "only the inner circle lies on the box's bottom face"
    );

    let offset = (16.0f64 - 4.0).sqrt();
    let (inner, outer) = (12.0 - offset, 12.0 + offset);
    let mut on_inner = 0;
    let mut on_outer = 0;

    for (_, edge) in &sections {
        for step in 0..=8 {
            let point = edge.evaluate(step as f64 / 8.0);
            assert!(
                (point.z + 2.0).abs() < 1e-9,
                "a section of the cutting plane should stay on it; z = {}",
                point.z
            );
        }
        let radius = axis_distance(edge.evaluate(0.5));
        if (radius - inner).abs() < 1e-9 {
            on_inner += 1;
        } else if (radius - outer).abs() < 1e-9 {
            on_outer += 1;
        } else {
            panic!("section at radius {radius:.6}, expected {inner:.6} or {outer:.6}");
        }
    }

    assert_eq!(on_inner, 4, "four quarter arcs on the inner circle");
    assert_eq!(
        on_outer, 0,
        "the outer circle runs outside the face and is clipped away"
    );
}

#[test]
fn test_a_torus_patch_splits_along_its_section() {
    let tol = Tolerance::default();
    let torus = PrimitiveBuilder::make_torus(12.0, 4.0).expect("torus");
    let cutter = BrepTransform::translate_solid(
        &PrimitiveBuilder::make_box(20.0, 20.0, 20.0).expect("box"),
        Vec3::new(-10.0, -10.0, -2.0),
    );

    // 箱の底面が作る断面だけを見る。側面の交線はパラメータ線ではないので、
    // ここで見ている「断面で割る」経路の担当ではない。
    let sections = section_curves(&torus, &cutter, &tol)
        .into_iter()
        .filter(|(_, edge)| {
            (0..=8).all(|step| (edge.evaluate(step as f64 / 8.0).z + 2.0).abs() < 1e-9)
        });
    for (face_index, edge) in sections {
        let face = &torus.outer_shell.faces[face_index];
        let pieces = BrepIntersectionBuilder::split_face_by_edge(face, &edge, &tol)
            .unwrap_or_else(|err| panic!("torus face {face_index} should split: {err}"));
        assert_eq!(pieces.len(), 2, "a section splits a patch in two");

        for piece in &pieces {
            assert!(
                piece.outer_wire.is_closed(&tol),
                "a split piece should close"
            );
            // 側辺は元の子午線弧の一部でなければならない。直線に置き換えると
            // トーラスの上から外れる。
            for oriented in &piece.outer_wire.edges {
                for step in 0..=8 {
                    let point = oriented.evaluate_normalized(step as f64 / 8.0);
                    let radial = axis_distance(point);
                    let from_core = ((radial - 12.0).powi(2) + point.z * point.z).sqrt();
                    assert!(
                        (from_core - 4.0).abs() < 1e-6,
                        "a boundary point left the torus by {:.3e}",
                        (from_core - 4.0).abs()
                    );
                }
            }
        }
    }
}

#[test]
fn test_the_same_path_still_sections_a_cylinder_and_a_cone() {
    let tol = Tolerance::default();
    let subjects: [(&str, Solid, f64); 2] = [
        (
            "cylinder",
            PrimitiveBuilder::make_cylinder(10.0, 40.0).expect("cylinder"),
            10.0,
        ),
        (
            "cone",
            PrimitiveBuilder::make_cone(10.0, 4.0, 20.0).expect("cone"),
            7.0,
        ),
    ];

    for (name, solid, expected_radius) in subjects {
        // z = 10 で切る。円柱は半径10のまま、円錐はそこで半径7。
        let cutter = BrepTransform::translate_solid(
            &PrimitiveBuilder::make_box(40.0, 40.0, 40.0).expect("box"),
            Vec3::new(-20.0, -20.0, 10.0),
        );
        let sections = section_curves(&solid, &cutter, &tol);
        assert_eq!(sections.len(), 4, "{name}: four quarter arcs");
        for (_, edge) in &sections {
            let radius = axis_distance(edge.evaluate(0.5));
            assert!(
                (radius - expected_radius).abs() < 1e-9,
                "{name}: section at radius {radius:.6}, expected {expected_radius:.6}"
            );
        }
    }
}
