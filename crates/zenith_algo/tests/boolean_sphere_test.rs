//! Cutting a sphere with a slab.
//!
//! A plane square to a sphere's axis meets it along a parallel, which is one of
//! the sphere's own parameter lines, so the same path that sections a cylinder
//! or a torus handles it. What was missing was the splitting.
//!
//! A sphere's polar patches are three-sided: the pole is a corner where the two
//! meridians meet, not an edge, so there is one section and two sides rather
//! than two and two. The splitter required four, and refused every one of them.
//! The piece nearer the pole stays three-sided and the other becomes four.
//!
//! The slab reaches well past the sphere sideways, so the only thing cutting it
//! is that one plane.

use zenith_algo::{BooleanEngine, BooleanOpType, BrepTransform, MassCalculator, PrimitiveBuilder};
use zenith_math::{Tolerance, Vec3};
use zenith_tess::TessellationParams;
use zenith_topo::Solid;

fn volume(solid: &Solid) -> f64 {
    MassCalculator::compute_from_brep(
        solid,
        &TessellationParams {
            u_divisions: 64,
            v_divisions: 64,
        },
    )
    .volume
}

/// Volume of a spherical cap of height `h` on a sphere of radius `r`.
fn spherical_cap(radius: f64, height: f64) -> f64 {
    std::f64::consts::PI * height * height * (3.0 * radius - height) / 3.0
}

#[test]
fn test_a_slab_cuts_a_sphere_into_a_cap_and_the_rest() {
    let tol = Tolerance::default();
    let sphere = PrimitiveBuilder::make_sphere(10.0).expect("sphere");
    // z = -2 より上を覆うスラブ。切るのは底面 z = -2 だけ。
    let slab = BrepTransform::translate_solid(
        &PrimitiveBuilder::make_box(60.0, 60.0, 40.0).expect("slab"),
        Vec3::new(-30.0, -30.0, -2.0),
    );

    // 下側に残るのは高さ 10 - 2 = 8 の球冠。
    let cap = spherical_cap(10.0, 8.0);
    let whole = 4.0 / 3.0 * std::f64::consts::PI * 1000.0;

    let subjects = [
        (BooleanOpType::Union, 60.0 * 60.0 * 40.0 + cap),
        (BooleanOpType::Difference, cap),
        (BooleanOpType::Intersection, whole - cap),
    ];

    for (op, expected) in subjects {
        let result = BooleanEngine::boolean_solids_exact_result(&sphere, &slab, op, &tol)
            .unwrap_or_else(|err| panic!("{op:?} of sphere and slab should succeed: {err}"));
        assert_eq!(result.solids.len(), 1);

        let solid = &result.solids[0];
        assert!(
            solid.outer_shell.validate_closed(&tol).is_valid(),
            "{op:?} should close"
        );

        let measured = volume(solid);
        let relative = (measured - expected).abs() / expected;
        assert!(
            relative < 1e-6,
            "{op:?} gave {measured:.4}, closed form {expected:.4} (relative {relative:.2e})"
        );
    }
}

#[test]
fn test_the_cap_keeps_the_pole_and_the_flat_face_it_was_cut_on() {
    let tol = Tolerance::default();
    let sphere = PrimitiveBuilder::make_sphere(10.0).expect("sphere");
    let slab = BrepTransform::translate_solid(
        &PrimitiveBuilder::make_box(60.0, 60.0, 40.0).expect("slab"),
        Vec3::new(-30.0, -30.0, -2.0),
    );

    let result =
        BooleanEngine::boolean_solids_exact_result(&sphere, &slab, BooleanOpType::Difference, &tol)
            .expect("difference");
    let cap = &result.solids[0];

    // 4枚の球面ピース（極を含む三辺の側）と、切り口の円板1枚。
    let planar: Vec<_> = cap
        .outer_shell
        .faces
        .iter()
        .filter(|face| matches!(face.geometry, zenith_topo::FaceGeometry::Plane(_)))
        .collect();
    assert_eq!(planar.len(), 1, "one flat face where the plane cut");
    assert!(planar[0].inner_wires.is_empty(), "the cut face is a disc");

    // 円板の半径は sqrt(100 - 4)。
    let expected = std::f64::consts::PI * (100.0 - 4.0);
    let area = MassCalculator::compute_face_integral(
        planar[0],
        &TessellationParams {
            u_divisions: 64,
            v_divisions: 64,
        },
    )
    .0;
    let relative = (area - expected).abs() / expected;
    assert!(
        relative < 1e-4,
        "cut disc area {area:.4}, closed form {expected:.4} (relative {relative:.2e})"
    );

    // 極が残っていること。三辺のピースが極を頂点として保っているか。
    let lowest = cap
        .outer_shell
        .faces
        .iter()
        .flat_map(|face| face.outer_wire.sample_points(8))
        .fold(f64::INFINITY, |lowest, point| lowest.min(point.z));
    assert!(
        (lowest + 10.0).abs() < 1e-6,
        "the cap should still reach the pole at z = -10, lowest was {lowest:.6}"
    );
}
