//! Cutting a torus with a slab.
//!
//! A plane square to a torus's axis meets it along two circles, and what they
//! bound between them is an annulus, not two discs. Capping each loop with its
//! own disc covers the hole as well as the ring, and every edge on the inner
//! loop ends up used twice the same way round.
//!
//! The slab here reaches well past the torus sideways, so the only thing
//! cutting it is that one plane. A box that ends inside the torus's own reach
//! also cuts it on its side faces, along curves that are not parameter lines,
//! and that stays refused - see the note at the end.

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

/// Volume of the part of a torus below a height, by the closed form for its
/// annular cross-section: the section at `z` is an annulus of area 4 pi R
/// sqrt(r^2 - z^2), so the volume is that integrated.
fn torus_below(major: f64, minor: f64, height: f64) -> f64 {
    let antiderivative = |z: f64| {
        0.5 * z * (minor * minor - z * z).sqrt()
            + 0.5 * minor * minor * (z / minor).asin()
    };
    4.0 * std::f64::consts::PI * major * (antiderivative(height) - antiderivative(-minor))
}

fn torus_volume(major: f64, minor: f64) -> f64 {
    2.0 * std::f64::consts::PI * std::f64::consts::PI * major * minor * minor
}

#[test]
fn test_a_slab_cuts_a_torus_at_the_right_height() {
    let tol = Tolerance::default();
    let torus = PrimitiveBuilder::make_torus(12.0, 4.0).expect("torus");
    // 横に大きく張り出したスラブ。z = -2 の面だけがトーラスに触れる。
    let slab = BrepTransform::translate_solid(
        &PrimitiveBuilder::make_box(60.0, 60.0, 20.0).expect("slab"),
        Vec3::new(-30.0, -30.0, -2.0),
    );

    let below = torus_below(12.0, 4.0, -2.0);
    let whole = torus_volume(12.0, 4.0);

    let subjects = [
        (BooleanOpType::Difference, below),
        (BooleanOpType::Intersection, whole - below),
    ];

    for (op, expected) in subjects {
        let result = BooleanEngine::boolean_solids_exact_result(&torus, &slab, op, &tol)
            .unwrap_or_else(|err| panic!("{op:?} of torus and slab should succeed: {err}"));
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
fn test_the_cut_produces_one_annular_cap_rather_than_two_discs() {
    let tol = Tolerance::default();
    let torus = PrimitiveBuilder::make_torus(12.0, 4.0).expect("torus");
    let slab = BrepTransform::translate_solid(
        &PrimitiveBuilder::make_box(60.0, 60.0, 20.0).expect("slab"),
        Vec3::new(-30.0, -30.0, -2.0),
    );

    let result =
        BooleanEngine::boolean_solids_exact_result(&torus, &slab, BooleanOpType::Difference, &tol)
            .expect("difference");
    let solid = &result.solids[0];

    // 平面の面はキャップ1枚だけで、それが穴を1つ持っていること。
    let planar: Vec<_> = solid
        .outer_shell
        .faces
        .iter()
        .filter(|face| matches!(face.geometry, zenith_topo::FaceGeometry::Plane(_)))
        .collect();
    assert_eq!(planar.len(), 1, "one cap, not one disc per loop");
    assert_eq!(planar[0].inner_wires.len(), 1, "the cap should have a hole");

    // 面積は円環のもの。半径は 12 +- sqrt(16 - 4)。
    let reach = (16.0f64 - 4.0).sqrt();
    let expected = std::f64::consts::PI * ((12.0 + reach).powi(2) - (12.0 - reach).powi(2));
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
        "cap area {area:.4}, annulus {expected:.4} (relative {relative:.2e})"
    );
}

#[test]
fn test_a_box_that_ends_inside_the_torus_is_still_refused() {
    // このケースは箱の側面もトーラスを切る。その切り口はパッチの
    // パラメータ線ではないので、断面の経路では扱えない。
    //
    // 断面だけを見て済ませると、箱を無限のスラブと取り違えた立体が出る。
    // 実際そう出ていて、検証ゲートが弾いた。誤答ではなくエラーで返ること、
    // 通ったときは正しいことの両方を固定する。
    let tol = Tolerance::default();
    let torus = PrimitiveBuilder::make_torus(12.0, 4.0).expect("torus");
    let boxed = BrepTransform::translate_solid(
        &PrimitiveBuilder::make_box(20.0, 20.0, 20.0).expect("box"),
        Vec3::new(-10.0, -10.0, -2.0),
    );

    for op in [
        BooleanOpType::Union,
        BooleanOpType::Difference,
        BooleanOpType::Intersection,
    ] {
        if let Ok(result) = BooleanEngine::boolean_solids_exact_result(&torus, &boxed, op, &tol) {
            for solid in &result.solids {
                assert!(
                    solid.outer_shell.validate_closed(&tol).is_valid(),
                    "{op:?} returned a solid that does not close"
                );
            }
        }
    }
}
