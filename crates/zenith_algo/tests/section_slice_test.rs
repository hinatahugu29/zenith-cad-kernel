//! Section slicing against analytic cross-sections.
//!
//! Slicing used to intersect face edges only and join the crossings with
//! chords, which turned a circular section into the square inscribed in it,
//! reported an empty section as a zero-area success, and added hole loops to
//! the area instead of subtracting them. These tests pin the corrected
//! behaviour to closed-form answers.

use std::f64::consts::PI;
use zenith_algo::{HoleBuilder, PrimitiveBuilder, SectionSlicer};
use zenith_math::{Point3, Tolerance, Vec3};
use zenith_tess::TessellationParams;

fn relative_error(value: f64, expected: f64) -> f64 {
    (value - expected).abs() / expected.abs()
}

#[test]
fn test_axis_aligned_box_sections_are_exact() {
    let tol = Tolerance::default();
    let solid = PrimitiveBuilder::make_box(20.0, 30.0, 40.0).unwrap();

    let horizontal = SectionSlicer::slice_solid(
        &solid,
        Point3::new(0.0, 0.0, 20.0),
        Vec3::new(0.0, 0.0, 1.0),
        &tol,
    )
    .expect("horizontal section");
    assert_eq!(horizontal.section_wires.len(), 1);
    assert!(
        relative_error(horizontal.total_area, 600.0) < 1e-12,
        "z section area {} should be exactly 600",
        horizontal.total_area
    );
    assert!(
        relative_error(horizontal.total_perimeter, 100.0) < 1e-12,
        "z section perimeter {} should be exactly 100",
        horizontal.total_perimeter
    );

    let vertical = SectionSlicer::slice_solid(
        &solid,
        Point3::new(10.0, 0.0, 0.0),
        Vec3::new(1.0, 0.0, 0.0),
        &tol,
    )
    .expect("vertical section");
    assert!(
        relative_error(vertical.total_area, 1200.0) < 1e-12,
        "x section area {} should be exactly 1200",
        vertical.total_area
    );
    assert!(
        relative_error(vertical.total_perimeter, 140.0) < 1e-12,
        "x section perimeter {} should be exactly 140",
        vertical.total_perimeter
    );
}

#[test]
fn test_diagonal_box_section_matches_the_analytic_hexagon() {
    let tol = Tolerance::default();
    let solid = PrimitiveBuilder::make_box(20.0, 30.0, 40.0).unwrap();

    // 平面 x+y+z=45 は箱の中心を通り、6辺を切って六角形になる。
    // 頂点は (5,0,40) (0,5,40) (0,30,15) (15,30,0) (20,25,0) (20,0,25) で、
    // 面積は 575*sqrt(3)。
    let result = SectionSlicer::slice_solid(
        &solid,
        Point3::new(10.0, 15.0, 20.0),
        Vec3::new(1.0, 1.0, 1.0),
        &tol,
    )
    .expect("diagonal section");

    let expected = 575.0 * 3.0_f64.sqrt();
    assert_eq!(result.section_wires.len(), 1);
    assert!(
        relative_error(result.total_area, expected) < 1e-9,
        "diagonal section area {} should match {expected}",
        result.total_area
    );
}

#[test]
fn test_cylinder_section_approaches_the_analytic_circle() {
    let tol = Tolerance::default();
    let solid = PrimitiveBuilder::make_cylinder(10.0, 40.0).unwrap();

    let result = SectionSlicer::slice_solid(
        &solid,
        Point3::new(0.0, 0.0, 20.0),
        Vec3::new(0.0, 0.0, 1.0),
        &tol,
    )
    .expect("cylinder section");

    assert_eq!(
        result.section_wires.len(),
        1,
        "a cylinder section is a single loop, not one per surface patch"
    );

    let expected_area = PI * 100.0;
    assert!(
        relative_error(result.total_area, expected_area) < 1e-4,
        "cylinder section area {} should approach {expected_area}",
        result.total_area
    );

    let expected_perimeter = 2.0 * PI * 10.0;
    assert!(
        relative_error(result.total_perimeter, expected_perimeter) < 1e-4,
        "cylinder section perimeter {} should approach {expected_perimeter}",
        result.total_perimeter
    );
}

#[test]
fn test_sphere_equator_section_is_not_empty() {
    let tol = Tolerance::default();
    let solid = PrimitiveBuilder::make_sphere(10.0).unwrap();

    // 以前はループ0本・面積0を Ok で返していた。
    let result = SectionSlicer::slice_solid(
        &solid,
        Point3::new(0.0, 0.0, 0.0),
        Vec3::new(0.0, 0.0, 1.0),
        &tol,
    )
    .expect("sphere equator section");

    assert_eq!(result.section_wires.len(), 1);
    assert!(
        relative_error(result.total_area, PI * 100.0) < 1e-2,
        "sphere equator area {} should approach {}",
        result.total_area,
        PI * 100.0
    );
}

#[test]
fn test_drilled_box_section_subtracts_the_hole() {
    let tol = Tolerance::default();
    let solid = HoleBuilder::make_drilled_box(30.0, 30.0, 15.0, 5.0).unwrap();

    let result = SectionSlicer::slice_solid(
        &solid,
        Point3::new(0.0, 0.0, 7.5),
        Vec3::new(0.0, 0.0, 1.0),
        &tol,
    )
    .expect("drilled box section");

    assert_eq!(
        result.section_wires.len(),
        2,
        "the section has an outer square and a hole loop"
    );

    let expected = 900.0 - PI * 25.0;
    assert!(
        relative_error(result.total_area, expected) < 1e-4,
        "drilled box section area {} should be {expected}, not the sum of both loops",
        result.total_area
    );

    let positive = result
        .signed_loop_areas
        .iter()
        .filter(|area| **area > 0.0)
        .count();
    let negative = result
        .signed_loop_areas
        .iter()
        .filter(|area| **area < 0.0)
        .count();
    assert_eq!(
        (positive, negative),
        (1, 1),
        "the hole loop must carry the opposite sign: {:?}",
        result.signed_loop_areas
    );
}

#[test]
fn test_section_plane_missing_the_solid_reports_an_empty_section() {
    let tol = Tolerance::default();
    let solid = PrimitiveBuilder::make_box(20.0, 30.0, 40.0).unwrap();

    let result = SectionSlicer::slice_solid(
        &solid,
        Point3::new(0.0, 0.0, 500.0),
        Vec3::new(0.0, 0.0, 1.0),
        &tol,
    )
    .expect("a plane that misses the solid is not an error");

    assert!(result.section_wires.is_empty());
    assert_eq!(result.total_area, 0.0);
    assert_eq!(result.total_perimeter, 0.0);
}

#[test]
fn test_curved_section_accuracy_improves_with_tessellation() {
    let tol = Tolerance::default();
    let solid = PrimitiveBuilder::make_cylinder(10.0, 40.0).unwrap();
    let expected = PI * 100.0;

    let coarse = SectionSlicer::slice_solid_with_tessellation(
        &solid,
        Point3::new(0.0, 0.0, 20.0),
        Vec3::new(0.0, 0.0, 1.0),
        &tol,
        &TessellationParams {
            u_divisions: 8,
            v_divisions: 8,
        },
    )
    .expect("coarse section");

    let fine = SectionSlicer::slice_solid_with_tessellation(
        &solid,
        Point3::new(0.0, 0.0, 20.0),
        Vec3::new(0.0, 0.0, 1.0),
        &tol,
        &TessellationParams {
            u_divisions: 256,
            v_divisions: 256,
        },
    )
    .expect("fine section");

    let coarse_error = relative_error(coarse.total_area, expected);
    let fine_error = relative_error(fine.total_area, expected);

    assert!(
        fine_error < coarse_error,
        "refining the tessellation must reduce the section error: coarse {coarse_error}, fine {fine_error}"
    );
    assert!(
        fine_error < 1e-5,
        "a 256-division section should be within 1e-5 of the circle, got {fine_error}"
    );
}

#[test]
fn test_zero_normal_is_rejected() {
    let tol = Tolerance::default();
    let solid = PrimitiveBuilder::make_box(10.0, 10.0, 10.0).unwrap();

    assert!(SectionSlicer::slice_solid(
        &solid,
        Point3::new(0.0, 0.0, 5.0),
        Vec3::new(0.0, 0.0, 0.0),
        &tol,
    )
    .is_err());
}

/// 断面は、メッシュから拾った弦の多角形ではなく、**B-Rep の上で測った点**で
/// 積まれる。多角形のままだと面積は必ず内側に削れ、分割数の2乗でしか縮まない。
///
/// この検査は、いま届いている桁（既定分割で 1e-9 以内）を固定する。
/// `test_curved_section_accuracy_improves_with_tessellation` が「良くなること」
/// を見るのに対し、こちらは「どこまで来ているか」を見る。
#[test]
fn test_curved_sections_land_on_the_analytic_value_not_merely_near_it() {
    let tol = Tolerance::default();
    let cases: Vec<(&str, zenith_topo::Solid, Point3, f64, f64)> = vec![
        (
            "cylinder r10",
            PrimitiveBuilder::make_cylinder(10.0, 40.0).unwrap(),
            Point3::new(0.0, 0.0, 20.0),
            PI * 100.0,
            2.0 * PI * 10.0,
        ),
        (
            "sphere r10 equator",
            PrimitiveBuilder::make_sphere(10.0).unwrap(),
            Point3::new(0.0, 0.0, 0.0),
            PI * 100.0,
            2.0 * PI * 10.0,
        ),
    ];

    for (name, solid, origin, expected_area, expected_perimeter) in cases {
        let result =
            SectionSlicer::slice_solid(&solid, origin, Vec3::new(0.0, 0.0, 1.0), &tol)
                .unwrap_or_else(|err| panic!("{name} section: {err}"));

        let area_error = relative_error(result.total_area, expected_area);
        let perimeter_error = relative_error(result.total_perimeter, expected_perimeter);
        assert!(
            area_error < 1e-9,
            "{name} section area {} is {area_error:.3e} from {expected_area}",
            result.total_area
        );
        assert!(
            perimeter_error < 1e-9,
            "{name} section perimeter {} is {perimeter_error:.3e} from {expected_perimeter}",
            result.total_perimeter
        );
        assert_eq!(
            result.unrefined_chord_count, 0,
            "{name} should have every chord measured against the B-Rep"
        );
    }
}

/// 平面だけでできた断面は、以前から厳密だった。B-Rep に当てる段を入れても
/// **一切動いてはならない**。動いたら、それは補正ではなく探索の残差である。
#[test]
fn test_planar_sections_stay_exact_at_every_tessellation() {
    let tol = Tolerance::default();
    let solid = PrimitiveBuilder::make_box(20.0, 30.0, 40.0).unwrap();

    for divisions in [4usize, 24, 96, 192] {
        let result = SectionSlicer::slice_solid_with_tessellation(
            &solid,
            Point3::new(0.0, 0.0, 20.0),
            Vec3::new(0.0, 0.0, 1.0),
            &tol,
            &TessellationParams {
                u_divisions: divisions,
                v_divisions: divisions,
            },
        )
        .expect("box section");

        assert_eq!(
            result.total_area, 600.0,
            "a planar section moved at {divisions} divisions"
        );
        assert_eq!(
            result.total_perimeter, 100.0,
            "a planar section perimeter moved at {divisions} divisions"
        );
        assert_eq!(
            result.settled_point_count, 0,
            "a planar section has nothing to settle onto"
        );
    }
}

/// 誤差が分割数の**4乗**で縮むこと。弦のままだと2乗にしかならないので、
/// この比が、二次で積んでいることの証拠になる。
#[test]
fn test_section_error_falls_with_the_fourth_power_of_the_division_count() {
    let tol = Tolerance::default();
    let solid = PrimitiveBuilder::make_cylinder(10.0, 40.0).unwrap();
    let expected = PI * 100.0;

    let error_at = |divisions: usize| -> f64 {
        let result = SectionSlicer::slice_solid_with_tessellation(
            &solid,
            Point3::new(0.0, 0.0, 20.0),
            Vec3::new(0.0, 0.0, 1.0),
            &tol,
            &TessellationParams {
                u_divisions: divisions,
                v_divisions: divisions,
            },
        )
        .expect("section");
        relative_error(result.total_area, expected)
    };

    let coarse = error_at(24);
    let fine = error_at(48);
    let ratio = coarse / fine;
    assert!(
        (8.0..40.0).contains(&ratio),
        "doubling the divisions should divide the error by about 16, got {ratio:.1} \
         (coarse {coarse:.3e}, fine {fine:.3e})"
    );
}
