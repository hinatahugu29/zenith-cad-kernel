//! Section slicing against analytic cross-sections.
//!
//! Slicing used to intersect face edges only and join the crossings with
//! chords, which turned a circular section into the square inscribed in it,
//! reported an empty section as a zero-area success, and added hole loops to
//! the area instead of subtracting them. These tests pin the corrected
//! behaviour to closed-form answers.

use std::f64::consts::PI;
use zenith_algo::{HoleBuilder, LoftBuilder, PrimitiveBuilder, ProfileBuilder, SectionSlicer};
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
        // 実測 4.83e-11（314.15926534 対 314.15926536）。許容は 1e-4 だった。
        relative_error(result.total_area, expected_area) < 1e-10,
        "cylinder section area {} should approach {expected_area}",
        result.total_area
    );

    let expected_perimeter = 2.0 * PI * 10.0;
    assert!(
        // 実測 2.41e-11（62.831853070 対 62.831853072）。許容は 1e-4 だった。
        relative_error(result.total_perimeter, expected_perimeter) < 1e-10,
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
    // 許容は 1e-2 だった。断面は 4-10 で B-Rep の上で測るようになっており、
    // ここも実測は 4.83e-11（314.15926534 対 314.15926536）。8桁ぶん緩い許容は、
    // そこが崩れても何も言わない。
    assert!(
        relative_error(result.total_area, PI * 100.0) < 1e-10,
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
        // 実測 4.62e-12（821.46018366405 対 821.46018366026）。許容は 1e-4 だった。
        relative_error(result.total_area, expected) < 1e-11,
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

/// 面の**すぐ手前**を切ったとき、**近い別の答えを返さない**。
///
/// 距離は公差内で 0 に丸めてから符号で分類する。丸めの境目に面が来ると、同じ
/// 平面に乗っているはずの頂点が丸め誤差だけで「平面上」と「正側」に割れ、その
/// 辺が面の**内部で**輪郭の断片として出てくる。実測（円 → 長方形 → 楕円の
/// 多断面ロフト、天面の 1e-6 下、32分割）で、**閉じたループを16本・面積 38.03**
/// を返していた。正しい楕円は 1413.72 で、これは誤答だった。しかも128分割では
/// 「輪郭が閉じない」と断っていて、刻みで答えが変わっていた。
///
/// 面の**上**（距離ちょうど 0）は今までどおり切れる。そこは分類が割れない。
#[test]
fn a_plane_grazing_a_face_is_refused_by_name() {
    let tol = Tolerance::default();
    let solid = PrimitiveBuilder::make_box(20.0, 30.0, 40.0).expect("box");
    let params = TessellationParams {
        u_divisions: 16,
        v_divisions: 16,
    };
    let slice_at = |z: f64| {
        SectionSlicer::slice_solid_with_tessellation(
            &solid,
            Point3::new(0.0, 0.0, z),
            Vec3::new(0.0, 0.0, 1.0),
            &tol,
            &params,
        )
    };

    // 面の**上**（距離ちょうど 0）は今までどおり通る。ただし**上下で答えが
    // 違う**。1辺が平面に乗った三角形は「正側の三角形からだけ」輪郭を出す
    // 規約なので、
    //
    // | 平面 | 返るもの |
    // | :--- | :--- |
    // | 底面ちょうど（z = 0） | 面そのもの（600） |
    // | 天面ちょうど（z = 40） | **空の断面**（正側に三角形が無い） |
    //
    // 半開区間の切り方で、二重に数えないための規約から来ている。**呼ぶ側から
    // 見ると罠**なので、ここに書き留めておく。天面の断面が要るなら、面から
    // 4·公差 より離して切ること。
    let bottom = slice_at(0.0).expect("a plane exactly on the bottom cap still has a section");
    assert!(
        (bottom.total_area - 600.0).abs() < 1e-9,
        "bottom cap: got {}",
        bottom.total_area
    );

    let top = slice_at(40.0).expect("a plane exactly on the top cap is not an error");
    assert!(
        top.section_wires.is_empty() && top.total_area == 0.0,
        "the top cap comes back empty under the half-open rule, got {} loop(s) and area {}",
        top.section_wires.len(),
        top.total_area
    );

    // 直方体の座標には丸め誤差が無いので、公差ぶん手前で切っても分類は
    // 割れない。**ここが断られたら、締めすぎ。**
    let just_above = slice_at(tol.linear)
        .expect("exact coordinates do not split, so this is still answerable");
    assert!(
        (just_above.total_area - 600.0).abs() < 1e-9,
        "got {}",
        just_above.total_area
    );

    // 割れるのは、面の座標そのものに丸め誤差が乗っているとき。円 → 長方形 →
    // 楕円の多断面ロフトの天面がそれで、その 1e-6 下を切ると 32分割で
    // **閉じたループ16本・面積 38.03** が返っていた（正しい楕円は 1413.72）。
    let circle = ProfileBuilder::make_circle(
        20.0,
        Point3::new(0.0, 0.0, 0.0),
        Vec3::new(0.0, 0.0, 1.0),
        Vec3::new(1.0, 0.0, 0.0),
    )
    .expect("circle");
    let rectangle = ProfileBuilder::make_rectangle(
        36.0,
        24.0,
        Point3::new(0.0, 0.0, 30.0),
        Vec3::new(0.0, 0.0, 1.0),
        Vec3::new(1.0, 0.0, 0.0),
    )
    .expect("rectangle");
    let ellipse = ProfileBuilder::make_ellipse(
        30.0,
        15.0,
        Point3::new(0.0, 0.0, 60.0),
        Vec3::new(0.0, 0.0, 1.0),
        Vec3::new(1.0, 0.0, 0.0),
    )
    .expect("ellipse");
    let duct = LoftBuilder::loft_solid(&[circle, rectangle, ellipse], 2, &tol).expect("loft");

    let params32 = TessellationParams {
        u_divisions: 32,
        v_divisions: 32,
    };
    let error = SectionSlicer::slice_solid_with_tessellation(
        &duct,
        Point3::new(0.0, 0.0, 60.0 - tol.linear),
        Vec3::new(0.0, 0.0, 1.0),
        &tol,
        &params32,
    )
    .err()
    .expect("a plane one tolerance under the loft's top cap must be refused, not answered");
    assert!(
        error.contains("grazes the boundary"),
        "the refusal should name the reason, got: {error}"
    );

    // 少し離せば、楕円の断面がそのまま返る。
    let clear = SectionSlicer::slice_solid_with_tessellation(
        &duct,
        Point3::new(0.0, 0.0, 60.0 - 1e-4),
        Vec3::new(0.0, 0.0, 1.0),
        &tol,
        &params32,
    )
    .expect("away from the cap the section is answerable");
    let ellipse_area = PI * 30.0 * 15.0;
    assert!(
        relative_error(clear.total_area, ellipse_area) < 2e-4,
        "the section just under the cap should be the ellipse {ellipse_area}, got {}",
        clear.total_area
    );

    // 離れていれば、これまでどおり答える。
    let inside = slice_at(20.0).expect("a plane through the middle still slices");
    assert!(
        (inside.total_area - 600.0).abs() < 1e-9,
        "got {}",
        inside.total_area
    );
}
