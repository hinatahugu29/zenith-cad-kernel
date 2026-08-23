//! Reading STEP back, including files this kernel did not write.
//!
//! Export had been checked against OpenCASCADE; import had never been measured.
//! Two defects turned up immediately, both invisible from inside because our
//! own files avoid them: a face whose bounds are all plain FACE_BOUND was
//! rejected outright, and a full circle - what every writer uses for the rim of
//! a cylinder - was read as a zero-length line.

use std::f64::consts::PI;

use zenith_algo::{MassCalculator, PrimitiveBuilder};
use zenith_io::{StepExporter, StepImporter};
use zenith_math::Tolerance;
use zenith_tess::TessellationParams;
use zenith_topo::Solid;

fn volume(solid: &Solid) -> f64 {
    MassCalculator::compute_from_brep(
        solid,
        &TessellationParams {
            u_divisions: 32,
            v_divisions: 32,
        },
    )
    .volume
}

fn round_trip(solid: &Solid, name: &str) -> Solid {
    let step = StepExporter::export_solid_to_string(solid, name);
    StepImporter::import_solid_from_str(&step)
        .unwrap_or_else(|err| panic!("{name} should import back: {err}"))
}

#[test]
fn test_primitives_survive_a_step_round_trip() {
    let tol = Tolerance::default();

    let subjects: Vec<(&str, Solid, f64)> = vec![
        (
            "BOX",
            PrimitiveBuilder::make_box(20.0, 30.0, 40.0).unwrap(),
            24000.0,
        ),
        (
            "CYL",
            PrimitiveBuilder::make_cylinder(10.0, 40.0).unwrap(),
            PI * 100.0 * 40.0,
        ),
        (
            "SPH",
            PrimitiveBuilder::make_sphere(10.0).unwrap(),
            4.0 / 3.0 * PI * 1000.0,
        ),
        (
            "CONE",
            PrimitiveBuilder::make_cone(10.0, 4.0, 20.0).unwrap(),
            PI * 20.0 / 3.0 * (100.0 + 40.0 + 16.0),
        ),
        (
            "TOR",
            PrimitiveBuilder::make_torus(12.0, 4.0).unwrap(),
            2.0 * PI * PI * 12.0 * 16.0,
        ),
    ];

    for (name, solid, analytic) in subjects {
        let imported = round_trip(&solid, name);

        assert_eq!(
            imported.outer_shell.faces.len(),
            solid.outer_shell.faces.len(),
            "{name}: the face count should survive the round trip"
        );

        let report = imported.outer_shell.validate_closed(&tol);
        assert!(
            report.is_valid(),
            "{name}: imported shell is invalid: {:?}",
            report.errors
        );

        let imported_volume = volume(&imported);
        assert!(
            (imported_volume - analytic).abs() / analytic < 1e-9,
            "{name}: imported volume {imported_volume} should still be {analytic}"
        );
    }
}

#[test]
fn test_faces_whose_bounds_are_all_plain_face_bound_are_readable() {
    // FACE_OUTER_BOUND は FACE_BOUND の subtype であって必須ではない。
    // OpenCASCADE をはじめ多くの書き手はすべて FACE_BOUND として出すので、
    // これを必須にしていると他カーネルのファイルが一切読めない。
    let solid = PrimitiveBuilder::make_box(20.0, 30.0, 40.0).unwrap();
    let step = StepExporter::export_solid_to_string(&solid, "BOX");
    let relabelled = step.replace("FACE_OUTER_BOUND", "FACE_BOUND");
    assert!(!relabelled.contains("FACE_OUTER_BOUND"));

    let imported = StepImporter::import_solid_from_str(&relabelled)
        .expect("a file using only FACE_BOUND must still be readable");

    assert_eq!(imported.outer_shell.faces.len(), 6);
    let imported_volume = volume(&imported);
    assert!(
        (imported_volume - 24000.0).abs() / 24000.0 < 1e-9,
        "volume {imported_volume} should be 24000 after reading plain bounds"
    );
}

#[test]
fn test_a_drilled_block_keeps_its_hole_through_a_round_trip() {
    // 穴あき平面は FACE_BOUND の内側ループを持つ。外周の選び方を誤ると
    // 穴と外周が入れ替わり、体積が変わる。
    let tol = Tolerance::default();
    let solid = zenith_algo::HoleBuilder::make_drilled_box(30.0, 30.0, 15.0, 5.0).unwrap();
    let imported = round_trip(&solid, "DRILLED");

    let expected = 30.0 * 30.0 * 15.0 - PI * 25.0 * 15.0;
    let imported_volume = volume(&imported);
    assert!(
        (imported_volume - expected).abs() / expected < 1e-9,
        "drilled volume {imported_volume} should be {expected}"
    );

    let report = imported.outer_shell.validate_closed(&tol);
    assert!(report.is_valid(), "imported shell invalid: {:?}", report.errors);
}

#[test]
fn test_a_planar_face_bounded_by_one_full_circle_has_the_circle_area() {
    // 完全円1本で囲まれた平面は、他カーネルが書く円柱の端面そのもの。
    // 面積を1つの求積則で曲線全体にあてると、4区間の円で 1.4% ずれる。
    let step = "ISO-10303-21;\nHEADER;\nENDSEC;\nDATA;\n\
         #1 = CARTESIAN_POINT('',(0.0,0.0,0.0));\n\
         #2 = DIRECTION('',(0.0,0.0,1.0));\n\
         #3 = DIRECTION('',(1.0,0.0,0.0));\n\
         #4 = AXIS2_PLACEMENT_3D('',#1,#2,#3);\n\
         #5 = CIRCLE('',#4,10.0);\n\
         #6 = CARTESIAN_POINT('',(10.0,0.0,0.0));\n\
         #7 = VERTEX_POINT('',#6);\n\
         #8 = EDGE_CURVE('',#7,#7,#5,.T.);\n\
         #9 = ORIENTED_EDGE('',*,*,#8,.T.);\n\
         #10 = EDGE_LOOP('',(#9));\n\
         #11 = FACE_BOUND('',#10,.T.);\n\
         #12 = PLANE('',#4);\n\
         #13 = ADVANCED_FACE('',(#11),#12,.T.);\n\
         ENDSEC;\nEND-ISO-10303-21;\n";

    let face = StepImporter::import_face_from_str(step, 13)
        .expect("a plane bounded by one circle should be readable");

    let (area, _volume) = MassCalculator::compute_face_integral(
        &face,
        &TessellationParams {
            u_divisions: 32,
            v_divisions: 32,
        },
    );

    let expected = PI * 100.0;
    assert!(
        (area - expected).abs() / expected < 1e-9,
        "a disc of radius 10 has area {expected}, got {area}"
    );
}

#[test]
fn test_a_full_circle_edge_is_read_as_a_circle() {
    // 円柱の縁は完全円として書かれる。始点と終点が一致するので、端点から
    // 掃引角を推測すると長さ0の弧になってしまう。
    let step = format!(
        "ISO-10303-21;\nHEADER;\nENDSEC;\nDATA;\n\
         #1 = CARTESIAN_POINT('',(0.0,0.0,0.0));\n\
         #2 = DIRECTION('',(0.0,0.0,1.0));\n\
         #3 = DIRECTION('',(1.0,0.0,0.0));\n\
         #4 = AXIS2_PLACEMENT_3D('',#1,#2,#3);\n\
         #5 = CIRCLE('',#4,10.0);\n\
         #6 = CARTESIAN_POINT('',(10.0,0.0,0.0));\n\
         #7 = VERTEX_POINT('',#6);\n\
         #8 = EDGE_CURVE('',#7,#7,#5,.T.);\n\
         ENDSEC;\nEND-ISO-10303-21;\n"
    );

    // ソリッド全体を組まずに、そのエッジだけを読む。
    let edge = StepImporter::import_edge_from_str(&step, 8)
        .expect("a closed circular edge should be readable");
    let curve = &edge.curve;

    // 完全円なので、周長は 2*pi*r に一致するはず。
    let samples = 512;
    let (t_min, t_max) = curve.param_range();
    let mut length = 0.0;
    let mut previous = curve.evaluate(t_min);
    for index in 1..=samples {
        let t = t_min + (t_max - t_min) * (index as f64 / samples as f64);
        let point = curve.evaluate(t);
        length += (point - previous).norm();
        previous = point;
    }

    let expected = 2.0 * PI * 10.0;
    assert!(
        (length - expected).abs() / expected < 1e-4,
        "a full circle of radius 10 should be {expected} long, got {length}"
    );
}
