//! Guards the STEP export details that decide whether another kernel can read
//! the result as a solid at all.
//!
//! These were found by round-tripping through FreeCAD / OpenCASCADE: a missing
//! `CURVE()` in the complex curve entity made OCC silently drop the bound of
//! every plane trimmed by spline arcs, which turned cylinders, cones, drilled
//! boxes and swept pipes into unusable Compounds. The failure was invisible
//! from inside this kernel, so it is pinned here at the text level.

use zenith_algo::{HoleBuilder, PrimitiveBuilder, SweepBuilder};
use zenith_geom::NurbsCurve3;
use zenith_io::StepExporter;
use zenith_math::Point3;

fn export(name: &str, solid: &zenith_topo::Solid) -> String {
    StepExporter::export_solid_to_string(solid, name)
}

#[test]
fn test_rational_curves_declare_the_full_complex_entity() {
    let step = export(
        "CYL",
        &PrimitiveBuilder::make_cylinder(10.0, 40.0).unwrap(),
    );

    assert!(
        step.contains("RATIONAL_B_SPLINE_CURVE"),
        "a cylinder must export rational arcs"
    );

    // AP214 の複合エンティティは全スーパータイプを列挙する必要がある。
    // CURVE() が欠けると OpenCASCADE が境界ループを捨てる。
    for occurrence in step.match_indices("RATIONAL_B_SPLINE_CURVE") {
        let prefix = &step[..occurrence.0];
        let entity_start = prefix.rfind("( BOUNDED_CURVE()").expect("complex curve entity");
        let entity = &step[entity_start..occurrence.0];
        assert!(
            entity.contains(" CURVE() "),
            "complex curve entity is missing CURVE(): {entity}"
        );
    }
}

#[test]
fn test_rational_surfaces_declare_the_full_complex_entity() {
    let step = export("SPH", &PrimitiveBuilder::make_sphere(10.0).unwrap());

    assert!(step.contains("RATIONAL_B_SPLINE_SURFACE"));
    for occurrence in step.match_indices("RATIONAL_B_SPLINE_SURFACE") {
        let prefix = &step[..occurrence.0];
        let entity_start = prefix
            .rfind("( BOUNDED_SURFACE()")
            .expect("complex surface entity");
        let entity = &step[entity_start..];
        let entity_end = entity.find(");").unwrap_or(entity.len());
        let entity = &entity[..entity_end];
        assert!(
            entity.contains("SURFACE()"),
            "complex surface entity is missing SURFACE(): {entity}"
        );
    }
}

#[test]
fn test_planar_faces_bounded_by_arcs_are_exported_with_shared_edges() {
    // 円柱の端面は平面をスプライン円弧で囲んだ面。側面と同じ EDGE_CURVE を
    // 共有していないと、読み手が閉シェルを組み立てられない。
    let step = export(
        "CYL",
        &PrimitiveBuilder::make_cylinder(10.0, 40.0).unwrap(),
    );

    let edge_curve_count = step.matches("EDGE_CURVE(").count();
    let oriented_edge_count = step.matches("ORIENTED_EDGE(").count();

    assert!(
        oriented_edge_count > edge_curve_count,
        "every edge should be referenced twice: {edge_curve_count} curves, {oriented_edge_count} uses"
    );
    assert_eq!(
        oriented_edge_count,
        edge_curve_count * 2,
        "a closed manifold uses each edge exactly twice"
    );
}

#[test]
fn test_closed_surfaces_declare_their_closed_flags() {
    // トーラス・球は分割済みなので個々のパッチは閉じていない。閉フラグは
    // 幾何から判定されるので、開いたパッチでは .F. のままであるべき。
    let step = export("TOR", &PrimitiveBuilder::make_torus(12.0, 4.0).unwrap());
    assert!(step.contains("B_SPLINE_SURFACE(2,2"));
    assert!(
        !step.contains(".T.,.T.,.F.)"),
        "individual torus patches are not closed surfaces"
    );
}

#[test]
fn test_curved_solids_export_a_manifold_solid_brep() {
    let path = NurbsCurve3::bspline_from_points(
        3,
        vec![
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(10.0, 0.0, 10.0),
            Point3::new(20.0, 20.0, 25.0),
            Point3::new(30.0, 20.0, 40.0),
        ],
    )
    .unwrap();

    let subjects = [
        ("CYL", PrimitiveBuilder::make_cylinder(10.0, 40.0).unwrap()),
        (
            "CONE",
            PrimitiveBuilder::make_cone(10.0, 4.0, 20.0).unwrap(),
        ),
        ("SPH", PrimitiveBuilder::make_sphere(10.0).unwrap()),
        ("TOR", PrimitiveBuilder::make_torus(12.0, 4.0).unwrap()),
        (
            "DRILL",
            HoleBuilder::make_drilled_box(30.0, 30.0, 15.0, 5.0).unwrap(),
        ),
        (
            "PIPE",
            SweepBuilder::sweep_circle_along_curve(&path, 3.5, 16).unwrap(),
        ),
    ];

    for (name, solid) in subjects {
        let step = export(name, &solid);
        assert!(
            step.contains("MANIFOLD_SOLID_BREP"),
            "{name} must export as a solid, not a surface model"
        );
        assert!(
            step.contains("CLOSED_SHELL"),
            "{name} must export a closed shell"
        );
    }
}

/// ISO 10303-21 では、複数の型を1つの実体にまとめた「複合エンティティ実体」は
/// 外側を丸括弧で囲まなければならない（`#N = ( A(..) B(..) );`）。
///
/// `GEOMETRIC_REPRESENTATION_CONTEXT` の行だけこれが抜けており、全出力ファイルが
/// 構文違反のまま書かれていた。OpenCASCADE のパーサが寛容なので FreeCAD の
/// 突き合わせでは表に出ず、24/24 も 7/7 も通り続けていた。厳格なパーサを持つ
/// 他社 CAD では拒否され得るので、ここで文面として押さえる。
fn unparenthesised_complex_entities(step: &str) -> Vec<String> {
    let mut offenders = Vec::new();
    for line in step.lines() {
        let Some(rest) = line.split_once(" = ") else {
            continue;
        };
        if !rest.0.starts_with('#') {
            continue;
        }
        let body = rest.1.trim_end().trim_end_matches(';').trim();
        // 括弧で囲まれていれば複合エンティティとして正しい形。
        if body.starts_with('(') {
            continue;
        }
        // 単純エンティティは `NAME(...)` ちょうどで終わる。最初の '(' に対応する
        // ')' より後ろに中身があれば、それは囲み忘れた複合エンティティである。
        let Some(open) = body.find('(') else {
            continue;
        };
        let mut depth = 0usize;
        let mut close = None;
        for (index, ch) in body[open..].char_indices() {
            match ch {
                '(' => depth += 1,
                ')' => {
                    depth -= 1;
                    if depth == 0 {
                        close = Some(open + index);
                        break;
                    }
                }
                _ => {}
            }
        }
        if let Some(close) = close {
            if !body[close + 1..].trim().is_empty() {
                offenders.push(line.to_string());
            }
        }
    }
    offenders
}

#[test]
fn test_every_complex_entity_instance_is_parenthesised() {
    let subjects: Vec<(&str, zenith_topo::Solid)> = vec![
        ("BOX", PrimitiveBuilder::make_box(20.0, 30.0, 40.0).unwrap()),
        ("CYL", PrimitiveBuilder::make_cylinder(10.0, 40.0).unwrap()),
        ("SPH", PrimitiveBuilder::make_sphere(10.0).unwrap()),
        (
            "DRILLED",
            HoleBuilder::make_drilled_box(30.0, 30.0, 15.0, 5.0).unwrap(),
        ),
    ];

    for (name, solid) in &subjects {
        let step = export(name, solid);
        let offenders = unparenthesised_complex_entities(&step);
        assert!(
            offenders.is_empty(),
            "{name}: {} complex entity instance(s) are missing the enclosing parentheses \
             required by ISO 10303-21, first: {}",
            offenders.len(),
            offenders[0]
        );
    }
}

/// 稜の 2D 境界（p-curve）を書いているか、そして構造が正しいか。
///
/// ここには長らく「p-curve は出さない。OCC 自身も出さないし、無くても
/// 往復する」と書いてありました。往復するのは本当ですが、それは
/// OpenCASCADE が 2D 境界を自分で求め直せるからで、求め直さない読み手には
/// 効きません。
///
/// 構造の要は「1本の稜につき、それを使う面の数だけ p-curve が並ぶ」ことです。
/// 閉じた多様体では稜はちょうど2枚に共有されるので、`SURFACE_CURVE` ごとに
/// `PCURVE` はちょうど2本になります。1本しか無ければ、片側の面の 2D 境界を
/// 落としています。
#[test]
fn test_every_edge_carries_a_pcurve_for_each_face_that_uses_it() {
    for (name, solid) in [
        ("BOX", PrimitiveBuilder::make_box(20.0, 30.0, 40.0).unwrap()),
        ("CYL", PrimitiveBuilder::make_cylinder(10.0, 40.0).unwrap()),
        ("SPH", PrimitiveBuilder::make_sphere(10.0).unwrap()),
        (
            "DRILLED",
            HoleBuilder::make_drilled_box(30.0, 30.0, 15.0, 5.0).unwrap(),
        ),
    ] {
        let step = export(name, &solid);

        let edge_curves = step.matches("= EDGE_CURVE").count();
        let surface_curves = step.matches("= SURFACE_CURVE").count();
        assert_eq!(
            surface_curves, edge_curves,
            "{name}: every edge should carry its 2D boundary, \
             {surface_curves} of {edge_curves} do"
        );

        for line in step.lines().filter(|line| line.contains("= SURFACE_CURVE")) {
            // SURFACE_CURVE('',#3d,(#p1,#p2),.PCURVE_S1.)
            let inside = line
                .split_once(",(")
                .and_then(|(_, rest)| rest.split_once("),"))
                .map(|(list, _)| list)
                .unwrap_or("");
            let count = inside.split(',').filter(|part| part.trim().starts_with('#')).count();
            assert_eq!(
                count, 2,
                "{name}: an edge is shared by two faces, so it needs two p-curves; \
                 this one has {count}: {line}"
            );
        }
    }
}

/// 2D 曲線は 2 座標で書くこと。
///
/// `CARTESIAN_POINT` に 3 つ書くと、読み手は 3D 曲線として解釈して
/// パラメータ空間の点にならない。
#[test]
fn test_parametric_points_have_two_coordinates() {
    let step = export("CYL", &PrimitiveBuilder::make_cylinder(10.0, 40.0).unwrap());
    assert!(
        step.contains("PARAMETRIC_REPRESENTATION_CONTEXT()"),
        "the 2D curves need a parametric context to live in"
    );
    // `CARTESIAN_POINT('',(u,v))` はカンマが2つ、3D は3つ。
    let two_coordinate_points = step
        .lines()
        .filter(|line| line.contains("= CARTESIAN_POINT"))
        .filter(|line| line.matches(',').count() == 2)
        .count();
    assert!(
        two_coordinate_points > 0,
        "writing p-curves means writing points in 2D"
    );
}
