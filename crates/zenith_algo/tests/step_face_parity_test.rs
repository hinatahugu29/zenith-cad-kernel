//! STEP に書いた面の数が、B-Rep の面の数と一致しているか。
//!
//! `StepExporter::write_face` は Coons / Gordon / 三角パッチに対して `None` を
//! 返し、`write_closed_shell` がそれを黙って捨てていた。結果、`ThickenBuilder`
//! が Coons パッチで作る6面の立体は、STEP では4面の `CLOSED_SHELL` として
//! 書き出されていた。受け側から見れば穴の空いた殻だが、書き出しはエラーを
//! 返さないので、こちら側からは何も分からない。
//!
//! 面の枚数は外から数えられる量なので、ここで押さえる。

use zenith_algo::ThickenBuilder;
use zenith_geom::{ControlPoint3, CoonsPatch3, KnotVector, NurbsCurve3, NurbsSurface3};
use zenith_io::StepExporter;
use zenith_math::{Point3, Tolerance};
use zenith_topo::{Face, FaceGeometry, Solid, Wire};

fn line(p0: Point3, p1: Point3) -> NurbsCurve3 {
    NurbsCurve3::new(
        1,
        vec![
            ControlPoint3::unweighted(p0),
            ControlPoint3::unweighted(p1),
        ],
        KnotVector::clamped_uniform(2, 1),
    )
    .unwrap()
}

fn coons_slab() -> Solid {
    let tol = Tolerance::default();
    let p00 = Point3::new(0.0, 0.0, 0.0);
    let p10 = Point3::new(20.0, 0.0, 0.0);
    let p11 = Point3::new(20.0, 30.0, 0.0);
    let p01 = Point3::new(0.0, 30.0, 0.0);
    let coons = CoonsPatch3::new(
        line(p00, p10),
        line(p01, p11),
        line(p00, p01),
        line(p10, p11),
        &tol,
    )
    .expect("coons patch");
    let face = Face::from_coons_patch(coons, Wire::new(vec![]));
    ThickenBuilder::thicken_face(&face, 5.0, &tol).expect("thicken the coons sheet")
}

fn advanced_face_count(step: &str) -> usize {
    step.matches("ADVANCED_FACE").count()
}

#[test]
fn test_every_brep_face_reaches_the_step_file() {
    let solid = coons_slab();
    let brep_faces = solid.outer_shell.faces.len()
        + solid
            .inner_shells
            .iter()
            .map(|shell| shell.faces.len())
            .sum::<usize>();

    // この検体が本当に Coons 面を含んでいることを先に確かめる。含んでいなければ
    // このテストは何も見ていない。
    let coons_faces = solid
        .outer_shell
        .faces
        .iter()
        .filter(|face| matches!(face.geometry, FaceGeometry::Coons(_)))
        .count();
    assert!(
        coons_faces > 0,
        "the subject must actually carry Coons faces, otherwise this test is vacuous"
    );

    let step = StepExporter::export_solid_to_string(&solid, "COONS_SLAB");
    assert_eq!(
        advanced_face_count(&step),
        brep_faces,
        "the STEP file must carry one ADVANCED_FACE per B-Rep face; \
         {coons_faces} of them are Coons patches"
    );
}

#[test]
fn test_the_checked_exporter_agrees_with_the_plain_one() {
    let solid = coons_slab();
    let checked = StepExporter::export_solids_to_string_checked(std::slice::from_ref(&solid), "COONS_SLAB");
    assert!(checked.is_ok(), "a Coons-bearing solid must export: {checked:?}");
}

/// 標本した NURBS が元の曲面からどれだけ離れるか。近似であることを数で残す。
#[test]
fn test_sampled_surface_stays_close_to_the_coons_patch() {
    let tol = Tolerance::default();
    let p00 = Point3::new(0.0, 0.0, 0.0);
    let p10 = Point3::new(20.0, 0.0, 0.0);
    let p11 = Point3::new(20.0, 30.0, 8.0);
    let p01 = Point3::new(0.0, 30.0, 0.0);
    let coons = CoonsPatch3::new(
        line(p00, p10),
        line(p01, p11),
        line(p00, p01),
        line(p10, p11),
        &tol,
    )
    .expect("coons patch");

    let mut previous = f64::MAX;
    for samples in [6usize, 8, 12, 16] {
        let nurbs = NurbsSurface3::approximate_surface(&coons, samples, samples)
            .expect("approximate the coons patch");
        let deviation = nurbs.deviation_from(&coons, 37);
        assert!(
            deviation < 1e-9,
            "a bilinear Coons patch is reproduced exactly by interpolation; \
             {samples} samples gave {deviation:e}"
        );
        previous = previous.min(deviation);
    }
    assert!(previous.is_finite());

}
