//! P-curves have to be right where nobody built them.
//!
//! A face's p-curves were made by projecting each edge at 8 evenly spaced
//! parameters, and shell validation checked them at 8 evenly spaced parameters.
//! The check landed on the very points the curve was built from, where it is
//! exact by construction, and so it reported zero for a curve that swept right
//! round a sphere in between. This measures at counts the construction never
//! used, which is the only way the question means anything.
//!
//! The fixtures were written by OpenCASCADE 7.8 and cover the two paths that
//! were wrong: a spherical face whose boundary walks its seam, and planar caps
//! that came out of a B-spline conversion.

use zenith_algo::PrimitiveBuilder;
use zenith_geom::Surface3;
use zenith_io::StepImporter;
use zenith_math::{Point3, Tolerance};
use zenith_topo::{Face, FaceGeometry, Solid};

/// The worst distance between a face's p-curves and its 3D edges, measured at
/// `samples` evenly spaced parameters per edge.
fn worst_pcurve_distance(face: &Face, samples: usize) -> f64 {
    let tol = Tolerance::default();
    let Ok(pcurves) = face.pcurves(&tol) else {
        return 0.0;
    };
    let evaluate = |u: f64, v: f64| -> Option<Point3> {
        match &face.geometry {
            FaceGeometry::Plane(plane) => Some(plane.evaluate(u, v)),
            FaceGeometry::Nurbs(surface) => Some(surface.evaluate(u, v)),
            _ => None,
        }
    };

    let mut worst: f64 = 0.0;
    for (edge, segment) in face
        .outer_wire
        .edges
        .iter()
        .zip(pcurves.outer_loop.segments.iter())
    {
        let (t_min, t_max) = segment.curve.param_range();
        for step in 0..=samples {
            let fraction = step as f64 / samples as f64;
            let uv = segment.curve.evaluate(t_min + (t_max - t_min) * fraction);
            let Some(from_pcurve) = evaluate(uv.x, uv.y) else {
                return 0.0;
            };
            worst = worst.max((from_pcurve - edge.evaluate_normalized(fraction)).norm());
        }
    }
    worst
}

/// The worst over every face, at several sample counts. Eight is what the
/// p-curves are built from; the rest share only the two endpoints with it.
fn worst_over_solid(solid: &Solid) -> f64 {
    let mut worst: f64 = 0.0;
    for face in &solid.outer_shell.faces {
        for samples in [8usize, 9, 16, 37, 64, 101] {
            worst = worst.max(worst_pcurve_distance(face, samples));
        }
    }
    worst
}

fn read(text: &str, name: &str) -> Solid {
    let solids = StepImporter::import_solids_from_str(text)
        .unwrap_or_else(|err| panic!("{name} should import: {err}"));
    solids.into_iter().next().expect("one solid")
}

#[test]
fn test_a_seam_walking_face_keeps_its_pcurve_on_its_edges() {
    // 半球。球面の外周は継ぎ目の子午線を往復してから赤道を回る。
    // 継ぎ目上の点は領域の両端どちらの名前でも呼べるので、投影が遠いほうの
    // 名前を拾うと、隣の標本との間で p-curve が球を一周してしまう。
    // 半径10の球に対して 20.0、つまり直径ぶん外れていた。
    let solid = read(
        include_str!("fixtures/occ_reference_sphere_capped.step"),
        "sphere_capped",
    );
    let worst = worst_over_solid(&solid);
    assert!(
        worst < 1e-6,
        "capped sphere p-curves stray by {worst:.3e} away from their construction points"
    );
}

#[test]
fn test_a_converted_planar_face_keeps_its_pcurve_on_its_edges() {
    // B-spline 化された平面キャップ。曲面の写像はアフィンなので p-curve は
    // 厳密に作れる。折れ線で近似していたときは円が八角形になり、辺から
    // 0.889 外れ、面積も 282.47（正しくは 314.16）まで落ちていた。
    let solid = read(
        include_str!("fixtures/occ_reference_cylinder_nurbs.step"),
        "cylinder_nurbs",
    );
    let worst = worst_over_solid(&solid);
    assert!(
        worst < 1e-6,
        "converted cylinder p-curves stray by {worst:.3e} away from their construction points"
    );
}

#[test]
fn test_our_own_curved_solids_keep_their_pcurves_on_their_edges() {
    // 自前で作った立体も同じ物差しで測る。読み込んだ形状だけの話ではない。
    let subjects: [(&str, Solid); 4] = [
        ("cylinder", PrimitiveBuilder::make_cylinder(10.0, 40.0).unwrap()),
        ("sphere", PrimitiveBuilder::make_sphere(10.0).unwrap()),
        ("cone", PrimitiveBuilder::make_cone(10.0, 4.0, 20.0).unwrap()),
        ("torus", PrimitiveBuilder::make_torus(12.0, 4.0).unwrap()),
    ];

    for (name, solid) in subjects {
        let worst = worst_over_solid(&solid);
        assert!(
            worst < 1e-6,
            "{name} p-curves stray by {worst:.3e} away from their construction points"
        );
    }
}
