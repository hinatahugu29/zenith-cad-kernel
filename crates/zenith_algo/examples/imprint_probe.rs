//! Looks at the intersection segments a face actually receives.
//!
//! Splitting a face needs curves that run from its boundary to its boundary.
//! Rotated-box booleans skip most of their splits, and the question is whether
//! the segments handed to the splitter are incomplete or merely need
//! assembling. This prints, per face, every intersection segment with whether
//! each end lands on that face's boundary.
//!
//! Run with: cargo run --release -p zenith_algo --example imprint_probe

use std::collections::BTreeMap;

use zenith_algo::{BrepIntersectionBuilder, BrepTransform, PrimitiveBuilder};
use zenith_math::{Point3, Tolerance, Vec3};
use zenith_topo::{Face, FaceGeometry, Solid};

/// Distance from a point to the face's boundary, measured against the polyline
/// through sampled points rather than to the samples themselves. Measuring to
/// the nearest sample overstates the distance by up to half the sample spacing,
/// which is enough to make a point sitting exactly on a long edge look like it
/// is floating in the interior.
fn distance_to_boundary(face: &Face, point: Point3) -> f64 {
    let mut best = f64::INFINITY;
    for oriented in &face.outer_wire.edges {
        let curve = &oriented.edge.curve;
        let (t_min, t_max) = curve.param_range();
        const SAMPLES: usize = 64;

        let mut previous = curve.evaluate(t_min);
        for index in 1..=SAMPLES {
            let t = t_min + (t_max - t_min) * (index as f64 / SAMPLES as f64);
            let current = curve.evaluate(t);

            let segment = current - previous;
            let length_squared = segment.norm_squared();
            let distance = if length_squared <= f64::EPSILON {
                (point - previous).norm()
            } else {
                let s = ((point - previous).dot(&segment) / length_squared).clamp(0.0, 1.0);
                (point - (previous + segment * s)).norm()
            };
            best = best.min(distance);
            previous = current;
        }
    }
    best
}

fn face_kind(face: &Face) -> &'static str {
    match &face.geometry {
        FaceGeometry::Plane(_) => "plane",
        FaceGeometry::Nurbs(_) => "nurbs",
        _ => "other",
    }
}

fn report(name: &str, a: &Solid, b: &Solid) {
    let tol = Tolerance::default();
    println!("=== {name}");

    let candidates = BrepIntersectionBuilder::collect_intersection_edge_candidates(
        &a.outer_shell.faces,
        &b.outer_shell.faces,
        &tol,
    );
    println!("    {} intersection edge(s) in total", candidates.len());

    let mut by_face_a: BTreeMap<usize, Vec<_>> = BTreeMap::new();
    for candidate in &candidates {
        by_face_a
            .entry(candidate.face_a_index)
            .or_default()
            .push(candidate);
    }

    for (face_index, entries) in &by_face_a {
        let face = &a.outer_shell.faces[*face_index];
        println!(
            "    face A{face_index} ({}) receives {} segment(s)",
            face_kind(face),
            entries.len()
        );

        for entry in entries {
            let start = entry.edge.start_vertex.point;
            let end = entry.edge.end_vertex.point;
            let start_gap = distance_to_boundary(face, start);
            let end_gap = distance_to_boundary(face, end);
            let verdict = match (start_gap <= tol.linear * 10.0, end_gap <= tol.linear * 10.0) {
                (true, true) => "crosses the face",
                (true, false) => "starts on the boundary, ends inside",
                (false, true) => "starts inside, ends on the boundary",
                (false, false) => "both ends inside",
            };
            println!(
                "        ({:.3},{:.3},{:.3}) -> ({:.3},{:.3},{:.3})  len {:.3}  gaps {:.3}/{:.3}  {verdict}",
                start.x,
                start.y,
                start.z,
                end.x,
                end.y,
                end.z,
                (end - start).norm(),
                start_gap,
                end_gap
            );
        }
    }
    println!();
}

fn main() {
    let boxa = PrimitiveBuilder::make_box(20.0, 20.0, 20.0).unwrap();
    let rotated = BrepTransform::transform_solid(
        &BrepTransform::translate_solid(&boxa, Vec3::new(10.0, 10.0, 0.0)),
        &zenith_math::Transform3::from_axis_angle(
            &Vec3::new(0.0, 0.0, 1.0),
            std::f64::consts::FRAC_PI_4,
        ),
    )
    .unwrap();
    report("rotated boxes", &boxa, &rotated);

    // 比較対象: 既に成功している角重なりのボックス同士。
    let corner = BrepTransform::translate_solid(&boxa, Vec3::new(10.0, 10.0, 10.0));
    report("corner-overlapping boxes (already works)", &boxa, &corner);
}
