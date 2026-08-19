//! Prints why a face refuses to be split.
//!
//! The batch splitter swallows the reason and reports only a count, so a face
//! that visibly ought to split and does not gives no clue. This calls the
//! splitter directly and shows the message.
//!
//! Run with: cargo run --release -p zenith_algo --example split_error_probe

use std::collections::BTreeMap;

use zenith_algo::{BrepIntersectionBuilder, BrepTransform, PrimitiveBuilder};
use zenith_math::{Tolerance, Vec3};
use zenith_topo::{FaceGeometry, Solid};

fn attempt(label: &str, solid: &Solid, index: usize, edges: &[zenith_topo::Edge], tol: &Tolerance) {
    let face = &solid.outer_shell.faces[index];
    let kind = match &face.geometry {
        FaceGeometry::Plane(_) => "plane",
        FaceGeometry::Nurbs(_) => "nurbs",
        _ => "other",
    };
    println!("    {label}{index} ({kind}), {} segment(s)", edges.len());

    for (edge_index, edge) in edges.iter().enumerate() {
        match BrepIntersectionBuilder::split_face_by_edge(face, edge, tol) {
            Ok(pieces) => println!("        edge {edge_index}: split into {} piece(s)", pieces.len()),
            Err(err) => println!("        edge {edge_index}: {err}"),
        }
    }

    if edges.len() >= 2 {
        match BrepIntersectionBuilder::split_planar_face_by_edge_chain(face, edges, tol) {
            Ok(pieces) => println!("        chain of all: {} piece(s)", pieces.len()),
            Err(err) => println!("        chain of all: {err}"),
        }
    }

    match BrepIntersectionBuilder::split_face_by_edges(face, edges, tol) {
        Ok(result) => println!(
            "        batch: {} piece(s), applied {}, skipped {}",
            result.faces.len(),
            result.applied_split_count,
            result.skipped_split_count
        ),
        Err(err) => println!("        batch: {err}"),
    }
}

fn main() {
    let tol = Tolerance::default();
    let boxa = PrimitiveBuilder::make_box(20.0, 20.0, 20.0).unwrap();
    let rotated = BrepTransform::transform_solid(
        &BrepTransform::translate_solid(&boxa, Vec3::new(10.0, 10.0, 0.0)),
        &zenith_math::Transform3::from_axis_angle(
            &Vec3::new(0.0, 0.0, 1.0),
            std::f64::consts::FRAC_PI_4,
        ),
    )
    .unwrap();
    let lifted = BrepTransform::translate_solid(&rotated, Vec3::new(0.0, 0.0, 7.0));

    let candidates = BrepIntersectionBuilder::collect_intersection_edge_candidates(
        &boxa.outer_shell.faces,
        &lifted.outer_shell.faces,
        &tol,
    );

    let mut by_a: BTreeMap<usize, Vec<zenith_topo::Edge>> = BTreeMap::new();
    let mut by_b: BTreeMap<usize, Vec<zenith_topo::Edge>> = BTreeMap::new();
    for candidate in &candidates {
        by_a.entry(candidate.face_a_index)
            .or_default()
            .push(candidate.edge.clone());
        by_b.entry(candidate.face_b_index)
            .or_default()
            .push(candidate.edge.clone());
    }

    println!("operand A:");
    for (index, edges) in &by_a {
        attempt("A", &boxa, *index, edges, &tol);
    }

    println!("operand B:");
    for (index, edges) in &by_b {
        attempt("B", &lifted, *index, edges, &tol);
    }
}
