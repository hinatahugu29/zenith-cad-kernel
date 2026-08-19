//! Reports how far the exact boolean pipeline gets on the cases that matter.
//!
//! Drilling a hole through a block is the most ordinary boolean a CAD kernel is
//! asked for, and it currently fails. The pipeline already counts what it did
//! at each stage, so this prints those counts rather than guessing which stage
//! gives up.
//!
//! Run with: cargo run --release -p zenith_algo --example boolean_pipeline_probe

use zenith_algo::{
    BooleanEngine, BooleanOpType, BrepIntersectionBuilder, BrepTransform, FaceIntersectionKind,
    PrimitiveBuilder,
};
use zenith_math::{Tolerance, Vec3};
use zenith_topo::{FaceGeometry, Solid};

fn face_kind(solid: &Solid, index: usize) -> &'static str {
    match &solid.outer_shell.faces[index].geometry {
        FaceGeometry::Plane(_) => "plane",
        FaceGeometry::Nurbs(_) => "nurbs",
        _ => "other",
    }
}

fn probe(name: &str, a: &Solid, b: &Solid, op: BooleanOpType) {
    let tol = Tolerance::default();
    println!("=== {name} ({op:?})");

    let candidates = BrepIntersectionBuilder::collect_face_pair_candidates(
        &a.outer_shell.faces,
        &b.outer_shell.faces,
        &tol,
    );

    let mut supported = 0;
    let mut unsupported = 0;
    let mut by_pair: Vec<String> = Vec::new();
    for candidate in &candidates {
        let label = format!(
            "{}x{}",
            face_kind(a, candidate.face_a_index),
            face_kind(b, candidate.face_b_index)
        );
        if matches!(candidate.kind, FaceIntersectionKind::Unsupported) {
            unsupported += 1;
            by_pair.push(format!("{label}:unsupported"));
        } else {
            supported += 1;
        }
    }
    by_pair.sort();
    by_pair.dedup();

    println!(
        "    face-pair candidates {} ({supported} usable, {unsupported} unsupported)",
        candidates.len()
    );
    if !by_pair.is_empty() {
        println!("    unsupported pair kinds: {}", by_pair.join(", "));
    }

    match BooleanEngine::prepare_exact_boolean(a, b, op, &tol) {
        Ok(report) => {
            println!(
                "    intersection edges {}, planar split candidates {}, classified {}",
                report.intersection_edge_candidate_count,
                report.planar_split_candidate_count,
                report.classified_split_candidate_count
            );
            println!(
                "    batch splits: {} faces touched, {} applied, {} skipped",
                report.planar_batch_split_face_count,
                report.planar_batch_applied_split_count,
                report.planar_batch_skipped_split_count
            );
            println!(
                "    selected face pieces {}, cap loops {}, cap faces {}",
                report.selected_face_piece_count,
                report.planar_cap_loop_count,
                report.planar_cap_face_count
            );
            println!(
                "    stitching: {} unmatched, {} non-manifold, {} same-direction edge uses",
                report.selected_face_unmatched_edge_use_count,
                report.selected_face_non_manifold_edge_use_count,
                report.selected_face_same_direction_edge_use_count
            );
        }
        Err(err) => println!("    preparation failed: {err}"),
    }

    match BooleanEngine::boolean_solids_exact_result(a, b, op, &tol) {
        Ok(result) => println!("    RESULT: {} solid(s)", result.solids.len()),
        Err(err) => println!(
            "    RESULT: error - {}",
            err.chars().take(90).collect::<String>()
        ),
    }
    println!();
}

fn main() {
    let block = PrimitiveBuilder::make_box(40.0, 40.0, 20.0).unwrap();
    let drill = BrepTransform::translate_solid(
        &PrimitiveBuilder::make_cylinder(6.0, 60.0).unwrap(),
        Vec3::new(20.0, 20.0, -20.0),
    );

    probe(
        "block minus through drill",
        &block,
        &drill,
        BooleanOpType::Difference,
    );
    probe("block union drill", &block, &drill, BooleanOpType::Union);
    probe(
        "block intersect drill",
        &block,
        &drill,
        BooleanOpType::Intersection,
    );

    // 座ぐり: 既に穴のある板へ、同軸で太い浅い穴を足す。
    let pilot = BrepTransform::translate_solid(
        &PrimitiveBuilder::make_cylinder(5.0, 60.0).unwrap(),
        Vec3::new(20.0, 20.0, -20.0),
    );
    let small_block = PrimitiveBuilder::make_box(40.0, 40.0, 20.0).unwrap();
    if let Ok(drilled) = BooleanEngine::boolean_solids_exact_result(
        &small_block,
        &pilot,
        BooleanOpType::Difference,
        &Tolerance::default(),
    ) {
        let drilled = drilled.solids.into_iter().next().unwrap();
        let counterbore = BrepTransform::translate_solid(
            &PrimitiveBuilder::make_cylinder(9.0, 40.0).unwrap(),
            Vec3::new(20.0, 20.0, 14.0),
        );
        probe(
            "counterbore on an already drilled block",
            &drilled,
            &counterbore,
            BooleanOpType::Difference,
        );
    }

    // 面が接しているだけで重なりがない配置。差は A そのものになるはず。
    let boxa = PrimitiveBuilder::make_box(20.0, 20.0, 20.0).unwrap();
    let flush = BrepTransform::translate_solid(&boxa, Vec3::new(20.0, 0.0, 0.0));
    probe("flush boxes", &boxa, &flush, BooleanOpType::Difference);

    // 任意角度で重なるボックス同士。平面同士だけで済むのに未対応。
    let rotated = BrepTransform::transform_solid(
        &BrepTransform::translate_solid(&boxa, Vec3::new(10.0, 10.0, 0.0)),
        &zenith_math::Transform3::from_axis_angle(
            &Vec3::new(0.0, 0.0, 1.0),
            std::f64::consts::FRAC_PI_4,
        ),
    )
    .unwrap();
    probe("rotated boxes", &boxa, &rotated, BooleanOpType::Union);

    let cyl_a = PrimitiveBuilder::make_cylinder(10.0, 40.0).unwrap();
    let cyl_b = BrepTransform::transform_solid(
        &BrepTransform::translate_solid(&cyl_a, Vec3::new(0.0, -20.0, 20.0)),
        &zenith_math::Transform3::from_axis_angle(
            &Vec3::new(1.0, 0.0, 0.0),
            std::f64::consts::FRAC_PI_2,
        ),
    )
    .unwrap();
    probe("cylinder cross union", &cyl_a, &cyl_b, BooleanOpType::Union);
}
