//! Lists exactly which face pieces the boolean selection keeps.
//!
//! Drilling a block should keep ten faces: the block's six, with the two the
//! drill passes through carrying a hole, plus the drill's four side patches
//! reversed. The selection keeps fourteen, so this prints every piece with the
//! region it was classified into, to find the four that do not belong.
//!
//! Run with: cargo run --release -p zenith_algo --example boolean_selection_probe

use zenith_algo::{
    BooleanOpType, BrepIntersectionBuilder, BrepTransform, MassCalculator, PrimitiveBuilder,
};
use zenith_math::{Tolerance, Vec3};
use zenith_tess::TessellationParams;
use zenith_topo::FaceGeometry;

fn main() {
    let tol = Tolerance::default();

    // 既定は穴あけ。引数 "rotated" で回転ボックスの和集合を見る。
    let case = std::env::args().nth(1).unwrap_or_else(|| "drill".to_string());

    let (solid_a, solid_b, op) = if case == "rotated" {
        let boxa = PrimitiveBuilder::make_box(20.0, 20.0, 20.0).unwrap();
        let rotated = BrepTransform::transform_solid(
            &BrepTransform::translate_solid(&boxa, Vec3::new(10.0, 10.0, 0.0)),
            &zenith_math::Transform3::from_axis_angle(
                &Vec3::new(0.0, 0.0, 1.0),
                std::f64::consts::FRAC_PI_4,
            ),
        )
        .unwrap();
        (boxa, rotated, BooleanOpType::Union)
    } else {
        let block = PrimitiveBuilder::make_box(40.0, 40.0, 20.0).unwrap();
        let drill = BrepTransform::translate_solid(
            &PrimitiveBuilder::make_cylinder(6.0, 60.0).unwrap(),
            Vec3::new(20.0, 20.0, -20.0),
        );
        (block, drill, BooleanOpType::Difference)
    };
    let (block, drill) = (solid_a, solid_b);
    println!("case: {case} ({op:?})");

    let selection = BrepIntersectionBuilder::collect_selected_boolean_face_pieces(
        &block, &drill, op, &tol,
    );

    println!("selected {} face pieces", selection.selected_face_pieces.len());
    println!(
        "{:<4} {:<8} {:<10} {:<8} {:>12} {:>7} {:>7}",
        "#", "operand", "region", "kind", "area", "outer", "holes"
    );
    println!("{}", "-".repeat(70));

    let params = TessellationParams {
        u_divisions: 16,
        v_divisions: 16,
    };

    for (index, piece) in selection.selected_face_pieces.iter().enumerate() {
        let kind = match &piece.face.geometry {
            FaceGeometry::Plane(_) => "plane",
            FaceGeometry::Nurbs(_) => "nurbs",
            _ => "other",
        };
        let (area, _volume) = MassCalculator::compute_face_integral(&piece.face, &params);

        let mut z_min = f64::INFINITY;
        let mut z_max = f64::NEG_INFINITY;
        for point in piece.face.outer_wire.sample_points(8) {
            z_min = z_min.min(point.z);
            z_max = z_max.max(point.z);
        }

        println!(
            "{index:<4} {:<8} {:<10} {kind:<8} {area:>12.4} {:>7} {:>7}   z [{z_min:.2}, {z_max:.2}]",
            format!("{:?}", piece.operand),
            format!("{:?}", piece.location),
            piece.face.outer_wire.edges.len(),
            piece.face.inner_wires.len()
        );
    }

    println!();
    println!("batch splits on operand A: {} entries", selection.batch_splits.splits_a.len());
    for split in &selection.batch_splits.splits_a {
        println!(
            "    face {} split by {} edge(s) -> {} piece(s), applied {}, skipped {}",
            split.face_index,
            split.split_edge_count,
            split.result.faces.len(),
            split.result.applied_split_count,
            split.result.skipped_split_count
        );
    }
    println!("batch splits on operand B: {} entries", selection.batch_splits.splits_b.len());
    for split in &selection.batch_splits.splits_b {
        println!(
            "    face {} split by {} edge(s) -> {} piece(s), applied {}, skipped {}",
            split.face_index,
            split.split_edge_count,
            split.result.faces.len(),
            split.result.applied_split_count,
            split.result.skipped_split_count
        );
    }

    println!();
    let report = &selection.stitch_report;
    println!("stitching diagnosis: {report:?}");
}
