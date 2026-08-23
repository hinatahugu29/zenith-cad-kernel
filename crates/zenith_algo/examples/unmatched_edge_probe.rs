//! 縫えなかった稜を、位置と持ち主で名指しする。
//!
//! 断り文が言うのは本数だけです。「3本あぶれた」では、足りないのが面なのか、
//! 向きなのか、そもそも別の場所を指しているのかが分かりません。
//!
//! ここは組み立てに使われた面片を自分で走査し、**どの稜が1回しか使われて
//! いないか**を、中点の座標と、その稜を持っている面片（どちらの立体の、
//! どの領域区分か）と一緒に出します。
//!
//! ```bash
//! cargo run --release -p zenith_algo --example unmatched_edge_probe
//! cargo run --release -p zenith_algo --example unmatched_edge_probe -- sphere corner difference
//! ```

use std::collections::BTreeMap;
use std::path::PathBuf;

use zenith_algo::{
    BooleanOpType, BrepIntersectionBuilder, BrepTransform, PrimitiveBuilder,
};
use zenith_io::StepImporter;
use zenith_math::{Point3, Tolerance, Vec3};
use zenith_tess::{tessellate_solid, TessellationParams, TriangleMesh};
use zenith_topo::Solid;

fn params() -> TessellationParams {
    TessellationParams {
        u_divisions: 64,
        v_divisions: 64,
    }
}

fn mesh_bounds(mesh: &TriangleMesh) -> (Point3, Point3) {
    let mut low = Point3::new(f64::MAX, f64::MAX, f64::MAX);
    let mut high = Point3::new(f64::MIN, f64::MIN, f64::MIN);
    for vertex in &mesh.positions {
        low.x = low.x.min(vertex.x);
        low.y = low.y.min(vertex.y);
        low.z = low.z.min(vertex.z);
        high.x = high.x.max(vertex.x);
        high.y = high.y.max(vertex.y);
        high.z = high.z.max(vertex.z);
    }
    (low, high)
}

fn cutter(kind: &str, low: &Point3, high: &Point3) -> Option<Solid> {
    let size = Vec3::new(high.x - low.x, high.y - low.y, high.z - low.z);
    match kind {
        "slab" => {
            let solid = PrimitiveBuilder::make_box(size.x * 0.6, size.y * 2.0, size.z * 2.0).ok()?;
            Some(BrepTransform::translate_solid(
                &solid,
                Vec3::new(
                    low.x - size.x * 0.11,
                    low.y - size.y * 0.5,
                    low.z - size.z * 0.5,
                ),
            ))
        }
        "drill" => {
            let radius = size.x.min(size.y) * 0.18;
            let solid = PrimitiveBuilder::make_cylinder(radius, size.z * 3.0).ok()?;
            Some(BrepTransform::translate_solid(
                &solid,
                Vec3::new(
                    (low.x + high.x) * 0.5,
                    (low.y + high.y) * 0.5,
                    low.z - size.z,
                ),
            ))
        }
        "corner" => {
            let solid =
                PrimitiveBuilder::make_box(size.x * 0.45, size.y * 0.45, size.z * 0.45).ok()?;
            Some(BrepTransform::translate_solid(
                &solid,
                Vec3::new(
                    high.x - size.x * 0.30,
                    high.y - size.y * 0.30,
                    high.z - size.z * 0.30,
                ),
            ))
        }
        _ => None,
    }
}

/// 稜を座標で束ねる鍵。**辺の実体（id）では束ねられません** — 面片ごとに
/// 作り直された稜は別の id を名乗るので、位置で見ます。
fn key_of(start: Point3, end: Point3, grid: f64) -> (i64, i64, i64, i64, i64, i64) {
    let quantise = |value: f64| (value / grid).round() as i64;
    let a = (quantise(start.x), quantise(start.y), quantise(start.z));
    let b = (quantise(end.x), quantise(end.y), quantise(end.z));
    // 向きに依らず同じ鍵にする。
    if a <= b {
        (a.0, a.1, a.2, b.0, b.1, b.2)
    } else {
        (b.0, b.1, b.2, a.0, a.1, a.2)
    }
}

fn main() {
    let args: Vec<String> = std::env::args().skip(1).collect();
    let subject = args.first().cloned().unwrap_or_else(|| "sphere".to_string());
    let kind = args.get(1).cloned().unwrap_or_else(|| "corner".to_string());
    let op = match args.get(2).map(|s| s.as_str()) {
        Some("union") => BooleanOpType::Union,
        Some("intersection") => BooleanOpType::Intersection,
        _ => BooleanOpType::Difference,
    };

    let tol = Tolerance::default();
    let path = PathBuf::from(concat!(env!("CARGO_MANIFEST_DIR"), "/tests/fixtures"))
        .join(format!("occ_reference_{subject}.step"));
    let Ok(solids) = StepImporter::import_solids_from_file(&path) else {
        println!("{subject} could not be read");
        return;
    };
    let Some(read) = solids.first() else { return };

    // **本番と同じ入力で見ます。** 演算は入口で立体を整えるので、整えずに
    // 走査すると別の走行の稜を数えることになります（4-49）。
    let a = zenith_algo::Regularizer::hold_like_our_own(read, &tol);
    let (low, high) = mesh_bounds(&tessellate_solid(&a, &params()));
    let Some(b) = cutter(&kind, &low, &high) else {
        println!("the cutter could not be built");
        return;
    };
    let b = zenith_algo::Regularizer::hold_like_our_own(&b, &tol);

    let assembly = BrepIntersectionBuilder::collect_boolean_shell_assembly(&a, &b, op, &tol);
    let pieces = &assembly.assembly.selected_face_pieces;
    println!("{subject} / {kind} / {op:?}");
    println!(
        "  {} face piece(s), {} cap face(s), stitch: {} unmatched, {} non-manifold, {} same-direction",
        pieces.len(),
        assembly.assembly.cap_face_count,
        assembly.assembly.stitch_report.unmatched_edge_use_count,
        assembly.assembly.stitch_report.non_manifold_edge_use_count,
        assembly.assembly.stitch_report.same_direction_edge_use_count,
    );

    // 位置で束ねる格子。公差より粗く、形より細かく。
    let extent = (high - low).norm().max(1.0);
    let grid = (tol.linear * 10.0).max(extent * 1e-9);

    let mut uses: BTreeMap<(i64, i64, i64, i64, i64, i64), Vec<(usize, Point3)>> = BTreeMap::new();
    for (index, piece) in pieces.iter().enumerate() {
        for wire in std::iter::once(&piece.face.outer_wire).chain(piece.face.inner_wires.iter()) {
            for oriented in &wire.edges {
                let start = oriented.edge.start_vertex.point;
                let end = oriented.edge.end_vertex.point;
                let middle = oriented.evaluate_normalized(0.5);
                uses.entry(key_of(start, end, grid))
                    .or_default()
                    .push((index, middle));
            }
        }
    }

    println!();
    println!("  edges used exactly once:");
    let mut lonely = 0usize;
    for (_key, users) in &uses {
        if users.len() != 1 {
            continue;
        }
        lonely += 1;
        let (index, middle) = users[0];
        let piece = &pieces[index];
        println!(
            "    piece {index:>3} ({:?}, {:?}{}) mid ({:.4} {:.4} {:.4})",
            piece.operand,
            piece.location,
            if piece.reverse_orientation { ", reversed" } else { "" },
            middle.x,
            middle.y,
            middle.z
        );
        // 端点も出す。**中点が同じでも端点が違えば別の稜**で、相手が
        // いないのはその食い違いのことがあります。
        for wire in std::iter::once(&piece.face.outer_wire).chain(piece.face.inner_wires.iter()) {
            for oriented in &wire.edges {
                let start = oriented.edge.start_vertex.point;
                let end = oriented.edge.end_vertex.point;
                if key_of(start, end, grid) != *_key {
                    continue;
                }
                println!(
                    "          from ({:.4} {:.4} {:.4}) to ({:.4} {:.4} {:.4})",
                    start.x, start.y, start.z, end.x, end.y, end.z
                );
            }
        }
    }
    if lonely == 0 {
        println!("    none");
    }

    println!();
    println!("  face pieces:");
    for (index, piece) in pieces.iter().enumerate() {
        let edges: usize = std::iter::once(&piece.face.outer_wire)
            .chain(piece.face.inner_wires.iter())
            .map(|wire| wire.edges.len())
            .sum();
        println!(
            "    {index:>3} {:?} {:?}{} {edges} edge use(s)",
            piece.operand,
            piece.location,
            if piece.reverse_orientation { " reversed" } else { "" }
        );
        // **輪を丸ごと出す。** 縫えない稜だけ見ていると、「片が足りない」のか
        // 「片の輪が元の稜をそのまま抱えている」のかが分かれません。
        // 実測: 蓋の片が、割ったはずのスプラインを端から端まで持っていました。
        if std::env::args().any(|arg| arg == "--wires") {
            for wire in
                std::iter::once(&piece.face.outer_wire).chain(piece.face.inner_wires.iter())
            {
                for oriented in &wire.edges {
                    let (t0, t1) = oriented.edge.curve.param_range();
                    let start = oriented.edge.curve.evaluate(t0);
                    let end = oriented.edge.curve.evaluate(t1);
                    println!(
                        "        ({:8.4} {:8.4} {:8.4}) -> ({:8.4} {:8.4} {:8.4})",
                        start.x, start.y, start.z, end.x, end.y, end.z
                    );
                }
            }
        }
    }
}
