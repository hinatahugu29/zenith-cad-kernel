//! 交線を、**端点まで**並べる。
//!
//! 「交線 N 本、蓋 0 枚」で止まる配置では、N 本が閉じた輪になっていない
//! ことがあります。蓋は閉じた輪からしか作れないので、そこが切れていれば
//! 切り口の面は作れず、縫合は未整合の稜を残します。
//!
//! **どの端点で輪が切れているか**は、本数だけでは分かりません。ここは
//! 1本ずつ端点を出し、他の稜と繋がっていない端点を名指しします。
//!
//! ```bash
//! cargo run --release -p zenith_algo --example intersection_edge_probe -- extruded_spline slab
//! ```

use std::path::PathBuf;

use zenith_algo::{BrepIntersectionBuilder, BrepTransform, PrimitiveBuilder, Regularizer};
use zenith_io::StepImporter;
use zenith_math::{Point3, Tolerance, Vec3};
use zenith_tess::{tessellate_solid, TessellationParams, TriangleMesh};
use zenith_topo::Solid;

/// **`foreign_boolean_probe` と同じ刻み**。切り手は境界箱から置くので、
/// ここが違うと同じ名前の配置が別の配置になります（実測でずれていました）。
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

fn faces(solid: &Solid) -> Vec<zenith_topo::Face> {
    solid
        .outer_shell
        .faces
        .iter()
        .cloned()
        .chain(
            solid
                .inner_shells
                .iter()
                .flat_map(|shell| shell.faces.iter().cloned()),
        )
        .collect()
}

fn main() {
    let mut args = std::env::args().skip(1);
    let subject = args.next().unwrap_or_else(|| "extruded_spline".to_string());
    let kind = args.next().unwrap_or_else(|| "slab".to_string());
    let tol = Tolerance::default();

    let path = PathBuf::from(concat!(env!("CARGO_MANIFEST_DIR"), "/tests/fixtures"))
        .join(format!("occ_reference_{subject}.step"));
    let solid = StepImporter::import_solids_from_file(&path)
        .expect("the fixture must be readable")
        .into_iter()
        .next()
        .expect("one solid");

    let mesh = tessellate_solid(&solid, &params());
    let (low, high) = mesh_bounds(&mesh);
    let cutter = cutter(&kind, &low, &high).expect("a cutter");

    let held_a = Regularizer::hold_like_our_own(&solid, &tol);
    let held_b = Regularizer::hold_like_our_own(&cutter, &tol);
    let faces_a = faces(&held_a);
    let faces_b = faces(&held_b);

    println!("{subject} / {kind}");
    println!("  A as held: {} face(s)", faces_a.len());
    println!("  B as held: {} face(s)", faces_b.len());

    let edges =
        BrepIntersectionBuilder::collect_intersection_edge_candidates(&faces_a, &faces_b, &tol);
    println!("\n  {} intersection edge(s)", edges.len());
    let mut ends: Vec<(Point3, Point3)> = Vec::new();
    for candidate in &edges {
        let (t0, t1) = candidate.edge.curve.param_range();
        let start = candidate.edge.curve.evaluate(t0);
        let end = candidate.edge.curve.evaluate(t1);
        println!(
            "    A{:<3} x B{:<3} deg {}  ({:8.4} {:8.4} {:8.4}) -> ({:8.4} {:8.4} {:8.4})",
            candidate.face_a_index,
            candidate.face_b_index,
            candidate.edge.curve.degree,
            start.x,
            start.y,
            start.z,
            end.x,
            end.y,
            end.z
        );
        ends.push((start, end));
    }

    // **端点が何本の稜に共有されているか。** 蓋を作るには交線が閉じた輪に
    // ならなければならず、閉じているかどうかはここで決まります。ちょうど 2 本
    // でない端点があれば、そこで輪は閉じません。
    split_report(&faces_a, &faces_b, &tol);
    refusal_report(&faces_a, &faces_b, &edges, &tol);
    chain_report(&faces_a, &faces_b, &edges, &tol);
    println!("\n  endpoints shared by exactly one edge (the loop breaks here):");
    let mut lonely = 0usize;
    for (index, point) in ends
        .iter()
        .flat_map(|(a, b)| [*a, *b])
        .enumerate()
    {
        let touching = ends
            .iter()
            .flat_map(|(a, b)| [*a, *b])
            .filter(|other| (*other - point).norm() <= tol.linear * 1000.0)
            .count();
        if touching < 2 {
            println!(
                "    edge {} end {}  ({:8.4} {:8.4} {:8.4})",
                index / 2,
                index % 2,
                point.x,
                point.y,
                point.z
            );
            lonely += 1;
        }
    }
    if lonely == 0 {
        println!("    none; every endpoint meets another edge");
    }
}

/// 交線を渡された面が、実際に割れたかどうか。
///
/// 「交線 N 本」まで来ていても、面が割れていなければ切り口は縫えません。
/// **どの面が割れ、どの面が割れなかったか**は、段の合計（"applied batch
/// splits"）では分かりません。
fn split_report(
    faces_a: &[zenith_topo::Face],
    faces_b: &[zenith_topo::Face],
    tol: &Tolerance,
) {
    let splits = BrepIntersectionBuilder::collect_planar_face_batch_splits(faces_a, faces_b, tol);
    for (label, batch, faces) in [
        ("A", &splits.splits_a, faces_a),
        ("B", &splits.splits_b, faces_b),
    ] {
        println!("\n  {label} faces that were split:");
        if batch.is_empty() {
            println!("    none");
        }
        for split in batch {
            println!(
                "    {label}{:<3} {} split edge(s) -> {} piece(s) (applied {}, skipped {})",
                split.face_index,
                split.split_edge_count,
                split.result.faces.len(),
                split.result.applied_split_count,
                split.result.skipped_split_count
            );
        }
        let _ = faces;
    }
}

/// 割れなかった面については、**断り文をそのまま**出す。
fn refusal_report(
    faces_a: &[zenith_topo::Face],
    faces_b: &[zenith_topo::Face],
    edges: &[zenith_algo::IntersectionEdgeCandidate],
    tol: &Tolerance,
) {
    println!("\n  what each face said when asked to split:");
    for candidate in edges {
        for (label, face) in [
            ("A", &faces_a[candidate.face_a_index]),
            ("B", &faces_b[candidate.face_b_index]),
        ] {
            let index = if label == "A" {
                candidate.face_a_index
            } else {
                candidate.face_b_index
            };
            match BrepIntersectionBuilder::split_face_by_edge(face, &candidate.edge, tol) {
                Ok(pieces) => println!("    {label}{index:<3} -> {} piece(s)", pieces.len()),
                Err(err) => println!(
                    "    {label}{index:<3} -> {}",
                    err.chars().take(400).collect::<String>()
                ),
            }
        }
    }
}

/// 面に来た交線を**まとめて**1本の切り込みとして当てたとき、何が起きるか。
///
/// 曲面同士の交線は相手のパッチの境界で細切れになって届くので、1本ずつでは
/// どれも面の内側で終わります。最後の受け皿は鎖にまとめて当て直しますが、
/// そこが何と言っているかは段の合計には出ません。
fn chain_report(
    faces_a: &[zenith_topo::Face],
    faces_b: &[zenith_topo::Face],
    edges: &[zenith_algo::IntersectionEdgeCandidate],
    tol: &Tolerance,
) {
    use std::collections::BTreeMap;
    let mut by_a: BTreeMap<usize, Vec<zenith_topo::Edge>> = BTreeMap::new();
    let mut by_b: BTreeMap<usize, Vec<zenith_topo::Edge>> = BTreeMap::new();
    for candidate in edges {
        by_a.entry(candidate.face_a_index)
            .or_default()
            .push(candidate.edge.clone());
        by_b.entry(candidate.face_b_index)
            .or_default()
            .push(candidate.edge.clone());
    }

    println!("\n  the whole set of edges on a face, taken as one chain:");
    for (label, groups, faces) in [("A", &by_a, faces_a), ("B", &by_b, faces_b)] {
        for (index, group) in groups {
            match zenith_algo::FaceSplitter::split_by_chain(&faces[*index], group, tol) {
                Ok((pieces, report)) => println!(
                    "    {label}{index:<3} {} edge(s) -> {} piece(s), area residual {:.3e}",
                    group.len(),
                    pieces.len(),
                    report.area_residual
                ),
                Err(err) => println!(
                    "    {label}{index:<3} {} edge(s) -> {}",
                    group.len(),
                    err.chars().take(200).collect::<String>()
                ),
            }
        }
    }
}
