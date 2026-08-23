//! 読んだ立体を切るとき、パイプラインがどこで足りなくなるかを出す。
//!
//! `foreign_boolean_probe` は 74 件中 70 件を断ります。断ることは欠陥では
//! ありませんが、**どの段で足りないのかは断り文だけでは分かりません**。
//! ここは `boolean_pipeline_probe` と同じ内訳を、ビルダーの立体ではなく
//! **読んだ立体**に対して出します。
//!
//! 同じ形をビルダーでも作って並べます。読んだほうだけ落ちるなら、原因は
//! 形ではなく**その形の持ち方**（面の分かれ方・継ぎ目・曲面の種類）です。
//!
//! ```bash
//! cargo run --release -p zenith_algo --example foreign_boolean_why
//! cargo run --release -p zenith_algo --example foreign_boolean_why -- cone slab difference
//! ```

use std::path::PathBuf;

use zenith_algo::{
    BooleanEngine, BooleanOpType, BrepIntersectionBuilder, BrepTransform, FaceIntersectionKind,
    PrimitiveBuilder,
};
use zenith_io::StepImporter;
use zenith_math::{Point3, Tolerance, Vec3};
use zenith_tess::{tessellate_solid, TessellationParams, TriangleMesh};
use zenith_topo::{FaceGeometry, Solid};

fn params() -> TessellationParams {
    TessellationParams {
        u_divisions: 32,
        v_divisions: 32,
    }
}

fn face_kind(solid: &Solid, index: usize) -> &'static str {
    match &solid.outer_shell.faces[index].geometry {
        FaceGeometry::Plane(_) => "plane",
        FaceGeometry::Nurbs(_) => "nurbs",
        _ => "other",
    }
}

/// 面の分かれ方を1行で。読んだ立体とビルダーの立体を並べるための物差し。
fn shape_of(solid: &Solid) -> String {
    let mut planes = 0;
    let mut nurbs = 0;
    let mut other = 0;
    let mut seam_only = 0;
    let mut edges = 0;
    for face in &solid.outer_shell.faces {
        match &face.geometry {
            FaceGeometry::Plane(_) => planes += 1,
            FaceGeometry::Nurbs(_) => nurbs += 1,
            _ => other += 1,
        }
        if face.has_seam_only_boundary(1e-6) {
            seam_only += 1;
        }
        edges += face.outer_wire.edges.len();
        for wire in &face.inner_wires {
            edges += wire.edges.len();
        }
    }
    format!(
        "{} face(s) ({planes} plane, {nurbs} nurbs, {other} other), {seam_only} closed over the whole surface, {edges} edge use(s)",
        solid.outer_shell.faces.len()
    )
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

fn cutter(kind: &str, low: &Point3, high: &Point3) -> Result<Solid, String> {
    let size = Vec3::new(high.x - low.x, high.y - low.y, high.z - low.z);
    match kind {
        "slab" => {
            let solid = PrimitiveBuilder::make_box(size.x * 0.6, size.y * 2.0, size.z * 2.0)?;
            Ok(BrepTransform::translate_solid(
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
            let solid = PrimitiveBuilder::make_cylinder(radius, size.z * 3.0)?;
            Ok(BrepTransform::translate_solid(
                &solid,
                Vec3::new(
                    (low.x + high.x) * 0.5,
                    (low.y + high.y) * 0.5,
                    low.z - size.z,
                ),
            ))
        }
        "corner" => {
            let solid = PrimitiveBuilder::make_box(size.x * 0.45, size.y * 0.45, size.z * 0.45)?;
            Ok(BrepTransform::translate_solid(
                &solid,
                Vec3::new(
                    high.x - size.x * 0.30,
                    high.y - size.y * 0.30,
                    high.z - size.z * 0.30,
                ),
            ))
        }
        other => Err(format!("unknown cutter {other}")),
    }
}

fn report(label: &str, a: &Solid, b: &Solid, op: BooleanOpType) {
    let tol = Tolerance::default();
    println!("  --- {label}");
    println!("      A: {}", shape_of(a));

    let candidates = BrepIntersectionBuilder::collect_face_pair_candidates(
        &a.outer_shell.faces,
        &b.outer_shell.faces,
        &tol,
    );
    let mut unsupported_pairs: Vec<String> = Vec::new();
    let mut supported = 0;
    for candidate in &candidates {
        if matches!(candidate.kind, FaceIntersectionKind::Unsupported) {
            unsupported_pairs.push(format!(
                "{}x{}",
                face_kind(a, candidate.face_a_index),
                face_kind(b, candidate.face_b_index)
            ));
        } else {
            supported += 1;
        }
    }
    unsupported_pairs.sort();
    unsupported_pairs.dedup();
    // どのペアが見つかったかを名前で出す。**見つからなかったペア**が効くので、
    // 数だけでは足りません。
    let found: Vec<String> = candidates
        .iter()
        .map(|candidate| {
            format!(
                "A{}({}) x B{}({})",
                candidate.face_a_index,
                face_kind(a, candidate.face_a_index),
                candidate.face_b_index,
                face_kind(b, candidate.face_b_index)
            )
        })
        .collect();
    println!("      pairs found: {}", found.join(", "));
    // ペアに挙がらない面があるので、面ごとの境界箱を出す。候補の絞り込みは
    // これで行われるので、ここが違えば以降は何も起きません。
    for (index, face) in a.outer_shell.faces.iter().enumerate() {
        let mut low = Point3::new(f64::MAX, f64::MAX, f64::MAX);
        let mut high = Point3::new(f64::MIN, f64::MIN, f64::MIN);
        let mut edge_count = 0;
        for wire in std::iter::once(&face.outer_wire).chain(face.inner_wires.iter()) {
            for oriented in &wire.edges {
                edge_count += 1;
                for step in 0..=8 {
                    let point = oriented.evaluate_normalized(step as f64 / 8.0);
                    low.x = low.x.min(point.x);
                    low.y = low.y.min(point.y);
                    low.z = low.z.min(point.z);
                    high.x = high.x.max(point.x);
                    high.y = high.y.max(point.y);
                    high.z = high.z.max(point.z);
                }
            }
        }
        // 支持曲面そのものの広がりも見る。境界だけ狭めて曲面は全周のまま、
        // という持ち方だと、交線を求める段は全周に対して答えます。
        let support = match &face.geometry {
            FaceGeometry::Nurbs(surface) => {
                let ((u0, u1), (v0, v1)) = zenith_geom::Surface3::param_range(surface);
                let mut slow = Point3::new(f64::MAX, f64::MAX, f64::MAX);
                let mut shigh = Point3::new(f64::MIN, f64::MIN, f64::MIN);
                for i in 0..=8 {
                    for j in 0..=8 {
                        let point = zenith_geom::Surface3::evaluate(
                            surface,
                            u0 + (u1 - u0) * i as f64 / 8.0,
                            v0 + (v1 - v0) * j as f64 / 8.0,
                        );
                        slow.x = slow.x.min(point.x);
                        slow.y = slow.y.min(point.y);
                        slow.z = slow.z.min(point.z);
                        shigh.x = shigh.x.max(point.x);
                        shigh.y = shigh.y.max(point.y);
                        shigh.z = shigh.z.max(point.z);
                    }
                }
                format!(
                    "  support ({:.3} {:.3} {:.3})-({:.3} {:.3} {:.3})",
                    slow.x, slow.y, slow.z, shigh.x, shigh.y, shigh.z
                )
            }
            _ => String::new(),
        };
        println!(
            "        A{index} ({}) {edge_count} edge(s) boundary ({:.3} {:.3} {:.3})-({:.3} {:.3} {:.3}){support}",
            face_kind(a, index),
            low.x, low.y, low.z, high.x, high.y, high.z
        );
    }
    println!(
        "      face pairs {} ({supported} usable, {} unsupported{})",
        candidates.len(),
        candidates.len() - supported,
        if unsupported_pairs.is_empty() {
            String::new()
        } else {
            format!(": {}", unsupported_pairs.join(", "))
        }
    );

    match BooleanEngine::prepare_exact_boolean(a, b, op, &tol) {
        Ok(r) => {
            println!(
                "      intersection edges {}, planar split candidates {}, classified {}",
                r.intersection_edge_candidate_count,
                r.planar_split_candidate_count,
                r.classified_split_candidate_count
            );
            println!(
                "      batch splits: {} faces touched, {} applied, {} skipped",
                r.planar_batch_split_face_count,
                r.planar_batch_applied_split_count,
                r.planar_batch_skipped_split_count
            );
            println!(
                "      selected pieces {}, cap loops {}, cap faces {}",
                r.selected_face_piece_count, r.planar_cap_loop_count, r.planar_cap_face_count
            );
            println!(
                "      stitching: {} unmatched, {} non-manifold, {} same-direction",
                r.selected_face_unmatched_edge_use_count,
                r.selected_face_non_manifold_edge_use_count,
                r.selected_face_same_direction_edge_use_count
            );
        }
        Err(err) => println!("      preparation failed: {err}"),
    }

    match BooleanEngine::boolean_solids_exact_result(a, b, op, &tol) {
        Ok(result) => println!("      RESULT: {} solid(s)", result.solids.len()),
        Err(err) => println!(
            "      RESULT: {}",
            err.split(';').next().unwrap_or(&err).chars().take(70).collect::<String>()
        ),
    }
}

/// 制御点が1つの平面に乗っていれば、その平面を返す。
///
/// B-spline 曲面は制御点の凸包に含まれます。制御点が平面上にあるなら、
/// 曲面はその平面から出られません。標本を見て当てるのではなく、
/// **制御点だけで決まります**。
fn plane_of_control_net(surface: &zenith_geom::NurbsSurface3) -> Option<zenith_geom::PlaneSurface3> {
    let points: Vec<Point3> = surface
        .control_points
        .iter()
        .flat_map(|row| row.iter().map(|control| control.point))
        .collect();
    if points.len() < 3 {
        return None;
    }
    let origin = points[0];
    // 原点から最も遠い2点で軸を張る。細長い網でも退化しない選び方。
    let first = points
        .iter()
        .max_by(|a, b| (**a - origin).norm().total_cmp(&(**b - origin).norm()))?;
    let u_axis = *first - origin;
    if u_axis.norm() <= 1e-12 {
        return None;
    }
    let second = points.iter().max_by(|a, b| {
        (*a - origin)
            .cross(&u_axis)
            .norm()
            .total_cmp(&(*b - origin).cross(&u_axis).norm())
    })?;
    let v_axis = *second - origin;
    let normal = u_axis.cross(&v_axis);
    if normal.norm() <= 1e-12 {
        return None;
    }
    let normal = normal / normal.norm();

    // 網の広がりに対して相対で見る。絶対値では大きい形が落ちます。
    let extent = points
        .iter()
        .map(|point| (*point - origin).norm())
        .fold(0.0f64, f64::max)
        .max(1.0);
    for point in &points {
        if (*point - origin).dot(&normal).abs() > extent * 1e-12 {
            return None;
        }
    }
    // 向きは元の曲面から取る。適当に張ると法線が裏返り、面が支持曲面と
    // 食い違って立体そのものが無効になります（実測でそうなりました）。
    let ((u_min, u_max), (v_min, v_max)) = surface.param_range();
    let wanted = zenith_geom::Surface3::normal(
        surface,
        (u_min + u_max) * 0.5,
        (v_min + v_max) * 0.5,
    )?;
    let normal = if normal.dot(&wanted) >= 0.0 {
        normal
    } else {
        -normal
    };
    zenith_geom::PlaneSurface3::new(origin, u_axis, normal.cross(&u_axis))
}

/// 同じ形をビルダーでも作れる検体だけ、並べて比べる。
fn builder_twin(subject: &str) -> Option<Solid> {
    match subject {
        // occ_reference_cone: r10 -> r4, h20
        "cone" => PrimitiveBuilder::make_cone(10.0, 4.0, 20.0).ok(),
        // occ_reference_cylinder_nurbs: r10, h40
        "cylinder_nurbs" => PrimitiveBuilder::make_cylinder(10.0, 40.0).ok(),
        // occ_reference_sphere: r10
        "sphere" => PrimitiveBuilder::make_sphere(10.0).ok(),
        // occ_reference_torus: R12 r4
        "torus" => PrimitiveBuilder::make_torus(12.0, 4.0).ok(),
        _ => None,
    }
}

fn main() {
    let args: Vec<String> = std::env::args().skip(1).collect();
    let subjects: Vec<String> = if args.is_empty() {
        ["cone", "cylinder_nurbs", "sphere", "torus"]
            .iter()
            .map(|s| s.to_string())
            .collect()
    } else {
        vec![args[0].clone()]
    };
    let cutters: Vec<String> = if args.len() > 1 {
        vec![args[1].clone()]
    } else {
        ["slab", "drill", "corner"]
            .iter()
            .map(|s| s.to_string())
            .collect()
    };
    let ops: Vec<(String, BooleanOpType)> = if args.len() > 2 {
        let op = match args[2].as_str() {
            "union" => BooleanOpType::Union,
            "intersection" => BooleanOpType::Intersection,
            _ => BooleanOpType::Difference,
        };
        vec![(args[2].clone(), op)]
    } else {
        vec![("difference".to_string(), BooleanOpType::Difference)]
    };

    for subject in &subjects {
        let path = PathBuf::from(concat!(env!("CARGO_MANIFEST_DIR"), "/tests/fixtures"))
            .join(format!("occ_reference_{subject}.step"));
        let Ok(solids) = StepImporter::import_solids_from_file(&path) else {
            println!("{subject}: could not be read");
            continue;
        };
        let read = &solids[0];
        let (low, high) = mesh_bounds(&tessellate_solid(read, &params()));
        let twin = builder_twin(subject);

        println!("=== {subject}");
        println!("    read    : {}", shape_of(read));
        match &twin {
            Some(twin) => println!("    builder : {}", shape_of(twin)),
            None => println!("    builder : no twin defined"),
        }

        // 全周1枚の面を刻んでから同じことをする。刻むだけで通るなら、
        // 足りないのはブーリアンではなく**入力の持ち方**です。
        let (regularized, regularize_report) =
            zenith_algo::Regularizer::regularize_solid(read, &Tolerance::default());
        println!(
            "    regular : {}  (wrapped faces split {}, left alone {})",
            shape_of(&regularized),
            regularize_report.wrapped_faces_split,
            regularize_report.wrapped_faces_left_alone
        );
        println!(
            "              held by p-curves {}, closed edges split {}",
            regularize_report.faces_held_by_pcurves, regularize_report.closed_edges_split
        );
        for reason in &regularize_report.left_alone_reasons {
            println!("              left alone: {reason}");
        }

        // 制御点が1平面に乗っている NURBS 面を、平面として持ち直した変種。
        // B-spline 曲面は制御点の凸包に含まれるので、制御点が平面上にあれば
        // 曲面もその平面上にあります。**近似ではなく定理**です。
        let mut as_planes = read.clone();
        let mut converted = 0;
        for face in &mut as_planes.outer_shell.faces {
            if let FaceGeometry::Nurbs(surface) = &face.geometry {
                if let Some(plane) = plane_of_control_net(surface) {
                    face.geometry = FaceGeometry::Plane(plane);
                    face.pcurves = face.derive_plane_pcurves().ok();
                    converted += 1;
                }
            }
        }
        println!(
            "    planes  : {}  ({converted} nurbs face(s) recognised as planes)",
            shape_of(&as_planes)
        );

        // 平面として持ち直してから正規化する。平面の p-curve は射影で厳密に
        // 出せるので守りが外れ、上下の円が刻めるようになるはず。ビルダーの
        // 持ち方に一番近い形です。
        let (planes_regular, planes_report) =
            zenith_algo::Regularizer::regularize_solid(&as_planes, &Tolerance::default());
        println!(
            "    pl+reg  : {}  (wrapped faces split {}, left alone {}, closed edges split {}, held by p-curves {})",
            shape_of(&planes_regular),
            planes_report.wrapped_faces_split,
            planes_report.wrapped_faces_left_alone,
            planes_report.closed_edges_split,
            planes_report.faces_held_by_pcurves
        );

        // 仮説の確認だけのための変種。p-curve を捨てると、キャップの守りが
        // 外れて上下の円が刻めるようになります。**捨てると積分が変わる**ので
        // これは答えとして使えません。構造として通るかだけを見ます。
        let mut stripped = read.clone();
        for face in &mut stripped.outer_shell.faces {
            face.pcurves = None;
        }
        let (stripped_regular, stripped_report) =
            zenith_algo::Regularizer::regularize_solid(&stripped, &Tolerance::default());
        println!(
            "    stripped: {}  (wrapped faces split {}, left alone {}, closed edges split {})",
            shape_of(&stripped_regular),
            stripped_report.wrapped_faces_split,
            stripped_report.wrapped_faces_left_alone,
            stripped_report.closed_edges_split
        );

        for kind in &cutters {
            let Ok(b) = cutter(kind, &low, &high) else {
                continue;
            };
            for (op_name, op) in &ops {
                println!("  {kind} / {op_name}");
                report("read", read, &b, *op);
                report("read, regularized", &regularized, &b, *op);
                report("read, planar faces recognised", &as_planes, &b, *op);
                report("read, planes recognised then regularized", &planes_regular, &b, *op);
                report(
                    "read, p-curves dropped then regularized (structure only)",
                    &stripped_regular,
                    &b,
                    *op,
                );
                if let Some(twin) = &twin {
                    report("builder twin", twin, &b, *op);
                }
            }
        }
        println!();
    }

    println!("The twin is the control. Where the builder solid passes and the");
    println!("read one does not, the shape is not what differs - the way the");
    println!("shape is held is.");
}
