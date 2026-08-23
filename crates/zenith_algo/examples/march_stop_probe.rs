//! 交線の辿りが**どこで終わったか**を、面の組ごとに出す。
//!
//! 「交線 N 本」という数え方では、次の3つが区別できません。
//!
//! - 種が見つからず、そもそも辿っていない
//! - 辿ったが、面の**内側**で止まった（切り込みとしては半端）
//! - 辿って、面の縁まで届いた（正しい）
//!
//! `MarchedIntersection` は `stopped_at_boundary` と `stopped_at_tangency` を
//! 持っていますが、ブーリアンはそれを捨てて曲線だけ受け取ります。**面の内側で
//! 終わった交線も、そのまま切り込みとして渡っています。**
//!
//! 実測（`cone_full` を 27 度傾けたドリルで抜く）: 円錐の側面は4分の1面に
//! 割れていて、そのうち1枚には交線が1本も届かず、届いた交線も端が面の縁
//! （`x = 0` と `y = 0` の平面）ではなく内側で止まっています。輪が閉じないので
//! 蓋が作れず、180演算のうち27件がこれで断られます（HANDOVER 4-61、3-N-1）。
//!
//! ```bash
//! cargo run --release -p zenith_algo --example march_stop_probe -- cone_full "tilted drill"
//! ```

use std::path::PathBuf;

use zenith_algo::{BrepTransform, PrimitiveBuilder, Regularizer};
use zenith_geom::{ControlPoint3, IntersectionMarcher, KnotVector, NurbsSurface3, Surface3};
use zenith_io::StepImporter;
use zenith_math::{Point3, Tolerance, Transform3, Vec3};
use zenith_tess::{tessellate_solid, TessellationParams, TriangleMesh};
use zenith_topo::{Face, FaceGeometry, Solid};

/// **`foreign_boolean_probe` と同じ刻み。** 切り手は境界箱から置くので、
/// ここが違うと同じ名前の配置が別の配置になります。
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

/// 切り手を境界箱の中心まわりに 27 度傾ける。`foreign_boolean_probe` と同じ。
fn tilt(solid: &Solid, low: &Point3, high: &Point3) -> Option<Solid> {
    let centre = Vec3::new(
        (low.x + high.x) * 0.5,
        (low.y + high.y) * 0.5,
        (low.z + high.z) * 0.5,
    );
    let axis = Vec3::new(1.0, 1.0, 1.0);
    let transform = Transform3::from_translation(centre)
        .compose(&Transform3::from_axis_angle(&axis, 27f64.to_radians()))
        .compose(&Transform3::from_translation(-centre));
    BrepTransform::transform_solid(solid, &transform).ok()
}

fn cutter(kind: &str, low: &Point3, high: &Point3) -> Option<Solid> {
    if let Some(base) = kind.strip_prefix("tilted ") {
        return tilt(&cutter(base, low, high)?, low, high);
    }
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

fn faces(solid: &Solid) -> Vec<Face> {
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

/// 平面の面を、その境界が占めるぶんちょうどの1次×1次パッチにする。
///
/// 平面のパラメータ範囲は無限なので、そのままではマーチングに渡せません。
/// `brep_intersection` の `planar_face_as_patch` と同じ組み方です。
fn planar_face_as_patch(face: &Face, plane: &zenith_geom::PlaneSurface3) -> Option<NurbsSurface3> {
    let points = face.outer_wire.sample_points(8);
    if points.is_empty() {
        return None;
    }
    let (mut u_min, mut u_max) = (f64::INFINITY, f64::NEG_INFINITY);
    let (mut v_min, mut v_max) = (f64::INFINITY, f64::NEG_INFINITY);
    for point in &points {
        let offset = point - plane.origin;
        let u = offset.dot(&plane.u_axis);
        let v = offset.dot(&plane.v_axis);
        u_min = u_min.min(u);
        u_max = u_max.max(u);
        v_min = v_min.min(v);
        v_max = v_max.max(v);
    }
    if !(u_max > u_min && v_max > v_min) {
        return None;
    }
    let corner = |u: f64, v: f64| ControlPoint3::unweighted(plane.evaluate(u, v));
    NurbsSurface3::new(
        1,
        1,
        vec![
            vec![corner(u_min, v_min), corner(u_min, v_max)],
            vec![corner(u_max, v_min), corner(u_max, v_max)],
        ],
        KnotVector::clamped_uniform(2, 1),
        KnotVector::clamped_uniform(2, 1),
    )
    .ok()
}

fn patch_of(face: &Face) -> Option<NurbsSurface3> {
    match &face.geometry {
        FaceGeometry::Nurbs(surface) => Some(surface.clone()),
        FaceGeometry::Plane(plane) => planar_face_as_patch(face, plane),
        _ => None,
    }
}

fn main() {
    let mut args = std::env::args().skip(1);
    let subject = args.next().unwrap_or_else(|| "cone_full".to_string());
    let kind = args.next().unwrap_or_else(|| "tilted drill".to_string());
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
    let tool = cutter(&kind, &low, &high).expect("a cutter");

    let held_a = Regularizer::hold_like_our_own(&solid, &tol);
    let held_b = Regularizer::hold_like_our_own(&tool, &tol);
    let faces_a = faces(&held_a);
    let faces_b = faces(&held_b);

    println!("{subject} / {kind}");
    println!(
        "  A {} face(s), B {} face(s)\n",
        faces_a.len(),
        faces_b.len()
    );
    println!(
        "{:<10} {:>5} {:>6} {:>7} {:>10} {:>10}  {}",
        "pair", "seeds", "points", "closed", "off/near", "fit", "stopped because"
    );
    println!("{}", "-".repeat(96));

    let deviation_limit = tol.linear;
    let mut traced = 0usize;
    let mut ended_inside = 0usize;

    for (index_a, face_a) in faces_a.iter().enumerate() {
        let Some(patch_a) = patch_of(face_a) else {
            continue;
        };
        for (index_b, face_b) in faces_b.iter().enumerate() {
            let Some(patch_b) = patch_of(face_b) else {
                continue;
            };

            let seeds = IntersectionMarcher::find_seeds(&patch_a, &patch_b, 12, 4);

            // **種が無いことは、交わらないことではありません。** 格子で
            // 両方のパッチを撒いて、いちばん近い点対の距離を測ります。0 に
            // 近いのに種が無いなら、見落としです。
            let grid = 64usize;
            let sample = |patch: &NurbsSurface3| {
                let ((u0, u1), (v0, v1)) = patch.param_range();
                let mut points = Vec::with_capacity((grid + 1) * (grid + 1));
                for i in 0..=grid {
                    for j in 0..=grid {
                        let u = u0 + (u1 - u0) * i as f64 / grid as f64;
                        let v = v0 + (v1 - v0) * j as f64 / grid as f64;
                        points.push(patch.evaluate(u, v));
                    }
                }
                points
            };
            let points_a = sample(&patch_a);
            let points_b = sample(&patch_b);
            let mut closest = f64::INFINITY;
            for a in &points_a {
                for b in &points_b {
                    let distance = (a - b).norm();
                    if distance < closest {
                        closest = distance;
                    }
                }
            }

            if seeds.is_empty() {
                // 格子の刻みより近ければ、交わっている可能性が高い。
                let mark = if closest <= 1.0 {
                    "**no seed, but the patches nearly touch**"
                } else {
                    "no seed (the patches are apart)"
                };
                println!(
                    "A{index_a:<2} x B{index_b:<3} {:>5} {:>6} {:>7} {closest:>10.2e} {:>10}  {mark}",
                    0, "-", "-", "-"
                );
                continue;
            }

            // 歩幅は `intersect_nurbs_patches` と同じ決め方。
            let extent = 40.0_f64;
            let mut step;
            let mut best: Option<(usize, bool, bool, bool, f64, f64)> = None;
            for (seed_u, seed_v) in seeds.iter().take(4) {
                step = (extent * 0.1).max(tol.linear * 100.0);
                for _ in 0..8 {
                    let Some(marched) = IntersectionMarcher::march(
                        &patch_a, &patch_b, *seed_u, *seed_v, step, 2048, &tol,
                    ) else {
                        step *= 0.5;
                        continue;
                    };
                    let fit = IntersectionMarcher::fit_curve(&patch_a, &patch_b, &marched, 3)
                        .map(|(_, deviation)| deviation)
                        .unwrap_or(f64::INFINITY);
                    if index_a == 0 && index_b == 3 && fit_first(&marched) {
                        let first = marched.points[0].point;
                        let last = marched.points[marched.points.len() - 1].point;
                        println!(
                            "      raw march A0 x B3: ({:.4} {:.4} {:.4}) -> ({:.4} {:.4} {:.4})",
                            first.x, first.y, first.z, last.x, last.y, last.z
                        );
                    }
                    let record = (
                        marched.points.len(),
                        marched.closed,
                        marched.stopped_at_boundary,
                        marched.stopped_at_tangency,
                        marched.worst_off_surface,
                        fit,
                    );
                    if best.is_none_or(|current| record.5 < current.5) {
                        best = Some(record);
                    }
                    if fit <= deviation_limit {
                        break;
                    }
                    step *= 0.5;
                }
                if best.map(|record| record.5 <= deviation_limit).unwrap_or(false) {
                    break;
                }
            }

            let Some((points, closed, at_boundary, at_tangency, off, fit)) = best else {
                // **種はあったのに、一度も辿れていません。** ここが表から
                // 抜けていると「交わらない組」と見分けが付きません。
                println!(
                    "A{index_a:<2} x B{index_b:<3} {:>5} {:>6} {:>7} {closest:>10.2e} {:>10}  **seeded, but no march succeeded**",
                    seeds.len(),
                    "-",
                    "-",
                    "-"
                );
                continue;
            };
            traced += 1;
            // **面の縁にも着かず、閉じてもいない交線は、切り込みとして半端です。**
            let inside = !closed && !at_boundary;
            if inside {
                ended_inside += 1;
            }
            let reason = match (closed, at_boundary, at_tangency) {
                (true, _, _) => "closed on itself",
                (_, true, true) => "reached the patch edge (and a tangency)",
                (_, true, false) => "reached the patch edge",
                (_, false, true) => "**a tangency, inside the patch**",
                (_, false, false) => "**stopped inside the patch**",
            };
            println!(
                "A{index_a:<2} x B{index_b:<3} {:>5} {points:>6} {:>7} {off:>10.2e} {fit:>10.2e}  {reason}",
                seeds.len(),
                if closed { "yes" } else { "no" }
            );
        }
    }

    println!("{}", "-".repeat(96));
    println!("{traced} pair(s) traced, {ended_inside} of them ended inside a patch");
    println!();
    for point in [
        Point3::new(1.6256, -3.9533, 11.4511),
        Point3::new(4.1378, 0.4696, 11.6712),
    ] {
        loose_end_report(&faces_a, &faces_b, point);
    }

    println!("
A branch that neither closes nor reaches a patch edge is half a cut.");
    println!("Passing it on as an intersection edge is how the loop fails to close.");
}

/// 宙に浮いた端点が、どのパッチに乗っているかを測る。
///
/// **交線がパッチの縁で終わったなら、その先は隣のパッチに続いています。**
/// 端点は交線の上の点なので、隣の組をそこから辿り始められるはずです。
/// 端点が実際に隣のパッチに乗っているかどうかは、測らないと分かりません。
fn loose_end_report(faces_a: &[Face], faces_b: &[Face], point: Point3) {
    use zenith_geom::ExtremumEngine;
    println!("\n  loose end ({:.4} {:.4} {:.4}) sits on:", point.x, point.y, point.z);
    for (label, faces) in [("A", faces_a), ("B", faces_b)] {
        for (index, face) in faces.iter().enumerate() {
            let Some(patch) = patch_of(face) else { continue };
            let Ok(projection) = ExtremumEngine::point_to_surface(point, &patch, 64, 1e-13) else {
                continue;
            };
            if projection.distance <= 1e-3 {
                let ((u0, u1), (v0, v1)) = patch.param_range();
                let at_edge = |value: f64, low: f64, high: f64| {
                    let margin = (high - low).abs().max(1.0) * 1e-6;
                    (value - low).abs() <= margin || (value - high).abs() <= margin
                };
                let edge = at_edge(projection.u, u0, u1) || at_edge(projection.v, v0, v1);
                println!(
                    "    {label}{index:<2} distance {:.2e}  (u {:.4}, v {:.4}){}",
                    projection.distance,
                    projection.u,
                    projection.v,
                    if edge { "  <- on the patch edge" } else { "" }
                );
            }
        }
    }
}

/// 生の辿りの端点を1回だけ出すための目印。
fn fit_first(marched: &zenith_geom::MarchedIntersection) -> bool {
    use std::sync::atomic::{AtomicBool, Ordering};
    static DONE: AtomicBool = AtomicBool::new(false);
    !marched.points.is_empty() && !DONE.swap(true, Ordering::Relaxed)
}
