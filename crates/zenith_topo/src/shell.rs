use crate::face::{Face, FaceGeometry};
use serde::{Deserialize, Serialize};
use std::sync::atomic::{AtomicU64, Ordering};
use zenith_math::{BoundingBox3, Point2, Point3, Tolerance};

static SHELL_ID_GEN: AtomicU64 = AtomicU64::new(1);

/// B-Rep シェル（Shell: 接続された面の集合）
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct Shell {
    pub id: u64,
    pub faces: Vec<Face>,
    pub is_closed: bool,
}

/// Shell の位相検証結果
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct ShellValidationReport {
    pub face_count: usize,
    pub edge_use_count: usize,
    pub open_wire_count: usize,
    pub unmatched_edge_use_count: usize,
    pub non_manifold_edge_use_count: usize,
    pub same_direction_edge_use_count: usize,
    pub duplicate_edge_use_count: usize,
    pub duplicate_face_count: usize,
    pub degenerate_face_count: usize,
    pub min_planar_face_area: f64,
    pub degenerate_edge_use_count: usize,
    pub min_edge_use_length: f64,
    pub non_finite_point_count: usize,
    pub planar_face_orientation_mismatch_count: usize,
    pub min_planar_face_oriented_area: f64,
    pub edge_curve_endpoint_mismatch_count: usize,
    pub max_edge_curve_endpoint_distance: f64,
    pub off_surface_boundary_count: usize,
    pub max_boundary_surface_distance: f64,
    pub pcurve_mismatch_count: usize,
    pub max_pcurve_distance: f64,
    /// 幾何的には対になっているのに、**別の稜の実体**を指している辺の使用数。
    ///
    /// 閉性は座標で判定されるので、同じ位置に別々の `Edge` が並んでいても
    /// 「閉じている」と出る。その立体には「この稜を共有する2面」が引けず、
    /// 稜を選ぶ演算（フィレット・面取り・履歴）が掛からない。これは**診断**
    /// であってゲートではない（`is_valid` には影響しない）。
    #[serde(default)]
    pub unshared_edge_entity_use_count: usize,
    pub errors: Vec<String>,
}

impl ShellValidationReport {
    pub fn is_valid(&self) -> bool {
        self.errors.is_empty()
    }
}

#[derive(Debug, Clone, Copy)]
struct EdgeUse {
    edge_id: u64,
    face_index: usize,
    wire_index: usize,
    edge_index: usize,
    start: Point3,
    end: Point3,
    /// A point from the middle of the curve, to tell apart two edges that run
    /// between the same pair of vertices. A torus written as one face has two
    /// such edges - the seam the long way round and the seam the short way -
    /// and both begin and end at the same point, so endpoints alone made every
    /// use on that face look like a mate of every other.
    middle: Point3,
}

/// How many places along each edge the p-curve is checked.
///
/// Deliberately coprime with the 8 the p-curves are built from: sampling at the
/// construction points measures nothing, because the curve passes through them
/// exactly by construction, and a curve that swept right round a sphere in
/// between still read as zero.
const PCURVE_VALIDATION_SAMPLES: usize = 37;

impl Shell {
    pub fn new(faces: Vec<Face>, is_closed: bool) -> Self {
        Self {
            id: SHELL_ID_GEN.fetch_add(1, Ordering::Relaxed),
            faces,
            is_closed,
        }
    }

    /// 開いたシェル（サーフェスモデル、シートボディ）
    pub fn open(faces: Vec<Face>) -> Self {
        Self::new(faces, false)
    }

    /// 閉じたシェル（ソリッドの境界）
    pub fn closed(faces: Vec<Face>) -> Self {
        Self::new(faces, true)
    }

    /// シェルを構成する全面の軸平行バウンディングボックス (AABB) を計算
    pub fn bounding_box(&self) -> BoundingBox3 {
        let mut bbox = BoundingBox3::empty();
        for face in &self.faces {
            bbox.extend_bbox(&face.bounding_box());
        }
        bbox
    }

    /// 閉シェルとして最低限の位相条件を検証する。
    pub fn validate_closed(&self, tol: &Tolerance) -> ShellValidationReport {
        let mut report = ShellValidationReport {
            face_count: self.faces.len(),
            edge_use_count: 0,
            open_wire_count: 0,
            unmatched_edge_use_count: 0,
            non_manifold_edge_use_count: 0,
            same_direction_edge_use_count: 0,
            duplicate_edge_use_count: 0,
            duplicate_face_count: 0,
            degenerate_face_count: 0,
            min_planar_face_area: f64::INFINITY,
            degenerate_edge_use_count: 0,
            min_edge_use_length: f64::INFINITY,
            non_finite_point_count: 0,
            planar_face_orientation_mismatch_count: 0,
            min_planar_face_oriented_area: f64::INFINITY,
            edge_curve_endpoint_mismatch_count: 0,
            max_edge_curve_endpoint_distance: 0.0,
            off_surface_boundary_count: 0,
            max_boundary_surface_distance: 0.0,
            pcurve_mismatch_count: 0,
            max_pcurve_distance: 0.0,
            unshared_edge_entity_use_count: 0,
            errors: Vec::new(),
        };

        if self.faces.is_empty() {
            report.errors.push("Shell has no faces".to_string());
            return report;
        }

        validate_duplicate_faces(&self.faces, &mut report, tol);

        let mut edge_uses = Vec::new();

        for (face_index, face) in self.faces.iter().enumerate() {
            validate_planar_face_orientation(face_index, face, &mut report, tol);

            let boundary_report = face.validate_boundary_on_surface(tol, 8);
            report.off_surface_boundary_count += boundary_report.off_surface_point_count;
            report.max_boundary_surface_distance = report
                .max_boundary_surface_distance
                .max(boundary_report.max_distance);
            for error in boundary_report.errors {
                report.errors.push(format!("Face {face_index}: {error}"));
            }

            if face.pcurves.is_some() {
                // 構成に使った数と互いに素な標本数にする。p-curve は辺を8等分
                // して作られるので、8で測ると自分が通ることの分かっている点しか
                // 見ない。37 なら共有するのは両端だけになる。
                match face.validate_pcurves(tol, PCURVE_VALIDATION_SAMPLES) {
                    Ok(pcurve_report) => {
                        report.pcurve_mismatch_count += pcurve_report.mismatch_count;
                        report.max_pcurve_distance =
                            report.max_pcurve_distance.max(pcurve_report.max_distance);
                        for error in pcurve_report.errors {
                            report.errors.push(format!("Face {face_index}: {error}"));
                        }
                    }
                    Err(err) => {
                        report.pcurve_mismatch_count += 1;
                        report.errors.push(format!(
                            "Face {face_index}: p-curve validation failed: {err}"
                        ));
                    }
                }
            }

            // 境界を持たない面は、閉じていないのではなく囲むものが無い。
            if !face.outer_wire.edges.is_empty() && !face.outer_wire.is_closed(tol) {
                report.open_wire_count += 1;
                report
                    .errors
                    .push(format!("Face {face_index} outer wire is open"));
            }
            collect_wire_edge_uses(
                face_index,
                0,
                &face.outer_wire.edges,
                &mut edge_uses,
                &mut report,
                tol,
            );

            for (inner_index, wire) in face.inner_wires.iter().enumerate() {
                if !wire.is_closed(tol) {
                    report.open_wire_count += 1;
                    report.errors.push(format!(
                        "Face {face_index} inner wire {inner_index} is open"
                    ));
                }
                collect_wire_edge_uses(
                    face_index,
                    inner_index + 1,
                    &wire.edges,
                    &mut edge_uses,
                    &mut report,
                    tol,
                );
            }
        }

        report.edge_use_count = edge_uses.len();
        if report.edge_use_count == 0 {
            report.min_edge_use_length = 0.0;
        }
        validate_duplicate_edge_uses(&edge_uses, &mut report, tol);

        for edge_use in &edge_uses {
            let mates: Vec<&EdgeUse> = edge_uses
                .iter()
                .filter(|candidate| {
                    !same_edge_use(edge_use, candidate)
                        && same_undirected_edge(edge_use, candidate, tol.linear)
                })
                .collect();
            let mate_count = mates.len();

            if mate_count == 0 {
                report.unmatched_edge_use_count += 1;
                report.errors.push(format!(
                    "Edge use f{}:w{}:e{} has no matching mate",
                    edge_use.face_index, edge_use.wire_index, edge_use.edge_index
                ));
            } else if mate_count > 1 {
                report.non_manifold_edge_use_count += 1;
                report.errors.push(format!(
                    "Edge use f{}:w{}:e{} has {mate_count} matching mates",
                    edge_use.face_index, edge_use.wire_index, edge_use.edge_index
                ));
            } else if mates[0].edge_id != edge_use.edge_id {
                // 座標では対になっているが、実体が別。閉じてはいるが、この稜
                // からもう一方の面を引くことはできない。診断として数だけ残す。
                report.unshared_edge_entity_use_count += 1;
            }

            if mate_count == 1 && !opposite_direction_edge(edge_use, mates[0], tol.linear) {
                report.same_direction_edge_use_count += 1;
                report.errors.push(format!(
                    "Edge use f{}:w{}:e{} and f{}:w{}:e{} share the same direction",
                    edge_use.face_index,
                    edge_use.wire_index,
                    edge_use.edge_index,
                    mates[0].face_index,
                    mates[0].wire_index,
                    mates[0].edge_index
                ));
            }
        }

        report
    }

    pub fn is_topologically_closed(&self, tol: &Tolerance) -> bool {
        self.validate_closed(tol).is_valid()
    }
}

fn collect_wire_edge_uses(
    face_index: usize,
    wire_index: usize,
    edges: &[crate::edge::OrientedEdge],
    edge_uses: &mut Vec<EdgeUse>,
    report: &mut ShellValidationReport,
    tol: &Tolerance,
) {
    for (edge_index, edge) in edges.iter().enumerate() {
        validate_finite_edge_use_points(face_index, wire_index, edge_index, edge, report);

        let edge_length = sampled_edge_length(edge, 8);
        if !edge_length.is_finite() {
            report.non_finite_point_count += 1;
            report.errors.push(format!(
                "Edge use f{face_index}:w{wire_index}:e{edge_index} has non-finite sampled length"
            ));
        } else if edge_length <= tol.linear {
            report.min_edge_use_length = report.min_edge_use_length.min(edge_length);
            report.degenerate_edge_use_count += 1;
            report.errors.push(format!(
                "Edge use f{face_index}:w{wire_index}:e{edge_index} is degenerate; sampled length {edge_length:.6e}"
            ));
        } else {
            report.min_edge_use_length = report.min_edge_use_length.min(edge_length);
        }

        let curve_start_distance =
            (edge.evaluate_normalized(0.0) - edge.start_vertex().point).norm();
        let curve_end_distance = (edge.evaluate_normalized(1.0) - edge.end_vertex().point).norm();
        let max_distance = curve_start_distance.max(curve_end_distance);
        report.max_edge_curve_endpoint_distance =
            report.max_edge_curve_endpoint_distance.max(max_distance);
        if max_distance > tol.linear {
            report.edge_curve_endpoint_mismatch_count += 1;
            report.errors.push(format!(
                "Edge use f{face_index}:w{wire_index}:e{edge_index} curve endpoints differ from vertices by {max_distance:.6e}"
            ));
        }

        edge_uses.push(EdgeUse {
            edge_id: edge.edge.id,
            face_index,
            wire_index,
            edge_index,
            start: edge.start_vertex().point,
            end: edge.end_vertex().point,
            middle: edge.evaluate_normalized(0.5),
        });
    }
}

fn validate_duplicate_faces(faces: &[Face], report: &mut ShellValidationReport, tol: &Tolerance) {
    let signatures: Vec<Option<Vec<QuantizedPoint3>>> = faces
        .iter()
        .map(|face| face_boundary_signature(face, tol.linear))
        .collect();

    for i in 0..signatures.len() {
        let Some(left) = &signatures[i] else {
            continue;
        };
        for (j, right) in signatures.iter().enumerate().skip(i + 1) {
            let Some(right) = right else {
                continue;
            };
            if left == right {
                report.duplicate_face_count += 1;
                report.errors.push(format!(
                    "Faces {i} and {j} have duplicate boundary signatures"
                ));
            }
        }
    }
}

fn validate_duplicate_edge_uses(
    edge_uses: &[EdgeUse],
    report: &mut ShellValidationReport,
    tol: &Tolerance,
) {
    for i in 0..edge_uses.len() {
        if points_same(edge_uses[i].start, edge_uses[i].end, tol.linear) {
            continue;
        }
        for j in i + 1..edge_uses.len() {
            if points_same(edge_uses[j].start, edge_uses[j].end, tol.linear) {
                continue;
            }
            if same_directed_edge(&edge_uses[i], &edge_uses[j], tol.linear) {
                report.duplicate_edge_use_count += 1;
                report.errors.push(format!(
                    "Edge uses f{}:w{}:e{} and f{}:w{}:e{} are duplicate directed uses",
                    edge_uses[i].face_index,
                    edge_uses[i].wire_index,
                    edge_uses[i].edge_index,
                    edge_uses[j].face_index,
                    edge_uses[j].wire_index,
                    edge_uses[j].edge_index
                ));
            }
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
struct QuantizedPoint3 {
    x: i64,
    y: i64,
    z: i64,
}

fn face_boundary_signature(face: &Face, tol: f64) -> Option<Vec<QuantizedPoint3>> {
    let mut points = Vec::new();
    append_wire_signature_points(&face.outer_wire, &mut points, tol)?;
    for wire in &face.inner_wires {
        append_wire_signature_points(wire, &mut points, tol)?;
    }

    points.sort_unstable();
    points.dedup();
    if points.is_empty() {
        None
    } else {
        Some(points)
    }
}

fn append_wire_signature_points(
    wire: &crate::wire::Wire,
    points: &mut Vec<QuantizedPoint3>,
    tol: f64,
) -> Option<()> {
    for edge in &wire.edges {
        points.push(quantized_point3(edge.start_vertex().point, tol)?);
        points.push(quantized_point3(edge.end_vertex().point, tol)?);
        // **端点だけでは面を見分けられません。** 同じ2点を結ぶ別々の弧で
        // 囲まれた2枚の面は、端点の集合が同じなので「重複した面」と報告され
        // ます。トーラスを傾けたスラブで切ると、管の底の継ぎ目の上で実際に
        // そうなります（中点は 3.839 離れており、別の曲線です)。
        //
        // `EdgeUse` は前から中点を持っています。署名だけが持っていません
        // でした（4-65）。
        points.push(quantized_point3(edge.evaluate_normalized(0.5), tol)?);
    }
    Some(())
}

fn quantized_point3(point: Point3, tol: f64) -> Option<QuantizedPoint3> {
    if !point3_is_finite(point) {
        return None;
    }

    let scale = tol.max(1e-12);
    Some(QuantizedPoint3 {
        x: (point.x / scale).round() as i64,
        y: (point.y / scale).round() as i64,
        z: (point.z / scale).round() as i64,
    })
}

fn validate_finite_edge_use_points(
    face_index: usize,
    wire_index: usize,
    edge_index: usize,
    edge: &crate::edge::OrientedEdge,
    report: &mut ShellValidationReport,
) {
    let checks = [
        ("start vertex", edge.start_vertex().point),
        ("end vertex", edge.end_vertex().point),
        ("curve start", edge.evaluate_normalized(0.0)),
        ("curve midpoint", edge.evaluate_normalized(0.5)),
        ("curve end", edge.evaluate_normalized(1.0)),
    ];

    for (label, point) in checks {
        if !point3_is_finite(point) {
            report.non_finite_point_count += 1;
            report.errors.push(format!(
                "Edge use f{face_index}:w{wire_index}:e{edge_index} has non-finite {label}"
            ));
        }
    }
}

fn point3_is_finite(point: Point3) -> bool {
    point.x.is_finite() && point.y.is_finite() && point.z.is_finite()
}

fn sampled_edge_length(edge: &crate::edge::OrientedEdge, segments: usize) -> f64 {
    let segments = segments.max(1);
    let mut length = 0.0;
    let mut prev = edge.evaluate_normalized(0.0);
    for i in 1..=segments {
        let t = i as f64 / segments as f64;
        let current = edge.evaluate_normalized(t);
        length += (current - prev).norm();
        prev = current;
    }
    length
}

fn validate_planar_face_orientation(
    face_index: usize,
    face: &Face,
    report: &mut ShellValidationReport,
    tol: &Tolerance,
) {
    // NURBS面も同じ規約に従う: 外側トリムループの回り方と face.orientation が
    // 一致していなければ、面の外向き法線が材料の反対を向いてしまう。
    // ただし縫い目だけで構成されるループ（球・トーラスの1面表現）は
    // UV 上で面積を囲まないので対象外。
    let seam_only_loop_allowed = match &face.geometry {
        FaceGeometry::Plane(_) => false,
        FaceGeometry::Nurbs(_) => true,
        _ => return,
    };

    // 縫い目だけのループは面積では見分けられない。縫い目上の点は UV で
    // 両端どちらにも写るので、投影がどちらを選ぶかで符号付き面積が揺れる。
    // 代わりに位相で見る: どの辺も同じループ内にもう一度現れるなら、
    // その面は曲面全体であって、ループは何も囲んでいない。
    if seam_only_loop_allowed && face.has_seam_only_boundary(tol.linear) {
        return;
    }

    let Ok(pcurves) = face.pcurves(tol) else {
        return;
    };
    let area = pcurve_loop_signed_area(&pcurves.outer_loop.segments, 8);
    if seam_only_loop_allowed && area.abs() <= tol.parametric {
        return;
    }
    if area.abs() <= tol.parametric {
        report.degenerate_face_count += 1;
        report.min_planar_face_area = report.min_planar_face_area.min(area.abs());
        report.errors.push(format!(
            "Face {face_index} planar p-curve outer loop is degenerate; area {:.6e}",
            area.abs()
        ));
        return;
    }
    report.min_planar_face_area = report.min_planar_face_area.min(area.abs());

    let oriented_area = if face.orientation.is_forward() {
        area
    } else {
        -area
    };
    report.min_planar_face_oriented_area = report.min_planar_face_oriented_area.min(oriented_area);
    if oriented_area <= tol.parametric {
        report.planar_face_orientation_mismatch_count += 1;
        // **どちらが裏返っているのかを名指しします**（4-280）。
        //
        // 符号だけでは、**ワイヤの巻きが逆**なのか**平面の法線が逆**なのかが
        // 分かりません。3D のワイヤから法線を起こして、平面の法線と突き合わせ
        // ます。同じ向きなら「巻きが逆」、逆向きなら「平面が逆」です。
        if std::env::var_os("ZENITH_ORIENT_WHY").is_some() {
            let points = face.outer_wire.sample_points(24);
            let mut normal = zenith_math::Vec3::zeros();
            for index in 0..points.len() {
                let a = points[index];
                let b = points[(index + 1) % points.len()];
                normal += a.coords.cross(&b.coords);
            }
            let plane_normal = match &face.geometry {
                FaceGeometry::Plane(plane) => {
                    // **uv の右ねじと、持っている法線が一致しているか**（4-281）。
                    // p-curve の符号付き面積は `u × v` まわりで決まるので、
                    // ここがずれていると符号だけが逆になります。
                    let handed = plane.u_axis.cross(&plane.v_axis);
                    let consistent = handed.normalize().dot(&plane.normal.normalize());
                    eprintln!(
                        "ORIENTWHY   平面の uv: u×v と法線の内積 {consistent:+.6}（{}）",
                        if consistent > 0.0 {
                            "一致"
                        } else {
                            "**逆。符号付き面積が裏返ります**"
                        }
                    );
                    plane.normal
                }
                _ => zenith_math::Vec3::z(),
            };
            let agreement = if normal.norm() > 0.0 {
                normal.normalize().dot(&plane_normal.normalize())
            } else {
                0.0
            };
            // **UV で確かめます**（4-283）。3D のベクトル面積は曲面の上では
            // 使えないので（4-282）、**p-curve の点を曲面へ写し、そこでの
            // 法線と、隣り合う3点の作る向き**を突き合わせます。
            //
            // 見るのは1点だけで足ります——「UV で反時計回り」と「その点で
            // 曲面の法線まわりに反時計回り」は、**パッチの UV が右手系なら
            // 同じこと**です。ずれていれば、そこが原因です。
            if let FaceGeometry::Nurbs(surface) = &face.geometry {
                if let Some(segment) = pcurves.outer_loop.segments.first() {
                    let (t0, t1) = segment.curve.param_range();
                    let a = segment.curve.evaluate(t0);
                    let b = segment.curve.evaluate(t0 + (t1 - t0) * 0.5);
                    let mid = zenith_math::Point2::new((a.x + b.x) * 0.5, (a.y + b.y) * 0.5);
                    if let Some(normal) = surface.normal(mid.x, mid.y) {
                        let du = surface.evaluate(mid.x + 1e-6, mid.y)
                            - surface.evaluate(mid.x - 1e-6, mid.y);
                        let dv = surface.evaluate(mid.x, mid.y + 1e-6)
                            - surface.evaluate(mid.x, mid.y - 1e-6);
                        let handed = du.cross(&dv);
                        eprintln!(
                            "ORIENTWHY   UV の手: ∂u×∂v と法線の内積 {:+.4}（{}）",
                            handed.normalize().dot(&normal.normalize()),
                            if handed.normalize().dot(&normal.normalize()) > 0.0 {
                                "右手系"
                            } else {
                                "**左手系。UV の符号がひっくり返ります**"
                            }
                        );
                    }
                }
            }
            eprintln!(
                "ORIENTWHY 面 {face_index}（{}）: p-curve は {}、符号付き面積 {oriented_area:.6e}、面の向き {}、ワイヤの法線との内積 {agreement:+.6}（{}）",
                match &face.geometry {
                    FaceGeometry::Plane(_) => "平面",
                    FaceGeometry::Nurbs(_) => "**NURBS**",
                    _ => "その他",
                },
                if face.pcurves.is_some() {
                    "**ファイル／取り込みが持たせたもの**"
                } else {
                    "その場で投影したもの"
                },
                if face.orientation.is_forward() { "正" } else { "逆" },
                if agreement > 0.0 {
                    "同じ向き＝**ワイヤの巻きが逆**"
                } else {
                    "逆向き＝**平面の法線が逆**"
                }
            );
        }
        report.errors.push(format!(
            // **「planar」と書いていました**（4-281）。この検査は NURBS の面も
            // 見ます。実測でここに出た3枚は**すべて NURBS** で、文面を平面だと
            // 読んで一度誤った記録を書きました（4-280 の訂正）。
            "Face {face_index} {} p-curve loop is inconsistent with face orientation; oriented area {oriented_area:.6e}",
            match &face.geometry {
                FaceGeometry::Plane(_) => "planar",
                FaceGeometry::Nurbs(_) => "NURBS",
                _ => "surface",
            }
        ));
    }
}

fn pcurve_loop_signed_area(
    segments: &[crate::face::FacePcurveSegment],
    samples_per_segment: usize,
) -> f64 {
    let mut points = Vec::new();
    for (segment_index, segment) in segments.iter().enumerate() {
        let segment_points = segment.curve.sample_points(samples_per_segment);
        let start_index = usize::from(segment_index > 0);
        for point in segment_points.into_iter().skip(start_index) {
            points.push(point);
        }
    }

    if points.len() > 1 && points_same_2d(points[0], *points.last().unwrap(), 1e-9) {
        points.pop();
    }

    signed_area_2d(&points)
}

fn signed_area_2d(points: &[Point2]) -> f64 {
    if points.len() < 3 {
        return 0.0;
    }

    let mut area = 0.0;
    for i in 0..points.len() {
        let current = points[i];
        let next = points[(i + 1) % points.len()];
        area += current.x * next.y - next.x * current.y;
    }

    area * 0.5
}

fn points_same_2d(a: Point2, b: Point2, tol: f64) -> bool {
    (a - b).norm() <= tol
}

fn same_edge_use(a: &EdgeUse, b: &EdgeUse) -> bool {
    a.face_index == b.face_index && a.wire_index == b.wire_index && a.edge_index == b.edge_index
}

fn same_undirected_edge(a: &EdgeUse, b: &EdgeUse, tol: f64) -> bool {
    points_same(a.middle, b.middle, tol)
        && (points_same(a.start, b.start, tol) && points_same(a.end, b.end, tol)
            || points_same(a.start, b.end, tol) && points_same(a.end, b.start, tol))
}

/// 2つの稜の使用が、**同じ稜を同じ向きに**使っているか。
///
/// **端点だけでは足りません。** 同じ2点を結ぶ別々の弧は普通にあります。
/// トーラスを傾けたスラブで切ると、管の底の継ぎ目の上で四半パッチが2枚
/// 出会い、同じ2点を結ぶ2本の弧が出ます（中点は 3.839 離れています）。
/// 端点だけで見ると、閉じた殻なのに「同じ稜を2度同じ向きに使っている」と
/// 報告されます（4-65）。
///
/// `EdgeUse` は前から `middle` を持っていました。**使っていなかっただけ**です。
fn same_directed_edge(a: &EdgeUse, b: &EdgeUse, tol: f64) -> bool {
    points_same(a.start, b.start, tol) && points_same(a.end, b.end, tol) && same_middle(a, b, tol)
}

fn opposite_direction_edge(a: &EdgeUse, b: &EdgeUse, tol: f64) -> bool {
    points_same(a.start, b.end, tol) && points_same(a.end, b.start, tol) && same_middle(a, b, tol)
}

/// 途中の点が同じか。公差は稜の長さに対する相対で取ります——分割の仕方が
/// 違う2枚が同じ稜を持つとき、中点は丸め誤差ぶん動きます。
fn same_middle(a: &EdgeUse, b: &EdgeUse, tol: f64) -> bool {
    let reach = (a.start - a.end).norm().max(1.0);
    points_same(a.middle, b.middle, tol.max(reach * 1e-6))
}

fn points_same(a: Point3, b: Point3, tol: f64) -> bool {
    (a - b).norm() <= tol
}
