use crate::face::{Face, FaceGeometry};
use serde::{Deserialize, Serialize};
use std::sync::atomic::{AtomicU64, Ordering};
use zenith_math::{Point2, Point3, Tolerance};

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
    pub errors: Vec<String>,
}

impl ShellValidationReport {
    pub fn is_valid(&self) -> bool {
        self.errors.is_empty()
    }
}

#[derive(Debug, Clone, Copy)]
struct EdgeUse {
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
                match face.validate_pcurves(tol, 8) {
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

            if !face.outer_wire.is_closed(tol) {
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
            } else if !opposite_direction_edge(edge_use, mates[0], tol.linear) {
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
        report.errors.push(format!(
            "Face {face_index} planar p-curve loop is inconsistent with face orientation; oriented area {oriented_area:.6e}"
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

fn same_directed_edge(a: &EdgeUse, b: &EdgeUse, tol: f64) -> bool {
    points_same(a.start, b.start, tol) && points_same(a.end, b.end, tol)
}

fn opposite_direction_edge(a: &EdgeUse, b: &EdgeUse, tol: f64) -> bool {
    points_same(a.start, b.end, tol) && points_same(a.end, b.start, tol)
}

fn points_same(a: Point3, b: Point3, tol: f64) -> bool {
    (a - b).norm() <= tol
}
