use crate::face::Face;
use serde::{Deserialize, Serialize};
use std::sync::atomic::{AtomicU64, Ordering};
use zenith_math::{Point3, Tolerance};

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

        let mut edge_uses = Vec::new();

        for (face_index, face) in self.faces.iter().enumerate() {
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
            collect_wire_edge_uses(face_index, 0, &face.outer_wire.edges, &mut edge_uses);

            for (inner_index, wire) in face.inner_wires.iter().enumerate() {
                if !wire.is_closed(tol) {
                    report.open_wire_count += 1;
                    report.errors.push(format!(
                        "Face {face_index} inner wire {inner_index} is open"
                    ));
                }
                collect_wire_edge_uses(face_index, inner_index + 1, &wire.edges, &mut edge_uses);
            }
        }

        report.edge_use_count = edge_uses.len();

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
) {
    for (edge_index, edge) in edges.iter().enumerate() {
        edge_uses.push(EdgeUse {
            face_index,
            wire_index,
            edge_index,
            start: edge.start_vertex().point,
            end: edge.end_vertex().point,
        });
    }
}

fn same_edge_use(a: &EdgeUse, b: &EdgeUse) -> bool {
    a.face_index == b.face_index && a.wire_index == b.wire_index && a.edge_index == b.edge_index
}

fn same_undirected_edge(a: &EdgeUse, b: &EdgeUse, tol: f64) -> bool {
    points_same(a.start, b.start, tol) && points_same(a.end, b.end, tol)
        || points_same(a.start, b.end, tol) && points_same(a.end, b.start, tol)
}

fn opposite_direction_edge(a: &EdgeUse, b: &EdgeUse, tol: f64) -> bool {
    points_same(a.start, b.end, tol) && points_same(a.end, b.start, tol)
}

fn points_same(a: Point3, b: Point3, tol: f64) -> bool {
    (a - b).norm() <= tol
}
