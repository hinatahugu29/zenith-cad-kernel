//! Correctness gate for exact B-Rep boolean results.
//!
//! Closed-manifoldness is necessary for a boolean result but not sufficient:
//! handing back one operand untouched is perfectly manifold and still wrong.
//! This module adds the two independent checks that catch that class of silent
//! failure - the volume bounds implied by the operation, and point membership
//! agreement between the result and the boolean predicate applied to the
//! operands.

use crate::boolean::BooleanOpType;
use crate::mass_properties::MassCalculator;
use zenith_math::{Point3, RobustPredicates, Tolerance, Vec3};
use zenith_tess::{tessellate_solid, TessellationParams, TriangleMesh};
use zenith_topo::Solid;

/// How thoroughly a boolean result is checked before it is handed back.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct BooleanVerificationParams {
    /// Tessellation used for the membership meshes and the volume integrals.
    pub tessellation: TessellationParams,
    /// Number of membership sample points drawn from the combined bounding box.
    pub sample_count: usize,
    /// Relative slack on the volume bounds, scaled by the larger operand volume.
    pub volume_relative_tolerance: f64,
    /// Fraction of unambiguous samples allowed to disagree before the result is
    /// rejected. Kept small; it absorbs isolated ray-casting glitches only.
    pub membership_mismatch_fraction: f64,
}

impl Default for BooleanVerificationParams {
    fn default() -> Self {
        Self {
            tessellation: TessellationParams {
                u_divisions: 12,
                v_divisions: 12,
            },
            sample_count: 384,
            volume_relative_tolerance: 1e-3,
            membership_mismatch_fraction: 0.01,
        }
    }
}

/// Outcome of verifying one boolean result.
#[derive(Debug, Clone, PartialEq)]
pub struct BooleanResultReport {
    pub op: BooleanOpType,
    pub volume_a: f64,
    pub volume_b: f64,
    pub volume_result: f64,
    pub result_solid_count: usize,
    pub invalid_shell_count: usize,
    pub sample_count: usize,
    pub classified_sample_count: usize,
    pub membership_mismatch_count: usize,
    pub errors: Vec<String>,
}

impl BooleanResultReport {
    pub fn is_valid(&self) -> bool {
        self.errors.is_empty()
    }

    /// Single-line summary suitable for an error message.
    pub fn summary(&self) -> String {
        format!(
            "volume A={:.6}, B={:.6}, result={:.6} over {} solid(s); {} invalid shell(s); membership {} of {} classified samples disagreed ({} drawn): {}",
            self.volume_a,
            self.volume_b,
            self.volume_result,
            self.result_solid_count,
            self.invalid_shell_count,
            self.membership_mismatch_count,
            self.classified_sample_count,
            self.sample_count,
            self.errors.join("; ")
        )
    }
}

pub struct BooleanResultVerifier;

impl BooleanResultVerifier {
    pub fn verify(
        solid_a: &Solid,
        solid_b: &Solid,
        result: &[Solid],
        op: BooleanOpType,
        tol: &Tolerance,
    ) -> BooleanResultReport {
        Self::verify_with_params(
            solid_a,
            solid_b,
            result,
            op,
            tol,
            &BooleanVerificationParams::default(),
        )
    }

    pub fn verify_with_params(
        solid_a: &Solid,
        solid_b: &Solid,
        result: &[Solid],
        op: BooleanOpType,
        tol: &Tolerance,
        params: &BooleanVerificationParams,
    ) -> BooleanResultReport {
        let mut report = BooleanResultReport {
            op,
            volume_a: 0.0,
            volume_b: 0.0,
            volume_result: 0.0,
            result_solid_count: result.len(),
            invalid_shell_count: 0,
            sample_count: 0,
            classified_sample_count: 0,
            membership_mismatch_count: 0,
            errors: Vec::new(),
        };

        if result.is_empty() {
            report
                .errors
                .push("Boolean result has no solids".to_string());
            return report;
        }

        // 1. Topology: every result shell, cavities included, must still be a
        //    valid closed shell.
        for (index, solid) in result.iter().enumerate() {
            let shells = std::iter::once(&solid.outer_shell).chain(solid.inner_shells.iter());
            for (shell_index, shell) in shells.enumerate() {
                let shell_report = shell.validate_closed(tol);
                if shell_report.is_valid() {
                    continue;
                }
                report.invalid_shell_count += 1;
                let first = shell_report
                    .errors
                    .first()
                    .cloned()
                    .unwrap_or_else(|| "unknown shell error".to_string());
                let label = if shell_index == 0 {
                    "outer shell".to_string()
                } else {
                    format!("cavity shell {}", shell_index - 1)
                };
                report
                    .errors
                    .push(format!("result solid {index} has an invalid {label}: {first}"));
            }
        }

        // 2. Volume bounds implied by the operation.
        report.volume_a = MassCalculator::compute_from_brep(solid_a, &params.tessellation).volume;
        report.volume_b = MassCalculator::compute_from_brep(solid_b, &params.tessellation).volume;
        report.volume_result = result
            .iter()
            .map(|s| MassCalculator::compute_from_brep(s, &params.tessellation).volume)
            .sum();

        let va = report.volume_a;
        let vb = report.volume_b;
        let vr = report.volume_result;
        let eps = params.volume_relative_tolerance * va.abs().max(vb.abs()).max(1.0);

        if vr <= eps {
            report
                .errors
                .push(format!("result volume {vr:.6} is not positive"));
        }

        match op {
            BooleanOpType::Union => {
                if vr < va.max(vb) - eps {
                    report.errors.push(format!(
                        "union volume {vr:.6} is smaller than the larger operand {:.6}",
                        va.max(vb)
                    ));
                }
                if vr > va + vb + eps {
                    report.errors.push(format!(
                        "union volume {vr:.6} exceeds the operand sum {:.6}",
                        va + vb
                    ));
                }
            }
            BooleanOpType::Intersection => {
                if vr > va.min(vb) + eps {
                    report.errors.push(format!(
                        "intersection volume {vr:.6} exceeds the smaller operand {:.6}",
                        va.min(vb)
                    ));
                }
            }
            BooleanOpType::Difference => {
                if vr > va + eps {
                    report
                        .errors
                        .push(format!("difference volume {vr:.6} exceeds operand A {va:.6}"));
                }
                if vr < va - vb - eps {
                    report.errors.push(format!(
                        "difference volume {vr:.6} is below the lower bound {:.6}",
                        va - vb
                    ));
                }
            }
        }

        // 3. Point membership: the result must agree with the boolean predicate
        //    applied to the operands, which is what catches an operand being
        //    passed straight through as the answer.
        let mesh_a = tessellate_solid(solid_a, &params.tessellation);
        let mesh_b = tessellate_solid(solid_b, &params.tessellation);
        let mut mesh_r = TriangleMesh::new();
        for solid in result {
            mesh_r.merge(&tessellate_solid(solid, &params.tessellation));
        }

        let bbox_a = mesh_bbox(&mesh_a);
        let bbox_b = mesh_bbox(&mesh_b);
        let bbox_r = mesh_bbox(&mesh_r);

        let Some((min_pt, max_pt)) = combined_bbox(&[bbox_a, bbox_b, bbox_r]) else {
            return report;
        };

        let span = Vec3::new(
            max_pt.x - min_pt.x,
            max_pt.y - min_pt.y,
            max_pt.z - min_pt.z,
        );

        for index in 0..params.sample_count {
            let point = Point3::new(
                min_pt.x + span.x * halton(index + 1, 2),
                min_pt.y + span.y * halton(index + 1, 3),
                min_pt.z + span.z * halton(index + 1, 5),
            );
            report.sample_count += 1;

            let Some(in_a) = classify_point(point, &mesh_a, bbox_a) else {
                continue;
            };
            let Some(in_b) = classify_point(point, &mesh_b, bbox_b) else {
                continue;
            };
            let Some(in_r) = classify_point(point, &mesh_r, bbox_r) else {
                continue;
            };

            let expected = match op {
                BooleanOpType::Union => in_a || in_b,
                BooleanOpType::Intersection => in_a && in_b,
                BooleanOpType::Difference => in_a && !in_b,
            };

            report.classified_sample_count += 1;
            if expected != in_r {
                report.membership_mismatch_count += 1;
            }
        }

        if report.classified_sample_count > 0 {
            let allowed = (report.classified_sample_count as f64
                * params.membership_mismatch_fraction)
                .ceil() as usize;
            if report.membership_mismatch_count > allowed {
                report.errors.push(format!(
                    "{} of {} classified sample points disagree with the {:?} predicate (allowed {allowed})",
                    report.membership_mismatch_count, report.classified_sample_count, op
                ));
            }
        }

        let _ = tol;
        report
    }
}

/// Inside test with a three-ray consensus. Returns `None` when the rays
/// disagree, which is how near-surface and edge-grazing samples get dropped
/// instead of being counted as mismatches.
fn classify_point(
    point: Point3,
    mesh: &TriangleMesh,
    bbox: Option<(Point3, Point3)>,
) -> Option<bool> {
    let Some((min_pt, max_pt)) = bbox else {
        return Some(false);
    };

    let margin = 1e-9;
    if point.x < min_pt.x - margin
        || point.y < min_pt.y - margin
        || point.z < min_pt.z - margin
        || point.x > max_pt.x + margin
        || point.y > max_pt.y + margin
        || point.z > max_pt.z + margin
    {
        return Some(false);
    }

    let rays = [
        Vec3::new(1.0, 0.000137, 0.000289),
        Vec3::new(0.000191, 1.0, -0.000233),
        Vec3::new(-0.000271, 0.000163, 1.0),
    ];

    let mut first: Option<bool> = None;
    for direction in rays {
        let inside = ray_parity_inside(point, direction.normalize(), mesh);
        match first {
            None => first = Some(inside),
            Some(previous) if previous != inside => return None,
            Some(_) => {}
        }
    }

    first
}

fn ray_parity_inside(point: Point3, direction: Vec3, mesh: &TriangleMesh) -> bool {
    let mut hits = 0usize;
    for tri in &mesh.indices {
        let a = mesh.positions[tri[0] as usize];
        let b = mesh.positions[tri[1] as usize];
        let c = mesh.positions[tri[2] as usize];
        if RobustPredicates::ray_triangle_intersect(point, direction, a, b, c).is_some() {
            hits += 1;
        }
    }
    hits % 2 == 1
}

fn mesh_bbox(mesh: &TriangleMesh) -> Option<(Point3, Point3)> {
    if mesh.positions.is_empty() {
        return None;
    }
    let mut min_pt = Point3::new(f64::INFINITY, f64::INFINITY, f64::INFINITY);
    let mut max_pt = Point3::new(f64::NEG_INFINITY, f64::NEG_INFINITY, f64::NEG_INFINITY);
    for p in &mesh.positions {
        min_pt.x = min_pt.x.min(p.x);
        min_pt.y = min_pt.y.min(p.y);
        min_pt.z = min_pt.z.min(p.z);
        max_pt.x = max_pt.x.max(p.x);
        max_pt.y = max_pt.y.max(p.y);
        max_pt.z = max_pt.z.max(p.z);
    }
    Some((min_pt, max_pt))
}

fn combined_bbox(boxes: &[Option<(Point3, Point3)>]) -> Option<(Point3, Point3)> {
    let mut result: Option<(Point3, Point3)> = None;
    for entry in boxes.iter().flatten() {
        let (min_pt, max_pt) = *entry;
        result = Some(match result {
            None => (min_pt, max_pt),
            Some((cur_min, cur_max)) => (
                Point3::new(
                    cur_min.x.min(min_pt.x),
                    cur_min.y.min(min_pt.y),
                    cur_min.z.min(min_pt.z),
                ),
                Point3::new(
                    cur_max.x.max(max_pt.x),
                    cur_max.y.max(max_pt.y),
                    cur_max.z.max(max_pt.z),
                ),
            ),
        });
    }
    result
}

/// Halton sequence, so the samples spread evenly without a random source and
/// stay identical between runs.
fn halton(mut index: usize, base: usize) -> f64 {
    let mut result = 0.0;
    let mut fraction = 1.0 / base as f64;
    while index > 0 {
        result += (index % base) as f64 * fraction;
        index /= base;
        fraction /= base as f64;
    }
    result
}
