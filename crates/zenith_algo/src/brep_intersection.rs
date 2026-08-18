use zenith_math::{BoundingBox3, Point3, Tolerance, Vec3, Vec3Ext};
use zenith_topo::{Face, FaceGeometry};

#[derive(Debug, Clone, PartialEq)]
pub enum FaceIntersectionKind {
    Line { point: Point3, direction: Vec3 },
    Coincident,
    Unsupported,
}

#[derive(Debug, Clone, PartialEq)]
pub struct FaceIntersectionCandidate {
    pub face_a_index: usize,
    pub face_b_index: usize,
    pub kind: FaceIntersectionKind,
}

pub struct BrepIntersectionBuilder;

impl BrepIntersectionBuilder {
    pub fn collect_face_pair_candidates(
        faces_a: &[Face],
        faces_b: &[Face],
        tol: &Tolerance,
    ) -> Vec<FaceIntersectionCandidate> {
        let mut candidates = Vec::new();
        let bboxes_a: Vec<Option<BoundingBox3>> = faces_a.iter().map(face_boundary_bbox).collect();
        let bboxes_b: Vec<Option<BoundingBox3>> = faces_b.iter().map(face_boundary_bbox).collect();

        for (face_a_index, face_a) in faces_a.iter().enumerate() {
            for (face_b_index, face_b) in faces_b.iter().enumerate() {
                if !face_bboxes_intersect(
                    bboxes_a[face_a_index].as_ref(),
                    bboxes_b[face_b_index].as_ref(),
                    tol,
                ) {
                    continue;
                }
                if let Some(kind) = intersect_face_supports(face_a, face_b, tol) {
                    candidates.push(FaceIntersectionCandidate {
                        face_a_index,
                        face_b_index,
                        kind,
                    });
                }
            }
        }

        candidates
    }
}

fn face_boundary_bbox(face: &Face) -> Option<BoundingBox3> {
    let mut bbox = BoundingBox3::empty();
    for point in face.outer_wire.sample_points(12) {
        if point3_is_finite(point) {
            bbox.extend_point(point);
        }
    }
    for wire in &face.inner_wires {
        for point in wire.sample_points(12) {
            if point3_is_finite(point) {
                bbox.extend_point(point);
            }
        }
    }

    bbox.is_valid().then_some(bbox)
}

fn face_bboxes_intersect(
    bbox_a: Option<&BoundingBox3>,
    bbox_b: Option<&BoundingBox3>,
    tol: &Tolerance,
) -> bool {
    match (bbox_a, bbox_b) {
        (Some(a), Some(b)) => a.intersects(b, tol.linear),
        _ => true,
    }
}

fn point3_is_finite(point: Point3) -> bool {
    point.x.is_finite() && point.y.is_finite() && point.z.is_finite()
}

fn intersect_face_supports(
    face_a: &Face,
    face_b: &Face,
    tol: &Tolerance,
) -> Option<FaceIntersectionKind> {
    match (&face_a.geometry, &face_b.geometry) {
        (FaceGeometry::Plane(plane_a), FaceGeometry::Plane(plane_b)) => Some(intersect_planes(
            plane_a.origin,
            oriented_plane_normal(face_a),
            plane_b.origin,
            oriented_plane_normal(face_b),
            tol,
        )),
        (FaceGeometry::Nurbs(_), FaceGeometry::Nurbs(_)) => Some(FaceIntersectionKind::Unsupported),
        (FaceGeometry::Plane(_), FaceGeometry::Nurbs(_))
        | (FaceGeometry::Nurbs(_), FaceGeometry::Plane(_)) => {
            Some(FaceIntersectionKind::Unsupported)
        }
        _ => None,
    }
}

fn oriented_plane_normal(face: &Face) -> Vec3 {
    let FaceGeometry::Plane(plane) = &face.geometry else {
        return Vec3::new(0.0, 0.0, 0.0);
    };

    if face.orientation.is_forward() {
        plane.normal
    } else {
        -plane.normal
    }
}

fn intersect_planes(
    origin_a: Point3,
    normal_a: Vec3,
    origin_b: Point3,
    normal_b: Vec3,
    tol: &Tolerance,
) -> FaceIntersectionKind {
    let Some(n1) = normal_a.try_normalize_safe(1e-12) else {
        return FaceIntersectionKind::Unsupported;
    };
    let Some(n2) = normal_b.try_normalize_safe(1e-12) else {
        return FaceIntersectionKind::Unsupported;
    };

    let direction = n1.cross(&n2);
    if direction.norm() <= tol.angular {
        let plane_offset = (origin_b - origin_a).dot(&n1).abs();
        if plane_offset <= tol.linear {
            return FaceIntersectionKind::Coincident;
        }
        return FaceIntersectionKind::Unsupported;
    }

    let direction_norm_sq = direction.norm_squared();
    let d1 = n1.dot(&origin_a.coords);
    let d2 = n2.dot(&origin_b.coords);
    let point_vec = (n2 * d1 - n1 * d2).cross(&direction) / direction_norm_sq;

    FaceIntersectionKind::Line {
        point: Point3::from(point_vec),
        direction: direction.normalize(),
    }
}
