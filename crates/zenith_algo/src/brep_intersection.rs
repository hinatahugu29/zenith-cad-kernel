use zenith_geom::{NurbsCurve3, PlaneSurface3};
use zenith_math::{BoundingBox3, Point2, Point3, Tolerance, Vec3, Vec3Ext};
use zenith_tess::{tessellate_solid, TessellationParams, TriangleMesh};
use zenith_topo::Solid;
use zenith_topo::{Edge, Face, FaceGeometry, FacePcurveLoop, OrientedEdge, Vertex, Wire};

#[derive(Debug, Clone, PartialEq)]
pub enum FaceIntersectionKind {
    Line {
        point: Point3,
        direction: Vec3,
        segment_start: Point3,
        segment_end: Point3,
    },
    Coincident,
    Unsupported,
}

#[derive(Debug, Clone, PartialEq)]
pub struct FaceIntersectionCandidate {
    pub face_a_index: usize,
    pub face_b_index: usize,
    pub kind: FaceIntersectionKind,
}

#[derive(Debug, Clone, PartialEq)]
pub struct IntersectionEdgeCandidate {
    pub face_a_index: usize,
    pub face_b_index: usize,
    pub edge: Edge,
}

#[derive(Debug, Clone, PartialEq)]
pub struct PlanarFaceSplitCandidate {
    pub face_a_index: usize,
    pub face_b_index: usize,
    pub split_edge: Edge,
    pub split_faces_a: Vec<Face>,
    pub split_faces_b: Vec<Face>,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum FaceRegionLocation {
    Inside,
    Outside,
    Boundary,
}

#[derive(Debug, Clone, PartialEq)]
pub struct ClassifiedFacePiece {
    pub face: Face,
    pub location: FaceRegionLocation,
}

#[derive(Debug, Clone, PartialEq)]
pub struct ClassifiedPlanarFaceSplitCandidate {
    pub face_a_index: usize,
    pub face_b_index: usize,
    pub split_edge: Edge,
    pub split_faces_a: Vec<ClassifiedFacePiece>,
    pub split_faces_b: Vec<ClassifiedFacePiece>,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum BooleanOperand {
    A,
    B,
}

#[derive(Debug, Clone, PartialEq)]
pub struct SelectedBooleanFacePiece {
    pub operand: BooleanOperand,
    pub face: Face,
    pub location: FaceRegionLocation,
    pub reverse_orientation: bool,
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
                if let Some(kind) = intersect_face_supports(face_a, face_b, tol)
                    .and_then(|kind| {
                        clip_candidate_to_face_bboxes(
                            kind,
                            bboxes_a[face_a_index].as_ref(),
                            bboxes_b[face_b_index].as_ref(),
                            tol,
                        )
                    })
                    .and_then(|kind| clip_candidate_to_planar_trims(kind, face_a, face_b, tol))
                {
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

    pub fn collect_intersection_edge_candidates(
        faces_a: &[Face],
        faces_b: &[Face],
        tol: &Tolerance,
    ) -> Vec<IntersectionEdgeCandidate> {
        Self::collect_face_pair_candidates(faces_a, faces_b, tol)
            .into_iter()
            .filter_map(|candidate| {
                let FaceIntersectionKind::Line {
                    segment_start,
                    segment_end,
                    ..
                } = candidate.kind
                else {
                    return None;
                };

                if (segment_end - segment_start).norm() <= tol.linear {
                    return None;
                }

                let curve =
                    NurbsCurve3::bspline_from_points(1, vec![segment_start, segment_end]).ok()?;
                let start_vertex = Vertex::new(segment_start, tol.linear);
                let end_vertex = Vertex::new(segment_end, tol.linear);
                let edge = Edge::new(curve, start_vertex, end_vertex, tol.linear);

                Some(IntersectionEdgeCandidate {
                    face_a_index: candidate.face_a_index,
                    face_b_index: candidate.face_b_index,
                    edge,
                })
            })
            .collect()
    }

    pub fn collect_planar_face_split_candidates(
        faces_a: &[Face],
        faces_b: &[Face],
        tol: &Tolerance,
    ) -> Vec<PlanarFaceSplitCandidate> {
        Self::collect_intersection_edge_candidates(faces_a, faces_b, tol)
            .into_iter()
            .filter_map(|candidate| {
                let split_faces_a = Self::split_planar_face_by_edge(
                    &faces_a[candidate.face_a_index],
                    &candidate.edge,
                    tol,
                )
                .ok()?;
                let split_faces_b = Self::split_planar_face_by_edge(
                    &faces_b[candidate.face_b_index],
                    &candidate.edge,
                    tol,
                )
                .ok()?;

                Some(PlanarFaceSplitCandidate {
                    face_a_index: candidate.face_a_index,
                    face_b_index: candidate.face_b_index,
                    split_edge: candidate.edge,
                    split_faces_a,
                    split_faces_b,
                })
            })
            .collect()
    }

    pub fn collect_classified_planar_face_split_candidates(
        solid_a: &Solid,
        solid_b: &Solid,
        tol: &Tolerance,
    ) -> Vec<ClassifiedPlanarFaceSplitCandidate> {
        let mesh_a = tessellate_solid(solid_a, &TessellationParams::default());
        let mesh_b = tessellate_solid(solid_b, &TessellationParams::default());

        Self::collect_planar_face_split_candidates(
            &solid_a.outer_shell.faces,
            &solid_b.outer_shell.faces,
            tol,
        )
        .into_iter()
        .map(|candidate| {
            let split_faces_a = candidate
                .split_faces_a
                .into_iter()
                .map(|face| ClassifiedFacePiece {
                    location: classify_face_against_mesh(&face, &mesh_b, tol),
                    face,
                })
                .collect();
            let split_faces_b = candidate
                .split_faces_b
                .into_iter()
                .map(|face| ClassifiedFacePiece {
                    location: classify_face_against_mesh(&face, &mesh_a, tol),
                    face,
                })
                .collect();

            ClassifiedPlanarFaceSplitCandidate {
                face_a_index: candidate.face_a_index,
                face_b_index: candidate.face_b_index,
                split_edge: candidate.split_edge,
                split_faces_a,
                split_faces_b,
            }
        })
        .collect()
    }

    pub fn classify_face_against_solid(
        face: &Face,
        solid: &Solid,
        tol: &Tolerance,
    ) -> FaceRegionLocation {
        let mesh = tessellate_solid(solid, &TessellationParams::default());
        classify_face_against_mesh(face, &mesh, tol)
    }

    pub fn select_boolean_face_pieces(
        candidate: &ClassifiedPlanarFaceSplitCandidate,
        op: crate::BooleanOpType,
    ) -> Vec<SelectedBooleanFacePiece> {
        let mut selected = Vec::new();

        for piece in &candidate.split_faces_a {
            if keep_piece(BooleanOperand::A, piece.location, op) {
                selected.push(SelectedBooleanFacePiece {
                    operand: BooleanOperand::A,
                    face: piece.face.clone(),
                    location: piece.location,
                    reverse_orientation: false,
                });
            }
        }

        for piece in &candidate.split_faces_b {
            if keep_piece(BooleanOperand::B, piece.location, op) {
                selected.push(SelectedBooleanFacePiece {
                    operand: BooleanOperand::B,
                    face: piece.face.clone(),
                    location: piece.location,
                    reverse_orientation: op == crate::BooleanOpType::Difference,
                });
            }
        }

        selected
    }

    pub fn split_planar_face_by_edge(
        face: &Face,
        split_edge: &Edge,
        tol: &Tolerance,
    ) -> Result<Vec<Face>, String> {
        let FaceGeometry::Plane(plane) = &face.geometry else {
            return Err("Only planar faces can be split by an intersection edge".to_string());
        };
        if !face.inner_wires.is_empty() {
            return Err(
                "Planar face splitting with inner wires is not implemented yet".to_string(),
            );
        }
        if split_edge.curve.degree != 1 || split_edge.curve.control_points.len() != 2 {
            return Err("Only linear split edges are supported".to_string());
        }

        let boundary = face.outer_wire.sample_points(1);
        if boundary.len() < 3 {
            return Err("Cannot split a face with fewer than three boundary points".to_string());
        }

        let start = split_edge.start_vertex.point;
        let end = split_edge.end_vertex.point;
        if (end - start).norm() <= tol.linear {
            return Err("Split edge is degenerate".to_string());
        }
        if !point_lies_on_plane(start, plane, tol) || !point_lies_on_plane(end, plane, tol) {
            return Err("Split edge endpoints must lie on the planar face".to_string());
        }

        let boundary_uv: Vec<Point2> = boundary
            .iter()
            .map(|point| project_to_plane_uv(*point, plane))
            .collect();
        let start_uv = project_to_plane_uv(start, plane);
        let end_uv = project_to_plane_uv(end, plane);
        let start_hit = boundary_hit(start, start_uv, &boundary_uv, tol)
            .ok_or_else(|| "Split edge start does not lie on the outer boundary".to_string())?;
        let end_hit = boundary_hit(end, end_uv, &boundary_uv, tol)
            .ok_or_else(|| "Split edge end does not lie on the outer boundary".to_string())?;
        if boundary_hits_same(&start_hit, &end_hit, tol) {
            return Err("Split edge endpoints collapse on the boundary".to_string());
        }
        let mid_uv = project_to_plane_uv(start + (end - start) * 0.5, plane);
        if !point_in_polygon_2d(mid_uv, &boundary_uv, tol.parametric)
            || point_on_polygon_boundary(mid_uv, &boundary_uv, tol.parametric)
        {
            return Err("Split edge must cross the face interior".to_string());
        }

        let loop_a = clean_loop_points(
            boundary_path_between(&boundary, &start_hit, &end_hit)
                .into_iter()
                .chain([start])
                .collect(),
            tol,
        );
        let loop_b = clean_loop_points(
            boundary_path_between(&boundary, &end_hit, &start_hit)
                .into_iter()
                .chain([end])
                .collect(),
            tol,
        );

        if loop_a.len() < 3 || loop_b.len() < 3 {
            return Err("Split edge did not produce two valid face loops".to_string());
        }

        let face_a = face_from_polygon(face, loop_a, tol)?;
        let face_b = face_from_polygon(face, loop_b, tol)?;
        Ok(vec![face_a, face_b])
    }
}

#[derive(Debug, Clone)]
struct BoundaryHit {
    segment_index: usize,
    point: Point3,
}

fn boundary_hit(
    point: Point3,
    uv: Point2,
    boundary_uv: &[Point2],
    tol: &Tolerance,
) -> Option<BoundaryHit> {
    let mut best = None;
    let mut best_distance = f64::INFINITY;
    for i in 0..boundary_uv.len() {
        let a = boundary_uv[i];
        let b = boundary_uv[(i + 1) % boundary_uv.len()];
        let ab = b - a;
        let len_sq = ab.norm_squared();
        if len_sq <= tol.parametric.max(1e-12) {
            continue;
        }
        let t = ((uv - a).dot(&ab) / len_sq).clamp(0.0, 1.0);
        let closest = a + ab * t;
        let distance = (uv - closest).norm();
        if distance <= tol.parametric.max(tol.linear) * 10.0 && distance < best_distance {
            best = Some(BoundaryHit {
                segment_index: i,
                point,
            });
            best_distance = distance;
        }
    }

    best
}

fn boundary_hits_same(a: &BoundaryHit, b: &BoundaryHit, tol: &Tolerance) -> bool {
    (a.point - b.point).norm() <= tol.linear
}

fn point_lies_on_plane(point: Point3, plane: &PlaneSurface3, tol: &Tolerance) -> bool {
    (point - plane.origin).dot(&plane.normal).abs() <= tol.linear * 10.0
}

fn boundary_path_between(boundary: &[Point3], from: &BoundaryHit, to: &BoundaryHit) -> Vec<Point3> {
    let mut path = vec![from.point];
    let n = boundary.len();
    let mut index = (from.segment_index + 1) % n;

    loop {
        path.push(boundary[index]);
        if index == to.segment_index {
            break;
        }
        index = (index + 1) % n;
    }
    path.push(to.point);
    path
}

fn clean_loop_points(points: Vec<Point3>, tol: &Tolerance) -> Vec<Point3> {
    let mut clean = Vec::new();
    for point in points {
        if clean
            .last()
            .map(|last: &Point3| (point - *last).norm() <= tol.linear)
            .unwrap_or(false)
        {
            continue;
        }
        clean.push(point);
    }

    if clean.len() > 1 && (clean[0] - *clean.last().unwrap()).norm() <= tol.linear {
        clean.pop();
    }

    clean
}

fn face_from_polygon(
    template: &Face,
    points: Vec<Point3>,
    tol: &Tolerance,
) -> Result<Face, String> {
    let mut oriented_edges = Vec::with_capacity(points.len());
    for i in 0..points.len() {
        let start = points[i];
        let end = points[(i + 1) % points.len()];
        if (end - start).norm() <= tol.linear {
            return Err("Split face loop contains a degenerate edge".to_string());
        }
        let curve =
            NurbsCurve3::bspline_from_points(1, vec![start, end]).map_err(|err| err.to_string())?;
        let start_vertex = Vertex::new(start, tol.linear);
        let end_vertex = Vertex::new(end, tol.linear);
        let edge = Edge::new(curve, start_vertex, end_vertex, tol.linear);
        oriented_edges.push(OrientedEdge::forward(edge));
    }

    Ok(Face::new(
        template.geometry.clone(),
        Wire::new(oriented_edges),
        Vec::new(),
        template.orientation,
        template.tolerance,
    ))
}

fn classify_face_against_mesh(
    face: &Face,
    mesh: &TriangleMesh,
    tol: &Tolerance,
) -> FaceRegionLocation {
    let sample = representative_face_point(face);
    if point_mesh_distance(sample, mesh) <= tol.linear * 100.0 {
        return FaceRegionLocation::Boundary;
    }
    if crate::BooleanEngine::is_point_inside_mesh(sample, mesh) {
        FaceRegionLocation::Inside
    } else {
        FaceRegionLocation::Outside
    }
}

fn keep_piece(
    operand: BooleanOperand,
    location: FaceRegionLocation,
    op: crate::BooleanOpType,
) -> bool {
    match (op, operand, location) {
        (
            crate::BooleanOpType::Union,
            _,
            FaceRegionLocation::Outside | FaceRegionLocation::Boundary,
        ) => true,
        (crate::BooleanOpType::Union, _, FaceRegionLocation::Inside) => false,
        (
            crate::BooleanOpType::Intersection,
            _,
            FaceRegionLocation::Inside | FaceRegionLocation::Boundary,
        ) => true,
        (crate::BooleanOpType::Intersection, _, FaceRegionLocation::Outside) => false,
        (
            crate::BooleanOpType::Difference,
            BooleanOperand::A,
            FaceRegionLocation::Outside | FaceRegionLocation::Boundary,
        ) => true,
        (crate::BooleanOpType::Difference, BooleanOperand::A, FaceRegionLocation::Inside) => false,
        (
            crate::BooleanOpType::Difference,
            BooleanOperand::B,
            FaceRegionLocation::Inside | FaceRegionLocation::Boundary,
        ) => true,
        (crate::BooleanOpType::Difference, BooleanOperand::B, FaceRegionLocation::Outside) => false,
    }
}

fn representative_face_point(face: &Face) -> Point3 {
    let points = face.outer_wire.sample_points(2);
    if points.is_empty() {
        return Point3::new(0.0, 0.0, 0.0);
    }

    let sum = points
        .iter()
        .fold(Vec3::new(0.0, 0.0, 0.0), |acc, point| acc + point.coords);
    Point3::from(sum / points.len() as f64)
}

fn point_mesh_distance(point: Point3, mesh: &TriangleMesh) -> f64 {
    mesh.indices
        .iter()
        .map(|tri| {
            let a = mesh.positions[tri[0] as usize];
            let b = mesh.positions[tri[1] as usize];
            let c = mesh.positions[tri[2] as usize];
            point_triangle_distance(point, a, b, c)
        })
        .fold(f64::INFINITY, f64::min)
}

fn point_triangle_distance(point: Point3, a: Point3, b: Point3, c: Point3) -> f64 {
    let ab = b - a;
    let ac = c - a;
    let ap = point - a;

    let d1 = ab.dot(&ap);
    let d2 = ac.dot(&ap);
    if d1 <= 0.0 && d2 <= 0.0 {
        return (point - a).norm();
    }

    let bp = point - b;
    let d3 = ab.dot(&bp);
    let d4 = ac.dot(&bp);
    if d3 >= 0.0 && d4 <= d3 {
        return (point - b).norm();
    }

    let vc = d1 * d4 - d3 * d2;
    if vc <= 0.0 && d1 >= 0.0 && d3 <= 0.0 {
        let v = d1 / (d1 - d3);
        return (point - (a + ab * v)).norm();
    }

    let cp = point - c;
    let d5 = ab.dot(&cp);
    let d6 = ac.dot(&cp);
    if d6 >= 0.0 && d5 <= d6 {
        return (point - c).norm();
    }

    let vb = d5 * d2 - d1 * d6;
    if vb <= 0.0 && d2 >= 0.0 && d6 <= 0.0 {
        let w = d2 / (d2 - d6);
        return (point - (a + ac * w)).norm();
    }

    let va = d3 * d6 - d5 * d4;
    if va <= 0.0 && (d4 - d3) >= 0.0 && (d5 - d6) >= 0.0 {
        let w = (d4 - d3) / ((d4 - d3) + (d5 - d6));
        return (point - (b + (c - b) * w)).norm();
    }

    let denom = 1.0 / (va + vb + vc);
    let v = vb * denom;
    let w = vc * denom;
    let closest = a + ab * v + ac * w;
    (point - closest).norm()
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

fn clip_candidate_to_face_bboxes(
    kind: FaceIntersectionKind,
    bbox_a: Option<&BoundingBox3>,
    bbox_b: Option<&BoundingBox3>,
    tol: &Tolerance,
) -> Option<FaceIntersectionKind> {
    match kind {
        FaceIntersectionKind::Line {
            point, direction, ..
        } => {
            let bbox_a = bbox_a?;
            let bbox_b = bbox_b?;
            let overlap = bbox_overlap(bbox_a, bbox_b, tol.linear)?;
            let (t_min, t_max) = clip_line_to_bbox(point, direction, &overlap, tol.linear)?;
            Some(FaceIntersectionKind::Line {
                point,
                direction,
                segment_start: point + direction * t_min,
                segment_end: point + direction * t_max,
            })
        }
        other => Some(other),
    }
}

fn bbox_overlap(a: &BoundingBox3, b: &BoundingBox3, tol: f64) -> Option<BoundingBox3> {
    let min = Point3::new(
        a.min.x.max(b.min.x) - tol,
        a.min.y.max(b.min.y) - tol,
        a.min.z.max(b.min.z) - tol,
    );
    let max = Point3::new(
        a.max.x.min(b.max.x) + tol,
        a.max.y.min(b.max.y) + tol,
        a.max.z.min(b.max.z) + tol,
    );
    (min.x <= max.x && min.y <= max.y && min.z <= max.z)
        .then_some(BoundingBox3::from_min_max(min, max))
}

fn clip_line_to_bbox(
    point: Point3,
    direction: Vec3,
    bbox: &BoundingBox3,
    tol: f64,
) -> Option<(f64, f64)> {
    let mut t_min = f64::NEG_INFINITY;
    let mut t_max = f64::INFINITY;

    for axis in 0..3 {
        let p = point[axis];
        let d = direction[axis];
        let min = bbox.min[axis];
        let max = bbox.max[axis];
        if d.abs() <= tol.max(1e-12) {
            if p < min || p > max {
                return None;
            }
            continue;
        }

        let t1 = (min - p) / d;
        let t2 = (max - p) / d;
        t_min = t_min.max(t1.min(t2));
        t_max = t_max.min(t1.max(t2));
        if t_min > t_max {
            return None;
        }
    }

    (t_min.is_finite() && t_max.is_finite() && t_min <= t_max).then_some((t_min, t_max))
}

fn clip_candidate_to_planar_trims(
    kind: FaceIntersectionKind,
    face_a: &Face,
    face_b: &Face,
    tol: &Tolerance,
) -> Option<FaceIntersectionKind> {
    match kind {
        FaceIntersectionKind::Line {
            point,
            direction,
            segment_start,
            segment_end,
        } => {
            let mut interval = (0.0, 1.0);
            interval = clip_segment_to_planar_face_trim(
                segment_start,
                segment_end,
                face_a,
                interval,
                tol,
            )?;
            interval = clip_segment_to_planar_face_trim(
                segment_start,
                segment_end,
                face_b,
                interval,
                tol,
            )?;
            if interval.1 - interval.0 <= tol.linear {
                return None;
            }

            let segment_vec = segment_end - segment_start;
            Some(FaceIntersectionKind::Line {
                point,
                direction,
                segment_start: segment_start + segment_vec * interval.0,
                segment_end: segment_start + segment_vec * interval.1,
            })
        }
        other => Some(other),
    }
}

fn clip_segment_to_planar_face_trim(
    segment_start: Point3,
    segment_end: Point3,
    face: &Face,
    current_interval: (f64, f64),
    tol: &Tolerance,
) -> Option<(f64, f64)> {
    let FaceGeometry::Plane(plane) = &face.geometry else {
        return Some(current_interval);
    };
    let Ok(pcurves) = face.pcurves(tol) else {
        return Some(current_interval);
    };
    let polygon = sample_pcurve_loop(&pcurves.outer_loop, 10);
    if polygon.len() < 3 {
        return Some(current_interval);
    }

    let uv_start = project_to_plane_uv(segment_start, plane);
    let uv_end = project_to_plane_uv(segment_end, plane);
    let intervals = segment_inside_polygon_intervals(uv_start, uv_end, &polygon, tol.parametric);
    intersect_interval_set(current_interval, &intervals, tol.parametric)
}

fn sample_pcurve_loop(loop_data: &FacePcurveLoop, samples_per_segment: usize) -> Vec<Point2> {
    let mut points = Vec::new();
    for (segment_index, segment) in loop_data.segments.iter().enumerate() {
        let segment_points = segment.curve.sample_points(samples_per_segment);
        let start_index = usize::from(segment_index > 0);
        points.extend(segment_points.into_iter().skip(start_index));
    }

    if points.len() > 1 && points_same_2d(points[0], *points.last().unwrap(), 1e-9) {
        points.pop();
    }

    points
}

fn segment_inside_polygon_intervals(
    start: Point2,
    end: Point2,
    polygon: &[Point2],
    tol: f64,
) -> Vec<(f64, f64)> {
    let dir = end - start;
    if dir.norm() <= tol.max(1e-12) {
        return Vec::new();
    }

    let mut cuts = vec![0.0, 1.0];
    for i in 0..polygon.len() {
        let a = polygon[i];
        let b = polygon[(i + 1) % polygon.len()];
        if let Some(t) = segment_segment_intersection_t(start, end, a, b, tol) {
            cuts.push(t.clamp(0.0, 1.0));
        }
    }
    cuts.sort_by(|a, b| a.total_cmp(b));
    cuts.dedup_by(|a, b| (*a - *b).abs() <= tol.max(1e-9));

    let mut intervals = Vec::new();
    for pair in cuts.windows(2) {
        let t0 = pair[0];
        let t1 = pair[1];
        if t1 - t0 <= tol.max(1e-9) {
            continue;
        }
        let mid_t = (t0 + t1) * 0.5;
        let mid = start + dir * mid_t;
        if point_in_polygon_2d(mid, polygon, tol) {
            intervals.push((t0, t1));
        }
    }

    for &t in &cuts {
        let point = start + dir * t;
        if point_on_polygon_boundary(point, polygon, tol) {
            intervals.push((t, t));
        }
    }

    merge_intervals(intervals, tol.max(1e-9))
}

fn intersect_interval_set(
    current: (f64, f64),
    intervals: &[(f64, f64)],
    tol: f64,
) -> Option<(f64, f64)> {
    let mut best = None;
    let mut best_len = 0.0;

    for interval in intervals {
        let start = current.0.max(interval.0);
        let end = current.1.min(interval.1);
        let len = end - start;
        if len > tol.max(1e-9) && len > best_len {
            best = Some((start, end));
            best_len = len;
        }
    }

    best
}

fn merge_intervals(mut intervals: Vec<(f64, f64)>, tol: f64) -> Vec<(f64, f64)> {
    if intervals.is_empty() {
        return intervals;
    }
    intervals.sort_by(|a, b| a.0.total_cmp(&b.0));
    let mut merged = Vec::new();
    let mut current = intervals[0];
    for interval in intervals.into_iter().skip(1) {
        if interval.0 <= current.1 + tol {
            current.1 = current.1.max(interval.1);
        } else {
            merged.push(current);
            current = interval;
        }
    }
    merged.push(current);
    merged
}

fn segment_segment_intersection_t(
    p0: Point2,
    p1: Point2,
    q0: Point2,
    q1: Point2,
    tol: f64,
) -> Option<f64> {
    let r = p1 - p0;
    let s = q1 - q0;
    let denom = cross2(r, s);
    if denom.abs() <= tol.max(1e-12) {
        return None;
    }

    let qp = q0 - p0;
    let t = cross2(qp, s) / denom;
    let u = cross2(qp, r) / denom;
    (t >= -tol && t <= 1.0 + tol && u >= -tol && u <= 1.0 + tol).then_some(t)
}

fn point_in_polygon_2d(point: Point2, polygon: &[Point2], tol: f64) -> bool {
    if point_on_polygon_boundary(point, polygon, tol) {
        return true;
    }

    let mut inside = false;
    for i in 0..polygon.len() {
        let a = polygon[i];
        let b = polygon[(i + 1) % polygon.len()];
        let crosses = (a.y > point.y) != (b.y > point.y);
        if crosses {
            let x = a.x + (point.y - a.y) * (b.x - a.x) / (b.y - a.y);
            if x > point.x {
                inside = !inside;
            }
        }
    }

    inside
}

fn point_on_polygon_boundary(point: Point2, polygon: &[Point2], tol: f64) -> bool {
    for i in 0..polygon.len() {
        if point_on_segment_2d(point, polygon[i], polygon[(i + 1) % polygon.len()], tol) {
            return true;
        }
    }
    false
}

fn point_on_segment_2d(point: Point2, a: Point2, b: Point2, tol: f64) -> bool {
    let ab = b - a;
    let ap = point - a;
    let len_sq = ab.norm_squared();
    if len_sq <= tol.max(1e-12) {
        return (point - a).norm() <= tol;
    }
    let t = ap.dot(&ab) / len_sq;
    if t < -tol || t > 1.0 + tol {
        return false;
    }
    let closest = a + ab * t.clamp(0.0, 1.0);
    (point - closest).norm() <= tol.max(1e-9)
}

fn project_to_plane_uv(point: Point3, plane: &PlaneSurface3) -> Point2 {
    let rel = point - plane.origin;
    let uu = plane.u_axis.dot(&plane.u_axis);
    let uv = plane.u_axis.dot(&plane.v_axis);
    let vv = plane.v_axis.dot(&plane.v_axis);
    let ru = rel.dot(&plane.u_axis);
    let rv = rel.dot(&plane.v_axis);
    let det = uu * vv - uv * uv;

    if det.abs() <= 1e-15 {
        return Point2::new(0.0, 0.0);
    }

    Point2::new((ru * vv - rv * uv) / det, (rv * uu - ru * uv) / det)
}

fn cross2(a: nalgebra::Vector2<f64>, b: nalgebra::Vector2<f64>) -> f64 {
    a.x * b.y - a.y * b.x
}

fn points_same_2d(a: Point2, b: Point2, tol: f64) -> bool {
    (a - b).norm() <= tol
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
        segment_start: Point3::from(point_vec),
        segment_end: Point3::from(point_vec),
    }
}
