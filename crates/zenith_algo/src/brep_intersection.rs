use crate::cap::CapBuilder;
use std::collections::BTreeMap;
use zenith_geom::{ControlPoint3, KnotVector, NurbsCurve3, NurbsSurface3, PlaneSurface3};
use zenith_math::{BoundingBox3, Point2, Point3, Tolerance, Vec2, Vec3, Vec3Ext};
use zenith_tess::{tessellate_solid, TessellationParams, TriangleMesh};
use zenith_topo::{
    Edge, Face, FaceGeometry, FacePcurveLoop, Orientation, OrientedEdge, Vertex, Wire,
};
use zenith_topo::{Shell, Solid};

#[derive(Debug, Clone, PartialEq)]
pub enum FaceIntersectionKind {
    Line {
        point: Point3,
        direction: Vec3,
        segment_start: Point3,
        segment_end: Point3,
    },
    Curve {
        edge: Edge,
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

#[derive(Debug, Clone, PartialEq)]
pub struct PlanarFaceMultiSplitResult {
    pub faces: Vec<Face>,
    pub applied_split_count: usize,
    pub skipped_split_count: usize,
}

#[derive(Debug, Clone, PartialEq)]
pub struct PlanarFaceBatchSplit {
    pub face_index: usize,
    pub split_edge_count: usize,
    pub result: PlanarFaceMultiSplitResult,
}

#[derive(Debug, Clone, PartialEq)]
pub struct PlanarOperandBatchSplits {
    pub splits_a: Vec<PlanarFaceBatchSplit>,
    pub splits_b: Vec<PlanarFaceBatchSplit>,
}

#[derive(Debug, Clone, PartialEq)]
pub struct IntersectionEdgeLoop {
    pub edges: Vec<Edge>,
}

#[derive(Debug, Clone, PartialEq)]
pub struct IntersectionEdgeLoopExtraction {
    pub loops: Vec<IntersectionEdgeLoop>,
    pub skipped_edge_count: usize,
}

#[derive(Debug, Clone, PartialEq)]
pub struct PlanarCapGeneration {
    pub edge_loop_extraction: IntersectionEdgeLoopExtraction,
    pub cap_faces: Vec<Face>,
    pub failed_loop_count: usize,
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

#[derive(Debug, Clone, PartialEq)]
pub struct BooleanFaceSelection {
    pub batch_splits: PlanarOperandBatchSplits,
    pub selected_face_pieces: Vec<SelectedBooleanFacePiece>,
    pub stitch_report: SelectedFaceStitchReport,
}

#[derive(Debug, Clone, PartialEq)]
pub struct BooleanFaceAssembly {
    pub selected_face_pieces: Vec<SelectedBooleanFacePiece>,
    pub cap_face_count: usize,
    pub stitch_report: SelectedFaceStitchReport,
}

#[derive(Debug, Clone, PartialEq)]
pub struct BooleanShellAssembly {
    pub selection: BooleanFaceSelection,
    pub cap_generation: PlanarCapGeneration,
    pub assembly: BooleanFaceAssembly,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct SelectedFaceStitchReport {
    pub face_piece_count: usize,
    pub edge_use_count: usize,
    pub matched_edge_pair_count: usize,
    pub unmatched_edge_use_count: usize,
    pub non_manifold_edge_use_count: usize,
    pub same_direction_edge_use_count: usize,
}

impl SelectedFaceStitchReport {
    pub fn is_closed_manifold(&self) -> bool {
        self.face_piece_count > 0
            && self.edge_use_count > 0
            && self.unmatched_edge_use_count == 0
            && self.non_manifold_edge_use_count == 0
            && self.same_direction_edge_use_count == 0
            && self.edge_use_count == self.matched_edge_pair_count * 2
    }
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
                let edge = match candidate.kind {
                    FaceIntersectionKind::Line {
                        segment_start,
                        segment_end,
                        ..
                    } => {
                        if (segment_end - segment_start).norm() <= tol.linear {
                            return None;
                        }

                        let curve =
                            NurbsCurve3::bspline_from_points(1, vec![segment_start, segment_end])
                                .ok()?;
                        let start_vertex = Vertex::new(segment_start, tol.linear);
                        let end_vertex = Vertex::new(segment_end, tol.linear);
                        Edge::new(curve, start_vertex, end_vertex, tol.linear)
                    }
                    FaceIntersectionKind::Curve { edge } => edge,
                    _ => return None,
                };

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
                let split_faces_a = Self::split_face_by_edge(
                    &faces_a[candidate.face_a_index],
                    &candidate.edge,
                    tol,
                )
                .ok()?;
                let split_faces_b = Self::split_face_by_edge(
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

    pub fn collect_planar_face_batch_splits(
        faces_a: &[Face],
        faces_b: &[Face],
        tol: &Tolerance,
    ) -> PlanarOperandBatchSplits {
        let edge_candidates = Self::collect_intersection_edge_candidates(faces_a, faces_b, tol);
        let mut edges_by_face_a: BTreeMap<usize, Vec<Edge>> = BTreeMap::new();
        let mut edges_by_face_b: BTreeMap<usize, Vec<Edge>> = BTreeMap::new();

        for candidate in edge_candidates {
            edges_by_face_a
                .entry(candidate.face_a_index)
                .or_default()
                .push(candidate.edge.clone());
            edges_by_face_b
                .entry(candidate.face_b_index)
                .or_default()
                .push(candidate.edge);
        }

        PlanarOperandBatchSplits {
            splits_a: collect_batch_splits_for_faces(faces_a, edges_by_face_a, tol),
            splits_b: collect_batch_splits_for_faces(faces_b, edges_by_face_b, tol),
        }
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

    pub fn collect_selected_boolean_face_pieces(
        solid_a: &Solid,
        solid_b: &Solid,
        op: crate::BooleanOpType,
        tol: &Tolerance,
    ) -> BooleanFaceSelection {
        let batch_splits = Self::collect_planar_face_batch_splits(
            &solid_a.outer_shell.faces,
            &solid_b.outer_shell.faces,
            tol,
        );
        let mesh_a = tessellate_solid(solid_a, &TessellationParams::default());
        let mesh_b = tessellate_solid(solid_b, &TessellationParams::default());

        let mut selected_face_pieces = Vec::new();
        selected_face_pieces.extend(select_operand_faces_after_batch_split(
            &solid_a.outer_shell.faces,
            &batch_splits.splits_a,
            BooleanOperand::A,
            &mesh_b,
            op,
            tol,
        ));
        selected_face_pieces.extend(select_operand_faces_after_batch_split(
            &solid_b.outer_shell.faces,
            &batch_splits.splits_b,
            BooleanOperand::B,
            &mesh_a,
            op,
            tol,
        ));
        let stitch_report = diagnose_selected_face_stitching(&selected_face_pieces, tol);

        BooleanFaceSelection {
            batch_splits,
            selected_face_pieces,
            stitch_report,
        }
    }

    pub fn assemble_selected_face_pieces_with_caps(
        pieces: &[SelectedBooleanFacePiece],
        cap_faces: &[Face],
        tol: &Tolerance,
    ) -> BooleanFaceAssembly {
        let mut selected_face_pieces = pieces.to_vec();
        for face in cap_faces {
            let forward_piece = SelectedBooleanFacePiece {
                operand: BooleanOperand::A,
                face: face.clone(),
                location: FaceRegionLocation::Boundary,
                reverse_orientation: false,
            };
            let reversed_piece = SelectedBooleanFacePiece {
                reverse_orientation: true,
                ..forward_piece.clone()
            };

            let mut forward_pieces = selected_face_pieces.clone();
            forward_pieces.push(forward_piece.clone());
            let forward_score =
                stitch_report_score(&diagnose_selected_face_stitching(&forward_pieces, tol));

            let mut reversed_pieces = selected_face_pieces.clone();
            reversed_pieces.push(reversed_piece.clone());
            let reversed_score =
                stitch_report_score(&diagnose_selected_face_stitching(&reversed_pieces, tol));

            if reversed_score < forward_score {
                selected_face_pieces.push(reversed_piece);
            } else {
                selected_face_pieces.push(forward_piece);
            }
        }
        let stitch_report = diagnose_selected_face_stitching(&selected_face_pieces, tol);

        BooleanFaceAssembly {
            selected_face_pieces,
            cap_face_count: cap_faces.len(),
            stitch_report,
        }
    }

    pub fn collect_boolean_shell_assembly(
        solid_a: &Solid,
        solid_b: &Solid,
        op: crate::BooleanOpType,
        tol: &Tolerance,
    ) -> BooleanShellAssembly {
        let selection = Self::collect_selected_boolean_face_pieces(solid_a, solid_b, op, tol);
        let edge_candidates = Self::collect_intersection_edge_candidates(
            &solid_a.outer_shell.faces,
            &solid_b.outer_shell.faces,
            tol,
        );
        let cap_generation =
            Self::build_planar_caps_from_intersection_edge_candidates(&edge_candidates, tol);
        let assembly = Self::assemble_selected_face_pieces_with_caps(
            &selection.selected_face_pieces,
            &cap_generation.cap_faces,
            tol,
        );

        BooleanShellAssembly {
            selection,
            cap_generation,
            assembly,
        }
    }

    pub fn diagnose_selected_face_stitching(
        pieces: &[SelectedBooleanFacePiece],
        tol: &Tolerance,
    ) -> SelectedFaceStitchReport {
        diagnose_selected_face_stitching(pieces, tol)
    }

    pub fn build_solid_from_selected_face_pieces(
        pieces: &[SelectedBooleanFacePiece],
        tol: &Tolerance,
    ) -> Result<Solid, String> {
        let stitch_report = diagnose_selected_face_stitching(pieces, tol);
        if !stitch_report.is_closed_manifold() {
            return Err(format!(
                "Selected face pieces are not stitchable: {} unmatched edge uses, {} non-manifold edge uses, {} same-direction edge uses",
                stitch_report.unmatched_edge_use_count,
                stitch_report.non_manifold_edge_use_count,
                stitch_report.same_direction_edge_use_count
            ));
        }

        let faces = pieces
            .iter()
            .map(|piece| {
                if piece.reverse_orientation {
                    reverse_face_orientation(&piece.face)
                } else {
                    piece.face.clone()
                }
            })
            .collect();
        Solid::try_simple(Shell::closed(faces), tol).map_err(|err| err.to_string())
    }

    pub fn build_planar_cap_from_edge_loop(
        edges: &[Edge],
        tol: &Tolerance,
    ) -> Result<Face, String> {
        let wire = order_edges_into_closed_wire(edges, tol)?;
        let face = CapBuilder::make_planar_cap(wire)?;
        let pcurve_report = face.validate_pcurves(tol, 4)?;
        if !pcurve_report.is_valid() {
            return Err(format!(
                "Generated cap p-curves are invalid with {} mismatches",
                pcurve_report.mismatch_count
            ));
        }
        Ok(face)
    }

    pub fn collect_closed_intersection_edge_loops(
        edges: &[Edge],
        tol: &Tolerance,
    ) -> IntersectionEdgeLoopExtraction {
        collect_closed_intersection_edge_loops(edges, tol)
    }

    pub fn build_planar_caps_from_intersection_edges(
        edges: &[Edge],
        tol: &Tolerance,
    ) -> PlanarCapGeneration {
        let edge_loop_extraction = collect_closed_intersection_edge_loops(edges, tol);
        let mut cap_faces = Vec::new();
        let mut failed_loop_count = 0;

        for edge_loop in &edge_loop_extraction.loops {
            match Self::build_planar_cap_from_edge_loop(&edge_loop.edges, tol) {
                Ok(face) => cap_faces.push(face),
                Err(_) => failed_loop_count += 1,
            }
        }

        PlanarCapGeneration {
            edge_loop_extraction,
            cap_faces,
            failed_loop_count,
        }
    }

    pub fn build_planar_caps_from_intersection_edge_candidates(
        candidates: &[IntersectionEdgeCandidate],
        tol: &Tolerance,
    ) -> PlanarCapGeneration {
        let edges: Vec<Edge> = candidates
            .iter()
            .map(|candidate| candidate.edge.clone())
            .collect();
        Self::build_planar_caps_from_intersection_edges(&edges, tol)
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
        let boundary = face.outer_wire.sample_points(1);
        if boundary.len() < 3 {
            return Err("Cannot split a face with fewer than three boundary points".to_string());
        }

        let start = split_edge.start_vertex.point;
        let end = split_edge.end_vertex.point;
        if (end - start).norm() <= tol.linear {
            return Err("Split edge is degenerate".to_string());
        }
        if !edge_lies_on_plane(split_edge, plane, tol) {
            return Err("Split edge must lie on the planar face".to_string());
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
        let mid_uv = project_to_plane_uv(edge_midpoint(split_edge), plane);
        if !point_in_polygon_2d(mid_uv, &boundary_uv, tol.parametric)
            || point_on_polygon_boundary(mid_uv, &boundary_uv, tol.parametric)
        {
            return Err("Split edge must cross the face interior".to_string());
        }

        let loop_a = clean_loop_points(boundary_path_between(&boundary, &start_hit, &end_hit), tol);
        let loop_b = clean_loop_points(boundary_path_between(&boundary, &end_hit, &start_hit), tol);

        if loop_a.len() < 2 || loop_b.len() < 2 {
            return Err("Split edge did not produce two valid face loops".to_string());
        }

        let face_a = face_from_boundary_path_and_split_edge(
            face,
            loop_a,
            split_edge,
            Orientation::Reversed,
            tol,
        )?;
        let face_b = face_from_boundary_path_and_split_edge(
            face,
            loop_b,
            split_edge,
            Orientation::Forward,
            tol,
        )?;
        Ok(vec![face_a, face_b])
    }

    pub fn split_face_by_edge(
        face: &Face,
        split_edge: &Edge,
        tol: &Tolerance,
    ) -> Result<Vec<Face>, String> {
        match &face.geometry {
            FaceGeometry::Plane(_) => Self::split_planar_face_by_edge(face, split_edge, tol),
            FaceGeometry::Nurbs(surface) => split_cylinder_side_face_by_horizontal_edge(
                face, surface, split_edge, tol,
            )
            .or_else(|horizontal_error| {
                split_cylinder_side_face_by_vertical_edge(face, surface, split_edge, tol)
                    .map_err(|vertical_error| format!("{horizontal_error}; {vertical_error}"))
            }),
            _ => Err("Face splitting is not implemented for this geometry".to_string()),
        }
    }

    pub fn split_planar_face_by_edges(
        face: &Face,
        split_edges: &[Edge],
        tol: &Tolerance,
    ) -> Result<PlanarFaceMultiSplitResult, String> {
        let FaceGeometry::Plane(_) = &face.geometry else {
            return Err("Only planar faces can be split by intersection edges".to_string());
        };
        if !face.inner_wires.is_empty() {
            return Err(
                "Planar face splitting with inner wires is not implemented yet".to_string(),
            );
        }

        let mut faces = vec![face.clone()];
        let mut applied_split_count = 0;
        let mut skipped_split_count = 0;

        for split_edge in split_edges {
            let mut next_faces = Vec::new();
            let mut applied_this_edge = false;

            for current_face in faces {
                match Self::split_planar_face_by_edge(&current_face, split_edge, tol) {
                    Ok(split_faces) => {
                        applied_split_count += 1;
                        applied_this_edge = true;
                        next_faces.extend(split_faces);
                    }
                    Err(_) => next_faces.push(current_face),
                }
            }

            if !applied_this_edge {
                skipped_split_count += 1;
            }
            faces = next_faces;
        }

        Ok(PlanarFaceMultiSplitResult {
            faces,
            applied_split_count,
            skipped_split_count,
        })
    }

    pub fn split_face_by_edges(
        face: &Face,
        split_edges: &[Edge],
        tol: &Tolerance,
    ) -> Result<PlanarFaceMultiSplitResult, String> {
        let mut faces = vec![face.clone()];
        let mut applied_split_count = 0;
        let mut skipped_split_count = 0;

        for split_edge in split_edges {
            let mut next_faces = Vec::new();
            let mut applied_this_edge = false;

            for current_face in faces {
                match Self::split_face_by_edge(&current_face, split_edge, tol) {
                    Ok(split_faces) => {
                        applied_split_count += 1;
                        applied_this_edge = true;
                        next_faces.extend(split_faces);
                    }
                    Err(_) => next_faces.push(current_face),
                }
            }

            if !applied_this_edge {
                skipped_split_count += 1;
            }
            faces = next_faces;
        }

        Ok(PlanarFaceMultiSplitResult {
            faces,
            applied_split_count,
            skipped_split_count,
        })
    }
}

#[derive(Debug, Clone)]
struct BoundaryHit {
    segment_index: usize,
    point: Point3,
}

#[derive(Debug, Clone, Copy)]
struct StitchEdgeUse {
    start: Point3,
    end: Point3,
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

fn collect_batch_splits_for_faces(
    faces: &[Face],
    edges_by_face: BTreeMap<usize, Vec<Edge>>,
    tol: &Tolerance,
) -> Vec<PlanarFaceBatchSplit> {
    edges_by_face
        .into_iter()
        .filter_map(|(face_index, split_edges)| {
            let face = faces.get(face_index)?;
            let result =
                BrepIntersectionBuilder::split_face_by_edges(face, &split_edges, tol).ok()?;
            (result.applied_split_count > 0).then_some(PlanarFaceBatchSplit {
                face_index,
                split_edge_count: split_edges.len(),
                result,
            })
        })
        .collect()
}

fn order_edges_into_closed_wire(edges: &[Edge], tol: &Tolerance) -> Result<Wire, String> {
    if edges.len() < 3 {
        return Err("A cap loop needs at least three edges".to_string());
    }

    let mut remaining: Vec<Edge> = edges.to_vec();
    let first = remaining.remove(0);
    let loop_start = first.start_vertex.point;
    let mut current_end = first.end_vertex.point;
    let mut oriented_edges = vec![OrientedEdge::forward(first)];

    while !remaining.is_empty() {
        let next_index = remaining.iter().position(|edge| {
            points_same_3d(edge.start_vertex.point, current_end, tol.linear)
                || points_same_3d(edge.end_vertex.point, current_end, tol.linear)
        });
        let Some(next_index) = next_index else {
            return Err("Cap edges do not form a continuous loop".to_string());
        };

        let edge = remaining.remove(next_index);
        if points_same_3d(edge.start_vertex.point, current_end, tol.linear) {
            current_end = edge.end_vertex.point;
            oriented_edges.push(OrientedEdge::forward(edge));
        } else {
            current_end = edge.start_vertex.point;
            oriented_edges.push(OrientedEdge::reversed(edge));
        }
    }

    if !points_same_3d(current_end, loop_start, tol.linear) {
        return Err("Cap edges do not close".to_string());
    }

    let wire = Wire::new(oriented_edges);
    if !wire.is_closed(tol) {
        return Err("Ordered cap wire is not closed".to_string());
    }

    Ok(wire)
}

fn collect_closed_intersection_edge_loops(
    edges: &[Edge],
    tol: &Tolerance,
) -> IntersectionEdgeLoopExtraction {
    let mut remaining: Vec<Edge> = edges.to_vec();
    let mut loops = Vec::new();
    let mut skipped_edge_count = 0;

    while !remaining.is_empty() {
        let first = remaining.remove(0);
        let loop_start = first.start_vertex.point;
        let mut current_end = first.end_vertex.point;
        let mut loop_edges = vec![first];

        while !points_same_3d(current_end, loop_start, tol.linear) {
            let next_index = remaining.iter().position(|edge| {
                points_same_3d(edge.start_vertex.point, current_end, tol.linear)
                    || points_same_3d(edge.end_vertex.point, current_end, tol.linear)
            });
            let Some(next_index) = next_index else {
                break;
            };

            let edge = remaining.remove(next_index);
            current_end = if points_same_3d(edge.start_vertex.point, current_end, tol.linear) {
                edge.end_vertex.point
            } else {
                edge.start_vertex.point
            };
            loop_edges.push(edge);
        }

        if loop_edges.len() >= 3 && points_same_3d(current_end, loop_start, tol.linear) {
            loops.push(IntersectionEdgeLoop { edges: loop_edges });
        } else {
            skipped_edge_count += loop_edges.len();
        }
    }

    IntersectionEdgeLoopExtraction {
        loops,
        skipped_edge_count,
    }
}

fn select_operand_faces_after_batch_split(
    faces: &[Face],
    batch_splits: &[PlanarFaceBatchSplit],
    operand: BooleanOperand,
    other_mesh: &TriangleMesh,
    op: crate::BooleanOpType,
    tol: &Tolerance,
) -> Vec<SelectedBooleanFacePiece> {
    let split_faces_by_index: BTreeMap<usize, Vec<Face>> = batch_splits
        .iter()
        .map(|batch| (batch.face_index, batch.result.faces.clone()))
        .collect();
    let mut selected = Vec::new();

    for (face_index, original_face) in faces.iter().enumerate() {
        let face_pieces = split_faces_by_index
            .get(&face_index)
            .map(|faces| faces.as_slice())
            .unwrap_or_else(|| std::slice::from_ref(original_face));

        for face in face_pieces {
            let location = classify_face_against_mesh(face, other_mesh, tol);
            if keep_piece(operand, location, op) {
                selected.push(SelectedBooleanFacePiece {
                    operand,
                    face: face.clone(),
                    location,
                    reverse_orientation: operand == BooleanOperand::B
                        && op == crate::BooleanOpType::Difference,
                });
            }
        }
    }

    selected
}

fn boundary_hits_same(a: &BoundaryHit, b: &BoundaryHit, tol: &Tolerance) -> bool {
    (a.point - b.point).norm() <= tol.linear
}

fn point_lies_on_plane(point: Point3, plane: &PlaneSurface3, tol: &Tolerance) -> bool {
    (point - plane.origin).dot(&plane.normal).abs() <= tol.linear * 10.0
}

fn edge_lies_on_plane(edge: &Edge, plane: &PlaneSurface3, tol: &Tolerance) -> bool {
    let (t_min, t_max) = edge.curve.param_range();
    (0..=8).all(|i| {
        let t = t_min + (t_max - t_min) * (i as f64 / 8.0);
        point_lies_on_plane(edge.curve.evaluate(t), plane, tol)
    })
}

fn edge_midpoint(edge: &Edge) -> Point3 {
    let (t_min, t_max) = edge.curve.param_range();
    edge.curve.evaluate((t_min + t_max) * 0.5)
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

fn face_from_boundary_path_and_split_edge(
    template: &Face,
    boundary_path: Vec<Point3>,
    split_edge: &Edge,
    split_orientation: Orientation,
    tol: &Tolerance,
) -> Result<Face, String> {
    if boundary_path.len() < 2 {
        return Err("Split face loop contains too few boundary points".to_string());
    }

    let mut oriented_edges = Vec::with_capacity(boundary_path.len());
    for window in boundary_path.windows(2) {
        let start = window[0];
        let end = window[1];
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

    let expected_start = *boundary_path.last().unwrap();
    let expected_end = boundary_path[0];
    let oriented_split = OrientedEdge::new(split_edge.clone(), split_orientation);
    if (oriented_split.start_vertex().point - expected_start).norm() > tol.linear * 10.0
        || (oriented_split.end_vertex().point - expected_end).norm() > tol.linear * 10.0
    {
        return Err("Split edge orientation does not close the split loop".to_string());
    }
    oriented_edges.push(oriented_split);

    Ok(Face::new(
        template.geometry.clone(),
        Wire::new(oriented_edges),
        Vec::new(),
        template.orientation,
        template.tolerance,
    ))
}

fn split_cylinder_side_face_by_horizontal_edge(
    face: &Face,
    surface: &NurbsSurface3,
    split_edge: &Edge,
    tol: &Tolerance,
) -> Result<Vec<Face>, String> {
    if !face.inner_wires.is_empty() {
        return Err("NURBS face splitting with inner wires is not implemented yet".to_string());
    }
    let Some((z_min, z_max)) = cylinder_patch_z_span(surface, tol) else {
        return Err("Only recognized cylinder-side NURBS patches can be split".to_string());
    };
    if !edge_lies_on_recognized_cylinder_patch(split_edge, surface, tol) {
        return Err("Split edge must lie on the NURBS face".to_string());
    }

    let ((u_min, u_max), (v_min, v_max)) = surface.param_range();
    let split_z = split_edge.start_vertex.point.z;
    if (split_edge.end_vertex.point.z - split_z).abs() > tol.linear {
        return Err("Cylinder-side split edge must be horizontal".to_string());
    }
    if split_z <= z_min + tol.linear || split_z >= z_max - tol.linear {
        return Err("Cylinder-side split edge must cross the face interior".to_string());
    }

    let split_v = v_min + (v_max - v_min) * ((split_z - z_min) / (z_max - z_min));
    let bottom_start = surface.evaluate(u_min, v_min);
    let bottom_end = surface.evaluate(u_max, v_min);
    let split_start = surface.evaluate(u_min, split_v);
    let split_end = surface.evaluate(u_max, split_v);
    let top_start = surface.evaluate(u_min, v_max);
    let top_end = surface.evaluate(u_max, v_max);

    let split_forward = orient_edge_for_points(split_edge, split_start, split_end, tol)
        .ok_or_else(|| "Split edge endpoints do not match cylinder side boundaries".to_string())?;
    let split_reversed = OrientedEdge::new(
        split_forward.edge.clone(),
        split_forward.orientation.reversed(),
    );
    let bottom_edge = horizontal_section_edge(surface, z_min, z_min, z_max, tol)?;
    let top_edge = horizontal_section_edge(surface, z_max, z_min, z_max, tol)?;
    let left_lower = Edge::line_between(
        Vertex::new(bottom_start, tol.linear),
        Vertex::new(split_start, tol.linear),
    )?;
    let right_lower = Edge::line_between(
        Vertex::new(bottom_end, tol.linear),
        Vertex::new(split_end, tol.linear),
    )?;
    let left_upper = Edge::line_between(
        Vertex::new(split_start, tol.linear),
        Vertex::new(top_start, tol.linear),
    )?;
    let right_upper = Edge::line_between(
        Vertex::new(split_end, tol.linear),
        Vertex::new(top_end, tol.linear),
    )?;

    let lower = Face::new(
        face.geometry.clone(),
        Wire::new(vec![
            OrientedEdge::forward(bottom_edge),
            OrientedEdge::forward(right_lower),
            split_reversed,
            OrientedEdge::reversed(left_lower),
        ]),
        Vec::new(),
        face.orientation,
        face.tolerance,
    );
    let upper = Face::new(
        face.geometry.clone(),
        Wire::new(vec![
            split_forward,
            OrientedEdge::forward(right_upper),
            OrientedEdge::reversed(top_edge),
            OrientedEdge::reversed(left_upper),
        ]),
        Vec::new(),
        face.orientation,
        face.tolerance,
    );

    for split_face in [&lower, &upper] {
        if !split_face.outer_wire.is_closed(tol) {
            return Err("Cylinder-side split produced an open wire".to_string());
        }
        let report = split_face.validate_pcurves(tol, 8)?;
        if !report.is_valid() {
            return Err(format!(
                "Cylinder-side split p-curves are invalid with {} mismatches",
                report.mismatch_count
            ));
        }
    }

    Ok(vec![lower, upper])
}

/// Splits a recognized cylinder-side patch along a vertical ruling edge.
///
/// The ruling corresponds to a `u = const` iso-line, so both halves reuse the
/// original NURBS support surface and are trimmed by narrowed boundary wires
/// whose horizontal arcs are exact rational Bezier sub-arcs.
fn split_cylinder_side_face_by_vertical_edge(
    face: &Face,
    surface: &NurbsSurface3,
    split_edge: &Edge,
    tol: &Tolerance,
) -> Result<Vec<Face>, String> {
    if !face.inner_wires.is_empty() {
        return Err("NURBS face splitting with inner wires is not implemented yet".to_string());
    }
    let Some((center, _radius, z_min, z_max)) = cylinder_patch_axis_circle(surface, tol) else {
        return Err("Only recognized cylinder-side NURBS patches can be split".to_string());
    };
    if !edge_lies_on_recognized_cylinder_patch(split_edge, surface, tol) {
        return Err("Split edge must lie on the NURBS face".to_string());
    }

    let edge_start = split_edge.start_vertex.point;
    let edge_end = split_edge.end_vertex.point;
    if (Point2::new(edge_start.x, edge_start.y) - Point2::new(edge_end.x, edge_end.y)).norm()
        > tol.linear * 10.0
    {
        return Err("Cylinder-side split edge must be a vertical ruling".to_string());
    }
    let edge_z_low = edge_start.z.min(edge_end.z);
    let edge_z_high = edge_start.z.max(edge_end.z);
    if (edge_z_low - z_min).abs() > tol.linear * 10.0
        || (edge_z_high - z_max).abs() > tol.linear * 10.0
    {
        return Err("Cylinder-side ruling split must span the full patch height".to_string());
    }

    let ((u_min, u_max), (v_min, v_max)) = surface.param_range();
    let split_xy = Point2::new(edge_start.x, edge_start.y);
    let u_split = cylinder_patch_u_for_point(surface, split_xy, center, v_min, tol)
        .ok_or_else(|| "Split ruling does not lie on the patch angular span".to_string())?;
    if u_split <= u_min + tol.parametric || u_split >= u_max - tol.parametric {
        return Err("Cylinder-side ruling split must cross the face interior".to_string());
    }

    let bottom_curve = horizontal_section_curve(surface, z_min, z_min, z_max, tol)
        .ok_or_else(|| "Failed to build cylinder bottom section curve".to_string())?;
    let top_curve = horizontal_section_curve(surface, z_max, z_min, z_max, tol)
        .ok_or_else(|| "Failed to build cylinder top section curve".to_string())?;
    let (bottom_left_curve, bottom_right_curve) = bottom_curve
        .split_bezier_at(u_split)
        .ok_or_else(|| "Cylinder bottom arc is not a single splittable Bezier span".to_string())?;
    let (top_left_curve, top_right_curve) = top_curve
        .split_bezier_at(u_split)
        .ok_or_else(|| "Cylinder top arc is not a single splittable Bezier span".to_string())?;

    let bottom_start = surface.evaluate(u_min, v_min);
    let bottom_split = surface.evaluate(u_split, v_min);
    let bottom_end = surface.evaluate(u_max, v_min);
    let top_start = surface.evaluate(u_min, v_max);
    let top_split = surface.evaluate(u_split, v_max);
    let top_end = surface.evaluate(u_max, v_max);

    // The incoming candidate ruling may carry broad-phase clipping slack on its
    // endpoints, so the shared edge is rebuilt exactly from the patch iso-line
    // after confirming the candidate matches it.
    if orient_edge_for_points(split_edge, bottom_split, top_split, tol).is_none() {
        return Err("Split ruling endpoints do not match the patch height".to_string());
    }
    let split_ruling = Edge::line_between(
        Vertex::new(bottom_split, tol.linear),
        Vertex::new(top_split, tol.linear),
    )?;
    let split_forward = OrientedEdge::forward(split_ruling.clone());
    let split_reversed = OrientedEdge::reversed(split_ruling);

    let bottom_left = curve_edge(bottom_left_curve, bottom_start, bottom_split, tol);
    let bottom_right = curve_edge(bottom_right_curve, bottom_split, bottom_end, tol);
    let top_left = curve_edge(top_left_curve, top_start, top_split, tol);
    let top_right = curve_edge(top_right_curve, top_split, top_end, tol);
    let left_ruling = Edge::line_between(
        Vertex::new(bottom_start, tol.linear),
        Vertex::new(top_start, tol.linear),
    )?;
    let right_ruling = Edge::line_between(
        Vertex::new(bottom_end, tol.linear),
        Vertex::new(top_end, tol.linear),
    )?;

    let left = Face::new(
        face.geometry.clone(),
        Wire::new(vec![
            OrientedEdge::forward(bottom_left),
            split_forward,
            OrientedEdge::reversed(top_left),
            OrientedEdge::reversed(left_ruling),
        ]),
        Vec::new(),
        face.orientation,
        face.tolerance,
    );
    let right = Face::new(
        face.geometry.clone(),
        Wire::new(vec![
            OrientedEdge::forward(bottom_right),
            OrientedEdge::forward(right_ruling),
            OrientedEdge::reversed(top_right),
            split_reversed,
        ]),
        Vec::new(),
        face.orientation,
        face.tolerance,
    );

    for split_face in [&left, &right] {
        if !split_face.outer_wire.is_closed(tol) {
            return Err("Cylinder-side ruling split produced an open wire".to_string());
        }
        let report = split_face.validate_pcurves(tol, 8)?;
        if !report.is_valid() {
            return Err(format!(
                "Cylinder-side ruling split p-curves are invalid with {} mismatches",
                report.mismatch_count
            ));
        }
    }

    Ok(vec![left, right])
}

fn curve_edge(curve: NurbsCurve3, start: Point3, end: Point3, tol: &Tolerance) -> Edge {
    Edge::new(
        curve,
        Vertex::new(start, tol.linear),
        Vertex::new(end, tol.linear),
        tol.linear,
    )
}

/// Finds the `u` parameter whose patch point matches `target` in XY.
///
/// The angle around the axis is monotonic in `u` over a sub-half-turn rational
/// arc, so a bisection on the swept-angle ratio converges without needing a
/// closed form for the rational quadratic parameterization.
fn cylinder_patch_u_for_point(
    surface: &NurbsSurface3,
    target: Point2,
    center: Point2,
    v: f64,
    tol: &Tolerance,
) -> Option<f64> {
    let ((u_min, u_max), _) = surface.param_range();
    let angle_at = |u: f64| {
        let point = surface.evaluate(u, v);
        angle_at_center(Point2::new(point.x, point.y), center)
    };

    let start_angle = angle_at(u_min);
    let sweep = wrap_signed_angle(angle_at(u_max) - start_angle);
    if sweep.abs() <= tol.angular {
        return None;
    }

    let target_ratio = wrap_signed_angle(angle_at_center(target, center) - start_angle) / sweep;
    let margin = tol.angular / sweep.abs();
    if !(-margin..=1.0 + margin).contains(&target_ratio) {
        return None;
    }

    let mut low = u_min;
    let mut high = u_max;
    for _ in 0..100 {
        let mid = 0.5 * (low + high);
        let ratio = wrap_signed_angle(angle_at(mid) - start_angle) / sweep;
        if ratio < target_ratio {
            low = mid;
        } else {
            high = mid;
        }
    }

    let u = 0.5 * (low + high);
    let point = surface.evaluate(u, v);
    ((Point2::new(point.x, point.y) - target).norm() <= tol.linear * 10.0).then_some(u)
}

fn horizontal_section_edge(
    surface: &NurbsSurface3,
    z: f64,
    z_min: f64,
    z_max: f64,
    tol: &Tolerance,
) -> Result<Edge, String> {
    let curve = horizontal_section_curve(surface, z, z_min, z_max, tol)
        .ok_or_else(|| "Failed to build cylinder horizontal section curve".to_string())?;
    let (t_min, t_max) = curve.param_range();
    let start = curve.evaluate(t_min);
    let end = curve.evaluate(t_max);
    Ok(Edge::new(
        curve,
        Vertex::new(start, tol.linear),
        Vertex::new(end, tol.linear),
        tol.linear,
    ))
}

fn orient_edge_for_points(
    edge: &Edge,
    start: Point3,
    end: Point3,
    tol: &Tolerance,
) -> Option<OrientedEdge> {
    if (edge.start_vertex.point - start).norm() <= tol.linear * 10.0
        && (edge.end_vertex.point - end).norm() <= tol.linear * 10.0
    {
        return Some(OrientedEdge::forward(edge.clone()));
    }
    if (edge.end_vertex.point - start).norm() <= tol.linear * 10.0
        && (edge.start_vertex.point - end).norm() <= tol.linear * 10.0
    {
        return Some(OrientedEdge::reversed(edge.clone()));
    }
    None
}

fn edge_lies_on_recognized_cylinder_patch(
    edge: &Edge,
    surface: &NurbsSurface3,
    tol: &Tolerance,
) -> bool {
    let Some((center, radius, z_min, z_max)) = cylinder_patch_axis_circle(surface, tol) else {
        return false;
    };
    let (t_min, t_max) = edge.curve.param_range();
    (0..=8).all(|i| {
        let t = t_min + (t_max - t_min) * (i as f64 / 8.0);
        let point = edge.curve.evaluate(t);
        let radial_distance = (Point2::new(point.x, point.y) - center).norm();
        point.z >= z_min - tol.linear * 10.0
            && point.z <= z_max + tol.linear * 10.0
            && (radial_distance - radius).abs() <= tol.linear * 10.0
    })
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

fn diagnose_selected_face_stitching(
    pieces: &[SelectedBooleanFacePiece],
    tol: &Tolerance,
) -> SelectedFaceStitchReport {
    let edge_uses = collect_stitch_edge_uses(pieces);
    let mut matched_edge_pair_count = 0;
    let mut unmatched_edge_use_count = 0;
    let mut non_manifold_edge_use_count = 0;
    let mut same_direction_edge_use_count = 0;

    for i in 0..edge_uses.len() {
        let mates: Vec<usize> = edge_uses
            .iter()
            .enumerate()
            .filter_map(|(j, candidate)| {
                (i != j && same_undirected_stitch_edge(&edge_uses[i], candidate, tol.linear))
                    .then_some(j)
            })
            .collect();

        match mates.len() {
            0 => unmatched_edge_use_count += 1,
            1 => {
                let mate = mates[0];
                if i < mate {
                    matched_edge_pair_count += 1;
                }
                if !opposite_stitch_edge_direction(&edge_uses[i], &edge_uses[mate], tol.linear) {
                    same_direction_edge_use_count += 1;
                }
            }
            _ => non_manifold_edge_use_count += 1,
        }
    }

    SelectedFaceStitchReport {
        face_piece_count: pieces.len(),
        edge_use_count: edge_uses.len(),
        matched_edge_pair_count,
        unmatched_edge_use_count,
        non_manifold_edge_use_count,
        same_direction_edge_use_count,
    }
}

fn stitch_report_score(report: &SelectedFaceStitchReport) -> (usize, usize, usize) {
    (
        report.unmatched_edge_use_count,
        report.non_manifold_edge_use_count,
        report.same_direction_edge_use_count,
    )
}

fn collect_stitch_edge_uses(pieces: &[SelectedBooleanFacePiece]) -> Vec<StitchEdgeUse> {
    let mut edge_uses = Vec::new();
    for piece in pieces {
        collect_wire_stitch_edge_uses(
            &piece.face.outer_wire,
            piece.reverse_orientation,
            &mut edge_uses,
        );
        for wire in &piece.face.inner_wires {
            collect_wire_stitch_edge_uses(wire, piece.reverse_orientation, &mut edge_uses);
        }
    }

    edge_uses
}

fn collect_wire_stitch_edge_uses(
    wire: &Wire,
    reverse_orientation: bool,
    edge_uses: &mut Vec<StitchEdgeUse>,
) {
    for edge in &wire.edges {
        let start = edge.start_vertex().point;
        let end = edge.end_vertex().point;
        if reverse_orientation {
            edge_uses.push(StitchEdgeUse {
                start: end,
                end: start,
            });
        } else {
            edge_uses.push(StitchEdgeUse { start, end });
        }
    }
}

fn reverse_face_orientation(face: &Face) -> Face {
    let outer_wire = reverse_wire(&face.outer_wire);
    let inner_wires = face.inner_wires.iter().map(reverse_wire).collect();
    Face::new(
        face.geometry.clone(),
        outer_wire,
        inner_wires,
        face.orientation.reversed(),
        face.tolerance,
    )
}

fn reverse_wire(wire: &Wire) -> Wire {
    Wire::new(
        wire.edges
            .iter()
            .rev()
            .map(|edge| OrientedEdge::new(edge.edge.clone(), edge.orientation.reversed()))
            .collect(),
    )
}

fn same_undirected_stitch_edge(a: &StitchEdgeUse, b: &StitchEdgeUse, tol: f64) -> bool {
    (points_same_3d(a.start, b.start, tol) && points_same_3d(a.end, b.end, tol))
        || (points_same_3d(a.start, b.end, tol) && points_same_3d(a.end, b.start, tol))
}

fn opposite_stitch_edge_direction(a: &StitchEdgeUse, b: &StitchEdgeUse, tol: f64) -> bool {
    points_same_3d(a.start, b.end, tol) && points_same_3d(a.end, b.start, tol)
}

fn points_same_3d(a: Point3, b: Point3, tol: f64) -> bool {
    (a - b).norm() <= tol
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
        (FaceGeometry::Plane(plane), FaceGeometry::Nurbs(surface)) => Some(
            intersect_plane_cylinder_patch(plane, oriented_plane_normal(face_a), surface, tol),
        ),
        (FaceGeometry::Nurbs(surface), FaceGeometry::Plane(plane)) => Some(
            intersect_plane_cylinder_patch(plane, oriented_plane_normal(face_b), surface, tol),
        ),
        (FaceGeometry::Nurbs(_), FaceGeometry::Nurbs(_)) => Some(FaceIntersectionKind::Unsupported),
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

/// Dispatches plane/cylinder-patch support intersection by plane orientation.
///
/// Only axis-aligned cases are recognized so far: a plane perpendicular to the
/// cylinder axis yields a horizontal section arc, and a plane parallel to the
/// axis yields a vertical ruling line.
fn intersect_plane_cylinder_patch(
    plane: &PlaneSurface3,
    plane_normal: Vec3,
    surface: &NurbsSurface3,
    tol: &Tolerance,
) -> FaceIntersectionKind {
    let Some(normal) = plane_normal.try_normalize_safe(1e-12) else {
        return FaceIntersectionKind::Unsupported;
    };
    let axis = Vec3::new(0.0, 0.0, 1.0);
    if normal.cross(&axis).norm() <= tol.angular {
        return intersect_horizontal_plane_cylinder_patch(plane, surface, tol);
    }
    if normal.dot(&axis).abs() <= tol.angular {
        return intersect_vertical_plane_cylinder_patch(plane, normal, surface, tol);
    }

    FaceIntersectionKind::Unsupported
}

fn intersect_horizontal_plane_cylinder_patch(
    plane: &PlaneSurface3,
    surface: &NurbsSurface3,
    tol: &Tolerance,
) -> FaceIntersectionKind {
    let Some((z_min, z_max)) = cylinder_patch_z_span(surface, tol) else {
        return FaceIntersectionKind::Unsupported;
    };
    let z = plane.origin.z;
    if z < z_min - tol.linear || z > z_max + tol.linear {
        return FaceIntersectionKind::Unsupported;
    }

    let Some(curve) = horizontal_section_curve(surface, z.clamp(z_min, z_max), z_min, z_max, tol)
    else {
        return FaceIntersectionKind::Unsupported;
    };
    let (t_min, t_max) = curve.param_range();
    let start = curve.evaluate(t_min);
    let end = curve.evaluate(t_max);
    if !point_lies_on_plane(start, plane, tol) || !point_lies_on_plane(end, plane, tol) {
        return FaceIntersectionKind::Unsupported;
    }

    FaceIntersectionKind::Curve {
        edge: Edge::new(
            curve,
            Vertex::new(start, tol.linear),
            Vertex::new(end, tol.linear),
            tol.linear,
        ),
    }
}

/// Intersects a plane parallel to the cylinder axis with a recognized cylinder patch.
///
/// The support intersection of an infinite plane and a full cylinder is zero,
/// one, or two vertical rulings. Only the case where exactly one ruling lands on
/// this patch's angular span is promoted; a plane cutting the same patch twice is
/// left `Unsupported` until the split stage can consume multiple rulings per pair.
fn intersect_vertical_plane_cylinder_patch(
    plane: &PlaneSurface3,
    plane_normal: Vec3,
    surface: &NurbsSurface3,
    tol: &Tolerance,
) -> FaceIntersectionKind {
    let Some((center, radius, z_min, z_max)) = cylinder_patch_axis_circle(surface, tol) else {
        return FaceIntersectionKind::Unsupported;
    };

    let normal_xy = Vec2::new(plane_normal.x, plane_normal.y);
    let Some(normal_xy) = normal_xy.try_normalize(1e-12) else {
        return FaceIntersectionKind::Unsupported;
    };
    let tangent_xy = Vec2::new(-normal_xy.y, normal_xy.x);

    let origin_xy = Point2::new(plane.origin.x, plane.origin.y);
    let center_offset = (center - origin_xy).dot(&normal_xy);
    let foot = center - normal_xy * center_offset;
    let half_chord_sq = radius * radius - center_offset * center_offset;
    if half_chord_sq < -tol.linear * radius.max(1.0) {
        return FaceIntersectionKind::Unsupported;
    }

    let half_chord = half_chord_sq.max(0.0).sqrt();
    let offsets: Vec<f64> = if half_chord <= tol.linear {
        vec![0.0]
    } else {
        vec![half_chord, -half_chord]
    };

    let mut hits = Vec::new();
    for offset in offsets {
        let hit = foot + tangent_xy * offset;
        if !point_lies_on_cylinder_patch_arc(surface, hit, center, tol) {
            continue;
        }
        if hits
            .iter()
            .any(|existing| points_same_2d(*existing, hit, tol.linear))
        {
            continue;
        }
        hits.push(hit);
    }

    if hits.len() != 1 {
        return FaceIntersectionKind::Unsupported;
    }

    let hit = hits[0];
    let segment_start = Point3::new(hit.x, hit.y, z_min);
    let segment_end = Point3::new(hit.x, hit.y, z_max);
    if !point_lies_on_plane(segment_start, plane, tol)
        || !point_lies_on_plane(segment_end, plane, tol)
    {
        return FaceIntersectionKind::Unsupported;
    }

    FaceIntersectionKind::Line {
        point: segment_start,
        direction: Vec3::new(0.0, 0.0, 1.0),
        segment_start,
        segment_end,
    }
}

/// Recovers the axis circle (XY center, radius) and Z span of a recognized
/// Z-axis cylinder patch, or `None` when the patch is not such a cylinder.
fn cylinder_patch_axis_circle(
    surface: &NurbsSurface3,
    tol: &Tolerance,
) -> Option<(Point2, f64, f64, f64)> {
    let (z_min, z_max) = cylinder_patch_z_span(surface, tol)?;
    let section = horizontal_section_curve(surface, z_min, z_min, z_max, tol)?;
    let samples = sample_section_points_2d(&section, 8);
    let center = circumcenter_2d(
        samples[0],
        samples[samples.len() / 2],
        samples[samples.len() - 1],
        tol,
    )?;
    let radius = (samples[0] - center).norm();
    if radius <= tol.linear {
        return None;
    }
    if samples
        .iter()
        .any(|sample| ((sample - center).norm() - radius).abs() > tol.linear)
    {
        return None;
    }

    Some((center, radius, z_min, z_max))
}

fn sample_section_points_2d(section: &NurbsCurve3, segments: usize) -> Vec<Point2> {
    let (t_min, t_max) = section.param_range();
    (0..=segments)
        .map(|step| {
            let t = t_min + (t_max - t_min) * (step as f64 / segments as f64);
            let point = section.evaluate(t);
            Point2::new(point.x, point.y)
        })
        .collect()
}

fn circumcenter_2d(a: Point2, b: Point2, c: Point2, tol: &Tolerance) -> Option<Point2> {
    let ab = b - a;
    let ac = c - a;
    let det = 2.0 * cross2(ab, ac);
    if det.abs() <= tol.parametric {
        return None;
    }

    let ab_sq = ab.norm_squared();
    let ac_sq = ac.norm_squared();
    let center_offset = Vec2::new(
        (ac.y * ab_sq - ab.y * ac_sq) / det,
        (ab.x * ac_sq - ac.x * ab_sq) / det,
    );

    Some(a + center_offset)
}

/// Tests whether an XY point at cylinder radius falls inside the patch's angular
/// span. Angles are compared instead of distances so a quarter-arc patch is not
/// widened by polyline sampling error.
fn point_lies_on_cylinder_patch_arc(
    surface: &NurbsSurface3,
    point: Point2,
    center: Point2,
    tol: &Tolerance,
) -> bool {
    let Some((z_min, z_max)) = cylinder_patch_z_span(surface, tol) else {
        return false;
    };
    let Some(section) = horizontal_section_curve(surface, z_min, z_min, z_max, tol) else {
        return false;
    };

    let samples = sample_section_points_2d(&section, 1);
    let start_angle = angle_at_center(samples[0], center);
    let end_angle = angle_at_center(samples[samples.len() - 1], center);
    let sweep = wrap_signed_angle(end_angle - start_angle);
    if sweep.abs() <= tol.angular {
        return false;
    }

    let point_angle = angle_at_center(point, center);
    let ratio = wrap_signed_angle(point_angle - start_angle) / sweep;
    let margin = tol.angular / sweep.abs();

    (-margin..=1.0 + margin).contains(&ratio)
}

fn angle_at_center(point: Point2, center: Point2) -> f64 {
    let offset = point - center;
    offset.y.atan2(offset.x)
}

fn wrap_signed_angle(angle: f64) -> f64 {
    let two_pi = std::f64::consts::TAU;
    let mut wrapped = angle % two_pi;
    if wrapped > std::f64::consts::PI {
        wrapped -= two_pi;
    } else if wrapped <= -std::f64::consts::PI {
        wrapped += two_pi;
    }
    wrapped
}

fn cylinder_patch_z_span(surface: &NurbsSurface3, tol: &Tolerance) -> Option<(f64, f64)> {
    if surface.degree_u != 2
        || surface.degree_v != 1
        || surface.control_points.len() != 3
        || surface.control_points.iter().any(|row| row.len() != 2)
    {
        return None;
    }

    for row in &surface.control_points {
        let bottom = row[0];
        let top = row[1];
        if (bottom.weight - top.weight).abs() > tol.linear
            || (bottom.point.x - top.point.x).abs() > tol.linear
            || (bottom.point.y - top.point.y).abs() > tol.linear
        {
            return None;
        }
    }

    let z_min = surface.control_points[0][0].point.z;
    let z_max = surface.control_points[0][1].point.z;
    if z_max - z_min <= tol.linear {
        return None;
    }
    if surface.control_points.iter().any(|row| {
        (row[0].point.z - z_min).abs() > tol.linear || (row[1].point.z - z_max).abs() > tol.linear
    }) {
        return None;
    }

    Some((z_min, z_max))
}

fn horizontal_section_curve(
    surface: &NurbsSurface3,
    z: f64,
    z_min: f64,
    z_max: f64,
    tol: &Tolerance,
) -> Option<NurbsCurve3> {
    let alpha = (z - z_min) / (z_max - z_min);
    if alpha < -tol.parametric || alpha > 1.0 + tol.parametric {
        return None;
    }

    let control_points = surface
        .control_points
        .iter()
        .map(|row| {
            let bottom = row[0].to_homogeneous();
            let top = row[1].to_homogeneous();
            ControlPoint3::from_homogeneous(&(bottom * (1.0 - alpha) + top * alpha))
        })
        .collect();

    NurbsCurve3::new(
        surface.degree_u,
        control_points,
        KnotVector::new(surface.knots_u.knots.clone()),
    )
    .ok()
}
