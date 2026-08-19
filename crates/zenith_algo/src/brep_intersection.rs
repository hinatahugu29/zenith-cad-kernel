use crate::cap::CapBuilder;
use std::collections::BTreeMap;
use zenith_geom::{ControlPoint3, KnotVector, NurbsCurve3, NurbsSurface3, PlaneSurface3, Surface3};
use zenith_math::{BoundingBox3, Point2, Point3, Tolerance, Vec2, Vec3, Vec3Ext};
use zenith_tess::{tessellate_solid, TessellationParams, TriangleMesh};
use zenith_topo::{Edge, Face, FaceGeometry, FacePcurveLoop, OrientedEdge, Vertex, Wire};
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

        // 同じ平面に重なって乗る面は、両オペランドから同じ領域が採られる。
        // そのまま縫うと同じ稜を4回使うことになるので、ここで解消する。
        resolve_coincident_face_pieces(&mut selected_face_pieces, tol);

        // 隣り合う面の片方だけが辺の途中で切られていると、辺の長さが食い違って
        // 縫合が合わない。相手が持つ頂点を境界辺へ刻み込んで対応させる。
        // 面の形は変わらず、境界に頂点が増えるだけ。
        let mut imprint_points = Vec::new();
        for candidate in Self::collect_intersection_edge_candidates(
            &solid_a.outer_shell.faces,
            &solid_b.outer_shell.faces,
            tol,
        ) {
            imprint_points.push(candidate.edge.start_vertex.point);
            imprint_points.push(candidate.edge.end_vertex.point);
        }

        let imprinted = imprint_vertices_on_edges(
            selected_face_pieces
                .iter()
                .map(|piece| piece.face.clone())
                .collect(),
            &imprint_points,
            tol,
        );
        for (piece, face) in selected_face_pieces.iter_mut().zip(imprinted) {
            piece.face = face;
        }

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

            let base_score = stitch_report_score(&diagnose_selected_face_stitching(
                &selected_face_pieces,
                tol,
            ));
            let (best_score, best_piece) = if reversed_score < forward_score {
                (reversed_score, reversed_piece)
            } else {
                (forward_score, forward_piece)
            };
            // 既に閉じている選択面にキャップを足すと二重になるため、
            // ステッチが改善する場合だけ採用する
            if best_score < base_score {
                selected_face_pieces.push(best_piece);
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

        let faces: Vec<Face> = pieces
            .iter()
            .map(|piece| {
                if piece.reverse_orientation {
                    reverse_face_orientation(&piece.face)
                } else {
                    piece.face.clone()
                }
            })
            .collect();

        // 各面は独立に分割されるので、隣り合う面が同じ稜を別々のエッジとして
        // 作ってしまう。幾何的には閉じていても、エッジの同一性が共有されて
        // いないと他カーネルは閉シェルと認めない（OpenCASCADE は Solid では
        // なく Shell として読む）。ここで実体を一本化する。
        let faces = unify_coincident_edges(faces, tol);

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

    /// Splits a planar face into two faces along an intersection edge.
    ///
    /// The split endpoints are located on the real boundary curves, and the
    /// boundary edges they land on are subdivided exactly, so a chord landing on
    /// a circular face keeps its arcs instead of degrading into a polyline.
    pub fn split_planar_face_by_edge(
        face: &Face,
        split_edge: &Edge,
        tol: &Tolerance,
    ) -> Result<Vec<Face>, String> {
        Self::split_planar_face_by_edge_chain(face, std::slice::from_ref(split_edge), tol)
    }

    /// Splits a planar face along a connected chain of intersection edges.
    ///
    /// Where a corner of the other solid pokes through a face, the imprint of
    /// its boundary is several segments meeting at interior corners: each stops
    /// in the middle of the face and only the chain reaches the boundary at
    /// both ends.
    pub fn split_planar_face_by_edge_chain(
        face: &Face,
        chain: &[Edge],
        tol: &Tolerance,
    ) -> Result<Vec<Face>, String> {
        if chain.is_empty() {
            return Err("Split chain is empty".to_string());
        }
        if chain.len() == 1 {
            return Self::split_planar_face_by_single_edge(face, &chain[0], tol);
        }

        let FaceGeometry::Plane(plane) = &face.geometry else {
            return Err("Only planar faces can be split by an intersection edge".to_string());
        };
        if !face.inner_wires.is_empty() {
            return Err("Planar face splitting with inner wires is not implemented yet".to_string());
        }
        let boundary = &face.outer_wire.edges;
        if boundary.len() < 3 {
            return Err("Cannot split a face with fewer than three boundary edges".to_string());
        }

        for edge in chain {
            if (edge.end_vertex.point - edge.start_vertex.point).norm() <= tol.linear {
                return Err("Split edge is degenerate".to_string());
            }
            if !edge_lies_on_plane(edge, plane, tol) {
                return Err("Split edge must lie on the planar face".to_string());
            }
        }

        let ordered = order_edges_into_open_chain(chain, tol)?;
        let start = ordered.first().unwrap().start_vertex().point;
        let end = ordered.last().unwrap().end_vertex().point;

        let start_hit = locate_point_on_wire(boundary, start, tol)
            .ok_or_else(|| "Split chain start does not lie on the outer boundary".to_string())?;
        let end_hit = locate_point_on_wire(boundary, end, tol)
            .ok_or_else(|| "Split chain end does not lie on the outer boundary".to_string())?;
        if start_hit.edge_index == end_hit.edge_index && (start_hit.t - end_hit.t).abs() <= 1e-9 {
            return Err("Split chain endpoints collapse to one boundary point".to_string());
        }

        let boundary_uv: Vec<Point2> = face
            .outer_wire
            .sample_points(16)
            .iter()
            .map(|point| project_to_plane_uv(*point, plane))
            .collect();
        for edge in chain {
            let mid_uv = project_to_plane_uv(edge_midpoint(edge), plane);
            if !point_in_polygon_2d(mid_uv, &boundary_uv, tol.parametric)
                || point_on_polygon_boundary(mid_uv, &boundary_uv, tol.parametric)
            {
                return Err("Split chain must cross the face interior".to_string());
            }
        }

        let path_a = wire_path_between(boundary, &start_hit, &end_hit, tol)?;
        let path_b = wire_path_between(boundary, &end_hit, &start_hit, tol)?;

        let face_a = face_from_wire_path_and_split_chain(face, path_a, &ordered, tol)?;
        let face_b = face_from_wire_path_and_split_chain(face, path_b, &ordered, tol)?;
        Ok(vec![face_a, face_b])
    }

    fn split_planar_face_by_single_edge(
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
        let boundary = &face.outer_wire.edges;
        if boundary.len() < 3 {
            return Err("Cannot split a face with fewer than three boundary edges".to_string());
        }

        let start = split_edge.start_vertex.point;
        let end = split_edge.end_vertex.point;
        if (end - start).norm() <= tol.linear {
            return Err("Split edge is degenerate".to_string());
        }
        if !edge_lies_on_plane(split_edge, plane, tol) {
            return Err("Split edge must lie on the planar face".to_string());
        }

        let start_hit = locate_point_on_wire(boundary, start, tol)
            .ok_or_else(|| "Split edge start does not lie on the outer boundary".to_string())?;
        let end_hit = locate_point_on_wire(boundary, end, tol)
            .ok_or_else(|| "Split edge end does not lie on the outer boundary".to_string())?;
        // 同じ境界辺の上に両端が乗るのは正当な配置。ただし同じ点に潰れて
        // いるなら切り込みにならない。
        if start_hit.edge_index == end_hit.edge_index
            && (start_hit.t - end_hit.t).abs() <= 1e-9
        {
            return Err("Split edge endpoints collapse to one boundary point".to_string());
        }

        let boundary_uv: Vec<Point2> = face
            .outer_wire
            .sample_points(16)
            .iter()
            .map(|point| project_to_plane_uv(*point, plane))
            .collect();
        let mid_uv = project_to_plane_uv(edge_midpoint(split_edge), plane);
        if !point_in_polygon_2d(mid_uv, &boundary_uv, tol.parametric)
            || point_on_polygon_boundary(mid_uv, &boundary_uv, tol.parametric)
        {
            return Err("Split edge must cross the face interior".to_string());
        }

        let path_a = wire_path_between(boundary, &start_hit, &end_hit, tol)?;
        let path_b = wire_path_between(boundary, &end_hit, &start_hit, tol)?;

        let face_a = face_from_wire_path_and_split_edge(face, path_a, split_edge, tol)?;
        let face_b = face_from_wire_path_and_split_edge(face, path_b, split_edge, tol)?;
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

    /// Imprints a closed intersection loop that lies strictly inside a planar
    /// face, producing the region inside the loop and the remainder of the face
    /// carrying the loop as a hole.
    ///
    /// This is the case a boundary-to-boundary split cannot express: the cut
    /// curves never reach the face boundary, so the face is not cut in two but
    /// perforated.
    pub fn split_planar_face_by_interior_loop(
        face: &Face,
        split_edges: &[Edge],
        tol: &Tolerance,
    ) -> Result<Vec<Face>, String> {
        let FaceGeometry::Plane(plane) = &face.geometry else {
            return Err("Only planar faces can be imprinted by an interior loop".to_string());
        };
        for edge in split_edges {
            if !edge_lies_on_plane(edge, plane, tol) {
                return Err("Imprint loop edges must lie on the planar face".to_string());
            }
        }

        let loop_wire = order_edges_into_closed_wire(split_edges, tol)?;
        let boundary_uv: Vec<Point2> = face
            .outer_wire
            .sample_points(16)
            .iter()
            .map(|point| project_to_plane_uv(*point, plane))
            .collect();
        if boundary_uv.len() < 3 {
            return Err("Face boundary is too coarse to imprint".to_string());
        }

        let loop_uv: Vec<Point2> = loop_wire
            .sample_points(16)
            .iter()
            .map(|point| project_to_plane_uv(*point, plane))
            .collect();
        if loop_uv.len() < 3 {
            return Err("Imprint loop is too coarse".to_string());
        }
        for uv in &loop_uv {
            if !point_in_polygon_2d(*uv, &boundary_uv, tol.parametric)
                || point_on_polygon_boundary(*uv, &boundary_uv, tol.parametric)
            {
                return Err("Imprint loop must stay strictly inside the face".to_string());
            }
        }

        // 穴ループは外周ループと逆回りに、内側の面は同じ回りに揃える
        let same_winding = signed_area_2d(&boundary_uv) * signed_area_2d(&loop_uv) > 0.0;
        let reversed_loop = reverse_wire(&loop_wire);
        let (inner_face_wire, hole_wire) = if same_winding {
            (loop_wire, reversed_loop)
        } else {
            (reversed_loop, loop_wire)
        };

        // 既にある穴を、新しいループの内と外に振り分ける。座ぐりのように
        // 既存の穴を囲むループを刻むと、内側の面はドーナツ状になる。
        let mut holes_inside = Vec::new();
        let mut holes_outside = Vec::new();
        for wire in &face.inner_wires {
            let wire_uv: Vec<Point2> = wire
                .sample_points(16)
                .iter()
                .map(|point| project_to_plane_uv(*point, plane))
                .collect();
            if wire_uv.is_empty() {
                holes_outside.push(wire.clone());
                continue;
            }

            let inside_count = wire_uv
                .iter()
                .filter(|uv| point_in_polygon_2d(**uv, &loop_uv, tol.parametric))
                .count();

            // 新しいループと既存の穴が交差している場合は、この単純な振り分けでは
            // 表せないので手を出さない。
            if inside_count == wire_uv.len() {
                holes_inside.push(wire.clone());
            } else if inside_count == 0 {
                holes_outside.push(wire.clone());
            } else {
                return Err(
                    "Imprint loop crosses an existing hole; that case is not implemented"
                        .to_string(),
                );
            }
        }

        let inner_face = Face::new(
            face.geometry.clone(),
            inner_face_wire,
            holes_inside,
            face.orientation,
            face.tolerance,
        );
        let mut outer_holes = holes_outside;
        outer_holes.push(hole_wire);
        let outer_face = Face::new(
            face.geometry.clone(),
            face.outer_wire.clone(),
            outer_holes,
            face.orientation,
            face.tolerance,
        );

        for piece in [&inner_face, &outer_face] {
            let report = piece.validate_pcurves(tol, 8)?;
            if !report.is_valid() {
                return Err(format!(
                    "Imprinted face p-curves are invalid with {} mismatches",
                    report.mismatch_count
                ));
            }
        }

        Ok(vec![inner_face, outer_face])
    }

    pub fn split_face_by_edges(
        face: &Face,
        split_edges: &[Edge],
        tol: &Tolerance,
    ) -> Result<PlanarFaceMultiSplitResult, String> {
        // 面の内部で閉じるループは境界間分割では表せないので、先に刻印を試す
        if split_edges.len() >= 3 {
            if let Ok(faces) = Self::split_planar_face_by_interior_loop(face, split_edges, tol) {
                return Ok(PlanarFaceMultiSplitResult {
                    faces,
                    applied_split_count: 1,
                    skipped_split_count: 0,
                });
            }
        }

        let mut faces = vec![face.clone()];
        let mut applied_split_count = 0;
        let mut skipped_split_count: usize = 0;

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

        // 1本ずつではどれも面を横断できなかった平面に限り、内部の角で繋がった
        // 交線を鎖にまとめて切り込みとして試す。面の境界に沿って走る交線は
        // 切り込みではないので、鎖に混ぜる前に外す。
        if applied_split_count == 0
            && matches!(face.geometry, FaceGeometry::Plane(_))
            && split_edges.len() >= 2
        {
            let cutting: Vec<Edge> = deduplicate_split_edges(split_edges, tol)
                .into_iter()
                .filter(|edge| !edge_runs_along_face_boundary(face, edge, tol))
                .collect();
            let chains = group_edges_into_chains(&cutting, tol);

            let mut chain_faces = vec![face.clone()];
            let mut applied: usize = 0;
            for chain in chains.iter().filter(|chain| chain.len() >= 2) {
                let mut next_faces = Vec::new();
                for current_face in chain_faces {
                    match Self::split_planar_face_by_edge_chain(&current_face, chain, tol) {
                        Ok(split_faces) => {
                            applied += 1;
                            next_faces.extend(split_faces);
                        }
                        Err(_) => next_faces.push(current_face),
                    }
                }
                chain_faces = next_faces;
            }

            if applied > 0 {
                return Ok(PlanarFaceMultiSplitResult {
                    faces: chain_faces,
                    applied_split_count: applied,
                    skipped_split_count: skipped_split_count.saturating_sub(applied),
                });
            }
        }

        Ok(PlanarFaceMultiSplitResult {
            faces,
            applied_split_count,
            skipped_split_count,
        })
    }
}

#[derive(Debug, Clone, Copy)]
struct StitchEdgeUse {
    start: Point3,
    end: Point3,
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

/// A located point on a wire: which oriented edge carries it, and where.
#[derive(Debug, Clone, Copy)]
struct WireHit {
    edge_index: usize,
    /// Normalized parameter along the oriented edge, in traversal direction.
    t: f64,
}

/// Finds which boundary edge a point sits on, refining the parameter by a
/// ternary search on the distance to the real curve rather than to a sampled
/// polyline.
fn locate_point_on_wire(edges: &[OrientedEdge], point: Point3, tol: &Tolerance) -> Option<WireHit> {
    const COARSE_SAMPLES: usize = 64;
    const REFINE_STEPS: usize = 80;

    let mut best: Option<(f64, WireHit)> = None;
    for (edge_index, edge) in edges.iter().enumerate() {
        let distance_at = |t: f64| (edge.evaluate_normalized(t) - point).norm();

        let mut coarse_index = 0;
        let mut coarse_distance = f64::INFINITY;
        for step in 0..=COARSE_SAMPLES {
            let distance = distance_at(step as f64 / COARSE_SAMPLES as f64);
            if distance < coarse_distance {
                coarse_distance = distance;
                coarse_index = step;
            }
        }

        let mut low = (coarse_index.saturating_sub(1)) as f64 / COARSE_SAMPLES as f64;
        let mut high = (coarse_index + 1).min(COARSE_SAMPLES) as f64 / COARSE_SAMPLES as f64;
        for _ in 0..REFINE_STEPS {
            let third = (high - low) / 3.0;
            let left = low + third;
            let right = high - third;
            if distance_at(left) < distance_at(right) {
                high = right;
            } else {
                low = left;
            }
        }

        let t = 0.5 * (low + high);
        let distance = distance_at(t);
        if distance > tol.linear * 10.0 {
            continue;
        }
        if best.is_none_or(|(best_distance, _)| distance < best_distance) {
            best = Some((distance, WireHit { edge_index, t }));
        }
    }

    best.map(|(_, hit)| hit)
}

/// Walks the wire from one hit to the other in traversal order, subdividing the
/// two boundary edges the hits landed on.
fn wire_path_between(
    edges: &[OrientedEdge],
    from: &WireHit,
    to: &WireHit,
    tol: &Tolerance,
) -> Result<Vec<OrientedEdge>, String> {
    // 両端が同じ境界辺の上にあるとき、片方の経路はその辺の内側の一区間で
    // 済む。一周する経路と区別しないと、両側とも遠回りになって面が切れない。
    // 小さな出っ張りが一辺から入って同じ辺から出る配置で実際に起きる。
    if from.edge_index == to.edge_index && from.t < to.t {
        return match oriented_edge_portion(&edges[from.edge_index], from.t, to.t, tol)? {
            Some(portion) => Ok(vec![portion]),
            None => Err("Split edge did not produce a boundary path".to_string()),
        };
    }

    let mut path = Vec::new();
    if let Some(tail) = oriented_edge_portion(&edges[from.edge_index], from.t, 1.0, tol)? {
        path.push(tail);
    }

    let mut index = (from.edge_index + 1) % edges.len();
    while index != to.edge_index {
        path.push(edges[index].clone());
        index = (index + 1) % edges.len();
    }

    if let Some(head) = oriented_edge_portion(&edges[to.edge_index], 0.0, to.t, tol)? {
        path.push(head);
    }

    if path.is_empty() {
        return Err("Split edge did not produce a boundary path".to_string());
    }
    Ok(path)
}

/// Returns the `[t_start, t_end]` portion of an oriented edge in traversal
/// direction, or `None` when the portion collapses to a point.
fn oriented_edge_portion(
    edge: &OrientedEdge,
    t_start: f64,
    t_end: f64,
    tol: &Tolerance,
) -> Result<Option<OrientedEdge>, String> {
    let length = oriented_edge_length(edge);
    let parametric_tol = if length > tol.linear {
        (tol.linear / length).min(1e-6)
    } else {
        1e-6
    };
    let keeps_start = t_start <= parametric_tol;
    let keeps_end = t_end >= 1.0 - parametric_tol;

    if keeps_start && keeps_end {
        return Ok(Some(edge.clone()));
    }
    if t_end - t_start <= parametric_tol {
        return Ok(None);
    }
    if !keeps_start && !keeps_end {
        return Err("Boundary edge would need two split points".to_string());
    }

    let split_t = if keeps_start { t_end } else { t_start };
    let (low, high) = split_oriented_edge_at(edge, split_t, tol)?;
    Ok(Some(if keeps_start { low } else { high }))
}

/// Subdivides an oriented edge at a normalized traversal parameter, returning
/// the portions before and after it in traversal order.
fn split_oriented_edge_at(
    edge: &OrientedEdge,
    t: f64,
    tol: &Tolerance,
) -> Result<(OrientedEdge, OrientedEdge), String> {
    let curve = &edge.edge.curve;
    let (t_min, t_max) = curve.param_range();
    let curve_param = if edge.orientation.is_forward() {
        t_min + (t_max - t_min) * t
    } else {
        t_max - (t_max - t_min) * t
    };

    let (low, high) = curve
        .split_bezier_at(curve_param)
        .ok_or_else(|| "Boundary edge is not a single splittable Bezier span".to_string())?;
    let (first, second) = if edge.orientation.is_forward() {
        (low, high)
    } else {
        (high, low)
    };

    Ok((
        OrientedEdge::new(curve_to_edge(first, tol), edge.orientation),
        OrientedEdge::new(curve_to_edge(second, tol), edge.orientation),
    ))
}

fn curve_to_edge(curve: NurbsCurve3, tol: &Tolerance) -> Edge {
    let (t_min, t_max) = curve.param_range();
    let start = curve.evaluate(t_min);
    let end = curve.evaluate(t_max);
    Edge::new(
        curve,
        Vertex::new(start, tol.linear),
        Vertex::new(end, tol.linear),
        tol.linear,
    )
}

fn oriented_edge_length(edge: &OrientedEdge) -> f64 {
    edge.sample_points(8, true)
        .windows(2)
        .map(|pair| (pair[1] - pair[0]).norm())
        .sum()
}

/// True when an intersection segment lies along the face's own boundary.
///
/// Such a segment records that the two solids touch along that edge, not that
/// one cuts the other, so it must be kept out of the cutting chains.
fn edge_runs_along_face_boundary(face: &Face, edge: &Edge, tol: &Tolerance) -> bool {
    let FaceGeometry::Plane(plane) = &face.geometry else {
        return false;
    };
    let boundary_uv: Vec<Point2> = face
        .outer_wire
        .sample_points(16)
        .iter()
        .map(|point| project_to_plane_uv(*point, plane))
        .collect();
    if boundary_uv.len() < 3 {
        return false;
    }

    let mid_uv = project_to_plane_uv(edge_midpoint(edge), plane);
    point_on_polygon_boundary(mid_uv, &boundary_uv, tol.parametric)
}

/// Removes segments that describe the same span, in either direction.
fn deduplicate_split_edges(edges: &[Edge], tol: &Tolerance) -> Vec<Edge> {
    let mut unique: Vec<Edge> = Vec::with_capacity(edges.len());
    for edge in edges {
        let start = edge.start_vertex.point;
        let end = edge.end_vertex.point;
        let midpoint = edge_midpoint(edge);
        let duplicate = unique.iter().any(|existing| {
            let existing_start = existing.start_vertex.point;
            let existing_end = existing.end_vertex.point;
            let same_span = (points_same_3d(existing_start, start, tol.linear)
                && points_same_3d(existing_end, end, tol.linear))
                || (points_same_3d(existing_start, end, tol.linear)
                    && points_same_3d(existing_end, start, tol.linear));
            same_span && points_same_3d(edge_midpoint(existing), midpoint, tol.linear * 10.0)
        });
        if !duplicate {
            unique.push(edge.clone());
        }
    }
    unique
}

/// Links edges end to end into chains, by coincident endpoints.
fn group_edges_into_chains(edges: &[Edge], tol: &Tolerance) -> Vec<Vec<Edge>> {
    let mut remaining: Vec<Edge> = edges.to_vec();
    let mut chains: Vec<Vec<Edge>> = Vec::new();

    while !remaining.is_empty() {
        let seed = remaining.remove(0);
        let mut head = seed.start_vertex.point;
        let mut tail = seed.end_vertex.point;
        let mut chain = vec![seed];

        loop {
            let extension = remaining.iter().position(|candidate| {
                let candidate_start = candidate.start_vertex.point;
                let candidate_end = candidate.end_vertex.point;
                points_same_3d(candidate_start, tail, tol.linear)
                    || points_same_3d(candidate_end, tail, tol.linear)
                    || points_same_3d(candidate_start, head, tol.linear)
                    || points_same_3d(candidate_end, head, tol.linear)
            });

            let Some(index) = extension else {
                break;
            };
            let edge = remaining.remove(index);
            let start = edge.start_vertex.point;
            let end = edge.end_vertex.point;

            if points_same_3d(start, tail, tol.linear) {
                tail = end;
            } else if points_same_3d(end, tail, tol.linear) {
                tail = start;
            } else if points_same_3d(start, head, tol.linear) {
                head = end;
            } else {
                head = start;
            }
            chain.push(edge);
        }

        chains.push(chain);
    }

    chains
}

/// Orders a chain's edges head to tail, giving each its traversal direction.
fn order_edges_into_open_chain(
    edges: &[Edge],
    tol: &Tolerance,
) -> Result<Vec<OrientedEdge>, String> {
    if edges.len() == 1 {
        return Ok(vec![OrientedEdge::forward(edges[0].clone())]);
    }

    let mut remaining: Vec<Edge> = edges.to_vec();
    let mut chain = vec![OrientedEdge::forward(remaining.remove(0))];

    loop {
        let tail = chain.last().unwrap().end_vertex().point;
        let Some(index) = remaining.iter().position(|edge| {
            points_same_3d(edge.start_vertex.point, tail, tol.linear)
                || points_same_3d(edge.end_vertex.point, tail, tol.linear)
        }) else {
            break;
        };
        let edge = remaining.remove(index);
        if points_same_3d(edge.start_vertex.point, tail, tol.linear) {
            chain.push(OrientedEdge::forward(edge));
        } else {
            chain.push(OrientedEdge::reversed(edge));
        }
    }

    loop {
        let head = chain.first().unwrap().start_vertex().point;
        let Some(index) = remaining.iter().position(|edge| {
            points_same_3d(edge.end_vertex.point, head, tol.linear)
                || points_same_3d(edge.start_vertex.point, head, tol.linear)
        }) else {
            break;
        };
        let edge = remaining.remove(index);
        if points_same_3d(edge.end_vertex.point, head, tol.linear) {
            chain.insert(0, OrientedEdge::forward(edge));
        } else {
            chain.insert(0, OrientedEdge::reversed(edge));
        }
    }

    if !remaining.is_empty() {
        return Err(format!(
            "Split edges do not form a single chain; {} edge(s) are left over",
            remaining.len()
        ));
    }

    Ok(chain)
}

fn face_from_wire_path_and_split_chain(
    template: &Face,
    mut path: Vec<OrientedEdge>,
    chain: &[OrientedEdge],
    tol: &Tolerance,
) -> Result<Face, String> {
    let expected_start = path.last().unwrap().end_vertex().point;
    let expected_end = path[0].start_vertex().point;

    let chain_start = chain.first().unwrap().start_vertex().point;
    let chain_end = chain.last().unwrap().end_vertex().point;

    if points_same_3d(chain_start, expected_start, tol.linear)
        && points_same_3d(chain_end, expected_end, tol.linear)
    {
        path.extend(chain.iter().cloned());
    } else if points_same_3d(chain_end, expected_start, tol.linear)
        && points_same_3d(chain_start, expected_end, tol.linear)
    {
        path.extend(chain.iter().rev().map(|oriented| {
            OrientedEdge::new(oriented.edge.clone(), oriented.orientation.reversed())
        }));
    } else {
        return Err("Split chain orientation does not close the split loop".to_string());
    }

    let wire = Wire::new(path);
    if !wire.is_closed(tol) {
        return Err("Planar split produced an open wire".to_string());
    }

    Ok(Face::new(
        template.geometry.clone(),
        wire,
        Vec::new(),
        template.orientation,
        template.tolerance,
    ))
}

fn face_from_wire_path_and_split_edge(
    template: &Face,
    mut path: Vec<OrientedEdge>,
    split_edge: &Edge,
    tol: &Tolerance,
) -> Result<Face, String> {
    let expected_start = path.last().unwrap().end_vertex().point;
    let expected_end = path[0].start_vertex().point;
    let oriented_split = orient_edge_for_points(split_edge, expected_start, expected_end, tol)
        .ok_or_else(|| "Split edge orientation does not close the split loop".to_string())?;
    path.push(oriented_split);

    let wire = Wire::new(path);
    if !wire.is_closed(tol) {
        return Err("Planar split produced an open wire".to_string());
    }

    Ok(Face::new(
        template.geometry.clone(),
        wire,
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
    let Some(patch) = recognize_cylinder_patch(surface, tol) else {
        return Err("Only recognized cylinder-side NURBS patches can be split".to_string());
    };
    if !edge_lies_on_recognized_cylinder_patch(split_edge, surface, tol) {
        return Err("Split edge must lie on the NURBS face".to_string());
    }

    // 一度分割された面は、境界ワイヤだけが変わって元の曲面をそのまま持ち続ける。
    // したがって分割可否も分割後の境界も、曲面の全体範囲ではなく「その面が実際に
    // 占めている範囲」で決めなければならない。曲面側を見ていたために、z=-20..0 の
    // ピースを z=20 の円弧で「分割」して重なったピースを作っていた。
    let bounds = cylinder_face_bounds(face, &patch, tol)
        .ok_or_else(|| "Cylinder-side face boundary is not a four-sided patch".to_string())?;

    let (split_start, split_end) = ruling_boundary_endpoints(
        split_edge,
        bounds.bottom_start,
        bounds.bottom_end,
        &patch,
        tol,
    )
    .ok_or_else(|| "Split edge endpoints do not match cylinder side boundaries".to_string())?;

    for endpoint in [split_start, split_end] {
        let axial = patch.axial_coordinate(endpoint);
        if axial <= bounds.bottom_axial + tol.linear || axial >= bounds.top_axial - tol.linear {
            return Err("Cylinder-side split edge must cross the face interior".to_string());
        }
    }

    let bottom_start = bounds.bottom_start;
    let bottom_end = bounds.bottom_end;
    let top_start = bounds.top_start;
    let top_end = bounds.top_end;

    let split_forward = orient_edge_for_points(split_edge, split_start, split_end, tol)
        .ok_or_else(|| "Split edge endpoints do not match cylinder side boundaries".to_string())?;
    let split_reversed = OrientedEdge::new(
        split_forward.edge.clone(),
        split_forward.orientation.reversed(),
    );
    // 面から取り出した円弧は向きが揃っているとは限らないので、明示的に
    // bottom_start -> bottom_end / top_start -> top_end に向ける。
    let bottom_oriented =
        orient_edge_for_points(&bounds.bottom_edge, bottom_start, bottom_end, tol)
            .ok_or_else(|| "Cylinder-side bottom arc does not match the face corners".to_string())?;
    let top_oriented = orient_edge_for_points(&bounds.top_edge, top_start, top_end, tol)
        .ok_or_else(|| "Cylinder-side top arc does not match the face corners".to_string())?;
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

    // 境界は「面の外向きから見て反時計回り」で組む。向きフラグが Reversed の
    // 面（前段のブーリアンで裏返された穴の内壁など）では外向きが逆なので、
    // 同じ順序で組むと巡回だけが逆さになり、縫合で同方向のエッジ使用として
    // 現れる。フラグに合わせて巡回を揃える。
    let orient_wire = |edges: Vec<OrientedEdge>| {
        if face.orientation.is_forward() {
            Wire::new(edges)
        } else {
            Wire::new(
                edges
                    .into_iter()
                    .rev()
                    .map(|oriented| {
                        OrientedEdge::new(oriented.edge, oriented.orientation.reversed())
                    })
                    .collect(),
            )
        }
    };

    let lower = Face::new(
        face.geometry.clone(),
        orient_wire(vec![
            bottom_oriented.clone(),
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
        orient_wire(vec![
            split_forward,
            OrientedEdge::forward(right_upper),
            OrientedEdge::new(
                top_oriented.edge.clone(),
                top_oriented.orientation.reversed(),
            ),
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

/// The extent a cylinder-side face actually occupies, read from its boundary
/// wire rather than from the surface it sits on.
///
/// Splitting a face does not shrink its surface - only its wire changes - so a
/// piece that covers a quarter of the patch still reports the whole patch's
/// parameter range. Anything that reasons about "where this face is" has to
/// consult the wire, or it will happily split a piece with an edge that lies
/// outside it.
struct CylinderFaceBounds {
    bottom_edge: Edge,
    top_edge: Edge,
    bottom_axial: f64,
    top_axial: f64,
    bottom_start: Point3,
    bottom_end: Point3,
    top_start: Point3,
    top_end: Point3,
}

fn cylinder_face_bounds(
    face: &Face,
    patch: &CylinderPatch,
    tol: &Tolerance,
) -> Option<CylinderFaceBounds> {
    if face.outer_wire.edges.len() != 4 {
        return None;
    }

    // 円弧側の辺は両端の軸方向座標が等しく、ルーリング側は異なる。
    let mut arcs: Vec<(&Edge, f64)> = Vec::new();
    for oriented in &face.outer_wire.edges {
        let edge = &oriented.edge;
        let start_axial = patch.axial_coordinate(edge.start_vertex.point);
        let end_axial = patch.axial_coordinate(edge.end_vertex.point);
        if (start_axial - end_axial).abs() <= tol.linear * 10.0 {
            arcs.push((edge, 0.5 * (start_axial + end_axial)));
        }
    }

    if arcs.len() != 2 {
        return None;
    }

    let (mut bottom, mut top) = (arcs[0], arcs[1]);
    if bottom.1 > top.1 {
        std::mem::swap(&mut bottom, &mut top);
    }
    if (top.1 - bottom.1).abs() <= tol.linear {
        return None;
    }

    let bottom_start = bottom.0.start_vertex.point;
    let bottom_end = bottom.0.end_vertex.point;

    // 上側の円弧の端点を、下側と同じルーリングに乗るように対応づける。
    let on_same_ruling = |point: Point3, base: Point3| {
        let offset = point - base;
        (offset - patch.axis * offset.dot(&patch.axis)).norm() <= tol.linear * 10.0
    };

    let (top_start, top_end) = {
        let candidate_start = top.0.start_vertex.point;
        let candidate_end = top.0.end_vertex.point;
        if on_same_ruling(candidate_start, bottom_start) && on_same_ruling(candidate_end, bottom_end)
        {
            (candidate_start, candidate_end)
        } else if on_same_ruling(candidate_end, bottom_start)
            && on_same_ruling(candidate_start, bottom_end)
        {
            (candidate_end, candidate_start)
        } else {
            return None;
        }
    };

    Some(CylinderFaceBounds {
        bottom_edge: bottom.0.clone(),
        top_edge: top.0.clone(),
        bottom_axial: bottom.1,
        top_axial: top.1,
        bottom_start,
        bottom_end,
        top_start,
        top_end,
    })
}

/// Matches a split edge's endpoints to the patch's two boundary rulings.
///
/// Returns the endpoints ordered as (on the `u_min` ruling, on the `u_max`
/// ruling), or `None` when either endpoint is not on a boundary ruling.
fn ruling_boundary_endpoints(
    split_edge: &Edge,
    bottom_start: Point3,
    bottom_end: Point3,
    patch: &CylinderPatch,
    tol: &Tolerance,
) -> Option<(Point3, Point3)> {
    let on_ruling = |point: Point3, ruling_base: Point3| {
        let offset = point - ruling_base;
        (offset - patch.axis * offset.dot(&patch.axis)).norm() <= tol.linear * 10.0
    };

    let start = split_edge.start_vertex.point;
    let end = split_edge.end_vertex.point;
    if on_ruling(start, bottom_start) && on_ruling(end, bottom_end) {
        return Some((start, end));
    }
    if on_ruling(end, bottom_start) && on_ruling(start, bottom_end) {
        return Some((end, start));
    }

    None
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
    let Some(patch) = recognize_cylinder_patch(surface, tol) else {
        return Err("Only recognized cylinder-side NURBS patches can be split".to_string());
    };
    if !edge_lies_on_recognized_cylinder_patch(split_edge, surface, tol) {
        return Err("Split edge must lie on the NURBS face".to_string());
    }

    let edge_start = split_edge.start_vertex.point;
    let edge_end = split_edge.end_vertex.point;
    let along_axis = (edge_end - edge_start).dot(&patch.axis);
    if ((edge_end - edge_start) - patch.axis * along_axis).norm() > tol.linear * 10.0 {
        return Err("Cylinder-side split edge must be a ruling along the patch axis".to_string());
    }
    let edge_low = patch
        .axial_coordinate(edge_start)
        .min(patch.axial_coordinate(edge_end));
    let edge_high = patch
        .axial_coordinate(edge_start)
        .max(patch.axial_coordinate(edge_end));
    if edge_low.abs() > tol.linear * 10.0 || (edge_high - patch.height).abs() > tol.linear * 10.0 {
        return Err("Cylinder-side ruling split must span the full patch height".to_string());
    }

    let ((u_min, u_max), (v_min, v_max)) = surface.param_range();
    let u_split = cylinder_patch_u_for_point(surface, &patch, edge_start, tol)
        .ok_or_else(|| "Split ruling does not lie on the patch angular span".to_string())?;
    if u_split <= u_min + tol.parametric || u_split >= u_max - tol.parametric {
        return Err("Cylinder-side ruling split must cross the face interior".to_string());
    }

    let bottom_curve = cylinder_section_curve(surface, 0.0)
        .ok_or_else(|| "Failed to build cylinder bottom section curve".to_string())?;
    let top_curve = cylinder_section_curve(surface, 1.0)
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

/// Finds the `u` parameter whose patch point matches `target`.
///
/// The angle around the axis is monotonic in `u` over a sub-half-turn rational
/// arc, so a bisection on the swept-angle ratio converges without needing a
/// closed form for the rational quadratic parameterization.
fn cylinder_patch_u_for_point(
    surface: &NurbsSurface3,
    patch: &CylinderPatch,
    target: Point3,
    tol: &Tolerance,
) -> Option<f64> {
    let ((u_min, u_max), (v_min, _)) = surface.param_range();
    let (start_angle, sweep) = cylinder_patch_angle_span(surface, patch, tol)?;
    let angle_at = |u: f64| patch.angle_of(surface.evaluate(u, v_min));

    let target_ratio = wrap_signed_angle(patch.angle_of(target) - start_angle) / sweep;
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
    let point = surface.evaluate(u, v_min);
    let radial_offset = point - target;
    (radial_offset.norm() <= tol.linear * 10.0).then_some(u)
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
    let Some(patch) = recognize_cylinder_patch(surface, tol) else {
        return false;
    };
    let (t_min, t_max) = edge.curve.param_range();
    (0..=8).all(|i| {
        let t = t_min + (t_max - t_min) * (i as f64 / 8.0);
        let point = edge.curve.evaluate(t);
        let axial = patch.axial_coordinate(point);
        axial >= -tol.linear * 10.0
            && axial <= patch.height + tol.linear * 10.0
            && (patch.radial_distance(point) - patch.radius).abs() <= tol.linear * 10.0
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

/// Replaces edges that describe the same curve with a single shared edge.
///
/// Faces are split independently, so two neighbouring pieces each build their
/// own edge along the seam between them. Geometrically that is fine and the
/// shell still closes, but edge *identity* is what a downstream kernel checks:
/// with two distinct edges along one seam, OpenCASCADE reads the result as an
/// open shell rather than a solid. Unifying them here is the sewing step.
fn unify_coincident_edges(faces: Vec<Face>, tol: &Tolerance) -> Vec<Face> {
    let mut canonical: Vec<Edge> = Vec::new();

    let mut resolve = |edge: &Edge| -> Option<OrientedEdge> {
        let start = edge.start_vertex.point;
        let end = edge.end_vertex.point;

        for existing in canonical.iter() {
            let existing_start = existing.start_vertex.point;
            let existing_end = existing.end_vertex.point;

            if points_same_3d(existing_start, start, tol.linear)
                && points_same_3d(existing_end, end, tol.linear)
                && curves_coincide(existing, edge, tol)
            {
                return Some(OrientedEdge::forward(existing.clone()));
            }
            if points_same_3d(existing_start, end, tol.linear)
                && points_same_3d(existing_end, start, tol.linear)
                && curves_coincide(existing, edge, tol)
            {
                return Some(OrientedEdge::reversed(existing.clone()));
            }
        }

        canonical.push(edge.clone());
        Some(OrientedEdge::forward(edge.clone()))
    };

    let mut rewrite_wire = |wire: &Wire| -> Wire {
        let edges = wire
            .edges
            .iter()
            .map(|oriented| {
                let Some(resolved) = resolve(&oriented.edge) else {
                    return oriented.clone();
                };
                let orientation = if oriented.orientation.is_forward() {
                    resolved.orientation
                } else {
                    resolved.orientation.reversed()
                };
                OrientedEdge::new(resolved.edge, orientation)
            })
            .collect();
        Wire::new(edges)
    };

    faces
        .iter()
        .map(|face| {
            Face::new(
                face.geometry.clone(),
                rewrite_wire(&face.outer_wire),
                face.inner_wires.iter().map(&mut rewrite_wire).collect(),
                face.orientation,
                face.tolerance,
            )
        })
        .collect()
}

/// Two edges describe the same curve when sampling them at matched parameters
/// stays within tolerance. Endpoints alone are not enough: a straight edge and
/// an arc can share both.
fn curves_coincide(a: &Edge, b: &Edge, tol: &Tolerance) -> bool {
    const SAMPLES: usize = 5;

    let forward = points_same_3d(a.start_vertex.point, b.start_vertex.point, tol.linear);
    let (a_min, a_max) = a.curve.param_range();
    let (b_min, b_max) = b.curve.param_range();

    (0..=SAMPLES).all(|index| {
        let t = index as f64 / SAMPLES as f64;
        let s = if forward { t } else { 1.0 - t };
        let point_a = a.curve.evaluate(a_min + (a_max - a_min) * t);
        let point_b = b.curve.evaluate(b_min + (b_max - b_min) * s);
        points_same_3d(point_a, point_b, tol.linear * 10.0)
    })
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
    // 穴のある面の重心は穴の中に落ちることがあり、内外判定が反転する
    if !face.inner_wires.is_empty() {
        if let FaceGeometry::Plane(plane) = &face.geometry {
            if let Some(point) = planar_point_clear_of_holes(face, plane) {
                return point;
            }
        }
    }

    let points = face.outer_wire.sample_points(2);
    if points.is_empty() {
        return Point3::new(0.0, 0.0, 0.0);
    }

    let sum = points
        .iter()
        .fold(Vec3::new(0.0, 0.0, 0.0), |acc, point| acc + point.coords);
    Point3::from(sum / points.len() as f64)
}

/// Picks the material point of a pierced planar face that sits furthest from
/// every trim loop, so classification samples solid material and not a hole.
fn planar_point_clear_of_holes(face: &Face, plane: &zenith_geom::PlaneSurface3) -> Option<Point3> {
    const GRID: usize = 24;

    let outer: Vec<Point2> = face
        .outer_wire
        .sample_points(8)
        .iter()
        .map(|point| project_to_plane_uv(*point, plane))
        .collect();
    if outer.len() < 3 {
        return None;
    }
    let holes: Vec<Vec<Point2>> = face
        .inner_wires
        .iter()
        .map(|wire| {
            wire.sample_points(8)
                .iter()
                .map(|point| project_to_plane_uv(*point, plane))
                .collect()
        })
        .collect();

    let (mut min_u, mut max_u) = (f64::INFINITY, f64::NEG_INFINITY);
    let (mut min_v, mut max_v) = (f64::INFINITY, f64::NEG_INFINITY);
    for uv in &outer {
        min_u = min_u.min(uv.x);
        max_u = max_u.max(uv.x);
        min_v = min_v.min(uv.y);
        max_v = max_v.max(uv.y);
    }

    let mut best: Option<(f64, Point2)> = None;
    for i in 1..GRID {
        for j in 1..GRID {
            let uv = Point2::new(
                min_u + (max_u - min_u) * (i as f64 / GRID as f64),
                min_v + (max_v - min_v) * (j as f64 / GRID as f64),
            );
            if !point_in_polygon_2d(uv, &outer, 0.0) {
                continue;
            }
            if holes.iter().any(|hole| point_in_polygon_2d(uv, hole, 0.0)) {
                continue;
            }

            let clearance = std::iter::once(&outer)
                .chain(holes.iter())
                .map(|polygon| polygon_min_distance_2d(uv, polygon))
                .fold(f64::INFINITY, f64::min);
            if best.is_none_or(|(best_clearance, _)| clearance > best_clearance) {
                best = Some((clearance, uv));
            }
        }
    }

    best.map(|(_, uv)| plane.evaluate(uv.x, uv.y))
}

fn polygon_min_distance_2d(point: Point2, polygon: &[Point2]) -> f64 {
    let mut best = f64::INFINITY;
    for i in 0..polygon.len() {
        let a = polygon[i];
        let b = polygon[(i + 1) % polygon.len()];
        let ab = b - a;
        let len_sq = ab.norm_squared();
        let closest = if len_sq <= 1e-18 {
            a
        } else {
            a + ab * ((point - a).dot(&ab) / len_sq).clamp(0.0, 1.0)
        };
        best = best.min((point - closest).norm());
    }
    best
}

fn signed_area_2d(polygon: &[Point2]) -> f64 {
    let mut area = 0.0;
    for i in 0..polygon.len() {
        let a = polygon[i];
        let b = polygon[(i + 1) % polygon.len()];
        area += a.x * b.y - b.x * a.y;
    }
    area * 0.5
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
            // 存在判定には公差ぶんの余裕を持たせるが、区間そのものは余裕なしの
            // 重なりで切る。余裕を残したままだと交線が面の外へわずかにはみ出し、
            // 後段のループ組み立てで端点が一致しなくなる。
            let padded = bbox_overlap(bbox_a, bbox_b, tol.linear)?;
            let exact = bbox_overlap(bbox_a, bbox_b, 0.0);
            let (t_min, t_max) = exact
                .as_ref()
                .and_then(|overlap| clip_line_to_bbox(point, direction, overlap, tol.linear))
                .filter(|(t_min, t_max)| t_max - t_min > tol.linear)
                .or_else(|| clip_line_to_bbox(point, direction, &padded, tol.linear))?;
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
    let uv_start = project_to_plane_uv(segment_start, plane);
    let uv_end = project_to_plane_uv(segment_end, plane);
    // 評価できないループはこれまでどおりクリップ対象外として素通しする
    let Some(intervals) =
        segment_inside_pcurve_loop_intervals(uv_start, uv_end, &pcurves.outer_loop, tol)
    else {
        return Some(current_interval);
    };
    intersect_interval_set(current_interval, &intervals, tol.parametric)
}

/// Trim-clips a UV segment against a p-curve loop.
///
/// Interval endpoints come from solving the segment against each p-curve span
/// analytically, so a straight cut across a circular face stops exactly on the
/// arc rather than on a sampled chord that sits a sagitta short of it. Only the
/// inside/outside classification between two crossings still uses a densely
/// sampled polygon, where the test points are far from the boundary.
fn segment_inside_pcurve_loop_intervals(
    start: Point2,
    end: Point2,
    loop_data: &FacePcurveLoop,
    tol: &Tolerance,
) -> Option<Vec<(f64, f64)>> {
    const CLASSIFICATION_SAMPLES: usize = 32;

    let direction = end - start;
    if direction.norm() <= tol.parametric.max(1e-12) {
        return None;
    }
    let polygon = sample_pcurve_loop(loop_data, CLASSIFICATION_SAMPLES);
    if polygon.len() < 3 {
        return None;
    }

    let mut cuts = vec![0.0, 1.0];
    for segment in &loop_data.segments {
        match pcurve_segment_crossings(&segment.curve, start, direction) {
            Some(crossings) => cuts.extend(crossings),
            None => {
                // 高次スパンは従来どおり折れ線近似で交差位置を求める
                let points = segment.curve.sample_points(CLASSIFICATION_SAMPLES);
                for pair in points.windows(2) {
                    if let Some(t) =
                        segment_segment_intersection_t(start, end, pair[0], pair[1], tol.parametric)
                    {
                        cuts.push(t);
                    }
                }
            }
        }
    }

    let cut_tol = tol.parametric.max(1e-9);
    cuts.retain(|t| (-cut_tol..=1.0 + cut_tol).contains(t));
    for t in cuts.iter_mut() {
        *t = t.clamp(0.0, 1.0);
    }
    cuts.sort_by(|a, b| a.total_cmp(b));
    cuts.dedup_by(|a, b| (*a - *b).abs() <= cut_tol);

    let mut intervals = Vec::new();
    for pair in cuts.windows(2) {
        let (t0, t1) = (pair[0], pair[1]);
        if t1 - t0 <= cut_tol {
            continue;
        }
        let mid = start + direction * ((t0 + t1) * 0.5);
        if point_in_polygon_2d(mid, &polygon, tol.parametric) {
            intervals.push((t0, t1));
        }
    }

    Some(merge_intervals(intervals, cut_tol))
}

/// Solves a single-span rational p-curve against an infinite UV line.
///
/// The signed distance to the line is a Bernstein polynomial in the homogeneous
/// numerator, so degree 1 and 2 spans (lines and exact conic arcs) have closed
/// form roots. Returns `None` for spans this solver does not cover.
fn pcurve_segment_crossings(
    curve: &zenith_geom::NurbsCurve2,
    start: Point2,
    direction: Vec2,
) -> Option<Vec<f64>> {
    let order = curve.degree + 1;
    if curve.control_points.len() != order || curve.knots.knots.len() != order * 2 {
        return None;
    }
    if curve.degree == 0 || curve.degree > 2 {
        return None;
    }

    let normal = Vec2::new(-direction.y, direction.x);
    let offset = normal.dot(&start.coords);
    let bernstein: Vec<f64> = curve
        .control_points
        .iter()
        .map(|control_point| {
            let homogeneous = control_point.to_homogeneous();
            normal.x * homogeneous.x + normal.y * homogeneous.y - offset * homogeneous.z
        })
        .collect();

    let roots = match curve.degree {
        1 => solve_linear_bernstein(bernstein[0], bernstein[1]),
        _ => solve_quadratic_bernstein(bernstein[0], bernstein[1], bernstein[2]),
    };

    let (t_min, t_max) = curve.param_range();
    let direction_norm_sq = direction.norm_squared();
    Some(
        roots
            .into_iter()
            .map(|local| {
                let point = curve.evaluate(t_min + (t_max - t_min) * local);
                (point - start).dot(&direction) / direction_norm_sq
            })
            .collect(),
    )
}

fn solve_linear_bernstein(b0: f64, b1: f64) -> Vec<f64> {
    let slope = b1 - b0;
    if slope.abs() <= f64::EPSILON {
        return Vec::new();
    }
    let t = -b0 / slope;
    (0.0..=1.0).contains(&t).then_some(t).into_iter().collect()
}

fn solve_quadratic_bernstein(b0: f64, b1: f64, b2: f64) -> Vec<f64> {
    let a = b0 - 2.0 * b1 + b2;
    let b = 2.0 * (b1 - b0);
    let c = b0;

    if a.abs() <= f64::EPSILON {
        return solve_linear_bernstein(c, c + b);
    }

    let discriminant = b * b - 4.0 * a * c;
    if discriminant < 0.0 {
        return Vec::new();
    }

    let root = discriminant.sqrt();
    [(-b - root) / (2.0 * a), (-b + root) / (2.0 * a)]
        .into_iter()
        .filter(|t| (0.0..=1.0).contains(t))
        .collect()
}

fn sample_pcurve_loop(loop_data: &FacePcurveLoop, samples_per_segment: usize) -> Vec<Point2> {
    // 先頭点は前区間の終点と一致するときだけ落とす。縮退エッジを持つ面では
    // UV 上に正当な跳びがあるため、無条件に落とすと領域が欠ける。
    let mut points: Vec<Point2> = Vec::new();
    for segment in loop_data.segments.iter() {
        for uv in segment.curve.sample_points(samples_per_segment) {
            if points
                .last()
                .is_some_and(|last| points_same_2d(uv, *last, 1e-9))
            {
                continue;
            }
            points.push(uv);
        }
    }

    if points.len() > 1 && points_same_2d(points[0], *points.last().unwrap(), 1e-9) {
        points.pop();
    }

    points
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

/// A NURBS patch recognized as a piece of a circular cylinder of any axis.
#[derive(Debug, Clone, Copy)]
struct CylinderPatch {
    axis: Vec3,
    base_center: Point3,
    radius: f64,
    height: f64,
    frame_u: Vec3,
    frame_v: Vec3,
}

impl CylinderPatch {
    /// Distance along the axis from the base circle.
    fn axial_coordinate(&self, point: Point3) -> f64 {
        (point - self.base_center).dot(&self.axis)
    }

    /// Distance from the axis line.
    fn radial_distance(&self, point: Point3) -> f64 {
        let offset = point - self.base_center;
        (offset - self.axis * offset.dot(&self.axis)).norm()
    }

    /// Angle around the axis in the patch frame.
    fn angle_of(&self, point: Point3) -> f64 {
        let offset = point - self.base_center;
        offset.dot(&self.frame_v).atan2(offset.dot(&self.frame_u))
    }
}

/// Recognizes a cylinder-side patch without assuming a Z axis.
///
/// The patch qualifies when it is linear in `v` with one shared ruling vector,
/// and its `v = 0` section is a circle around that ruling direction.
fn recognize_cylinder_patch(surface: &NurbsSurface3, tol: &Tolerance) -> Option<CylinderPatch> {
    if surface.degree_v != 1 || surface.degree_u != 2 {
        return None;
    }
    if surface.control_points.len() != surface.degree_u + 1
        || surface.control_points.iter().any(|row| row.len() != 2)
    {
        return None;
    }

    let mut ruling: Option<Vec3> = None;
    for row in &surface.control_points {
        let (bottom, top) = (row[0], row[1]);
        if (bottom.weight - top.weight).abs() > tol.linear {
            return None;
        }
        let offset = top.point - bottom.point;
        match ruling {
            None => ruling = Some(offset),
            Some(first) => {
                if (offset - first).norm() > tol.linear {
                    return None;
                }
            }
        }
    }

    let ruling = ruling?;
    let height = ruling.norm();
    if height <= tol.linear {
        return None;
    }
    let axis = ruling / height;
    let frame_u = axis_perpendicular(axis)?;
    let frame_v = axis.cross(&frame_u);

    let section = cylinder_section_curve(surface, 0.0)?;
    let samples = sample_curve_points(&section, 8);
    let origin = samples[0];
    if samples
        .iter()
        .any(|sample| (sample - origin).dot(&axis).abs() > tol.linear)
    {
        return None;
    }

    let to_frame = |point: Point3| {
        let offset = point - origin;
        Point2::new(offset.dot(&frame_u), offset.dot(&frame_v))
    };
    let center_2d = circumcenter_2d(
        to_frame(samples[0]),
        to_frame(samples[samples.len() / 2]),
        to_frame(samples[samples.len() - 1]),
        tol,
    )?;
    let base_center = origin + frame_u * center_2d.x + frame_v * center_2d.y;

    let radius = (samples[0] - base_center).norm();
    if radius <= tol.linear {
        return None;
    }
    if samples
        .iter()
        .any(|sample| ((sample - base_center).norm() - radius).abs() > tol.linear)
    {
        return None;
    }

    Some(CylinderPatch {
        axis,
        base_center,
        radius,
        height,
        frame_u,
        frame_v,
    })
}

fn axis_perpendicular(axis: Vec3) -> Option<Vec3> {
    let seed = if axis.x.abs() < 0.9 {
        Vec3::new(1.0, 0.0, 0.0)
    } else {
        Vec3::new(0.0, 1.0, 0.0)
    };
    (seed - axis * seed.dot(&axis)).try_normalize_safe(1e-12)
}

/// The `v = alpha` iso-section of a ruled patch, as an exact rational curve.
fn cylinder_section_curve(surface: &NurbsSurface3, alpha: f64) -> Option<NurbsCurve3> {
    if !(-1e-9..=1.0 + 1e-9).contains(&alpha) {
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

fn sample_curve_points(curve: &NurbsCurve3, segments: usize) -> Vec<Point3> {
    let (t_min, t_max) = curve.param_range();
    (0..=segments)
        .map(|step| curve.evaluate(t_min + (t_max - t_min) * (step as f64 / segments as f64)))
        .collect()
}

/// Start angle and signed sweep of the patch around its axis.
fn cylinder_patch_angle_span(
    surface: &NurbsSurface3,
    patch: &CylinderPatch,
    tol: &Tolerance,
) -> Option<(f64, f64)> {
    let section = cylinder_section_curve(surface, 0.0)?;
    let (t_min, t_max) = section.param_range();
    let start_angle = patch.angle_of(section.evaluate(t_min));
    let sweep = wrap_signed_angle(patch.angle_of(section.evaluate(t_max)) - start_angle);
    (sweep.abs() > tol.angular).then_some((start_angle, sweep))
}

/// Tests whether a point at cylinder radius falls inside the patch angular span.
///
/// Angles are compared instead of distances so a quarter-arc patch is not
/// widened by polyline sampling error.
fn point_lies_on_cylinder_patch_arc(
    surface: &NurbsSurface3,
    patch: &CylinderPatch,
    point: Point3,
    tol: &Tolerance,
) -> bool {
    let Some((start_angle, sweep)) = cylinder_patch_angle_span(surface, patch, tol) else {
        return false;
    };
    let ratio = wrap_signed_angle(patch.angle_of(point) - start_angle) / sweep;
    let margin = tol.angular / sweep.abs();

    (-margin..=1.0 + margin).contains(&ratio)
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

/// Dispatches plane/cylinder-patch support intersection by plane orientation.
///
/// Two cases are recognized so far: a plane perpendicular to the cylinder axis
/// yields a section arc, and a plane parallel to the axis yields a ruling line.
fn intersect_plane_cylinder_patch(
    plane: &PlaneSurface3,
    plane_normal: Vec3,
    surface: &NurbsSurface3,
    tol: &Tolerance,
) -> FaceIntersectionKind {
    let Some(normal) = plane_normal.try_normalize_safe(1e-12) else {
        return FaceIntersectionKind::Unsupported;
    };
    let Some(patch) = recognize_cylinder_patch(surface, tol) else {
        return FaceIntersectionKind::Unsupported;
    };

    if normal.cross(&patch.axis).norm() <= tol.angular {
        return intersect_section_plane_cylinder_patch(plane, surface, &patch, tol);
    }
    if normal.dot(&patch.axis).abs() <= tol.angular {
        return intersect_ruling_plane_cylinder_patch(plane, normal, surface, &patch, tol);
    }

    intersect_oblique_plane_cylinder_patch(plane, normal, surface, &patch, tol)
}

/// Intersects a plane oblique to the cylinder axis, producing an elliptical arc.
///
/// Projecting the base section arc along the axis onto the cutting plane is an
/// affine map, and rational NURBS are closed under affine maps, so the ellipse
/// comes out exactly: same degree, same knots, same weights, control points
/// moved along the axis. No approximation and no new curve class are needed.
///
/// The arc is only accepted when it stays inside the patch's axial band. Every
/// control point of a rational Bezier bounds the curve for any affine functional,
/// so checking the projected control points is a sound test; a plane that leaves
/// the band would need the arc clipped, which the split stage cannot consume yet.
fn intersect_oblique_plane_cylinder_patch(
    plane: &PlaneSurface3,
    plane_normal: Vec3,
    surface: &NurbsSurface3,
    patch: &CylinderPatch,
    tol: &Tolerance,
) -> FaceIntersectionKind {
    let denominator = plane_normal.dot(&patch.axis);
    if denominator.abs() <= tol.angular {
        return FaceIntersectionKind::Unsupported;
    }
    let Some(base) = cylinder_section_curve(surface, 0.0) else {
        return FaceIntersectionKind::Unsupported;
    };

    let mut control_points = Vec::with_capacity(base.control_points.len());
    for control_point in &base.control_points {
        let shift = -plane_normal.dot(&(control_point.point - plane.origin)) / denominator;
        let projected = control_point.point + patch.axis * shift;
        let axial = patch.axial_coordinate(projected);
        if axial < -tol.linear || axial > patch.height + tol.linear {
            return FaceIntersectionKind::Unsupported;
        }
        control_points.push(ControlPoint3::new(projected, control_point.weight));
    }

    let Ok(curve) = NurbsCurve3::new(base.degree, control_points, base.knots.clone()) else {
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

fn intersect_section_plane_cylinder_patch(
    plane: &PlaneSurface3,
    surface: &NurbsSurface3,
    patch: &CylinderPatch,
    tol: &Tolerance,
) -> FaceIntersectionKind {
    let alpha = patch.axial_coordinate(plane.origin) / patch.height;
    if alpha < -tol.parametric || alpha > 1.0 + tol.parametric {
        return FaceIntersectionKind::Unsupported;
    }

    let Some(curve) = cylinder_section_curve(surface, alpha.clamp(0.0, 1.0)) else {
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

/// Intersects a plane parallel to the cylinder axis with a recognized patch.
///
/// The support intersection of an infinite plane and a full cylinder is zero,
/// one, or two rulings. Only the case where exactly one ruling lands on this
/// patch's angular span is promoted; a plane cutting the same patch twice is
/// left `Unsupported` until the split stage can consume multiple rulings per
/// face pair.
fn intersect_ruling_plane_cylinder_patch(
    plane: &PlaneSurface3,
    plane_normal: Vec3,
    surface: &NurbsSurface3,
    patch: &CylinderPatch,
    tol: &Tolerance,
) -> FaceIntersectionKind {
    let normal_2d = Vec2::new(
        plane_normal.dot(&patch.frame_u),
        plane_normal.dot(&patch.frame_v),
    );
    let Some(normal_2d) = normal_2d.try_normalize(1e-12) else {
        return FaceIntersectionKind::Unsupported;
    };
    let tangent_2d = Vec2::new(-normal_2d.y, normal_2d.x);

    let origin_offset = plane.origin - patch.base_center;
    let origin_2d = Point2::new(
        origin_offset.dot(&patch.frame_u),
        origin_offset.dot(&patch.frame_v),
    );
    // 軸中心は frame 原点にあるので、中心から直線までの符号付き距離は -o.n
    let center_offset = -origin_2d.coords.dot(&normal_2d);
    let foot = Point2::from(-normal_2d * center_offset);
    let half_chord_sq = patch.radius * patch.radius - center_offset * center_offset;
    if half_chord_sq < -tol.linear * patch.radius.max(1.0) {
        return FaceIntersectionKind::Unsupported;
    }

    let half_chord = half_chord_sq.max(0.0).sqrt();
    let offsets: Vec<f64> = if half_chord <= tol.linear {
        vec![0.0]
    } else {
        vec![half_chord, -half_chord]
    };

    let mut hits: Vec<Point3> = Vec::new();
    for offset in offsets {
        let hit_2d = foot + tangent_2d * offset;
        let hit = patch.base_center + patch.frame_u * hit_2d.x + patch.frame_v * hit_2d.y;
        if !point_lies_on_cylinder_patch_arc(surface, patch, hit, tol) {
            continue;
        }
        if hits
            .iter()
            .any(|existing| points_same_3d(*existing, hit, tol.linear))
        {
            continue;
        }
        hits.push(hit);
    }

    if hits.len() != 1 {
        return FaceIntersectionKind::Unsupported;
    }

    let segment_start = hits[0];
    let segment_end = segment_start + patch.axis * patch.height;
    if !point_lies_on_plane(segment_start, plane, tol)
        || !point_lies_on_plane(segment_end, plane, tol)
    {
        return FaceIntersectionKind::Unsupported;
    }

    FaceIntersectionKind::Line {
        point: segment_start,
        direction: patch.axis,
        segment_start,
        segment_end,
    }
}

/// Subdivides boundary edges wherever another face's vertex sits in their
/// interior.
///
/// Where two solids meet along part of an edge, one face keeps the full edge
/// while its neighbour has already been cut, so the two do not correspond and
/// the shell will not stitch. Splitting the longer edge at the neighbour's
/// vertex is what makes them match. The face keeps its shape; only its
/// boundary gains a vertex.
fn imprint_vertices_on_edges(
    faces: Vec<Face>,
    extra_points: &[Point3],
    tol: &Tolerance,
) -> Vec<Face> {
    let mut points: Vec<Point3> = Vec::new();
    let add_point = |point: Point3, points: &mut Vec<Point3>| {
        if !points
            .iter()
            .any(|existing| points_same_3d(*existing, point, tol.linear))
        {
            points.push(point);
        }
    };

    for point in extra_points {
        add_point(*point, &mut points);
    }
    for face in &faces {
        for wire in std::iter::once(&face.outer_wire).chain(face.inner_wires.iter()) {
            for oriented in &wire.edges {
                add_point(oriented.edge.start_vertex.point, &mut points);
                add_point(oriented.edge.end_vertex.point, &mut points);
            }
        }
    }

    let rewrite_wire = |wire: &Wire| -> Wire {
        let mut edges = Vec::with_capacity(wire.edges.len());
        for oriented in &wire.edges {
            match split_edge_at_interior_points(&oriented.edge, &points, tol) {
                Some(pieces) => {
                    let ordered: Vec<Edge> = if oriented.orientation.is_forward() {
                        pieces
                    } else {
                        pieces.into_iter().rev().collect()
                    };
                    for piece in ordered {
                        edges.push(OrientedEdge::new(piece, oriented.orientation));
                    }
                }
                None => edges.push(oriented.clone()),
            }
        }
        Wire::new(edges)
    };

    faces
        .iter()
        .map(|face| {
            Face::new(
                face.geometry.clone(),
                rewrite_wire(&face.outer_wire),
                face.inner_wires.iter().map(rewrite_wire).collect(),
                face.orientation,
                face.tolerance,
            )
        })
        .collect()
}

/// Splits one edge at every supplied point lying strictly inside it.
fn split_edge_at_interior_points(
    edge: &Edge,
    points: &[Point3],
    tol: &Tolerance,
) -> Option<Vec<Edge>> {
    let start = edge.start_vertex.point;
    let end = edge.end_vertex.point;

    let mut interior: Vec<(f64, Point3)> = Vec::new();
    for point in points {
        if points_same_3d(*point, start, tol.linear) || points_same_3d(*point, end, tol.linear) {
            continue;
        }
        let Some(parameter) = curve_parameter_of_point(&edge.curve, *point, tol) else {
            continue;
        };
        if interior
            .iter()
            .any(|(existing, _)| (existing - parameter).abs() <= 1e-9)
        {
            continue;
        }
        interior.push((parameter, *point));
    }

    if interior.is_empty() {
        return None;
    }
    interior.sort_by(|a, b| a.0.total_cmp(&b.0));

    let mut pieces: Vec<Edge> = Vec::new();
    let mut remaining_curve = edge.curve.clone();
    let mut remaining_start = edge.start_vertex.clone();

    for (_, point) in &interior {
        let parameter = curve_parameter_of_point(&remaining_curve, *point, tol)?;
        let (left, right) = remaining_curve.split_bezier_at(parameter)?;
        let cut_vertex = Vertex::new(*point, tol.linear);
        pieces.push(Edge::new(
            left,
            remaining_start.clone(),
            cut_vertex.clone(),
            tol.linear,
        ));
        remaining_curve = right;
        remaining_start = cut_vertex;
    }

    pieces.push(Edge::new(
        remaining_curve,
        remaining_start,
        edge.end_vertex.clone(),
        tol.linear,
    ));

    Some(pieces)
}

/// The parameter at which a curve passes through a point, or `None` when it
/// does not pass through it within tolerance or only touches an end.
fn curve_parameter_of_point(
    curve: &zenith_geom::NurbsCurve3,
    point: Point3,
    tol: &Tolerance,
) -> Option<f64> {
    let (t_min, t_max) = curve.param_range();
    if t_max - t_min <= f64::EPSILON {
        return None;
    }

    const COARSE_SAMPLES: usize = 64;
    let mut best_t = t_min;
    let mut best_distance = f64::INFINITY;
    for index in 0..=COARSE_SAMPLES {
        let t = t_min + (t_max - t_min) * (index as f64 / COARSE_SAMPLES as f64);
        let distance = (curve.evaluate(t) - point).norm();
        if distance < best_distance {
            best_distance = distance;
            best_t = t;
        }
    }

    let mut low = (best_t - (t_max - t_min) / COARSE_SAMPLES as f64).max(t_min);
    let mut high = (best_t + (t_max - t_min) / COARSE_SAMPLES as f64).min(t_max);
    for _ in 0..64 {
        let mid_low = low + (high - low) / 3.0;
        let mid_high = high - (high - low) / 3.0;
        if (curve.evaluate(mid_low) - point).norm() <= (curve.evaluate(mid_high) - point).norm() {
            high = mid_high;
        } else {
            low = mid_low;
        }
    }
    let t = 0.5 * (low + high);
    if (curve.evaluate(t) - point).norm() > tol.linear * 10.0 {
        return None;
    }

    let span = t_max - t_min;
    if t <= t_min + span * 1e-9 || t >= t_max - span * 1e-9 {
        return None;
    }

    Some(t)
}

/// Removes the duplication that arises when both operands contribute the same
/// patch of surface.
///
/// Where two solids share part of a plane, splitting produces the same region
/// on both sides and the selection keeps both, so the shared region's edges end
/// up used four times instead of twice. Which copy survives depends on how the
/// two faces face:
///
/// - pointing the same way, the two describe one piece of the result's
///   boundary, so one copy is kept
/// - pointing opposite ways, the region is interior to the result and neither
///   copy belongs to its boundary
fn resolve_coincident_face_pieces(pieces: &mut Vec<SelectedBooleanFacePiece>, tol: &Tolerance) {
    let mut drop_flags = vec![false; pieces.len()];

    for left in 0..pieces.len() {
        if drop_flags[left] {
            continue;
        }
        for right in (left + 1)..pieces.len() {
            if drop_flags[right] || pieces[left].operand == pieces[right].operand {
                continue;
            }
            let Some(same_direction) =
                coincident_face_direction(&pieces[left], &pieces[right], tol)
            else {
                continue;
            };

            if same_direction {
                // 同じ向きなら、結果の境界に現れるのは1枚だけ。
                drop_flags[right] = true;
            } else {
                // 逆向きなら、その領域は結果の内部に呑まれる。
                drop_flags[left] = true;
                drop_flags[right] = true;
            }
            break;
        }
    }

    let mut index = 0;
    pieces.retain(|_| {
        let keep = !drop_flags[index];
        index += 1;
        keep
    });
}

/// `Some(true)` when the two pieces occupy the same patch of plane facing the
/// same way, `Some(false)` when they face opposite ways, `None` when they are
/// not the same patch at all.
fn coincident_face_direction(
    left: &SelectedBooleanFacePiece,
    right: &SelectedBooleanFacePiece,
    tol: &Tolerance,
) -> Option<bool> {
    let (FaceGeometry::Plane(left_plane), FaceGeometry::Plane(right_plane)) =
        (&left.face.geometry, &right.face.geometry)
    else {
        return None;
    };

    let left_normal = selected_piece_normal(left, left_plane)?;
    let right_normal = selected_piece_normal(right, right_plane)?;

    // 同一平面か。法線が平行で、原点間の距離が面内に収まっていること。
    if left_normal.cross(&right_normal).norm() > 1e-9 {
        return None;
    }
    if (right_plane.origin - left_plane.origin)
        .dot(&left_normal)
        .abs()
        > tol.linear * 10.0
    {
        return None;
    }

    // 同じ領域か。境界サンプルの重心と広がりで判定する。分割後は重なる領域が
    // そのまま一枚の面になっているので、これで十分に区別できる。
    let left_points = left.face.outer_wire.sample_points(16);
    let right_points = right.face.outer_wire.sample_points(16);
    if left_points.is_empty() || right_points.is_empty() {
        return None;
    }

    let centroid = |points: &[Point3]| {
        let mut sum = Vec3::zeros();
        for point in points {
            sum += point.coords;
        }
        Point3::from(sum / points.len() as f64)
    };
    let extent = |points: &[Point3], centre: Point3| {
        points
            .iter()
            .map(|point| (*point - centre).norm())
            .fold(0.0f64, f64::max)
    };

    let left_centre = centroid(&left_points);
    let right_centre = centroid(&right_points);
    let left_extent = extent(&left_points, left_centre);

    let scale = left_extent.max(1.0);
    if (right_centre - left_centre).norm() > scale * 1e-6 {
        return None;
    }
    if (extent(&right_points, right_centre) - left_extent).abs() > scale * 1e-6 {
        return None;
    }

    Some(left_normal.dot(&right_normal) > 0.0)
}

/// The outward normal a selected piece will have in the assembled result,
/// after both the face's own orientation flag and the boolean's reversal.
fn selected_piece_normal(
    piece: &SelectedBooleanFacePiece,
    plane: &zenith_geom::PlaneSurface3,
) -> Option<Vec3> {
    let mut normal = plane.normal.try_normalize(1e-12)?;
    if !piece.face.orientation.is_forward() {
        normal = -normal;
    }
    if piece.reverse_orientation {
        normal = -normal;
    }
    Some(normal)
}
