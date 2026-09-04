use crate::cap::CapBuilder;
use crate::MassCalculator;
use std::collections::BTreeMap;
use zenith_geom::{
    ControlPoint3, ExtremumEngine, KnotVector, NurbsCurve3, NurbsSurface3, PlaneSurface3, Surface3,
};
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
    /// 交わりが1本の曲線では足りない場合。
    ///
    /// 平面がトーラスを軸と平行に切ると、交わりは2本の閉曲線になる。1本しか
    /// 返さないと残りは無かったことになり、面はそこで割れない。
    Curves {
        edges: Vec<Edge>,
    },
    Coincident,
    Unsupported,
}

#[derive(Debug, Clone, PartialEq)]
pub struct FaceIntersectionCandidate {
    pub face_a_index: usize,
    pub face_b_index: usize,
    pub kind: FaceIntersectionKind,
    /// この交線が**解析的に**出たものか。辿って出したものは `false`。
    ///
    /// # なぜ旗が要るのか
    ///
    /// この文書は「旗を立てず、測る」を通してきました。ここは例外です
    /// ——**測っても分からないと測った上での**判断です（4-181）。
    ///
    /// 接する所の継ぎ目では、辿って出した端も解析的に出した端も、
    /// **面から `1e-14` の内側**にあります。2e-4 の位置ずれが残差に
    /// まったく現れません（4-180 の縮退）。だから「どちらが厳密か」は
    /// 出来上がったものを測っても言えません。**作られ方を覚えておく
    /// しかありません。**
    pub analytic: bool,
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
    /// 組み立てに使った面の組の候補の数。
    ///
    /// 数え上げのためだけに走査をやり直さなくて済むよう、ここに残す。
    pub face_pair_candidate_count: usize,
    /// 組み立てに使った交線の候補そのもの。同上。
    pub edge_candidates: Vec<IntersectionEdgeCandidate>,
    pub selection: BooleanFaceSelection,
    pub cap_generation: PlanarCapGeneration,
    pub assembly: BooleanFaceAssembly,
    /// **接しているだけとして落とした交線**（4-189）。
    ///
    /// 空でないなら、その配置は**接触を含みます**。接触は、演算によっては
    /// 答えを非多様体にします（差では材料の厚みが 0 になる）。落とした線は
    /// 面を割らないので縫合は通ってしまい、**そのまま返すと誤答になります**。
    ///
    /// **本数ではなく線そのものを残します。** 材料を数えるには、数えたい
    /// 場所——つまり接している線——が要ります。候補から外してしまうと、
    /// [`crate::contact::find_result_pinch`] は測る場所を失います
    /// （2026/08/30 に一度そうしました）。
    pub dropped_contact_curves: Vec<Edge>,
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
                if let Some((kind, analytic)) = intersect_face_supports(face_a, face_b, tol)
                    .and_then(|(kind, analytic)| {
                        clip_candidate_to_face_bboxes(
                            kind,
                            bboxes_a[face_a_index].as_ref(),
                            bboxes_b[face_b_index].as_ref(),
                            tol,
                        )
                        .map(|kind| (kind, analytic))
                    })
                    .and_then(|(kind, analytic)| {
                        clip_candidate_to_planar_trims(kind, face_a, face_b, tol)
                            .map(|kind| (kind, analytic))
                    })
                {
                    candidates.push(FaceIntersectionCandidate {
                        face_a_index,
                        face_b_index,
                        kind,
                        analytic,
                    });
                }
            }
        }

        // **相手のいない端から、隣の組を辿り直します。** 組を独立に辿ると、
        // 隣の升に入るぶんの弧が短いときに抜けます（4-62）。
        Self::trace_from_loose_ends_into(
            faces_a,
            faces_b,
            &bboxes_a,
            &bboxes_b,
            &mut candidates,
            tol,
        );

        // **接する所の継ぎ目を、解析的な交線から作り直します**（4-182）。
        Self::rebuild_tangent_joints(faces_a, faces_b, &mut candidates, tol);

        candidates
    }

    /// 辿って出した弧の端を、**隣の解析的な交線と、自分のもう一方の面との
    /// 交わり**として作り直す。
    ///
    /// # なぜ「作る」のか
    ///
    /// 接する所の継ぎ目は、**そこにある点から選べません**（4-181）。集まった
    /// 端はどれも `√ε` ずれていて、**候補の中に正解がありません**。面への
    /// 距離で選ぼうとすると全部が `1e-14` に見え（4-180 の縮退）、隣の交線
    /// への距離で選んでも最良が `1.086e-4` で 0 になりません。
    ///
    /// 抜け道は1つだけです——**縮退した2面の交わりを先に解析的に持ち、
    /// それと3枚目の面との交わりとして解く**。
    ///
    /// ```text
    /// 接する円（トーラス × 上面、厳密）  ∩  壁の平面
    ///   → 1未知数1式、横断的 → 倍精度いっぱい
    /// ```
    ///
    /// 実測（4-179）の狙い値は `10 + √44 = 16.633249581` です。
    fn rebuild_tangent_joints(
        faces_a: &[Face],
        faces_b: &[Face],
        candidates: &mut Vec<FaceIntersectionCandidate>,
        tol: &Tolerance,
    ) {
        let window = tol.linear * 1000.0;
        if !(window > 0.0) {
            return;
        }
        let explain = std::env::var_os("ZENITH_JOINT_WHY").is_some();

        // 解析的に出た交線だけを控える。
        let exact: Vec<(usize, usize, usize, NurbsCurve3)> = candidates
            .iter()
            .enumerate()
            .filter(|(_, candidate)| candidate.analytic)
            .flat_map(|(index, candidate)| {
                candidate_edges(&candidate.kind)
                    .into_iter()
                    .map(|edge| {
                        (
                            index,
                            candidate.face_a_index,
                            candidate.face_b_index,
                            edge.curve.clone(),
                        )
                    })
                    .collect::<Vec<_>>()
            })
            .collect();
        if exact.is_empty() {
            return;
        }

        // 動かす先を先に決めてから、まとめて動かす（借用のため）。
        let mut moves: Vec<(usize, usize, bool, Point3)> = Vec::new();
        for (index, candidate) in candidates.iter().enumerate() {
            if candidate.analytic {
                continue;
            }
            for (slot, edge) in candidate_edges(&candidate.kind).iter().enumerate() {
                for is_start in [true, false] {
                    let point = if is_start {
                        edge.start_vertex.point
                    } else {
                        edge.end_vertex.point
                    };
                    for (_, exact_a, exact_b, curve) in &exact {
                        // 面をちょうど1枚だけ共有していること。
                        let shares_a = *exact_a == candidate.face_a_index;
                        let shares_b = *exact_b == candidate.face_b_index;
                        if shares_a == shares_b {
                            continue;
                        }
                        // 共有していないほうが「3枚目の面」。
                        let third = if shares_a {
                            faces_b.get(candidate.face_b_index)
                        } else {
                            faces_a.get(candidate.face_a_index)
                        };
                        let Some(third) = third else {
                            continue;
                        };
                        let FaceGeometry::Plane(plane) = &third.geometry else {
                            // 平面以外はまだ扱いません。**測っていない**ので。
                            continue;
                        };
                        let Ok(nearby) = ExtremumEngine::point_to_curve(point, curve, 64, 1e-13)
                        else {
                            continue;
                        };
                        if nearby.distance > window {
                            continue;
                        }
                        let normal = oriented_plane_normal(third);
                        let Some(t) =
                            solve_curve_on_plane(curve, nearby.parameter, plane.origin, normal)
                        else {
                            continue;
                        };
                        let landed = curve.evaluate(t);
                        let moved = (landed - point).norm();
                        if moved <= tol.linear || moved > window {
                            continue;
                        }
                        if explain {
                            eprintln!(
                                "JOINTWHY tsukutta ({:.9} {:.9} {:.9}) -> ({:.9} {:.9} {:.9}) ugoki {moved:.3e}",
                                point.x, point.y, point.z, landed.x, landed.y, landed.z
                            );
                        }
                        moves.push((index, slot, is_start, landed));
                        break;
                    }
                }
            }
        }

        for (index, slot, is_start, target) in moves {
            move_candidate_edge_end(&mut candidates[index], slot, is_start, target, tol);
        }
    }

    fn trace_from_loose_ends_into(
        faces_a: &[Face],
        faces_b: &[Face],
        bboxes_a: &[Option<BoundingBox3>],
        bboxes_b: &[Option<BoundingBox3>],
        candidates: &mut Vec<FaceIntersectionCandidate>,
        tol: &Tolerance,
    ) {
        trace_from_loose_ends(faces_a, faces_b, bboxes_a, bboxes_b, candidates, tol);
    }

    /// 面の組が1つでも交わり得るか、**交線を求めずに**答える。
    ///
    /// `collect_face_pair_candidates` は組ごとに交線を辿るので、「あるか
    /// ないか」を訊くだけのために呼ぶと高くつく。1回のブーリアンで交線の
    /// 走査は3回走っており、そのうち1回はこの問いだけだった。
    ///
    /// answering `true` は「交わっているかもしれない」であって「交わって
    /// いる」ではない。呼び手はその先で本当の交線を求める。
    pub fn any_face_pair_may_intersect(
        faces_a: &[Face],
        faces_b: &[Face],
        tol: &Tolerance,
    ) -> bool {
        let bboxes_a: Vec<Option<BoundingBox3>> = faces_a.iter().map(face_boundary_bbox).collect();
        let bboxes_b: Vec<Option<BoundingBox3>> = faces_b.iter().map(face_boundary_bbox).collect();

        for (index_a, face_a) in faces_a.iter().enumerate() {
            for (index_b, face_b) in faces_b.iter().enumerate() {
                if !face_bboxes_intersect(
                    bboxes_a[index_a].as_ref(),
                    bboxes_b[index_b].as_ref(),
                    tol,
                ) {
                    continue;
                }
                // 幾何の組み合わせとして扱えるかだけを見る。交線は求めない。
                let supported = matches!(
                    (&face_a.geometry, &face_b.geometry),
                    (FaceGeometry::Plane(_), FaceGeometry::Plane(_))
                        | (FaceGeometry::Plane(_), FaceGeometry::Nurbs(_))
                        | (FaceGeometry::Nurbs(_), FaceGeometry::Plane(_))
                        | (FaceGeometry::Nurbs(_), FaceGeometry::Nurbs(_))
                );
                if supported {
                    return true;
                }
            }
        }

        false
    }

    pub fn collect_intersection_edge_candidates(
        faces_a: &[Face],
        faces_b: &[Face],
        tol: &Tolerance,
    ) -> Vec<IntersectionEdgeCandidate> {
        Self::intersection_edge_candidates_from_face_pairs(
            Self::collect_face_pair_candidates(faces_a, faces_b, tol),
            tol,
        )
    }

    /// 既に求めてある面の組の交わりから、辺の候補を組む。
    ///
    /// 面の組を探す段は組ごとにマーチングを走らせるので、同じ問いを二度
    /// 走らせないために、求めた候補を渡せる形に分けてある。
    pub fn intersection_edge_candidates_from_face_pairs(
        candidates: Vec<FaceIntersectionCandidate>,
        tol: &Tolerance,
    ) -> Vec<IntersectionEdgeCandidate> {
        candidates
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
                    FaceIntersectionKind::Curves { edges } => {
                        return Some(
                            edges
                                .into_iter()
                                .map(|edge| IntersectionEdgeCandidate {
                                    face_a_index: candidate.face_a_index,
                                    face_b_index: candidate.face_b_index,
                                    edge,
                                })
                                .collect::<Vec<_>>(),
                        )
                    }
                    _ => return None,
                };

                Some(vec![IntersectionEdgeCandidate {
                    face_a_index: candidate.face_a_index,
                    face_b_index: candidate.face_b_index,
                    edge,
                }])
            })
            .flatten()
            // **1点に潰れた交線は、接触の記録です。位相を作らせません**
            // （3-1 の規約。2026/08/25、4-80）。
            //
            // 球が箱の面に極で触れると、そこに長さ 0 の「交線」が立ちます
            // （実測: `box × sphere` で6本、すべて極の1点）。切り込みとして
            // 渡すと、箱の面に**行って戻るだけの切れ目**が入り、その面の
            // メッシュに穴が開きます（実測: union の結果に、対になっていない
            // 稜が2本と長さ 0 の稜が1本）。
            //
            // 点で触れているだけの場所は、割る理由になりません。
            .filter(|candidate| sampled_edge_extent(&candidate.edge) > tol.linear)
            .collect()
    }

    pub fn collect_planar_face_split_candidates(
        faces_a: &[Face],
        faces_b: &[Face],
        tol: &Tolerance,
    ) -> Vec<PlanarFaceSplitCandidate> {
        Self::planar_face_split_candidates_from_edge_candidates(
            faces_a,
            faces_b,
            Self::collect_intersection_edge_candidates(faces_a, faces_b, tol),
            tol,
        )
    }

    /// 立体の面を、**組み立てが使うのと同じ並び**で返す（内側シェル込み）。
    ///
    /// 交線の候補が持つ添字はこの並びを指します。外側シェルだけを渡すと、
    /// 空洞のある立体で範囲外になります（4-141）。
    pub fn all_faces_of(solid: &Solid) -> Vec<Face> {
        all_solid_faces(solid)
    }

    /// 既に求めてある交線の候補から、面の分割候補を組む。
    pub fn planar_face_split_candidates_from_edge_candidates(
        faces_a: &[Face],
        faces_b: &[Face],
        candidates: Vec<IntersectionEdgeCandidate>,
        tol: &Tolerance,
    ) -> Vec<PlanarFaceSplitCandidate> {
        candidates
            .into_iter()
            .filter_map(|candidate| {
                // **添字で落ちません。** 呼び側が組み立てと違う並びを渡す
                // ことがあります（4-141 で実際に落ちました）。合わない
                // 候補は捨てて先へ進みます——**カーネルがパニックするのは、
                // 誤答より悪い**からです。
                let face_a = faces_a.get(candidate.face_a_index)?;
                let face_b = faces_b.get(candidate.face_b_index)?;
                let split_faces_a = Self::split_face_by_edge(face_a, &candidate.edge, tol).ok()?;
                let split_faces_b = Self::split_face_by_edge(face_b, &candidate.edge, tol).ok()?;

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
        Self::batch_splits_from_candidates(faces_a, faces_b, edge_candidates, tol)
    }

    /// 既に求めてある交線の候補から面の分割を組む。
    ///
    /// 交線の走査は面の組ごとにマーチングを走らせるので、1回のブーリアンで
    /// 何度も呼ぶと効いてくる。求めた候補は使い回す。
    pub fn batch_splits_from_candidates(
        faces_a: &[Face],
        faces_b: &[Face],
        edge_candidates: Vec<IntersectionEdgeCandidate>,
        tol: &Tolerance,
    ) -> PlanarOperandBatchSplits {
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
        let splits = Self::collect_planar_face_split_candidates(
            &solid_a.outer_shell.faces,
            &solid_b.outer_shell.faces,
            tol,
        );
        Self::classified_planar_face_split_candidates_from_splits(solid_a, solid_b, splits, tol)
    }

    /// 既に求めてある面の分割候補を、相手の立体に対して内外で色分けする。
    pub fn classified_planar_face_split_candidates_from_splits(
        solid_a: &Solid,
        solid_b: &Solid,
        splits: Vec<PlanarFaceSplitCandidate>,
        tol: &Tolerance,
    ) -> Vec<ClassifiedPlanarFaceSplitCandidate> {
        let mesh_a = tessellate_solid(solid_a, &TessellationParams::default());
        let mesh_b = tessellate_solid(solid_b, &TessellationParams::default());

        splits
            .into_iter()
            .map(|candidate| {
                let split_faces_a = candidate
                    .split_faces_a
                    .into_iter()
                    .map(|face| ClassifiedFacePiece {
                        location: classify_face_against_mesh(&face, &mesh_b, Some(solid_b), tol),
                        face,
                    })
                    .collect();
                let split_faces_b = candidate
                    .split_faces_b
                    .into_iter()
                    .map(|face| ClassifiedFacePiece {
                        location: classify_face_against_mesh(&face, &mesh_a, Some(solid_a), tol),
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
        classify_face_against_mesh(face, &mesh, Some(solid), tol)
    }

    /// [`Self::classify_face_against_solid`] と同じ判定を、**メッシュを
    /// 使い回して**行う。
    ///
    /// あちらは呼ぶたびに相手を丸ごとテッセレーションします。面ごとに
    /// 呼ぶと面の枚数だけ張り直すので、16面のトーラスなら16回です。
    /// 立体まるごとを判定するところでは、こちらで1回に済ませます。
    pub fn classify_face_against_solid_mesh(
        face: &Face,
        solid: &Solid,
        mesh: &TriangleMesh,
        tol: &Tolerance,
    ) -> FaceRegionLocation {
        classify_face_against_mesh(face, mesh, Some(solid), tol)
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
        let candidates = Self::collect_intersection_edge_candidates(
            &solid_a.outer_shell.faces,
            &solid_b.outer_shell.faces,
            tol,
        );
        Self::selected_face_pieces_from_candidates(solid_a, solid_b, candidates, op, tol)
    }

    /// 既に求めてある交線の候補から選別まで進める。
    pub fn selected_face_pieces_from_candidates(
        solid_a: &Solid,
        solid_b: &Solid,
        candidates: Vec<IntersectionEdgeCandidate>,
        op: crate::BooleanOpType,
        tol: &Tolerance,
    ) -> BooleanFaceSelection {
        let faces_a = all_solid_faces(solid_a);
        let faces_b = all_solid_faces(solid_b);
        let batch_splits =
            Self::batch_splits_from_candidates(&faces_a, &faces_b, candidates.clone(), tol);
        let mesh_a = tessellate_solid(solid_a, &TessellationParams::default());
        let mesh_b = tessellate_solid(solid_b, &TessellationParams::default());

        let mut selected_face_pieces = Vec::new();
        let inner_a = face_comes_from_inner_shell(solid_a);
        let inner_b = face_comes_from_inner_shell(solid_b);
        selected_face_pieces.extend(select_operand_faces_after_batch_split(
            &faces_a,
            &inner_a,
            &batch_splits.splits_a,
            BooleanOperand::A,
            &mesh_b,
            Some(solid_b),
            op,
            tol,
        ));
        selected_face_pieces.extend(select_operand_faces_after_batch_split(
            &faces_b,
            &inner_b,
            &batch_splits.splits_b,
            BooleanOperand::B,
            &mesh_a,
            Some(solid_a),
            op,
            tol,
        ));

        // **面積を囲まない面片は、面ではありません。**
        //
        // 相手の立体の稜が、こちらの面の**境界の上**に乗る配置——45度回した
        // 箱どうしがそれ——では、面をその稜で割ると「面そのもの」と「行って
        // 戻るだけの切れ目」の2枚が出ます。切れ目のほうは面積 0 で、境界を
        // 1本も新しく持ちませんが、**同じ稜をもう2回使います**。それが
        // 縫合の非多様体として出ていました（HANDOVER 3-N-1。実測では
        // `box × box` 45度回転の差で 12、積で 22）。
        //
        // 落としても形は動きません。面積 0 の面は立体の境界に何も足しません。
        selected_face_pieces.retain(|piece| !face_encloses_no_area(&piece.face, tol));

        // **長さ 0 の稜は、稜ではありません。**
        //
        // 交線が接点で終わる配置では、同じ接点で終わる弧が複数あります
        // （半径の等しい直交2円柱では、2本の楕円がそこで交わります）。弧の
        // 端が接点にきちんと着地するようになると（4-128）、割った輪の中に
        // **行って戻る長さ 0 の稜**が残ります。実測: 和で 1本が1回だけ、
        // 1本が3回使われ、縫合の非多様体として出ていました。
        //
        // 落としても輪は繋がったままです——両端が同じ点なので、前後の稜は
        // もともと繋がっています。面の形も動きません。
        for piece in &mut selected_face_pieces {
            remove_degenerate_wire_edges(&mut piece.face, tol);
        }
        selected_face_pieces.retain(|piece| piece.face.outer_wire.edges.len() >= 2);

        // 同じ平面に重なって乗る面は、両オペランドから同じ領域が採られる。
        // そのまま縫うと同じ稜を4回使うことになるので、ここで解消する。
        resolve_coincident_face_pieces(&mut selected_face_pieces, tol);

        // 隣り合う面の片方だけが辺の途中で切られていると、辺の長さが食い違って
        // 縫合が合わない。相手が持つ頂点を境界辺へ刻み込んで対応させる。
        // 面の形は変わらず、境界に頂点が増えるだけ。
        let mut imprint_points = Vec::new();
        for candidate in &candidates {
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

        // **輪の中の「行って戻るだけ」を畳みます**（4-205）。面積を囲む片の
        // 中に切れ目が1本混ざっていると、4-74 の判定（片ごと丸ごと面積 0 か）
        // には掛かりません。実測: `cone × torus` の和で、同じ稜を2回使う片が
        // 2枚残り、縫合が非多様体（8）になっていました。
        //
        // **刻み込みの後に置きます。** 前に置くと、切れ目が刻み込みで
        // 2本に割れたあとの形（実測では7辺の輪）を見られません。
        for piece in &mut selected_face_pieces {
            collapse_there_and_back(&mut piece.face, tol);
        }
        selected_face_pieces.retain(|piece| piece.face.outer_wire.edges.len() >= 2);

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

            let base = stitch_report_score(&diagnose_selected_face_stitching(
                &selected_face_pieces,
                tol,
            ));
            let (best_score, best_piece) = if reversed_score < forward_score {
                (reversed_score, reversed_piece)
            } else {
                (forward_score, forward_piece)
            };
            // **蓋は塞ぐことはあっても壊してはいけない。**
            //
            // ここは以前 `(未整合, 非多様体, 同方向)` の辞書順で比べていた。
            // 未整合が先頭なので、**非多様体を6本作ってでも未整合を2本減らす**
            // 選択が「改善」と判定される。実測: 押し出したスプラインをスラブで
            // 切ると (10, 0, 0) から (8, 6, 0) になり、それが採用されていた。
            // B 側の切断面が既に選ばれているところへ蓋を重ねた形である。
            //
            // 未整合は「まだ閉じていない」、非多様体は「壊れている」。
            // 減らしてよいのは前者だけで、後者を増やす取引は無い。
            let closes_without_breaking =
                best_score.0 < base.0 && best_score.1 <= base.1 && best_score.2 <= base.2;
            if closes_without_breaking {
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
        let faces_a = all_solid_faces(solid_a);
        let faces_b = all_solid_faces(solid_b);
        let mut face_pair_candidates = Self::collect_face_pair_candidates(&faces_a, &faces_b, tol);
        // **接しているだけの線では、面を割りません**（4-184。規約 3-1）。
        let dropped_contact_curves = drop_non_bounding_contact_curves(
            solid_a,
            solid_b,
            &faces_a,
            &faces_b,
            &mut face_pair_candidates,
            tol,
        );
        let face_pair_candidate_count = face_pair_candidates.len();
        let mut edge_candidates =
            Self::intersection_edge_candidates_from_face_pairs(face_pair_candidates, tol);
        edge_candidates.extend(collect_edges_already_on_a_plane(&faces_a, &faces_b, tol));
        edge_candidates.extend(
            collect_edges_already_on_a_plane(&faces_b, &faces_a, tol)
                .into_iter()
                .map(|candidate| IntersectionEdgeCandidate {
                    // 役割が入れ替わっているので、添字を戻す。
                    face_a_index: candidate.face_b_index,
                    face_b_index: candidate.face_a_index,
                    edge: candidate.edge,
                }),
        );
        let selection = Self::selected_face_pieces_from_candidates(
            solid_a,
            solid_b,
            edge_candidates.clone(),
            op,
            tol,
        );
        let cap_generation = Self::build_planar_caps_grouped_by_planar_face(
            &edge_candidates,
            &faces_a,
            &faces_b,
            tol,
        );
        let assembly = Self::assemble_selected_face_pieces_with_caps(
            &selection.selected_face_pieces,
            &cap_generation.cap_faces,
            tol,
        );

        BooleanShellAssembly {
            face_pair_candidate_count,
            edge_candidates,
            selection,
            cap_generation,
            assembly,
            dropped_contact_curves,
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

    /// 選んだ面から立体を組む。**離れた塊は別々の立体にします。**
    ///
    /// 板をスロットで分断すると、答えは2つの塊です。以前はそれを1枚のシェルに
    /// まとめて**1つの `Solid`** として返していました。
    ///
    /// **どの検査にも掛かりません。** 体積は発散定理が両方を足すので正しく
    /// 出ますし（実測 10464.5286、閉じた式どおり）、各塊が閉じているので
    /// シェルは「閉じている」と判定され、384点の内外判定も通ります。
    /// **位相だけが違い、位相を見る検査がありませんでした。**
    pub fn build_solids_from_selected_face_pieces(
        pieces: &[SelectedBooleanFacePiece],
        tol: &Tolerance,
    ) -> Result<Vec<Solid>, String> {
        let groups = connected_piece_groups(pieces, tol);
        if groups.len() <= 1 {
            return Self::build_solid_from_selected_face_pieces(pieces, tol)
                .map(|solid| vec![solid]);
        }

        let mut solids = Vec::with_capacity(groups.len());
        for group in groups {
            let subset: Vec<SelectedBooleanFacePiece> = group
                .into_iter()
                .map(|index| pieces[index].clone())
                .collect();
            solids.push(Self::build_solid_from_selected_face_pieces(&subset, tol)?);
        }
        Ok(nest_cavity_shells_into_solids(solids, tol))
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

        let mut wires: Vec<Wire> = Vec::with_capacity(edge_loop_extraction.loops.len());
        let mut failed_loop_count = 0;
        let trace = std::env::var_os("ZENITH_CAP_TRACE").is_some();
        for edge_loop in &edge_loop_extraction.loops {
            match order_edges_into_closed_wire(&edge_loop.edges, tol) {
                Ok(wire) => wires.push(wire),
                // **輪は見つかったのに閉じたワイヤにならない**、という段が
                // あります。「蓋 0 枚」だけでは、輪が無いのか、輪はあるのに
                // 並べられないのかが分かりません（4-220）。
                Err(reason) => {
                    if trace {
                        eprintln!(
                            "      輪 {} 本を閉じたワイヤにできない: {}",
                            edge_loop.edges.len(),
                            reason.chars().take(150).collect::<String>()
                        );
                    }
                    failed_loop_count += 1
                }
            }
        }

        let (cap_faces, nesting_failures) = build_caps_from_nested_loops(wires, tol);
        failed_loop_count += nesting_failures;

        PlanarCapGeneration {
            edge_loop_extraction,
            cap_faces,
            failed_loop_count,
        }
    }

    /// 蓋の材料を、対の**平面側の面**ごとに束ねる。
    ///
    /// [`build_planar_caps_from_intersection_edge_candidates`] は常に
    /// `face_b` で束ねます。**B が平面ならそれで合いますが、B が曲面だと
    /// 合いません。**
    ///
    /// 実測（4-121、箱を 19 度・円柱を 27 度回した和）: 切り口の楕円は
    /// 円柱の**4枚のパッチにまたがります**。`face_b` で束ねると1枚あたり
    /// 2本ずつに割れ、どの束も輪になりません。
    ///
    /// ```text
    /// cap group face_b 0: 2 edge(s) in -> 0 loop(s), 2 skipped, 0 cap face(s)
    /// cap group face_b 1: 2 edge(s) in -> 0 loop(s), 2 skipped, 0 cap face(s)
    /// cap group face_b 2: 2 edge(s) in -> 0 loop(s), 2 skipped, 0 cap face(s)
    /// cap group face_b 3: 2 edge(s) in -> 0 loop(s), 2 skipped, 0 cap face(s)
    /// ```
    ///
    /// 蓋が乗る平面は、対のうち**平面のほう**が決めます。どちらが A で
    /// どちらが B かは関係ありません。両方が平面のときは `face_b` を採り、
    /// 従来と同じ束ね方になります（既に通っている配置を動かさないため）。
    /// 両方が曲面のときも `face_b` のままです——その組では、この関数が
    /// 決められることはありません。
    pub fn build_planar_caps_grouped_by_planar_face(
        candidates: &[IntersectionEdgeCandidate],
        faces_a: &[Face],
        faces_b: &[Face],
        tol: &Tolerance,
    ) -> PlanarCapGeneration {
        let is_plane = |faces: &[Face], index: usize| {
            faces
                .get(index)
                .is_some_and(|face| matches!(face.geometry, FaceGeometry::Plane(_)))
        };

        let mut groups: BTreeMap<(u8, usize), Vec<Edge>> = BTreeMap::new();
        for candidate in candidates {
            let key = if is_plane(faces_b, candidate.face_b_index) {
                (1u8, candidate.face_b_index)
            } else if is_plane(faces_a, candidate.face_a_index) {
                (0u8, candidate.face_a_index)
            } else {
                (1u8, candidate.face_b_index)
            };
            groups.entry(key).or_default().push(candidate.edge.clone());
        }

        let mut all_loops = Vec::new();
        let mut all_cap_faces = Vec::new();
        let mut total_failed_loop_count = 0;
        let mut total_skipped_edge_count = 0;

        for ((side, index), edges) in groups {
            // **同じ弧が別々の実体で2本来ることがあります**（4-80）。そのまま
            // 輪を辿ると、2本を往復するだけの「輪」になり、囲む面積が 0 に
            // なって「平面と読めない」で落ちます（4-220 の実測）。分割の側は
            // 既に同じ重複除去を通しています。
            let edges = deduplicate_split_edges(&edges, tol);
            let cap_gen = Self::build_planar_caps_from_intersection_edges(&edges, tol);
            if std::env::var_os("ZENITH_CAP_TRACE").is_some() {
                if cap_gen.edge_loop_extraction.loops.is_empty() {
                    // **輪にならなかったときは、材料そのものを見せる。**
                    // 「0 loops」だけでは、端が繋がっていないのか、本数が
                    // 足りないのか、向きの問題なのかが分からない。
                    for edge in &edges {
                        let (a, b) = (edge.start_vertex.point, edge.end_vertex.point);
                        eprintln!(
                            "      稜 ({:.9} {:.9} {:.9}) -> ({:.9} {:.9} {:.9}) 長さ {:.9}",
                            a.x,
                            a.y,
                            a.z,
                            b.x,
                            b.y,
                            b.z,
                            (b - a).norm()
                        );
                    }
                }
                eprintln!(
                    "    cap group {} {index}: {} edge(s) in -> {} loop(s), {} skipped, {} cap face(s), {} failed loop(s)",
                    if side == 0 { "face_a" } else { "face_b" },
                    edges.len(),
                    cap_gen.edge_loop_extraction.loops.len(),
                    cap_gen.edge_loop_extraction.skipped_edge_count,
                    cap_gen.cap_faces.len(),
                    cap_gen.failed_loop_count
                );
            }
            total_skipped_edge_count += cap_gen.edge_loop_extraction.skipped_edge_count;
            all_loops.extend(cap_gen.edge_loop_extraction.loops);
            all_cap_faces.extend(cap_gen.cap_faces);
            total_failed_loop_count += cap_gen.failed_loop_count;
        }

        PlanarCapGeneration {
            edge_loop_extraction: IntersectionEdgeLoopExtraction {
                loops: all_loops,
                skipped_edge_count: total_skipped_edge_count,
            },
            cap_faces: all_cap_faces,
            failed_loop_count: total_failed_loop_count,
        }
    }

    pub fn build_planar_caps_from_intersection_edge_candidates(
        candidates: &[IntersectionEdgeCandidate],
        tol: &Tolerance,
    ) -> PlanarCapGeneration {
        let mut candidates_by_face_b: BTreeMap<usize, Vec<Edge>> = BTreeMap::new();
        for candidate in candidates {
            candidates_by_face_b
                .entry(candidate.face_b_index)
                .or_default()
                .push(candidate.edge.clone());
        }

        let mut all_loops = Vec::new();
        let mut all_cap_faces = Vec::new();
        let mut total_failed_loop_count = 0;
        let mut total_skipped_edge_count = 0;

        for (_face_b_index, edges) in candidates_by_face_b {
            let cap_gen = Self::build_planar_caps_from_intersection_edges(&edges, tol);
            if std::env::var_os("ZENITH_CAP_TRACE").is_some() {
                eprintln!(
                    "    cap group face_b {_face_b_index}: {} edge(s) in -> {} loop(s), {} skipped, {} cap face(s), {} failed loop(s)",
                    edges.len(),
                    cap_gen.edge_loop_extraction.loops.len(),
                    cap_gen.edge_loop_extraction.skipped_edge_count,
                    cap_gen.cap_faces.len(),
                    cap_gen.failed_loop_count
                );
            }
            total_skipped_edge_count += cap_gen.edge_loop_extraction.skipped_edge_count;
            all_loops.extend(cap_gen.edge_loop_extraction.loops);
            all_cap_faces.extend(cap_gen.cap_faces);
            total_failed_loop_count += cap_gen.failed_loop_count;
        }

        PlanarCapGeneration {
            edge_loop_extraction: IntersectionEdgeLoopExtraction {
                loops: all_loops,
                skipped_edge_count: total_skipped_edge_count,
            },
            cap_faces: all_cap_faces,
            failed_loop_count: total_failed_loop_count,
        }
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
        distribute_inner_wires(face, vec![face_a, face_b], plane, tol)
    }

    fn split_planar_face_by_single_edge(
        face: &Face,
        split_edge: &Edge,
        tol: &Tolerance,
    ) -> Result<Vec<Face>, String> {
        let FaceGeometry::Plane(plane) = &face.geometry else {
            return Err("Only planar faces can be split by an intersection edge".to_string());
        };
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
        if start_hit.edge_index == end_hit.edge_index && (start_hit.t - end_hit.t).abs() <= 1e-9 {
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
        distribute_inner_wires(face, vec![face_a, face_b], plane, tol)
    }

    pub fn split_face_by_edge(
        face: &Face,
        split_edge: &Edge,
        tol: &Tolerance,
    ) -> Result<Vec<Face>, String> {
        match &face.geometry {
            FaceGeometry::Plane(_) => Self::split_planar_face_by_edge(face, split_edge, tol),
            FaceGeometry::Nurbs(surface) => {
                split_cylinder_side_face_by_horizontal_edge(face, surface, split_edge, tol)
                    .or_else(|horizontal_error| {
                        split_cylinder_side_face_by_vertical_edge(face, surface, split_edge, tol)
                            .map_err(|vertical_error| {
                                format!("{horizontal_error}; {vertical_error}")
                            })
                    })
                    .or_else(|cylinder_errors| {
                        // 円柱・円錐でない面。断面が面の等パラメータ線になっていれば、
                        // 形が何であれ同じやり方で割れる。トーラスがこれ。
                        split_patch_face_by_section_edge(face, surface, split_edge, tol)
                            .map_err(|general_error| format!("{cylinder_errors}; {general_error}"))
                    })
                    .or_else(|iso_errors| {
                        // 等パラメータ線でない切り口。境界の巡回を割るだけの一般の
                        // 分割にかける。**面積の和が元に戻ることを測って**から採る。
                        // 閉じたワイヤになっただけでは、領域の取り違えは分からない。
                        let (pieces, report) = crate::FaceSplitter::split_by_curve(
                            face, split_edge, tol,
                        )
                        .map_err(|general_error| format!("{iso_errors}; {general_error}"))?;
                        if report.area_residual > 1e-6 {
                            return Err(format!(
                                "{iso_errors}; the general split lost {:.3e} of the face area",
                                report.area_residual
                            ));
                        }
                        Ok(pieces)
                    })
            }
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
        let mut faces = vec![face.clone()];
        let mut applied_split_count = 0;
        let mut skipped_split_count = 0;

        for split_edge in split_edges {
            let mut next_faces = Vec::new();
            let mut applied_this_edge = false;

            // 割れなかった理由は、これまで捨てていました。**飛ばされた交線は
            // 面を割らないまま残る**ので、反対側の割られた面片と同じ稜を
            // 重ねて使い、縫合が非多様体になります（`box × box` 45度回転の
            // difference / intersection がそれで、HANDOVER 3-2）。
            // `ZENITH_SPLIT_WHY=1` で理由が1行ずつ出ます。
            let explain = std::env::var_os("ZENITH_SPLIT_WHY").is_some();
            let mut reasons: Vec<String> = Vec::new();

            for current_face in faces {
                match Self::split_planar_face_by_edge(&current_face, split_edge, tol) {
                    Ok(split_faces) => {
                        applied_split_count += 1;
                        applied_this_edge = true;
                        next_faces.extend(split_faces);
                    }
                    Err(reason) => {
                        if explain {
                            reasons.push(reason);
                        }
                        next_faces.push(current_face);
                    }
                }
            }

            if !applied_this_edge {
                skipped_split_count += 1;
                if explain {
                    let start = split_edge.start_vertex.point;
                    let end = split_edge.end_vertex.point;
                    eprintln!(
                        "SPLITWHY edge ({:.4} {:.4} {:.4}) -> ({:.4} {:.4} {:.4}) split nothing",
                        start.x, start.y, start.z, end.x, end.y, end.z
                    );
                    for reason in &reasons {
                        eprintln!("  {}", reason.chars().take(140).collect::<String>());
                    }
                }
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
            // 刻印は内部のループ1本ぶんしか扱えない。平面がトーラスを切ると
            // 入れ子の2本になり、面は3つの領域に分かれる。
            if let Some(regions) = split_planar_face_by_interior_loops(face, split_edges, tol) {
                return Ok(PlanarFaceMultiSplitResult {
                    applied_split_count: regions.len().saturating_sub(1),
                    skipped_split_count: 0,
                    faces: regions,
                });
            }
        }

        let mut faces = vec![face.clone()];
        let mut applied_split_count = 0;
        let mut skipped_split_count: usize = 0;
        // 1本では当たらなかった稜。**あとで鎖にまとめて当て直します。**
        let mut leftover: Vec<Edge> = Vec::new();

        for split_edge in split_edges {
            let mut next_faces = Vec::new();
            let mut applied_this_edge = false;

            // 割れなかった理由は、これまで捨てていました。**飛ばされた交線は
            // 面を割らないまま残る**ので、反対側の割られた面片と同じ稜を重ねて
            // 使い、縫合が非多様体になります（`box × box` 45度回転の
            // difference / intersection がそれ。HANDOVER 3-2）。
            // `ZENITH_SPLIT_WHY=1` で理由が1行ずつ出ます。
            let explain = std::env::var_os("ZENITH_SPLIT_WHY").is_some();
            let mut reasons: Vec<String> = Vec::new();

            for current_face in faces {
                match Self::split_face_by_edge(&current_face, split_edge, tol) {
                    Ok(split_faces) => {
                        applied_split_count += 1;
                        applied_this_edge = true;
                        next_faces.extend(split_faces);
                    }
                    Err(reason) => {
                        if explain {
                            reasons.push(reason);
                        }
                        next_faces.push(current_face);
                    }
                }
            }

            if !applied_this_edge {
                skipped_split_count += 1;
                if explain {
                    let start = split_edge.start_vertex.point;
                    let end = split_edge.end_vertex.point;
                    eprintln!(
                        "SPLITWHY ({:.4} {:.4} {:.4}) -> ({:.4} {:.4} {:.4}) split nothing",
                        start.x, start.y, start.z, end.x, end.y, end.z
                    );
                    for reason in &reasons {
                        eprintln!("  {}", reason.chars().take(160).collect::<String>());
                    }
                }
                leftover.push(split_edge.clone());
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

            let explain = std::env::var_os("ZENITH_SPLIT_WHY").is_some();
            if explain {
                eprintln!(
                    "SPLITWHY   鎖にまとめ直す: 交線 {} 本 → 鎖 {} 本（2本以上のもの {} 本）",
                    cutting.len(),
                    chains.len(),
                    chains.iter().filter(|chain| chain.len() >= 2).count()
                );
            }

            let mut chain_faces = vec![face.clone()];
            let mut applied: usize = 0;
            let mut applied_chain: Vec<bool> = vec![false; chains.len()];
            for (chain_index, chain) in chains.iter().enumerate() {
                if chain.len() < 2 {
                    continue;
                }
                let mut next_faces = Vec::new();
                for current_face in chain_faces {
                    match Self::split_planar_face_by_edge_chain(&current_face, chain, tol) {
                        Ok(split_faces) => {
                            applied += 1;
                            applied_chain[chain_index] = true;
                            next_faces.extend(split_faces);
                        }
                        Err(reason) => {
                            // **理由を捨てない。** 鎖にまとめ直しても割れない
                            // ときは、ここが最後の砦なので、断った理由が
                            // 分からないと次に測るところが決まらない。
                            if explain {
                                eprintln!(
                                    "SPLITWHY     鎖 {} 本を当てて断られた: {}",
                                    chain.len(),
                                    reason.chars().take(160).collect::<String>()
                                );
                            }
                            next_faces.push(current_face)
                        }
                    }
                }
                chain_faces = next_faces;
            }

            // **当たらなかった鎖を、切り詰めてから当て直します**（4-219）。
            //
            // 切り手の平面がこれです。箱の底面（z = 25）は、断面の輪でまず
            // 2枚に割れます。ところが穴の壁の切り口
            // `(-5, 3.3167, 25) -> (10, 3.3167, 25)` は、**割れた片の境界
            // （円柱の弧）を 0.536 追い越して**いるので当たりません。
            //
            // **1本の鎖も見ます。** 上の輪は2本以上でないと閉じませんが、
            // 切り詰めたあとは1本でも境界から境界へ渡れます。
            if applied > 0 {
                for (chain_index, chain) in chains.iter().enumerate() {
                    if applied_chain[chain_index] {
                        continue;
                    }
                    let mut next_faces = Vec::new();
                    for current_face in chain_faces {
                        let Some(clipped) = clip_chain_to_face_trim(&current_face, chain, tol)
                        else {
                            next_faces.push(current_face);
                            continue;
                        };
                        match crate::FaceSplitter::split_by_chain(&current_face, &clipped, tol) {
                            Ok((pieces, report))
                                if report.area_residual <= 1e-6 && pieces.len() >= 2 =>
                            {
                                applied += 1;
                                next_faces.extend(pieces);
                            }
                            other => {
                                if explain {
                                    match &other {
                                        Ok((pieces, report)) => eprintln!(
                                            "SPLITWHY     切り詰めた鎖 {} 本: 片 {} 枚、面積残差 {:.3e} で採らず",
                                            clipped.len(),
                                            pieces.len(),
                                            report.area_residual
                                        ),
                                        Err(reason) => eprintln!(
                                            "SPLITWHY     切り詰めた鎖 {} 本: {}",
                                            clipped.len(),
                                            reason.chars().take(160).collect::<String>()
                                        ),
                                    }
                                }
                                next_faces.push(current_face)
                            }
                        }
                    }
                    chain_faces = next_faces;
                }
            }

            if applied > 0 {
                return Ok(PlanarFaceMultiSplitResult {
                    faces: chain_faces,
                    applied_split_count: applied,
                    skipped_split_count: skipped_split_count.saturating_sub(applied),
                });
            }
        }

        // ここまでで1つも入らなかったときだけ、**端で繋がった1本の切り込み**
        // として当て直す。曲面同士の交線は相手のパッチの境界で細切れになって
        // 届き、どの片も面の内側で終わるので、1本ずつでは必ず断られる。
        //
        // 順序が要る。専用の経路を差し置いてここを先に通すと、回転した箱同士の
        // 和のように、既に通っていた割り方が別の割り方に置き換わって壊れる。
        //
        // **1本でもここへ来ます**（2026/08/25）。以前は「2本以上」を条件に
        // していたので、**曲面に届いた交線がちょうど1本のとき、汎用の
        // 面分割器を一度も試さずに諦めていました**。球を平面で切ると、球の
        // パッチ1枚に弧が1本だけ届きます。専用の経路は「円柱の側面と分かる
        // パッチしか割れない」と断り、そこで終わっていました（3-N-1）。
        //
        // ここは**どの経路も入らなかったときの受け皿**なので、広げても
        // 既に通っている割り方を横取りしません。
        if applied_split_count == 0 && !split_edges.is_empty() {
            // **鎖は1本とは限らない。** ここは来た稜を丸ごと1本の切り込みと
            // して渡していた。トーラス片をドリルで抜く配置では、ドリルの側面
            // 1枚に**出入り2箇所ぶん**の稜が届く。まとめて渡すと
            // 「4 loose ends, not two」と断られ、面はそのまま残っていた。
            //
            // 端の繋がりで鎖に分け、**1本ずつ順に**当てる。平面の経路は既に
            // そうしている（`group_edges_into_chains`）。
            let chains = group_edges_into_chains(&deduplicate_split_edges(split_edges, tol), tol);
            let why = std::env::var_os("ZENITH_SPLIT_WHY").is_some();
            if why {
                eprintln!(
                    "CHAINWHY 汎用の鎖分割へ: 交線 {} 本 → 鎖 {} 本",
                    split_edges.len(),
                    chains.len()
                );
            }
            let mut chain_faces = vec![face.clone()];
            let mut applied: usize = 0;
            for (chain_index, chain) in chains.iter().enumerate() {
                // **鎖ごとに1行にまとめます**（4-304）。
                //
                // 下の断り文は**面片ごと**に出ます。鎖は割ったあとの片すべてに
                // 当て直されるので、「別の片に属する鎖」も必ず1回は
                // 「境界から離れている」と断られます。**その1行だけを読むと、
                // 交線が短いように見えます**——実測でそう読みかけました。
                //
                // 知りたいのは「この鎖は**どこかで**当たったか」と、
                // 当たらなかったときの**いちばん惜しかった外れ**です。
                let mut chain_applied = 0usize;
                let mut best_gap = f64::INFINITY;
                let mut next_faces = Vec::new();
                for current_face in chain_faces {
                    // **閉じた輪は、鎖の分割では表せません**（4-304）。
                    //
                    // 刻印は `split_face_by_edges` の入口で1度だけ試して
                    // いましたが、そこには**届いた交線が丸ごと**渡ります。
                    // `linkrods.step` では 12 本が 4 つの輪に分かれており、
                    // まとめて渡すと「Cap edges do not form a continuous
                    // loop」で必ず落ちます。**輪ごとに試さないと当たりません。**
                    //
                    // **輪ごとに渡すのは、内側の輪でも同じです**（4-306）。
                    // 下の「最後の受け皿」は `split_by_interior_loop` を
                    // 呼んでいましたが、**交線を丸ごと**渡していました。
                    // `linkrods.step` では 32 本が 4 つの輪に分かれており、
                    // `order_closed_loop` は必ず落ちます。**4-304 で
                    // `split_by_chain` に入れたのと同じ誤りが、こちらに
                    // 残っていました。**
                    //
                    // 閉じた輪が面の内部で閉じていることは測ってあります
                    // （4-305。境界まで 0.345 ちょうど）。**切り込みでは
                    // なく穴**なので、ここで拾います。
                    let outcome = match crate::FaceSplitter::split_by_chain(&current_face, chain, tol)
                    {
                        Err(reason) => {
                            match crate::FaceSplitter::split_by_interior_loop(
                                &current_face,
                                chain,
                                tol,
                            ) {
                                // **元の理由のまま断ります。** 内側の輪でも
                                // ないなら、「開けなかった」を別の理由に
                                // 化けさせません。
                                Err(_) => Err(reason),
                                ok => ok,
                            }
                        }
                        ok => ok,
                    };
                    match outcome {
                        Ok((pieces, report))
                            if report.area_residual <= 1e-6 && pieces.len() >= 2 =>
                        {
                            applied += 1;
                            chain_applied += 1;
                            next_faces.extend(pieces);
                        }
                        // **理由を捨てない。** ここは最後の受け皿なので、
                        // 断られた理由が読めないと、次に測るところが決まらない。
                        other => {
                            if let Err(reason) = &other {
                                if let Some(gap) = reason
                                    .strip_prefix("the splitting curve ends ")
                                    .and_then(|rest| rest.split(' ').next())
                                    .and_then(|number| number.parse::<f64>().ok())
                                {
                                    best_gap = best_gap.min(gap);
                                }
                            }
                            if why {
                                match &other {
                                    Ok((pieces, report)) => eprintln!(
                                        "CHAINWHY   鎖 {} 本: 片 {} 枚、面積残差 {:.3e} で採らず",
                                        chain.len(),
                                        pieces.len(),
                                        report.area_residual
                                    ),
                                    Err(reason) => eprintln!(
                                        "CHAINWHY   鎖 {} 本: {}",
                                        chain.len(),
                                        reason.chars().take(160).collect::<String>()
                                    ),
                                }
                            }
                            next_faces.push(current_face)
                        }
                    }
                }
                if why {
                    if chain_applied > 0 {
                        eprintln!(
                            "CHAINWHY 鎖 {chain_index}（{} 本）: 当たった {chain_applied} 回",
                            chain.len()
                        );
                    } else if best_gap.is_finite() {
                        eprintln!(
                            "CHAINWHY 鎖 {chain_index}（{} 本）: **どの片にも当たらず**、いちばん惜しい外れ {best_gap:.3e}",
                            chain.len()
                        );
                    } else {
                        eprintln!(
                            "CHAINWHY 鎖 {chain_index}（{} 本）: **どの片にも当たらず**、境界の外れではない理由",
                            chain.len()
                        );
                    }
                }
                chain_faces = next_faces;
            }
            if applied > 0 {
                return Ok(PlanarFaceMultiSplitResult {
                    faces: chain_faces,
                    applied_split_count: split_edges.len(),
                    skipped_split_count: 0,
                });
            }

            // **そのままで入らなかったら、トリム境界まで切り詰めて当て直す。**
            //
            // 曲面のブーリアン結果をもう一度切ると、2段目の交線は支持パッチの
            // 縁まで伸びます。面の境界は1段目で削られた分だけ内側にあるので、
            // 端が **0.54〜1.00** 余ります（4-213）。切り詰めると、実測で
            // **5.6e-10** まで境界に着き、そのまま割れます（面 A0・A3 で
            // 残差 2.175e-9、A5 で 7.057e-15）。
            //
            // **上を試したあとに置いてあります。** 切り詰めは推測を含む
            // （どの交点で切るか）ので、素のままで通る割り方を横取りさせません。
            // **鎖にしてから、鎖の両端だけを切り詰めます。** 稜ごとに切ると、
            // 鎖の内側の継ぎ目（相方と共有する点）まで切ってしまいます。
            let clipped_chains: Vec<Vec<Edge>> = chains
                .iter()
                .map(|chain| {
                    clip_chain_to_face_trim(face, chain, tol).unwrap_or_else(|| chain.clone())
                })
                .collect();
            if clipped_chains
                .iter()
                .zip(chains.iter())
                .any(|(after, before)| {
                    after.len() != before.len()
                        || after
                            .iter()
                            .zip(before.iter())
                            .any(|(a, b)| a.curve != b.curve)
                })
            {
                let chains = clipped_chains;
                if why {
                    eprintln!("CHAINWHY 切り詰めて当て直す: 鎖 {} 本", chains.len());
                }
                let mut chain_faces = vec![face.clone()];
                let mut applied: usize = 0;
                for chain in chains.iter() {
                    let mut next_faces = Vec::new();
                    for current_face in chain_faces {
                        match crate::FaceSplitter::split_by_chain(&current_face, chain, tol) {
                            Ok((pieces, report))
                                if report.area_residual <= 1e-6 && pieces.len() >= 2 =>
                            {
                                applied += 1;
                                next_faces.extend(pieces);
                            }
                            other => {
                                if why {
                                    match &other {
                                        Ok((pieces, report)) => eprintln!(
                                            "CHAINWHY   切り詰めた鎖 {} 本: 片 {} 枚、面積残差 {:.3e} で採らず",
                                            chain.len(),
                                            pieces.len(),
                                            report.area_residual
                                        ),
                                        Err(reason) => eprintln!(
                                            "CHAINWHY   切り詰めた鎖 {} 本: {}",
                                            chain.len(),
                                            reason.chars().take(160).collect::<String>()
                                        ),
                                    }
                                }
                                next_faces.push(current_face)
                            }
                        }
                    }
                    chain_faces = next_faces;
                }
                if applied > 0 {
                    return Ok(PlanarFaceMultiSplitResult {
                        faces: chain_faces,
                        applied_split_count: split_edges.len(),
                        skipped_split_count: 0,
                    });
                }
            }

            // 繋いでも境界に届かない切り込みは、**面の内側で閉じている**
            // ことがあります。球の角を箱で削ると、3枚の面が作る3本の弧が
            // 球面上で輪になり、どの弧も面の境界には着きません。
            // `split_by_chain` は「境界に届かない」と断り、面は無傷のまま
            // 残って、相手側の弧に相手がいなくなります（4-50）。
            //
            // ここも**最後の受け皿**です。境界から境界へ届く切り込みが
            // あるならそちらが先に通っているはずで、ここへは来ません。
            if let Ok((pieces, report)) =
                crate::FaceSplitter::split_by_interior_loop(face, split_edges, tol)
            {
                if report.area_residual <= 1e-6 && pieces.len() >= 2 {
                    return Ok(PlanarFaceMultiSplitResult {
                        faces: pieces,
                        applied_split_count: split_edges.len(),
                        skipped_split_count: 0,
                    });
                }
            }

            // **切り込みが穴を横切る**配置。円環の面を外周から穴まで走る
            // 切り込みで割るとこれになります。ここまでのどの経路も、
            // 切り込みの端を外側のワイヤにしか探しません。
            //
            // ここも最後の受け皿です。上のどれかが通っていれば来ません。
            if let Ok(pieces) = split_planar_face_across_holes(face, split_edges, tol) {
                if pieces.len() >= 2 {
                    return Ok(PlanarFaceMultiSplitResult {
                        faces: pieces,
                        applied_split_count: split_edges.len(),
                        skipped_split_count: 0,
                    });
                }
            }
        }

        // **1本でも当たると、鎖の経路が一度も走りませんでした。**
        //
        // 上の2つの受け皿はどちらも `applied_split_count == 0` を条件にして
        // います。1枚の面に**2本の鎖**が届き、片方が1本の稜で単独に当たると、
        // もう片方は当たらないまま捨てられます。実測（全周円錐を傾けた
        // ドリルで抜く）:
        //
        // ```text
        // A0   3 split edge(s) -> 2 piece(s) (applied 1, skipped 2)
        // B1   4 split edge(s) -> 2 piece(s) (applied 1, skipped 3)
        // ```
        //
        // **当たらなかった稜だけ**を鎖にまとめて、当たった片に対して当て直し
        // ます。全部当たっていれば残りは無く、ここは素通りします。
        let leftover_why = std::env::var_os("ZENITH_SPLIT_WHY").is_some();
        if leftover_why && !leftover.is_empty() {
            eprintln!(
                "LEFTWHY 当たらなかった稜 {} 本（当たった割り {}）",
                leftover.len(),
                applied_split_count
            );
        }
        if applied_split_count > 0 && leftover.len() >= 2 {
            let chains = group_edges_into_chains(&deduplicate_split_edges(&leftover, tol), tol);
            if leftover_why {
                eprintln!("LEFTWHY   残りを鎖に: {} 本", chains.len());
            }
            for chain in chains.iter().filter(|chain| chain.len() >= 2) {
                let mut next_faces = Vec::new();
                let mut applied_this_chain = false;
                for current_face in faces {
                    match crate::FaceSplitter::split_by_chain(&current_face, chain, tol) {
                        Ok((pieces, report))
                            if report.area_residual <= 1e-6 && pieces.len() >= 2 =>
                        {
                            applied_this_chain = true;
                            next_faces.extend(pieces);
                        }
                        // **理由を捨てない。** ここは「1本でも当たると鎖が
                        // 走らない」を埋めるための受け皿なので、断られた
                        // 理由が読めないと、埋まっているのかも分からない。
                        other => {
                            if leftover_why {
                                match &other {
                                    Ok((pieces, report)) => eprintln!(
                                        "LEFTWHY   鎖 {} 本: 片 {} 枚、面積残差 {:.3e} で採らず",
                                        chain.len(),
                                        pieces.len(),
                                        report.area_residual
                                    ),
                                    Err(reason) => eprintln!(
                                        "LEFTWHY   鎖 {} 本: {}",
                                        chain.len(),
                                        reason.chars().take(160).collect::<String>()
                                    ),
                                }
                            }
                            next_faces.push(current_face)
                        }
                    }
                }
                faces = next_faces;
                if applied_this_chain {
                    applied_split_count += chain.len();
                    skipped_split_count = skipped_split_count.saturating_sub(chain.len());
                }
            }

            // **残りも、トリム境界まで切り詰めてから当て直します**（4-219）。
            //
            // 切り手の平面がこれです。箱の底面（z = 25）は、断面の輪で
            // まず2枚に割れます。ところが穴の壁の切り口
            // `(-5, 3.3167, 25) -> (10, 3.3167, 25)` は、**割れた片の境界
            // （円柱の弧）を 0.536 追い越して**いるので当たりません。
            // 4-217 の切り詰めは「1つも当たらなかったとき」の受け皿に入れた
            // ので、**一部だけ当たったここには効いていませんでした**。
            //
            // **鎖の長さで絞りません。** 上の輪は 2本以上でないと閉じま
            // せんが、ここは1本でも境界から境界へ渡れます。
            for chain in group_edges_into_chains(&deduplicate_split_edges(&leftover, tol), tol) {
                let mut next_faces = Vec::new();
                let mut applied_this_chain = false;
                for current_face in faces {
                    let clipped = clip_chain_to_face_trim(&current_face, &chain, tol);
                    let Some(clipped) = clipped else {
                        next_faces.push(current_face);
                        continue;
                    };
                    match crate::FaceSplitter::split_by_chain(&current_face, &clipped, tol) {
                        Ok((pieces, report))
                            if report.area_residual <= 1e-6 && pieces.len() >= 2 =>
                        {
                            applied_this_chain = true;
                            next_faces.extend(pieces);
                        }
                        other => {
                            if leftover_why {
                                match &other {
                                    Ok((pieces, report)) => eprintln!(
                                        "LEFTWHY   切り詰めた鎖 {} 本: 片 {} 枚、面積残差 {:.3e} で採らず",
                                        clipped.len(),
                                        pieces.len(),
                                        report.area_residual
                                    ),
                                    Err(reason) => eprintln!(
                                        "LEFTWHY   切り詰めた鎖 {} 本: {}",
                                        clipped.len(),
                                        reason.chars().take(160).collect::<String>()
                                    ),
                                }
                            }
                            next_faces.push(current_face)
                        }
                    }
                }
                faces = next_faces;
                if applied_this_chain {
                    applied_split_count += chain.len();
                    skipped_split_count = skipped_split_count.saturating_sub(chain.len());
                }
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
    /// **この稜を出した面の番号**（4-292）。
    ///
    /// 座標だけでは「どの面の片が合っていないか」が言えません。相手のいない
    /// 稜が 56 本あっても、**それが何枚の面から来ているのか**——1枚が丸ごと
    /// 浮いているのか、多くの面が1本ずつ足りないのか——で、次に見る場所が
    /// 変わります。診断にしか使いません。
    face_id: u64,
    start: Point3,
    end: Point3,
    /// 稜の途中の点。**端点だけでは稜を見分けられません。**
    ///
    /// 同じ2点を結ぶ別々の弧は普通にあります。トーラスを傾けたスラブで切ると、
    /// 管の底の継ぎ目（半径 12・z = -4 の円）の上で2つの四半パッチが出会い、
    /// 継ぎ目の同じ2点を結ぶ**2本の弧**が交線として出ます。中点は 3.839 離れて
    /// おり、別の曲線です。
    ///
    /// 端点だけで突き合わせると、この2本が1本と数えられ、閉じた殻なのに
    /// 「同じ稜が3回使われている」（非多様体）と報告されます。**壊れていたのは
    /// 立体ではなく、数え方でした**（4-65）。
    middle: Point3,
    /// **診断だけに使います。** どちらの立体の面片から来た稜か。
    /// 照合には使いません（4-174）。
    operand: BooleanOperand,
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

/// Builds cap faces from closed loops, letting a loop inside another become a
/// hole in it rather than a disc of its own.
///
/// A plane through a torus leaves two loops, and what they bound between them
/// is an annulus. Capping each with its own disc covers the hole as well as the
/// ring, and every edge of the inner loop ends up used twice the same way round,
/// which is what the stitching reported.
///
/// Loops are only compared with loops in the same plane. Two rings at different
/// heights are not nested however they look from above.
/// Splits a planar face along closed loops that lie inside it.
///
/// A loop that closes without touching the boundary is not a cut across the
/// face, it is the outline of a region within it, and the face has to come
/// apart into the regions those outlines mark off. Taking the loop apart into
/// single edges and asking each to reach the boundary - which is what the cut
/// path does - leaves every one of them refused and the face whole.
///
/// A plane through a torus leaves two such loops, one inside the other, and the
/// face they sit on falls into three regions: what is outside both, the ring
/// between them, and the disc inside. Which of those belong to the answer is
/// the selection stage's business; this only has to produce them.
///
/// Returns `None` when the edges are not closed loops inside the face, so the
/// ordinary cut path keeps handling those.
fn split_planar_face_by_interior_loops(
    face: &Face,
    split_edges: &[Edge],
    tol: &Tolerance,
) -> Option<Vec<Face>> {
    let FaceGeometry::Plane(plane) = &face.geometry else {
        return None;
    };
    if !face.inner_wires.is_empty() || split_edges.is_empty() {
        return None;
    }

    let extraction = collect_closed_intersection_edge_loops(split_edges, tol);
    if extraction.loops.is_empty() || extraction.skipped_edge_count > 0 {
        return None;
    }

    let boundary: Vec<Point2> = face
        .outer_wire
        .sample_points(24)
        .iter()
        .map(|point| project_to_plane_uv(*point, plane))
        .collect();

    let mut wires: Vec<Wire> = Vec::with_capacity(extraction.loops.len());
    let mut outlines: Vec<Vec<Point2>> = Vec::with_capacity(extraction.loops.len());
    for edge_loop in &extraction.loops {
        let wire = order_edges_into_closed_wire(&edge_loop.edges, tol).ok()?;
        let outline: Vec<Point2> = wire
            .sample_points(16)
            .iter()
            .map(|point| project_to_plane_uv(*point, plane))
            .collect();
        // 面の内側で閉じていなければ、これは切り込みのほう。
        for point in &outline {
            if !point_in_polygon_2d(*point, &boundary, tol.parametric)
                || point_on_polygon_boundary(*point, &boundary, tol.parametric)
            {
                return None;
            }
        }
        wires.push(wire);
        outlines.push(outline);
    }

    // 入れ子の深さ。境界そのものを深さ0とし、ループはそれより内側にある。
    let mut depth = vec![1usize; outlines.len()];
    for (inner, outline) in outlines.iter().enumerate() {
        for (outer, candidate) in outlines.iter().enumerate() {
            if outer != inner && point_in_polygon_2d(outline[0], candidate, tol.parametric) {
                depth[inner] += 1;
            }
        }
    }

    let contains = |outer: usize, inner: usize| {
        outer != inner && point_in_polygon_2d(outlines[inner][0], &outlines[outer], tol.parametric)
    };

    // 元の面がどちら回りを外周としているか。分けた領域も同じ約束に揃える。
    let parent_winding = signed_area_2d(&boundary).signum();

    let mut regions = Vec::with_capacity(outlines.len() + 1);

    let outer_children: Vec<usize> = (0..outlines.len()).filter(|i| depth[*i] == 1).collect();
    regions.push(planar_region_face(
        face,
        face.outer_wire.clone(),
        parent_winding,
        parent_winding,
        &outer_children,
        &wires,
        &outlines,
    ));

    for (index, wire) in wires.iter().enumerate() {
        let children: Vec<usize> = (0..outlines.len())
            .filter(|other| depth[*other] == depth[index] + 1 && contains(index, *other))
            .collect();
        regions.push(planar_region_face(
            face,
            wire.clone(),
            signed_area_2d(&outlines[index]).signum(),
            parent_winding,
            &children,
            &wires,
            &outlines,
        ));
    }

    Some(regions)
}

/// One region of a decomposed planar face: an outline, and the loops that sit
/// directly inside it as holes.
fn planar_region_face(
    face: &Face,
    outer: Wire,
    outer_winding: f64,
    parent_winding: f64,
    children: &[usize],
    wires: &[Wire],
    outlines: &[Vec<Point2>],
) -> Face {
    let outer = if outer_winding == parent_winding {
        outer
    } else {
        reversed_wire(&outer)
    };

    let holes: Vec<Wire> = children
        .iter()
        .map(|child| {
            if signed_area_2d(&outlines[*child]).signum() == parent_winding {
                reversed_wire(&wires[*child])
            } else {
                wires[*child].clone()
            }
        })
        .collect();

    Face::new(
        face.geometry.clone(),
        outer,
        holes,
        face.orientation,
        face.tolerance,
    )
}

fn build_caps_from_nested_loops(wires: Vec<Wire>, tol: &Tolerance) -> (Vec<Face>, usize) {
    let mut faces = Vec::new();
    let mut failures = 0;

    let trace = std::env::var_os("ZENITH_CAP_TRACE").is_some();
    for group in group_coplanar_loops(&wires, tol) {
        // 面の枠は、この組の最初のループから取る。同一平面なので共通に使える。
        let Some((origin, normal)) = planar_loop_frame(&wires[group[0]]) else {
            // **輪が平面と読めない**、という段があります。「蓋 0 枚」だけでは
            // ここか、枠の軸か、面の作成か、検算かが分かりません（4-220）。
            if trace {
                // **折り返した「輪」がここに来ます。** 同じ弧を往復すると
                // 囲む面積が 0 になり、Newell の法線が取れません（4-220）。
                // 点をそのまま出すのは、それを見分けるためです。
                let points = wires[group[0]].sample_points(4);
                eprintln!(
                    "      輪 {} 本が平面と読めない（標本 {} 点）",
                    wires[group[0]].edges.len(),
                    points.len()
                );
                for (index, point) in points.iter().enumerate() {
                    eprintln!(
                        "        {index:>2} ({:.4} {:.4} {:.4})",
                        point.x, point.y, point.z
                    );
                }
            }
            failures += group.len();
            continue;
        };
        let Some(frame_u) = plane_frame_axis(normal) else {
            if trace {
                eprintln!(
                    "      枠の軸が取れない（法線 {:.4} {:.4} {:.4}）",
                    normal.x, normal.y, normal.z
                );
            }
            failures += group.len();
            continue;
        };
        let frame_v = normal.cross(&frame_u);
        let to_plane = |point: Point3| {
            let offset = point - origin;
            Point2::new(offset.dot(&frame_u), offset.dot(&frame_v))
        };

        let outlines: Vec<Vec<Point2>> = group
            .iter()
            .map(|index| {
                wires[*index]
                    .sample_points(16)
                    .into_iter()
                    .map(to_plane)
                    .collect()
            })
            .collect();

        // 何枚のループに囲まれているか。偶数なら外周、奇数なら穴。
        let mut depth = vec![0usize; group.len()];
        for (inner, outline) in outlines.iter().enumerate() {
            let Some(probe) = outline.first().copied() else {
                continue;
            };
            for (outer, candidate) in outlines.iter().enumerate() {
                if outer == inner {
                    continue;
                }
                if point_in_polygon_2d(probe, candidate, tol.parametric) {
                    depth[inner] += 1;
                }
            }
        }

        for (position, index) in group.iter().enumerate() {
            if depth[position] % 2 != 0 {
                continue;
            }
            // 自分をすぐ内側から囲まれているループが、この面の穴。
            //
            // 穴は外周と逆回りでなければならない。面ごと裏返しても両方の
            // 向きが同時に変わるだけなので、ここで揃えておかないと後から
            // 直せない。組み立て側は面全体の表裏しか試さない。
            let outer_winding = signed_area_2d(&outlines[position]).signum();
            let holes: Vec<Wire> = group
                .iter()
                .enumerate()
                .filter(|(other, _)| {
                    depth[*other] == depth[position] + 1
                        && point_in_polygon_2d(
                            outlines[*other][0],
                            &outlines[position],
                            tol.parametric,
                        )
                })
                .map(|(other, other_index)| {
                    let wire = wires[*other_index].clone();
                    if signed_area_2d(&outlines[other]).signum() == outer_winding {
                        reversed_wire(&wire)
                    } else {
                        wire
                    }
                })
                .collect();

            let trace = std::env::var_os("ZENITH_CAP_TRACE").is_some();
            match CapBuilder::make_planar_cap_with_holes(wires[*index].clone(), holes) {
                Ok(face) => match face.validate_pcurves(tol, 4) {
                    Ok(report) if report.is_valid() => faces.push(face),
                    // **どちらで落ちたかを言う。** 「蓋が 0 枚」だけでは、
                    // 面が作れないのか、作った面が検算を通らないのかが
                    // 分からない（4-220）。
                    other => {
                        if trace {
                            match &other {
                                Ok(report) => eprintln!(
                                    "      蓋の p-curve 検算が通らない: {}",
                                    report
                                        .errors
                                        .first()
                                        .map(|e| e.chars().take(150).collect::<String>())
                                        .unwrap_or_default()
                                ),
                                Err(reason) => eprintln!(
                                    "      蓋の p-curve が取れない: {}",
                                    reason.chars().take(150).collect::<String>()
                                ),
                            }
                        }
                        failures += 1
                    }
                },
                Err(reason) => {
                    if trace {
                        eprintln!(
                            "      蓋の面が作れない: {}",
                            reason.chars().take(150).collect::<String>()
                        );
                    }
                    failures += 1
                }
            }
        }
    }

    (faces, failures)
}

/// The same wire walked the other way round.
fn reversed_wire(wire: &Wire) -> Wire {
    Wire::new(
        wire.edges
            .iter()
            .rev()
            .map(|oriented| {
                OrientedEdge::new(oriented.edge.clone(), oriented.orientation.reversed())
            })
            .collect(),
    )
}

/// Groups loops that lie in the same plane.
fn group_coplanar_loops(wires: &[Wire], tol: &Tolerance) -> Vec<Vec<usize>> {
    let frames: Vec<Option<(Point3, Vec3)>> = wires.iter().map(planar_loop_frame).collect();
    let mut groups: Vec<Vec<usize>> = Vec::new();

    for (index, frame) in frames.iter().enumerate() {
        let Some((origin, normal)) = frame else {
            groups.push(vec![index]);
            continue;
        };
        let mut joined = false;
        for group in groups.iter_mut() {
            let Some((other_origin, other_normal)) = frames[group[0]] else {
                continue;
            };
            let parallel = normal.dot(&other_normal).abs() >= 1.0 - tol.angular;
            let same_level = (origin - other_origin).dot(&other_normal).abs() <= tol.linear * 10.0;
            if parallel && same_level {
                group.push(index);
                joined = true;
                break;
            }
        }
        if !joined {
            groups.push(vec![index]);
        }
    }

    groups
}

/// A point on a closed planar wire and the plane's unit normal.
fn planar_loop_frame(wire: &Wire) -> Option<(Point3, Vec3)> {
    let points = wire.sample_points(8);
    if points.len() < 3 {
        return None;
    }

    let mut center = Vec3::new(0.0, 0.0, 0.0);
    for point in &points {
        center += point.coords;
    }
    let center = Point3::from(center / points.len() as f64);

    // Newell 法。三点だけで法線を取ると、ほぼ一直線の並びで壊れる。
    let mut normal = Vec3::new(0.0, 0.0, 0.0);
    for index in 0..points.len() {
        let current = points[index];
        let next = points[(index + 1) % points.len()];
        normal.x += (current.y - next.y) * (current.z + next.z);
        normal.y += (current.z - next.z) * (current.x + next.x);
        normal.z += (current.x - next.x) * (current.y + next.y);
    }

    normal
        .try_normalize_safe(1e-12)
        .map(|normal| (center, normal))
}

/// Any unit vector lying in the plane with the given normal.
fn plane_frame_axis(normal: Vec3) -> Option<Vec3> {
    let seed = if normal.x.abs() < 0.9 {
        Vec3::new(1.0, 0.0, 0.0)
    } else {
        Vec3::new(0.0, 1.0, 0.0)
    };
    (seed - normal * seed.dot(&normal)).try_normalize_safe(1e-12)
}

fn order_edges_into_closed_wire(edges: &[Edge], tol: &Tolerance) -> Result<Wire, String> {
    // **1本で閉じている交線は、それだけでワイヤです**（4-289）。
    //
    // 平面が丸棒を横切ると、交線は閉じた1本になります。`< 3` で断っていたので、
    // 輪として拾えるようにしたあとも（同じ 4-289）、ここで落ちていました
    // ——**同じ条件が2か所にありました**。
    if edges.len() == 1
        && points_same_3d(
            edges[0].start_vertex.point,
            edges[0].end_vertex.point,
            tol.linear,
        )
        && sampled_edge_extent(&edges[0]) > tol.linear
    {
        return Ok(Wire::new(vec![OrientedEdge::forward(edges[0].clone())]));
    }
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

        // **1本で閉じている交線は、それだけで輪です**（4-289）。
        //
        // 平面が丸棒を横切ると、交線は**閉じた1本**になります（端が同じ点）。
        // `>= 3` で弾いていたので、そういう輪は材料から丸ごと落ちていました
        // ——実測（OCCT の `linkrods.step`）: 蓋の材料 10 本のうち **4 本**が
        // これで、**蓋が1枚も作れず**ブーリアンが断られていました。
        //
        // **広がりを確かめてから通します。** 端が同じでも潰れているものは、
        // 接触の記録であって輪ではありません（3-1 の規約）。
        let closed = points_same_3d(current_end, loop_start, tol.linear);
        let single_closed_loop =
            loop_edges.len() == 1 && closed && sampled_edge_extent(&loop_edges[0]) > tol.linear;
        if closed && (loop_edges.len() >= 3 || single_closed_loop) {
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
    inner_shell_flags: &[bool],
    batch_splits: &[PlanarFaceBatchSplit],
    operand: BooleanOperand,
    other_mesh: &TriangleMesh,
    other_solid: Option<&Solid>,
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
            let location = classify_face_against_mesh(face, other_mesh, other_solid, tol);
            if std::env::var_os("ZENITH_SELECT_WHY").is_some() {
                // **採否そのものを見せる。** 片が欠けているとき、割れて
                // いないのか、割れたが採られなかったのかは、ここでしか
                // 区別できない。
                let point = representative_face_point(face);
                let piece_area = crate::MassCalculator::compute_face_integral(
                    face,
                    &TessellationParams::default(),
                )
                .0;
                // **面積を積む側が見ている領域**を、そのまま測る。
                // `face_parameter_area` は p-curve から、こちらは
                // 三角形分割から。食い違えば、経路が違う。
                let triangulated = {
                    let uv =
                        zenith_tess::face_uv_triangulation(face, &TessellationParams::default());
                    let mut sum = 0.0;
                    for triangle in &uv.triangles {
                        let (a, b, c) = (
                            uv.uvs[triangle[0]],
                            uv.uvs[triangle[1]],
                            uv.uvs[triangle[2]],
                        );
                        sum += ((b.x - a.x) * (c.y - a.y) - (b.y - a.y) * (c.x - a.x)).abs() * 0.5;
                    }
                    sum
                };
                let from_pcurves = zenith_tess::face_parameter_area(face).unwrap_or(f64::NAN);
                if (triangulated - from_pcurves).abs() > 1e-9
                    && std::env::var_os("ZENITH_TRI_WHY").is_some()
                {
                    // **三角形分割が p-curve より広い。** どの三角形が
                    // 余分なのかを、切り欠きの中に重心がある三角形として出す。
                    let uv =
                        zenith_tess::face_uv_triangulation(face, &TessellationParams::default());
                    eprintln!(
                        "TRIWHY 三角形 {} 枚、p-curve {from_pcurves:.6} < 三角形 {triangulated:.6}（差 {:.6}）",
                        uv.triangles.len(),
                        triangulated - from_pcurves
                    );
                    let (mut signed, mut negative_area, mut negative_count) = (0.0, 0.0, 0usize);
                    let mut worst: Option<(f64, f64, f64)> = None;
                    for triangle in &uv.triangles {
                        let (a, b, c) = (
                            uv.uvs[triangle[0]],
                            uv.uvs[triangle[1]],
                            uv.uvs[triangle[2]],
                        );
                        let twice = (b.x - a.x) * (c.y - a.y) - (b.y - a.y) * (c.x - a.x);
                        signed += twice * 0.5;
                        if twice < 0.0 {
                            negative_count += 1;
                            negative_area += -twice * 0.5;
                            let centre = (
                                (a.x + b.x + c.x) / 3.0,
                                (a.y + b.y + c.y) / 3.0,
                                -twice * 0.5,
                            );
                            if worst.is_none_or(|w| centre.2 > w.2) {
                                worst = Some(centre);
                            }
                        }
                    }
                    eprintln!(
                        "TRIWHY   符号付きの和 {signed:.6}、裏返った三角形 {negative_count} 枚（面積の合計 {negative_area:.6}）"
                    );
                    if let Some((u, v, area)) = worst {
                        eprintln!(
                            "TRIWHY   いちばん大きい裏返り: 重心 ({u:.6},{v:.6}) 面積 {area:.6}"
                        );
                    }
                }
                eprintln!(
                    "SELECTWHY {:?} 面 {face_index} の片 id {} （{} 枚中）: {:?} → {} 面積 {piece_area:.6} uv[p-curve {from_pcurves:.6} / 三角形 {triangulated:.6}] 代表点 ({:.4} {:.4} {:.4})",
                    operand,
                    face.id,
                    face_pieces.len(),
                    location,
                    if keep_piece(operand, location, op) {
                        "採る"
                    } else {
                        "落とす"
                    },
                    point.x,
                    point.y,
                    point.z
                );
                if std::env::var_os("ZENITH_SELECT_UV").is_some() {
                    // 片の境界を uv で見る。3D 面積だけ違って uv 面積が同じ
                    // なら、**同じ広さの別の場所**を占めているか、ループが
                    // 自分と重なっている。
                    if let Some(pcurves) = face
                        .pcurves
                        .clone()
                        .or_else(|| face.pcurves(&Tolerance::default()).ok())
                    {
                        let mut points: Vec<zenith_math::Point2> = Vec::new();
                        for segment in &pcurves.outer_loop.segments {
                            let (t0, t1) = segment.curve.param_range();
                            for step in 0..=8 {
                                points.push(
                                    segment.curve.evaluate(t0 + (t1 - t0) * step as f64 / 8.0),
                                );
                            }
                        }
                        let (mut u0, mut u1, mut v0, mut v1) = (
                            f64::INFINITY,
                            f64::NEG_INFINITY,
                            f64::INFINITY,
                            f64::NEG_INFINITY,
                        );
                        for point in &points {
                            u0 = u0.min(point.x);
                            u1 = u1.max(point.x);
                            v0 = v0.min(point.y);
                            v1 = v1.max(point.y);
                        }
                        eprintln!(
                            "SELECTUV   uv 範囲 u {u0:.6}..{u1:.6}、v {v0:.6}..{v1:.6}、稜 {}、点 {}",
                            pcurves.outer_loop.segments.len(),
                            points.len()
                        );
                        for (segment_index, segment) in
                            pcurves.outer_loop.segments.iter().enumerate()
                        {
                            let (t0, t1) = segment.curve.param_range();
                            let (a, b) = (segment.curve.evaluate(t0), segment.curve.evaluate(t1));
                            eprintln!(
                                "SELECTUV     稜 {segment_index}: ({:.6},{:.6}) -> ({:.6},{:.6})",
                                a.x, a.y, b.x, b.y
                            );
                        }
                    }
                }
                if std::env::var_os("ZENITH_SELECT_EDGES").is_some()
                    && keep_piece(operand, location, op)
                {
                    // 採った片の境界そのもの。**相手を欠いている稜が
                    // どの片から来たのか**は、これがないと辿れない。
                    for oriented in &face.outer_wire.edges {
                        let (a, b) = (oriented.start_vertex(), oriented.end_vertex());
                        eprintln!(
                            "SELECTEDGE   ({:.4} {:.4} {:.4}) -> ({:.4} {:.4} {:.4})",
                            a.point.x, a.point.y, a.point.z, b.point.x, b.point.y, b.point.z
                        );
                    }
                }
            }
            if keep_piece(operand, location, op) {
                // **空洞の壁は、この立体では実効法線が材料の中を向いています**
                // （外側シェルは外向き。4-144）。切り手が空洞を貫くと壁が
                // 外側の境界に繋がるので、ここで揃えないと縫合が
                // 「同方向の稜」になります。
                let from_inner_shell = inner_shell_flags.get(face_index).copied().unwrap_or(false);
                let reverse_for_difference =
                    operand == BooleanOperand::B && op == crate::BooleanOpType::Difference;
                selected.push(SelectedBooleanFacePiece {
                    operand,
                    face: face.clone(),
                    location,
                    reverse_orientation: reverse_for_difference != from_inner_shell,
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
        let (_low, rest) = split_oriented_edge_at(edge, t_start, tol)?;
        let rem_t = ((t_end - t_start) / (1.0 - t_start)).clamp(0.0, 1.0);
        let (portion, _high) = split_oriented_edge_at(&rest, rem_t, tol)?;
        return Ok(Some(portion));
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

    // **1スパンとは限らない。** ここは `split_bezier_at` だけを使っていて、
    // 内部ノットを持つ境界稜——押し出したスプラインの輪郭がまさにこれ——を
    // 「割れない」と断っていた。断られると面はそのまま残り、切り口が縫えない。
    //
    // `split_at` はノット挿入で割るので、多スパンでも有理でも通る。**割った
    // 2本は元の曲線と同じ点を通る**（重みは挿入で保たれる）。1スパンでは
    // これまでどおり de Casteljau のほうが安いので、先にそちらを試す。
    let (low, high) = curve
        .split_bezier_at(curve_param)
        .or_else(|| curve.split_at(curve_param))
        .ok_or_else(|| "Boundary edge cannot be subdivided at the landing point".to_string())?;
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
/// 点から面の外周ワイヤまでの距離。
fn distance_to_outer_wire(face: &Face, point: Point3) -> f64 {
    let mut best = f64::MAX;
    for oriented in &face.outer_wire.edges {
        if let Ok(result) =
            zenith_geom::ExtremumEngine::point_to_curve(point, &oriented.edge.curve, 64, 1e-14)
        {
            best = best.min(result.distance);
        }
    }
    best
}

/// `lo..hi` の中で「境界までの距離」を最小にする位置を、黄金分割で詰める。
fn nearest_approach_to_wire(
    curve: &NurbsCurve3,
    face: &Face,
    mut lo: f64,
    mut hi: f64,
) -> (f64, f64) {
    let phi = (5.0_f64.sqrt() - 1.0) * 0.5;
    let mut c = hi - phi * (hi - lo);
    let mut d = lo + phi * (hi - lo);
    let mut distance_c = distance_to_outer_wire(face, curve.evaluate(c));
    let mut distance_d = distance_to_outer_wire(face, curve.evaluate(d));
    for _ in 0..120 {
        if distance_c < distance_d {
            hi = d;
            d = c;
            distance_d = distance_c;
            c = hi - phi * (hi - lo);
            distance_c = distance_to_outer_wire(face, curve.evaluate(c));
        } else {
            lo = c;
            c = d;
            distance_c = distance_d;
            d = lo + phi * (hi - lo);
            distance_d = distance_to_outer_wire(face, curve.evaluate(d));
        }
        if (hi - lo).abs() < 1e-15 {
            break;
        }
    }
    let at = 0.5 * (lo + hi);
    (at, distance_to_outer_wire(face, curve.evaluate(at)))
}

/// 切り込みの**自由端**を、面のトリム境界まで切り詰める。
///
/// # なぜ要るか
///
/// 曲面のブーリアン結果をもう一度切ると、2段目の交線は**支持パッチの縁**
/// まで伸びます。ところが面の実際の境界は、1段目で削られた分だけ内側へ
/// 後退しています。その差が実測で **0.54〜1.00**（4-213）——`split_by_chain`
/// は「端が境界から離れている」と断ります。
///
/// # どこを切ってよいか
///
/// **鎖の自由端だけ**です。鎖の内側の継ぎ目は、相方の稜と共有している点で、
/// **境界の上にある必要はありません**。そこまで切り詰めると鎖が壊れます。
/// 最初はここを区別せずに「境界に乗っていない端」を全部切ろうとして、
/// ドリル穴の壁で `None` を返して何もできませんでした（4-217）。
///
/// # どう切るか
///
/// **3D で、境界までの距離が 0 になるところ**を探します。UV の多角形で
/// 挟み込むやり方は 4-213 で試して、端が 1.9e-5 の桁でしか合わず、吸着で
/// 曲線が浮きました（4-215）。ここは距離を黄金分割で詰めるので、実測で
/// **5.6e-10** まで合います。切るのは `split_at` で厳密に行い、**当てはめ
/// 直しません**——当てはめ直すと自分の当てはめ誤差を測ることになります
/// （4-213 の教訓）。
///
/// 境界を跨いでいない自由端は切れません。そのときはその側を動かしません。
fn clip_edge_ends_to_face_trim(
    face: &Face,
    edge: &Edge,
    clip_start: bool,
    clip_end: bool,
    tol: &Tolerance,
) -> Option<Edge> {
    let curve = &edge.curve;
    let (t0, t1) = curve.param_range();
    let on_boundary = tol.linear.max(1e-6);
    let start_gap = distance_to_outer_wire(face, curve.evaluate(t0));
    let end_gap = distance_to_outer_wire(face, curve.evaluate(t1));
    let want_start = clip_start && start_gap > on_boundary;
    let want_end = clip_end && end_gap > on_boundary;
    if !want_start && !want_end {
        return None;
    }

    let steps = 64usize;
    let at = |index: usize| t0 + (t1 - t0) * index as f64 / steps as f64;
    let gaps: Vec<f64> = (0..=steps)
        .map(|index| distance_to_outer_wire(face, curve.evaluate(at(index))))
        .collect();

    // **距離の極小はいくらでもあります。** 0 に落ちたものだけが交点です。
    let mut crossings: Vec<f64> = Vec::new();
    for index in 1..steps {
        if gaps[index] < gaps[index - 1] && gaps[index] <= gaps[index + 1] {
            let (found, gap) = nearest_approach_to_wire(curve, face, at(index - 1), at(index + 1));
            if gap <= on_boundary {
                crossings.push(found);
            }
        }
    }
    if crossings.is_empty() {
        return None;
    }
    crossings.sort_by(|a, b| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal));

    let span = t1 - t0;
    // **切る端から数えて、いちばん近い交点**まで詰めます。
    let head = if want_start { crossings[0] } else { t0 };
    let tail = if want_end {
        crossings[crossings.len() - 1]
    } else {
        t1
    };
    if tail <= head + span * 1e-12 {
        return None;
    }

    let after_head = if head <= t0 + span * 1e-12 {
        curve.clone()
    } else {
        curve.split_at(head)?.1
    };
    let (head_min, head_max) = after_head.param_range();
    let clipped = if tail >= head_max - (head_max - head_min) * 1e-12 {
        after_head
    } else {
        after_head.split_at(tail)?.0
    };

    let (clipped_min, clipped_max) = clipped.param_range();
    let start = clipped.evaluate(clipped_min);
    let end = clipped.evaluate(clipped_max);
    if (start - end).norm() <= tol.linear {
        return None;
    }
    Some(Edge::new(
        clipped,
        Vertex::from_point(start),
        Vertex::from_point(end),
        tol.linear,
    ))
}

/// 鎖の**両端だけ**を、面のトリム境界まで切り詰める。
///
/// 鎖の内側の継ぎ目（相方と共有している点）は動かしません。1本も動かな
/// ければ `None`。
fn clip_chain_to_face_trim(face: &Face, chain: &[Edge], tol: &Tolerance) -> Option<Vec<Edge>> {
    // 端点が鎖の中で何回使われているか。2回なら継ぎ目、1回なら自由端。
    let shared = |point: Point3, skip: usize| {
        chain.iter().enumerate().any(|(index, other)| {
            index != skip
                && (points_same_3d(other.start_vertex.point, point, tol.linear)
                    || points_same_3d(other.end_vertex.point, point, tol.linear))
        })
    };

    let mut out = chain.to_vec();
    let mut moved = false;
    for index in 0..chain.len() {
        let edge = &chain[index];
        let clip_start = !shared(edge.start_vertex.point, index);
        let clip_end = !shared(edge.end_vertex.point, index);
        if !clip_start && !clip_end {
            continue;
        }
        if let Some(clipped) = clip_edge_ends_to_face_trim(face, edge, clip_start, clip_end, tol) {
            out[index] = clipped;
            moved = true;
        }
    }
    if moved {
        Some(out)
    } else {
        None
    }
}

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

    let Some((split_start, split_end)) = ruling_boundary_endpoints(
        split_edge,
        bounds.bottom_start,
        bounds.bottom_end,
        &patch,
        tol,
    ) else {
        if std::env::var_os("ZENITH_SPLIT_WHY").is_some() {
            let (a, b) = (split_edge.start_vertex.point, split_edge.end_vertex.point);
            eprintln!(
                "CYLWHY 端点が母線に乗らない: 交線 ({:.4} {:.4} {:.4})-({:.4} {:.4} {:.4})",
                a.x, a.y, a.z, b.x, b.y, b.z
            );
            eprintln!(
                "CYLWHY   面の隅 底 ({:.4} {:.4} {:.4})-({:.4} {:.4} {:.4}) 天 ({:.4} {:.4} {:.4})-({:.4} {:.4} {:.4})",
                bounds.bottom_start.x, bounds.bottom_start.y, bounds.bottom_start.z,
                bounds.bottom_end.x, bounds.bottom_end.y, bounds.bottom_end.z,
                bounds.top_start.x, bounds.top_start.y, bounds.top_start.z,
                bounds.top_end.x, bounds.top_end.y, bounds.top_end.z
            );
            eprintln!(
                "CYLWHY   交線の端点の 軸方向 {:.4} / {:.4}、半径 {:.4} / {:.4}（面の半径 {:.4}、高さ {:.4}）",
                patch.axial_coordinate(a),
                patch.axial_coordinate(b),
                patch.radial_distance(a),
                patch.radial_distance(b),
                patch.radius,
                patch.height
            );
        }
        return Err("Split edge endpoints do not match cylinder side boundaries".to_string());
    };

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
        orient_edge_for_points(&bounds.bottom_edge, bottom_start, bottom_end, tol).ok_or_else(
            || "Cylinder-side bottom arc does not match the face corners".to_string(),
        )?;
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

    // 巡回は `cylinder_face_bounds` が返す `bottom_start -> bottom_end` に
    // 従います。これは**元のワイヤがその円弧を辿った向き**なので、下の順序で
    // 組めば元の巡回がそのまま続きます。
    //
    // ここには以前、面の向きフラグが `Reversed` なら全体を逆順にする補正が
    // 入っていました。**あれは固有方向を読んでいたことへの埋め合わせ**で、
    // 独立して必要な処理ではありませんでした。巡回を正しく読むと二重補正に
    // なり、穴あきの板をスラブで削る側が直ると counterbore が壊れます
    // （両方を実測して確かめました）。片方を消して両方通ります。
    let orient_wire = Wire::new;

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

/// Splits a four-sided patch face along a section that crosses it.
///
/// This asks nothing about what the surface is. The section edge is a circle
/// about some axis - fitting it gives both - and from there the face's own
/// boundary sorts itself out: two of its edges have both ends at the same
/// distance along that axis and are the sections it already runs between, and
/// the other two carry it from one to the other.
///
/// Both pieces are built by walking the original wire in the order it is
/// already in, keeping each edge's direction as that wire uses it. Reading the
/// edges' own directions instead lets the two pieces come out wound opposite to
/// one another whenever an edge happens to be stored the other way round, and
/// the shell then fails to stitch with every shared edge used twice the same
/// way. The sides are cut where the section meets them rather than redrawn as
/// straight lines, which is right for a ruling and wrong for a torus's
/// meridian arcs.
fn split_patch_face_by_section_edge(
    face: &Face,
    surface: &NurbsSurface3,
    split_edge: &Edge,
    tol: &Tolerance,
) -> Result<Vec<Face>, String> {
    if !face.inner_wires.is_empty() {
        return Err("NURBS face splitting with inner wires is not implemented yet".to_string());
    }
    let boundary = &face.outer_wire.edges;
    if boundary.len() != 3 && boundary.len() != 4 {
        return Err("Only a three- or four-sided patch face can be split".to_string());
    }

    // 分割線が本当にこの面の上にあるか。曲面の種類を問わず、投影して測る。
    let scale = sampled_edge_extent(split_edge).max(1.0);
    for point in sample_curve_points(&split_edge.curve, 12) {
        zenith_geom::work_counter::count_section_split_projection();
        let projection = { ExtremumEngine::point_to_surface(point, surface, 32, tol.parametric) }
            .map_err(|err| format!("Section edge could not be projected: {err}"))?;
        if projection.distance > tol.linear * 10.0 * scale {
            return Err(format!(
                "Section edge leaves the face by {:.3e}",
                projection.distance
            ));
        }
    }

    // 分割線そのものが軸を教えてくれる。円として当てはめれば、法線が軸、
    // 中心が軸上の点。向きは断面の上下を呼び分けるだけに使う。
    let (center, _, normal) = fit_section_circle(&split_edge.curve, tol)
        .ok_or_else(|| "Section edge is not a circle about an axis".to_string())?;
    let axis = normal.ok_or_else(|| "Section edge has collapsed to a point".to_string())?;
    let axial = |point: Point3| (point - center).dot(&axis);

    let split_start = split_edge.start_vertex.point;
    let split_end = split_edge.end_vertex.point;
    if (axial(split_start) - axial(split_end)).abs() > tol.linear * 10.0 * scale {
        return Err("Section edge does not stay at one level".to_string());
    }
    let split_axial = 0.5 * (axial(split_start) + axial(split_end));

    // 境界の4辺のうち、両端の軸方向座標が等しいものが既存の断面。
    let mut section_indices = Vec::new();
    for (index, oriented) in boundary.iter().enumerate() {
        let (start_axial, end_axial) = (
            axial(oriented.start_vertex().point),
            axial(oriented.end_vertex().point),
        );
        if (start_axial - end_axial).abs() <= tol.linear * 10.0 * scale {
            section_indices.push((index, 0.5 * (start_axial + end_axial)));
        }
    }
    // 断面が1本しかない面は、反対側が一点に潰れている。球の極や円錐の頂点が
    // これで、境界は3辺になる。潰れた側には辺が現れないので、上下の片方は
    // その一点が受け持つ。
    if section_indices.len() == 1 && boundary.len() == 3 {
        return split_apex_patch_by_section_edge(
            face,
            split_edge,
            section_indices[0].0,
            &axial,
            scale,
            tol,
        );
    }
    if section_indices.len() != 2 || boundary.len() != 4 {
        return Err("Face boundary does not read as sections and sides".to_string());
    }
    // 巡回の上で断面と側辺が交互に並んでいなければ、四辺形として読めない。
    if (section_indices[1].0 + 4 - section_indices[0].0) % 4 != 2 {
        return Err("Face boundary sections are not opposite each other".to_string());
    }

    let (bottom_index, bottom_axial) = if section_indices[0].1 <= section_indices[1].1 {
        section_indices[0]
    } else {
        section_indices[1]
    };
    let top_axial = if section_indices[0].0 == bottom_index {
        section_indices[1].1
    } else {
        section_indices[0].1
    };

    let margin = tol.linear * scale;
    if split_axial <= bottom_axial + margin || split_axial >= top_axial - margin {
        return Err("Section edge must cross the face interior".to_string());
    }

    // 巡回の並び: 下の断面 -> 側辺A -> 上の断面 -> 側辺B。
    let bottom = boundary[bottom_index].clone();
    let side_a = boundary[(bottom_index + 1) % 4].clone();
    let top = boundary[(bottom_index + 2) % 4].clone();
    let side_b = boundary[(bottom_index + 3) % 4].clone();

    // 分割線の端点のどちらがどちらの側辺に乗るか。
    let (point_a, point_b) = if edge_reaches_point(&side_a, split_start, tol) {
        (split_start, split_end)
    } else if edge_reaches_point(&side_a, split_end, tol) {
        (split_end, split_start)
    } else {
        return Err("Section endpoints do not land on the face sides".to_string());
    };
    if !edge_reaches_point(&side_b, point_b, tol) {
        return Err("Section endpoints do not land on the face sides".to_string());
    }

    let (a_first, a_second) = cut_oriented_edge(&side_a, point_a, tol)
        .ok_or_else(|| "Could not cut the first side at the section".to_string())?;
    let (b_first, b_second) = cut_oriented_edge(&side_b, point_b, tol)
        .ok_or_else(|| "Could not cut the second side at the section".to_string())?;

    let split_a_to_b = orient_edge_for_points(split_edge, point_a, point_b, tol)
        .ok_or_else(|| "Section endpoints do not match the cut sides".to_string())?;
    let split_b_to_a = OrientedEdge::new(
        split_a_to_b.edge.clone(),
        split_a_to_b.orientation.reversed(),
    );

    let lower = Face::new(
        face.geometry.clone(),
        Wire::new(vec![bottom, a_first, split_a_to_b, b_second]),
        Vec::new(),
        face.orientation,
        face.tolerance,
    );
    let upper = Face::new(
        face.geometry.clone(),
        Wire::new(vec![a_second, top, b_first, split_b_to_a]),
        Vec::new(),
        face.orientation,
        face.tolerance,
    );

    for piece in [&lower, &upper] {
        if !piece.outer_wire.is_closed(tol) {
            return Err("Section split produced an open wire".to_string());
        }
        let report = piece.validate_pcurves(tol, 8)?;
        if !report.is_valid() {
            return Err(format!(
                "Section split p-curves are invalid with {} mismatches",
                report.mismatch_count
            ));
        }
    }

    Ok(vec![lower, upper])
}

/// Splits a patch whose far side has closed to a point.
///
/// A sphere's polar patch and a cone's tip are three-sided: one section, two
/// sides, and a corner where the sides meet. The section runs between one pair
/// of the sides' ends and the point takes the place of the other, so the piece
/// nearer the point stays three-sided while the other becomes four.
///
/// Both are cut out of the original cycle in the order it already uses, so the
/// winding carries over; the sides are cut where the section meets them rather
/// than redrawn.
fn split_apex_patch_by_section_edge(
    face: &Face,
    split_edge: &Edge,
    section_index: usize,
    axial: &dyn Fn(Point3) -> f64,
    scale: f64,
    tol: &Tolerance,
) -> Result<Vec<Face>, String> {
    let boundary = &face.outer_wire.edges;
    let section = boundary[section_index].clone();
    // 巡回は 断面 -> 断面の終点から頂点へ -> 頂点から断面の始点へ。
    let from_section = boundary[(section_index + 1) % 3].clone();
    let to_section = boundary[(section_index + 2) % 3].clone();

    let apex = from_section.end_vertex().point;
    if (apex - to_section.start_vertex().point).norm() > tol.linear * 10.0 {
        return Err("The two sides do not meet at a point".to_string());
    }

    let section_axial =
        0.5 * (axial(section.start_vertex().point) + axial(section.end_vertex().point));
    let apex_axial = axial(apex);
    let split_axial =
        0.5 * (axial(split_edge.start_vertex.point) + axial(split_edge.end_vertex.point));

    let margin = tol.linear * scale;
    let (low, high) = if section_axial <= apex_axial {
        (section_axial, apex_axial)
    } else {
        (apex_axial, section_axial)
    };
    if split_axial <= low + margin || split_axial >= high - margin {
        return Err("Section edge must cross the face interior".to_string());
    }

    let (point_from, point_to) =
        if edge_reaches_point(&from_section, split_edge.start_vertex.point, tol) {
            (split_edge.start_vertex.point, split_edge.end_vertex.point)
        } else if edge_reaches_point(&from_section, split_edge.end_vertex.point, tol) {
            (split_edge.end_vertex.point, split_edge.start_vertex.point)
        } else {
            return Err("Section endpoints do not land on the face sides".to_string());
        };
    if !edge_reaches_point(&to_section, point_to, tol) {
        return Err("Section endpoints do not land on the face sides".to_string());
    }

    // from_section は 断面の終点 -> 頂点。先に来るのが断面側の断片。
    let (near_section, near_apex) = cut_oriented_edge(&from_section, point_from, tol)
        .ok_or_else(|| "Could not cut the side leaving the section".to_string())?;
    // to_section は 頂点 -> 断面の始点。先に来るのが頂点側の断片。
    let (apex_side, section_side) = cut_oriented_edge(&to_section, point_to, tol)
        .ok_or_else(|| "Could not cut the side returning to the section".to_string())?;

    let split_from_to = orient_edge_for_points(split_edge, point_from, point_to, tol)
        .ok_or_else(|| "Section endpoints do not match the cut sides".to_string())?;
    let split_to_from = OrientedEdge::new(
        split_from_to.edge.clone(),
        split_from_to.orientation.reversed(),
    );

    let banded = Face::new(
        face.geometry.clone(),
        Wire::new(vec![section, near_section, split_from_to, section_side]),
        Vec::new(),
        face.orientation,
        face.tolerance,
    );
    let tipped = Face::new(
        face.geometry.clone(),
        Wire::new(vec![near_apex, apex_side, split_to_from]),
        Vec::new(),
        face.orientation,
        face.tolerance,
    );

    for piece in [&banded, &tipped] {
        if !piece.outer_wire.is_closed(tol) {
            return Err("Apex split produced an open wire".to_string());
        }
        let report = piece.validate_pcurves(tol, 8)?;
        if !report.is_valid() {
            return Err(format!(
                "Apex split p-curves are invalid with {} mismatches",
                report.mismatch_count
            ));
        }
    }

    Ok(vec![banded, tipped])
}

/// How far an edge reaches, as a length to scale tolerances against.
/// 面の輪から、長さ 0 の稜を取り除く。
///
/// 両端が同じ点にある稜は、境界に何も足しません。**取り除いても輪は
/// 繋がったまま**です（前後の稜はもともと同じ点で出会っています）。
/// 残しておくと、同じ稜が1回だけ、あるいは3回使われて、縫合が
/// 非多様体になります（4-128）。
///
/// **輪が2本未満になる場合は触りません。** そこまで潰れているなら、
/// それは輪ではなく別の欠陥なので、隠さずに残します。
/// 輪の中の「行って戻るだけ」を畳む。
///
/// # 何を落とすのか
///
/// 隣り合う2本が**同じ弧を逆向きに**辿っているとき、その2本は面積を1つも
/// 囲みません。**切れ目**です。落としても輪は閉じたままで、面の形も動きま
/// せん——出て戻るだけなので、前後の稜はもともと繋がっています。
///
/// # なぜ要るのか
///
/// 4-74 で「面積を囲まない**面片**」を落とすようにしました。それは片ごと
/// 丸ごとの話で、**面積を囲む片の中に切れ目が1本混ざっている**場合は残り
/// ます。実測（`cone × torus` の和。4-205）:
///
/// ```text
/// piece #10 (B) outer 7 edges
///   [3] (-2.800 9.600 0.000) -> (-6.000 0.000 0.000)
///   [4] (-6.000 0.000 0.000) -> (-2.800 9.600 0.000)   <- 戻っている
/// ```
///
/// この片は面積を囲むので 4-74 の判定には掛からず、**同じ稜を2回使う**まま
/// 縫合に回って非多様体になっていました（和が「未実装」として断られる）。
///
/// 3-1 の規約——**接触は、それ自体では位相を作らない**——がそのまま実装に
/// なります。
fn collapse_there_and_back(face: &mut Face, tol: &Tolerance) {
    let fold = |wire: &mut zenith_topo::Wire| {
        // 1周のうちに複数あることがあるので、変化が無くなるまで回します。
        loop {
            let count = wire.edges.len();
            if count < 2 {
                return;
            }
            let mut dropped = None;
            for index in 0..count {
                let next = (index + 1) % count;
                let here = &wire.edges[index];
                let after = &wire.edges[next];
                // **向きの印でも、媒介変数でも見分けられません。**
                //
                // 割った結果は両側が別々の `Edge` の実体を持ちます。向きの印は
                // 揃っているとは限らず（実測では `Forward` と `Reversed`）、
                // 曲線の媒介変数の取り方も違うので、同じ t で標本を突き合わせる
                // `same_edge_geometry` は**同じ弧でも外します**（実測: 中点は
                // 小数4桁まで一致しているのに一致しないと判定されました）。
                //
                // 端点が入れ替わっていて、中点が同じなら、同じ弧を逆に辿って
                // います。縫合の突き合わせ（`same_undirected_stitch_edge`）と
                // 同じ見方です。
                if !points_same_3d(
                    here.end_vertex().point,
                    after.start_vertex().point,
                    tol.linear,
                ) || !points_same_3d(
                    here.start_vertex().point,
                    after.end_vertex().point,
                    tol.linear,
                ) {
                    continue;
                }
                let middle_here = edge_midpoint(&here.edge);
                let middle_after = edge_midpoint(&after.edge);
                if !points_same_3d(middle_here, middle_after, tol.linear * 10.0) {
                    continue;
                }
                dropped = Some((index, next));
                break;
            }
            let Some((first, second)) = dropped else {
                return;
            };
            // 2本落として輪が持たなくなるなら、触りません。面積 0 の片は
            // 別の判定（4-74）が落とします。
            if count < 4 {
                return;
            }
            let mut kept = Vec::with_capacity(count - 2);
            for (index, oriented) in wire.edges.iter().enumerate() {
                if index == first || index == second {
                    continue;
                }
                kept.push(oriented.clone());
            }
            wire.edges = kept;
        }
    };
    fold(&mut face.outer_wire);
    for wire in &mut face.inner_wires {
        fold(wire);
    }
}

fn remove_degenerate_wire_edges(face: &mut Face, tol: &Tolerance) {
    let prune = |wire: &mut zenith_topo::Wire| {
        let kept: Vec<_> = wire
            .edges
            .iter()
            .filter(|oriented| sampled_edge_extent(&oriented.edge) > tol.linear)
            .cloned()
            .collect();
        if kept.len() >= 2 && kept.len() < wire.edges.len() {
            wire.edges = kept;
        }
    };
    prune(&mut face.outer_wire);
    for wire in &mut face.inner_wires {
        prune(wire);
    }
}

fn sampled_edge_extent(edge: &Edge) -> f64 {
    let samples = sample_curve_points(&edge.curve, 8);
    let origin = samples[0];
    samples
        .iter()
        .fold(0.0f64, |worst, sample| worst.max((sample - origin).norm()))
}

/// Whether a point lies on an edge, anywhere along it.
fn edge_reaches_point(oriented: &OrientedEdge, point: Point3, tol: &Tolerance) -> bool {
    let scale = sampled_edge_extent(&oriented.edge).max(1.0);
    ExtremumEngine::point_to_curve(point, &oriented.edge.curve, 64, tol.parametric)
        .map(|projection| projection.distance <= tol.linear * 10.0 * scale)
        .unwrap_or(false)
}

/// Cuts an edge use at a point on it, into the part reached first and the part
/// reached second, both facing the way the wire already walks them.
fn cut_oriented_edge(
    oriented: &OrientedEdge,
    point: Point3,
    tol: &Tolerance,
) -> Option<(OrientedEdge, OrientedEdge)> {
    let projection =
        ExtremumEngine::point_to_curve(point, &oriented.edge.curve, 64, tol.parametric).ok()?;
    let scale = sampled_edge_extent(&oriented.edge).max(1.0);
    if projection.distance > tol.linear * 10.0 * scale {
        return None;
    }

    let (first_half, second_half) = oriented.edge.curve.split_bezier_at(projection.parameter)?;
    let middle = Vertex::new(point, tol.linear);
    let lower = Edge::new(
        first_half,
        oriented.edge.start_vertex.clone(),
        middle.clone(),
        tol.linear,
    );
    let upper = Edge::new(
        second_half,
        middle,
        oriented.edge.end_vertex.clone(),
        tol.linear,
    );

    if oriented.orientation.is_forward() {
        Some((OrientedEdge::forward(lower), OrientedEdge::forward(upper)))
    } else {
        // 逆向きに使われている辺では、先に通るのは曲線の後半のほう。
        Some((OrientedEdge::reversed(upper), OrientedEdge::reversed(lower)))
    }
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
    //
    // **ワイヤがその辺をどちら向きに巡回しているかを一緒に持ちます。**
    // 以前は辺の固有方向だけを見ていました。ワイヤが逆向きに使っている辺では
    // 分割後の巡回が反転し、隣の無傷な面と同じ向きで辺を共有します。実測では
    // 穴あきの板をスラブで削ると、穴の内壁だけが反転して同方向の辺使用が
    // 16 残り、組み立てが止まりました。4-5 が別の経路で直したのと同じ罠です。
    let mut arcs: Vec<(&Edge, f64, bool)> = Vec::new();
    for oriented in &face.outer_wire.edges {
        let edge = &oriented.edge;
        let start_axial = patch.axial_coordinate(edge.start_vertex.point);
        let end_axial = patch.axial_coordinate(edge.end_vertex.point);
        if (start_axial - end_axial).abs() <= tol.linear * 10.0 {
            arcs.push((
                edge,
                0.5 * (start_axial + end_axial),
                oriented.orientation.is_forward(),
            ));
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

    // 巡回の向きに合わせて始点と終点を取る。
    let (bottom_start, bottom_end) = if bottom.2 {
        (bottom.0.start_vertex.point, bottom.0.end_vertex.point)
    } else {
        (bottom.0.end_vertex.point, bottom.0.start_vertex.point)
    };

    // 上側の円弧の端点を、下側と同じルーリングに乗るように対応づける。
    let on_same_ruling = |point: Point3, base: Point3| patch.on_same_ruling(point, base, tol);

    let (top_start, top_end) = {
        let candidate_start = top.0.start_vertex.point;
        let candidate_end = top.0.end_vertex.point;
        if on_same_ruling(candidate_start, bottom_start)
            && on_same_ruling(candidate_end, bottom_end)
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
    let on_ruling =
        |point: Point3, ruling_base: Point3| patch.on_same_ruling(point, ruling_base, tol);

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

    let left = hold_piece_like_its_face(
        Face::new(
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
        ),
        tol,
    );
    let right = hold_piece_like_its_face(
        Face::new(
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
        ),
        tol,
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
            && (patch.radial_distance(point) - patch.radius_at(axial)).abs() <= tol.linear * 10.0
    })
}

/// メッシュの広がり（対角）。**形の大きさに合わせた物差しが要る**ところで
/// 使います。
fn mesh_extent(mesh: &TriangleMesh) -> f64 {
    let mut low = Point3::new(f64::MAX, f64::MAX, f64::MAX);
    let mut high = Point3::new(f64::MIN, f64::MIN, f64::MIN);
    for point in &mesh.positions {
        low.x = low.x.min(point.x);
        low.y = low.y.min(point.y);
        low.z = low.z.min(point.z);
        high.x = high.x.max(point.x);
        high.y = high.y.max(point.y);
        high.z = high.z.max(point.z);
    }
    if low.x > high.x {
        return 0.0;
    }
    (high - low).norm()
}

fn classify_face_against_mesh(
    face: &Face,
    mesh: &TriangleMesh,
    other: Option<&Solid>,
    tol: &Tolerance,
) -> FaceRegionLocation {
    let sample = representative_face_point(face);
    // **メッシュの弦誤差では、曲面の重なりが見えません。**
    //
    // 「相手の境界の上にあるか」をメッシュとの距離だけで見ていました。
    // 平面ならメッシュは厳密なので効きますが、**曲面ではメッシュが弦で
    // 近似されている**ので、面の上の点でも 1e-4 より離れます。
    //
    // 実測（4-134）: まったく同じトーラス2つの和で、**16面のどれも
    // `Boundary` と判定されません**。外側の片は `Outside`、内側の片は
    // `Inside` になり、和で半分だけが採られて縫合が開きます。球・円柱・
    // 円錐も同じで、**箱だけが通っていました**（平面はメッシュが厳密）。
    //
    // メッシュで決まらないときだけ、B-Rep の面へ厳密に当てます。**遠い面
    // では走りません**——メッシュとの距離が形の 1% より遠ければ、そこは
    // 相手の境界ではありません。
    let scale = mesh_extent(mesh).max(1.0);
    let near = |point: Point3| {
        let to_mesh = point_mesh_distance(point, mesh);
        if to_mesh <= tol.linear * 100.0 {
            return true;
        }
        if to_mesh > scale * 1e-2 {
            return false;
        }
        let Some(solid) = other else {
            return false;
        };
        // **近いものが1つあれば足ります**（4-294）。全部の面へ全域射影して
        // 集めるのは高すぎます——実測でここが律速でした。
        crate::distance::has_boundary_within(point, solid, tol.linear * 100.0)
    };
    if near(sample) {
        // **1点で触れていることは、重なっていることではありません。**
        //
        // 代表点は面の中でいちばん縁から遠い点なので、**接触点にちょうど
        // 落ちることがあります**。実測（`box × cone (apex on a face)`）:
        // 円錐の頂点 (10,10,20) が箱の上面の中心そのもので、上面 1枚が
        // `Boundary` と判定されて積に採られ、その4辺があぶれていました。
        // 真の積は円錐なので、上面は採ってはいけません。
        //
        // 重なっているなら、**面の中のどこを取っても相手の表面の近くに
        // ある**はずです。散らして取り直して、そうなっていなければ
        // `Boundary` を取り下げます。
        //
        // **下げる方向にしか効かせません。** 最初の点が相手から離れて
        // いるときは、この検査を通しません——`Inside` / `Outside` を
        // 新たに `Boundary` に変えることはないので、いま通っている経路の
        // 判定は動きません。
        let spread = spread_face_points(face, 9);
        if spread.len() >= 4 {
            let near_count = spread.iter().filter(|point| near(**point)).count();
            if near_count * 2 < spread.len() {
                let off: Vec<Point3> = spread.into_iter().filter(|point| !near(*point)).collect();
                let inside = off
                    .iter()
                    .filter(|point| crate::BooleanEngine::is_point_inside_mesh(**point, mesh))
                    .count();
                return if inside * 2 > off.len() {
                    FaceRegionLocation::Inside
                } else {
                    FaceRegionLocation::Outside
                };
            }
        }
        return FaceRegionLocation::Boundary;
    }
    // **内外を多数決にする、を試して戻しました**（4-166）。
    //
    // ここは代表点1つで決めています。1点が動けば裏返るので、散らした点の
    // 多数決にすれば頑健になるはず——と考えて入れ、測りました。
    //
    // **揺れは止まりませんでした。** 射影の詰めを 8段 → 9段 に増やすと、
    // 多数決を入れても入れなくても `contact_placement_probe` は
    // **81 ok → 78 ok** になります。仕事だけが 2% 増えました
    // （30,894,913 → 31,527,229）。
    //
    // **揺れているのは分類ではありません。** 断り文を読むと、新たに
    // 断られる3演算は `sphere × cylinder (eccentric)` で、理由は
    // 「まだ実装していない」——**組み立てが閉じない**ほうです。射影が
    // 変わると**接点で終わる交線の端**がわずかに動き、A 側と B 側が
    // 突き合わなくなります（4-128 で直したのと同じ機構）。
    //
    // **直す場所は接点の着地のほうです。** ここではありません。
    if crate::BooleanEngine::is_point_inside_mesh(sample, mesh) {
        FaceRegionLocation::Inside
    } else {
        FaceRegionLocation::Outside
    }
}

/// 面の中の点を、**散らして**取る。
///
/// `representative_face_point` は「縁からいちばん遠い1点」なので、面が
/// 相手と面積をもって重なっているのか、1点で触れているだけなのかを
/// 区別できません。ここは同じトリム領域から、**離れた場所の点**を
/// 集めます。穴の中と縁の上は避けます（そこは面の材料ではありません）。
fn spread_face_points(face: &Face, want: usize) -> Vec<Point3> {
    if want == 0 {
        return Vec::new();
    }

    let pick_spread = |mut all: Vec<Point3>| -> Vec<Point3> {
        if all.len() <= want {
            return all;
        }
        // 端を含めつつ等間隔に間引く。固まった点ばかり取ると、
        // 「散らして取った」ことになりません。
        let stride = all.len() as f64 / want as f64;
        let picked: Vec<Point3> = (0..want)
            .map(|index| all[((index as f64 + 0.5) * stride) as usize % all.len()])
            .collect();
        all.clear();
        picked
    };

    if let FaceGeometry::Plane(plane) = &face.geometry {
        const GRID: usize = 12;
        let outer: Vec<Point2> = face
            .outer_wire
            .sample_points(8)
            .iter()
            .map(|point| project_to_plane_uv(*point, plane))
            .collect();
        if outer.len() >= 3 {
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
            let span = (max_u - min_u).max(max_v - min_v);
            // 縁のすぐ内側は取りません。トリムの標本誤差で外に出ます。
            let margin = span * 1e-3;
            let mut inside_points = Vec::new();
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
                    if clearance <= margin {
                        continue;
                    }
                    inside_points.push(plane.evaluate(uv.x, uv.y));
                }
            }
            if inside_points.len() >= 4 {
                return pick_spread(inside_points);
            }
        }
        return Vec::new();
    }

    if let FaceGeometry::Nurbs(surface) = &face.geometry {
        // **三角化しません**（4-161）。p-curve の多角形から直接散らします。
        if let Some((outer, holes)) = pcurve_uv_polygons(face) {
            let picked = uv_points_clear_of_holes(&outer, &holes, want);
            if picked.len() >= 4 {
                return picked
                    .into_iter()
                    .map(|uv| surface.evaluate(uv.x, uv.y))
                    .collect();
            }
        }
        // 多角形から取れなかったときの受け皿。**点を選ぶだけなので、
        // 細分は掛けません**（4-160）。
        let domain = zenith_tess::face_uv_triangulation_for_point_picking(
            face,
            &zenith_tess::TessellationParams::default(),
        );
        let mut by_area: Vec<(f64, Point3)> = domain
            .triangles
            .iter()
            .filter_map(|triangle| {
                let a = domain.uvs[triangle[0]];
                let b = domain.uvs[triangle[1]];
                let c = domain.uvs[triangle[2]];
                let area = 0.5 * ((b.x - a.x) * (c.y - a.y) - (b.y - a.y) * (c.x - a.x)).abs();
                if area <= 0.0 {
                    return None;
                }
                Some((
                    area,
                    surface.evaluate((a.x + b.x + c.x) / 3.0, (a.y + b.y + c.y) / 3.0),
                ))
            })
            .collect();
        // 大きい三角形から取ります。潰れた三角形の重心は面の内側とは
        // 限りません。
        by_area.sort_by(|left, right| right.0.total_cmp(&left.0));
        let points: Vec<Point3> = by_area.into_iter().map(|(_, point)| point).collect();
        if points.len() >= 4 {
            return points.into_iter().take(want).collect();
        }
        return Vec::new();
    }

    Vec::new()
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

    // **座標で待ち伏せる口**（`ZENITH_STITCH_WATCH="x,y,z"`。4-294）。
    //
    // 「相手のいない稜」だけを出しても、**その場所に誰がいるのか**が分かり
    // ません。指定した点のそばを通る稜を、**合っているものも含めて**全部
    // 出します。口の円に壁の面が来ているか、といった問いはこれで決まります。
    if let Some(watch) = std::env::var("ZENITH_STITCH_WATCH").ok().and_then(|value| {
        let parts: Vec<f64> = value
            .split(',')
            .filter_map(|part| part.trim().parse().ok())
            .collect();
        (parts.len() == 3).then(|| Point3::new(parts[0], parts[1], parts[2]))
    }) {
        for use_ in edge_uses.iter() {
            let near = [use_.start, use_.end, use_.middle]
                .iter()
                .map(|point| (point - watch).norm())
                .fold(f64::INFINITY, f64::min);
            if near > 0.6 {
                continue;
            }
            eprintln!(
                "STITCHWATCH {:?} 面 {} ({:.6} {:.6} {:.6}) -> ({:.6} {:.6} {:.6}) mid ({:.6} {:.6} {:.6}) 近さ {near:.6}",
                use_.operand,
                use_.face_id,
                use_.start.x, use_.start.y, use_.start.z,
                use_.end.x, use_.end.y, use_.end.z,
                use_.middle.x, use_.middle.y, use_.middle.z
            );
        }
    }

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
            0 => {
                unmatched_edge_use_count += 1;
                // **どの稜が余ったのかを出す口。** 数だけでは、面片が
                // 足りないのか、突き合わせ方が違うのかが分かりません。
                // `ZENITH_SPLIT_WHY` と同じ流儀です。
                //
                // **いちばん近い他の稜使用も出します**（4-186）。相手が
                // 「少しずれた所に居る」のか「そもそも居ない」のかで、
                // 直す先が変わります——前者は位置の話、後者は選び方の話
                // です。距離は中点どうしで測ります（端点だけでは、同じ2点を
                // 結ぶ別の弧と見分けられません。4-65）。
                if std::env::var_os("ZENITH_STITCH_WHY").is_some() {
                    let use_ = &edge_uses[i];
                    let nearest = edge_uses
                        .iter()
                        .enumerate()
                        .filter(|(j, _)| *j != i)
                        .map(|(_, other)| ((other.middle - use_.middle).norm(), other))
                        .min_by(|left, right| left.0.total_cmp(&right.0));
                    if let Some((distance, other)) = nearest {
                        let length = (other.end - other.start).norm();
                        eprintln!(
                            "STITCHWHY   いちばん近い相手 {:?} 中点まで {distance:.9} 長さ {length:.9}",
                            other.operand
                        );
                        // **同じ線の上に乗っているのか**（4-304）。中点が
                        // 一致していても、それだけでは「相手の弦の真ん中に
                        // 居る」としか言えません。**A の端点が相手の弦から
                        // どれだけ離れているか**を測れば、「割り方の食い違い」
                        // （＝同じ線の上に乗っている）と「別の線」（＝乗って
                        // いない）が分かれます。
                        let to_segment = |point: Point3| {
                            let direction = other.end - other.start;
                            let squared = direction.norm_squared();
                            if squared <= 0.0 {
                                return (point - other.start).norm();
                            }
                            let t = ((point - other.start).dot(&direction) / squared)
                                .clamp(0.0, 1.0);
                            (point - (other.start + direction * t)).norm()
                        };
                        eprintln!(
                            "STITCHWHY   その相手の弦まで: 始点 {:.9} 終点 {:.9} 中点 {:.9}",
                            to_segment(use_.start),
                            to_segment(use_.end),
                            to_segment(use_.middle)
                        );
                        eprintln!(
                            "STITCHWHY   その相手 面 {} ({:.9} {:.9} {:.9}) -> ({:.9} {:.9} {:.9})",
                            other.face_id,
                            other.start.x,
                            other.start.y,
                            other.start.z,
                            other.end.x,
                            other.end.y,
                            other.end.z
                        );
                    }
                    // **端点に何本つながっているか**（4-292）。
                    //
                    // 閉じた殻では、どの頂点にも少なくとも2本の稜が集まります。
                    // **1本しか来ていない端**は、そこで割られた側と割られて
                    // いない側が食い違っている印（T 字の頂点）です。相手までの
                    // 距離だけでは、「位置がずれている」のか「そもそも割られて
                    // いない」のかが分かりません。
                    let touching = |point: Point3| {
                        edge_uses
                            .iter()
                            .enumerate()
                            .filter(|(j, other)| {
                                *j != i
                                    && (points_same_3d(other.start, point, tol.linear)
                                        || points_same_3d(other.end, point, tol.linear))
                            })
                            .count()
                    };
                    eprintln!(
                        "STITCHWHY   端につながる他の稜: 始点 {} 本、終点 {} 本",
                        touching(use_.start),
                        touching(use_.end)
                    );
                    eprintln!(
                        "STITCHWHY unmatched {:?} 面 {} ({:.9} {:.9} {:.9}) -> ({:.9} {:.9} {:.9}) mid ({:.9} {:.9} {:.9}) len {:.9}",
                        use_.operand,
                        use_.face_id,
                        use_.start.x,
                        use_.start.y,
                        use_.start.z,
                        use_.end.x,
                        use_.end.y,
                        use_.end.z,
                        use_.middle.x,
                        use_.middle.y,
                        use_.middle.z,
                        (use_.end - use_.start).norm()
                    );
                }
            }
            1 => {
                let mate = mates[0];
                if i < mate {
                    matched_edge_pair_count += 1;
                }
                if !opposite_stitch_edge_direction(&edge_uses[i], &edge_uses[mate], tol.linear) {
                    same_direction_edge_use_count += 1;
                    // **「同方向」は閉じた多様体では起こりえません。**
                    // 数だけでは、どの面のどの稜なのかが分かりません。
                    if i < mate && std::env::var_os("ZENITH_STITCH_WHY").is_some() {
                        let use_a = &edge_uses[i];
                        eprintln!(
                            "STITCHWHY same-direction ({:.6} {:.6} {:.6}) -> ({:.6} {:.6} {:.6}) mid ({:.6} {:.6} {:.6})",
                            use_a.start.x, use_a.start.y, use_a.start.z,
                            use_a.end.x, use_a.end.y, use_a.end.z,
                            use_a.middle.x, use_a.middle.y, use_a.middle.z
                        );
                    }
                }
            }
            count => {
                non_manifold_edge_use_count += 1;
                // **何回使われている稜なのかを出す口。** 「非多様体」とだけ
                // 言われても、3回なのか4回なのかで話が違います。4回なら、
                // **本来は稜でないものを稜にしている**疑いがあります
                // （接する線がまさにそれ。HANDOVER 3-1「接触は、それ自体
                // では位相を作らない」）。
                if std::env::var_os("ZENITH_STITCH_WHY").is_some() {
                    let use_ = &edge_uses[i];
                    eprintln!(
                        "STITCHWHY non-manifold x{} {:?} ({:.9} {:.9} {:.9}) -> ({:.9} {:.9} {:.9}) mid ({:.9} {:.9} {:.9})",
                        count + 1,
                        use_.operand,
                        use_.start.x,
                        use_.start.y,
                        use_.start.z,
                        use_.end.x,
                        use_.end.y,
                        use_.end.z,
                        use_.middle.x,
                        use_.middle.y,
                        use_.middle.z
                    );
                }
            }
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
    // **選ばれた面片を出す口**（4-187）。
    //
    // あぶれた稜の相手が居ないとき、原因は「割れていない」か「割れたが
    // 選ばれていない」かのどちらかです。**稜の側からは区別できません。**
    // 面片の側で、どこの面が何枚残ったかを見ます。
    //
    // `SelectedBooleanFacePiece` は元の面の番号を持っていないので、
    // **輪の重心**で見分けます（平面の壁ならこれで足ります）。
    if std::env::var_os("ZENITH_PIECE_WHY").is_some() {
        eprintln!("PIECEWHY ==== 面片 {} 枚 ====", pieces.len());
        for piece in pieces {
            let points: Vec<Point3> = piece
                .face
                .outer_wire
                .edges
                .iter()
                .map(|oriented| oriented.edge.start_vertex.point)
                .collect();
            if points.is_empty() {
                continue;
            }
            let mut centre = Vec3::zeros();
            for point in &points {
                centre += point.coords;
            }
            centre /= points.len() as f64;
            eprintln!(
                "PIECEWHY {:?} {:?} 重心 ({:.4} {:.4} {:.4}) 稜 {}",
                piece.operand,
                piece.location,
                centre.x,
                centre.y,
                centre.z,
                piece.face.outer_wire.edges.len()
            );
        }
    }
    let mut edge_uses = Vec::new();
    for piece in pieces {
        collect_wire_stitch_edge_uses(
            &piece.face.outer_wire,
            piece.face.id,
            piece.reverse_orientation,
            piece.operand,
            &mut edge_uses,
        );
        for wire in &piece.face.inner_wires {
            collect_wire_stitch_edge_uses(
                wire,
                piece.face.id,
                piece.reverse_orientation,
                piece.operand,
                &mut edge_uses,
            );
        }
    }

    edge_uses
}

fn collect_wire_stitch_edge_uses(
    wire: &Wire,
    face_id: u64,
    reverse_orientation: bool,
    operand: BooleanOperand,
    edge_uses: &mut Vec<StitchEdgeUse>,
) {
    for edge in &wire.edges {
        let start = edge.start_vertex().point;
        let end = edge.end_vertex().point;
        // 稜の途中の点。向きに依らないので、反転しても同じものを使います。
        let middle = edge.evaluate_normalized(0.5);
        if reverse_orientation {
            edge_uses.push(StitchEdgeUse {
                face_id,
                start: end,
                end: start,
                middle,
                operand,
            });
        } else {
            edge_uses.push(StitchEdgeUse {
                face_id,
                start,
                end,
                middle,
                operand,
            });
        }
    }
}

/// 面を辺で繋いで、離れた塊ごとに添字をまとめる。
///
/// 辺の同一性は端点の座標で見ます。分割された面は同じ稜を別々の `Edge` として
/// 持つことがあり（`unify_coincident_edges` が一本化するのはこの後です）、
/// 実体で繋ぐと全部が離れて見えます。
/// 近い点を1つの代表へ寄せる。**丸めと違って、升の境の位置に依りません。**
///
/// 升目で引いてから、**まわりの升も見て**公差の内側かを測ります。見つから
/// なければ新しい代表になります。
struct PointWelder {
    grid: f64,
    limit: f64,
    cells: BTreeMap<(i64, i64, i64), Vec<usize>>,
    points: Vec<Point3>,
}

impl PointWelder {
    fn new(grid: f64, limit: f64) -> Self {
        Self {
            grid,
            limit,
            cells: BTreeMap::new(),
            points: Vec::new(),
        }
    }

    fn cell(&self, point: Point3) -> (i64, i64, i64) {
        (
            (point.x / self.grid).floor() as i64,
            (point.y / self.grid).floor() as i64,
            (point.z / self.grid).floor() as i64,
        )
    }

    fn representative(&mut self, point: Point3) -> usize {
        let (cx, cy, cz) = self.cell(point);
        for dx in -1..=1 {
            for dy in -1..=1 {
                for dz in -1..=1 {
                    let Some(bucket) = self.cells.get(&(cx + dx, cy + dy, cz + dz)) else {
                        continue;
                    };
                    for index in bucket {
                        if (self.points[*index] - point).norm() <= self.limit {
                            return *index;
                        }
                    }
                }
            }
        }
        let index = self.points.len();
        self.points.push(point);
        self.cells.entry((cx, cy, cz)).or_default().push(index);
        index
    }

    fn point(&self, index: usize) -> Point3 {
        self.points[index]
    }
}

fn connected_piece_groups(pieces: &[SelectedBooleanFacePiece], tol: &Tolerance) -> Vec<Vec<usize>> {
    let groups = connected_piece_groups_inner(pieces, tol);
    if std::env::var_os("ZENITH_PIECE_WHY").is_some() {
        let sizes: Vec<String> = groups.iter().map(|g| g.len().to_string()).collect();
        eprintln!(
            "PIECEWHY 塊 {} 個: {} 枚（合計 {}）",
            groups.len(),
            sizes.join(" / "),
            pieces.len()
        );
    }
    groups
}

fn connected_piece_groups_inner(
    pieces: &[SelectedBooleanFacePiece],
    tol: &Tolerance,
) -> Vec<Vec<usize>> {
    let grid = tol.linear.max(1e-9);
    // **丸めは公差ではありません**（4-188）。
    //
    // ここは長らく「座標を升目の番号に丸めて、同じ番号なら同じ点」と
    // していました。**升の境をまたぐと、いくら近くても別物になります。**
    //
    // 実測（箱の上面とトーラスの上端が接する和）: 同じ弦を、天面の側は
    //
    // ```text
    // (0, 3.366750, 20) - (0, 16.633250, 20)
    // ```
    //
    // 壁の側は
    //
    // ```text
    // (0, 3.366751, 20) - (0, 16.633249, 20)
    // ```
    //
    // と持っていました。真値は `10 ± √44`（3.3667504 / 16.6332496）で、
    // 2つの版の差は **1.6e-7**——公差 `1e-6` の 1/6 です。それでも
    // `.5` の境をまたいだので、**升の番号が 1 ずれました**。
    //
    // 結果、42枚の面片が**9つの島**に割れ（30 / 1×4 / 2×4）、本体だけを
    // 立体にしようとして「12本があぶれている」と断っていました。
    //
    // **溶接します。** 近くの升も見て、公差の内側にある点は同じ代表へ
    // 寄せます。丸めと違って、境の位置に依りません。
    let welder = PointWelder::new(grid, tol.linear);
    let mut welder = welder;
    let mut key = |point: Point3| welder.representative(point);

    // **端点だけでは、別々の稜が同じものに見えます。**
    //
    // 球の経線はどれも極から極へ走るので、**8本すべてが同じ端点の対**を
    // 持ちます。端点だけで照合すると、経度の違う稜が全部1本と数えられ、
    // 別々の閉じた曲面が「繋がっている」ことになります。
    //
    // 実測（4-137）: 半径 5 の球を z 軸まわりに 30 度回して和を取ると、
    // **16面（8 + 8）が1つの立体として返り**、体積が 1047.197551 と
    // ちょうど2倍になります。**稜はどれもちょうど2回使われるので、
    // `validate_closed` も「妥当な閉シェル」と答えます**——中身は
    // 二重被覆です。
    //
    // 中点を鍵に足せば、経度の違う稜は別物になります。
    let mut users: BTreeMap<(usize, usize, usize), Vec<usize>> = BTreeMap::new();
    for (index, piece) in pieces.iter().enumerate() {
        let face = &piece.face;
        for wire in std::iter::once(&face.outer_wire).chain(face.inner_wires.iter()) {
            for edge in &wire.edges {
                let (start, end) = edge.edge.curve.param_range();
                let a = key(edge.edge.curve.evaluate(start));
                let b = key(edge.edge.curve.evaluate(end));
                let middle = key(edge.edge.curve.evaluate((start + end) * 0.5));
                let pair = if a <= b { (a, b) } else { (b, a) };
                users
                    .entry((pair.0, pair.1, middle))
                    .or_default()
                    .push(index);
            }
        }
    }

    // **繋がらなかった稜を出す口**（4-188）。孤立した面片が、どの稜で
    // 本体に繋がるはずだったのかを見ます。
    if std::env::var_os("ZENITH_PIECE_WHY").is_some() {
        for (ids, sharing) in users.iter() {
            if sharing.len() == 1 {
                let point = |id: usize| welder.point(id);
                let (a, b, m) = (point(ids.0), point(ids.1), point(ids.2));
                eprintln!(
                    "PIECEWHY   相手のいない稜 面片{} ({:.7} {:.7} {:.7})-({:.7} {:.7} {:.7}) 中点 ({:.7} {:.7} {:.7})",
                    sharing[0], a.x, a.y, a.z, b.x, b.y, b.z, m.x, m.y, m.z
                );
            }
        }
    }

    let mut neighbours: Vec<Vec<usize>> = vec![Vec::new(); pieces.len()];
    for sharing in users.values() {
        for (position, left) in sharing.iter().enumerate() {
            for right in sharing.iter().skip(position + 1) {
                if left != right {
                    neighbours[*left].push(*right);
                    neighbours[*right].push(*left);
                }
            }
        }
    }

    let mut seen = vec![false; pieces.len()];
    let mut groups = Vec::new();
    for start in 0..pieces.len() {
        if seen[start] {
            continue;
        }
        let mut group = Vec::new();
        let mut stack = vec![start];
        seen[start] = true;
        while let Some(index) = stack.pop() {
            group.push(index);
            for next in &neighbours[index] {
                if !seen[*next] {
                    seen[*next] = true;
                    stack.push(*next);
                }
            }
        }
        group.sort_unstable();
        groups.push(group);
    }
    groups
}

pub(crate) fn all_solid_faces(solid: &Solid) -> Vec<Face> {
    let mut faces = solid.outer_shell.faces.clone();
    for inner in &solid.inner_shells {
        faces.extend(inner.faces.clone());
    }
    faces
}

/// `all_solid_faces` と同じ並びで、**その面が内側シェル（空洞）から来たか**。
///
/// **並びに潰すと、どちらのシェルから来たかが失われます。** 空洞の壁は、
/// この立体では**実効法線が材料の中を向いています**（外側シェルは外向き。
/// 2026/08/28 実測、4-144）。切り手が空洞を貫くと空洞の壁が外側の境界に
/// 繋がるので、**そのままでは巻きが食い違います**。
pub(crate) fn face_comes_from_inner_shell(solid: &Solid) -> Vec<bool> {
    let mut flags = vec![false; solid.outer_shell.faces.len()];
    for inner in &solid.inner_shells {
        flags.extend(std::iter::repeat_n(true, inner.faces.len()));
    }
    flags
}

fn nest_cavity_shells_into_solids(simple_solids: Vec<Solid>, _tol: &Tolerance) -> Vec<Solid> {
    if simple_solids.len() <= 1 {
        return simple_solids;
    }

    let params = TessellationParams::default();
    // **体積は先に1回ずつ measure してから並べます。**
    //
    // 比較関数の中で積んでいたので、`sort_by` が比較するたびに立体を丸ごと
    // 積分し直していました（比較は O(n log n) 回あります）。1回の積分は面を
    // 全部三角形に割って6点則を当てるので、並べ替えの代金としては桁違いです。
    // 順序は同じです。
    let mut with_volume: Vec<(f64, Solid)> = simple_solids
        .into_iter()
        .map(|solid| {
            let volume = MassCalculator::compute_volume_from_brep(&solid, &params);
            (volume, solid)
        })
        .collect();
    // **大きさで並べます。符号ではありません。**
    //
    // 空洞になる塊は**内向き**で出てくることがあり（差の B 側は面を反転して
    // 採るので）、そのとき符号つき体積は負になります。符号で並べると、
    // 内向きの塊が「いちばん小さい」ことになって、入れ子の親子が
    // 取り違えられます。大きさで並べれば向きに依存しません。
    with_volume.sort_by(|(a, _), (b, _)| {
        b.abs()
            .partial_cmp(&a.abs())
            .unwrap_or(std::cmp::Ordering::Equal)
    });
    let shell_volumes: Vec<f64> = with_volume.iter().map(|(volume, _)| *volume).collect();
    let simple_solids: Vec<Solid> = with_volume.into_iter().map(|(_, solid)| solid).collect();

    let meshes: Vec<TriangleMesh> = simple_solids
        .iter()
        .map(|s| tessellate_solid(s, &params))
        .collect();

    let mut contained_in: Vec<Option<usize>> = vec![None; simple_solids.len()];

    for j in 0..simple_solids.len() {
        if simple_solids[j].outer_shell.faces.is_empty() {
            continue;
        }
        let sample = representative_face_point(&simple_solids[j].outer_shell.faces[0]);
        for i in 0..j {
            if crate::BooleanEngine::is_point_inside_mesh(sample, &meshes[i]) {
                contained_in[j] = Some(i);
                break;
            }
        }
    }

    let mut outer_to_inners: BTreeMap<usize, Vec<Shell>> = BTreeMap::new();
    let mut root_indices = Vec::new();

    for (j, parent) in contained_in.iter().enumerate() {
        if let Some(p) = parent {
            // **空洞シェルは、外殻と同じ向きで持ちます。**
            //
            // `MassCalculator::compute_from_brep` は「空洞シェルは外殻と
            // 同じ向きで保持されるため、寄与を反転して足す」と書いて
            // あります。**その約束が守られていませんでした。**
            //
            // 差の B 側は面を反転して採るので（`reverse_orientation`）、
            // 空洞になる塊は**内向き**で出てくることがあります。内向きの
            // まま入れると、反転して足すぶんと合わせて**符号が2回変わり**、
            // 体積が `A - B` ではなく `A + B` になります。
            //
            // 実測（4-133）: `cone - sphere` は球が円錐に完全に入って
            // いる（触れてもいない r=1.5 でも）のに、空洞シェルの符号つき
            // 体積が **-14.137167** で、結果が 2108.53 = A + B。
            // `box - sphere (r=5)` は **+523.598776** で正しく 7476.40。
            // **同じ経路で向きが揃っていませんでした。**
            //
            // ここで揃えます。**形は変わりません**——同じ面を、向きだけ
            // 約束どおりにします。
            let shell = &simple_solids[j].outer_shell;
            let cavity = if shell_volumes[j] < 0.0 {
                Shell::closed(shell.faces.iter().map(reverse_face_orientation).collect())
            } else {
                shell.clone()
            };
            outer_to_inners.entry(*p).or_default().push(cavity);
        } else {
            root_indices.push(j);
        }
    }

    let mut result = Vec::new();
    for root in root_indices {
        let outer_shell = simple_solids[root].outer_shell.clone();
        let inner_shells = outer_to_inners.remove(&root).unwrap_or_default();
        result.push(Solid::new(outer_shell, inner_shells));
    }

    result
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
    let ends_match = (points_same_3d(a.start, b.start, tol) && points_same_3d(a.end, b.end, tol))
        || (points_same_3d(a.start, b.end, tol) && points_same_3d(a.end, b.start, tol));
    // **端点が同じでも、別の弧かもしれません。** 途中の点まで見ます。
    // 公差は端点と同じでは足りません——分割の仕方が違う2枚が同じ稜を持つとき、
    // 中点は稜の長さに対して丸め誤差ぶん動きます。稜の長さに対する相対で取ります。
    let reach = (a.start - a.end).norm().max(1.0);
    ends_match && points_same_3d(a.middle, b.middle, tol.max(reach * 1e-6))
}

fn opposite_stitch_edge_direction(a: &StitchEdgeUse, b: &StitchEdgeUse, tol: f64) -> bool {
    points_same_3d(a.start, b.end, tol) && points_same_3d(a.end, b.start, tol)
}

fn points_same_3d(a: Point3, b: Point3, tol: f64) -> bool {
    (a - b).norm() <= tol
}

/// 面を代表する点。内外判定はこの1点で決まるので、**面の上になければ
/// ならない**。
///
/// 以前は境界の標本の平均だった。平面ならそれで面の上に来るが、曲がった面では
/// 来ない。円柱の四半パッチなら、境界の平均は軸のほうへ引っ込んだ位置にあり、
/// 相手の立体に対する内外は面のそれと関係なく決まる。曲面の面が割られる
/// までは表に出なかった（丸ごとの面は相手と交わらないか、判定が偶然合って
/// いた）が、交線で割った断片ではそのまま誤りになる。円柱を円柱で貫くと、
/// 貫かれた側の側面はどの断片も選ばれなかった。
///
/// トリム領域を三角形に割り、**いちばん大きい三角形の重心**を UV で取って
/// 曲面に載せる。三角形は領域の内側にあり、大きいものを選べば境界からも
/// 離れている。
/// トリムループを uv の多角形として取り出す。外周と穴。
///
/// **三角化しません。** 点を1つ選ぶだけなら多角形で足ります（4-161）。
fn pcurve_uv_polygons(face: &Face) -> Option<(Vec<Point2>, Vec<Vec<Point2>>)> {
    const PER_SEGMENT: usize = 24;
    let pcurves = face.pcurves.as_ref()?;
    let sample = |loop_ref: &zenith_topo::FacePcurveLoop| -> Vec<Point2> {
        let mut points = Vec::new();
        for segment in &loop_ref.segments {
            let (t_min, t_max) = segment.curve.param_range();
            if (t_max - t_min).abs() <= f64::EPSILON {
                continue;
            }
            for step in 0..PER_SEGMENT {
                let t = t_min + (t_max - t_min) * (step as f64 / PER_SEGMENT as f64);
                points.push(segment.curve.evaluate(t));
            }
        }
        points
    };
    let outer = sample(&pcurves.outer_loop);
    if outer.len() < 3 {
        return None;
    }
    let holes: Vec<Vec<Point2>> = pcurves
        .inner_loops
        .iter()
        .map(sample)
        .filter(|hole| hole.len() >= 3)
        .collect();
    Some((outer, holes))
}

/// uv の多角形の内側から、縁からいちばん遠い点を格子で探す。
///
/// `want` 個返します。**散らして取るために、見つけた点を等間隔に
/// 間引きます**（固まった点ばかりでは「散らした」ことになりません）。
///
/// 平面側の [`planar_point_clear_of_holes`] と同じやり方です。あちらは
/// 3D の境界点を平面へ射影しますが、曲面は **p-curve が最初から uv に
/// ある**ので、そのまま使えます。
fn uv_points_clear_of_holes(outer: &[Point2], holes: &[Vec<Point2>], want: usize) -> Vec<Point2> {
    const GRID: usize = 24;
    if want == 0 || outer.len() < 3 {
        return Vec::new();
    }
    let (mut min_u, mut max_u) = (f64::INFINITY, f64::NEG_INFINITY);
    let (mut min_v, mut max_v) = (f64::INFINITY, f64::NEG_INFINITY);
    for uv in outer {
        min_u = min_u.min(uv.x);
        max_u = max_u.max(uv.x);
        min_v = min_v.min(uv.y);
        max_v = max_v.max(uv.y);
    }
    let span = (max_u - min_u).max(max_v - min_v);
    if !(span > 0.0) {
        return Vec::new();
    }
    let margin = span * 1e-3;

    let mut inside: Vec<(f64, Point2)> = Vec::new();
    for i in 1..GRID {
        for j in 1..GRID {
            let uv = Point2::new(
                min_u + (max_u - min_u) * (i as f64 / GRID as f64),
                min_v + (max_v - min_v) * (j as f64 / GRID as f64),
            );
            if !point_in_polygon_2d(uv, outer, 0.0) {
                continue;
            }
            if holes.iter().any(|hole| point_in_polygon_2d(uv, hole, 0.0)) {
                continue;
            }
            let clearance = std::iter::once(outer)
                .chain(holes.iter().map(|hole| hole.as_slice()))
                .map(|polygon| polygon_min_distance_2d(uv, polygon))
                .fold(f64::INFINITY, f64::min);
            if clearance <= margin {
                continue;
            }
            inside.push((clearance, uv));
        }
    }
    if inside.is_empty() {
        return Vec::new();
    }
    if want == 1 {
        // いちばん縁から遠い点。
        let best = inside
            .iter()
            .max_by(|left, right| left.0.total_cmp(&right.0))
            .map(|(_, uv)| *uv);
        return best.into_iter().collect();
    }
    if inside.len() <= want {
        return inside.into_iter().map(|(_, uv)| uv).collect();
    }
    let stride = inside.len() as f64 / want as f64;
    (0..want)
        .map(|index| inside[((index as f64 + 0.5) * stride) as usize % inside.len()].1)
        .collect()
}

fn representative_face_point(face: &Face) -> Point3 {
    if let FaceGeometry::Plane(plane) = &face.geometry {
        if let Some(point) = planar_point_clear_of_holes(face, plane) {
            return point;
        }
    }

    if let FaceGeometry::Nurbs(surface) = &face.geometry {
        // **三角化しません**（4-161）。p-curve の多角形から直接選びます。
        if let Some((outer, holes)) = pcurve_uv_polygons(face) {
            if let Some(uv) = uv_points_clear_of_holes(&outer, &holes, 1).first() {
                return surface.evaluate(uv.x, uv.y);
            }
        }
        if let Some((u, v)) = largest_domain_triangle_centroid(face) {
            return surface.evaluate(u, v);
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

/// トリム領域の三角形のうち、いちばん大きいものの重心（UV）。
fn largest_domain_triangle_centroid(face: &Face) -> Option<(f64, f64)> {
    // **点を選ぶだけなので、細分は掛けません**（4-160）。
    let domain = zenith_tess::face_uv_triangulation_for_point_picking(
        face,
        &zenith_tess::TessellationParams::default(),
    );
    let mut best: Option<(f64, (f64, f64))> = None;
    for triangle in &domain.triangles {
        let a = domain.uvs[triangle[0]];
        let b = domain.uvs[triangle[1]];
        let c = domain.uvs[triangle[2]];
        let area = 0.5 * ((b.x - a.x) * (c.y - a.y) - (b.y - a.y) * (c.x - a.x)).abs();
        let centroid = ((a.x + b.x + c.x) / 3.0, (a.y + b.y + c.y) / 3.0);
        if best
            .as_ref()
            .map(|(found, _)| area > *found)
            .unwrap_or(true)
        {
            best = Some((area, centroid));
        }
    }
    best.map(|(_, centroid)| centroid)
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

/// 面の境界を確実に含む箱。
///
/// **標本から作った箱は上界ではありません。** ここは長らく境界を12点ずつ
/// 標本して箱にしていました。曲がった境界では標本の間で外へ膨らむので、
/// 出来た箱は本物より小さくなります。有理曲線はパラメータが角度に比例しない
/// ので、「端と真ん中を取れば足りる」も成り立ちません。
///
/// 実測: 半径10の円柱を120度ずつに割った断片で、境界の本当の y の最大は 10、
/// 標本の箱は **9.993**。この箱は候補の絞り込みと交線の切り詰めの両方に
/// 使われるので、x=-0.2 の平面との交線（y=9.998）が**箱の外**と判定され、
/// 面の組ごと落ちていました。落ちた組は「交わらない」と同じ扱いになります。
///
/// 制御点で測ります。B-spline 曲線は制御点の凸包に含まれる（重みが正で
/// あれば有理でも同じ）ので、制御点の箱は**必ず曲線を含みます**。絞り込みの
/// 箱は大きすぎる方向に外れるべきで、小さすぎる方向に外れてはいけません。
/// 標本も併せて入れます——重みが負の曲線が来ても悪化させないためです。
/// 割った片に、元の面が持っていた穴を配り直す。
///
/// **穴を持つ平面は、これまで一切割れませんでした。** 実務では穴あき板の面
/// そのもので、円環の上面もこれです。実測: 輪の角を箱で削る配置は、上面
/// （外周 r10・穴 r4 の円環）が割れないせいで、切り口の三角形が縫えずに
/// 落ちていました。
///
/// 切り込みが穴に触れない配置に限ります。触れているなら穴自体を割ることに
/// なり、それは別の仕事です。**できないことは、名前を挙げて断ります。**
fn distribute_inner_wires(
    original: &Face,
    pieces: Vec<Face>,
    plane: &zenith_geom::PlaneSurface3,
    tol: &Tolerance,
) -> Result<Vec<Face>, String> {
    if original.inner_wires.is_empty() {
        return Ok(pieces);
    }

    // 片ごとの外周を uv の多角形にしておく。
    let polygons: Vec<Vec<Point2>> = pieces
        .iter()
        .map(|piece| {
            piece
                .outer_wire
                .sample_points(24)
                .iter()
                .map(|point| project_to_plane_uv(*point, plane))
                .collect()
        })
        .collect();

    let mut assigned: Vec<Vec<Wire>> = vec![Vec::new(); pieces.len()];
    for wire in &original.inner_wires {
        // 穴の代表点。**辺の中点を使います** — 頂点は角にあたることがあり、
        // 内外判定が割れます。
        let Some(first) = wire.edges.first() else {
            continue;
        };
        let sample = project_to_plane_uv(edge_midpoint(&first.edge), plane);

        let mut owners = polygons.iter().enumerate().filter(|(_, polygon)| {
            polygon.len() >= 3 && point_in_polygon_2d(sample, polygon, tol.parametric)
        });
        let Some((index, _)) = owners.next() else {
            return Err(
                "the cut passes through a hole; splitting a face across its inner wire is not implemented"
                    .to_string(),
            );
        };
        if owners.next().is_some() {
            return Err(
                "a hole landed in more than one piece; the cut probably crosses it".to_string(),
            );
        }
        assigned[index].push(wire.clone());
    }

    Ok(pieces
        .into_iter()
        .zip(assigned)
        .map(|(piece, inners)| {
            if inners.is_empty() {
                return piece;
            }
            Face::new(
                piece.geometry.clone(),
                piece.outer_wire.clone(),
                inners,
                piece.orientation,
                piece.tolerance,
            )
        })
        .collect())
}

fn face_boundary_bbox(face: &Face) -> Option<BoundingBox3> {
    let mut bbox = BoundingBox3::empty();

    let take = |wire: &zenith_topo::Wire, bbox: &mut BoundingBox3| {
        for oriented in &wire.edges {
            extend_by_curve_hull(bbox, &oriented.edge.curve);
        }
    };

    take(&face.outer_wire, &mut bbox);
    for wire in &face.inner_wires {
        take(wire, &mut bbox);
    }

    bbox.is_valid().then_some(bbox)
}

/// 曲線を細分し、断片ごとの制御多角形で箱を広げる。
///
/// **緩い上界では足りません。** この箱は2つの仕事をしており、片方は緩めては
/// いけないからです。
///
/// 1. 面の組を候補に入れるかの絞り込み — **外してはいけない**
/// 2. 求めた交線の切り詰め（`clip_candidate_to_face_bboxes`）— **緩めてはいけない**
///
/// 生の制御多角形は 1 に足りて 2 に緩すぎます。実測: `occ_reference_revolved_ring`
/// を角の箱で削る配置で、交線が面の外まで残って縫えない稜が 11 本出て、
/// **それまで通っていた配置が拒否になりました**。
///
/// 曲線を分けると、断片の制御多角形は曲線に近づきます（はみ出しは区間長の
/// 2乗で縮む）。分けても凸包の性質は保たれるので、**保証つきのまま、きつい
/// 箱**になります。3回（8断片）で、円弧のはみ出しはおよそ 1/64 です。
///
/// 分ける回数を増やせばもっときつくなりますが、絞り込みは面の枚数ぶん走る
/// ので、際限なくは分けられません。
fn extend_by_curve_hull(bbox: &mut BoundingBox3, curve: &zenith_geom::NurbsCurve3) {
    const LEVELS: usize = 3;

    // 重みが正でなければ凸包の性質が使えません。そのときだけ標本で妥協します
    // （**標本は上界ではありません**が、生の制御点より実物に近い）。
    if curve
        .control_points
        .iter()
        .any(|control| !(control.weight > 0.0))
    {
        let (t0, t1) = curve.param_range();
        for step in 0..=64 {
            let point = curve.evaluate(t0 + (t1 - t0) * step as f64 / 64.0);
            if point3_is_finite(point) {
                bbox.extend_point(point);
            }
        }
        return;
    }

    let mut pieces = vec![curve.clone()];
    for _ in 0..LEVELS {
        let mut next = Vec::with_capacity(pieces.len() * 2);
        for piece in pieces {
            let (t0, t1) = piece.param_range();
            match piece.split_at((t0 + t1) * 0.5) {
                Some((left, right)) => {
                    next.push(left);
                    next.push(right);
                }
                // 分けられない曲線は、そのままの制御多角形で使う。
                None => next.push(piece),
            }
        }
        pieces = next;
    }

    for piece in &pieces {
        for control in &piece.control_points {
            if point3_is_finite(control.point) {
                bbox.extend_point(control.point);
            }
        }
    }
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
            let padded = bbox_overlap_where_needed(bbox_a, bbox_b, tol.linear)?;
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

/// 重なりの箱。**余裕は、それが要る軸にだけ当てる。**
///
/// 全部の軸に一律で当てると、**要らない軸にも余裕が残ります**。接触配置が
/// まさにそれで、余裕なしの重なりは接している軸の厚さが 0 になるため
/// `clip_line_to_bbox` が通らず、余裕つきの箱へ落ちます。**そのとき、
/// 接していない軸にも余裕が乗り、交線の端点が公差ぶんだけ面の外へ出ます。**
///
/// 実測（4-145、他カーネルの円柱 × 空洞つき箱）: 半径 10 の円柱は箱の面
/// `y = ±10` に線で接します。そこで作られた交線の端点が
/// **z = −7.000001**（= −7 − `tol.linear`）になり、同じ稜を別の面の組から
/// 作った `z = −7.000000` と**公差の外で食い違いました**。あぶれた稜として
/// 縫合が止まり、3演算とも断られていました。
///
/// **厚さが 0 の軸だけ広げれば、探すのに足りて、答えは動きません。**
fn bbox_overlap_where_needed(a: &BoundingBox3, b: &BoundingBox3, tol: f64) -> Option<BoundingBox3> {
    let axis = |low_a: f64, high_a: f64, low_b: f64, high_b: f64| {
        let low = low_a.max(low_b);
        let high = high_a.min(high_b);
        if high - low > tol {
            // この軸は厚みがある。広げない。
            (low, high)
        } else {
            (low - tol, high + tol)
        }
    };
    let (min_x, max_x) = axis(a.min.x, a.max.x, b.min.x, b.max.x);
    let (min_y, max_y) = axis(a.min.y, a.max.y, b.min.y, b.max.y);
    let (min_z, max_z) = axis(a.min.z, a.max.z, b.min.z, b.max.z);
    let min = Point3::new(min_x, min_y, min_z);
    let max = Point3::new(max_x, max_y, max_z);
    (min.x <= max.x && min.y <= max.y && min.z <= max.z)
        .then_some(BoundingBox3::from_min_max(min, max))
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
            // **1本の直線が、面の中で2本以上に分かれることがあります。**
            // 穴あきの面を横切る直線がそれで、円環を横切れば材料の上に乗るのは
            // 2区間です。長いほうだけ返していたので、輪をスラブで切ると
            // 稜が8本のところ6本しか出ず、切り口のループが閉じませんでした。
            let mut intervals = clip_segment_to_planar_face_trim_all(
                segment_start,
                segment_end,
                face_a,
                &[(0.0, 1.0)],
                tol,
            )?;
            intervals = clip_segment_to_planar_face_trim_all(
                segment_start,
                segment_end,
                face_b,
                &intervals,
                tol,
            )?;
            let segment_vec = segment_end - segment_start;
            let intervals: Vec<(f64, f64)> = intervals
                .into_iter()
                .filter(|(start, end)| (segment_vec * (end - start)).norm() > tol.linear)
                .collect();

            match intervals.len() {
                0 => None,
                1 => {
                    let (start, end) = intervals[0];
                    Some(FaceIntersectionKind::Line {
                        point,
                        direction,
                        segment_start: segment_start + segment_vec * start,
                        segment_end: segment_start + segment_vec * end,
                    })
                }
                _ => {
                    let mut edges = Vec::with_capacity(intervals.len());
                    for (start, end) in intervals {
                        let from = segment_start + segment_vec * start;
                        let to = segment_start + segment_vec * end;
                        let Ok(curve) = NurbsCurve3::bspline_from_points(1, vec![from, to]) else {
                            continue;
                        };
                        edges.push(Edge::new(
                            curve,
                            Vertex::new(from, tol.linear),
                            Vertex::new(to, tol.linear),
                            tol.linear,
                        ));
                    }
                    match edges.len() {
                        0 => None,
                        1 => Some(FaceIntersectionKind::Curve {
                            edge: edges.into_iter().next().unwrap(),
                        }),
                        _ => Some(FaceIntersectionKind::Curves { edges }),
                    }
                }
            }
        }
        FaceIntersectionKind::Curve { edge } => {
            let pieces = clip_curve_to_both_planar_trims(&edge, face_a, face_b, tol);
            match pieces.len() {
                0 => None,
                1 => Some(FaceIntersectionKind::Curve {
                    edge: pieces.into_iter().next().unwrap(),
                }),
                _ => Some(FaceIntersectionKind::Curves { edges: pieces }),
            }
        }
        FaceIntersectionKind::Curves { edges } => {
            let pieces: Vec<Edge> = edges
                .iter()
                .flat_map(|edge| clip_curve_to_both_planar_trims(edge, face_a, face_b, tol))
                .collect();
            match pieces.len() {
                0 => None,
                1 => Some(FaceIntersectionKind::Curve {
                    edge: pieces.into_iter().next().unwrap(),
                }),
                _ => Some(FaceIntersectionKind::Curves { edges: pieces }),
            }
        }
        other => Some(other),
    }
}

fn clip_curve_to_both_planar_trims(
    edge: &Edge,
    face_a: &Face,
    face_b: &Face,
    tol: &Tolerance,
) -> Vec<Edge> {
    let mut pieces = vec![edge.clone()];
    for face in [face_a, face_b] {
        pieces = pieces
            .iter()
            .flat_map(|piece| {
                clip_curve_to_planar_face_trim(piece, face, tol)
                    .unwrap_or_else(|| vec![piece.clone()])
            })
            .collect();
    }
    pieces
}

/// 交線を、平面の面のトリム境界で切る。
///
/// 曲面と平面が交わっても、その交線が平面の**面**の中に収まっているとは
/// 限らない。トーラスを箱の底面が切ると、断面は入れ子の2つの円になり、
/// 外側（半径 15.464）は面（|x|, |y| <= 10）からはみ出す。はみ出したまま
/// 渡すと、面の内側で閉じるループとしても、境界から境界へ届く切り込みとしても
/// 読めず、面はそこで割れない。
///
/// `None` は「切る必要が無い、または切れない」で、呼び手は元のまま使う。
/// 全部が外なら空の `Vec` を返す。
fn clip_curve_to_planar_face_trim(edge: &Edge, face: &Face, tol: &Tolerance) -> Option<Vec<Edge>> {
    let FaceGeometry::Plane(plane) = &face.geometry else {
        return None;
    };
    let pcurves = face.pcurves(tol).ok()?;
    let polygon = sample_pcurve_loop(&pcurves.outer_loop, 48);
    if polygon.len() < 3 {
        return None;
    }
    // **穴の中は面の外です。** 外周だけで見ていたので、穴を通る交線が面の上に
    // あることになっていました。円環の平らなキャップに、穴を素通りする丸棒の
    // 側面が作る円がそれで、実際には触れていないのに交線として通ります。
    let holes: Vec<Vec<Point2>> = pcurves
        .inner_loops
        .iter()
        .map(|inner| sample_pcurve_loop(inner, 48))
        .filter(|polygon| polygon.len() >= 3)
        .collect();

    let (t0, t1) = edge.curve.param_range();
    let span = t1 - t0;
    if span <= f64::EPSILON {
        return None;
    }
    // 面の広がりに対する余裕。境界の上に乗った点は内側として扱う。
    let margin = tol.parametric.max(1e-9);
    let inside = |t: f64| -> bool {
        let point = edge.curve.evaluate(t.clamp(t0, t1));
        let uv = project_to_plane_uv(point, plane);
        // **境目は曲線そのもので決めます。** 折れ線で判定すると、二分は
        // 折れ線に詰まり、面の境界からたわみのぶん（半径10・48刻みで
        // 1.3e-3）ずれた位置で切れます（4-63）。
        let in_outer = point_inside_pcurve_loop(uv, &pcurves.outer_loop, tol)
            .unwrap_or_else(|| point_in_polygon_2d(uv, &polygon, margin));
        if !in_outer {
            return false;
        }
        // 穴の内側は面ではない。
        !pcurves
            .inner_loops
            .iter()
            .zip(holes.iter())
            .any(|(inner, hole)| {
                point_inside_pcurve_loop(uv, inner, tol)
                    .unwrap_or_else(|| point_in_polygon_2d(uv, hole, -margin))
            })
    };

    const SAMPLES: usize = 257;
    let flags: Vec<bool> = (0..=SAMPLES)
        .map(|step| inside(t0 + span * step as f64 / SAMPLES as f64))
        .collect();

    // 全部内側なら切る必要が無い。ここで返さないと、多角形近似のぶんだけ
    // 端が削れて、いま通っている切り方が変わってしまう。
    if flags.iter().all(|value| *value) {
        return None;
    }
    if flags.iter().all(|value| !*value) {
        return Some(Vec::new());
    }

    // 内側と外側が入れ替わる区間を二分で詰める。
    let crossing = |a: f64, b: f64| -> f64 {
        let (mut low, mut high) = (a, b);
        let low_inside = inside(low);
        for _ in 0..60 {
            let middle = (low + high) * 0.5;
            if inside(middle) == low_inside {
                low = middle;
            } else {
                high = middle;
            }
        }
        (low + high) * 0.5
    };

    let mut intervals: Vec<(f64, f64)> = Vec::new();
    let mut current: Option<f64> = if flags[0] { Some(t0) } else { None };
    for step in 1..=SAMPLES {
        let previous = t0 + span * (step - 1) as f64 / SAMPLES as f64;
        let here = t0 + span * step as f64 / SAMPLES as f64;
        if flags[step] == flags[step - 1] {
            continue;
        }
        let boundary = crossing(previous, here);
        match current.take() {
            Some(start) => intervals.push((start, boundary)),
            None => current = Some(boundary),
        }
    }
    if let Some(start) = current {
        intervals.push((start, t1));
    }

    let mut pieces = Vec::new();
    for (start, end) in intervals {
        if end - start <= span * 1e-9 {
            continue;
        }
        let Some(piece) = subcurve_between(&edge.curve, start, end) else {
            continue;
        };
        let (a, b) = piece.param_range();
        let start_point = piece.evaluate(a);
        let end_point = piece.evaluate(b);
        if (end_point - start_point).norm() <= tol.linear {
            continue;
        }
        pieces.push(Edge::new(
            piece,
            Vertex::new(start_point, tol.linear),
            Vertex::new(end_point, tol.linear),
            edge.tolerance,
        ));
    }
    Some(pieces)
}

/// 曲線の `a` から `b` までを取り出す。
fn subcurve_between(curve: &NurbsCurve3, a: f64, b: f64) -> Option<NurbsCurve3> {
    let (t0, t1) = curve.param_range();
    let span = (t1 - t0).abs().max(1.0);
    let mut piece = curve.clone();
    if a > t0 + span * 1e-12 {
        piece = piece.split_at(a)?.1;
    }
    if b < t1 - span * 1e-12 {
        piece = piece.split_at(b)?.0;
    }
    Some(piece)
}

/// 面のトリムで残る区間を、**1つに絞らずに全部**返す。
///
/// `clip_segment_to_planar_face_trim` は一番長い区間だけを返します。1本の
/// 交線が面の中で2本以上に分かれる形——穴あきの面を横切る直線——では、
/// それでは足りません。輪をスラブで切ると、切り口の断面は穴の両側の2つの
/// 矩形になり、必要な稜は8本です。1区間しか返さないと6本になり、蓋の
/// ループがどちらも閉じません。
fn clip_segment_to_planar_face_trim_all(
    segment_start: Point3,
    segment_end: Point3,
    face: &Face,
    current: &[(f64, f64)],
    tol: &Tolerance,
) -> Option<Vec<(f64, f64)>> {
    let FaceGeometry::Plane(plane) = &face.geometry else {
        return Some(current.to_vec());
    };
    let Ok(pcurves) = face.pcurves(tol) else {
        return Some(current.to_vec());
    };
    let uv_start = project_to_plane_uv(segment_start, plane);
    let uv_end = project_to_plane_uv(segment_end, plane);
    // 評価できないループはこれまでどおりクリップ対象外として素通しする
    let Some(mut inside) =
        segment_inside_pcurve_loop_intervals(uv_start, uv_end, &pcurves.outer_loop, tol)
    else {
        return Some(current.to_vec());
    };

    // 穴の中は面の外。
    for inner in &pcurves.inner_loops {
        let Some(hole) = segment_inside_pcurve_loop_intervals(uv_start, uv_end, inner, tol) else {
            continue;
        };
        inside = subtract_intervals(inside, &hole, tol.parametric);
        if inside.is_empty() {
            return None;
        }
    }

    let epsilon = tol.parametric.max(1e-12);
    let mut out = Vec::new();
    for (low, high) in current {
        for (start, end) in &inside {
            let from = low.max(*start);
            let to = high.min(*end);
            if to - from > epsilon {
                out.push((from, to));
            }
        }
    }
    if out.is_empty() {
        return None;
    }
    out.sort_by(|a, b| a.0.total_cmp(&b.0));
    Some(out)
}

/// `base` から `cut` を取り除く。どちらも昇順に整っている必要はありません。
fn subtract_intervals(base: Vec<(f64, f64)>, cut: &[(f64, f64)], tol: f64) -> Vec<(f64, f64)> {
    let epsilon = tol.max(1e-12);
    let mut remaining = base;
    for hole in cut {
        let mut next = Vec::with_capacity(remaining.len() + 1);
        for (start, end) in remaining {
            // 重なりが無ければそのまま残す。
            if hole.1 <= start + epsilon || hole.0 >= end - epsilon {
                next.push((start, end));
                continue;
            }
            if hole.0 > start + epsilon {
                next.push((start, hole.0));
            }
            if hole.1 < end - epsilon {
                next.push((hole.1, end));
            }
        }
        remaining = next;
        if remaining.is_empty() {
            break;
        }
    }
    remaining
        .into_iter()
        .filter(|(start, end)| end - start > epsilon)
        .collect()
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
                // 閉じた式が無い次数（3次以上、あるいは多スパン）。
                // **折れ線で代用しない。** 32点の折れ線で交差位置を取ると、
                // 弦と曲線のずれがそのまま交点のずれになる。実測: 押し出した
                // スプラインを平面で切ると、側面から出た交線の端が y = 2.7446、
                // 同じスプラインを境界に持つ蓋から出た交線の端が y = 2.7423 で、
                // **2.3e-3 ずれて輪が閉じず**、蓋が1枚も作れていなかった。
                //
                // 直線までの符号付き距離は曲線に沿って滑らかなので、符号の
                // 変わる区間を折れ線で挟んでから、曲線そのもので二分する。
                // 次数に依らず丸め誤差まで詰まる。
                for t in pcurve_crossings_by_bisection(
                    &segment.curve,
                    start,
                    direction,
                    CLASSIFICATION_SAMPLES,
                ) {
                    cuts.push(t);
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

/// 直線と p-curve の交点を、**曲線そのもの**の上で求める。
///
/// 直線までの符号付き距離 `f(t) = n·(C(t) - start)` は曲線に沿って滑らか
/// なので、符号が変わる区間を粗い標本で挟み、そこを二分すれば次数に依らず
/// 丸め誤差まで詰められる。返すのは直線側のパラメータ（`start` を 0、
/// `start + direction` を 1 とする）。
///
/// **折れ線の交点で代用しないための関数である。** 弦と曲線のずれはそのまま
/// 交点のずれになり、同じ曲線を境界に持つ別の面から出た交線と端が食い違う。
fn pcurve_crossings_by_bisection(
    curve: &zenith_geom::NurbsCurve2,
    start: Point2,
    direction: Vec2,
    samples: usize,
) -> Vec<f64> {
    let normal = Vec2::new(-direction.y, direction.x);
    let signed = |t: f64| normal.dot(&(curve.evaluate(t) - start));
    let on_line = |t: f64| {
        let point = curve.evaluate(t);
        (point - start).dot(&direction) / direction.norm_squared()
    };

    let (t_min, t_max) = curve.param_range();
    let samples = samples.max(4);
    let mut found = Vec::new();
    // 標本そのものが線に乗っている場合。二分では挟めないので拾っておく。
    let zero = normal.norm() * 1e-12;

    let mut previous = (t_min, signed(t_min));
    if previous.1.abs() <= zero {
        found.push(on_line(t_min));
    }
    for step in 1..=samples {
        let t = t_min + (t_max - t_min) * (step as f64 / samples as f64);
        let value = signed(t);
        if value.abs() <= zero {
            found.push(on_line(t));
        } else if previous.1 * value < 0.0 {
            // 符号が変わった。曲線の上で二分する。
            let (mut low, mut low_value) = previous;
            let mut high = t;
            for _ in 0..80 {
                let mid = 0.5 * (low + high);
                if mid <= low || mid >= high {
                    break;
                }
                let mid_value = signed(mid);
                if mid_value == 0.0 {
                    low = mid;
                    break;
                }
                if low_value * mid_value < 0.0 {
                    high = mid;
                } else {
                    low = mid;
                    low_value = mid_value;
                }
            }
            found.push(on_line(0.5 * (low + high)));
        }
        previous = (t, value);
    }
    found
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

/// 面の組の交わりと、**それが解析的に出たかどうか**。
///
/// 辿って出したものは `false` です。理由は
/// [`FaceIntersectionCandidate::analytic`]。
fn intersect_face_supports(
    face_a: &Face,
    face_b: &Face,
    tol: &Tolerance,
) -> Option<(FaceIntersectionKind, bool)> {
    match (&face_a.geometry, &face_b.geometry) {
        (FaceGeometry::Plane(plane_a), FaceGeometry::Plane(plane_b)) => Some((
            intersect_planes(
                plane_a.origin,
                oriented_plane_normal(face_a),
                plane_b.origin,
                oriented_plane_normal(face_b),
                tol,
            ),
            true,
        )),
        (FaceGeometry::Plane(plane), FaceGeometry::Nurbs(surface)) => {
            // **同一平面の2つの円が接するだけなら、交線はありません**（4-197）。
            if planar_rim_touches_section_circle(face_a, plane, surface, tol) {
                return None;
            }
            Some(
                match intersect_plane_cylinder_patch(
                    plane,
                    oriented_plane_normal(face_a),
                    surface,
                    tol,
                ) {
                    FaceIntersectionKind::Unsupported => (
                        intersect_planar_face_with_patch(face_a, plane, surface, tol),
                        false,
                    ),
                    kind => (kind, true),
                },
            )
        }
        (FaceGeometry::Nurbs(surface), FaceGeometry::Plane(plane)) => {
            if planar_rim_touches_section_circle(face_b, plane, surface, tol) {
                // **黙って落とさない**（`ZENITH_SSI_WHY=1`。4-269）。
                //
                // ここは「接しているだけ」と判断して組を捨てる出口です。
                // 捨てた組は交線を1本も出さないので、面が割れず、分類が
                // 丸ごと片側に倒れます。**理由が出ないと、交線が 0 本という
                // 事実だけが残ります**（4-267 の `linkrods.step` がそれでした）。
                if std::env::var_os("ZENITH_SSI_WHY").is_some() {
                    eprintln!(
                        "SSIWHY 曲面×平面: 平面の縁が断面の円に接していると見て組を捨てました"
                    );
                }
                return None;
            }
            Some(
                match intersect_plane_cylinder_patch(
                    plane,
                    oriented_plane_normal(face_b),
                    surface,
                    tol,
                ) {
                    FaceIntersectionKind::Unsupported => (
                        intersect_planar_face_with_patch(face_b, plane, surface, tol),
                        false,
                    ),
                    kind => (kind, true),
                },
            )
        }
        (FaceGeometry::Nurbs(surface_a), FaceGeometry::Nurbs(surface_b)) => {
            // **接する母線は、辿らずに解析的に出します**（4-190）。
            //
            // 交線まるごとが接している場合、辿りは向きを決められません。
            // 実測（4-170）: 半径 6 の平行な円柱2本を軸間 12 で接させると、
            // 本来 40 の直線1本であるべき母線が、**長さ 3e-4 の破片 8本**に
            // なっていました。短すぎて材料も数えられず（輪の半径が公差の
            // 3倍）、断り文が「未実装」に落ちていました。
            //
            // 軸が平行で、軸間距離が半径の和（外接）か差（内接）なら、
            // **接する母線は1本に決まります**。辿る必要がありません。
            // **1点で触れるだけなら、交線はありません**（4-197。規約 3-1）。
            //
            // 測って決めようとした手は4つとも壁に当たりました（4-180、
            // 4-181、4-195、4-196）——接触では、区別したい量がいつも公差の
            // 下にあります。**形から決めます。**
            if revolution_patches_touch_at_a_point(surface_a, surface_b, tol) {
                return None;
            }
            match intersect_tangent_cylinder_patches(surface_a, surface_b, tol) {
                Some(kind) => Some((kind, true)),
                None => Some((intersect_nurbs_patches(surface_a, surface_b, tol), false)),
            }
        }
        _ => None,
    }
}

/// 平面の面と曲面パッチの交線を、曲面同士と同じやり方で辿る。
///
/// 平面が軸に垂直なときは断面が等パラメータ線になり、専用の経路が扱う。
/// そうでない切り方——トーラスを縦に切る、球を斜めに切る——はそこを外れ、
/// これまで `Unsupported` だった。マーチングは平面も曲面も区別しないが、
/// [`PlaneSurface3`] のパラメータ範囲は無限なので、そのままでは渡せない
/// （種を撒く格子も、領域の縁への着地も決まらない）。**面が実際に占める
/// ぶんだけの有界なパッチ**に直してから渡す。
fn intersect_planar_face_with_patch(
    planar_face: &Face,
    plane: &zenith_geom::PlaneSurface3,
    surface: &NurbsSurface3,
    tol: &Tolerance,
) -> FaceIntersectionKind {
    let Some(patch) = planar_face_as_patch(planar_face, plane) else {
        // **黙って落とさない**（4-269）。平面の面を有界なパッチに直せない
        // なら、交線は辿れません。
        if std::env::var_os("ZENITH_SSI_WHY").is_some() {
            eprintln!("SSIWHY 平面の面を有界なパッチに直せませんでした（境界の点が取れない）");
        }
        return FaceIntersectionKind::Unsupported;
    };
    intersect_nurbs_patches(&patch, surface, tol)
}

/// 平面の面を、その境界が占める範囲ちょうどの1次×1次パッチにする。
fn planar_face_as_patch(face: &Face, plane: &zenith_geom::PlaneSurface3) -> Option<NurbsSurface3> {
    use zenith_geom::{ControlPoint3, KnotVector};

    let points = face.outer_wire.sample_points(8);
    if points.is_empty() {
        return None;
    }
    let (mut u_min, mut u_max) = (f64::INFINITY, f64::NEG_INFINITY);
    let (mut v_min, mut v_max) = (f64::INFINITY, f64::NEG_INFINITY);
    for point in &points {
        let uv = project_to_plane_uv(*point, plane);
        u_min = u_min.min(uv.x);
        u_max = u_max.max(uv.x);
        v_min = v_min.min(uv.y);
        v_max = v_max.max(uv.y);
    }
    if !(u_max > u_min && v_max > v_min) {
        return None;
    }
    // **広げない。** 交線はこのパッチの縁で止まり、その端がそのまま面を割る
    // 切り込みの端になる。1e-6 だけ広げてみたところ、20 幅の面では端が 2e-5
    // だけ面の外に出て、割った片のワイヤがそこで開いた。

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

/// 曲面同士の交線を辿って1本の辺にする。
///
/// ここに来るまで、この組み合わせは `Unsupported` を返していた。交線が
/// 取れなければブーリアンは始まらない。
///
/// **要求した精度に届かなければ `Unsupported` を返す。** 近いところを通る
/// もっともらしい曲線を渡すと、その先の分割と選別が静かに間違う。
fn intersect_nurbs_patches(
    surface_a: &NurbsSurface3,
    surface_b: &NurbsSurface3,
    tol: &Tolerance,
) -> FaceIntersectionKind {
    let extent = surface_patch_extent(surface_a).max(surface_patch_extent(surface_b));
    let first_step = (extent * 0.1).max(tol.linear * 100.0);
    let deviation_limit = tol.linear;

    // 枝は2本まで見る。1枚のパッチと1枚のパッチが3本以上で交わる配置は
    // 今の検体には無く、上限を上げるとブーリアンの走査が目に見えて遅くなる。
    let branches = zenith_geom::IntersectionMarcher::fit_all_branches(
        surface_a,
        surface_b,
        first_step,
        deviation_limit,
        2,
        tol,
    );

    // `ZENITH_SSI_WHY=1` で、辿れた枝と、落とした枝の理由が1行ずつ出ます。
    // **交線が1本も取れないと、その面の組は `Unsupported` になり、ブーリアンは
    // そこから先に進めません。** 落ちた理由が分からないと、交差の実装が悪いのか
    // 種が見つからないのかを切り分けられません。
    let explain = std::env::var_os("ZENITH_SSI_WHY").is_some();
    if explain {
        // **許容と、その元になった大きさも出します**（4-223）。
        // 「何本辿れたか」だけでは、公差をいじっても数字が動かない理由が
        // 分かりません。`deviation_limit` は絶対値なので、模型が小さいと
        // 相対では緩くなります。
        eprintln!(
            "SSIWHY {} branch(es) marched  パッチの大きさ {extent:.6e}、             当てはめの許容 {deviation_limit:.3e}（相対 {:.3e}）、最初の歩幅 {first_step:.3e}",
            branches.len(),
            deviation_limit / extent.max(f64::MIN_POSITIVE)
        );
        for (_, _, deviation) in &branches {
            eprintln!(
                "SSIWHY   採った枝のずれ {deviation:.3e}（相対 {:.3e}）",
                deviation / extent.max(f64::MIN_POSITIVE)
            );
        }
    }

    let mut edges = Vec::new();
    for (curve, marched, _) in branches {
        // パッチの**縁に沿って**走る交線は、切り込みではなく接触の記録である。
        // 平面がトーラスの赤道を通ると、そこはパッチの境界そのものなので、
        // 縁に沿った線がいくつも出る。切り込みとして渡すと、面はそこで割れずに
        // 「境界に届かない」と断られ、しかも本物の切り込みを押しのける。
        if marched_runs_along_a_patch_edge(&marched, surface_a, surface_b) {
            if explain {
                eprintln!(
                    "  dropped: runs along a patch edge ({} points)",
                    marched.points.len()
                );
            }
            continue;
        }
        let (t0, t1) = curve.param_range();
        let start = curve.evaluate(t0);
        let end = curve.evaluate(t1);
        if (end - start).norm() <= tol.linear {
            if explain {
                eprintln!(
                    "  dropped: the two ends meet ({:.3e} apart, {} marched points)",
                    (end - start).norm(),
                    marched.points.len()
                );
            }
            continue;
        }
        if explain {
            eprintln!(
                "  kept: ({:.3} {:.3} {:.3}) -> ({:.3} {:.3} {:.3}), {} marched points",
                start.x,
                start.y,
                start.z,
                end.x,
                end.y,
                end.z,
                marched.points.len()
            );
        }
        edges.push(Edge::new(
            curve,
            Vertex::new(start, tol.linear),
            Vertex::new(end, tol.linear),
            tol.linear,
        ));
    }

    match edges.len() {
        0 => FaceIntersectionKind::Unsupported,
        1 => FaceIntersectionKind::Curve {
            edge: edges.into_iter().next().unwrap(),
        },
        _ => FaceIntersectionKind::Curves { edges },
    }
}

/// 辿った点が、どちらかのパッチの縁に**ずっと**乗っているか。
///
/// 端の1点や2点が縁に来るのは普通のこと（交線はパッチの縁で終わる）。
/// 見るのは全部の点である。
fn marched_runs_along_a_patch_edge(
    marched: &zenith_geom::MarchedIntersection,
    surface_a: &NurbsSurface3,
    surface_b: &NurbsSurface3,
) -> bool {
    let ((ua0, ua1), (va0, va1)) = surface_a.param_range();
    let ((ub0, ub1), (vb0, vb1)) = surface_b.param_range();
    let at = |value: f64, low: f64, high: f64| {
        let margin = (high - low).abs().max(1.0) * 1e-7;
        (value - low).abs() <= margin || (value - high).abs() <= margin
    };

    let on_a_edge = marched
        .points
        .iter()
        .all(|p| at(p.uv1.0, ua0, ua1) || at(p.uv1.1, va0, va1));
    let on_b_edge = marched
        .points
        .iter()
        .all(|p| at(p.uv2.0, ub0, ub1) || at(p.uv2.1, vb0, vb1));
    on_a_edge && on_b_edge
}

/// パッチの広がり。歩幅を形の大きさに合わせるために使う。
fn surface_patch_extent(surface: &NurbsSurface3) -> f64 {
    let ((u0, u1), (v0, v1)) = surface.param_range();
    let corners = [
        surface.evaluate(u0, v0),
        surface.evaluate(u1, v0),
        surface.evaluate(u0, v1),
        surface.evaluate(u1, v1),
        surface.evaluate((u0 + u1) * 0.5, (v0 + v1) * 0.5),
    ];
    let mut worst: f64 = 0.0;
    for (index, a) in corners.iter().enumerate() {
        for b in corners.iter().skip(index + 1) {
            worst = worst.max((a - b).norm());
        }
    }
    // **1.0 の床を置いてはいけません**（4-223）。
    //
    // 床の意図は「潰れたパッチで 0 を返さない」ことですが、**絶対値で
    // 置くと小さい模型で嘘になります**。実測: 差し渡し 0.21 のパッチが
    // 「1.0」と報告され、`first_step = extent * 0.1` が **0.1**——
    // **模型の半分**の歩幅で辿り始めていました。
    //
    // 潰れているかどうかだけを見て、大きさは測ったとおりに返します。
    if worst > 0.0 {
        worst
    } else {
        1.0
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
/// A ruled band of a surface of revolution: the side of a cylinder, or of a
/// cone, which is the same shape with the two end radii different.
struct CylinderPatch {
    axis: Vec3,
    base_center: Point3,
    radius: f64,
    top_radius: f64,
    height: f64,
    frame_u: Vec3,
    frame_v: Vec3,
}

impl CylinderPatch {
    /// Distance along the axis from the base circle.
    fn axial_coordinate(&self, point: Point3) -> f64 {
        (point - self.base_center).dot(&self.axis)
    }

    /// The radius the patch has at a given distance along the axis.
    fn radius_at(&self, axial: f64) -> f64 {
        if self.height <= 0.0 {
            return self.radius;
        }
        self.radius + (self.top_radius - self.radius) * (axial / self.height)
    }

    /// Whether the two ends have the same radius, which makes this a cylinder.
    ///
    /// The plane cuts that give a ruling or an ellipse only do so on a
    /// cylinder; the same plane through a cone gives a hyperbola or a parabola,
    /// and neither is a parameter line of the patch.
    fn is_cylindrical(&self, tol: &Tolerance) -> bool {
        (self.top_radius - self.radius).abs()
            <= tol.linear * self.radius.max(self.top_radius).max(1.0)
    }

    /// Distance from the axis line.
    fn radial_distance(&self, point: Point3) -> f64 {
        let offset = point - self.base_center;
        (offset - self.axis * offset.dot(&self.axis)).norm()
    }

    /// Whether two points sit on the same ruling of the patch.
    ///
    /// Asking whether one is directly above the other only works on a cylinder;
    /// a cone's rulings lean inwards, so the two ends of one ruling are at
    /// different distances from the axis. What a ruling does hold constant is
    /// the angle around the axis, so that is what to compare - scaled back into
    /// a length so the tolerance means the same thing at any radius.
    fn on_same_ruling(&self, point: Point3, base: Point3, tol: &Tolerance) -> bool {
        let limit = tol.linear * 10.0;
        let (reach_point, reach_base) = (self.radial_distance(point), self.radial_distance(base));
        // 頂点はあらゆる母線の上にある。そこでは角度が定まらない。
        if reach_point.min(reach_base) <= limit {
            return true;
        }
        let gap = (self.angle_of(point) - self.angle_of(base)).abs();
        let wrapped = gap.min(std::f64::consts::PI * 2.0 - gap);
        wrapped * reach_point.max(reach_base) <= limit
    }

    /// Angle around the axis in the patch frame.
    fn angle_of(&self, point: Point3) -> f64 {
        let offset = point - self.base_center;
        offset.dot(&self.frame_v).atan2(offset.dot(&self.frame_u))
    }
}

/// Centre, radius and plane normal of a section curve that is a circle.
///
/// The normal is `None` when the section has collapsed to a point, which is
/// what the row at a cone's apex does; there is a centre and a zero radius, but
/// no plane to speak of.
fn fit_section_circle(curve: &NurbsCurve3, tol: &Tolerance) -> Option<(Point3, f64, Option<Vec3>)> {
    let samples = sample_curve_points(curve, 8);
    let origin = samples[0];
    let extent = samples
        .iter()
        .fold(0.0f64, |worst, sample| worst.max((sample - origin).norm()));
    if extent <= tol.linear {
        return Some((origin, 0.0, None));
    }

    // 断面が平面に乗っているか。乗っていなければ円ではない。
    let mut normal = Vec3::new(0.0, 0.0, 0.0);
    for window in samples.windows(3) {
        let candidate = (window[1] - window[0]).cross(&(window[2] - window[0]));
        if candidate.norm() > normal.norm() {
            normal = candidate;
        }
    }
    let normal = normal.try_normalize_safe(1e-12)?;
    if samples
        .iter()
        .any(|sample| (sample - origin).dot(&normal).abs() > tol.linear * extent.max(1.0))
    {
        return None;
    }

    let frame_u = axis_perpendicular(normal)?;
    let frame_v = normal.cross(&frame_u);
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
    let center = origin + frame_u * center_2d.x + frame_v * center_2d.y;
    let radius = (samples[0] - center).norm();
    if radius <= tol.linear {
        return None;
    }

    // 全ての標本が同じ半径にあるか。円弧でなければここで落ちる。
    if samples
        .iter()
        .any(|sample| ((sample - center).norm() - radius).abs() > tol.linear * radius.max(1.0))
    {
        return None;
    }

    Some((center, radius, Some(normal)))
}

/// Recognizes the ruled side of a cylinder or a cone, without assuming an axis.
///
/// The two boundary sections carry everything needed, so the patch is read from
/// them rather than from the control net: fit each as a circle, take the axis
/// from the line joining their centres, and then check that sections taken part
/// way along still land where those two ends imply. That last check is what
/// separates a cone from a ruled surface that merely happens to end on two
/// circles, and it is the reason the reading can be trusted rather than assumed.
///
/// Reading the control net directly, as this used to, required every ruling to
/// be the same vector. That is true of a cylinder and false of a cone, so every
/// cone was refused and no plane could be intersected with one.
fn recognize_cylinder_patch(surface: &NurbsSurface3, tol: &Tolerance) -> Option<CylinderPatch> {
    if surface.degree_v != 1 || surface.degree_u != 2 {
        return None;
    }
    if surface.control_points.len() != surface.degree_u + 1
        || surface.control_points.iter().any(|row| row.len() != 2)
    {
        return None;
    }

    let base = cylinder_section_curve(surface, 0.0)?;
    let top = cylinder_section_curve(surface, 1.0)?;
    let (base_center, radius, base_normal) = fit_section_circle(&base, tol)?;
    let (top_center, top_radius, top_normal) = fit_section_circle(&top, tol)?;

    let scale = radius.max(top_radius);
    if scale <= tol.linear {
        return None;
    }

    let span = top_center - base_center;
    let height = span.norm();
    if height <= tol.linear {
        return None;
    }
    let axis = span / height;

    // 断面の平面は軸に直交していなければならない。頂点に潰れた側は面を持たない
    // ので、そこは見ない。
    for normal in [base_normal, top_normal].into_iter().flatten() {
        if normal.cross(&axis).norm() > tol.angular {
            return None;
        }
    }

    let frame_u = axis_perpendicular(axis)?;
    let frame_v = axis.cross(&frame_u);

    // 途中の断面が、両端が張る円錐（半径が等しければ円柱）の上に乗っているか。
    for step in 1..4 {
        let alpha = step as f64 / 4.0;
        let section = cylinder_section_curve(surface, alpha)?;
        let expected_center = base_center + span * alpha;
        let expected_radius = radius + (top_radius - radius) * alpha;
        for sample in sample_curve_points(&section, 8) {
            let offset = sample - expected_center;
            let axial = offset.dot(&axis);
            let radial = (offset - axis * axial).norm();
            if axial.abs() > tol.linear * scale
                || (radial - expected_radius).abs() > tol.linear * scale
            {
                return None;
            }
        }
    }

    Some(CylinderPatch {
        axis,
        base_center,
        radius,
        top_radius,
        height,
        frame_u,
        frame_v,
    })
}

/// Any unit vector at right angles to the given one.
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
/// 接している2つの円柱面の、**接する母線**を解析的に返す。
///
/// 軸が平行で、軸間距離が `r1 + r2`（外接）か `|r1 - r2|`（内接）のときだけ。
/// **推測しません**——両方が円柱と認識でき、軸が平行で、距離が合うことを
/// 測ってから返します。1つでも外れたら `None` で、従来どおり辿ります。
///
/// 母線の範囲は、2つのパッチが軸方向で重なるぶんに切ります。
/// 軸が平行な2つの円柱・円錐面が、**1点だけで触れている**か。
///
/// # なぜ形から決めるのか
///
/// 接触では、区別したい量がいつも公差の下にあります。実測（4-196）:
/// 上向きの円錐2本が1点で触れる配置で、辿りが出した破片は相手の面の
/// **3.5e-9 外**でした——公差 `1e-6` の 1/300 で、内外の検査では
/// 落とせません。**測って決める手は4つとも外しました**（4-180、4-181、
/// 4-195、4-196）。
///
/// # 形の条件
///
/// 半径は軸方向に**線形**です（円柱は一定、円錐は一次）。だから
///
/// ```text
/// 隙間(s) = 軸間距離 − (r_A(s) + r_B(s))     外から触れる場合
/// ```
///
/// も線形で、**符号を変えずに 0 に触れるなら、触れるのは端の1点だけ**です。
/// 途中で負になるなら本当に交わっていますし、0 が続くなら**線**で接して
/// います（そちらは [`intersect_tangent_cylinder_patches`] の仕事）。
///
/// 内から触れる場合は `軸間距離 − |r_A(s) − r_B(s)|` を同じように見ます。
///
/// **最後に、その点が本当に両方の面の上にあるかを測ってから**返します。
/// 四半パッチなので、触れる場所が両方の角度の範囲に入っているとは限りません。
/// 平面の面の**縁の円**と、その平面が曲面から切り出す**断面の円**が、
/// **接するだけ**か。
///
/// # なぜ形から決めるのか
///
/// 実測（4-193、4-194）: 同一平面で内接する2つの円、外接する2つの円の
/// どちらでも、接点のまわりに**長さ 3〜5e-4 の破片**が出て、4枚の面片に
/// 共有されていました。接点は二重根なので `√(2Rδ)` 離れた2点で「交わる」と
/// 計算されるからです。**測って落とす手は4つとも壁に当たりました**
/// （4-180、4-181、4-195、4-196）。
///
/// # 形の条件
///
/// 2つの円が同一平面にあり、中心間距離が
///
/// ```text
/// r1 + r2       外から接する
/// |r1 - r2|     内から接する
/// ```
///
/// なら、触れるのは**1点だけ**です。規約 3-1 により、そこは位相を作りません。
///
/// **推測しません**——縁が本当に円か、断面が本当に円か、同一平面か、
/// 全部測ってから返します。
fn planar_rim_touches_section_circle(
    planar_face: &Face,
    plane: &PlaneSurface3,
    surface: &NurbsSurface3,
    tol: &Tolerance,
) -> bool {
    let Some((rim_center, rim_radius, rim_normal)) = planar_face_rim_circle(planar_face) else {
        return false;
    };
    let Some(patch) = recognize_cylinder_patch(surface, tol) else {
        return false;
    };
    // 断面が円になるのは、平面が軸に垂直なときだけ。
    let Some(normal) = oriented_plane_normal_of(plane).try_normalize_safe(1e-12) else {
        return false;
    };
    if normal.cross(&patch.axis).norm() > tol.angular {
        return false;
    }
    if rim_normal.cross(&patch.axis).norm() > tol.angular {
        return false;
    }
    // 断面の円は、平面の高さでの半径。
    let axial = (plane.origin - patch.base_center).dot(&patch.axis);
    if axial < -tol.linear || axial > patch.height + tol.linear {
        return false;
    }
    let section_radius = patch.radius_at(axial.clamp(0.0, patch.height));
    let section_center = patch.base_center + patch.axis * axial;

    // 2つの円が同じ平面にあるか。
    let scale = rim_radius.max(section_radius).max(1.0);
    let limit = tol.linear * scale;
    if ((section_center - rim_center).dot(&normal)).abs() > limit {
        return false;
    }

    let between = section_center - rim_center;
    let distance = (between - normal * between.dot(&normal)).norm();
    let outside = (distance - (rim_radius + section_radius)).abs() <= limit;
    let inside = (distance - (rim_radius - section_radius).abs()).abs() <= limit;
    if !(outside || inside) {
        return false;
    }
    if distance <= limit {
        // 同心。接するのではなく重なります。
        return false;
    }
    if std::env::var_os("ZENITH_POINT_TOUCH_WHY").is_some() {
        eprintln!(
            "POINTTOUCH 同一平面の円が{}接する 半径 {rim_radius:.6} と {section_radius:.6}、中心間 {distance:.6}",
            if outside { "外から" } else { "内から" }
        );
    }
    true
}

/// 平面の面の外周が**円**なら、その中心・半径・法線。
fn planar_face_rim_circle(face: &Face) -> Option<(Point3, f64, Vec3)> {
    let edges = &face.outer_wire.edges;
    if edges.is_empty() {
        return None;
    }
    let mut samples: Vec<Point3> = Vec::new();
    for oriented in edges {
        let curve = &oriented.edge.curve;
        let (t_min, t_max) = curve.param_range();
        if !(t_max > t_min) {
            continue;
        }
        for step in 0..4 {
            samples.push(curve.evaluate(t_min + (t_max - t_min) * (step as f64 / 4.0)));
        }
    }
    if samples.len() < 6 {
        return None;
    }
    let (center, radius, normal) = fit_circle_through(
        samples[0],
        samples[samples.len() / 3],
        samples[2 * samples.len() / 3],
    )?;
    // **主張で終わらせません。** 全部の標本がその円に乗るか測ります。
    let limit = radius.max(1.0) * 1e-9;
    for point in &samples {
        let offset = point - center;
        if offset.dot(&normal).abs() > limit {
            return None;
        }
        if (offset.norm() - radius).abs() > limit {
            return None;
        }
    }
    Some((center, radius, normal))
}

/// 3点を通る円。
fn fit_circle_through(a: Point3, b: Point3, c: Point3) -> Option<(Point3, f64, Vec3)> {
    let (ab, ac) = (b - a, c - a);
    let normal = ab.cross(&ac);
    let norm = normal.norm();
    let scale = ab.norm().max(ac.norm()).max(1.0);
    if norm <= scale * scale * 1e-12 {
        return None;
    }
    let normal = normal / norm;
    let center = a + (ab * ac.dot(&ac) * ab.dot(&ab).recip().recip()).scale(0.0) + {
        let denominator = 2.0 * norm * norm;
        let alpha = ac.dot(&ac) * ab.dot(&(ab - ac)) / denominator;
        let beta = ab.dot(&ab) * ac.dot(&(ac - ab)) / denominator;
        ab * alpha + ac * beta
    };
    let radius = (a - center).norm();
    Some((center, radius, normal))
}

/// 平面そのものの法線（面の向きは見ません）。
fn oriented_plane_normal_of(plane: &PlaneSurface3) -> Vec3 {
    plane.normal
}

fn revolution_patches_touch_at_a_point(
    surface_a: &NurbsSurface3,
    surface_b: &NurbsSurface3,
    tol: &Tolerance,
) -> bool {
    let (Some(patch_a), Some(patch_b)) = (
        recognize_cylinder_patch(surface_a, tol),
        recognize_cylinder_patch(surface_b, tol),
    ) else {
        return false;
    };
    if patch_a.axis.cross(&patch_b.axis).norm() > tol.angular {
        return false;
    }
    let axis = patch_a.axis;
    let facing = patch_a.axis.dot(&patch_b.axis).signum();

    let between = patch_b.base_center - patch_a.base_center;
    let across = between - axis * between.dot(&axis);
    let distance = across.norm();
    let scale = patch_a
        .radius
        .max(patch_a.top_radius)
        .max(patch_b.radius)
        .max(patch_b.top_radius)
        .max(1.0);
    let limit = tol.linear * scale;
    if !(distance > limit) {
        // 同軸。1点では触れません。
        return false;
    }

    // A の底を原点にした軸方向の座標。
    let b_low = between
        .dot(&axis)
        .min(between.dot(&axis) + facing * patch_b.height);
    let b_high = between
        .dot(&axis)
        .max(between.dot(&axis) + facing * patch_b.height);
    let low = 0.0f64.max(b_low);
    let high = patch_a.height.min(b_high);
    if !(high - low > limit) {
        return false;
    }

    let radius_a = |s: f64| patch_a.radius_at(s);
    let radius_b = |s: f64| {
        let along = if facing >= 0.0 {
            s - between.dot(&axis)
        } else {
            between.dot(&axis) - s
        };
        patch_b.radius_at(along)
    };

    // 外から触れる隙間と、内から触れる隙間。どちらも `s` について線形です。
    for inside in [false, true] {
        let gap = |s: f64| {
            if inside {
                distance - (radius_a(s) - radius_b(s)).abs()
            } else {
                distance - (radius_a(s) + radius_b(s))
            }
        };
        let (at_low, at_high) = (gap(low), gap(high));
        // **符号を変えたら、本当に交わっています。**
        if at_low < -limit && at_high < -limit {
            continue;
        }
        if at_low.min(at_high) < -limit {
            continue;
        }
        // **0 が続くなら線で接しています。** ここでは扱いません。
        if at_low.abs() <= limit && at_high.abs() <= limit {
            continue;
        }
        // 端のどちらかで 0 に触れているか。
        let touch = if at_low.abs() <= limit {
            low
        } else if at_high.abs() <= limit {
            high
        } else {
            continue;
        };

        // **触れる点が本当に両方の面の上にあるか、測ります。**
        let Some(direction) = across.try_normalize_safe(1e-12) else {
            continue;
        };
        let sign = if inside && radius_b(touch) > radius_a(touch) {
            -1.0
        } else {
            1.0
        };
        let point = patch_a.base_center + axis * touch + direction * (radius_a(touch) * sign);
        let on_both = [surface_a, surface_b].iter().all(|surface| {
            {
                zenith_geom::work_counter::count_tangent_patch_projection();
                ExtremumEngine::point_to_surface(point, surface, 64, 1e-13)
            }
                .map(|projection| projection.distance <= limit)
                .unwrap_or(false)
        });
        if on_both {
            if std::env::var_os("ZENITH_POINT_TOUCH_WHY").is_some() {
                eprintln!(
                    "POINTTOUCH 1点で触れる（{}）({:.6} {:.6} {:.6})",
                    if inside { "内から" } else { "外から" },
                    point.x,
                    point.y,
                    point.z
                );
            }
            return true;
        }
    }
    false
}

fn intersect_tangent_cylinder_patches(
    surface_a: &NurbsSurface3,
    surface_b: &NurbsSurface3,
    tol: &Tolerance,
) -> Option<FaceIntersectionKind> {
    let patch_a = recognize_cylinder_patch(surface_a, tol)?;
    let patch_b = recognize_cylinder_patch(surface_b, tol)?;
    if !patch_a.is_cylindrical(tol) || !patch_b.is_cylindrical(tol) {
        return None;
    }
    // 軸が平行か。向きは逆でも構いません。
    if patch_a.axis.cross(&patch_b.axis).norm() > tol.angular {
        return None;
    }
    let axis = patch_a.axis;

    // 軸から軸への、軸に直交する隔たり。
    let between = patch_b.base_center - patch_a.base_center;
    let across = between - axis * between.dot(&axis);
    let distance = across.norm();
    let (radius_a, radius_b) = (patch_a.radius, patch_b.radius);
    let scale = radius_a.max(radius_b).max(1.0);

    // 外接か内接か。**どちらでもなければ接していません。**
    let outside = (distance - (radius_a + radius_b)).abs() <= tol.linear * scale;
    let inside = (distance - (radius_a - radius_b).abs()).abs() <= tol.linear * scale;
    if !(outside || inside) {
        return None;
    }
    let direction = across.try_normalize_safe(1e-12)?;

    // 接する点は、A の軸から半径ぶん。内接で B のほうが大きいなら逆向きです。
    let sign = if outside || radius_a >= radius_b {
        1.0
    } else {
        -1.0
    };
    let foot = patch_a.base_center + direction * (radius_a * sign);

    // 軸方向に、2つのパッチが重なるぶんだけ。
    let axial_of = |point: Point3| (point - foot).dot(&axis);
    let (a_low, a_high) = {
        let low = axial_of(patch_a.base_center);
        (low, low + patch_a.height)
    };
    let (b_low, b_high) = {
        let low = axial_of(patch_b.base_center);
        (low, low + patch_b.height)
    };
    let low = a_low.max(b_low);
    let high = a_high.min(b_high);
    if !(high - low > tol.linear * 100.0) {
        return None;
    }

    // **主張で終わらせません。** 取り出した線が本当に両方の面に乗るか測ります。
    let start = foot + axis * low;
    let end = foot + axis * high;
    for step in 0..=8 {
        let point = start + (end - start) * (step as f64 / 8.0);
        for patch in [surface_a, surface_b] {
            zenith_geom::work_counter::count_tangent_patch_projection();
            let Ok(projection) = ExtremumEngine::point_to_surface(point, patch, 64, 1e-13) else {
                return None;
            };
            if projection.distance > tol.linear * scale {
                return None;
            }
        }
    }

    Some(FaceIntersectionKind::Line {
        point: start,
        direction: axis,
        segment_start: start,
        segment_end: end,
    })
}

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
        // 円柱でも円錐でもない面。それでも、平面に平行な等パラメータ線を
        // 持っているなら断面はその線として厳密に取り出せる。トーラスがこれ。
        return intersect_plane_by_iso_section(plane, normal, surface, tol);
    };

    if normal.cross(&patch.axis).norm() <= tol.angular {
        return intersect_section_plane_cylinder_patch(plane, surface, &patch, tol);
    }
    // 軸に平行な平面が母線を切り、斜めの平面が楕円を切るのは円柱の話。
    // 円錐では双曲線・放物線になり、いずれもパッチのパラメータ線ではない。
    if !patch.is_cylindrical(tol) {
        return FaceIntersectionKind::Unsupported;
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

/// Intersects a plane with any patch that has an iso-line lying in it.
///
/// A surface of revolution cut square to its axis meets the plane along one of
/// its own parameter lines, whatever the surface is: a cylinder, a cone, a
/// torus. The line is exact - it comes out of the control net - and it runs
/// from one edge of the patch to the other, which is what the split stage
/// needs.
///
/// Nothing here recognizes a shape. It asks two questions of the patch and
/// takes the answers: does the distance along the plane's normal depend on one
/// parameter alone, and is there a value of that parameter where the distance
/// is zero? Then it checks the line it found really does lie in the plane, and
/// refuses if it does not. A patch that is not a surface of revolution about
/// this normal fails the first question; one the plane misses fails the second.
///
/// Both parameter directions are tried, because which one carries the axial
/// direction is a matter of how the builder laid the patch out: a cylinder's
/// runs along v, a torus's along u.
fn intersect_plane_by_iso_section(
    plane: &PlaneSurface3,
    plane_normal: Vec3,
    surface: &NurbsSurface3,
    tol: &Tolerance,
) -> FaceIntersectionKind {
    for along_u in [false, true] {
        if let Some(kind) = iso_section_along(plane, plane_normal, surface, tol, along_u) {
            return kind;
        }
    }
    FaceIntersectionKind::Unsupported
}

fn iso_section_along(
    plane: &PlaneSurface3,
    plane_normal: Vec3,
    surface: &NurbsSurface3,
    tol: &Tolerance,
    along_u: bool,
) -> Option<FaceIntersectionKind> {
    let ((u_min, u_max), (v_min, v_max)) = surface.param_range();
    if !(u_max > u_min && v_max > v_min) {
        return None;
    }
    // 断面を決めるほうのパラメータと、その線に沿って動くほうのパラメータ。
    let (section_min, section_max) = if along_u {
        (u_min, u_max)
    } else {
        (v_min, v_max)
    };
    let (along_min, along_max) = if along_u {
        (v_min, v_max)
    } else {
        (u_min, u_max)
    };

    let evaluate = |section: f64, along: f64| {
        if along_u {
            surface.evaluate(section, along)
        } else {
            surface.evaluate(along, section)
        }
    };
    let offset_at =
        |section: f64, along: f64| (evaluate(section, along) - plane.origin).dot(&plane_normal);

    let extent = (surface.evaluate(u_max, v_max) - surface.evaluate(u_min, v_min))
        .norm()
        .max(1.0);
    let limit = tol.linear * extent;

    // 法線方向の距離が、断面を決めるパラメータだけで決まるか。もう一方を
    // 動かして変わるようなら、この向きは回転軸ではない。
    let offset_of = |section: f64| offset_at(section, along_min);
    for step in 0..=4 {
        let section = section_min + (section_max - section_min) * step as f64 / 4.0;
        let reference = offset_of(section);
        for along_step in 1..=4 {
            let along = along_min + (along_max - along_min) * along_step as f64 / 4.0;
            if (offset_at(section, along) - reference).abs() > limit {
                return None;
            }
        }
    }

    let (low, high) = (offset_of(section_min), offset_of(section_max));
    if low.min(high) > limit || low.max(high) < -limit {
        return None;
    }
    if (high - low).abs() <= limit {
        // パッチ全体が平面と同じ高さにある。断面ではなく重なりなので、
        // ここで promote するものは無い。
        return None;
    }

    // 単調でなければ交わりが2本以上あり得る。分割段は面の組ごとに1本しか
    // 受け取れないので、そこは promote しない。
    let mut previous = low;
    for step in 1..=16 {
        let section = section_min + (section_max - section_min) * step as f64 / 16.0;
        let current = offset_of(section);
        if (current - previous) * (high - low) < -limit {
            return None;
        }
        previous = current;
    }

    // 二分法。単調なので挟み撃ちで決まる。
    let (mut lower, mut upper) = if low <= high {
        (section_min, section_max)
    } else {
        (section_max, section_min)
    };
    for _ in 0..80 {
        let middle = 0.5 * (lower + upper);
        if offset_of(middle) < 0.0 {
            lower = middle;
        } else {
            upper = middle;
        }
    }
    let section = 0.5 * (lower + upper);

    let curve = if along_u {
        surface.iso_curve_u(section)?
    } else {
        surface.iso_curve_v(section)?
    };

    // 主張で終わらせない。取り出した線が本当に平面の上にあるか測る。
    let (t_min, t_max) = curve.param_range();
    for step in 0..=16 {
        let point = curve.evaluate(t_min + (t_max - t_min) * step as f64 / 16.0);
        if (point - plane.origin).dot(&plane_normal).abs() > limit {
            return None;
        }
    }

    let start = curve.evaluate(t_min);
    let end = curve.evaluate(t_max);
    Some(FaceIntersectionKind::Curve {
        edge: Edge::new(
            curve,
            Vertex::new(start, tol.linear),
            Vertex::new(end, tol.linear),
            tol.linear,
        ),
    })
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
    let gap = (curve.evaluate(t) - point).norm();
    if gap > tol.linear * 10.0 {
        // **どれだけ届かなかったか**を出します（4-303）。読んだ立体は面が
        // 5.6e-4 まで粗さを持つので（4-285）、**絶対 1e-5 では刻み込めない**
        // 可能性があります。刻めなければ、片方だけが割ったまま残ります。
        // **惜しかったものだけを出します。** 全部出すと 41,672 行になり、
        // そのほとんどは「そもそも別の場所にある点」です（実測: 外れの中央値
        // 0.54）。**見たいのは、受け入れ幅のすぐ外にいるもの**です。
        if std::env::var_os("ZENITH_IMPRINT_WHY").is_some() && gap < tol.linear * 1000.0 {
            eprintln!(
                "IMPRINTWHY 稜に刻めませんでした: 外れ {gap:.6e}（受け入れ {:.6e}）",
                tol.linear * 10.0
            );
        }
        return None;
    }

    let span = t_max - t_min;
    if t <= t_min + span * 1e-9 || t >= t_max - span * 1e-9 {
        return None;
    }

    Some(t)
}

/// **相手が既に持っている稜のうち、こちらの平面をまたいでいるもの**を、
/// 交線の候補として拾う。
///
/// # なぜ要るか
///
/// 切る平面が、相手の立体の**継ぎ目にちょうど重なる**ことがあります。球を
/// 中心を通る平面で切ると、切り口の大円は球のパッチの経線そのものです
/// （実測: `box × sphere` で、球が持つ8本の稜が `x = 20` の上に**厳密に**
/// 乗っています。長さは 15.705 = πr/2）。
///
/// このとき、**マーチングは何も見つけません**。どのパッチも平面を内部で
/// 横切っておらず、境界で接しているだけだからです（実測: 16組すべてで
/// 「0 branch」。HANDOVER 4-78 の欠陥1）。辿る必要はありません——
/// **交線はもう相手の稜として存在します。**
///
/// # 何を「またいでいる」と呼ぶか
///
/// 稜が平面の上に乗っているだけでは足りません。**その稜を共有する相手の2枚が、
/// 平面の反対側にある**ことを見ます。
///
/// - 球の経線: 隣り合うパッチは経度 0..90 と 90..180 で、`x = 20` の反対側
///   にあります。**またいでいる**
/// - 箱どうしを面で合わせた配置: 合わさっている面は平面の**上**にあり、
///   反対側にはなりません。**またいでいない**（同一平面の重なりは別の経路が
///   扱います）
///
/// これを見ないと、面を突き合わせただけの配置に交線を作ってしまいます。
fn collect_edges_already_on_a_plane(
    planar_faces: &[Face],
    other_faces: &[Face],
    tol: &Tolerance,
) -> Vec<IntersectionEdgeCandidate> {
    let mut out = Vec::new();

    for (face_a_index, face_a) in planar_faces.iter().enumerate() {
        let FaceGeometry::Plane(plane) = &face_a.geometry else {
            continue;
        };

        for (face_b_index, face_b) in other_faces.iter().enumerate() {
            for oriented in &face_b.outer_wire.edges {
                let edge = &oriented.edge;
                if !edge_lies_on_plane(edge, plane, tol) {
                    continue;
                }
                if sampled_edge_extent(edge) <= tol.linear {
                    continue;
                }
                // 平面の面の中に入っていなければ、この面は割れません。
                if !edge_midpoint_inside_planar_face(face_a, plane, edge, tol) {
                    continue;
                }
                if !other_solid_crosses_here(other_faces, edge, plane, tol) {
                    continue;
                }
                if std::env::var_os("ZENITH_ONPLANE_WHY").is_some() {
                    let s = edge.start_vertex.point;
                    let e = edge.end_vertex.point;
                    eprintln!(
                        "ONPLANE face {face_a_index} takes edge {} ({:.3} {:.3} {:.3})->({:.3} {:.3} {:.3}) from face {face_b_index}",
                        edge.id, s.x, s.y, s.z, e.x, e.y, e.z
                    );
                }
                out.push(IntersectionEdgeCandidate {
                    face_a_index,
                    face_b_index,
                    edge: edge.clone(),
                });
            }
        }
    }

    out
}

/// 稜の中点が、平面の面のトリムの内側にあるか。
fn edge_midpoint_inside_planar_face(
    face: &Face,
    plane: &PlaneSurface3,
    edge: &Edge,
    tol: &Tolerance,
) -> bool {
    let boundary: Vec<Point2> = face
        .outer_wire
        .sample_points(16)
        .iter()
        .map(|point| project_to_plane_uv(*point, plane))
        .collect();
    if boundary.len() < 3 {
        return false;
    }
    let uv = project_to_plane_uv(edge_midpoint(edge), plane);
    point_in_polygon_2d(uv, &boundary, tol.parametric)
}

/// この稜のところで、相手の立体が平面をまたいでいるか。
///
/// 稜を共有する面を集め、その面の代表点が平面のどちら側にあるかを見ます。
/// 両側にあれば、またいでいます。
fn other_solid_crosses_here(
    other_faces: &[Face],
    edge: &Edge,
    plane: &PlaneSurface3,
    tol: &Tolerance,
) -> bool {
    let mut positive = false;
    let mut negative = false;

    // **`id` では突き合わせられません。** 同じ弧を共有していても、面ごとに
    // 別の `Edge` の実体を持っていることがあります（実測: 球の経線は隣り合う
    // パッチで id 13 と 21 でした。座標は同じです）。位置で見ます。
    let start = edge.start_vertex.point;
    let end = edge.end_vertex.point;
    let middle = edge_midpoint(edge);
    let same_edge = |other: &Edge| {
        let other_start = other.start_vertex.point;
        let other_end = other.end_vertex.point;
        let spans_match = (points_same_3d(other_start, start, tol.linear)
            && points_same_3d(other_end, end, tol.linear))
            || (points_same_3d(other_start, end, tol.linear)
                && points_same_3d(other_end, start, tol.linear));
        spans_match && points_same_3d(edge_midpoint(other), middle, tol.linear * 10.0)
    };

    for face in other_faces {
        let uses_edge = std::iter::once(&face.outer_wire)
            .chain(face.inner_wires.iter())
            .flat_map(|wire| wire.edges.iter())
            .any(|oriented| same_edge(&oriented.edge));
        if !uses_edge {
            continue;
        }
        // 面の代表点（境界の標本の平均）で側を決めます。面が平面の上に
        // 丸ごと乗っているなら、どちらでもありません。
        let points = face.outer_wire.sample_points(16);
        if points.is_empty() {
            continue;
        }
        let mut sum = 0.0;
        let mut count = 0.0;
        for point in &points {
            sum += (point - plane.origin).dot(&plane.normal);
            count += 1.0;
        }
        let side = sum / count;
        if side > tol.linear {
            positive = true;
        } else if side < -tol.linear {
            negative = true;
        }
    }

    positive && negative
}

/// 面片が面積を囲んでいないか。
///
/// 外側のワイヤを折れ線に落とし、Newell の式でベクトル面積を取ります。行って
/// 戻るだけのワイヤはここで**ちょうど 0** になります。曲がった面でも折れ線は
/// 面積を過小評価するだけなので、健全な面が 0 と判定されることはありません。
///
/// 判定は長さで正規化します（`面積 <= 公差 × 周長`）。「平均して公差より薄い」
/// という意味で、大きさの単位に依りません。
fn face_encloses_no_area(face: &Face, tol: &Tolerance) -> bool {
    let points = face.outer_wire.sample_points(8);
    if points.len() < 3 {
        return true;
    }

    // **重心を引いてから足します。** 原点から遠い面では、大きな数どうしの
    // 差になって桁が落ちます。和は平行移動で変わらないので、形は動きません。
    let mut center = Vec3::zeros();
    for point in &points {
        center += point.coords;
    }
    center /= points.len() as f64;

    let mut vector_area = Vec3::zeros();
    let mut perimeter = 0.0;
    for index in 0..points.len() {
        let current = points[index].coords - center;
        let next = points[(index + 1) % points.len()].coords - center;
        vector_area += current.cross(&next);
        perimeter += (next - current).norm();
    }
    let area = vector_area.norm() * 0.5;

    perimeter <= tol.linear || area <= tol.linear * perimeter
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

    if std::env::var_os("ZENITH_OVERLAP_WHY").is_some() {
        // **同一平面に乗る2枚が、どれだけ重なっているか。**
        //
        // いまの判定は「面全体の一致」（重心と広がり）なので、部分的な
        // 重なりは捕まりません。実測（4-124）で、残るメッシュ非多様体は
        // 全件この形でした。**まず重なりの大きさを測ります。**
        for left in 0..pieces.len() {
            for right in (left + 1)..pieces.len() {
                let (FaceGeometry::Plane(left_plane), FaceGeometry::Plane(right_plane)) =
                    (&pieces[left].face.geometry, &pieces[right].face.geometry)
                else {
                    continue;
                };
                let (Some(left_normal), Some(right_normal)) = (
                    selected_piece_normal(&pieces[left], left_plane),
                    selected_piece_normal(&pieces[right], right_plane),
                ) else {
                    continue;
                };
                if left_normal.cross(&right_normal).norm() > 1e-9 {
                    continue;
                }
                if (right_plane.origin - left_plane.origin)
                    .dot(&left_normal)
                    .abs()
                    > tol.linear * 10.0
                {
                    continue;
                }
                let left_points = pieces[left].face.outer_wire.sample_points(16);
                let right_points = pieces[right].face.outer_wire.sample_points(16);
                if left_points.is_empty() || right_points.is_empty() {
                    continue;
                }
                let centre = |points: &[Point3]| {
                    let mut sum = Vec3::zeros();
                    for point in points {
                        sum += point.coords;
                    }
                    Point3::from(sum / points.len() as f64)
                };
                let (lc, rc) = (centre(&left_points), centre(&right_points));
                eprintln!(
                    "OVERLAPWHY 同一平面の2枚: {:?} 面 id {} と {:?} 面 id {}、重心の隔たり {:.4}、向き {}",
                    pieces[left].operand,
                    pieces[left].face.id,
                    pieces[right].operand,
                    pieces[right].face.id,
                    (rc - lc).norm(),
                    if left_normal.dot(&right_normal) > 0.0 {
                        "同じ"
                    } else {
                        "逆"
                    }
                );
            }
        }
    }

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
        // **平面でなければ諦める、ではありません。**
        //
        // ここは長らく平面だけを見ていました。**曲面の重なりは一度も
        // 解消されていません**でした。実測（4-134）: まったく同じ球の和は
        // **重なった立体を2つ**返し、体積が 1047.19755（真値の2倍）に
        // なります。トーラスも同じ。円柱・円錐・球・トーラスの差と積は
        // 断られます。**箱だけが通っていました。**
        return curved_coincident_face_direction(left, right, tol);
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

/// 曲面の面片どうしが、**同じ場所を同じ向きで**占めているか。
///
/// 平面の判定（[`coincident_face_direction`]）と同じ考え方を、曲面へ
/// 広げたものです。平面は「同一平面か」を法線と原点で決められますが、
/// 曲面はそうはいかないので、**面の上の点を相手の面へ当てて**測ります。
///
/// 採るのは次を**全部**満たすときだけです。平面側と同じ厳しさにして
/// あります——**緩めると、重なっていない面まで落ちます。**
///
/// 1. 境界の標本の重心と広がりが一致する（平面側と同じ、相対 1e-6）
/// 2. 片方の代表点が、もう片方の曲面の上に乗っている
/// 3. その点で法線の向きが決まる（同じか、逆か）
fn curved_coincident_face_direction(
    left: &SelectedBooleanFacePiece,
    right: &SelectedBooleanFacePiece,
    tol: &Tolerance,
) -> Option<bool> {
    let left_points = left.face.outer_wire.sample_points(16);
    let right_points = right.face.outer_wire.sample_points(16);
    if left_points.len() < 3 || right_points.len() < 3 {
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

    // **境界が同じでも、中身が同じとは限りません。** 面の中の点を1つ取って、
    // 相手の曲面の上に乗っているかを見ます。
    let sample = representative_face_point(&left.face);
    let left_normal = piece_normal_at(left, sample, tol)?;
    let right_normal = piece_normal_at(right, sample, tol)?;

    Some(left_normal.dot(&right_normal) > 0.0)
}

/// 面片が組み上がった結果で持つ外向き法線を、3D の点のところで求める。
///
/// 点が面の曲面の上に乗っていなければ `None` です（別の場所にある面）。
fn piece_normal_at(
    piece: &SelectedBooleanFacePiece,
    point: Point3,
    tol: &Tolerance,
) -> Option<Vec3> {
    let mut normal = match &piece.face.geometry {
        FaceGeometry::Plane(plane) => {
            if (point - plane.origin).dot(&plane.normal).abs() > tol.linear * 10.0 {
                return None;
            }
            plane.normal.try_normalize(1e-12)?
        }
        FaceGeometry::Nurbs(surface) => {
            zenith_geom::work_counter::count_piece_normal_projection();
            let projection =
                { ExtremumEngine::point_to_surface(point, surface, 32, tol.parametric).ok()? };
            // 乗っていなければ、同じ場所の面ではありません。
            let scale = (point - Point3::origin()).norm().max(1.0);
            if projection.distance > tol.linear * 100.0 * scale {
                return None;
            }
            surface.normal(projection.u, projection.v)?
        }
        _ => return None,
    };
    if !piece.face.orientation.is_forward() {
        normal = -normal;
    }
    if piece.reverse_orientation {
        normal = -normal;
    }
    Some(normal)
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

/// 切り込みが**穴を横切る**平面の面を割る。
///
/// 円環の面（外周＋穴）を、外周から穴まで走る切り込みで割る配置がこれです。
/// 通常の分割は切り込みの端を**外側のワイヤにしか探さない**ので
/// 「Split edge end does not lie on the outer boundary」で断ります。穴が
/// 片方の片に丸ごと入る配置は `distribute_inner_wires` が扱いますが、
/// **穴自体が割れる**配置はそこでも扱えません。
///
/// ここは uv の平面アレンジメントを組みます。外側・内側すべてのワイヤを
/// 着地点で細分し、切り込みを加えて、**入ってきた向きの逆から時計回りに
/// 1つ手前の弧**へ進む標準の巡回で面を取り出します。
///
/// **最後の受け皿です。** 既存の経路が通る配置はここへ来ません。
///
/// 取り出した領域の面積の和が元の面積に戻らなければ断ります。閉じた輪に
/// なっただけでは、領域の取り違えは分かりません。
fn split_planar_face_across_holes(
    face: &Face,
    split_edges: &[Edge],
    tol: &Tolerance,
) -> Result<Vec<Face>, String> {
    let FaceGeometry::Plane(plane) = &face.geometry else {
        return Err("Only planar faces can be split across a hole".to_string());
    };
    if face.inner_wires.is_empty() {
        return Err("This face has no holes to cut across".to_string());
    }

    let wires: Vec<&Wire> = std::iter::once(&face.outer_wire)
        .chain(face.inner_wires.iter())
        .collect();

    // 1. 切り込みの端が、どのワイヤのどこに乗るか。
    let cuts = deduplicate_split_edges(split_edges, tol);
    let mut landings: Vec<Vec<WireHit>> = vec![Vec::new(); wires.len()];
    let mut crosses_a_hole = false;
    for cut in &cuts {
        // **どの点かを言います。** 断り文が「どこかの端」としか言わないと、
        // 追いかける人は面の全部の稜を自分で測ることになります。
        let name = |point: zenith_math::Point3| {
            format!(
                "a cut end ({:.4} {:.4} {:.4}) does not lie on any wire of the face",
                point.x, point.y, point.z
            )
        };
        // **切り込みどうしの継ぎ目は、境界に乗らなくてかまいません。**
        // 切り込みが鎖になっていると、中の継ぎ目は面の内側にあります。実測:
        // 輪を傾けたスラブで切ると、鎖の継ぎ目が半径 9.9137（外周 10、穴 4）
        // に来て、「どのワイヤにも乗らない」と断っていました（4-66）。
        // 境界に着かなければならないのは、**相手のいない端**だけです。
        let shared_with_another_cut = |point: zenith_math::Point3| {
            cuts.iter()
                .flat_map(|other| [other.start_vertex.point, other.end_vertex.point])
                .filter(|other| (*other - point).norm() <= tol.linear * 1000.0)
                .count()
                >= 2
        };
        let start = locate_on_any_wire(&wires, cut.start_vertex.point, tol);
        let end = locate_on_any_wire(&wires, cut.end_vertex.point, tol);
        if start.is_none() && !shared_with_another_cut(cut.start_vertex.point) {
            return Err(name(cut.start_vertex.point));
        }
        if end.is_none() && !shared_with_another_cut(cut.end_vertex.point) {
            return Err(name(cut.end_vertex.point));
        }
        // **穴に関わっているかどうかで決めます。** 「外周と穴をつなぐ切り込みが
        // あるか」で見ていたので、穴の縁だけに着く切り込み——傾けたドリルが
        // 円環の中を噛む配置がそれです——を断っていました（4-66）。
        // ここは最後の受け皿なので、普通の経路で割れる面はここへ来ません。
        for landing in [&start, &end].into_iter().flatten() {
            if landing.0 > 0 {
                crosses_a_hole = true;
            }
        }
        if let Some(start) = start {
            landings[start.0].push(start.1);
        }
        if let Some(end) = end {
            landings[end.0].push(end.1);
        }
    }
    if !crosses_a_hole {
        // 穴に関わっていないなら、ここの仕事ではありません。
        return Err("no cut reaches an inner wire".to_string());
    }

    // 2. ワイヤを着地点で細分して弧にする。着地の無いワイヤは丸ごと残し、
    //    最後にどの領域の穴かを決めます。
    let mut arcs: Vec<Arc> = Vec::new();
    let mut nodes: Vec<Point3> = Vec::new();
    let mut free_wires: Vec<Wire> = Vec::new();
    for (index, wire) in wires.iter().enumerate() {
        let mut hits = landings[index].clone();
        if hits.is_empty() {
            if index > 0 {
                // 触られていない穴は、あとで含む領域へ配ります。
                free_wires.push((*wire).clone());
                continue;
            }
            // **外周は、切り込みが当たっていなくてもグラフに要ります。**
            // 落とすと、外周に囲まれた領域そのものが巡回に出てきません。
            // 円環の穴の縁から出て穴の縁へ戻る切り込みでは、答えの片方が
            // 「外周を外側の輪、[穴の弧＋切り込み] を内側の輪」に持つ面に
            // なりますが、外周が無いとそれを組み立てられません（4-67）。
            let mut pieces: Vec<Vec<OrientedEdge>> = wire
                .edges
                .iter()
                .map(|oriented| vec![oriented.clone()])
                .collect();
            // 閉じた稜が1本だけの輪（全周の円がこれ）は、そのままだと
            // 始点と終点が同じで弧になりません。半分に割ります。
            if pieces.len() == 1 {
                let oriented = &wire.edges[0];
                let Ok(Some(first)) = oriented_edge_portion(oriented, 0.0, 0.5, tol) else {
                    continue;
                };
                let Ok(Some(second)) = oriented_edge_portion(oriented, 0.5, 1.0, tol) else {
                    continue;
                };
                pieces = vec![vec![first], vec![second]];
            }
            for path in pieces {
                let start = node_index(&mut nodes, oriented_start_point(&path[0]), tol);
                let end = node_index(&mut nodes, oriented_end_point(&path[path.len() - 1]), tol);
                if start == end {
                    continue;
                }
                arcs.push(Arc {
                    from: start,
                    to: end,
                    path,
                });
            }
            continue;
        }
        hits.sort_by(|a, b| a.edge_index.cmp(&b.edge_index).then(a.t.total_cmp(&b.t)));
        hits.dedup_by(|a, b| a.edge_index == b.edge_index && (a.t - b.t).abs() <= 1e-9);
        if hits.len() < 2 {
            return Err("a wire was met by the cut only once".to_string());
        }
        for step in 0..hits.len() {
            let from = &hits[step];
            let to = &hits[(step + 1) % hits.len()];
            let path = wire_path_between(&wire.edges, from, to, tol)?;
            if path.is_empty() {
                continue;
            }
            let start = node_index(&mut nodes, oriented_start_point(&path[0]), tol);
            let end = node_index(&mut nodes, oriented_end_point(&path[path.len() - 1]), tol);
            if start == end {
                continue;
            }
            arcs.push(Arc {
                from: start,
                to: end,
                path,
            });
        }
    }

    // 3. 切り込みを弧として加えます。
    for cut in &cuts {
        let start_point = cut.start_vertex.point;
        let end_point = cut.end_vertex.point;
        let start = node_index(&mut nodes, start_point, tol);
        let end = node_index(&mut nodes, end_point, tol);
        if start == end {
            continue;
        }
        let forward = OrientedEdge::new(cut.clone(), zenith_topo::Orientation::Forward);
        let runs_forward = (forward.evaluate_normalized(0.0) - start_point).norm()
            <= (forward.evaluate_normalized(1.0) - start_point).norm();
        let forward = if runs_forward {
            forward
        } else {
            OrientedEdge::new(cut.clone(), zenith_topo::Orientation::Reversed)
        };
        arcs.push(Arc {
            from: start,
            to: end,
            path: vec![forward],
        });
    }

    // 弧はすべて両向きに持ちます。巡回はそれを前提にします。
    let mut directed: Vec<Arc> = Vec::new();
    for arc in arcs {
        let reversed = Arc {
            from: arc.to,
            to: arc.from,
            path: arc
                .path
                .iter()
                .rev()
                .map(|oriented| {
                    OrientedEdge::new(oriented.edge.clone(), oriented.orientation.reversed())
                })
                .collect(),
        };
        directed.push(arc);
        directed.push(reversed);
    }

    // 4. 節点ごとに、出ていく弧を uv の角度で並べます。
    let departure = |arc: &Arc| -> f64 {
        let first = &arc.path[0];
        let a = project_to_plane_uv(first.evaluate_normalized(0.0), plane);
        let b = project_to_plane_uv(first.evaluate_normalized(0.02), plane);
        (b.y - a.y).atan2(b.x - a.x)
    };
    let mut out_of: Vec<Vec<usize>> = vec![Vec::new(); nodes.len()];
    for (index, arc) in directed.iter().enumerate() {
        out_of[arc.from].push(index);
    }
    for list in out_of.iter_mut() {
        list.sort_by(|a, b| departure(&directed[*a]).total_cmp(&departure(&directed[*b])));
    }
    // 逆向きの弧の番号。両向きを続けて入れたので、偶奇の反転で出ます。
    let reverse_of = |index: usize| if index % 2 == 0 { index + 1 } else { index - 1 };

    // 5. 巡回。入ってきた弧の逆から見て、**反時計回りに1つ手前**の弧へ進むと、
    //    有界な面は反時計回り（符号付き面積が正）で取れます。
    let mut visited = vec![false; directed.len()];
    let mut cycles: Vec<Vec<usize>> = Vec::new();
    for seed in 0..directed.len() {
        if visited[seed] {
            continue;
        }
        let mut cycle = Vec::new();
        let mut current = seed;
        while !visited[current] {
            visited[current] = true;
            cycle.push(current);
            let back = reverse_of(current);
            let node = directed[back].from;
            let ring = &out_of[node];
            let Some(position) = ring.iter().position(|candidate| *candidate == back) else {
                return Err("the arrangement lost an arc at a node".to_string());
            };
            current = ring[(position + ring.len() - 1) % ring.len()];
        }
        if cycle.len() >= 2 {
            cycles.push(cycle);
        }
    }

    // 6. 元の面の内側にある巡回だけ残します。
    let outer_polygon: Vec<Point2> = face
        .outer_wire
        .sample_points(96)
        .iter()
        .map(|point| project_to_plane_uv(*point, plane))
        .collect();
    let hole_polygons: Vec<Vec<Point2>> = face
        .inner_wires
        .iter()
        .map(|wire| {
            wire.sample_points(96)
                .iter()
                .map(|point| project_to_plane_uv(*point, plane))
                .collect()
        })
        .collect();

    // **元の面と同じ巻き方で返します。** アレンジメントの巡回は uv で
    // 反時計回りの有界面を出しますが、面が裏向きなら内側は時計回りです。
    // 揃えないと、縫合で「同じ向きに2度使われた稜」が出ます（4-46 と同じ
    // 症状。実測で 28 本出ました）。
    let face_winding = signed_area_2d(&outer_polygon).signum();
    let mut regions: Vec<Wire> = Vec::new();
    let mut outlines: Vec<Vec<Point2>> = Vec::new();
    // 面ごとに、その巡回が通った節点。**外形をどの面に付けるかを決めるのに
    // 要ります。** 同じ連結成分の外形を自分の面に付けると、面積が 0 になって
    // しまいます（実測でそうなりました。外周の外形が外周の面の穴になり、
    // 314.16 の面が 0 になりました）。
    let mut region_nodes: Vec<std::collections::BTreeSet<usize>> = Vec::new();
    // 連結成分の外形（面積が負の閉路を裏返したもの）と、通った節点。
    let mut component_outlines: Vec<(Wire, Vec<Point2>, std::collections::BTreeSet<usize>)> =
        Vec::new();
    for cycle in &cycles {
        let mut edges = Vec::new();
        let mut touched: std::collections::BTreeSet<usize> = std::collections::BTreeSet::new();
        for index in cycle {
            edges.extend(directed[*index].path.iter().cloned());
            touched.insert(directed[*index].from);
            touched.insert(directed[*index].to);
        }
        let wire = Wire::new(edges);
        let polygon: Vec<Point2> = wire
            .sample_points(96)
            .iter()
            .map(|point| project_to_plane_uv(*point, plane))
            .collect();
        if polygon.len() < 3 {
            continue;
        }
        // **面積が負の閉路は、その連結成分の「外形」です。** 捨てずに取って
        // おいて、あとでそれを含む面の穴にします。外周と穴まわりが繋がって
        // いない配置では、答えの片方が穴のある面になり、その穴がこれです
        // （4-67）。
        if signed_area_2d(&polygon) <= 0.0 {
            let mut reversed_polygon = polygon.clone();
            reversed_polygon.reverse();
            // **穴の輪は、外周と逆向きでなければなりません。** 巡回が出す
            // 外形は既に外周と逆向き（面積が負）なので、そのまま持ちます。
            // 裏返して持つと、縫合で「同じ向きに2度使われた稜」が出ます
            // （実測で 12 本。片は1枚だけで、それが相手全部と衝突しました）。
            // 判定に使う多角形のほうだけ、向きを揃えたものを持ちます。
            component_outlines.push((wire, reversed_polygon, touched));
            continue;
        }
        // **代表点は穴を避けて取ります。** 外周に囲まれた領域の重心は、
        // 円環では穴の中に落ちます。そこで内外を訊くと、正しい領域が
        // 「穴の中だから面ではない」と捨てられます（4-67）。
        let Some(inside) = interior_sample_2d(&polygon, &hole_polygons, tol.parametric) else {
            continue;
        };
        if !point_in_polygon_2d(inside, &outer_polygon, tol.parametric) {
            continue;
        }
        regions.push(if face_winding < 0.0 {
            reversed_wire(&wire)
        } else {
            wire
        });
        outlines.push(polygon);
        region_nodes.push(touched);
    }

    if regions.len() < 2 {
        return Err("cutting across the hole did not divide the face".to_string());
    }

    // 8. 連結成分の外形を、それを含むいちばん小さい面の穴にします。
    //
    // 外周と、穴まわり（穴の弧＋切り込み）が繋がっていない配置では、答えの
    // 片方が**穴のある面**になります。巡回は連結成分ごとにしか閉路を出せない
    // ので、その穴は「別の成分の外形」として出てきます。
    let mut holes_for: Vec<Vec<Wire>> = vec![Vec::new(); regions.len()];
    for (outline_wire, outline_polygon, outline_nodes) in &component_outlines {
        let Some(sample) = interior_sample_2d(outline_polygon, &[], tol.parametric) else {
            continue;
        };
        // 含む面のうち、いちばん小さいものに付けます。
        let mut best: Option<(usize, f64)> = None;
        for (index, region_polygon) in outlines.iter().enumerate() {
            // **同じ連結成分の面には付けません。** 外形はその成分の外側の
            // 縁なので、自分の面の穴にすると面積が 0 になります。
            if !region_nodes[index].is_disjoint(outline_nodes) {
                continue;
            }
            if !point_in_polygon_2d(sample, region_polygon, tol.parametric) {
                continue;
            }
            let area = signed_area_2d(region_polygon).abs();
            if best.is_none_or(|(_, best_area)| area < best_area) {
                best = Some((index, area));
            }
        }
        // どの面にも含まれない外形は、いちばん外側の成分のものです。
        // それは面ではなく「面の外」なので、捨てます。
        if let Some((index, _)) = best {
            // 面のほうは `face_winding` に揃えてあるので、穴はその逆にします。
            holes_for[index].push(if face_winding < 0.0 {
                reversed_wire(outline_wire)
            } else {
                outline_wire.clone()
            });
        }
    }

    let mut faces: Vec<Face> = regions
        .into_iter()
        .zip(holes_for)
        .map(|(wire, holes)| {
            Face::new(
                face.geometry.clone(),
                wire,
                holes,
                face.orientation,
                face.tolerance,
            )
        })
        .collect();
    if !free_wires.is_empty() {
        let carrier = Face::new(
            face.geometry.clone(),
            face.outer_wire.clone(),
            free_wires,
            face.orientation,
            face.tolerance,
        );
        faces = distribute_inner_wires(&carrier, faces, plane, tol)?;
    }

    // **面積の和が元に戻ること。** 閉じた輪になっただけでは、領域の
    // 取り違えは分かりません。
    //
    // 面積は uv の多角形ではなく、**面そのものの積分**で測ります。多角形は
    // 曲がった境界を弦で置き換えるので、切り込みが傾いていると誤差が判定の
    // 帯を超えます。実測: 輪を傾けたスラブで切ると相対 1.1e-5 で、帯は 1e-6
    // でした。**判定に使う量は、判定の帯より細かく測れていなければなりません**
    // （4-66）。
    let integral_params = TessellationParams::default();
    let expected = MassCalculator::compute_face_integral(face, &integral_params)
        .0
        .abs();
    let got: f64 = faces
        .iter()
        .map(|piece| {
            MassCalculator::compute_face_integral(piece, &integral_params)
                .0
                .abs()
        })
        .sum();
    if expected > 0.0 && (got - expected).abs() / expected > 1e-6 {
        return Err(format!(
            "the regions add up to {got:.6e} against the face's {expected:.6e} ({} face(s), {} outline(s))",
            faces.len(),
            component_outlines.len()
        ));
    }

    Ok(faces)
}

/// アレンジメントの1本の弧。節点から節点までの、境界または切り込みの一部。
struct Arc {
    from: usize,
    to: usize,
    path: Vec<OrientedEdge>,
}

fn oriented_start_point(edge: &OrientedEdge) -> Point3 {
    edge.evaluate_normalized(0.0)
}

fn oriented_end_point(edge: &OrientedEdge) -> Point3 {
    edge.evaluate_normalized(1.0)
}

/// 点が既にある節点と同じならその番号を、無ければ足して新しい番号を返す。
fn node_index(nodes: &mut Vec<Point3>, point: Point3, tol: &Tolerance) -> usize {
    let limit = tol.linear * 1000.0;
    if let Some(index) = nodes
        .iter()
        .position(|node| (*node - point).norm() <= limit)
    {
        return index;
    }
    nodes.push(point);
    nodes.len() - 1
}

/// 外側・内側すべてのワイヤに対して着地を探し、いちばん近いものを返す。
fn locate_on_any_wire(wires: &[&Wire], point: Point3, tol: &Tolerance) -> Option<(usize, WireHit)> {
    let mut best: Option<(f64, usize, WireHit)> = None;
    for (index, wire) in wires.iter().enumerate() {
        let Some(hit) = locate_point_on_wire(&wire.edges, point, tol) else {
            continue;
        };
        let distance = (wire.edges[hit.edge_index].evaluate_normalized(hit.t) - point).norm();
        if best
            .as_ref()
            .is_none_or(|(best_distance, _, _)| distance < *best_distance)
        {
            best = Some((distance, index, hit));
        }
    }
    best.map(|(_, index, hit)| (index, hit))
}

/// 多角形の**内側**の点を1つ返す。重心は凹形では外に出るので、辺から内側へ
/// わずかに寄せた点を順に試す。
fn interior_sample_2d(polygon: &[Point2], holes: &[Vec<Point2>], tol: f64) -> Option<Point2> {
    let usable = |candidate: Point2| {
        point_in_polygon_2d(candidate, polygon, tol)
            && !point_on_polygon_boundary(candidate, polygon, tol)
            && !holes
                .iter()
                .any(|hole| point_in_polygon_2d(candidate, hole, tol))
    };
    let count = polygon.len() as f64;
    let sum = polygon.iter().fold(Point2::new(0.0, 0.0), |sum, point| {
        Point2::new(sum.x + point.x, sum.y + point.y)
    });
    let centroid = Point2::new(sum.x / count, sum.y / count);
    if usable(centroid) {
        return Some(centroid);
    }

    let extent = polygon
        .iter()
        .map(|point| (point - centroid).norm())
        .fold(0.0_f64, f64::max)
        .max(1.0);
    let step = extent * 1e-4;
    for index in 0..polygon.len() {
        let a = polygon[index];
        let b = polygon[(index + 1) % polygon.len()];
        let mid = Point2::new((a.x + b.x) * 0.5, (a.y + b.y) * 0.5);
        let along = b - a;
        let Some(inward) = Vec2::new(-along.y, along.x).try_normalize(1e-12) else {
            continue;
        };
        for sign in [1.0_f64, -1.0] {
            let candidate = mid + inward * (step * sign);
            if usable(candidate) {
                return Some(candidate);
            }
        }
    }
    None
}

/// 面の外周が、その面の向きに対して正しく巻かれているか。
///
/// p-curve の符号付き面積を面の向きで符号調整したものが正なら正しい、
/// という約束です（`Regularizer` と同じ判定）。
///
/// **割った断片を、この約束に合わせるために要ります。** 断片の輪を決め打ちの
/// 順で組む分割器は、元の面がどちら巻きでも同じ順で返すので、裏向きの面
/// ——輪の穴の壁がそれです——を割ると断片が裏返ります。縫合では「同じ向きに
/// 2度使われた稜」として出ます（4-46 と同じ症状）。
fn face_loop_matches_orientation(face: &Face, tol: &Tolerance) -> bool {
    let Ok(pcurves) = face.pcurves(tol) else {
        // p-curve が出せない面はここでは判定しない。後段の検証に任せる。
        return true;
    };
    let mut area = 0.0;
    let mut previous: Option<Point2> = None;
    let mut first: Option<Point2> = None;
    for segment in &pcurves.outer_loop.segments {
        let (t0, t1) = segment.curve.param_range();
        const SAMPLES: usize = 8;
        for step in 0..=SAMPLES {
            let point = segment
                .curve
                .evaluate(t0 + (t1 - t0) * step as f64 / SAMPLES as f64);
            if first.is_none() {
                first = Some(point);
            }
            if let Some(last) = previous {
                area += last.x * point.y - point.x * last.y;
            }
            previous = Some(point);
        }
    }
    if let (Some(last), Some(start)) = (previous, first) {
        area += last.x * start.y - start.x * last.y;
    }
    let area = area * 0.5;
    let oriented = if face.orientation.is_forward() {
        area
    } else {
        -area
    };
    oriented > tol.parametric
}

/// 巻き方が約束から外れている断片を巻き直す。
fn hold_piece_like_its_face(piece: Face, tol: &Tolerance) -> Face {
    if face_loop_matches_orientation(&piece, tol) {
        return piece;
    }
    let rewound = Face::new(
        piece.geometry.clone(),
        reversed_wire(&piece.outer_wire),
        piece.inner_wires.iter().map(reversed_wire).collect(),
        piece.orientation,
        piece.tolerance,
    );
    // 巻き直して約束に合うようになったときだけ採ります。合わないなら、
    // 原因は巻き方ではありません。
    if face_loop_matches_orientation(&rewound, tol) {
        rewound
    } else {
        piece
    }
}

/// 相手のいない交線の端から、隣の面の組を辿り直す。
///
/// # なぜ要るか
///
/// 面の組は1つずつ独立に辿っています。種は 12x12 の格子から選ぶので、
/// **その升に入る弧が短いと、どの種も弧に乗りません**。交線は隣の組へ
/// 続いているのに、そこだけ抜けます。
///
/// 実測（全周円錐を 27 度傾けたドリルで抜く、`march_stop_probe`）: 辿りは
/// 8組とも正しくパッチの縁まで届いているのに、宙に浮いた端点が4つ残ります。
/// その端点を測ると、**隣のパッチの継ぎ目にちょうど乗っています**。
///
/// ```text
/// loose end (1.6256 -3.9533 11.4511) sits on:
///     A0  distance 2.44e-5  (u 0.1854, v 0.4274)
///     B2  distance 1.69e-5  (u 1.0000, v 0.5454)  <- パッチの縁
///     B3  distance 2.45e-6  (u 0.0000, v 0.5454)
/// ```
///
/// `A0 x B3` は B3 の縁で終わり、続きは `A0 x B2` にあります。その組は
/// 「種はあったが一度も辿れなかった」と出ていました。
///
/// # やること
///
/// **端点は交線の上の点です。** そこを種にすれば、ニュートンは1歩で乗ります。
/// 相手のいない端点を拾い、その点を含む「まだ交線の無い」面の組を、その点から
/// 辿り直します。
///
/// 端点が2本の交線で共有されていれば、そこは既に繋がっているので触りません。
fn trace_from_loose_ends(
    faces_a: &[Face],
    faces_b: &[Face],
    bboxes_a: &[Option<BoundingBox3>],
    bboxes_b: &[Option<BoundingBox3>],
    candidates: &mut Vec<FaceIntersectionCandidate>,
    tol: &Tolerance,
) {
    // 端点をすべて集め、共有されていないものだけ残す。
    let mut ends: Vec<(usize, usize, Point3)> = Vec::new();
    for candidate in candidates.iter() {
        for point in candidate_end_points(&candidate.kind) {
            ends.push((candidate.face_a_index, candidate.face_b_index, point));
        }
    }
    if ends.is_empty() {
        return;
    }
    let join = tol.linear * 1000.0;
    let loose: Vec<(usize, usize, Point3)> = ends
        .iter()
        .filter(|(_, _, point)| {
            ends.iter()
                .filter(|(_, _, other)| (*other - *point).norm() <= join)
                .count()
                < 2
        })
        .cloned()
        .collect();

    // **既に「交線を持っている」組だけを既済とします。**
    // 交わりを求めて `Unsupported` になった組も候補の列には入っているので、
    // 素通しで数えると「もう見た」と判定してしまい、辿り直しが一度も
    // 走りません（実測でそうなっていました）。
    let mut covered: std::collections::BTreeSet<(usize, usize)> = candidates
        .iter()
        .filter(|candidate| !candidate_end_points(&candidate.kind).is_empty())
        .map(|candidate| (candidate.face_a_index, candidate.face_b_index))
        .collect();

    let mut added = Vec::new();
    for (from_a, from_b, point) in loose {
        for (index_a, face_a) in faces_a.iter().enumerate() {
            for (index_b, face_b) in faces_b.iter().enumerate() {
                if covered.contains(&(index_a, index_b)) {
                    continue;
                }
                // 元の組そのものは飛ばす。続きは**隣**にあります。
                if index_a == from_a && index_b == from_b {
                    continue;
                }
                if !face_bboxes_intersect(
                    bboxes_a[index_a].as_ref(),
                    bboxes_b[index_b].as_ref(),
                    tol,
                ) {
                    continue;
                }
                let (Some(patch_a), Some(patch_b)) = (face_patch(face_a), face_patch(face_b))
                else {
                    continue;
                };
                // **その点が両方のパッチに乗っていること。** 乗っていない組は
                // 続きではありません。
                let Some(seed) = seed_on_patch(&patch_a, point, tol) else {
                    continue;
                };
                if seed_on_patch(&patch_b, point, tol).is_none() {
                    continue;
                }

                let extent = surface_patch_extent(&patch_a).max(surface_patch_extent(&patch_b));
                let first_step = (extent * 0.1).max(tol.linear * 100.0);
                let Some(edge) = march_one_branch(&patch_a, &patch_b, seed, first_step, tol) else {
                    continue;
                };
                covered.insert((index_a, index_b));
                added.push(FaceIntersectionCandidate {
                    face_a_index: index_a,
                    face_b_index: index_b,
                    kind: FaceIntersectionKind::Curve { edge },
                    analytic: false,
                });
            }
        }
    }
    candidates.extend(added);
}

/// 面の交わりが持っている端点。
fn candidate_end_points(kind: &FaceIntersectionKind) -> Vec<Point3> {
    let ends_of = |edge: &Edge| {
        let (t0, t1) = edge.curve.param_range();
        vec![edge.curve.evaluate(t0), edge.curve.evaluate(t1)]
    };
    match kind {
        FaceIntersectionKind::Curve { edge } => ends_of(edge),
        FaceIntersectionKind::Curves { edges } => edges.iter().flat_map(ends_of).collect(),
        FaceIntersectionKind::Line {
            segment_start,
            segment_end,
            ..
        } => vec![*segment_start, *segment_end],
        _ => Vec::new(),
    }
}

/// 接しているだけで、**答えの縁になっていない**交線を落とす。
///
/// # なぜ
///
/// > HANDOVER 3-1: **接触は、それ自体では位相を作らない**
///
/// 実測（4-183、箱の上面とトーラスの上端が接する和）: 接する円
/// （半径 12、`z = 20`）を稜として面を割ったため、**1本の稜に4枚の面片**
/// （箱の上面 2枚＋トーラス 2枚）が付き、非多様体になっていました。32本
/// すべてがそれでした。トーラスは箱の上面に内側から触れているだけで、
/// 和の境界はそこで箱の上面がそのまま続きます。**割ってはいけない線**です。
///
/// # 測り方
///
/// **推測しません。2つとも測ってから落とします。**
///
/// 1. **本当に接しているか。** 交線の上の3点で、両方の面の法線が平行か
///    （外積が `1e-6` 未満）。横断的に交わる線はここで残ります
/// 2. **本当に縁でないか。** 交線の両側へ、共通の接平面の中で少しずらした
///    点を取り、**両側で内外が同じ**なら、その線は面を割っていません
///
/// 片方でも外れたら落としません。
fn drop_non_bounding_contact_curves(
    solid_a: &Solid,
    solid_b: &Solid,
    faces_a: &[Face],
    faces_b: &[Face],
    candidates: &mut Vec<FaceIntersectionCandidate>,
    tol: &Tolerance,
) -> Vec<Edge> {
    let mut dropped: Vec<Edge> = Vec::new();
    let explain = std::env::var_os("ZENITH_CONTACT_CURVE_WHY").is_some();
    candidates.retain(|candidate| {
        let edges = candidate_edges_including_lines(&candidate.kind, tol);
        if edges.is_empty() {
            return true;
        }
        let (Some(face_a), Some(face_b)) = (
            faces_a.get(candidate.face_a_index),
            faces_b.get(candidate.face_b_index),
        ) else {
            return true;
        };
        let (Some(patch_a), Some(patch_b)) = (face_patch(face_a), face_patch(face_b)) else {
            return true;
        };
        for edge in &edges {
            if !contact_curve_is_not_bounding(
                solid_a, solid_b, &patch_a, &patch_b, edge, tol, explain,
            ) {
                return true;
            }
        }
        if explain {
            eprintln!(
                "CONTACTCURVE 落とす 面A {} 面B {}",
                candidate.face_a_index, candidate.face_b_index
            );
        }
        dropped.extend(edges);
        false
    });
    dropped
}

/// 1本の弧について、[`drop_non_bounding_contact_curves`] の2つを測る。
fn contact_curve_is_not_bounding(
    solid_a: &Solid,
    solid_b: &Solid,
    patch_a: &NurbsSurface3,
    patch_b: &NurbsSurface3,
    edge: &Edge,
    tol: &Tolerance,
    explain: bool,
) -> bool {
    let (t_min, t_max) = edge.curve.param_range();
    if !(t_max > t_min) {
        return false;
    }
    let span = (edge.curve.evaluate(t_max) - edge.curve.evaluate(t_min)).norm();
    if span <= tol.linear * 100.0 {
        return false;
    }
    let radius = span * 1e-2;
    if radius <= tol.linear * 100.0 {
        return false;
    }

    for fraction in [0.25_f64, 0.5, 0.75] {
        let t = t_min + (t_max - t_min) * fraction;
        let point = edge.curve.evaluate(t);
        let Some(tangent) = edge.curve.evaluate_derivatives(t, 1)[1].try_normalize_safe(1e-12)
        else {
            return false;
        };
        let (Some(normal_a), Some(normal_b)) = (
            surface_normal_at(patch_a, point),
            surface_normal_at(patch_b, point),
        ) else {
            return false;
        };
        // 1. 本当に接しているか。横断的に交わる線はここで残ります。
        let sine = normal_a.cross(&normal_b).norm();
        if sine > 1e-6 {
            return false;
        }

        // 2. **一方の材料が、他方に含まれているか。**
        //
        // 面の上をずらして測るやり方は駄目でした（4-184）——接している
        // 向きへ射影が潰れて、**ずらした点が元の線へ戻ります**。同じ縮退
        // です。
        //
        // 代わりに、`contact` と同じ**交線に垂直な輪**で測ります。輪は
        // どちらの面からも離れているので、内外がちゃんと決まります。
        // 輪の上で B の材料が A に含まれる（か、その逆）なら、そこで
        // B は A を切っていません——**答えの縁ではありません**。
        let (u, v) = frame_across(tangent);
        let mut a_only = false;
        let mut b_only = false;
        let mut decided = 0usize;
        for step in 0..64 {
            let angle = std::f64::consts::TAU * step as f64 / 64.0;
            let probe = point + (u * angle.cos() + v * angle.sin()) * radius;
            let (Some(in_a), Some(in_b)) = (
                crate::boolean_validation::exact_inside(probe, solid_a, tol),
                crate::boolean_validation::exact_inside(probe, solid_b, tol),
            ) else {
                continue;
            };
            decided += 1;
            if in_a && !in_b {
                a_only = true;
            }
            if in_b && !in_a {
                b_only = true;
            }
        }
        if decided < 32 {
            // 半分も決まらないなら、測れていません。触りません。
            return false;
        }
        if a_only && b_only {
            if explain {
                eprintln!("CONTACTCURVE 縁です（正弦 {sine:.3e}、位置 {fraction}）");
            }
            return false;
        }
    }
    true
}

/// 向きに垂直な、正規直交の2本。
fn frame_across(direction: Vec3) -> (Vec3, Vec3) {
    let helper = if direction.x.abs() < 0.9 {
        Vec3::new(1.0, 0.0, 0.0)
    } else {
        Vec3::new(0.0, 1.0, 0.0)
    };
    let u = direction
        .cross(&helper)
        .try_normalize_safe(1e-12)
        .unwrap_or(Vec3::new(0.0, 1.0, 0.0));
    let v = direction.cross(&u);
    (u, v)
}

/// 点が乗っているところでの、面の法線。
fn surface_normal_at(patch: &NurbsSurface3, point: Point3) -> Option<Vec3> {
    zenith_geom::work_counter::count_normal_projection();
    let projection = ExtremumEngine::point_to_surface(point, patch, 64, 1e-13).ok()?;
    let (_, du, dv) = patch.evaluate_derivatives_1st(projection.u, projection.v);
    du.cross(&dv).try_normalize_safe(1e-12)
}

/// 候補が持つ交線を、**まっすぐな線も含めて**取り出す。
///
/// `candidate_edges` は `Curve` / `Curves` しか見ません。`Line` は `Edge` を
/// 持たないので、そのままでは**接触の判定を素通りします**。
///
/// 実測（4-192）: 内側から接する円柱2本の和が、非多様体の稜 16本で
/// 断られていました。接する母線は 4-190 で `Line` として出るように
/// なったのに、接触の判定（4-184）はそれを見ていなかったのです。
/// **答えは A そのもの**（B は中に入っている）なので、断るのは誤りです。
fn candidate_edges_including_lines(kind: &FaceIntersectionKind, tol: &Tolerance) -> Vec<Edge> {
    match kind {
        FaceIntersectionKind::Line {
            segment_start,
            segment_end,
            ..
        } => {
            let Ok(curve) = NurbsCurve3::bspline_from_points(1, vec![*segment_start, *segment_end])
            else {
                return Vec::new();
            };
            vec![Edge::new(
                curve,
                Vertex::new(*segment_start, tol.linear),
                Vertex::new(*segment_end, tol.linear),
                tol.linear,
            )]
        }
        other => candidate_edges(other).into_iter().cloned().collect(),
    }
}

/// 候補が持つ弧を借りる。
fn candidate_edges(kind: &FaceIntersectionKind) -> Vec<&Edge> {
    match kind {
        FaceIntersectionKind::Curve { edge } => vec![edge],
        FaceIntersectionKind::Curves { edges } => edges.iter().collect(),
        _ => Vec::new(),
    }
}

/// 曲線が平面を横切る媒介変数を、ニュートンで解く。
///
/// **1未知数1式です。** 横断的なら二重根にならず、倍精度いっぱいまで
/// 詰まります（4-180）。
fn solve_curve_on_plane(
    curve: &NurbsCurve3,
    seed: f64,
    origin: Point3,
    normal: Vec3,
) -> Option<f64> {
    let normal = normal.try_normalize_safe(1e-12)?;
    let (t_min, t_max) = curve.param_range();
    let mut t = seed.clamp(t_min, t_max);
    for _ in 0..40 {
        let value = (curve.evaluate(t) - origin).dot(&normal);
        if value.abs() <= 1e-15 * (t_max - t_min).abs().max(1.0) {
            return Some(t);
        }
        let slope = curve.evaluate_derivatives(t, 1)[1].dot(&normal);
        if slope.abs() <= 1e-14 {
            // 曲線が平面に接している。ここでは決まりません。
            return None;
        }
        let next = (t - value / slope).clamp(t_min, t_max);
        if (next - t).abs() <= f64::EPSILON * t.abs().max(1.0) {
            t = next;
            break;
        }
        t = next;
    }
    let value = (curve.evaluate(t) - origin).dot(&normal);
    value.abs().le(&1e-9).then_some(t)
}

/// 弧の端を、指定の点へ動かす。
///
/// **制御点も動かします。** 頂点だけ動かすと、曲線の端と食い違います。
/// クランプした NURBS は最初と最後の制御点をそのまま通るので、そこを
/// 置き換えれば端は厳密にその点になります。**ずれるのは最後の1区間だけ**
/// で、その幅は動かした距離（実測 2e-4）を超えません——**元からその
/// くらいずれていた**ところです（4-179）。
fn move_candidate_edge_end(
    candidate: &mut FaceIntersectionCandidate,
    edge_slot: usize,
    is_start: bool,
    target: Point3,
    tol: &Tolerance,
) {
    let edge = match &mut candidate.kind {
        FaceIntersectionKind::Curve { edge } if edge_slot == 0 => edge,
        FaceIntersectionKind::Curves { edges } => match edges.get_mut(edge_slot) {
            Some(edge) => edge,
            None => return,
        },
        _ => return,
    };

    let mut control_points = edge.curve.control_points.clone();
    if control_points.len() < 2 {
        return;
    }
    let corner = if is_start {
        0
    } else {
        control_points.len() - 1
    };
    control_points[corner] = ControlPoint3::new(target, control_points[corner].weight);
    let Ok(curve) = NurbsCurve3::new(edge.curve.degree, control_points, edge.curve.knots.clone())
    else {
        return;
    };

    let (t_min, t_max) = curve.param_range();
    let start = if is_start {
        target
    } else {
        curve.evaluate(t_min)
    };
    let end = if is_start {
        curve.evaluate(t_max)
    } else {
        target
    };
    *edge = Edge::new(
        curve,
        Vertex::new(start, tol.linear),
        Vertex::new(end, tol.linear),
        tol.linear,
    );
}

/// 面をマーチングに渡せるパッチにする。平面は境界が占めるぶんだけの
/// 1次×1次パッチに直します（パラメータ範囲が無限なので、そのままでは
/// 渡せません）。
fn face_patch(face: &Face) -> Option<NurbsSurface3> {
    match &face.geometry {
        FaceGeometry::Nurbs(surface) => Some(surface.clone()),
        FaceGeometry::Plane(plane) => planar_face_as_patch(face, plane),
        _ => None,
    }
}

/// 点がパッチの上に乗っていれば、その `(u, v)` を返す。
fn seed_on_patch(patch: &NurbsSurface3, point: Point3, tol: &Tolerance) -> Option<(f64, f64)> {
    zenith_geom::work_counter::count_seed_on_patch_projection();
    let projection = { ExtremumEngine::point_to_surface(point, patch, 64, 1e-13).ok()? };
    // 辿りの端点は交線の上に丸め誤差ぶんだけ乗っています。実測で 2.4e-5。
    // 公差そのものでは締めすぎるので、辿りの精度に合わせます。
    let limit = (tol.linear * 1000.0).max(1e-4);
    (projection.distance <= limit).then_some((projection.u, projection.v))
}

/// 種から1本だけ辿って、要求精度の曲線にする。
///
/// `fit_all_branches` と同じ刻みの決め方（測った収束次数から予測）を使います。
/// 種が既に交線の上にあるので、枝を探す必要はありません。
fn march_one_branch(
    patch_a: &NurbsSurface3,
    patch_b: &NurbsSurface3,
    seed: (f64, f64),
    first_step: f64,
    tol: &Tolerance,
) -> Option<Edge> {
    let deviation_limit = tol.linear;
    let mut step = first_step;
    let mut previous: Option<(f64, f64)> = None;
    for _ in 0..8 {
        let Some(marched) = zenith_geom::IntersectionMarcher::march(
            patch_a, patch_b, seed.0, seed.1, step, 2048, tol,
        ) else {
            step *= 0.5;
            continue;
        };
        if marched.points.len() < 4 {
            step *= 0.5;
            continue;
        }
        let Some((curve, deviation)) =
            zenith_geom::IntersectionMarcher::fit_curve(patch_a, patch_b, &marched, 3)
        else {
            step *= 0.5;
            continue;
        };
        if deviation <= deviation_limit {
            let (t0, t1) = curve.param_range();
            let start = curve.evaluate(t0);
            let end = curve.evaluate(t1);
            if (end - start).norm() <= tol.linear {
                return None;
            }
            return Some(Edge::new(
                curve,
                Vertex::new(start, tol.linear),
                Vertex::new(end, tol.linear),
                tol.linear,
            ));
        }
        if marched.points.len() >= 2048 {
            return None;
        }
        let next = match previous {
            None => step * 0.5,
            Some((before_step, before_deviation)) => {
                let step_ratio = before_step / step;
                let deviation_ratio = before_deviation / deviation;
                if !(step_ratio > 1.0 && deviation_ratio > 1.0) {
                    step * 0.5
                } else {
                    let order = (deviation_ratio.ln() / step_ratio.ln()).clamp(0.5, 6.0);
                    let shrink = (deviation / deviation_limit).powf(1.0 / order) * 1.5;
                    step / shrink.clamp(2.0, 16.0)
                }
            }
        };
        previous = Some((step, deviation));
        step = next;
    }
    None
}

/// 点がトリムループの内側かを、**交点の個数**で答える。
///
/// # なぜ要るか
///
/// 交線を面のトリムで切るとき、内外の境目は二分で詰めます。**その内外判定が
/// 折れ線に対してだと、詰め先は折れ線であって面の境界ではありません。**
/// 実測: 円錐の上面（半径 4）の境界を1区間 48 点で刻むと、切り口の端が
/// 半径 3.99954 に落ちます——真の円の内側 4.6e-4。同じ点を別の面のクリップが
/// 出すと 3.8251 で、**どちらの折れ線で切ったかで 5e-4 食い違います**。
/// 分割はそれを「the splitting curve ends 5.073e-4 away from the boundary」
/// として断ります（許容 8e-5）。
///
/// # やり方
///
/// 点からループの外まで線分を引き、**交わった回数の偶奇**で決めます。線分と
/// p-curve の交点は閉じた式（1スパンの1次・2次）か曲線上の二分で厳密に出る
/// ので（4-56）、境目は丸め誤差まで詰まります。
///
/// **区間の中点を折れ線で見る方法は使えません。** 点が境界のすぐ近くにある
/// とき——二分の終盤はまさにそこです——中点も境界のすぐ近くにあり、折れ線の
/// 判定が外れます。実測でそれをやって、ずれが 5.073e-4 から 1.473e-3 へ
/// 悪化しました（4-63）。
///
/// 交点が重なる向き（境界の頂点を貫く、接する）では数が狂うので、重なりが
/// あれば別の向きで測り直します。3方向とも駄目なら `None` を返し、呼び手は
/// 従来どおり折れ線で見ます。
fn point_inside_pcurve_loop(
    point: Point2,
    loop_data: &FacePcurveLoop,
    _tol: &Tolerance,
) -> Option<bool> {
    // ループの広がり。線分はここから確実に外へ出る長さにします。
    let mut low = Point2::new(f64::INFINITY, f64::INFINITY);
    let mut high = Point2::new(f64::NEG_INFINITY, f64::NEG_INFINITY);
    for segment in &loop_data.segments {
        for control_point in &segment.curve.control_points {
            let uv = control_point.point;
            low.x = low.x.min(uv.x);
            low.y = low.y.min(uv.y);
            high.x = high.x.max(uv.x);
            high.y = high.y.max(uv.y);
        }
    }
    if !(low.x.is_finite() && high.x.is_finite()) {
        return None;
    }
    let span = (high.x - low.x).abs().max((high.y - low.y).abs()).max(1.0);
    let reach = span * 4.0;
    let separation = 1e-9;

    for raw_direction in [
        Vec2::new(1.0, 0.37),
        Vec2::new(-0.41, 1.0),
        Vec2::new(0.73, -1.0),
    ] {
        let Some(unit) = raw_direction.try_normalize(1e-12) else {
            continue;
        };
        let direction = unit * reach;

        let mut crossings: Vec<f64> = Vec::new();
        let mut usable = true;
        for segment in &loop_data.segments {
            let found = match pcurve_segment_crossings(&segment.curve, point, direction) {
                Some(values) => values,
                None => pcurve_crossings_by_bisection(&segment.curve, point, direction, 32),
            };
            for t in found {
                // 線分の手前と奥だけを見ます。始点そのものに乗った交点は、
                // 点が境界の上にあるということなので、内側として扱います。
                if t < -separation || t > 1.0 + separation {
                    continue;
                }
                if t.abs() <= separation {
                    return Some(true);
                }
                crossings.push(t);
            }
        }

        crossings.sort_by(f64::total_cmp);
        for pair in crossings.windows(2) {
            if (pair[1] - pair[0]).abs() <= separation {
                // 頂点を貫いた、あるいは接した。この向きでは数えられません。
                usable = false;
                break;
            }
        }
        if !usable {
            continue;
        }
        return Some(crossings.len() % 2 == 1);
    }
    None
}
