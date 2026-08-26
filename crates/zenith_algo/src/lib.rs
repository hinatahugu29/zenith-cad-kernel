//! Zenith Algo: CADモデリングアルゴリズムライブラリ

pub mod boolean;
pub mod boolean_validation;
pub mod bolt;
pub mod brep_intersection;
pub mod brep_transform;
pub mod cap;
pub mod chamfer;
pub mod contact;
mod circular_fillet;
mod cylinder_boolean;
pub mod direct_edit;
pub mod distance;
pub mod draft;
pub mod edge_blend;
pub mod extrude;
pub mod face_split;
pub mod feature_tree;
pub mod fillet;
pub mod flange;
pub mod gear;
pub mod helix;
pub mod hole;

pub mod interference;
pub mod loft;
pub mod mass_properties;
pub mod merge_faces;
pub mod mirror;
mod orthogonal_boolean;

pub mod patch_builder;
pub mod pattern;
pub mod polyline;
pub mod primitive;
pub mod profile;
pub mod regularize;
pub mod rib;
pub mod shaft;


pub mod revolve;
pub mod sew;
pub mod shell;
pub mod shelling;
pub mod sketch_solver;
pub mod slice;
pub mod sweep;
pub mod thicken;

use zenith_math::Tolerance;
use zenith_topo::{Shell, Solid};

pub use boolean::{
    BooleanEngine, BooleanOpType, ExactBooleanPreparationReport, ExactBooleanResult,
};
pub use boolean_validation::{
    exact_inside, BooleanResultReport, BooleanResultVerifier, BooleanVerificationParams,
};
pub use bolt::BoltBuilder;
pub use brep_intersection::{
    BooleanFaceAssembly, BooleanFaceSelection, BooleanOperand, BooleanShellAssembly,
    BrepIntersectionBuilder, ClassifiedFacePiece, ClassifiedPlanarFaceSplitCandidate,
    FaceIntersectionCandidate, FaceIntersectionKind, FaceRegionLocation, IntersectionEdgeCandidate,
    IntersectionEdgeLoop, IntersectionEdgeLoopExtraction, PlanarCapGeneration,
    PlanarFaceBatchSplit, PlanarFaceMultiSplitResult, PlanarFaceSplitCandidate,
    PlanarOperandBatchSplits, SelectedBooleanFacePiece, SelectedFaceStitchReport,
};
pub use brep_transform::BrepTransform;
pub use distance::{nearest_boundary_projection, BoundaryProjection, DistanceEngine, DistanceResult};
pub use face_split::{FaceSplitReport, FaceSplitter, MultiSplitReport};
pub use regularize::{RegularizeReport, Regularizer, StepInterop};
pub use cap::CapBuilder;
pub use chamfer::ChamferBuilder;
pub use contact::{find_result_pinch, pinch_along_edge, ContactPinch};
pub use direct_edit::{DirectModeling, EdgeInspection, EdgeKind, FaceInspection};
pub use draft::DraftBuilder;
pub use edge_blend::{BlendKind, BlendableEdge, EdgeBlendReport, EdgeBlender};
pub use extrude::ExtrudeBuilder;
pub use feature_tree::{
    edge_signature, match_edge, BooleanKind, FeatureNode, FeatureOp, FeatureTree,
};
pub use fillet::FilletBuilder;
pub use flange::FlangeBuilder;
pub use gear::{GearBuilder, RootFilletGeneration};
pub use helix::HelixBuilder;
pub use hole::HoleBuilder;
pub use interference::{ClashStatus, InterferenceChecker, InterferenceReport};

pub use loft::LoftBuilder;
pub use mass_properties::{MassCalculator, MassProperties};
pub use merge_faces::{FaceMerger, MergeReport};
pub use mirror::MirrorBuilder;
pub use patch_builder::CurvePatchBuilder;
pub use pattern::PatternBuilder;
pub use polyline::{PathSegment, PolylineBuilder};
pub use primitive::PrimitiveBuilder;
pub use profile::ProfileBuilder;
pub use rib::RibBuilder;
pub use shaft::ShaftBuilder;


pub use revolve::RevolveBuilder;
pub use sew::{SewReport, Sewer};
pub use shell::ShellBuilder;
pub use shelling::ShellingBuilder;
pub use sketch_solver::{
    CircleId, Constraint, LineId, PointId, SketchCircle, SketchConstraintStatus, SketchLine,
    SketchPoint, SketchSolver,
};
pub use slice::{SectionSliceResult, SectionSlicer};
pub use sweep::SweepBuilder;
pub use thicken::ThickenBuilder;


/// ビルダーの共通出口。
///
/// ここで**平面を平面として持ち直す**。制御点が公差内で同一平面に乗る有理
/// NURBS 面は、像が制御点の凸包に入る以上その平面に乗っているので、これは
/// 近似ではなく持ち方を直しているだけ。
///
/// 直しておかないと、平面しか受け付けない演算（面の併合、稜のフィレット・
/// 面取り）がその立体に一切掛からない。実測で `HoleBuilder::make_drilled_box`
/// は 16 面すべてが NURBS でフィレットの候補が **0 本**、`GearBuilder` は
/// 110 面中 36 面が平面なのに NURBS だった。`planar_face_audit` が全ビルダー
/// について常時数えている。
pub(crate) fn validated_solid(shell: Shell) -> Result<Solid, String> {
    let tol = Tolerance::default();
    let (shell, _converted) = merge_faces::FaceMerger::planarize_shell(&shell, &tol);
    Solid::try_simple(shell, &tol).map_err(|err| err.to_string())
}
