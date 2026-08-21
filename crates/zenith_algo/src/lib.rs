//! Zenith Algo: CADモデリングアルゴリズムライブラリ

pub mod boolean;
pub mod boolean_validation;
pub mod bolt;
pub mod brep_intersection;
pub mod brep_transform;
pub mod cap;
pub mod chamfer;
mod cylinder_boolean;
pub mod direct_edit;
pub mod distance;
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
pub mod mirror;
mod orthogonal_boolean;

pub mod patch_builder;
pub mod pattern;
pub mod polyline;
pub mod primitive;
pub mod regularize;
pub mod shaft;


pub mod revolve;
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
    BooleanResultReport, BooleanResultVerifier, BooleanVerificationParams,
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
pub use distance::{DistanceEngine, DistanceResult};
pub use face_split::{FaceSplitReport, FaceSplitter, MultiSplitReport};
pub use regularize::{RegularizeReport, Regularizer, StepInterop};
pub use cap::CapBuilder;
pub use chamfer::ChamferBuilder;
pub use direct_edit::{DirectModeling, EdgeInspection, EdgeKind, FaceInspection};
pub use extrude::ExtrudeBuilder;
pub use feature_tree::{FeatureNode, FeatureOp, FeatureTree};
pub use fillet::FilletBuilder;
pub use flange::FlangeBuilder;
pub use gear::GearBuilder;
pub use helix::HelixBuilder;
pub use hole::HoleBuilder;
pub use interference::{ClashStatus, InterferenceChecker, InterferenceReport};

pub use loft::LoftBuilder;
pub use mass_properties::{MassCalculator, MassProperties};
pub use mirror::MirrorBuilder;
pub use patch_builder::CurvePatchBuilder;
pub use pattern::PatternBuilder;
pub use polyline::{PathSegment, PolylineBuilder};
pub use primitive::PrimitiveBuilder;
pub use shaft::ShaftBuilder;


pub use revolve::RevolveBuilder;
pub use shell::ShellBuilder;
pub use shelling::ShellingBuilder;
pub use sketch_solver::{
    CircleId, Constraint, LineId, PointId, SketchCircle, SketchLine, SketchPoint, SketchSolver,
};
pub use slice::{SectionSliceResult, SectionSlicer};
pub use sweep::SweepBuilder;
pub use thicken::ThickenBuilder;


pub(crate) fn validated_solid(shell: Shell) -> Result<Solid, String> {
    Solid::try_simple(shell, &Tolerance::default()).map_err(|err| err.to_string())
}
