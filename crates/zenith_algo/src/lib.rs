//! Zenith Algo: CADモデリングアルゴリズムライブラリ

pub mod boolean;
pub mod brep_intersection;
pub mod cap;
pub mod chamfer;
pub mod direct_edit;
pub mod extrude;
pub mod feature_tree;
pub mod fillet;
pub mod hole;
pub mod loft;
pub mod mass_properties;
pub mod patch_builder;
pub mod primitive;
pub mod revolve;
pub mod shell;
pub mod sketch_solver;
pub mod sweep;
pub mod thicken;

use zenith_math::Tolerance;
use zenith_topo::{Shell, Solid};

pub use boolean::{BooleanEngine, BooleanOpType};
pub use brep_intersection::{
    BooleanOperand, BrepIntersectionBuilder, ClassifiedFacePiece,
    ClassifiedPlanarFaceSplitCandidate, FaceIntersectionCandidate, FaceIntersectionKind,
    FaceRegionLocation, IntersectionEdgeCandidate, PlanarFaceSplitCandidate,
    SelectedBooleanFacePiece,
};
pub use cap::CapBuilder;
pub use chamfer::ChamferBuilder;
pub use direct_edit::{DirectModeling, EdgeInspection, EdgeKind, FaceInspection};
pub use extrude::ExtrudeBuilder;
pub use feature_tree::{FeatureNode, FeatureOp, FeatureTree};
pub use fillet::FilletBuilder;
pub use hole::HoleBuilder;
pub use loft::LoftBuilder;
pub use mass_properties::{MassCalculator, MassProperties};
pub use patch_builder::CurvePatchBuilder;
pub use primitive::PrimitiveBuilder;
pub use revolve::RevolveBuilder;
pub use shell::ShellBuilder;
pub use sketch_solver::{
    CircleId, Constraint, LineId, PointId, SketchCircle, SketchLine, SketchPoint, SketchSolver,
};
pub use sweep::SweepBuilder;
pub use thicken::ThickenBuilder;

pub(crate) fn validated_solid(shell: Shell) -> Result<Solid, String> {
    Solid::try_simple(shell, &Tolerance::default()).map_err(|err| err.to_string())
}
