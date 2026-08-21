//! Zenith Topo: 境界表現（B-Rep）トポロジーデータ構造ライブラリ

pub mod assembly;
pub mod edge;
pub mod face;
pub mod persistent_id;
pub mod shader_payload;
pub mod shape;
pub mod shell;
pub mod solid;
pub mod vertex;
pub mod wire;

pub use assembly::{Assembly, ComponentInstance, Transform3};
pub use edge::{Edge, Orientation, OrientedEdge};
pub use face::{
    Face, FaceBoundaryValidationReport, FaceGeometry, FacePcurveLoop, FacePcurveSegment,
    FacePcurves, PcurveValidationReport,
};
pub use persistent_id::{
    EdgeSignature, GeometricMatcher, GeometricSignature, PersistentId, SemanticTag,
};
pub use shader_payload::{ShaderBRepPayload, ShaderEdgeData, ShaderFaceData, ShaderSurfaceType};
pub use shape::Shape;
pub use shell::{Shell, ShellValidationReport};
pub use solid::{Solid, SolidValidationError};
pub use vertex::Vertex;
pub use wire::Wire;
