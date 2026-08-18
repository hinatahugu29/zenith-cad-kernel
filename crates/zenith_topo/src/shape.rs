use crate::edge::Edge;
use crate::face::Face;
use crate::shell::Shell;
use crate::solid::Solid;
use crate::vertex::Vertex;
use crate::wire::Wire;
use serde::{Deserialize, Serialize};

/// 汎用トポロジー形状（OCCTの TopoDS_Shape に相当）
#[allow(clippy::large_enum_variant)]
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub enum Shape {
    Vertex(Vertex),
    Edge(Edge),
    Wire(Wire),
    Face(Face),
    Shell(Shell),
    Solid(Solid),
    Compound(Vec<Shape>),
}
