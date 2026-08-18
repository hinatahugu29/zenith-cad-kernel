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

impl Shape {
    /// 複数 Solid を compound shape としてまとめる。
    pub fn compound_solids(solids: Vec<Solid>) -> Self {
        match solids.len() {
            0 => Shape::Compound(Vec::new()),
            1 => Shape::Solid(solids.into_iter().next().unwrap()),
            _ => Shape::Compound(solids.into_iter().map(Shape::Solid).collect()),
        }
    }

    /// Shape ツリー内の Solid 参照を深さ優先で収集する。
    pub fn solids(&self) -> Vec<&Solid> {
        let mut solids = Vec::new();
        self.collect_solids(&mut solids);
        solids
    }

    /// Shape ツリー内の Solid 数を返す。
    pub fn solid_count(&self) -> usize {
        self.solids().len()
    }

    /// Shape ツリーから Solid を深さ優先で取り出す。
    pub fn into_solids(self) -> Vec<Solid> {
        match self {
            Shape::Solid(solid) => vec![solid],
            Shape::Compound(shapes) => shapes
                .into_iter()
                .flat_map(Shape::into_solids)
                .collect::<Vec<_>>(),
            _ => Vec::new(),
        }
    }

    fn collect_solids<'a>(&'a self, solids: &mut Vec<&'a Solid>) {
        match self {
            Shape::Solid(solid) => solids.push(solid),
            Shape::Compound(shapes) => {
                for shape in shapes {
                    shape.collect_solids(solids);
                }
            }
            _ => {}
        }
    }
}
