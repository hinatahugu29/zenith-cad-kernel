use crate::{FaceGeometry, Solid};
use serde::{Deserialize, Serialize};

/// シェーダー用幾何曲面タイプ
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
#[repr(u32)]
pub enum ShaderSurfaceType {
    Plane = 0,
    Cylinder = 1,
    Cone = 2,
    Sphere = 3,
    Torus = 4,
    Nurbs = 5,
}

/// シェーダー用SDFプリミティブ形状タイプ
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
#[repr(u32)]
pub enum ShaderPrimitiveType {
    Box = 0,
    Cylinder = 1,
    Sphere = 2,
    Cone = 3,
    Torus = 4,
    CustomBRep = 5,
}

/// 単一プリミティブのSDFレイマーチング用パラメータ
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct ShaderPrimitiveData {
    pub prim_type: ShaderPrimitiveType,
    pub center: [f32; 3],
    pub dimensions: [f32; 4], // [dx/r/R, dy/h/r_tube, dz/r2, param4]
    pub rotation: [f32; 4],   // Quaternion [x, y, z, w]
}

/// 単一Faceのシェーダー用パラメータ
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct ShaderFaceData {
    pub surface_type: ShaderSurfaceType,
    /// 幾何パラメータ（位置・軸・半径・寸法等）
    pub params: [f32; 16],
    /// UVトリムポリライン座標列: [ [u0, v0, u1, v1, ...], ... ]
    pub trim_loops: Vec<Vec<f32>>,
}

/// 単一Edgeのシェーダー用ワイヤフレームデータ
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct ShaderEdgeData {
    pub points: Vec<[f32; 3]>,
}

/// B-Rep ソリッド全体のシェーダー用ペイロード
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct ShaderBRepPayload {
    pub primitive: Option<ShaderPrimitiveData>,
    pub faces: Vec<ShaderFaceData>,
    pub edges: Vec<ShaderEdgeData>,
    pub bbox_min: [f32; 3],
    pub bbox_max: [f32; 3],
}

impl ShaderBRepPayload {
    /// プリミティブ定義からSDFシェーダーペイロードを即時生成
    pub fn new_box(center: [f32; 3], half_extents: [f32; 3]) -> Self {
        let min_pt = [
            center[0] - half_extents[0],
            center[1] - half_extents[1],
            center[2] - half_extents[2],
        ];
        let max_pt = [
            center[0] + half_extents[0],
            center[1] + half_extents[1],
            center[2] + half_extents[2],
        ];
        Self {
            primitive: Some(ShaderPrimitiveData {
                prim_type: ShaderPrimitiveType::Box,
                center,
                dimensions: [half_extents[0], half_extents[1], half_extents[2], 0.0],
                rotation: [0.0, 0.0, 0.0, 1.0],
            }),
            faces: Vec::new(),
            edges: Vec::new(),
            bbox_min: min_pt,
            bbox_max: max_pt,
        }
    }

    pub fn new_cylinder(center: [f32; 3], radius: f32, half_height: f32) -> Self {
        let min_pt = [
            center[0] - radius,
            center[1] - radius,
            center[2] - half_height,
        ];
        let max_pt = [
            center[0] + radius,
            center[1] + radius,
            center[2] + half_height,
        ];
        Self {
            primitive: Some(ShaderPrimitiveData {
                prim_type: ShaderPrimitiveType::Cylinder,
                center,
                dimensions: [radius, half_height, 0.0, 0.0],
                rotation: [0.0, 0.0, 0.0, 1.0],
            }),
            faces: Vec::new(),
            edges: Vec::new(),
            bbox_min: min_pt,
            bbox_max: max_pt,
        }
    }

    pub fn new_sphere(center: [f32; 3], radius: f32) -> Self {
        let min_pt = [center[0] - radius, center[1] - radius, center[2] - radius];
        let max_pt = [center[0] + radius, center[1] + radius, center[2] + radius];
        Self {
            primitive: Some(ShaderPrimitiveData {
                prim_type: ShaderPrimitiveType::Sphere,
                center,
                dimensions: [radius, 0.0, 0.0, 0.0],
                rotation: [0.0, 0.0, 0.0, 1.0],
            }),
            faces: Vec::new(),
            edges: Vec::new(),
            bbox_min: min_pt,
            bbox_max: max_pt,
        }
    }

    pub fn new_cone(center: [f32; 3], r1: f32, r2: f32, half_height: f32) -> Self {
        let max_r = r1.max(r2);
        let min_pt = [
            center[0] - max_r,
            center[1] - max_r,
            center[2] - half_height,
        ];
        let max_pt = [
            center[0] + max_r,
            center[1] + max_r,
            center[2] + half_height,
        ];
        Self {
            primitive: Some(ShaderPrimitiveData {
                prim_type: ShaderPrimitiveType::Cone,
                center,
                dimensions: [r1, r2, half_height, 0.0],
                rotation: [0.0, 0.0, 0.0, 1.0],
            }),
            faces: Vec::new(),
            edges: Vec::new(),
            bbox_min: min_pt,
            bbox_max: max_pt,
        }
    }

    pub fn new_torus(center: [f32; 3], major_r: f32, minor_r: f32) -> Self {
        let outer_r = major_r + minor_r;
        let min_pt = [
            center[0] - outer_r,
            center[1] - outer_r,
            center[2] - minor_r,
        ];
        let max_pt = [
            center[0] + outer_r,
            center[1] + outer_r,
            center[2] + minor_r,
        ];
        Self {
            primitive: Some(ShaderPrimitiveData {
                prim_type: ShaderPrimitiveType::Torus,
                center,
                dimensions: [major_r, minor_r, 0.0, 0.0],
                rotation: [0.0, 0.0, 0.0, 1.0],
            }),
            faces: Vec::new(),
            edges: Vec::new(),
            bbox_min: min_pt,
            bbox_max: max_pt,
        }
    }

    /// Solid から解析的シェーダー用ペイロードを自動抽出
    pub fn from_solid(solid: &Solid) -> Self {
        let mut faces_data = Vec::with_capacity(solid.outer_shell.faces.len());
        let mut edges_data = Vec::new();

        let mut min_pt = [f32::INFINITY; 3];
        let mut max_pt = [f32::NEG_INFINITY; 3];

        for face in &solid.outer_shell.faces {
            let (surf_type, params) = Self::extract_surface_params(&face.geometry);

            // トリムループの抽出（エッジの頂点座標から）
            let mut trim_loops = Vec::new();
            let mut outer_loop_pts = Vec::new();
            for oe in &face.outer_wire.edges {
                let p = oe.edge.start_vertex.point;
                min_pt[0] = min_pt[0].min(p.x as f32);
                min_pt[1] = min_pt[1].min(p.y as f32);
                min_pt[2] = min_pt[2].min(p.z as f32);
                max_pt[0] = max_pt[0].max(p.x as f32);
                max_pt[1] = max_pt[1].max(p.y as f32);
                max_pt[2] = max_pt[2].max(p.z as f32);

                outer_loop_pts.push(p.x as f32);
                outer_loop_pts.push(p.y as f32);
            }
            trim_loops.push(outer_loop_pts);

            // ワイヤフレームエッジの収集
            for oe in &face.outer_wire.edges {
                let p0 = oe.edge.start_vertex.point;
                let p1 = oe.edge.end_vertex.point;
                edges_data.push(ShaderEdgeData {
                    points: vec![
                        [p0.x as f32, p0.y as f32, p0.z as f32],
                        [p1.x as f32, p1.y as f32, p1.z as f32],
                    ],
                });
            }

            faces_data.push(ShaderFaceData {
                surface_type: surf_type,
                params,
                trim_loops,
            });
        }

        Self {
            primitive: Some(ShaderPrimitiveData {
                prim_type: ShaderPrimitiveType::CustomBRep,
                center: [
                    (min_pt[0] + max_pt[0]) * 0.5,
                    (min_pt[1] + max_pt[1]) * 0.5,
                    (min_pt[2] + max_pt[2]) * 0.5,
                ],
                dimensions: [
                    (max_pt[0] - min_pt[0]) * 0.5,
                    (max_pt[1] - min_pt[1]) * 0.5,
                    (max_pt[2] - min_pt[2]) * 0.5,
                    0.0,
                ],
                rotation: [0.0, 0.0, 0.0, 1.0],
            }),
            faces: faces_data,
            edges: edges_data,
            bbox_min: min_pt,
            bbox_max: max_pt,
        }
    }

    fn extract_surface_params(geom: &FaceGeometry) -> (ShaderSurfaceType, [f32; 16]) {
        let mut p = [0.0f32; 16];
        match geom {
            FaceGeometry::Plane(plane) => {
                p[0] = plane.origin.x as f32;
                p[1] = plane.origin.y as f32;
                p[2] = plane.origin.z as f32;
                p[3] = 1.0; // w

                p[4] = plane.normal.x as f32;
                p[5] = plane.normal.y as f32;
                p[6] = plane.normal.z as f32;

                p[8] = plane.u_axis.x as f32;
                p[9] = plane.u_axis.y as f32;
                p[10] = plane.u_axis.z as f32;

                p[12] = plane.v_axis.x as f32;
                p[13] = plane.v_axis.y as f32;
                p[14] = plane.v_axis.z as f32;
                (ShaderSurfaceType::Plane, p)
            }
            FaceGeometry::Nurbs(nurbs) => {
                let num_u = nurbs.control_points.len();
                let num_v = nurbs.control_points[0].len();
                p[0] = nurbs.degree_u as f32;
                p[1] = nurbs.degree_v as f32;
                p[2] = num_u as f32;
                p[3] = num_v as f32;
                (ShaderSurfaceType::Nurbs, p)
            }
            _ => (ShaderSurfaceType::Plane, p),
        }
    }
}
