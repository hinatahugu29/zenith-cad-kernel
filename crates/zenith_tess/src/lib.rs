//! Zenith Tess: B-Repおよび曲面の高品質・適応的テッセレーションライブラリ

pub mod mesh;
pub mod surface_tess;

pub use mesh::TriangleMesh;
pub use surface_tess::{
    face_uv_triangulation, tessellate_face, tessellate_shell, tessellate_solid, tessellate_surface,
    TessellationParams, UvTriangulation,
};
