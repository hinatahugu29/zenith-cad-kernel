//! Zenith Tess: B-Repおよび曲面の高品質・適応的テッセレーションライブラリ

pub mod mesh;
pub mod stitched;
pub mod surface_tess;

pub use mesh::TriangleMesh;
pub use stitched::{face_triangle_counts, tessellate_solid_stitched};
pub use surface_tess::{
    face_parameter_area, face_uv_triangulation, face_uv_triangulation_for_point_picking, tessellate_face, tessellate_shell, tessellate_solid, tessellate_surface,
    TessellationParams, UvTriangulation,
};
