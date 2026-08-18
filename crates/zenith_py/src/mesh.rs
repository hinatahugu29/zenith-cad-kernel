use pyo3::exceptions::PyValueError;
use pyo3::prelude::*;
use zenith_algo::MassCalculator;
use zenith_io::{ObjExporter, StlExporter};
use zenith_tess::TriangleMesh;

/// Blender向け高速メッシュコンテナ
#[pyclass(name = "Mesh")]
#[derive(Clone)]
pub struct PyMesh {
    pub mesh: TriangleMesh,
}

#[pymethods]
impl PyMesh {
    /// 頂点座標リスト [[x, y, z], ...]
    #[getter]
    pub fn vertices(&self) -> Vec<[f64; 3]> {
        self.mesh
            .positions
            .iter()
            .map(|p| [p.x, p.y, p.z])
            .collect()
    }

    /// 法線ベクトルリスト [[nx, ny, nz], ...]
    #[getter]
    pub fn normals(&self) -> Vec<[f64; 3]> {
        self.mesh.normals.iter().map(|n| [n.x, n.y, n.z]).collect()
    }

    /// UV座標リスト [[u, v], ...]
    #[getter]
    pub fn uvs(&self) -> Vec<[f64; 2]> {
        self.mesh.uvs.iter().map(|uv| [uv.x, uv.y]).collect()
    }

    /// 三角形インデックスリスト [[i0, i1, i2], ...]
    #[getter]
    pub fn faces(&self) -> Vec<[u32; 3]> {
        self.mesh.indices.clone()
    }

    /// 頂点数
    #[getter]
    pub fn num_vertices(&self) -> usize {
        self.mesh.num_vertices()
    }

    /// 面数
    #[getter]
    pub fn num_faces(&self) -> usize {
        self.mesh.num_triangles()
    }

    /// 表面積 (mm^2)
    #[getter]
    pub fn surface_area(&self) -> f64 {
        MassCalculator::compute_from_mesh(&self.mesh).surface_area
    }

    /// 体積 (mm^3)
    #[getter]
    pub fn volume(&self) -> f64 {
        MassCalculator::compute_from_mesh(&self.mesh).volume
    }

    /// 重心座標 [x, y, z] (mm)
    #[getter]
    pub fn center_of_mass(&self) -> [f64; 3] {
        let cm = MassCalculator::compute_from_mesh(&self.mesh).center_of_mass;
        [cm.x, cm.y, cm.z]
    }

    /// Wavefront OBJ 文字列を取得
    #[pyo3(signature = (object_name=None))]
    pub fn to_obj_string(&self, object_name: Option<&str>) -> String {
        self.mesh
            .to_obj_string(object_name.unwrap_or("zenith_mesh"))
    }

    /// OBJファイルへ保存
    #[pyo3(signature = (path, object_name=None))]
    pub fn export_obj(&self, path: &str, object_name: Option<&str>) -> PyResult<()> {
        ObjExporter::export_to_file(&self.mesh, path, object_name.unwrap_or("zenith_mesh"))
            .map_err(|e| PyValueError::new_err(format!("Failed to export OBJ: {}", e)))
    }

    /// 3Dプリント用 Binary STL ファイルへ保存
    #[pyo3(signature = (path))]
    pub fn export_stl(&self, path: &str) -> PyResult<()> {
        StlExporter::export_binary(&self.mesh, path)
            .map_err(|e| PyValueError::new_err(format!("Failed to export STL: {}", e)))
    }

    /// 二面角しきい値 (deg) に基づく CAD 的特徴エッジ（稜線）インデックス [[i0, i1], ...]
    #[pyo3(signature = (angle_deg=25.0))]
    pub fn feature_edges(&self, angle_deg: f64) -> Vec<[u32; 2]> {
        self.mesh.extract_feature_edges(angle_deg)
    }

    /// 疑似 3 灯ランバートライティング頂点カラー [[r, g, b, a], ...]
    #[pyo3(signature = (base_rgb=(0.6, 0.6, 0.6), selected=false))]
    pub fn shaded_colors(&self, base_rgb: (f32, f32, f32), selected: bool) -> Vec<[f32; 4]> {
        self.mesh
            .compute_shaded_colors([base_rgb.0, base_rgb.1, base_rgb.2], selected)
    }
}
