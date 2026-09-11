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
    /// 頂点法線リスト [[x, y, z], ...]
    ///
    /// > **溶接した頂点は、集まる面の法線の平均**です（4-425）。
    /// > **1 つの頂点は 1 つの法線しか持てない**ので、**箱の隅では
    /// > 3 面の平均**（`(±1, ±1, ±1)/√3`）になります——**平らな陰影が
    /// > 要るなら、受け取った側で面ごとに割り直してください。**
    /// >
    /// > **向きは保証します**——**どの三角形から見ても、面法線との
    /// > 内積は正**です（`check_python_surface.py` が 6 形 × 刻み 2 つで
    /// > 見ています）。**2026/09/11 までは、箱の 8 頂点すべてが
    /// > `(0, 0, 1)`** で、**6 面のうち 4 面で真逆**でした。
    pub fn normals(&self) -> Vec<[f64; 3]> {
        self.mesh.normals.iter().map(|n| [n.x, n.y, n.z]).collect()
    }

    /// UV座標リスト [[u, v], ...]
    ///
    /// > **⚠ 正規化していません**（4-425）。**`uv` は、その頂点が
    /// > 乗っている面の媒介変数そのもの**です。**[0, 1] に収まっている
    /// > とは限りません。**
    /// >
    /// > * **球・円柱・トーラス（こちらの作り）は [0, 1]**——
    /// >   **その曲面の媒介変数の範囲がそれだから**であって、
    /// >   **正規化しているからではありません**
    /// > * **平面は、平面へ落とした座標**（模型の寸法。`30x20x10` の箱で
    /// >   `[0, 30]`）
    /// > * **読み込んだ曲面は、そのファイルの範囲**——実測:
    /// >   `occ_reference_cylinder_nurbs.step` は **u [0, 6.185]（≒ 2π）、
    /// >   v [0, 40.0]（＝ 高さ）**
    /// >
    /// > **テクスチャ座標として使うなら、面ごとに割り直してください。**
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
