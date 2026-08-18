use pyo3::exceptions::PyValueError;
use pyo3::prelude::*;
use zenith_tess::{tessellate_solid, TessellationParams};

use crate::mesh::PyMesh;

/// 外部STEPファイルをインポートしてメッシュ化
#[pyfunction]
#[pyo3(signature = (file_path, u_divisions = 16, v_divisions = 16))]
pub fn import_step_file(
    file_path: &str,
    u_divisions: usize,
    v_divisions: usize,
) -> PyResult<PyMesh> {
    let solid = zenith_io::StepImporter::import_solid_from_file(file_path)
        .map_err(|e| PyValueError::new_err(format!("STEP import failed: {}", e)))?;

    let params = TessellationParams {
        u_divisions,
        v_divisions,
    };
    let mesh = tessellate_solid(&solid, &params);
    Ok(PyMesh { mesh })
}
