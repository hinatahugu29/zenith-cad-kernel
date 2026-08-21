use pyo3::exceptions::PyValueError;
use pyo3::prelude::*;
use zenith_algo::{CurvePatchBuilder, PrimitiveBuilder};
use zenith_geom::{ControlPoint3, KnotVector, NurbsCurve3};
use zenith_io::StepExporter;
use zenith_math::{Point3, Tolerance};
use zenith_tess::{tessellate_face, tessellate_solid, TessellationParams};

use crate::mesh::PyMesh;

/// 4つの制御点リストからPlasticity風カーブパッチサーフェスを構築し、テッセレーションメッシュを返す
#[pyfunction]
#[pyo3(signature = (c0, c1, d0, d1, u_divisions = 24, v_divisions = 24))]
pub fn make_curve_patch(
    c0: Vec<[f64; 3]>,
    c1: Vec<[f64; 3]>,
    d0: Vec<[f64; 3]>,
    d1: Vec<[f64; 3]>,
    u_divisions: usize,
    v_divisions: usize,
) -> PyResult<PyMesh> {
    let to_nurbs = |pts: Vec<[f64; 3]>| -> PyResult<NurbsCurve3> {
        let n = pts.len();
        if n < 2 {
            return Err(PyValueError::new_err(
                "Each curve must have at least 2 points",
            ));
        }
        let degree = (n - 1).min(3);
        let ctrl_pts = pts
            .into_iter()
            .map(|p| ControlPoint3::unweighted(Point3::new(p[0], p[1], p[2])))
            .collect();
        let knots = KnotVector::clamped_uniform(n, degree);
        NurbsCurve3::new(degree, ctrl_pts, knots)
            .map_err(|e| PyValueError::new_err(format!("Failed to create NURBS curve: {}", e)))
    };

    let curve_c0 = to_nurbs(c0)?;
    let curve_c1 = to_nurbs(c1)?;
    let curve_d0 = to_nurbs(d0)?;
    let curve_d1 = to_nurbs(d1)?;

    let tol = Tolerance::default();
    let face = CurvePatchBuilder::build_from_4_curves(curve_c0, curve_c1, curve_d0, curve_d1, &tol)
        .map_err(|e| PyValueError::new_err(format!("Curve patch building failed: {}", e)))?;

    let params = TessellationParams {
        u_divisions,
        v_divisions,
    };
    let mesh = tessellate_face(&face, &params);
    Ok(PyMesh { mesh })
}

/// 直方体ソリッドを生成
#[pyfunction]
#[pyo3(signature = (dx, dy, dz, u_divisions = 4, v_divisions = 4, step_path = None))]
pub fn make_box(
    dx: f64,
    dy: f64,
    dz: f64,
    u_divisions: usize,
    v_divisions: usize,
    step_path: Option<&str>,
) -> PyResult<PyMesh> {
    let solid = PrimitiveBuilder::make_box(dx, dy, dz)
        .map_err(|e| PyValueError::new_err(format!("Box generation failed: {}", e)))?;

    if let Some(path) = step_path {
        StepExporter::export_solid_to_file(&solid, path, "ZENITH_BOX")
            .map_err(|e| PyValueError::new_err(format!("STEP export failed: {}", e)))?;
    }

    let params = TessellationParams {
        u_divisions,
        v_divisions,
    };
    let mesh = tessellate_solid(&solid, &params);
    Ok(PyMesh { mesh })
}

/// 円柱ソリッドの生成（STEP対応）
#[pyfunction]
#[pyo3(signature = (radius, height, u_divisions = 16, v_divisions = 16, step_path = None))]
pub fn make_cylinder(
    radius: f64,
    height: f64,
    u_divisions: usize,
    v_divisions: usize,
    step_path: Option<&str>,
) -> PyResult<PyMesh> {
    let solid = PrimitiveBuilder::make_cylinder(radius, height)
        .map_err(|e| PyValueError::new_err(format!("Cylinder creation failed: {}", e)))?;

    if let Some(path) = step_path {
        StepExporter::export_solid_to_file(&solid, path, "ZENITH_CYLINDER")
            .map_err(|e| PyValueError::new_err(format!("STEP export failed: {}", e)))?;
    }

    let params = TessellationParams {
        u_divisions,
        v_divisions,
    };
    let mesh = tessellate_solid(&solid, &params);
    Ok(PyMesh { mesh })
}

/// 球体ソリッドの生成（STEP対応）
#[pyfunction]
#[pyo3(signature = (radius, u_divisions = 32, v_divisions = 32, step_path = None))]
pub fn make_sphere(
    radius: f64,
    u_divisions: usize,
    v_divisions: usize,
    step_path: Option<&str>,
) -> PyResult<PyMesh> {
    let solid = PrimitiveBuilder::make_sphere(radius)
        .map_err(|e| PyValueError::new_err(format!("Sphere creation failed: {}", e)))?;

    if let Some(path) = step_path {
        StepExporter::export_solid_to_file(&solid, path, "ZENITH_SPHERE")
            .map_err(|e| PyValueError::new_err(format!("STEP export failed: {}", e)))?;
    }

    let params = TessellationParams {
        u_divisions,
        v_divisions,
    };
    let mesh = tessellate_solid(&solid, &params);
    Ok(PyMesh { mesh })
}

/// 円錐 / 円錐台ソリッドの生成（STEP対応）
#[pyfunction]
#[pyo3(signature = (r_bottom, r_top, height, u_divisions = 16, v_divisions = 16, step_path = None))]
pub fn make_cone(
    r_bottom: f64,
    r_top: f64,
    height: f64,
    u_divisions: usize,
    v_divisions: usize,
    step_path: Option<&str>,
) -> PyResult<PyMesh> {
    let solid = PrimitiveBuilder::make_cone(r_bottom, r_top, height)
        .map_err(|e| PyValueError::new_err(format!("Cone creation failed: {}", e)))?;

    if let Some(path) = step_path {
        StepExporter::export_solid_to_file(&solid, path, "ZENITH_CONE")
            .map_err(|e| PyValueError::new_err(format!("STEP export failed: {}", e)))?;
    }

    let params = TessellationParams {
        u_divisions,
        v_divisions,
    };
    let mesh = tessellate_solid(&solid, &params);
    Ok(PyMesh { mesh })
}

/// トーラス（ドーナツ）ソリッドの生成
#[pyfunction]
#[pyo3(signature = (r_major, r_minor, u_divisions = 32, v_divisions = 16))]
pub fn make_torus(
    r_major: f64,
    r_minor: f64,
    u_divisions: usize,
    v_divisions: usize,
) -> PyResult<PyMesh> {
    let solid = PrimitiveBuilder::make_torus(r_major, r_minor)
        .map_err(|e| PyValueError::new_err(format!("Torus creation failed: {}", e)))?;

    let params = TessellationParams {
        u_divisions,
        v_divisions,
    };
    let mesh = tessellate_solid(&solid, &params);
    Ok(PyMesh { mesh })
}

/// 正多角柱（Prism）ソリッドの生成（STEP対応）
#[pyfunction]
#[pyo3(signature = (num_sides, radius, height, u_divisions = 4, v_divisions = 4, step_path = None))]
pub fn make_regular_prism(
    num_sides: usize,
    radius: f64,
    height: f64,
    u_divisions: usize,
    v_divisions: usize,
    step_path: Option<&str>,
) -> PyResult<PyMesh> {
    let solid = PrimitiveBuilder::make_regular_prism(num_sides, radius, height)
        .map_err(|e| PyValueError::new_err(format!("Regular prism creation failed: {}", e)))?;

    if let Some(path) = step_path {
        StepExporter::export_solid_to_file(&solid, path, "ZENITH_PRISM")
            .map_err(|e| PyValueError::new_err(format!("STEP export failed: {}", e)))?;
    }

    let params = TessellationParams {
        u_divisions,
        v_divisions,
    };
    let mesh = tessellate_solid(&solid, &params);
    Ok(PyMesh { mesh })
}
