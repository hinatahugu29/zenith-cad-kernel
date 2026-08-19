use pyo3::exceptions::PyValueError;
use pyo3::prelude::*;
use zenith_algo::{CapBuilder, DirectModeling, PrimitiveBuilder};
use zenith_math::Point3;
use zenith_tess::{tessellate_face, tessellate_solid, TessellationParams};

use crate::mesh::PyMesh;

/// 単一垂直エッジを指定してフィレットを適用
#[pyfunction]
#[pyo3(signature = (dx, dy, dz, edge_index, radius, u_divisions = 16, v_divisions = 16))]
pub fn fillet_box_single_edge(
    dx: f64,
    dy: f64,
    dz: f64,
    edge_index: usize,
    radius: f64,
    u_divisions: usize,
    v_divisions: usize,
) -> PyResult<PyMesh> {
    let solid = DirectModeling::fillet_box_single_edge(dx, dy, dz, edge_index, radius)
        .map_err(|e| PyValueError::new_err(format!("Single fillet failed: {}", e)))?;

    let params = TessellationParams {
        u_divisions,
        v_divisions,
    };
    let mesh = tessellate_solid(&solid, &params);
    Ok(PyMesh { mesh })
}

/// 単一垂直エッジを指定して45度面取りを適用（STEP対応）
#[pyfunction]
#[pyo3(signature = (dx, dy, dz, edge_index, distance, u_divisions = 4, v_divisions = 4, step_path = None))]
pub fn chamfer_box_single_edge(
    dx: f64,
    dy: f64,
    dz: f64,
    edge_index: usize,
    distance: f64,
    u_divisions: usize,
    v_divisions: usize,
    step_path: Option<&str>,
) -> PyResult<PyMesh> {
    let solid = DirectModeling::chamfer_box_single_edge(dx, dy, dz, edge_index, distance)
        .map_err(|e| PyValueError::new_err(format!("Single chamfer failed: {}", e)))?;

    if let Some(path) = step_path {
        zenith_io::StepExporter::export_solid_to_file(&solid, path, "ZENITH_CHAMFER_BOX")
            .map_err(|e| PyValueError::new_err(format!("STEP export failed: {}", e)))?;
    }

    let params = TessellationParams {
        u_divisions,
        v_divisions,
    };
    let mesh = tessellate_solid(&solid, &params);
    Ok(PyMesh { mesh })
}


/// 直方体の特定面を法線方向にPush-Pull押し出し
#[pyfunction]
#[pyo3(signature = (dx, dy, dz, face_index, distance, u_divisions = 4, v_divisions = 4))]
pub fn push_pull_box(
    dx: f64,
    dy: f64,
    dz: f64,
    face_index: usize,
    distance: f64,
    u_divisions: usize,
    v_divisions: usize,
) -> PyResult<PyMesh> {
    let solid = PrimitiveBuilder::make_box(dx, dy, dz)
        .map_err(|e| PyValueError::new_err(format!("Box failed: {}", e)))?;

    let modified = DirectModeling::push_pull_face(&solid, face_index, distance)
        .map_err(|e| PyValueError::new_err(format!("Push-pull failed: {}", e)))?;

    let params = TessellationParams {
        u_divisions,
        v_divisions,
    };
    let mesh = tessellate_solid(&modified, &params);
    Ok(PyMesh { mesh })
}

/// 直方体の特定面を角度傾斜（Taper / Draft）
#[pyfunction]
#[pyo3(signature = (dx, dy, dz, face_index, angle_deg, u_divisions = 4, v_divisions = 4))]
pub fn taper_box(
    dx: f64,
    dy: f64,
    dz: f64,
    face_index: usize,
    angle_deg: f64,
    u_divisions: usize,
    v_divisions: usize,
) -> PyResult<PyMesh> {
    let solid = PrimitiveBuilder::make_box(dx, dy, dz)
        .map_err(|e| PyValueError::new_err(format!("Box failed: {}", e)))?;

    let axis_origin = Point3::new(0.0, 0.0, dz);
    let axis_dir = zenith_math::Vec3::new(1.0, 0.0, 0.0);

    let modified = DirectModeling::taper_face(&solid, face_index, axis_origin, axis_dir, angle_deg)
        .map_err(|e| PyValueError::new_err(format!("Taper failed: {}", e)))?;

    let params = TessellationParams {
        u_divisions,
        v_divisions,
    };
    let mesh = tessellate_solid(&modified, &params);
    Ok(PyMesh { mesh })
}

/// 任意の閉じた3Dポリラインワイヤを平面Face（Planar Cap）で塞ぐ
#[pyfunction]
#[pyo3(signature = (wire_points, u_divisions = 16, v_divisions = 16))]
pub fn cap_planar_wire(
    wire_points: Vec<[f64; 3]>,
    u_divisions: usize,
    v_divisions: usize,
) -> PyResult<PyMesh> {
    let n = wire_points.len();
    if n < 3 {
        return Err(PyValueError::new_err(
            "Planar cap requires at least 3 points",
        ));
    }
    let mut edges = Vec::with_capacity(n);
    for i in 0..n {
        let p_curr = Point3::new(wire_points[i][0], wire_points[i][1], wire_points[i][2]);
        let p_next = Point3::new(
            wire_points[(i + 1) % n][0],
            wire_points[(i + 1) % n][1],
            wire_points[(i + 1) % n][2],
        );
        let v_curr = zenith_topo::Vertex::from_point(p_curr);
        let v_next = zenith_topo::Vertex::from_point(p_next);
        let e = zenith_topo::Edge::line_between(v_curr, v_next)
            .map_err(|e| PyValueError::new_err(format!("Edge creation failed: {}", e)))?;
        edges.push(zenith_topo::OrientedEdge::forward(e));
    }
    let wire = zenith_topo::Wire::new(edges);
    let face = CapBuilder::make_planar_cap(wire)
        .map_err(|e| PyValueError::new_err(format!("Planar cap failed: {}", e)))?;

    let params = TessellationParams {
        u_divisions,
        v_divisions,
    };
    let mesh = tessellate_face(&face, &params);
    Ok(PyMesh { mesh })
}

/// 任意の閉じた3Dポリラインワイヤをドーム状曲面パッチ（Dome Cap）で塞ぐ
#[pyfunction]
#[pyo3(signature = (wire_points, bulge = 5.0, u_divisions = 24, v_divisions = 24))]
pub fn cap_dome_wire(
    wire_points: Vec<[f64; 3]>,
    bulge: f64,
    u_divisions: usize,
    v_divisions: usize,
) -> PyResult<PyMesh> {
    let n = wire_points.len();
    if n < 3 {
        return Err(PyValueError::new_err("Dome cap requires at least 3 points"));
    }
    let mut edges = Vec::with_capacity(n);
    for i in 0..n {
        let p_curr = Point3::new(wire_points[i][0], wire_points[i][1], wire_points[i][2]);
        let p_next = Point3::new(
            wire_points[(i + 1) % n][0],
            wire_points[(i + 1) % n][1],
            wire_points[(i + 1) % n][2],
        );
        let v_curr = zenith_topo::Vertex::from_point(p_curr);
        let v_next = zenith_topo::Vertex::from_point(p_next);
        let e = zenith_topo::Edge::line_between(v_curr, v_next)
            .map_err(|e| PyValueError::new_err(format!("Edge creation failed: {}", e)))?;
        edges.push(zenith_topo::OrientedEdge::forward(e));
    }
    let wire = zenith_topo::Wire::new(edges);
    let face = CapBuilder::make_dome_patch(wire, bulge)
        .map_err(|e| PyValueError::new_err(format!("Dome cap failed: {}", e)))?;

    let params = TessellationParams {
        u_divisions,
        v_divisions,
    };
    let mesh = tessellate_face(&face, &params);
    Ok(PyMesh { mesh })
}
