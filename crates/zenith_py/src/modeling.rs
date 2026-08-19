use pyo3::exceptions::PyValueError;
use pyo3::prelude::*;
use zenith_algo::{
    ChamferBuilder, CurvePatchBuilder, FilletBuilder, HoleBuilder, LoftBuilder, RevolveBuilder,
    ShellBuilder, SweepBuilder, ThickenBuilder,
};
use zenith_geom::{ControlPoint3, KnotVector, NurbsCurve3};
use zenith_io::StepExporter;
use zenith_math::{Point3, Tolerance};
use zenith_tess::{tessellate_solid, tessellate_surface, TessellationParams};

use crate::mesh::PyMesh;

/// 4エッジに面取りを適用した直方体メッシュおよびSTEP出力
#[pyfunction]
#[pyo3(signature = (dx, dy, dz, chamfer, u_divisions = 8, v_divisions = 8, step_path = None))]
pub fn make_chamfered_box(
    dx: f64,
    dy: f64,
    dz: f64,
    chamfer: f64,
    u_divisions: usize,
    v_divisions: usize,
    step_path: Option<&str>,
) -> PyResult<PyMesh> {
    let tol = Tolerance::default();
    let solid = ChamferBuilder::chamfer_box_z_edges(dx, dy, dz, chamfer, &tol)
        .map_err(|e| PyValueError::new_err(format!("Chamfer box creation failed: {}", e)))?;

    if let Some(path) = step_path {
        StepExporter::export_solid_to_file(&solid, path, "ZENITH_CHAMFERED_BOX")
            .map_err(|e| PyValueError::new_err(format!("STEP export failed: {}", e)))?;
    }

    let params = TessellationParams {
        u_divisions,
        v_divisions,
    };
    let mesh = tessellate_solid(&solid, &params);
    Ok(PyMesh { mesh })
}

/// 4エッジにフィレットを適用した直方体メッシュおよびSTEP出力
#[pyfunction]
#[pyo3(signature = (dx, dy, dz, radius, u_divisions = 16, v_divisions = 16, step_path = None))]
pub fn make_filleted_box(
    dx: f64,
    dy: f64,
    dz: f64,
    radius: f64,
    u_divisions: usize,
    v_divisions: usize,
    step_path: Option<&str>,
) -> PyResult<PyMesh> {
    let tol = Tolerance::default();
    let solid = FilletBuilder::fillet_box_z_edges(dx, dy, dz, radius, &tol)
        .map_err(|e| PyValueError::new_err(format!("Fillet box creation failed: {}", e)))?;

    if let Some(path) = step_path {
        StepExporter::export_solid_to_file(&solid, path, "ZENITH_FILLETED_BOX")
            .map_err(|e| PyValueError::new_err(format!("STEP export failed: {}", e)))?;
    }

    let params = TessellationParams {
        u_divisions,
        v_divisions,
    };
    let mesh = tessellate_solid(&solid, &params);
    Ok(PyMesh { mesh })
}

/// 貫通穴を開けた直方体ソリッドの生成（STEP対応）
#[pyfunction]
#[pyo3(signature = (dx, dy, dz, hole_radius, u_divisions = 16, v_divisions = 16, step_path = None))]
pub fn make_drilled_box(
    dx: f64,
    dy: f64,
    dz: f64,
    hole_radius: f64,
    u_divisions: usize,
    v_divisions: usize,
    step_path: Option<&str>,
) -> PyResult<PyMesh> {
    let solid = HoleBuilder::make_drilled_box(dx, dy, dz, hole_radius)
        .map_err(|e| PyValueError::new_err(format!("Drilled box creation failed: {}", e)))?;

    if let Some(path) = step_path {
        StepExporter::export_solid_to_file(&solid, path, "ZENITH_DRILLED_BOX")
            .map_err(|e| PyValueError::new_err(format!("STEP export failed: {}", e)))?;
    }

    let params = TessellationParams {
        u_divisions,
        v_divisions,
    };
    let mesh = tessellate_solid(&solid, &params);
    Ok(PyMesh { mesh })
}

/// 3Dカーブパスに沿った円形断面スイープ（パイプ）ソリッドの生成（STEP対応）
#[pyfunction]
#[pyo3(signature = (path_points, radius, num_sections = 16, u_divisions = 16, v_divisions = 16, step_path = None))]
pub fn make_sweep_pipe(
    path_points: Vec<[f64; 3]>,
    radius: f64,
    num_sections: usize,
    u_divisions: usize,
    v_divisions: usize,
    step_path: Option<&str>,
) -> PyResult<PyMesh> {
    let n = path_points.len();
    if n < 2 {
        return Err(PyValueError::new_err(
            "Sweep path requires at least 2 points",
        ));
    }
    let degree = (n - 1).min(3);
    let ctrl_pts = path_points
        .into_iter()
        .map(|p| ControlPoint3::unweighted(Point3::new(p[0], p[1], p[2])))
        .collect();
    let knots = KnotVector::clamped_uniform(n, degree);
    let path = NurbsCurve3::new(degree, ctrl_pts, knots)
        .map_err(|e| PyValueError::new_err(format!("Path curve creation failed: {}", e)))?;

    let solid = SweepBuilder::sweep_circle_along_curve(&path, radius, num_sections)
        .map_err(|e| PyValueError::new_err(format!("Sweep failed: {}", e)))?;

    if let Some(path_str) = step_path {
        StepExporter::export_solid_to_file(&solid, path_str, "ZENITH_SWEEP_PIPE")
            .map_err(|e| PyValueError::new_err(format!("STEP export failed: {}", e)))?;
    }

    let params = TessellationParams {
        u_divisions,
        v_divisions,
    };
    let mesh = tessellate_solid(&solid, &params);
    Ok(PyMesh { mesh })
}

/// 3次元NURBS曲線の回転体を生成
#[pyfunction]
#[pyo3(signature = (profile_points, axis_origin = [0.0, 0.0, 0.0], axis_dir = [0.0, 0.0, 1.0], angle_deg = 360.0, u_divisions = 32, v_divisions = 32))]
pub fn make_revolve(
    profile_points: Vec<[f64; 3]>,
    axis_origin: [f64; 3],
    axis_dir: [f64; 3],
    angle_deg: f64,
    u_divisions: usize,
    v_divisions: usize,
) -> PyResult<PyMesh> {
    let n = profile_points.len();
    if n < 2 {
        return Err(PyValueError::new_err("Profile must have at least 2 points"));
    }
    let degree = (n - 1).min(3);
    let ctrl_pts = profile_points
        .into_iter()
        .map(|p| ControlPoint3::unweighted(Point3::new(p[0], p[1], p[2])))
        .collect();
    let knots = KnotVector::clamped_uniform(n, degree);
    let curve = NurbsCurve3::new(degree, ctrl_pts, knots)
        .map_err(|e| PyValueError::new_err(format!("NURBS creation failed: {}", e)))?;

    let tol = Tolerance::default();
    let origin = Point3::new(axis_origin[0], axis_origin[1], axis_origin[2]);
    let dir = zenith_math::Vec3::new(axis_dir[0], axis_dir[1], axis_dir[2]);
    let angle_rad = angle_deg.to_radians();

    let surf = RevolveBuilder::revolve_curve(&curve, origin, dir, angle_rad, &tol)
        .map_err(|e| PyValueError::new_err(format!("Revolve failed: {}", e)))?;

    let params = TessellationParams {
        u_divisions,
        v_divisions,
    };
    let mesh = tessellate_surface(&surf, &params, zenith_topo::Orientation::Forward);
    Ok(PyMesh { mesh })
}

/// 断面曲線群からロフト曲面を生成（不揃いな点数のプロファイルも自動対応）
#[pyfunction]
#[pyo3(signature = (profiles, degree_v = 2, u_divisions = 24, v_divisions = 24, step_path = None))]
pub fn make_loft(
    profiles: Vec<Vec<[f64; 3]>>,
    degree_v: usize,
    u_divisions: usize,
    v_divisions: usize,
    step_path: Option<&str>,
) -> PyResult<PyMesh> {
    if profiles.len() < 2 {
        return Err(PyValueError::new_err("Loft requires at least 2 profiles"));
    }

    let mut curves = Vec::with_capacity(profiles.len());
    for p_pts in profiles {
        if p_pts.len() < 2 {
            return Err(PyValueError::new_err(
                "Each profile must contain at least 2 points",
            ));
        }
        let degree = (p_pts.len() - 1).min(3);
        let pts: Vec<Point3> = p_pts
            .into_iter()
            .map(|p| Point3::new(p[0], p[1], p[2]))
            .collect();
        let c = NurbsCurve3::bspline_from_points(degree, pts)
            .map_err(PyValueError::new_err)?;
        curves.push(c);
    }

    let tol = Tolerance::default();
    let surf = LoftBuilder::loft_curves(&curves, degree_v, &tol)
        .map_err(|e| PyValueError::new_err(format!("Loft failed: {}", e)))?;

    let _ = step_path; // 将来のSTEPサーフェス出力用

    let params = TessellationParams {
        u_divisions,
        v_divisions,
    };
    let mesh = tessellate_surface(&surf, &params, zenith_topo::Orientation::Forward);
    Ok(PyMesh { mesh })
}

/// 閉じた断面ポリライン群から完全閉B-Repロフトソリッドを生成（STEP対応）
#[pyfunction]
#[pyo3(signature = (sections, degree_v = 2, u_divisions = 24, v_divisions = 24, step_path = None))]
pub fn make_loft_solid(
    sections: Vec<Vec<[f64; 3]>>,
    degree_v: usize,
    u_divisions: usize,
    v_divisions: usize,
    step_path: Option<&str>,
) -> PyResult<PyMesh> {
    if sections.len() < 2 {
        return Err(PyValueError::new_err("Loft solid requires at least 2 sections"));
    }

    let tol = Tolerance::default();
    let mut wires = Vec::with_capacity(sections.len());

    for sec in sections {
        let n = sec.len();
        if n < 3 {
            return Err(PyValueError::new_err(
                "Each section must have at least 3 vertices to form a closed wire",
            ));
        }

        let vertices: Vec<zenith_topo::Vertex> = sec
            .iter()
            .map(|p| zenith_topo::Vertex::from_point(Point3::new(p[0], p[1], p[2])))
            .collect();

        let mut edges = Vec::with_capacity(n);
        for i in 0..n {
            let next_i = (i + 1) % n;
            let edge = zenith_topo::Edge::line_between(
                vertices[i].clone(),
                vertices[next_i].clone(),
            )
            .map_err(|e| PyValueError::new_err(format!("Edge creation failed: {}", e)))?;
            edges.push(zenith_topo::OrientedEdge::forward(edge));
        }
        wires.push(zenith_topo::Wire::new(edges));
    }

    let solid = LoftBuilder::loft_solid(&wires, degree_v, &tol)
        .map_err(|e| PyValueError::new_err(format!("Loft solid failed: {}", e)))?;

    if let Some(path) = step_path {
        StepExporter::export_solid_to_file(&solid, path, "ZENITH_LOFT_SOLID")
            .map_err(|e| PyValueError::new_err(format!("STEP export failed: {}", e)))?;
    }

    let params = TessellationParams {
        u_divisions,
        v_divisions,
    };
    let mesh = tessellate_solid(&solid, &params);
    Ok(PyMesh { mesh })
}


/// シェル化・肉厚中空ソリッド（容器・ケーシング）の生成
#[pyfunction]
#[pyo3(signature = (dx, dy, dz, thickness, open_face_index = 1, u_divisions = 4, v_divisions = 4))]
pub fn make_hollow_box(
    dx: f64,
    dy: f64,
    dz: f64,
    thickness: f64,
    open_face_index: usize,
    u_divisions: usize,
    v_divisions: usize,
) -> PyResult<PyMesh> {
    let solid = ShellBuilder::make_hollow_box(dx, dy, dz, thickness, open_face_index)
        .map_err(|e| PyValueError::new_err(format!("Hollow box failed: {}", e)))?;

    let params = TessellationParams {
        u_divisions,
        v_divisions,
    };
    let mesh = tessellate_solid(&solid, &params);
    Ok(PyMesh { mesh })
}

/// 2つのメッシュ間のブーリアン演算 (0: Union, 1: Difference, 2: Intersection)
#[pyfunction]
#[pyo3(signature = (mesh_a, mesh_b, op_type = 1))]
pub fn make_boolean(mesh_a: &PyMesh, mesh_b: &PyMesh, op_type: u8) -> PyResult<PyMesh> {
    let op = match op_type {
        0 => zenith_algo::BooleanOpType::Union,
        1 => zenith_algo::BooleanOpType::Difference,
        2 => zenith_algo::BooleanOpType::Intersection,
        _ => {
            return Err(PyValueError::new_err(
                "Invalid op_type: 0=Union, 1=Difference, 2=Intersection",
            ))
        }
    };

    let result = zenith_algo::BooleanEngine::boolean_meshes(&mesh_a.mesh, &mesh_b.mesh, op)
        .map_err(|e| PyValueError::new_err(format!("Boolean failed: {}", e)))?;

    Ok(PyMesh { mesh: result })
}

/// 4境界自由曲面パッチに厚みを与えてソリッド化
#[pyfunction]
#[pyo3(signature = (c_u0, c_u1, c_0v, c_1v, thickness = 2.0, u_divisions = 16, v_divisions = 16))]
pub fn thicken_surface_patch(
    c_u0: Vec<[f64; 3]>,
    c_u1: Vec<[f64; 3]>,
    c_0v: Vec<[f64; 3]>,
    c_1v: Vec<[f64; 3]>,
    thickness: f64,
    u_divisions: usize,
    v_divisions: usize,
) -> PyResult<PyMesh> {
    let curve_u0 = NurbsCurve3::bspline_from_points(
        3,
        c_u0.into_iter()
            .map(|p| Point3::new(p[0], p[1], p[2]))
            .collect(),
    )
    .map_err(|e| PyValueError::new_err(format!("Invalid curve u0: {}", e)))?;
    let curve_u1 = NurbsCurve3::bspline_from_points(
        3,
        c_u1.into_iter()
            .map(|p| Point3::new(p[0], p[1], p[2]))
            .collect(),
    )
    .map_err(|e| PyValueError::new_err(format!("Invalid curve u1: {}", e)))?;
    let curve_0v = NurbsCurve3::bspline_from_points(
        3,
        c_0v.into_iter()
            .map(|p| Point3::new(p[0], p[1], p[2]))
            .collect(),
    )
    .map_err(|e| PyValueError::new_err(format!("Invalid curve 0v: {}", e)))?;
    let curve_1v = NurbsCurve3::bspline_from_points(
        3,
        c_1v.into_iter()
            .map(|p| Point3::new(p[0], p[1], p[2]))
            .collect(),
    )
    .map_err(|e| PyValueError::new_err(format!("Invalid curve 1v: {}", e)))?;

    let tol = Tolerance::default();
    let face = CurvePatchBuilder::build_from_4_curves(curve_u0, curve_u1, curve_0v, curve_1v, &tol)
        .map_err(|e| PyValueError::new_err(format!("Patch creation failed: {}", e)))?;

    let solid = ThickenBuilder::thicken_face(&face, thickness, &tol)
        .map_err(|e| PyValueError::new_err(format!("Thicken failed: {}", e)))?;

    let params = TessellationParams {
        u_divisions,
        v_divisions,
    };
    let mesh = tessellate_solid(&solid, &params);
    Ok(PyMesh { mesh })
}
