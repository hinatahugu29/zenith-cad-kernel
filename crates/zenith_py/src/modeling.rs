use pyo3::exceptions::PyValueError;
use pyo3::prelude::*;
use zenith_algo::{
    ChamferBuilder, CurvePatchBuilder, FilletBuilder, HoleBuilder, LoftBuilder, RevolveBuilder,
    ShellBuilder, SweepBuilder, ThickenBuilder,
};
use zenith_geom::{ControlPoint3, KnotVector, NurbsCurve3};
use zenith_io::StepExporter;
use zenith_math::{Point3, Tolerance, Vec3};

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

/// 任意の閉じた2D/3D断面ポリラインを3Dパス曲線に沿ってスイープした完全閉B-Repソリッドの生成（STEP対応）
#[pyfunction]
#[pyo3(signature = (profile_points, path_points, num_sections = 16, u_divisions = 16, v_divisions = 16, step_path = None))]
pub fn make_sweep_wire(
    profile_points: Vec<[f64; 3]>,
    path_points: Vec<[f64; 3]>,
    num_sections: usize,
    u_divisions: usize,
    v_divisions: usize,
    step_path: Option<&str>,
) -> PyResult<PyMesh> {
    let k = profile_points.len();
    if k < 3 {
        return Err(PyValueError::new_err(
            "Sweep profile wire requires at least 3 points",
        ));
    }
    let n_path = path_points.len();
    if n_path < 2 {
        return Err(PyValueError::new_err(
            "Sweep path requires at least 2 points",
        ));
    }

    let tol = Tolerance::default();

    // 断面ワイヤの構築
    let vertices: Vec<zenith_topo::Vertex> = profile_points
        .iter()
        .map(|p| zenith_topo::Vertex::from_point(Point3::new(p[0], p[1], p[2])))
        .collect();

    let mut edges = Vec::with_capacity(k);
    for i in 0..k {
        let next_i = (i + 1) % k;
        let edge = zenith_topo::Edge::line_between(vertices[i].clone(), vertices[next_i].clone())
            .map_err(|e| PyValueError::new_err(format!("Edge creation failed: {}", e)))?;
        edges.push(zenith_topo::OrientedEdge::forward(edge));
    }
    let profile_wire = zenith_topo::Wire::new(edges);

    // 3D パス曲線の構築
    let degree = (n_path - 1).min(3);
    let path_ctrl_pts = path_points
        .into_iter()
        .map(|p| ControlPoint3::unweighted(Point3::new(p[0], p[1], p[2])))
        .collect();
    let knots = KnotVector::clamped_uniform(n_path, degree);
    let path = NurbsCurve3::new(degree, path_ctrl_pts, knots)
        .map_err(|e| PyValueError::new_err(format!("Path curve creation failed: {}", e)))?;

    let solid = SweepBuilder::sweep_wire_along_curve(&profile_wire, &path, num_sections, &tol)
        .map_err(|e| PyValueError::new_err(format!("Sweep wire failed: {}", e)))?;

    if let Some(path_str) = step_path {
        StepExporter::export_solid_to_file(&solid, path_str, "ZENITH_SWEEP_WIRE")
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
        let c = NurbsCurve3::bspline_from_points(degree, pts).map_err(PyValueError::new_err)?;
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
        return Err(PyValueError::new_err(
            "Loft solid requires at least 2 sections",
        ));
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
            let edge =
                zenith_topo::Edge::line_between(vertices[i].clone(), vertices[next_i].clone())
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

/// 外側境界と穴（内側ループ）を持つ2Dプロファイルの中空押し出しSolid生成（STEP対応）
#[pyfunction]
#[pyo3(signature = (outer_profile, inner_profiles, extrude_dir = [0.0, 0.0, 10.0], u_divisions = 4, v_divisions = 4, step_path = None))]
pub fn make_hollow_extrusion(
    outer_profile: Vec<[f64; 3]>,
    inner_profiles: Vec<Vec<[f64; 3]>>,
    extrude_dir: [f64; 3],
    u_divisions: usize,
    v_divisions: usize,
    step_path: Option<&str>,
) -> PyResult<PyMesh> {
    let n_out = outer_profile.len();
    if n_out < 3 {
        return Err(PyValueError::new_err(
            "Outer profile requires at least 3 points",
        ));
    }

    let tol = Tolerance::default();

    let make_wire = |pts: &[[f64; 3]]| -> PyResult<zenith_topo::Wire> {
        let n = pts.len();
        let vertices: Vec<zenith_topo::Vertex> = pts
            .iter()
            .map(|p| zenith_topo::Vertex::from_point(Point3::new(p[0], p[1], p[2])))
            .collect();
        let mut edges = Vec::with_capacity(n);
        for i in 0..n {
            let next_i = (i + 1) % n;
            let edge =
                zenith_topo::Edge::line_between(vertices[i].clone(), vertices[next_i].clone())
                    .map_err(|e| PyValueError::new_err(format!("Edge creation failed: {}", e)))?;
            edges.push(zenith_topo::OrientedEdge::forward(edge));
        }
        Ok(zenith_topo::Wire::new(edges))
    };

    let outer_wire = make_wire(&outer_profile)?;
    let mut inner_wires = Vec::with_capacity(inner_profiles.len());
    for hole in &inner_profiles {
        if hole.len() < 3 {
            return Err(PyValueError::new_err(
                "Each hole profile requires at least 3 points",
            ));
        }
        inner_wires.push(make_wire(hole)?);
    }

    let dir = Vec3::new(extrude_dir[0], extrude_dir[1], extrude_dir[2]);
    let solid =
        zenith_algo::ExtrudeBuilder::extrude_face_with_holes(&outer_wire, &inner_wires, dir, &tol)
            .map_err(|e| PyValueError::new_err(format!("Hollow extrusion failed: {}", e)))?;

    if let Some(path) = step_path {
        StepExporter::export_solid_to_file(&solid, path, "ZENITH_HOLLOW_EXTRUSION")
            .map_err(|e| PyValueError::new_err(format!("STEP export failed: {}", e)))?;
    }

    let params = TessellationParams {
        u_divisions,
        v_divisions,
    };
    let mesh = tessellate_solid(&solid, &params);
    Ok(PyMesh { mesh })
}

/// ドラフト（抜き勾配）角度付き押し出しSolid生成（STEP対応）
#[pyfunction]
#[pyo3(signature = (profile, extrude_dir = [0.0, 0.0, 10.0], draft_angle_deg = 5.0, u_divisions = 4, v_divisions = 4, step_path = None))]
pub fn make_draft_extrusion(
    profile: Vec<[f64; 3]>,
    extrude_dir: [f64; 3],
    draft_angle_deg: f64,
    u_divisions: usize,
    v_divisions: usize,
    step_path: Option<&str>,
) -> PyResult<PyMesh> {
    let n = profile.len();
    if n < 3 {
        return Err(PyValueError::new_err("Profile requires at least 3 points"));
    }

    let tol = Tolerance::default();
    let vertices: Vec<zenith_topo::Vertex> = profile
        .iter()
        .map(|p| zenith_topo::Vertex::from_point(Point3::new(p[0], p[1], p[2])))
        .collect();

    let mut edges = Vec::with_capacity(n);
    for i in 0..n {
        let next_i = (i + 1) % n;
        let edge = zenith_topo::Edge::line_between(vertices[i].clone(), vertices[next_i].clone())
            .map_err(|e| PyValueError::new_err(format!("Edge creation failed: {}", e)))?;
        edges.push(zenith_topo::OrientedEdge::forward(edge));
    }
    let wire = zenith_topo::Wire::new(edges);
    let dir = Vec3::new(extrude_dir[0], extrude_dir[1], extrude_dir[2]);
    let draft_rad = draft_angle_deg.to_radians();

    let solid = zenith_algo::ExtrudeBuilder::extrude_wire_with_draft(&wire, dir, draft_rad, &tol)
        .map_err(|e| PyValueError::new_err(format!("Draft extrude failed: {}", e)))?;

    if let Some(path) = step_path {
        StepExporter::export_solid_to_file(&solid, path, "ZENITH_DRAFT_EXTRUSION")
            .map_err(|e| PyValueError::new_err(format!("STEP export failed: {}", e)))?;
    }

    let params = TessellationParams {
        u_divisions,
        v_divisions,
    };
    let mesh = tessellate_solid(&solid, &params);
    Ok(PyMesh { mesh })
}

/// 閉断面ワイヤの360度回転体完全閉Solid生成（STEP対応）
#[pyfunction]
#[pyo3(signature = (profile, axis_origin = [0.0, 0.0, 0.0], axis_dir = [0.0, 0.0, 1.0], u_divisions = 8, v_divisions = 8, step_path = None))]
pub fn make_revolve_solid(
    profile: Vec<[f64; 3]>,
    axis_origin: [f64; 3],
    axis_dir: [f64; 3],
    u_divisions: usize,
    v_divisions: usize,
    step_path: Option<&str>,
) -> PyResult<PyMesh> {
    let n = profile.len();
    if n < 3 {
        return Err(PyValueError::new_err("Profile requires at least 3 points"));
    }

    let tol = Tolerance::default();
    let vertices: Vec<zenith_topo::Vertex> = profile
        .iter()
        .map(|p| zenith_topo::Vertex::from_point(Point3::new(p[0], p[1], p[2])))
        .collect();

    let mut edges = Vec::with_capacity(n);
    for i in 0..n {
        let next_i = (i + 1) % n;
        let edge = zenith_topo::Edge::line_between(vertices[i].clone(), vertices[next_i].clone())
            .map_err(|e| PyValueError::new_err(format!("Edge creation failed: {}", e)))?;
        edges.push(zenith_topo::OrientedEdge::forward(edge));
    }
    let wire = zenith_topo::Wire::new(edges);
    let orig = Point3::new(axis_origin[0], axis_origin[1], axis_origin[2]);
    let dir = Vec3::new(axis_dir[0], axis_dir[1], axis_dir[2]);

    let solid = zenith_algo::RevolveBuilder::revolve_wire_solid(&wire, orig, dir, &tol)
        .map_err(|e| PyValueError::new_err(format!("Revolve solid failed: {}", e)))?;

    if let Some(path) = step_path {
        StepExporter::export_solid_to_file(&solid, path, "ZENITH_REVOLVE_SOLID")
            .map_err(|e| PyValueError::new_err(format!("STEP export failed: {}", e)))?;
    }

    let params = TessellationParams {
        u_divisions,
        v_divisions,
    };
    let mesh = tessellate_solid(&solid, &params);
    Ok(PyMesh { mesh })
}

/// 閉断面ワイヤの部分角度（0 < angle_deg <= 360）回転体完全閉Solid生成（端面キャップ・STEP対応）
#[pyfunction]
#[pyo3(signature = (profile, axis_origin = [0.0, 0.0, 0.0], axis_dir = [0.0, 0.0, 1.0], angle_deg = 90.0, u_divisions = 8, v_divisions = 8, step_path = None))]
pub fn make_partial_revolve_solid(
    profile: Vec<[f64; 3]>,
    axis_origin: [f64; 3],
    axis_dir: [f64; 3],
    angle_deg: f64,
    u_divisions: usize,
    v_divisions: usize,
    step_path: Option<&str>,
) -> PyResult<PyMesh> {
    let n = profile.len();
    if n < 3 {
        return Err(PyValueError::new_err("Profile requires at least 3 points"));
    }

    let tol = Tolerance::default();
    let vertices: Vec<zenith_topo::Vertex> = profile
        .iter()
        .map(|p| zenith_topo::Vertex::from_point(Point3::new(p[0], p[1], p[2])))
        .collect();

    let mut edges = Vec::with_capacity(n);
    for i in 0..n {
        let next_i = (i + 1) % n;
        let edge = zenith_topo::Edge::line_between(vertices[i].clone(), vertices[next_i].clone())
            .map_err(|e| PyValueError::new_err(format!("Edge creation failed: {}", e)))?;
        edges.push(zenith_topo::OrientedEdge::forward(edge));
    }
    let wire = zenith_topo::Wire::new(edges);
    let orig = Point3::new(axis_origin[0], axis_origin[1], axis_origin[2]);
    let dir = Vec3::new(axis_dir[0], axis_dir[1], axis_dir[2]);
    let angle_rad = angle_deg.to_radians();

    let solid =
        zenith_algo::RevolveBuilder::revolve_wire_partial_solid(&wire, orig, dir, angle_rad, &tol)
            .map_err(|e| PyValueError::new_err(format!("Partial revolve solid failed: {}", e)))?;

    if let Some(path) = step_path {
        StepExporter::export_solid_to_file(&solid, path, "ZENITH_PARTIAL_REVOLVE_SOLID")
            .map_err(|e| PyValueError::new_err(format!("STEP export failed: {}", e)))?;
    }

    let params = TessellationParams {
        u_divisions,
        v_divisions,
    };
    let mesh = tessellate_solid(&solid, &params);
    Ok(PyMesh { mesh })
}

/// 閉断面ワイヤの3D螺旋（ヘリカル）スイープSolid生成（スプリング・ネジ山・STEP対応）
#[pyfunction]
#[pyo3(signature = (profile, radius = 15.0, pitch = 10.0, turns = 2.0, axis_origin = [0.0, 0.0, 0.0], axis_dir = [0.0, 0.0, 1.0], num_sections = 32, u_divisions = 8, v_divisions = 8, step_path = None))]
pub fn make_helix_solid(
    profile: Vec<[f64; 3]>,
    radius: f64,
    pitch: f64,
    turns: f64,
    axis_origin: [f64; 3],
    axis_dir: [f64; 3],
    num_sections: usize,
    u_divisions: usize,
    v_divisions: usize,
    step_path: Option<&str>,
) -> PyResult<PyMesh> {
    let n = profile.len();
    if n < 3 {
        return Err(PyValueError::new_err("Profile requires at least 3 points"));
    }

    let tol = Tolerance::default();
    let vertices: Vec<zenith_topo::Vertex> = profile
        .iter()
        .map(|p| zenith_topo::Vertex::from_point(Point3::new(p[0], p[1], p[2])))
        .collect();

    let mut edges = Vec::with_capacity(n);
    for i in 0..n {
        let next_i = (i + 1) % n;
        let edge = zenith_topo::Edge::line_between(vertices[i].clone(), vertices[next_i].clone())
            .map_err(|e| PyValueError::new_err(format!("Edge creation failed: {}", e)))?;
        edges.push(zenith_topo::OrientedEdge::forward(edge));
    }
    let wire = zenith_topo::Wire::new(edges);
    let orig = Point3::new(axis_origin[0], axis_origin[1], axis_origin[2]);
    let dir = Vec3::new(axis_dir[0], axis_dir[1], axis_dir[2]);

    let solid = zenith_algo::HelixBuilder::sweep_wire_along_helix(
        &wire,
        radius,
        pitch,
        turns,
        orig,
        dir,
        num_sections,
        &tol,
    )
    .map_err(|e| PyValueError::new_err(format!("Helix sweep failed: {}", e)))?;

    if let Some(path) = step_path {
        StepExporter::export_solid_to_file(&solid, path, "ZENITH_HELIX_SOLID")
            .map_err(|e| PyValueError::new_err(format!("STEP export failed: {}", e)))?;
    }

    let params = TessellationParams {
        u_divisions,
        v_divisions,
    };
    let mesh = tessellate_solid(&solid, &params);
    Ok(PyMesh { mesh })
}

/// 任意対称平面に対する直方体のミラー反転複製Solid生成（STEP対応）
#[pyfunction]
#[pyo3(signature = (dx, dy, dz, plane_origin = [0.0, 0.0, 0.0], plane_normal = [1.0, 0.0, 0.0], u_divisions = 4, v_divisions = 4, step_path = None))]
pub fn make_mirror_box(
    dx: f64,
    dy: f64,
    dz: f64,
    plane_origin: [f64; 3],
    plane_normal: [f64; 3],
    u_divisions: usize,
    v_divisions: usize,
    step_path: Option<&str>,
) -> PyResult<PyMesh> {
    let tol = Tolerance::default();
    let base_box = zenith_algo::PrimitiveBuilder::make_box(dx, dy, dz)
        .map_err(|e| PyValueError::new_err(format!("Box creation failed: {}", e)))?;

    let orig = Point3::new(plane_origin[0], plane_origin[1], plane_origin[2]);
    let norm = Vec3::new(plane_normal[0], plane_normal[1], plane_normal[2]);

    let mirrored = zenith_algo::MirrorBuilder::mirror_solid(&base_box, orig, norm, &tol)
        .map_err(|e| PyValueError::new_err(format!("Mirror solid failed: {}", e)))?;

    if let Some(path) = step_path {
        StepExporter::export_solid_to_file(&mirrored, path, "ZENITH_MIRRORED_BOX")
            .map_err(|e| PyValueError::new_err(format!("STEP export failed: {}", e)))?;
    }

    let params = TessellationParams {
        u_divisions,
        v_divisions,
    };
    let mesh = tessellate_solid(&mirrored, &params);
    Ok(PyMesh { mesh })
}

/// 原本ソリッドとミラー反転ソリッドの左右対称ペア（Compound Solid Pair）の生成＆STEP出力
#[pyfunction]
#[pyo3(signature = (dx = 30.0, dy = 50.0, dz = 20.0, offset_x = 10.0, chamfer_dist = 6.0, plane_origin = [0.0, 0.0, 0.0], plane_normal = [1.0, 0.0, 0.0], u_divisions = 4, v_divisions = 4, step_path = None))]
pub fn make_mirror_compound_casing(
    dx: f64,
    dy: f64,
    dz: f64,
    offset_x: f64,
    chamfer_dist: f64,
    plane_origin: [f64; 3],
    plane_normal: [f64; 3],
    u_divisions: usize,
    v_divisions: usize,
    step_path: Option<&str>,
) -> PyResult<PyMesh> {
    let tol = Tolerance::default();

    // 1. 原本ソリッドの生成: +X 側に offset_x 離れた位置に配置し、単一エッジに面取りを施した非対称ケーシング
    let base_box =
        zenith_algo::DirectModeling::chamfer_box_single_edge(dx, dy, dz, 0, chamfer_dist)
            .map_err(|e| PyValueError::new_err(format!("Chamfer box failed: {}", e)))?;

    let base_solid =
        zenith_algo::BrepTransform::translate_solid(&base_box, Vec3::new(offset_x, 0.0, 0.0));

    let orig = Point3::new(plane_origin[0], plane_origin[1], plane_origin[2]);
    let norm = Vec3::new(plane_normal[0], plane_normal[1], plane_normal[2]);

    // 2. ミラー反転ソリッドの生成
    let mirrored_solid = zenith_algo::MirrorBuilder::mirror_solid(&base_solid, orig, norm, &tol)
        .map_err(|e| PyValueError::new_err(format!("Mirror solid failed: {}", e)))?;

    // 3. 原本＋ミラー反転の Compound Shape として STEP 出力
    if let Some(path) = step_path {
        let compound =
            zenith_topo::Shape::compound_solids(vec![base_solid.clone(), mirrored_solid.clone()]);
        StepExporter::export_shape_to_file(&compound, path, "ZENITH_MIRRORED_CASING_PAIR")
            .map_err(|e| PyValueError::new_err(format!("STEP export failed: {}", e)))?;
    }

    let params = TessellationParams {
        u_divisions,
        v_divisions,
    };
    let mesh1 = tessellate_solid(&base_solid, &params);
    let mesh2 = tessellate_solid(&mirrored_solid, &params);

    // 2つのメッシュを合体してプレビュー返却
    let mut combined_pos = mesh1.positions.clone();
    let mut combined_normals = mesh1.normals.clone();
    let mut combined_uvs = mesh1.uvs.clone();
    let mut combined_indices = mesh1.indices.clone();
    let n_v1 = mesh1.positions.len() as u32;

    combined_pos.extend_from_slice(&mesh2.positions);
    combined_normals.extend_from_slice(&mesh2.normals);
    combined_uvs.extend_from_slice(&mesh2.uvs);
    for tri in &mesh2.indices {
        combined_indices.push([tri[0] + n_v1, tri[1] + n_v1, tri[2] + n_v1]);
    }

    let mesh = zenith_tess::TriangleMesh {
        positions: combined_pos,
        normals: combined_normals,
        uvs: combined_uvs,
        indices: combined_indices,
    };

    Ok(PyMesh { mesh })
}

/// 直方体の両端面（底面・天面）を開口した角パイプ中空ソリッドの生成（STEP対応）

#[pyfunction]
#[pyo3(signature = (dx, dy, dz, thickness, u_divisions = 4, v_divisions = 4, step_path = None))]
pub fn make_through_hollow_box(
    dx: f64,
    dy: f64,
    dz: f64,
    thickness: f64,
    u_divisions: usize,
    v_divisions: usize,
    step_path: Option<&str>,
) -> PyResult<PyMesh> {
    let solid = zenith_algo::ShellBuilder::make_through_hollow_box(dx, dy, dz, thickness)
        .map_err(|e| PyValueError::new_err(format!("Through tube failed: {}", e)))?;

    if let Some(path) = step_path {
        StepExporter::export_solid_to_file(&solid, path, "ZENITH_THROUGH_TUBE")
            .map_err(|e| PyValueError::new_err(format!("STEP export failed: {}", e)))?;
    }

    let params = TessellationParams {
        u_divisions,
        v_divisions,
    };
    let mesh = tessellate_solid(&solid, &params);
    Ok(PyMesh { mesh })
}

/// ガイドレール曲線群（Guide Curves）に沿った閉断面ワイヤ群のロフト完全閉Solid生成（STEP対応）
#[pyfunction]
#[pyo3(signature = (sections, guide_curves, degree_v = 2, u_divisions = 8, v_divisions = 8, step_path = None))]
pub fn make_guided_loft_solid(
    sections: Vec<Vec<[f64; 3]>>,
    guide_curves: Vec<Vec<[f64; 3]>>,
    degree_v: usize,
    u_divisions: usize,
    v_divisions: usize,
    step_path: Option<&str>,
) -> PyResult<PyMesh> {
    if sections.len() < 2 {
        return Err(PyValueError::new_err("Loft requires at least 2 sections"));
    }
    if guide_curves.is_empty() {
        return Err(PyValueError::new_err(
            "Guided loft requires at least 1 guide curve",
        ));
    }

    let tol = Tolerance::default();
    let make_wire = |pts: &[[f64; 3]]| -> PyResult<zenith_topo::Wire> {
        let n = pts.len();
        if n < 3 {
            return Err(PyValueError::new_err("Profile requires at least 3 points"));
        }
        let vertices: Vec<zenith_topo::Vertex> = pts
            .iter()
            .map(|p| zenith_topo::Vertex::from_point(Point3::new(p[0], p[1], p[2])))
            .collect();
        let mut edges = Vec::with_capacity(n);
        for i in 0..n {
            let next_i = (i + 1) % n;
            let edge =
                zenith_topo::Edge::line_between(vertices[i].clone(), vertices[next_i].clone())
                    .map_err(|e| PyValueError::new_err(format!("Edge creation failed: {}", e)))?;
            edges.push(zenith_topo::OrientedEdge::forward(edge));
        }
        Ok(zenith_topo::Wire::new(edges))
    };

    let mut section_wires = Vec::with_capacity(sections.len());
    for sec in &sections {
        section_wires.push(make_wire(sec)?);
    }

    let mut guides = Vec::with_capacity(guide_curves.len());
    for g_pts in &guide_curves {
        let n = g_pts.len();
        if n < 2 {
            return Err(PyValueError::new_err(
                "Guide curve requires at least 2 points",
            ));
        }
        let degree = (n - 1).min(3);
        let cps = g_pts
            .iter()
            .map(|p| zenith_geom::ControlPoint3::unweighted(Point3::new(p[0], p[1], p[2])))
            .collect();
        let knots = zenith_geom::KnotVector::clamped_uniform(n, degree);
        let curve = zenith_geom::NurbsCurve3::new(degree, cps, knots)
            .map_err(|e| PyValueError::new_err(format!("Guide curve creation failed: {}", e)))?;
        guides.push(curve);
    }

    let solid =
        zenith_algo::LoftBuilder::loft_solid_guided(&section_wires, &guides, degree_v, &tol)
            .map_err(|e| PyValueError::new_err(format!("Guided loft failed: {}", e)))?;

    if let Some(path) = step_path {
        StepExporter::export_solid_to_file(&solid, path, "ZENITH_GUIDED_LOFT")
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

/// 3Dポリライン（折れ線・角丸めフィレット付き）に沿った円形パイプソリッドの生成（STEP対応）
#[pyfunction]
#[pyo3(signature = (path_points, pipe_radius = 4.0, corner_radius = 0.0, u_divisions = 16, v_divisions = 16, step_path = None))]
pub fn make_polyline_pipe(
    path_points: Vec<[f64; 3]>,
    pipe_radius: f64,
    corner_radius: f64,
    u_divisions: usize,
    v_divisions: usize,
    step_path: Option<&str>,
) -> PyResult<PyMesh> {
    let tol = Tolerance::default();
    let pts: Vec<_> = path_points
        .into_iter()
        .map(|p| Point3::new(p[0], p[1], p[2]))
        .collect();

    let solid =
        zenith_algo::PolylineBuilder::sweep_pipe_polyline(&pts, pipe_radius, corner_radius, &tol)
            .map_err(|e| PyValueError::new_err(format!("Polyline pipe failed: {}", e)))?;

    if let Some(path) = step_path {
        StepExporter::export_solid_to_file(&solid, path, "ZENITH_POLYLINE_PIPE")
            .map_err(|e| PyValueError::new_err(format!("STEP export failed: {}", e)))?;
    }

    let params = TessellationParams {
        u_divisions,
        v_divisions,
    };
    let mesh = tessellate_solid(&solid, &params);
    Ok(PyMesh { mesh })
}

/// 3Dポリライン（折れ線・角丸めフィレット付き）に沿った任意閉断面スイープソリッドの生成（STEP対応）
#[pyfunction]
#[pyo3(signature = (profile_points, path_points, corner_radius = 0.0, u_divisions = 12, v_divisions = 12, step_path = None))]
pub fn make_polyline_sweep(
    profile_points: Vec<[f64; 3]>,
    path_points: Vec<[f64; 3]>,
    corner_radius: f64,
    u_divisions: usize,
    v_divisions: usize,
    step_path: Option<&str>,
) -> PyResult<PyMesh> {
    let tol = Tolerance::default();
    let prof: Vec<_> = profile_points
        .into_iter()
        .map(|p| Point3::new(p[0], p[1], p[2]))
        .collect();
    let path: Vec<_> = path_points
        .into_iter()
        .map(|p| Point3::new(p[0], p[1], p[2]))
        .collect();

    let solid =
        zenith_algo::PolylineBuilder::sweep_wire_polyline(&prof, &path, corner_radius, &tol)
            .map_err(|e| PyValueError::new_err(format!("Polyline sweep failed: {}", e)))?;

    if let Some(path) = step_path {
        StepExporter::export_solid_to_file(&solid, path, "ZENITH_POLYLINE_SWEEP")
            .map_err(|e| PyValueError::new_err(format!("STEP export failed: {}", e)))?;
    }

    let params = TessellationParams {
        u_divisions,
        v_divisions,
    };
    let mesh = tessellate_solid(&solid, &params);
    Ok(PyMesh { mesh })
}

/// 天面が開口した均一肉厚の薄肉直方体容器（Open-Top Box）の生成（STEP対応）
#[pyfunction]
#[pyo3(signature = (dx, dy, dz, thickness, u_divisions = 8, v_divisions = 8, step_path = None))]
pub fn make_open_box(
    dx: f64,
    dy: f64,
    dz: f64,
    thickness: f64,
    u_divisions: usize,
    v_divisions: usize,
    step_path: Option<&str>,
) -> PyResult<PyMesh> {
    let solid = zenith_algo::ShellingBuilder::make_open_box(dx, dy, dz, thickness)
        .map_err(|e| PyValueError::new_err(format!("Open box creation failed: {}", e)))?;

    if let Some(path) = step_path {
        StepExporter::export_solid_to_file(&solid, path, "ZENITH_OPEN_BOX")
            .map_err(|e| PyValueError::new_err(format!("STEP export failed: {}", e)))?;
    }

    let params = TessellationParams {
        u_divisions,
        v_divisions,
    };
    let mesh = tessellate_solid(&solid, &params);
    Ok(PyMesh { mesh })
}

/// 直方体を任意平面で切断し、断面積、外周長、および断面多角形頂点列を算出
#[pyfunction]
#[pyo3(signature = (dx, dy, dz, plane_origin, plane_normal))]
pub fn slice_box_by_plane(
    dx: f64,
    dy: f64,
    dz: f64,
    plane_origin: [f64; 3],
    plane_normal: [f64; 3],
) -> PyResult<(f64, f64, Vec<Vec<[f64; 3]>>)> {
    let tol = Tolerance::default();
    let solid = zenith_algo::PrimitiveBuilder::make_box(dx, dy, dz)
        .map_err(|e| PyValueError::new_err(format!("Box creation failed: {}", e)))?;

    let p_orig = Point3::new(plane_origin[0], plane_origin[1], plane_origin[2]);
    let p_norm = Vec3::new(plane_normal[0], plane_normal[1], plane_normal[2]);

    let res = zenith_algo::SectionSlicer::slice_solid(&solid, p_orig, p_norm, &tol)
        .map_err(|e| PyValueError::new_err(format!("Section slicing failed: {}", e)))?;

    let mut loops = Vec::new();
    for wire in res.section_wires {
        let mut loop_pts = Vec::new();
        for oe in wire.edges {
            let p = oe.start_vertex().point;
            loop_pts.push([p.x, p.y, p.z]);
        }
        loops.push(loop_pts);
    }

    Ok((res.total_area, res.total_perimeter, loops))
}

/// 直方体の高精度物理特性・重心・慣性モーメントを計算
#[pyfunction]
#[pyo3(signature = (dx, dy, dz, _density = 1.0))]
pub fn compute_box_mass_properties(
    dx: f64,
    dy: f64,
    dz: f64,
    _density: f64,
) -> PyResult<(f64, f64, [f64; 3], [f64; 3])> {
    let solid = zenith_algo::PrimitiveBuilder::make_box(dx, dy, dz)
        .map_err(|e| PyValueError::new_err(format!("Box creation failed: {}", e)))?;

    let params = TessellationParams {
        u_divisions: 8,
        v_divisions: 8,
    };
    let props = zenith_algo::MassCalculator::compute_from_brep(&solid, &params);

    let center = [
        props.center_of_mass.x,
        props.center_of_mass.y,
        props.center_of_mass.z,
    ];
    let inertia = [
        props.inertia_diagonal.x,
        props.inertia_diagonal.y,
        props.inertia_diagonal.z,
    ];

    Ok((props.volume, props.surface_area, center, inertia))
}

/// 2つの直方体間の干渉・クリアランスを判定
#[pyfunction]
#[pyo3(signature = (dx1, dy1, dz1, offset1, dx2, dy2, dz2, offset2))]
pub fn check_boxes_interference(
    dx1: f64,
    dy1: f64,
    dz1: f64,
    offset1: [f64; 3],
    dx2: f64,
    dy2: f64,
    dz2: f64,
    offset2: [f64; 3],
) -> PyResult<(String, f64, f64, String)> {
    let tol = Tolerance::default();
    let box1 = zenith_algo::PrimitiveBuilder::make_box(dx1, dy1, dz1)
        .map_err(|e| PyValueError::new_err(format!("Box1 creation failed: {}", e)))?;
    let box2 = zenith_algo::PrimitiveBuilder::make_box(dx2, dy2, dz2)
        .map_err(|e| PyValueError::new_err(format!("Box2 creation failed: {}", e)))?;

    // 移動トランスフォーム適用
    let s1 = zenith_algo::BrepTransform::translate_solid(
        &box1,
        Vec3::new(offset1[0], offset1[1], offset1[2]),
    );
    let s2 = zenith_algo::BrepTransform::translate_solid(
        &box2,
        Vec3::new(offset2[0], offset2[1], offset2[2]),
    );

    let report = zenith_algo::InterferenceChecker::check(&s1, &s2, &tol);

    let status_str = match report.status {
        zenith_algo::ClashStatus::Clearance => "Clearance",
        zenith_algo::ClashStatus::Touching => "Touching",
        zenith_algo::ClashStatus::Clash => "Clash",
    };

    Ok((
        status_str.to_string(),
        report.min_distance,
        report.overlap_volume,
        report.message,
    ))
}

/// インボリュート平歯車（Spur Gear）B-Rep Solid生成（STEP出力対応）
///
/// 歯面は基礎円のインボリュート。`bore_radius` は**穴を開けません**——歯底
/// 半径の下限に効くだけです。軸穴が要るなら円柱との差で開けてください。
#[pyfunction]
#[pyo3(signature = (module = 2.0, teeth = 18, pressure_angle = 20.0, thickness = 10.0, bore_radius = 5.0, u_divisions = 8, v_divisions = 8, step_path = None))]
pub fn make_spur_gear(
    module: f64,
    teeth: usize,
    pressure_angle: f64,
    thickness: f64,
    bore_radius: f64,
    u_divisions: usize,
    v_divisions: usize,
    step_path: Option<&str>,
) -> PyResult<PyMesh> {
    let solid = zenith_algo::gear::GearBuilder::make_spur_gear(
        module,
        teeth,
        pressure_angle,
        thickness,
        bore_radius,
    )
    .map_err(|e| PyValueError::new_err(format!("Spur gear creation failed: {}", e)))?;

    if let Some(path) = step_path {
        StepExporter::export_solid_to_file(&solid, path, "ZENITH_SPUR_GEAR")
            .map_err(|e| PyValueError::new_err(format!("STEP export failed: {}", e)))?;
    }

    let params = TessellationParams {
        u_divisions,
        v_divisions,
    };
    let mesh = tessellate_solid(&solid, &params);
    Ok(PyMesh { mesh })
}

/// 直交直方体同士の厳密 B-Rep ブーリアン演算（Union/Difference/Intersection・STEP出力対応）
#[pyfunction]
#[pyo3(signature = (dx1, dy1, dz1, offset1, dx2, dy2, dz2, offset2, op_type = 1, u_divisions = 8, v_divisions = 8, step_path = None))]
pub fn make_exact_box_boolean(
    dx1: f64,
    dy1: f64,
    dz1: f64,
    offset1: [f64; 3],
    dx2: f64,
    dy2: f64,
    dz2: f64,
    offset2: [f64; 3],
    op_type: u8,
    u_divisions: usize,
    v_divisions: usize,
    step_path: Option<&str>,
) -> PyResult<PyMesh> {
    let tol = Tolerance::default();
    let box1 = zenith_algo::PrimitiveBuilder::make_box(dx1, dy1, dz1)
        .map_err(|e| PyValueError::new_err(format!("Box1 creation failed: {}", e)))?;
    let box2 = zenith_algo::PrimitiveBuilder::make_box(dx2, dy2, dz2)
        .map_err(|e| PyValueError::new_err(format!("Box2 creation failed: {}", e)))?;

    let s1 = zenith_algo::BrepTransform::translate_solid(
        &box1,
        Vec3::new(offset1[0], offset1[1], offset1[2]),
    );
    let s2 = zenith_algo::BrepTransform::translate_solid(
        &box2,
        Vec3::new(offset2[0], offset2[1], offset2[2]),
    );

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

    let result_solid = zenith_algo::BooleanEngine::boolean_solids_exact(&s1, &s2, op, &tol)
        .map_err(|e| PyValueError::new_err(format!("Exact B-Rep boolean failed: {}", e)))?;

    if let Some(path) = step_path {
        StepExporter::export_solid_to_file(&result_solid, path, "ZENITH_EXACT_BOOLEAN")
            .map_err(|e| PyValueError::new_err(format!("STEP export failed: {}", e)))?;
    }

    let params = TessellationParams {
        u_divisions,
        v_divisions,
    };
    let mesh = tessellate_solid(&result_solid, &params);
    Ok(PyMesh { mesh })
}

/// 直方体を円柱でくり抜く（貫通穴・止まり穴）厳密B-Repブーリアン。
///
/// `axis` は円柱の向き。既定の +Z 以外を指定すると、その向きに回してから
/// `drill_offset` だけ平行移動する。円柱が立体を貫通すれば貫通穴に、内部で
/// 止まれば止まり穴になる。演算が対応範囲外なら、もっともらしい形ではなく
/// エラーを返す。
#[pyfunction]
#[pyo3(signature = (
    dx, dy, dz, box_offset,
    radius, height, drill_offset,
    axis = [0.0, 0.0, 1.0],
    op_type = 1,
    u_divisions = 32,
    v_divisions = 32,
    step_path = None
))]
#[allow(clippy::too_many_arguments)]
pub fn make_exact_drill_boolean(
    dx: f64,
    dy: f64,
    dz: f64,
    box_offset: [f64; 3],
    radius: f64,
    height: f64,
    drill_offset: [f64; 3],
    axis: [f64; 3],
    op_type: u8,
    u_divisions: usize,
    v_divisions: usize,
    step_path: Option<&str>,
) -> PyResult<PyMesh> {
    let tol = Tolerance::default();

    let block = zenith_algo::PrimitiveBuilder::make_box(dx, dy, dz)
        .map_err(|e| PyValueError::new_err(format!("Block creation failed: {}", e)))?;
    let block = zenith_algo::BrepTransform::translate_solid(
        &block,
        Vec3::new(box_offset[0], box_offset[1], box_offset[2]),
    );

    let drill = zenith_algo::PrimitiveBuilder::make_cylinder(radius, height)
        .map_err(|e| PyValueError::new_err(format!("Drill creation failed: {}", e)))?;

    // 既定の +Z から指定軸へ回す。
    let direction = Vec3::new(axis[0], axis[1], axis[2]);
    let drill = match direction.try_normalize(1e-12) {
        None => return Err(PyValueError::new_err("Drill axis must not be zero")),
        Some(unit) => {
            let z = Vec3::new(0.0, 0.0, 1.0);
            let dot = unit.dot(&z).clamp(-1.0, 1.0);
            if dot > 1.0 - 1e-12 {
                drill
            } else if dot < -1.0 + 1e-12 {
                let flip = zenith_math::Transform3::from_axis_angle(
                    &Vec3::new(1.0, 0.0, 0.0),
                    std::f64::consts::PI,
                );
                zenith_algo::BrepTransform::transform_solid(&drill, &flip).map_err(|e| {
                    PyValueError::new_err(format!("Drill orientation failed: {}", e))
                })?
            } else {
                let rotation_axis = z.cross(&unit);
                let transform =
                    zenith_math::Transform3::from_axis_angle(&rotation_axis, dot.acos());
                zenith_algo::BrepTransform::transform_solid(&drill, &transform).map_err(|e| {
                    PyValueError::new_err(format!("Drill orientation failed: {}", e))
                })?
            }
        }
    };
    let drill = zenith_algo::BrepTransform::translate_solid(
        &drill,
        Vec3::new(drill_offset[0], drill_offset[1], drill_offset[2]),
    );

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

    let result = zenith_algo::BooleanEngine::boolean_solids_exact_result(&block, &drill, op, &tol)
        .map_err(|e| PyValueError::new_err(format!("Exact B-Rep drill boolean failed: {}", e)))?;

    if let Some(path) = step_path {
        StepExporter::export_solids_to_file(&result.solids, path, "ZENITH_DRILL_BOOLEAN")
            .map_err(|e| PyValueError::new_err(format!("STEP export failed: {}", e)))?;
    }

    let params = TessellationParams {
        u_divisions,
        v_divisions,
    };
    Ok(PyMesh {
        mesh: result.tessellate(&params),
    })
}

/// 丸線ヘリカルスプリング（Round-Wire Spring）ソリッドの生成（STEP対応）
#[pyfunction]
#[pyo3(signature = (radius = 10.0, pitch = 8.0, turns = 3.0, wire_radius = 1.5, u_divisions = 16, v_divisions = 16, step_path = None))]
pub fn make_round_wire_spring(
    radius: f64,
    pitch: f64,
    turns: f64,
    wire_radius: f64,
    u_divisions: usize,
    v_divisions: usize,
    step_path: Option<&str>,
) -> PyResult<PyMesh> {
    let tol = Tolerance::default();
    let solid = zenith_algo::HelixBuilder::make_round_wire_spring(
        radius,
        pitch,
        turns,
        wire_radius,
        Point3::origin(),
        Vec3::z(),
        &tol,
    )
    .map_err(|e| PyValueError::new_err(format!("Round wire spring generation failed: {}", e)))?;

    if let Some(path) = step_path {
        StepExporter::export_solid_to_file(&solid, path, "ZENITH_ROUND_WIRE_SPRING")
            .map_err(|e| PyValueError::new_err(format!("STEP export failed: {}", e)))?;
    }

    let params = TessellationParams {
        u_divisions,
        v_divisions,
    };
    let mesh = tessellate_solid(&solid, &params);
    Ok(PyMesh { mesh })
}

/// ボルト頭沈めザグリ穴（Counterbore Hole）直方体ソリッドの生成（STEP対応）
#[pyfunction]
#[pyo3(signature = (dx = 50.0, dy = 50.0, dz = 25.0, hole_radius = 4.0, cb_radius = 8.0, cb_depth = 5.0, u_divisions = 16, v_divisions = 16, step_path = None))]
pub fn make_counterbore_hole_box(
    dx: f64,
    dy: f64,
    dz: f64,
    hole_radius: f64,
    cb_radius: f64,
    cb_depth: f64,
    u_divisions: usize,
    v_divisions: usize,
    step_path: Option<&str>,
) -> PyResult<PyMesh> {
    let solid = zenith_algo::HoleBuilder::make_counterbore_hole_box(
        dx,
        dy,
        dz,
        hole_radius,
        cb_radius,
        cb_depth,
    )
    .map_err(|e| PyValueError::new_err(format!("Counterbore box generation failed: {}", e)))?;

    if let Some(path) = step_path {
        StepExporter::export_solid_to_file(&solid, path, "ZENITH_COUNTERBORE_BOX")
            .map_err(|e| PyValueError::new_err(format!("STEP export failed: {}", e)))?;
    }

    let params = TessellationParams {
        u_divisions,
        v_divisions,
    };
    let mesh = tessellate_solid(&solid, &params);
    Ok(PyMesh { mesh })
}

/// 正六角ナット（Hex Nut）ソリッドの生成（STEP対応）
#[pyfunction]
#[pyo3(signature = (across_flats = 16.0, hole_radius = 4.25, thickness = 8.0, u_divisions = 16, v_divisions = 16, step_path = None))]
pub fn make_hex_nut(
    across_flats: f64,
    hole_radius: f64,
    thickness: f64,
    u_divisions: usize,
    v_divisions: usize,
    step_path: Option<&str>,
) -> PyResult<PyMesh> {
    let solid = zenith_algo::HoleBuilder::make_hex_nut(across_flats, hole_radius, thickness)
        .map_err(|e| PyValueError::new_err(format!("Hex nut generation failed: {}", e)))?;

    if let Some(path) = step_path {
        StepExporter::export_solid_to_file(&solid, path, "ZENITH_HEX_NUT")
            .map_err(|e| PyValueError::new_err(format!("STEP export failed: {}", e)))?;
    }

    let params = TessellationParams {
        u_divisions,
        v_divisions,
    };
    let mesh = tessellate_solid(&solid, &params);
    Ok(PyMesh { mesh })
}

/// 2つの直方体間のハイブリッド厳密干渉解析（B-Rep積計算による干渉体積算出）
#[pyfunction]
#[pyo3(signature = (min_a, max_a, min_b, max_b))]
pub fn check_exact_boxes_interference(
    min_a: [f64; 3],
    max_a: [f64; 3],
    min_b: [f64; 3],
    max_b: [f64; 3],
) -> PyResult<(String, f64)> {
    let box_a = zenith_algo::PrimitiveBuilder::make_box(
        max_a[0] - min_a[0],
        max_a[1] - min_a[1],
        max_a[2] - min_a[2],
    )
    .map_err(|e| PyValueError::new_err(format!("Box A creation failed: {}", e)))?;
    let box_a = zenith_algo::BrepTransform::translate_solid(
        &box_a,
        Vec3::new(min_a[0], min_a[1], min_a[2]),
    );

    let box_b = zenith_algo::PrimitiveBuilder::make_box(
        max_b[0] - min_b[0],
        max_b[1] - min_b[1],
        max_b[2] - min_b[2],
    )
    .map_err(|e| PyValueError::new_err(format!("Box B creation failed: {}", e)))?;
    let box_b = zenith_algo::BrepTransform::translate_solid(
        &box_b,
        Vec3::new(min_b[0], min_b[1], min_b[2]),
    );

    let tol = Tolerance::default();
    let (report, _exact_solid) =
        zenith_algo::InterferenceChecker::check_exact(&box_a, &box_b, &tol);

    let status_str = match report.status {
        zenith_algo::ClashStatus::Clearance => "Clearance",
        zenith_algo::ClashStatus::Touching => "Touching",
        zenith_algo::ClashStatus::Clash => "Clash",
    };

    Ok((status_str.to_string(), report.overlap_volume))
}

/// 六角ボルト（Hex Bolt）ソリッドの生成（STEP対応）
#[pyfunction]
#[pyo3(signature = (across_flats = 16.0, head_thickness = 6.4, shank_radius = 5.0, shank_length = 30.0, u_divisions = 16, v_divisions = 16, step_path = None))]
pub fn make_hex_bolt(
    across_flats: f64,
    head_thickness: f64,
    shank_radius: f64,
    shank_length: f64,
    u_divisions: usize,
    v_divisions: usize,
    step_path: Option<&str>,
) -> PyResult<PyMesh> {
    let solid = zenith_algo::BoltBuilder::make_hex_bolt(
        across_flats,
        head_thickness,
        shank_radius,
        shank_length,
    )
    .map_err(|e| PyValueError::new_err(format!("Hex bolt generation failed: {}", e)))?;

    if let Some(path) = step_path {
        StepExporter::export_solid_to_file(&solid, path, "ZENITH_HEX_BOLT")
            .map_err(|e| PyValueError::new_err(format!("STEP export failed: {}", e)))?;
    }

    let params = TessellationParams {
        u_divisions,
        v_divisions,
    };
    let mesh = tessellate_solid(&solid, &params);
    Ok(PyMesh { mesh })
}

/// 段付きシャフト（Stepped Shaft）ソリッドの生成（STEP対応）
#[pyfunction]
#[pyo3(signature = (sections, u_divisions = 16, v_divisions = 16, step_path = None))]
pub fn make_stepped_shaft(
    sections: Vec<(f64, f64)>,
    u_divisions: usize,
    v_divisions: usize,
    step_path: Option<&str>,
) -> PyResult<PyMesh> {
    let solid = zenith_algo::ShaftBuilder::make_stepped_shaft(&sections)
        .map_err(|e| PyValueError::new_err(format!("Stepped shaft generation failed: {}", e)))?;

    if let Some(path) = step_path {
        StepExporter::export_solid_to_file(&solid, path, "ZENITH_STEPPED_SHAFT")
            .map_err(|e| PyValueError::new_err(format!("STEP export failed: {}", e)))?;
    }

    let params = TessellationParams {
        u_divisions,
        v_divisions,
    };
    let mesh = tessellate_solid(&solid, &params);
    Ok(PyMesh { mesh })
}

/// キー溝（Keyway）付き軸ソリッドの生成（STEP対応）
#[pyfunction]
#[pyo3(signature = (radius = 12.0, length = 50.0, key_width = 6.0, key_depth = 3.5, key_length = 20.0, key_z_pos = 15.0, u_divisions = 16, v_divisions = 16, step_path = None))]
pub fn make_shaft_with_keyway(
    radius: f64,
    length: f64,
    key_width: f64,
    key_depth: f64,
    key_length: f64,
    key_z_pos: f64,
    u_divisions: usize,
    v_divisions: usize,
    step_path: Option<&str>,
) -> PyResult<PyMesh> {
    let base_shaft = zenith_algo::ShaftBuilder::make_stepped_shaft(&[(radius, length)])
        .map_err(|e| PyValueError::new_err(format!("Base shaft generation failed: {}", e)))?;

    let solid = zenith_algo::ShaftBuilder::make_shaft_with_keyway(
        &base_shaft,
        radius,
        key_width,
        key_depth,
        key_length,
        key_z_pos,
    )
    .map_err(|e| PyValueError::new_err(format!("Keyway shaft generation failed: {}", e)))?;

    if let Some(path) = step_path {
        StepExporter::export_solid_to_file(&solid, path, "ZENITH_KEYWAY_SHAFT")
            .map_err(|e| PyValueError::new_err(format!("STEP export failed: {}", e)))?;
    }

    let params = TessellationParams {
        u_divisions,
        v_divisions,
    };
    let mesh = tessellate_solid(&solid, &params);
    Ok(PyMesh { mesh })
}

/// 2つの直方体間の表面最短距離探索
#[pyfunction]
#[pyo3(signature = (min_a, max_a, min_b, max_b))]
pub fn compute_boxes_min_distance(
    min_a: [f64; 3],
    max_a: [f64; 3],
    min_b: [f64; 3],
    max_b: [f64; 3],
) -> PyResult<f64> {
    let box_a = zenith_algo::PrimitiveBuilder::make_box(
        max_a[0] - min_a[0],
        max_a[1] - min_a[1],
        max_a[2] - min_a[2],
    )
    .map_err(|e| PyValueError::new_err(format!("Box A creation failed: {}", e)))?;
    let box_a = zenith_algo::BrepTransform::translate_solid(
        &box_a,
        Vec3::new(min_a[0], min_a[1], min_a[2]),
    );

    let box_b = zenith_algo::PrimitiveBuilder::make_box(
        max_b[0] - min_b[0],
        max_b[1] - min_b[1],
        max_b[2] - min_b[2],
    )
    .map_err(|e| PyValueError::new_err(format!("Box B creation failed: {}", e)))?;
    let box_b = zenith_algo::BrepTransform::translate_solid(
        &box_b,
        Vec3::new(min_b[0], min_b[1], min_b[2]),
    );

    let tol = Tolerance::default();
    let result = zenith_algo::DistanceEngine::compute_min_distance(&box_a, &box_b, &tol);

    Ok(result.min_distance)
}

/// 皿モミ穴（Countersink Hole）付き直方体ソリッドの生成（STEP対応）
#[pyfunction]
#[pyo3(signature = (box_w = 40.0, box_d = 40.0, box_h = 20.0, hole_r = 3.0, cs_r = 6.0, cs_angle_deg = 90.0, center_x = 20.0, center_y = 20.0, u_divisions = 16, v_divisions = 16, step_path = None))]
pub fn make_countersink_hole_box(
    box_w: f64,
    box_d: f64,
    box_h: f64,
    hole_r: f64,
    cs_r: f64,
    cs_angle_deg: f64,
    center_x: f64,
    center_y: f64,
    u_divisions: usize,
    v_divisions: usize,
    step_path: Option<&str>,
) -> PyResult<PyMesh> {
    let solid = zenith_algo::HoleBuilder::make_countersink_hole_box(
        box_w,
        box_d,
        box_h,
        hole_r,
        cs_r,
        cs_angle_deg,
        center_x,
        center_y,
    )
    .map_err(|e| PyValueError::new_err(format!("Countersink hole generation failed: {}", e)))?;

    if let Some(path) = step_path {
        StepExporter::export_solid_to_file(&solid, path, "ZENITH_COUNTERSINK_BOX")
            .map_err(|e| PyValueError::new_err(format!("STEP export failed: {}", e)))?;
    }

    let params = TessellationParams {
        u_divisions,
        v_divisions,
    };
    let mesh = tessellate_solid(&solid, &params);
    Ok(PyMesh { mesh })
}

/// PCD等配ボルト穴付き円形フランジソリッドの生成（STEP対応）
#[pyfunction]
#[pyo3(signature = (outer_radius = 40.0, thickness = 10.0, center_hole_radius = 15.0, pcd_radius = 28.0, num_bolt_holes = 4, bolt_hole_radius = 3.5, u_divisions = 16, v_divisions = 16, step_path = None))]
pub fn make_circular_flange(
    outer_radius: f64,
    thickness: f64,
    center_hole_radius: f64,
    pcd_radius: f64,
    num_bolt_holes: usize,
    bolt_hole_radius: f64,
    u_divisions: usize,
    v_divisions: usize,
    step_path: Option<&str>,
) -> PyResult<PyMesh> {
    let solid = zenith_algo::FlangeBuilder::make_circular_flange(
        outer_radius,
        thickness,
        center_hole_radius,
        pcd_radius,
        num_bolt_holes,
        bolt_hole_radius,
    )
    .map_err(|e| PyValueError::new_err(format!("Circular flange generation failed: {}", e)))?;

    if let Some(path) = step_path {
        StepExporter::export_solid_to_file(&solid, path, "ZENITH_CIRCULAR_FLANGE")
            .map_err(|e| PyValueError::new_err(format!("STEP export failed: {}", e)))?;
    }

    let params = TessellationParams {
        u_divisions,
        v_divisions,
    };
    let mesh = tessellate_solid(&solid, &params);
    Ok(PyMesh { mesh })
}

/// 環状溝（Annular Groove / 止め輪・Oリング溝）付き軸ソリッドの生成（STEP対応）
#[pyfunction]
#[pyo3(signature = (shaft_radius = 15.0, shaft_length = 60.0, groove_width = 4.0, groove_depth = 2.5, groove_z_pos = 25.0, u_divisions = 16, v_divisions = 16, step_path = None))]
pub fn make_shaft_with_annular_groove(
    shaft_radius: f64,
    shaft_length: f64,
    groove_width: f64,
    groove_depth: f64,
    groove_z_pos: f64,
    u_divisions: usize,
    v_divisions: usize,
    step_path: Option<&str>,
) -> PyResult<PyMesh> {
    let base_shaft = zenith_algo::ShaftBuilder::make_stepped_shaft(&[(shaft_radius, shaft_length)])
        .map_err(|e| PyValueError::new_err(format!("Base shaft generation failed: {}", e)))?;

    let solid = zenith_algo::ShaftBuilder::make_shaft_with_annular_groove(
        &base_shaft,
        shaft_radius,
        groove_width,
        groove_depth,
        groove_z_pos,
    )
    .map_err(|e| PyValueError::new_err(format!("Grooved shaft generation failed: {}", e)))?;

    if let Some(path) = step_path {
        StepExporter::export_solid_to_file(&solid, path, "ZENITH_GROOVED_SHAFT")
            .map_err(|e| PyValueError::new_err(format!("STEP export failed: {}", e)))?;
    }

    let params = TessellationParams {
        u_divisions,
        v_divisions,
    };
    let mesh = tessellate_solid(&solid, &params);
    Ok(PyMesh { mesh })
}

/// 直方体断面を 2D DXF ファイルへ出力
#[pyfunction]
#[pyo3(signature = (box_w, box_d, box_h, plane_origin, plane_normal, dxf_path))]
pub fn export_box_section_dxf(
    box_w: f64,
    box_d: f64,
    box_h: f64,
    plane_origin: [f64; 3],
    plane_normal: [f64; 3],
    dxf_path: &str,
) -> PyResult<usize> {
    let solid = zenith_algo::PrimitiveBuilder::make_box(box_w, box_d, box_h)
        .map_err(|e| PyValueError::new_err(format!("Box generation failed: {}", e)))?;

    let origin = zenith_math::Point3::new(plane_origin[0], plane_origin[1], plane_origin[2]);
    let normal = zenith_math::Vec3::new(plane_normal[0], plane_normal[1], plane_normal[2]);

    let tol = Tolerance::default();
    let result = zenith_algo::SectionSlicer::slice_solid(&solid, origin, normal, &tol)
        .map_err(|e| PyValueError::new_err(format!("Section slicing failed: {}", e)))?;

    let loops: Vec<Vec<zenith_math::Point3>> = result
        .section_wires
        .iter()
        .map(|w| {
            w.edges
                .iter()
                .map(|oe| oe.edge.start_vertex.point)
                .collect()
        })
        .collect();

    zenith_io::DxfExporter::export_loops_to_file(&loops, dxf_path)
        .map_err(|e| PyValueError::new_err(format!("DXF export failed: {}", e)))?;

    Ok(loops.len())
}
