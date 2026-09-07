use pyo3::exceptions::PyValueError;
use pyo3::prelude::*;
use zenith_algo::PrimitiveBuilder;
use zenith_topo::ShaderBRepPayload;

/// 直方体の解析的シェーダーペイロードJSONを取得
#[pyfunction]
#[pyo3(signature = (dx = 10.0, dy = 10.0, dz = 10.0))]
pub fn get_box_shader_payload(dx: f64, dy: f64, dz: f64) -> PyResult<String> {
    let solid = PrimitiveBuilder::make_box(dx, dy, dz)
        .map_err(|e| PyValueError::new_err(format!("Box failed: {}", e)))?;
    let payload = ShaderBRepPayload::from_solid(&solid);
    serde_json::to_string(&payload)
        .map_err(|e| PyValueError::new_err(format!("Serialization failed: {}", e)))
}

/// 任意プリミティブのSDFシェーダーペイロードJSONを取得
#[pyfunction]
#[pyo3(signature = (prim_type, p0 = 10.0, p1 = 10.0, p2 = 10.0, p3 = 0.0))]
pub fn get_primitive_shader_payload(
    prim_type: &str,
    p0: f64,
    p1: f64,
    p2: f64,
    p3: f64,
) -> PyResult<String> {
    let _ = p3;
    let payload = match prim_type.to_lowercase().as_str() {
        "box" => ShaderBRepPayload::new_box(
            [0.0, 0.0, 0.0],
            [(p0 * 0.5) as f32, (p1 * 0.5) as f32, (p2 * 0.5) as f32],
        ),
        "cylinder" => {
            ShaderBRepPayload::new_cylinder([0.0, 0.0, 0.0], p0 as f32, (p1 * 0.5) as f32)
        }
        "sphere" => ShaderBRepPayload::new_sphere([0.0, 0.0, 0.0], p0 as f32),
        "cone" => {
            ShaderBRepPayload::new_cone([0.0, 0.0, 0.0], p0 as f32, p1 as f32, (p2 * 0.5) as f32)
        }
        "torus" => ShaderBRepPayload::new_torus([0.0, 0.0, 0.0], p0 as f32, p1 as f32),
        _ => {
            return Err(PyValueError::new_err(format!(
                "Unknown primitive type: {}",
                prim_type
            )))
        }
    };

    serde_json::to_string(&payload)
        .map_err(|e| PyValueError::new_err(format!("Serialization failed: {}", e)))
}

/// 2D幾何拘束スケッチソルバー（Python連携）
#[pyfunction]
#[pyo3(signature = (points_json, constraints_json))]
pub fn solve_2d_sketch(points_json: &str, constraints_json: &str) -> PyResult<String> {
    // 簡易パース＆解決
    let pts: Vec<[f64; 2]> = serde_json::from_str(points_json)
        .map_err(|e| PyValueError::new_err(format!("Invalid points JSON: {}", e)))?;

    let mut solver = zenith_algo::SketchSolver::new();
    let mut pt_ids = Vec::with_capacity(pts.len());
    for (i, p) in pts.iter().enumerate() {
        if i == 0 {
            pt_ids.push(solver.add_fixed_point(p[0], p[1]));
        } else {
            pt_ids.push(solver.add_point(p[0], p[1]));
        }
    }

    // 制約JSONパース（[ {"type": "distance", "p1": 0, "p2": 1, "value": 10.0}, ... ]）
    let raw_constraints: Vec<serde_json::Value> = serde_json::from_str(constraints_json)
        .map_err(|e| PyValueError::new_err(format!("Invalid constraints JSON: {}", e)))?;

    // **黙って落としません**（4-401）。
    //
    // ここは長らく、**知らない種類を `_ => {}` で無視**し、**番号が範囲の
    // 外なら拘束ごと捨て**、**`value` が無ければ 10.0 を入れて**いました。
    //
    // **呼んだ側は「解けた」点を受け取ります**——**自分が掛けたつもりの
    // 拘束が、1 本も入っていなくても**です。**綴りを 1 文字間違えるだけで、
    // 何も掛かっていない図が「解けた」と返ります。**
    //
    // **このリポジトリの決まりどおり、名指しで断ります。**
    let index_of = |value: Option<&serde_json::Value>, field: &str| -> PyResult<usize> {
        let raw = value.and_then(|v| v.as_u64()).ok_or_else(|| {
            PyValueError::new_err(format!("拘束に {field} がありません（整数で渡してください）"))
        })? as usize;
        if raw >= pt_ids.len() {
            return Err(PyValueError::new_err(format!(
                "{field} の点の番号 {raw} が範囲の外です（点は {} 個）",
                pt_ids.len()
            )));
        }
        Ok(raw)
    };

    for c in raw_constraints {
        let c_type = c.get("type").and_then(|v| v.as_str()).unwrap_or("");
        match c_type {
            "horizontal" => {
                let (p1, p2) = (index_of(c.get("p1"), "p1")?, index_of(c.get("p2"), "p2")?);
                solver.add_constraint(zenith_algo::Constraint::Horizontal(pt_ids[p1], pt_ids[p2]));
            }
            "vertical" => {
                let (p1, p2) = (index_of(c.get("p1"), "p1")?, index_of(c.get("p2"), "p2")?);
                solver.add_constraint(zenith_algo::Constraint::Vertical(pt_ids[p1], pt_ids[p2]));
            }
            "distance" => {
                let (p1, p2) = (index_of(c.get("p1"), "p1")?, index_of(c.get("p2"), "p2")?);
                // **`value` が無ければ断ります。** 既定の 10.0 を入れると、
                // **頼んでいない寸法**が掛かります——**綴りを 1 文字
                // 間違えただけで、10 mm の拘束が入っていました。**
                let val = c.get("value").and_then(|v| v.as_f64()).ok_or_else(|| {
                    PyValueError::new_err("distance に value がありません".to_string())
                })?;
                solver.add_constraint(zenith_algo::Constraint::Distance(
                    pt_ids[p1], pt_ids[p2], val,
                ));
            }
            // **知らない種類は断ります。** 黙って無視すると、**掛けたつもりの
            // 拘束が 1 本も入っていない図**が「解けた」と返ります。
            other => {
                return Err(PyValueError::new_err(format!(
                    "知らない拘束の種類です: {other:?}（horizontal / vertical / distance）"
                )))
            }
        }
    }

    solver
        .solve(50, 1e-6)
        .map_err(|e| PyValueError::new_err(format!("Sketch solve failed: {}", e)))?;

    let result_pts: Vec<[f64; 2]> = pt_ids
        .iter()
        .filter_map(|&id| solver.get_point(id))
        .collect();
    serde_json::to_string(&result_pts)
        .map_err(|e| PyValueError::new_err(format!("Result serialization failed: {}", e)))
}
