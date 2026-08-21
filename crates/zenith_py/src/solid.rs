//! Python から立体そのものを持ち回るためのハンドル。
//!
//! これまでの Python API は `make_箱に穴(...) -> Mesh` の形しかありませんでした。
//! 立体を受け取って返す型が無いので、Python 側では
//!
//! - 作った立体を次の演算に渡す
//! - ブーリアンの結果を丸める
//! - 稜を選んでフィレットを掛ける
//!
//! のどれもできず、組み合わせが要るたびに Rust 側へ専用関数
//! （`make_exact_drill_boolean` など）を足すしかありませんでした。
//!
//! `Solid` はその欠けていた型です。各メソッドは**新しい立体を返す**ので、
//! 元の立体は変わりません。失敗は例外になり、近い別形状は返しません。
//!
//! ```python
//! import zenith_cad as z
//!
//! part = z.Solid.box(40, 40, 20).difference(
//!     z.Solid.cylinder(6, 20).translated(20, 20, 0)
//! )
//! for edge in part.blendable_edges():
//!     part = part.fillet_edge(edge["edge_id"], 2.0)
//! part.to_step("part.step")
//! ```

use pyo3::exceptions::PyValueError;
use pyo3::prelude::*;
use pyo3::types::{PyDict, PyList};

use zenith_algo::{
    BooleanEngine, BooleanOpType, BrepTransform, DirectModeling, EdgeBlender, EdgeKind,
    InterferenceChecker, MassCalculator, MirrorBuilder, PrimitiveBuilder, Sewer, StepInterop,
};
use zenith_io::StepImporter;
use zenith_math::{Point3, Tolerance, Transform3, Vec3};
use zenith_tess::{tessellate_solid, TessellationParams};
use zenith_topo::Solid;

use crate::mesh::PyMesh;

fn invalid(message: impl std::fmt::Display) -> PyErr {
    PyValueError::new_err(message.to_string())
}

/// B-Rep ソリッド。すべてのメソッドは新しい `Solid` を返す。
#[pyclass(name = "Solid")]
#[derive(Clone)]
pub struct PySolid {
    pub solid: Solid,
}

impl PySolid {
    fn wrap(solid: Solid) -> Self {
        Self { solid }
    }
}

#[pymethods]
impl PySolid {
    // ------------------------------------------------------------------
    // 生成
    // ------------------------------------------------------------------

    /// 原点を角とする直方体
    #[staticmethod]
    #[pyo3(name = "box")]
    pub fn box_(dx: f64, dy: f64, dz: f64) -> PyResult<Self> {
        PrimitiveBuilder::make_box(dx, dy, dz)
            .map(Self::wrap)
            .map_err(invalid)
    }

    /// 底面中心を原点、軸を +Z とする円柱
    #[staticmethod]
    pub fn cylinder(radius: f64, height: f64) -> PyResult<Self> {
        PrimitiveBuilder::make_cylinder(radius, height)
            .map(Self::wrap)
            .map_err(invalid)
    }

    /// 中心を原点とする球
    #[staticmethod]
    pub fn sphere(radius: f64) -> PyResult<Self> {
        PrimitiveBuilder::make_sphere(radius)
            .map(Self::wrap)
            .map_err(invalid)
    }

    /// 円錐台（`r_top` に 0 を渡せば円錐）
    #[staticmethod]
    pub fn cone(r_bottom: f64, r_top: f64, height: f64) -> PyResult<Self> {
        PrimitiveBuilder::make_cone(r_bottom, r_top, height)
            .map(Self::wrap)
            .map_err(invalid)
    }

    /// トーラス
    #[staticmethod]
    pub fn torus(major_radius: f64, minor_radius: f64) -> PyResult<Self> {
        PrimitiveBuilder::make_torus(major_radius, minor_radius)
            .map(Self::wrap)
            .map_err(invalid)
    }

    /// 正 `sides` 角柱
    #[staticmethod]
    pub fn regular_prism(sides: usize, radius: f64, height: f64) -> PyResult<Self> {
        PrimitiveBuilder::make_regular_prism(sides, radius, height)
            .map(Self::wrap)
            .map_err(invalid)
    }

    /// STEP ファイルから読み込む（複数ソリッドがあれば最初の1つ）
    #[staticmethod]
    pub fn from_step(path: &str) -> PyResult<Self> {
        StepImporter::import_solid_from_file(path)
            .map(Self::wrap)
            .map_err(invalid)
    }

    /// STEP ファイルの全ソリッド
    #[staticmethod]
    pub fn all_from_step(path: &str) -> PyResult<Vec<Self>> {
        StepImporter::import_solids_from_file(path)
            .map(|solids| solids.into_iter().map(Self::wrap).collect())
            .map_err(invalid)
    }

    // ------------------------------------------------------------------
    // 変換
    // ------------------------------------------------------------------

    /// 平行移動した複製
    pub fn translated(&self, dx: f64, dy: f64, dz: f64) -> Self {
        Self::wrap(BrepTransform::translate_solid(
            &self.solid,
            Vec3::new(dx, dy, dz),
        ))
    }

    /// 任意軸まわりに回転した複製
    pub fn rotated(
        &self,
        axis_origin: [f64; 3],
        axis_dir: [f64; 3],
        angle_deg: f64,
    ) -> PyResult<Self> {
        let axis = Vec3::new(axis_dir[0], axis_dir[1], axis_dir[2]);
        if axis.norm() <= 1e-12 {
            return Err(invalid("The rotation axis has no direction"));
        }
        let origin = Vec3::new(axis_origin[0], axis_origin[1], axis_origin[2]);
        let transform = Transform3::from_translation(origin)
            .compose(&Transform3::from_axis_angle(&axis, angle_deg.to_radians()))
            .compose(&Transform3::from_translation(-origin));
        BrepTransform::transform_solid(&self.solid, &transform)
            .map(Self::wrap)
            .map_err(invalid)
    }

    /// 平面に対する鏡像複製
    pub fn mirrored(&self, plane_origin: [f64; 3], plane_normal: [f64; 3]) -> PyResult<Self> {
        let normal = Vec3::new(plane_normal[0], plane_normal[1], plane_normal[2]);
        if normal.norm() <= 1e-12 {
            return Err(invalid("The mirror plane has no normal"));
        }
        MirrorBuilder::mirror_solid(
            &self.solid,
            Point3::new(plane_origin[0], plane_origin[1], plane_origin[2]),
            normal,
            &Tolerance::default(),
        )
        .map(Self::wrap)
        .map_err(invalid)
    }

    // ------------------------------------------------------------------
    // ブーリアン
    // ------------------------------------------------------------------

    /// 和
    pub fn union(&self, other: &PySolid) -> PyResult<Self> {
        self.boolean(other, BooleanOpType::Union)
    }

    /// 差
    pub fn difference(&self, other: &PySolid) -> PyResult<Self> {
        self.boolean(other, BooleanOpType::Difference)
    }

    /// 積
    pub fn intersection(&self, other: &PySolid) -> PyResult<Self> {
        self.boolean(other, BooleanOpType::Intersection)
    }

    /// 複数のツール立体を順に差し引く
    pub fn difference_all(&self, tools: Vec<PySolid>) -> PyResult<Self> {
        let solids: Vec<Solid> = tools.into_iter().map(|tool| tool.solid).collect();
        BooleanEngine::boolean_solids_batch(
            &self.solid,
            &solids,
            BooleanOpType::Difference,
            &Tolerance::default(),
        )
        .map(Self::wrap)
        .map_err(invalid)
    }

    // ------------------------------------------------------------------
    // 稜のフィーチャー
    // ------------------------------------------------------------------

    /// すべての稜の下調べ。`edge_id` はこの立体の中で有効。
    pub fn edges<'py>(&self, py: Python<'py>) -> PyResult<Bound<'py, PyList>> {
        let mut seen: Vec<u64> = Vec::new();
        let rows = PyList::empty(py);
        for face in &self.solid.outer_shell.faces {
            for wire in std::iter::once(&face.outer_wire).chain(face.inner_wires.iter()) {
                for oriented in &wire.edges {
                    let id = oriented.edge.id;
                    if seen.contains(&id) {
                        continue;
                    }
                    seen.push(id);

                    let inspection =
                        DirectModeling::inspect_solid_edge(&self.solid, id).map_err(invalid)?;
                    let row = PyDict::new(py);
                    row.set_item("edge_id", id)?;
                    row.set_item("length", inspection.length)?;
                    row.set_item(
                        "start",
                        [
                            inspection.start_point.x,
                            inspection.start_point.y,
                            inspection.start_point.z,
                        ],
                    )?;
                    row.set_item(
                        "end",
                        [
                            inspection.end_point.x,
                            inspection.end_point.y,
                            inspection.end_point.z,
                        ],
                    )?;
                    row.set_item(
                        "midpoint",
                        [
                            inspection.midpoint.x,
                            inspection.midpoint.y,
                            inspection.midpoint.z,
                        ],
                    )?;
                    row.set_item("dihedral_angle_deg", inspection.dihedral_angle_deg)?;
                    row.set_item(
                        "kind",
                        match inspection.kind {
                            EdgeKind::Convex => "convex",
                            EdgeKind::Concave => "concave",
                            EdgeKind::Smooth => "smooth",
                            EdgeKind::FreeBoundary => "free",
                        },
                    )?;
                    rows.append(row)?;
                }
            }
        }
        Ok(rows)
    }

    /// フィレット・面取りを掛けられる稜だけを、上限値つきで返す
    pub fn blendable_edges<'py>(&self, py: Python<'py>) -> PyResult<Bound<'py, PyList>> {
        let rows = PyList::empty(py);
        for edge in EdgeBlender::blendable_edges(&self.solid) {
            let row = PyDict::new(py);
            row.set_item("edge_id", edge.edge_id)?;
            row.set_item("length", edge.length)?;
            row.set_item("dihedral_angle_deg", edge.dihedral_angle_deg)?;
            row.set_item("max_fillet_radius", edge.max_fillet_radius)?;
            row.set_item("max_chamfer_distance", edge.max_chamfer_distance)?;
            rows.append(row)?;
        }
        Ok(rows)
    }

    /// 1本の稜に半径 `radius` のフィレットを掛ける
    pub fn fillet_edge(&self, edge_id: u64, radius: f64) -> PyResult<Self> {
        EdgeBlender::fillet_edge(&self.solid, edge_id, radius)
            .map(Self::wrap)
            .map_err(invalid)
    }

    /// 1本の稜に距離 `distance` の面取りを掛ける
    pub fn chamfer_edge(&self, edge_id: u64, distance: f64) -> PyResult<Self> {
        EdgeBlender::chamfer_edge(&self.solid, edge_id, distance)
            .map(Self::wrap)
            .map_err(invalid)
    }

    /// 複数の稜に同じ半径でフィレットを掛ける
    pub fn fillet_edges(&self, edge_ids: Vec<u64>, radius: f64) -> PyResult<Self> {
        let requests: Vec<(u64, f64)> = edge_ids.into_iter().map(|id| (id, radius)).collect();
        EdgeBlender::fillet_edges(&self.solid, &requests)
            .map(Self::wrap)
            .map_err(invalid)
    }

    /// 複数の稜に同じ距離で面取りを掛ける
    pub fn chamfer_edges(&self, edge_ids: Vec<u64>, distance: f64) -> PyResult<Self> {
        let requests: Vec<(u64, f64)> = edge_ids.into_iter().map(|id| (id, distance)).collect();
        EdgeBlender::chamfer_edges(&self.solid, &requests)
            .map(Self::wrap)
            .map_err(invalid)
    }

    // ------------------------------------------------------------------
    // 面のフィーチャー
    // ------------------------------------------------------------------

    /// 面の一覧（`face_index` は下の操作でそのまま使える）
    pub fn faces<'py>(&self, py: Python<'py>) -> PyResult<Bound<'py, PyList>> {
        let rows = PyList::empty(py);
        for (index, face) in self.solid.outer_shell.faces.iter().enumerate() {
            let inspection = DirectModeling::inspect_face(face).map_err(invalid)?;
            let row = PyDict::new(py);
            row.set_item("face_index", index)?;
            row.set_item("area", inspection.area)?;
            row.set_item(
                "centroid",
                [
                    inspection.centroid.x,
                    inspection.centroid.y,
                    inspection.centroid.z,
                ],
            )?;
            row.set_item(
                "normal",
                [
                    inspection.normal.x,
                    inspection.normal.y,
                    inspection.normal.z,
                ],
            )?;
            row.set_item(
                "surface",
                match face.geometry {
                    zenith_topo::FaceGeometry::Plane(_) => "plane",
                    zenith_topo::FaceGeometry::Nurbs(_) => "nurbs",
                    zenith_topo::FaceGeometry::Coons(_) => "coons",
                    zenith_topo::FaceGeometry::Gordon(_) => "gordon",
                    zenith_topo::FaceGeometry::Triangular(_) => "triangular",
                },
            )?;
            rows.append(row)?;
        }
        Ok(rows)
    }

    /// 面を法線方向に押し引きする
    pub fn push_pull_face(&self, face_index: usize, distance: f64) -> PyResult<Self> {
        DirectModeling::push_pull_face(&self.solid, face_index, distance)
            .map(Self::wrap)
            .map_err(invalid)
    }

    /// 面を軸まわりに傾ける（抜き勾配）
    pub fn taper_face(
        &self,
        face_index: usize,
        axis_origin: [f64; 3],
        axis_dir: [f64; 3],
        angle_deg: f64,
    ) -> PyResult<Self> {
        DirectModeling::taper_face(
            &self.solid,
            face_index,
            Point3::new(axis_origin[0], axis_origin[1], axis_origin[2]),
            Vec3::new(axis_dir[0], axis_dir[1], axis_dir[2]),
            angle_deg,
        )
        .map(Self::wrap)
        .map_err(invalid)
    }

    // ------------------------------------------------------------------
    // 測る・出す
    // ------------------------------------------------------------------

    /// 体積・表面積・重心・慣性主成分。B-Rep の面そのものを積分する。
    #[pyo3(signature = (u_divisions = 32, v_divisions = 32))]
    pub fn mass_properties<'py>(
        &self,
        py: Python<'py>,
        u_divisions: usize,
        v_divisions: usize,
    ) -> PyResult<Bound<'py, PyDict>> {
        let properties = MassCalculator::compute_from_brep(
            &self.solid,
            &TessellationParams {
                u_divisions,
                v_divisions,
            },
        );
        let out = PyDict::new(py);
        out.set_item("volume", properties.volume)?;
        out.set_item("surface_area", properties.surface_area)?;
        out.set_item(
            "center_of_mass",
            [
                properties.center_of_mass.x,
                properties.center_of_mass.y,
                properties.center_of_mass.z,
            ],
        )?;
        out.set_item(
            "inertia_diagonal",
            [
                properties.inertia_diagonal.x,
                properties.inertia_diagonal.y,
                properties.inertia_diagonal.z,
            ],
        )?;
        Ok(out)
    }

    /// 位相の検査結果。`valid` が偽なら `errors` に理由が入る。
    ///
    /// `unshared_edge_entity_uses` は「座標では閉じているが、稜が実体として
    /// 共有されていない」箇所の数。ここが 0 でない立体には稜を選ぶ演算が
    /// 掛からない。`sewn()` で直せる。
    pub fn validate<'py>(&self, py: Python<'py>) -> PyResult<Bound<'py, PyDict>> {
        let report = self.solid.outer_shell.validate_closed(&Tolerance::default());
        let out = PyDict::new(py);
        out.set_item("valid", report.is_valid())?;
        out.set_item("face_count", report.face_count)?;
        out.set_item("edge_use_count", report.edge_use_count)?;
        out.set_item("unmatched_edge_uses", report.unmatched_edge_use_count)?;
        out.set_item("non_manifold_edge_uses", report.non_manifold_edge_use_count)?;
        out.set_item(
            "unshared_edge_entity_uses",
            report.unshared_edge_entity_use_count,
        )?;
        out.set_item("errors", report.errors.clone())?;
        Ok(out)
    }

    /// 同じ位置に並んでいる稜と頂点を1つの実体に束ねた複製
    pub fn sewn(&self) -> PyResult<Self> {
        Sewer::sew_solid(&self.solid, &Tolerance::default())
            .map(|(solid, _report)| Self::wrap(solid))
            .map_err(invalid)
    }

    /// もう1つの立体との干渉状態: "clearance" / "touching" / "clash"
    pub fn clash_status(&self, other: &PySolid) -> String {
        match InterferenceChecker::check(&self.solid, &other.solid, &Tolerance::default()).status {
            zenith_algo::ClashStatus::Clearance => "clearance".to_string(),
            zenith_algo::ClashStatus::Touching => "touching".to_string(),
            zenith_algo::ClashStatus::Clash => "clash".to_string(),
        }
    }

    /// 表示用メッシュ
    #[pyo3(signature = (u_divisions = 16, v_divisions = 16))]
    pub fn tessellate(&self, u_divisions: usize, v_divisions: usize) -> PyMesh {
        PyMesh {
            mesh: tessellate_solid(
                &self.solid,
                &TessellationParams {
                    u_divisions,
                    v_divisions,
                },
            ),
        }
    }

    /// STEP ファイルに書く（`StepInterop` を通すので正規化される）
    #[pyo3(signature = (path, product_name = "zenith_solid"))]
    pub fn to_step(&self, path: &str, product_name: &str) -> PyResult<()> {
        StepInterop::export_solid_to_file(&self.solid, path, product_name, &Tolerance::default())
            .map(|_report| ())
            .map_err(invalid)
    }

    /// STEP を文字列で受け取る
    #[pyo3(signature = (product_name = "zenith_solid"))]
    pub fn to_step_string(&self, product_name: &str) -> String {
        StepInterop::export_solid_to_string(&self.solid, product_name, &Tolerance::default()).0
    }

    /// 面の枚数
    #[getter]
    pub fn face_count(&self) -> usize {
        self.solid.outer_shell.faces.len()
    }

    /// 体積（既定の分割での測定値）
    #[getter]
    pub fn volume(&self) -> f64 {
        MassCalculator::compute_from_brep(
            &self.solid,
            &TessellationParams {
                u_divisions: 32,
                v_divisions: 32,
            },
        )
        .volume
    }

    fn __repr__(&self) -> String {
        format!(
            "Solid(faces={}, volume={:.6})",
            self.solid.outer_shell.faces.len(),
            self.volume()
        )
    }
}

impl PySolid {
    fn boolean(&self, other: &PySolid, op: BooleanOpType) -> PyResult<Self> {
        BooleanEngine::boolean_solids_exact(&self.solid, &other.solid, op, &Tolerance::default())
            .map(Self::wrap)
            .map_err(invalid)
    }
}
