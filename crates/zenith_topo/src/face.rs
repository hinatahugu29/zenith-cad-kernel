use crate::edge::{Orientation, OrientedEdge};
use crate::wire::Wire;
use serde::{Deserialize, Serialize};
use std::sync::atomic::{AtomicU64, Ordering};
use zenith_geom::{
    ControlPoint2, CoonsPatch3, ExtremumEngine, GordonSurface3, KnotVector, NurbsCurve2,
    NurbsSurface3, PlaneSurface3, Surface3, TriangularPatch3,
};
use zenith_math::{Point2, Point3, Tolerance, Vec3};

static FACE_ID_GEN: AtomicU64 = AtomicU64::new(1);

/// 面の幾何形状表現
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub enum FaceGeometry {
    Plane(PlaneSurface3),
    Nurbs(NurbsSurface3),
    Coons(CoonsPatch3),
    Gordon(GordonSurface3),
    Triangular(TriangularPatch3),
}

/// B-Rep フェイス（Face: 曲面 + 境界ワイヤ）
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct Face {
    pub id: u64,
    pub geometry: FaceGeometry,
    pub outer_wire: Wire,
    pub inner_wires: Vec<Wire>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub pcurves: Option<FacePcurves>,
    pub orientation: Orientation,
    pub tolerance: f64,
}

/// Face 境界と支持曲面の検証結果
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct FaceBoundaryValidationReport {
    pub sampled_point_count: usize,
    pub off_surface_point_count: usize,
    pub max_distance: f64,
    pub errors: Vec<String>,
}

/// Face 上の1本の3D Edgeに対応する2Dパラメータ空間曲線
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct FacePcurveSegment {
    pub edge_id: u64,
    pub orientation: Orientation,
    pub curve: NurbsCurve2,
}

/// Face 上の1つのトリムループ
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct FacePcurveLoop {
    pub segments: Vec<FacePcurveSegment>,
}

/// Face の外側・内側トリムをp-curveとして表したもの
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct FacePcurves {
    pub outer_loop: FacePcurveLoop,
    pub inner_loops: Vec<FacePcurveLoop>,
}

/// p-curve と3D境界曲線の一致検証結果
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct PcurveValidationReport {
    pub sampled_point_count: usize,
    pub mismatch_count: usize,
    pub max_distance: f64,
    pub errors: Vec<String>,
}

impl FaceBoundaryValidationReport {
    pub fn is_valid(&self) -> bool {
        self.errors.is_empty()
    }
}

impl PcurveValidationReport {
    pub fn is_valid(&self) -> bool {
        self.errors.is_empty()
    }
}

impl Face {
    pub fn new(
        geometry: FaceGeometry,
        outer_wire: Wire,
        inner_wires: Vec<Wire>,
        orientation: Orientation,
        tolerance: f64,
    ) -> Self {
        let mut face = Self {
            id: FACE_ID_GEN.fetch_add(1, Ordering::Relaxed),
            geometry,
            outer_wire,
            inner_wires,
            pcurves: None,
            orientation,
            tolerance,
        };

        match &face.geometry {
            FaceGeometry::Plane(_) => {
                if let Ok(pcurves) = face.derive_plane_pcurves() {
                    face.pcurves = Some(pcurves);
                }
            }
            FaceGeometry::Nurbs(_) => {
                if let Ok(pcurves) = face.derive_nurbs_boundary_pcurves(&Tolerance::default(), 8) {
                    face.pcurves = Some(pcurves);
                }
            }
            _ => {}
        }

        face
    }

    /// 穴なしのFaceを作成
    /// Whether the face covers its whole surface, with nothing trimmed away.
    ///
    /// A closed surface written as a single face - the way OpenCASCADE writes a
    /// sphere or a torus - has no real boundary. Either there is none at all,
    /// STEP's VERTEX_LOOP standing in as a single point, or what stands in for
    /// one is the seam, traversed once each way, so every edge in the loop
    /// appears twice.
    ///
    /// Such a loop encloses the whole parameter domain, but it cannot be read
    /// that way from its p-curves: a point on the seam maps to both ends of the
    /// domain at once, so the signed area it traces depends on which end the
    /// projection happens to pick. The topology says it plainly, so ask that.
    pub fn has_seam_only_boundary(&self, tol: f64) -> bool {
        if !self.inner_wires.is_empty() {
            return false;
        }
        let edges = &self.outer_wire.edges;
        if edges.is_empty() {
            return true;
        }
        edges.iter().enumerate().all(|(index, edge)| {
            edges.iter().enumerate().any(|(other_index, other)| {
                other_index != index && same_edge_geometry(edge, other, tol)
            })
        })
    }

    pub fn simple(geometry: FaceGeometry, outer_wire: Wire) -> Self {
        Self::new(geometry, outer_wire, Vec::new(), Orientation::Forward, 1e-6)
    }

    /// CoonsパッチからFaceを直接生成（4境界ワイヤ付き）
    pub fn from_coons_patch(coons: CoonsPatch3, wire: Wire) -> Self {
        Self::simple(FaceGeometry::Coons(coons), wire)
    }

    /// NURBS曲面からFaceを直接生成
    pub fn from_nurbs_surface(nurbs: NurbsSurface3, wire: Wire) -> Self {
        Self::simple(FaceGeometry::Nurbs(nurbs), wire)
    }

    /// Plane Face の3D境界曲線を、支持平面上の2D p-curveへ射影する。
    pub fn derive_plane_pcurves(&self) -> Result<FacePcurves, String> {
        let FaceGeometry::Plane(plane) = &self.geometry else {
            return Err("Plane p-curves can only be derived for planar faces".to_string());
        };

        let outer_loop = derive_wire_plane_pcurves(&self.outer_wire, plane)?;
        let mut inner_loops = Vec::with_capacity(self.inner_wires.len());
        for wire in &self.inner_wires {
            inner_loops.push(derive_wire_plane_pcurves(wire, plane)?);
        }

        Ok(FacePcurves {
            outer_loop,
            inner_loops,
        })
    }

    /// 保持済みp-curveがあればそれを使い、なければPlane境界から導出する。
    pub fn plane_pcurves(&self) -> Result<FacePcurves, String> {
        if let Some(pcurves) = &self.pcurves {
            return Ok(pcurves.clone());
        }

        self.derive_plane_pcurves()
    }

    /// NURBS Face の3D EdgeをUV空間のp-curveへ導出する。
    pub fn derive_nurbs_boundary_pcurves(
        &self,
        tol: &Tolerance,
        samples_per_edge: usize,
    ) -> Result<FacePcurves, String> {
        let FaceGeometry::Nurbs(surface) = &self.geometry else {
            return Err("NURBS p-curves can only be derived for NURBS faces".to_string());
        };

        let outer_loop =
            derive_wire_nurbs_boundary_pcurves(&self.outer_wire, surface, tol, samples_per_edge)?;
        let mut inner_loops = Vec::with_capacity(self.inner_wires.len());
        for wire in &self.inner_wires {
            inner_loops.push(derive_wire_nurbs_boundary_pcurves(
                wire,
                surface,
                tol,
                samples_per_edge,
            )?);
        }

        Ok(FacePcurves {
            outer_loop,
            inner_loops,
        })
    }

    /// 保持済みp-curveがあればそれを使い、なければFace種別に応じて導出する。
    pub fn pcurves(&self, tol: &Tolerance) -> Result<FacePcurves, String> {
        if let Some(pcurves) = &self.pcurves {
            return Ok(pcurves.clone());
        }

        match &self.geometry {
            FaceGeometry::Plane(_) => self.derive_plane_pcurves(),
            FaceGeometry::Nurbs(_) => self.derive_nurbs_boundary_pcurves(tol, 8),
            _ => Err("p-curves are not implemented for this face geometry".to_string()),
        }
    }

    /// Plane Face にp-curveを保持させる。
    pub fn attach_plane_pcurves(&mut self) -> Result<(), String> {
        self.pcurves = Some(self.derive_plane_pcurves()?);
        Ok(())
    }

    /// Plane Face にp-curveを保持した新しいFaceを返す。
    pub fn with_plane_pcurves(mut self) -> Result<Self, String> {
        self.attach_plane_pcurves()?;
        Ok(self)
    }

    /// NURBS Face に境界p-curveを保持させる。
    pub fn attach_nurbs_boundary_pcurves(
        &mut self,
        tol: &Tolerance,
        samples_per_edge: usize,
    ) -> Result<(), String> {
        self.pcurves = Some(self.derive_nurbs_boundary_pcurves(tol, samples_per_edge)?);
        Ok(())
    }

    /// NURBS Face に境界p-curveを保持した新しいFaceを返す。
    pub fn with_nurbs_boundary_pcurves(
        mut self,
        tol: &Tolerance,
        samples_per_edge: usize,
    ) -> Result<Self, String> {
        self.attach_nurbs_boundary_pcurves(tol, samples_per_edge)?;
        Ok(self)
    }

    /// Face 種別に応じて、p-curveが3D境界曲線と一致しているか検証する。
    pub fn validate_pcurves(
        &self,
        tol: &Tolerance,
        samples_per_edge: usize,
    ) -> Result<PcurveValidationReport, String> {
        let pcurves = self.pcurves(tol)?;
        let mut report = PcurveValidationReport {
            sampled_point_count: 0,
            mismatch_count: 0,
            max_distance: 0.0,
            errors: Vec::new(),
        };

        validate_face_pcurve_loop(
            self,
            "outer",
            &self.outer_wire,
            &pcurves.outer_loop,
            tol,
            samples_per_edge,
            &mut report,
        );

        for (idx, (wire, pcurve_loop)) in self
            .inner_wires
            .iter()
            .zip(pcurves.inner_loops.iter())
            .enumerate()
        {
            validate_face_pcurve_loop(
                self,
                &format!("inner {idx}"),
                wire,
                pcurve_loop,
                tol,
                samples_per_edge,
                &mut report,
            );
        }

        Ok(report)
    }

    /// Plane Face のp-curveが3D境界曲線と一致しているか検証する。
    pub fn validate_plane_pcurves(
        &self,
        tol: &Tolerance,
        samples_per_edge: usize,
    ) -> Result<PcurveValidationReport, String> {
        let FaceGeometry::Plane(plane) = &self.geometry else {
            return Err("Plane p-curves can only be validated for planar faces".to_string());
        };

        let pcurves = self.plane_pcurves()?;
        let mut report = PcurveValidationReport {
            sampled_point_count: 0,
            mismatch_count: 0,
            max_distance: 0.0,
            errors: Vec::new(),
        };

        validate_plane_pcurve_loop(
            plane,
            "outer",
            &self.outer_wire,
            &pcurves.outer_loop,
            tol,
            samples_per_edge,
            &mut report,
        );

        for (idx, (wire, pcurve_loop)) in self
            .inner_wires
            .iter()
            .zip(pcurves.inner_loops.iter())
            .enumerate()
        {
            validate_plane_pcurve_loop(
                plane,
                &format!("inner {idx}"),
                wire,
                pcurve_loop,
                tol,
                samples_per_edge,
                &mut report,
            );
        }

        Ok(report)
    }

    /// 境界ワイヤのサンプル点が支持曲面上に乗っているか検証する。
    pub fn validate_boundary_on_surface(
        &self,
        tol: &Tolerance,
        samples_per_edge: usize,
    ) -> FaceBoundaryValidationReport {
        let mut report = FaceBoundaryValidationReport {
            sampled_point_count: 0,
            off_surface_point_count: 0,
            max_distance: 0.0,
            errors: Vec::new(),
        };

        // 直線エッジも内部点まで見る。端点だけを見ると、曲面を横切る弦が
        // 「両端が面上にある」だけで通ってしまう。
        let mut loops = vec![(
            "outer".to_string(),
            dense_loop_points(&self.outer_wire, samples_per_edge),
        )];
        for (idx, wire) in self.inner_wires.iter().enumerate() {
            loops.push((
                format!("inner {idx}"),
                dense_loop_points(wire, samples_per_edge),
            ));
        }

        for (loop_name, points) in loops {
            for point in points {
                report.sampled_point_count += 1;
                let distance = self.distance_to_surface(point);
                report.max_distance = report.max_distance.max(distance);
                if distance > tol.linear {
                    report.off_surface_point_count += 1;
                    report.errors.push(format!(
                        "Face {} boundary point on {loop_name} loop is off surface by {distance:.6e}",
                        self.id
                    ));
                }
            }
        }

        report
    }

    fn distance_to_surface(&self, point: Point3) -> f64 {
        match &self.geometry {
            FaceGeometry::Plane(plane) => (point - plane.origin).dot(&plane.normal).abs(),
            FaceGeometry::Nurbs(surface) => {
                ExtremumEngine::point_to_surface(point, surface, 24, 1e-9)
                    .map(|projection| projection.distance)
                    .unwrap_or(f64::INFINITY)
            }
            FaceGeometry::Coons(surface) => sampled_surface_distance(point, surface, 16),
            FaceGeometry::Gordon(surface) => sampled_surface_distance(point, surface, 16),
            FaceGeometry::Triangular(surface) => sampled_surface_distance(point, surface, 16),
        }
    }
}

fn derive_wire_plane_pcurves(wire: &Wire, plane: &PlaneSurface3) -> Result<FacePcurveLoop, String> {
    let mut segments = Vec::with_capacity(wire.edges.len());

    for oriented_edge in &wire.edges {
        let mut control_points: Vec<ControlPoint2> = oriented_edge
            .edge
            .curve
            .control_points
            .iter()
            .map(|cp| ControlPoint2::new(project_to_plane_uv(cp.point, plane), cp.weight))
            .collect();
        let mut knots = oriented_edge.edge.curve.knots.clone();

        if !oriented_edge.orientation.is_forward() {
            control_points.reverse();
            knots = reverse_knot_vector(&knots, oriented_edge.edge.curve.degree);
        }

        let curve = NurbsCurve2::new(oriented_edge.edge.curve.degree, control_points, knots)?;
        segments.push(FacePcurveSegment {
            edge_id: oriented_edge.edge.id,
            orientation: oriented_edge.orientation,
            curve,
        });
    }

    Ok(FacePcurveLoop { segments })
}

fn derive_wire_nurbs_boundary_pcurves(
    wire: &Wire,
    surface: &NurbsSurface3,
    tol: &Tolerance,
    samples_per_edge: usize,
) -> Result<FacePcurveLoop, String> {
    let mut segments = Vec::with_capacity(wire.edges.len());

    for oriented_edge in &wire.edges {
        let curve = match_nurbs_boundary_pcurve(oriented_edge, surface, tol, samples_per_edge)?;
        segments.push(FacePcurveSegment {
            edge_id: oriented_edge.edge.id,
            orientation: oriented_edge.orientation,
            curve,
        });
    }

    Ok(FacePcurveLoop { segments })
}

fn match_nurbs_boundary_pcurve(
    edge: &crate::edge::OrientedEdge,
    surface: &NurbsSurface3,
    tol: &Tolerance,
    samples_per_edge: usize,
) -> Result<NurbsCurve2, String> {
    if let Ok(curve) = match_nurbs_outer_boundary_pcurve(edge, surface, tol, samples_per_edge) {
        return Ok(curve);
    }

    if let Ok(curve) = match_affine_patch_pcurve(edge, surface, tol) {
        return Ok(curve);
    }

    project_edge_to_nurbs_pcurve(edge, surface, tol, samples_per_edge)
}

/// A surface whose parameters map to space by an affine transform.
///
/// Converting a solid to B-splines turns every planar face into a patch like
/// this: degree one each way, four corners, no weights. The parameters are then
/// just coordinates in the plane, and there is nothing to approximate.
struct AffinePatch {
    origin: Point3,
    du: Vec3,
    dv: Vec3,
    u_min: f64,
    v_min: f64,
}

impl AffinePatch {
    /// The parameters naming `point`, or `None` when it is off the plane.
    fn parameters_of(&self, point: Point3, limit: f64) -> Option<Point2> {
        let relative = point - self.origin;
        let uu = self.du.dot(&self.du);
        let uv = self.du.dot(&self.dv);
        let vv = self.dv.dot(&self.dv);
        let ru = relative.dot(&self.du);
        let rv = relative.dot(&self.dv);

        let det = uu * vv - uv * uv;
        if det.abs() <= 1e-15 {
            return None;
        }
        let along_u = (ru * vv - rv * uv) / det;
        let along_v = (rv * uu - ru * uv) / det;

        // 面内に無い点は写せない。有理曲線の制御点は曲面上には無いが、
        // 平面の上には乗っているので、これで弾かれない。
        if (relative - self.du * along_u - self.dv * along_v).norm() > limit {
            return None;
        }

        Some(Point2::new(self.u_min + along_u, self.v_min + along_v))
    }
}

/// Reads a surface's affine map, and checks that it really follows one.
fn affine_patch(surface: &NurbsSurface3, tol: &Tolerance) -> Option<AffinePatch> {
    let ((u_min, u_max), (v_min, v_max)) = surface.param_range();
    let u_span = u_max - u_min;
    let v_span = v_max - v_min;
    if !(u_span > 0.0 && v_span > 0.0) {
        return None;
    }

    let origin = surface.evaluate(u_min, v_min);
    let du = (surface.evaluate(u_max, v_min) - origin) / u_span;
    let dv = (surface.evaluate(u_min, v_max) - origin) / v_span;

    // 三隅から取った写像が本当に曲面と一致するか、格子で確かめる。
    // 一致しなければアフィンではないので、近似の経路に任せる。
    let extent = du.norm().max(dv.norm()).max(1.0) * u_span.max(v_span);
    let limit = tol.linear.max(1e-9) * extent.max(1.0);
    for i in 0..=4 {
        for j in 0..=4 {
            let u = u_min + u_span * i as f64 / 4.0;
            let v = v_min + v_span * j as f64 / 4.0;
            let expected = origin + du * (u - u_min) + dv * (v - v_min);
            if (surface.evaluate(u, v) - expected).norm() > limit {
                return None;
            }
        }
    }

    Some(AffinePatch {
        origin,
        du,
        dv,
        u_min,
        v_min,
    })
}

/// The exact p-curve of an edge on a surface whose parameters are affine.
///
/// An affine map carries a NURBS curve to a NURBS curve of the same degree,
/// knots and weights, so the p-curve is the edge's own control points put
/// through the map. Nothing is sampled and nothing is approximated, which is
/// what the polyline path could not manage: a circle on such a patch came
/// through as an octagon, 0.889 off its own edge and a tenth short on area.
fn match_affine_patch_pcurve(
    edge: &crate::edge::OrientedEdge,
    surface: &NurbsSurface3,
    tol: &Tolerance,
) -> Result<NurbsCurve2, String> {
    let patch = affine_patch(surface, tol)
        .ok_or_else(|| "Surface parameters are not affine".to_string())?;

    let scale = edge
        .edge
        .curve
        .control_points
        .iter()
        .fold(0.0f64, |worst, cp| {
            worst.max((cp.point - patch.origin).norm())
        })
        .max(1.0);
    let limit = tol.linear.max(1e-9) * scale;

    let mut control_points: Vec<ControlPoint2> = Vec::with_capacity(
        edge.edge.curve.control_points.len(),
    );
    for cp in &edge.edge.curve.control_points {
        let uv = patch.parameters_of(cp.point, limit).ok_or_else(|| {
            format!("Edge {} leaves the plane of the patch", edge.edge.id)
        })?;
        control_points.push(ControlPoint2::new(uv, cp.weight));
    }

    let mut knots = edge.edge.curve.knots.clone();
    if !edge.orientation.is_forward() {
        control_points.reverse();
        knots = reverse_knot_vector(&knots, edge.edge.curve.degree);
    }

    let curve = NurbsCurve2::new(edge.edge.curve.degree, control_points, knots)?;

    // 主張で終わらせない。出来た p-curve を曲面に戻して、辺と一致するか測る。
    let (t_min, t_max) = curve.param_range();
    for step in 0..=16 {
        let fraction = step as f64 / 16.0;
        let uv = curve.evaluate(t_min + (t_max - t_min) * fraction);
        let from_surface = surface.evaluate(uv.x, uv.y);
        let from_edge = edge.evaluate_normalized(fraction);
        if (from_surface - from_edge).norm() > limit {
            return Err(format!(
                "Edge {} affine p-curve differs from the edge",
                edge.edge.id
            ));
        }
    }

    Ok(curve)
}

fn match_nurbs_outer_boundary_pcurve(
    edge: &crate::edge::OrientedEdge,
    surface: &NurbsSurface3,
    tol: &Tolerance,
    samples_per_edge: usize,
) -> Result<NurbsCurve2, String> {
    let ((u_min, u_max), (v_min, v_max)) = surface.param_range();
    let candidates = [
        (Point2::new(u_min, v_min), Point2::new(u_max, v_min)),
        (Point2::new(u_max, v_min), Point2::new(u_min, v_min)),
        (Point2::new(u_max, v_max), Point2::new(u_min, v_max)),
        (Point2::new(u_min, v_max), Point2::new(u_max, v_max)),
        (Point2::new(u_min, v_max), Point2::new(u_min, v_min)),
        (Point2::new(u_min, v_min), Point2::new(u_min, v_max)),
        (Point2::new(u_max, v_min), Point2::new(u_max, v_max)),
        (Point2::new(u_max, v_max), Point2::new(u_max, v_min)),
    ];

    // 候補は「パラメータ空間の直線」なので、辺のパラメータ付けがその直線に
    // アフィンに乗っているときしか正しくない。ここは構成と同じ分割数で
    // 測っているので、乗っていない辺が節点だけで一致して通る余地がある。
    // その場合でも 37 点で測り直す `validate_pcurves` が落とすため、誤答には
    // ならない（明示的なエラーになる）。実際にそれで落ちる検体はまだ
    // 見つかっていないので、直す根拠が測れるまで触らない。
    let samples = samples_per_edge.max(2);
    let mut best: Option<(f64, Point2, Point2)> = None;

    for (uv0, uv1) in candidates {
        let mut max_distance: f64 = 0.0;
        for i in 0..=samples {
            let t = i as f64 / samples as f64;
            let uv = uv0 + (uv1 - uv0) * t;
            let surface_point = surface.evaluate(uv.x, uv.y);
            let edge_point = edge.evaluate_normalized(t);
            max_distance = max_distance.max((surface_point - edge_point).norm());
        }

        if best
            .as_ref()
            .map(|(best_distance, _, _)| max_distance < *best_distance)
            .unwrap_or(true)
        {
            best = Some((max_distance, uv0, uv1));
        }
    }

    let Some((distance, uv0, uv1)) = best else {
        return Err("No NURBS boundary candidate was evaluated".to_string());
    };

    if distance > tol.linear.max(1e-6) * 10.0 {
        return Err(format!(
            "Edge {} is not on a NURBS iso-boundary; max distance {distance:.6e}",
            edge.edge.id
        ));
    }

    NurbsCurve2::bspline_from_points(1, vec![uv0, uv1])
}

/// Projects a 3D edge onto a NURBS surface as a polyline in parameter space.
///
/// Two things decide whether this is any good, and only one of them used to be
/// checked. Even spacing confirmed that the sampled points sat on the surface,
/// and never that the straight run between two of them followed the edge: a
/// circle came through as an octagon, every corner exactly on the surface and
/// the trimmed region a tenth too small. So the samples are placed where the
/// curve needs them, judged by how far the chord strays from the edge, which is
/// the same quantity validation measures.
///
/// The knots are the edge parameters the samples were taken at, so a given
/// fraction along the p-curve is the same fraction along the edge. Spacing the
/// knots evenly while spacing the samples unevenly breaks that correspondence,
/// and the p-curve then reads as wrong even where it is right.
fn project_edge_to_nurbs_pcurve(
    edge: &crate::edge::OrientedEdge,
    surface: &NurbsSurface3,
    tol: &Tolerance,
    samples_per_edge: usize,
) -> Result<NurbsCurve2, String> {
    let on_surface_limit = tol.linear.max(1e-6) * 10.0;
    let mut max_distance: f64 = 0.0;

    let mut project = |t: f64, seed: Option<Point2>| -> Result<Point2, String> {
        let point = edge.evaluate_normalized(t);
        let projection = match seed {
            Some(uv) => ExtremumEngine::point_to_surface_seeded(
                point,
                surface,
                uv.x,
                uv.y,
                32,
                tol.parametric,
            )?,
            None => ExtremumEngine::point_to_surface(point, surface, 32, tol.parametric)?,
        };
        max_distance = max_distance.max(projection.distance);
        Ok(Point2::new(projection.u, projection.v))
    };

    let start = samples_per_edge.max(4);
    let mut parameters: Vec<f64> = (0..=start).map(|i| i as f64 / start as f64).collect();
    let mut uv_points: Vec<Point2> = Vec::with_capacity(parameters.len());
    for t in &parameters {
        uv_points.push(project(*t, None)?);
    }
    settle_seam_parameters(&mut uv_points, surface, tol);

    // 弦の中点が辺から離れている区間を割る。継ぎ目をまたぐ区間はここでは
    // 詰められないので、割らずに残す。
    //
    // 走査は**通しで何度も回す**。1回の通しで各区間はたかだか1度しか割らない
    // （割った点の先へ進む）ので、詰まらない区間が予算を独り占めしない。
    //
    // 以前は1本の走査で、割った直後に同じ左半分を見直していた。辺自身が曲面
    // から公差ぎりぎり（実測 9.78e-7）離れている区間は、いくら割っても弦が
    // 公差 1e-6 を切れない。そこで **最初の区間だけを 4096 点の上限まで割り
    // 続け、残りの区間は最初の8分割のまま**という p-curve が出来ていた。
    // 3D の辺から最大 8.5e-3 離れる。皿モミ穴は 64 組中 7 組がこれで落ちて
    // いた（`countersink_range_probe`）。
    let deflection = tol.linear;
    const MAX_POINTS: usize = 4096;
    const MAX_PASSES: usize = 24;
    for _pass in 0..MAX_PASSES {
        let mut split_any = false;
        let mut index = 0;
        while index + 1 < parameters.len() && parameters.len() < MAX_POINTS {
            let (t0, t1) = (parameters[index], parameters[index + 1]);
            let middle = (t0 + t1) * 0.5;
            let chord = uv_points[index] + (uv_points[index + 1] - uv_points[index]) * 0.5;
            let strayed =
                (surface.evaluate(chord.x, chord.y) - edge.evaluate_normalized(middle)).norm();

            if strayed <= deflection || (t1 - t0) <= 1e-9 {
                index += 1;
                continue;
            }

            let uv = project(middle, Some(chord))?;
            // 継ぎ目をまたぐ区間は、割っても弦が縮まない。無限に割らないよう抜ける。
            // パラメータ空間で湾曲する曲線（有理パッチ上の直線など）の膨らみを
            // 誤認してスキップしないよう、区間長に応じたマージンを設ける。
            let before = uv_points[index];
            let after = uv_points[index + 1];
            let margin =
                ((after.x - before.x).abs().max((after.y - before.y).abs()) * 0.5).max(1e-4);
            let inside = uv.x >= before.x.min(after.x) - margin
                && uv.x <= before.x.max(after.x) + margin
                && uv.y >= before.y.min(after.y) - margin
                && uv.y <= before.y.max(after.y) + margin;
            if !inside {
                index += 1;
                continue;
            }

            parameters.insert(index + 1, middle);
            uv_points.insert(index + 1, uv);
            split_any = true;
            // 割ってできた点の先へ進む。両側の半分は次の通しで見る。
            index += 2;
        }

        if !split_any || parameters.len() >= MAX_POINTS {
            break;
        }
    }

    if max_distance > on_surface_limit {
        return Err(format!(
            "Edge {} projection to NURBS surface exceeds tolerance; max distance {max_distance:.6e}",
            edge.edge.id
        ));
    }

    // 1次のクランプ節点列。節点をとった辺のパラメータをそのまま使う。
    let mut knots = Vec::with_capacity(parameters.len() + 2);
    knots.push(parameters[0]);
    knots.extend(parameters.iter().copied());
    knots.push(parameters[parameters.len() - 1]);

    NurbsCurve2::new(1, uv_points.into_iter().map(ControlPoint2::unweighted).collect(), KnotVector::new(knots))
}

/// Rewrites parameters that landed on a seam so the run reads as one path.
///
/// A point on the seam of a closed surface has two names, one at each end of
/// the domain, and the projection has no reason to prefer either. When a run of
/// samples picks the far one for its first point and the near one for the rest,
/// the straight line joining them in parameter space crosses the whole domain,
/// and the p-curve sweeps right round the surface between two neighbouring
/// samples. That is where the twenty-unit gap on a radius-ten sphere came from:
/// not an approximation, but a curve going the wrong way round.
///
/// Samples away from a seam are unambiguous, so they are the ones to trust; a
/// seam sample is moved to whichever end sits nearer them. A run that is all
/// seam is already consistent and is left alone.
fn settle_seam_parameters(uv_points: &mut [Point2], surface: &NurbsSurface3, tol: &Tolerance) {
    let ((u_min, u_max), (v_min, v_max)) = surface.param_range();
    settle_seam_axis(uv_points, u_min, u_max, tol, surface, true);
    settle_seam_axis(uv_points, v_min, v_max, tol, surface, false);
}

fn settle_seam_axis(
    uv_points: &mut [Point2],
    min: f64,
    max: f64,
    tol: &Tolerance,
    surface: &NurbsSurface3,
    along_u: bool,
) {
    let span = max - min;
    if span <= 0.0 || uv_points.len() < 2 {
        return;
    }
    let edge_of_domain = span * 1e-9;
    let coordinate = |uv: &Point2| if along_u { uv.x } else { uv.y };

    let at_seam: Vec<bool> = uv_points
        .iter()
        .map(|uv| {
            let value = coordinate(uv);
            (value - min).abs() <= edge_of_domain || (value - max).abs() <= edge_of_domain
        })
        .collect();

    // 端に居ない標本が一つも無ければ、寄せる先が無い。全部が継ぎ目上なら
    // すでに揃っている。
    if at_seam.iter().all(|seam| *seam) || at_seam.iter().all(|seam| !*seam) {
        return;
    }

    for index in 0..uv_points.len() {
        if !at_seam[index] {
            continue;
        }
        // 一番近い、端に居ない標本を基準にする。
        let Some(reference) = (0..uv_points.len())
            .filter(|other| !at_seam[*other])
            .min_by_key(|other| other.abs_diff(index))
            .map(|other| coordinate(&uv_points[other]))
        else {
            continue;
        };

        let nearer = if (reference - min).abs() <= (reference - max).abs() {
            min
        } else {
            max
        };
        let current = coordinate(&uv_points[index]);
        if (current - nearer).abs() <= edge_of_domain {
            continue;
        }

        // 両端が同じ点を指していることを確かめてから置き換える。継ぎ目でない
        // 端をまたいでしまうと、面から外れた p-curve になる。
        let other_end = uv_points[index];
        let moved = if along_u {
            Point2::new(nearer, other_end.y)
        } else {
            Point2::new(other_end.x, nearer)
        };
        let before = surface.evaluate(other_end.x, other_end.y);
        let after = surface.evaluate(moved.x, moved.y);
        if (after - before).norm() > tol.linear.max(1e-9) {
            continue;
        }

        uv_points[index] = moved;
    }
}

fn project_to_plane_uv(point: Point3, plane: &PlaneSurface3) -> Point2 {
    let rel = point - plane.origin;
    let uu = plane.u_axis.dot(&plane.u_axis);
    let uv = plane.u_axis.dot(&plane.v_axis);
    let vv = plane.v_axis.dot(&plane.v_axis);
    let ru = rel.dot(&plane.u_axis);
    let rv = rel.dot(&plane.v_axis);
    let det = uu * vv - uv * uv;

    if det.abs() <= 1e-15 {
        return Point2::new(0.0, 0.0);
    }

    Point2::new((ru * vv - rv * uv) / det, (rv * uu - ru * uv) / det)
}

fn reverse_knot_vector(knots: &KnotVector, degree: usize) -> KnotVector {
    let u_min = knots.start_param(degree);
    let num_ctrl_pts = knots.knots.len() - degree - 1;
    let u_max = knots.end_param(num_ctrl_pts);
    let reversed = knots
        .knots
        .iter()
        .rev()
        .map(|k| u_min + u_max - *k)
        .collect();
    KnotVector::new(reversed)
}

fn validate_plane_pcurve_loop(
    plane: &PlaneSurface3,
    loop_name: &str,
    wire: &Wire,
    pcurve_loop: &FacePcurveLoop,
    tol: &Tolerance,
    samples_per_edge: usize,
    report: &mut PcurveValidationReport,
) {
    if wire.edges.len() != pcurve_loop.segments.len() {
        report.errors.push(format!(
            "{loop_name} loop has {} edges but {} p-curve segments",
            wire.edges.len(),
            pcurve_loop.segments.len()
        ));
        report.mismatch_count += 1;
        return;
    }

    let samples = samples_per_edge.max(2);
    for (edge_index, (edge, pcurve)) in wire
        .edges
        .iter()
        .zip(pcurve_loop.segments.iter())
        .enumerate()
    {
        let (t_min, t_max) = pcurve.curve.param_range();
        for i in 0..=samples {
            let normalized_t = i as f64 / samples as f64;
            let pcurve_t = t_min + normalized_t * (t_max - t_min);
            let uv = pcurve.curve.evaluate(pcurve_t);
            let point_from_pcurve = plane.evaluate(uv.x, uv.y);
            let point_from_edge = edge.evaluate_normalized(normalized_t);
            let distance = (point_from_pcurve - point_from_edge).norm();

            report.sampled_point_count += 1;
            report.max_distance = report.max_distance.max(distance);

            if distance > tol.linear {
                report.mismatch_count += 1;
                report.errors.push(format!(
                    "{loop_name} loop edge {edge_index} p-curve differs from 3D edge by {distance:.6e}"
                ));
            }
        }
    }
}

fn validate_face_pcurve_loop(
    face: &Face,
    loop_name: &str,
    wire: &Wire,
    pcurve_loop: &FacePcurveLoop,
    tol: &Tolerance,
    samples_per_edge: usize,
    report: &mut PcurveValidationReport,
) {
    if wire.edges.len() != pcurve_loop.segments.len() {
        report.errors.push(format!(
            "{loop_name} loop has {} edges but {} p-curve segments",
            wire.edges.len(),
            pcurve_loop.segments.len()
        ));
        report.mismatch_count += 1;
        return;
    }

    let samples = samples_per_edge.max(2);
    for (edge_index, (edge, pcurve)) in wire
        .edges
        .iter()
        .zip(pcurve_loop.segments.iter())
        .enumerate()
    {
        let (t_min, t_max) = pcurve.curve.param_range();
        for i in 0..=samples {
            let normalized_t = i as f64 / samples as f64;
            let pcurve_t = t_min + normalized_t * (t_max - t_min);
            let uv = pcurve.curve.evaluate(pcurve_t);
            let point_from_pcurve = match &face.geometry {
                FaceGeometry::Plane(plane) => plane.evaluate(uv.x, uv.y),
                FaceGeometry::Nurbs(surface) => surface.evaluate(uv.x, uv.y),
                _ => {
                    report.errors.push(
                        "p-curve validation is not implemented for this face geometry".to_string(),
                    );
                    report.mismatch_count += 1;
                    return;
                }
            };
            let point_from_edge = edge.evaluate_normalized(normalized_t);
            let distance = (point_from_pcurve - point_from_edge).norm();

            report.sampled_point_count += 1;
            report.max_distance = report.max_distance.max(distance);

            if distance > tol.linear.max(1e-6) * 10.0 {
                report.mismatch_count += 1;
                report.errors.push(format!(
                    "{loop_name} loop edge {edge_index} p-curve differs from 3D edge by {distance:.6e}"
                ));
            }
        }
    }
}

fn sampled_surface_distance<S: Surface3>(point: Point3, surface: &S, samples: usize) -> f64 {
    let ((u_min, u_max), (v_min, v_max)) = surface.param_range();
    let mut min_distance = f64::INFINITY;
    let steps = samples.max(2);

    for i in 0..=steps {
        let u = u_min + (i as f64 / steps as f64) * (u_max - u_min);
        for j in 0..=steps {
            let v = v_min + (j as f64 / steps as f64) * (v_max - v_min);
            let distance = (surface.evaluate(u, v) - point).norm();
            min_distance = min_distance.min(distance);
        }
    }

    min_distance
}

/// Samples every edge of a loop at interior parameters as well as its ends.
///
/// `Wire::sample_points` shortcuts linear edges to their endpoints, which is
/// right for display but hides a chord drawn across a curved face.
fn dense_loop_points(wire: &Wire, samples_per_edge: usize) -> Vec<Point3> {
    let steps = samples_per_edge.max(2);
    let mut points = Vec::with_capacity(wire.edges.len() * (steps + 1));
    for edge in &wire.edges {
        for step in 0..=steps {
            points.push(edge.evaluate_normalized(step as f64 / steps as f64));
        }
    }
    points
}

/// Whether two edge uses run along the same curve, either way round.
///
/// The midpoint is what separates two edges that share both vertices: a torus
/// written as one face has a seam the long way round and a seam the short way,
/// and both begin and end at the same point.
fn same_edge_geometry(a: &OrientedEdge, b: &OrientedEdge, tol: f64) -> bool {
    if (a.evaluate_normalized(0.5) - b.evaluate_normalized(0.5)).norm() > tol {
        return false;
    }
    let (a_start, a_end) = (a.start_vertex().point, a.end_vertex().point);
    let (b_start, b_end) = (b.start_vertex().point, b.end_vertex().point);
    let close = |left: Point3, right: Point3| (left - right).norm() <= tol;
    close(a_start, b_start) && close(a_end, b_end)
        || close(a_start, b_end) && close(a_end, b_start)
}
