use zenith_geom::Surface3;
use zenith_math::{Point2, Point3, Vec3};
use zenith_tess::{face_uv_triangulation, TessellationParams, TriangleMesh};
use zenith_topo::{Face, FaceGeometry, FacePcurveLoop, Shell, Solid};

/// 幾何特性・物性値（体積、表面積、重心、慣性モーメント）
#[derive(Debug, Clone, PartialEq)]
pub struct MassProperties {
    /// 表面積 (mm^2)
    pub surface_area: f64,
    /// 体積 (mm^3)
    pub volume: f64,
    /// 重心座標 (mm)
    pub center_of_mass: Point3,
    /// 慣性モーメント主成分 (Ixx, Iyy, Izz) (mm^5 または密度1時の単位)
    pub inertia_diagonal: Vec3,
}

/// ガウスの発散定理に基づく高精度物性値計算エンジン
pub struct MassCalculator;

/// Symmetric degree-4 quadrature rule on the unit triangle.
///
/// Barycentric coordinates and weights that sum to one, so an integral over a
/// triangle is `area * sum(w_i * f(p_i))`. Degree 4 means the rule is exact for
/// quadratics and cubics, which is what makes a moderately refined domain reach
/// analytic accuracy instead of the mesh's linear error.
const TRIANGLE_QUADRATURE: [(f64, f64, f64, f64); 6] = [
    (
        0.445948490915965,
        0.445948490915965,
        0.108103018168070,
        0.223381589678011,
    ),
    (
        0.445948490915965,
        0.108103018168070,
        0.445948490915965,
        0.223381589678011,
    ),
    (
        0.108103018168070,
        0.445948490915965,
        0.445948490915965,
        0.223381589678011,
    ),
    (
        0.091576213509771,
        0.091576213509771,
        0.816847572980459,
        0.109951743655322,
    ),
    (
        0.091576213509771,
        0.816847572980459,
        0.091576213509771,
        0.109951743655322,
    ),
    (
        0.816847572980459,
        0.091576213509771,
        0.091576213509771,
        0.109951743655322,
    ),
];

impl MassCalculator {
    /// Computes mass properties by integrating over the B-Rep faces themselves.
    ///
    /// The surface is evaluated inside each parameter-domain triangle rather
    /// than replaced by its chord, so accuracy comes from the exact geometry
    /// instead of from how finely the display mesh happens to be tessellated.
    /// `compute_from_mesh` stays available as the preview path.
    pub fn compute_from_brep(solid: &Solid, params: &TessellationParams) -> MassProperties {
        let mut accumulator = SurfaceIntegral::default();
        accumulator.add_shell(&solid.outer_shell, params, 1.0);
        // 空洞シェルは外殻と同じ向きで保持されるため、寄与を反転して足す
        for inner in &solid.inner_shells {
            accumulator.add_shell(inner, params, -1.0);
        }

        accumulator.finish()
    }

    /// Integrates a single face, returning its area and its contribution to the
    /// enclosed volume.
    pub fn compute_face_integral(face: &Face, params: &TessellationParams) -> (f64, f64) {
        let mut accumulator = SurfaceIntegral::default();
        accumulator.add_face(face, params, 1.0);
        (accumulator.area, accumulator.volume)
    }

    /// テッセレーションメッシュから厳密な幾何特性・物性値を計算（発散定理・四面体積分）
    pub fn compute_from_mesh(mesh: &TriangleMesh) -> MassProperties {
        let mut total_area = 0.0;
        let mut total_vol = 0.0;
        let mut cx_sum = 0.0;
        let mut cy_sum = 0.0;
        let mut cz_sum = 0.0;

        let mut ixx = 0.0;
        let mut iyy = 0.0;
        let mut izz = 0.0;

        for tri in &mesh.indices {
            let p0 = mesh.positions[tri[0] as usize];
            let p1 = mesh.positions[tri[1] as usize];
            let p2 = mesh.positions[tri[2] as usize];

            // 1. 表面積
            let cross = (p1 - p0).cross(&(p2 - p0));
            let area = 0.5 * cross.norm();
            total_area += area;

            // 2. 符号付き体積 (Signed Volume of Tetrahedron with origin)
            let det = p0.x * (p1.y * p2.z - p1.z * p2.y) - p0.y * (p1.x * p2.z - p1.z * p2.x)
                + p0.z * (p1.x * p2.y - p1.y * p2.x);
            let vol = det / 6.0;
            total_vol += vol;

            // 3. 重心寄与
            cx_sum += (p0.x + p1.x + p2.x) * vol * 0.25;
            cy_sum += (p0.y + p1.y + p2.y) * vol * 0.25;
            cz_sum += (p0.z + p1.z + p2.z) * vol * 0.25;

            // 4. 慣性モーメント寄与 (各四面体の2次モーメント)
            let x2 =
                p0.x * p0.x + p1.x * p1.x + p2.x * p2.x + p0.x * p1.x + p1.x * p2.x + p2.x * p0.x;
            let y2 =
                p0.y * p0.y + p1.y * p1.y + p2.y * p2.y + p0.y * p1.y + p1.y * p2.y + p2.y * p0.y;
            let z2 =
                p0.z * p0.z + p1.z * p1.z + p2.z * p2.z + p0.z * p1.z + p1.z * p2.z + p2.z * p0.z;

            ixx += vol * (y2 + z2) / 10.0;
            iyy += vol * (x2 + z2) / 10.0;
            izz += vol * (x2 + y2) / 10.0;
        }

        let total_vol_abs = total_vol.abs();
        let (cm_x, cm_y, cm_z) = if total_vol_abs > 1e-12 {
            (cx_sum / total_vol, cy_sum / total_vol, cz_sum / total_vol)
        } else {
            (0.0, 0.0, 0.0)
        };

        MassProperties {
            surface_area: total_area,
            volume: total_vol_abs,
            center_of_mass: Point3::new(cm_x, cm_y, cm_z),
            inertia_diagonal: Vec3::new(ixx.abs(), iyy.abs(), izz.abs()),
        }
    }
}

/// パラメータ三角形を、またいでいるノット線で割る。
///
/// 割った断片は元の三角形をちょうど覆う（重なりも隙間もない）。切り口は
/// 直線なので、面積の和は元と厳密に等しい。またいでいなければ何もしない
/// ので、内部ノットを持たない曲面には費用が乗らない。
fn split_triangle_at_breaks(
    triangle: [Point2; 3],
    u_breaks: &[f64],
    v_breaks: &[f64],
    out: &mut Vec<[Point2; 3]>,
) {
    let mut current = vec![triangle];
    for (axis, breaks) in [(0usize, u_breaks), (1usize, v_breaks)] {
        for value in breaks {
            let mut next = Vec::with_capacity(current.len());
            for piece in current.drain(..) {
                split_triangle_on_axis(piece, axis, *value, &mut next);
            }
            current = next;
        }
    }
    out.append(&mut current);
}

fn split_triangle_on_axis(
    triangle: [Point2; 3],
    axis: usize,
    value: f64,
    out: &mut Vec<[Point2; 3]>,
) {
    let coordinate = |point: &Point2| if axis == 0 { point.x } else { point.y };
    let signed: [f64; 3] = [
        coordinate(&triangle[0]) - value,
        coordinate(&triangle[1]) - value,
        coordinate(&triangle[2]) - value,
    ];
    // またいでいなければ触らない。境界に乗っているだけの頂点は「またぎ」に
    // 数えない——幅ゼロの断片を作るだけだからである。
    let has_below = signed.iter().any(|d| *d < 0.0);
    let has_above = signed.iter().any(|d| *d > 0.0);
    if !(has_below && has_above) {
        out.push(triangle);
        return;
    }

    let mut below: Vec<Point2> = Vec::with_capacity(4);
    let mut above: Vec<Point2> = Vec::with_capacity(4);
    for index in 0..3 {
        let next = (index + 1) % 3;
        let (a, b) = (triangle[index], triangle[next]);
        let (da, db) = (signed[index], signed[next]);

        if da <= 0.0 {
            below.push(a);
        }
        if da >= 0.0 {
            above.push(a);
        }
        if (da < 0.0 && db > 0.0) || (da > 0.0 && db < 0.0) {
            let t = da / (da - db);
            let crossing = a + (b - a) * t;
            below.push(crossing);
            above.push(crossing);
        }
    }

    fan(&below, out);
    fan(&above, out);
}

fn fan(polygon: &[Point2], out: &mut Vec<[Point2; 3]>) {
    if polygon.len() < 3 {
        return;
    }
    for index in 1..polygon.len() - 1 {
        out.push([polygon[0], polygon[index], polygon[index + 1]]);
    }
}

/// 曲面が `(u, v)` についてアフィンなら、その枠 `(origin, u_axis, v_axis)` を返す。
///
/// `p(u, v) = origin + u * u_axis + v * v_axis` が全域で成り立つときだけ、
/// グリーンの定理で領域積分を境界の線積分に落とせる。落とせれば、円形の
/// トリム境界は内接多角形ではなく真の値に積まれる。
///
/// **次数で判定してはいけません。** 1x1 次のパッチは一般には双1次で、
/// `uv` の交差項を持つ。4隅が平行四辺形でなければ位置はアフィンではなく、
/// 定理の前提が崩れる。有理パッチも重みが一定でなければアフィンではない。
/// どちらも「1次だから平面のはず」を通すと静かに間違う。格子の**全点**で
/// 測って確かめる。角だけを見ると、双1次のパッチは4隅では必ず一致する。
fn affine_frame_of(surface: &impl Surface3) -> Option<(Point3, Vec3, Vec3)> {
    let ((u_min, u_max), (v_min, v_max)) = surface.param_range();
    if !(u_min.is_finite() && u_max.is_finite() && v_min.is_finite() && v_max.is_finite()) {
        return None;
    }
    let u_span = u_max - u_min;
    let v_span = v_max - v_min;
    if u_span <= 0.0 || v_span <= 0.0 {
        return None;
    }

    let corner = surface.evaluate(u_min, v_min);
    let u_axis = (surface.evaluate(u_max, v_min) - corner) / u_span;
    let v_axis = (surface.evaluate(u_min, v_max) - corner) / v_span;
    if u_axis.cross(&v_axis).norm() <= f64::EPSILON {
        return None;
    }
    let origin = corner - u_axis * u_min - v_axis * v_min;

    // 許容はパッチの実寸に対する比で決める。絶対値で決めると、大きな面では
    // 甘く、小さな面では通らない。
    let extent = (u_axis.norm() * u_span).max(v_axis.norm() * v_span);
    let limit = extent * 1e-12;

    // 標本数は次数やノットと互いに素になるように選ぶ。等分点は制御点や
    // ノットに当たりやすく、そこは構成上一致することがある。
    const U_SAMPLES: usize = 17;
    const V_SAMPLES: usize = 19;
    for i in 0..=U_SAMPLES {
        let u = u_min + u_span * (i as f64) / (U_SAMPLES as f64);
        for j in 0..=V_SAMPLES {
            let v = v_min + v_span * (j as f64) / (V_SAMPLES as f64);
            let predicted = origin + u_axis * u + v_axis * v;
            if (surface.evaluate(u, v) - predicted).norm() > limit {
                return None;
            }
        }
    }

    Some((origin, u_axis, v_axis))
}

/// Running totals of the divergence-theorem surface integrals.
#[derive(Debug, Default, Clone, Copy)]
struct SurfaceIntegral {
    area: f64,
    volume: f64,
    moment: Vec3,
    second_moment: Vec3,
}

impl SurfaceIntegral {
    fn add_shell(&mut self, shell: &Shell, params: &TessellationParams, sign: f64) {
        for face in &shell.faces {
            self.add_face(face, params, sign);
        }
    }

    fn add_face(&mut self, face: &Face, params: &TessellationParams, sign: f64) {
        zenith_geom::work_counter::count_face_integral();
        let orientation_sign = if face.orientation.is_forward() {
            sign
        } else {
            -sign
        };
        match &face.geometry {
            FaceGeometry::Plane(surface) => {
                // 平面はトリム境界の線積分で解析的に積める
                if self.add_affine_face(
                    face,
                    surface.origin,
                    surface.u_axis,
                    surface.v_axis,
                    orientation_sign,
                ) {
                    return;
                }
                self.add_surface(face, surface, params, orientation_sign)
            }
            FaceGeometry::Nurbs(surface) => {
                // 他カーネルは平面を 1x1 次のパッチとして書くことがある。
                // 幾何は平面なのに種別が Nurbs なので、そのままでは下の
                // 求積に落ち、トリム境界が弦の折れ線になる。円形のキャップは
                // 内接多角形として積まれ、**分割数を上げても動かない**
                // 一方向の不足が残る（実測 -2.56e-5、16分割でも512分割でも
                // 小数9桁目まで同じ値）。厳密な経路は既にあるので、そこへ通す。
                if face.pcurves.is_some() {
                    if let Some((origin, u_axis, v_axis)) = affine_frame_of(surface) {
                        if self.add_affine_face(face, origin, u_axis, v_axis, orientation_sign) {
                            return;
                        }
                    }
                }
                self.add_surface(face, surface, params, orientation_sign)
            }
            FaceGeometry::Coons(surface) => {
                self.add_surface(face, surface, params, orientation_sign)
            }
            FaceGeometry::Gordon(surface) => {
                self.add_surface(face, surface, params, orientation_sign)
            }
            FaceGeometry::Triangular(surface) => {
                self.add_surface(face, surface, params, orientation_sign)
            }
        }
    }

    /// Integrates a planar face analytically over its trim loops.
    ///
    /// On a plane the position is affine in `(u, v)`, so every integrand the
    /// divergence theorem needs is polynomial there, and Green's theorem turns
    /// the domain integrals into line integrals along the p-curves. That removes
    /// the polygonal approximation of curved trim boundaries entirely: a circular
    /// cap integrates to its true area, not to an inscribed polygon's.
    ///
    /// Returns false when the face has no usable p-curves, so the caller can
    /// fall back to domain quadrature.
    fn add_affine_face(
        &mut self,
        face: &Face,
        origin: Point3,
        u_axis: Vec3,
        v_axis: Vec3,
        orientation_sign: f64,
    ) -> bool {
        let Ok(pcurves) = face.plane_pcurves() else {
            return false;
        };

        let mut moments = loop_uv_moments(&pcurves.outer_loop);
        let outer_area = moments[index_uv(0, 0)];
        if outer_area.abs() <= f64::EPSILON {
            return false;
        }
        for hole in &pcurves.inner_loops {
            let hole_moments = loop_uv_moments(hole);
            // 穴は外周と逆符号で効かなければならない
            let sign = if hole_moments[index_uv(0, 0)].signum() == outer_area.signum() {
                -1.0
            } else {
                1.0
            };
            for (total, part) in moments.iter_mut().zip(hole_moments.iter()) {
                *total += part * sign;
            }
        }
        // ループの回り方に依らず、領域積分が正になるよう正規化する
        let normalization = outer_area.signum();
        for moment in moments.iter_mut() {
            *moment *= normalization;
        }

        let jacobian_vector = u_axis.cross(&v_axis);
        let jacobian = jacobian_vector.norm();
        if jacobian <= f64::EPSILON {
            return false;
        }
        let normal = jacobian_vector / jacobian * orientation_sign;

        let area = jacobian * moments[index_uv(0, 0)];
        self.area += area;
        self.volume += origin.coords.dot(&normal) * area / 3.0;

        for axis in 0..3 {
            let base = origin.coords[axis];
            let du = u_axis[axis];
            let dv = v_axis[axis];

            let squared = base * base * moments[index_uv(0, 0)]
                + du * du * moments[index_uv(2, 0)]
                + dv * dv * moments[index_uv(0, 2)]
                + 2.0 * base * du * moments[index_uv(1, 0)]
                + 2.0 * base * dv * moments[index_uv(0, 1)]
                + 2.0 * du * dv * moments[index_uv(1, 1)];
            let cubed = base * base * base * moments[index_uv(0, 0)]
                + 3.0 * base * base * (du * moments[index_uv(1, 0)] + dv * moments[index_uv(0, 1)])
                + 3.0
                    * base
                    * (du * du * moments[index_uv(2, 0)]
                        + 2.0 * du * dv * moments[index_uv(1, 1)]
                        + dv * dv * moments[index_uv(0, 2)])
                + du * du * du * moments[index_uv(3, 0)]
                + 3.0 * du * du * dv * moments[index_uv(2, 1)]
                + 3.0 * du * dv * dv * moments[index_uv(1, 2)]
                + dv * dv * dv * moments[index_uv(0, 3)];

            self.moment[axis] += normal[axis] * jacobian * squared / 2.0;
            self.second_moment[axis] += normal[axis] * jacobian * cubed / 3.0;
        }

        true
    }

    fn add_surface(
        &mut self,
        face: &Face,
        surface: &impl Surface3,
        params: &TessellationParams,
        orientation_sign: f64,
    ) {
        let domain = face_uv_triangulation(face, params);
        // 三角形がノット区間をまたいでいたら、そこで割ってから当てる。
        // B-spline が滑らかなのは区間の内側だけで、またいだまま次数4の
        // 則を当てても次数が効かない。トリムされた面の三角形は earcut と
        // 細分で作られていて、ノット線を知らない。
        //
        // **セル線でも切ろうとして、二度外しました。** 経緯は 4-21 に。
        let (u_breaks, v_breaks) = surface.integration_breaks();
        let mut pieces: Vec<[Point2; 3]> = Vec::new();
        for triangle in &domain.triangles {
            let corners = [
                domain.uvs[triangle[0]],
                domain.uvs[triangle[1]],
                domain.uvs[triangle[2]],
            ];
            split_triangle_at_breaks(corners, &u_breaks, &v_breaks, &mut pieces);
        }

        for corners in &pieces {
            let corners = *corners;
            let parametric_area = 0.5
                * ((corners[1] - corners[0]).x * (corners[2] - corners[0]).y
                    - (corners[1] - corners[0]).y * (corners[2] - corners[0]).x);
            if parametric_area.abs() <= f64::EPSILON {
                continue;
            }

            for (l0, l1, l2, weight) in TRIANGLE_QUADRATURE {
                let uv = Point2::from(
                    corners[0].coords * l0 + corners[1].coords * l1 + corners[2].coords * l2,
                );
                let (point, du, dv) = surface.evaluate_with_derivatives(uv.x, uv.y);
                let jacobian = du.cross(&dv);
                if !jacobian.x.is_finite() || !jacobian.y.is_finite() || !jacobian.z.is_finite() {
                    continue;
                }

                // 面素ベクトル: パラメータ三角形の向きと面の向きを両方反映する
                // 三角化の向きは任意なので大きさだけ使い、向きは面の向きで決める
                let scale = weight * parametric_area.abs() * orientation_sign;
                let area_vector = jacobian * scale;
                self.area += jacobian.norm() * weight * parametric_area.abs();
                self.volume += point.coords.dot(&area_vector) / 3.0;
                self.moment += Vec3::new(
                    point.x * point.x * area_vector.x,
                    point.y * point.y * area_vector.y,
                    point.z * point.z * area_vector.z,
                ) / 2.0;
                self.second_moment += Vec3::new(
                    point.x * point.x * point.x * area_vector.x,
                    point.y * point.y * point.y * area_vector.y,
                    point.z * point.z * point.z * area_vector.z,
                ) / 3.0;
            }
        }
    }

    fn finish(self) -> MassProperties {
        let center_of_mass = if self.volume.abs() > 1e-12 {
            Point3::from(self.moment / self.volume)
        } else {
            Point3::new(0.0, 0.0, 0.0)
        };

        // Ixx = ∫(y^2 + z^2) dV を、発散定理で得た ∫x^3 n_x/3 などから組み立てる
        let inertia = Vec3::new(
            self.second_moment.y + self.second_moment.z,
            self.second_moment.x + self.second_moment.z,
            self.second_moment.x + self.second_moment.y,
        );

        MassProperties {
            surface_area: self.area,
            volume: self.volume,
            center_of_mass,
            inertia_diagonal: Vec3::new(inertia.x.abs(), inertia.y.abs(), inertia.z.abs()),
        }
    }
}

/// 10-point Gauss-Legendre nodes and weights on [-1, 1].
///
/// The trim p-curves are rational, so the line integrals are not polynomial and
/// no rule is exact; ten points bring a rational quadratic arc to roughly
/// machine precision, which is what makes the analytic path worth taking.
const GAUSS_LEGENDRE_10: [(f64, f64); 10] = [
    (-0.973906528517172, 0.066671344308688),
    (-0.865063366688985, 0.149451349150581),
    (-0.679409568299024, 0.219086362515982),
    (-0.433395394129247, 0.269266719309996),
    (-0.148874338981631, 0.295524224714753),
    (0.148874338981631, 0.295524224714753),
    (0.433395394129247, 0.269266719309996),
    (0.679409568299024, 0.219086362515982),
    (0.865063366688985, 0.149451349150581),
    (0.973906528517172, 0.066671344308688),
];

/// Index of the moment `∫∫ u^p v^q du dv` for `p + q <= 3`.
const fn index_uv(p: usize, q: usize) -> usize {
    match (p, q) {
        (0, 0) => 0,
        (1, 0) => 1,
        (0, 1) => 2,
        (2, 0) => 3,
        (1, 1) => 4,
        (0, 2) => 5,
        (3, 0) => 6,
        (2, 1) => 7,
        (1, 2) => 8,
        _ => 9,
    }
}

/// Signed moments `∫∫ u^p v^q du dv` of the region enclosed by a trim loop.
///
/// Green's theorem gives `∫∫ u^p v^q du dv = ∮ u^(p+1)/(p+1) * v^q dv`, so the
/// whole set follows from one pass along the loop's p-curves.
fn loop_uv_moments(pcurve_loop: &FacePcurveLoop) -> [f64; 10] {
    let mut moments = [0.0; 10];

    for segment in &pcurve_loop.segments {
        let (t_min, t_max) = segment.curve.param_range();
        if (t_max - t_min).abs() <= f64::EPSILON {
            continue;
        }

        // ノット区間ごとに積む。B-spline はノットをまたぐと滑らかでなくなるので、
        // 曲線全体に一つの求積則をあてると、たとえば4区間で書かれた完全円の
        // 面積が 1.4% ずれる。単一区間の曲線ではこの分割は何も変えない。
        for (span_start, span_end) in knot_spans(&segment.curve, t_min, t_max) {
            let half_span = 0.5 * (span_end - span_start);
            let midpoint = 0.5 * (span_start + span_end);
            if half_span.abs() <= f64::EPSILON {
                continue;
            }

            for (node, weight) in GAUSS_LEGENDRE_10 {
                let t = midpoint + half_span * node;
                let point = segment.curve.evaluate(t);
                let slope = segment.curve.evaluate_derivative(t);
                let scale = weight * half_span * slope.y;

                let u_powers = [
                    1.0,
                    point.x,
                    point.x * point.x,
                    point.x.powi(3),
                    point.x.powi(4),
                ];
                let v_powers = [1.0, point.y, point.y * point.y, point.y.powi(3)];

                for p in 0..4 {
                    for q in 0..(4 - p) {
                        moments[index_uv(p, q)] +=
                            u_powers[p + 1] / (p as f64 + 1.0) * v_powers[q] * scale;
                    }
                }
            }
        }
    }

    moments
}

/// The curve's distinct knot spans inside `[t_min, t_max]`.
///
/// A single-span curve yields exactly one interval, so callers that only ever
/// see Bezier segments are unaffected.
fn knot_spans(curve: &zenith_geom::NurbsCurve2, t_min: f64, t_max: f64) -> Vec<(f64, f64)> {
    let mut breaks: Vec<f64> = vec![t_min];
    for knot in &curve.knots.knots {
        if *knot > t_min + f64::EPSILON && *knot < t_max - f64::EPSILON {
            breaks.push(*knot);
        }
    }
    breaks.push(t_max);
    breaks.sort_by(f64::total_cmp);
    breaks.dedup_by(|a, b| (*a - *b).abs() <= (t_max - t_min).abs() * 1e-12);

    breaks
        .windows(2)
        .map(|window| (window[0], window[1]))
        .collect()
}
