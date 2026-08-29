use zenith_geom::Surface3;
use zenith_math::{Point2, Point3, Vec3, Vec3Ext};
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
    /// 慣性テンソルの対角成分 (Ixx, Iyy, Izz)。**原点を通る座標軸まわり**。
    ///
    /// 主慣性モーメントではありません。主慣性モーメントは重心を通る主軸まわり
    /// の値で、`principal_moments()` が返します。慣性積を無視してこの3つを主値
    /// として使うと、対称でない形では答えが変わります。
    ///
    /// 実測: 直方体・円柱・球・円錐・原点から離した直方体のいずれも、原点
    /// まわりの閉じた式と **1.8e-13 以内**（`inertia_probe`）。
    pub inertia_diagonal: Vec3,
    /// 慣性積のもとになる積 (∫xy dV, ∫yz dV, ∫zx dV)。**原点まわり**。
    ///
    /// 慣性テンソルの非対角成分は符号が反転した `-∫xy dV` です。テンソルとして
    /// 使うときは `inertia_tensor()` を通してください。
    #[cfg_attr(not(doc), doc(hidden))]
    pub inertia_products: Vec3,
}

impl MassProperties {
    /// 原点を通る座標軸まわりの慣性テンソル（密度1）
    pub fn inertia_tensor(&self) -> [[f64; 3]; 3] {
        let (ixy, iyz, izx) = (
            self.inertia_products.x,
            self.inertia_products.y,
            self.inertia_products.z,
        );
        [
            [self.inertia_diagonal.x, -ixy, -izx],
            [-ixy, self.inertia_diagonal.y, -iyz],
            [-izx, -iyz, self.inertia_diagonal.z],
        ]
    }

    /// 任意の点を通る座標軸まわりの慣性テンソル（平行軸の定理）
    pub fn inertia_tensor_about(&self, point: Point3) -> [[f64; 3]; 3] {
        let mass = self.volume;
        let d = point - self.center_of_mass;
        let c = self.center_of_mass - Point3::origin();

        // まず重心まわりへ移し、そこから目的の点へ移す
        let about_origin = self.inertia_tensor();
        let mut tensor = [[0.0f64; 3]; 3];
        for row in 0..3 {
            for column in 0..3 {
                let shift_from_origin = if row == column {
                    c.norm_squared() - c[row] * c[column]
                } else {
                    -c[row] * c[column]
                };
                let shift_to_point = if row == column {
                    d.norm_squared() - d[row] * d[column]
                } else {
                    -d[row] * d[column]
                };
                tensor[row][column] =
                    about_origin[row][column] - mass * shift_from_origin + mass * shift_to_point;
            }
        }
        tensor
    }

    /// 重心を通る座標軸まわりの慣性テンソル
    pub fn inertia_tensor_about_center_of_mass(&self) -> [[f64; 3]; 3] {
        self.inertia_tensor_about(self.center_of_mass)
    }

    /// 主慣性モーメント（重心を通る主軸まわり、昇順）
    ///
    /// 重心まわりの慣性テンソルの固有値です。対称でない形では
    /// `inertia_diagonal` とは一致しません。
    pub fn principal_moments(&self) -> Vec3 {
        let mut values = symmetric_eigenvalues(self.inertia_tensor_about_center_of_mass());
        values.sort_by(f64::total_cmp);
        Vec3::new(values[0], values[1], values[2])
    }
}

/// 3x3 対称行列の固有値（ヤコビ法）
fn symmetric_eigenvalues(matrix: [[f64; 3]; 3]) -> [f64; 3] {
    let mut a = matrix;
    for _sweep in 0..64 {
        let (mut p, mut q, mut largest) = (0usize, 1usize, 0.0f64);
        for row in 0..3 {
            for column in (row + 1)..3 {
                if a[row][column].abs() > largest {
                    largest = a[row][column].abs();
                    p = row;
                    q = column;
                }
            }
        }
        let scale = a[0][0].abs().max(a[1][1].abs()).max(a[2][2].abs()).max(1.0);
        if largest <= 1e-18 * scale {
            break;
        }

        let theta = 0.5 * (a[q][q] - a[p][p]) / a[p][q];
        let t = theta.signum() / (theta.abs() + (theta * theta + 1.0).sqrt());
        let cos = 1.0 / (t * t + 1.0).sqrt();
        let sin = t * cos;

        let mut next = a;
        for k in 0..3 {
            next[p][k] = cos * a[p][k] - sin * a[q][k];
            next[q][k] = sin * a[p][k] + cos * a[q][k];
        }
        let rotated = next;
        for k in 0..3 {
            next[k][p] = cos * rotated[k][p] - sin * rotated[k][q];
            next[k][q] = sin * rotated[k][p] + cos * rotated[k][q];
        }
        a = next;
    }
    [a[0][0], a[1][1], a[2][2]]
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

    /// 体積だけを求める。**慣性は計算しません。**
    ///
    /// [`Self::compute_from_brep`] は重心と慣性テンソルまで積みます。
    /// ところが**ブーリアンの検証ゲートは体積しか読みません**
    /// （`boolean_validation.rs`、`brep_intersection.rs` の並べ替え）。
    /// 慣性を諦めてよいと分かっていれば、円柱の面は**中身を刻まずに**
    /// 積めます（4-156）。
    ///
    /// 実測（4-156）: 45ケースの面積分の呼び出しは 1,068 回で、うち
    /// `compute_face_integral` から来るのは 84 回だけでした。**残りは
    /// ここから来ています。**
    pub fn compute_volume_from_brep(solid: &Solid, params: &TessellationParams) -> f64 {
        let mut accumulator = SurfaceIntegral {
            area_and_volume_only: true,
            ..Default::default()
        };
        accumulator.add_shell(&solid.outer_shell, params, 1.0);
        for inner in &solid.inner_shells {
            accumulator.add_shell(inner, params, -1.0);
        }
        accumulator.volume
    }

    /// Integrates a single face, returning its area and its contribution to the
    /// enclosed volume.
    pub fn compute_face_integral(face: &Face, params: &TessellationParams) -> (f64, f64) {
        let mut accumulator = SurfaceIntegral {
            area_and_volume_only: true,
            ..Default::default()
        };
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
        let mut ixy = 0.0;
        let mut iyz = 0.0;
        let mut izx = 0.0;

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

            // 5. 慣性積の寄与。原点を頂点とする四面体で
            //    ∫xy dV = V (2 Σ x_i y_i + Σ_{i≠j} x_i y_j) / 20。
            let cross_moment = |a: [f64; 3], b: [f64; 3]| -> f64 {
                let sum_a = a[0] + a[1] + a[2];
                let sum_b = b[0] + b[1] + b[2];
                let paired = a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
                (sum_a * sum_b + paired) / 20.0
            };
            let xs = [p0.x, p1.x, p2.x];
            let ys = [p0.y, p1.y, p2.y];
            let zs = [p0.z, p1.z, p2.z];
            ixy += vol * cross_moment(xs, ys);
            iyz += vol * cross_moment(ys, zs);
            izx += vol * cross_moment(zs, xs);
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
            inertia_products: Vec3::new(ixy, iyz, izx),
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
    /// **慣性まで要らない呼び出しでは立てます。**
    ///
    /// ブーリアンが 90% の時間を使っている面積の検算
    /// （`compute_face_integral`）は、面積と体積しか読みません。慣性を
    /// 諦めてよいなら、円柱は**面の中身を刻まずに**積めます（4-156）。
    area_and_volume_only: bool,
    area: f64,
    volume: f64,
    moment: Vec3,
    second_moment: Vec3,
    /// (∫xy dV, ∫yz dV, ∫zx dV)
    products: Vec3,
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

        // 慣性積 ∫xy dV = ∮ (x^2 y / 2) n_x dA。x も y も (u, v) の1次なので
        // x^2 y は3次で、ここにある モーメントだけで閉じる。
        //
        // ∫ a^2 b = b0 ∫a^2 + bu ∫(a^2 u) + bv ∫(a^2 v)
        let squared_times = |i: usize, j: usize, base: f64, du: f64, dv: f64| -> f64 {
            base * base * moments[index_uv(i, j)]
                + 2.0 * base * du * moments[index_uv(i + 1, j)]
                + 2.0 * base * dv * moments[index_uv(i, j + 1)]
                + du * du * moments[index_uv(i + 2, j)]
                + 2.0 * du * dv * moments[index_uv(i + 1, j + 1)]
                + dv * dv * moments[index_uv(i, j + 2)]
        };
        for (slot, (a_axis, b_axis)) in [(0usize, 1usize), (1, 2), (2, 0)].into_iter().enumerate() {
            let (a0, au, av) = (origin.coords[a_axis], u_axis[a_axis], v_axis[a_axis]);
            let (b0, bu, bv) = (origin.coords[b_axis], u_axis[b_axis], v_axis[b_axis]);
            let integral = b0 * squared_times(0, 0, a0, au, av)
                + bu * squared_times(1, 0, a0, au, av)
                + bv * squared_times(0, 1, a0, au, av);
            self.products[slot] += normal[a_axis] * jacobian * integral / 2.0;
        }

        true
    }

    /// 回転面の一部である面を、**トリム境界だけで**積む。
    ///
    /// 球とトーラスがここに入ります（面積分の 53.9%。4-154）。
    ///
    /// ```text
    /// 面積      = ε · ∮ G(ψ) dφ            G = r(ρ_c ψ + r sinψ)
    /// 体積の寄与 = s · σ · ε · ∮ H(ψ) dφ / 3
    /// ```
    ///
    /// `ε` は回り方の符号、`σ` はパラメータの向きの符号。**どちらも
    /// 出てきた答えを測って決めます。**
    fn add_revolution_face(
        &mut self,
        face: &Face,
        surface: &impl Surface3,
        orientation_sign: f64,
    ) -> bool {
        let Some(pcurves) = face.pcurves.as_ref() else {
            return false;
        };
        let Some(patch) = recognise_revolution_patch(surface) else {
            return false;
        };
        let Some(mut moments) = loop_revolution_moments(&pcurves.outer_loop, surface, &patch) else {
            return false;
        };
        let outer = moments[0];
        if outer.abs() <= f64::EPSILON {
            return false;
        }
        for hole in &pcurves.inner_loops {
            let Some(part) = loop_revolution_moments(hole, surface, &patch) else {
                return false;
            };
            let sign = if part[0].signum() == outer.signum() {
                -1.0
            } else {
                1.0
            };
            for (total, piece) in moments.iter_mut().zip(part.iter()) {
                *total += piece * sign;
            }
        }
        let normalisation = outer.signum();
        for moment in moments.iter_mut() {
            *moment *= normalisation;
        }

        // **世界の原点から見た位置のぶん**を足します。
        //
        // 体積は `∬ p·(p_u × p_v)/3` で、`p` は世界の原点からの位置です。
        // 上の `volume_antiderivative` は軸の上の点から見たぶんしか
        // 積んでいないので、差のぶんを面素ベクトルの積分に掛けて足します。
        let origin = patch.origin.coords;
        let area_vector = patch.e1 * moments[3] * -1.0
            + patch.e2 * moments[4] * -1.0
            + patch.axis * moments[5];
        let volume_moment = moments[2] + origin.dot(&area_vector);

        let area = moments[1];
        if std::env::var_os("ZENITH_REV_WHY").is_some() {
            eprintln!(
                "REVWHY r={:.6} rc={:.6} 軸=({:.4},{:.4},{:.4}) 符号={} 基準u={:.4} 積分=[{:.6},{:.6},{:.6}]",
                patch.meridian_radius, patch.meridian_offset,
                patch.axis.x, patch.axis.y, patch.axis.z,
                patch.jacobian_sign, patch.reference_u,
                moments[0], moments[1], moments[2]
            );
        }
        if !(area.is_finite() && area >= 0.0) {
            return false;
        }
        self.area += area;
        self.volume += orientation_sign * patch.jacobian_sign * volume_moment / 3.0;
        if std::env::var_os("ZENITH_INTEGRAL_WHY").is_some() {
            eprintln!("INTEGRALWHY 回転面（解析。中身を刻まない） 三角形 0");
        }
        true
    }

    /// 母線が直線の回転面（**円柱と円錐**）の一部である面を、
    /// **トリム境界だけで**積む。
    ///
    /// 積めたら `true`。積めなければ何も足さずに `false` を返すので、
    /// 呼び手は従来どおり三角形で積みます。
    ///
    /// 式は [`loop_ruled_revolution_moments`] に。**半径を一定にすると
    /// 4-156 の円柱の式にそのまま戻ります**（4-158 で確かめました）。
    fn add_ruled_revolution_face(
        &mut self,
        face: &Face,
        surface: &impl Surface3,
        orientation_sign: f64,
    ) -> bool {
        let Some(pcurves) = face.pcurves.as_ref() else {
            return false;
        };
        let Some(patch) = recognise_ruled_revolution_patch(surface) else {
            return false;
        };
        let ((_, _), (v_min, v_max)) = surface.param_range();
        // 角を読む `v` は、半径がいちばん大きい端にします（潰れた端を避ける）。
        let frame_v = if patch.radius(v_min).abs() >= patch.radius(v_max).abs() {
            v_min
        } else {
            v_max
        };

        let Some(mut moments) =
            loop_ruled_revolution_moments(&pcurves.outer_loop, surface, &patch, frame_v)
        else {
            return false;
        };
        let outer = moments[0];
        if outer.abs() <= f64::EPSILON {
            return false;
        }
        for hole in &pcurves.inner_loops {
            let Some(part) = loop_ruled_revolution_moments(hole, surface, &patch, frame_v) else {
                return false;
            };
            // 穴は外周と逆符号で効かなければならない（平面の経路と同じ規約）。
            let sign = if part[0].signum() == outer.signum() {
                -1.0
            } else {
                1.0
            };
            for (total, piece) in moments.iter_mut().zip(part.iter()) {
                *total += piece * sign;
            }
        }
        // 回り方に依らず、領域積分が正になるよう正規化する。
        let normalisation = outer.signum();
        for moment in moments.iter_mut() {
            *moment *= normalisation;
        }

        let area = patch.slant_rate() * moments[1];
        if !(area.is_finite() && area >= 0.0) {
            return false;
        }
        self.area += area;
        self.volume += orientation_sign * patch.angle_sign * moments[2] / 3.0;
        if std::env::var_os("ZENITH_INTEGRAL_WHY").is_some() {
            eprintln!("INTEGRALWHY 母線が直線の回転面（解析。中身を刻まない） 三角形 0");
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
        // **円柱なら、面の中身を刻まずに積みます**（4-156）。
        //
        // 面積と体積しか要らない呼び出しに限ります。慣性まで解析で書くのは
        // 別の仕事で、まだやっていません。
        //
        // `ZENITH_NO_ANALYTIC_FACE=1` で止まります。**答えが変わります**——
        // 解析と三角形を突き合わせるための口で、速くするために外す口では
        // ありません。
        if self.area_and_volume_only
            && std::env::var_os("ZENITH_NO_ANALYTIC_FACE").is_none()
            && (self.add_ruled_revolution_face(face, surface, orientation_sign)
                || self.add_revolution_face(face, surface, orientation_sign))
        {
            return;
        }

        let domain = face_uv_triangulation(face, params);

        // **面積分の内訳を数える口**（9-1 / 9-3 の「解析曲面を持つか」を
        // 決めるため。三角形の枚数が仕事量そのものなので、そこを数えます）。
        //
        // `ZENITH_INTEGRAL_WHY=1` で、1回ぶんに「曲面の種類」と「三角形の
        // 枚数」を出します。円柱・円錐の四半パッチは次数 2 × 1・制御点
        // 3 × 2 なので、そこで見分けます（`recognize_cylinder_patch` と
        // 同じ形の判定です）。
        if std::env::var_os("ZENITH_INTEGRAL_WHY").is_some() {
            let kind = match &face.geometry {
                FaceGeometry::Nurbs(nurbs) => {
                    let rows = nurbs.control_points.len();
                    let cols = nurbs.control_points.first().map(|r| r.len()).unwrap_or(0);
                    let ragged = nurbs.control_points.iter().any(|r| r.len() != cols);
                    // v を動かしてヤコビアンが変わらなければ、母線に沿って
                    // 半径が一定——つまり真の円柱です。円錐は変わります。
                    let ((u0, u1), (v0, v1)) = surface.param_range();
                    let middle = (u0 + u1) * 0.5;
                    let jacobian = |v: f64| {
                        let (_, du, dv) = surface.evaluate_with_derivatives(middle, v);
                        let cross: zenith_math::Vec3 = du.cross(&dv);
                        cross.norm()
                    };
                    let low = jacobian(v0);
                    let high = jacobian(v1);
                    let uniform = (low - high).abs() <= low.abs().max(1.0) * 1e-9;
                    format!(
                        "NURBS 次数{}x{} 制御点{}x{}{} v方向一定{}",
                        nurbs.degree_u,
                        nurbs.degree_v,
                        rows,
                        cols,
                        if ragged { "（不揃い）" } else { "" },
                        if uniform { "○" } else { "×" }
                    )
                }
                FaceGeometry::Plane(_) => "平面（線積分に落ちなかったもの）".to_string(),
                _ => "その他".to_string(),
            };
            eprintln!(
                "INTEGRALWHY {kind} 三角形 {}",
                domain.triangles.len()
            );
        }

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
                // ∫xy dV = ∮ (x^2 y / 2) n_x dA など
                self.products += Vec3::new(
                    point.x * point.x * point.y * area_vector.x,
                    point.y * point.y * point.z * area_vector.y,
                    point.z * point.z * point.x * area_vector.z,
                ) / 2.0;
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
            inertia_products: self.products,
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
/// 回転面のパッチ。**球とトーラスは、これ1つで両方とも入ります。**
///
/// 実測（4-157）: 球の `u` 等パラメータ線は、球心を中心とする半径 10 の円。
/// トーラスのそれは、芯の円の上を中心とする半径 4 の円。**どちらも
/// 「半径一定の円弧を、軸まわりに回した面」**でした。
struct RevolutionPatch {
    axis: Vec3,
    /// 軸の上の点。**子午線円の中心と同じ高さ**に取ります。
    origin: Point3,
    /// 軸に直交する枠。回した角をここで測ります。
    e1: Vec3,
    e2: Vec3,
    /// 子午線円の半径。
    meridian_radius: f64,
    /// 子午線円の中心の、軸からの距離。球なら 0。
    meridian_offset: f64,
    /// 子午線の角と回した角の、進む向きの積の符号。
    /// **理屈で決めず、中央で測ります。**
    jacobian_sign: f64,
    /// 回した角を読む基準の `u`。**極を避けるため**、軸から
    /// いちばん遠い所を使います。
    reference_u: f64,
}

impl RevolutionPatch {
    /// 点を「軸からの距離」と「軸に沿った高さ」に落とす。
    fn cylindrical(&self, point: Point3) -> (f64, f64) {
        let offset = point - self.origin;
        let z = offset.dot(&self.axis);
        let radial = offset - self.axis * z;
        (radial.norm(), z)
    }

    /// 子午線上の角度。
    fn meridian_angle(&self, point: Point3) -> f64 {
        let (rho, z) = self.cylindrical(point);
        z.atan2(rho - self.meridian_offset)
    }

    /// 面積の被積分関数の原始関数。sympy と手計算の両方で確かめました（4-157）。
    fn area_antiderivative(&self, psi: f64) -> f64 {
        let (r, rc) = (self.meridian_radius, self.meridian_offset);
        r * (rc * psi + r * psi.sin())
    }

    /// 体積の被積分関数の原始関数。同上。
    ///
    /// **これは「軸の上の点から見た」ぶんです。** 世界の原点から見た位置の
    /// ぶんは別に足します（下の2つ。4-157 で、原点にある球とトーラスでは
    /// 0 なので気づけず、動かした途端に出ました）。
    fn volume_antiderivative(&self, psi: f64) -> f64 {
        let (r, rc) = (self.meridian_radius, self.meridian_offset);
        -r / 2.0
            * (3.0 * r * rc * psi
                + 2.0 * r * r * psi.sin()
                + r * rc * (2.0 * psi).sin() / 2.0
                + 2.0 * rc * rc * psi.sin())
    }

    /// 面素ベクトルの、軸に直交する向きの成分の原始関数。
    /// `∫ ρ·dz/dψ dψ`。
    fn axial_antiderivative(&self, psi: f64) -> f64 {
        let (r, rc) = (self.meridian_radius, self.meridian_offset);
        r * rc * psi.sin() + r * r * (psi / 2.0 + (2.0 * psi).sin() / 4.0)
    }

    /// 面素ベクトルの、軸方向の成分の原始関数。`∫ ρ·dρ/dψ dψ`。
    fn radial_antiderivative(&self, psi: f64) -> f64 {
        let (r, rc) = (self.meridian_radius, self.meridian_offset);
        r * rc * psi.cos() - r * r * psi.sin().powi(2) / 2.0
    }
}

/// 3点から円を当てる。中心・半径・その平面の法線。
fn fit_circle(a: Point3, b: Point3, c: Point3) -> Option<(Point3, f64, Vec3)> {
    let (ab, ac) = (b - a, c - a);
    let normal = ab.cross(&ac);
    let norm = normal.norm();
    let scale = ab.norm().max(ac.norm()).max(1.0);
    if norm <= scale * scale * 1e-12 {
        return None;
    }
    let denominator = 2.0 * norm * norm;
    let alpha = ac.norm_squared() * ab.dot(&(ab - ac)) / denominator;
    let beta = ab.norm_squared() * ac.dot(&(ac - ab)) / denominator;
    let centre = Point3::from(a.coords + ab * alpha + ac * beta);
    let radius = (a - centre).norm();
    if radius <= scale * 1e-12 {
        return None;
    }
    Some((centre, radius, normal / norm))
}

/// 面が**回転面の一部**なら、積むのに要る量を返す。
///
/// **推測しません。** 子午線が本当に半径一定の円か、それが軸まわりに回って
/// いるか、標本がその面に乗るか——全部測ってから返します。1つでも外れたら
/// `None` を返し、従来どおり三角形で積みます。
fn recognise_revolution_patch(surface: &impl Surface3) -> Option<RevolutionPatch> {
    let ((u_min, u_max), (v_min, v_max)) = surface.param_range();
    let (span_u, span_v) = (u_max - u_min, v_max - v_min);
    if !(span_u.abs() > 0.0 && span_v.abs() > 0.0) {
        return None;
    }
    let at = |t: f64, w: f64| surface.evaluate(u_min + span_u * t, v_min + span_v * w);

    let mut fitted = Vec::with_capacity(3);
    for w in [0.0f64, 0.5, 1.0] {
        fitted.push(fit_circle(at(0.0, w), at(0.5, w), at(1.0, w))?);
    }
    let radius = fitted[0].1;
    let scale = radius.max((at(0.0, 0.0) - at(0.0, 1.0)).norm()).max(1.0);
    if fitted.iter().any(|(_, r, _)| (r - radius).abs() > radius * 1e-9) {
        return None;
    }

    let axis = fitted[0].2.cross(&fitted[2].2).try_normalize_safe(1e-12)?;

    let moved = (fitted[2].0 - fitted[0].0).norm();
    let (origin, meridian_offset) = if moved <= scale * 1e-9 {
        (fitted[0].0, 0.0)
    } else {
        let (centre, offset, normal) = fit_circle(fitted[0].0, fitted[1].0, fitted[2].0)?;
        if 1.0 - normal.dot(&axis).abs() > 1e-9 {
            return None;
        }
        (centre, offset)
    };

    let candidate = RevolutionPatch {
        axis,
        origin,
        e1: Vec3::new(1.0, 0.0, 0.0),
        e2: Vec3::new(0.0, 1.0, 0.0),
        meridian_radius: radius,
        meridian_offset,
        jacobian_sign: 1.0,
        reference_u: u_min + span_u * 0.5,
    };

    for t in [0.0f64, 0.125, 0.375, 0.625, 0.875, 1.0] {
        for w in [0.0f64, 0.25, 0.75, 1.0] {
            let (rho, z) = candidate.cylindrical(at(t, w));
            let residual = ((rho - meridian_offset).powi(2) + z * z).sqrt() - radius;
            if residual.abs() > radius * 1e-9 {
                return None;
            }
        }
    }

    let mut reference_u = candidate.reference_u;
    let mut best_rho = 0.0;
    for t in 0..=8 {
        let u = u_min + span_u * (t as f64 / 8.0);
        let (rho, _) = candidate.cylindrical(surface.evaluate(u, v_min));
        if rho > best_rho {
            best_rho = rho;
            reference_u = u;
        }
    }
    if best_rho <= scale * 1e-9 {
        return None;
    }
    let reference = surface.evaluate(reference_u, v_min);
    let offset = reference - origin;
    let e1 = (offset - axis * offset.dot(&axis)).try_normalize_safe(1e-12)?;
    let e2 = axis.cross(&e1);

    let middle_u = u_min + span_u * 0.5;
    let middle_v = v_min + span_v * 0.5;
    let step_u = span_u * 1e-6;
    let step_v = span_v * 1e-6;
    let psi_of = |u: f64, v: f64| candidate.meridian_angle(surface.evaluate(u, v));
    let phi_of = |v: f64| {
        let point = surface.evaluate(reference_u, v);
        let offset = point - origin;
        let radial = offset - axis * offset.dot(&axis);
        radial.dot(&e2).atan2(radial.dot(&e1))
    };
    let d_psi = psi_of(middle_u + step_u, middle_v) - psi_of(middle_u - step_u, middle_v);
    let d_phi = phi_of(middle_v + step_v) - phi_of(middle_v - step_v);
    if d_psi == 0.0 || d_phi == 0.0 {
        return None;
    }

    Some(RevolutionPatch {
        e1,
        e2,
        jacobian_sign: (d_psi * d_phi).signum(),
        reference_u,
        ..candidate
    })
}

/// トリム境界だけを回って、領域積分を6つ求める。**u 方向には刻みません。**
///
/// 回した角の増分は曲面の導関数から厳密に取ります——`v` 方向の導関数は
/// 接線向きで、大きさが「軸からの距離 × 角の増分率」なので割れば出ます。
/// **極を避けるため、軸からいちばん遠い `u` で読みます**（回した角は `v`
/// だけの関数なので、どの `u` で読んでも同じです）。
///
/// ## **輪は自分で閉じます**
///
/// p-curve のループは、**3D で潰れる辺を落として**返ってきます。球の
/// パッチは実測で **3区間**で、極の辺がありません（4-157）。グリーンの
/// 定理は閉じた輪を要求するので、隙間があれば `(u, v)` の直線で埋めます。
///
/// **落ちていたのは、ちょうど全部でした**——閉じずに積むと、球の面積が
/// 0 になります（残りの辺の寄与が 0 だったため）。
fn loop_revolution_moments(
    pcurve_loop: &FacePcurveLoop,
    surface: &impl Surface3,
    patch: &RevolutionPatch,
) -> Option<[f64; 6]> {
    let mut moments = [0.0f64; 6];
    let mut previous_psi: Option<f64> = None;
    let mut failed = false;

    // 1区間ぶんを積む。`uv_at` は媒介変数から `(u, v)`、`slope_at` はその微分。
    let mut integrate = |uv_at: &dyn Fn(f64) -> Point2,
                         slope_at: &dyn Fn(f64) -> Point2,
                         previous_psi: &mut Option<f64>| {
        for (node, weight) in GAUSS_LEGENDRE_10 {
            let t = 0.5 * (node + 1.0);
            let uv = uv_at(t);
            let slope = slope_at(t);

            // 子午線の角は繋げて読みます。トーラスの内側で折り返しを
            // またぐと、切れたところで正弦が飛びます。
            let mut psi = patch.meridian_angle(surface.evaluate(uv.x, uv.y));
            if let Some(last) = *previous_psi {
                let turns = ((psi - last) / std::f64::consts::TAU).round();
                psi -= turns * std::f64::consts::TAU;
            }
            *previous_psi = Some(psi);

            let (point, _, dv_vector) =
                surface.evaluate_with_derivatives(patch.reference_u, uv.y);
            let offset = point - patch.origin;
            let axial = offset.dot(&patch.axis);
            let radial = offset - patch.axis * axial;
            let rho = radial.norm();
            if rho <= f64::EPSILON {
                failed = true;
                return;
            }
            let unit = radial / rho;
            let tangential = patch.axis.cross(&unit);
            let angle_rate = dv_vector.dot(&tangential) / rho;
            let (cos_phi, sin_phi) = (unit.dot(&patch.e1), unit.dot(&patch.e2));

            // 0.5 は t を [-1,1] から [0,1] へ移したぶん。
            let scale = 0.5 * weight * angle_rate * slope.y;
            moments[0] += psi * scale;
            moments[1] += patch.area_antiderivative(psi) * scale;
            moments[2] += patch.volume_antiderivative(psi) * scale;
            let axial_moment = patch.axial_antiderivative(psi);
            moments[3] += axial_moment * cos_phi * scale;
            moments[4] += axial_moment * sin_phi * scale;
            moments[5] += patch.radial_antiderivative(psi) * scale;
        }
    };

    // 区間の並びを (u, v) で拾い、隙間があれば直線で埋める。
    let mut first: Option<Point2> = None;
    let mut cursor: Option<Point2> = None;
    for segment in &pcurve_loop.segments {
        let (t_min, t_max) = segment.curve.param_range();
        if (t_max - t_min).abs() <= f64::EPSILON {
            continue;
        }
        let start = segment.curve.evaluate(t_min);
        if first.is_none() {
            first = Some(start);
        }
        if let Some(previous) = cursor {
            if (start - previous).norm() > 1e-12 {
                let gap_start = previous;
                let gap_end = start;
                let step = gap_end - gap_start;
                integrate(
                    &|t| Point2::from(gap_start.coords + step * t),
                    &|_| Point2::from(step),
                    &mut previous_psi,
                );
            }
        }
        for (span_start, span_end) in knot_spans(&segment.curve, t_min, t_max) {
            let width = span_end - span_start;
            if width.abs() <= f64::EPSILON {
                continue;
            }
            integrate(
                &|t| segment.curve.evaluate(span_start + width * t),
                &|t| (segment.curve.evaluate_derivative(span_start + width * t) * width).into(),
                &mut previous_psi,
            );
        }
        cursor = Some(segment.curve.evaluate(t_max));
    }
    // 最後から最初へ戻る辺。**ここが球の極の辺です。**
    if let (Some(start), Some(end)) = (first, cursor) {
        if (start - end).norm() > 1e-12 {
            let step = start - end;
            integrate(
                &|t| Point2::from(end.coords + step * t),
                &|_| Point2::from(step),
                &mut previous_psi,
            );
        }
    }

    if failed {
        return None;
    }
    moments.iter().all(|m| m.is_finite()).then_some(moments)
}

/// 母線が直線の回転面。**円柱と円錐は、これ1つで両方とも入ります。**
///
/// 実測（4-158）: 円錐の `u` 等パラメータ線は軸まわりの円で、**半径が `v`
/// で線形に変わります**（10 → 5 → 0）。母線は直線です。円柱は「半径が
/// 変わらない円錐」なので、同じ式で書けます。
///
/// 式も確かめました——半径を一定にすると、4-156 の円柱の式に**そのまま
/// 戻ります**。
struct RuledRevolutionPatch {
    axis: Vec3,
    /// `v` の下端における、軸の上の点。
    origin: Point3,
    /// 軸に直交する枠。回した角をここで測ります。
    e1: Vec3,
    e2: Vec3,
    /// `v` の下端の半径と、`v` に対する増分率。
    radius_at_start: f64,
    radius_rate: f64,
    /// 軸に沿った高さの、`v` に対する増分率。下端は 0。
    axial_rate: f64,
    /// 回した角の、`u` に対する進む向きの符号。**中央で測って決めます。**
    angle_sign: f64,
    /// `v` の下端。
    v_start: f64,
}

impl RuledRevolutionPatch {
    fn radius(&self, v: f64) -> f64 {
        self.radius_at_start + self.radius_rate * (v - self.v_start)
    }

    fn axial(&self, v: f64) -> f64 {
        self.axial_rate * (v - self.v_start)
    }

    /// 面素ベクトルの大きさの、半径で割ったぶん。`sqrt(dρ/dv² + dz/dv²)`。
    fn slant_rate(&self) -> f64 {
        (self.radius_rate * self.radius_rate + self.axial_rate * self.axial_rate).sqrt()
    }
}

/// 面が**母線の直線な回転面**（円柱・円錐）の一部なら、積むのに要る量を返す。
///
/// **推測しません。** 等パラメータ線が円か、半径が `v` で線形か、母線が
/// 直線か、標本がその面に乗るか——全部測ってから返します。1つでも外れたら
/// `None` を返し、従来どおり三角形で積みます。
fn recognise_ruled_revolution_patch(surface: &impl Surface3) -> Option<RuledRevolutionPatch> {
    let ((u_min, u_max), (v_min, v_max)) = surface.param_range();
    let (span_u, span_v) = (u_max - u_min, v_max - v_min);
    if !(span_u.abs() > 0.0 && span_v.abs() > 0.0) {
        return None;
    }
    let at = |t: f64, w: f64| surface.evaluate(u_min + span_u * t, v_min + span_v * w);

    // **両端は避けます。** 真の円錐は片方が1点に潰れていて、円が当たりません。
    let samples = [0.2f64, 0.5, 0.8];
    let mut fitted = Vec::with_capacity(3);
    for w in samples {
        fitted.push(fit_circle(at(0.0, w), at(0.5, w), at(1.0, w))?);
    }
    let scale = fitted[0].1.max((at(0.0, 0.0) - at(0.0, 1.0)).norm()).max(1.0);

    // 半径が `w` で線形か。
    let (r_low, r_mid, r_high) = (fitted[0].1, fitted[1].1, fitted[2].1);
    if (r_mid - (r_low + r_high) * 0.5).abs() > scale * 1e-9 {
        return None;
    }
    // 中心が一直線に並ぶか。
    let along = fitted[2].0 - fitted[0].0;
    let length = along.norm();
    if length <= scale * 1e-9 {
        return None;
    }
    let axis = along / length;
    let middle_offset = fitted[1].0 - fitted[0].0;
    if (middle_offset - along * 0.5).norm() > scale * 1e-9 {
        return None;
    }
    // 等パラメータ線の平面が、軸に直交するか。
    for (_, _, normal) in &fitted {
        if 1.0 - normal.dot(&axis).abs() > 1e-9 {
            return None;
        }
    }

    // `w` から実際の `v` へ戻す。
    let width = samples[2] - samples[0];
    let radius_rate_w = (r_high - r_low) / width;
    let axial_rate_w = length / width;
    let radius_at_start = r_low - radius_rate_w * samples[0];
    let origin = Point3::from(fitted[0].0.coords - axis * (axial_rate_w * samples[0]));

    // 母線が直線か。中点が両端の平均に乗るか。
    for t in [0.0f64, 0.25, 0.5, 0.75, 1.0] {
        let (a, m, b) = (at(t, 0.0), at(t, 0.5), at(t, 1.0));
        let average = Point3::from((a.coords + b.coords) * 0.5);
        if (m - average).norm() > scale * 1e-9 {
            return None;
        }
    }

    let candidate = RuledRevolutionPatch {
        axis,
        origin,
        e1: Vec3::new(1.0, 0.0, 0.0),
        e2: Vec3::new(0.0, 1.0, 0.0),
        radius_at_start,
        radius_rate: radius_rate_w / span_v,
        axial_rate: axial_rate_w / span_v,
        angle_sign: 1.0,
        v_start: v_min,
    };

    // 標本が本当にこの面に乗るか。
    for t in [0.0f64, 0.125, 0.375, 0.625, 0.875, 1.0] {
        for w in [0.0f64, 0.25, 0.5, 0.75, 1.0] {
            let v = v_min + span_v * w;
            let point = at(t, w);
            let offset = point - candidate.origin;
            let axial = offset.dot(&axis);
            let radial = (offset - axis * axial).norm();
            if (axial - candidate.axial(v)).abs() > scale * 1e-9 {
                return None;
            }
            if (radial - candidate.radius(v)).abs() > scale * 1e-9 {
                return None;
            }
        }
    }

    // 枠は、半径がいちばん大きい端で取ります（潰れた端を避けるため）。
    let frame_v = if candidate.radius(v_min).abs() >= candidate.radius(v_max).abs() {
        v_min
    } else {
        v_max
    };
    let reference = surface.evaluate(u_min, frame_v);
    let offset = reference - candidate.origin;
    let e1 = (offset - axis * offset.dot(&axis)).try_normalize_safe(1e-12)?;
    let e2 = axis.cross(&e1);

    // 回した角の進む向きは、中央で測って決めます。
    let angle_of = |u: f64| {
        let point = surface.evaluate(u, frame_v);
        let offset = point - candidate.origin;
        let radial = offset - axis * offset.dot(&axis);
        radial.dot(&e2).atan2(radial.dot(&e1))
    };
    let step = span_u * 1e-6;
    let middle_u = u_min + span_u * 0.5;
    let delta = angle_of(middle_u + step) - angle_of(middle_u - step);
    if delta == 0.0 {
        return None;
    }

    Some(RuledRevolutionPatch {
        e1,
        e2,
        angle_sign: delta.signum(),
        ..candidate
    })
}

/// トリム境界だけを回って、領域積分を3つ求める。**u 方向には刻みません。**
///
/// ```text
/// 面積 = L · ε · ∮ ρ(v)·θ dv                  L = sqrt(dρ/dv² + dz/dv²)
/// 体積 = s·σ·ε/3 · ∮ [A(v)·θ + B(v)(O·e1 sinθ − O·e2 cosθ)] dv
/// ```
///
/// `A(v) = ρ·(−dρ/dv·(O·a + z) + dz/dv·ρ)`、`B(v) = ρ·dz/dv`。
/// **半径を一定にすると 4-156 の円柱の式に戻ります**（確かめました）。
///
/// **輪は自分で閉じます。** p-curve のループは 3D で潰れる辺を落として
/// 返るので（真の円錐の頂点、球の極。4-157）、隙間は `(u, v)` の直線で
/// 埋めます。
fn loop_ruled_revolution_moments(
    pcurve_loop: &FacePcurveLoop,
    surface: &impl Surface3,
    patch: &RuledRevolutionPatch,
    frame_v: f64,
) -> Option<[f64; 3]> {
    let origin = patch.origin.coords;
    let axial_origin = origin.dot(&patch.axis);
    let (o1, o2) = (origin.dot(&patch.e1), origin.dot(&patch.e2));
    let mut moments = [0.0f64; 3];
    let mut previous_angle: Option<f64> = None;

    let mut integrate = |uv_at: &dyn Fn(f64) -> Point2,
                         slope_at: &dyn Fn(f64) -> Point2,
                         previous_angle: &mut Option<f64>| {
        for (node, weight) in GAUSS_LEGENDRE_10 {
            let t = 0.5 * (node + 1.0);
            let uv = uv_at(t);
            let slope = slope_at(t);

            // 角は、半径がいちばん大きい端で読みます（潰れた端を避ける）。
            // 角は `u` だけの関数なので、どの `v` で読んでも同じです。
            let point = surface.evaluate(uv.x, frame_v);
            let offset = point - patch.origin;
            let radial = offset - patch.axis * offset.dot(&patch.axis);
            let mut theta = radial.dot(&patch.e2).atan2(radial.dot(&patch.e1));
            if let Some(last) = *previous_angle {
                let turns = ((theta - last) / std::f64::consts::TAU).round();
                theta -= turns * std::f64::consts::TAU;
            }
            *previous_angle = Some(theta);

            let rho = patch.radius(uv.y);
            let z = patch.axial(uv.y);
            let a_term = rho
                * (-patch.radius_rate * (axial_origin + z) + patch.axial_rate * rho);
            let b_term = rho * patch.axial_rate;

            // 0.5 は t を [-1,1] から [0,1] へ移したぶん。
            let scale = 0.5 * weight * slope.y;
            moments[0] += theta * scale;
            moments[1] += rho * theta * scale;
            moments[2] +=
                (a_term * theta + b_term * (o1 * theta.sin() - o2 * theta.cos())) * scale;
        }
    };

    let mut first: Option<Point2> = None;
    let mut cursor: Option<Point2> = None;
    for segment in &pcurve_loop.segments {
        let (t_min, t_max) = segment.curve.param_range();
        if (t_max - t_min).abs() <= f64::EPSILON {
            continue;
        }
        let start = segment.curve.evaluate(t_min);
        if first.is_none() {
            first = Some(start);
        }
        if let Some(previous) = cursor {
            if (start - previous).norm() > 1e-12 {
                let step = start - previous;
                integrate(
                    &|t| Point2::from(previous.coords + step * t),
                    &|_| Point2::from(step),
                    &mut previous_angle,
                );
            }
        }
        for (span_start, span_end) in knot_spans(&segment.curve, t_min, t_max) {
            let width = span_end - span_start;
            if width.abs() <= f64::EPSILON {
                continue;
            }
            integrate(
                &|t| segment.curve.evaluate(span_start + width * t),
                &|t| (segment.curve.evaluate_derivative(span_start + width * t) * width).into(),
                &mut previous_angle,
            );
        }
        cursor = Some(segment.curve.evaluate(t_max));
    }
    if let (Some(start), Some(end)) = (first, cursor) {
        if (start - end).norm() > 1e-12 {
            let step = start - end;
            integrate(
                &|t| Point2::from(end.coords + step * t),
                &|_| Point2::from(step),
                &mut previous_angle,
            );
        }
    }

    moments.iter().all(|m| m.is_finite()).then_some(moments)
}

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
