use crate::bspline_basis::KnotVector;
use serde::{Deserialize, Serialize};
use zenith_math::{Point3, Vec3, Vec3Ext};

/// 制御点（座標 + 重み）
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct ControlPoint3 {
    pub point: Point3,
    pub weight: f64,
}

impl ControlPoint3 {
    pub fn new(point: Point3, weight: f64) -> Self {
        Self { point, weight }
    }

    pub fn unweighted(point: Point3) -> Self {
        Self { point, weight: 1.0 }
    }

    /// 同次座標ベクトル (wx, wy, wz, w)
    pub fn to_homogeneous(&self) -> nalgebra::Vector4<f64> {
        nalgebra::Vector4::new(
            self.point.x * self.weight,
            self.point.y * self.weight,
            self.point.z * self.weight,
            self.weight,
        )
    }

    /// 同次座標ベクトルから復元
    pub fn from_homogeneous(v: &nalgebra::Vector4<f64>) -> Self {
        let w = v.w;
        if w.abs() > 1e-15 {
            Self {
                point: Point3::new(v.x / w, v.y / w, v.z / w),
                weight: w,
            }
        } else {
            Self {
                point: Point3::new(v.x, v.y, v.z),
                weight: w,
            }
        }
    }
}

/// 3次元 NURBS / B-Spline 曲線
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct NurbsCurve3 {
    pub degree: usize,
    pub control_points: Vec<ControlPoint3>,
    pub knots: KnotVector,
}

impl NurbsCurve3 {
    /// 新規作成（バリデーション付き）
    pub fn new(
        degree: usize,
        control_points: Vec<ControlPoint3>,
        knots: KnotVector,
    ) -> Result<Self, String> {
        let n = control_points.len();
        if n < degree + 1 {
            return Err(format!(
                "Number of control points ({}) must be >= degree + 1 ({})",
                n,
                degree + 1
            ));
        }
        let expected_knots = n + degree + 1;
        if knots.knots.len() != expected_knots {
            return Err(format!(
                "Knot vector length ({}) does not match control points + degree + 1 ({})",
                knots.knots.len(),
                expected_knots
            ));
        }

        Ok(Self {
            degree,
            control_points,
            knots,
        })
    }

    /// 非有理（全重み1.0）のB-Spline曲線を制御点から簡易作成（クランプ均等結び目）
    pub fn bspline_from_points(degree: usize, points: Vec<Point3>) -> Result<Self, String> {
        let n = points.len();
        let knots = KnotVector::clamped_uniform(n, degree);
        let ctrl_pts = points.into_iter().map(ControlPoint3::unweighted).collect();
        Self::new(degree, ctrl_pts, knots)
    }

    /// 与えた点を**すべて通る**クランプ B-spline を作る。
    ///
    /// `bspline_from_points` は渡した点を制御点として使うので、曲線は端の2点
    /// 以外を通らない。交線を辿った点列のように「この位置を通ってほしい」
    /// 点があるときは、それでは足りない。
    ///
    /// 手順は The NURBS Book の A9.1（大域補間）。弦長で媒介変数を割り当て、
    /// ノットはその平均で置き、`N P = Q` を解く。有理ではない（重みは 1）。
    pub fn interpolate_points(degree: usize, points: &[Point3]) -> Result<Self, String> {
        let count = points.len();
        if count < 2 {
            return Err("interpolation needs at least two points".to_string());
        }
        let degree = degree.min(count - 1).max(1);

        // 1. 弦長で媒介変数を割り当てる。等間隔だと、点の間隔が変わるところで
        //    曲線が膨らむ。
        let mut lengths = Vec::with_capacity(count);
        lengths.push(0.0);
        let mut total = 0.0;
        for index in 1..count {
            total += (points[index] - points[index - 1]).norm();
            lengths.push(total);
        }
        if total <= f64::EPSILON {
            return Err("interpolation points are all at the same place".to_string());
        }
        let parameters: Vec<f64> = lengths.iter().map(|length| length / total).collect();

        // 2. ノットは媒介変数の移動平均。こう置くと係数行列が正則になる
        //    （A9.1 の根拠）。
        let mut knots = vec![0.0; degree + 1];
        for j in 1..count.saturating_sub(degree) {
            let mean: f64 = parameters[j..j + degree].iter().sum::<f64>() / degree as f64;
            knots.push(mean);
        }
        knots.extend(std::iter::repeat(1.0).take(degree + 1));
        let knot_vector = KnotVector::new(knots);

        // 3. N P = Q を解く。点の数は交線1本ぶん程度なので密行列で足りる。
        let mut matrix = nalgebra::DMatrix::<f64>::zeros(count, count);
        for (row, parameter) in parameters.iter().enumerate() {
            let span = knot_vector.find_span(count, degree, *parameter);
            let basis = knot_vector.basis_functions(span, degree, *parameter);
            for (offset, value) in basis.iter().enumerate().take(degree + 1) {
                let column = span - degree + offset;
                if column < count {
                    matrix[(row, column)] = *value;
                }
            }
        }

        let mut rhs = nalgebra::DMatrix::<f64>::zeros(count, 3);
        for (row, point) in points.iter().enumerate() {
            rhs[(row, 0)] = point.x;
            rhs[(row, 1)] = point.y;
            rhs[(row, 2)] = point.z;
        }

        let solution = matrix
            .lu()
            .solve(&rhs)
            .ok_or_else(|| "the interpolation system is singular".to_string())?;

        let control_points = (0..count)
            .map(|row| {
                ControlPoint3::unweighted(Point3::new(
                    solution[(row, 0)],
                    solution[(row, 1)],
                    solution[(row, 2)],
                ))
            })
            .collect();

        Self::new(degree, control_points, knot_vector)
    }

    /// 媒介変数とノットを外から与えて補間する。
    ///
    /// テンソル積で曲面を補間するときは、行ごとに弦長で決めた媒介変数を使うと
    /// 行どうしで食い違う。共通の媒介変数を渡せる口が要る。
    pub fn interpolate_points_with(
        degree: usize,
        points: &[Point3],
        parameters: &[f64],
        knots: &KnotVector,
    ) -> Result<Self, String> {
        let count = points.len();
        if count < 2 || parameters.len() != count {
            return Err("interpolation needs one parameter per point".to_string());
        }

        let mut matrix = nalgebra::DMatrix::<f64>::zeros(count, count);
        for (row, parameter) in parameters.iter().enumerate() {
            let span = knots.find_span(count, degree, *parameter);
            let basis = knots.basis_functions(span, degree, *parameter);
            for (offset, value) in basis.iter().enumerate().take(degree + 1) {
                let column = span - degree + offset;
                if column < count {
                    matrix[(row, column)] = *value;
                }
            }
        }

        let mut rhs = nalgebra::DMatrix::<f64>::zeros(count, 3);
        for (row, point) in points.iter().enumerate() {
            rhs[(row, 0)] = point.x;
            rhs[(row, 1)] = point.y;
            rhs[(row, 2)] = point.z;
        }

        let solution = matrix
            .lu()
            .solve(&rhs)
            .ok_or_else(|| "the interpolation system is singular".to_string())?;

        let control_points = (0..count)
            .map(|row| {
                ControlPoint3::unweighted(Point3::new(
                    solution[(row, 0)],
                    solution[(row, 1)],
                    solution[(row, 2)],
                ))
            })
            .collect();

        Self::new(degree, control_points, knots.clone())
    }

    /// 弦長で媒介変数を割り当て、その平均でノットを置く。
    ///
    /// 補間の媒介変数とノットの決め方は The NURBS Book の A9.1 に従う。
    pub fn interpolation_parameters(points: &[Point3]) -> Option<Vec<f64>> {
        let count = points.len();
        if count < 2 {
            return None;
        }
        let mut lengths = Vec::with_capacity(count);
        lengths.push(0.0);
        let mut total = 0.0;
        for index in 1..count {
            total += (points[index] - points[index - 1]).norm();
            lengths.push(total);
        }
        if total <= f64::EPSILON {
            return None;
        }
        Some(lengths.iter().map(|length| length / total).collect())
    }

    /// 媒介変数の移動平均でノットを置く。
    pub fn interpolation_knots(parameters: &[f64], degree: usize) -> KnotVector {
        let count = parameters.len();
        let mut knots = vec![0.0; degree + 1];
        for j in 1..count.saturating_sub(degree) {
            let mean: f64 = parameters[j..j + degree].iter().sum::<f64>() / degree as f64;
            knots.push(mean);
        }
        knots.extend(std::iter::repeat(1.0).take(degree + 1));
        KnotVector::new(knots)
    }

    /// パラメータ方向を反転した同じ曲線を返す。
    pub fn reversed(&self) -> Self {
        let start = self.knots.knots.first().copied().unwrap_or(0.0);
        let end = self.knots.knots.last().copied().unwrap_or(1.0);
        let knots = self
            .knots
            .knots
            .iter()
            .rev()
            .map(|k| start + end - *k)
            .collect();

        Self {
            degree: self.degree,
            control_points: self.control_points.iter().rev().copied().collect(),
            knots: KnotVector::new(knots),
        }
    }

    /// 単一ベジエ区間（内部ノットなしのクランプ曲線）をパラメータ `t` で2分割する。
    ///
    /// 同次座標系での有理 de Casteljau 分割なので、真円弧などの有理曲線も
    /// 重みを保ったまま厳密に分割される。内部ノットを持つ曲線や範囲外の `t`
    /// では `None` を返す。
    pub fn split_bezier_at(&self, t: f64) -> Option<(Self, Self)> {
        let order = self.degree + 1;
        if self.control_points.len() != order || self.knots.knots.len() != order * 2 {
            return None;
        }

        let (t_min, t_max) = self.param_range();
        if t_max - t_min <= f64::EPSILON {
            return None;
        }
        let alpha = (t - t_min) / (t_max - t_min);
        if !(f64::EPSILON..=1.0 - f64::EPSILON).contains(&alpha) {
            return None;
        }

        let mut level: Vec<nalgebra::Vector4<f64>> = self
            .control_points
            .iter()
            .map(|point| point.to_homogeneous())
            .collect();
        let mut left = vec![level[0]];
        let mut right = vec![level[order - 1]];

        while level.len() > 1 {
            level = level
                .windows(2)
                .map(|pair| pair[0] * (1.0 - alpha) + pair[1] * alpha)
                .collect();
            left.push(level[0]);
            right.push(level[level.len() - 1]);
        }
        right.reverse();

        let build = |points: Vec<nalgebra::Vector4<f64>>| {
            let control_points = points
                .iter()
                .map(ControlPoint3::from_homogeneous)
                .collect::<Vec<_>>();
            Self::new(
                self.degree,
                control_points,
                KnotVector::clamped_uniform(order, self.degree),
            )
            .ok()
        };

        Some((build(left)?, build(right)?))
    }

    /// ノット `t` を `times` 回挿入した、形の変わらない同じ曲線を返す。
    ///
    /// Boehm の挿入を同次座標で行うので、有理曲線でも重みごと厳密に保たれる。
    /// 端のノットや、多重度が次数を超える挿入は `None` を返す。
    pub fn insert_knot(&self, t: f64, times: usize) -> Option<Self> {
        if times == 0 {
            return Some(self.clone());
        }
        let p = self.degree;
        let (t_min, t_max) = self.param_range();
        let span = (t_max - t_min).abs().max(1.0);
        if t <= t_min + span * 1e-12 || t >= t_max - span * 1e-12 {
            return None;
        }

        let mut knots = self.knots.knots.clone();
        let mut points: Vec<nalgebra::Vector4<f64>> = self
            .control_points
            .iter()
            .map(|point| point.to_homogeneous())
            .collect();

        let multiplicity = knots
            .iter()
            .filter(|k| (**k - t).abs() <= span * 1e-12)
            .count();
        if multiplicity + times > p {
            return None;
        }

        for _ in 0..times {
            // t を含む区間 [knots[k], knots[k+1]) を探す。
            let k = match knots.windows(2).position(|w| w[0] <= t && t < w[1]) {
                Some(index) => index,
                None => return None,
            };
            if k < p {
                return None;
            }

            let mut next = Vec::with_capacity(points.len() + 1);
            next.extend_from_slice(&points[..k - p + 1]);
            for i in (k - p + 1)..=k {
                let denom = knots[i + p] - knots[i];
                let alpha = if denom.abs() <= f64::EPSILON {
                    0.0
                } else {
                    (t - knots[i]) / denom
                };
                next.push(points[i] * alpha + points[i - 1] * (1.0 - alpha));
            }
            next.extend_from_slice(&points[k..]);

            knots.insert(k + 1, t);
            points = next;
        }

        let control_points = points.iter().map(ControlPoint3::from_homogeneous).collect();
        Self::new(p, control_points, KnotVector::new(knots)).ok()
    }

    /// パラメータ `t` で2本のクランプ曲線に分割する。
    ///
    /// `split_bezier_at` と違い、内部ノットを持つ曲線でも割れる。全周の円を
    /// 四半円弧に刻むのはこの経路で、有理の重みは挿入で保たれるので、割った
    /// 後の2本は元の曲線と同じ点を通る。
    pub fn split_at(&self, t: f64) -> Option<(Self, Self)> {
        let p = self.degree;
        let (t_min, t_max) = self.param_range();
        let span = (t_max - t_min).abs().max(1.0);
        let multiplicity = self
            .knots
            .knots
            .iter()
            .filter(|k| (**k - t).abs() <= span * 1e-12)
            .count();
        if multiplicity > p {
            return None;
        }

        let raised = self.insert_knot(t, p - multiplicity)?;
        let knots = &raised.knots.knots;
        let first = knots.iter().position(|k| (*k - t).abs() <= span * 1e-12)?;
        if first == 0 || first + p > knots.len() {
            return None;
        }

        let mut left_knots = knots[..first].to_vec();
        left_knots.extend(std::iter::repeat(t).take(p + 1));
        let left = Self::new(
            p,
            raised.control_points[..first].to_vec(),
            KnotVector::new(left_knots),
        )
        .ok()?;

        let mut right_knots: Vec<f64> = std::iter::repeat(t).take(p + 1).collect();
        right_knots.extend_from_slice(&knots[first + p..]);
        let right = Self::new(
            p,
            raised.control_points[first - 1..].to_vec(),
            KnotVector::new(right_knots),
        )
        .ok()?;

        Some((left, right))
    }

    /// パラメータ有効範囲 [u_min, u_max]


    pub fn param_range(&self) -> (f64, f64) {
        (
            self.knots.start_param(self.degree),
            self.knots.end_param(self.control_points.len()),
        )
    }

    /// 曲線上の3次元座標を評価（Algorithm A3.1 / A4.1 from The NURBS Book）
    pub fn evaluate(&self, u: f64) -> Point3 {
        let span = self
            .knots
            .find_span(self.control_points.len(), self.degree, u);
        let basis = self.knots.basis_functions(span, self.degree, u);

        let mut c_w = nalgebra::Vector4::zeros();
        for (i, basis_value) in basis.iter().enumerate().take(self.degree + 1) {
            let idx = span - self.degree + i;
            let p_w = self.control_points[idx].to_homogeneous();
            c_w += p_w * *basis_value;
        }

        ControlPoint3::from_homogeneous(&c_w).point
    }

    /// 曲線上の点および 1階・2階導関数（接線、加速度）を評価 (Algorithm A4.2)
    pub fn evaluate_derivatives(&self, u: f64, num_ders: usize) -> Vec<Vec3> {
        let span = self
            .knots
            .find_span(self.control_points.len(), self.degree, u);
        let ders_basis = self
            .knots
            .ders_basis_functions(span, self.degree, num_ders, u);

        // 同次座標での導関数を計算
        let mut a_ders = vec![nalgebra::Vector4::zeros(); num_ders + 1];
        for (k, a_der) in a_ders.iter_mut().enumerate().take(num_ders + 1) {
            for (i, basis_der) in ders_basis[k].iter().enumerate().take(self.degree + 1) {
                let idx = span - self.degree + i;
                let p_w = self.control_points[idx].to_homogeneous();
                *a_der += p_w * *basis_der;
            }
        }

        // 有理化（同次座標からの射影導関数）
        let mut ck = vec![Vec3::zeros(); num_ders + 1];
        let mut w_ders = vec![0.0; num_ders + 1];
        for k in 0..=num_ders {
            w_ders[k] = a_ders[k].w;
        }

        for k in 0..=num_ders {
            let mut v = Vec3::new(a_ders[k].x, a_ders[k].y, a_ders[k].z);
            for i in 1..=k {
                let binom = zenith_math::BernsteinPolynomial::binomial(k, i);
                v -= ck[k - i] * (binom * w_ders[i]);
            }
            if w_ders[0].abs() > 1e-15 {
                ck[k] = v / w_ders[0];
            }
        }

        ck
    }

    /// 接線ベクトル（正規化）
    pub fn tangent(&self, u: f64) -> Option<Vec3> {
        let ders = self.evaluate_derivatives(u, 1);
        ders.get(1).and_then(|d1| d1.try_normalize_safe(1e-12))
    }

    /// 曲線上の3次元同次座標ベクトル (wx, wy, wz, w) を評価
    pub fn evaluate_homogeneous(&self, u: f64) -> nalgebra::Vector4<f64> {
        let span = self
            .knots
            .find_span(self.control_points.len(), self.degree, u);
        let basis = self.knots.basis_functions(span, self.degree, u);

        let mut c_w = nalgebra::Vector4::zeros();
        for (i, basis_value) in basis.iter().enumerate().take(self.degree + 1) {
            let idx = span - self.degree + i;
            let p_w = self.control_points[idx].to_homogeneous();
            c_w += p_w * *basis_value;
        }
        c_w
    }

    /// 曲線を指定された制御点数と次数で滑らかに再サンプルし、統一されたNURBS曲線を生成
    pub fn resample_clamped(&self, num_points: usize, target_degree: usize) -> Result<Self, String> {
        let n = num_points.max(target_degree + 1);
        let (u_min, u_max) = self.param_range();

        let mut ctrl_pts = Vec::with_capacity(n);
        for i in 0..n {
            let t = i as f64 / (n - 1) as f64;
            let u = u_min + t * (u_max - u_min);
            let h_pt = self.evaluate_homogeneous(u);
            ctrl_pts.push(ControlPoint3::from_homogeneous(&h_pt));
        }

        let knots = KnotVector::clamped_uniform(n, target_degree);
        Self::new(target_degree, ctrl_pts, knots)
    }

    /// 複数のNURBS曲線の次数と制御点数を互換化（統一）する
    pub fn make_compatible(
        curves: &[NurbsCurve3],
        num_control_points: Option<usize>,
    ) -> Result<Vec<NurbsCurve3>, String> {
        if curves.is_empty() {
            return Ok(Vec::new());
        }

        let first = &curves[0];
        let all_same = curves.iter().all(|c| {
            c.degree == first.degree
                && c.control_points.len() == first.control_points.len()
                && c.knots.knots == first.knots.knots
        });

        if all_same && num_control_points.is_none() {
            return Ok(curves.to_vec());
        }

        let max_degree = curves.iter().map(|c| c.degree).max().unwrap_or(first.degree);
        let max_points = curves
            .iter()
            .map(|c| c.control_points.len())
            .max()
            .unwrap_or(max_degree + 1);
        let target_points = num_control_points.unwrap_or(max_points).max(max_degree + 1);

        let mut compatible = Vec::with_capacity(curves.len());
        for c in curves {
            if c.degree == max_degree && c.control_points.len() == target_points {
                compatible.push(c.clone());
            } else {
                compatible.push(c.resample_clamped(target_points, max_degree)?);
            }
        }

        Ok(compatible)
    }

}


#[cfg(test)]
mod tests {
    use super::{ControlPoint3, NurbsCurve3};
    use crate::bspline_basis::KnotVector;
    use zenith_math::Point3;

    #[test]
    fn reversed_curve_preserves_shape_with_opposite_parameter_direction() {
        let curve = NurbsCurve3::new(
            2,
            vec![
                ControlPoint3::unweighted(Point3::new(0.0, 0.0, 0.0)),
                ControlPoint3::new(Point3::new(5.0, 10.0, 0.0), 0.8),
                ControlPoint3::unweighted(Point3::new(10.0, 0.0, 0.0)),
            ],
            KnotVector::new(vec![0.0, 0.0, 0.0, 2.0, 2.0, 2.0]),
        )
        .unwrap();
        let reversed = curve.reversed();

        assert_eq!(reversed.degree, 2);
        assert_eq!(reversed.knots.knots, vec![0.0, 0.0, 0.0, 2.0, 2.0, 2.0]);
        assert_eq!(reversed.control_points[0], curve.control_points[2]);
        assert!((reversed.evaluate(0.0) - curve.evaluate(2.0)).norm() < 1e-9);
        assert!((reversed.evaluate(0.75) - curve.evaluate(1.25)).norm() < 1e-9);
        assert!((reversed.evaluate(2.0) - curve.evaluate(0.0)).norm() < 1e-9);
    }

    /// 重み 1, √2/2 を繰り返す4弧の有理2次曲線。半径 `r` の真円。
    fn full_circle(r: f64) -> NurbsCurve3 {
        let w = std::f64::consts::FRAC_1_SQRT_2;
        let pts = vec![
            ControlPoint3::unweighted(Point3::new(r, 0.0, 0.0)),
            ControlPoint3::new(Point3::new(r, r, 0.0), w),
            ControlPoint3::unweighted(Point3::new(0.0, r, 0.0)),
            ControlPoint3::new(Point3::new(-r, r, 0.0), w),
            ControlPoint3::unweighted(Point3::new(-r, 0.0, 0.0)),
            ControlPoint3::new(Point3::new(-r, -r, 0.0), w),
            ControlPoint3::unweighted(Point3::new(0.0, -r, 0.0)),
            ControlPoint3::new(Point3::new(r, -r, 0.0), w),
            ControlPoint3::unweighted(Point3::new(r, 0.0, 0.0)),
        ];
        NurbsCurve3::new(
            2,
            pts,
            KnotVector::new(vec![
                0.0, 0.0, 0.0, 0.25, 0.25, 0.5, 0.5, 0.75, 0.75, 1.0, 1.0, 1.0,
            ]),
        )
        .unwrap()
    }

    #[test]
    fn inserting_a_knot_does_not_move_the_curve() {
        let circle = full_circle(10.0);
        let refined = circle.insert_knot(0.375, 1).expect("insert");

        assert_eq!(refined.control_points.len(), circle.control_points.len() + 1);
        assert_eq!(refined.knots.knots.len(), circle.knots.knots.len() + 1);

        // 挿入に使った位置と互いに素な標本で測る。構成点の上だけで比べても
        // 何も分からない（p-curve の検証で一度踏んだ間違い）。
        let mut worst: f64 = 0.0;
        for i in 0..=97 {
            let t = i as f64 / 97.0;
            worst = worst.max((refined.evaluate(t) - circle.evaluate(t)).norm());
        }
        assert!(worst < 1e-12, "knot insertion moved the curve by {worst}");
    }

    #[test]
    fn splitting_a_full_circle_keeps_both_halves_on_the_circle() {
        let r = 10.0;
        let circle = full_circle(r);
        let (left, right) = circle.split_at(0.25).expect("split");

        for (piece, lo, hi) in [(&left, 0.0, 0.25), (&right, 0.25, 1.0)] {
            let (a, b) = piece.param_range();
            assert!((a - lo).abs() < 1e-12 && (b - hi).abs() < 1e-12);
            let mut worst: f64 = 0.0;
            let mut off_circle: f64 = 0.0;
            for i in 0..=53 {
                let t = lo + (hi - lo) * (i as f64 / 53.0);
                let p = piece.evaluate(t);
                worst = worst.max((p - circle.evaluate(t)).norm());
                off_circle = off_circle.max(((p - Point3::origin()).norm() - r).abs());
            }
            assert!(worst < 1e-12, "split piece drifted by {worst}");
            assert!(off_circle < 1e-12, "split piece left the circle by {off_circle}");
        }
    }

    #[test]
    fn splitting_at_an_existing_interior_knot_is_exact() {
        let circle = full_circle(4.0);
        // 0.5 は既に多重度 2（= 次数）なので、挿入なしで割れるはず。
        let (left, right) = circle.split_at(0.5).expect("split at existing knot");
        assert_eq!(left.control_points.len(), 5);
        assert_eq!(right.control_points.len(), 5);
        for i in 0..=41 {
            let t = 0.5 * (i as f64 / 41.0);
            assert!((left.evaluate(t) - circle.evaluate(t)).norm() < 1e-13);
            assert!((right.evaluate(0.5 + t) - circle.evaluate(0.5 + t)).norm() < 1e-13);
        }
    }

    #[test]
    fn a_split_refuses_the_ends_rather_than_returning_a_degenerate_piece() {
        let circle = full_circle(1.0);
        assert!(circle.split_at(0.0).is_none());
        assert!(circle.split_at(1.0).is_none());
        assert!(circle.insert_knot(0.5, 3).is_none());
    }
}
