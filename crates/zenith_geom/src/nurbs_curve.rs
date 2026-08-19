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
}
