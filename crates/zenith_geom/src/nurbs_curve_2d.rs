use crate::bspline_basis::KnotVector;
use serde::{Deserialize, Serialize};
use zenith_math::{Point2, Vec2};

/// 2次元制御点（UV空間座標 + 重み）
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct ControlPoint2 {
    pub point: Point2,
    pub weight: f64,
}

impl ControlPoint2 {
    pub fn new(point: Point2, weight: f64) -> Self {
        Self { point, weight }
    }

    pub fn unweighted(point: Point2) -> Self {
        Self { point, weight: 1.0 }
    }

    pub fn to_homogeneous(&self) -> nalgebra::Vector3<f64> {
        nalgebra::Vector3::new(
            self.point.x * self.weight,
            self.point.y * self.weight,
            self.weight,
        )
    }

    pub fn from_homogeneous(v: &nalgebra::Vector3<f64>) -> Self {
        let w = v.z;
        if w.abs() > 1e-15 {
            Self {
                point: Point2::new(v.x / w, v.y / w),
                weight: w,
            }
        } else {
            Self {
                point: Point2::new(v.x, v.y),
                weight: w,
            }
        }
    }
}

/// 2次元 NURBS / B-Spline 曲線（曲面上のUVパラメータ空間トリム曲線: PCurve）
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct NurbsCurve2 {
    pub degree: usize,
    pub control_points: Vec<ControlPoint2>,
    pub knots: KnotVector,
}

impl NurbsCurve2 {
    pub fn new(
        degree: usize,
        control_points: Vec<ControlPoint2>,
        knots: KnotVector,
    ) -> Result<Self, String> {
        let n = control_points.len();
        if n < degree + 1 {
            return Err(format!(
                "Control points count ({}) must be >= degree + 1 ({})",
                n,
                degree + 1
            ));
        }
        let expected_knots = n + degree + 1;
        if knots.knots.len() != expected_knots {
            return Err(format!(
                "Knot vector length mismatch: {} vs {}",
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

    pub fn bspline_from_points(degree: usize, points: Vec<Point2>) -> Result<Self, String> {
        let n = points.len();
        let knots = KnotVector::clamped_uniform(n, degree);
        let ctrl_pts = points.into_iter().map(ControlPoint2::unweighted).collect();
        Self::new(degree, ctrl_pts, knots)
    }

    pub fn param_range(&self) -> (f64, f64) {
        (
            self.knots.start_param(self.degree),
            self.knots.end_param(self.control_points.len()),
        )
    }

    pub fn evaluate(&self, t: f64) -> Point2 {
        let span = self
            .knots
            .find_span(self.control_points.len(), self.degree, t);
        let basis = self.knots.basis_functions(span, self.degree, t);

        let mut c_w = nalgebra::Vector3::zeros();
        for (i, basis_value) in basis.iter().enumerate().take(self.degree + 1) {
            let idx = span - self.degree + i;
            let pw = self.control_points[idx].to_homogeneous();
            c_w += pw * *basis_value;
        }

        ControlPoint2::from_homogeneous(&c_w).point
    }

    /// サンプル点列の取得
    /// パラメータ `t` における1階微分 dC/dt（有理曲線の商の微分則）
    ///
    /// トリム境界に沿った線積分（面積・モーメントの厳密計算）に必要。
    pub fn evaluate_derivative(&self, t: f64) -> Vec2 {
        let count = self.control_points.len();
        let span = self.knots.find_span(count, self.degree, t);
        let ders = self.knots.ders_basis_functions(span, self.degree, 1, t);

        let mut value = nalgebra::Vector3::zeros();
        let mut slope = nalgebra::Vector3::zeros();
        for (i, (basis, basis_slope)) in ders[0]
            .iter()
            .zip(ders[1].iter())
            .enumerate()
            .take(self.degree + 1)
        {
            let control_point = self.control_points[span - self.degree + i].to_homogeneous();
            value += control_point * *basis;
            slope += control_point * *basis_slope;
        }

        let weight = value.z;
        if weight.abs() <= 1e-15 {
            return Vec2::zeros();
        }
        let point = Vec2::new(value.x, value.y) / weight;

        (Vec2::new(slope.x, slope.y) - point * slope.z) / weight
    }

    pub fn sample_points(&self, num_samples: usize) -> Vec<Point2> {
        let (t_min, t_max) = self.param_range();
        let n = num_samples.max(2);
        let mut pts = Vec::with_capacity(n);
        for i in 0..n {
            let t = t_min + (t_max - t_min) * (i as f64 / (n - 1) as f64);
            pts.push(self.evaluate(t));
        }
        pts
    }
}
