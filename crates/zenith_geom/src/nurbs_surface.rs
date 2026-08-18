use crate::bspline_basis::KnotVector;
use crate::nurbs_curve::ControlPoint3;
use serde::{Deserialize, Serialize};
use zenith_math::{Point3, Vec3, Vec3Ext};

/// 3次元 NURBS / B-Spline 曲面
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct NurbsSurface3 {
    pub degree_u: usize,
    pub degree_v: usize,
    /// 制御点グリッド: [row_u][col_v]
    pub control_points: Vec<Vec<ControlPoint3>>,
    pub knots_u: KnotVector,
    pub knots_v: KnotVector,
}

impl NurbsSurface3 {
    /// 新規作成（バリデーション付き）
    pub fn new(
        degree_u: usize,
        degree_v: usize,
        control_points: Vec<Vec<ControlPoint3>>,
        knots_u: KnotVector,
        knots_v: KnotVector,
    ) -> Result<Self, String> {
        let num_u = control_points.len();
        if num_u < degree_u + 1 {
            return Err(format!(
                "Number of U control points ({}) must be >= degree_u + 1 ({})",
                num_u,
                degree_u + 1
            ));
        }
        let num_v = control_points[0].len();
        if num_v < degree_v + 1 {
            return Err(format!(
                "Number of V control points ({}) must be >= degree_v + 1 ({})",
                num_v,
                degree_v + 1
            ));
        }
        for (i, row) in control_points.iter().enumerate() {
            if row.len() != num_v {
                return Err(format!(
                    "Row {} has length {} but expected {}",
                    i,
                    row.len(),
                    num_v
                ));
            }
        }

        let expected_knots_u = num_u + degree_u + 1;
        if knots_u.knots.len() != expected_knots_u {
            return Err(format!(
                "Knot vector U length ({}) does not match expected ({})",
                knots_u.knots.len(),
                expected_knots_u
            ));
        }

        let expected_knots_v = num_v + degree_v + 1;
        if knots_v.knots.len() != expected_knots_v {
            return Err(format!(
                "Knot vector V length ({}) does not match expected ({})",
                knots_v.knots.len(),
                expected_knots_v
            ));
        }

        Ok(Self {
            degree_u,
            degree_v,
            control_points,
            knots_u,
            knots_v,
        })
    }

    /// パラメータ有効範囲 [u_min, u_max] x [v_min, v_max]
    pub fn param_range(&self) -> ((f64, f64), (f64, f64)) {
        let u_range = (
            self.knots_u.start_param(self.degree_u),
            self.knots_u.end_param(self.control_points.len()),
        );
        let v_range = (
            self.knots_v.start_param(self.degree_v),
            self.knots_v.end_param(self.control_points[0].len()),
        );
        (u_range, v_range)
    }

    /// 曲面上の3次元座標を評価（Algorithm A3.5 / A4.3 from The NURBS Book）
    pub fn evaluate(&self, u: f64, v: f64) -> Point3 {
        let num_u = self.control_points.len();
        let num_v = self.control_points[0].len();

        let span_u = self.knots_u.find_span(num_u, self.degree_u, u);
        let span_v = self.knots_v.find_span(num_v, self.degree_v, v);

        let basis_u = self.knots_u.basis_functions(span_u, self.degree_u, u);
        let basis_v = self.knots_v.basis_functions(span_v, self.degree_v, v);

        let mut s_w = nalgebra::Vector4::zeros();

        for (l, basis_v_value) in basis_v.iter().enumerate().take(self.degree_v + 1) {
            let mut temp = nalgebra::Vector4::zeros();
            let v_idx = span_v - self.degree_v + l;
            for (k, basis_u_value) in basis_u.iter().enumerate().take(self.degree_u + 1) {
                let u_idx = span_u - self.degree_u + k;
                let pw = self.control_points[u_idx][v_idx].to_homogeneous();
                temp += pw * *basis_u_value;
            }
            s_w += temp * *basis_v_value;
        }

        ControlPoint3::from_homogeneous(&s_w).point
    }

    /// 曲面上の点および U方向・V方向の偏導関数 (Du, Dv) を評価
    pub fn evaluate_derivatives_1st(&self, u: f64, v: f64) -> (Point3, Vec3, Vec3) {
        let num_u = self.control_points.len();
        let num_v = self.control_points[0].len();

        let span_u = self.knots_u.find_span(num_u, self.degree_u, u);
        let span_v = self.knots_v.find_span(num_v, self.degree_v, v);

        let ders_u = self
            .knots_u
            .ders_basis_functions(span_u, self.degree_u, 1, u);
        let ders_v = self
            .knots_v
            .ders_basis_functions(span_v, self.degree_v, 1, v);

        // a_{k, l} (k in 0..=1, l in 0..=1)
        let mut a = vec![vec![nalgebra::Vector4::zeros(); 2]; 2];

        for k in 0..=1 {
            for l in 0..=1 {
                for (j, ders_v_value) in ders_v[l].iter().enumerate().take(self.degree_v + 1) {
                    let v_idx = span_v - self.degree_v + j;
                    let mut temp = nalgebra::Vector4::zeros();
                    for (i, ders_u_value) in ders_u[k].iter().enumerate().take(self.degree_u + 1) {
                        let u_idx = span_u - self.degree_u + i;
                        let pw = self.control_points[u_idx][v_idx].to_homogeneous();
                        temp += pw * *ders_u_value;
                    }
                    a[k][l] += temp * *ders_v_value;
                }
            }
        }

        let p = ControlPoint3::from_homogeneous(&a[0][0]).point;
        let w = a[0][0].w;

        // dS/du = (a10 - p * w10) / w
        let a10 = Vec3::new(a[1][0].x, a[1][0].y, a[1][0].z);
        let w10 = a[1][0].w;
        let du = if w.abs() > 1e-15 {
            (a10 - p.coords * w10) / w
        } else {
            Vec3::zeros()
        };

        // dS/dv = (a01 - p * w01) / w
        let a01 = Vec3::new(a[0][1].x, a[0][1].y, a[0][1].z);
        let w01 = a[0][1].w;
        let dv = if w.abs() > 1e-15 {
            (a01 - p.coords * w01) / w
        } else {
            Vec3::zeros()
        };

        (p, du, dv)
    }

    /// 曲面の法線ベクトル（正規化）
    pub fn normal(&self, u: f64, v: f64) -> Option<Vec3> {
        let (_p, du, dv) = self.evaluate_derivatives_1st(u, v);
        let cross = du.cross(&dv);
        cross.try_normalize_safe(1e-12)
    }
}
