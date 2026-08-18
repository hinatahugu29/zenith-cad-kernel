use nalgebra::{DMatrix, DVector};
use serde::{Deserialize, Serialize};

/// スケッチ要素ID
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub struct PointId(pub usize);

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub struct LineId(pub usize);

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub struct CircleId(pub usize);

/// 2D点要素
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct SketchPoint {
    pub id: PointId,
    pub x: f64,
    pub y: f64,
    pub is_fixed: bool,
}

/// 2D線分要素
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct SketchLine {
    pub id: LineId,
    pub p1: PointId,
    pub p2: PointId,
}

/// 2D円要素
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct SketchCircle {
    pub id: CircleId,
    pub center: PointId,
    pub radius: f64,
}

/// 幾何拘束
#[derive(Debug, Clone, Serialize, Deserialize)]
pub enum Constraint {
    /// 2点の一致
    Coincident(PointId, PointId),
    /// 2点を結ぶ線分が水平 (y1 = y2)
    Horizontal(PointId, PointId),
    /// 2点を結ぶ線分が垂直 (x1 = x2)
    Vertical(PointId, PointId),
    /// 2点間の距離
    Distance(PointId, PointId, f64),
    /// 2点間の水平距離 (x2 - x1 = d)
    HorizontalDistance(PointId, PointId, f64),
    /// 2点間の垂直距離 (y2 - y1 = d)
    VerticalDistance(PointId, PointId, f64),
    /// 2線分が平行
    Parallel(LineId, LineId),
    /// 2線分が直交
    Perpendicular(LineId, LineId),
    /// 線分と円が接する
    TangentLineCircle(LineId, CircleId),
    /// 2線分の長さが等しい
    EqualLength(LineId, LineId),
    /// 円の半径固定
    Radius(CircleId, f64),
    /// 点の座標固定
    FixedPoint(PointId, f64, f64),
}

/// 2Dスケッチ拘束ソルバー
#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct SketchSolver {
    pub points: Vec<SketchPoint>,
    pub lines: Vec<SketchLine>,
    pub circles: Vec<SketchCircle>,
    pub constraints: Vec<Constraint>,
}

impl SketchSolver {
    pub fn new() -> Self {
        Self::default()
    }

    pub fn add_point(&mut self, x: f64, y: f64) -> PointId {
        let id = PointId(self.points.len());
        self.points.push(SketchPoint {
            id,
            x,
            y,
            is_fixed: false,
        });
        id
    }

    pub fn add_fixed_point(&mut self, x: f64, y: f64) -> PointId {
        let id = PointId(self.points.len());
        self.points.push(SketchPoint {
            id,
            x,
            y,
            is_fixed: true,
        });
        self.constraints.push(Constraint::FixedPoint(id, x, y));
        id
    }

    pub fn add_line(&mut self, p1: PointId, p2: PointId) -> LineId {
        let id = LineId(self.lines.len());
        self.lines.push(SketchLine { id, p1, p2 });
        id
    }

    pub fn add_circle(&mut self, center: PointId, radius: f64) -> CircleId {
        let id = CircleId(self.circles.len());
        self.circles.push(SketchCircle { id, center, radius });
        id
    }

    pub fn add_constraint(&mut self, constraint: Constraint) {
        self.constraints.push(constraint);
    }

    pub fn get_point(&self, id: PointId) -> Option<[f64; 2]> {
        self.points.get(id.0).map(|p| [p.x, p.y])
    }

    /// 拘束方程式系を解き、全点の座標を最適化
    pub fn solve(&mut self, max_iterations: usize, tolerance: f64) -> Result<usize, String> {
        let num_points = self.points.len();
        if num_points == 0 || self.constraints.is_empty() {
            return Ok(0);
        }

        // 変数ベクトル X = [x0, y0, x1, y1, ...]
        let num_vars = num_points * 2;
        let mut x = DVector::zeros(num_vars);
        for (i, p) in self.points.iter().enumerate() {
            x[2 * i] = p.x;
            x[2 * i + 1] = p.y;
        }

        let mut lambda = 1e-3; // Levenberg-Marquardt 減衰係数

        for iter in 0..max_iterations {
            let (f, j) = self.eval_residuals_and_jacobian(&x);
            let residual_norm = f.norm();

            if residual_norm < tolerance {
                // 収束達成
                for (i, p) in self.points.iter_mut().enumerate() {
                    p.x = x[2 * i];
                    p.y = x[2 * i + 1];
                }
                return Ok(iter);
            }

            // Gauss-Newton / LM 法: (J^T J + lambda * I) delta = -J^T F
            let jt = j.transpose();
            let jtj = &jt * &j;
            let jtf = &jt * &f;

            let mut jtj_lm = jtj.clone();
            for i in 0..num_vars {
                jtj_lm[(i, i)] += lambda * (jtj_lm[(i, i)].max(1.0));
            }

            // コレスキーまたはQR分解で解を計算
            let delta = match jtj_lm.qr().solve(&(-jtf)) {
                Some(d) => d,
                None => return Err(format!("Singular Jacobian at iteration {}", iter)),
            };

            let x_new = &x + &delta;
            let (f_new, _) = self.eval_residuals_and_jacobian(&x_new);

            if f_new.norm() < residual_norm {
                x = x_new;
                lambda = (lambda * 0.5).max(1e-7);
            } else {
                lambda = (lambda * 5.0).min(1e5);
            }
        }

        // 最終チェック
        let (f_final, _) = self.eval_residuals_and_jacobian(&x);
        if f_final.norm() < tolerance * 10.0 {
            for (i, p) in self.points.iter_mut().enumerate() {
                p.x = x[2 * i];
                p.y = x[2 * i + 1];
            }
            Ok(max_iterations)
        } else {
            Err(format!(
                "Constraint solver failed to converge: residual = {}",
                f_final.norm()
            ))
        }
    }

    /// 各拘束条件に対する残差ベクトル F とヤコビ行列 J を評価
    fn eval_residuals_and_jacobian(&self, x: &DVector<f64>) -> (DVector<f64>, DMatrix<f64>) {
        let mut residuals = Vec::new();
        let num_vars = x.len();
        let mut jacobian_rows: Vec<DVector<f64>> = Vec::new();

        for c in &self.constraints {
            match *c {
                Constraint::Coincident(p1, p2) => {
                    let (i1, i2) = (p1.0, p2.0);
                    // x1 - x2 = 0
                    residuals.push(x[2 * i1] - x[2 * i2]);
                    let mut row_x = DVector::zeros(num_vars);
                    row_x[2 * i1] = 1.0;
                    row_x[2 * i2] = -1.0;
                    jacobian_rows.push(row_x);

                    // y1 - y2 = 0
                    residuals.push(x[2 * i1 + 1] - x[2 * i2 + 1]);
                    let mut row_y = DVector::zeros(num_vars);
                    row_y[2 * i1 + 1] = 1.0;
                    row_y[2 * i2 + 1] = -1.0;
                    jacobian_rows.push(row_y);
                }
                Constraint::Horizontal(p1, p2) => {
                    let (i1, i2) = (p1.0, p2.0);
                    // y2 - y1 = 0
                    residuals.push(x[2 * i2 + 1] - x[2 * i1 + 1]);
                    let mut row = DVector::zeros(num_vars);
                    row[2 * i1 + 1] = -1.0;
                    row[2 * i2 + 1] = 1.0;
                    jacobian_rows.push(row);
                }
                Constraint::Vertical(p1, p2) => {
                    let (i1, i2) = (p1.0, p2.0);
                    // x2 - x1 = 0
                    residuals.push(x[2 * i2] - x[2 * i1]);
                    let mut row = DVector::zeros(num_vars);
                    row[2 * i1] = -1.0;
                    row[2 * i2] = 1.0;
                    jacobian_rows.push(row);
                }
                Constraint::Distance(p1, p2, target_d) => {
                    let (i1, i2) = (p1.0, p2.0);
                    let dx = x[2 * i2] - x[2 * i1];
                    let dy = x[2 * i2 + 1] - x[2 * i1 + 1];
                    let current_d = (dx * dx + dy * dy).sqrt().max(1e-12);

                    residuals.push(current_d - target_d);
                    let mut row = DVector::zeros(num_vars);
                    row[2 * i1] = -dx / current_d;
                    row[2 * i1 + 1] = -dy / current_d;
                    row[2 * i2] = dx / current_d;
                    row[2 * i2 + 1] = dy / current_d;
                    jacobian_rows.push(row);
                }
                Constraint::HorizontalDistance(p1, p2, target_d) => {
                    let (i1, i2) = (p1.0, p2.0);
                    residuals.push((x[2 * i2] - x[2 * i1]) - target_d);
                    let mut row = DVector::zeros(num_vars);
                    row[2 * i1] = -1.0;
                    row[2 * i2] = 1.0;
                    jacobian_rows.push(row);
                }
                Constraint::VerticalDistance(p1, p2, target_d) => {
                    let (i1, i2) = (p1.0, p2.0);
                    residuals.push((x[2 * i2 + 1] - x[2 * i1 + 1]) - target_d);
                    let mut row = DVector::zeros(num_vars);
                    row[2 * i1 + 1] = -1.0;
                    row[2 * i2 + 1] = 1.0;
                    jacobian_rows.push(row);
                }
                Constraint::Parallel(l1, l2) => {
                    let line1 = &self.lines[l1.0];
                    let line2 = &self.lines[l2.0];
                    let (i1a, i1b) = (line1.p1.0, line1.p2.0);
                    let (i2a, i2b) = (line2.p1.0, line2.p2.0);

                    let dx1 = x[2 * i1b] - x[2 * i1a];
                    let dy1 = x[2 * i1b + 1] - x[2 * i1a + 1];
                    let dx2 = x[2 * i2b] - x[2 * i2a];
                    let dy2 = x[2 * i2b + 1] - x[2 * i2a + 1];

                    // 外積: dx1 * dy2 - dy1 * dx2 = 0
                    residuals.push(dx1 * dy2 - dy1 * dx2);
                    let mut row = DVector::zeros(num_vars);
                    row[2 * i1a] = -dy2;
                    row[2 * i1a + 1] = dx2;
                    row[2 * i1b] = dy2;
                    row[2 * i1b + 1] = -dx2;

                    row[2 * i2a] = dy1;
                    row[2 * i2a + 1] = -dx1;
                    row[2 * i2b] = -dy1;
                    row[2 * i2b + 1] = dx1;
                    jacobian_rows.push(row);
                }
                Constraint::Perpendicular(l1, l2) => {
                    let line1 = &self.lines[l1.0];
                    let line2 = &self.lines[l2.0];
                    let (i1a, i1b) = (line1.p1.0, line1.p2.0);
                    let (i2a, i2b) = (line2.p1.0, line2.p2.0);

                    let dx1 = x[2 * i1b] - x[2 * i1a];
                    let dy1 = x[2 * i1b + 1] - x[2 * i1a + 1];
                    let dx2 = x[2 * i2b] - x[2 * i2a];
                    let dy2 = x[2 * i2b + 1] - x[2 * i2a + 1];

                    // 内積: dx1 * dx2 + dy1 * dy2 = 0
                    residuals.push(dx1 * dx2 + dy1 * dy2);
                    let mut row = DVector::zeros(num_vars);
                    row[2 * i1a] = -dx2;
                    row[2 * i1a + 1] = -dy2;
                    row[2 * i1b] = dx2;
                    row[2 * i1b + 1] = dy2;

                    row[2 * i2a] = -dx1;
                    row[2 * i2a + 1] = -dy1;
                    row[2 * i2b] = dx1;
                    row[2 * i2b + 1] = dy1;
                    jacobian_rows.push(row);
                }
                Constraint::EqualLength(l1, l2) => {
                    let line1 = &self.lines[l1.0];
                    let line2 = &self.lines[l2.0];
                    let (i1a, i1b) = (line1.p1.0, line1.p2.0);
                    let (i2a, i2b) = (line2.p1.0, line2.p2.0);

                    let dx1 = x[2 * i1b] - x[2 * i1a];
                    let dy1 = x[2 * i1b + 1] - x[2 * i1a + 1];
                    let len1 = (dx1 * dx1 + dy1 * dy1).sqrt().max(1e-12);

                    let dx2 = x[2 * i2b] - x[2 * i2a];
                    let dy2 = x[2 * i2b + 1] - x[2 * i2a + 1];
                    let len2 = (dx2 * dx2 + dy2 * dy2).sqrt().max(1e-12);

                    residuals.push(len1 - len2);
                    let mut row = DVector::zeros(num_vars);
                    row[2 * i1a] = -dx1 / len1;
                    row[2 * i1a + 1] = -dy1 / len1;
                    row[2 * i1b] = dx1 / len1;
                    row[2 * i1b + 1] = dy1 / len1;

                    row[2 * i2a] = dx2 / len2;
                    row[2 * i2a + 1] = dy2 / len2;
                    row[2 * i2b] = -dx2 / len2;
                    row[2 * i2b + 1] = -dy2 / len2;
                    jacobian_rows.push(row);
                }
                Constraint::TangentLineCircle(l, c_id) => {
                    let line = &self.lines[l.0];
                    let circle = &self.circles[c_id.0];
                    let (i1, i2) = (line.p1.0, line.p2.0);
                    let ic = circle.center.0;
                    let r = circle.radius;

                    let x1 = x[2 * i1];
                    let y1 = x[2 * i1 + 1];
                    let x2 = x[2 * i2];
                    let y2 = x[2 * i2 + 1];
                    let xc = x[2 * ic];
                    let yc = x[2 * ic + 1];

                    let dx = x2 - x1;
                    let dy = y2 - y1;
                    let len = (dx * dx + dy * dy).sqrt().max(1e-12);

                    // 点と直線の距離: |(x2 - x1)(y1 - yc) - (y2 - y1)(x1 - xc)| / len = r
                    let cross = dx * (y1 - yc) - dy * (x1 - xc);
                    residuals.push(cross.abs() / len - r);

                    // 数値微分近似でヤコビアン行を計算
                    let eps = 1e-7;
                    let mut row = DVector::zeros(num_vars);
                    for idx in [2 * i1, 2 * i1 + 1, 2 * i2, 2 * i2 + 1, 2 * ic, 2 * ic + 1] {
                        let mut x_plus = x.clone();
                        x_plus[idx] += eps;
                        let dx_p = x_plus[2 * i2] - x_plus[2 * i1];
                        let dy_p = x_plus[2 * i2 + 1] - x_plus[2 * i1 + 1];
                        let len_p = (dx_p * dx_p + dy_p * dy_p).sqrt().max(1e-12);
                        let cross_p = dx_p * (x_plus[2 * i1 + 1] - x_plus[2 * ic + 1])
                            - dy_p * (x_plus[2 * i1] - x_plus[2 * ic]);
                        let res_p = cross_p.abs() / len_p - r;
                        row[idx] = (res_p - (cross.abs() / len - r)) / eps;
                    }
                    jacobian_rows.push(row);
                }
                Constraint::Radius(_c_id, _r) => {
                    // 半径は円の固定値パラメータ
                }
                Constraint::FixedPoint(p, fix_x, fix_y) => {
                    let i = p.0;
                    residuals.push(x[2 * i] - fix_x);
                    let mut row_x = DVector::zeros(num_vars);
                    row_x[2 * i] = 1.0;
                    jacobian_rows.push(row_x);

                    residuals.push(x[2 * i + 1] - fix_y);
                    let mut row_y = DVector::zeros(num_vars);
                    row_y[2 * i + 1] = 1.0;
                    jacobian_rows.push(row_y);
                }
            }
        }

        let num_eqs = residuals.len();
        let f_vec = DVector::from_vec(residuals);
        let mut j_mat = DMatrix::zeros(num_eqs, num_vars);
        for (r, row) in jacobian_rows.iter().enumerate() {
            for c in 0..num_vars {
                j_mat[(r, c)] = row[c];
            }
        }

        (f_vec, j_mat)
    }
}
