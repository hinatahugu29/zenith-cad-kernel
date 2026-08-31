use nalgebra::{DMatrix, DVector};
use serde::{Deserialize, Serialize};

/// スケッチ要素ID
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub struct PointId(pub usize);

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub struct LineId(pub usize);

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub struct CircleId(pub usize);

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub struct ArcId(pub usize);

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

/// 2D円弧要素。
///
/// **中心・始点・終点は、どれも普通の点です。** 弧に専用の拘束を足して
/// いないのは、そうする必要がないからです——半径を揃えたければ中心から
/// 両端への `Distance` を、端を繋げたければ `Coincident` を掛ければ済みます。
/// **いまある拘束がそのまま効きます。**
///
/// 半径は「中心から始点まで」で決まります。終点がそこから外れていたら、
/// **推測せずに断ります**（[`crate::extract_loops`]）。
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct SketchArc {
    pub id: ArcId,
    pub center: PointId,
    pub start: PointId,
    pub end: PointId,
    /// 始点から終点へ、反時計回りに回るか。
    pub counterclockwise: bool,
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
    #[serde(default)]
    pub arcs: Vec<SketchArc>,
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

    /// 円弧を足す。**中心・始点・終点は既にある点**です。
    pub fn add_arc(
        &mut self,
        center: PointId,
        start: PointId,
        end: PointId,
        counterclockwise: bool,
    ) -> ArcId {
        let id = ArcId(self.arcs.len());
        self.arcs.push(SketchArc {
            id,
            center,
            start,
            end,
            counterclockwise,
        });
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

            // `is_fixed` を立てた点は動かさない。`add_fixed_point` は
            // `FixedPoint` 拘束も足すのでそちらでも止まるが、フィールドは
            // 公開されている。読まれない公開フィールドは、立てた人が「効いた」
            // と思い込む罠になる。
            let mut delta = delta;
            for (index, point) in self.points.iter().enumerate() {
                if point.is_fixed {
                    delta[2 * index] = 0.0;
                    delta[2 * index + 1] = 0.0;
                }
            }

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

    /// スケッチの自由度 (DOF: Degrees of Freedom) を解析
    /// 戻り値: (総変数自由度, 有効拘束ランク, 残余自由度)
    pub fn degrees_of_freedom(&self) -> (usize, usize, usize) {
        let analysis = self.analyse_freedom();
        (analysis.total_dof, analysis.rank, analysis.remaining_dof)
    }

    fn analyse_freedom(&self) -> FreedomAnalysis {
        let num_points = self.points.len();
        if num_points == 0 {
            return FreedomAnalysis::default();
        }
        let num_vars = num_points * 2;
        let mut x = DVector::zeros(num_vars);
        for (i, p) in self.points.iter().enumerate() {
            x[2 * i] = p.x;
            x[2 * i + 1] = p.y;
        }
        let (_, j) = self.eval_residuals_and_jacobian(&x);

        // ヤコビアンは全点ぶんの列を持っている（`eval_residuals_and_jacobian` は
        // `num_vars = 全点 x 2` で組む）。自由度と突き合わせるには、**動かせない
        // 点の列を落としてから**階数を取らなければならない。落とさずに数えて
        // いたので、階数が自由度を超えるという幾何学的にありえない値が出ていた
        // （`sketch_solver_probe` に「Total DOF = 2, Rank = 5」と印字されていた）。
        // `saturating_sub` がそれを 0 に丸めるため、自由度が残っているスケッチが
        // `FullyConstrained` と報告されていた。
        let free_columns: Vec<usize> = self
            .points
            .iter()
            .enumerate()
            .filter(|(_, point)| !point.is_fixed)
            .flat_map(|(index, _)| [2 * index, 2 * index + 1])
            .collect();
        let total_dof = free_columns.len();

        if j.nrows() == 0 || j.ncols() == 0 || total_dof == 0 {
            return FreedomAnalysis {
                total_dof,
                rank: 0,
                remaining_dof: total_dof,
                active_equations: 0,
            };
        }

        let free_jacobian = DMatrix::from_fn(j.nrows(), total_dof, |row, column| {
            j[(row, free_columns[column])]
        });

        // 自由変数に何も掛けていない式（`add_fixed_point` が入れる FixedPoint の
        // 2本など）は、制限したヤコビアンでは丸ごとゼロ行になる。階数には
        // 入りようがないので、冗長さを数えるときの式の本数からも外す。外さずに
        // 全拘束を数えると、点を1つ固定しただけで「冗長が2本ある」と報告する。
        let active_equations = (0..free_jacobian.nrows())
            .filter(|row| (0..total_dof).any(|column| free_jacobian[(*row, column)].abs() > 1e-12))
            .count();

        let svd = free_jacobian.svd(false, false);
        let tol = 1e-7;
        let rank = svd.singular_values.iter().filter(|&&s| s > tol).count();
        let remaining_dof = total_dof.saturating_sub(rank);
        FreedomAnalysis {
            total_dof,
            rank,
            remaining_dof,
            active_equations,
        }
    }

    /// 現在のスケッチの拘束状態を判定
    pub fn constraint_status(&self) -> SketchConstraintStatus {
        let FreedomAnalysis {
            total_dof,
            rank,
            remaining_dof,
            active_equations,
        } = self.analyse_freedom();
        if self.constraints.is_empty() && total_dof > 0 {
            return SketchConstraintStatus::UnderConstrained { remaining_dof };
        }
        // 式の本数は拘束の種類から数えるのではなく、**自由変数に実際に掛かって
        // いる行を数える**。種類から数えると、点の固定のように自由変数へ一切
        // 掛からない式まで冗長側に積み上がる。
        let num_eqs = active_equations;

        if num_eqs > rank && remaining_dof == 0 {
            SketchConstraintStatus::OverConstrained {
                redundant_constraints: num_eqs - rank,
            }
        } else if remaining_dof == 0 {
            SketchConstraintStatus::FullyConstrained
        } else {
            SketchConstraintStatus::UnderConstrained { remaining_dof }
        }
    }
}

/// スケッチ拘束状態
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum SketchConstraintStatus {
    /// 不足拘束（残余自由度あり）
    UnderConstrained { remaining_dof: usize },
    /// 完全拘束（形状が一意に決定）
    FullyConstrained,
    /// 過剰拘束（冗長または矛盾する拘束が存在）
    OverConstrained { redundant_constraints: usize },
}

/// 自由度解析の中間結果。
///
/// `degrees_of_freedom` の戻り値は3つ組で固定されているので、拘束状態の判定に
/// 必要な「自由変数に実際に掛かっている式の本数」を運ぶためにこれを使う。
#[derive(Debug, Clone, Copy, Default)]
struct FreedomAnalysis {
    total_dof: usize,
    rank: usize,
    remaining_dof: usize,
    active_equations: usize,
}
