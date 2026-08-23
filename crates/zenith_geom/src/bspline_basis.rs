use serde::{Deserialize, Serialize};

/// B-Spline / NURBS 結び目ベクトル（Knot Vector）
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct KnotVector {
    pub knots: Vec<f64>,
}

impl KnotVector {
    /// 新規作成（昇順チェック）
    pub fn new(knots: Vec<f64>) -> Self {
        assert!(knots.len() >= 2, "Knot vector must have at least 2 knots");
        for i in 0..knots.len() - 1 {
            assert!(
                knots[i] <= knots[i + 1],
                "Knot vector must be non-decreasing: knots[{}] = {} > knots[{}] = {}",
                i,
                knots[i],
                i + 1,
                knots[i + 1]
            );
        }
        Self { knots }
    }

    /// クランプされた均等結び目ベクトルの生成（端点多重度 = degree + 1）
    pub fn clamped_uniform(num_ctrl_pts: usize, degree: usize) -> Self {
        let mut knots = vec![0.0; degree + 1];

        let num_inner = num_ctrl_pts - degree - 1;
        for i in 1..=num_inner {
            knots.push(i as f64 / (num_inner + 1) as f64);
        }

        knots.extend(std::iter::repeat_n(1.0, degree + 1));

        Self { knots }
    }

    /// パラメータ範囲の最小値
    pub fn start_param(&self, degree: usize) -> f64 {
        self.knots[degree]
    }

    /// 反転したノットベクトルを生成
    pub fn reversed(&self) -> Self {
        let n = self.knots.len();
        let u_min = self.knots[0];
        let u_max = self.knots[n - 1];
        let mut new_knots = Vec::with_capacity(n);
        for i in 0..n {
            new_knots.push(u_max - self.knots[n - 1 - i] + u_min);
        }
        Self { knots: new_knots }
    }


    /// パラメータ範囲の最大値
    pub fn end_param(&self, num_ctrl_pts: usize) -> f64 {
        self.knots[num_ctrl_pts]
    }

    /// パラメータ u が属する結び目スパンのインデックス i を検索（Algorithm A2.1 from The NURBS Book）
    pub fn find_span(&self, num_ctrl_pts: usize, degree: usize, u: f64) -> usize {
        let n = num_ctrl_pts - 1;
        let u_end = self.knots[n + 1];

        // 端点処理
        if u >= u_end {
            return n;
        }
        let u_start = self.knots[degree];
        if u <= u_start {
            return degree;
        }

        // 二分探索
        let mut low = degree;
        let mut high = n + 1;
        let mut mid = (low + high) / 2;

        while u < self.knots[mid] || u >= self.knots[mid + 1] {
            if u < self.knots[mid] {
                high = mid;
            } else {
                low = mid;
            }
            mid = (low + high) / 2;
        }

        mid
    }

    /// ゼロでない基底関数の値を評価（Algorithm A2.2 from The NURBS Book）
    pub fn basis_functions(&self, span: usize, degree: usize, u: f64) -> Vec<f64> {
        let mut n = vec![0.0; degree + 1];
        let mut left = vec![0.0; degree + 1];
        let mut right = vec![0.0; degree + 1];
        n[0] = 1.0;

        for j in 1..=degree {
            left[j] = u - self.knots[span + 1 - j];
            right[j] = self.knots[span + j] - u;
            let mut saved = 0.0;

            for r in 0..j {
                let denom = right[r + 1] + left[j - r];
                let temp = if denom.abs() > 1e-15 {
                    n[r] / denom
                } else {
                    0.0
                };
                n[r] = saved + right[r + 1] * temp;
                saved = left[j - r] * temp;
            }
            n[j] = saved;
        }

        n
    }

    /// ゼロでない基底関数の k 階導関数までを評価（Algorithm A2.3 from The NURBS Book）
    pub fn ders_basis_functions(
        &self,
        span: usize,
        degree: usize,
        num_ders: usize,
        u: f64,
    ) -> Vec<Vec<f64>> {
        let p = degree;
        let n_ders = num_ders.min(p);
        let mut ders = vec![vec![0.0; p + 1]; num_ders + 1];

        let mut ndu = vec![vec![0.0; p + 1]; p + 1];
        let mut left = vec![0.0; p + 1];
        let mut right = vec![0.0; p + 1];

        ndu[0][0] = 1.0;
        for j in 1..=p {
            left[j] = u - self.knots[span + 1 - j];
            right[j] = self.knots[span + j] - u;
            let mut saved = 0.0;
            for r in 0..j {
                // Lower triangle
                ndu[j][r] = right[r + 1] + left[j - r];
                let temp = if ndu[j][r].abs() > 1e-15 {
                    ndu[r][j - 1] / ndu[j][r]
                } else {
                    0.0
                };
                // Upper triangle
                ndu[r][j] = saved + right[r + 1] * temp;
                saved = left[j - r] * temp;
            }
            ndu[j][j] = saved;
        }

        // Load the basis functions
        for j in 0..=p {
            ders[0][j] = ndu[j][p];
        }

        // Compute the derivatives
        let mut a = vec![vec![0.0; p + 1]; 2];
        for r in 0..=p {
            let mut s1 = 0;
            let mut s2 = 1;
            a[0][0] = 1.0;

            for k in 1..=n_ders {
                let mut d = 0.0;
                let rk = r as isize - k as isize;
                let pk = p as isize - k as isize;

                if r >= k {
                    a[s2][0] = a[s1][0] / ndu[pk as usize + 1][rk as usize];
                    d = a[s2][0] * ndu[rk as usize][pk as usize];
                }

                let j1 = if rk >= -1 { 1 } else { (-rk) as usize };
                let j2 = if (r as isize - 1) <= pk { k - 1 } else { p - r };

                for j in j1..=j2 {
                    a[s2][j] = (a[s1][j] - a[s1][j - 1])
                        / ndu[pk as usize + 1][(rk + j as isize) as usize];
                    d += a[s2][j] * ndu[(rk + j as isize) as usize][pk as usize];
                }

                if (r as isize) <= pk {
                    a[s2][k] = -a[s1][k - 1] / ndu[pk as usize + 1][r];
                    d += a[s2][k] * ndu[r][pk as usize];
                }

                ders[k][r] = d;
                std::mem::swap(&mut s1, &mut s2);
            }
        }

        // Multiply by the factors (p! / (p - k)!)
        let mut acc = p as f64;
        for (k, ders_k) in ders.iter_mut().enumerate().take(n_ders + 1).skip(1) {
            for value in ders_k.iter_mut().take(p + 1) {
                *value *= acc;
            }
            acc *= (p - k) as f64;
        }

        ders
    }
}
