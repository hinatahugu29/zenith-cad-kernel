/// Bernstein多項式および二項係数の計算ユーティリティ
pub struct BernsteinPolynomial;

impl BernsteinPolynomial {
    /// 二項係数 C(n, k)
    pub fn binomial(n: usize, k: usize) -> f64 {
        if k > n {
            return 0.0;
        }
        if k == 0 || k == n {
            return 1.0;
        }
        let k = k.min(n - k);
        let mut c = 1.0;
        for i in 0..k {
            c = c * (n - i) as f64 / (i + 1) as f64;
        }
        c
    }

    /// Bernstein基底関数の評価 B_{i, n}(t) (t in [0, 1])
    pub fn evaluate(i: usize, n: usize, t: f64) -> f64 {
        if i > n {
            return 0.0;
        }
        let t = t.clamp(0.0, 1.0);
        let c = Self::binomial(n, i);
        c * t.powi(i as i32) * (1.0 - t).powi((n - i) as i32)
    }

    /// すべてのBernstein基底関数 [B_{0,n}(t), ..., B_{n,n}(t)] を一括計算
    pub fn evaluate_all(n: usize, t: f64) -> Vec<f64> {
        let t = t.clamp(0.0, 1.0);
        let mut b = vec![0.0; n + 1];
        b[0] = 1.0;
        let u = 1.0 - t;
        for j in 1..=n {
            let mut saved = 0.0;
            for basis in b.iter_mut().take(j) {
                let temp = *basis;
                *basis = saved + u * temp;
                saved = t * temp;
            }
            b[j] = saved;
        }
        b
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_binomial() {
        assert_eq!(BernsteinPolynomial::binomial(4, 0), 1.0);
        assert_eq!(BernsteinPolynomial::binomial(4, 1), 4.0);
        assert_eq!(BernsteinPolynomial::binomial(4, 2), 6.0);
        assert_eq!(BernsteinPolynomial::binomial(4, 3), 4.0);
        assert_eq!(BernsteinPolynomial::binomial(4, 4), 1.0);
    }

    #[test]
    fn test_partition_of_unity() {
        for degree in 1..=5 {
            let t = 0.35;
            let basis = BernsteinPolynomial::evaluate_all(degree, t);
            let sum: f64 = basis.iter().sum();
            assert!((sum - 1.0).abs() < 1e-12);
        }
    }
}
