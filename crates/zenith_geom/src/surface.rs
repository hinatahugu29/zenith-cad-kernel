use crate::coons_patch::CoonsPatch3;
use crate::nurbs_surface::NurbsSurface3;
use serde::{Deserialize, Serialize};
use zenith_math::{Point3, Vec3, Vec3Ext};

/// 汎用3次元曲面トレイト
pub trait Surface3: std::fmt::Debug + Send + Sync {
    /// UVパラメータ範囲 [u_min, u_max] x [v_min, v_max]
    fn param_range(&self) -> ((f64, f64), (f64, f64));
    /// UV座標における3次元座標の評価
    fn evaluate(&self, u: f64, v: f64) -> Point3;
    /// UV座標における法線ベクトルの評価（正規化）
    fn normal(&self, u: f64, v: f64) -> Option<Vec3>;

    /// 積分をここで切らなければならないパラメータ値（内部ノット）。
    ///
    /// B-spline が滑らかなのは各ノット区間の**内側だけ**である。区間を
    /// またぐ三角形の上で高次の求積を当てると、折れた被積分関数を見ることに
    /// なり、次数が効かず2次までしか落ちない。実測では、他カーネルの円柱の
    /// 側面（有理2次・3スパン）が、**12分割で 1.98e-9、16分割で 1.72e-4**
    /// になる。12 は 1/3 と 2/3 にちょうど乗り、16 は乗らない。乗らない側は
    /// 512分割まで刻んでも 1.68e-7 で、12分割に届かない。
    ///
    /// 既定は空で、ノットを持たない曲面は何も払わない。
    fn integration_breaks(&self) -> (Vec<f64>, Vec<f64>) {
        (Vec::new(), Vec::new())
    }

    /// UV座標における点と1階偏微分 (S, dS/du, dS/dv)
    ///
    /// 面積分（面積・体積・重心）に必要な面素 `dS/du x dS/dv` を得るための
    /// 入口。既定実装は中心差分で、解析微分を持つ曲面はこれを上書きする。
    fn evaluate_with_derivatives(&self, u: f64, v: f64) -> (Point3, Vec3, Vec3) {
        let ((u_min, u_max), (v_min, v_max)) = self.param_range();
        let du_step = ((u_max - u_min) * 1e-6).max(1e-9);
        let dv_step = ((v_max - v_min) * 1e-6).max(1e-9);

        let (u_low, u_high) = ((u - du_step).max(u_min), (u + du_step).min(u_max));
        let (v_low, v_high) = ((v - dv_step).max(v_min), (v + dv_step).min(v_max));

        let point = self.evaluate(u, v);
        let du = if u_high > u_low {
            (self.evaluate(u_high, v) - self.evaluate(u_low, v)) / (u_high - u_low)
        } else {
            Vec3::zeros()
        };
        let dv = if v_high > v_low {
            (self.evaluate(u, v_high) - self.evaluate(u, v_low)) / (v_high - v_low)
        } else {
            Vec3::zeros()
        };

        (point, du, dv)
    }
}

/// 3次元平面曲面
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct PlaneSurface3 {
    pub origin: Point3,
    pub u_axis: Vec3,
    pub v_axis: Vec3,
    pub normal: Vec3,
}

impl PlaneSurface3 {
    pub fn new(origin: Point3, u_axis: Vec3, v_axis: Vec3) -> Option<Self> {
        let u_axis = u_axis.try_normalize_safe(1e-12)?;
        let v_axis = v_axis.try_normalize_safe(1e-12)?;
        let normal = u_axis.cross(&v_axis).try_normalize_safe(1e-12)?;
        Some(Self {
            origin,
            u_axis,
            v_axis,
            normal,
        })
    }
}

impl Surface3 for PlaneSurface3 {
    fn param_range(&self) -> ((f64, f64), (f64, f64)) {
        (
            (-f64::INFINITY, f64::INFINITY),
            (-f64::INFINITY, f64::INFINITY),
        )
    }

    fn evaluate(&self, u: f64, v: f64) -> Point3 {
        self.origin + self.u_axis * u + self.v_axis * v
    }

    fn normal(&self, _u: f64, _v: f64) -> Option<Vec3> {
        Some(self.normal)
    }

    fn evaluate_with_derivatives(&self, u: f64, v: f64) -> (Point3, Vec3, Vec3) {
        (self.evaluate(u, v), self.u_axis, self.v_axis)
    }
}

impl Surface3 for NurbsSurface3 {
    fn integration_breaks(&self) -> (Vec<f64>, Vec<f64>) {
        fn interior(knots: &[f64], start: f64, end: f64) -> Vec<f64> {
            let mut values: Vec<f64> = Vec::new();
            for knot in knots {
                if *knot <= start || *knot >= end {
                    continue;
                }
                // 重複ノットは1本の線である。二重ノットをそのまま並べると
                // 幅ゼロの帯を作ってしまう。
                if values
                    .last()
                    .map(|last| (*last - *knot).abs() <= 1e-12)
                    .unwrap_or(false)
                {
                    continue;
                }
                values.push(*knot);
            }
            values
        }

        let ((u_min, u_max), (v_min, v_max)) = self.param_range();
        (
            interior(&self.knots_u.knots, u_min, u_max),
            interior(&self.knots_v.knots, v_min, v_max),
        )
    }

    fn param_range(&self) -> ((f64, f64), (f64, f64)) {
        self.param_range()
    }

    fn evaluate(&self, u: f64, v: f64) -> Point3 {
        self.evaluate(u, v)
    }

    fn normal(&self, u: f64, v: f64) -> Option<Vec3> {
        self.normal(u, v)
    }

    fn evaluate_with_derivatives(&self, u: f64, v: f64) -> (Point3, Vec3, Vec3) {
        self.evaluate_derivatives_1st(u, v)
    }
}

impl Surface3 for CoonsPatch3 {
    fn param_range(&self) -> ((f64, f64), (f64, f64)) {
        ((0.0, 1.0), (0.0, 1.0))
    }

    fn evaluate(&self, u: f64, v: f64) -> Point3 {
        self.evaluate(u, v)
    }

    fn normal(&self, u: f64, v: f64) -> Option<Vec3> {
        self.normal(u, v)
    }
}
