use crate::nurbs_curve::NurbsCurve3;
use crate::surface::Surface3;
use serde::{Deserialize, Serialize};
use zenith_math::{Point3, Point3Ext, Tolerance, Vec3, Vec3Ext};

/// 3本の境界曲線から定義される三角形パッチ（Triangular Coons Patch）
/// 重心座標系 (u, v, w) (u + v + w = 1, u, v, w >= 0) による補間
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct TriangularPatch3 {
    /// 境界カーブ 0: (u=0 の対辺, v: 0 -> 1, w: 1 -> 0)
    pub c0: NurbsCurve3,
    /// 境界カーブ 1: (v=0 の対辺, w: 0 -> 1, u: 1 -> 0)
    pub c1: NurbsCurve3,
    /// 境界カーブ 2: (w=0 の対辺, u: 0 -> 1, v: 1 -> 0)
    pub c2: NurbsCurve3,
    /// 3隅のコーナー頂点
    pub p0: Point3, // c1(1) == c2(0)
    pub p1: Point3, // c2(1) == c0(0)
    pub p2: Point3, // c0(1) == c1(0)
}

impl TriangularPatch3 {
    /// 3本の境界曲線から三角形パッチを作成（3頂点のトポロジー連続性を自動検証）
    pub fn new(
        c0: NurbsCurve3,
        c1: NurbsCurve3,
        c2: NurbsCurve3,
        tol: &Tolerance,
    ) -> Result<Self, String> {
        let (t0_min, t0_max) = c0.param_range();
        let (t1_min, t1_max) = c1.param_range();
        let (t2_min, t2_max) = c2.param_range();

        let pt_c0_0 = c0.evaluate(t0_min);
        let pt_c0_1 = c0.evaluate(t0_max);
        let pt_c1_0 = c1.evaluate(t1_min);
        let pt_c1_1 = c1.evaluate(t1_max);
        let pt_c2_0 = c2.evaluate(t2_min);
        let pt_c2_1 = c2.evaluate(t2_max);

        // コーナー連続性の確認: c2(1) == c0(0), c0(1) == c1(0), c1(1) == c2(0)
        if !pt_c2_1.is_coincident_with(&pt_c0_0, tol.linear) {
            return Err(format!(
                "Corner P1 mismatch: c2(1)={:?} != c0(0)={:?} (dist={})",
                pt_c2_1,
                pt_c0_0,
                pt_c2_1.distance_to(&pt_c0_0)
            ));
        }
        if !pt_c0_1.is_coincident_with(&pt_c1_0, tol.linear) {
            return Err(format!(
                "Corner P2 mismatch: c0(1)={:?} != c1(0)={:?} (dist={})",
                pt_c0_1,
                pt_c1_0,
                pt_c0_1.distance_to(&pt_c1_0)
            ));
        }
        if !pt_c1_1.is_coincident_with(&pt_c2_0, tol.linear) {
            return Err(format!(
                "Corner P0 mismatch: c1(1)={:?} != c2(0)={:?} (dist={})",
                pt_c1_1,
                pt_c2_0,
                pt_c1_1.distance_to(&pt_c2_0)
            ));
        }

        let p1 = Point3::from((pt_c2_1.coords + pt_c0_0.coords) * 0.5);
        let p2 = Point3::from((pt_c0_1.coords + pt_c1_0.coords) * 0.5);
        let p0 = Point3::from((pt_c1_1.coords + pt_c2_0.coords) * 0.5);

        Ok(Self {
            c0,
            c1,
            c2,
            p0,
            p1,
            p2,
        })
    }

    /// 重心座標 (u, v, w) (u+v+w=1) による3次元座標の評価（Gregory-Charrot 三角形Coonsパッチ）
    pub fn evaluate_barycentric(&self, u: f64, v: f64, w: f64) -> Point3 {
        // 各頂点特異点の処理
        if u >= 1.0 - 1e-10 {
            return self.p0;
        }
        if v >= 1.0 - 1e-10 {
            return self.p1;
        }
        if w >= 1.0 - 1e-10 {
            return self.p2;
        }

        // Side-vertex 補間
        // カーブ0上の点 (u=0 の対辺): パラメータ t = v / (v + w)
        let t0 = v / (v + w);
        let (t0_min, t0_max) = self.c0.param_range();
        let pt0 = self.c0.evaluate(t0_min + t0 * (t0_max - t0_min));

        // カーブ1上の点 (v=0 の対辺): パラメータ t = w / (w + u)
        let t1 = w / (w + u);
        let (t1_min, t1_max) = self.c1.param_range();
        let pt1 = self.c1.evaluate(t1_min + t1 * (t1_max - t1_min));

        // カーブ2上の点 (w=0 の対辺): パラメータ t = u / (u + v)
        let t2 = u / (u + v);
        let (t2_min, t2_max) = self.c2.param_range();
        let pt2 = self.c2.evaluate(t2_min + t2 * (t2_max - t2_min));

        // ブレンド重み
        let blend_0 = (v * w) / (u * v + v * w + w * u);
        let blend_1 = (w * u) / (u * v + v * w + w * u);
        let blend_2 = (u * v) / (u * v + v * w + w * u);

        let pt = pt0.coords * blend_0 + pt1.coords * blend_1 + pt2.coords * blend_2;
        Point3::from(pt)
    }

    /// UV座標 (u in [0, 1], v in [0, 1 - u]) での評価
    pub fn evaluate(&self, u: f64, v: f64) -> Point3 {
        let u = u.clamp(0.0, 1.0);
        let v = v.clamp(0.0, 1.0 - u);
        let w = (1.0 - u - v).max(0.0);
        self.evaluate_barycentric(u, v, w)
    }

    /// 数値微分による法線ベクトルの計算
    pub fn normal(&self, u: f64, v: f64) -> Option<Vec3> {
        let eps = 1e-5;
        let u_p = (u + eps).min(0.999);
        let u_m = (u - eps).max(0.001);
        let du = (self.evaluate(u_p, v) - self.evaluate(u_m, v)) / (u_p - u_m);

        let v_p = (v + eps).min(1.0 - u - 0.001);
        let v_m = (v - eps).max(0.001);
        let dv = (self.evaluate(u, v_p) - self.evaluate(u, v_m)) / (v_p - v_m);

        du.cross(&dv).try_normalize_safe(1e-12)
    }
}

impl Surface3 for TriangularPatch3 {
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
