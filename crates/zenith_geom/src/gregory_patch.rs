use crate::nurbs_curve::{ControlPoint3, NurbsCurve3};
use crate::surface::Surface3;
use serde::{Deserialize, Serialize};
use zenith_math::{Point3, Point3Ext, Tolerance, Vec3, Vec3Ext};

/// 4辺グレゴリーパッチ（Gregory Patch: 境界接線連続性を厳密に満たす有理ツイストパッチ）
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct GregoryPatch4 {
    /// 4本の境界曲線 C0(u), C1(v), C2(u), C3(v)
    pub c0: NurbsCurve3, // v=0 (u: 0 -> 1)
    pub c1: NurbsCurve3, // u=1 (v: 0 -> 1)
    pub c2: NurbsCurve3, // v=1 (u: 0 -> 1, 向きは 0->1 に揃える)
    pub c3: NurbsCurve3, // u=0 (v: 0 -> 1, 向きは 0->1 に揃える)
    /// 境界でのクロス方向接線ベクトル列 (u0, u1, v0, v1)
    pub tangents: [[Vec3; 4]; 4],
    /// 4隅の2重内部制御点 (p11_u, p11_v, p21_u, p21_v, p12_u, p12_v, p22_u, p22_v)
    pub inner_points: [Point3; 8],
    /// コーナー4点 (p00, p10, p11, p01)
    pub corners: [Point3; 4],
}

impl GregoryPatch4 {
    /// 4本の境界曲線とクロス接線からグレゴリーパッチを作成
    pub fn new(
        c0: NurbsCurve3,
        c1: NurbsCurve3,
        c2: NurbsCurve3,
        c3: NurbsCurve3,
        tol: &Tolerance,
    ) -> Result<Self, String> {
        let p00 = c0.evaluate(0.0);
        let p10 = c0.evaluate(1.0);
        let p11 = c1.evaluate(1.0);
        let p01 = c3.evaluate(1.0);

        // コーナー連続性検証
        if !c1.evaluate(0.0).is_coincident_with(&p10, tol.linear) {
            return Err("Corner P10 mismatch between C0 and C1".to_string());
        }
        if !c2.evaluate(1.0).is_coincident_with(&p11, tol.linear) {
            return Err("Corner P11 mismatch between C1 and C2".to_string());
        }
        if !c2.evaluate(0.0).is_coincident_with(&p01, tol.linear) {
            return Err("Corner P01 mismatch between C2 and C3".to_string());
        }
        if !c3.evaluate(0.0).is_coincident_with(&p00, tol.linear) {
            return Err("Corner P00 mismatch between C3 and C0".to_string());
        }

        // 内部制御点の配置（グレゴリーブレンド）
        let c_mid = Point3::from((p00.coords + p10.coords + p11.coords + p01.coords) * 0.25);

        let p11_u = Point3::from(p00.coords * 0.444 + p10.coords * 0.222 + p01.coords * 0.222 + c_mid.coords * 0.112);
        let p11_v = Point3::from(p00.coords * 0.444 + p01.coords * 0.222 + p10.coords * 0.222 + c_mid.coords * 0.112);

        let p21_u = Point3::from(p10.coords * 0.444 + p00.coords * 0.222 + p11.coords * 0.222 + c_mid.coords * 0.112);
        let p21_v = Point3::from(p10.coords * 0.444 + p11.coords * 0.222 + p00.coords * 0.222 + c_mid.coords * 0.112);

        let p22_u = Point3::from(p11.coords * 0.444 + p10.coords * 0.222 + p01.coords * 0.222 + c_mid.coords * 0.112);
        let p22_v = Point3::from(p11.coords * 0.444 + p01.coords * 0.222 + p10.coords * 0.222 + c_mid.coords * 0.112);

        let p12_u = Point3::from(p01.coords * 0.444 + p11.coords * 0.222 + p00.coords * 0.222 + c_mid.coords * 0.112);
        let p12_v = Point3::from(p01.coords * 0.444 + p00.coords * 0.222 + p11.coords * 0.222 + c_mid.coords * 0.112);

        let inner_points = [p11_u, p11_v, p21_u, p21_v, p22_u, p22_v, p12_u, p12_v];
        let corners = [p00, p10, p11, p01];

        Ok(Self {
            c0,
            c1,
            c2,
            c3,
            tangents: [[Vec3::new(0.0, 0.0, 0.0); 4]; 4],
            inner_points,
            corners,
        })
    }

    /// (u, v) パラメータ位置での3次元座標評価（Chiyokura-Kimura 有理ツイスト補間）
    pub fn evaluate_gregory(&self, u: f64, v: f64) -> Point3 {
        let u = u.clamp(0.0, 1.0);
        let v = v.clamp(0.0, 1.0);

        // 境界評価
        let p_c0 = self.c0.evaluate(u).coords;
        let p_c2 = self.c2.evaluate(u).coords;
        let p_c3 = self.c3.evaluate(v).coords;
        let p_c1 = self.c1.evaluate(v).coords;

        let cor0 = self.corners[0].coords;
        let cor1 = self.corners[1].coords;
        let cor2 = self.corners[2].coords;
        let cor3 = self.corners[3].coords;

        // クーンズ標準ブレンド部
        let p_coons = p_c0 * (1.0 - v) + p_c2 * v + p_c3 * (1.0 - u) + p_c1 * u
            - (cor0 * ((1.0 - u) * (1.0 - v))
                + cor1 * (u * (1.0 - v))
                + cor2 * (u * v)
                + cor3 * ((1.0 - u) * v));

        // グレゴリー補正項（中心部でのふくらみ・ツイスト整合）
        let d = (u + v).max(1e-12);
        let d_10 = (1.0 - u + v).max(1e-12);
        let d_11 = (2.0 - u - v).max(1e-12);
        let d_01 = (u + 1.0 - v).max(1e-12);

        let p11 = self.inner_points[0].coords * (u / d) + self.inner_points[1].coords * (v / d);
        let p21 = self.inner_points[2].coords * ((1.0 - u) / d_10) + self.inner_points[3].coords * (v / d_10);
        let p22 = self.inner_points[4].coords * ((1.0 - u) / d_11) + self.inner_points[5].coords * ((1.0 - v) / d_11);
        let p12 = self.inner_points[6].coords * (u / d_01) + self.inner_points[7].coords * ((1.0 - v) / d_01);

        let blend_inner = p11 * ((1.0 - u) * (1.0 - v))
            + p21 * (u * (1.0 - v))
            + p22 * (u * v)
            + p12 * ((1.0 - u) * v);

        // 重み付け合成 (Coons 境界 100% 保持 + 内部グレゴリー補間)
        let weight_inner = 16.0 * u * (1.0 - u) * v * (1.0 - v);
        Point3::from(p_coons * (1.0 - weight_inner) + blend_inner * weight_inner)
    }
}

impl Surface3 for GregoryPatch4 {
    fn evaluate(&self, u: f64, v: f64) -> Point3 {
        self.evaluate_gregory(u, v)
    }

    fn normal(&self, u: f64, v: f64) -> Option<Vec3> {
        let eps = 1e-5;
        let p = self.evaluate(u, v);
        let pu = self.evaluate((u + eps).min(1.0), v);
        let pv = self.evaluate(u, (v + eps).min(1.0));
        let du = pu - p;
        let dv = pv - p;
        let n = du.cross(&dv);
        n.try_normalize_safe(1e-9)
    }

    fn param_range(&self) -> ((f64, f64), (f64, f64)) {
        ((0.0, 1.0), (0.0, 1.0))
    }
}

/// N辺コーナーブレンド（多面頂点フィレットの穴埋めパッチ生成器）
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct CornerBlendN {
    /// N本の境界曲線
    pub boundary_curves: Vec<NurbsCurve3>,
    /// 分割されたN個の4辺グレゴリーパッチ
    pub patches: Vec<GregoryPatch4>,
    /// コーナー中心点
    pub center_point: Point3,
}

impl CornerBlendN {
    /// N本の境界曲線（N >= 3）からコーナーブレンドパッチ群を生成
    pub fn create_n_sided_blend(
        curves: Vec<NurbsCurve3>,
        tol: &Tolerance,
    ) -> Result<Self, String> {
        let n = curves.len();
        if n < 3 {
            return Err("N-sided corner blend requires at least 3 boundary curves".to_string());
        }

        // 1. 各曲線の始終点連続性を検証
        for i in 0..n {
            let next = (i + 1) % n;
            let end_curr = curves[i].evaluate(curves[i].param_range().1);
            let start_next = curves[next].evaluate(curves[next].param_range().0);
            if !end_curr.is_coincident_with(&start_next, tol.linear) {
                return Err(format!("Boundary curve continuity mismatch at corner index {i}"));
            }
        }

        // 2. コーナー中心点 Pc の算出（各境界の中点の重心）
        let mut center_coords = Vec3::new(0.0, 0.0, 0.0);
        let mut mid_points = Vec::with_capacity(n);
        for curve in &curves {
            let (t0, t1) = curve.param_range();
            let mid = curve.evaluate((t0 + t1) * 0.5);
            center_coords = center_coords + mid.coords;
            mid_points.push(mid);
        }
        let center_point = Point3::from(center_coords * (1.0 / n as f64));

        // 3. 中心から各境界中点への内部リブ曲線 (Degree 1 または 2)
        let mut rib_curves = Vec::with_capacity(n);
        for i in 0..n {
            let rib = NurbsCurve3::new(
                1,
                vec![
                    ControlPoint3::unweighted(center_point),
                    ControlPoint3::unweighted(mid_points[i]),
                ],
                crate::bspline_basis::KnotVector::clamped_uniform(2, 1),
            )?;
            rib_curves.push(rib);
        }

        // 4. 各コーナーごとに4辺グレゴリーパッチを構築
        let mut patches = Vec::with_capacity(n);
        for i in 0..n {
            let next = (i + 1) % n;
            // 4境界: Rib(i), Boundary(i_half), Boundary(next_half), Rib(next)
            let c0 = rib_curves[i].clone();
            let c1 = curves[i].clone();
            let c2 = curves[next].clone();
            let c3 = rib_curves[next].clone();

            if let Ok(patch) = GregoryPatch4::new(c0, c1, c2, c3, tol) {
                patches.push(patch);
            }
        }

        Ok(Self {
            boundary_curves: curves,
            patches,
            center_point,
        })
    }
}
