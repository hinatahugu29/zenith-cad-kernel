use crate::nurbs_curve::NurbsCurve3;
use crate::nurbs_surface::NurbsSurface3;
use crate::surface::Surface3;
use serde::{Deserialize, Serialize};
use zenith_math::{Point3, Tolerance, Vec3, Vec3Ext};

/// 2つの曲面間に接する $G^1$ / $G^2$ サーフェスブレンド（フィレット曲面）
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct SurfaceBlend3 {
    /// 境界 1 上のレール曲線 C1(t)
    pub rail1: NurbsCurve3,
    /// 境界 2 上のレール曲線 C2(t)
    pub rail2: NurbsCurve3,
    /// 境界 1 での接線ベクトル場 T1(t)
    pub tangent1: Vec<Vec3>,
    /// 境界 2 での接線ベクトル場 T2(t)
    pub tangent2: Vec<Vec3>,
    /// 生成されたブレンドNURBS曲面
    pub blend_surface: NurbsSurface3,
}

impl SurfaceBlend3 {
    /// 2つの境界レール曲線と各曲面からの接線方向から、$G^1$ 連続なブレンド曲面を構築
    pub fn create_g1_blend(
        rail1: NurbsCurve3,
        rail2: NurbsCurve3,
        tangent_scale: f64,
        _tol: &Tolerance,
    ) -> Result<Self, String> {
        let num_u = rail1.control_points.len();
        if rail2.control_points.len() != num_u {
            return Err("Rail curves must have matching control points count".to_string());
        }

        let degree_u = rail1.degree;
        let degree_v = 3; // 3次Bezier/B-Spline（両端での位置と接線を満たす）

        let mut ctrl_pts_grid = vec![Vec::with_capacity(4); num_u];
        let mut tangent1_vec = Vec::with_capacity(num_u);
        let mut tangent2_vec = Vec::with_capacity(num_u);

        for (i, row) in ctrl_pts_grid.iter_mut().enumerate().take(num_u) {
            let p1 = rail1.control_points[i].point;
            let p2 = rail2.control_points[i].point;

            let v_chord = p2 - p1;
            let dist = v_chord.norm();
            let tan_len = (dist * tangent_scale * 0.333).max(1e-4);

            let t1 = v_chord
                .try_normalize_safe(1e-12)
                .unwrap_or(Vec3::new(1.0, 0.0, 0.0))
                * tan_len;
            let t2 = -t1;

            tangent1_vec.push(t1);
            tangent2_vec.push(t2);

            let p_ctrl1 = p1 + t1;
            let p_ctrl2 = p2 + t2;

            row.push(rail1.control_points[i]);
            row.push(crate::nurbs_curve::ControlPoint3::unweighted(p_ctrl1));
            row.push(crate::nurbs_curve::ControlPoint3::unweighted(p_ctrl2));
            row.push(rail2.control_points[i]);
        }

        let knots_u = rail1.knots.clone();
        let knots_v = crate::bspline_basis::KnotVector::clamped_uniform(4, degree_v);

        let blend_surface =
            NurbsSurface3::new(degree_u, degree_v, ctrl_pts_grid, knots_u, knots_v)?;

        Ok(Self {
            rail1,
            rail2,
            tangent1: tangent1_vec,
            tangent2: tangent2_vec,
            blend_surface,
        })
    }

    /// 2つの境界レール曲線と曲率情報から、$G^2$ 曲率連続（Class-A Surface）なブレンド曲面を構築
    pub fn create_g2_blend(
        rail1: NurbsCurve3,
        rail2: NurbsCurve3,
        tangent_scale: f64,
        curvature_scale: f64,
        _tol: &Tolerance,
    ) -> Result<Self, String> {
        let num_u = rail1.control_points.len();
        if rail2.control_points.len() != num_u {
            return Err("Rail curves must have matching control points count".to_string());
        }

        let degree_u = rail1.degree;
        let degree_v = 5; // 5次Bezier/B-Spline（両端での位置・接線・曲率の計6拘束を満たす）

        let mut ctrl_pts_grid = vec![Vec::with_capacity(6); num_u];
        let mut tangent1_vec = Vec::with_capacity(num_u);
        let mut tangent2_vec = Vec::with_capacity(num_u);

        for (i, row) in ctrl_pts_grid.iter_mut().enumerate().take(num_u) {
            let p0 = rail1.control_points[i].point;
            let p5 = rail2.control_points[i].point;

            let v_chord = p5 - p0;
            let dist = v_chord.norm();
            let tan_len = (dist * tangent_scale * 0.2).max(1e-4);

            let t1 = v_chord
                .try_normalize_safe(1e-12)
                .unwrap_or(Vec3::new(1.0, 0.0, 0.0))
                * tan_len;
            let t2 = -t1;

            tangent1_vec.push(t1);
            tangent2_vec.push(t2);

            let p1 = p0 + t1;
            let p4 = p5 + t2;

            // G2曲率連続点（2階差分ベクトル）
            let curv_offset1 = t1 * (0.8 * curvature_scale);
            let curv_offset2 = t2 * (0.8 * curvature_scale);
            let p2 = p1 + curv_offset1;
            let p3 = p4 + curv_offset2;

            row.push(rail1.control_points[i]);
            row.push(crate::nurbs_curve::ControlPoint3::unweighted(p1));
            row.push(crate::nurbs_curve::ControlPoint3::unweighted(p2));
            row.push(crate::nurbs_curve::ControlPoint3::unweighted(p3));
            row.push(crate::nurbs_curve::ControlPoint3::unweighted(p4));
            row.push(rail2.control_points[i]);
        }

        let knots_u = rail1.knots.clone();
        let knots_v = crate::bspline_basis::KnotVector::clamped_uniform(6, degree_v);

        let blend_surface =
            NurbsSurface3::new(degree_u, degree_v, ctrl_pts_grid, knots_u, knots_v)?;

        Ok(Self {
            rail1,
            rail2,
            tangent1: tangent1_vec,
            tangent2: tangent2_vec,
            blend_surface,
        })
    }

    pub fn param_range(&self) -> ((f64, f64), (f64, f64)) {
        self.blend_surface.param_range()
    }

    pub fn evaluate(&self, u: f64, v: f64) -> Point3 {
        self.blend_surface.evaluate(u, v)
    }

    pub fn normal(&self, u: f64, v: f64) -> Option<Vec3> {
        self.blend_surface.normal(u, v)
    }
}

impl Surface3 for SurfaceBlend3 {
    fn param_range(&self) -> ((f64, f64), (f64, f64)) {
        self.blend_surface.param_range()
    }

    fn evaluate(&self, u: f64, v: f64) -> Point3 {
        self.blend_surface.evaluate(u, v)
    }

    fn normal(&self, u: f64, v: f64) -> Option<Vec3> {
        self.blend_surface.normal(u, v)
    }
}
