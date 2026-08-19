use zenith_geom::{ControlPoint3, KnotVector, NurbsCurve3, NurbsSurface3};
use zenith_math::{Point3, Tolerance, Vec3, Vec3Ext};

/// 回転体（Revolve）モデリングアルゴリズム
pub struct RevolveBuilder;

impl RevolveBuilder {
    /// 3次元NURBS曲線を軸まわりに回転させて有理NURBS回転曲面を生成
    /// `axis_origin`: 回転軸上の点, `axis_dir`: 回転軸ベクトル, `angle_rad`: 回転角度 (0 < angle <= 2*PI)
    pub fn revolve_curve(
        curve: &NurbsCurve3,
        axis_origin: Point3,
        axis_dir: Vec3,
        angle_rad: f64,
        _tol: &Tolerance,
    ) -> Result<NurbsSurface3, String> {
        let axis_dir = axis_dir
            .try_normalize_safe(1e-12)
            .ok_or("Axis direction is zero")?;
        let num_u = curve.control_points.len();
        let degree_u = curve.degree;

        // 4セグメント（90度ごと）の有理B-Spline円弧回転（The NURBS Book Algorithm A8.1）
        // 簡易実装として4分割の等価NURBS回転グリッドを構築
        let num_segments = 4;
        let d_theta = angle_rad / num_segments as f64;
        let num_v = 2 * num_segments + 1;
        let degree_v = 2;

        let wm = (d_theta / 2.0).cos(); // 重み係数

        let mut ctrl_pts_grid = vec![Vec::with_capacity(num_v); num_u];

        for (i, cp) in curve.control_points.iter().enumerate() {
            let p = cp.point;
            // 軸への直交射影点
            let v_p = p - axis_origin;
            let proj_len = v_p.dot(&axis_dir);
            let p_center = axis_origin + axis_dir * proj_len;
            let v_radial = p - p_center;
            let radius = v_radial.norm();

            if radius < 1e-12 {
                // 軸上の特異点。位置は回転で動かないが、重みは他の行と同じ
                // 円弧パターン (1, cos(dtheta/2), 1, ...) を保たなければ
                // テンソル積の分母が分離できなくなり、曲面全体が歪む。
                for column in 0..num_v {
                    let arc_weight = if column % 2 == 1 { wm } else { 1.0 };
                    ctrl_pts_grid[i].push(ControlPoint3::new(p, cp.weight * arc_weight));
                }
                continue;
            }

            let x_axis = v_radial / radius;
            let y_axis = axis_dir.cross(&x_axis);

            let mut theta: f64 = 0.0;
            for seg in 0..num_segments {
                let p_start = p_center + (x_axis * theta.cos() + y_axis * theta.sin()) * radius;
                let theta_mid = theta + d_theta / 2.0;
                let p_mid = p_center
                    + (x_axis * theta_mid.cos() + y_axis * theta_mid.sin()) * (radius / wm);
                let theta_end = theta + d_theta;
                let p_end =
                    p_center + (x_axis * theta_end.cos() + y_axis * theta_end.sin()) * radius;

                if seg == 0 {
                    ctrl_pts_grid[i].push(ControlPoint3::new(p_start, cp.weight));
                }
                ctrl_pts_grid[i].push(ControlPoint3::new(p_mid, cp.weight * wm));
                ctrl_pts_grid[i].push(ControlPoint3::new(p_end, cp.weight));

                theta = theta_end;
            }
        }

        // V方向の結び目ベクトル（4セグメント円弧用: [0,0,0, 0.25,0.25, 0.5,0.5, 0.75,0.75, 1,1,1]）
        let mut knots_v = vec![0.0, 0.0, 0.0];
        for s in 1..num_segments {
            let val = s as f64 / num_segments as f64;
            knots_v.push(val);
            knots_v.push(val);
        }
        knots_v.extend_from_slice(&[1.0, 1.0, 1.0]);

        let knots_u = curve.knots.clone();
        let knot_vec_v = KnotVector::new(knots_v);

        NurbsSurface3::new(degree_u, degree_v, ctrl_pts_grid, knots_u, knot_vec_v)
    }
}
