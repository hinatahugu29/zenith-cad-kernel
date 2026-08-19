use zenith_geom::{ControlPoint3, KnotVector, NurbsCurve3};
use zenith_math::{Point3, Tolerance, Vec3, Vec3Ext};
use zenith_topo::{Solid, Wire};

/// 螺旋（ヘリカル・スパイラル）モデリングアルゴリズム
pub struct HelixBuilder;

impl HelixBuilder {
    /// 3次元NURBS螺旋（ヘリックス）パス曲線の生成
    /// `radius`: 半径, `pitch`: 1回転あたりの進み量, `turns`: 巻き数（> 0.0）, `axis_origin`: 軸原点, `axis_dir`: 軸方向
    pub fn build_helix_curve(
        radius: f64,
        pitch: f64,
        turns: f64,
        axis_origin: Point3,
        axis_dir: Vec3,
        _tol: &Tolerance,
    ) -> Result<NurbsCurve3, String> {
        if radius <= 1e-9 {
            return Err("Helix radius must be positive".to_string());
        }
        if turns <= 1e-6 {
            return Err("Helix turns must be positive".to_string());
        }
        let axis_dir_norm = axis_dir
            .try_normalize_safe(1e-12)
            .ok_or("Axis direction is zero")?;

        let arb = if axis_dir_norm.x.abs() < 0.9 {
            Vec3::new(1.0, 0.0, 0.0)
        } else {
            Vec3::new(0.0, 1.0, 0.0)
        };
        let x_axis = axis_dir_norm.cross(&arb).normalize();
        let y_axis = axis_dir_norm.cross(&x_axis).normalize();

        // 90度（PI/2）ごとにセグメント分割
        let total_angle = turns * std::f64::consts::TAU;
        let num_segments = (turns * 4.0).ceil() as usize;
        let d_theta = total_angle / num_segments as f64;
        let dz = pitch * (d_theta / std::f64::consts::TAU);
        let wm = (d_theta / 2.0).cos();

        let num_cps = 2 * num_segments + 1;
        let mut control_points = Vec::with_capacity(num_cps);

        for seg in 0..num_segments {
            let theta_start = seg as f64 * d_theta;
            let theta_mid = theta_start + d_theta / 2.0;
            let theta_end = theta_start + d_theta;

            let z_start = seg as f64 * dz;
            let z_mid = z_start + dz / 2.0;
            let z_end = (seg + 1) as f64 * dz;

            let p0 = axis_origin
                + (x_axis * theta_start.cos() + y_axis * theta_start.sin()) * radius
                + axis_dir_norm * z_start;
            let p_mid = axis_origin
                + (x_axis * theta_mid.cos() + y_axis * theta_mid.sin()) * (radius / wm)
                + axis_dir_norm * z_mid;
            let p1 = axis_origin
                + (x_axis * theta_end.cos() + y_axis * theta_end.sin()) * radius
                + axis_dir_norm * z_end;

            if seg == 0 {
                control_points.push(ControlPoint3::unweighted(p0));
            }
            control_points.push(ControlPoint3::new(p_mid, wm));
            control_points.push(ControlPoint3::unweighted(p1));
        }

        let mut knots = Vec::with_capacity(num_cps + 3);
        knots.push(0.0);
        knots.push(0.0);
        knots.push(0.0);
        for seg in 1..num_segments {
            let u = seg as f64 / num_segments as f64;
            knots.push(u);
            knots.push(u);
        }
        knots.push(1.0);
        knots.push(1.0);
        knots.push(1.0);

        let knot_vec = KnotVector::new(knots);
        NurbsCurve3::new(2, control_points, knot_vec)

    }

    /// 閉断面ワイヤを螺旋パスに沿ってスイープした完全閉B-Repソリッド（スプリング・ネジ等）を生成
    pub fn sweep_wire_along_helix(
        profile_wire: &Wire,
        radius: f64,
        pitch: f64,
        turns: f64,
        axis_origin: Point3,
        axis_dir: Vec3,
        num_sections: usize,
        tol: &Tolerance,
    ) -> Result<Solid, String> {
        let helix_path =
            Self::build_helix_curve(radius, pitch, turns, axis_origin, axis_dir, tol)?;
        let sections = num_sections.max((turns * 16.0).ceil() as usize);
        crate::SweepBuilder::sweep_wire_along_curve(profile_wire, &helix_path, sections, tol)
    }
}
