use zenith_geom::{ControlPoint3, KnotVector, NurbsCurve3};
use zenith_math::{Point3, Tolerance, Vec3, Vec3Ext};
use zenith_topo::{Solid, Wire};

/// 螺旋（ヘリカル・スパイラル）モデリングアルゴリズム
pub struct HelixBuilder;

impl HelixBuilder {
    /// 3次元NURBS螺旋（ヘリックス）パス曲線の生成
    /// `radius`: 半径, `pitch`: 1回転あたりの進み量, `turns`: 巻き数（> 0.0）, `axis_origin`: 軸原点, `axis_dir`: 軸方向
    /// 公差から刻みを決めて螺旋を組む。
    ///
    /// 螺旋は有理曲線では**厳密に表せない**。xy は真円になるが、真の螺旋は
    /// z が角度に比例するのに対し、有理2次の角度 θ(t) は t に比例しない。
    /// 両者は各区間の t = 0, 1/2, 1 で一致し、その間でずれる。90度刻みだと
    /// 半径10・ピッチ6の螺旋で高さが **3.16e-2**（ピッチの 0.53%）外れる。
    ///
    /// 刻みを細かくすれば減る。減り方は [`segments_per_turn_for`] に測った
    /// 値ごと書いてある。
    pub fn build_helix_curve(
        radius: f64,
        pitch: f64,
        turns: f64,
        axis_origin: Point3,
        axis_dir: Vec3,
        tol: &Tolerance,
    ) -> Result<NurbsCurve3, String> {
        let per_turn = Self::segments_per_turn_for(pitch, tol);
        Self::build_helix_curve_with_segments(radius, pitch, turns, axis_origin, axis_dir, per_turn)
    }

    /// 1周をいくつに刻むと、高さのずれが線形公差に収まるか。
    ///
    /// ずれは刻み角の**3乗**で減る。実測（半径10・ピッチ6・2周、1周を
    /// 4, 8, 16, 32, 64, 128 に刻んだとき）:
    ///
    /// ```text
    /// 3.1632e-2  3.7679e-3  4.6552e-4  5.8021e-5  7.2474e-6  9.0576e-7
    ///        8.40       8.09       8.02       8.01       8.00
    /// ```
    ///
    /// 係数は**ピッチに比例し、半径には依らない**（ずれは z 方向にしか
    /// 出ない。半径10と30で同じ値になる）。4刻みで `pitch * 5.272e-3` なので、
    /// そこから解く。
    pub fn segments_per_turn_for(pitch: f64, tol: &Tolerance) -> usize {
        let allowed = tol.linear.max(1e-12);
        let at_four = pitch.abs() * 5.272e-3;
        if at_four <= allowed {
            return 4;
        }
        // at_four * (4 / n)^3 <= allowed
        let needed = 4.0 * (at_four / allowed).cbrt();
        (needed.ceil() as usize).clamp(4, 4096)
    }

    /// 1周あたりの刻み数を指定して螺旋を組む。刻みと精度の関係を測るために
    /// 開けてある。
    pub fn build_helix_curve_with_segments(
        radius: f64,
        pitch: f64,
        turns: f64,
        axis_origin: Point3,
        axis_dir: Vec3,
        segments_per_turn: usize,
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

        let total_angle = turns * std::f64::consts::TAU;
        let num_segments = (turns * segments_per_turn.max(4) as f64).ceil() as usize;
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
        // ステーション数は掃引の精度をそのまま決める。断面の重心が経路の上に
        // あり経路に垂直なら `V = A x L` がきっかり成り立つので、そこからの
        // ずれで測れる。半径10・ピッチ6・2周、2x2 の角断面での実測:
        //
        // ```text
        // stations   32        64        128       256       512      1024
        // rel error  2.773e-5  2.924e-6  2.200e-7  1.491e-8  9.98e-10 3.17e-11
        // ratio                    9.5      13.3      14.8      15.0     31.5
        // ```
        //
        // 刻みの**4乗**で縮む。既定の下限は1周あたり 64 で、上の例なら
        // 2.2e-7 に当たる。それより要るなら `num_sections` で上げる。
        let sections = num_sections.max((turns * 64.0).ceil() as usize);
        crate::SweepBuilder::sweep_wire_along_curve(profile_wire, &helix_path, sections, tol)
    }
}
