use std::f64::consts::PI;
use zenith_geom::{ControlPoint3, KnotVector, NurbsSurface3, PlaneSurface3};
use zenith_math::{Point3, Vec3};
use zenith_topo::{Edge, Face, FaceGeometry, OrientedEdge, Shell, Solid, Vertex, Wire};

/// インボリュート平歯車（Spur Gear）B-Rep ソリッドビルダー
pub struct GearBuilder;

impl GearBuilder {
    /// モジュール m, 歯数 z, 圧力角 deg, 厚み thickness からインボリュート平歯車ソリッドを構築
    pub fn make_spur_gear(
        module: f64,
        teeth: usize,
        pressure_angle_deg: f64,
        thickness: f64,
        bore_radius: f64,
    ) -> Result<Solid, String> {
        if module <= 0.0 {
            return Err("Module must be positive".to_string());
        }
        if teeth < 4 {
            return Err("Number of teeth must be at least 4".to_string());
        }
        if thickness <= 0.0 {
            return Err("Thickness must be positive".to_string());
        }

        let z = teeth as f64;
        let m = module;
        let alpha = pressure_angle_deg.to_radians();

        let r = m * z * 0.5; // ピッチ円半径
        let r_b = r * alpha.cos(); // 基礎円半径
        let r_a = r + m; // 歯先円半径
        let r_f = (r - 1.25 * m).max(r_b * 0.8).max(bore_radius + 0.5 * m); // 歯底円半径

        let pitch_angle = 2.0 * PI / z;
        let half_tooth_angle = pitch_angle * 0.25;

        // 1歯あたり4つのキーポイント（重複なし）
        let mut base_pts = Vec::with_capacity(teeth * 4);

        for i in 0..teeth {
            let center_angle = (i as f64) * pitch_angle;

            // 1. 歯底左 (root left)
            let a_root_l = center_angle - half_tooth_angle * 1.5;
            base_pts.push(Point3::new(r_f * a_root_l.cos(), r_f * a_root_l.sin(), 0.0));

            // 2. 歯先左 (tip left)
            let a_tip_l = center_angle - half_tooth_angle * 0.5;
            base_pts.push(Point3::new(r_a * a_tip_l.cos(), r_a * a_tip_l.sin(), 0.0));

            // 3. 歯先右 (tip right)
            let a_tip_r = center_angle + half_tooth_angle * 0.5;
            base_pts.push(Point3::new(r_a * a_tip_r.cos(), r_a * a_tip_r.sin(), 0.0));

            // 4. 歯底右 (root right)
            let a_root_r = center_angle + half_tooth_angle * 1.5;
            base_pts.push(Point3::new(r_f * a_root_r.cos(), r_f * a_root_r.sin(), 0.0));
        }

        let n = base_pts.len();
        let mut top_pts = Vec::with_capacity(n);
        for p in &base_pts {
            top_pts.push(Point3::new(p.x, p.y, thickness));
        }

        let mut v_bot = Vec::with_capacity(n);
        let mut v_top = Vec::with_capacity(n);
        for i in 0..n {
            v_bot.push(Vertex::from_point(base_pts[i]));
            v_top.push(Vertex::from_point(top_pts[i]));
        }

        let mut eb = Vec::with_capacity(n);
        let mut et = Vec::with_capacity(n);
        let mut ev = Vec::with_capacity(n);

        for i in 0..n {
            let next_i = (i + 1) % n;
            eb.push(Edge::line_between(v_bot[i].clone(), v_bot[next_i].clone())?);
            et.push(Edge::line_between(v_top[i].clone(), v_top[next_i].clone())?);
            ev.push(Edge::line_between(v_bot[i].clone(), v_top[i].clone())?);
        }

        let mut faces = Vec::new();

        // 側面 Face 群 (n面)
        for i in 0..n {
            let next_i = (i + 1) % n;
            let row0 = vec![
                ControlPoint3::unweighted(base_pts[i]),
                ControlPoint3::unweighted(top_pts[i]),
            ];
            let row1 = vec![
                ControlPoint3::unweighted(base_pts[next_i]),
                ControlPoint3::unweighted(top_pts[next_i]),
            ];
            let s = NurbsSurface3::new(
                1, 1,
                vec![row0, row1],
                KnotVector::clamped_uniform(2, 1),
                KnotVector::clamped_uniform(2, 1),
            )?;
            let wire = Wire::new(vec![
                OrientedEdge::forward(eb[i].clone()),
                OrientedEdge::forward(ev[next_i].clone()),
                OrientedEdge::reversed(et[i].clone()),
                OrientedEdge::reversed(ev[i].clone()),
            ]);
            faces.push(Face::simple(FaceGeometry::Nurbs(s), wire));
        }

        // 底面 Face (-Z 法線)
        let p_bot = PlaneSurface3::new(
            Point3::new(0.0, 0.0, 0.0),
            Vec3::new(0.0, 1.0, 0.0),
            Vec3::new(1.0, 0.0, 0.0),
        )
        .ok_or("gear plane bot")?;

        let mut bot_edges = Vec::with_capacity(n);
        for i in (0..n).rev() {
            bot_edges.push(OrientedEdge::reversed(eb[i].clone()));
        }
        faces.push(Face::simple(FaceGeometry::Plane(p_bot), Wire::new(bot_edges)));

        // 天面 Face (+Z 法線)
        let p_top = PlaneSurface3::new(
            Point3::new(0.0, 0.0, thickness),
            Vec3::new(1.0, 0.0, 0.0),
            Vec3::new(0.0, 1.0, 0.0),
        )
        .ok_or("gear plane top")?;

        let mut top_edges = Vec::with_capacity(n);
        for i in 0..n {
            top_edges.push(OrientedEdge::forward(et[i].clone()));
        }
        faces.push(Face::simple(FaceGeometry::Plane(p_top), Wire::new(top_edges)));

        let shell = Shell::closed(faces);
        let gear_solid = crate::validated_solid(shell)?;

        Ok(gear_solid)
    }
}
