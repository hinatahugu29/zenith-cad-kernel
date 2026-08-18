use zenith_geom::{ControlPoint3, KnotVector, NurbsCurve3, NurbsSurface3, PlaneSurface3};
use zenith_math::Vec3;
use zenith_topo::{Edge, Face, FaceGeometry, OrientedEdge, Shell, Solid, Vertex, Wire};

/// 3Dガイド曲線に沿った断面スイープ（パイプ・チューブ・異形断面）ビルダー
pub struct SweepBuilder;

impl SweepBuilder {
    /// 3D NURBS軌道曲線（パス）に沿って半径 radius の円形断面を掃引したスイープソリッドを生成（Rodrigues回転最小ねじれ標架による滑らかなパイプ）
    pub fn sweep_circle_along_curve(
        path: &NurbsCurve3,
        radius: f64,
        num_sections: usize,
    ) -> Result<Solid, String> {
        let n_sec = num_sections.max(8);
        let t_min = path.knots.knots[path.degree];
        let t_max = path.knots.knots[path.knots.knots.len() - 1 - path.degree];

        // 1. 各断面位置での位置と接線ベクトルのサンプリング
        let mut pts = Vec::with_capacity(n_sec);
        for i in 0..n_sec {
            let u = i as f64 / (n_sec - 1) as f64;
            let t = t_min + u * (t_max - t_min);
            let pt = path.evaluate(t);
            let ders = path.evaluate_derivatives(t, 1);
            let tangent = if ders.len() > 1 && ders[1].norm() > 1e-9 {
                ders[1].normalize()
            } else {
                Vec3::new(0.0, 0.0, 1.0)
            };
            pts.push((pt, tangent));
        }

        // 2. 滑らかな最小回転標架 (Parallel Transport Frame / RMF) の算出
        let mut frames = Vec::with_capacity(n_sec);

        // 初期標架 (t0, n0, b0)
        let t0 = pts[0].1;
        let n0 = if t0.x.abs() < 0.9 {
            t0.cross(&Vec3::new(1.0, 0.0, 0.0)).normalize()
        } else {
            t0.cross(&Vec3::new(0.0, 1.0, 0.0)).normalize()
        };
        let b0 = t0.cross(&n0).normalize();
        frames.push((pts[0].0, t0, n0, b0));

        // Rodrigues回転による連続標架伝播
        for i in 0..n_sec - 1 {
            let (_, t_curr, n_curr, _) = frames[i];
            let (x_next, t_next) = pts[i + 1];

            // t_curr から t_next への回転軸と角度
            let dot = t_curr.dot(&t_next).clamp(-1.0, 1.0);
            let axis = t_curr.cross(&t_next);
            let axis_len = axis.norm();

            let mut n_next = if axis_len > 1e-8 {
                let u_axis = axis / axis_len;
                let angle = dot.acos();
                // Rodrigues' rotation formula: v_rot = v*cos + (axis x v)*sin + axis*(axis.v)*(1-cos)
                let c = angle.cos();
                let s = angle.sin();
                n_curr * c + u_axis.cross(&n_curr) * s + u_axis * (u_axis.dot(&n_curr) * (1.0 - c))
            } else if dot < -0.99999 {
                -n_curr
            } else {
                n_curr
            };

            // 直交化 (Gram-Schmidt)
            n_next = (n_next - t_next * t_next.dot(&n_next)).normalize();
            let b_next = t_next.cross(&n_next).normalize();
            frames.push((x_next, t_next, n_next, b_next));
        }

        // 3. 4つの四分円有理NURBSサーフェスパッチ（パイプの外周側面）の構築
        let weight = std::f64::consts::FRAC_1_SQRT_2;
        let mut faces = Vec::new();

        let mut bottom_ring_edges = Vec::new();
        let mut top_ring_edges = Vec::new();

        // 4象限 (0..4) の曲面パッチ
        for quad in 0..4 {
            let (ang0, ang1) = match quad {
                0 => (0.0, std::f64::consts::FRAC_PI_2),
                1 => (std::f64::consts::FRAC_PI_2, std::f64::consts::PI),
                2 => (std::f64::consts::PI, 3.0 * std::f64::consts::FRAC_PI_2),
                _ => (
                    3.0 * std::f64::consts::FRAC_PI_2,
                    2.0 * std::f64::consts::PI,
                ),
            };

            let mut row0 = Vec::with_capacity(n_sec);
            let mut row1 = Vec::with_capacity(n_sec);
            let mut row2 = Vec::with_capacity(n_sec);

            for &(ctr, _t, normal, binormal) in frames.iter().take(n_sec) {
                let p_s = ctr + normal * (radius * ang0.cos()) + binormal * (radius * ang0.sin());
                let p_e = ctr + normal * (radius * ang1.cos()) + binormal * (radius * ang1.sin());
                let corner = ctr + (p_s - ctr) + (p_e - ctr);

                row0.push(ControlPoint3::unweighted(p_s));
                row1.push(ControlPoint3::new(corner, weight));
                row2.push(ControlPoint3::unweighted(p_e));
            }

            let s = NurbsSurface3::new(
                2,
                1,
                vec![row0.clone(), row1.clone(), row2.clone()],
                KnotVector::clamped_uniform(3, 2),
                KnotVector::clamped_uniform(n_sec, 1),
            )?;

            let p_start_bot = row0[0].point;
            let p_end_bot = row2[0].point;
            let p_start_top = row0[n_sec - 1].point;
            let p_end_top = row2[n_sec - 1].point;

            let vb_s = Vertex::from_point(p_start_bot);
            let vb_e = Vertex::from_point(p_end_bot);
            let vt_s = Vertex::from_point(p_start_top);
            let vt_e = Vertex::from_point(p_end_top);

            let arc_b = Edge::new(
                NurbsCurve3::new(
                    2,
                    vec![row0[0], row1[0], row2[0]],
                    KnotVector::clamped_uniform(3, 2),
                )?,
                vb_s.clone(),
                vb_e.clone(),
                1e-6,
            );
            let arc_t = Edge::new(
                NurbsCurve3::new(
                    2,
                    vec![row0[n_sec - 1], row1[n_sec - 1], row2[n_sec - 1]],
                    KnotVector::clamped_uniform(3, 2),
                )?,
                vt_s.clone(),
                vt_e.clone(),
                1e-6,
            );

            let ev_s = Edge::line_between(vb_s.clone(), vt_s.clone())?;
            let ev_e = Edge::line_between(vb_e.clone(), vt_e.clone())?;

            bottom_ring_edges.push(arc_b.clone());
            top_ring_edges.push(arc_t.clone());

            let wire = Wire::new(vec![
                OrientedEdge::forward(arc_b),
                OrientedEdge::forward(ev_e),
                OrientedEdge::reversed(arc_t),
                OrientedEdge::reversed(ev_s),
            ]);

            faces.push(Face::simple(FaceGeometry::Nurbs(s), wire));
        }

        // 4. 始点端面 (Start Cap: PLANE, 外向き法線 -t0)
        let (ctr0, _t0, n0, b0) = frames[0];
        let p_start_cap = PlaneSurface3::new(ctr0, b0, n0).ok_or("start cap plane")?;
        let wire_start_cap = Wire::new(vec![
            OrientedEdge::reversed(bottom_ring_edges[3].clone()),
            OrientedEdge::reversed(bottom_ring_edges[2].clone()),
            OrientedEdge::reversed(bottom_ring_edges[1].clone()),
            OrientedEdge::reversed(bottom_ring_edges[0].clone()),
        ]);
        faces.push(Face::simple(
            FaceGeometry::Plane(p_start_cap),
            wire_start_cap,
        ));

        // 5. 終点端面 (End Cap: PLANE, 外向き法線 +t1)
        let (ctr1, _t1, n1, b1) = frames[n_sec - 1];
        let p_end_cap = PlaneSurface3::new(ctr1, n1, b1).ok_or("end cap plane")?;
        let wire_end_cap = Wire::new(vec![
            OrientedEdge::forward(top_ring_edges[0].clone()),
            OrientedEdge::forward(top_ring_edges[1].clone()),
            OrientedEdge::forward(top_ring_edges[2].clone()),
            OrientedEdge::forward(top_ring_edges[3].clone()),
        ]);
        faces.push(Face::simple(FaceGeometry::Plane(p_end_cap), wire_end_cap));

        let shell = Shell::closed(faces);
        crate::validated_solid(shell)
    }
}
