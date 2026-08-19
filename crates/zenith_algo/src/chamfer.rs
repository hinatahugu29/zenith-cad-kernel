use zenith_geom::PlaneSurface3;
use zenith_math::{Point3, Tolerance, Vec3};
use zenith_topo::{Edge, Face, FaceGeometry, OrientedEdge, Shell, Solid, Vertex, Wire};

/// エッジ面取り（Chamfer）ビルダー
pub struct ChamferBuilder;

impl ChamferBuilder {
    /// 直方体のZ軸方向4エッジに面取り（Chamfer: 距離 c mm）を適用した完全閉B-Repソリッドを生成
    pub fn chamfer_box_z_edges(
        dx: f64,
        dy: f64,
        dz: f64,
        chamfer_dist: f64,
        _tol: &Tolerance,
    ) -> Result<Solid, String> {
        if chamfer_dist < 0.0 {
            return Err(format!(
                "Chamfer distance must not be negative, got {chamfer_dist}"
            ));
        }
        let c = chamfer_dist;
        if c <= 1e-6 {
            return crate::primitive::PrimitiveBuilder::make_box(dx, dy, dz);
        }
        if 2.0 * c >= dx.min(dy) {
            return Err(format!(
                "Chamfer distance {c} must be smaller than half the shorter side ({})",
                dx.min(dy) * 0.5
            ));
        }

        // 1. 底面 (z=0) の8頂点（反時計回り）
        let p_b = [
            Point3::new(c, 0.0, 0.0),      // 0: 前面左
            Point3::new(dx - c, 0.0, 0.0), // 1: 前面右
            Point3::new(dx, c, 0.0),       // 2: 右面手前
            Point3::new(dx, dy - c, 0.0),  // 3: 右面奥
            Point3::new(dx - c, dy, 0.0),  // 4: 背面右
            Point3::new(c, dy, 0.0),       // 5: 背面左
            Point3::new(0.0, dy - c, 0.0), // 6: 左面奥
            Point3::new(0.0, c, 0.0),      // 7: 左面手前
        ];

        // 2. 天面 (z=dz) の8頂点
        let p_t = [
            Point3::new(c, 0.0, dz),
            Point3::new(dx - c, 0.0, dz),
            Point3::new(dx, c, dz),
            Point3::new(dx, dy - c, dz),
            Point3::new(dx - c, dy, dz),
            Point3::new(c, dy, dz),
            Point3::new(0.0, dy - c, dz),
            Point3::new(0.0, c, dz),
        ];

        let vb: Vec<Vertex> = p_b.iter().map(|p| Vertex::from_point(*p)).collect();
        let vt: Vec<Vertex> = p_t.iter().map(|p| Vertex::from_point(*p)).collect();

        // 3. 底面エッジ（8本）と天面エッジ（8本）
        let mut eb = Vec::with_capacity(8);
        let mut et = Vec::with_capacity(8);
        for i in 0..8 {
            let next = (i + 1) % 8;
            eb.push(Edge::line_between(vb[i].clone(), vb[next].clone())?);
            et.push(Edge::line_between(vt[i].clone(), vt[next].clone())?);
        }

        // 4. 垂直エッジ（8本）
        let mut ev = Vec::with_capacity(8);
        for i in 0..8 {
            ev.push(Edge::line_between(vb[i].clone(), vt[i].clone())?);
        }

        let mut faces = Vec::with_capacity(10);

        // 5. 8つの側面Face（4つの主平面 + 4つの面取り平面）
        // 0: 前面 (Front: vb0 -> vb1 -> vt1 -> vt0)
        let p_front =
            PlaneSurface3::new(p_b[0], Vec3::new(1.0, 0.0, 0.0), Vec3::new(0.0, 0.0, 1.0))
                .ok_or("plane front")?;
        faces.push(Face::simple(
            FaceGeometry::Plane(p_front),
            Wire::new(vec![
                OrientedEdge::forward(eb[0].clone()),
                OrientedEdge::forward(ev[1].clone()),
                OrientedEdge::reversed(et[0].clone()),
                OrientedEdge::reversed(ev[0].clone()),
            ]),
        ));

        // 1: 右前 面取り面 (Chamfer 1: vb1 -> vb2 -> vt2 -> vt1)
        let u_ch1 = Vec3::new(1.0, 1.0, 0.0).normalize();
        let p_ch1 =
            PlaneSurface3::new(p_b[1], u_ch1, Vec3::new(0.0, 0.0, 1.0)).ok_or("plane ch1")?;
        faces.push(Face::simple(
            FaceGeometry::Plane(p_ch1),
            Wire::new(vec![
                OrientedEdge::forward(eb[1].clone()),
                OrientedEdge::forward(ev[2].clone()),
                OrientedEdge::reversed(et[1].clone()),
                OrientedEdge::reversed(ev[1].clone()),
            ]),
        ));

        // 2: 右面 (Right: vb2 -> vb3 -> vt3 -> vt2)
        let p_right =
            PlaneSurface3::new(p_b[2], Vec3::new(0.0, 1.0, 0.0), Vec3::new(0.0, 0.0, 1.0))
                .ok_or("plane right")?;
        faces.push(Face::simple(
            FaceGeometry::Plane(p_right),
            Wire::new(vec![
                OrientedEdge::forward(eb[2].clone()),
                OrientedEdge::forward(ev[3].clone()),
                OrientedEdge::reversed(et[2].clone()),
                OrientedEdge::reversed(ev[2].clone()),
            ]),
        ));

        // 3: 右奥 面取り面 (Chamfer 2: vb3 -> vb4 -> vt4 -> vt3)
        let u_ch2 = Vec3::new(-1.0, 1.0, 0.0).normalize();
        let p_ch2 =
            PlaneSurface3::new(p_b[3], u_ch2, Vec3::new(0.0, 0.0, 1.0)).ok_or("plane ch2")?;
        faces.push(Face::simple(
            FaceGeometry::Plane(p_ch2),
            Wire::new(vec![
                OrientedEdge::forward(eb[3].clone()),
                OrientedEdge::forward(ev[4].clone()),
                OrientedEdge::reversed(et[3].clone()),
                OrientedEdge::reversed(ev[3].clone()),
            ]),
        ));

        // 4: 背面 (Back: vb4 -> vb5 -> vt5 -> vt4)
        let p_back =
            PlaneSurface3::new(p_b[4], Vec3::new(-1.0, 0.0, 0.0), Vec3::new(0.0, 0.0, 1.0))
                .ok_or("plane back")?;
        faces.push(Face::simple(
            FaceGeometry::Plane(p_back),
            Wire::new(vec![
                OrientedEdge::forward(eb[4].clone()),
                OrientedEdge::forward(ev[5].clone()),
                OrientedEdge::reversed(et[4].clone()),
                OrientedEdge::reversed(ev[4].clone()),
            ]),
        ));

        // 5: 左奥 面取り面 (Chamfer 3: vb5 -> vb6 -> vt6 -> vt5)
        let u_ch3 = Vec3::new(-1.0, -1.0, 0.0).normalize();
        let p_ch3 =
            PlaneSurface3::new(p_b[5], u_ch3, Vec3::new(0.0, 0.0, 1.0)).ok_or("plane ch3")?;
        faces.push(Face::simple(
            FaceGeometry::Plane(p_ch3),
            Wire::new(vec![
                OrientedEdge::forward(eb[5].clone()),
                OrientedEdge::forward(ev[6].clone()),
                OrientedEdge::reversed(et[5].clone()),
                OrientedEdge::reversed(ev[5].clone()),
            ]),
        ));

        // 6: 左面 (Left: vb6 -> vb7 -> vt7 -> vt6)
        let p_left =
            PlaneSurface3::new(p_b[6], Vec3::new(0.0, -1.0, 0.0), Vec3::new(0.0, 0.0, 1.0))
                .ok_or("plane left")?;
        faces.push(Face::simple(
            FaceGeometry::Plane(p_left),
            Wire::new(vec![
                OrientedEdge::forward(eb[6].clone()),
                OrientedEdge::forward(ev[7].clone()),
                OrientedEdge::reversed(et[6].clone()),
                OrientedEdge::reversed(ev[6].clone()),
            ]),
        ));

        // 7: 左前 面取り面 (Chamfer 4: vb7 -> vb0 -> vt0 -> vt7)
        let u_ch4 = Vec3::new(1.0, -1.0, 0.0).normalize();
        let p_ch4 =
            PlaneSurface3::new(p_b[7], u_ch4, Vec3::new(0.0, 0.0, 1.0)).ok_or("plane ch4")?;
        faces.push(Face::simple(
            FaceGeometry::Plane(p_ch4),
            Wire::new(vec![
                OrientedEdge::forward(eb[7].clone()),
                OrientedEdge::forward(ev[0].clone()),
                OrientedEdge::reversed(et[7].clone()),
                OrientedEdge::reversed(ev[7].clone()),
            ]),
        ));

        // 6. 底面 (Bottom: -Z, 反時計回り: vb7 -> vb6 -> ... -> vb0)
        let p_bottom = PlaneSurface3::new(
            Point3::new(0.0, 0.0, 0.0),
            Vec3::new(0.0, 1.0, 0.0),
            Vec3::new(1.0, 0.0, 0.0),
        )
        .ok_or("plane bottom")?;
        let wire_bottom = Wire::new(vec![
            OrientedEdge::reversed(eb[7].clone()),
            OrientedEdge::reversed(eb[6].clone()),
            OrientedEdge::reversed(eb[5].clone()),
            OrientedEdge::reversed(eb[4].clone()),
            OrientedEdge::reversed(eb[3].clone()),
            OrientedEdge::reversed(eb[2].clone()),
            OrientedEdge::reversed(eb[1].clone()),
            OrientedEdge::reversed(eb[0].clone()),
        ]);
        faces.push(Face::simple(FaceGeometry::Plane(p_bottom), wire_bottom));

        // 7. 天面 (Top: +Z, 反時計回り: vt0 -> vt1 -> ... -> vt7)
        let p_top = PlaneSurface3::new(
            Point3::new(0.0, 0.0, dz),
            Vec3::new(1.0, 0.0, 0.0),
            Vec3::new(0.0, 1.0, 0.0),
        )
        .ok_or("plane top")?;
        let wire_top = Wire::new(vec![
            OrientedEdge::forward(et[0].clone()),
            OrientedEdge::forward(et[1].clone()),
            OrientedEdge::forward(et[2].clone()),
            OrientedEdge::forward(et[3].clone()),
            OrientedEdge::forward(et[4].clone()),
            OrientedEdge::forward(et[5].clone()),
            OrientedEdge::forward(et[6].clone()),
            OrientedEdge::forward(et[7].clone()),
        ]);
        faces.push(Face::simple(FaceGeometry::Plane(p_top), wire_top));

        let shell = Shell::closed(faces);
        crate::validated_solid(shell)
    }
}
