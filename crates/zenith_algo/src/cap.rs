use zenith_geom::{ControlPoint3, KnotVector, NurbsSurface3, PlaneSurface3};
use zenith_math::{Point3, Vec3};
use zenith_topo::{Face, FaceGeometry, Orientation, Wire};

/// 穴埋め・端面キャップ・N辺パッチビルダー（Plasticity / Rhino風 自由曲面穴塞ぎ）
pub struct CapBuilder;

impl CapBuilder {
    /// 任意の平面境界ワイヤから平面Face（Planar Cap）を生成して穴を塞ぐ
    pub fn make_planar_cap(wire: Wire) -> Result<Face, String> {
        Self::make_planar_cap_with_holes(wire, Vec::new())
    }

    /// A planar cap with holes in it.
    ///
    /// Cutting a torus with a plane leaves two loops, one inside the other, and
    /// what they bound between them is an annulus. Closing each loop with its
    /// own disc covers the hole as well as the ring, and leaves every edge on
    /// the inner loop used twice in the same direction.
    pub fn make_planar_cap_with_holes(wire: Wire, holes: Vec<Wire>) -> Result<Face, String> {
        if wire.edges.is_empty() {
            return Err("Wire has no edges".to_string());
        }

        // 頂点群から中心点と法線ベクトルを算出（Newell's method）
        let mut center = Point3::new(0.0, 0.0, 0.0);
        let mut normal = Vec3::new(0.0, 0.0, 0.0);
        let n_edges = wire.edges.len();

        let mut pts = Vec::with_capacity(n_edges);
        for oe in &wire.edges {
            pts.push(oe.start_vertex().point);
        }

        for p in &pts {
            center.x += p.x;
            center.y += p.y;
            center.z += p.z;
        }
        center.x /= n_edges as f64;
        center.y /= n_edges as f64;
        center.z /= n_edges as f64;

        // Newellの多角形法線算出
        for i in 0..pts.len() {
            let curr = pts[i];
            let next = pts[(i + 1) % pts.len()];
            normal.x += (curr.y - next.y) * (curr.z + next.z);
            normal.y += (curr.z - next.z) * (curr.x + next.x);
            normal.z += (curr.x - next.x) * (curr.y + next.y);
        }

        if normal.norm() < 1e-9 {
            return Err("Degenerate planar wire".to_string());
        }
        let normal = normal.normalize();

        // 平面上の主軸 u_axis, v_axis
        let u_axis = if normal.x.abs() < 0.9 {
            normal.cross(&Vec3::new(1.0, 0.0, 0.0)).normalize()
        } else {
            normal.cross(&Vec3::new(0.0, 1.0, 0.0)).normalize()
        };
        let v_axis = normal.cross(&u_axis).normalize();

        let plane = PlaneSurface3::new(center, u_axis, v_axis)
            .ok_or("Failed to create planar cap surface")?;

        if holes.is_empty() {
            return Ok(Face::simple(FaceGeometry::Plane(plane), wire));
        }

        Ok(Face::new(
            FaceGeometry::Plane(plane),
            wire,
            holes,
            Orientation::Forward,
            1e-6,
        ))
    }

    /// 任意の3D境界ワイヤからドーム状・滑らかなテンション曲面パッチ（Dome Cap / Curved Patch）を生成
    pub fn make_dome_patch(wire: Wire, bulge: f64) -> Result<Face, String> {
        if wire.edges.is_empty() {
            return Err("Wire has no edges".to_string());
        }

        let n = wire.edges.len();
        let mut pts = Vec::with_capacity(n);
        for oe in &wire.edges {
            pts.push(oe.start_vertex().point);
        }

        let mut center = Point3::new(0.0, 0.0, 0.0);
        let mut normal = Vec3::new(0.0, 0.0, 0.0);
        for p in &pts {
            center.x += p.x;
            center.y += p.y;
            center.z += p.z;
        }
        center.x /= n as f64;
        center.y /= n as f64;
        center.z /= n as f64;

        for i in 0..n {
            let curr = pts[i];
            let next = pts[(i + 1) % n];
            normal.x += (curr.y - next.y) * (curr.z + next.z);
            normal.y += (curr.z - next.z) * (curr.x + next.x);
            normal.z += (curr.x - next.x) * (curr.y + next.y);
        }
        let normal = if normal.norm() > 1e-9 {
            normal.normalize()
        } else {
            Vec3::new(0.0, 0.0, 1.0)
        };

        // ドーム頂点
        let apex = center + normal * bulge;

        // 4境界CoonsパッチまたはNURBS曲面として構成
        let p0 = pts[0];
        let p1 = pts[n / 4];
        let p2 = pts[n / 2];
        let p3 = pts[3 * n / 4];

        let mid = |a: Point3, b: Point3| -> Point3 { Point3::from((a.coords + b.coords) * 0.5) };

        let row0 = vec![
            ControlPoint3::unweighted(p0),
            ControlPoint3::unweighted(mid(p0, p3)),
            ControlPoint3::unweighted(p3),
        ];
        let row1 = vec![
            ControlPoint3::unweighted(mid(p0, p1)),
            ControlPoint3::unweighted(apex),
            ControlPoint3::unweighted(mid(p2, p3)),
        ];
        let row2 = vec![
            ControlPoint3::unweighted(p1),
            ControlPoint3::unweighted(mid(p1, p2)),
            ControlPoint3::unweighted(p2),
        ];

        let nurbs = NurbsSurface3::new(
            2,
            2,
            vec![row0, row1, row2],
            KnotVector::clamped_uniform(3, 2),
            KnotVector::clamped_uniform(3, 2),
        )?;

        Ok(Face::simple(FaceGeometry::Nurbs(nurbs), wire))
    }
}
