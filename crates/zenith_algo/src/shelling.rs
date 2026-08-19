//! Zenith Algo: 薄肉シェル化エンジン (Thin-Wall Hollow Shelling)
//! 任意のソリッドから指定開口面を除去し、均一肉厚 t で中空ソリッドを自動構築。

use zenith_geom::PlaneSurface3;
use zenith_math::{Point3, Vec3};
use zenith_topo::{Edge, Face, FaceGeometry, OrientedEdge, Shell, Solid, Vertex, Wire};

pub struct ShellingBuilder;

fn make_plane_face(
    origin: Point3,
    u: Vec3,
    v_axis: Vec3,
    edges: Vec<OrientedEdge>,
) -> Result<Face, String> {
    let plane = PlaneSurface3::new(origin, u, v_axis).ok_or("Failed to create plane")?;
    let wire = Wire::new(edges);
    Ok(Face::simple(FaceGeometry::Plane(plane), wire))
}

impl ShellingBuilder {
    /// 直方体ソリッド（dx, dy, dz）から天面開口を除去し、
    /// 肉厚 thickness の薄肉ボックス容器（Open-Top Box）を構築（完全マニホールド閉B-Rep）
    pub fn make_open_box(
        dx: f64,
        dy: f64,
        dz: f64,
        thickness: f64,
    ) -> Result<Solid, String> {
        let t = thickness;
        if t <= 1e-6 || dx <= 2.0 * t || dy <= 2.0 * t || dz <= t {
            return Err("Wall thickness is too large for box dimensions".to_string());
        }

        // 外側 8 頂点 (vo0..vo7)
        // vo0=(0,0,0), vo1=(dx,0,0), vo2=(dx,dy,0), vo3=(0,dy,0)
        // vo4=(0,0,dz), vo5=(dx,0,dz), vo6=(dx,dy,dz), vo7=(0,dy,dz)
        let vo = [
            Vertex::from_point(Point3::new(0.0, 0.0, 0.0)),
            Vertex::from_point(Point3::new(dx, 0.0, 0.0)),
            Vertex::from_point(Point3::new(dx, dy, 0.0)),
            Vertex::from_point(Point3::new(0.0, dy, 0.0)),
            Vertex::from_point(Point3::new(0.0, 0.0, dz)),
            Vertex::from_point(Point3::new(dx, 0.0, dz)),
            Vertex::from_point(Point3::new(dx, dy, dz)),
            Vertex::from_point(Point3::new(0.0, dy, dz)),
        ];

        // 内側 8 頂点 (vi0..vi7) (z_top = dz で天面開口リム)
        // vi0=(t,t,t), vi1=(dx-t,t,t), vi2=(dx-t,dy-t,t), vi3=(t,dy-t,t)
        // vi4=(t,t,dz), vi5=(dx-t,t,dz), vi6=(dx-t,dy-t,dz), vi7=(t,dy-t,dz)
        let vi = [
            Vertex::from_point(Point3::new(t, t, t)),
            Vertex::from_point(Point3::new(dx - t, t, t)),
            Vertex::from_point(Point3::new(dx - t, dy - t, t)),
            Vertex::from_point(Point3::new(t, dy - t, t)),
            Vertex::from_point(Point3::new(t, t, dz)),
            Vertex::from_point(Point3::new(dx - t, t, dz)),
            Vertex::from_point(Point3::new(dx - t, dy - t, dz)),
            Vertex::from_point(Point3::new(t, dy - t, dz)),
        ];

        // 外側エッジ群
        let eo_01 = Edge::line_between(vo[0].clone(), vo[1].clone())?;
        let eo_12 = Edge::line_between(vo[1].clone(), vo[2].clone())?;
        let eo_23 = Edge::line_between(vo[2].clone(), vo[3].clone())?;
        let eo_30 = Edge::line_between(vo[3].clone(), vo[0].clone())?;

        let eo_45 = Edge::line_between(vo[4].clone(), vo[5].clone())?;
        let eo_56 = Edge::line_between(vo[5].clone(), vo[6].clone())?;
        let eo_67 = Edge::line_between(vo[6].clone(), vo[7].clone())?;
        let eo_74 = Edge::line_between(vo[7].clone(), vo[4].clone())?;

        let eo_04 = Edge::line_between(vo[0].clone(), vo[4].clone())?;
        let eo_15 = Edge::line_between(vo[1].clone(), vo[5].clone())?;
        let eo_26 = Edge::line_between(vo[2].clone(), vo[6].clone())?;
        let eo_37 = Edge::line_between(vo[3].clone(), vo[7].clone())?;

        // 内側エッジ群
        let ei_01 = Edge::line_between(vi[0].clone(), vi[1].clone())?;
        let ei_12 = Edge::line_between(vi[1].clone(), vi[2].clone())?;
        let ei_23 = Edge::line_between(vi[2].clone(), vi[3].clone())?;
        let ei_30 = Edge::line_between(vi[3].clone(), vi[0].clone())?;

        let ei_45 = Edge::line_between(vi[4].clone(), vi[5].clone())?;
        let ei_56 = Edge::line_between(vi[5].clone(), vi[6].clone())?;
        let ei_67 = Edge::line_between(vi[6].clone(), vi[7].clone())?;
        let ei_74 = Edge::line_between(vi[7].clone(), vi[4].clone())?;

        let ei_04 = Edge::line_between(vi[0].clone(), vi[4].clone())?;
        let ei_15 = Edge::line_between(vi[1].clone(), vi[5].clone())?;
        let ei_26 = Edge::line_between(vi[2].clone(), vi[6].clone())?;
        let ei_37 = Edge::line_between(vi[3].clone(), vi[7].clone())?;

        // 1. 外側底面 (Bottom: -Z, 法線 -Z) vo0 -> vo3 -> vo2 -> vo1
        let f_bot = make_plane_face(
            Point3::new(0.0, 0.0, 0.0),
            Vec3::new(0.0, dy, 0.0),
            Vec3::new(dx, 0.0, 0.0),
            vec![
                OrientedEdge::reversed(eo_30.clone()),
                OrientedEdge::reversed(eo_23.clone()),
                OrientedEdge::reversed(eo_12.clone()),
                OrientedEdge::reversed(eo_01.clone()),
            ],
        )?;

        // 2. 内側底面 (Inner Bottom: +Z, 法線 +Z) vi0 -> vi1 -> vi2 -> vi3
        let f_in_bot = make_plane_face(
            Point3::new(t, t, t),
            Vec3::new(dx - 2.0 * t, 0.0, 0.0),
            Vec3::new(0.0, dy - 2.0 * t, 0.0),
            vec![
                OrientedEdge::forward(ei_01.clone()),
                OrientedEdge::forward(ei_12.clone()),
                OrientedEdge::forward(ei_23.clone()),
                OrientedEdge::forward(ei_30.clone()),
            ],
        )?;

        // 3. 外側4側面
        // Front (y=0, 法線 -Y): vo0 -> vo1 -> vo5 -> vo4
        let f_front = make_plane_face(
            Point3::new(0.0, 0.0, 0.0),
            Vec3::new(dx, 0.0, 0.0),
            Vec3::new(0.0, 0.0, dz),
            vec![
                OrientedEdge::forward(eo_01.clone()),
                OrientedEdge::forward(eo_15.clone()),
                OrientedEdge::reversed(eo_45.clone()),
                OrientedEdge::reversed(eo_04.clone()),
            ],
        )?;

        // Right (x=dx, 法線 +X): vo1 -> vo2 -> vo6 -> vo5
        let f_right = make_plane_face(
            Point3::new(dx, 0.0, 0.0),
            Vec3::new(0.0, dy, 0.0),
            Vec3::new(0.0, 0.0, dz),
            vec![
                OrientedEdge::forward(eo_12.clone()),
                OrientedEdge::forward(eo_26.clone()),
                OrientedEdge::reversed(eo_56.clone()),
                OrientedEdge::reversed(eo_15.clone()),
            ],
        )?;

        // Back (y=dy, 法線 +Y): vo2 -> vo3 -> vo7 -> vo6
        let f_back = make_plane_face(
            Point3::new(0.0, dy, 0.0),
            Vec3::new(0.0, 0.0, dz),
            Vec3::new(dx, 0.0, 0.0),
            vec![
                OrientedEdge::forward(eo_23.clone()),
                OrientedEdge::forward(eo_37.clone()),
                OrientedEdge::reversed(eo_67.clone()),
                OrientedEdge::reversed(eo_26.clone()),
            ],
        )?;

        // Left (x=0, 法線 -X): vo3 -> vo0 -> vo4 -> vo7
        let f_left = make_plane_face(
            Point3::new(0.0, 0.0, 0.0),
            Vec3::new(0.0, 0.0, dz),
            Vec3::new(0.0, dy, 0.0),
            vec![
                OrientedEdge::forward(eo_30.clone()),
                OrientedEdge::forward(eo_04.clone()),
                OrientedEdge::reversed(eo_74.clone()),
                OrientedEdge::reversed(eo_37.clone()),
            ],
        )?;

        // 4. 内側4側面 (キャビティ壁面)
        // Inner Front (y=t, 法線 +Y): vi0 -> vi4 -> vi5 -> vi1
        let f_in_front = make_plane_face(
            Point3::new(t, t, t),
            Vec3::new(0.0, 0.0, dz - t),
            Vec3::new(dx - 2.0 * t, 0.0, 0.0),
            vec![
                OrientedEdge::forward(ei_04.clone()),
                OrientedEdge::forward(ei_45.clone()),
                OrientedEdge::reversed(ei_15.clone()),
                OrientedEdge::reversed(ei_01.clone()),
            ],
        )?;

        // Inner Right (x=dx-t, 法線 -X): vi1 -> vi5 -> vi6 -> vi2
        let f_in_right = make_plane_face(
            Point3::new(dx - t, t, t),
            Vec3::new(0.0, 0.0, dz - t),
            Vec3::new(0.0, dy - 2.0 * t, 0.0),
            vec![
                OrientedEdge::forward(ei_15.clone()),
                OrientedEdge::forward(ei_56.clone()),
                OrientedEdge::reversed(ei_26.clone()),
                OrientedEdge::reversed(ei_12.clone()),
            ],
        )?;

        // Inner Back (y=dy-t, 法線 -Y): vi2 -> vi6 -> vi7 -> vi3
        let f_in_back = make_plane_face(
            Point3::new(t, dy - t, t),
            Vec3::new(dx - 2.0 * t, 0.0, 0.0),
            Vec3::new(0.0, 0.0, dz - t),
            vec![
                OrientedEdge::forward(ei_26.clone()),
                OrientedEdge::forward(ei_67.clone()),
                OrientedEdge::reversed(ei_37.clone()),
                OrientedEdge::reversed(ei_23.clone()),
            ],
        )?;

        // Inner Left (x=t, 法線 +X): vi3 -> vi7 -> vi4 -> vi0
        let f_in_left = make_plane_face(
            Point3::new(t, t, t),
            Vec3::new(0.0, dy - 2.0 * t, 0.0),
            Vec3::new(0.0, 0.0, dz - t),
            vec![
                OrientedEdge::forward(ei_37.clone()),
                OrientedEdge::forward(ei_74.clone()),
                OrientedEdge::reversed(ei_04.clone()),
                OrientedEdge::reversed(ei_30.clone()),
            ],
        )?;

        // 5. 天面開口部リム (Top Rim: z=dz, 外側ループ vo4..vo7 と 内側穴ループ vi4..vi7)
        let f_rim = Face::new(
            FaceGeometry::Plane(PlaneSurface3::new(Point3::new(0.0, 0.0, dz), Vec3::new(1.0, 0.0, 0.0), Vec3::new(0.0, 1.0, 0.0)).unwrap()),
            Wire::new(vec![
                OrientedEdge::forward(eo_45.clone()),
                OrientedEdge::forward(eo_56.clone()),
                OrientedEdge::forward(eo_67.clone()),
                OrientedEdge::forward(eo_74.clone()),
            ]),
            vec![
                Wire::new(vec![
                    OrientedEdge::reversed(ei_74.clone()),
                    OrientedEdge::reversed(ei_67.clone()),
                    OrientedEdge::reversed(ei_56.clone()),
                    OrientedEdge::reversed(ei_45.clone()),
                ])
            ],
            zenith_topo::Orientation::Forward,
            1e-6,
        );

        let faces = vec![
            f_bot, f_in_bot,
            f_front, f_right, f_back, f_left,
            f_in_front, f_in_right, f_in_back, f_in_left,
            f_rim,
        ];

        let shell = Shell::closed(faces);
        crate::validated_solid(shell)
    }
}
