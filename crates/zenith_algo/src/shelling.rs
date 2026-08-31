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
    pub fn make_open_box(dx: f64, dy: f64, dz: f64, thickness: f64) -> Result<Solid, String> {
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
            FaceGeometry::Plane(
                PlaneSurface3::new(
                    Point3::new(0.0, 0.0, dz),
                    Vec3::new(1.0, 0.0, 0.0),
                    Vec3::new(0.0, 1.0, 0.0),
                )
                .unwrap(),
            ),
            Wire::new(vec![
                OrientedEdge::forward(eo_45.clone()),
                OrientedEdge::forward(eo_56.clone()),
                OrientedEdge::forward(eo_67.clone()),
                OrientedEdge::forward(eo_74.clone()),
            ]),
            vec![Wire::new(vec![
                OrientedEdge::reversed(ei_74.clone()),
                OrientedEdge::reversed(ei_67.clone()),
                OrientedEdge::reversed(ei_56.clone()),
                OrientedEdge::reversed(ei_45.clone()),
            ])],
            zenith_topo::Orientation::Forward,
            1e-6,
        );

        let faces = vec![
            f_bot, f_in_bot, f_front, f_right, f_back, f_left, f_in_front, f_in_right, f_in_back,
            f_in_left, f_rim,
        ];

        let shell = Shell::closed(faces);
        let solid = Solid::new(shell, Vec::new());
        crate::sew::Sewer::sew_solid(&solid, &zenith_math::Tolerance::default()).map(|(s, _)| s)
    }

    /// 円柱（radius, height）から天面開口を除去し、
    /// 肉厚 thickness の薄肉円筒カップ容器（Open-Top Cylinder）を構築
    pub fn make_open_cylinder(radius: f64, height: f64, thickness: f64) -> Result<Solid, String> {
        let t = thickness;
        let r_out = radius;
        let r_in = radius - t;
        let h_out = height;

        if t <= 1e-6 || r_out <= t || h_out <= t {
            return Err(format!(
                "Wall thickness {t} is too large for cylinder (radius={r_out}, height={h_out})"
            ));
        }

        let weight = std::f64::consts::FRAC_1_SQRT_2;

        // 1. 外側 4頂点 (底面 z=0) & 4頂点 (天面リム z=h_out)
        let p_out_bot = [
            Point3::new(r_out, 0.0, 0.0),
            Point3::new(0.0, r_out, 0.0),
            Point3::new(-r_out, 0.0, 0.0),
            Point3::new(0.0, -r_out, 0.0),
        ];
        let p_out_top = [
            Point3::new(r_out, 0.0, h_out),
            Point3::new(0.0, r_out, h_out),
            Point3::new(-r_out, 0.0, h_out),
            Point3::new(0.0, -r_out, h_out),
        ];

        // 2. 内側 4頂点 (底面 z=t) & 4頂点 (天面リム z=h_out)
        let p_in_bot = [
            Point3::new(r_in, 0.0, t),
            Point3::new(0.0, r_in, t),
            Point3::new(-r_in, 0.0, t),
            Point3::new(0.0, -r_in, t),
        ];
        let p_in_top = [
            Point3::new(r_in, 0.0, h_out),
            Point3::new(0.0, r_in, h_out),
            Point3::new(-r_in, 0.0, h_out),
            Point3::new(0.0, -r_in, h_out),
        ];

        let v_out_bot: Vec<Vertex> = p_out_bot.iter().map(|p| Vertex::from_point(*p)).collect();
        let v_out_top: Vec<Vertex> = p_out_top.iter().map(|p| Vertex::from_point(*p)).collect();
        let v_in_bot: Vec<Vertex> = p_in_bot.iter().map(|p| Vertex::from_point(*p)).collect();
        let v_in_top: Vec<Vertex> = p_in_top.iter().map(|p| Vertex::from_point(*p)).collect();

        // 垂直エッジ
        let mut ev_out = Vec::with_capacity(4);
        let mut ev_in = Vec::with_capacity(4);
        for i in 0..4 {
            ev_out.push(Edge::line_between(
                v_out_bot[i].clone(),
                v_out_top[i].clone(),
            )?);
            ev_in.push(Edge::line_between(
                v_in_bot[i].clone(),
                v_in_top[i].clone(),
            )?);
        }

        // 外側円弧 & 内側円弧
        let mut e_out_bot = Vec::with_capacity(4);
        let mut e_out_top = Vec::with_capacity(4);
        let mut e_in_bot = Vec::with_capacity(4);
        let mut e_in_top = Vec::with_capacity(4);
        let mut faces = Vec::with_capacity(10);

        for i in 0..4 {
            let next = (i + 1) % 4;
            let (cx_out, cy_out) = match i {
                0 => (r_out, r_out),
                1 => (-r_out, r_out),
                2 => (-r_out, -r_out),
                _ => (r_out, -r_out),
            };
            let (cx_in, cy_in) = match i {
                0 => (r_in, r_in),
                1 => (-r_in, r_in),
                2 => (-r_in, -r_in),
                _ => (r_in, -r_in),
            };

            let corner_out_bot = Point3::new(cx_out, cy_out, 0.0);
            let corner_out_top = Point3::new(cx_out, cy_out, h_out);
            let corner_in_bot = Point3::new(cx_in, cy_in, t);
            let corner_in_top = Point3::new(cx_in, cy_in, h_out);

            let arc_out_bot = Edge::new(
                zenith_geom::NurbsCurve3::new(
                    2,
                    vec![
                        zenith_geom::ControlPoint3::unweighted(p_out_bot[i]),
                        zenith_geom::ControlPoint3::new(corner_out_bot, weight),
                        zenith_geom::ControlPoint3::unweighted(p_out_bot[next]),
                    ],
                    zenith_geom::KnotVector::clamped_uniform(3, 2),
                )?,
                v_out_bot[i].clone(),
                v_out_bot[next].clone(),
                1e-6,
            );

            let arc_out_top = Edge::new(
                zenith_geom::NurbsCurve3::new(
                    2,
                    vec![
                        zenith_geom::ControlPoint3::unweighted(p_out_top[i]),
                        zenith_geom::ControlPoint3::new(corner_out_top, weight),
                        zenith_geom::ControlPoint3::unweighted(p_out_top[next]),
                    ],
                    zenith_geom::KnotVector::clamped_uniform(3, 2),
                )?,
                v_out_top[i].clone(),
                v_out_top[next].clone(),
                1e-6,
            );

            let arc_in_bot = Edge::new(
                zenith_geom::NurbsCurve3::new(
                    2,
                    vec![
                        zenith_geom::ControlPoint3::unweighted(p_in_bot[i]),
                        zenith_geom::ControlPoint3::new(corner_in_bot, weight),
                        zenith_geom::ControlPoint3::unweighted(p_in_bot[next]),
                    ],
                    zenith_geom::KnotVector::clamped_uniform(3, 2),
                )?,
                v_in_bot[i].clone(),
                v_in_bot[next].clone(),
                1e-6,
            );

            let arc_in_top = Edge::new(
                zenith_geom::NurbsCurve3::new(
                    2,
                    vec![
                        zenith_geom::ControlPoint3::unweighted(p_in_top[i]),
                        zenith_geom::ControlPoint3::new(corner_in_top, weight),
                        zenith_geom::ControlPoint3::unweighted(p_in_top[next]),
                    ],
                    zenith_geom::KnotVector::clamped_uniform(3, 2),
                )?,
                v_in_top[i].clone(),
                v_in_top[next].clone(),
                1e-6,
            );

            e_out_bot.push(arc_out_bot.clone());
            e_out_top.push(arc_out_top.clone());
            e_in_bot.push(arc_in_bot.clone());
            e_in_top.push(arc_in_top.clone());

            // 外側円筒側面パッチ (法線外向き)
            let row0 = vec![
                zenith_geom::ControlPoint3::unweighted(p_out_bot[i]),
                zenith_geom::ControlPoint3::unweighted(p_out_top[i]),
            ];
            let row1 = vec![
                zenith_geom::ControlPoint3::new(corner_out_bot, weight),
                zenith_geom::ControlPoint3::new(corner_out_top, weight),
            ];
            let row2 = vec![
                zenith_geom::ControlPoint3::unweighted(p_out_bot[next]),
                zenith_geom::ControlPoint3::unweighted(p_out_top[next]),
            ];
            let surf_out = zenith_geom::NurbsSurface3::new(
                2,
                1,
                vec![row0, row1, row2],
                zenith_geom::KnotVector::clamped_uniform(3, 2),
                zenith_geom::KnotVector::clamped_uniform(2, 1),
            )?;
            let wire_out = Wire::new(vec![
                OrientedEdge::forward(arc_out_bot.clone()),
                OrientedEdge::forward(ev_out[next].clone()),
                OrientedEdge::reversed(arc_out_top.clone()),
                OrientedEdge::reversed(ev_out[i].clone()),
            ]);
            faces.push(Face::simple(FaceGeometry::Nurbs(surf_out), wire_out));

            // 内側円筒側面パッチ (キャビティ壁面、法線内向き)
            let in_row0 = vec![
                zenith_geom::ControlPoint3::unweighted(p_in_bot[next]),
                zenith_geom::ControlPoint3::unweighted(p_in_top[next]),
            ];
            let in_row1 = vec![
                zenith_geom::ControlPoint3::new(corner_in_bot, weight),
                zenith_geom::ControlPoint3::new(corner_in_top, weight),
            ];
            let in_row2 = vec![
                zenith_geom::ControlPoint3::unweighted(p_in_bot[i]),
                zenith_geom::ControlPoint3::unweighted(p_in_top[i]),
            ];
            let surf_in = zenith_geom::NurbsSurface3::new(
                2,
                1,
                vec![in_row0, in_row1, in_row2],
                zenith_geom::KnotVector::clamped_uniform(3, 2),
                zenith_geom::KnotVector::clamped_uniform(2, 1),
            )?;
            let wire_in = Wire::new(vec![
                OrientedEdge::forward(arc_in_top.clone()),
                OrientedEdge::reversed(ev_in[next].clone()),
                OrientedEdge::reversed(arc_in_bot.clone()),
                OrientedEdge::forward(ev_in[i].clone()),
            ]);
            faces.push(Face::simple(FaceGeometry::Nurbs(surf_in), wire_in));
        }

        // 外側底面 (Plane z=0, 法線 -Z)
        let plane_bot = PlaneSurface3::new(
            Point3::new(0.0, 0.0, 0.0),
            Vec3::new(0.0, 1.0, 0.0),
            Vec3::new(1.0, 0.0, 0.0),
        )
        .unwrap();
        let wire_bot = Wire::new(vec![
            OrientedEdge::reversed(e_out_bot[3].clone()),
            OrientedEdge::reversed(e_out_bot[2].clone()),
            OrientedEdge::reversed(e_out_bot[1].clone()),
            OrientedEdge::reversed(e_out_bot[0].clone()),
        ]);
        faces.push(Face::simple(FaceGeometry::Plane(plane_bot), wire_bot));

        // 内側底面 (Plane z=t, 法線 +Z)
        let plane_in_bot = PlaneSurface3::new(
            Point3::new(0.0, 0.0, t),
            Vec3::new(1.0, 0.0, 0.0),
            Vec3::new(0.0, 1.0, 0.0),
        )
        .unwrap();
        let wire_in_bot = Wire::new(vec![
            OrientedEdge::forward(e_in_bot[0].clone()),
            OrientedEdge::forward(e_in_bot[1].clone()),
            OrientedEdge::forward(e_in_bot[2].clone()),
            OrientedEdge::forward(e_in_bot[3].clone()),
        ]);
        faces.push(Face::simple(FaceGeometry::Plane(plane_in_bot), wire_in_bot));

        // 天面開口部リム (Plane z=h_out, 外側円弧 e_out_top と 内側円弧 e_in_top)
        let plane_rim = PlaneSurface3::new(
            Point3::new(0.0, 0.0, h_out),
            Vec3::new(1.0, 0.0, 0.0),
            Vec3::new(0.0, 1.0, 0.0),
        )
        .unwrap();
        let outer_rim_wire = Wire::new(vec![
            OrientedEdge::forward(e_out_top[0].clone()),
            OrientedEdge::forward(e_out_top[1].clone()),
            OrientedEdge::forward(e_out_top[2].clone()),
            OrientedEdge::forward(e_out_top[3].clone()),
        ]);
        let inner_rim_wire = Wire::new(vec![
            OrientedEdge::reversed(e_in_top[3].clone()),
            OrientedEdge::reversed(e_in_top[2].clone()),
            OrientedEdge::reversed(e_in_top[1].clone()),
            OrientedEdge::reversed(e_in_top[0].clone()),
        ]);
        let f_rim = Face::new(
            FaceGeometry::Plane(plane_rim),
            outer_rim_wire,
            vec![inner_rim_wire],
            zenith_topo::Orientation::Forward,
            1e-6,
        );
        faces.push(f_rim);

        let shell = Shell::closed(faces);
        let solid = Solid::new(shell, Vec::new());
        crate::sew::Sewer::sew_solid(&solid, &zenith_math::Tolerance::default()).map(|(s, _)| s)
    }

    /// スロット柱（length, radius, height）から天面開口を除去し、
    /// 肉厚 thickness の薄肉スロットトレイ容器（Open-Top Slot Tray）を構築
    pub fn make_open_slot_prism(
        length: f64,
        radius: f64,
        height: f64,
        thickness: f64,
    ) -> Result<Solid, String> {
        let t = thickness;
        let l_half = length * 0.5;
        let r_out = radius;
        let r_in = radius - t;
        let h_out = height;

        if t <= 1e-6 || r_out <= t || h_out <= t || length <= 1e-6 {
            return Err(format!(
                "Wall thickness {t} is too large for slot prism (length={length}, radius={r_out}, height={h_out})"
            ));
        }

        let weight = std::f64::consts::FRAC_1_SQRT_2;

        // 6頂点 (2D)
        let loc_out = [
            (-l_half, -r_out),
            (l_half, -r_out),
            (l_half + r_out, 0.0),
            (l_half, r_out),
            (-l_half, r_out),
            (-l_half - r_out, 0.0),
        ];
        let loc_in = [
            (-l_half, -r_in),
            (l_half, -r_in),
            (l_half + r_in, 0.0),
            (l_half, r_in),
            (-l_half, r_in),
            (-l_half - r_in, 0.0),
        ];

        // 3D 頂点
        let p_out_bot: Vec<Point3> = loc_out
            .iter()
            .map(|&(x, y)| Point3::new(x, y, 0.0))
            .collect();
        let p_out_top: Vec<Point3> = loc_out
            .iter()
            .map(|&(x, y)| Point3::new(x, y, h_out))
            .collect();
        let p_in_bot: Vec<Point3> = loc_in.iter().map(|&(x, y)| Point3::new(x, y, t)).collect();
        let p_in_top: Vec<Point3> = loc_in
            .iter()
            .map(|&(x, y)| Point3::new(x, y, h_out))
            .collect();

        let v_out_bot: Vec<Vertex> = p_out_bot.iter().map(|p| Vertex::from_point(*p)).collect();
        let v_out_top: Vec<Vertex> = p_out_top.iter().map(|p| Vertex::from_point(*p)).collect();
        let v_in_bot: Vec<Vertex> = p_in_bot.iter().map(|p| Vertex::from_point(*p)).collect();
        let v_in_top: Vec<Vertex> = p_in_top.iter().map(|p| Vertex::from_point(*p)).collect();

        // 垂直エッジ (6本外側 + 6本内側)
        let mut ev_out = Vec::with_capacity(6);
        let mut ev_in = Vec::with_capacity(6);
        for i in 0..6 {
            ev_out.push(Edge::line_between(
                v_out_bot[i].clone(),
                v_out_top[i].clone(),
            )?);
            ev_in.push(Edge::line_between(
                v_in_bot[i].clone(),
                v_in_top[i].clone(),
            )?);
        }

        // 周方向エッジ
        let mut e_out_bot = Vec::with_capacity(6);
        let mut e_out_top = Vec::with_capacity(6);
        let mut e_in_bot = Vec::with_capacity(6);
        let mut e_in_top = Vec::with_capacity(6);
        let mut faces = Vec::with_capacity(15);

        for i in 0..6 {
            let next = (i + 1) % 6;
            let is_arc = matches!(i, 1 | 2 | 4 | 5);

            if !is_arc {
                // 直線部エッジ
                let eo_b = Edge::line_between(v_out_bot[i].clone(), v_out_bot[next].clone())?;
                let eo_t = Edge::line_between(v_out_top[i].clone(), v_out_top[next].clone())?;
                let ei_b = Edge::line_between(v_in_bot[i].clone(), v_in_bot[next].clone())?;
                let ei_t = Edge::line_between(v_in_top[i].clone(), v_in_top[next].clone())?;

                e_out_bot.push(eo_b.clone());
                e_out_top.push(eo_t.clone());
                e_in_bot.push(ei_b.clone());
                e_in_top.push(ei_t.clone());

                // 外側平面側面
                let y_val = if i == 0 { -r_out } else { r_out };
                let (u_vec, v_vec) = if i == 0 {
                    (Vec3::new(length, 0.0, 0.0), Vec3::new(0.0, 0.0, h_out))
                } else {
                    (Vec3::new(-length, 0.0, 0.0), Vec3::new(0.0, 0.0, h_out))
                };
                let origin_out = Point3::new(loc_out[i].0, y_val, 0.0);
                let p_surf_out = PlaneSurface3::new(origin_out, u_vec, v_vec).unwrap();
                let wire_out = Wire::new(vec![
                    OrientedEdge::forward(eo_b),
                    OrientedEdge::forward(ev_out[next].clone()),
                    OrientedEdge::reversed(eo_t),
                    OrientedEdge::reversed(ev_out[i].clone()),
                ]);
                faces.push(Face::simple(FaceGeometry::Plane(p_surf_out), wire_out));

                // 内側平面側面 (キャビティ壁面、法線内向き)
                let in_y_val = if i == 0 { -r_in } else { r_in };
                let origin_in = Point3::new(loc_in[i].0, in_y_val, t);
                let (in_u_vec, in_v_vec) = if i == 0 {
                    (Vec3::new(0.0, 0.0, h_out - t), Vec3::new(length, 0.0, 0.0))
                } else {
                    (Vec3::new(0.0, 0.0, h_out - t), Vec3::new(-length, 0.0, 0.0))
                };
                let p_surf_in = PlaneSurface3::new(origin_in, in_u_vec, in_v_vec).unwrap();
                let wire_in = Wire::new(vec![
                    OrientedEdge::forward(ev_in[i].clone()),
                    OrientedEdge::forward(ei_t),
                    OrientedEdge::reversed(ev_in[next].clone()),
                    OrientedEdge::reversed(ei_b),
                ]);
                faces.push(Face::simple(FaceGeometry::Plane(p_surf_in), wire_in));
            } else {
                // 円弧部
                let ((cx_out, cy_out), (cx_in, cy_in)) = match i {
                    1 => ((l_half + r_out, -r_out), (l_half + r_in, -r_in)),
                    2 => ((l_half + r_out, r_out), (l_half + r_in, r_in)),
                    4 => ((-l_half - r_out, r_out), (-l_half - r_in, r_in)),
                    _ => ((-l_half - r_out, -r_out), (-l_half - r_in, -r_in)),
                };

                let corner_out_bot = Point3::new(cx_out, cy_out, 0.0);
                let corner_out_top = Point3::new(cx_out, cy_out, h_out);
                let corner_in_bot = Point3::new(cx_in, cy_in, t);
                let corner_in_top = Point3::new(cx_in, cy_in, h_out);

                let arc_out_bot = Edge::new(
                    zenith_geom::NurbsCurve3::new(
                        2,
                        vec![
                            zenith_geom::ControlPoint3::unweighted(p_out_bot[i]),
                            zenith_geom::ControlPoint3::new(corner_out_bot, weight),
                            zenith_geom::ControlPoint3::unweighted(p_out_bot[next]),
                        ],
                        zenith_geom::KnotVector::clamped_uniform(3, 2),
                    )?,
                    v_out_bot[i].clone(),
                    v_out_bot[next].clone(),
                    1e-6,
                );

                let arc_out_top = Edge::new(
                    zenith_geom::NurbsCurve3::new(
                        2,
                        vec![
                            zenith_geom::ControlPoint3::unweighted(p_out_top[i]),
                            zenith_geom::ControlPoint3::new(corner_out_top, weight),
                            zenith_geom::ControlPoint3::unweighted(p_out_top[next]),
                        ],
                        zenith_geom::KnotVector::clamped_uniform(3, 2),
                    )?,
                    v_out_top[i].clone(),
                    v_out_top[next].clone(),
                    1e-6,
                );

                let arc_in_bot = Edge::new(
                    zenith_geom::NurbsCurve3::new(
                        2,
                        vec![
                            zenith_geom::ControlPoint3::unweighted(p_in_bot[i]),
                            zenith_geom::ControlPoint3::new(corner_in_bot, weight),
                            zenith_geom::ControlPoint3::unweighted(p_in_bot[next]),
                        ],
                        zenith_geom::KnotVector::clamped_uniform(3, 2),
                    )?,
                    v_in_bot[i].clone(),
                    v_in_bot[next].clone(),
                    1e-6,
                );

                let arc_in_top = Edge::new(
                    zenith_geom::NurbsCurve3::new(
                        2,
                        vec![
                            zenith_geom::ControlPoint3::unweighted(p_in_top[i]),
                            zenith_geom::ControlPoint3::new(corner_in_top, weight),
                            zenith_geom::ControlPoint3::unweighted(p_in_top[next]),
                        ],
                        zenith_geom::KnotVector::clamped_uniform(3, 2),
                    )?,
                    v_in_top[i].clone(),
                    v_in_top[next].clone(),
                    1e-6,
                );

                e_out_bot.push(arc_out_bot.clone());
                e_out_top.push(arc_out_top.clone());
                e_in_bot.push(arc_in_bot.clone());
                e_in_top.push(arc_in_top.clone());

                // 外側円筒側面パッチ
                let row0 = vec![
                    zenith_geom::ControlPoint3::unweighted(p_out_bot[i]),
                    zenith_geom::ControlPoint3::unweighted(p_out_top[i]),
                ];
                let row1 = vec![
                    zenith_geom::ControlPoint3::new(corner_out_bot, weight),
                    zenith_geom::ControlPoint3::new(corner_out_top, weight),
                ];
                let row2 = vec![
                    zenith_geom::ControlPoint3::unweighted(p_out_bot[next]),
                    zenith_geom::ControlPoint3::unweighted(p_out_top[next]),
                ];
                let surf_out = zenith_geom::NurbsSurface3::new(
                    2,
                    1,
                    vec![row0, row1, row2],
                    zenith_geom::KnotVector::clamped_uniform(3, 2),
                    zenith_geom::KnotVector::clamped_uniform(2, 1),
                )?;
                let wire_out = Wire::new(vec![
                    OrientedEdge::forward(arc_out_bot),
                    OrientedEdge::forward(ev_out[next].clone()),
                    OrientedEdge::reversed(arc_out_top.clone()),
                    OrientedEdge::reversed(ev_out[i].clone()),
                ]);
                faces.push(Face::simple(FaceGeometry::Nurbs(surf_out), wire_out));

                // 内側円筒側面パッチ (キャビティ壁面)
                let in_row0 = vec![
                    zenith_geom::ControlPoint3::unweighted(p_in_bot[next]),
                    zenith_geom::ControlPoint3::unweighted(p_in_top[next]),
                ];
                let in_row1 = vec![
                    zenith_geom::ControlPoint3::new(corner_in_bot, weight),
                    zenith_geom::ControlPoint3::new(corner_in_top, weight),
                ];
                let in_row2 = vec![
                    zenith_geom::ControlPoint3::unweighted(p_in_bot[i]),
                    zenith_geom::ControlPoint3::unweighted(p_in_top[i]),
                ];
                let surf_in = zenith_geom::NurbsSurface3::new(
                    2,
                    1,
                    vec![in_row0, in_row1, in_row2],
                    zenith_geom::KnotVector::clamped_uniform(3, 2),
                    zenith_geom::KnotVector::clamped_uniform(2, 1),
                )?;
                let wire_in = Wire::new(vec![
                    OrientedEdge::forward(arc_in_top),
                    OrientedEdge::reversed(ev_in[next].clone()),
                    OrientedEdge::reversed(arc_in_bot),
                    OrientedEdge::forward(ev_in[i].clone()),
                ]);
                faces.push(Face::simple(FaceGeometry::Nurbs(surf_in), wire_in));
            }
        }

        // 外側底面 (Plane z=0, 法線 -Z)
        let plane_bot = PlaneSurface3::new(
            Point3::new(0.0, 0.0, 0.0),
            Vec3::new(0.0, 1.0, 0.0),
            Vec3::new(1.0, 0.0, 0.0),
        )
        .unwrap();
        let wire_bot = Wire::new(vec![
            OrientedEdge::reversed(e_out_bot[5].clone()),
            OrientedEdge::reversed(e_out_bot[4].clone()),
            OrientedEdge::reversed(e_out_bot[3].clone()),
            OrientedEdge::reversed(e_out_bot[2].clone()),
            OrientedEdge::reversed(e_out_bot[1].clone()),
            OrientedEdge::reversed(e_out_bot[0].clone()),
        ]);
        faces.push(Face::simple(FaceGeometry::Plane(plane_bot), wire_bot));

        // 内側底面 (Plane z=t, 法線 +Z)
        let plane_in_bot = PlaneSurface3::new(
            Point3::new(0.0, 0.0, t),
            Vec3::new(1.0, 0.0, 0.0),
            Vec3::new(0.0, 1.0, 0.0),
        )
        .unwrap();
        let wire_in_bot = Wire::new(vec![
            OrientedEdge::forward(e_in_bot[0].clone()),
            OrientedEdge::forward(e_in_bot[1].clone()),
            OrientedEdge::forward(e_in_bot[2].clone()),
            OrientedEdge::forward(e_in_bot[3].clone()),
            OrientedEdge::forward(e_in_bot[4].clone()),
            OrientedEdge::forward(e_in_bot[5].clone()),
        ]);
        faces.push(Face::simple(FaceGeometry::Plane(plane_in_bot), wire_in_bot));

        // 天面開口部リム (Plane z=h_out, 外側スロットループ e_out_top と 内側スロット穴ループ e_in_top)
        let plane_rim = PlaneSurface3::new(
            Point3::new(0.0, 0.0, h_out),
            Vec3::new(1.0, 0.0, 0.0),
            Vec3::new(0.0, 1.0, 0.0),
        )
        .unwrap();
        let outer_rim_wire = Wire::new(vec![
            OrientedEdge::forward(e_out_top[0].clone()),
            OrientedEdge::forward(e_out_top[1].clone()),
            OrientedEdge::forward(e_out_top[2].clone()),
            OrientedEdge::forward(e_out_top[3].clone()),
            OrientedEdge::forward(e_out_top[4].clone()),
            OrientedEdge::forward(e_out_top[5].clone()),
        ]);
        let inner_rim_wire = Wire::new(vec![
            OrientedEdge::reversed(e_in_top[5].clone()),
            OrientedEdge::reversed(e_in_top[4].clone()),
            OrientedEdge::reversed(e_in_top[3].clone()),
            OrientedEdge::reversed(e_in_top[2].clone()),
            OrientedEdge::reversed(e_in_top[1].clone()),
            OrientedEdge::reversed(e_in_top[0].clone()),
        ]);
        let f_rim = Face::new(
            FaceGeometry::Plane(plane_rim),
            outer_rim_wire,
            vec![inner_rim_wire],
            zenith_topo::Orientation::Forward,
            1e-6,
        );
        faces.push(f_rim);

        let shell = Shell::closed(faces);
        let solid = Solid::new(shell, Vec::new());
        crate::sew::Sewer::sew_solid(&solid, &zenith_math::Tolerance::default()).map(|(s, _)| s)
    }
}
