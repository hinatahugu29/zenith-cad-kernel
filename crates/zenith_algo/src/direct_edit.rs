use zenith_geom::PlaneSurface3;
use zenith_math::{Point3, Vec3};
use zenith_topo::{Edge, Face, FaceGeometry, OrientedEdge, Shell, Solid, Vertex, Wire};

/// 面・辺の幾何情報クエリ結果
#[derive(Debug, Clone, PartialEq)]
pub struct FaceInspection {
    pub area: f64,
    pub centroid: Point3,
    pub normal: Vec3,
    /// XY平面 (+Z軸) とのなす角度 (deg: 0〜180)
    pub angle_to_xy_deg: f64,
    /// XZ平面 (+Y軸) とのなす角度 (deg)
    pub angle_to_xz_deg: f64,
    /// YZ平面 (+X軸) とのなす角度 (deg)
    pub angle_to_yz_deg: f64,
}

/// エッジの幾何的性質（凸・凹・スムーズ・境界）
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum EdgeKind {
    /// 凸エッジ（外側角: フィレット/面取りの対象）
    Convex,
    /// 凹エッジ（内側角: リブ/内角フィレットの対象）
    Concave,
    /// スムーズエッジ（180度 G1連続接続）
    Smooth,
    /// 自由境界エッジ（単一面のみに所属）
    FreeBoundary,
}

#[derive(Debug, Clone, PartialEq)]
pub struct EdgeInspection {
    pub length: f64,
    pub start_point: Point3,
    pub end_point: Point3,
    pub midpoint: Point3,
    pub tangent: Vec3,
    /// 共有する2面の二面角 (deg: 0〜360)
    pub dihedral_angle_deg: Option<f64>,
    /// エッジの性質
    pub kind: EdgeKind,
}

/// Plasticity風 ダイレクトモデリング・幾何クエリエンジン
pub struct DirectModeling;

impl DirectModeling {
    /// 選択されたFaceの厳密な面積・重心・法線ベクトル・各座標平面との角度を解析
    pub fn inspect_face(face: &Face) -> Result<FaceInspection, String> {
        let tess_params = zenith_tess::TessellationParams {
            u_divisions: 16,
            v_divisions: 16,
        };
        let mesh = zenith_tess::tessellate_face(face, &tess_params);
        let mass = crate::mass_properties::MassCalculator::compute_from_mesh(&mesh);

        let normal = match &face.geometry {
            FaceGeometry::Plane(p) => {
                let n = p.normal;
                if face.orientation.is_forward() {
                    n
                } else {
                    -n
                }
            }
            FaceGeometry::Nurbs(s) => {
                let n = s.normal(0.5, 0.5).unwrap_or(Vec3::new(0.0, 0.0, 1.0));
                if face.orientation.is_forward() {
                    n
                } else {
                    -n
                }
            }
            _ => {
                if mesh.normals.is_empty() {
                    Vec3::new(0.0, 0.0, 1.0)
                } else {
                    let mut sum_n = Vec3::new(0.0, 0.0, 0.0);
                    for n in &mesh.normals {
                        sum_n += *n;
                    }
                    sum_n.normalize()
                }
            }
        };

        let z_axis = Vec3::new(0.0, 0.0, 1.0);
        let y_axis = Vec3::new(0.0, 1.0, 0.0);
        let x_axis = Vec3::new(1.0, 0.0, 0.0);

        let angle_to_xy_deg = normal.dot(&z_axis).clamp(-1.0, 1.0).acos().to_degrees();
        let angle_to_xz_deg = normal.dot(&y_axis).clamp(-1.0, 1.0).acos().to_degrees();
        let angle_to_yz_deg = normal.dot(&x_axis).clamp(-1.0, 1.0).acos().to_degrees();

        Ok(FaceInspection {
            area: mass.surface_area,
            centroid: mass.center_of_mass,
            normal,
            angle_to_xy_deg,
            angle_to_xz_deg,
            angle_to_yz_deg,
        })
    }

    /// 選択されたEdgeの長さ・端点・接線方向を解析
    pub fn inspect_edge(edge: &Edge) -> EdgeInspection {
        let p_s = edge.start_vertex.point;
        let p_e = edge.end_vertex.point;
        let len = (p_e - p_s).norm();
        let mid = Point3::from((p_s.coords + p_e.coords) * 0.5);
        let tangent = if len > 1e-9 {
            (p_e - p_s).normalize()
        } else {
            Vec3::new(1.0, 0.0, 0.0)
        };

        EdgeInspection {
            length: len,
            start_point: p_s,
            end_point: p_e,
            midpoint: mid,
            tangent,
            dihedral_angle_deg: None,
            kind: EdgeKind::FreeBoundary,
        }
    }

    /// ソリッド内の特定エッジについて、隣接2面との二面角および凸/凹/スムーズ判定を行う
    pub fn inspect_solid_edge(solid: &Solid, edge_id: u64) -> Result<EdgeInspection, String> {
        let mut matching_edge = None;
        let mut adjacent_faces = Vec::new();

        for face in &solid.outer_shell.faces {
            for oe in &face.outer_wire.edges {
                if oe.edge.id == edge_id {
                    if matching_edge.is_none() {
                        matching_edge = Some(oe.edge.clone());
                    }
                    adjacent_faces.push(face.clone());
                }
            }
        }

        let edge =
            matching_edge.ok_or_else(|| format!("Edge ID {} not found in solid", edge_id))?;
        let mut insp = Self::inspect_edge(&edge);

        if adjacent_faces.len() == 2 {
            let insp_a = Self::inspect_face(&adjacent_faces[0])?;
            let insp_b = Self::inspect_face(&adjacent_faces[1])?;

            let n_a = insp_a.normal;
            let n_b = insp_b.normal;

            let dot = n_a.dot(&n_b).clamp(-1.0, 1.0);
            let angle_deg = dot.acos().to_degrees();

            let cross = n_a.cross(&n_b);
            let is_convex = cross.dot(&insp.tangent) >= -1e-6;

            let kind = if angle_deg < 1e-3 {
                EdgeKind::Smooth
            } else if is_convex {
                EdgeKind::Convex
            } else {
                EdgeKind::Concave
            };

            insp.dihedral_angle_deg = Some(180.0 - angle_deg);
            insp.kind = kind;
        }

        Ok(insp)
    }

    /// 選択されたFaceをその法線方向に距離 distance だけ押し出し（Push-Pull）するソリッド変形
    pub fn push_pull_face(
        solid: &Solid,
        face_index: usize,
        distance: f64,
    ) -> Result<Solid, String> {
        if distance.abs() < 1e-6 {
            return Ok(solid.clone());
        }

        let faces = &solid.outer_shell.faces;
        if face_index >= faces.len() {
            return Err("Invalid face index".to_string());
        }

        let target_face = &faces[face_index];
        let insp = Self::inspect_face(target_face)?;
        let offset_vec = insp.normal * distance;

        // 対象面の頂点セットを収集
        let mut target_vertex_pts = Vec::new();
        for oe in &target_face.outer_wire.edges {
            target_vertex_pts.push(oe.start_vertex().point);
            target_vertex_pts.push(oe.end_vertex().point);
        }

        let is_target_pt =
            |p: Point3| -> bool { target_vertex_pts.iter().any(|tp| (p - *tp).norm() < 1e-5) };

        let mut new_faces = Vec::with_capacity(faces.len());

        for f in faces.iter() {
            let mut new_edges = Vec::new();
            let mut shifted_pts = Vec::new();

            for oe in &f.outer_wire.edges {
                let p_s = oe.start_vertex().point;
                let p_e = oe.end_vertex().point;

                let new_p_s = if is_target_pt(p_s) {
                    p_s + offset_vec
                } else {
                    p_s
                };
                let new_p_e = if is_target_pt(p_e) {
                    p_e + offset_vec
                } else {
                    p_e
                };

                shifted_pts.push(new_p_s);

                let vs = Vertex::from_point(new_p_s);
                let ve = Vertex::from_point(new_p_e);
                let e = Edge::line_between(vs, ve)?;
                // ワイヤ進行順で作られたエッジのため forward で追加
                new_edges.push(OrientedEdge::forward(e));
            }

            // 移動後の境界点から平面を張り直し、Face幾何とWireを一致させる。
            let updated_geom = match &f.geometry {
                FaceGeometry::Plane(p) => {
                    if shifted_pts.len() >= 3 {
                        let origin = shifted_pts[0];
                        let u = shifted_pts[1] - shifted_pts[0];
                        let v = shifted_pts[shifted_pts.len() - 1] - shifted_pts[0];
                        FaceGeometry::Plane(PlaneSurface3::new(origin, u, v).unwrap_or(*p))
                    } else {
                        FaceGeometry::Plane(*p)
                    }
                }
                other => other.clone(),
            };

            new_faces.push(Face::simple(updated_geom, Wire::new(new_edges)));
        }

        let shell = Shell::closed(new_faces);
        crate::validated_solid(shell)
    }

    /// 選択されたFaceを特定の回転軸エッジまわりに角度 angle_deg だけ傾斜（Taper / Draft）
    pub fn taper_face(
        solid: &Solid,
        face_index: usize,
        axis_origin: Point3,
        axis_dir: Vec3,
        angle_deg: f64,
    ) -> Result<Solid, String> {
        if angle_deg.abs() < 1e-6 {
            return Ok(solid.clone());
        }

        let faces = &solid.outer_shell.faces;
        if face_index >= faces.len() {
            return Err("Invalid face index".to_string());
        }

        let rad = angle_deg.to_radians();
        let u_axis = axis_dir.normalize();
        let c = rad.cos();
        let s = rad.sin();

        let rotate_pt = |p: Point3| -> Point3 {
            let v = p - axis_origin;
            let v_rot = v * c + u_axis.cross(&v) * s + u_axis * (u_axis.dot(&v) * (1.0 - c));
            axis_origin + v_rot
        };

        let mut new_faces = faces.clone();
        let target_face = &faces[face_index];

        let updated_face = match &target_face.geometry {
            FaceGeometry::Plane(p) => {
                let new_origin = rotate_pt(p.origin);
                let new_u = p.u_axis * c
                    + u_axis.cross(&p.u_axis) * s
                    + u_axis * (u_axis.dot(&p.u_axis) * (1.0 - c));
                let new_v = p.v_axis * c
                    + u_axis.cross(&p.v_axis) * s
                    + u_axis * (u_axis.dot(&p.v_axis) * (1.0 - c));
                let new_p =
                    PlaneSurface3::new(new_origin, new_u, new_v).ok_or("taper plane fail")?;

                let mut new_edges = Vec::new();
                for oe in &target_face.outer_wire.edges {
                    let vs = Vertex::from_point(rotate_pt(oe.start_vertex().point));
                    let ve = Vertex::from_point(rotate_pt(oe.end_vertex().point));
                    let e = Edge::line_between(vs, ve)?;
                    new_edges.push(OrientedEdge::forward(e));
                }
                Face::simple(FaceGeometry::Plane(new_p), Wire::new(new_edges))
            }
            _ => return Err("Taper currently supports planar faces".to_string()),
        };

        new_faces[face_index] = updated_face;
        let shell = Shell::closed(new_faces);
        crate::validated_solid(shell)
    }

    /// 直方体の指定した単一垂直エッジ（0: X=0,Y=0; 1: X=dx,Y=0; 2: X=dx,Y=dy; 3: X=0,Y=dy）に半径 radius のフィレットを適用
    pub fn fillet_box_single_edge(
        dx: f64,
        dy: f64,
        dz: f64,
        edge_index: usize,
        radius: f64,
    ) -> Result<Solid, String> {
        if radius <= 0.0 || radius >= dx || radius >= dy {
            return Err(
                "Fillet radius must be positive and smaller than box dimensions".to_string(),
            );
        }

        let r = radius;
        let weight = std::f64::consts::FRAC_1_SQRT_2;

        // エッジ0 (原点 X=0, Y=0) をフィレットする場合
        if edge_index == 0 {
            // 底面多角形頂点（原点角を (r, 0) -> (0, r) の円弧で置換）
            // p0=(r, 0, 0), p1=(dx, 0, 0), p2=(dx, dy, 0), p3=(0, dy, 0), p4=(0, r, 0)
            let pb0 = Point3::new(r, 0.0, 0.0);
            let pb1 = Point3::new(dx, 0.0, 0.0);
            let pb2 = Point3::new(dx, dy, 0.0);
            let pb3 = Point3::new(0.0, dy, 0.0);
            let pb4 = Point3::new(0.0, r, 0.0);

            let pt0 = Point3::new(r, 0.0, dz);
            let pt1 = Point3::new(dx, 0.0, dz);
            let pt2 = Point3::new(dx, dy, dz);
            let pt3 = Point3::new(0.0, dy, dz);
            let pt4 = Point3::new(0.0, r, dz);

            let vb: Vec<Vertex> = vec![pb0, pb1, pb2, pb3, pb4]
                .into_iter()
                .map(Vertex::from_point)
                .collect();
            let vt: Vec<Vertex> = vec![pt0, pt1, pt2, pt3, pt4]
                .into_iter()
                .map(Vertex::from_point)
                .collect();

            // 垂直エッジ 5本
            let mut ev = Vec::new();
            for i in 0..5 {
                ev.push(Edge::line_between(vb[i].clone(), vt[i].clone())?);
            }

            // 直線底面・天面エッジ 4本
            let mut eb = Vec::new();
            let mut et = Vec::new();
            for i in 0..4 {
                eb.push(Edge::line_between(vb[i].clone(), vb[i + 1].clone())?);
                et.push(Edge::line_between(vt[i].clone(), vt[i + 1].clone())?);
            }

            // フィレット円弧エッジ (vb4 -> vb0, vt4 -> vt0)
            let corner_b = Point3::new(0.0, 0.0, 0.0);
            let corner_t = Point3::new(0.0, 0.0, dz);

            let arc_b = Edge::new(
                zenith_geom::NurbsCurve3::new(
                    2,
                    vec![
                        zenith_geom::ControlPoint3::unweighted(pb4),
                        zenith_geom::ControlPoint3::new(corner_b, weight),
                        zenith_geom::ControlPoint3::unweighted(pb0),
                    ],
                    zenith_geom::KnotVector::clamped_uniform(3, 2),
                )?,
                vb[4].clone(),
                vb[0].clone(),
                1e-6,
            );

            let arc_t = Edge::new(
                zenith_geom::NurbsCurve3::new(
                    2,
                    vec![
                        zenith_geom::ControlPoint3::unweighted(pt4),
                        zenith_geom::ControlPoint3::new(corner_t, weight),
                        zenith_geom::ControlPoint3::unweighted(pt0),
                    ],
                    zenith_geom::KnotVector::clamped_uniform(3, 2),
                )?,
                vt[4].clone(),
                vt[0].clone(),
                1e-6,
            );

            let mut faces = Vec::new();

            // 1. 平面側面 4面
            for i in 0..4 {
                let p_orig = vb[i].point;
                let u = vb[i + 1].point - vb[i].point;
                let v = vt[i].point - vb[i].point;
                let plane = PlaneSurface3::new(p_orig, u, v).ok_or("plane fail")?;
                let wire = Wire::new(vec![
                    OrientedEdge::forward(eb[i].clone()),
                    OrientedEdge::forward(ev[i + 1].clone()),
                    OrientedEdge::reversed(et[i].clone()),
                    OrientedEdge::reversed(ev[i].clone()),
                ]);
                faces.push(Face::simple(FaceGeometry::Plane(plane), wire));
            }

            // 2. 円弧フィレット側面 1面 (有理NURBS)
            let row0 = vec![
                zenith_geom::ControlPoint3::unweighted(pb4),
                zenith_geom::ControlPoint3::unweighted(pt4),
            ];
            let row1 = vec![
                zenith_geom::ControlPoint3::new(corner_b, weight),
                zenith_geom::ControlPoint3::new(corner_t, weight),
            ];
            let row2 = vec![
                zenith_geom::ControlPoint3::unweighted(pb0),
                zenith_geom::ControlPoint3::unweighted(pt0),
            ];

            let surf = zenith_geom::NurbsSurface3::new(
                2,
                1,
                vec![row0, row1, row2],
                zenith_geom::KnotVector::clamped_uniform(3, 2),
                zenith_geom::KnotVector::clamped_uniform(2, 1),
            )?;
            let wire_fillet = Wire::new(vec![
                OrientedEdge::forward(arc_b.clone()),
                OrientedEdge::forward(ev[0].clone()),
                OrientedEdge::reversed(arc_t.clone()),
                OrientedEdge::reversed(ev[4].clone()),
            ]);
            faces.push(Face::simple(FaceGeometry::Nurbs(surf), wire_fillet));

            // 3. 底面 (-Z PLANE)
            let p_bot = PlaneSurface3::new(
                Point3::new(0.0, 0.0, 0.0),
                Vec3::new(0.0, 1.0, 0.0),
                Vec3::new(1.0, 0.0, 0.0),
            )
            .ok_or("bot plane")?;
            let wire_bot = Wire::new(vec![
                OrientedEdge::reversed(arc_b),
                OrientedEdge::reversed(eb[3].clone()),
                OrientedEdge::reversed(eb[2].clone()),
                OrientedEdge::reversed(eb[1].clone()),
                OrientedEdge::reversed(eb[0].clone()),
            ]);
            faces.push(Face::simple(FaceGeometry::Plane(p_bot), wire_bot));

            // 4. 天面 (+Z PLANE)
            let p_top = PlaneSurface3::new(
                Point3::new(0.0, 0.0, dz),
                Vec3::new(1.0, 0.0, 0.0),
                Vec3::new(0.0, 1.0, 0.0),
            )
            .ok_or("top plane")?;
            let wire_top = Wire::new(vec![
                OrientedEdge::forward(et[0].clone()),
                OrientedEdge::forward(et[1].clone()),
                OrientedEdge::forward(et[2].clone()),
                OrientedEdge::forward(et[3].clone()),
                OrientedEdge::forward(arc_t),
            ]);
            faces.push(Face::simple(FaceGeometry::Plane(p_top), wire_top));

            let shell = Shell::closed(faces);
            crate::validated_solid(shell)
        } else {
            Err("Edge index 0 is currently demonstrated for single edge fillet".to_string())
        }
    }

    /// 複数面の同時オフセット変形（Move / Offset Multiple Faces）
    pub fn offset_multiple_faces(solid: &Solid, offsets: &[(usize, f64)]) -> Result<Solid, String> {
        let mut current_solid = solid.clone();
        for &(face_idx, dist) in offsets {
            if dist.abs() > 1e-6 {
                current_solid = Self::push_pull_face(&current_solid, face_idx, dist)?;
            }
        }
        Ok(current_solid)
    }

    /// 3Dエッジ曲線の接線方向への延長（Extend Edge）
    pub fn extend_edge(
        edge: &Edge,
        extend_start_dist: f64,
        extend_end_dist: f64,
    ) -> Result<Edge, String> {
        let p_start = edge.start_vertex.point;
        let p_end = edge.end_vertex.point;
        let insp = Self::inspect_edge(edge);
        let tangent = insp.tangent;

        let new_p_start = p_start - tangent * extend_start_dist;
        let new_p_end = p_end + tangent * extend_end_dist;

        let new_v_start = Vertex::from_point(new_p_start);
        let new_v_end = Vertex::from_point(new_p_end);

        Edge::line_between(new_v_start, new_v_end)
    }
}
