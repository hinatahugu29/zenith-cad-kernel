use std::collections::BTreeMap;
use zenith_geom::{ControlPoint3, NurbsSurface3, PlaneSurface3};
use zenith_math::{Point3, Transform3, Vec3};
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
    /// 選択されたFaceを法線方向に distance だけ押し出す（Push/Pull）
    ///
    /// 既存の曲線・曲面は作り直さず変換する。動かす頂点をすべて含む面は剛体
    /// 移動し、片側だけ動く面はワイヤだけ差し替えて支持曲面を保つ。円柱側面の
    /// ような v 方向に線形な面は、動く側の制御点行だけを平行移動して伸ばす。
    /// 表現できない編集は直線で近似せず、明示的に失敗する。
    pub fn push_pull_face(
        solid: &Solid,
        face_index: usize,
        distance: f64,
    ) -> Result<Solid, String> {
        if distance.abs() < 1e-6 {
            return Ok(solid.clone());
        }

        let faces = &solid.outer_shell.faces;
        let target_face = faces.get(face_index).ok_or("Invalid face index")?;
        let inspection = Self::inspect_face(target_face)?;
        let offset = inspection.normal * distance;

        let mut moved_points = Vec::new();
        for edge in &target_face.outer_wire.edges {
            moved_points.push(edge.start_vertex().point);
            moved_points.push(edge.end_vertex().point);
        }
        let is_moved = |point: Point3| -> bool {
            moved_points
                .iter()
                .any(|moved| (point - *moved).norm() < 1e-5)
        };

        // 共有エッジは元のIDで1度だけ作り直し、全ての面で同じ実体を使う
        let mut rebuilt: BTreeMap<u64, Edge> = BTreeMap::new();
        for face in faces {
            for wire in std::iter::once(&face.outer_wire).chain(face.inner_wires.iter()) {
                for oriented in &wire.edges {
                    let edge = &oriented.edge;
                    if rebuilt.contains_key(&edge.id) {
                        continue;
                    }
                    rebuilt.insert(edge.id, push_pull_edge(edge, offset, &is_moved)?);
                }
            }
        }

        let rebuild_wire = |wire: &Wire| -> Wire {
            Wire::new(
                wire.edges
                    .iter()
                    .map(|oriented| {
                        OrientedEdge::new(rebuilt[&oriented.edge.id].clone(), oriented.orientation)
                    })
                    .collect(),
            )
        };

        let mut new_faces = Vec::with_capacity(faces.len());
        for face in faces {
            let mut moved_count = 0;
            let mut vertex_count = 0;
            for wire in std::iter::once(&face.outer_wire).chain(face.inner_wires.iter()) {
                for oriented in &wire.edges {
                    vertex_count += 1;
                    if is_moved(oriented.start_vertex().point) {
                        moved_count += 1;
                    }
                }
            }

            if moved_count == 0 {
                new_faces.push(face.clone());
                continue;
            }

            let geometry = if moved_count == vertex_count {
                // 面全体が動くので幾何ごと剛体移動する
                crate::BrepTransform::translate_face(face, offset).geometry
            } else {
                extended_face_geometry(face, offset, &is_moved)?
            };

            new_faces.push(Face::new(
                geometry,
                rebuild_wire(&face.outer_wire),
                face.inner_wires.iter().map(rebuild_wire).collect(),
                face.orientation,
                face.tolerance,
            ));
        }

        crate::validated_solid(Shell::closed(new_faces))
    }

    /// 選択されたFaceを回転軸まわりに angle_deg だけ傾斜（Taper / Draft）
    ///
    /// 回転は剛体変換なので、対象面と一緒に動く曲線はそのまま変換される。
    /// 片側だけが動く隣接面は境界から平面を張り直し、非平面になる編集や
    /// 曲面をまたぐ編集は近似せずに失敗する。
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
        let target_face = faces.get(face_index).ok_or("Invalid face index")?;
        let axis = axis_dir
            .try_normalize(1e-12)
            .ok_or("Taper axis direction is zero")?;

        let rotation = Transform3::from_translation(axis_origin.coords)
            .compose(&Transform3::from_axis_angle(&axis, angle_deg.to_radians()))
            .compose(&Transform3::from_translation(-axis_origin.coords));

        let mut moved_points = Vec::new();
        for edge in &target_face.outer_wire.edges {
            moved_points.push(edge.start_vertex().point);
            moved_points.push(edge.end_vertex().point);
        }
        let is_moved = |point: Point3| -> bool {
            moved_points
                .iter()
                .any(|moved| (point - *moved).norm() < 1e-5)
        };

        let mut rebuilt: BTreeMap<u64, Edge> = BTreeMap::new();
        for face in faces {
            for wire in std::iter::once(&face.outer_wire).chain(face.inner_wires.iter()) {
                for oriented in &wire.edges {
                    let edge = &oriented.edge;
                    if rebuilt.contains_key(&edge.id) {
                        continue;
                    }
                    let start_moved = is_moved(edge.start_vertex.point);
                    let end_moved = is_moved(edge.end_vertex.point);
                    let new_edge = match (start_moved, end_moved) {
                        (false, false) => edge.clone(),
                        (true, true) => crate::BrepTransform::transform_edge(edge, &rotation)?,
                        _ => {
                            if edge.curve.degree != 1 || edge.curve.control_points.len() != 2 {
                                return Err(
                                    "Taper would have to rebuild a curved side edge; extending the adjacent surfaces is not implemented"
                                        .to_string(),
                                );
                            }
                            let start = if start_moved {
                                rotation.transform_point(&edge.start_vertex.point)
                            } else {
                                edge.start_vertex.point
                            };
                            let end = if end_moved {
                                rotation.transform_point(&edge.end_vertex.point)
                            } else {
                                edge.end_vertex.point
                            };
                            Edge::line_between(Vertex::from_point(start), Vertex::from_point(end))?
                        }
                    };
                    rebuilt.insert(edge.id, new_edge);
                }
            }
        }

        let rebuild_wire = |wire: &Wire| -> Wire {
            Wire::new(
                wire.edges
                    .iter()
                    .map(|oriented| {
                        OrientedEdge::new(rebuilt[&oriented.edge.id].clone(), oriented.orientation)
                    })
                    .collect(),
            )
        };

        let mut new_faces = Vec::with_capacity(faces.len());
        for face in faces {
            let mut moved_count = 0;
            let mut vertex_count = 0;
            for wire in std::iter::once(&face.outer_wire).chain(face.inner_wires.iter()) {
                for oriented in &wire.edges {
                    vertex_count += 1;
                    if is_moved(oriented.start_vertex().point) {
                        moved_count += 1;
                    }
                }
            }

            if moved_count == 0 {
                new_faces.push(face.clone());
                continue;
            }

            let outer_wire = rebuild_wire(&face.outer_wire);
            let geometry = if moved_count == vertex_count {
                crate::BrepTransform::transform_face(face, &rotation)?.geometry
            } else {
                let FaceGeometry::Plane(plane) = &face.geometry else {
                    return Err(
                        "Taper across a curved adjacent face is not implemented".to_string()
                    );
                };
                FaceGeometry::Plane(refit_plane(&outer_wire.sample_points(4), plane)?)
            };

            new_faces.push(Face::new(
                geometry,
                outer_wire,
                face.inner_wires.iter().map(rebuild_wire).collect(),
                face.orientation,
                face.tolerance,
            ));
        }

        crate::validated_solid(Shell::closed(new_faces))
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

/// Rebuilds one edge for a push/pull, preserving its curve wherever the edit is
/// a rigid motion of the whole edge.
fn push_pull_edge(
    edge: &Edge,
    offset: Vec3,
    is_moved: &impl Fn(Point3) -> bool,
) -> Result<Edge, String> {
    let start_moved = is_moved(edge.start_vertex.point);
    let end_moved = is_moved(edge.end_vertex.point);

    match (start_moved, end_moved) {
        (false, false) => Ok(edge.clone()),
        (true, true) => Ok(crate::BrepTransform::translate_edge(edge, offset)),
        _ => {
            // 片端だけ動く辺は、直線なら伸縮で表せる。曲線の場合は隣接曲面を
            // 延長して再トリムする必要があり、直線で近似すると円弧が失われる。
            if edge.curve.degree != 1 || edge.curve.control_points.len() != 2 {
                return Err(
                    "Push-pull would have to rebuild a curved side edge; extending the adjacent surfaces is not implemented"
                        .to_string(),
                );
            }
            let start = if start_moved {
                edge.start_vertex.point + offset
            } else {
                edge.start_vertex.point
            };
            let end = if end_moved {
                edge.end_vertex.point + offset
            } else {
                edge.end_vertex.point
            };
            Edge::line_between(Vertex::from_point(start), Vertex::from_point(end))
        }
    }
}

/// Returns the geometry of a face only part of whose boundary moves.
///
/// A planar side wall keeps its plane as long as the push slides along it. A
/// surface that is linear in `v` - a cylinder or cone side patch - is extended
/// by translating the control row on the moving side, which keeps it exact.
fn extended_face_geometry(
    face: &Face,
    offset: Vec3,
    is_moved: &impl Fn(Point3) -> bool,
) -> Result<FaceGeometry, String> {
    match &face.geometry {
        FaceGeometry::Plane(plane) => {
            if offset.dot(&plane.normal).abs() > 1e-9 * offset.norm().max(1.0) {
                return Err(
                    "Push-pull direction is not tangent to an adjacent planar face".to_string(),
                );
            }
            Ok(FaceGeometry::Plane(*plane))
        }
        FaceGeometry::Nurbs(surface) => extend_ruled_surface(surface, offset, is_moved)
            .map(FaceGeometry::Nurbs)
            .ok_or_else(|| {
                "Push-pull across this curved face is not implemented; only surfaces linear in v can be extended"
                    .to_string()
            }),
        _ => Err("Push-pull across this face geometry is not implemented".to_string()),
    }
}

fn extend_ruled_surface(
    surface: &NurbsSurface3,
    offset: Vec3,
    is_moved: &impl Fn(Point3) -> bool,
) -> Option<NurbsSurface3> {
    if surface.degree_v != 1 || surface.control_points.iter().any(|row| row.len() != 2) {
        return None;
    }

    let ((u_min, u_max), (v_min, v_max)) = surface.param_range();
    let start_side_moves =
        is_moved(surface.evaluate(u_min, v_min)) && is_moved(surface.evaluate(u_max, v_min));
    let end_side_moves =
        is_moved(surface.evaluate(u_min, v_max)) && is_moved(surface.evaluate(u_max, v_max));

    let moving_column = match (start_side_moves, end_side_moves) {
        (true, false) => 0,
        (false, true) => 1,
        _ => return None,
    };

    let mut extended = surface.clone();
    for row in extended.control_points.iter_mut() {
        let control_point = row[moving_column];
        row[moving_column] = ControlPoint3::new(control_point.point + offset, control_point.weight);
    }

    Some(extended)
}

/// Rebuilds the plane of a face whose boundary moved.
///
/// The original plane is kept when the new boundary still lies on it. Otherwise
/// a plane is fitted through the boundary with Newell's method, oriented to
/// agree with the original normal so the face keeps its outward sense, and every
/// boundary point is checked against it - an edit that leaves the boundary
/// non-planar is refused rather than silently approximated.
fn refit_plane(points: &[Point3], original: &PlaneSurface3) -> Result<PlaneSurface3, String> {
    const PLANARITY_TOLERANCE: f64 = 1e-9;

    if points.len() < 3 {
        return Err("A planar face needs at least three boundary points".to_string());
    }
    let extent = points
        .iter()
        .map(|point| (point - points[0]).norm())
        .fold(0.0, f64::max)
        .max(1.0);
    let tolerance = PLANARITY_TOLERANCE * extent;

    if points
        .iter()
        .all(|point| (point - original.origin).dot(&original.normal).abs() <= tolerance)
    {
        return Ok(*original);
    }

    let mut normal = Vec3::zeros();
    for (index, current) in points.iter().enumerate() {
        let next = points[(index + 1) % points.len()];
        normal += Vec3::new(
            (current.y - next.y) * (current.z + next.z),
            (current.z - next.z) * (current.x + next.x),
            (current.x - next.x) * (current.y + next.y),
        );
    }
    let mut normal = normal
        .try_normalize(1e-12)
        .ok_or("Edited face boundary is degenerate")?;
    if normal.dot(&original.normal) < 0.0 {
        normal = -normal;
    }

    let origin = points[0];
    let in_plane = points
        .iter()
        .map(|point| point - origin)
        .find_map(|offset| (offset - normal * offset.dot(&normal)).try_normalize(1e-12))
        .ok_or("Edited face boundary is degenerate")?;
    let plane = PlaneSurface3::new(origin, in_plane, normal.cross(&in_plane))
        .ok_or("Failed to rebuild the edited face plane")?;

    if points
        .iter()
        .any(|point| (point - plane.origin).dot(&plane.normal).abs() > tolerance)
    {
        return Err("Taper left an adjacent face non-planar".to_string());
    }

    Ok(plane)
}
