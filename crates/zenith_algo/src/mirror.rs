use zenith_geom::{
    ControlPoint3, CoonsPatch3, GordonSurface3, NurbsCurve3, NurbsSurface3, PlaneSurface3,
    TriangularPatch3,
};
use zenith_math::{Point3, Tolerance, Vec3, Vec3Ext};
use zenith_topo::{Edge, Face, FaceGeometry, OrientedEdge, Shape, Shell, Solid, Vertex, Wire};

/// ミラー（鏡像反転）モデリングアルゴリズム
pub struct MirrorBuilder;

impl MirrorBuilder {
    /// 任意の対称平面（点 `plane_origin` と法線 `plane_normal`）に対してソリッドを鏡像反転複製
    pub fn mirror_solid(
        solid: &Solid,
        plane_origin: Point3,
        plane_normal: Vec3,
        tol: &Tolerance,
    ) -> Result<Solid, String> {
        let n = plane_normal
            .try_normalize_safe(1e-12)
            .ok_or("Mirror plane normal is zero")?;

        let mirrored_outer = Self::mirror_shell(&solid.outer_shell, plane_origin, n, tol)?;
        let mut mirrored_inners = Vec::with_capacity(solid.inner_shells.len());
        for inner in &solid.inner_shells {
            mirrored_inners.push(Self::mirror_shell(inner, plane_origin, n, tol)?);
        }

        let new_solid = Solid::new(mirrored_outer, mirrored_inners);
        let report = new_solid.outer_shell.validate_closed(tol);
        if !report.is_valid() {
            return Err(format!("Mirrored solid validation failed: {:?}", report.errors));
        }
        Ok(new_solid)
    }

    /// 原本ソリッドと鏡像反転ソリッドを複合Shape（Compound）として生成
    pub fn mirror_compound(
        solid: &Solid,
        plane_origin: Point3,
        plane_normal: Vec3,
        tol: &Tolerance,
    ) -> Result<Shape, String> {
        let mirrored = Self::mirror_solid(solid, plane_origin, plane_normal, tol)?;
        Ok(Shape::compound_solids(vec![solid.clone(), mirrored]))
    }

    /// シェルの鏡像反転
    pub fn mirror_shell(
        shell: &Shell,
        plane_origin: Point3,
        plane_normal: Vec3,
        tol: &Tolerance,
    ) -> Result<Shell, String> {
        let mirrored_faces = shell
            .faces
            .iter()
            .map(|f| Self::mirror_face(f, plane_origin, plane_normal, tol))
            .collect::<Result<Vec<Face>, String>>()?;

        let new_shell = if shell.is_closed {
            Shell::closed(mirrored_faces)
        } else {
            Shell::open(mirrored_faces)
        };
        Ok(new_shell)
    }

    /// 面の鏡像反転（幾何鏡像 ＋ ワイヤ巡回反転による右手系マニホールド維持）
    pub fn mirror_face(
        face: &Face,
        plane_origin: Point3,
        plane_normal: Vec3,
        _tol: &Tolerance,
    ) -> Result<Face, String> {
        let geom = Self::mirror_face_geometry(&face.geometry, plane_origin, plane_normal)?;
        let mirrored_outer_wire =
            Self::mirror_wire_reversed(&face.outer_wire, plane_origin, plane_normal);
        let mirrored_inner_wires = face
            .inner_wires
            .iter()
            .map(|w| Self::mirror_wire_reversed(w, plane_origin, plane_normal))
            .collect();

        Ok(Face::new(
            geom,
            mirrored_outer_wire,
            mirrored_inner_wires,
            face.orientation,
            face.tolerance,
        ))
    }

    /// ワイヤを鏡像変換し、さらにループの巡回順序とエッジ向きを反転
    fn mirror_wire_reversed(wire: &Wire, plane_origin: Point3, plane_normal: Vec3) -> Wire {
        let mirrored_edges: Vec<OrientedEdge> = wire
            .edges
            .iter()
            .rev()
            .map(|oe| {
                let m_edge = Self::mirror_edge(&oe.edge, plane_origin, plane_normal);
                OrientedEdge::new(m_edge, oe.orientation.reversed())
            })
            .collect();
        Wire::new(mirrored_edges)
    }

    /// エッジの鏡像変換
    pub fn mirror_edge(edge: &Edge, plane_origin: Point3, plane_normal: Vec3) -> Edge {
        let m_curve = Self::mirror_nurbs_curve(&edge.curve, plane_origin, plane_normal);
        let m_start = Self::mirror_vertex(&edge.start_vertex, plane_origin, plane_normal);
        let m_end = Self::mirror_vertex(&edge.end_vertex, plane_origin, plane_normal);
        Edge::new(m_curve, m_start, m_end, edge.tolerance)
    }

    /// 頂点の鏡像変換
    pub fn mirror_vertex(vertex: &Vertex, plane_origin: Point3, plane_normal: Vec3) -> Vertex {
        let p_mirrored = Self::mirror_point(vertex.point, plane_origin, plane_normal);
        Vertex::new(p_mirrored, vertex.tolerance)
    }

    /// 3D点の鏡像変換: P' = P - 2 ((P - P0) . n) n
    pub fn mirror_point(p: Point3, plane_origin: Point3, plane_normal: Vec3) -> Point3 {
        let diff = p - plane_origin;
        let dist = diff.dot(&plane_normal);
        p - plane_normal * (2.0 * dist)
    }

    /// 3Dベクトルの鏡像変換: v' = v - 2 (v . n) n
    pub fn mirror_vector(v: Vec3, plane_normal: Vec3) -> Vec3 {
        let dist = v.dot(&plane_normal);
        v - plane_normal * (2.0 * dist)
    }

    /// NURBS曲線の鏡像変換
    pub fn mirror_nurbs_curve(
        curve: &NurbsCurve3,
        plane_origin: Point3,
        plane_normal: Vec3,
    ) -> NurbsCurve3 {
        let cps = curve
            .control_points
            .iter()
            .map(|cp| {
                ControlPoint3::new(
                    Self::mirror_point(cp.point, plane_origin, plane_normal),
                    cp.weight,
                )
            })
            .collect();
        NurbsCurve3 {
            degree: curve.degree,
            control_points: cps,
            knots: curve.knots.clone(),
        }
    }

    /// 面支持幾何の鏡像変換
    fn mirror_face_geometry(
        geom: &FaceGeometry,
        plane_origin: Point3,
        plane_normal: Vec3,
    ) -> Result<FaceGeometry, String> {
        match geom {
            FaceGeometry::Plane(plane) => {
                let orig = Self::mirror_point(plane.origin, plane_origin, plane_normal);
                let u = Self::mirror_vector(plane.u_axis, plane_normal);
                let v = -Self::mirror_vector(plane.v_axis, plane_normal);
                let pl = PlaneSurface3::new(orig, u, v).ok_or("Failed mirrored plane")?;
                Ok(FaceGeometry::Plane(pl))
            }
            FaceGeometry::Nurbs(surface) => {
                // V方向を反転させて右手系法線を維持
                let cps: Vec<Vec<ControlPoint3>> = surface
                    .control_points
                    .iter()
                    .map(|row| {
                        row.iter()
                            .rev()
                            .map(|cp| {
                                ControlPoint3::new(
                                    Self::mirror_point(cp.point, plane_origin, plane_normal),
                                    cp.weight,
                                )
                            })
                            .collect()
                    })
                    .collect();
                let k_min = surface.knots_v.knots[0];
                let k_max = *surface.knots_v.knots.last().unwrap();
                let reversed_knots = surface
                    .knots_v
                    .knots
                    .iter()
                    .rev()
                    .map(|&k| k_min + k_max - k)
                    .collect();
                let knots_v = zenith_geom::KnotVector::new(reversed_knots);
                let s = NurbsSurface3 {
                    degree_u: surface.degree_u,
                    degree_v: surface.degree_v,
                    control_points: cps,
                    knots_u: surface.knots_u.clone(),
                    knots_v,
                };
                Ok(FaceGeometry::Nurbs(s))
            }


            FaceGeometry::Coons(patch) => {
                let c0 = Self::mirror_nurbs_curve(&patch.c0, plane_origin, plane_normal);
                let c1 = Self::mirror_nurbs_curve(&patch.c1, plane_origin, plane_normal);
                let d0 = Self::mirror_nurbs_curve(&patch.d0, plane_origin, plane_normal);
                let d1 = Self::mirror_nurbs_curve(&patch.d1, plane_origin, plane_normal);
                let p00 = Self::mirror_point(patch.p00, plane_origin, plane_normal);
                let p10 = Self::mirror_point(patch.p10, plane_origin, plane_normal);
                let p01 = Self::mirror_point(patch.p01, plane_origin, plane_normal);
                let p11 = Self::mirror_point(patch.p11, plane_origin, plane_normal);
                Ok(FaceGeometry::Coons(CoonsPatch3 {
                    c0,
                    c1,
                    d0,
                    d1,
                    p00,
                    p10,
                    p01,
                    p11,
                }))
            }
            FaceGeometry::Gordon(gordon) => {
                let u_curves = gordon
                    .u_curves
                    .iter()
                    .map(|c| Self::mirror_nurbs_curve(c, plane_origin, plane_normal))
                    .collect();
                let v_curves = gordon
                    .v_curves
                    .iter()
                    .map(|c| Self::mirror_nurbs_curve(c, plane_origin, plane_normal))
                    .collect();
                let isect = gordon
                    .intersection_points
                    .iter()
                    .map(|row| {
                        row.iter()
                            .map(|p| Self::mirror_point(*p, plane_origin, plane_normal))
                            .collect()
                    })
                    .collect();
                Ok(FaceGeometry::Gordon(GordonSurface3 {
                    u_params: gordon.u_params.clone(),
                    v_params: gordon.v_params.clone(),
                    u_curves,
                    v_curves,
                    intersection_points: isect,
                }))
            }

            FaceGeometry::Triangular(tri) => {
                let c0 = Self::mirror_nurbs_curve(&tri.c0, plane_origin, plane_normal);
                let c1 = Self::mirror_nurbs_curve(&tri.c1, plane_origin, plane_normal);
                let c2 = Self::mirror_nurbs_curve(&tri.c2, plane_origin, plane_normal);
                let p0 = Self::mirror_point(tri.p0, plane_origin, plane_normal);
                let p1 = Self::mirror_point(tri.p1, plane_origin, plane_normal);
                let p2 = Self::mirror_point(tri.p2, plane_origin, plane_normal);
                Ok(FaceGeometry::Triangular(TriangularPatch3 {
                    c0,
                    c1,
                    c2,
                    p0,
                    p1,
                    p2,
                }))
            }
        }
    }
}
