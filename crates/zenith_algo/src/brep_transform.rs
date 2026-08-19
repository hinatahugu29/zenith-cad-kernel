use zenith_geom::{
    ControlPoint3, CoonsPatch3, GordonSurface3, NurbsCurve3, NurbsSurface3, PlaneSurface3,
    TriangularPatch3,
};
use zenith_math::{Transform3, Vec3};
use zenith_topo::{Edge, Face, FaceGeometry, OrientedEdge, Shell, Solid, Vertex, Wire};

pub struct BrepTransform;

impl BrepTransform {
    pub fn translate_solid(solid: &Solid, offset: Vec3) -> Solid {
        Solid::new(
            Self::translate_shell(&solid.outer_shell, offset),
            solid
                .inner_shells
                .iter()
                .map(|shell| Self::translate_shell(shell, offset))
                .collect(),
        )
    }

    pub fn translate_shell(shell: &Shell, offset: Vec3) -> Shell {
        Shell::new(
            shell
                .faces
                .iter()
                .map(|face| Self::translate_face(face, offset))
                .collect(),
            shell.is_closed,
        )
    }

    pub fn translate_face(face: &Face, offset: Vec3) -> Face {
        Face::new(
            translate_face_geometry(&face.geometry, offset),
            translate_wire(&face.outer_wire, offset),
            face.inner_wires
                .iter()
                .map(|wire| translate_wire(wire, offset))
                .collect(),
            face.orientation,
            face.tolerance,
        )
    }

    /// Applies a rigid transform (rotation plus translation) to a solid.
    ///
    /// Rigid transforms map every supported geometry class onto itself: NURBS
    /// control points move with the body, weights are unchanged, and a plane
    /// stays a plane. Non-rigid transforms are rejected because they would turn
    /// circles into ellipses and offsets into non-offsets, which the current
    /// geometry recognizers do not model.
    pub fn transform_solid(solid: &Solid, transform: &Transform3) -> Result<Solid, String> {
        ensure_rigid(transform)?;
        Ok(Solid::new(
            Self::transform_shell_unchecked(&solid.outer_shell, transform),
            solid
                .inner_shells
                .iter()
                .map(|shell| Self::transform_shell_unchecked(shell, transform))
                .collect(),
        ))
    }

    pub fn transform_shell(shell: &Shell, transform: &Transform3) -> Result<Shell, String> {
        ensure_rigid(transform)?;
        Ok(Self::transform_shell_unchecked(shell, transform))
    }

    pub fn transform_face(face: &Face, transform: &Transform3) -> Result<Face, String> {
        ensure_rigid(transform)?;
        Ok(transform_face_unchecked(face, transform))
    }

    fn transform_shell_unchecked(shell: &Shell, transform: &Transform3) -> Shell {
        Shell::new(
            shell
                .faces
                .iter()
                .map(|face| transform_face_unchecked(face, transform))
                .collect(),
            shell.is_closed,
        )
    }

    /// 単一エッジの平行移動（曲線と頂点をそのまま移す）
    pub fn translate_edge(edge: &Edge, offset: Vec3) -> Edge {
        translate_edge(edge, offset)
    }

    pub fn reverse_shell_orientation(shell: &Shell) -> Shell {
        Shell::new(
            shell
                .faces
                .iter()
                .map(Self::reverse_face_orientation)
                .collect(),
            shell.is_closed,
        )
    }

    pub fn reverse_face_orientation(face: &Face) -> Face {
        Face::new(
            face.geometry.clone(),
            reverse_wire_orientation(&face.outer_wire),
            face.inner_wires
                .iter()
                .map(reverse_wire_orientation)
                .collect(),
            face.orientation.reversed(),
            face.tolerance,
        )
    }
}

fn translate_face_geometry(geometry: &FaceGeometry, offset: Vec3) -> FaceGeometry {
    match geometry {
        FaceGeometry::Plane(plane) => FaceGeometry::Plane(PlaneSurface3 {
            origin: plane.origin + offset,
            u_axis: plane.u_axis,
            v_axis: plane.v_axis,
            normal: plane.normal,
        }),
        FaceGeometry::Nurbs(surface) => {
            FaceGeometry::Nurbs(translate_nurbs_surface(surface, offset))
        }
        FaceGeometry::Coons(patch) => FaceGeometry::Coons(translate_coons_patch(patch, offset)),
        FaceGeometry::Gordon(surface) => {
            FaceGeometry::Gordon(translate_gordon_surface(surface, offset))
        }
        FaceGeometry::Triangular(patch) => {
            FaceGeometry::Triangular(translate_triangular_patch(patch, offset))
        }
    }
}

fn translate_wire(wire: &Wire, offset: Vec3) -> Wire {
    Wire::new(
        wire.edges
            .iter()
            .map(|edge| OrientedEdge::new(translate_edge(&edge.edge, offset), edge.orientation))
            .collect(),
    )
}

fn reverse_wire_orientation(wire: &Wire) -> Wire {
    Wire::new(
        wire.edges
            .iter()
            .rev()
            .map(|edge| OrientedEdge::new(edge.edge.clone(), edge.orientation.reversed()))
            .collect(),
    )
}

fn translate_edge(edge: &Edge, offset: Vec3) -> Edge {
    Edge::new(
        translate_nurbs_curve(&edge.curve, offset),
        translate_vertex(&edge.start_vertex, offset),
        translate_vertex(&edge.end_vertex, offset),
        edge.tolerance,
    )
}

fn translate_vertex(vertex: &Vertex, offset: Vec3) -> Vertex {
    Vertex::new(vertex.point + offset, vertex.tolerance)
}

fn translate_nurbs_curve(curve: &NurbsCurve3, offset: Vec3) -> NurbsCurve3 {
    NurbsCurve3 {
        degree: curve.degree,
        control_points: curve
            .control_points
            .iter()
            .map(|cp| translate_control_point(*cp, offset))
            .collect(),
        knots: curve.knots.clone(),
    }
}

fn translate_nurbs_surface(surface: &NurbsSurface3, offset: Vec3) -> NurbsSurface3 {
    NurbsSurface3 {
        degree_u: surface.degree_u,
        degree_v: surface.degree_v,
        control_points: surface
            .control_points
            .iter()
            .map(|row| {
                row.iter()
                    .map(|cp| translate_control_point(*cp, offset))
                    .collect()
            })
            .collect(),
        knots_u: surface.knots_u.clone(),
        knots_v: surface.knots_v.clone(),
    }
}

fn translate_coons_patch(patch: &CoonsPatch3, offset: Vec3) -> CoonsPatch3 {
    CoonsPatch3 {
        c0: translate_nurbs_curve(&patch.c0, offset),
        c1: translate_nurbs_curve(&patch.c1, offset),
        d0: translate_nurbs_curve(&patch.d0, offset),
        d1: translate_nurbs_curve(&patch.d1, offset),
        p00: patch.p00 + offset,
        p10: patch.p10 + offset,
        p01: patch.p01 + offset,
        p11: patch.p11 + offset,
    }
}

fn translate_gordon_surface(surface: &GordonSurface3, offset: Vec3) -> GordonSurface3 {
    GordonSurface3 {
        u_params: surface.u_params.clone(),
        v_params: surface.v_params.clone(),
        u_curves: surface
            .u_curves
            .iter()
            .map(|curve| translate_nurbs_curve(curve, offset))
            .collect(),
        v_curves: surface
            .v_curves
            .iter()
            .map(|curve| translate_nurbs_curve(curve, offset))
            .collect(),
        intersection_points: surface
            .intersection_points
            .iter()
            .map(|row| row.iter().map(|point| *point + offset).collect())
            .collect(),
    }
}

fn translate_triangular_patch(patch: &TriangularPatch3, offset: Vec3) -> TriangularPatch3 {
    TriangularPatch3 {
        c0: translate_nurbs_curve(&patch.c0, offset),
        c1: translate_nurbs_curve(&patch.c1, offset),
        c2: translate_nurbs_curve(&patch.c2, offset),
        p0: patch.p0 + offset,
        p1: patch.p1 + offset,
        p2: patch.p2 + offset,
    }
}

fn translate_control_point(control_point: ControlPoint3, offset: Vec3) -> ControlPoint3 {
    ControlPoint3::new(control_point.point + offset, control_point.weight)
}

fn ensure_rigid(transform: &Transform3) -> Result<(), String> {
    const RIGID_TOLERANCE: f64 = 1e-9;

    let axes = [
        transform.transform_vector(&Vec3::new(1.0, 0.0, 0.0)),
        transform.transform_vector(&Vec3::new(0.0, 1.0, 0.0)),
        transform.transform_vector(&Vec3::new(0.0, 0.0, 1.0)),
    ];
    for (index, axis) in axes.iter().enumerate() {
        if (axis.norm() - 1.0).abs() > RIGID_TOLERANCE {
            return Err(format!(
                "B-Rep transform must be rigid; axis {index} is scaled by {}",
                axis.norm()
            ));
        }
        for other in axes.iter().skip(index + 1) {
            if axis.dot(other).abs() > RIGID_TOLERANCE {
                return Err("B-Rep transform must be rigid; axes are not orthogonal".to_string());
            }
        }
    }
    if axes[0].cross(&axes[1]).dot(&axes[2]) < 0.0 {
        return Err("B-Rep transform must preserve handedness".to_string());
    }

    Ok(())
}

fn transform_face_unchecked(face: &Face, transform: &Transform3) -> Face {
    Face::new(
        transform_face_geometry(&face.geometry, transform),
        transform_wire(&face.outer_wire, transform),
        face.inner_wires
            .iter()
            .map(|wire| transform_wire(wire, transform))
            .collect(),
        face.orientation,
        face.tolerance,
    )
}

fn transform_face_geometry(geometry: &FaceGeometry, transform: &Transform3) -> FaceGeometry {
    match geometry {
        FaceGeometry::Plane(plane) => {
            let origin = transform.transform_point(&plane.origin);
            let u_axis = transform.transform_vector(&plane.u_axis);
            let v_axis = transform.transform_vector(&plane.v_axis);
            FaceGeometry::Plane(PlaneSurface3::new(origin, u_axis, v_axis).unwrap_or(
                PlaneSurface3 {
                    origin,
                    u_axis,
                    v_axis,
                    normal: transform.transform_vector(&plane.normal),
                },
            ))
        }
        FaceGeometry::Nurbs(surface) => {
            FaceGeometry::Nurbs(transform_nurbs_surface(surface, transform))
        }
        FaceGeometry::Coons(patch) => FaceGeometry::Coons(CoonsPatch3 {
            c0: transform_nurbs_curve(&patch.c0, transform),
            c1: transform_nurbs_curve(&patch.c1, transform),
            d0: transform_nurbs_curve(&patch.d0, transform),
            d1: transform_nurbs_curve(&patch.d1, transform),
            p00: transform.transform_point(&patch.p00),
            p10: transform.transform_point(&patch.p10),
            p01: transform.transform_point(&patch.p01),
            p11: transform.transform_point(&patch.p11),
        }),
        FaceGeometry::Gordon(surface) => FaceGeometry::Gordon(GordonSurface3 {
            u_params: surface.u_params.clone(),
            v_params: surface.v_params.clone(),
            u_curves: surface
                .u_curves
                .iter()
                .map(|curve| transform_nurbs_curve(curve, transform))
                .collect(),
            v_curves: surface
                .v_curves
                .iter()
                .map(|curve| transform_nurbs_curve(curve, transform))
                .collect(),
            intersection_points: surface
                .intersection_points
                .iter()
                .map(|row| {
                    row.iter()
                        .map(|point| transform.transform_point(point))
                        .collect()
                })
                .collect(),
        }),
        FaceGeometry::Triangular(patch) => FaceGeometry::Triangular(TriangularPatch3 {
            c0: transform_nurbs_curve(&patch.c0, transform),
            c1: transform_nurbs_curve(&patch.c1, transform),
            c2: transform_nurbs_curve(&patch.c2, transform),
            p0: transform.transform_point(&patch.p0),
            p1: transform.transform_point(&patch.p1),
            p2: transform.transform_point(&patch.p2),
        }),
    }
}

fn transform_wire(wire: &Wire, transform: &Transform3) -> Wire {
    Wire::new(
        wire.edges
            .iter()
            .map(|edge| OrientedEdge::new(transform_edge(&edge.edge, transform), edge.orientation))
            .collect(),
    )
}

fn transform_edge(edge: &Edge, transform: &Transform3) -> Edge {
    Edge::new(
        transform_nurbs_curve(&edge.curve, transform),
        transform_vertex(&edge.start_vertex, transform),
        transform_vertex(&edge.end_vertex, transform),
        edge.tolerance,
    )
}

fn transform_vertex(vertex: &Vertex, transform: &Transform3) -> Vertex {
    Vertex::new(transform.transform_point(&vertex.point), vertex.tolerance)
}

fn transform_nurbs_curve(curve: &NurbsCurve3, transform: &Transform3) -> NurbsCurve3 {
    NurbsCurve3 {
        degree: curve.degree,
        control_points: curve
            .control_points
            .iter()
            .map(|control_point| transform_control_point(*control_point, transform))
            .collect(),
        knots: curve.knots.clone(),
    }
}

fn transform_nurbs_surface(surface: &NurbsSurface3, transform: &Transform3) -> NurbsSurface3 {
    NurbsSurface3 {
        degree_u: surface.degree_u,
        degree_v: surface.degree_v,
        control_points: surface
            .control_points
            .iter()
            .map(|row| {
                row.iter()
                    .map(|control_point| transform_control_point(*control_point, transform))
                    .collect()
            })
            .collect(),
        knots_u: surface.knots_u.clone(),
        knots_v: surface.knots_v.clone(),
    }
}

fn transform_control_point(control_point: ControlPoint3, transform: &Transform3) -> ControlPoint3 {
    ControlPoint3::new(
        transform.transform_point(&control_point.point),
        control_point.weight,
    )
}
