use zenith_geom::{
    ControlPoint3, CoonsPatch3, GordonSurface3, NurbsCurve3, NurbsSurface3, PlaneSurface3,
    TriangularPatch3,
};
use zenith_math::Vec3;
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
