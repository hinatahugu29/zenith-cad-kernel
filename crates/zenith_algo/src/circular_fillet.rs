//! Exact first-stage fillet for the circular rim of a cylinder.
//!
//! This is deliberately narrower than a rolling-ball fillet.  It constructs
//! the one analytic case whose contact curves and blend surface are known in
//! closed form: the convex top rim of a right circular cylinder.  The side and
//! cap are retrimmed and four exact rational torus patches replace the sharp
//! edge.  General circular-edge recognition and in-place topology editing sit
//! above this geometric primitive; they must not approximate an unsupported
//! edge with this shape.

use std::f64::consts::{FRAC_1_SQRT_2, FRAC_PI_2, SQRT_2};

use zenith_geom::{ControlPoint3, KnotVector, NurbsCurve3, NurbsSurface3, PlaneSurface3};
use zenith_math::{Point3, Tolerance, Transform3, Vec3, Vec3Ext};
use zenith_topo::{
    Edge, Face, FaceGeometry, Orientation, OrientedEdge, Shell, Solid, Vertex, Wire,
};

use crate::{BlendableEdge, BrepTransform, EdgeBlendReport, FilletBuilder};

impl FilletBuilder {
    /// Builds a right circular cylinder whose convex top rim has an exact
    /// constant-radius fillet.
    ///
    /// The cylinder is aligned with +Z, starts at `z = 0`, and ends at
    /// `z = height`.  The blend is a quarter of a torus with major radius
    /// `radius - fillet_radius`.  Both circles are split into four rational
    /// quadratic patches, matching the regular topology used by the cylinder
    /// and torus primitives.
    ///
    /// This method is the geometry foundation for selected circular-edge
    /// editing.  It does not claim to fillet an arbitrary curved edge.
    pub fn fillet_cylinder_top_edge(
        radius: f64,
        height: f64,
        fillet_radius: f64,
        _tol: &Tolerance,
    ) -> Result<Solid, String> {
        if radius <= 1e-6 || height <= 1e-6 {
            return Err(format!(
                "Cylinder radius and height must be positive, got radius={radius}, height={height}"
            ));
        }
        if fillet_radius < 0.0 {
            return Err(format!(
                "Fillet radius must not be negative, got {fillet_radius}"
            ));
        }
        if fillet_radius <= 1e-6 {
            return crate::PrimitiveBuilder::make_cylinder(radius, height);
        }
        if fillet_radius >= radius || fillet_radius >= height {
            return Err(format!(
                "Top-rim fillet radius {fillet_radius} must be smaller than both cylinder radius {radius} and height {height}"
            ));
        }

        build_top_rounded_cylinder(radius, height, fillet_radius)
    }
}

/// Tries the deliberately narrow selected-edge entry point built on the exact
/// primitive above.  `Ok(None)` means that the edge/solid is not the supported
/// pure right-cylinder case and lets the general blender produce its existing
/// diagnostic.
pub(crate) fn try_fillet_cylinder_rim(
    solid: &Solid,
    edge_id: u64,
    fillet_radius: f64,
) -> Result<Option<(Solid, EdgeBlendReport)>, String> {
    let Some(site) = CircularCylinderRim::recognize(solid, edge_id) else {
        return Ok(None);
    };
    if fillet_radius >= site.radius || fillet_radius >= site.height {
        return Err(format!(
            "Circular cylinder rim fillet radius {fillet_radius} must be smaller than both cylinder radius {:.6} and height {:.6}",
            site.radius, site.height
        ));
    }

    let canonical = FilletBuilder::fillet_cylinder_top_edge(
        site.radius,
        site.height,
        fillet_radius,
        &Tolerance::default(),
    )?;
    let transform = site.canonical_to_world();
    let result = BrepTransform::transform_solid(&canonical, &transform)?;
    let removed = removed_volume(site.radius, fillet_radius);
    Ok(Some((
        result,
        EdgeBlendReport {
            dihedral_angle_deg: 90.0,
            // Selecting one patch edge propagates over the complete smooth
            // circular chain, matching normal CAD edge-filleting semantics.
            edge_length: std::f64::consts::TAU * site.radius,
            setback: fillet_radius,
            predicted_removed_volume: removed,
        },
    )))
}

pub(crate) fn circular_cylinder_blendable(solid: &Solid, edge_id: u64) -> Option<BlendableEdge> {
    let site = CircularCylinderRim::recognize(solid, edge_id)?;
    let max = site.radius.min(site.height) * 0.999;
    Some(BlendableEdge {
        edge_id,
        length: std::f64::consts::TAU * site.radius,
        dihedral_angle_deg: 90.0,
        max_fillet_radius: max,
        // Circular chamfers are not implemented.  Keep zero rather than
        // advertising a distance that chamfer_edge cannot honour.
        max_chamfer_distance: 0.0,
    })
}

fn removed_volume(radius: f64, fillet: f64) -> f64 {
    let major = radius - fillet;
    std::f64::consts::PI
        * (major * fillet * fillet * (2.0 - std::f64::consts::PI * 0.5) + fillet.powi(3) / 3.0)
}

struct CircularCylinderRim {
    top_center: Point3,
    outward_axis: Vec3,
    x_axis: Vec3,
    radius: f64,
    height: f64,
}

impl CircularCylinderRim {
    fn recognize(solid: &Solid, edge_id: u64) -> Option<Self> {
        if !solid.inner_shells.is_empty() {
            return None;
        }
        let faces = &solid.outer_shell.faces;
        let mut selected: Option<Edge> = None;
        for face in faces {
            for wire in std::iter::once(&face.outer_wire).chain(face.inner_wires.iter()) {
                for oriented in &wire.edges {
                    if oriented.edge.id == edge_id {
                        selected = Some(oriented.edge.clone());
                    }
                }
            }
        }
        let selected = selected?;
        // Rigid transforms currently rebuild the edge objects of each face, so
        // a geometrically shared edge can carry different IDs afterwards.
        // Selection still starts from the requested ID, but adjacency must be
        // recovered from the edge geometry rather than assuming object identity.
        let mut uses = Vec::new();
        for (face_index, face) in faces.iter().enumerate() {
            let matches = std::iter::once(&face.outer_wire)
                .chain(face.inner_wires.iter())
                .flat_map(|wire| wire.edges.iter())
                .any(|oriented| same_edge_geometry(&selected, &oriented.edge));
            if matches {
                uses.push(face_index);
            }
        }
        if uses.len() != 2 || uses[0] == uses[1] {
            return None;
        }

        let cap_index = uses
            .iter()
            .copied()
            .find(|index| matches!(faces[*index].geometry, FaceGeometry::Plane(_)))?;
        if !uses
            .iter()
            .copied()
            .any(|index| matches!(faces[index].geometry, FaceGeometry::Nurbs(_)))
        {
            return None;
        }
        let cap = &faces[cap_index];
        let FaceGeometry::Plane(cap_plane) = &cap.geometry else {
            return None;
        };

        let (top_center, radius, x_axis) = circle_from_edge(&selected)?;
        let plane_distance = (top_center - cap_plane.origin).dot(&cap_plane.normal).abs();
        let scale = radius.max(1.0);
        if plane_distance > 1e-7 * scale {
            return None;
        }
        let outward_axis = effective_plane_normal(cap).try_normalize_safe(1e-12)?;
        if selected
            .curve
            .evaluate_derivatives(0.5, 1)
            .get(1)
            .map(|tangent| tangent.dot(&outward_axis).abs() > 1e-7 * tangent.norm().max(1.0))
            .unwrap_or(true)
        {
            return None;
        }

        let inward = -outward_axis;
        let mut vertices = Vec::new();
        for face in faces {
            for wire in std::iter::once(&face.outer_wire).chain(face.inner_wires.iter()) {
                for oriented in &wire.edges {
                    vertices.push(oriented.edge.start_vertex.point);
                    vertices.push(oriented.edge.end_vertex.point);
                }
            }
        }
        let height = vertices
            .iter()
            .map(|point| (*point - top_center).dot(&inward))
            .fold(0.0_f64, f64::max);
        if height <= 1e-6 {
            return None;
        }

        let tolerance = 2e-6 * radius.max(height).max(1.0);
        if vertices.iter().any(|point| {
            let offset = *point - top_center;
            let axial = offset.dot(&inward);
            let radial = (offset - inward * axial).norm();
            axial < -tolerance || axial > height + tolerance || (radial - radius).abs() > tolerance
        }) {
            return None;
        }

        let mut planar_count = 0usize;
        for face in faces {
            match &face.geometry {
                FaceGeometry::Plane(_) => {
                    planar_count += 1;
                    let normal = effective_plane_normal(face);
                    if normal.dot(&outward_axis).abs() < 1.0 - 1e-7 {
                        return None;
                    }
                    let point = face.outer_wire.edges.first()?.start_vertex().point;
                    let axial = (point - top_center).dot(&inward);
                    if axial.abs() > tolerance && (axial - height).abs() > tolerance {
                        return None;
                    }
                }
                FaceGeometry::Nurbs(surface) => {
                    for iu in 0..=4 {
                        for iv in 0..=4 {
                            let point = surface.evaluate(iu as f64 / 4.0, iv as f64 / 4.0);
                            let offset = point - top_center;
                            let axial = offset.dot(&inward);
                            let radial = (offset - inward * axial).norm();
                            if axial < -tolerance
                                || axial > height + tolerance
                                || (radial - radius).abs() > tolerance
                            {
                                return None;
                            }
                        }
                    }
                }
                _ => return None,
            }
        }
        if planar_count != 2 {
            return None;
        }

        Some(Self {
            top_center,
            outward_axis,
            x_axis,
            radius,
            height,
        })
    }

    fn canonical_to_world(&self) -> Transform3 {
        let z = self.outward_axis;
        let x = self.x_axis;
        let y = z.cross(&x).normalize();
        let base = self.top_center - z * self.height;
        let mut matrix = nalgebra::Matrix4::identity();
        for row in 0..3 {
            matrix[(row, 0)] = x[row];
            matrix[(row, 1)] = y[row];
            matrix[(row, 2)] = z[row];
            matrix[(row, 3)] = base[row];
        }
        Transform3 { matrix }
    }
}

fn effective_plane_normal(face: &Face) -> Vec3 {
    let FaceGeometry::Plane(plane) = &face.geometry else {
        unreachable!("called only for a planar face")
    };
    if face.orientation == Orientation::Forward {
        plane.normal
    } else {
        -plane.normal
    }
}

fn circle_from_edge(edge: &Edge) -> Option<(Point3, f64, Vec3)> {
    let a = edge.curve.evaluate(0.0);
    let b = edge.curve.evaluate(1.0 / 3.0);
    let c = edge.curve.evaluate(2.0 / 3.0);
    let u = b - a;
    let v = c - a;
    let cross = u.cross(&v);
    let denominator = 2.0 * cross.norm_squared();
    if denominator <= 1e-20 {
        return None;
    }
    let center =
        a + (u.norm_squared() * v.cross(&cross) + v.norm_squared() * cross.cross(&u)) / denominator;
    let radius = (a - center).norm();
    if radius <= 1e-6 {
        return None;
    }
    let normal = cross.try_normalize_safe(1e-12)?;
    let tolerance = 2e-7 * radius.max(1.0);
    for step in 0..=24 {
        let point = edge.curve.evaluate(step as f64 / 24.0);
        let offset = point - center;
        if (offset.norm() - radius).abs() > tolerance || offset.dot(&normal).abs() > tolerance {
            return None;
        }
    }
    Some((center, radius, (a - center) / radius))
}

fn same_edge_geometry(a: &Edge, b: &Edge) -> bool {
    let scale = (a.start_vertex.point - a.end_vertex.point)
        .norm()
        .max((b.start_vertex.point - b.end_vertex.point).norm())
        .max(1.0);
    let tolerance = 2e-7 * scale;
    let direct = (0..=8).all(|step| {
        let t = step as f64 / 8.0;
        (a.curve.evaluate(t) - b.curve.evaluate(t)).norm() <= tolerance
    });
    let reversed = (0..=8).all(|step| {
        let t = step as f64 / 8.0;
        (a.curve.evaluate(t) - b.curve.evaluate(1.0 - t)).norm() <= tolerance
    });
    direct || reversed
}

fn build_top_rounded_cylinder(radius: f64, height: f64, fillet: f64) -> Result<Solid, String> {
    let join_z = height - fillet;
    let top_radius = radius - fillet;
    let theta = |index: usize| FRAC_PI_2 * (index % 4) as f64;
    let point = |radial: f64, z: f64, angle: f64| {
        Point3::new(radial * angle.cos(), radial * angle.sin(), z)
    };
    let angular_tangent_intersection = |radial: f64, z: f64, angle: f64| {
        let middle = angle + FRAC_PI_2 * 0.5;
        Point3::new(
            SQRT_2 * radial * middle.cos(),
            SQRT_2 * radial * middle.sin(),
            z,
        )
    };

    let bottom: Vec<Vertex> = (0..4)
        .map(|i| Vertex::from_point(point(radius, 0.0, theta(i))))
        .collect();
    let join: Vec<Vertex> = (0..4)
        .map(|i| Vertex::from_point(point(radius, join_z, theta(i))))
        .collect();
    let top: Vec<Vertex> = (0..4)
        .map(|i| Vertex::from_point(point(top_radius, height, theta(i))))
        .collect();

    let circular_arc =
        |radial: f64, z: f64, index: usize, start: Vertex, end: Vertex| -> Result<Edge, String> {
            let curve = NurbsCurve3::new(
                2,
                vec![
                    ControlPoint3::unweighted(start.point),
                    ControlPoint3::new(
                        angular_tangent_intersection(radial, z, theta(index)),
                        FRAC_1_SQRT_2,
                    ),
                    ControlPoint3::unweighted(end.point),
                ],
                KnotVector::clamped_uniform(3, 2),
            )?;
            Ok(Edge::new(curve, start, end, 1e-6))
        };

    let mut bottom_arcs = Vec::with_capacity(4);
    let mut join_arcs = Vec::with_capacity(4);
    let mut top_arcs = Vec::with_capacity(4);
    let mut vertical_edges = Vec::with_capacity(4);
    let mut profile_edges = Vec::with_capacity(4);
    for i in 0..4 {
        let next = (i + 1) % 4;
        bottom_arcs.push(circular_arc(
            radius,
            0.0,
            i,
            bottom[i].clone(),
            bottom[next].clone(),
        )?);
        join_arcs.push(circular_arc(
            radius,
            join_z,
            i,
            join[i].clone(),
            join[next].clone(),
        )?);
        top_arcs.push(circular_arc(
            top_radius,
            height,
            i,
            top[i].clone(),
            top[next].clone(),
        )?);
        vertical_edges.push(Edge::line_between(bottom[i].clone(), join[i].clone())?);

        // The two tangents of the quarter-circle profile meet at (R, H).
        let profile = NurbsCurve3::new(
            2,
            vec![
                ControlPoint3::unweighted(join[i].point),
                ControlPoint3::new(point(radius, height, theta(i)), FRAC_1_SQRT_2),
                ControlPoint3::unweighted(top[i].point),
            ],
            KnotVector::clamped_uniform(3, 2),
        )?;
        profile_edges.push(Edge::new(profile, join[i].clone(), top[i].clone(), 1e-6));
    }

    let mut faces = Vec::with_capacity(10);

    // The remaining cylindrical side, split into four regular patches.
    for i in 0..4 {
        let next = (i + 1) % 4;
        let surface = NurbsSurface3::new(
            2,
            1,
            vec![
                vec![
                    ControlPoint3::unweighted(bottom[i].point),
                    ControlPoint3::unweighted(join[i].point),
                ],
                vec![
                    ControlPoint3::new(
                        angular_tangent_intersection(radius, 0.0, theta(i)),
                        FRAC_1_SQRT_2,
                    ),
                    ControlPoint3::new(
                        angular_tangent_intersection(radius, join_z, theta(i)),
                        FRAC_1_SQRT_2,
                    ),
                ],
                vec![
                    ControlPoint3::unweighted(bottom[next].point),
                    ControlPoint3::unweighted(join[next].point),
                ],
            ],
            KnotVector::clamped_uniform(3, 2),
            KnotVector::clamped_uniform(2, 1),
        )?;
        faces.push(Face::simple(
            FaceGeometry::Nurbs(surface),
            Wire::new(vec![
                OrientedEdge::forward(bottom_arcs[i].clone()),
                OrientedEdge::forward(vertical_edges[next].clone()),
                OrientedEdge::reversed(join_arcs[i].clone()),
                OrientedEdge::reversed(vertical_edges[i].clone()),
            ]),
        ));
    }

    // Four exact rational torus patches.  The profile rows are reversed for
    // an outward du x dv normal, as in PrimitiveBuilder::make_torus_patches.
    for i in 0..4 {
        let mut rows = Vec::with_capacity(3);
        for (radial, z, profile_weight) in [
            (radius, join_z, 1.0),
            (radius, height, FRAC_1_SQRT_2),
            (top_radius, height, 1.0),
        ] {
            rows.push(vec![
                ControlPoint3::new(point(radial, z, theta(i)), profile_weight),
                ControlPoint3::new(
                    angular_tangent_intersection(radial, z, theta(i)),
                    profile_weight * FRAC_1_SQRT_2,
                ),
                ControlPoint3::new(point(radial, z, theta(i + 1)), profile_weight),
            ]);
        }
        rows.reverse();
        let surface = NurbsSurface3::new(
            2,
            2,
            rows,
            KnotVector::clamped_uniform(3, 2),
            KnotVector::clamped_uniform(3, 2),
        )?;
        faces.push(Face::simple(
            FaceGeometry::Nurbs(surface),
            Wire::new(vec![
                OrientedEdge::forward(join_arcs[i].clone()),
                OrientedEdge::forward(profile_edges[(i + 1) % 4].clone()),
                OrientedEdge::reversed(top_arcs[i].clone()),
                OrientedEdge::reversed(profile_edges[i].clone()),
            ]),
        ));
    }

    let bottom_plane = PlaneSurface3::new(
        Point3::new(0.0, 0.0, 0.0),
        Vec3::new(0.0, 1.0, 0.0),
        Vec3::new(1.0, 0.0, 0.0),
    )
    .ok_or("Bottom cap plane is degenerate")?;
    faces.push(Face::simple(
        FaceGeometry::Plane(bottom_plane),
        Wire::new(vec![
            OrientedEdge::reversed(bottom_arcs[3].clone()),
            OrientedEdge::reversed(bottom_arcs[2].clone()),
            OrientedEdge::reversed(bottom_arcs[1].clone()),
            OrientedEdge::reversed(bottom_arcs[0].clone()),
        ]),
    ));

    let top_plane = PlaneSurface3::new(
        Point3::new(0.0, 0.0, height),
        Vec3::new(1.0, 0.0, 0.0),
        Vec3::new(0.0, 1.0, 0.0),
    )
    .ok_or("Top cap plane is degenerate")?;
    faces.push(Face::simple(
        FaceGeometry::Plane(top_plane),
        Wire::new(vec![
            OrientedEdge::forward(top_arcs[0].clone()),
            OrientedEdge::forward(top_arcs[1].clone()),
            OrientedEdge::forward(top_arcs[2].clone()),
            OrientedEdge::forward(top_arcs[3].clone()),
        ]),
    ));

    crate::validated_solid(Shell::closed(faces))
}
