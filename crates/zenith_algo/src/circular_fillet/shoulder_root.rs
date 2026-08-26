//! Exact local blending of a circular boss or stepped-shaft root.
//!
//! The supported site is deliberately strict: an annular planar shoulder has
//! a circular inner wire, an unbroken right-cylindrical or conical side runs
//! outward from that wire, and the side ends at another parallel planar face. The operation
//! keeps every unrelated face, shortens only that side, enlarges
//! only the selected shoulder inner wire, and inserts four exact rational
//! quarter-torus or conical patches.

use std::collections::BTreeSet;
use std::f64::consts::{FRAC_1_SQRT_2, FRAC_PI_2, PI, SQRT_2, TAU};

use zenith_geom::{ControlPoint3, KnotVector, NurbsCurve3, NurbsSurface3};
use zenith_math::{Point3, Tolerance, Vec3, Vec3Ext};
use zenith_topo::{Edge, Face, FaceGeometry, OrientedEdge, Shell, Solid, Vertex, Wire};

use crate::{BlendableEdge, EdgeBlendReport};

pub(crate) fn try_fillet_shoulder_root(
    solid: &Solid,
    edge_id: u64,
    radius: f64,
) -> Result<Option<(Solid, EdgeBlendReport)>, String> {
    let Some(site) = ShoulderRoot::recognize(solid, edge_id) else {
        return Ok(None);
    };
    if !(radius > 1e-6) || !radius.is_finite() {
        return Err(format!(
            "Shoulder-root fillet radius must be finite and larger than 1e-6, got {radius}"
        ));
    }
    let margin = 1e-6 * site.max_radius.max(site.root_radius).max(1.0);
    if radius >= site.max_radius - margin {
        return Err(format!(
            "Shoulder-root fillet radius {radius} must be smaller than the available setback {:.6}",
            site.max_radius
        ));
    }
    let result = site.apply(solid, ShoulderRootBlend::Fillet(radius))?;
    Ok(Some((
        result,
        EdgeBlendReport {
            dihedral_angle_deg: 270.0 + site.slope.atan().to_degrees(),
            edge_length: TAU * site.root_radius,
            setback: radius,
            // This concave blend adds material; removed volume is signed.
            predicted_removed_volume: -site.fillet_added_volume(radius),
        },
    )))
}

pub(crate) fn try_chamfer_shoulder_root(
    solid: &Solid,
    edge_id: u64,
    distance: f64,
) -> Result<Option<(Solid, EdgeBlendReport)>, String> {
    let Some(site) = ShoulderRoot::recognize(solid, edge_id) else {
        return Ok(None);
    };
    if !(distance > 1e-6) || !distance.is_finite() {
        return Err(format!(
            "Shoulder-root chamfer distance must be finite and larger than 1e-6, got {distance}"
        ));
    }
    let margin = 1e-6 * site.max_radius.max(site.root_radius).max(1.0);
    if distance >= site.max_radius - margin {
        return Err(format!(
            "Shoulder-root chamfer distance {distance} must be smaller than the available setback {:.6}",
            site.max_radius
        ));
    }
    let result = site.apply(solid, ShoulderRootBlend::Chamfer(distance))?;
    Ok(Some((
        result,
        EdgeBlendReport {
            dihedral_angle_deg: 270.0,
            edge_length: TAU * site.root_radius,
            setback: distance,
            // This concave chamfer adds material; removed volume is signed.
            predicted_removed_volume: -chamfer_added_volume(site.root_radius, distance),
        },
    )))
}

pub(crate) fn shoulder_root_blendable(solid: &Solid, edge_id: u64) -> Option<BlendableEdge> {
    let site = ShoulderRoot::recognize(solid, edge_id)?;
    Some(BlendableEdge {
        edge_id,
        length: TAU * site.root_radius,
        dihedral_angle_deg: 270.0 + site.slope.atan().to_degrees(),
        max_fillet_radius: site.max_radius * 0.999,
        max_chamfer_distance: if site.slope.abs() <= 1e-10 {
            site.max_radius * 0.999
        } else {
            0.0
        },
    })
}

fn chamfer_added_volume(shaft_radius: f64, distance: f64) -> f64 {
    PI * distance * distance * (shaft_radius + distance / 3.0)
}

#[derive(Clone, Copy)]
enum ShoulderRootBlend {
    Fillet(f64),
    Chamfer(f64),
}

struct ShoulderRoot {
    shoulder_index: usize,
    upper_planar_index: usize,
    side_indices: BTreeSet<usize>,
    shoulder_inner_index: usize,
    center: Point3,
    outward_axis: Vec3,
    x_axis: Vec3,
    root_radius: f64,
    top_radius: f64,
    slope: f64,
    height: f64,
    max_radius: f64,
}

impl ShoulderRoot {
    fn recognize(solid: &Solid, edge_id: u64) -> Option<Self> {
        if !solid.inner_shells.is_empty() {
            return None;
        }
        let faces = &solid.outer_shell.faces;
        let selected = faces
            .iter()
            .flat_map(all_wires)
            .flat_map(|wire| wire.edges.iter())
            .find(|oriented| oriented.edge.id == edge_id)?
            .edge
            .clone();

        let (shoulder_index, shoulder_inner_index) =
            faces.iter().enumerate().find_map(|(face_index, face)| {
                if !matches!(face.geometry, FaceGeometry::Plane(_)) {
                    return None;
                }
                face.inner_wires
                    .iter()
                    .enumerate()
                    .find_map(|(wire_index, wire)| {
                        wire.edges
                            .iter()
                            .any(|edge| super::same_edge_geometry(&selected, &edge.edge))
                            .then_some((face_index, wire_index))
                    })
            })?;
        let shoulder = &faces[shoulder_index];
        let root_wire = &shoulder.inner_wires[shoulder_inner_index];
        if !matches!(root_wire.edges.len(), 1 | 4) {
            return None;
        }

        let (center, root_radius, x_axis) = super::circle_from_edge(&selected)?;
        let outward_axis = super::effective_plane_normal(shoulder).try_normalize_safe(1e-12)?;
        let tolerance = 2e-6 * root_radius.max(1.0);
        if !wire_is_circle(root_wire, center, root_radius, outward_axis, tolerance) {
            return None;
        }

        let mut side_indices = BTreeSet::new();
        for root_edge in &root_wire.edges {
            let users: Vec<usize> = faces
                .iter()
                .enumerate()
                .filter(|(_, face)| {
                    all_wires(face).any(|wire| {
                        wire.edges.iter().any(|candidate| {
                            super::same_edge_geometry(&root_edge.edge, &candidate.edge)
                        })
                    })
                })
                .map(|(index, _)| index)
                .collect();
            if users.len() != 2 || !users.contains(&shoulder_index) {
                return None;
            }
            let side = *users.iter().find(|index| **index != shoulder_index)?;
            if !matches!(faces[side].geometry, FaceGeometry::Nurbs(_)) {
                return None;
            }
            side_indices.insert(side);
        }
        if side_indices.len() != root_wire.edges.len() {
            return None;
        }

        let mut height: Option<f64> = None;
        let mut top_radius: Option<f64> = None;
        let mut upper_edges = Vec::with_capacity(root_wire.edges.len());
        for side_index in &side_indices {
            let face = &faces[*side_index];
            if face.outer_wire.edges.len() != 4 || !face.inner_wires.is_empty() {
                return None;
            }
            let upper = face.outer_wire.edges.iter().find(|candidate| {
                if root_wire
                    .edges
                    .iter()
                    .any(|root| super::same_edge_geometry(&root.edge, &candidate.edge))
                {
                    return false;
                }
                let Some((other_center, other_radius, _)) =
                    super::circle_from_edge(&candidate.edge)
                else {
                    return false;
                };
                let axial = (other_center - center).dot(&outward_axis);
                let sideways = other_center - center - outward_axis * axial;
                axial > tolerance && sideways.norm() <= tolerance && other_radius > tolerance
            })?;
            let (other_center, other_radius, _) = super::circle_from_edge(&upper.edge)?;
            let this_height = (other_center - center).dot(&outward_axis);
            if let Some(expected) = height {
                if (this_height - expected).abs() > tolerance {
                    return None;
                }
            } else {
                height = Some(this_height);
            }
            if let Some(expected) = top_radius {
                if (other_radius - expected).abs() > tolerance {
                    return None;
                }
            } else {
                top_radius = Some(other_radius);
            }
            upper_edges.push(upper.edge.clone());
        }
        let height = height?;
        let top_radius = top_radius?;
        if height <= 1e-6 {
            return None;
        }

        let slope = (top_radius - root_radius) / height;
        let surface_tolerance = 3e-6 * root_radius.max(top_radius).max(height).max(1.0);
        for side_index in &side_indices {
            let FaceGeometry::Nurbs(surface) = &faces[*side_index].geometry else {
                return None;
            };
            let ((u_min, u_max), (v_min, v_max)) = surface.param_range();
            for iu in 0..=6 {
                for iv in 0..=6 {
                    let u = u_min + (u_max - u_min) * iu as f64 / 6.0;
                    let v = v_min + (v_max - v_min) * iv as f64 / 6.0;
                    let point = surface.evaluate(u, v);
                    let offset = point - center;
                    let axial = offset.dot(&outward_axis);
                    let radial = (offset - outward_axis * axial).norm();
                    // A trimmed B-Rep face may retain a supporting cylinder
                    // whose parameter domain extends beyond its boundary
                    // wires (the stepped-shaft builder intentionally overlaps
                    // adjacent cylinders before Boolean union). The wires
                    // above establish the actual axial interval; here only
                    // prove that the support is the same right cylinder or cone.
                    let expected = root_radius + slope * axial;
                    if (radial - expected).abs() > surface_tolerance {
                        return None;
                    }
                }
            }
        }

        let upper_center = center + outward_axis * height;
        let upper_planar_index = faces.iter().enumerate().find_map(|(index, face)| {
            if index == shoulder_index || !matches!(face.geometry, FaceGeometry::Plane(_)) {
                return None;
            }
            if super::effective_plane_normal(face).dot(&outward_axis) < 1.0 - 1e-7 {
                return None;
            }
            (matches!(face.outer_wire.edges.len(), 1 | 4)
                && wire_is_circle(
                    &face.outer_wire,
                    upper_center,
                    top_radius,
                    outward_axis,
                    tolerance,
                )
                && upper_edges.iter().all(|edge| {
                    face.outer_wire
                        .edges
                        .iter()
                        .any(|candidate| super::same_edge_geometry(edge, &candidate.edge))
                }))
            .then_some(index)
        })?;

        let clearance =
            radial_clearance(shoulder, shoulder_inner_index, center, outward_axis)? - root_radius;
        let slope_norm = slope.hypot(1.0);
        let radial_factor = slope_norm + slope;
        let axial_factor = 1.0 + slope / slope_norm;
        let max_radius = (clearance / radial_factor).min(height / axial_factor);
        if max_radius <= 1e-6 {
            return None;
        }

        Some(Self {
            shoulder_index,
            upper_planar_index,
            side_indices,
            shoulder_inner_index,
            center,
            outward_axis,
            x_axis,
            root_radius,
            top_radius,
            slope,
            height,
            max_radius,
        })
    }

    fn fillet_added_volume(&self, fillet: f64) -> f64 {
        let norm = self.slope.hypot(1.0);
        let centre_radius = self.root_radius + fillet * (norm + self.slope);
        let contact_z = fillet * (1.0 + self.slope / norm);
        let arc_primitive = |z: f64| {
            let shifted = z - fillet;
            let root = (fillet * fillet - shifted * shifted).max(0.0).sqrt();
            let integral_root = 0.5
                * (shifted * root + fillet * fillet * (shifted / fillet).clamp(-1.0, 1.0).asin());
            centre_radius * centre_radius * z - 2.0 * centre_radius * integral_root
                + fillet * fillet * z
                - shifted.powi(3) / 3.0
        };
        let cone_primitive = |z: f64| {
            self.root_radius * self.root_radius * z
                + self.root_radius * self.slope * z * z
                + self.slope * self.slope * z.powi(3) / 3.0
        };
        PI * ((arc_primitive(contact_z) - arc_primitive(0.0))
            - (cone_primitive(contact_z) - cone_primitive(0.0)))
    }

    fn apply(&self, solid: &Solid, blend: ShoulderRootBlend) -> Result<Solid, String> {
        let axis = self.outward_axis;
        let x = (self.x_axis - axis * self.x_axis.dot(&axis))
            .try_normalize_safe(1e-12)
            .ok_or("Shoulder-root radial axis is degenerate")?;
        let y = axis.cross(&x).normalize();
        let theta = |index: usize| FRAC_PI_2 * (index % 4) as f64;
        let point = |radial: f64, height: f64, angle: f64| {
            self.center + x * (radial * angle.cos()) + y * (radial * angle.sin()) + axis * height
        };
        let tangent = |radial: f64, height: f64, index: usize| {
            let middle = theta(index) + FRAC_PI_2 * 0.5;
            point(SQRT_2 * radial, height, middle)
        };

        let (base_radius, join_radius, join_height, profile_spec) = match blend {
            ShoulderRootBlend::Fillet(fillet) => {
                let norm = self.slope.hypot(1.0);
                let centre_radius = self.root_radius + fillet * (norm + self.slope);
                let join_radius = centre_radius - fillet / norm;
                let join_height = fillet * (1.0 + self.slope / norm);
                let side_angle = PI - self.slope.atan();
                let end_angle = 3.0 * FRAC_PI_2;
                let sweep = end_angle - side_angle;
                if !(sweep > 1e-8 && sweep < PI - 1e-8) {
                    return Err("Shoulder-root cone has no regular circular fillet arc".into());
                }
                let weight = (sweep * 0.5).cos();
                let middle = (side_angle + end_angle) * 0.5;
                let control_radius = centre_radius + fillet * middle.cos() / weight;
                let control_height = fillet + fillet * middle.sin() / weight;
                (
                    centre_radius,
                    join_radius,
                    join_height,
                    Some((control_radius, control_height, weight)),
                )
            }
            ShoulderRootBlend::Chamfer(distance) => {
                if self.slope.abs() > 1e-10 {
                    return Err("Equal-setback chamfer for a non-right-angle shoulder root is not yet supported".into());
                }
                (
                    self.root_radius + distance,
                    self.root_radius,
                    distance,
                    None,
                )
            }
        };
        let base: Vec<Vertex> = (0..4)
            .map(|i| Vertex::from_point(point(base_radius, 0.0, theta(i))))
            .collect();
        let join: Vec<Vertex> = (0..4)
            .map(|i| Vertex::from_point(point(join_radius, join_height, theta(i))))
            .collect();
        let top: Vec<Vertex> = (0..4)
            .map(|i| Vertex::from_point(point(self.top_radius, self.height, theta(i))))
            .collect();

        let arc = |radial: f64,
                   height: f64,
                   index: usize,
                   start: Vertex,
                   end: Vertex|
         -> Result<Edge, String> {
            let curve = NurbsCurve3::new(
                2,
                vec![
                    ControlPoint3::unweighted(start.point),
                    ControlPoint3::new(tangent(radial, height, index), FRAC_1_SQRT_2),
                    ControlPoint3::unweighted(end.point),
                ],
                KnotVector::clamped_uniform(3, 2),
            )?;
            Ok(Edge::new(curve, start, end, 1e-6))
        };

        let mut base_arcs = Vec::with_capacity(4);
        let mut join_arcs = Vec::with_capacity(4);
        let mut top_arcs = Vec::with_capacity(4);
        let mut vertical = Vec::with_capacity(4);
        let mut profiles = Vec::with_capacity(4);
        for i in 0..4 {
            let next = (i + 1) % 4;
            base_arcs.push(arc(
                base_radius,
                0.0,
                i,
                base[i].clone(),
                base[next].clone(),
            )?);
            join_arcs.push(arc(
                join_radius,
                join_height,
                i,
                join[i].clone(),
                join[next].clone(),
            )?);
            top_arcs.push(arc(
                self.top_radius,
                self.height,
                i,
                top[i].clone(),
                top[next].clone(),
            )?);
            vertical.push(Edge::line_between(join[i].clone(), top[i].clone())?);
            profiles.push(match blend {
                ShoulderRootBlend::Fillet(_) => {
                    let (control_radius, control_height, weight) = profile_spec.unwrap();
                    let profile = NurbsCurve3::new(
                        2,
                        vec![
                            ControlPoint3::unweighted(join[i].point),
                            ControlPoint3::new(
                                point(control_radius, control_height, theta(i)),
                                weight,
                            ),
                            ControlPoint3::unweighted(base[i].point),
                        ],
                        KnotVector::clamped_uniform(3, 2),
                    )?;
                    Edge::new(profile, join[i].clone(), base[i].clone(), 1e-6)
                }
                ShoulderRootBlend::Chamfer(_) => {
                    Edge::line_between(join[i].clone(), base[i].clone())?
                }
            });
        }

        let mut replacement = Vec::with_capacity(8);
        for i in 0..4 {
            let next = (i + 1) % 4;
            let surface = NurbsSurface3::new(
                2,
                1,
                vec![
                    vec![
                        ControlPoint3::unweighted(join[i].point),
                        ControlPoint3::unweighted(top[i].point),
                    ],
                    vec![
                        ControlPoint3::new(tangent(join_radius, join_height, i), FRAC_1_SQRT_2),
                        ControlPoint3::new(tangent(self.top_radius, self.height, i), FRAC_1_SQRT_2),
                    ],
                    vec![
                        ControlPoint3::unweighted(join[next].point),
                        ControlPoint3::unweighted(top[next].point),
                    ],
                ],
                KnotVector::clamped_uniform(3, 2),
                KnotVector::clamped_uniform(2, 1),
            )?;
            replacement.push(Face::simple(
                FaceGeometry::Nurbs(surface),
                Wire::new(vec![
                    OrientedEdge::forward(join_arcs[i].clone()),
                    OrientedEdge::forward(vertical[next].clone()),
                    OrientedEdge::reversed(top_arcs[i].clone()),
                    OrientedEdge::reversed(vertical[i].clone()),
                ]),
            ));
        }
        for i in 0..4 {
            let next = (i + 1) % 4;
            let (profile_degree, profile) = match blend {
                ShoulderRootBlend::Fillet(_) => (
                    2,
                    vec![
                        (join_radius, join_height, 1.0),
                        profile_spec.unwrap(),
                        (base_radius, 0.0, 1.0),
                    ],
                ),
                ShoulderRootBlend::Chamfer(_) => (
                    1,
                    vec![(join_radius, join_height, 1.0), (base_radius, 0.0, 1.0)],
                ),
            };
            let rows = profile
                .into_iter()
                .map(|(radial, height, weight)| {
                    vec![
                        ControlPoint3::new(point(radial, height, theta(i)), weight),
                        ControlPoint3::new(tangent(radial, height, i), weight * FRAC_1_SQRT_2),
                        ControlPoint3::new(point(radial, height, theta(next)), weight),
                    ]
                })
                .collect();
            let surface = NurbsSurface3::new(
                profile_degree,
                2,
                rows,
                KnotVector::clamped_uniform(profile_degree + 1, profile_degree),
                KnotVector::clamped_uniform(3, 2),
            )?;
            replacement.push(Face::simple(
                FaceGeometry::Nurbs(surface),
                Wire::new(vec![
                    OrientedEdge::forward(profiles[i].clone()),
                    OrientedEdge::forward(base_arcs[i].clone()),
                    OrientedEdge::reversed(profiles[next].clone()),
                    OrientedEdge::reversed(join_arcs[i].clone()),
                ]),
            ));
        }

        let shoulder_wire = Wire::new(
            (0..4)
                .rev()
                .map(|i| OrientedEdge::reversed(base_arcs[i].clone()))
                .collect(),
        );
        let upper_wire = Wire::new(
            (0..4)
                .map(|i| OrientedEdge::forward(top_arcs[i].clone()))
                .collect(),
        );

        let mut faces = Vec::with_capacity(solid.outer_shell.faces.len() + 4);
        for (index, face) in solid.outer_shell.faces.iter().enumerate() {
            if self.side_indices.contains(&index) {
                continue;
            }
            if index == self.shoulder_index {
                faces.push(replace_inner_wire_preserving_id(
                    face,
                    self.shoulder_inner_index,
                    shoulder_wire.clone(),
                ));
            } else if index == self.upper_planar_index {
                faces.push(replace_outer_wire_preserving_id(face, upper_wire.clone()));
            } else {
                faces.push(face.clone());
            }
        }
        faces.extend(replacement);
        Solid::try_new(
            Shell::closed(faces),
            solid.inner_shells.clone(),
            &Tolerance::default(),
        )
        .map_err(|error| format!("Local shoulder-root blend produced an invalid solid: {error}"))
    }
}

fn all_wires(face: &Face) -> impl Iterator<Item = &Wire> {
    std::iter::once(&face.outer_wire).chain(face.inner_wires.iter())
}

fn wire_is_circle(wire: &Wire, center: Point3, radius: f64, normal: Vec3, tolerance: f64) -> bool {
    wire.edges.iter().all(|oriented| {
        let Some((edge_center, edge_radius, _)) = super::circle_from_edge(&oriented.edge) else {
            return false;
        };
        let (t_min, t_max) = oriented.edge.curve.param_range();
        (edge_center - center).norm() <= tolerance
            && (edge_radius - radius).abs() <= tolerance
            && (0..=8).all(|step| {
                let t = t_min + (t_max - t_min) * step as f64 / 8.0;
                let point = oriented.edge.curve.evaluate(t);
                (point - center).dot(&normal).abs() <= tolerance
            })
    })
}

fn radial_clearance(
    shoulder: &Face,
    selected_inner: usize,
    center: Point3,
    normal: Vec3,
) -> Option<f64> {
    let mut nearest = f64::INFINITY;
    for wire in std::iter::once(&shoulder.outer_wire).chain(
        shoulder
            .inner_wires
            .iter()
            .enumerate()
            .filter(|(index, _)| *index != selected_inner)
            .map(|(_, wire)| wire),
    ) {
        for edge in &wire.edges {
            let (t_min, t_max) = edge.edge.curve.param_range();
            for step in 0..=64 {
                let t = t_min + (t_max - t_min) * step as f64 / 64.0;
                let point = edge.edge.curve.evaluate(t);
                let offset = point - center;
                let radial = (offset - normal * offset.dot(&normal)).norm();
                nearest = nearest.min(radial);
            }
        }
    }
    nearest.is_finite().then_some(nearest)
}

fn replace_inner_wire_preserving_id(face: &Face, index: usize, wire: Wire) -> Face {
    let mut inners = face.inner_wires.clone();
    inners[index] = wire;
    let mut replacement = Face::new(
        face.geometry.clone(),
        face.outer_wire.clone(),
        inners,
        face.orientation,
        face.tolerance,
    );
    replacement.id = face.id;
    replacement
}

fn replace_outer_wire_preserving_id(face: &Face, wire: Wire) -> Face {
    let mut replacement = Face::new(
        face.geometry.clone(),
        wire,
        face.inner_wires.clone(),
        face.orientation,
        face.tolerance,
    );
    replacement.id = face.id;
    replacement
}
