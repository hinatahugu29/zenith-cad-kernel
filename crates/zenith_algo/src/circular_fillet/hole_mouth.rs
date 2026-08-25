//! Exact local blending of a circular through-hole mouth.
//!
//! Unlike the pure-cylinder path, this operation keeps every unrelated face.
//! It retrims one planar inner wire, shortens only the four bore patches, and
//! inserts four exact rational quarter-torus or conical patches. Recognition is kept
//! deliberately strict: this first form accepts an unbroken cylindrical
//! through hole between two planar inner wires, represented either by four
//! quadrant arcs or by one imported full-circle edge. Raw sectorised builder
//! output must be passed through `FaceMerger::simplify_solid` first.

use std::collections::BTreeSet;
use std::f64::consts::{FRAC_1_SQRT_2, FRAC_PI_2, PI, SQRT_2, TAU};

use zenith_geom::{ControlPoint3, KnotVector, NurbsCurve3, NurbsSurface3};
use zenith_math::{Point3, Tolerance, Vec3, Vec3Ext};
use zenith_topo::{Edge, Face, FaceGeometry, OrientedEdge, Shell, Solid, Vertex, Wire};

use crate::{BlendableEdge, EdgeBlendReport};

pub(crate) fn try_fillet_hole_mouth(
    solid: &Solid,
    edge_id: u64,
    radius: f64,
) -> Result<Option<(Solid, EdgeBlendReport)>, String> {
    let Some(site) = HoleMouth::recognize(solid, edge_id) else {
        return Ok(None);
    };
    let limit_margin = 1e-6 * site.max_radius.max(site.hole_radius).max(1.0);
    if radius >= site.max_radius - limit_margin {
        return Err(format!(
            "Hole-mouth fillet radius {radius} must be smaller than the available setback {:.6}",
            site.max_radius
        ));
    }
    let result = site.apply(solid, HoleMouthBlend::Fillet(radius))?;
    Ok(Some((
        result,
        EdgeBlendReport {
            dihedral_angle_deg: 90.0,
            edge_length: TAU * site.hole_radius,
            setback: radius,
            predicted_removed_volume: removed_volume(site.hole_radius, radius),
        },
    )))
}

pub(crate) fn try_chamfer_hole_mouth(
    solid: &Solid,
    edge_id: u64,
    distance: f64,
) -> Result<Option<(Solid, EdgeBlendReport)>, String> {
    let Some(site) = HoleMouth::recognize(solid, edge_id) else {
        return Ok(None);
    };
    let limit_margin = 1e-6 * site.max_radius.max(site.hole_radius).max(1.0);
    if distance >= site.max_radius - limit_margin {
        return Err(format!(
            "Hole-mouth chamfer distance {distance} must be smaller than the available setback {:.6}",
            site.max_radius
        ));
    }
    let result = site.apply(solid, HoleMouthBlend::Chamfer(distance))?;
    Ok(Some((
        result,
        EdgeBlendReport {
            dihedral_angle_deg: 90.0,
            edge_length: TAU * site.hole_radius,
            setback: distance,
            predicted_removed_volume: chamfer_removed_volume(site.hole_radius, distance),
        },
    )))
}

pub(crate) fn hole_mouth_blendable(solid: &Solid, edge_id: u64) -> Option<BlendableEdge> {
    let site = HoleMouth::recognize(solid, edge_id)?;
    Some(BlendableEdge {
        edge_id,
        length: TAU * site.hole_radius,
        dihedral_angle_deg: 90.0,
        max_fillet_radius: site.max_radius * 0.999,
        max_chamfer_distance: site.max_radius * 0.999,
    })
}

fn chamfer_removed_volume(hole_radius: f64, distance: f64) -> f64 {
    PI * distance * distance * (hole_radius + distance / 3.0)
}

#[derive(Clone, Copy)]
enum HoleMouthBlend {
    Fillet(f64),
    Chamfer(f64),
}

impl HoleMouthBlend {
    fn setback(self) -> f64 {
        match self {
            Self::Fillet(value) | Self::Chamfer(value) => value,
        }
    }
}

fn removed_volume(hole_radius: f64, fillet: f64) -> f64 {
    PI * (hole_radius * fillet * fillet * (2.0 - PI * 0.5)
        + fillet.powi(3) * (5.0 / 3.0 - PI * 0.5))
}

struct HoleMouth {
    cap_index: usize,
    opposite_cap_index: usize,
    bore_indices: BTreeSet<usize>,
    cap_inner_index: usize,
    opposite_inner_index: usize,
    center: Point3,
    outward_axis: Vec3,
    x_axis: Vec3,
    hole_radius: f64,
    depth: f64,
    max_radius: f64,
}

impl HoleMouth {
    fn recognize(solid: &Solid, edge_id: u64) -> Option<Self> {
        let faces = &solid.outer_shell.faces;
        let selected = faces
            .iter()
            .flat_map(all_wires)
            .flat_map(|wire| wire.edges.iter())
            .find(|oriented| oriented.edge.id == edge_id)?
            .edge
            .clone();

        let (cap_index, cap_inner_index) = faces.iter().enumerate().find_map(|(fi, face)| {
            if !matches!(face.geometry, FaceGeometry::Plane(_)) {
                return None;
            }
            face.inner_wires.iter().enumerate().find_map(|(wi, wire)| {
                wire.edges
                    .iter()
                    .any(|edge| super::same_edge_geometry(&selected, &edge.edge))
                    .then_some((fi, wi))
            })
        })?;
        let cap = &faces[cap_index];
        let mouth_wire = &cap.inner_wires[cap_inner_index];
        if !matches!(mouth_wire.edges.len(), 1 | 4) {
            return None;
        }

        let (center, hole_radius, x_axis) = super::circle_from_edge(&selected)?;
        let outward_axis = super::effective_plane_normal(cap).try_normalize_safe(1e-12)?;
        let scale = hole_radius.max(1.0);
        let tolerance = 2e-6 * scale;
        if !wire_is_circle(mouth_wire, center, hole_radius, outward_axis, tolerance) {
            return None;
        }

        let mut bore_indices = BTreeSet::new();
        for mouth_edge in &mouth_wire.edges {
            let users: Vec<usize> = faces
                .iter()
                .enumerate()
                .filter(|(_, face)| {
                    all_wires(face).any(|wire| {
                        wire.edges.iter().any(|candidate| {
                            super::same_edge_geometry(&mouth_edge.edge, &candidate.edge)
                        })
                    })
                })
                .map(|(index, _)| index)
                .collect();
            if users.len() != 2 || !users.contains(&cap_index) {
                return None;
            }
            let bore = *users.iter().find(|index| **index != cap_index)?;
            if !matches!(faces[bore].geometry, FaceGeometry::Nurbs(_)) {
                return None;
            }
            bore_indices.insert(bore);
        }
        if bore_indices.len() != mouth_wire.edges.len() {
            return None;
        }

        let inward = -outward_axis;
        let mut depth: Option<f64> = None;
        let mut opposite_edges = Vec::with_capacity(4);
        for bore_index in &bore_indices {
            let face = &faces[*bore_index];
            if face.outer_wire.edges.len() != 4 || !face.inner_wires.is_empty() {
                return None;
            }
            let opposite = face.outer_wire.edges.iter().find(|candidate| {
                if mouth_wire
                    .edges
                    .iter()
                    .any(|mouth| super::same_edge_geometry(&mouth.edge, &candidate.edge))
                {
                    return false;
                }
                let Some((other_center, other_radius, _)) =
                    super::circle_from_edge(&candidate.edge)
                else {
                    return false;
                };
                let axial = (other_center - center).dot(&inward);
                let sideways = other_center - center - inward * axial;
                axial > tolerance
                    && sideways.norm() <= tolerance
                    && (other_radius - hole_radius).abs() <= tolerance
            })?;
            let (other_center, _, _) = super::circle_from_edge(&opposite.edge)?;
            let this_depth = (other_center - center).dot(&inward);
            if let Some(expected) = depth {
                if (this_depth - expected).abs() > tolerance {
                    return None;
                }
            } else {
                depth = Some(this_depth);
            }
            opposite_edges.push(opposite.edge.clone());
        }
        let depth = depth?;
        if depth <= 1e-6 {
            return None;
        }

        // Every selected side patch must be the unbroken cylindrical wall.
        let surface_tolerance = 3e-6 * hole_radius.max(depth).max(1.0);
        for bore_index in &bore_indices {
            let FaceGeometry::Nurbs(surface) = &faces[*bore_index].geometry else {
                return None;
            };
            let ((u_min, u_max), (v_min, v_max)) = surface.param_range();
            for iu in 0..=6 {
                for iv in 0..=6 {
                    let u = u_min + (u_max - u_min) * iu as f64 / 6.0;
                    let v = v_min + (v_max - v_min) * iv as f64 / 6.0;
                    let point = surface.evaluate(u, v);
                    let offset = point - center;
                    let axial = offset.dot(&inward);
                    let radial = (offset - inward * axial).norm();
                    if axial < -surface_tolerance
                        || axial > depth + surface_tolerance
                        || (radial - hole_radius).abs() > surface_tolerance
                    {
                        return None;
                    }
                }
            }
        }

        let opposite_center = center + inward * depth;
        let (opposite_cap_index, opposite_inner_index) =
            faces.iter().enumerate().find_map(|(fi, face)| {
                if fi == cap_index || !matches!(face.geometry, FaceGeometry::Plane(_)) {
                    return None;
                }
                if super::effective_plane_normal(face).dot(&outward_axis) > -1.0 + 1e-7 {
                    return None;
                }
                face.inner_wires.iter().enumerate().find_map(|(wi, wire)| {
                    (matches!(wire.edges.len(), 1 | 4)
                        && wire_is_circle(
                            wire,
                            opposite_center,
                            hole_radius,
                            outward_axis,
                            tolerance,
                        )
                        && opposite_edges.iter().all(|edge| {
                            wire.edges
                                .iter()
                                .any(|candidate| super::same_edge_geometry(edge, &candidate.edge))
                        }))
                    .then_some((fi, wi))
                })
            })?;

        let clearance = radial_clearance(cap, cap_inner_index, center, outward_axis)? - hole_radius;
        let max_radius = depth.min(clearance);
        if max_radius <= 1e-6 {
            return None;
        }

        Some(Self {
            cap_index,
            opposite_cap_index,
            bore_indices,
            cap_inner_index,
            opposite_inner_index,
            center,
            outward_axis,
            x_axis,
            hole_radius,
            depth,
            max_radius,
        })
    }

    fn apply(&self, solid: &Solid, blend: HoleMouthBlend) -> Result<Solid, String> {
        let axis = self.outward_axis;
        let inward = -axis;
        let x = (self.x_axis - axis * self.x_axis.dot(&axis))
            .try_normalize_safe(1e-12)
            .ok_or("Hole-mouth radial axis is degenerate")?;
        let y = axis.cross(&x).normalize();
        let theta = |index: usize| FRAC_PI_2 * (index % 4) as f64;
        let point = |radial: f64, depth: f64, angle: f64| {
            self.center + x * (radial * angle.cos()) + y * (radial * angle.sin()) + inward * depth
        };
        let tangent = |radial: f64, depth: f64, index: usize| {
            let middle = theta(index) + FRAC_PI_2 * 0.5;
            point(SQRT_2 * radial, depth, middle)
        };

        let setback = blend.setback();
        let mouth_radius = self.hole_radius + setback;
        let join_depth = setback;
        let mouth: Vec<Vertex> = (0..4)
            .map(|i| Vertex::from_point(point(mouth_radius, 0.0, theta(i))))
            .collect();
        let join: Vec<Vertex> = (0..4)
            .map(|i| Vertex::from_point(point(self.hole_radius, join_depth, theta(i))))
            .collect();
        let bottom: Vec<Vertex> = (0..4)
            .map(|i| Vertex::from_point(point(self.hole_radius, self.depth, theta(i))))
            .collect();

        let arc = |radial: f64,
                   depth: f64,
                   index: usize,
                   start: Vertex,
                   end: Vertex|
         -> Result<Edge, String> {
            let curve = NurbsCurve3::new(
                2,
                vec![
                    ControlPoint3::unweighted(start.point),
                    ControlPoint3::new(tangent(radial, depth, index), FRAC_1_SQRT_2),
                    ControlPoint3::unweighted(end.point),
                ],
                KnotVector::clamped_uniform(3, 2),
            )?;
            Ok(Edge::new(curve, start, end, 1e-6))
        };

        let mut mouth_arcs = Vec::with_capacity(4);
        let mut join_arcs = Vec::with_capacity(4);
        let mut bottom_arcs = Vec::with_capacity(4);
        let mut vertical = Vec::with_capacity(4);
        let mut profiles = Vec::with_capacity(4);
        for i in 0..4 {
            let next = (i + 1) % 4;
            mouth_arcs.push(arc(
                mouth_radius,
                0.0,
                i,
                mouth[i].clone(),
                mouth[next].clone(),
            )?);
            join_arcs.push(arc(
                self.hole_radius,
                join_depth,
                i,
                join[i].clone(),
                join[next].clone(),
            )?);
            bottom_arcs.push(arc(
                self.hole_radius,
                self.depth,
                i,
                bottom[i].clone(),
                bottom[next].clone(),
            )?);
            vertical.push(Edge::line_between(bottom[i].clone(), join[i].clone())?);
            profiles.push(match blend {
                HoleMouthBlend::Fillet(_) => {
                    let curve = NurbsCurve3::new(
                        2,
                        vec![
                            ControlPoint3::unweighted(join[i].point),
                            ControlPoint3::new(
                                point(self.hole_radius, 0.0, theta(i)),
                                FRAC_1_SQRT_2,
                            ),
                            ControlPoint3::unweighted(mouth[i].point),
                        ],
                        KnotVector::clamped_uniform(3, 2),
                    )?;
                    Edge::new(curve, join[i].clone(), mouth[i].clone(), 1e-6)
                }
                HoleMouthBlend::Chamfer(_) => {
                    Edge::line_between(join[i].clone(), mouth[i].clone())?
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
                        ControlPoint3::unweighted(bottom[next].point),
                        ControlPoint3::unweighted(join[next].point),
                    ],
                    vec![
                        ControlPoint3::new(tangent(self.hole_radius, self.depth, i), FRAC_1_SQRT_2),
                        ControlPoint3::new(tangent(self.hole_radius, join_depth, i), FRAC_1_SQRT_2),
                    ],
                    vec![
                        ControlPoint3::unweighted(bottom[i].point),
                        ControlPoint3::unweighted(join[i].point),
                    ],
                ],
                KnotVector::clamped_uniform(3, 2),
                KnotVector::clamped_uniform(2, 1),
            )?;
            replacement.push(Face::simple(
                FaceGeometry::Nurbs(surface),
                Wire::new(vec![
                    OrientedEdge::forward(vertical[i].clone()),
                    OrientedEdge::forward(join_arcs[i].clone()),
                    OrientedEdge::reversed(vertical[next].clone()),
                    OrientedEdge::reversed(bottom_arcs[i].clone()),
                ]),
            ));
        }
        for i in 0..4 {
            let next = (i + 1) % 4;
            let (profile_degree, profile) = match blend {
                HoleMouthBlend::Fillet(_) => (
                    2,
                    vec![
                        (self.hole_radius, join_depth, 1.0),
                        (self.hole_radius, 0.0, FRAC_1_SQRT_2),
                        (mouth_radius, 0.0, 1.0),
                    ],
                ),
                HoleMouthBlend::Chamfer(_) => (
                    1,
                    vec![
                        (self.hole_radius, join_depth, 1.0),
                        (mouth_radius, 0.0, 1.0),
                    ],
                ),
            };
            let rows = profile
                .into_iter()
                .map(|(radial, depth, weight)| {
                    vec![
                        ControlPoint3::new(point(radial, depth, theta(i)), weight),
                        ControlPoint3::new(tangent(radial, depth, i), weight * FRAC_1_SQRT_2),
                        ControlPoint3::new(point(radial, depth, theta(next)), weight),
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
                    OrientedEdge::forward(mouth_arcs[i].clone()),
                    OrientedEdge::reversed(profiles[next].clone()),
                    OrientedEdge::reversed(join_arcs[i].clone()),
                    OrientedEdge::forward(profiles[i].clone()),
                ]),
            ));
        }

        let mouth_wire = Wire::new(
            (0..4)
                .rev()
                .map(|i| OrientedEdge::reversed(mouth_arcs[i].clone()))
                .collect(),
        );
        let bottom_wire = Wire::new(
            (0..4)
                .map(|i| OrientedEdge::forward(bottom_arcs[i].clone()))
                .collect(),
        );

        let mut faces = Vec::with_capacity(solid.outer_shell.faces.len() + 4);
        for (index, face) in solid.outer_shell.faces.iter().enumerate() {
            if self.bore_indices.contains(&index) {
                continue;
            }
            if index == self.cap_index {
                faces.push(replace_inner_wire(
                    face,
                    self.cap_inner_index,
                    mouth_wire.clone(),
                ));
            } else if index == self.opposite_cap_index {
                faces.push(replace_inner_wire(
                    face,
                    self.opposite_inner_index,
                    bottom_wire.clone(),
                ));
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
        .map_err(|error| format!("Local hole-mouth blend produced an invalid solid: {error}"))
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
    cap: &Face,
    selected_inner: usize,
    center: Point3,
    normal: Vec3,
) -> Option<f64> {
    let mut nearest = f64::INFINITY;
    for wire in std::iter::once(&cap.outer_wire).chain(
        cap.inner_wires
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

fn replace_inner_wire(face: &Face, index: usize, wire: Wire) -> Face {
    let mut inners = face.inner_wires.clone();
    inners[index] = wire;
    Face::new(
        face.geometry.clone(),
        face.outer_wire.clone(),
        inners,
        face.orientation,
        face.tolerance,
    )
}
