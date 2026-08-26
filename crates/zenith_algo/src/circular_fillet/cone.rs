//! Exact constant-radius fillet for a planar cap rim of a pure cone/frustum.
//!
//! The rolling-ball centre is the intersection of the cap offset plane and
//! the cone offset line in a meridian section. Rotating the resulting circular
//! contact arc gives an exact torus patch even though the dihedral angle is not
//! ninety degrees. This module deliberately accepts only a pure cone/frustum;
//! it does not rebuild bosses, holes, or stepped shafts as a cone.

use std::f64::consts::{FRAC_1_SQRT_2, FRAC_PI_2, SQRT_2};

use zenith_geom::{ControlPoint3, KnotVector, NurbsCurve3, NurbsSurface3, PlaneSurface3};
use zenith_math::{Point3, Tolerance, Transform3, Vec3, Vec3Ext};
use zenith_topo::{Edge, Face, FaceGeometry, OrientedEdge, Shell, Solid, Vertex, Wire};

use crate::{
    BlendableEdge, BrepTransform, ChamferBuilder, EdgeBlendReport, FilletBuilder, PrimitiveBuilder,
};

use super::{circle_from_edge, effective_plane_normal, same_edge_geometry};

impl FilletBuilder {
    /// Builds a pure right cone/frustum whose top planar rim has an exact
    /// constant-radius fillet.
    ///
    /// `r_bottom` may be zero, which represents an apex at `z = 0` and a
    /// circular cap at `z = height`. `r_top` must be positive because this
    /// operation specifically fillets the top circular rim.
    pub fn fillet_cone_top_edge(
        r_bottom: f64,
        r_top: f64,
        height: f64,
        fillet_radius: f64,
        _tol: &Tolerance,
    ) -> Result<Solid, String> {
        if r_bottom < 0.0 || r_top <= 1e-6 || height <= 1e-6 {
            return Err(format!(
                "Cone radii and height are invalid: bottom={r_bottom}, top={r_top}, height={height}"
            ));
        }
        if (r_bottom - r_top).abs() <= 1e-7 * r_bottom.max(r_top).max(1.0) {
            return Err(
                "Equal cone radii describe a cylinder; use the cylinder rim builder".into(),
            );
        }
        if fillet_radius < 0.0 {
            return Err(format!(
                "Fillet radius must not be negative, got {fillet_radius}"
            ));
        }
        if fillet_radius <= 1e-6 {
            return unfilleted_cone_with_top_cap(r_bottom, r_top, height);
        }

        let geometry = ConeFilletGeometry::new(r_bottom, r_top, height, fillet_radius)?;
        build_top_filleted_cone(geometry)
    }
}

impl ChamferBuilder {
    /// Builds a pure right cone/frustum whose top planar rim has an exact
    /// equal-distance chamfer.
    ///
    /// `distance` is measured on the planar cap and along the conical
    /// generator. The cone is aligned with +Z and the selected cap is at
    /// `z = height`.
    pub fn chamfer_cone_top_edge(
        r_bottom: f64,
        r_top: f64,
        height: f64,
        distance: f64,
        _tol: &Tolerance,
    ) -> Result<Solid, String> {
        if r_bottom < 0.0 || r_top <= 1e-6 || height <= 1e-6 {
            return Err(format!(
                "Cone radii and height are invalid: bottom={r_bottom}, top={r_top}, height={height}"
            ));
        }
        if (r_bottom - r_top).abs() <= 1e-7 * r_bottom.max(r_top).max(1.0) {
            return Err(
                "Equal cone radii describe a cylinder; use the cylinder rim builder".into(),
            );
        }
        if distance < 0.0 || !distance.is_finite() {
            return Err(format!(
                "Chamfer distance must be finite and not negative, got {distance}"
            ));
        }
        if distance <= 1e-6 {
            return unfilleted_cone_with_top_cap(r_bottom, r_top, height);
        }

        let geometry = ConeChamferGeometry::new(r_bottom, r_top, height, distance)?;
        build_top_chamfered_cone(geometry)
    }
}

pub(crate) fn try_fillet_conical_rim(
    solid: &Solid,
    edge_id: u64,
    fillet_radius: f64,
) -> Result<Option<(Solid, EdgeBlendReport)>, String> {
    let Some(site) = PureConeRim::recognize(solid, edge_id) else {
        return Ok(None);
    };
    let geometry = ConeFilletGeometry::new(
        site.opposite_radius,
        site.selected_radius,
        site.height,
        fillet_radius,
    )?;
    let canonical = build_top_filleted_cone(geometry)?;
    let result = BrepTransform::transform_solid(&canonical, &site.canonical_to_world())?;

    Ok(Some((
        result,
        EdgeBlendReport {
            dihedral_angle_deg: geometry.dihedral().to_degrees(),
            edge_length: std::f64::consts::TAU * site.selected_radius,
            setback: geometry.cap_setback(),
            predicted_removed_volume: geometry.removed_volume(),
        },
    )))
}

pub(crate) fn try_chamfer_conical_rim(
    solid: &Solid,
    edge_id: u64,
    distance: f64,
) -> Result<Option<(Solid, EdgeBlendReport)>, String> {
    let Some(site) = PureConeRim::recognize(solid, edge_id) else {
        return Ok(None);
    };
    let geometry = ConeChamferGeometry::new(
        site.opposite_radius,
        site.selected_radius,
        site.height,
        distance,
    )?;
    let canonical = build_top_chamfered_cone(geometry)?;
    let result = BrepTransform::transform_solid(&canonical, &site.canonical_to_world())?;

    Ok(Some((
        result,
        EdgeBlendReport {
            dihedral_angle_deg: geometry.dihedral().to_degrees(),
            edge_length: std::f64::consts::TAU * site.selected_radius,
            setback: distance,
            predicted_removed_volume: geometry.removed_volume(),
        },
    )))
}

pub(crate) fn conical_rim_blendable(solid: &Solid, edge_id: u64) -> Option<BlendableEdge> {
    let site = PureConeRim::recognize(solid, edge_id)?;
    let unit =
        ConeFilletGeometry::slope_terms(site.opposite_radius, site.selected_radius, site.height);
    let radial_limit = site.selected_radius / (unit.0 + unit.1);
    let axial_limit = site.height / (1.0 + unit.1 / unit.0);
    let max = radial_limit.min(axial_limit) * 0.999;
    if !(max > 1e-6 && max.is_finite()) {
        return None;
    }
    Some(BlendableEdge {
        edge_id,
        length: std::f64::consts::TAU * site.selected_radius,
        dihedral_angle_deg: (FRAC_PI_2 - unit.1.atan()).to_degrees(),
        max_fillet_radius: max,
        max_chamfer_distance: site.selected_radius.min(site.height * unit.0) * 0.999,
    })
}

#[derive(Clone, Copy)]
struct ConeChamferGeometry {
    r_bottom: f64,
    r_top: f64,
    height: f64,
    distance: f64,
    slope: f64,
    side_radius: f64,
    side_z: f64,
    cap_radius: f64,
}

impl ConeChamferGeometry {
    fn new(r_bottom: f64, r_top: f64, height: f64, distance: f64) -> Result<Self, String> {
        if !(distance > 0.0 && distance.is_finite()) {
            return Err(format!(
                "Cone rim chamfer distance must be positive, got {distance}"
            ));
        }
        let slope = (r_top - r_bottom) / height;
        let norm = slope.hypot(1.0);
        let side_z = height - distance / norm;
        let side_radius = r_top - slope * distance / norm;
        let cap_radius = r_top - distance;
        let scale = r_bottom.max(r_top).max(height).max(1.0);
        let margin = 1e-8 * scale;
        if cap_radius <= margin {
            return Err(format!(
                "Cone rim chamfer distance {distance} collapses the remaining cap (radius {cap_radius:.6})"
            ));
        }
        if side_z <= margin || side_z >= height - margin {
            return Err(format!(
                "Cone rim chamfer distance {distance} reaches the opposite cap or misses the side (contact z {side_z:.6})"
            ));
        }
        if side_radius <= margin {
            return Err(format!(
                "Cone rim chamfer distance {distance} collapses the side contact (radius {side_radius:.6})"
            ));
        }
        Ok(Self {
            r_bottom,
            r_top,
            height,
            distance,
            slope,
            side_radius,
            side_z,
            cap_radius,
        })
    }

    fn dihedral(self) -> f64 {
        FRAC_PI_2 - self.slope.atan()
    }

    fn removed_volume(self) -> f64 {
        let norm = self.slope.hypot(1.0);
        let limit = self.distance / norm;
        let chamfer_slope = norm - self.slope;
        let primitive = |intercept: f64, slope: f64, t: f64| {
            intercept * intercept * t + intercept * slope * t * t + slope * slope * t.powi(3) / 3.0
        };
        std::f64::consts::PI
            * (primitive(self.r_top, -self.slope, limit)
                - primitive(self.cap_radius, chamfer_slope, limit))
    }
}

#[derive(Clone, Copy)]
struct ConeFilletGeometry {
    r_bottom: f64,
    r_top: f64,
    height: f64,
    fillet: f64,
    slope: f64,
    centre_radius: f64,
    centre_z: f64,
    side_radius: f64,
    side_z: f64,
    side_angle: f64,
}

impl ConeFilletGeometry {
    fn slope_terms(r_bottom: f64, r_top: f64, height: f64) -> (f64, f64) {
        let slope = (r_top - r_bottom) / height;
        (slope.hypot(1.0), slope)
    }

    fn new(r_bottom: f64, r_top: f64, height: f64, fillet: f64) -> Result<Self, String> {
        if !(fillet > 0.0 && fillet.is_finite()) {
            return Err(format!(
                "Cone rim fillet radius must be positive, got {fillet}"
            ));
        }
        let (slope_norm, slope) = Self::slope_terms(r_bottom, r_top, height);
        let centre_radius = r_top - fillet * (slope_norm + slope);
        let centre_z = height - fillet;
        let side_radius = centre_radius + fillet / slope_norm;
        let side_z = centre_z - fillet * slope / slope_norm;
        let scale = r_bottom.max(r_top).max(height).max(1.0);
        let margin = 1e-8 * scale;
        if centre_radius <= margin {
            return Err(format!(
                "Cone rim fillet radius {fillet} collapses the remaining cap (contact radius {centre_radius:.6})"
            ));
        }
        if side_z <= margin || side_z >= height - margin {
            return Err(format!(
                "Cone rim fillet radius {fillet} reaches the opposite cap or misses the side (contact z {side_z:.6})"
            ));
        }
        let side_angle = (-slope).atan();
        let sweep = FRAC_PI_2 - side_angle;
        if !(sweep > 1e-8 && sweep < std::f64::consts::PI - 1e-8) {
            return Err("Cone rim fillet has no regular circular contact arc".into());
        }
        Ok(Self {
            r_bottom,
            r_top,
            height,
            fillet,
            slope,
            centre_radius,
            centre_z,
            side_radius,
            side_z,
            side_angle,
        })
    }

    fn dihedral(self) -> f64 {
        FRAC_PI_2 - self.slope.atan()
    }

    fn cap_setback(self) -> f64 {
        self.r_top - self.centre_radius
    }

    fn original_volume(self) -> f64 {
        std::f64::consts::PI
            * self.height
            * (self.r_bottom * self.r_bottom + self.r_bottom * self.r_top + self.r_top * self.r_top)
            / 3.0
    }

    fn result_volume(self) -> f64 {
        let lower = std::f64::consts::PI
            * self.side_z
            * (self.r_bottom * self.r_bottom
                + self.r_bottom * self.side_radius
                + self.side_radius * self.side_radius)
            / 3.0;
        let primitive = |angle: f64| {
            let sine = angle.sin();
            let cosine = angle.cos();
            self.fillet * self.centre_radius * self.centre_radius * sine
                + self.fillet * self.fillet * self.centre_radius * (angle + sine * cosine)
                + self.fillet.powi(3) * (sine - sine.powi(3) / 3.0)
        };
        lower + std::f64::consts::PI * (primitive(FRAC_PI_2) - primitive(self.side_angle))
    }

    fn removed_volume(self) -> f64 {
        self.original_volume() - self.result_volume()
    }
}

struct PureConeRim {
    selected_center: Point3,
    outward_axis: Vec3,
    x_axis: Vec3,
    selected_radius: f64,
    opposite_radius: f64,
    height: f64,
}

impl PureConeRim {
    fn recognize(solid: &Solid, edge_id: u64) -> Option<Self> {
        if !solid.inner_shells.is_empty()
            || solid
                .outer_shell
                .faces
                .iter()
                .any(|face| !face.inner_wires.is_empty())
        {
            return None;
        }
        let faces = &solid.outer_shell.faces;
        let selected = faces
            .iter()
            .flat_map(|face| face.outer_wire.edges.iter())
            .find(|oriented| oriented.edge.id == edge_id)?
            .edge
            .clone();

        let uses: Vec<usize> = faces
            .iter()
            .enumerate()
            .filter_map(|(index, face)| {
                face.outer_wire
                    .edges
                    .iter()
                    .any(|oriented| same_edge_geometry(&selected, &oriented.edge))
                    .then_some(index)
            })
            .collect();
        if uses.len() != 2 {
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
        let (selected_center, selected_radius, x_axis) = circle_from_edge(&selected)?;
        let scale = selected_radius.max(1.0);
        if (selected_center - cap_plane.origin)
            .dot(&cap_plane.normal)
            .abs()
            > 1e-7 * scale
        {
            return None;
        }
        let outward_axis = effective_plane_normal(cap).try_normalize_safe(1e-12)?;
        let edge_plane_tolerance = 2e-7 * scale;
        for step in 0..=16 {
            let point = selected.curve.evaluate(step as f64 / 16.0);
            if (point - cap_plane.origin).dot(&outward_axis).abs() > edge_plane_tolerance {
                return None;
            }
        }
        let (curve_min, curve_max) = selected.curve.param_range();
        if selected
            .curve
            .evaluate_derivatives((curve_min + curve_max) * 0.5, 1)
            .get(1)
            .map(|tangent| tangent.dot(&outward_axis).abs() > 1e-7 * tangent.norm().max(1.0))
            .unwrap_or(true)
        {
            return None;
        }
        let inward = -outward_axis;

        let vertices: Vec<Point3> = faces
            .iter()
            .flat_map(|face| face.outer_wire.edges.iter())
            .flat_map(|oriented| {
                [
                    oriented.edge.start_vertex.point,
                    oriented.edge.end_vertex.point,
                ]
            })
            .collect();
        let height = vertices
            .iter()
            .map(|point| (*point - selected_center).dot(&inward))
            .fold(0.0_f64, f64::max);
        if height <= 1e-6 {
            return None;
        }
        let tolerance = 2e-6 * selected_radius.max(height).max(1.0);
        let opposite: Vec<f64> = vertices
            .iter()
            .filter_map(|point| {
                let offset = *point - selected_center;
                let axial = offset.dot(&inward);
                ((axial - height).abs() <= tolerance).then_some((offset - inward * axial).norm())
            })
            .collect();
        if opposite.is_empty() {
            return None;
        }
        let opposite_radius = opposite.iter().sum::<f64>() / opposite.len() as f64;
        if (selected_radius - opposite_radius).abs() <= tolerance {
            return None;
        }

        for point in &vertices {
            let offset = *point - selected_center;
            let axial = offset.dot(&inward);
            let radial = (offset - inward * axial).norm();
            let on_selected =
                axial.abs() <= tolerance && (radial - selected_radius).abs() <= tolerance;
            let on_opposite = (axial - height).abs() <= tolerance
                && (radial - opposite_radius).abs() <= tolerance;
            if !on_selected && !on_opposite {
                return None;
            }
        }

        let planar_count = faces
            .iter()
            .filter(|face| matches!(face.geometry, FaceGeometry::Plane(_)))
            .count();
        let expected_planar = if opposite_radius <= tolerance { 1 } else { 2 };
        if planar_count != expected_planar {
            return None;
        }
        let mut side_count = 0usize;
        for face in faces {
            match &face.geometry {
                FaceGeometry::Plane(_) => {
                    let normal = effective_plane_normal(face);
                    if normal.dot(&outward_axis).abs() < 1.0 - 1e-7 {
                        return None;
                    }
                }
                FaceGeometry::Nurbs(surface) => {
                    side_count += 1;
                    for iu in 0..=4 {
                        for iv in 0..=4 {
                            let point = surface.evaluate(iu as f64 / 4.0, iv as f64 / 4.0);
                            let offset = point - selected_center;
                            let axial = offset.dot(&inward);
                            if axial < -tolerance || axial > height + tolerance {
                                return None;
                            }
                            let radial = (offset - inward * axial).norm();
                            let expected = selected_radius
                                + (opposite_radius - selected_radius) * axial / height;
                            if (radial - expected).abs() > tolerance {
                                return None;
                            }
                        }
                    }
                }
                _ => return None,
            }
        }
        if side_count == 0 {
            return None;
        }

        Some(Self {
            selected_center,
            outward_axis,
            x_axis,
            selected_radius,
            opposite_radius,
            height,
        })
    }

    fn canonical_to_world(&self) -> Transform3 {
        let z = self.outward_axis;
        let x = self.x_axis;
        let y = z.cross(&x).normalize();
        let base = self.selected_center - z * self.height;
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

fn unfilleted_cone_with_top_cap(r_bottom: f64, r_top: f64, height: f64) -> Result<Solid, String> {
    if r_bottom > 1e-6 {
        return PrimitiveBuilder::make_cone(r_bottom, r_top, height);
    }
    let cone = PrimitiveBuilder::make_cone(r_top, 0.0, height)?;
    let flip = Transform3::from_axis_angle(&Vec3::new(1.0, 0.0, 0.0), std::f64::consts::PI);
    let flipped = BrepTransform::transform_solid(&cone, &flip)?;
    Ok(BrepTransform::translate_solid(
        &flipped,
        Vec3::new(0.0, 0.0, height),
    ))
}

fn build_top_filleted_cone(geometry: ConeFilletGeometry) -> Result<Solid, String> {
    let rb = geometry.r_bottom;
    let h = geometry.height;
    let theta = |index: usize| FRAC_PI_2 * (index % 4) as f64;
    let point = |radial: f64, z: f64, angle: f64| {
        Point3::new(radial * angle.cos(), radial * angle.sin(), z)
    };
    let angular_control = |radial: f64, z: f64, angle: f64| {
        let middle = angle + FRAC_PI_2 * 0.5;
        Point3::new(
            SQRT_2 * radial * middle.cos(),
            SQRT_2 * radial * middle.sin(),
            z,
        )
    };
    let circular_arc =
        |radial: f64, z: f64, index: usize, start: Vertex, end: Vertex| -> Result<Edge, String> {
            Ok(Edge::new(
                NurbsCurve3::new(
                    2,
                    vec![
                        ControlPoint3::unweighted(start.point),
                        ControlPoint3::new(angular_control(radial, z, theta(index)), FRAC_1_SQRT_2),
                        ControlPoint3::unweighted(end.point),
                    ],
                    KnotVector::clamped_uniform(3, 2),
                )?,
                start,
                end,
                1e-6,
            ))
        };

    let join: Vec<Vertex> = (0..4)
        .map(|i| Vertex::from_point(point(geometry.side_radius, geometry.side_z, theta(i))))
        .collect();
    let top: Vec<Vertex> = (0..4)
        .map(|i| Vertex::from_point(point(geometry.centre_radius, h, theta(i))))
        .collect();
    let bottom: Vec<Vertex> = if rb > 1e-8 {
        (0..4)
            .map(|i| Vertex::from_point(point(rb, 0.0, theta(i))))
            .collect()
    } else {
        Vec::new()
    };
    let apex = (rb <= 1e-8).then(|| Vertex::from_point(Point3::new(0.0, 0.0, 0.0)));

    let mut bottom_arcs = Vec::new();
    let mut join_arcs = Vec::with_capacity(4);
    let mut top_arcs = Vec::with_capacity(4);
    let mut side_edges = Vec::with_capacity(4);
    let mut profile_edges = Vec::with_capacity(4);

    let profile_sweep = FRAC_PI_2 - geometry.side_angle;
    let profile_weight = (profile_sweep * 0.5).cos();
    let profile_middle = (FRAC_PI_2 + geometry.side_angle) * 0.5;
    let control_radius =
        geometry.centre_radius + geometry.fillet * profile_middle.cos() / profile_weight;
    let control_z = geometry.centre_z + geometry.fillet * profile_middle.sin() / profile_weight;

    for i in 0..4 {
        let next = (i + 1) % 4;
        if rb > 1e-8 {
            bottom_arcs.push(circular_arc(
                rb,
                0.0,
                i,
                bottom[i].clone(),
                bottom[next].clone(),
            )?);
            side_edges.push(Edge::line_between(bottom[i].clone(), join[i].clone())?);
        } else {
            side_edges.push(Edge::line_between(apex.clone().unwrap(), join[i].clone())?);
        }
        join_arcs.push(circular_arc(
            geometry.side_radius,
            geometry.side_z,
            i,
            join[i].clone(),
            join[next].clone(),
        )?);
        top_arcs.push(circular_arc(
            geometry.centre_radius,
            h,
            i,
            top[i].clone(),
            top[next].clone(),
        )?);

        let profile = NurbsCurve3::new(
            2,
            vec![
                ControlPoint3::unweighted(join[i].point),
                ControlPoint3::new(point(control_radius, control_z, theta(i)), profile_weight),
                ControlPoint3::unweighted(top[i].point),
            ],
            KnotVector::clamped_uniform(3, 2),
        )?;
        profile_edges.push(Edge::new(profile, join[i].clone(), top[i].clone(), 1e-6));
    }

    let mut faces = Vec::with_capacity(if rb > 1e-8 { 10 } else { 9 });
    for i in 0..4 {
        let next = (i + 1) % 4;
        let side_rows = if rb > 1e-8 {
            vec![
                vec![
                    ControlPoint3::unweighted(bottom[i].point),
                    ControlPoint3::unweighted(join[i].point),
                ],
                vec![
                    ControlPoint3::new(angular_control(rb, 0.0, theta(i)), FRAC_1_SQRT_2),
                    ControlPoint3::new(
                        angular_control(geometry.side_radius, geometry.side_z, theta(i)),
                        FRAC_1_SQRT_2,
                    ),
                ],
                vec![
                    ControlPoint3::unweighted(bottom[next].point),
                    ControlPoint3::unweighted(join[next].point),
                ],
            ]
        } else {
            let apex_point = apex.as_ref().unwrap().point;
            vec![
                vec![
                    ControlPoint3::unweighted(apex_point),
                    ControlPoint3::unweighted(join[i].point),
                ],
                vec![
                    ControlPoint3::new(apex_point, FRAC_1_SQRT_2),
                    ControlPoint3::new(
                        angular_control(geometry.side_radius, geometry.side_z, theta(i)),
                        FRAC_1_SQRT_2,
                    ),
                ],
                vec![
                    ControlPoint3::unweighted(apex_point),
                    ControlPoint3::unweighted(join[next].point),
                ],
            ]
        };
        let side_surface = NurbsSurface3::new(
            2,
            1,
            side_rows,
            KnotVector::clamped_uniform(3, 2),
            KnotVector::clamped_uniform(2, 1),
        )?;
        let side_wire = if rb > 1e-8 {
            Wire::new(vec![
                OrientedEdge::forward(bottom_arcs[i].clone()),
                OrientedEdge::forward(side_edges[next].clone()),
                OrientedEdge::reversed(join_arcs[i].clone()),
                OrientedEdge::reversed(side_edges[i].clone()),
            ])
        } else {
            Wire::new(vec![
                OrientedEdge::forward(side_edges[next].clone()),
                OrientedEdge::reversed(join_arcs[i].clone()),
                OrientedEdge::reversed(side_edges[i].clone()),
            ])
        };
        faces.push(Face::simple(FaceGeometry::Nurbs(side_surface), side_wire));
    }

    for i in 0..4 {
        let mut rows = Vec::with_capacity(3);
        for (radial, z, weight) in [
            (geometry.side_radius, geometry.side_z, 1.0),
            (control_radius, control_z, profile_weight),
            (geometry.centre_radius, h, 1.0),
        ] {
            rows.push(vec![
                ControlPoint3::new(point(radial, z, theta(i)), weight),
                ControlPoint3::new(angular_control(radial, z, theta(i)), weight * FRAC_1_SQRT_2),
                ControlPoint3::new(point(radial, z, theta(i + 1)), weight),
            ]);
        }
        rows.reverse();
        let torus = NurbsSurface3::new(
            2,
            2,
            rows,
            KnotVector::clamped_uniform(3, 2),
            KnotVector::clamped_uniform(3, 2),
        )?;
        faces.push(Face::simple(
            FaceGeometry::Nurbs(torus),
            Wire::new(vec![
                OrientedEdge::forward(join_arcs[i].clone()),
                OrientedEdge::forward(profile_edges[(i + 1) % 4].clone()),
                OrientedEdge::reversed(top_arcs[i].clone()),
                OrientedEdge::reversed(profile_edges[i].clone()),
            ]),
        ));
    }

    if rb > 1e-8 {
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
    }

    let top_plane = PlaneSurface3::new(
        Point3::new(0.0, 0.0, h),
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

fn build_top_chamfered_cone(geometry: ConeChamferGeometry) -> Result<Solid, String> {
    let rb = geometry.r_bottom;
    let h = geometry.height;
    let theta = |index: usize| FRAC_PI_2 * (index % 4) as f64;
    let point = |radial: f64, z: f64, angle: f64| {
        Point3::new(radial * angle.cos(), radial * angle.sin(), z)
    };
    let angular_control = |radial: f64, z: f64, angle: f64| {
        let middle = angle + FRAC_PI_2 * 0.5;
        Point3::new(
            SQRT_2 * radial * middle.cos(),
            SQRT_2 * radial * middle.sin(),
            z,
        )
    };
    let circular_arc =
        |radial: f64, z: f64, index: usize, start: Vertex, end: Vertex| -> Result<Edge, String> {
            Ok(Edge::new(
                NurbsCurve3::new(
                    2,
                    vec![
                        ControlPoint3::unweighted(start.point),
                        ControlPoint3::new(angular_control(radial, z, theta(index)), FRAC_1_SQRT_2),
                        ControlPoint3::unweighted(end.point),
                    ],
                    KnotVector::clamped_uniform(3, 2),
                )?,
                start,
                end,
                1e-6,
            ))
        };

    let join: Vec<Vertex> = (0..4)
        .map(|i| Vertex::from_point(point(geometry.side_radius, geometry.side_z, theta(i))))
        .collect();
    let top: Vec<Vertex> = (0..4)
        .map(|i| Vertex::from_point(point(geometry.cap_radius, h, theta(i))))
        .collect();
    let bottom: Vec<Vertex> = if rb > 1e-8 {
        (0..4)
            .map(|i| Vertex::from_point(point(rb, 0.0, theta(i))))
            .collect()
    } else {
        Vec::new()
    };
    let apex = (rb <= 1e-8).then(|| Vertex::from_point(Point3::new(0.0, 0.0, 0.0)));

    let mut bottom_arcs = Vec::new();
    let mut join_arcs = Vec::with_capacity(4);
    let mut top_arcs = Vec::with_capacity(4);
    let mut side_edges = Vec::with_capacity(4);
    let mut profile_edges = Vec::with_capacity(4);
    for i in 0..4 {
        let next = (i + 1) % 4;
        if rb > 1e-8 {
            bottom_arcs.push(circular_arc(
                rb,
                0.0,
                i,
                bottom[i].clone(),
                bottom[next].clone(),
            )?);
            side_edges.push(Edge::line_between(bottom[i].clone(), join[i].clone())?);
        } else {
            side_edges.push(Edge::line_between(apex.clone().unwrap(), join[i].clone())?);
        }
        join_arcs.push(circular_arc(
            geometry.side_radius,
            geometry.side_z,
            i,
            join[i].clone(),
            join[next].clone(),
        )?);
        top_arcs.push(circular_arc(
            geometry.cap_radius,
            h,
            i,
            top[i].clone(),
            top[next].clone(),
        )?);
        profile_edges.push(Edge::line_between(join[i].clone(), top[i].clone())?);
    }

    let mut faces = Vec::with_capacity(if rb > 1e-8 { 10 } else { 9 });
    for i in 0..4 {
        let next = (i + 1) % 4;
        let side_rows = if rb > 1e-8 {
            vec![
                vec![
                    ControlPoint3::unweighted(bottom[i].point),
                    ControlPoint3::unweighted(join[i].point),
                ],
                vec![
                    ControlPoint3::new(angular_control(rb, 0.0, theta(i)), FRAC_1_SQRT_2),
                    ControlPoint3::new(
                        angular_control(geometry.side_radius, geometry.side_z, theta(i)),
                        FRAC_1_SQRT_2,
                    ),
                ],
                vec![
                    ControlPoint3::unweighted(bottom[next].point),
                    ControlPoint3::unweighted(join[next].point),
                ],
            ]
        } else {
            let apex_point = apex.as_ref().unwrap().point;
            vec![
                vec![
                    ControlPoint3::unweighted(apex_point),
                    ControlPoint3::unweighted(join[i].point),
                ],
                vec![
                    ControlPoint3::new(apex_point, FRAC_1_SQRT_2),
                    ControlPoint3::new(
                        angular_control(geometry.side_radius, geometry.side_z, theta(i)),
                        FRAC_1_SQRT_2,
                    ),
                ],
                vec![
                    ControlPoint3::unweighted(apex_point),
                    ControlPoint3::unweighted(join[next].point),
                ],
            ]
        };
        let side_surface = NurbsSurface3::new(
            2,
            1,
            side_rows,
            KnotVector::clamped_uniform(3, 2),
            KnotVector::clamped_uniform(2, 1),
        )?;
        let side_wire = if rb > 1e-8 {
            Wire::new(vec![
                OrientedEdge::forward(bottom_arcs[i].clone()),
                OrientedEdge::forward(side_edges[next].clone()),
                OrientedEdge::reversed(join_arcs[i].clone()),
                OrientedEdge::reversed(side_edges[i].clone()),
            ])
        } else {
            Wire::new(vec![
                OrientedEdge::forward(side_edges[next].clone()),
                OrientedEdge::reversed(join_arcs[i].clone()),
                OrientedEdge::reversed(side_edges[i].clone()),
            ])
        };
        faces.push(Face::simple(FaceGeometry::Nurbs(side_surface), side_wire));
    }

    for i in 0..4 {
        let rows = vec![
            vec![
                ControlPoint3::unweighted(top[i].point),
                ControlPoint3::new(
                    angular_control(geometry.cap_radius, h, theta(i)),
                    FRAC_1_SQRT_2,
                ),
                ControlPoint3::unweighted(top[(i + 1) % 4].point),
            ],
            vec![
                ControlPoint3::unweighted(join[i].point),
                ControlPoint3::new(
                    angular_control(geometry.side_radius, geometry.side_z, theta(i)),
                    FRAC_1_SQRT_2,
                ),
                ControlPoint3::unweighted(join[(i + 1) % 4].point),
            ],
        ];
        let chamfer = NurbsSurface3::new(
            1,
            2,
            rows,
            KnotVector::clamped_uniform(2, 1),
            KnotVector::clamped_uniform(3, 2),
        )?;
        faces.push(Face::simple(
            FaceGeometry::Nurbs(chamfer),
            Wire::new(vec![
                OrientedEdge::forward(join_arcs[i].clone()),
                OrientedEdge::forward(profile_edges[(i + 1) % 4].clone()),
                OrientedEdge::reversed(top_arcs[i].clone()),
                OrientedEdge::reversed(profile_edges[i].clone()),
            ]),
        ));
    }

    if rb > 1e-8 {
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
    }

    let top_plane = PlaneSurface3::new(
        Point3::new(0.0, 0.0, h),
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
