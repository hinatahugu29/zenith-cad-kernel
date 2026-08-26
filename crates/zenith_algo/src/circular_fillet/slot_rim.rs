//! Exact analytic blending for the top rim of a slot column (stadium prism).
//!
//! A slot top rim consists of two straight convex edges and four quarter-circular
//! convex arc edges meeting an orthogonal top plane. The exact blend is:
//! - Quarter-cylinder patches along the straight segments.
//! - Quarter-torus patches along the semicircular arc segments.
//! - G1 smooth watertight continuity across all seams.

use std::f64::consts::{FRAC_1_SQRT_2, PI, TAU};

use zenith_geom::{ControlPoint3, KnotVector, NurbsCurve3, NurbsSurface3, PlaneSurface3};
use zenith_math::{Point3, Tolerance, Vec3, Vec3Ext};
use zenith_topo::{Edge, Face, FaceGeometry, OrientedEdge, Shell, Solid, Vertex, Wire};

use crate::{BlendableEdge, EdgeBlendReport};

pub(crate) fn try_fillet_slot_rim(
    solid: &Solid,
    edge_id: u64,
    radius: f64,
) -> Result<Option<(Solid, EdgeBlendReport)>, String> {
    let Some(site) = SlotRim::recognize(solid, edge_id) else {
        return Ok(None);
    };
    if !(radius > 1e-6) || !radius.is_finite() {
        return Err(format!(
            "Slot-rim fillet radius must be finite and larger than 1e-6, got {radius}"
        ));
    }
    if radius >= site.radius || radius >= site.height {
        return Err(format!(
            "Slot-rim fillet radius {radius} must be smaller than radius {:.6} and height {:.6}",
            site.radius, site.height
        ));
    }
    let result = site.apply(solid, SlotRimBlend::Fillet(radius))?;
    Ok(Some((
        result,
        EdgeBlendReport {
            dihedral_angle_deg: 90.0,
            edge_length: 2.0 * site.length + TAU * site.radius,
            setback: radius,
            predicted_removed_volume: site.fillet_removed_volume(radius),
        },
    )))
}

pub(crate) fn try_chamfer_slot_rim(
    solid: &Solid,
    edge_id: u64,
    distance: f64,
) -> Result<Option<(Solid, EdgeBlendReport)>, String> {
    let Some(site) = SlotRim::recognize(solid, edge_id) else {
        return Ok(None);
    };
    if !(distance > 1e-6) || !distance.is_finite() {
        return Err(format!(
            "Slot-rim chamfer distance must be finite and larger than 1e-6, got {distance}"
        ));
    }
    if distance >= site.radius || distance >= site.height {
        return Err(format!(
            "Slot-rim chamfer distance {distance} must be smaller than radius {:.6} and height {:.6}",
            site.radius, site.height
        ));
    }
    let result = site.apply(solid, SlotRimBlend::Chamfer(distance))?;
    Ok(Some((
        result,
        EdgeBlendReport {
            dihedral_angle_deg: 90.0,
            edge_length: 2.0 * site.length + TAU * site.radius,
            setback: distance,
            predicted_removed_volume: site.chamfer_removed_volume(distance),
        },
    )))
}

pub(crate) fn slot_rim_blendable(solid: &Solid, edge_id: u64) -> Option<BlendableEdge> {
    let site = SlotRim::recognize(solid, edge_id)?;
    Some(BlendableEdge {
        edge_id,
        length: 2.0 * site.length + TAU * site.radius,
        dihedral_angle_deg: 90.0,
        max_fillet_radius: site.radius.min(site.height) * 0.999,
        max_chamfer_distance: site.radius.min(site.height) * 0.999,
    })
}

#[derive(Clone, Copy)]
enum SlotRimBlend {
    Fillet(f64),
    Chamfer(f64),
}

struct SlotRim {
    top_index: usize,
    side_indices: Vec<usize>,
    center: Point3,
    axis_z: Vec3,
    axis_x: Vec3,
    axis_y: Vec3,
    length: f64,
    radius: f64,
    height: f64,
}

impl SlotRim {
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

        // 天面 (Top Plane) のアウターワイヤ内に選択稜があるか探す
        let top_index = faces.iter().position(|face| {
            if !matches!(face.geometry, FaceGeometry::Plane(_)) {
                return false;
            }
            face.outer_wire
                .edges
                .iter()
                .any(|edge| super::same_edge_geometry(&selected, &edge.edge))
        })?;

        let top_face = &faces[top_index];
        let rim_wire = &top_face.outer_wire;
        if rim_wire.edges.len() != 6 {
            return None;
        }

        let normal = super::effective_plane_normal(top_face).try_normalize_safe(1e-12)?;
        let mut straights = Vec::new();
        let mut arcs = Vec::new();

        for oriented in &rim_wire.edges {
            if let Some((center, r, _)) = super::circle_from_edge(&oriented.edge) {
                arcs.push((oriented.clone(), center, r));
            } else {
                let start = oriented.edge.start_vertex.point;
                let end = oriented.edge.end_vertex.point;
                let span = end - start;
                straights.push((oriented.clone(), start, end, span.norm()));
            }
        }

        if straights.len() != 2 || arcs.len() != 4 {
            return None;
        }

        let length = straights[0].3;
        if (straights[1].3 - length).abs() > 1e-4 * length.max(1.0) {
            return None;
        }

        let radius = arcs[0].2;
        if arcs.iter().any(|(_, _, r)| (*r - radius).abs() > 1e-4 * radius.max(1.0)) {
            return None;
        }

        // 側面 6面の収集
        let mut side_indices = Vec::with_capacity(6);
        for rim_edge in &rim_wire.edges {
            let users: Vec<usize> = faces
                .iter()
                .enumerate()
                .filter(|(_, face)| {
                    all_wires(face).any(|wire| {
                        wire.edges.iter().any(|candidate| {
                            super::same_edge_geometry(&rim_edge.edge, &candidate.edge)
                        })
                    })
                })
                .map(|(index, _)| index)
                .collect();
            if users.len() != 2 || !users.contains(&top_index) {
                return None;
            }
            let side = *users.iter().find(|index| **index != top_index)?;
            side_indices.push(side);
        }

        let straight_dir = (straights[0].2 - straights[0].1).normalize();
        let axis_z = normal;
        let axis_x = straight_dir;
        let axis_y = axis_z.cross(&axis_x).normalize();
        let center = (straights[0].1 + straights[1].1.coords) * 0.5;

        let side0 = &faces[side_indices[0]];
        let height = match &side0.geometry {
            FaceGeometry::Plane(p) => p.v_axis.norm(),
            FaceGeometry::Nurbs(n) => (n.control_points[0][1].point - n.control_points[0][0].point).norm(),
            _ => 10.0,
        };

        Some(SlotRim {
            top_index,
            side_indices,
            center,
            axis_z,
            axis_x,
            axis_y,
            length,
            radius,
            height,
        })
    }

    fn fillet_removed_volume(&self, r: f64) -> f64 {
        let straight_vol = 2.0 * self.length * r * r * (1.0 - PI * 0.25);
        let circular_vol = PI * ((self.radius - r) * r * r * (2.0 - PI * 0.5) + r.powi(3) / 3.0);
        straight_vol + circular_vol
    }

    fn chamfer_removed_volume(&self, d: f64) -> f64 {
        let straight_vol = self.length * d * d;
        let circular_vol = PI * d * d * (self.radius - d / 3.0);
        straight_vol + circular_vol
    }

    fn apply(&self, solid: &Solid, blend: SlotRimBlend) -> Result<Solid, String> {
        let r_fillet = match blend {
            SlotRimBlend::Fillet(r) => r,
            SlotRimBlend::Chamfer(d) => d,
        };

        let l_half = self.length * 0.5;
        let r_out = self.radius;
        let r_in = self.radius - r_fillet;
        let h_base = 0.0;
        let h_cut = self.height - r_fillet;
        let h_top = self.height;

        let weight = FRAC_1_SQRT_2;

        let pt_in = |local_x: f64, local_y: f64, local_z: f64| {
            // center は天面 (z=H) にあるので、local_z は天面からの相対高さ (z - H)
            self.center + self.axis_x * local_x + self.axis_y * local_y + self.axis_z * (local_z - h_top)
        };

        // ローカル (x, y) 座標
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
        let pb: Vec<Point3> = loc_out.iter().map(|&(x, y)| pt_in(x, y, h_base)).collect();
        let pc: Vec<Point3> = loc_out.iter().map(|&(x, y)| pt_in(x, y, h_cut)).collect();
        let pt: Vec<Point3> = loc_in.iter().map(|&(x, y)| pt_in(x, y, h_top)).collect();

        let vb: Vec<Vertex> = pb.iter().map(|p| Vertex::from_point(*p)).collect();
        let vc: Vec<Vertex> = pc.iter().map(|p| Vertex::from_point(*p)).collect();
        let vt: Vec<Vertex> = pt.iter().map(|p| Vertex::from_point(*p)).collect();

        // 6本の縦直線エッジ (ベース -> カット)
        let mut ev_lower = Vec::with_capacity(6);
        for i in 0..6 {
            ev_lower.push(Edge::line_between(vb[i].clone(), vc[i].clone())?);
        }

        // 6本のプロファイルエッジ (カット -> トップ)
        let mut ev_blend = Vec::with_capacity(6);
        for i in 0..6 {
            let ctrl = match blend {
                SlotRimBlend::Fillet(_) => {
                    let (x, y) = loc_out[i];
                    pt_in(x, y, h_top)
                }
                SlotRimBlend::Chamfer(_) => pt_in(0.0, 0.0, 0.0),
            };

            let edge = match blend {
                SlotRimBlend::Fillet(_) => {
                    let curve = NurbsCurve3::new(
                        2,
                        vec![
                            ControlPoint3::unweighted(pc[i]),
                            ControlPoint3::new(ctrl, weight),
                            ControlPoint3::unweighted(pt[i]),
                        ],
                        KnotVector::clamped_uniform(3, 2),
                    )?;
                    Edge::new(curve, vc[i].clone(), vt[i].clone(), 1e-6)
                }
                SlotRimBlend::Chamfer(_) => Edge::line_between(vc[i].clone(), vt[i].clone())?,
            };
            ev_blend.push(edge);
        }

        // 6本の下部エッジ、カットエッジ、天面エッジ
        let mut eb = Vec::with_capacity(6);
        let mut ec = Vec::with_capacity(6);
        let mut et = Vec::with_capacity(6);

        for i in 0..6 {
            let next = (i + 1) % 6;
            let is_arc = matches!(i, 1 | 2 | 4 | 5);

            let (edge_b, edge_c, edge_t) = if !is_arc {
                let edge_b = Edge::line_between(vb[i].clone(), vb[next].clone())?;
                let edge_c = Edge::line_between(vc[i].clone(), vc[next].clone())?;
                let edge_t = Edge::line_between(vt[i].clone(), vt[next].clone())?;
                (edge_b, edge_c, edge_t)
            } else {
                let ((cx_out, cy_out), (cx_in, cy_in)) = match i {
                    1 => (
                        (l_half + r_out, -r_out),
                        (l_half + r_in, -r_in),
                    ),
                    2 => (
                        (l_half + r_out, r_out),
                        (l_half + r_in, r_in),
                    ),
                    4 => (
                        (-l_half - r_out, r_out),
                        (-l_half - r_in, r_in),
                    ),
                    5 => (
                        (-l_half - r_out, -r_out),
                        (-l_half - r_in, -r_in),
                    ),
                    _ => unreachable!(),
                };

                let corner_b = pt_in(cx_out, cy_out, h_base);
                let corner_c = pt_in(cx_out, cy_out, h_cut);
                let corner_t = pt_in(cx_in, cy_in, h_top);

                let arc_b = Edge::new(
                    NurbsCurve3::new(
                        2,
                        vec![
                            ControlPoint3::unweighted(pb[i]),
                            ControlPoint3::new(corner_b, weight),
                            ControlPoint3::unweighted(pb[next]),
                        ],
                        KnotVector::clamped_uniform(3, 2),
                    )?,
                    vb[i].clone(),
                    vb[next].clone(),
                    1e-6,
                );
                let arc_c = Edge::new(
                    NurbsCurve3::new(
                        2,
                        vec![
                            ControlPoint3::unweighted(pc[i]),
                            ControlPoint3::new(corner_c, weight),
                            ControlPoint3::unweighted(pc[next]),
                        ],
                        KnotVector::clamped_uniform(3, 2),
                    )?,
                    vc[i].clone(),
                    vc[next].clone(),
                    1e-6,
                );
                let arc_t = Edge::new(
                    NurbsCurve3::new(
                        2,
                        vec![
                            ControlPoint3::unweighted(pt[i]),
                            ControlPoint3::new(corner_t, weight),
                            ControlPoint3::unweighted(pt[next]),
                        ],
                        KnotVector::clamped_uniform(3, 2),
                    )?,
                    vt[i].clone(),
                    vt[next].clone(),
                    1e-6,
                );
                (arc_b, arc_c, arc_t)
            };

            eb.push(edge_b);
            ec.push(edge_c);
            et.push(edge_t);
        }

        let mut new_faces = Vec::with_capacity(14);

        // 1. 短縮された側面 6枚 (ベース -> カット)
        for i in 0..6 {
            let next = (i + 1) % 6;
            let is_arc = matches!(i, 1 | 2 | 4 | 5);

            let face_geom = if !is_arc {
                let u_axis = pb[next] - pb[i];
                let v_axis = pc[i] - pb[i];
                let plane = PlaneSurface3::new(pb[i], u_axis, v_axis)
                    .ok_or("Failed to create side plane")?;
                FaceGeometry::Plane(plane)
            } else {
                let (cx_out, cy_out) = match i {
                    1 => (l_half + r_out, -r_out),
                    2 => (l_half + r_out, r_out),
                    4 => (-l_half - r_out, r_out),
                    5 => (-l_half - r_out, -r_out),
                    _ => unreachable!(),
                };
                let corner_b = pt_in(cx_out, cy_out, h_base);
                let corner_c = pt_in(cx_out, cy_out, h_cut);

                let surf = NurbsSurface3::new(
                    2,
                    1,
                    vec![
                        vec![
                            ControlPoint3::unweighted(pb[i]),
                            ControlPoint3::unweighted(pc[i]),
                        ],
                        vec![
                            ControlPoint3::new(corner_b, weight),
                            ControlPoint3::new(corner_c, weight),
                        ],
                        vec![
                            ControlPoint3::unweighted(pb[next]),
                            ControlPoint3::unweighted(pc[next]),
                        ],
                    ],
                    KnotVector::clamped_uniform(3, 2),
                    KnotVector::clamped_uniform(2, 1),
                )?;
                FaceGeometry::Nurbs(surf)
            };

            let wire = Wire::new(vec![
                OrientedEdge::forward(eb[i].clone()),
                OrientedEdge::forward(ev_lower[next].clone()),
                OrientedEdge::reversed(ec[i].clone()),
                OrientedEdge::reversed(ev_lower[i].clone()),
            ]);
            new_faces.push(Face::simple(face_geom, wire));
        }

        // 2. ブレンドパッチ 6枚 (カット -> トップ)
        for i in 0..6 {
            let next = (i + 1) % 6;
            let is_arc = matches!(i, 1 | 2 | 4 | 5);

            let face_geom = if !is_arc {
                let ctrl_i = pt_in(loc_out[i].0, loc_out[i].1, h_top);
                let ctrl_next = pt_in(loc_out[next].0, loc_out[next].1, h_top);

                let mut rows = match blend {
                    SlotRimBlend::Fillet(_) => vec![
                        vec![
                            ControlPoint3::unweighted(pc[i]),
                            ControlPoint3::unweighted(pc[next]),
                        ],
                        vec![
                            ControlPoint3::new(ctrl_i, weight),
                            ControlPoint3::new(ctrl_next, weight),
                        ],
                        vec![
                            ControlPoint3::unweighted(pt[i]),
                            ControlPoint3::unweighted(pt[next]),
                        ],
                    ],
                    SlotRimBlend::Chamfer(_) => vec![
                        vec![
                            ControlPoint3::unweighted(pc[i]),
                            ControlPoint3::unweighted(pc[next]),
                        ],
                        vec![
                            ControlPoint3::unweighted(pt[i]),
                            ControlPoint3::unweighted(pt[next]),
                        ],
                    ],
                };
                rows.reverse();
                let deg_u = match blend {
                    SlotRimBlend::Fillet(_) => 2,
                    SlotRimBlend::Chamfer(_) => 1,
                };
                let surf = NurbsSurface3::new(
                    deg_u,
                    1,
                    rows,
                    KnotVector::clamped_uniform(deg_u + 1, deg_u),
                    KnotVector::clamped_uniform(2, 1),
                )?;
                FaceGeometry::Nurbs(surf)
            } else {
                let (cx_out, cy_out) = match i {
                    1 => (l_half + r_out, -r_out),
                    2 => (l_half + r_out, r_out),
                    4 => (-l_half - r_out, r_out),
                    5 => (-l_half - r_out, -r_out),
                    _ => unreachable!(),
                };
                let (cx_in, cy_in) = match i {
                    1 => (l_half + r_in, -r_in),
                    2 => (l_half + r_in, r_in),
                    4 => (-l_half - r_in, r_in),
                    5 => (-l_half - r_in, -r_in),
                    _ => unreachable!(),
                };

                let corner_c = pt_in(cx_out, cy_out, h_cut);
                let corner_t = pt_in(cx_in, cy_in, h_top);
                let corner_ctrl = pt_in(cx_out, cy_out, h_top);

                let ctrl_i = pt_in(loc_out[i].0, loc_out[i].1, h_top);
                let ctrl_next = pt_in(loc_out[next].0, loc_out[next].1, h_top);

                let mut rows = match blend {
                    SlotRimBlend::Fillet(_) => vec![
                        vec![
                            ControlPoint3::unweighted(pc[i]),
                            ControlPoint3::new(corner_c, weight),
                            ControlPoint3::unweighted(pc[next]),
                        ],
                        vec![
                            ControlPoint3::new(ctrl_i, weight),
                            ControlPoint3::new(corner_ctrl, weight * weight),
                            ControlPoint3::new(ctrl_next, weight),
                        ],
                        vec![
                            ControlPoint3::unweighted(pt[i]),
                            ControlPoint3::new(corner_t, weight),
                            ControlPoint3::unweighted(pt[next]),
                        ],
                    ],
                    SlotRimBlend::Chamfer(_) => vec![
                        vec![
                            ControlPoint3::unweighted(pc[i]),
                            ControlPoint3::new(corner_c, weight),
                            ControlPoint3::unweighted(pc[next]),
                        ],
                        vec![
                            ControlPoint3::unweighted(pt[i]),
                            ControlPoint3::new(corner_t, weight),
                            ControlPoint3::unweighted(pt[next]),
                        ],
                    ],
                };
                rows.reverse();
                let deg_u = match blend {
                    SlotRimBlend::Fillet(_) => 2,
                    SlotRimBlend::Chamfer(_) => 1,
                };
                let surf = NurbsSurface3::new(
                    deg_u,
                    2,
                    rows,
                    KnotVector::clamped_uniform(deg_u + 1, deg_u),
                    KnotVector::clamped_uniform(3, 2),
                )?;
                FaceGeometry::Nurbs(surf)
            };

            let wire = Wire::new(vec![
                OrientedEdge::forward(ec[i].clone()),
                OrientedEdge::forward(ev_blend[next].clone()),
                OrientedEdge::reversed(et[i].clone()),
                OrientedEdge::reversed(ev_blend[i].clone()),
            ]);
            new_faces.push(Face::simple(face_geom, wire));
        }

        // 3. 短縮された天面 (正順 0..5)
        let plane_t = PlaneSurface3::new(
            pt_in(0.0, 0.0, h_top),
            self.axis_x,
            self.axis_y,
        )
        .ok_or("Failed to create top plane")?;
        let wire_t = Wire::new(vec![
            OrientedEdge::forward(et[0].clone()),
            OrientedEdge::forward(et[1].clone()),
            OrientedEdge::forward(et[2].clone()),
            OrientedEdge::forward(et[3].clone()),
            OrientedEdge::forward(et[4].clone()),
            OrientedEdge::forward(et[5].clone()),
        ]);
        new_faces.push(Face::simple(FaceGeometry::Plane(plane_t), wire_t));

        let mut final_faces = Vec::new();
        for (idx, face) in solid.outer_shell.faces.iter().enumerate() {
            if idx == self.top_index || self.side_indices.contains(&idx) {
                continue;
            }
            final_faces.push(face.clone());
        }

        final_faces.extend(new_faces);
        let raw_solid = Solid::try_new(
            Shell::closed(final_faces),
            solid.inner_shells.clone(),
            &Tolerance::default(),
        )
        .map_err(|error| format!("Slot-rim blend produced an invalid solid: {error}"))?;

        let (sewn, _) = crate::Sewer::sew_solid(&raw_solid, &Tolerance::default())
            .map_err(|error| format!("Slot-rim blend sewing failed: {error}"))?;
        Ok(sewn)
    }
}

fn all_wires(face: &Face) -> impl Iterator<Item = &Wire> {
    std::iter::once(&face.outer_wire).chain(face.inner_wires.iter())
}
