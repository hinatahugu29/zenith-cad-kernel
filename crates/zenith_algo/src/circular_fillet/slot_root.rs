//! Exact analytic blending of a slot (stadium) boss or pocket root.
//!
//! A slot profile consists of two parallel straight segments and two semicircular
//! arcs. The root blend against an orthogonal planar shoulder consists of:
//! - Exact quarter-cylinder patches along the straight segments.
//! - Exact quarter-torus patches along the semicircular arc segments.
//! - Perfect G1 continuity across the planar-to-radial transitions.

use std::f64::consts::{FRAC_1_SQRT_2, PI, TAU};

use zenith_geom::{ControlPoint3, KnotVector, NurbsCurve3, NurbsSurface3, PlaneSurface3};
use zenith_math::{Point3, Tolerance, Vec3, Vec3Ext};
use zenith_topo::{Edge, Face, FaceGeometry, OrientedEdge, Shell, Solid, Vertex, Wire};

use crate::{BlendableEdge, EdgeBlendReport};

pub(crate) fn try_fillet_slot_root(
    solid: &Solid,
    edge_id: u64,
    radius: f64,
) -> Result<Option<(Solid, EdgeBlendReport)>, String> {
    let Some(site) = SlotRoot::recognize(solid, edge_id) else {
        return Ok(None);
    };
    if !(radius > 1e-6) || !radius.is_finite() {
        return Err(format!(
            "Slot-root fillet radius must be finite and larger than 1e-6, got {radius}"
        ));
    }
    let margin = 1e-6 * site.max_radius.max(site.radius).max(1.0);
    if radius >= site.max_radius - margin {
        return Err(format!(
            "Slot-root fillet radius {radius} must be smaller than the available setback {:.6}",
            site.max_radius
        ));
    }
    let result = site.apply(solid, SlotRootBlend::Fillet(radius))?;
    Ok(Some((
        result,
        EdgeBlendReport {
            dihedral_angle_deg: 270.0,
            edge_length: 2.0 * site.length + TAU * site.radius,
            setback: radius,
            predicted_removed_volume: -site.fillet_added_volume(radius),
        },
    )))
}

pub(crate) fn try_chamfer_slot_root(
    solid: &Solid,
    edge_id: u64,
    distance: f64,
) -> Result<Option<(Solid, EdgeBlendReport)>, String> {
    let Some(site) = SlotRoot::recognize(solid, edge_id) else {
        return Ok(None);
    };
    if !(distance > 1e-6) || !distance.is_finite() {
        return Err(format!(
            "Slot-root chamfer distance must be finite and larger than 1e-6, got {distance}"
        ));
    }
    let margin = 1e-6 * site.max_chamfer_distance.max(site.radius).max(1.0);
    if distance >= site.max_chamfer_distance - margin {
        return Err(format!(
            "Slot-root chamfer distance {distance} must be smaller than the available setback {:.6}",
            site.max_chamfer_distance
        ));
    }
    let result = site.apply(solid, SlotRootBlend::Chamfer(distance))?;
    Ok(Some((
        result,
        EdgeBlendReport {
            dihedral_angle_deg: 270.0,
            edge_length: 2.0 * site.length + TAU * site.radius,
            setback: distance,
            predicted_removed_volume: -site.chamfer_added_volume(distance),
        },
    )))
}

pub(crate) fn slot_root_blendable(solid: &Solid, edge_id: u64) -> Option<BlendableEdge> {
    let site = SlotRoot::recognize(solid, edge_id)?;
    Some(BlendableEdge {
        edge_id,
        length: 2.0 * site.length + TAU * site.radius,
        dihedral_angle_deg: 270.0,
        max_fillet_radius: site.max_radius * 0.999,
        max_chamfer_distance: site.max_chamfer_distance * 0.999,
    })
}

#[derive(Clone, Copy)]
enum SlotRootBlend {
    Fillet(f64),
    Chamfer(f64),
}

struct SlotRoot {
    shoulder_index: usize,
    side_indices: Vec<usize>,
    shoulder_inner_index: usize,
    center: Point3,
    axis_z: Vec3,
    axis_x: Vec3,
    axis_y: Vec3,
    length: f64,
    radius: f64,
    height: f64,
    max_radius: f64,
    max_chamfer_distance: f64,
}

impl SlotRoot {
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

        // 平面肩 (Shoulder Plane) のインナーワイヤ内に選択稜があるか探す
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
        if root_wire.edges.len() != 6 {
            return None;
        }

        let normal = super::effective_plane_normal(shoulder).try_normalize_safe(1e-12)?;
        let mut straights = Vec::new();
        let mut arcs = Vec::new();

        for oriented in &root_wire.edges {
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

        // 側面の収集
        let mut side_indices = Vec::with_capacity(6);
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
            side_indices.push(side);
        }

        // 座標軸の決定
        let straight_dir = (straights[0].2 - straights[0].1).normalize();
        let axis_z = normal;
        let axis_x = straight_dir;
        let axis_y = axis_z.cross(&axis_x).normalize();
        let center = (straights[0].1 + straights[1].1.coords) * 0.5;

        // 高さの取得およびボス側面法線判定（中心から外向き (P - center) . N > 0）
        let side0 = &faces[side_indices[0]];
        let normal_side = face_sample_normal(side0);
        let pt_sample = match &side0.geometry {
            FaceGeometry::Plane(p) => p.origin,
            FaceGeometry::Nurbs(n) => n.control_points[0][0].point,
            _ => center,
        };
        let outward_from_center = (pt_sample - center).dot(&normal_side);
        if outward_from_center <= 1e-4 {
            return None;
        }

        let height = match &side0.geometry {
            FaceGeometry::Plane(p) => p.v_axis.norm(),
            FaceGeometry::Nurbs(n) => (n.control_points[0][1].point - n.control_points[0][0].point).norm(),
            _ => 10.0,
        };

        let max_radius = radius.min(height) * 0.8;
        let max_chamfer_distance = max_radius;

        Some(SlotRoot {
            shoulder_index,
            side_indices,
            shoulder_inner_index,
            center,
            axis_z,
            axis_x,
            axis_y,
            length,
            radius,
            height,
            max_radius,
            max_chamfer_distance,
        })
    }

    fn fillet_added_volume(&self, r: f64) -> f64 {
        let straight_vol = 2.0 * self.length * r * r * (1.0 - PI * 0.25);
        let circular_vol = PI * (self.radius * r * r * (2.0 - PI * 0.5) + r.powi(3) * (5.0 / 3.0 - PI * 0.5));
        straight_vol + circular_vol
    }

    fn chamfer_added_volume(&self, d: f64) -> f64 {
        let straight_vol = self.length * d * d;
        let circular_vol = PI * d * d * (self.radius + d / 3.0);
        straight_vol + circular_vol
    }

    fn apply(&self, solid: &Solid, blend: SlotRootBlend) -> Result<Solid, String> {
        let r_fillet = match blend {
            SlotRootBlend::Fillet(r) => r,
            SlotRootBlend::Chamfer(d) => d,
        };

        let l_half = self.length * 0.5;
        let r_in = self.radius;
        let r_out = self.radius + r_fillet;
        let h_join = r_fillet;
        let h_top = self.height;

        let weight = FRAC_1_SQRT_2;

        let pt_in = |local_x: f64, local_y: f64, local_z: f64| {
            self.center + self.axis_x * local_x + self.axis_y * local_y + self.axis_z * local_z
        };

        // ローカル (x, y) 座標
        let loc_b = [
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
        let pb: Vec<Point3> = loc_b.iter().map(|&(x, y)| pt_in(x, y, 0.0)).collect();
        let pj: Vec<Point3> = loc_in.iter().map(|&(x, y)| pt_in(x, y, h_join)).collect();
        let pt: Vec<Point3> = loc_in.iter().map(|&(x, y)| pt_in(x, y, h_top)).collect();

        let vb: Vec<Vertex> = pb.iter().map(|p| Vertex::from_point(*p)).collect();
        let vj: Vec<Vertex> = pj.iter().map(|p| Vertex::from_point(*p)).collect();
        let vt: Vec<Vertex> = pt.iter().map(|p| Vertex::from_point(*p)).collect();

        // 6本の縦直線エッジ (ジョイン -> トップ)
        let mut ev_upper = Vec::with_capacity(6);
        for i in 0..6 {
            ev_upper.push(Edge::line_between(vj[i].clone(), vt[i].clone())?);
        }

        // 6本のプロファイルエッジ (ジョイン -> ベース)
        let mut ev_blend = Vec::with_capacity(6);
        for i in 0..6 {
            let ctrl = match blend {
                SlotRootBlend::Fillet(_) => {
                    let (x, y) = loc_in[i];
                    pt_in(x, y, 0.0)
                }
                SlotRootBlend::Chamfer(_) => pt_in(0.0, 0.0, 0.0),
            };

            let edge = match blend {
                SlotRootBlend::Fillet(_) => {
                    let curve = NurbsCurve3::new(
                        2,
                        vec![
                            ControlPoint3::unweighted(pj[i]),
                            ControlPoint3::new(ctrl, weight),
                            ControlPoint3::unweighted(pb[i]),
                        ],
                        KnotVector::clamped_uniform(3, 2),
                    )?;
                    Edge::new(curve, vj[i].clone(), vb[i].clone(), 1e-6)
                }
                SlotRootBlend::Chamfer(_) => Edge::line_between(vj[i].clone(), vb[i].clone())?,
            };
            ev_blend.push(edge);
        }

        // 6本の下部エッジ、ジョインエッジ、上部エッジ
        let mut eb = Vec::with_capacity(6);
        let mut ej = Vec::with_capacity(6);
        let mut et = Vec::with_capacity(6);

        for i in 0..6 {
            let next = (i + 1) % 6;
            let is_arc = matches!(i, 1 | 2 | 4 | 5);

            let (edge_b, edge_j, edge_t) = if !is_arc {
                let edge_b = Edge::line_between(vb[i].clone(), vb[next].clone())?;
                let edge_j = Edge::line_between(vj[i].clone(), vj[next].clone())?;
                let edge_t = Edge::line_between(vt[i].clone(), vt[next].clone())?;
                (edge_b, edge_j, edge_t)
            } else {
                let ((cx_b, cy_b), (cx_j, cy_j), (cx_t, cy_t)) = match i {
                    1 => (
                        (l_half + r_out, -r_out),
                        (l_half + r_in, -r_in),
                        (l_half + r_in, -r_in),
                    ),
                    2 => (
                        (l_half + r_out, r_out),
                        (l_half + r_in, r_in),
                        (l_half + r_in, r_in),
                    ),
                    4 => (
                        (-l_half - r_out, r_out),
                        (-l_half - r_in, r_in),
                        (-l_half - r_in, r_in),
                    ),
                    5 => (
                        (-l_half - r_out, -r_out),
                        (-l_half - r_in, -r_in),
                        (-l_half - r_in, -r_in),
                    ),
                    _ => unreachable!(),
                };

                let corner_b = pt_in(cx_b, cy_b, 0.0);
                let corner_j = pt_in(cx_j, cy_j, h_join);
                let corner_t = pt_in(cx_t, cy_t, h_top);

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
                let arc_j = Edge::new(
                    NurbsCurve3::new(
                        2,
                        vec![
                            ControlPoint3::unweighted(pj[i]),
                            ControlPoint3::new(corner_j, weight),
                            ControlPoint3::unweighted(pj[next]),
                        ],
                        KnotVector::clamped_uniform(3, 2),
                    )?,
                    vj[i].clone(),
                    vj[next].clone(),
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
                (arc_b, arc_j, arc_t)
            };

            eb.push(edge_b);
            ej.push(edge_j);
            et.push(edge_t);
        }

        let mut new_faces = Vec::with_capacity(14);

        // 1. 短縮された側面 6枚 (ジョイン -> トップ)
        for i in 0..6 {
            let next = (i + 1) % 6;
            let is_arc = matches!(i, 1 | 2 | 4 | 5);

            let face_geom = if !is_arc {
                let u_axis = pj[next] - pj[i];
                let v_axis = pt[i] - pj[i];
                let plane = PlaneSurface3::new(pj[i], u_axis, v_axis)
                    .ok_or("Failed to create upper side plane")?;
                FaceGeometry::Plane(plane)
            } else {
                let (cx_j, cy_j) = match i {
                    1 => (l_half + r_in, -r_in),
                    2 => (l_half + r_in, r_in),
                    4 => (-l_half - r_in, r_in),
                    5 => (-l_half - r_in, -r_in),
                    _ => unreachable!(),
                };
                let corner_j = pt_in(cx_j, cy_j, h_join);
                let corner_t = pt_in(cx_j, cy_j, h_top);

                let surf = NurbsSurface3::new(
                    2,
                    1,
                    vec![
                        vec![
                            ControlPoint3::unweighted(pj[i]),
                            ControlPoint3::unweighted(pt[i]),
                        ],
                        vec![
                            ControlPoint3::new(corner_j, weight),
                            ControlPoint3::new(corner_t, weight),
                        ],
                        vec![
                            ControlPoint3::unweighted(pj[next]),
                            ControlPoint3::unweighted(pt[next]),
                        ],
                    ],
                    KnotVector::clamped_uniform(3, 2),
                    KnotVector::clamped_uniform(2, 1),
                )?;
                FaceGeometry::Nurbs(surf)
            };

            let wire = Wire::new(vec![
                OrientedEdge::forward(ej[i].clone()),
                OrientedEdge::forward(ev_upper[next].clone()),
                OrientedEdge::reversed(et[i].clone()),
                OrientedEdge::reversed(ev_upper[i].clone()),
            ]);
            new_faces.push(Face::simple(face_geom, wire));
        }

        // 2. ブレンドパッチ 6枚 (ベース -> ジョイン)
        for i in 0..6 {
            let next = (i + 1) % 6;
            let is_arc = matches!(i, 1 | 2 | 4 | 5);

            let face_geom = if !is_arc {
                let ctrl_i = pt_in(loc_in[i].0, loc_in[i].1, 0.0);
                let ctrl_next = pt_in(loc_in[next].0, loc_in[next].1, 0.0);

                let rows = match blend {
                    SlotRootBlend::Fillet(_) => vec![
                        vec![
                            ControlPoint3::unweighted(pj[i]),
                            ControlPoint3::unweighted(pj[next]),
                        ],
                        vec![
                            ControlPoint3::new(ctrl_i, weight),
                            ControlPoint3::new(ctrl_next, weight),
                        ],
                        vec![
                            ControlPoint3::unweighted(pb[i]),
                            ControlPoint3::unweighted(pb[next]),
                        ],
                    ],
                    SlotRootBlend::Chamfer(_) => vec![
                        vec![
                            ControlPoint3::unweighted(pj[i]),
                            ControlPoint3::unweighted(pj[next]),
                        ],
                        vec![
                            ControlPoint3::unweighted(pb[i]),
                            ControlPoint3::unweighted(pb[next]),
                        ],
                    ],
                };
                let deg_u = match blend {
                    SlotRootBlend::Fillet(_) => 2,
                    SlotRootBlend::Chamfer(_) => 1,
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
                let (cx_b, cy_b) = match i {
                    1 => (l_half + r_out, -r_out),
                    2 => (l_half + r_out, r_out),
                    4 => (-l_half - r_out, r_out),
                    5 => (-l_half - r_out, -r_out),
                    _ => unreachable!(),
                };
                let (cx_j, cy_j) = match i {
                    1 => (l_half + r_in, -r_in),
                    2 => (l_half + r_in, r_in),
                    4 => (-l_half - r_in, r_in),
                    5 => (-l_half - r_in, -r_in),
                    _ => unreachable!(),
                };

                let corner_b = pt_in(cx_b, cy_b, 0.0);
                let corner_j = pt_in(cx_j, cy_j, h_join);
                let corner_ctrl = pt_in(cx_j, cy_j, 0.0);

                let ctrl_i = pt_in(loc_in[i].0, loc_in[i].1, 0.0);
                let ctrl_next = pt_in(loc_in[next].0, loc_in[next].1, 0.0);

                let rows = match blend {
                    SlotRootBlend::Fillet(_) => vec![
                        vec![
                            ControlPoint3::unweighted(pj[i]),
                            ControlPoint3::new(corner_j, weight),
                            ControlPoint3::unweighted(pj[next]),
                        ],
                        vec![
                            ControlPoint3::new(ctrl_i, weight),
                            ControlPoint3::new(corner_ctrl, weight * weight),
                            ControlPoint3::new(ctrl_next, weight),
                        ],
                        vec![
                            ControlPoint3::unweighted(pb[i]),
                            ControlPoint3::new(corner_b, weight),
                            ControlPoint3::unweighted(pb[next]),
                        ],
                    ],
                    SlotRootBlend::Chamfer(_) => vec![
                        vec![
                            ControlPoint3::unweighted(pj[i]),
                            ControlPoint3::new(corner_j, weight),
                            ControlPoint3::unweighted(pj[next]),
                        ],
                        vec![
                            ControlPoint3::unweighted(pb[i]),
                            ControlPoint3::new(corner_b, weight),
                            ControlPoint3::unweighted(pb[next]),
                        ],
                    ],
                };
                let deg_u = match blend {
                    SlotRootBlend::Fillet(_) => 2,
                    SlotRootBlend::Chamfer(_) => 1,
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
                OrientedEdge::forward(ev_blend[i].clone()),
                OrientedEdge::forward(eb[i].clone()),
                OrientedEdge::reversed(ev_blend[next].clone()),
                OrientedEdge::reversed(ej[i].clone()),
            ]);
            new_faces.push(Face::simple(face_geom, wire));
        }

        // 3. 肩面の更新
        let new_shoulder_wire = Wire::new(vec![
            OrientedEdge::reversed(eb[5].clone()),
            OrientedEdge::reversed(eb[4].clone()),
            OrientedEdge::reversed(eb[3].clone()),
            OrientedEdge::reversed(eb[2].clone()),
            OrientedEdge::reversed(eb[1].clone()),
            OrientedEdge::reversed(eb[0].clone()),
        ]);

        let mut final_faces = Vec::new();
        for (idx, face) in solid.outer_shell.faces.iter().enumerate() {
            if self.side_indices.contains(&idx) {
                continue;
            }
            if idx == self.shoulder_index {
                let mut inners = face.inner_wires.clone();
                inners[self.shoulder_inner_index] = new_shoulder_wire.clone();
                let mut updated_face = Face::new(
                    face.geometry.clone(),
                    face.outer_wire.clone(),
                    inners,
                    face.orientation,
                    face.tolerance,
                );
                updated_face.id = face.id;
                final_faces.push(updated_face);
            } else {
                final_faces.push(face.clone());
            }
        }

        final_faces.extend(new_faces);
        Solid::try_new(
            Shell::closed(final_faces),
            solid.inner_shells.clone(),
            &Tolerance::default(),
        )
        .map_err(|error| format!("Slot-root blend produced an invalid solid: {error}"))
    }
}

fn all_wires(face: &Face) -> impl Iterator<Item = &Wire> {
    std::iter::once(&face.outer_wire).chain(face.inner_wires.iter())
}

fn face_sample_normal(face: &Face) -> Vec3 {
    match &face.geometry {
        FaceGeometry::Plane(p) => {
            if face.orientation == zenith_topo::Orientation::Forward {
                p.normal
            } else {
                -p.normal
            }
        }
        FaceGeometry::Nurbs(n) => {
            let ((u0, u1), (v0, v1)) = n.param_range();
            let u = (u0 + u1) * 0.5;
            let v = (v0 + v1) * 0.5;
            let n_vec = n.normal(u, v).unwrap_or(Vec3::new(0.0, 0.0, 1.0));
            if face.orientation == zenith_topo::Orientation::Forward {
                n_vec
            } else {
                -n_vec
            }
        }
        _ => Vec3::new(0.0, 0.0, 1.0),
    }
}
