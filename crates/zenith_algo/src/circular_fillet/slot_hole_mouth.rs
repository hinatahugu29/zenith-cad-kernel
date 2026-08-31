//! Exact local blending of a stadium slot through-hole mouth.
//!
//! A slot through-hole consists of a 6-edge inner wire (2 straight segments and
//! 4 quarter-circular arc segments) on a planar cap face, passing through to an
//! opposite planar face. The exact blend is:
//! - Retrimmed planar inner wire on the selected cap face.
//! - Shortened 6 bore faces (2 planes + 4 quarter-cylinders).
//! - 6 blend patches (2 planar/cylindrical patches along the straight segments +
//!   4 conical/toroidal patches along the curved corners).
//! - Retrimmed opposite planar face using the matched bottom wire.
//! - Watertight G1 continuity across all seams.

use std::f64::consts::{FRAC_1_SQRT_2, PI, TAU};

use zenith_geom::{ControlPoint3, KnotVector, NurbsCurve3, NurbsSurface3, PlaneSurface3};
use zenith_math::{Point3, Tolerance, Vec3, Vec3Ext};
use zenith_topo::{Edge, Face, FaceGeometry, OrientedEdge, Shell, Solid, Vertex, Wire};

use crate::{BlendableEdge, EdgeBlendReport};

pub(crate) fn try_fillet_slot_hole_mouth(
    solid: &Solid,
    edge_id: u64,
    radius: f64,
) -> Result<Option<(Solid, EdgeBlendReport)>, String> {
    let Some(site) = SlotHoleMouth::recognize(solid, edge_id) else {
        return Ok(None);
    };
    if !(radius > 1e-6) || !radius.is_finite() {
        return Err(format!(
            "Slot-hole fillet radius must be finite and positive, got {radius}"
        ));
    }
    if radius >= site.depth {
        return Err(format!(
            "Slot-hole fillet radius {radius} must be smaller than hole depth {:.6}",
            site.depth
        ));
    }
    let result = site.apply(solid, SlotHoleBlend::Fillet(radius))?;
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

pub(crate) fn try_chamfer_slot_hole_mouth(
    solid: &Solid,
    edge_id: u64,
    distance: f64,
) -> Result<Option<(Solid, EdgeBlendReport)>, String> {
    let Some(site) = SlotHoleMouth::recognize(solid, edge_id) else {
        return Ok(None);
    };
    if !(distance > 1e-6) || !distance.is_finite() {
        return Err(format!(
            "Slot-hole chamfer distance must be finite and positive, got {distance}"
        ));
    }
    if distance >= site.depth {
        return Err(format!(
            "Slot-hole chamfer distance {distance} must be smaller than hole depth {:.6}",
            site.depth
        ));
    }
    let result = site.apply(solid, SlotHoleBlend::Chamfer(distance))?;
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

pub(crate) fn slot_hole_mouth_blendable(solid: &Solid, edge_id: u64) -> Option<BlendableEdge> {
    let site = SlotHoleMouth::recognize(solid, edge_id)?;
    Some(BlendableEdge {
        edge_id,
        length: 2.0 * site.length + TAU * site.radius,
        dihedral_angle_deg: 90.0,
        max_fillet_radius: site.depth * 0.999,
        max_chamfer_distance: site.depth * 0.999,
    })
}

#[derive(Clone, Copy)]
enum SlotHoleBlend {
    Fillet(f64),
    Chamfer(f64),
}

struct SlotHoleMouth {
    cap_index: usize,
    inner_wire_index: usize,
    opposite_cap_index: usize,
    opposite_inner_index: usize,
    bore_indices: Vec<usize>,
    center: Point3,
    axis_z: Vec3,
    axis_x: Vec3,
    axis_y: Vec3,
    length: f64,
    radius: f64,
    depth: f64,
}

impl SlotHoleMouth {
    fn recognize(solid: &Solid, edge_id: u64) -> Option<Self> {
        let faces = &solid.outer_shell.faces;
        let _selected = faces
            .iter()
            .flat_map(all_wires)
            .flat_map(|wire| wire.edges.iter())
            .find(|oriented| oriented.edge.id == edge_id)?
            .edge
            .clone();

        // 1. 平面キャップのインナーワイヤ（穴ループ）内に選択稜があるか探す
        let (cap_index, inner_wire_index) = faces.iter().enumerate().find_map(|(fi, face)| {
            if !matches!(face.geometry, FaceGeometry::Plane(_)) {
                return None;
            }
            face.inner_wires.iter().enumerate().find_map(|(wi, wire)| {
                wire.edges
                    .iter()
                    .any(|edge| edge.edge.id == edge_id)
                    .then_some((fi, wi))
            })
        })?;

        let cap_face = &faces[cap_index];
        let mouth_wire = &cap_face.inner_wires[inner_wire_index];
        if mouth_wire.edges.len() != 6 {
            return None;
        }

        let normal = super::effective_plane_normal(cap_face).try_normalize_safe(1e-12)?;
        let mut straights = Vec::new();
        let mut arcs = Vec::new();

        for oriented in &mouth_wire.edges {
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
        if arcs
            .iter()
            .any(|(_, _, r)| (*r - radius).abs() > 1e-4 * radius.max(1.0))
        {
            return None;
        }

        // 側面 6面の収集
        let mut bore_indices = Vec::with_capacity(6);
        for mouth_edge in &mouth_wire.edges {
            let users: Vec<usize> = faces
                .iter()
                .enumerate()
                .filter(|(_, face)| {
                    all_wires(face).any(|wire| {
                        wire.edges.iter().any(|candidate| {
                            candidate.edge.id == mouth_edge.edge.id
                                || super::same_edge_geometry(&mouth_edge.edge, &candidate.edge)
                        })
                    })
                })
                .map(|(index, _)| index)
                .collect();
            if users.len() != 2 || !users.contains(&cap_index) {
                return None;
            }
            let bore = *users.iter().find(|index| **index != cap_index)?;
            bore_indices.push(bore);
        }

        let straight_dir = (straights[0].2 - straights[0].1).normalize();
        let axis_z = normal;
        let axis_x = straight_dir;
        let axis_y = axis_z.cross(&axis_x).normalize();
        let center = (straights[0].1 + straights[1].1.coords) * 0.5;

        // 反対側のキャップ面（opposite_cap）の検出
        let (opposite_cap_index, opposite_inner_index) =
            faces.iter().enumerate().find_map(|(fi, face)| {
                if fi == cap_index || !matches!(face.geometry, FaceGeometry::Plane(_)) {
                    return None;
                }
                let opp_normal = face_sample_normal(face);
                if opp_normal.dot(&axis_z) > -0.8 {
                    return None;
                }
                face.inner_wires
                    .iter()
                    .enumerate()
                    .find_map(|(wi, wire)| (wire.edges.len() == 6).then_some((fi, wi)))
            })?;

        // 穴の正確な深さ（天面から底面までの距離）
        let opp_face = &faces[opposite_cap_index];
        let opp_wire = &opp_face.inner_wires[opposite_inner_index];
        let opp_sample_pt = opp_wire.edges[0].edge.start_vertex.point;
        let depth = (center - opp_sample_pt).dot(&axis_z);
        if depth <= 1e-4 {
            return None;
        }

        Some(SlotHoleMouth {
            cap_index,
            inner_wire_index,
            opposite_cap_index,
            opposite_inner_index,
            bore_indices,
            center,
            axis_z,
            axis_x,
            axis_y,
            length,
            radius,
            depth,
        })
    }

    fn fillet_removed_volume(&self, r: f64) -> f64 {
        let straight_vol = 2.0 * self.length * r * r * (1.0 - PI * 0.25);
        let circular_vol =
            PI * (self.radius * r * r * (2.0 - PI * 0.5) + r.powi(3) * (5.0 / 3.0 - PI * 0.5));
        straight_vol + circular_vol
    }

    fn chamfer_removed_volume(&self, d: f64) -> f64 {
        let straight_vol = self.length * d * d;
        let circular_vol = PI * d * d * (self.radius + d / 3.0);
        straight_vol + circular_vol
    }

    fn apply(&self, solid: &Solid, blend: SlotHoleBlend) -> Result<Solid, String> {
        let setback = match blend {
            SlotHoleBlend::Fillet(r) => r,
            SlotHoleBlend::Chamfer(d) => d,
        };

        let l_half = self.length * 0.5;
        let r_in = self.radius;
        let r_out = self.radius + setback;
        let h_mouth = 0.0;
        let h_cut = -setback;
        let h_bottom = -self.depth;

        let weight = FRAC_1_SQRT_2;

        let pt_in = |local_x: f64, local_y: f64, local_z: f64| {
            self.center + self.axis_x * local_x + self.axis_y * local_y + self.axis_z * local_z
        };

        // ローカル (x, y) 座標
        let loc_in = [
            (-l_half, -r_in),
            (l_half, -r_in),
            (l_half + r_in, 0.0),
            (l_half, r_in),
            (-l_half, r_in),
            (-l_half - r_in, 0.0),
        ];

        let loc_out = [
            (-l_half, -r_out),
            (l_half, -r_out),
            (l_half + r_out, 0.0),
            (l_half, r_out),
            (-l_half, r_out),
            (-l_half - r_out, 0.0),
        ];

        // 3D 頂点
        let pm: Vec<Point3> = loc_out.iter().map(|&(x, y)| pt_in(x, y, h_mouth)).collect();
        let pc: Vec<Point3> = loc_in.iter().map(|&(x, y)| pt_in(x, y, h_cut)).collect();
        let pb: Vec<Point3> = loc_in.iter().map(|&(x, y)| pt_in(x, y, h_bottom)).collect();

        let vm: Vec<Vertex> = pm.iter().map(|p| Vertex::from_point(*p)).collect();
        let vc: Vec<Vertex> = pc.iter().map(|p| Vertex::from_point(*p)).collect();
        let vb: Vec<Vertex> = pb.iter().map(|p| Vertex::from_point(*p)).collect();

        // 6本の縦直線エッジ (カット -> ボトム)
        let mut ev_lower = Vec::with_capacity(6);
        for i in 0..6 {
            ev_lower.push(Edge::line_between(vc[i].clone(), vb[i].clone())?);
        }

        // 6本のプロファイルエッジ (マウス -> カット)
        let mut ev_blend = Vec::with_capacity(6);
        for i in 0..6 {
            let ctrl = match blend {
                SlotHoleBlend::Fillet(_) => {
                    let (x, y) = loc_in[i];
                    pt_in(x, y, h_mouth)
                }
                SlotHoleBlend::Chamfer(_) => pt_in(0.0, 0.0, 0.0),
            };

            let edge = match blend {
                SlotHoleBlend::Fillet(_) => {
                    let curve = NurbsCurve3::new(
                        2,
                        vec![
                            ControlPoint3::unweighted(pm[i]),
                            ControlPoint3::new(ctrl, weight),
                            ControlPoint3::unweighted(pc[i]),
                        ],
                        KnotVector::clamped_uniform(3, 2),
                    )?;
                    Edge::new(curve, vm[i].clone(), vc[i].clone(), 1e-6)
                }
                SlotHoleBlend::Chamfer(_) => Edge::line_between(vm[i].clone(), vc[i].clone())?,
            };
            ev_blend.push(edge);
        }

        // 6本のマウスエッジ、カットエッジ、ボトムエッジ
        let mut em = Vec::with_capacity(6);
        let mut ec = Vec::with_capacity(6);
        let mut eb = Vec::with_capacity(6);

        for i in 0..6 {
            let next = (i + 1) % 6;
            let is_arc = matches!(i, 1 | 2 | 4 | 5);

            let (edge_m, edge_c, edge_b) = if !is_arc {
                let edge_m = Edge::line_between(vm[i].clone(), vm[next].clone())?;
                let edge_c = Edge::line_between(vc[i].clone(), vc[next].clone())?;
                let edge_b = Edge::line_between(vb[i].clone(), vb[next].clone())?;
                (edge_m, edge_c, edge_b)
            } else {
                let ((cx_out, cy_out), (cx_in, cy_in)) = match i {
                    1 => ((l_half + r_out, -r_out), (l_half + r_in, -r_in)),
                    2 => ((l_half + r_out, r_out), (l_half + r_in, r_in)),
                    4 => ((-l_half - r_out, r_out), (-l_half - r_in, r_in)),
                    5 => ((-l_half - r_out, -r_out), (-l_half - r_in, -r_in)),
                    _ => unreachable!(),
                };

                let corner_m = pt_in(cx_out, cy_out, h_mouth);
                let corner_c = pt_in(cx_in, cy_in, h_cut);
                let corner_b = pt_in(cx_in, cy_in, h_bottom);

                let arc_m = Edge::new(
                    NurbsCurve3::new(
                        2,
                        vec![
                            ControlPoint3::unweighted(pm[i]),
                            ControlPoint3::new(corner_m, weight),
                            ControlPoint3::unweighted(pm[next]),
                        ],
                        KnotVector::clamped_uniform(3, 2),
                    )?,
                    vm[i].clone(),
                    vm[next].clone(),
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
                (arc_m, arc_c, arc_b)
            };

            em.push(edge_m);
            ec.push(edge_c);
            eb.push(edge_b);
        }

        let mut new_faces = Vec::with_capacity(14);

        // 1. 短縮された穴内壁 6枚 (カット -> ボトム)
        for i in 0..6 {
            let next = (i + 1) % 6;
            let is_arc = matches!(i, 1 | 2 | 4 | 5);

            let face_geom = if !is_arc {
                let u_axis = pc[next] - pc[i];
                let v_axis = pb[i] - pc[i];
                let plane = PlaneSurface3::new(pc[i], u_axis, v_axis)
                    .ok_or("Failed to create hole wall plane")?;
                FaceGeometry::Plane(plane)
            } else {
                let (cx_in, cy_in) = match i {
                    1 => (l_half + r_in, -r_in),
                    2 => (l_half + r_in, r_in),
                    4 => (-l_half - r_in, r_in),
                    5 => (-l_half - r_in, -r_in),
                    _ => unreachable!(),
                };
                let corner_c = pt_in(cx_in, cy_in, h_cut);
                let corner_b = pt_in(cx_in, cy_in, h_bottom);

                let surf = NurbsSurface3::new(
                    2,
                    1,
                    vec![
                        vec![
                            ControlPoint3::unweighted(pc[i]),
                            ControlPoint3::unweighted(pb[i]),
                        ],
                        vec![
                            ControlPoint3::new(corner_c, weight),
                            ControlPoint3::new(corner_b, weight),
                        ],
                        vec![
                            ControlPoint3::unweighted(pc[next]),
                            ControlPoint3::unweighted(pb[next]),
                        ],
                    ],
                    KnotVector::clamped_uniform(3, 2),
                    KnotVector::clamped_uniform(2, 1),
                )?;
                FaceGeometry::Nurbs(surf)
            };

            let wire = Wire::new(vec![
                OrientedEdge::forward(ec[i].clone()),
                OrientedEdge::forward(ev_lower[next].clone()),
                OrientedEdge::reversed(eb[i].clone()),
                OrientedEdge::reversed(ev_lower[i].clone()),
            ]);
            new_faces.push(Face::simple(face_geom, wire));
        }

        // 2. ブレンドパッチ 6枚 (マウス -> カット)
        for i in 0..6 {
            let next = (i + 1) % 6;
            let is_arc = matches!(i, 1 | 2 | 4 | 5);

            let face_geom = if !is_arc {
                let ctrl_i = pt_in(loc_in[i].0, loc_in[i].1, h_mouth);
                let ctrl_next = pt_in(loc_in[next].0, loc_in[next].1, h_mouth);

                let mut rows = match blend {
                    SlotHoleBlend::Fillet(_) => vec![
                        vec![
                            ControlPoint3::unweighted(pm[i]),
                            ControlPoint3::unweighted(pm[next]),
                        ],
                        vec![
                            ControlPoint3::new(ctrl_i, weight),
                            ControlPoint3::new(ctrl_next, weight),
                        ],
                        vec![
                            ControlPoint3::unweighted(pc[i]),
                            ControlPoint3::unweighted(pc[next]),
                        ],
                    ],
                    SlotHoleBlend::Chamfer(_) => vec![
                        vec![
                            ControlPoint3::unweighted(pm[i]),
                            ControlPoint3::unweighted(pm[next]),
                        ],
                        vec![
                            ControlPoint3::unweighted(pc[i]),
                            ControlPoint3::unweighted(pc[next]),
                        ],
                    ],
                };
                rows.reverse();
                let deg_u = match blend {
                    SlotHoleBlend::Fillet(_) => 2,
                    SlotHoleBlend::Chamfer(_) => 1,
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

                let corner_m = pt_in(cx_out, cy_out, h_mouth);
                let corner_c = pt_in(cx_in, cy_in, h_cut);
                let corner_ctrl = pt_in(cx_in, cy_in, h_mouth);

                let ctrl_i = pt_in(loc_in[i].0, loc_in[i].1, h_mouth);
                let ctrl_next = pt_in(loc_in[next].0, loc_in[next].1, h_mouth);

                let mut rows = match blend {
                    SlotHoleBlend::Fillet(_) => vec![
                        vec![
                            ControlPoint3::unweighted(pm[i]),
                            ControlPoint3::new(corner_m, weight),
                            ControlPoint3::unweighted(pm[next]),
                        ],
                        vec![
                            ControlPoint3::new(ctrl_i, weight),
                            ControlPoint3::new(corner_ctrl, weight * weight),
                            ControlPoint3::new(ctrl_next, weight),
                        ],
                        vec![
                            ControlPoint3::unweighted(pc[i]),
                            ControlPoint3::new(corner_c, weight),
                            ControlPoint3::unweighted(pc[next]),
                        ],
                    ],
                    SlotHoleBlend::Chamfer(_) => vec![
                        vec![
                            ControlPoint3::unweighted(pm[i]),
                            ControlPoint3::new(corner_m, weight),
                            ControlPoint3::unweighted(pm[next]),
                        ],
                        vec![
                            ControlPoint3::unweighted(pc[i]),
                            ControlPoint3::new(corner_c, weight),
                            ControlPoint3::unweighted(pc[next]),
                        ],
                    ],
                };
                rows.reverse();
                let deg_u = match blend {
                    SlotHoleBlend::Fillet(_) => 2,
                    SlotHoleBlend::Chamfer(_) => 1,
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
                OrientedEdge::forward(em[i].clone()),
                OrientedEdge::forward(ev_blend[next].clone()),
                OrientedEdge::reversed(ec[i].clone()),
                OrientedEdge::reversed(ev_blend[i].clone()),
            ]);
            new_faces.push(Face::simple(face_geom, wire));
        }

        // 3. 再トリムされた開口面（インナーワイヤを拡大したものに置換）
        let mouth_wire = Wire::new(
            (0..6)
                .rev()
                .map(|i| OrientedEdge::reversed(em[i].clone()))
                .collect(),
        );

        // 4. 再トリムされた底面キャップ面（インナーワイヤを更新）
        let bottom_wire = Wire::new(
            (0..6)
                .map(|i| OrientedEdge::forward(eb[i].clone()))
                .collect(),
        );

        let mut final_faces = Vec::with_capacity(solid.outer_shell.faces.len() + 6);
        for (idx, face) in solid.outer_shell.faces.iter().enumerate() {
            if self.bore_indices.contains(&idx) {
                continue;
            }
            if idx == self.cap_index {
                let mut inners = face.inner_wires.clone();
                inners[self.inner_wire_index] = mouth_wire.clone();
                final_faces.push(Face::new(
                    face.geometry.clone(),
                    face.outer_wire.clone(),
                    inners,
                    face.orientation,
                    face.tolerance,
                ));
            } else if idx == self.opposite_cap_index {
                let mut inners = face.inner_wires.clone();
                inners[self.opposite_inner_index] = bottom_wire.clone();
                final_faces.push(Face::new(
                    face.geometry.clone(),
                    face.outer_wire.clone(),
                    inners,
                    face.orientation,
                    face.tolerance,
                ));
            } else {
                final_faces.push(face.clone());
            }
        }

        final_faces.extend(new_faces);
        let raw_solid = Solid::try_new(
            Shell::closed(final_faces),
            solid.inner_shells.clone(),
            &Tolerance::default(),
        )
        .map_err(|error| format!("Slot-hole blend produced an invalid solid: {error}"))?;

        let (sewn, _) = crate::Sewer::sew_solid(&raw_solid, &Tolerance::default())
            .map_err(|error| format!("Slot-hole blend sewing failed: {error}"))?;
        Ok(sewn)
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
