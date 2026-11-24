//! 子午線の断面が直線の回転面 2 つ（円錐・円柱）が全周の円（四分円 4 本）で交わる凸の稜の
//! ブレンド（HANDOVER 4-975、既定オフ `ZENITH_REVOLVED_CORNER=1`）。
//!
//! 皿もみの底（円錐×円柱）。断面（ρ, d）の 2 本の直線の角で面取りの弦・フィレットの円を
//! 厳密に解き、四分円で回す。両側の 4 枚ずつの面を、奥の円の辺と頂点を残して作り直す。
//! 面の向きとワイヤの回り方は、断面から出る外向き法線にそろえて決める（元の面の
//! パラメータの向きに頼らない）。削れる体積は Pappus。

use std::f64::consts::{FRAC_1_SQRT_2, FRAC_PI_2, PI, SQRT_2, TAU};

use zenith_geom::{ControlPoint3, KnotVector, NurbsCurve3, NurbsSurface3};
use zenith_math::{Point3, Tolerance, Vec3, Vec3Ext};
use zenith_topo::{Edge, Face, FaceGeometry, Orientation, OrientedEdge, Shell, Solid, Vertex, Wire};

use crate::{BlendableEdge, EdgeBlendReport};

pub(crate) fn revolved_corner_enabled() -> bool {
    std::env::var("ZENITH_REVOLVED_CORNER").is_ok_and(|v| v == "1")
}

type P2 = (f64, f64);

fn sub(p: P2, q: P2) -> P2 {
    (p.0 - q.0, p.1 - q.1)
}
fn add(p: P2, q: P2) -> P2 {
    (p.0 + q.0, p.1 + q.1)
}
fn scale(p: P2, k: f64) -> P2 {
    (p.0 * k, p.1 * k)
}
fn norm(p: P2) -> f64 {
    p.0.hypot(p.1)
}
fn cross(p: P2, q: P2) -> f64 {
    p.0 * q.1 - p.1 * q.0
}

fn gauss_legendre(n: usize) -> Vec<(f64, f64)> {
    let mut out = Vec::with_capacity(n);
    for i in 1..=n {
        let mut x = (PI * (i as f64 - 0.25) / (n as f64 + 0.5)).cos();
        for _ in 0..100 {
            let (mut p0, mut p1) = (1.0, x);
            for k in 2..=n {
                let p2 = ((2 * k - 1) as f64 * x * p1 - (k - 1) as f64 * p0) / k as f64;
                p0 = p1;
                p1 = p2;
            }
            let dp = n as f64 * (x * p1 - p0) / (x * x - 1.0);
            let dx = p1 / dp;
            x -= dx;
            if dx.abs() < 1e-16 {
                break;
            }
        }
        let (mut p0, mut p1) = (1.0, x);
        for k in 2..=n {
            let p2 = ((2 * k - 1) as f64 * x * p1 - (k - 1) as f64 * p0) / k as f64;
            p0 = p1;
            p1 = p2;
        }
        let dp = n as f64 * (x * p1 - p0) / (x * x - 1.0);
        out.push((x, 2.0 / ((1.0 - x * x) * dp * dp)));
    }
    out
}

/// 線分 p→q の ∫ρ² dd
fn line_moment(p: P2, q: P2) -> f64 {
    (q.1 - p.1) * (p.0 * p.0 + p.0 * q.0 + q.0 * q.0) / 3.0
}

/// 中心 o・半径 r の円弧 p→q（短い向き）の ∫ρ² dd
fn arc_moment(o: P2, r: f64, p: P2, q: P2) -> f64 {
    let (vp, vq) = (sub(p, o), sub(q, o));
    let a0 = vp.1.atan2(vp.0);
    let delta = cross(vp, vq).atan2(vp.0 * vq.0 + vp.1 * vq.1);
    let mut sum = 0.0;
    for (x, w) in gauss_legendre(24) {
        let a = a0 + delta * 0.5 * (x + 1.0);
        let rho = o.0 + r * a.cos();
        sum += w * rho * rho * r * a.cos();
    }
    sum * delta * 0.5
}

#[derive(Clone, Copy)]
enum Kind {
    Fillet(f64),
    Chamfer(f64),
}

struct Section {
    /// 側 j の上の接点
    contact: [P2; 2],
    /// フィレットなら有理2次の中央（角 P0）と重み
    mid: Option<(P2, f64)>,
    volume: f64,
}

struct Corner {
    center: Point3,
    axis: Vec3,
    x: Vec3,
    y: Vec3,
    p0: P2,
    /// 側 j の奥の点（断面）
    far: [P2; 2],
    /// 側 j・四分円 i の面の番号
    sides: [[usize; 4]; 2],
    /// 側 j・四分円 i の奥の円の辺（F_i → F_next の向き）
    far_arcs: [[OrientedEdge; 4]; 2],
    /// 側 j の θ_i の奥の頂点
    far_vertices: [[Vertex; 4]; 2],
}

fn all_wires(face: &Face) -> impl Iterator<Item = &Wire> {
    std::iter::once(&face.outer_wire).chain(face.inner_wires.iter())
}

impl Corner {
    fn to2(&self, p: Point3) -> P2 {
        to2(self.center, self.axis, p)
    }
    fn point(&self, q: P2, angle: f64) -> Point3 {
        self.center + self.x * (q.0 * angle.cos()) + self.y * (q.0 * angle.sin()) - self.axis * q.1
    }
    /// 断面のベクトル n を角 angle で3次元へ
    fn vector(&self, n: P2, angle: f64) -> Vec3 {
        (self.x * angle.cos() + self.y * angle.sin()) * n.0 - self.axis * n.1
    }
    fn dir(&self, j: usize) -> P2 {
        let v = sub(self.far[j], self.p0);
        scale(v, 1.0 / norm(v))
    }
    /// 側 j の外向き法線（断面）: 直線に垂直で、もう一方の側から離れる向き
    fn outward(&self, j: usize) -> P2 {
        let (u, w) = (self.dir(j), self.dir(1 - j));
        let n = (-u.1, u.0);
        if n.0 * w.0 + n.1 * w.1 > 0.0 {
            scale(n, -1.0)
        } else {
            n
        }
    }

    fn recognize(solid: &Solid, edge_id: u64) -> Option<Self> {
        if !solid.inner_shells.is_empty() {
            return None;
        }
        let faces = &solid.outer_shell.faces;
        let selected = faces
            .iter()
            .flat_map(all_wires)
            .flat_map(|w| w.edges.iter())
            .find(|o| o.edge.id == edge_id)?
            .edge
            .clone();
        let (center, radius, _) = super::circle_from_edge(&selected)?;
        let (t0, t1) = selected.curve.param_range();
        let a = selected.curve.evaluate(t0) - center;
        let b = selected.curve.evaluate(0.5 * (t0 + t1)) - center;
        let axis = a.cross(&b).try_normalize_safe(1e-12)?;
        let tol = 2e-6 * radius.max(1.0);
        // 同じ円の上の辺（4 本）
        let mut ring: Vec<Edge> = Vec::new();
        for face in faces {
            for o in all_wires(face).flat_map(|w| w.edges.iter()) {
                let Some((c, r, _)) = super::circle_from_edge(&o.edge) else { continue };
                if (c - center).norm() > tol || (r - radius).abs() > tol {
                    continue;
                }
                let (s0, s1) = o.edge.curve.param_range();
                let m = o.edge.curve.evaluate(0.5 * (s0 + s1)) - center;
                if m.dot(&axis).abs() > tol {
                    continue;
                }
                if !ring.iter().any(|e| super::same_edge_geometry(e, &o.edge)) {
                    ring.push(o.edge.clone());
                }
            }
        }
        if ring.len() != 4 {
            return None;
        }
        let v0 = ring[0].start_vertex.point;
        let x = (v0 - center - axis * (v0 - center).dot(&axis)).try_normalize_safe(1e-12)?;
        let y = axis.cross(&x).normalize();
        let angle_of = |p: Point3| {
            let off = p - center;
            off.dot(&y).atan2(off.dot(&x)).rem_euclid(TAU)
        };
        let index_of = |p: Point3| -> Option<usize> {
            let a = angle_of(p) / FRAC_PI_2;
            let k = a.round();
            ((a - k).abs() <= 1e-6).then_some((k as usize) % 4)
        };
        let mouth_point = |i: usize| {
            center + (x * (FRAC_PI_2 * i as f64).cos() + y * (FRAC_PI_2 * i as f64).sin()) * radius
        };
        let p0 = (radius, 0.0);
        // 四分円 i の 2 枚の面（それぞれの奥の点・子午線・奥の辺）
        struct Patch {
            face: usize,
            far_i: Vertex,
            far_next: Vertex,
            far_arc: OrientedEdge,
            far2: P2,
        }
        let mut patches: Vec<Vec<Patch>> = (0..4).map(|_| Vec::new()).collect();
        let mut seen = [false; 4];
        for e in &ring {
            let (s, t) = (index_of(e.start_vertex.point)?, index_of(e.end_vertex.point)?);
            let i = if (s + 1) % 4 == t { s } else if (t + 1) % 4 == s { t } else { return None };
            if seen[i] {
                return None;
            }
            seen[i] = true;
            let next = (i + 1) % 4;
            let users: Vec<usize> = faces
                .iter()
                .enumerate()
                .filter(|(_, f)| all_wires(f).any(|w| w.edges.iter().any(|c| super::same_edge_geometry(e, &c.edge))))
                .map(|(fi, _)| fi)
                .collect();
            if users.len() != 2 {
                return None;
            }
            for &fi in &users {
                let face = &faces[fi];
                if !face.inner_wires.is_empty() || face.outer_wire.edges.len() != 4 {
                    return None;
                }
                let FaceGeometry::Nurbs(_) = &face.geometry else { return None };
                let others: Vec<&Edge> = face
                    .outer_wire
                    .edges
                    .iter()
                    .map(|o| &o.edge)
                    .filter(|c| !super::same_edge_geometry(e, c))
                    .collect();
                let (vi, vn) = (mouth_point(i), mouth_point(next));
                let touches = |c: &Edge, v: Point3| {
                    (c.start_vertex.point - v).norm() <= tol || (c.end_vertex.point - v).norm() <= tol
                };
                let mi = others.iter().find(|c| touches(c, vi))?;
                let mn = others.iter().find(|c| touches(c, vn))?;
                let far_arc = others.iter().find(|c| !touches(c, vi) && !touches(c, vn))?;
                let other_end = |c: &Edge, v: Point3| {
                    if (c.start_vertex.point - v).norm() <= tol {
                        c.end_vertex.clone()
                    } else {
                        c.start_vertex.clone()
                    }
                };
                let (fi_v, fn_v) = (other_end(mi, vi), other_end(mn, vn));
                // 奥の辺の向きを F_i → F_next に
                let arc = if (far_arc.start_vertex.point - fi_v.point).norm() <= tol
                    && (far_arc.end_vertex.point - fn_v.point).norm() <= tol
                {
                    OrientedEdge::forward((*far_arc).clone())
                } else if (far_arc.start_vertex.point - fn_v.point).norm() <= tol
                    && (far_arc.end_vertex.point - fi_v.point).norm() <= tol
                {
                    OrientedEdge::reversed((*far_arc).clone())
                } else {
                    return None;
                };
                // 子午線が直線
                for m in [mi, mn] {
                    let (a, b) = (m.start_vertex.point, m.end_vertex.point);
                    let (s0, s1) = m.curve.param_range();
                    let ab = b - a;
                    for k in 0..=8 {
                        let p = m.curve.evaluate(s0 + (s1 - s0) * k as f64 / 8.0);
                        let t = (p - a).dot(&ab) / ab.norm_squared();
                        if (p - (a + ab * t)).norm() > tol {
                            return None;
                        }
                    }
                }
                let far2 = to2(center, axis, fi_v.point);
                if norm(sub(to2(center, axis, fn_v.point), far2)) > tol || norm(sub(far2, p0)) <= tol {
                    return None;
                }
                patches[i].push(Patch { face: fi, far_i: fi_v, far_next: fn_v, far_arc: arc, far2 });
            }
        }
        // 側 0 は四分円 0 の最初の面と同じ奥の点を持つ面
        let ref_far = patches[0][0].far2;
        let mut sides = [[usize::MAX; 4]; 2];
        let mut far = [ref_far, ref_far];
        let mut arcs: [Vec<OrientedEdge>; 2] = [Vec::new(), Vec::new()];
        let mut fverts: [Vec<Vertex>; 2] = [Vec::new(), Vec::new()];
        for i in 0..4 {
            let (k0, k1) = if norm(sub(patches[i][0].far2, ref_far)) <= tol { (0, 1) } else { (1, 0) };
            if norm(sub(patches[i][k0].far2, ref_far)) > tol {
                return None;
            }
            if i == 0 {
                far[1] = patches[0][k1].far2;
            } else if norm(sub(patches[i][k1].far2, far[1])) > tol {
                return None;
            }
            for (j, k) in [(0, k0), (1, k1)] {
                let p = &patches[i][k];
                sides[j][i] = p.face;
                arcs[j].push(p.far_arc.clone());
                fverts[j].push(p.far_i.clone());
                let _ = &p.far_next;
            }
        }
        let site = Self {
            center,
            axis,
            x,
            y,
            p0,
            far,
            sides,
            far_arcs: [
                [arcs[0][0].clone(), arcs[0][1].clone(), arcs[0][2].clone(), arcs[0][3].clone()],
                [arcs[1][0].clone(), arcs[1][1].clone(), arcs[1][2].clone(), arcs[1][3].clone()],
            ],
            far_vertices: [
                [fverts[0][0].clone(), fverts[0][1].clone(), fverts[0][2].clone(), fverts[0][3].clone()],
                [fverts[1][0].clone(), fverts[1][1].clone(), fverts[1][2].clone(), fverts[1][3].clone()],
            ],
        };
        // 2 本の直線が角を成す（平行でない）
        if cross(site.dir(0), site.dir(1)).abs() < 1e-6 {
            return None;
        }
        // 面がその直線の回転面の上で、材料が角の中（凸）: 元の面の外向き法線と照合
        for j in 0..2 {
            let (u, n2) = (site.dir(j), site.outward(j));
            for i in 0..4 {
                let face = &faces[sides[j][i]];
                let FaceGeometry::Nurbs(surface) = &face.geometry else { return None };
                let ((u0, u1), (w0, w1)) = surface.param_range();
                for iu in 0..=4 {
                    for iw in 0..=4 {
                        let p = surface.evaluate(u0 + (u1 - u0) * iu as f64 / 4.0, w0 + (w1 - w0) * iw as f64 / 4.0);
                        if cross(u, sub(site.to2(p), p0)).abs() > tol {
                            return None;
                        }
                    }
                }
                let (um, wm) = (0.5 * (u0 + u1), 0.5 * (w0 + w1));
                let mut n = surface.normal(um, wm)?;
                if face.orientation == Orientation::Reversed {
                    n = -n;
                }
                let p = surface.evaluate(um, wm) - center;
                let ang = p.dot(&y).atan2(p.dot(&x));
                if n.dot(&site.vector(n2, ang)) < 0.5 {
                    return None;
                }
            }
        }
        Some(site)
    }

    fn solve(&self, kind: Kind) -> Result<Section, String> {
        let (u0, u1) = (self.dir(0), self.dir(1));
        let alpha = (u0.0 * u1.0 + u0.1 * u1.1).clamp(-1.0, 1.0).acos();
        if !(alpha > 1e-6 && alpha < PI - 1e-6) {
            return Err("The corner is degenerate".into());
        }
        let (t, mid) = match kind {
            Kind::Chamfer(s) => (s, None),
            Kind::Fillet(r) => {
                let t = r / (alpha * 0.5).tan();
                (t, Some((self.p0, (alpha * 0.5).sin())))
            }
        };
        let mut contact = [self.p0; 2];
        for j in 0..2 {
            let whole = norm(sub(self.far[j], self.p0));
            if t >= whole * (1.0 - 1e-6) {
                return Err("The blend reaches the far end of a face".into());
            }
            contact[j] = add(self.p0, scale(self.dir(j), t));
        }
        if contact.iter().any(|c| !(c.0 > 1e-6)) {
            return Err("The blend reaches the axis".into());
        }
        let mut m = line_moment(self.p0, contact[0]);
        m += match kind {
            Kind::Chamfer(_) => line_moment(contact[0], contact[1]),
            Kind::Fillet(r) => {
                let inward = scale(self.outward(0), -1.0);
                arc_moment(add(contact[0], scale(inward, r)), r, contact[0], contact[1])
            }
        };
        m += line_moment(contact[1], self.p0);
        Ok(Section { contact, mid, volume: TAU * (0.5 * m).abs() })
    }

    fn apply(&self, solid: &Solid, sec: &Section) -> Result<Solid, String> {
        let theta = |i: usize| FRAC_PI_2 * (i % 4) as f64;
        let ring = |q: P2, w: f64, i: usize| -> Vec<ControlPoint3> {
            let mid = theta(i) + FRAC_PI_2 * 0.5;
            vec![
                ControlPoint3::new(self.point(q, theta(i)), w),
                ControlPoint3::new(self.point((SQRT_2 * q.0, q.1), mid), w * FRAC_1_SQRT_2),
                ControlPoint3::new(self.point(q, theta(i + 1)), w),
            ]
        };
        let arc = |q: P2, i: usize, start: Vertex, end: Vertex| -> Result<Edge, String> {
            let curve = NurbsCurve3::new(2, ring(q, 1.0, i), KnotVector::clamped_uniform(3, 2))?;
            Ok(Edge::new(curve, start, end, 1e-6))
        };
        // 外向き法線 expected にそろえて面を作る（ワイヤの回り方と面の向きの両方）
        let make_face = |surface: NurbsSurface3, edges: Vec<OrientedEdge>, at: (P2, f64), expected: Vec3| -> Face {
            // ワイヤの回り方（Newell）
            let mut pts = Vec::new();
            for o in &edges {
                let (s0, s1) = o.edge.curve.param_range();
                for k in 0..4 {
                    let f = k as f64 / 4.0;
                    let t = if o.orientation == Orientation::Forward { s0 + (s1 - s0) * f } else { s1 - (s1 - s0) * f };
                    pts.push(o.edge.curve.evaluate(t));
                }
            }
            let mut newell = Vec3::zeros();
            for k in 0..pts.len() {
                let (p, q) = (pts[k], pts[(k + 1) % pts.len()]);
                newell += (p - Point3::origin()).cross(&(q - Point3::origin()));
            }
            let edges = if newell.dot(&expected) < 0.0 {
                edges.into_iter().rev().map(|o| OrientedEdge::new(o.edge, o.orientation.reversed())).collect()
            } else {
                edges
            };
            let ((u0, u1), (w0, w1)) = surface.param_range();
            let _ = at;
            let n = surface.normal(0.5 * (u0 + u1), 0.5 * (w0 + w1)).unwrap_or(expected);
            let orientation = if n.dot(&expected) < 0.0 { Orientation::Reversed } else { Orientation::Forward };
            Face::new(FaceGeometry::Nurbs(surface), Wire::new(edges), Vec::new(), orientation, 1e-6)
        };
        let contact: [Vec<Vertex>; 2] = [
            (0..4).map(|i| Vertex::from_point(self.point(sec.contact[0], theta(i)))).collect(),
            (0..4).map(|i| Vertex::from_point(self.point(sec.contact[1], theta(i)))).collect(),
        ];
        let mut replacement = Vec::new();
        let mut contact_arcs: [Vec<Edge>; 2] = [Vec::new(), Vec::new()];
        for j in 0..2 {
            for i in 0..4 {
                contact_arcs[j].push(arc(sec.contact[j], i, contact[j][i].clone(), contact[j][(i + 1) % 4].clone())?);
            }
        }
        let mid_angle = |i: usize| theta(i) + FRAC_PI_2 * 0.5;
        for j in 0..2 {
            let meridians: Vec<Edge> = (0..4)
                .map(|i| Edge::line_between(self.far_vertices[j][i].clone(), contact[j][i].clone()))
                .collect::<Result<_, _>>()?;
            for i in 0..4 {
                let next = (i + 1) % 4;
                let rows: Vec<Vec<ControlPoint3>> = [self.far[j], sec.contact[j]].iter().map(|&q| ring(q, 1.0, i)).collect();
                let surface = NurbsSurface3::new(
                    1,
                    2,
                    rows,
                    KnotVector::clamped_uniform(2, 1),
                    KnotVector::clamped_uniform(3, 2),
                )?;
                let edges = vec![
                    self.far_arcs[j][i].clone(),
                    OrientedEdge::forward(meridians[next].clone()),
                    OrientedEdge::reversed(contact_arcs[j][i].clone()),
                    OrientedEdge::reversed(meridians[i].clone()),
                ];
                let expected = self.vector(self.outward(j), mid_angle(i));
                replacement.push(make_face(surface, edges, (sec.contact[j], mid_angle(i)), expected));
            }
        }
        // ブレンドの面: 行は断面（contact0 → [mid] → contact1）、列は角
        let profiles: Vec<Edge> = (0..4)
            .map(|i| -> Result<Edge, String> {
                match sec.mid {
                    None => Edge::line_between(contact[0][i].clone(), contact[1][i].clone()),
                    Some((m, w)) => {
                        let curve = NurbsCurve3::new(
                            2,
                            vec![
                                ControlPoint3::unweighted(self.point(sec.contact[0], theta(i))),
                                ControlPoint3::new(self.point(m, theta(i)), w),
                                ControlPoint3::unweighted(self.point(sec.contact[1], theta(i))),
                            ],
                            KnotVector::clamped_uniform(3, 2),
                        )?;
                        Ok(Edge::new(curve, contact[0][i].clone(), contact[1][i].clone(), 1e-6))
                    }
                }
            })
            .collect::<Result<_, _>>()?;
        let rows_spec: Vec<(P2, f64)> = match sec.mid {
            None => vec![(sec.contact[0], 1.0), (sec.contact[1], 1.0)],
            Some((m, w)) => vec![(sec.contact[0], 1.0), (m, w), (sec.contact[1], 1.0)],
        };
        let deg = rows_spec.len() - 1;
        let blend_out = {
            let (a, b) = (self.outward(0), self.outward(1));
            let s = add(a, b);
            scale(s, 1.0 / norm(s))
        };
        for i in 0..4 {
            let next = (i + 1) % 4;
            let rows = rows_spec.iter().map(|&(q, w)| ring(q, w, i)).collect();
            let surface = NurbsSurface3::new(
                deg,
                2,
                rows,
                KnotVector::clamped_uniform(deg + 1, deg),
                KnotVector::clamped_uniform(3, 2),
            )?;
            let edges = vec![
                OrientedEdge::forward(contact_arcs[0][i].clone()),
                OrientedEdge::forward(profiles[next].clone()),
                OrientedEdge::reversed(contact_arcs[1][i].clone()),
                OrientedEdge::reversed(profiles[i].clone()),
            ];
            let expected = self.vector(blend_out, mid_angle(i));
            replacement.push(make_face(surface, edges, (sec.contact[0], mid_angle(i)), expected));
        }
        let removed: Vec<usize> = self.sides.iter().flatten().copied().collect();
        let mut faces: Vec<Face> = solid
            .outer_shell
            .faces
            .iter()
            .enumerate()
            .filter(|(k, _)| !removed.contains(k))
            .map(|(_, f)| f.clone())
            .collect();
        faces.extend(replacement);
        Solid::try_new(Shell::closed(faces), solid.inner_shells.clone(), &Tolerance::default())
            .map_err(|e| format!("Revolved corner blend produced an invalid solid: {e}"))
    }
}

fn to2(center: Point3, axis: Vec3, p: Point3) -> P2 {
    let off = p - center;
    let d = -off.dot(&axis);
    ((off + axis * d).norm(), d)
}

fn run(solid: &Solid, edge_id: u64, kind: Kind) -> Result<Option<(Solid, EdgeBlendReport)>, String> {
    if !revolved_corner_enabled() {
        return Ok(None);
    }
    let Some(site) = Corner::recognize(solid, edge_id) else {
        return Ok(None);
    };
    let sec = site.solve(kind)?;
    let result = site.apply(solid, &sec)?;
    let (u0, u1) = (site.dir(0), site.dir(1));
    let alpha = (u0.0 * u1.0 + u0.1 * u1.1).clamp(-1.0, 1.0).acos();
    Ok(Some((
        result,
        EdgeBlendReport {
            dihedral_angle_deg: 180.0 - alpha.to_degrees(),
            edge_length: TAU * site.p0.0,
            setback: norm(sub(sec.contact[0], site.p0)),
            predicted_removed_volume: sec.volume,
        },
    )))
}

pub(crate) fn try_fillet_revolved_corner(solid: &Solid, edge_id: u64, radius: f64) -> Result<Option<(Solid, EdgeBlendReport)>, String> {
    if !(radius > 1e-6) || !radius.is_finite() {
        return Ok(None);
    }
    run(solid, edge_id, Kind::Fillet(radius))
}

pub(crate) fn try_chamfer_revolved_corner(solid: &Solid, edge_id: u64, distance: f64) -> Result<Option<(Solid, EdgeBlendReport)>, String> {
    if !(distance > 1e-6) || !distance.is_finite() {
        return Ok(None);
    }
    run(solid, edge_id, Kind::Chamfer(distance))
}

pub(crate) fn revolved_corner_blendable(solid: &Solid, edge_id: u64) -> Option<BlendableEdge> {
    if !revolved_corner_enabled() {
        return None;
    }
    let site = Corner::recognize(solid, edge_id)?;
    let limit = |make: &dyn Fn(f64) -> Kind| {
        let (mut lo, mut hi) = (0.0, 4.0 * site.p0.0.max(norm(sub(site.far[0], site.p0))).max(norm(sub(site.far[1], site.p0))));
        for _ in 0..60 {
            let mid = 0.5 * (lo + hi);
            if site.solve(make(mid)).is_ok() {
                lo = mid;
            } else {
                hi = mid;
            }
        }
        lo * 0.999
    };
    let (u0, u1) = (site.dir(0), site.dir(1));
    let alpha = (u0.0 * u1.0 + u0.1 * u1.1).clamp(-1.0, 1.0).acos();
    Some(BlendableEdge {
        edge_id,
        length: TAU * site.p0.0,
        dihedral_angle_deg: 180.0 - alpha.to_degrees(),
        max_fillet_radius: limit(&|r| Kind::Fillet(r)),
        max_chamfer_distance: limit(&|s| Kind::Chamfer(s)),
    })
}
