//! 平面の内側ワイヤの全周の円（四分円 4 本）と、そこから材料の中へ下りる回転面
//! （円錐・凹の球）の間の稜のブレンド（HANDOVER 4-973、既定オフ `ZENITH_REVOLVED_MOUTH=1`）。
//!
//! 皿もみの口（上面×円錐）と球のくぼみの口（上面×凹の球）。`hole_mouth` の円柱を、
//! 子午線の断面が直線または円の回転面へ広げたもの。断面（半径 ρ、深さ d の半平面）で
//! 面取りの直線・フィレットの円弧を厳密に解き、四分円で回して面を作る。全周なので端は無い。
//! 側面の 4 枚は、断面の残り（接点から奥の円まで、または極まで）を回して作り直し、
//! 奥の円の辺（隣の面と共有）と頂点はそのまま使う。削れる体積は Pappus（2π∫∫ρ dA）。

use std::f64::consts::{FRAC_1_SQRT_2, FRAC_PI_2, PI, SQRT_2, TAU};

use zenith_geom::{ControlPoint3, KnotVector, NurbsCurve3, NurbsSurface3};
use zenith_math::{Point3, Tolerance, Vec3, Vec3Ext};
use zenith_topo::{Edge, Face, FaceGeometry, OrientedEdge, Shell, Solid, Vertex, Wire};

use crate::{BlendableEdge, EdgeBlendReport};

pub(crate) fn revolved_mouth_enabled() -> bool {
    std::env::var("ZENITH_REVOLVED_MOUTH").is_ok_and(|v| v == "1")
}

type P2 = (f64, f64);

#[derive(Clone, Copy, Debug)]
enum Profile {
    /// 直線（円錐）。口 p0 から奥 f へ。
    Line { p0: P2, f: P2 },
    /// 円（凹の球、材料は円の外）。中心は軸の上 (0, dc)、口 p0 から極 f=(0, dc+rs) へ。
    Circle { p0: P2, dc: f64, rs: f64, f: P2 },
}

#[derive(Clone, Copy)]
enum Kind {
    Fillet(f64),
    Chamfer(f64),
}

/// 断面の解: 平面の上の接点 A、回転面の上の接点 B、ブレンドの曲線（有理2次なら中央の
/// 制御点と重み）、側面の残りの曲線（B→F、有理2次なら中央と重み）。
#[derive(Clone, Copy, Debug)]
struct Section {
    a: P2,
    b: P2,
    blend_mid: Option<(P2, f64)>,
    side_mid: Option<(P2, f64)>,
    volume: f64,
}

fn sub(p: P2, q: P2) -> P2 {
    (p.0 - q.0, p.1 - q.1)
}
fn norm(p: P2) -> f64 {
    p.0.hypot(p.1)
}
fn scale(p: P2, k: f64) -> P2 {
    (p.0 * k, p.1 * k)
}
fn add(p: P2, q: P2) -> P2 {
    (p.0 + q.0, p.1 + q.1)
}

fn gauss_legendre(n: usize) -> Vec<(f64, f64)> {
    let mut out = Vec::with_capacity(n);
    for i in 0..n {
        let mut x = (PI * (i as f64 + 0.75) / (n as f64 + 0.5)).cos();
        let mut dp = 1.0;
        for _ in 0..100 {
            let (mut p0, mut p1) = (1.0, x);
            for k in 2..=n {
                let p2 = ((2 * k - 1) as f64 * x * p1 - (k - 1) as f64 * p0) / k as f64;
                p0 = p1;
                p1 = p2;
            }
            dp = n as f64 * (x * p1 - p0) / (x * x - 1.0);
            let dx = p1 / dp;
            x -= dx;
            if dx.abs() < 1e-16 {
                break;
            }
        }
        out.push((x, 2.0 / ((1.0 - x * x) * dp * dp)));
    }
    out
}

/// 線分 p→q の ∫ρ² dd
fn line_moment(p: P2, q: P2) -> f64 {
    (q.1 - p.1) * (p.0 * p.0 + p.0 * q.0 + q.0 * q.0) / 3.0
}

/// 中心 o・半径 r の円弧 p→q（短い向き）の ∫ρ² dd（角の3次の三角多項式なので GL で厳密）
fn arc_moment(o: P2, r: f64, p: P2, q: P2) -> f64 {
    let (vp, vq) = (sub(p, o), sub(q, o));
    let a0 = vp.1.atan2(vp.0);
    let delta = (vp.0 * vq.1 - vp.1 * vq.0).atan2(vp.0 * vq.0 + vp.1 * vq.1);
    let mut sum = 0.0;
    for (x, w) in gauss_legendre(24) {
        let a = a0 + delta * 0.5 * (x + 1.0);
        let rho = o.0 + r * a.cos();
        sum += w * rho * rho * r * a.cos();
    }
    sum * delta * 0.5
}

/// 中心 o の円の、点 p と q での接線の交点と、弧の重み cos(φ/2)
fn arc_mid(o: P2, p: P2, q: P2) -> Option<(P2, f64)> {
    let (vp, vq) = (sub(p, o), sub(q, o));
    let r = norm(vp);
    let cos_phi = ((vp.0 * vq.0 + vp.1 * vq.1) / (r * norm(vq))).clamp(-1.0, 1.0);
    let half = 0.5 * cos_phi.acos();
    if !(half.cos() > 1e-6) {
        return None;
    }
    let bis = add(vp, vq);
    let len = norm(bis);
    if len <= 1e-15 {
        return None;
    }
    Some((add(o, scale(bis, r / (len * half.cos()))), half.cos()))
}

impl Profile {
    fn p0(&self) -> P2 {
        match *self {
            Profile::Line { p0, .. } | Profile::Circle { p0, .. } => p0,
        }
    }
    fn far(&self) -> P2 {
        match *self {
            Profile::Line { f, .. } | Profile::Circle { f, .. } => f,
        }
    }

    /// 断面を解く。`clearance` は平面の上で口から使える半径の幅。
    fn solve(&self, kind: Kind, clearance: f64) -> Result<Section, String> {
        let p0 = self.p0();
        let (a, b, blend_mid) = match (*self, kind) {
            (Profile::Line { f, .. }, Kind::Chamfer(s)) => {
                let u = scale(sub(f, p0), 1.0 / norm(sub(f, p0)));
                ((p0.0 + s, p0.1), add(p0, scale(u, s)), None)
            }
            (Profile::Line { f, .. }, Kind::Fillet(r)) => {
                let u = scale(sub(f, p0), 1.0 / norm(sub(f, p0)));
                let alpha = u.1.atan2(u.0); // (1,0) から u への角（材料の側）
                if !(alpha > 1e-9 && alpha < PI - 1e-9) {
                    return Err("The mouth corner is not convex".into());
                }
                let t = r / (alpha * 0.5).tan();
                ((p0.0 + t, p0.1), add(p0, scale(u, t)), Some((p0, (alpha * 0.5).sin())))
            }
            (Profile::Circle { dc, rs, .. }, Kind::Chamfer(s)) => {
                let c = (0.0, dc);
                if s >= 2.0 * rs {
                    return Err("The chamfer is wider than the sphere".into());
                }
                let dpsi = 2.0 * (s / (2.0 * rs)).asin();
                let v = sub(p0, c);
                // 極の向き（+d）へ回す
                let rot = |ang: f64| add(c, (v.0 * ang.cos() - v.1 * ang.sin(), v.0 * ang.sin() + v.1 * ang.cos()));
                let (b1, b2) = (rot(dpsi), rot(-dpsi));
                let b = if b1.1 > b2.1 { b1 } else { b2 };
                ((p0.0 + s, p0.1), b, None)
            }
            (Profile::Circle { dc, rs, .. }, Kind::Fillet(r)) => {
                let c = (0.0, dc);
                let reach = rs + r;
                let rho2 = reach * reach - (r - dc) * (r - dc);
                if !(rho2 > 0.0) {
                    return Err("No fillet circle touches both the plane and the sphere".into());
                }
                let o = (rho2.sqrt(), r);
                let a = (o.0, 0.0);
                let b = add(c, scale(sub(o, c), rs / reach));
                let (mid, w) = arc_mid(o, a, b).ok_or("The fillet arc is degenerate")?;
                (a, b, Some((mid, w)))
            }
        };
        if a.0 - p0.0 >= clearance * (1.0 - 1e-9) {
            return Err(format!(
                "Mouth blend needs {:.6} on the face but only {clearance:.6} is free",
                a.0 - p0.0
            ));
        }
        // B は口と奥の間（奥にはまだ届かない）
        let f = self.far();
        let side_mid = match *self {
            Profile::Line { .. } => {
                let (whole, used) = (norm(sub(f, p0)), norm(sub(b, p0)));
                if used >= whole * (1.0 - 1e-6) {
                    return Err("The blend reaches the far end of the mouth face".into());
                }
                None
            }
            Profile::Circle { dc, .. } => {
                let c = (0.0, dc);
                // 口から B と、口から極の角
                let ang = |p: P2| {
                    let (v0, v) = (sub(p0, c), sub(p, c));
                    (v0.0 * v.1 - v0.1 * v.0).atan2(v0.0 * v.0 + v0.1 * v.1).abs()
                };
                if ang(b) >= ang(f) * (1.0 - 1e-6) {
                    return Err("The blend reaches the bottom of the dimple".into());
                }
                Some(arc_mid(c, f, b).ok_or("The remaining sphere arc is degenerate")?)
            }
        };
        // 削れる領域: P0→A（平面）、A→B（ブレンド）、B→P0（回転面）
        let mut m = line_moment(p0, a);
        m += match kind {
            Kind::Chamfer(_) => line_moment(a, b),
            Kind::Fillet(r) => {
                let o = match *self {
                    Profile::Line { .. } => (a.0, r),
                    Profile::Circle { .. } => (a.0, r),
                };
                arc_moment(o, r, a, b)
            }
        };
        m += match *self {
            Profile::Line { .. } => line_moment(b, p0),
            Profile::Circle { dc, rs, .. } => arc_moment((0.0, dc), rs, b, p0),
        };
        // Pappus: V = 2π ∫∫ρ dA = 2π |½∮ρ² dd|
        let volume = TAU * (0.5 * m).abs();
        Ok(Section { a, b, blend_mid, side_mid, volume })
    }
}

struct Mouth {
    cap_index: usize,
    cap_inner_index: usize,
    side_indices: [usize; 4],
    center: Point3,
    axis: Vec3,
    x: Vec3,
    y: Vec3,
    profile: Profile,
    clearance: f64,
    /// 奥の頂点（球なら 4 つとも極）
    far_vertices: [Vertex; 4],
    /// 側面 i の奥の円の辺（円錐のみ）。向きは F_next → F_i にそろえてある
    far_arcs: Option<[OrientedEdge; 4]>,
}

fn all_wires(face: &Face) -> impl Iterator<Item = &Wire> {
    std::iter::once(&face.outer_wire).chain(face.inner_wires.iter())
}

impl Mouth {
    fn to2(&self, p: Point3) -> P2 {
        let off = p - self.center;
        let d = -off.dot(&self.axis);
        let rho = (off + self.axis * d).norm();
        (rho, d)
    }

    fn point(&self, q: P2, angle: f64) -> Point3 {
        self.center + self.x * (q.0 * angle.cos()) + self.y * (q.0 * angle.sin()) - self.axis * q.1
    }

    fn recognize(solid: &Solid, edge_id: u64) -> Option<Self> {
        let faces = &solid.outer_shell.faces;
        let selected = faces
            .iter()
            .flat_map(all_wires)
            .flat_map(|w| w.edges.iter())
            .find(|o| o.edge.id == edge_id)?
            .edge
            .clone();
        let (cap_index, cap_inner_index) = faces.iter().enumerate().find_map(|(fi, face)| {
            if !matches!(face.geometry, FaceGeometry::Plane(_)) {
                return None;
            }
            face.inner_wires.iter().enumerate().find_map(|(wi, wire)| {
                wire.edges
                    .iter()
                    .any(|e| super::same_edge_geometry(&selected, &e.edge))
                    .then_some((fi, wi))
            })
        })?;
        let cap = &faces[cap_index];
        let wire = &cap.inner_wires[cap_inner_index];
        if wire.edges.len() != 4 {
            return None;
        }
        let (center, radius, _) = super::circle_from_edge(&selected)?;
        let axis = super::effective_plane_normal(cap).try_normalize_safe(1e-12)?;
        let tol = 2e-6 * radius.max(1.0);
        // 円の中心と半径、平面の上
        let mut corners = Vec::new();
        for o in &wire.edges {
            let (c, r, _) = super::circle_from_edge(&o.edge)?;
            if (c - center).norm() > tol || (r - radius).abs() > tol {
                return None;
            }
            corners.push(o.edge.start_vertex.point);
        }
        let v0 = corners[0];
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
        // 側面 i: θ_i から θ_{i+1} の弧に接する面
        let mut side_indices = [usize::MAX; 4];
        let mut mouth_edges: Vec<Option<Edge>> = vec![None; 4];
        for o in &wire.edges {
            let (s, e) = (index_of(o.edge.start_vertex.point)?, index_of(o.edge.end_vertex.point)?);
            let i = if (s + 1) % 4 == e { s } else if (e + 1) % 4 == s { e } else { return None };
            let users: Vec<usize> = faces
                .iter()
                .enumerate()
                .filter(|(fi, f)| {
                    *fi != cap_index
                        && all_wires(f).any(|w| w.edges.iter().any(|c| super::same_edge_geometry(&o.edge, &c.edge)))
                })
                .map(|(fi, _)| fi)
                .collect();
            if users.len() != 1 || !matches!(faces[users[0]].geometry, FaceGeometry::Nurbs(_)) {
                return None;
            }
            if side_indices[i] != usize::MAX {
                return None;
            }
            side_indices[i] = users[0];
            mouth_edges[i] = Some(o.edge.clone());
        }
        // 子午線 i（θ_i の口の頂点から奥へ）を側面 i から探す
        let mouth_point = |i: usize| center + (x * (FRAC_PI_2 * i as f64).cos() + y * (FRAC_PI_2 * i as f64).sin()) * radius;
        let mut meridians: Vec<Edge> = Vec::new();
        let mut far_vertices: Vec<Vertex> = Vec::new();
        for i in 0..4 {
            let face = &faces[side_indices[i]];
            if !face.inner_wires.is_empty() {
                return None;
            }
            let mouth = mouth_edges[i].as_ref()?;
            let vi = mouth_point(i);
            let m = face.outer_wire.edges.iter().find(|o| {
                !super::same_edge_geometry(&o.edge, mouth)
                    && ((o.edge.start_vertex.point - vi).norm() <= tol || (o.edge.end_vertex.point - vi).norm() <= tol)
            })?;
            let far = if (m.edge.start_vertex.point - vi).norm() <= tol {
                m.edge.end_vertex.clone()
            } else {
                m.edge.start_vertex.clone()
            };
            meridians.push(m.edge.clone());
            far_vertices.push(far);
        }
        // 断面を子午線 0 から読む
        let to2 = |p: Point3| {
            let off = p - center;
            let d = -off.dot(&axis);
            ((off + axis * d).norm(), d)
        };
        let p0 = (radius, 0.0);
        let f = to2(far_vertices[0].point);
        if !(f.1 > tol) {
            return None;
        }
        let m0 = &meridians[0];
        let profile = if let Some((c, rs, _)) = super::circle_from_edge(m0) {
            let off = c - center;
            let dc = -off.dot(&axis);
            if (off + axis * dc).norm() > tol || f.0 > tol || ((dc + rs) - f.1).abs() > tol {
                return None;
            }
            Profile::Circle { p0, dc, rs, f: (0.0, dc + rs) }
        } else {
            let (t0, t1) = m0.curve.param_range();
            let (a, b) = (m0.start_vertex.point, m0.end_vertex.point);
            let straight = (0..=8).all(|k| {
                let p = m0.curve.evaluate(t0 + (t1 - t0) * k as f64 / 8.0);
                let ab = b - a;
                let t = (p - a).dot(&ab) / ab.norm_squared();
                (p - (a + ab * t)).norm() <= tol
            });
            if !straight {
                return None;
            }
            Profile::Line { p0, f }
        };
        // 子午線の接線が材料の中（深さが増える向き）へ
        // 4 本の子午線・奥の頂点が同じ断面の上、面の点も回転面の上
        let on_profile = |q: P2| -> bool {
            match profile {
                Profile::Line { p0, f } => {
                    let (u, w) = (sub(f, p0), sub(q, p0));
                    let t = (u.0 * w.0 + u.1 * w.1) / (u.0 * u.0 + u.1 * u.1);
                    t >= -1e-9 && t <= 1.0 + 1e-9 && (u.0 * w.1 - u.1 * w.0).abs() / norm(u) <= tol
                }
                Profile::Circle { dc, rs, .. } => (norm(sub(q, (0.0, dc))) - rs).abs() <= tol && q.1 >= -tol,
            }
        };
        let on_surface = |q: P2| -> bool {
            match profile {
                Profile::Line { p0, f } => {
                    let (u, w) = (sub(f, p0), sub(q, p0));
                    (u.0 * w.1 - u.1 * w.0).abs() / norm(u) <= tol
                }
                Profile::Circle { dc, rs, .. } => (norm(sub(q, (0.0, dc))) - rs).abs() <= tol,
            }
        };
        for i in 0..4 {
            let fq = to2(far_vertices[i].point);
            if norm(sub(fq, f)) > tol {
                return None;
            }
            let (t0, t1) = meridians[i].curve.param_range();
            for k in 0..=8 {
                if !on_profile(to2(meridians[i].curve.evaluate(t0 + (t1 - t0) * k as f64 / 8.0))) {
                    return None;
                }
            }
            let FaceGeometry::Nurbs(surface) = &faces[side_indices[i]].geometry else { return None };
            let ((u0, u1), (w0, w1)) = surface.param_range();
            for iu in 0..=6 {
                for iw in 0..=6 {
                    let p = surface.evaluate(u0 + (u1 - u0) * iu as f64 / 6.0, w0 + (w1 - w0) * iw as f64 / 6.0);
                    // 面の支持曲面は面より広いことがある（球の残り）ので、範囲は見ず回転面の上かだけ
                    if !on_surface(to2(p)) {
                        return None;
                    }
                }
            }
        }
        // 奥の円の辺（円錐）か、極（球）
        let far_arcs = match profile {
            Profile::Line { .. } => {
                let mut arcs: Vec<OrientedEdge> = Vec::new();
                for i in 0..4 {
                    let next = (i + 1) % 4;
                    let face = &faces[side_indices[i]];
                    if face.outer_wire.edges.len() != 4 {
                        return None;
                    }
                    let mouth = mouth_edges[i].as_ref()?;
                    let arc = face.outer_wire.edges.iter().find(|o| {
                        !super::same_edge_geometry(&o.edge, mouth)
                            && !super::same_edge_geometry(&o.edge, &meridians[i])
                            && !super::same_edge_geometry(&o.edge, &meridians[next])
                    })?;
                    let (fi, fn_) = (far_vertices[i].point, far_vertices[next].point);
                    let e = arc.edge.clone();
                    if (e.start_vertex.point - fn_).norm() <= tol && (e.end_vertex.point - fi).norm() <= tol {
                        arcs.push(OrientedEdge::forward(e));
                    } else if (e.start_vertex.point - fi).norm() <= tol && (e.end_vertex.point - fn_).norm() <= tol {
                        arcs.push(OrientedEdge::reversed(e));
                    } else {
                        return None;
                    }
                }
                Some([arcs[0].clone(), arcs[1].clone(), arcs[2].clone(), arcs[3].clone()])
            }
            Profile::Circle { .. } => {
                for i in 0..4 {
                    if faces[side_indices[i]].outer_wire.edges.len() != 3 {
                        return None;
                    }
                }
                let pole = far_vertices[0].clone();
                far_vertices = vec![pole.clone(), pole.clone(), pole.clone(), pole];
                None
            }
        };
        // 平面の上で口から使える幅
        let mut nearest = f64::INFINITY;
        for (wi, w) in std::iter::once((usize::MAX, &cap.outer_wire)).chain(cap.inner_wires.iter().enumerate()) {
            if wi == cap_inner_index {
                continue;
            }
            for o in &w.edges {
                let (t0, t1) = o.edge.curve.param_range();
                for k in 0..=64 {
                    let p = o.edge.curve.evaluate(t0 + (t1 - t0) * k as f64 / 64.0);
                    nearest = nearest.min(to2(p).0);
                }
            }
        }
        let clearance = nearest - radius;
        if !(clearance > 1e-6) {
            return None;
        }
        Some(Self {
            cap_index,
            cap_inner_index,
            side_indices,
            center,
            axis,
            x,
            y,
            profile,
            clearance,
            far_vertices: [far_vertices[0].clone(), far_vertices[1].clone(), far_vertices[2].clone(), far_vertices[3].clone()],
            far_arcs,
        })
    }

    fn apply(&self, solid: &Solid, sec: &Section, kind: Kind) -> Result<Solid, String> {
        let theta = |i: usize| FRAC_PI_2 * (i % 4) as f64;
        // 断面の有理2次の制御点 (q, w) を、四分円 i で回した3行
        let ring = |q: P2, w: f64, i: usize| -> Vec<ControlPoint3> {
            let mid = theta(i) + FRAC_PI_2 * 0.5;
            vec![
                ControlPoint3::new(self.point(q, theta(i)), w),
                ControlPoint3::new(self.point((SQRT_2 * q.0, q.1), mid), w * FRAC_1_SQRT_2),
                ControlPoint3::new(self.point(q, theta(i + 1)), w),
            ]
        };
        let arc = |q: P2, i: usize, start: Vertex, end: Vertex| -> Result<Edge, String> {
            let row = ring(q, 1.0, i);
            let curve = NurbsCurve3::new(2, row, KnotVector::clamped_uniform(3, 2))?;
            Ok(Edge::new(curve, start, end, 1e-6))
        };
        // 断面の曲線（in-plane θ_i）p→q、中央 (mid, w) があれば有理2次
        let meridian = |p: P2, q: P2, mid: Option<(P2, f64)>, i: usize, start: Vertex, end: Vertex| -> Result<Edge, String> {
            match mid {
                None => Edge::line_between(start, end),
                Some((m, w)) => {
                    let curve = NurbsCurve3::new(
                        2,
                        vec![
                            ControlPoint3::unweighted(self.point(p, theta(i))),
                            ControlPoint3::new(self.point(m, theta(i)), w),
                            ControlPoint3::unweighted(self.point(q, theta(i))),
                        ],
                        KnotVector::clamped_uniform(3, 2),
                    )?;
                    Ok(Edge::new(curve, start, end, 1e-6))
                }
            }
        };
        let f = self.profile.far();
        let mouth: Vec<Vertex> = (0..4).map(|i| Vertex::from_point(self.point(sec.a, theta(i)))).collect();
        let join: Vec<Vertex> = (0..4).map(|i| Vertex::from_point(self.point(sec.b, theta(i)))).collect();
        let mut mouth_arcs = Vec::new();
        let mut join_arcs = Vec::new();
        let mut sides = Vec::new();
        let mut profiles = Vec::new();
        for i in 0..4 {
            let next = (i + 1) % 4;
            mouth_arcs.push(arc(sec.a, i, mouth[i].clone(), mouth[next].clone())?);
            join_arcs.push(arc(sec.b, i, join[i].clone(), join[next].clone())?);
            sides.push(meridian(f, sec.b, sec.side_mid, i, self.far_vertices[i].clone(), join[i].clone())?);
            profiles.push(meridian(sec.b, sec.a, sec.blend_mid, i, join[i].clone(), mouth[i].clone())?);
        }
        let mut replacement = Vec::new();
        // 側面: u は角（next, 中, i）、v は断面（F → B）
        let side_cols: Vec<(P2, f64)> = match sec.side_mid {
            None => vec![(f, 1.0), (sec.b, 1.0)],
            Some((m, w)) => vec![(f, 1.0), (m, w), (sec.b, 1.0)],
        };
        let side_deg = side_cols.len() - 1;
        for i in 0..4 {
            let next = (i + 1) % 4;
            let rows: Vec<Vec<ControlPoint3>> = (0..3)
                .map(|k| {
                    side_cols
                        .iter()
                        .map(|&(q, w)| {
                            let r = ring(q, w, i);
                            r[2 - k].clone()
                        })
                        .collect()
                })
                .collect();
            let surface = NurbsSurface3::new(
                2,
                side_deg,
                rows,
                KnotVector::clamped_uniform(3, 2),
                KnotVector::clamped_uniform(side_deg + 1, side_deg),
            )?;
            let mut edges = vec![
                OrientedEdge::forward(sides[i].clone()),
                OrientedEdge::forward(join_arcs[i].clone()),
                OrientedEdge::reversed(sides[next].clone()),
            ];
            if let Some(far) = &self.far_arcs {
                edges.push(far[i].clone());
            }
            replacement.push(Face::simple(FaceGeometry::Nurbs(surface), Wire::new(edges)));
        }
        // ブレンドの面: u は断面（B → A）、v は角（i, 中, next）
        let blend_rows: Vec<(P2, f64)> = match sec.blend_mid {
            None => vec![(sec.b, 1.0), (sec.a, 1.0)],
            Some((m, w)) => vec![(sec.b, 1.0), (m, w), (sec.a, 1.0)],
        };
        let deg = blend_rows.len() - 1;
        let _ = kind;
        for i in 0..4 {
            let next = (i + 1) % 4;
            let rows = blend_rows.iter().map(|&(q, w)| ring(q, w, i)).collect();
            let surface = NurbsSurface3::new(
                deg,
                2,
                rows,
                KnotVector::clamped_uniform(deg + 1, deg),
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
        let mouth_wire = Wire::new((0..4).rev().map(|i| OrientedEdge::reversed(mouth_arcs[i].clone())).collect());
        let mut faces = Vec::new();
        for (index, face) in solid.outer_shell.faces.iter().enumerate() {
            if self.side_indices.contains(&index) {
                continue;
            }
            if index == self.cap_index {
                let mut inners = face.inner_wires.clone();
                inners[self.cap_inner_index] = mouth_wire.clone();
                faces.push(Face::new(face.geometry.clone(), face.outer_wire.clone(), inners, face.orientation, face.tolerance));
            } else {
                faces.push(face.clone());
            }
        }
        faces.extend(replacement);
        Solid::try_new(Shell::closed(faces), solid.inner_shells.clone(), &Tolerance::default())
            .map_err(|e| format!("Revolved mouth blend produced an invalid solid: {e}"))
    }
}

fn run(solid: &Solid, edge_id: u64, kind: Kind) -> Result<Option<(Solid, EdgeBlendReport)>, String> {
    if !revolved_mouth_enabled() {
        return Ok(None);
    }
    let Some(site) = Mouth::recognize(solid, edge_id) else {
        return Ok(None);
    };
    let sec = site.profile.solve(kind, site.clearance)?;
    let result = site.apply(solid, &sec, kind)?;
    let p0 = site.profile.p0();
    let _ = site.to2(site.center);
    Ok(Some((
        result,
        EdgeBlendReport {
            dihedral_angle_deg: {
                let t = match site.profile {
                    Profile::Line { f, .. } => sub(f, p0),
                    Profile::Circle { dc, .. } => (p0.1 - dc, -(p0.0)),
                };
                let t = if t.1 < 0.0 { scale(t, -1.0) } else { t };
                t.1.atan2(t.0).to_degrees()
            },
            edge_length: TAU * p0.0,
            setback: sec.a.0 - p0.0,
            predicted_removed_volume: sec.volume,
        },
    )))
}

pub(crate) fn try_fillet_revolved_mouth(solid: &Solid, edge_id: u64, radius: f64) -> Result<Option<(Solid, EdgeBlendReport)>, String> {
    run(solid, edge_id, Kind::Fillet(radius))
}

pub(crate) fn try_chamfer_revolved_mouth(solid: &Solid, edge_id: u64, distance: f64) -> Result<Option<(Solid, EdgeBlendReport)>, String> {
    run(solid, edge_id, Kind::Chamfer(distance))
}

pub(crate) fn revolved_mouth_blendable(solid: &Solid, edge_id: u64) -> Option<BlendableEdge> {
    if !revolved_mouth_enabled() {
        return None;
    }
    let site = Mouth::recognize(solid, edge_id)?;
    let limit = |make: &dyn Fn(f64) -> Kind| {
        let (mut lo, mut hi) = (0.0, 4.0 * site.profile.p0().0.max(norm(sub(site.profile.far(), site.profile.p0()))));
        for _ in 0..60 {
            let mid = 0.5 * (lo + hi);
            if site.profile.solve(make(mid), site.clearance).is_ok() {
                lo = mid;
            } else {
                hi = mid;
            }
        }
        lo * 0.999
    };
    let p0 = site.profile.p0();
    Some(BlendableEdge {
        edge_id,
        length: TAU * p0.0,
        dihedral_angle_deg: 0.0,
        max_fillet_radius: limit(&|r| Kind::Fillet(r)),
        max_chamfer_distance: limit(&|s| Kind::Chamfer(s)),
    })
}
