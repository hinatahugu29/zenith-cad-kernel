//! 直線と円弧が**接線で繋がった**ループを上縁に持つ角柱の、上縁全周のブレンド
//! （ROADMAP 柱⑦、HANDOVER 4-934。`ZENITH_PRISM_RIM_BLEND`、既定オン、`=0`で切れる）。
//!
//! 縦稜を丸めた直方体（角丸の角柱）の上縁は、直線4本と円弧4本が接線で繋がった
//! 1本のループです。`slot_rim`（長円の柱、直線2本＋1/4円4本に決め打ち）と同じ
//! 作り方を、本数・円弧の角度・長さを問わない形に広げたもの:
//!
//! - 直線の部分は 1/4 円柱（面取りは平面）
//! - 円弧の部分は トーラスの一部（面取りは円錐の一部）——どちらも有理2次の
//!   厳密な面
//! - ループが接線で繋がっているので、内側へ `r` ずらしたループも同じ並びの
//!   直線と円弧になり、継ぎ目は G1 で閉じる
//!
//! 受け付ける形（どれか1つでも外れたら `None` を返し、ほかの経路に任せる）:
//! - 上面が平面で、選んだ稜がその外側ワイヤにある。内側ワイヤ（穴）はあってよい。
//!   穴は縮めた上面にそのまま残し、穴と外側のループの隔たりを上限に加える
//! - 外側ワイヤの稜は直線か円弧（中心角 180 度未満）で、隣どうしが接線で繋がる。
//!   円弧は凸（中心が上面の内側）。角（接線が折れる頂点）が1つでもあれば対象外
//!   ——その場合は既存の経路が稜を1本ずつ扱う
//! - 稜ごとに側面がちょうど1枚あり、その側面は「上の稜・下の稜・縦の直線2本」の
//!   4本で囲まれ、下の稜は上の稜を上面の法線の逆向きに高さ `h` だけ平行移動した
//!   もの（全部の側面で同じ `h`）
//!
//! 側面の下半分と下の稜は作り直し、`Sewer` で元の底面と縫い合わせる（`slot_rim`
//! と同じ）。

use std::f64::consts::{FRAC_1_SQRT_2, PI};

use zenith_geom::{ControlPoint3, KnotVector, NurbsCurve3, NurbsSurface3, PlaneSurface3};
use zenith_math::{Point3, Tolerance, Vec3, Vec3Ext};
use zenith_topo::{Edge, Face, FaceGeometry, OrientedEdge, Shell, Solid, Vertex, Wire};

use crate::{BlendableEdge, EdgeBlendReport};

pub(crate) fn enabled() -> bool {
    std::env::var("ZENITH_PRISM_RIM_BLEND")
        .map(|value| value != "0")
        .unwrap_or(true)
}

#[derive(Clone, Copy)]
enum Blend {
    Fillet(f64),
    Chamfer(f64),
}

impl Blend {
    fn setback(self) -> f64 {
        match self {
            Self::Fillet(value) | Self::Chamfer(value) => value,
        }
    }
}

pub(crate) fn try_fillet_prism_rim(
    solid: &Solid,
    edge_id: u64,
    radius: f64,
) -> Result<Option<(Solid, EdgeBlendReport)>, String> {
    try_blend(solid, edge_id, Blend::Fillet(radius))
}

pub(crate) fn try_chamfer_prism_rim(
    solid: &Solid,
    edge_id: u64,
    distance: f64,
) -> Result<Option<(Solid, EdgeBlendReport)>, String> {
    try_blend(solid, edge_id, Blend::Chamfer(distance))
}

pub(crate) fn prism_rim_blendable(solid: &Solid, edge_id: u64) -> Option<BlendableEdge> {
    if !enabled() {
        return None;
    }
    let site = PrismRim::recognize(solid, edge_id)?;
    let limit = site.limit() * 0.999;
    Some(BlendableEdge {
        edge_id,
        length: site.perimeter(),
        dihedral_angle_deg: 90.0,
        max_fillet_radius: limit,
        max_chamfer_distance: limit,
    })
}

fn try_blend(
    solid: &Solid,
    edge_id: u64,
    blend: Blend,
) -> Result<Option<(Solid, EdgeBlendReport)>, String> {
    if !enabled() {
        return Ok(None);
    }
    let Some(site) = PrismRim::recognize(solid, edge_id) else {
        return Ok(None);
    };
    let setback = blend.setback();
    if !(setback > 1e-6) || !setback.is_finite() {
        return Err(format!(
            "Prism-rim blend size must be finite and larger than 1e-6, got {setback}"
        ));
    }
    if setback >= site.limit() {
        return Err(format!(
            "Prism-rim blend size {setback} must be smaller than {:.6} (the smallest arc radius, the height, and half the narrowest width)",
            site.limit()
        ));
    }
    let result = site.apply(solid, blend)?;
    Ok(Some((
        result,
        EdgeBlendReport {
            dihedral_angle_deg: 90.0,
            edge_length: site.perimeter(),
            setback,
            predicted_removed_volume: site.removed_volume(blend),
        },
    )))
}

/// ループの1区間。
#[derive(Clone)]
enum Segment {
    Line {
        start: Point3,
        end: Point3,
    },
    Arc {
        start: Point3,
        end: Point3,
        center: Point3,
        radius: f64,
        /// 中心角（0 < angle < π）
        angle: f64,
        /// 凹の円弧（中心が上面の外側。4-942、`ZENITH_PRISM_CONCAVE_ARC`）
        concave: bool,
    },
}

impl Segment {
    fn start(&self) -> Point3 {
        match self {
            Self::Line { start, .. } | Self::Arc { start, .. } => *start,
        }
    }

    fn end(&self) -> Point3 {
        match self {
            Self::Line { end, .. } | Self::Arc { end, .. } => *end,
        }
    }

    fn length(&self) -> f64 {
        match self {
            Self::Line { start, end } => (end - start).norm(),
            Self::Arc { radius, angle, .. } => radius * angle,
        }
    }
}

struct PrismRim {
    top_index: usize,
    side_indices: Vec<usize>,
    normal: Vec3,
    segments: Vec<Segment>,
    height: f64,
    /// 上面の中で、向かい合う境界どうしの最小の隔たり（の半分を上限に使う）
    narrowest: f64,
    /// 上面の穴（内側ワイヤ）と外側のループの最小の隔たり。穴が無ければ無限大
    hole_clearance: f64,
    /// 角で止まる開いた鎖なら、その両端（4-937）。閉じたループなら None
    open: Option<OpenEnds>,
    /// 区間ごとに、側面が4本の帯か（false なら一般の平面、4-941）
    strip: Vec<bool>,
    /// 一般の平面の側面で、稜の両端から下りる稜の短い方
    general_reach: f64,
}

/// 開いた鎖の両端。角の先の稜（直線）と、その側面（蓋）。
struct OpenEnds {
    /// 鎖の始点の角へ入ってくる稜と、終点の角から出ていく稜（上面のワイヤの向き）
    prev: OrientedEdge,
    next: OrientedEdge,
    prev_len: f64,
    next_len: f64,
    cap_start: usize,
    cap_end: usize,
    /// `next` のあとから `prev` の手前までの、上面のワイヤの残りの稜
    rest: Vec<OrientedEdge>,
}

pub(crate) fn open_chain_enabled() -> bool {
    std::env::var("ZENITH_PRISM_OPEN_CHAIN")
        .map(|value| value != "0")
        .unwrap_or(true)
}

/// 上面のワイヤの稜を、直線か凸の円弧（中心角 180 度未満）として読む。
fn classify(oriented: &OrientedEdge, normal: Vec3) -> Option<Segment> {
    let start = oriented.start_vertex().point;
    let end = oriented.end_vertex().point;
    if let Some((center, radius, _)) = super::circle_from_edge(&oriented.edge) {
        // 進行方向の左が上面の内側。凸の円弧は中心が左にある。
        let chord = end - start;
        let middle = (start.coords + end.coords) * 0.5;
        let to_center = center.coords - middle;
        let side = normal.cross(&chord).dot(&to_center);
        let concave = side < 0.0;
        if side == 0.0 || (concave && !concave_arc_enabled()) {
            return None;
        }
        let a = (start - center) / radius;
        let b = (end - center) / radius;
        let angle = a.dot(&b).clamp(-1.0, 1.0).acos();
        if !(angle > 1e-6 && angle < PI - 1e-3) {
            return None;
        }
        // 円弧が上面の平面に乗っている
        if (center - start).dot(&normal).abs() > 1e-7 * radius.max(1.0) {
            return None;
        }
        Some(Segment::Arc {
            start,
            end,
            center,
            radius,
            angle,
            concave,
        })
    } else if is_straight(&oriented.edge) {
        Some(Segment::Line { start, end })
    } else {
        None
    }
}

impl PrismRim {
    fn recognize(solid: &Solid, edge_id: u64) -> Option<Self> {
        if !solid.inner_shells.is_empty() {
            return None;
        }
        let faces = &solid.outer_shell.faces;
        let candidates: Vec<usize> = faces
            .iter()
            .enumerate()
            .filter(|(_, face)| {
                matches!(face.geometry, FaceGeometry::Plane(_))
                    && face
                        .outer_wire
                        .edges
                        .iter()
                        .any(|oriented| oriented.edge.id == edge_id)
            })
            .map(|(index, _)| index)
            .collect();
        // 稜の両側が平面なら、どちらを「上面」と読むかで鎖が変わる（4-941）。
        // 先に見つかった方で読めなければ、もう一方も試す。
        let tries = if general_side_enabled() { candidates.len() } else { 1 };
        candidates
            .into_iter()
            .take(tries)
            .find_map(|top_index| Self::recognize_on(solid, edge_id, top_index))
    }

    fn recognize_on(solid: &Solid, edge_id: u64, top_index: usize) -> Option<Self> {
        let faces = &solid.outer_shell.faces;
        let top = &faces[top_index];
        let normal = super::effective_plane_normal(top).try_normalize_safe(1e-12)?;
        let rim = &top.outer_wire;
        if rim.edges.len() < 2 {
            return None;
        }

        // 外側ワイヤの稜を直線・凸の円弧に分類する（ほかの形は None）。
        let classified: Vec<Option<Segment>> = rim
            .edges
            .iter()
            .map(|oriented| classify(oriented, normal))
            .collect();
        let n_rim = rim.edges.len();
        // 稜 i と i+1 が接線で繋がるか
        let joint = |i: usize| -> bool {
            let (Some(here), Some(next)) = (&classified[i], &classified[(i + 1) % n_rim]) else {
                return false;
            };
            match (tangent_at(here, false, normal), tangent_at(next, true, normal)) {
                (Some(out), Some(into)) => (out - into).norm() <= 1e-6,
                _ => false,
            }
        };
        let picked = rim.edges.iter().position(|oriented| oriented.edge.id == edge_id)?;
        classified[picked].as_ref()?;
        let closed = (0..n_rim).all(joint);
        let chain: Vec<usize> = if closed {
            (0..n_rim).collect()
        } else {
            if !open_chain_enabled() {
                return None;
            }
            // 選んだ稜から、接線で繋がる限り前後へ伸ばす。角で止まる。
            let mut first = picked;
            while joint((first + n_rim - 1) % n_rim) {
                first = (first + n_rim - 1) % n_rim;
            }
            let mut chain = vec![first];
            let mut at = first;
            while joint(at) {
                at = (at + 1) % n_rim;
                chain.push(at);
            }
            chain
        };
        let segments: Vec<Segment> = chain
            .iter()
            .map(|index| classified[*index].clone())
            .collect::<Option<Vec<_>>>()?;
        // 円弧を含まない鎖（角のある多角形の稜1本）は既存の経路が扱う。
        if !segments.iter().any(|segment| matches!(segment, Segment::Arc { .. })) {
            return None;
        }
        let count = segments.len();

        // 開いた鎖の両端: 角の先の稜は直線で、角で直角に折れて上面の内側へ走る。
        // その稜の側面（蓋）は平面で、鎖の端の接線と直交する。
        let open = if closed {
            None
        } else {
            let prev_pos = (chain[0] + n_rim - 1) % n_rim;
            let next_pos = (chain[count - 1] + 1) % n_rim;
            // 前後が同じ1本の稜（鎖がその1本以外を全部覆う）でもよい。その稜は両端を詰める。
            if chain.contains(&prev_pos) || chain.contains(&next_pos) {
                return None;
            }
            let prev = rim.edges[prev_pos].clone();
            let next = rim.edges[next_pos].clone();
            let t_start = tangent_at(&segments[0], true, normal)?;
            let t_end = tangent_at(&segments[count - 1], false, normal)?;
            let inward_start = normal.cross(&t_start);
            let inward_end = normal.cross(&t_end);
            let direction = |oriented: &OrientedEdge| -> Option<(Vec3, f64)> {
                if !is_straight(&oriented.edge) {
                    return None;
                }
                let span = oriented.end_vertex().point - oriented.start_vertex().point;
                let length = span.norm();
                Some((span.try_normalize_safe(1e-12)?, length))
            };
            let (prev_dir, mut prev_len) = direction(&prev)?;
            let (next_dir, mut next_len) = direction(&next)?;
            if prev_pos == next_pos {
                // 両端から詰めるので、使えるのは半分ずつ
                prev_len *= 0.5;
                next_len *= 0.5;
            }
            if prev_dir.dot(&-inward_start) < 1.0 - 1e-9 || next_dir.dot(&inward_end) < 1.0 - 1e-9 {
                return None;
            }
            let cap_of = |oriented: &OrientedEdge, tangent: Vec3| -> Option<usize> {
                let users: Vec<usize> = faces
                    .iter()
                    .enumerate()
                    .filter(|(_, face)| {
                        std::iter::once(&face.outer_wire)
                            .chain(face.inner_wires.iter())
                            .any(|wire| wire.edges.iter().any(|e| e.edge.id == oriented.edge.id))
                    })
                    .map(|(index, _)| index)
                    .collect();
                if users.len() != 2 || !users.contains(&top_index) {
                    return None;
                }
                let cap = *users.iter().find(|index| **index != top_index)?;
                match &faces[cap].geometry {
                    FaceGeometry::Plane(plane)
                        if plane.normal.dot(&tangent).abs() >= 1.0 - 1e-9 =>
                    {
                        Some(cap)
                    }
                    _ => None,
                }
            };
            let cap_start = cap_of(&prev, t_start)?;
            let cap_end = cap_of(&next, t_end)?;
            Some(OpenEnds {
                prev,
                next,
                prev_len,
                next_len,
                cap_start,
                cap_end,
                rest: (0..n_rim)
                    .map(|offset| (next_pos + 1 + offset) % n_rim)
                    .take_while(|position| *position != prev_pos)
                    .map(|position| rim.edges[position].clone())
                    .collect(),
            })
        };

        // 側面: 稜ごとに1枚、4本で囲まれ、下の稜は上の稜の平行移動
        let mut side_indices = Vec::with_capacity(count);
        let mut strip: Vec<bool> = Vec::with_capacity(count);
        let mut general_reach = f64::INFINITY;
        let mut height: Option<f64> = None;
        for (segment_index, oriented) in chain.iter().map(|index| &rim.edges[*index]).enumerate() {
            let users: Vec<usize> = faces
                .iter()
                .enumerate()
                .filter(|(_, face)| {
                    std::iter::once(&face.outer_wire)
                        .chain(face.inner_wires.iter())
                        .any(|wire| wire.edges.iter().any(|e| e.edge.id == oriented.edge.id))
                })
                .map(|(index, _)| index)
                .collect();
            if users.len() != 2 || !users.contains(&top_index) {
                return None;
            }
            let side_index = *users.iter().find(|index| **index != top_index)?;
            let side = &faces[side_index];
            let is_strip = side.inner_wires.is_empty() && side.outer_wire.edges.len() == 4;
            if !is_strip {
                // 一般の平面の側面（4-941）: 直線の区間だけ。稜の両端で隣り合う
                // 2本が、上面の法線の逆向きに真っすぐ下りる。
                if !general_side_enabled()
                    || !matches!(segments[segment_index], Segment::Line { .. })
                    || !matches!(side.geometry, FaceGeometry::Plane(_))
                {
                    return None;
                }
                let reach = general_side_reach(side, oriented.edge.id, normal)?;
                general_reach = general_reach.min(reach);
                side_indices.push(side_index);
                strip.push(false);
                continue;
            }
            let position = side
                .outer_wire
                .edges
                .iter()
                .position(|e| e.edge.id == oriented.edge.id)?;
            let bottom = &side.outer_wire.edges[(position + 2) % 4];
            let up_a = &side.outer_wire.edges[(position + 1) % 4];
            let up_b = &side.outer_wire.edges[(position + 3) % 4];
            if !is_straight(&up_a.edge) || !is_straight(&up_b.edge) {
                return None;
            }
            let start = oriented.start_vertex().point;
            let end = oriented.end_vertex().point;
            let drop_at = |point: Point3| -> Option<f64> {
                // `point` から法線の逆向きに下ろした点が、下の稜の端にある
                [bottom.start_vertex().point, bottom.end_vertex().point]
                    .into_iter()
                    .find_map(|candidate| {
                        let offset = point - candidate;
                        let along = offset.dot(&normal);
                        ((offset - normal * along).norm() <= 1e-7 * along.abs().max(1.0)
                            && along > 1e-9)
                            .then_some(along)
                    })
            };
            let h_start = drop_at(start)?;
            let h_end = drop_at(end)?;
            let h = h_start;
            if (h_end - h).abs() > 1e-7 * h.max(1.0) {
                return None;
            }
            if let Some(previous) = height {
                if (previous - h).abs() > 1e-7 * h.max(1.0) {
                    return None;
                }
            }
            height = Some(h);
            strip.push(true);
            // 下の稜は上の稜を下ろしたもの（中点で確かめる）
            let (t0, t1) = oriented.edge.curve.param_range();
            let (b0, b1) = bottom.edge.curve.param_range();
            let top_mid = oriented.edge.curve.evaluate((t0 + t1) * 0.5) - normal * h;
            let bottom_mid = bottom.edge.curve.evaluate((b0 + b1) * 0.5);
            if (top_mid - bottom_mid).norm() > 1e-6 * h.max(1.0) {
                return None;
            }
            side_indices.push(side_index);
        }
        // 帯が無ければ、底の写しは使わない。高さは一般の側面の届く長さにする。
        let height = height.unwrap_or(general_reach);
        if !height.is_finite() {
            return None;
        }
        // 蓋と側面が同じ面なら作り直しがぶつかる
        if let Some(ends) = &open {
            if side_indices.contains(&ends.cap_start) || side_indices.contains(&ends.cap_end) {
                return None;
            }
        }

        // 上面の幅: 内向きの法線が向かい合う（内積 < -0.9）境界の点どうしの
        // 最短距離。内側へずらしたループが潰れないよう、その半分を上限に使う。
        // 円は直径、角丸の矩形は短い辺の幅になる。
        let samples: Vec<(Point3, Vec3)> = segments
            .iter()
            .flat_map(|segment| {
                (0..=16).filter_map(move |step| {
                    let fraction = step as f64 / 16.0;
                    let point = point_on(segment, fraction);
                    let tangent = match segment {
                        Segment::Line { start, end } => (end - start).try_normalize_safe(1e-12)?,
                        Segment::Arc { center, concave, .. } => {
                            let sign = if *concave { -1.0 } else { 1.0 };
                            (normal.cross(&(point - center)) * sign).try_normalize_safe(1e-12)?
                        }
                    };
                    Some((point, normal.cross(&tangent)))
                })
            })
            .collect();
        let mut narrowest = f64::INFINITY;
        for (a, inward_a) in &samples {
            for (b, inward_b) in &samples {
                if inward_a.dot(inward_b) < -0.9 {
                    narrowest = narrowest.min((a - b).norm());
                }
            }
        }

        // 穴の境界の点から、外側のループの区間までの厳密な距離の最小。
        // 穴の稜は細かく取る（ブレンドの帯に穴が掛からないための上限）。
        let mut hole_clearance = f64::INFINITY;
        for wire in &top.inner_wires {
            for oriented in &wire.edges {
                let (t0, t1) = oriented.edge.curve.param_range();
                for step in 0..=64 {
                    let point = oriented.edge.curve.evaluate(t0 + (t1 - t0) * step as f64 / 64.0);
                    if (point - rim_point(&segments)).dot(&normal).abs() > 1e-6 {
                        return None;
                    }
                    for segment in &segments {
                        hole_clearance = hole_clearance.min(distance_to(segment, point, normal));
                    }
                }
            }
        }

        Some(Self {
            strip,
            general_reach,
            open,
            hole_clearance,
            top_index,
            side_indices,
            normal,
            segments,
            height,
            narrowest,
        })
    }

    fn perimeter(&self) -> f64 {
        self.segments.iter().map(Segment::length).sum()
    }

    fn limit(&self) -> f64 {
        let smallest_radius = self
            .segments
            .iter()
            .filter_map(|segment| match segment {
                // 凹の円弧は内側へずらすと半径が増えるので、上限にならない
                Segment::Arc { radius, concave, .. } => (!concave).then_some(*radius),
                Segment::Line { .. } => None,
            })
            .fold(f64::INFINITY, f64::min);
        smallest_radius
            .min(self.height)
            .min(self.general_reach)
            .min(self.narrowest * 0.5)
            // 穴の稜は標本で測るので、少しだけ余裕を見る
            .min(self.hole_clearance * 0.98)
            .min(
                self.open
                    .as_ref()
                    .map_or(f64::INFINITY, |ends| ends.prev_len.min(ends.next_len)),
            )
    }

    /// 削れる体積の閉形式（パップス・ギュルダン）。
    fn removed_volume(&self, blend: Blend) -> f64 {
        let mut total = 0.0;
        for segment in &self.segments {
            total += match (segment, blend) {
                (Segment::Line { .. }, Blend::Fillet(r)) => segment.length() * r * r * (1.0 - PI / 4.0),
                (Segment::Line { .. }, Blend::Chamfer(d)) => segment.length() * d * d * 0.5,
                // 断面の重心は稜から内側へ。凸は中心へ近づき、凹は遠ざかる。
                (Segment::Arc { radius, angle, concave: false, .. }, Blend::Fillet(r)) => {
                    angle * ((radius - r) * r * r * (1.0 - PI / 4.0) + r.powi(3) / 6.0)
                }
                (Segment::Arc { radius, angle, concave: false, .. }, Blend::Chamfer(d)) => {
                    angle * (radius * d * d * 0.5 - d.powi(3) / 6.0)
                }
                (Segment::Arc { radius, angle, concave: true, .. }, Blend::Fillet(r)) => {
                    angle * ((radius + r) * r * r * (1.0 - PI / 4.0) - r.powi(3) / 6.0)
                }
                (Segment::Arc { radius, angle, concave: true, .. }, Blend::Chamfer(d)) => {
                    angle * (radius * d * d * 0.5 + d.powi(3) / 6.0)
                }
            };
        }
        total
    }

    fn apply(&self, solid: &Solid, blend: Blend) -> Result<Solid, String> {
        let r = blend.setback();
        let n = self.normal;
        let count = self.segments.len();
        // 頂点の数。開いた鎖は区間より1つ多い。
        let nv = if self.open.is_some() { count + 1 } else { count };
        let down = |point: Point3, depth: f64| point - n * depth;
        // 内側へずらす向き（進行方向の左）
        let inward = |segment: &Segment, at_start: bool| -> Vec3 {
            let tangent = tangent_at(segment, at_start, n).unwrap_or_else(Vec3::zeros);
            n.cross(&tangent)
        };

        // 頂点 i は区間 i の始点
        let mut rim_points: Vec<Point3> = self.segments.iter().map(Segment::start).collect();
        if self.open.is_some() {
            rim_points.push(self.segments[count - 1].end());
        }
        let pb: Vec<Point3> = rim_points.iter().map(|p| down(*p, self.height)).collect();
        let pc: Vec<Point3> = rim_points.iter().map(|p| down(*p, r)).collect();
        let pt: Vec<Point3> = (0..nv)
            .map(|i| {
                let step = if i < count {
                    inward(&self.segments[i], true)
                } else {
                    inward(&self.segments[count - 1], false)
                };
                rim_points[i] + step * r
            })
            .collect();

        let vb: Vec<Vertex> = pb.iter().map(|p| Vertex::from_point(*p)).collect();
        let vc: Vec<Vertex> = pc.iter().map(|p| Vertex::from_point(*p)).collect();
        let vt: Vec<Vertex> = pt.iter().map(|p| Vertex::from_point(*p)).collect();

        let weight_profile = FRAC_1_SQRT_2;

        let mut ev_lower = Vec::with_capacity(nv);
        let mut ev_blend = Vec::with_capacity(nv);
        for i in 0..nv {
            ev_lower.push(Edge::line_between(vb[i].clone(), vc[i].clone())?);
            let edge = match blend {
                Blend::Fillet(_) => Edge::new(
                    NurbsCurve3::new(
                        2,
                        vec![
                            ControlPoint3::unweighted(pc[i]),
                            ControlPoint3::new(rim_points[i], weight_profile),
                            ControlPoint3::unweighted(pt[i]),
                        ],
                        KnotVector::clamped_uniform(3, 2),
                    )?,
                    vc[i].clone(),
                    vt[i].clone(),
                    1e-6,
                ),
                Blend::Chamfer(_) => Edge::line_between(vc[i].clone(), vt[i].clone())?,
            };
            ev_blend.push(edge);
        }

        // 区間の3通りの写し（底・切り口・天）。円弧は有理2次で1本。
        struct Copy3 {
            bottom: Edge,
            cut: Edge,
            top: Edge,
        }
        let arc_edge = |from: Point3,
                        to: Point3,
                        middle: Point3,
                        weight: f64,
                        a: &Vertex,
                        b: &Vertex|
         -> Result<Edge, String> {
            Ok(Edge::new(
                NurbsCurve3::new(
                    2,
                    vec![
                        ControlPoint3::unweighted(from),
                        ControlPoint3::new(middle, weight),
                        ControlPoint3::unweighted(to),
                    ],
                    KnotVector::clamped_uniform(3, 2),
                )?,
                a.clone(),
                b.clone(),
                1e-6,
            ))
        };

        // 円弧の中間制御点（中心から角の二等分方向へ R / cos(θ/2)）
        let arc_middle = |center: Point3, start: Point3, end: Point3, radius: f64, angle: f64| {
            let bisector = ((start - center) + (end - center)).normalize();
            center + bisector * (radius / (angle * 0.5).cos())
        };

        let mut copies = Vec::with_capacity(count);
        for i in 0..count {
            let next = (i + 1) % nv;
            let copy = match &self.segments[i] {
                Segment::Line { .. } => Copy3 {
                    bottom: Edge::line_between(vb[i].clone(), vb[next].clone())?,
                    cut: Edge::line_between(vc[i].clone(), vc[next].clone())?,
                    top: Edge::line_between(vt[i].clone(), vt[next].clone())?,
                },
                Segment::Arc {
                    start,
                    end,
                    center,
                    radius,
                    angle,
                    concave,
                } => {
                    let weight = (angle * 0.5).cos();
                    let middle = arc_middle(*center, *start, *end, *radius, *angle);
                    let inner_radius = if *concave { radius + r } else { radius - r };
                    let inner_middle = arc_middle(*center, *start, *end, inner_radius, *angle);
                    Copy3 {
                        bottom: arc_edge(
                            pb[i],
                            pb[next],
                            down(middle, self.height),
                            weight,
                            &vb[i],
                            &vb[next],
                        )?,
                        cut: arc_edge(pc[i], pc[next], down(middle, r), weight, &vc[i], &vc[next])?,
                        top: arc_edge(pt[i], pt[next], inner_middle, weight, &vt[i], &vt[next])?,
                    }
                }
            };
            copies.push(copy);
        }

        let mut new_faces = Vec::with_capacity(2 * count + 1);

        // 1. 短くした側面
        for i in 0..count {
            let next = (i + 1) % nv;
            if !self.strip[i] {
                // 一般の平面（4-941）: 稜を切り口に替え、両端の2本を切り口まで詰める
                let original = &solid.outer_shell.faces[self.side_indices[i]];
                new_faces.push(rebuild_general_side(
                    original,
                    &copies[i].cut,
                    [(rim_points[i], pc[i]), (rim_points[next], pc[next])],
                )?);
                continue;
            }
            let geometry = match &self.segments[i] {
                Segment::Line { .. } => FaceGeometry::Plane(
                    PlaneSurface3::new(pb[i], pb[next] - pb[i], pc[i] - pb[i])
                        .ok_or("Failed to create a shortened side plane")?,
                ),
                Segment::Arc {
                    start,
                    end,
                    center,
                    radius,
                    angle,
                    concave,
                } => {
                    let weight = (angle * 0.5).cos();
                    let middle = arc_middle(*center, *start, *end, *radius, *angle);
                    FaceGeometry::Nurbs(NurbsSurface3::new(
                        2,
                        1,
                        vec![
                            vec![ControlPoint3::unweighted(pb[i]), ControlPoint3::unweighted(pc[i])],
                            vec![
                                ControlPoint3::new(down(middle, self.height), weight),
                                ControlPoint3::new(down(middle, r), weight),
                            ],
                            vec![
                                ControlPoint3::unweighted(pb[next]),
                                ControlPoint3::unweighted(pc[next]),
                            ],
                        ],
                        KnotVector::clamped_uniform(3, 2),
                        KnotVector::clamped_uniform(2, 1),
                    )?)
                }
            };
            let wire = Wire::new(vec![
                OrientedEdge::forward(copies[i].bottom.clone()),
                OrientedEdge::forward(ev_lower[next].clone()),
                OrientedEdge::reversed(copies[i].cut.clone()),
                OrientedEdge::reversed(ev_lower[i].clone()),
            ]);
            new_faces.push(Face::simple(geometry, wire));
        }

        // 2. ブレンドの面
        for i in 0..count {
            let next = (i + 1) % nv;
            let (columns, column_weights): (Vec<[Point3; 3]>, Vec<f64>) = match &self.segments[i] {
                Segment::Line { .. } => (
                    vec![
                        [pc[i], rim_points[i], pt[i]],
                        [pc[next], rim_points[next], pt[next]],
                    ],
                    vec![1.0, 1.0],
                ),
                Segment::Arc {
                    start,
                    end,
                    center,
                    radius,
                    angle,
                    concave,
                } => {
                    let weight = (angle * 0.5).cos();
                    let middle = arc_middle(*center, *start, *end, *radius, *angle);
                    let inner_radius = if *concave { radius + r } else { radius - r };
                    let inner_middle = arc_middle(*center, *start, *end, inner_radius, *angle);
                    (
                        vec![
                            [pc[i], rim_points[i], pt[i]],
                            [down(middle, r), middle, inner_middle],
                            [pc[next], rim_points[next], pt[next]],
                        ],
                        vec![1.0, weight, 1.0],
                    )
                }
            };
            // 行 = 断面の向き（切り口 → 角 → 天）、列 = 区間に沿う向き
            let profile: Vec<(usize, f64)> = match blend {
                Blend::Fillet(_) => vec![(0, 1.0), (1, weight_profile), (2, 1.0)],
                Blend::Chamfer(_) => vec![(0, 1.0), (2, 1.0)],
            };
            let mut rows: Vec<Vec<ControlPoint3>> = profile
                .iter()
                .map(|(slot, profile_weight)| {
                    columns
                        .iter()
                        .zip(column_weights.iter())
                        .map(|(column, column_weight)| {
                            ControlPoint3::new(column[*slot], profile_weight * column_weight)
                        })
                        .collect()
                })
                .collect();
            rows.reverse();
            let degree_u = profile.len() - 1;
            let degree_v = columns.len() - 1;
            let surface = NurbsSurface3::new(
                degree_u,
                degree_v,
                rows,
                KnotVector::clamped_uniform(degree_u + 1, degree_u),
                KnotVector::clamped_uniform(degree_v + 1, degree_v),
            )?;
            let wire = Wire::new(vec![
                OrientedEdge::forward(copies[i].cut.clone()),
                OrientedEdge::forward(ev_blend[next].clone()),
                OrientedEdge::reversed(copies[i].top.clone()),
                OrientedEdge::reversed(ev_blend[i].clone()),
            ]);
            new_faces.push(Face::simple(FaceGeometry::Nurbs(surface), wire));
        }

        // 3. 縮めた上面
        let u_axis = (pt[1] - pt[0]).normalize();
        let v_axis = n.cross(&u_axis);
        let plane = PlaneSurface3::new(pt[0], u_axis, v_axis).ok_or("Failed to create top plane")?;
        let mut top_edges: Vec<OrientedEdge> = copies
            .iter()
            .map(|copy| OrientedEdge::forward(copy.top.clone()))
            .collect();
        let mut cap_faces: Vec<(usize, Face)> = Vec::new();
        if let Some(ends) = &self.open {
            // 角の先の稜を、内側へずらした点まで詰める（上面のワイヤの向きで作る）
            let next_trimmed = Edge::line_between(
                vt[nv - 1].clone(),
                Vertex::from_point(ends.next.end_vertex().point),
            )?;
            let prev_trimmed = Edge::line_between(
                Vertex::from_point(ends.prev.start_vertex().point),
                vt[0].clone(),
            )?;
            let faces = &solid.outer_shell.faces;
            let start_corner = (rim_points[0], pt[0], pc[0], &ev_blend[0]);
            let end_corner = (rim_points[nv - 1], pt[nv - 1], pc[nv - 1], &ev_blend[nv - 1]);
            if ends.prev.edge.id == ends.next.edge.id {
                // 前後が同じ1本: 両端を詰めた1本にする。蓋も同じ1枚。
                top_edges.push(OrientedEdge::forward(Edge::line_between(
                    vt[nv - 1].clone(),
                    vt[0].clone(),
                )?));
                cap_faces.push((
                    ends.cap_start,
                    rebuild_cap(
                        &faces[ends.cap_start],
                        ends.prev.edge.id,
                        &[start_corner, end_corner],
                    )?,
                ));
            } else {
                top_edges.push(OrientedEdge::forward(next_trimmed));
                top_edges.extend(ends.rest.iter().cloned());
                top_edges.push(OrientedEdge::forward(prev_trimmed));
                cap_faces.push((
                    ends.cap_start,
                    rebuild_cap(&faces[ends.cap_start], ends.prev.edge.id, &[start_corner])?,
                ));
                cap_faces.push((
                    ends.cap_end,
                    rebuild_cap(&faces[ends.cap_end], ends.next.edge.id, &[end_corner])?,
                ));
            }
        }
        let top_wire = Wire::new(top_edges);
        let holes = solid.outer_shell.faces[self.top_index].inner_wires.clone();
        new_faces.push(Face::new(
            FaceGeometry::Plane(plane),
            top_wire,
            holes,
            zenith_topo::Orientation::Forward,
            1e-6,
        ));

        let mut final_faces: Vec<Face> = solid
            .outer_shell
            .faces
            .iter()
            .enumerate()
            .filter(|(index, _)| *index != self.top_index && !self.side_indices.contains(index))
            .map(|(index, face)| {
                cap_faces
                    .iter()
                    .find(|(cap, _)| *cap == index)
                    .map_or_else(|| face.clone(), |(_, rebuilt)| rebuilt.clone())
            })
            .collect();
        final_faces.extend(new_faces);
        let raw = Solid::try_new(
            Shell::closed(final_faces),
            solid.inner_shells.clone(),
            &Tolerance::default(),
        )
        .map_err(|error| format!("Prism-rim blend produced an invalid solid: {error}"))?;
        let (sewn, _) = crate::Sewer::sew_solid(&raw, &Tolerance::default())
            .map_err(|error| format!("Prism-rim blend sewing failed: {error}"))?;
        Ok(sewn)
    }
}

/// 開いた鎖の端の蓋を作り直す。角ごとに `(角, 内側へずらした点, 切り口の点,
/// 断面)` を受け取り、角に集まる2本（上面と共有する稜 `rim_edge_id` と縦の稜）を
/// それぞれ内側の点・切り口の点まで詰め、できた隙間に断面（切り口 → 内側）を入れる。
/// 新しい稜は作り直すので、隣の面とは最後の `Sewer` が縫い合わせる。
fn rebuild_cap(
    face: &Face,
    rim_edge_id: u64,
    corners: &[(Point3, Point3, Point3, &Edge)],
) -> Result<Face, String> {
    let near = |a: Point3, b: Point3| (a - b).norm() <= 1e-9;
    let mut pieces: Vec<OrientedEdge> = Vec::with_capacity(face.outer_wire.edges.len() + 2);
    let mut replaced = 0usize;
    for oriented in &face.outer_wire.edges {
        let mut from = oriented.start_vertex().point;
        let mut to = oriented.end_vertex().point;
        let mut touched = false;
        for (corner, top_point, cut_point, _) in corners {
            let moved = if oriented.edge.id == rim_edge_id {
                *top_point
            } else {
                *cut_point
            };
            if near(from, *corner) {
                from = moved;
                touched = true;
                replaced += 1;
            } else if near(to, *corner) {
                to = moved;
                touched = true;
                replaced += 1;
            }
        }
        if !touched {
            pieces.push(oriented.clone());
            continue;
        }
        if !is_straight(&oriented.edge) {
            return Err("The cap face of an open prism-rim chain has a curved edge at its corner".into());
        }
        pieces.push(OrientedEdge::forward(Edge::line_between(
            Vertex::from_point(from),
            Vertex::from_point(to),
        )?));
    }
    if replaced != 2 * corners.len() {
        return Err(format!(
            "The cap face of an open prism-rim chain moved {replaced} edge ends, expected {}",
            2 * corners.len()
        ));
    }
    // 隙間を断面で埋める
    let count = pieces.len();
    let mut wire = Vec::with_capacity(count + corners.len());
    for index in 0..count {
        let here_end = pieces[index].end_vertex().point;
        let next_start = pieces[(index + 1) % count].start_vertex().point;
        wire.push(pieces[index].clone());
        if near(here_end, next_start) {
            continue;
        }
        let profile = corners.iter().find_map(|(_, top_point, cut_point, profile)| {
            if near(here_end, *cut_point) && near(next_start, *top_point) {
                Some(OrientedEdge::forward((*profile).clone()))
            } else if near(here_end, *top_point) && near(next_start, *cut_point) {
                Some(OrientedEdge::reversed((*profile).clone()))
            } else {
                None
            }
        });
        wire.push(profile.ok_or("The cap face of an open prism-rim chain did not close")?);
    }
    Ok(Face::new(
        face.geometry.clone(),
        Wire::new(wire),
        face.inner_wires.clone(),
        face.orientation,
        face.tolerance,
    ))
}

pub(crate) fn concave_arc_enabled() -> bool {
    std::env::var("ZENITH_PRISM_CONCAVE_ARC")
        .map(|value| value != "0")
        .unwrap_or(false)
}

pub(crate) fn general_side_enabled() -> bool {
    std::env::var("ZENITH_PRISM_GENERAL_SIDE")
        .map(|value| value != "0")
        .unwrap_or(true)
}

/// 一般の平面の側面で、稜 `rim_edge_id` の両端に隣り合う2本が、上面の法線
/// `normal` の逆向きに真っすぐ下りるか。下りるなら短い方の長さ。
fn general_side_reach(side: &Face, rim_edge_id: u64, normal: Vec3) -> Option<f64> {
    if !side.inner_wires.is_empty() {
        return None;
    }
    let edges = &side.outer_wire.edges;
    let position = edges.iter().position(|e| e.edge.id == rim_edge_id)?;
    let count = edges.len();
    let rim = &edges[position];
    let ends = [rim.start_vertex().point, rim.end_vertex().point];
    let mut reach = f64::INFINITY;
    for neighbour in [&edges[(position + count - 1) % count], &edges[(position + 1) % count]] {
        if neighbour.edge.id == rim_edge_id || !is_straight(&neighbour.edge) {
            return None;
        }
        let (a, b) = (neighbour.start_vertex().point, neighbour.end_vertex().point);
        let (from, to) = if ends.iter().any(|p| (p - a).norm() <= 1e-9) {
            (a, b)
        } else if ends.iter().any(|p| (p - b).norm() <= 1e-9) {
            (b, a)
        } else {
            return None;
        };
        let span = to - from;
        let length = span.norm();
        if length <= 1e-12 || (span / length).dot(&-normal) < 1.0 - 1e-9 {
            return None;
        }
        reach = reach.min(length);
    }
    Some(reach)
}

/// 一般の平面の側面を作り直す: 稜を切り口 `cut`（上面のワイヤの向き）に替え、
/// 稜の両端に触れる稜の端を、切り口の点へ動かす。
fn rebuild_general_side(face: &Face, cut: &Edge, moves: [(Point3, Point3); 2]) -> Result<Face, String> {
    let near = |a: Point3, b: Point3| (a - b).norm() <= 1e-9;
    let rim_ends = [moves[0].0, moves[1].0];
    let mut wire = Vec::with_capacity(face.outer_wire.edges.len());
    for oriented in &face.outer_wire.edges {
        let from = oriented.start_vertex().point;
        let to = oriented.end_vertex().point;
        let is_rim = rim_ends.iter().any(|p| near(*p, from)) && rim_ends.iter().any(|p| near(*p, to));
        if is_rim {
            wire.push(if near(from, moves[0].0) {
                OrientedEdge::forward(cut.clone())
            } else {
                OrientedEdge::reversed(cut.clone())
            });
            continue;
        }
        let moved = |point: Point3| {
            moves
                .iter()
                .find(|(old, _)| near(*old, point))
                .map_or(point, |(_, new)| *new)
        };
        let (new_from, new_to) = (moved(from), moved(to));
        if near(new_from, from) && near(new_to, to) {
            wire.push(oriented.clone());
            continue;
        }
        if !is_straight(&oriented.edge) {
            return Err("A general side face of a prism-rim chain has a curved edge at the rim".into());
        }
        wire.push(OrientedEdge::forward(Edge::line_between(
            Vertex::from_point(new_from),
            Vertex::from_point(new_to),
        )?));
    }
    Ok(Face::new(
        face.geometry.clone(),
        Wire::new(wire),
        face.inner_wires.clone(),
        face.orientation,
        face.tolerance,
    ))
}

fn is_straight(edge: &Edge) -> bool {
    let start = edge.start_vertex.point;
    let end = edge.end_vertex.point;
    let span = end - start;
    let length = span.norm();
    if length <= 1e-12 {
        return false;
    }
    let direction = span / length;
    let (t0, t1) = edge.curve.param_range();
    (1..8).all(|step| {
        let point = edge.curve.evaluate(t0 + (t1 - t0) * step as f64 / 8.0);
        let offset = point - start;
        let along = offset.dot(&direction);
        (offset - direction * along).norm() <= 1e-9 * length.max(1.0)
    })
}

/// 区間の始点（`at_start`）または終点での、進行方向の単位接線。
fn tangent_at(segment: &Segment, at_start: bool, normal: Vec3) -> Option<Vec3> {
    match segment {
        Segment::Line { start, end } => (end - start).try_normalize_safe(1e-12),
        Segment::Arc {
            start,
            end,
            center,
            concave,
            ..
        } => {
            let point = if at_start { *start } else { *end };
            // 凸の円弧は上面の法線まわりに反時計回り、凹は時計回り
            let sign = if *concave { -1.0 } else { 1.0 };
            (normal.cross(&(point - center)) * sign).try_normalize_safe(1e-12)
        }
    }
}

fn rim_point(segments: &[Segment]) -> Point3 {
    segments[0].start()
}

/// 上面の平面内で、点から区間までの距離（点は平面に乗っている前提）。
fn distance_to(segment: &Segment, point: Point3, normal: Vec3) -> f64 {
    match segment {
        Segment::Line { start, end } => {
            let span = end - start;
            let t = ((point - start).dot(&span) / span.norm_squared()).clamp(0.0, 1.0);
            (point - (start + span * t)).norm()
        }
        Segment::Arc {
            start,
            end,
            center,
            radius,
            angle,
            concave,
        } => {
            let offset = point - center;
            let flat = offset - normal * offset.dot(&normal);
            let a = (start - center) / *radius;
            let b = (end - center) / *radius;
            // a から b へ回る向き（凹の円弧は逆回り）
            let perpendicular = (b - a * a.dot(&b)).normalize();
            let phi = flat.dot(&perpendicular).atan2(flat.dot(&a));
            if phi >= 0.0 && phi <= *angle && flat.norm() > 1e-12 {
                (flat.norm() - radius).abs()
            } else {
                (point - start).norm().min((point - end).norm())
            }
        }
    }
}

fn point_on(segment: &Segment, fraction: f64) -> Point3 {
    match segment {
        Segment::Line { start, end } => start + (end - start) * fraction,
        Segment::Arc {
            start,
            end,
            center,
            radius,
            angle,
            concave,
        } => {
            let a = (start - center) / *radius;
            let b = (end - center) / *radius;
            // a から b へ回る平面内の直交基底
            let perpendicular = (b - a * a.dot(&b)).normalize();
            let theta = angle * fraction;
            center + (a * theta.cos() + perpendicular * theta.sin()) * *radius
        }
    }
}
