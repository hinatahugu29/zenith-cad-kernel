//! 任意ソリッドの1本の稜に対するフィレット / 面取り。
//!
//! これまでのフィレットと面取りは `fillet_box_z_edges(dx, dy, dz, r)` のように
//! **寸法から直方体を作り直す**ビルダーでした。ブーリアンやロフトで出来た
//! 立体の稜を丸めることはできません。ここは `&Solid` と稜の ID を受け取り、
//! **元の立体のトポロジーを編集して**丸める演算子です。
//!
//! ## 扱える配置
//!
//! - 稜が直線であること
//! - 稜を共有する面がちょうど2枚で、どちらも平面であること
//! - 稜の両端の頂点にちょうど3枚目の面があり、それが平面で稜と直交すること
//! - 稜が凸であること（二面角が 180 度未満）
//!
//! この条件は「押し出し・角柱・それらのブーリアン結果の縦稜」をすべて含みます。
//! 加えて、純粋な直円柱と純粋な直円錐/円錐台の**平面キャップ × 回転側面の円形稜**を
//! 扱います。円弧1本を選ぶと滑らかな全周チェーンへ伝播し、フィレットは厳密な
//! 有理トーラス、円柱面取りは厳密な円錐台パッチで置き換えます。自作4分割円弧、
//! 外部CADの全周1本円、剛体配置後を同じ経路で認識します。ボス等の複合立体と
//! 円錐の円周面取りはまだ対象外です。
//! 満たさない配置は**近い別の形を返さず、理由を返して失敗します**。
//!
//! ## 測れること
//!
//! 二面角 `θ`、稜長 `L` に対して削れる体積は閉じた式で決まります。
//!
//! - フィレット半径 `r`: `L * r^2 * (cot(θ/2) - (π - θ)/2)`
//! - 面取り距離 `c`  : `L * c^2 * sin(θ) / 2`
//!
//! θ = 90 度を入れると、それぞれ `L r^2 (1 - π/4)` と `L c^2 / 2` という
//! 直方体版のテストが使っていた式に一致します。

use std::collections::BTreeMap;

use zenith_geom::{ControlPoint3, KnotVector, NurbsCurve3, NurbsSurface3, PlaneSurface3};
use zenith_math::{Point3, Tolerance, Vec3, Vec3Ext};
use zenith_topo::{Edge, Face, FaceGeometry, Orientation, OrientedEdge, Shell, Solid, Vertex, Wire};

/// 稜に施すブレンドの種類
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum BlendKind {
    /// 半径 `radius` の真円フィレット（有理2次曲面）
    Fillet { radius: f64 },
    /// 両面から等距離 `distance` の平面面取り
    Chamfer { distance: f64 },
}

impl BlendKind {
    /// 各面に沿って稜から後退する距離
    fn setback(&self, dihedral: f64) -> f64 {
        match self {
            // r * cot(θ/2)
            BlendKind::Fillet { radius } => radius / (dihedral * 0.5).tan(),
            BlendKind::Chamfer { distance } => *distance,
        }
    }

    /// 稜1単位長あたりに削れる断面積
    fn removed_section_area(&self, dihedral: f64) -> f64 {
        match self {
            BlendKind::Fillet { radius } => {
                let r2 = radius * radius;
                r2 * (1.0 / (dihedral * 0.5).tan() - 0.5 * (std::f64::consts::PI - dihedral))
            }
            BlendKind::Chamfer { distance } => 0.5 * distance * distance * dihedral.sin(),
        }
    }
}

/// ブレンドが実際に何をしたかの実測値
#[derive(Debug, Clone, PartialEq)]
pub struct EdgeBlendReport {
    /// 内側から測った二面角 (deg)
    pub dihedral_angle_deg: f64,
    /// 稜長
    pub edge_length: f64,
    /// 各面に沿った後退距離
    pub setback: f64,
    /// 閉じた式が予告する削れ体積
    pub predicted_removed_volume: f64,
}

/// 稜の性質だけを見るための下調べ結果
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct BlendableEdge {
    pub edge_id: u64,
    pub length: f64,
    pub dihedral_angle_deg: f64,
    /// この稜に施せるフィレット半径の上限（後退距離が隣接稜を食い切る手前）
    pub max_fillet_radius: f64,
    /// 同じく面取り距離の上限
    pub max_chamfer_distance: f64,
}

/// 任意ソリッドの稜に対するフィレット / 面取り演算子
pub struct EdgeBlender;

impl EdgeBlender {
    /// 半径 `radius` のフィレットを稜 `edge_id` に施す
    pub fn fillet_edge(solid: &Solid, edge_id: u64, radius: f64) -> Result<Solid, String> {
        if !(radius > 0.0) {
            return Err(format!("Fillet radius must be positive, got {radius}"));
        }
        Ok(Self::blend_edge(solid, edge_id, BlendKind::Fillet { radius })?.0)
    }

    /// 両面から `distance` 後退する面取りを稜 `edge_id` に施す
    pub fn chamfer_edge(solid: &Solid, edge_id: u64, distance: f64) -> Result<Solid, String> {
        if !(distance > 0.0) {
            return Err(format!("Chamfer distance must be positive, got {distance}"));
        }
        Ok(Self::blend_edge(solid, edge_id, BlendKind::Chamfer { distance })?.0)
    }

    /// 複数の稜に順に施す。稜 ID は施行のたびに保たれるので、呼び出し前に
    /// 一度だけ集めた ID をそのまま渡せる。頂点を共有する稜同士を同時に
    /// 指定した場合は、2本目で「3枚目の面」の条件が崩れて明示的に失敗する。
    pub fn fillet_edges(solid: &Solid, edges: &[(u64, f64)]) -> Result<Solid, String> {
        let mut current = solid.clone();
        for (index, (edge_id, radius)) in edges.iter().enumerate() {
            current = Self::fillet_edge(&current, *edge_id, *radius)
                .map_err(|err| format!("fillet {index} (edge {edge_id}): {err}"))?;
        }
        Ok(current)
    }

    /// 同じく面取りを順に施す
    pub fn chamfer_edges(solid: &Solid, edges: &[(u64, f64)]) -> Result<Solid, String> {
        let mut current = solid.clone();
        for (index, (edge_id, distance)) in edges.iter().enumerate() {
            current = Self::chamfer_edge(&current, *edge_id, *distance)
                .map_err(|err| format!("chamfer {index} (edge {edge_id}): {err}"))?;
        }
        Ok(current)
    }

    /// この立体で実際にブレンドできる稜を列挙する。
    ///
    /// 「試して失敗する」以外の方法で対象を選べるようにするためのもので、
    /// ここに出る稜は同じ条件判定を通っている。
    pub fn blendable_edges(solid: &Solid) -> Vec<BlendableEdge> {
        let mut ids: Vec<u64> = Vec::new();
        for face in &solid.outer_shell.faces {
            for wire in std::iter::once(&face.outer_wire).chain(face.inner_wires.iter()) {
                for oriented in &wire.edges {
                    if !ids.contains(&oriented.edge.id) {
                        ids.push(oriented.edge.id);
                    }
                }
            }
        }

        let mut out = Vec::new();
        for id in ids {
            if let Ok(site) = BlendSite::locate(solid, id) {
                // 後退距離が隣接稜の長さを食い切らない範囲を上限とする
                let max_setback = site.max_setback * 0.999;
                let half = site.dihedral * 0.5;
                out.push(BlendableEdge {
                    edge_id: id,
                    length: site.length,
                    dihedral_angle_deg: site.dihedral.to_degrees(),
                    max_fillet_radius: max_setback * half.tan(),
                    max_chamfer_distance: max_setback,
                });
            } else if let Some(edge) =
                crate::circular_fillet::circular_cylinder_blendable(solid, id)
            {
                out.push(edge);
            } else if let Some(edge) = crate::circular_fillet::cone::conical_rim_blendable(solid, id)
            {
                out.push(edge);
            }
        }
        out
    }

    /// フィレットと面取りの共通実装
    pub fn blend_edge(
        solid: &Solid,
        edge_id: u64,
        kind: BlendKind,
    ) -> Result<(Solid, EdgeBlendReport), String> {
        if let BlendKind::Fillet { radius } = kind {
            if let Some(result) =
                crate::circular_fillet::try_fillet_cylinder_rim(solid, edge_id, radius)?
            {
                return Ok(result);
            }
            if let Some(result) =
                crate::circular_fillet::cone::try_fillet_conical_rim(solid, edge_id, radius)?
            {
                return Ok(result);
            }
        }
        if let BlendKind::Chamfer { distance } = kind {
            if let Some(result) =
                crate::circular_fillet::try_chamfer_cylinder_rim(solid, edge_id, distance)?
            {
                return Ok(result);
            }
        }
        let site = BlendSite::locate(solid, edge_id)?;
        let setback = kind.setback(site.dihedral);

        if !(setback > 0.0) || !setback.is_finite() {
            return Err(format!(
                "Blend setback came out as {setback}, which is not a usable distance"
            ));
        }
        if setback >= site.max_setback {
            return Err(format!(
                "Blend needs {setback:.6} of setback along each face but the shortest neighbouring edge only allows {:.6}",
                site.max_setback
            ));
        }

        let report = EdgeBlendReport {
            dihedral_angle_deg: site.dihedral.to_degrees(),
            edge_length: site.length,
            setback,
            predicted_removed_volume: kind.removed_section_area(site.dihedral) * site.length,
        };

        let solid = site.apply(solid, kind, setback)?;
        Ok((solid, report))
    }
}

/// 稜とその周りの、ブレンドに必要な配置をすべて解いた状態
struct BlendSite {
    edge_id: u64,
    /// 稜を共有する2面の添字。`t1 x t2 . d > 0` になるよう並べ替えてある
    face_a: usize,
    face_b: usize,
    /// 稜の始点・終点（`d` は始点から終点向き）
    v_start: Point3,
    v_end: Point3,
    d: Vec3,
    length: f64,
    /// 面 a / b の外向き法線
    n_a: Vec3,
    n_b: Vec3,
    /// 稜から各面の内側へ向かう単位ベクトル（`d` と直交）
    t_a: Vec3,
    t_b: Vec3,
    /// 内側から測った二面角 (rad)
    dihedral: f64,
    /// 端の面（始点側・終点側）
    cap_start: usize,
    cap_end: usize,
    /// 端を詰める隣接稜: (edge_id, その稜のどちら側の端点を動かすか, 方向 t)
    neighbours: Vec<NeighbourTrim>,
    /// 隣接稜が許す後退距離の上限
    max_setback: f64,
}

#[derive(Debug, Clone, Copy)]
struct NeighbourTrim {
    edge_id: u64,
    /// true なら start_vertex を、false なら end_vertex を動かす
    move_start: bool,
    /// この稜が許す後退距離
    available: f64,
    /// 動かす頂点が稜の始点側か終点側か
    at_start_of_target: bool,
    /// 面 a 側か b 側か
    on_face_a: bool,
}

impl BlendSite {
    fn locate(solid: &Solid, edge_id: u64) -> Result<Self, String> {
        let faces = &solid.outer_shell.faces;

        // 1. 稜を共有する面と、その面での進行方向
        let mut uses: Vec<(usize, Orientation)> = Vec::new();
        let mut edge: Option<Edge> = None;
        for (index, face) in faces.iter().enumerate() {
            for wire in std::iter::once(&face.outer_wire).chain(face.inner_wires.iter()) {
                for oriented in &wire.edges {
                    if oriented.edge.id == edge_id {
                        edge = Some(oriented.edge.clone());
                        uses.push((index, oriented.orientation));
                    }
                }
            }
        }
        let edge = edge.ok_or_else(|| format!("Edge {edge_id} is not in this solid"))?;
        if uses.len() != 2 {
            return Err(format!(
                "Edge {edge_id} is used by {} face loops; blending needs exactly two",
                uses.len()
            ));
        }
        if uses[0].0 == uses[1].0 {
            return Err(format!(
                "Edge {edge_id} is used twice by the same face; blending needs two distinct faces"
            ));
        }

        let v_start = edge.start_vertex.point;
        let v_end = edge.end_vertex.point;
        let length = (v_end - v_start).norm();
        if length <= 1e-12 {
            return Err(format!("Edge {edge_id} has no length"));
        }
        let d = (v_end - v_start) / length;

        // 稜は直線でなければならない。曲がっていれば近似せずに断る。
        let deviation = max_chord_deviation(&edge.curve, v_start, d, length);
        if deviation > 1e-9 * length.max(1.0) {
            return Err(format!(
                "Edge {edge_id} is not a straight line (it leaves its chord by {deviation:.3e})"
            ));
        }

        // 2. 両面の外向き法線と、稜から面の内側を向く方向
        let mut side = Vec::new();
        for (face_index, orientation) in &uses {
            let face = &faces[*face_index];
            let FaceGeometry::Plane(plane) = &face.geometry else {
                return Err(format!(
                    "Face {face_index} beside edge {edge_id} is not planar; only planar pairs are blended"
                ));
            };
            let normal = if face.orientation.is_forward() {
                plane.normal
            } else {
                -plane.normal
            };
            // ワイヤは外向き法線まわりに反時計回りなので、進行方向の左が面の内側
            let travel = if orientation.is_forward() { d } else { -d };
            let inward = normal.cross(&travel);
            let inward = inward
                .try_normalize_safe(1e-12)
                .ok_or_else(|| format!("Face {face_index} normal is parallel to edge {edge_id}"))?;
            side.push((*face_index, normal, inward));
        }

        let (mut fa, mut na, mut ta) = side[0];
        let (mut fb, mut nb, mut tb) = side[1];
        // t_a x t_b . d > 0 になる並びに正規化しておくと、以降の向きが一意に決まる
        let handedness = ta.cross(&tb).dot(&d);
        if handedness.abs() < 1e-9 {
            return Err(format!(
                "Edge {edge_id} is tangential (the two faces meet without a corner); nothing to blend"
            ));
        }
        if handedness < 0.0 {
            std::mem::swap(&mut fa, &mut fb);
            std::mem::swap(&mut na, &mut nb);
            std::mem::swap(&mut ta, &mut tb);
        }

        // 3. 二面角。凸なら面 b の内側は面 a の外側から見て下にある。
        let cos = ta.dot(&tb).clamp(-1.0, 1.0);
        let raw = cos.acos();
        let dihedral = if tb.dot(&na) < 0.0 {
            raw
        } else {
            std::f64::consts::TAU - raw
        };
        if dihedral >= std::f64::consts::PI - 1e-9 {
            return Err(format!(
                "Edge {edge_id} has a {:.3} deg interior angle; only convex edges are blended",
                dihedral.to_degrees()
            ));
        }
        if dihedral <= 1e-9 {
            return Err(format!("Edge {edge_id} has a degenerate interior angle"));
        }

        // 4. 端の面。稜と直交する平面でなければ、丸めた面の端を平面で
        //    閉じられないのでここで断る。
        let cap_start = find_cap_face(faces, v_start, fa, fb, d, edge_id, "start")?;
        let cap_end = find_cap_face(faces, v_end, fa, fb, d, edge_id, "end")?;

        // 5. 端で詰める隣接稜
        let mut neighbours = Vec::new();
        for (at_start, vertex) in [(true, v_start), (false, v_end)] {
            for (on_face_a, face_index, dir) in [(true, fa, ta), (false, fb, tb)] {
                let trim = find_neighbour(
                    &faces[face_index],
                    edge_id,
                    vertex,
                    dir,
                    at_start,
                    on_face_a,
                    face_index,
                )?;
                neighbours.push(trim);
            }
        }
        let max_setback = neighbours
            .iter()
            .map(|n| n.available)
            .fold(f64::INFINITY, f64::min);

        Ok(Self {
            edge_id,
            face_a: fa,
            face_b: fb,
            v_start,
            v_end,
            d,
            length,
            n_a: na,
            n_b: nb,
            t_a: ta,
            t_b: tb,
            dihedral,
            cap_start,
            cap_end,
            neighbours,
            max_setback,
        })
    }

    fn apply(&self, solid: &Solid, kind: BlendKind, setback: f64) -> Result<Solid, String> {
        let tol = Tolerance::default();
        let faces = &solid.outer_shell.faces;

        // 新しい4頂点。稜の両端が、各面側へ setback だけ退いた点に割れる。
        let a_start = Vertex::from_point(self.v_start + self.t_a * setback);
        let a_end = Vertex::from_point(self.v_end + self.t_a * setback);
        let b_start = Vertex::from_point(self.v_start + self.t_b * setback);
        let b_end = Vertex::from_point(self.v_end + self.t_b * setback);

        // 面 a / b に残る、稜を置き換える直線
        let line_a = Edge::line_between(a_start.clone(), a_end.clone())?;
        let line_b = Edge::line_between(b_start.clone(), b_end.clone())?;

        // 端の稜。フィレットなら円弧、面取りなら直線。
        let corner_start = self.corner_edge(
            kind,
            setback,
            self.v_start,
            a_start.clone(),
            b_start.clone(),
        )?;
        let corner_end =
            self.corner_edge(kind, setback, self.v_end, a_end.clone(), b_end.clone())?;

        // 隣接稜を詰める。同じ稜の両端が動くこともあるので id ごとにまとめる。
        let mut trims: BTreeMap<u64, (Option<Vertex>, Option<Vertex>)> = BTreeMap::new();
        for neighbour in &self.neighbours {
            let vertex = match (neighbour.at_start_of_target, neighbour.on_face_a) {
                (true, true) => a_start.clone(),
                (true, false) => b_start.clone(),
                (false, true) => a_end.clone(),
                (false, false) => b_end.clone(),
            };
            let slot = trims.entry(neighbour.edge_id).or_insert((None, None));
            if neighbour.move_start {
                slot.0 = Some(vertex);
            } else {
                slot.1 = Some(vertex);
            }
        }

        let mut trimmed: BTreeMap<u64, Edge> = BTreeMap::new();
        for (id, (new_start, new_end)) in &trims {
            let original = find_edge(faces, *id)
                .ok_or_else(|| format!("Neighbouring edge {id} vanished while trimming"))?;
            let start = new_start.clone().unwrap_or_else(|| original.start_vertex.clone());
            let end = new_end.clone().unwrap_or_else(|| original.end_vertex.clone());
            if (end.point - start.point).norm() <= 1e-12 {
                return Err(format!(
                    "Trimming neighbouring edge {id} would collapse it; the blend is too large here"
                ));
            }
            let curve = NurbsCurve3::bspline_from_points(1, vec![start.point, end.point])?;
            trimmed.insert(
                *id,
                Edge {
                    id: *id,
                    curve,
                    start_vertex: start,
                    end_vertex: end,
                    tolerance: original.tolerance,
                },
            );
        }

        // 既存の面を作り直す
        let mut new_faces: Vec<Face> = Vec::with_capacity(faces.len() + 1);
        for (index, face) in faces.iter().enumerate() {
            let substitute = |oriented: &OrientedEdge| -> OrientedEdge {
                if oriented.edge.id == self.edge_id {
                    debug_assert!(index == self.face_a || index == self.face_b);
                    let replacement = if index == self.face_a { &line_a } else { &line_b };
                    OrientedEdge::new(replacement.clone(), oriented.orientation)
                } else if let Some(edge) = trimmed.get(&oriented.edge.id) {
                    OrientedEdge::new(edge.clone(), oriented.orientation)
                } else {
                    oriented.clone()
                }
            };

            let touches_start = index == self.cap_start;
            let touches_end = index == self.cap_end;

            let rebuild = |wire: &Wire| -> Result<Wire, String> {
                let edges: Vec<OrientedEdge> = wire.edges.iter().map(&substitute).collect();
                if touches_start || touches_end {
                    close_gap(edges, &[&corner_start, &corner_end], &tol)
                } else {
                    Ok(Wire::new(edges))
                }
            };

            let outer = rebuild(&face.outer_wire)?;
            let mut inners = Vec::with_capacity(face.inner_wires.len());
            for wire in &face.inner_wires {
                inners.push(rebuild(wire)?);
            }

            new_faces.push(Face::new(
                face.geometry.clone(),
                outer,
                inners,
                face.orientation,
                face.tolerance,
            ));
        }

        // ブレンド面そのもの
        let blend_wire = Wire::new(vec![
            OrientedEdge::forward(line_a.clone()),
            OrientedEdge::forward(corner_end.clone()),
            OrientedEdge::reversed(line_b.clone()),
            OrientedEdge::reversed(corner_start.clone()),
        ]);
        let geometry = self.blend_geometry(kind, setback, &a_start, &a_end, &b_start, &b_end)?;
        new_faces.push(Face::simple(geometry, blend_wire));

        Solid::try_new(Shell::closed(new_faces), solid.inner_shells.clone(), &tol)
            .map_err(|err| err.to_string())
    }

    /// 稜の端に入る新しい辺。フィレットは有理2次円弧、面取りは直線。
    fn corner_edge(
        &self,
        kind: BlendKind,
        setback: f64,
        apex: Point3,
        on_a: Vertex,
        on_b: Vertex,
    ) -> Result<Edge, String> {
        match kind {
            BlendKind::Chamfer { .. } => Edge::line_between(on_a, on_b),
            BlendKind::Fillet { .. } => {
                let _ = setback;
                // 制御点の中央は2本の接線の交点、つまり元の稜の端点そのもの。
                // 重みは弧の開き角 (π - θ) の半分の余弦 = sin(θ/2)。
                let weight = (self.dihedral * 0.5).sin();
                let curve = NurbsCurve3::new(
                    2,
                    vec![
                        ControlPoint3::unweighted(on_a.point),
                        ControlPoint3::new(apex, weight),
                        ControlPoint3::unweighted(on_b.point),
                    ],
                    KnotVector::clamped_uniform(3, 2),
                )?;
                Ok(Edge::new(curve, on_a, on_b, 1e-6))
            }
        }
    }

    /// ブレンド面の支持曲面。向きは作ってから法線を測って決める。
    fn blend_geometry(
        &self,
        kind: BlendKind,
        setback: f64,
        a_start: &Vertex,
        a_end: &Vertex,
        b_start: &Vertex,
        b_end: &Vertex,
    ) -> Result<FaceGeometry, String> {
        // 面の中央での外向き方向。凸稜のブレンドは、両面法線の二等分線を向く。
        let outward = (self.n_a + self.n_b)
            .try_normalize_safe(1e-12)
            .ok_or("The two faces beside this edge are exactly opposed")?;

        match kind {
            BlendKind::Chamfer { .. } => {
                let _ = setback;
                let across = a_start.point - b_start.point;
                let plane = PlaneSurface3::new(b_start.point, across, self.d)
                    .ok_or("Chamfer plane is degenerate")?;
                let plane = if plane.normal.dot(&outward) < 0.0 {
                    PlaneSurface3::new(b_start.point, -across, self.d)
                        .ok_or("Chamfer plane is degenerate")?
                } else {
                    plane
                };
                Ok(FaceGeometry::Plane(plane))
            }
            BlendKind::Fillet { .. } => {
                let weight = (self.dihedral * 0.5).sin();
                let rows_b_first = vec![
                    vec![
                        ControlPoint3::unweighted(b_start.point),
                        ControlPoint3::unweighted(b_end.point),
                    ],
                    vec![
                        ControlPoint3::new(self.v_start, weight),
                        ControlPoint3::new(self.v_end, weight),
                    ],
                    vec![
                        ControlPoint3::unweighted(a_start.point),
                        ControlPoint3::unweighted(a_end.point),
                    ],
                ];
                let mut rows = rows_b_first;
                let mut surface = NurbsSurface3::new(
                    2,
                    1,
                    rows.clone(),
                    KnotVector::clamped_uniform(3, 2),
                    KnotVector::clamped_uniform(2, 1),
                )?;
                let normal = surface
                    .normal(0.5, 0.5)
                    .ok_or("Fillet surface has no normal at its centre")?;
                if normal.dot(&outward) < 0.0 {
                    rows.reverse();
                    surface = NurbsSurface3::new(
                        2,
                        1,
                        rows,
                        KnotVector::clamped_uniform(3, 2),
                        KnotVector::clamped_uniform(2, 1),
                    )?;
                }
                Ok(FaceGeometry::Nurbs(surface))
            }
        }
    }
}

/// 稜の端の頂点にある、稜を共有する2面以外の面を探す
fn find_cap_face(
    faces: &[Face],
    vertex: Point3,
    face_a: usize,
    face_b: usize,
    d: Vec3,
    edge_id: u64,
    which: &str,
) -> Result<usize, String> {
    let mut found = Vec::new();
    for (index, face) in faces.iter().enumerate() {
        if index == face_a || index == face_b {
            continue;
        }
        let touches = std::iter::once(&face.outer_wire)
            .chain(face.inner_wires.iter())
            .any(|wire| {
                wire.edges
                    .iter()
                    .any(|oriented| (oriented.start_vertex().point - vertex).norm() <= 1e-9)
            });
        if touches {
            found.push(index);
        }
    }

    if found.len() != 1 {
        return Err(format!(
            "The {which} of edge {edge_id} has {} other faces around it; blending needs exactly one",
            found.len()
        ));
    }

    let index = found[0];
    let FaceGeometry::Plane(plane) = &faces[index].geometry else {
        return Err(format!(
            "The face closing the {which} of edge {edge_id} is not planar"
        ));
    };
    if plane.normal.dot(&d).abs() < 1.0 - 1e-9 {
        return Err(format!(
            "The face closing the {which} of edge {edge_id} is not perpendicular to it; the blend would need a trimmed end"
        ));
    }
    Ok(index)
}

/// ある面のワイヤで、対象の稜と頂点を共有するもう1本の稜を探す
fn find_neighbour(
    face: &Face,
    edge_id: u64,
    vertex: Point3,
    dir: Vec3,
    at_start_of_target: bool,
    on_face_a: bool,
    face_index: usize,
) -> Result<NeighbourTrim, String> {
    for wire in std::iter::once(&face.outer_wire).chain(face.inner_wires.iter()) {
        for oriented in &wire.edges {
            let edge = &oriented.edge;
            if edge.id == edge_id {
                continue;
            }
            let move_start = if (edge.start_vertex.point - vertex).norm() <= 1e-9 {
                true
            } else if (edge.end_vertex.point - vertex).norm() <= 1e-9 {
                false
            } else {
                continue;
            };

            let (from, to) = if move_start {
                (edge.start_vertex.point, edge.end_vertex.point)
            } else {
                (edge.end_vertex.point, edge.start_vertex.point)
            };
            let span = to - from;
            let available = span.norm();
            if available <= 1e-12 {
                return Err(format!("Neighbouring edge {} has no length", edge.id));
            }
            // 退く先が本当にこの稜の上に乗るか。乗らないなら、端の面が
            // 直交していないなど前提が崩れている。
            let along = span / available;
            if along.dot(&dir) < 1.0 - 1e-7 {
                return Err(format!(
                    "Neighbouring edge {} on face {face_index} does not run in the direction the blend retreats",
                    edge.id
                ));
            }
            let deviation = max_chord_deviation(&edge.curve, edge.start_vertex.point, along * if move_start { 1.0 } else { -1.0 }, available);
            if deviation > 1e-9 * available.max(1.0) {
                return Err(format!(
                    "Neighbouring edge {} is not straight; the blend would need it trimmed as a curve",
                    edge.id
                ));
            }

            return Ok(NeighbourTrim {
                edge_id: edge.id,
                move_start,
                available,
                at_start_of_target,
                on_face_a,
            });
        }
    }
    Err(format!(
        "Face {face_index} has no other edge at the end of edge {edge_id}"
    ))
}

/// 端を詰めた結果できた1箇所の切れ目を、候補の稜のどちらかで閉じる
fn close_gap(
    edges: Vec<OrientedEdge>,
    candidates: &[&Edge],
    tol: &Tolerance,
) -> Result<Wire, String> {
    let count = edges.len();
    for position in 0..count {
        let next = (position + 1) % count;
        let end = edges[position].end_vertex().point;
        let start = edges[next].start_vertex().point;
        if (start - end).norm() <= tol.linear {
            continue;
        }

        for candidate in candidates {
            let cs = candidate.start_vertex.point;
            let ce = candidate.end_vertex.point;
            let oriented = if (cs - end).norm() <= 1e-9 && (ce - start).norm() <= 1e-9 {
                Some(OrientedEdge::forward((*candidate).clone()))
            } else if (ce - end).norm() <= 1e-9 && (cs - start).norm() <= 1e-9 {
                Some(OrientedEdge::reversed((*candidate).clone()))
            } else {
                None
            };
            if let Some(oriented) = oriented {
                let mut filled = edges;
                filled.insert(next, oriented);
                return Ok(Wire::new(filled));
            }
        }

        return Err(
            "A trimmed loop left a gap that neither end of the blend closes".to_string(),
        );
    }

    Ok(Wire::new(edges))
}

fn find_edge(faces: &[Face], edge_id: u64) -> Option<Edge> {
    for face in faces {
        for wire in std::iter::once(&face.outer_wire).chain(face.inner_wires.iter()) {
            for oriented in &wire.edges {
                if oriented.edge.id == edge_id {
                    return Some(oriented.edge.clone());
                }
            }
        }
    }
    None
}

/// 曲線が始点から `dir` に伸びる弦からどれだけ離れるか
fn max_chord_deviation(curve: &NurbsCurve3, origin: Point3, dir: Vec3, length: f64) -> f64 {
    let mut worst: f64 = 0.0;
    for step in 0..=16 {
        let t = step as f64 / 16.0;
        let point = curve.evaluate(t);
        let offset = point - origin;
        let along = offset.dot(&dir).clamp(0.0, length);
        worst = worst.max((offset - dir * along).norm());
    }
    worst
}
