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
//! 外部CADの全周1本円、剛体配置後を同じ経路で認識します。さらに平面板の
//! 貫通円筒穴口は、選択した穴だけを局所再トリムしてフィレット/面取りできます。
//! 円筒ボス根元と段付き軸の小径側肩、平面肩と直円錐ボスの根元も、局所フィレット
//! / 面取りできます。非円形根元はまだ対象外です。
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
use zenith_topo::{
    Edge, Face, FaceGeometry, Orientation, OrientedEdge, Shell, Solid, Vertex, Wire,
};

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
            // 凹の稜では cot が負になるので絶対値（4-938）。凸では値は変わらない。
            BlendKind::Fillet { radius } => (radius / (dihedral * 0.5).tan()).abs(),
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
    /// 閉じた式が予告する符号付き除去体積（正は除去、負は材料追加）
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
    /// 1本の稜が現在の局所ブレンド契約を満たすか調べる。
    ///
    /// 成功時は許容寸法を返し、未対応の配置では `blend_edge` と同じ幾何判定に
    /// 基づく理由を返す。UI や外部ツールが、形状を変更せずに対象選択を検証する
    /// ための入口である。
    pub fn blendability(solid: &Solid, edge_id: u64) -> Result<BlendableEdge, String> {
        match Self::blendability_direct(solid, edge_id) {
            Ok(edge) => Ok(edge),
            Err(err) => match merged_twin(solid, edge_id) {
                Some((merged, twin)) => Self::blendability_direct(&merged, twin)
                    .map(|edge| BlendableEdge { edge_id, ..edge })
                    .map_err(|_| err),
                None => Err(err),
            },
        }
    }

    fn blendability_direct(solid: &Solid, edge_id: u64) -> Result<BlendableEdge, String> {
        match BlendSite::locate(solid, edge_id) {
            Ok(site) => {
                let max_setback = site.max_setback.min(site.hole_clearance(solid)) * 0.999;
                let half = site.dihedral * 0.5;
                Ok(BlendableEdge {
                    edge_id,
                    length: site.length,
                    dihedral_angle_deg: site.dihedral.to_degrees(),
                    // 円柱に当たる端は面取りだけ（4-962）
                    max_fillet_radius: if site.cyl_end.is_some() {
                        0.0
                    } else if site.cyl_side.is_some() {
                        site.cyl_side_max_fillet(max_setback)
                    } else {
                        max_setback * half.tan().abs()
                    },
                    max_chamfer_distance: match site.cyl_side {
                        Some(cs) => max_setback.min(cs.chord_limit * 0.999),
                        None => max_setback,
                    },
                })
            }
            Err(general_error) => {
                if let Some(edge) =
                    crate::circular_fillet::circular_cylinder_blendable(solid, edge_id)
                        .or_else(|| {
                            crate::circular_fillet::hole_mouth::hole_mouth_blendable(solid, edge_id)
                        })
                        .or_else(|| {
                            crate::circular_fillet::slot_hole_mouth::slot_hole_mouth_blendable(
                                solid, edge_id,
                            )
                        })
                        .or_else(|| {
                            crate::circular_fillet::shoulder_root::shoulder_root_blendable(
                                solid, edge_id,
                            )
                        })
                        .or_else(|| {
                            crate::circular_fillet::slot_root::slot_root_blendable(solid, edge_id)
                        })
                        .or_else(|| {
                            crate::circular_fillet::slot_rim::slot_rim_blendable(solid, edge_id)
                        })
                        .or_else(|| {
                            crate::circular_fillet::cone::conical_rim_blendable(solid, edge_id)
                        })
                        .or_else(|| {
                            crate::circular_fillet::sphere_cap::sphere_cap_rim_blendable(
                                solid, edge_id,
                            )
                        })
                        .or_else(|| {
                            crate::circular_fillet::prism_rim::prism_rim_blendable(solid, edge_id)
                        })
                        .or_else(|| arc_rim_blendable(solid, edge_id))
                        .or_else(|| {
                            crate::circular_fillet::revolved_mouth::revolved_mouth_blendable(solid, edge_id)
                        })
                        .or_else(|| {
                            crate::circular_fillet::revolved_corner::revolved_corner_blendable(solid, edge_id)
                        })
                {
                    Ok(edge)
                } else {
                    Err(general_error)
                }
            }
        }
    }

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
        let skip = skip_consumed_enabled();
        let mut current = solid.clone();
        // 前の丸めの鎖が一緒に丸めて消した稜と、その半径（4-947）
        let mut consumed: Vec<(u64, f64)> = Vec::new();
        for (index, (edge_id, radius)) in edges.iter().enumerate() {
            if skip && !edge_ids(&current).contains(edge_id) {
                if let Some((_, used)) = consumed.iter().find(|(id, _)| id == edge_id) {
                    if (used - radius).abs() <= 1e-12 * radius.abs().max(1.0) {
                        continue;
                    }
                    return Err(format!(
                        "fillet {index} (edge {edge_id}): already rounded with radius {used} by an earlier chain, asked for {radius}"
                    ));
                }
            }
            let before = if skip { edge_ids(&current) } else { Vec::new() };
            current = Self::fillet_edge(&current, *edge_id, *radius)
                .map_err(|err| format!("fillet {index} (edge {edge_id}): {err}"))?;
            if skip {
                let after = edge_ids(&current);
                consumed.extend(
                    before.into_iter().filter(|id| !after.contains(id)).map(|id| (id, *radius)),
                );
            }
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

        let out = ids
            .into_iter()
            .filter_map(|id| Self::blendability(solid, id).ok())
            .collect();
        out
    }

    /// フィレットと面取りの共通実装
    pub fn blend_edge(
        solid: &Solid,
        edge_id: u64,
        kind: BlendKind,
    ) -> Result<(Solid, EdgeBlendReport), String> {
        match Self::blend_edge_direct(solid, edge_id, kind) {
            Ok(result) => Ok(result),
            Err(err) => match merged_twin(solid, edge_id) {
                Some((merged, twin)) => Self::blend_edge_direct(&merged, twin, kind).map_err(|_| err),
                None => Err(err),
            },
        }
    }

    fn blend_edge_direct(
        solid: &Solid,
        edge_id: u64,
        kind: BlendKind,
    ) -> Result<(Solid, EdgeBlendReport), String> {
        if let BlendKind::Fillet { radius } = kind {
            if let Some(result) =
                crate::circular_fillet::hole_mouth::try_fillet_hole_mouth(solid, edge_id, radius)?
            {
                return Ok(result);
            }
            if let Some(result) =
                crate::circular_fillet::revolved_mouth::try_fillet_revolved_mouth(solid, edge_id, radius)?
            {
                return Ok(result);
            }
            if let Some(result) =
                crate::circular_fillet::revolved_corner::try_fillet_revolved_corner(solid, edge_id, radius)?
            {
                return Ok(result);
            }
            if let Some(result) =
                crate::circular_fillet::slot_hole_mouth::try_fillet_slot_hole_mouth(
                    solid, edge_id, radius,
                )?
            {
                return Ok(result);
            }
            if let Some(result) = crate::circular_fillet::shoulder_root::try_fillet_shoulder_root(
                solid, edge_id, radius,
            )? {
                return Ok(result);
            }
            if let Some(result) =
                crate::circular_fillet::slot_root::try_fillet_slot_root(solid, edge_id, radius)?
            {
                return Ok(result);
            }
            if let Some(result) =
                crate::circular_fillet::slot_rim::try_fillet_slot_rim(solid, edge_id, radius)?
            {
                return Ok(result);
            }
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
            if let Some(result) = crate::circular_fillet::sphere_cap::try_fillet_sphere_cap_rim(
                solid, edge_id, radius,
            )? {
                return Ok(result);
            }
            if let Some(result) =
                crate::circular_fillet::prism_rim::try_fillet_prism_rim(solid, edge_id, radius)?
            {
                return Ok(result);
            }
        }
        if let BlendKind::Chamfer { distance } = kind {
            if let Some(result) = crate::circular_fillet::hole_mouth::try_chamfer_hole_mouth(
                solid, edge_id, distance,
            )? {
                return Ok(result);
            }
            if let Some(result) =
                crate::circular_fillet::revolved_mouth::try_chamfer_revolved_mouth(solid, edge_id, distance)?
            {
                return Ok(result);
            }
            if let Some(result) =
                crate::circular_fillet::revolved_corner::try_chamfer_revolved_corner(solid, edge_id, distance)?
            {
                return Ok(result);
            }
            if let Some(result) =
                crate::circular_fillet::slot_hole_mouth::try_chamfer_slot_hole_mouth(
                    solid, edge_id, distance,
                )?
            {
                return Ok(result);
            }
            if let Some(result) = crate::circular_fillet::shoulder_root::try_chamfer_shoulder_root(
                solid, edge_id, distance,
            )? {
                return Ok(result);
            }
            if let Some(result) =
                crate::circular_fillet::slot_root::try_chamfer_slot_root(solid, edge_id, distance)?
            {
                return Ok(result);
            }
            if let Some(result) =
                crate::circular_fillet::slot_rim::try_chamfer_slot_rim(solid, edge_id, distance)?
            {
                return Ok(result);
            }
            if let Some(result) =
                crate::circular_fillet::try_chamfer_cylinder_rim(solid, edge_id, distance)?
            {
                return Ok(result);
            }
            if let Some(result) =
                crate::circular_fillet::cone::try_chamfer_conical_rim(solid, edge_id, distance)?
            {
                return Ok(result);
            }
            if let Some(result) = crate::circular_fillet::sphere_cap::try_chamfer_sphere_cap_rim(
                solid, edge_id, distance,
            )? {
                return Ok(result);
            }
            if let Some(result) = crate::circular_fillet::prism_rim::try_chamfer_prism_rim(
                solid, edge_id, distance,
            )? {
                return Ok(result);
            }
        }
        if let BlendKind::Chamfer { distance } = kind {
            if let Some(result) = try_chamfer_arc_rim(solid, edge_id, distance)? {
                return Ok(result);
            }
        }
        if let BlendKind::Fillet { .. } = kind {
            if arc_rim_blendable(solid, edge_id).is_some() {
                return Err(format!(
                    "Edge {edge_id} is a cylinder rim arc ending on side faces; only a chamfer is built there (the fillet end is not a rational curve)"
                ));
            }
        }
        let site = BlendSite::locate(solid, edge_id)?;
        if site.cyl_end.is_some() && matches!(kind, BlendKind::Fillet { .. }) {
            return Err(format!(
                "Edge {edge_id} ends on a cylinder; only a chamfer is built there (the fillet end is not a rational curve)"
            ));
        }
        let section = match site.cyl_side {
            Some(_) => Some(site.cyl_section(kind)?),
            None => None,
        };
        let setback = section.map_or_else(|| kind.setback(site.dihedral), |s| s.s_a);

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

        // 両側の面の穴（内側ワイヤ）が、削る帯の幅に入るなら断る。ちょうど接する
        // （肉厚 = 帯の幅）も、面の縁と穴が1点で触れる形になるので断る（4-954）。
        let clearance = site.hole_clearance(solid);
        if setback >= clearance * (1.0 - 1e-9) {
            return Err(format!(
                "Blend needs {setback:.6} of setback along each face but a hole in a face beside edge {edge_id} is only {clearance:.6} away"
            ));
        }

        let report = EdgeBlendReport {
            dihedral_angle_deg: site.dihedral.to_degrees(),
            edge_length: site.length,
            setback,
            predicted_removed_volume: match section {
                Some(s) => s.area * site.length,
                None => {
                    kind.removed_section_area(site.dihedral) * site.centroid_length(kind)
                        + site.cyl_end.map_or(0.0, |c| c.lambda_moment(setback))
                }
            },
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
    /// 斜めの端の面の外向き法線（直交する端なら None。`ZENITH_OBLIQUE_CAP`）
    cap_start_normal: Option<Vec3>,
    cap_end_normal: Option<Vec3>,
    /// 端を詰める隣接稜: (edge_id, その稜のどちら側の端点を動かすか, 方向 t)
    neighbours: Vec<NeighbourTrim>,
    /// 隣接稜が許す後退距離の上限
    max_setback: f64,
    /// 片端が円柱の側面に当たる（4-962、面取りのみ、`ZENITH_CYL_CAP_CHAMFER=1`）
    cyl_end: Option<CylEnd>,
    /// 片側の面が稜を母線に持つ凸の円柱（4-964、`ZENITH_CYL_SIDE_BLEND=1`）
    cyl_side: Option<CylSide>,
}

/// 稜を母線に持つ凸の円柱の面（軸は稜に平行、立体は円柱の内側）。
#[derive(Debug, Clone, Copy)]
struct CylSide {
    /// 円柱が面 a か
    on_a: bool,
    centre: Point3,
    axis: Vec3,
    radius: f64,
    /// 両端の円弧の、頂点からもう一方の端までの弦の短いほう
    chord_limit: f64,
    /// 凹の円柱（立体が円の外側、4-968）
    concave: bool,
    /// 稜が凹（凸の円柱と平面の入隅、材料を足す、4-970）
    concave_edge: bool,
}

/// 円柱の側の断面。頂点からのずれ（稜に直交する平面の中）。
#[derive(Debug, Clone, Copy)]
struct CylSection {
    /// 平面の側の後退距離
    s_a: f64,
    /// 円柱の側の後退点
    p_b: Vec3,
    /// フィレットの円弧の中央の制御点と重み
    mid: Vec3,
    weight: f64,
    /// 削れる断面積
    area: f64,
}

fn cyl_side_blend_enabled() -> bool {
    std::env::var("ZENITH_CYL_SIDE_BLEND").map_or(true, |v| v != "0")
}

/// 平面と凹の円柱の母線の凸の稜（4-968、既定オン、`ZENITH_CYL_CONCAVE_SIDE=0` で切る）。
/// 平面と凸の円柱の母線の凹の稜（入隅、4-970、既定オン、`ZENITH_CYL_SIDE_CONCAVE_EDGE=0` で切る）。
fn cyl_side_concave_edge_enabled() -> bool {
    std::env::var("ZENITH_CYL_SIDE_CONCAVE_EDGE").map_or(true, |v| v != "0")
}

fn cyl_concave_side_enabled() -> bool {
    std::env::var("ZENITH_CYL_CONCAVE_SIDE").map_or(true, |v| v != "0")
}

/// 面が、稜（v_start→v_end）を母線に持つ凸の円柱なら (軸の点, 軸, 半径, 頂点での外向き法線)。
fn cylinder_side_reading(
    surface: &NurbsSurface3,
    forward: bool,
    v_start: Point3,
    v_end: Point3,
    d: Vec3,
) -> Option<(Point3, Vec3, f64, Vec3, bool)> {
    let tol = Tolerance::default();
    let (base, radius, axis, _) = crate::brep_intersection::cylinder_patch_reading(surface, &tol)?;
    let axis = axis.try_normalize_safe(1e-12)?;
    if axis.dot(&d).abs() < 1.0 - 1e-12 {
        return None;
    }
    let radial = |p: Point3| {
        let offset = p - base;
        offset - axis * offset.dot(&axis)
    };
    let scale = radius.max(1.0);
    if (radial(v_start).norm() - radius).abs() > 1e-7 * scale
        || (radial(v_end).norm() - radius).abs() > 1e-7 * scale
    {
        return None;
    }
    // 面の向き: 面の中ほどの法線が、軸から外を向くか（外向きなら立体は内側＝凸）
    let ((u0, u1), (w0, w1)) = surface.param_range();
    let (um, wm) = ((u0 + u1) * 0.5, (w0 + w1) * 0.5);
    let normal = surface.normal(um, wm)?;
    let sign = normal.dot(&radial(surface.evaluate(um, wm))) * if forward { 1.0 } else { -1.0 };
    let concave = !(sign > 0.0);
    if concave && !(sign < 0.0 && cyl_concave_side_enabled()) {
        return None;
    }
    let base_on_v = base + axis * (v_start - base).dot(&axis);
    let outward = radial(v_start) / radius;
    Some((base_on_v, axis, radius, if concave { -outward } else { outward }, concave))
}

/// 断面を解く。平面の側（内向き t_p、外向き法線 n_p）と円柱（中心 cs.centre、半径 R、
/// 頂点は円の上）。面取りは頂点から弦 s の点、フィレットは直線と円に内接する半径 r の円。
fn cyl_section_2d(
    vertex: Point3,
    d: Vec3,
    t_p: Vec3,
    n_p: Vec3,
    t_q: Vec3,
    cs: &CylSide,
    kind: BlendKind,
) -> Result<CylSection, String> {
    let big = cs.radius;
    let raw = cs.centre - vertex;
    let c = raw - d * raw.dot(&d);
    let c_hat = c / big;
    let mut c_perp = d.cross(&c_hat);
    if c_perp.dot(&t_q) < 0.0 {
        c_perp = -c_perp;
    }
    // 断面の2次元の座標（原点は頂点）
    let e1 = t_p;
    let e2 = d.cross(&t_p);
    let flat = |v: Vec3| (v.dot(&e1), v.dot(&e2));
    let cross = |p: (f64, f64), q: (f64, f64)| p.0 * q.1 - p.1 * q.0;
    // 弧 p→q（中心 k、短い向き）の ½∮(x dy − y dx)
    let arc = |k: Vec3, p: Vec3, q: Vec3, radius: f64| {
        let (kp, kq) = (flat(p - k), flat(q - k));
        let delta = cross(kp, kq).atan2(kp.0 * kq.0 + kp.1 * kq.1);
        0.5 * (cross(flat(k), flat(q - p)) + radius * radius * delta)
    };
    let line = |p: Vec3, q: Vec3| 0.5 * cross(flat(p), flat(q));
    match kind {
        BlendKind::Chamfer { distance: s } => {
            if s >= 2.0 * big {
                return Err("The chamfer is wider than the cylinder beside the edge".into());
            }
            let alpha = s * s / (2.0 * big);
            let beta = (s * s - alpha * alpha).max(0.0).sqrt();
            let p_b = c_hat * alpha + c_perp * beta;
            let p_a = t_p * s;
            let area = line(Vec3::zeros(), p_a) + line(p_a, p_b) + arc(c, p_b, Vec3::zeros(), big);
            let area = if cs.concave_edge { -area.abs() } else { area.abs() };
            Ok(CylSection { s_a: s, p_b, mid: Vec3::zeros(), weight: 1.0, area })
        }
        BlendKind::Fillet { radius: r } => {
            if r >= big && !cs.concave && !cs.concave_edge {
                return Err("The fillet is larger than the cylinder beside the edge".into());
            }
            // 入隅（4-970）では円は空いた側（平面の外向き法線の側）に置く
            let m = if cs.concave_edge { n_p } else { -n_p };
            let w = m * r - c;
            let b = t_p.dot(&w);
            // 凸の円柱には内接（中心の距離 R−r）、凹の円柱・入隅には外接（R+r、4-968、4-970）
            let outside = cs.concave || cs.concave_edge;
            let reach = if outside { big + r } else { big - r };
            let disc = b * b - w.dot(&w) + reach * reach;
            if disc < 0.0 {
                return Err("No fillet circle touches both the plane and the cylinder".into());
            }
            let x = if outside {
                let (x1, x2) = (-b - disc.sqrt(), -b + disc.sqrt());
                if x1 > 0.0 { x1 } else { x2 }
            } else {
                -b - disc.sqrt()
            };
            if !(x > 0.0) {
                return Err("The fillet circle does not sit inside the corner".into());
            }
            let o = t_p * x + m * r;
            let p_a = t_p * x;
            let p_b = c + (o - c).normalize() * big;
            let cos_phi = ((p_a - o).dot(&(p_b - o)) / (r * r)).clamp(-1.0, 1.0);
            let half = 0.5 * cos_phi.acos();
            let mid = p_a - t_p * (r * half.tan());
            let area = line(Vec3::zeros(), p_a) + arc(o, p_a, p_b, r) + arc(c, p_b, Vec3::zeros(), big);
            let area = if cs.concave_edge { -area.abs() } else { area.abs() };
            Ok(CylSection { s_a: x, p_b, mid, weight: half.cos(), area })
        }
    }
}

/// 円柱の側の、端の面の上で頂点に繋がる円弧の稜（4-964）。弦の上限も返す。
fn find_ruling_arc_neighbour(
    face: &Face,
    edge_id: u64,
    vertex: Point3,
    cs: &CylSide,
    at_start: bool,
    on_face_a: bool,
) -> Result<(NeighbourTrim, f64), String> {
    let scale = cs.radius.max(1.0);
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
            let fail = |why: &str| Err(format!("Neighbouring edge {} on the cylinder: {why}", edge.id));
            let Some((centre, radius, _)) = crate::circular_fillet::circle_from_edge(edge) else {
                return fail("it is not a circular arc");
            };
            let offset = centre - cs.centre;
            if (radius - cs.radius).abs() > 1e-7 * scale
                || (offset - cs.axis * offset.dot(&cs.axis)).norm() > 1e-7 * scale
            {
                return fail("it is not on the cylinder");
            }
            let other = if move_start { edge.end_vertex.point } else { edge.start_vertex.point };
            let chord = (other - vertex).norm();
            if chord <= 1e-9 {
                return fail("it is a full circle");
            }
            return Ok((
                NeighbourTrim {
                    edge_id: edge.id,
                    move_start,
                    available: f64::INFINITY,
                    at_start_of_target: at_start,
                    on_face_a,
                    arc: Some((centre, radius, cs.axis)),
                },
                chord,
            ));
        }
    }
    Err(format!("Edge {edge_id} has no arc beside it on the cylinder"))
}

/// 稜の片端が当たる円柱。軸は稜に直交し、稜の片側の面（側面）は軸を含み、
/// もう片側の面（円の面）は軸に直交する（4-962）。二面角は 90 度になる。
#[derive(Debug, Clone, Copy)]
struct CylEnd {
    at_start: bool,
    /// 軸の上の点（円の面の高さに合わせてある）
    centre: Point3,
    axis: Vec3,
    radius: f64,
    /// 軸に直交する側の面が a か
    circle_on_a: bool,
    /// 円の面の内向き t_q と、側面の内向き t_p
    t_q: Vec3,
    t_p: Vec3,
    /// 端から外へ（終点なら d、始点なら −d）
    out: Vec3,
    /// 頂点から見た軸の位置: out 方向 cx、t_q 方向 cy
    cx: f64,
    cy: f64,
    vertex: Point3,
}

impl CylEnd {
    /// 円の面で t_q に u 退いた点が、円柱に当たるまで out へ滑る量（u=0 で 0）
    fn lambda(&self, u: f64) -> Option<f64> {
        let rest = self.radius * self.radius - (u - self.cy) * (u - self.cy);
        if rest < 0.0 {
            return None;
        }
        Some(self.cx - self.cx.signum() * rest.sqrt())
    }

    /// ∫₀ˢ (s−y) λ(y) dy（閉じた式）
    fn lambda_moment(&self, s: f64) -> f64 {
        let r = self.radius;
        let f = |v: f64| {
            let root = (r * r - v * v).max(0.0).sqrt();
            0.5 * (v * root + r * r * (v / r).clamp(-1.0, 1.0).asin())
        };
        let g = |v: f64| (r * r - v * v).max(0.0).powf(1.5) / 3.0;
        let at = |v: f64| (s - self.cy) * f(v) + g(v);
        let j = at(s - self.cy) - at(-self.cy);
        self.cx * s * s * 0.5 - self.cx.signum() * j
    }

    /// [0, reach] での |λ| の上限（極値は u = cy）
    fn lambda_bound(&self, reach: f64) -> f64 {
        let mut worst = self.lambda(reach).map_or(f64::INFINITY, f64::abs);
        if self.cy > 0.0 && self.cy < reach {
            worst = worst.max(self.lambda(self.cy).map_or(f64::INFINITY, f64::abs));
        }
        worst
    }
}

/// 端が円柱に当たる直線の稜の面取り（4-962）。既定オフ。
fn cyl_cap_chamfer_enabled() -> bool {
    std::env::var("ZENITH_CYL_CAP_CHAMFER").map_or(true, |v| v != "0")
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
    /// 円弧の隣の稜（中心、半径、軸）。詰めたあとも同じ円の弧にする（4-962）
    arc: Option<(Point3, f64, Vec3)>,
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
        // 片側が、稜を母線に持つ凸の円柱（4-964、`ZENITH_CYL_SIDE_BLEND=1`）
        let mut cyl_face: Option<(usize, Point3, Vec3, f64, bool)> = None;
        for (face_index, orientation) in &uses {
            let face = &faces[*face_index];
            let not_planar = || {
                format!(
                    "Face {face_index} beside edge {edge_id} is not planar; only planar pairs are blended"
                )
            };
            let normal = match &face.geometry {
                FaceGeometry::Plane(plane) => {
                    if face.orientation.is_forward() {
                        plane.normal
                    } else {
                        -plane.normal
                    }
                }
                FaceGeometry::Nurbs(surface) if cyl_side_blend_enabled() && cyl_face.is_none() => {
                    let (centre, axis, radius, normal, concave) =
                        cylinder_side_reading(surface, face.orientation.is_forward(), v_start, v_end, d)
                            .ok_or_else(not_planar)?;
                    cyl_face = Some((*face_index, centre, axis, radius, concave));
                    normal
                }
                _ => return Err(not_planar()),
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
        // 凹の稜（4-938、`ZENITH_CONCAVE_LINE_BLEND`）: 断面の円弧の中間制御点は
        // やはり元の稜の端点で、重み |sin(θ/2)|、後退距離 r |cot(θ/2)|。
        // 削れる体積の式は符号付きのまま材料の追加（負）を出す。
        let concave_ok = concave_line_blend_enabled()
            && dihedral > std::f64::consts::PI + 1e-6
            && dihedral < std::f64::consts::TAU - 1e-6;
        if dihedral >= std::f64::consts::PI - 1e-9 && !concave_ok {
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
        let concave_end = |reason: String| {
            if dihedral > std::f64::consts::PI {
                format!(
                    "Edge {edge_id} has a {:.3} deg interior angle; this concave edge cannot be blended where it ends: {reason}",
                    dihedral.to_degrees()
                )
            } else {
                reason
            }
        };
        // 端が円柱に当たるなら（4-962、凸の 90 度の稜、片端だけ）、その端を円柱で閉じる。
        let convex = dihedral < std::f64::consts::PI;
        let mut cyl_end: Option<CylEnd> = None;
        let cyl_side = cyl_face.map(|(index, centre, axis, radius, concave)| CylSide {
            on_a: index == fa,
            centre,
            axis,
            radius,
            chord_limit: f64::INFINITY,
            concave,
            concave_edge: !convex,
        });
        let concave_edge_ok = cyl_side_concave_edge_enabled() && cyl_side.is_some_and(|c| !c.concave);
        if cyl_side.is_some() && !convex && !concave_edge_ok {
            return Err(format!(
                "Edge {edge_id} between a plane and a cylinder is concave; that is not blended yet"
            ));
        }
        let mut cap_or_cylinder = |vertex: Point3, at_start: bool, which: &str| {
            match find_cap_face(faces, vertex, fa, fb, d, edge_id, which) {
                Ok(found) => Ok(found),
                Err(reason) => {
                    if convex && cyl_cap_chamfer_enabled() && cyl_end.is_none() && cyl_side.is_none() {
                        if let Some((index, cylinder)) =
                            find_cylinder_cap(faces, vertex, fa, fb, d, ta, tb, at_start)
                        {
                            cyl_end = Some(cylinder);
                            return Ok((index, None));
                        }
                    }
                    Err(concave_end(reason))
                }
            }
        };
        let (cap_start, cap_start_normal) = cap_or_cylinder(v_start, true, "start")?;
        let (cap_end, cap_end_normal) = cap_or_cylinder(v_end, false, "end")?;

        if dihedral > std::f64::consts::PI && (cap_start_normal.is_some() || cap_end_normal.is_some()) {
            return Err(format!(
                "Edge {edge_id} is concave and ends on an oblique face; that combination is not blended yet"
            ));
        }

        if cyl_side.is_some() && (cap_start_normal.is_some() || cap_end_normal.is_some()) {
            return Err(format!(
                "Edge {edge_id} between a plane and a cylinder ends on an oblique face; that is not blended yet"
            ));
        }
        let mut cyl_side = cyl_side;

        // 5. 端で詰める隣接稜
        let mut neighbours = Vec::new();
        for (at_start, vertex, cap_normal) in
            [(true, v_start, cap_start_normal), (false, v_end, cap_end_normal)]
        {
            for (on_face_a, face_index, dir, normal) in [(true, fa, ta, na), (false, fb, tb, nb)] {
                // 円柱の側: 隣の稜は端の面の上の円弧（4-964）。許す弦の長さは別に持つ。
                if let Some(cs) = cyl_side.as_mut().filter(|c| c.on_a == on_face_a) {
                    let (trim, chord) =
                        find_ruling_arc_neighbour(&faces[face_index], edge_id, vertex, cs, at_start, on_face_a)?;
                    cs.chord_limit = cs.chord_limit.min(chord);
                    neighbours.push(trim);
                    continue;
                }
                // 円柱の端の、円の面の側: 隣の稜は円柱の上の円弧（4-962）。
                if let Some(cylinder) = cyl_end.filter(|c| c.at_start == at_start && c.circle_on_a == on_face_a) {
                    neighbours.push(find_arc_neighbour(&faces[face_index], edge_id, &cylinder, on_face_a)?);
                    continue;
                }
                // 斜めの端では、面の内側へ退いた点は稜に沿って端の面まで滑る。
                // 後退距離 1 あたりの移動 k = t + d·λ。
                let step = retreat_step(dir, d, cap_normal);
                let scale = step.norm();
                let mut trim = find_neighbour(
                    &faces[face_index],
                    edge_id,
                    vertex,
                    step / scale,
                    normal,
                    at_start,
                    on_face_a,
                    face_index,
                )
                .map_err(|reason| {
                    if dihedral > std::f64::consts::PI {
                        // 凹の稜の端が凹の角（隣の壁を延ばす必要がある）など
                        format!(
                            "Edge {edge_id} has a {:.3} deg interior angle; this concave edge cannot be blended where it ends: {reason}",
                            dihedral.to_degrees()
                        )
                    } else {
                        reason
                    }
                })?;
                trim.available /= scale;
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
            cap_start_normal,
            cap_end_normal,
            neighbours,
            max_setback,
            cyl_end,
            cyl_side,
        })
    }

    /// 円柱の側の断面（4-964）。断面は稜に直交する平面で、角柱なので両端で同じ。
    fn cyl_section(&self, kind: BlendKind) -> Result<CylSection, String> {
        let cs = self.cyl_side.ok_or("not a plane-cylinder edge")?;
        let (t_p, n_p) = if cs.on_a { (self.t_b, self.n_b) } else { (self.t_a, self.n_a) };
        let t_q = if cs.on_a { self.t_a } else { self.t_b };
        let section = cyl_section_2d(self.v_start, self.d, t_p, n_p, t_q, &cs, kind)?;
        if section.p_b.norm() >= cs.chord_limit * (1.0 - 1e-9) {
            return Err(format!(
                "The blend reaches {:.6} along the cylinder but the arc beside the edge only allows {:.6}",
                section.p_b.norm(),
                cs.chord_limit
            ));
        }
        Ok(section)
    }

    /// 円柱の側のフィレットの半径の上限（二分法、4-964）。平面の側の後退と円柱の側の弦が
    /// どちらも許す範囲に収まる最大。
    fn cyl_side_max_fillet(&self, max_setback: f64) -> f64 {
        let Some(cs) = self.cyl_side else { return 0.0 };
        let fits = |r: f64| {
            self.cyl_section(BlendKind::Fillet { radius: r })
                .is_ok_and(|s| s.s_a < max_setback && s.p_b.norm() < cs.chord_limit * 0.999)
        };
        let (mut lo, mut hi) = (0.0, if cs.concave || cs.concave_edge { 10.0 * cs.radius } else { cs.radius });
        for _ in 0..100 {
            let mid = 0.5 * (lo + hi);
            if fits(mid) {
                lo = mid;
            } else {
                hi = mid;
            }
        }
        lo * 0.999
    }

    /// 削る帯が、稜の乗っていないワイヤ（穴、稜がポケットの口なら外側ワイヤ、端面の穴）に
    /// 触れずにいられる後退距離の上限（4-954、4-956）。無ければ無限大。
    ///
    /// - 両側の面: 帯は稜の真横（稜に沿った座標が稜の範囲、斜めの端の滑りぶん広げる）
    ///   だけを通るので、その範囲に入る部分の、稜から面の内側への距離の最小。
    /// - 端面（凸の稜のみ）: 後退距離 s の面取りが端面から削るのは、頂点と2つの後退点の
    ///   三角形 {u,v ≥ 0, u+v ≤ s}（u, v は2つの後退の向きの斜交座標）。フィレットが
    ///   削る所はその三角形の中に収まるので、u+v の最小で両方を守る。
    ///
    /// どれも1次式の最小を、直線と円弧の上で厳密に取る。ほかの曲線は 0（断る）。
    fn hole_clearance(&self, solid: &Solid) -> f64 {
        if !hole_clearance_enabled() {
            return f64::INFINITY;
        }
        let faces = &solid.outer_shell.faces;
        // 円柱の側（4-964）: 穴の見立ては平面の帯のものなので、穴があれば断る。
        if self.cyl_side.is_some() {
            let holed = [self.face_a, self.face_b, self.cap_start, self.cap_end]
                .iter()
                .any(|&i| !faces[i].inner_wires.is_empty());
            return if holed { 0.0 } else { f64::INFINITY };
        }
        let reach = if self.max_setback.is_finite() { self.max_setback } else { self.length };
        // 斜めの端では帯の角がその端の先へ滑る。滑りは端ごとに違うので、端ごとに広げる。
        let slide = |cap: Option<Vec3>| {
            [self.t_a, self.t_b]
                .iter()
                .map(|&dir| retreat_step(dir, self.d, cap).dot(&self.d).abs())
                .fold(0.0f64, f64::max)
        };
        let eps = 1e-9 * self.length.max(1.0);
        let (mut margin_start, mut margin_end) =
            (slide(self.cap_start_normal) * reach + eps, slide(self.cap_end_normal) * reach + eps);
        // 円柱の端では、円の面の側の帯の角が λ だけ外へ出入りする（4-962）。
        // 円柱の面に穴があれば、三角形の見立てが曲面では厳密でないので断る。
        if let Some(cylinder) = &self.cyl_end {
            let bound = cylinder.lambda_bound(reach) + eps;
            if cylinder.at_start {
                margin_start = margin_start.max(bound);
            } else {
                margin_end = margin_end.max(bound);
            }
            let cap = if cylinder.at_start { self.cap_start } else { self.cap_end };
            if !faces[cap].inner_wires.is_empty() {
                return 0.0;
            }
        }
        let mut nearest = f64::INFINITY;
        for (face_index, inward) in [(self.face_a, self.t_a), (self.face_b, self.t_b)] {
            let face = &faces[face_index];
            for wire in std::iter::once(&face.outer_wire).chain(face.inner_wires.iter()) {
                // 稜そのものが乗っているワイヤの隣の稜は、端の詰め（neighbours）が扱う。
                if wire.edges.iter().any(|oriented| oriented.edge.id == self.edge_id) {
                    continue;
                }
                for oriented in &wire.edges {
                    let bounds = [
                        (self.d, -margin_start, self.length + margin_end),
                        (inward, -eps, f64::INFINITY),
                    ];
                    match min_linear_on_edge(&oriented.edge, self.v_start, inward, &bounds) {
                        Ok(Some(found)) => nearest = nearest.min(found.max(0.0)),
                        Ok(None) => {}
                        Err(()) => return 0.0,
                    }
                }
            }
        }
        if self.dihedral < std::f64::consts::PI {
            for (cap, vertex, normal) in [
                (self.cap_start, self.v_start, self.cap_start_normal),
                (self.cap_end, self.v_end, self.cap_end_normal),
            ] {
                let ka = retreat_step(self.t_a, self.d, normal);
                let kb = retreat_step(self.t_b, self.d, normal);
                // 斜交座標の双対: du·ka = 1, du·kb = 0, dv·ka = 0, dv·kb = 1
                let (aa, ab, bb) = (ka.dot(&ka), ka.dot(&kb), kb.dot(&kb));
                let det = aa * bb - ab * ab;
                if det.abs() < 1e-15 {
                    continue;
                }
                let du = (ka * bb - kb * ab) / det;
                let dv = (kb * aa - ka * ab) / det;
                let face = &faces[cap];
                let near = |point: Point3| (point - vertex).norm() <= eps;
                for wire in std::iter::once(&face.outer_wire).chain(face.inner_wires.iter()) {
                    for oriented in &wire.edges {
                        let edge = &oriented.edge;
                        // 頂点に繋がる稜は三角形の辺そのもの（端の詰めが扱う）。
                        if near(edge.start_vertex.point) || near(edge.end_vertex.point) {
                            continue;
                        }
                        let bounds = [(du, -eps, f64::INFINITY), (dv, -eps, f64::INFINITY)];
                        match min_linear_on_edge(edge, vertex, du + dv, &bounds) {
                            Ok(Some(found)) => nearest = nearest.min(found.max(0.0)),
                            Ok(None) => {}
                            Err(()) => return 0.0,
                        }
                    }
                }
            }
        }
        nearest
    }

    /// 削れる断面の重心を通る、稜に平行な線の、両端の面に挟まれた長さ。
    /// 角柱を2枚の平面で切った体積は「断面積 × 重心での軸方向の長さ」。
    /// 端が両方とも直交なら稜の長さそのもの。
    fn centroid_length(&self, kind: BlendKind) -> f64 {
        if self.cap_start_normal.is_none() && self.cap_end_normal.is_none() {
            return self.length;
        }
        let theta = self.dihedral;
        let half = theta * 0.5;
        let setback = kind.setback(theta);
        // 二等分線（面の内側）に沿った重心の座標
        let along = match kind {
            BlendKind::Chamfer { .. } => 2.0 * setback * half.cos() / 3.0,
            BlendKind::Fillet { radius } => {
                let r = radius;
                let centre = r / half.sin();
                let alpha = (std::f64::consts::PI - theta) * 0.5;
                let kite = setback * r;
                let sector = r * r * alpha;
                let kite_moment = kite * (setback * half.cos() + centre) / 3.0;
                let sector_moment = sector * (centre - 2.0 * r * alpha.sin() / (3.0 * alpha));
                (kite_moment - sector_moment) / (kite - sector)
            }
        };
        let bisector = (self.t_a + self.t_b).normalize();
        let g = bisector * along;
        let shift = |normal: Option<Vec3>| normal.map_or(0.0, |n| -n.dot(&g) / n.dot(&self.d));
        self.length + shift(self.cap_end_normal) - shift(self.cap_start_normal)
    }

    fn apply(&self, solid: &Solid, kind: BlendKind, setback: f64) -> Result<Solid, String> {
        let tol = Tolerance::default();
        let faces = &solid.outer_shell.faces;

        // 新しい4頂点。稜の両端が、各面側へ setback だけ退いた点に割れる。
        let (cs, ce) = (self.cap_start_normal, self.cap_end_normal);
        let section = match self.cyl_side {
            Some(_) => Some(self.cyl_section(kind)?),
            None => None,
        };
        let point = |at_start: bool, on_a: bool| -> Result<Point3, String> {
            if let (Some(side), Some(sec)) = (self.cyl_side, section) {
                let vertex = if at_start { self.v_start } else { self.v_end };
                if side.on_a == on_a {
                    return Ok(vertex + sec.p_b);
                }
                let dir = if on_a { self.t_a } else { self.t_b };
                return Ok(vertex + dir * sec.s_a);
            }
            let (vertex, cap, dir) = (
                if at_start { self.v_start } else { self.v_end },
                if at_start { cs } else { ce },
                if on_a { self.t_a } else { self.t_b },
            );
            // 円柱の端の円の面の側は、退いた点を円柱まで滑らせる（4-962）
            if let Some(c) = self.cyl_end.filter(|c| c.at_start == at_start && c.circle_on_a == on_a) {
                let lambda = c
                    .lambda(setback)
                    .ok_or("The chamfer is wider than the cylinder it ends on")?;
                return Ok(vertex + c.t_q * setback + c.out * lambda);
            }
            Ok(vertex + retreat_step(dir, self.d, cap) * setback)
        };
        let a_start = Vertex::from_point(point(true, true)?);
        let a_end = Vertex::from_point(point(false, true)?);
        let b_start = Vertex::from_point(point(true, false)?);
        let b_end = Vertex::from_point(point(false, false)?);

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
        let mut corner_end =
            self.corner_edge(kind, setback, self.v_end, a_end.clone(), b_end.clone())?;
        let mut corner_start = corner_start;
        // 円柱の端（4-962）: 面取りの平面と円柱の交わりの楕円弧を、円柱の面の円の引数で。
        // p-curve も同じ引数で厳密に作り、面を作り直すときに当てはめと置き換える。
        let mut cylinder_pcurve: Option<zenith_geom::NurbsCurve2> = None;
        if let Some(c) = &self.cyl_end {
            let cap = if c.at_start { self.cap_start } else { self.cap_end };
            let FaceGeometry::Nurbs(surface) = &faces[cap].geometry else {
                return Err("The cylinder end face is not a NURBS cylinder".into());
            };
            let (on_a, on_b) =
                if c.at_start { (a_start.clone(), b_start.clone()) } else { (a_end.clone(), b_end.clone()) };
            let from = if c.circle_on_a { on_a.point } else { on_b.point };
            let (curve3, curve2) = cylinder_end_curves(surface, c, setback, from)
                .ok_or("The chamfer end could not be laid on the cylinder's own parameter")?;
            // 曲線は円の面の側から側面の側へ。辺は on_a → on_b。
            let (curve3, curve2) =
                if c.circle_on_a { (curve3, curve2) } else { (curve3.reversed(), reverse_bezier2(&curve2)) };
            let edge = Edge::new(curve3, on_a, on_b, 1e-6);
            if c.at_start {
                corner_start = edge;
            } else {
                corner_end = edge;
            }
            cylinder_pcurve = Some(curve2);
        }

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
            let start = new_start
                .clone()
                .unwrap_or_else(|| original.start_vertex.clone());
            let end = new_end
                .clone()
                .unwrap_or_else(|| original.end_vertex.clone());
            if (end.point - start.point).norm() <= 1e-12 {
                return Err(format!(
                    "Trimming neighbouring edge {id} would collapse it; the blend is too large here"
                ));
            }
            // 円弧の隣の稜は、同じ円の弧のまま詰める（4-962）
            let arc = self
                .neighbours
                .iter()
                .find(|n| n.edge_id == *id)
                .and_then(|n| n.arc);
            let curve = match arc {
                Some((centre, radius, axis)) => {
                    arc_on_circle(&original, centre, radius, axis, start.point, end.point)?
                }
                None => NurbsCurve3::bspline_from_points(1, vec![start.point, end.point])?,
            };
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
                    let replacement = if index == self.face_a {
                        &line_a
                    } else {
                        &line_b
                    };
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

            let mut rebuilt = Face::new(
                face.geometry.clone(),
                outer,
                inners,
                face.orientation,
                face.tolerance,
            );
            // 円柱の端の面: 楕円弧の p-curve を当てはめから厳密なものへ（4-962）
            if let (Some(c), Some(exact)) = (&self.cyl_end, &cylinder_pcurve) {
                let cap = if c.at_start { self.cap_start } else { self.cap_end };
                let corner_id = if c.at_start { corner_start.id } else { corner_end.id };
                if index == cap {
                    if let Some(pcurves) = rebuilt.pcurves.as_mut() {
                        for segment in pcurves.outer_loop.segments.iter_mut() {
                            if segment.edge_id == corner_id {
                                // 当てはめと同じ向き（端点が合う向き）で置き換える
                                let (f0, f1) = segment.curve.param_range();
                                let (e0, e1) = exact.param_range();
                                let same = (segment.curve.evaluate(f0) - exact.evaluate(e0)).norm()
                                    + (segment.curve.evaluate(f1) - exact.evaluate(e1)).norm();
                                let flipped = (segment.curve.evaluate(f0) - exact.evaluate(e1)).norm()
                                    + (segment.curve.evaluate(f1) - exact.evaluate(e0)).norm();
                                segment.curve =
                                    if same <= flipped { exact.clone() } else { reverse_bezier2(exact) };
                            }
                        }
                    }
                }
            }
            new_faces.push(rebuilt);
        }

        // ブレンド面そのもの
        let blend_wire = Wire::new(vec![
            OrientedEdge::forward(line_a.clone()),
            OrientedEdge::forward(corner_end.clone()),
            OrientedEdge::reversed(line_b.clone()),
            OrientedEdge::reversed(corner_start.clone()),
        ]);
        // 凹の稜では、同じ並びの輪が外向き法線に対して逆回りになる（4-938）
        let blend_wire = if self.dihedral > std::f64::consts::PI {
            Wire::new(
                blend_wire
                    .edges
                    .iter()
                    .rev()
                    .map(|oriented| OrientedEdge::new(oriented.edge.clone(), oriented.orientation.reversed()))
                    .collect(),
            )
        } else {
            blend_wire
        };
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
                let (apex, weight) = match self.cyl_side.map(|_| self.cyl_section(kind)).transpose()? {
                    Some(sec) => (apex + sec.mid, sec.weight),
                    None => (apex, (self.dihedral * 0.5).sin()),
                };
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
                let (mid, weight) = match self.cyl_side.map(|_| self.cyl_section(kind)).transpose()? {
                    Some(sec) => (sec.mid, sec.weight),
                    None => (Vec3::zeros(), (self.dihedral * 0.5).sin()),
                };
                let rows_b_first = vec![
                    vec![
                        ControlPoint3::unweighted(b_start.point),
                        ControlPoint3::unweighted(b_end.point),
                    ],
                    vec![
                        ControlPoint3::new(self.v_start + mid, weight),
                        ControlPoint3::new(self.v_end + mid, weight),
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
) -> Result<(usize, Option<Vec3>), String> {
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

    if found.is_empty() {
        return Err(format!(
            "The {which} of edge {edge_id} has no other face around it; the blend has nothing to end against"
        ));
    }

    // **端点に集まる面を数えるのではなく、蓋になれる面を数えます**（4-201）。
    //
    // 蓋の条件は「平面で、稜に垂直」です。以前は端点まわりの面を**全部**
    // 数えて「1枚だけ」を求めていました。**それは条件ではなく、たまたま
    // 素の箱でそうなる数**です。
    //
    // 実測（4-201）: 面取り済みの箱は角に面が増えるので、**48稜すべて**が
    // 「他の面が2枚ある」で断られていました。幾何のほうは対応済みの
    // 「直線稜×平面2面」です——**数え方だけが止めていました**。
    let caps: Vec<usize> = found
        .iter()
        .copied()
        .filter(|index| match &faces[*index].geometry {
            FaceGeometry::Plane(plane) => plane.normal.dot(&d).abs() >= 1.0 - 1e-9,
            _ => false,
        })
        .collect();
    match caps.len() {
        1 => Ok((caps[0], None)),
        0 if oblique_cap_enabled() => {
            // 斜めの平面の端（4-936）。ほかの面と取り違えないよう、1枚だけのときに限る。
            let oblique: Vec<(usize, Vec3)> = found
                .iter()
                .filter_map(|index| match &faces[*index].geometry {
                    FaceGeometry::Plane(plane) if plane.normal.dot(&d).abs() >= 0.1 => {
                        let normal = if faces[*index].orientation.is_forward() {
                            plane.normal
                        } else {
                            -plane.normal
                        };
                        Some((*index, normal.normalize()))
                    }
                    _ => None,
                })
                .collect();
            match oblique.as_slice() {
                [(index, normal)] => Ok((*index, Some(*normal))),
                _ => Err(format!(
                    "The {which} of edge {edge_id} has no planar face perpendicular to it; the blend would need a trimmed end"
                )),
            }
        }
        0 => Err(format!(
            "The {which} of edge {edge_id} has no planar face perpendicular to it; the blend would need a trimmed end"
        )),
        // **2枚以上あるなら、どちらへ突き当たるか決められません。**
        // 推測せずに断ります。
        count => Err(format!(
            "The {which} of edge {edge_id} has {count} planar faces perpendicular to it; which one closes the blend is not decided"
        )),
    }
}

fn concave_line_blend_enabled() -> bool {
    std::env::var("ZENITH_CONCAVE_LINE_BLEND")
        .map(|value| value != "0")
        .unwrap_or(true)
}

fn oblique_cap_enabled() -> bool {
    std::env::var("ZENITH_OBLIQUE_CAP")
        .map(|value| value != "0")
        .unwrap_or(true)
}

/// 面の内側へ後退距離 1 だけ退いたとき、端の点が動く量。直交する端では `t`
/// そのもの。斜めの端では、退いた点を稜の向き `d` に沿って端の面まで滑らせる。
fn retreat_step(t: Vec3, d: Vec3, cap_normal: Option<Vec3>) -> Vec3 {
    match cap_normal {
        None => t,
        Some(n) => t - d * (n.dot(&t) / n.dot(&d)),
    }
}

/// 稜の端の頂点に、稜の2面のほかに円柱の面が1枚だけあり、軸が稜に直交し、
/// 片方の面が軸を含み、もう片方が軸に直交するなら、その円柱（4-962）。
#[allow(clippy::too_many_arguments)]
fn find_cylinder_cap(
    faces: &[Face],
    vertex: Point3,
    face_a: usize,
    face_b: usize,
    d: Vec3,
    t_a: Vec3,
    t_b: Vec3,
    at_start: bool,
) -> Option<(usize, CylEnd)> {
    let tol = Tolerance::default();
    let mut found = None;
    for (index, face) in faces.iter().enumerate() {
        if index == face_a || index == face_b {
            continue;
        }
        let touches = std::iter::once(&face.outer_wire)
            .chain(face.inner_wires.iter())
            .any(|wire| wire.edges.iter().any(|o| (o.start_vertex().point - vertex).norm() <= 1e-9));
        if !touches {
            continue;
        }
        if found.is_some() {
            return None;
        }
        found = Some(index);
    }
    let index = found?;
    let FaceGeometry::Nurbs(surface) = &faces[index].geometry else {
        return None;
    };
    let (base, radius, axis, _) = crate::brep_intersection::cylinder_patch_reading(surface, &tol)?;
    let axis = axis.try_normalize_safe(1e-12)?;
    if axis.dot(&d).abs() > 1e-9 {
        return None;
    }
    let (circle_on_a, t_q, t_p) =
        if t_a.dot(&axis).abs() <= 1e-9 && t_b.dot(&axis).abs() >= 1.0 - 1e-9 {
            (true, t_a, t_b)
        } else if t_b.dot(&axis).abs() <= 1e-9 && t_a.dot(&axis).abs() >= 1.0 - 1e-9 {
            (false, t_b, t_a)
        } else {
            return None;
        };
    let out = if at_start { -d } else { d };
    let offset = base - vertex;
    let radial = offset - axis * offset.dot(&axis);
    if (radial.norm() - radius).abs() > 1e-7 * radius.max(1.0) {
        return None;
    }
    let (cx, cy) = (offset.dot(&out), offset.dot(&t_q));
    if std::env::var("ZENITH_CYL_CAP_WHY").is_ok_and(|v| v == "1") {
        eprintln!("CYLCAP 頂点の半径のずれ {:.3e} cx {cx:.12} cy {cy:.12} λ(0) {:.3e}", radial.norm() - radius, cx - cx.signum() * (radius * radius - cy * cy).max(0.0).sqrt());
    }
    if cx.abs() <= 1e-9 * radius {
        return None;
    }
    Some((
        index,
        CylEnd { at_start, centre: base, axis, radius, circle_on_a, t_q, t_p, out, cx, cy, vertex },
    ))
}

/// 円柱の端の、円の面の上で頂点に繋がる円弧の稜。退いた点は円柱に沿って
/// この弧の上を進む。許す後退距離は、弧のもう一方の端か、円が t_q の向きに
/// 折り返す点のうち近いほうまで（4-962）。
fn find_arc_neighbour(
    face: &Face,
    edge_id: u64,
    c: &CylEnd,
    on_face_a: bool,
) -> Result<NeighbourTrim, String> {
    let vertex = c.vertex;
    let scale = c.radius.max(1.0);
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
            let fail = |why: &str| {
                Err(format!("Neighbouring edge {} at the cylinder end: {why}", edge.id))
            };
            let Some((centre, radius, _)) = crate::circular_fillet::circle_from_edge(edge) else {
                return fail("it is not a circular arc");
            };
            let offset = centre - c.centre;
            if (radius - c.radius).abs() > 1e-7 * scale
                || (offset - c.axis * offset.dot(&c.axis)).norm() > 1e-7 * scale
            {
                return fail("it is not on the cylinder");
            }
            let other = if move_start { edge.end_vertex.point } else { edge.start_vertex.point };
            if (other - vertex).norm() <= 1e-9 {
                return fail("it is a full circle");
            }
            let u_other = (other - vertex).dot(&c.t_q);
            if u_other <= 0.0 {
                return fail("it does not run in the direction the blend retreats");
            }
            let mut available = u_other;
            let turn = c.cy + c.radius;
            let extreme = vertex + c.t_q * turn + c.out * c.cx;
            if on_arc(edge, centre, radius, extreme) {
                available = available.min(turn);
            }
            // 向きの確かめ: 退く点の通り道が、この弧に乗るか
            let half = available * 0.5;
            let Some(lambda) = c.lambda(half) else {
                return fail("the retreat leaves the cylinder");
            };
            if !on_arc(edge, centre, radius, vertex + c.t_q * half + c.out * lambda) {
                return fail("the retreat leaves the arc");
            }
            return Ok(NeighbourTrim {
                edge_id: edge.id,
                move_start,
                available,
                at_start_of_target: c.at_start,
                on_face_a,
                arc: Some((centre, radius, c.axis)),
            });
        }
    }
    Err(format!("Edge {edge_id} has no arc beside it at the cylinder end"))
}




/// 1区間の Bézier（節が対称）の p-curve を逆向きに。
fn reverse_bezier2(curve: &zenith_geom::NurbsCurve2) -> zenith_geom::NurbsCurve2 {
    let mut points = curve.control_points.clone();
    points.reverse();
    zenith_geom::NurbsCurve2 { degree: curve.degree, control_points: points, knots: curve.knots.clone() }
}

/// 同次座標の Bézier を [lo, hi] に切り詰める（de Casteljau）。
fn bezier_segment<const N: usize>(cps: &[[f64; N]], lo: f64, hi: f64) -> Vec<[f64; N]> {
    let split = |cps: &[[f64; N]], t: f64| -> (Vec<[f64; N]>, Vec<[f64; N]>) {
        let mut levels = vec![cps.to_vec()];
        while levels.last().map_or(0, |l| l.len()) > 1 {
            let prev = levels.last().cloned().unwrap_or_default();
            let next: Vec<[f64; N]> = prev
                .windows(2)
                .map(|p| {
                    let mut out = [0.0; N];
                    for j in 0..N {
                        out[j] = p[0][j] * (1.0 - t) + p[1][j] * t;
                    }
                    out
                })
                .collect();
            levels.push(next);
        }
        let left = levels.iter().map(|l| l[0]).collect();
        let right = levels.iter().rev().map(|l| l[l.len() - 1]).collect();
        (left, right)
    };
    let left = split(cps, hi).0;
    if hi <= 1e-15 {
        return left;
    }
    split(&left, lo / hi).1
}

/// 円柱の端の楕円弧を、円柱の面の円の引数で作る（4-962）。3D の曲線（有理2次）と
/// p-curve（有理3次）が同じ引数を持つので、検証の「p-curve と3Dの辺が合う」を満たす。
/// 円の面の側の点 `from`（円柱に乗る）から、頂点を後退距離だけ持ち上げた点へ。
fn cylinder_end_curves(
    surface: &NurbsSurface3,
    c: &CylEnd,
    setback: f64,
    from: Point3,
) -> Option<(NurbsCurve3, zenith_geom::NurbsCurve2)> {
    use zenith_geom::{ControlPoint2, NurbsCurve2};
    use zenith_math::Point2;
    if surface.degree_u != 2 || surface.degree_v != 1 {
        return None;
    }
    let rows = &surface.control_points;
    if rows.iter().any(|row| row.len() != 2) || surface.knots_v.knots.len() != 4 {
        return None;
    }
    let (v0, v1) = (surface.knots_v.knots[0], surface.knots_v.knots[3]);
    let knots = &surface.knots_u.knots;
    let n = rows.len();
    if n % 2 == 0 || knots.len() != n + 3 {
        return None;
    }
    let spans = (n - 1) / 2;
    let breaks: Vec<f64> = (0..=spans).map(|k| knots[2 * k + 1]).collect();
    for k in 0..=spans {
        let expected = if k == 0 || k == spans { 3 } else { 2 };
        if knots.iter().filter(|&&x| (x - breaks[k]).abs() <= 1e-15).count() != expected {
            return None;
        }
    }
    let scale = c.radius.max(1.0);
    for k in 0..spans {
        let span: Vec<(Point3, Point3, f64)> = (0..3)
            .map(|i| (rows[2 * k + i][0].point, rows[2 * k + i][1].point, rows[2 * k + i][0].weight))
            .collect();
        if (0..3).any(|i| (rows[2 * k + i][1].weight - span[i].2).abs() > 1e-15) {
            continue;
        }
        let lift = span[0].1 - span[0].0;
        let height = lift.norm();
        if height <= 1e-12
            || (0..3).any(|i| ((span[i].1 - span[i].0) - lift).norm() > 1e-9 * height.max(1.0))
        {
            continue;
        }
        let e = lift / height;
        let z_q = (c.vertex - span[0].0).dot(&e);
        let w: Vec<f64> = span.iter().map(|s| s.2).collect();
        let on_q: Vec<Point3> = span.iter().map(|s| s.0 + e * z_q).collect();
        // 円の面の高さの円で、点 x に当たる引数（2次方程式の根のうち、本当に乗るもの）
        let invert = |x: Point3| -> Option<f64> {
            let r: Vec<Vec3> = (0..3).map(|i| (on_q[i] - x) * w[i]).collect();
            let (p0, p1, p2) = (r[0], (r[1] - r[0]) * 2.0, r[0] - r[1] * 2.0 + r[2]);
            let mut roots = Vec::new();
            for j in 0..3 {
                let (qa, qb, qc) = (p2[j], p1[j], p0[j]);
                if qa.abs() > 1e-14 {
                    let disc = qb * qb - 4.0 * qa * qc;
                    if disc >= -1e-12 {
                        let root = disc.max(0.0).sqrt();
                        roots.push((-qb + root) / (2.0 * qa));
                        roots.push((-qb - root) / (2.0 * qa));
                    }
                } else if qb.abs() > 1e-14 {
                    roots.push(-qc / qb);
                }
            }
            roots
                .into_iter()
                .filter(|t| *t >= -1e-12 && *t <= 1.0 + 1e-12)
                .map(|t| t.clamp(0.0, 1.0))
                .find(|&t| {
                    let b = [(1.0 - t) * (1.0 - t), 2.0 * t * (1.0 - t), t * t];
                    let weight: f64 = (0..3).map(|i| b[i] * w[i]).sum();
                    let point = (0..3).fold(Vec3::zeros(), |acc, i| acc + on_q[i].coords * (b[i] * w[i]));
                    ((point / weight) - x.coords).norm() <= 1e-9 * scale
                })
        };
        let (Some(s_from), Some(s_vertex)) = (invert(from), invert(c.vertex)) else {
            continue;
        };
        let (lo, hi) = (s_from.min(s_vertex), s_from.max(s_vertex));
        if hi - lo <= 1e-12 {
            return None;
        }
        // 3D: 円の面の円を p ↦ p + t_p (s − (p−V)·t_q) で写す（制御点を写す）
        let lifted = |p: Point3| p + c.t_p * (setback - (p - c.vertex).dot(&c.t_q));
        let hom3: Vec<[f64; 4]> = (0..3)
            .map(|i| {
                let p = lifted(on_q[i]);
                [p.x * w[i], p.y * w[i], p.z * w[i], w[i]]
            })
            .collect();
        // 2D: (u, v)。v·W は2次、u·W は3次、W を3次に上げる
        let (ka, kb) = (breaks[k], breaks[k + 1]);
        let delta = kb - ka;
        let a = (z_q + c.t_p.dot(&e) * setback) / height;
        let b = -c.t_p.dot(&e) / height;
        let vq: Vec<f64> = (0..3)
            .map(|i| w[i] * (v0 + (v1 - v0) * (a + b * (on_q[i] - c.vertex).dot(&c.t_q))))
            .collect();
        let elevate = |q: &[f64]| [q[0], (q[0] + 2.0 * q[1]) / 3.0, (2.0 * q[1] + q[2]) / 3.0, q[2]];
        let (w3, v3) = (elevate(&w), elevate(&vq));
        let uw = [
            ka * w3[0],
            ka * w3[1] + delta * w[0] / 3.0,
            ka * w3[2] + delta * 2.0 * w[1] / 3.0,
            ka * w3[3] + delta * w[2],
        ];
        let hom2: Vec<[f64; 3]> = (0..4).map(|i| [uw[i], v3[i], w3[i]]).collect();
        let mut seg3 = bezier_segment(&hom3, lo, hi);
        let mut seg2 = bezier_segment(&hom2, lo, hi);
        if s_from > s_vertex {
            seg3.reverse();
            seg2.reverse();
        }
        if seg3.iter().any(|h| !(h[3] > 0.0)) || seg2.iter().any(|h| !(h[2] > 0.0)) {
            return None;
        }
        let curve3 = NurbsCurve3::new(
            2,
            seg3.iter()
                .map(|h| ControlPoint3::new(Point3::new(h[0] / h[3], h[1] / h[3], h[2] / h[3]), h[3]))
                .collect(),
            KnotVector::clamped_uniform(3, 2),
        )
        .ok()?;
        let curve2 = NurbsCurve2::new(
            3,
            seg2.iter()
                .map(|h| ControlPoint2::new(Point2::new(h[0] / h[2], h[1] / h[2]), h[2]))
                .collect(),
            KnotVector::clamped_uniform(4, 3),
        )
        .ok()?;
        return Some((curve3, curve2));
    }
    None
}
/// 元の円弧の稜と同じ回り方で、同じ円の上の start から end までの弧（四半周ごとに区切る）。
fn arc_on_circle(
    original: &Edge,
    centre: Point3,
    radius: f64,
    axis: Vec3,
    start: Point3,
    end: Point3,
) -> Result<NurbsCurve3, String> {
    let (t0, t1) = original.curve.param_range();
    let (o0, om, o1) = (
        original.curve.evaluate(t0) - centre,
        original.curve.evaluate((t0 + t1) * 0.5) - centre,
        original.curve.evaluate(t1) - centre,
    );
    let angle = |v: Vec3, x: Vec3, y: Vec3| {
        let a = v.dot(&y).atan2(v.dot(&x));
        if a < 0.0 { a + std::f64::consts::TAU } else { a }
    };
    let x0 = o0 / radius;
    // 元の弧が start→middle→end と左回りに進む軸の向き
    let y0 = axis.cross(&x0);
    let n = if angle(om, x0, y0) <= angle(o1, x0, y0) { axis } else { -axis };
    let x = (start - centre) / radius;
    let y = n.cross(&x);
    let sweep = angle(end - centre, x, y);
    if sweep <= 1e-12 {
        return Err("A trimmed arc came out with no length".into());
    }
    let spans = (sweep / std::f64::consts::FRAC_PI_2 - 1e-9).ceil().max(1.0) as usize;
    let step = sweep / spans as f64;
    let half = step * 0.5;
    let at = |phi: f64| centre + (x * phi.cos() + y * phi.sin()) * radius;
    let mut points = vec![ControlPoint3::unweighted(start)];
    let mut knots = vec![0.0, 0.0, 0.0];
    for span in 0..spans {
        let phi = step * span as f64;
        let mid = centre + (x * (phi + half).cos() + y * (phi + half).sin()) * (radius / half.cos());
        points.push(ControlPoint3::new(mid, half.cos()));
        let next = if span + 1 == spans { end } else { at(phi + step) };
        points.push(ControlPoint3::unweighted(next));
        if span + 1 == spans {
            knots.extend([1.0, 1.0, 1.0]);
        } else {
            let k = (span + 1) as f64 / spans as f64;
            knots.extend([k, k]);
        }
    }
    NurbsCurve3::new(2, points, KnotVector::new(knots))
}

/// ある面のワイヤで、対象の稜と頂点を共有するもう1本の稜を探す
fn find_neighbour(
    face: &Face,
    edge_id: u64,
    vertex: Point3,
    dir: Vec3,
    normal: Vec3,
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
            // 凹の角（面の内角が 180 度を超える頂点）では、隣の稜は退く向きと
            // 逆に走る。端点を退いた点へ動かすと、詰めるのではなく同じ直線の
            // まま延びる（4-940、`ZENITH_REFLEX_END`）。延ばす量に稜の長さの
            // 上限は無い。
            let extends = reflex_end_enabled() && along.dot(&dir) <= -(1.0 - 1e-7);
            if extends {
                let deviation = max_chord_deviation(
                    &edge.curve,
                    edge.start_vertex.point,
                    along * if move_start { 1.0 } else { -1.0 },
                    available,
                );
                if deviation > 1e-9 * available.max(1.0) {
                    return Err(format!(
                        "Neighbouring edge {} is not straight; it cannot be extended past the corner",
                        edge.id
                    ));
                }
                // 延びた頂点は、面の中を退く向きへ進む。面の境界（頂点に繋がる稜を
                // 除く）にぶつかるまでしか延ばせない（4-958。上限が無かったので、
                // 頂点が面の外、立体の外へ出た形を返していた）。
                let available = if reflex_reach_enabled() {
                    ray_exit(face, vertex, dir, normal)?
                } else {
                    f64::INFINITY
                };
                return Ok(NeighbourTrim {
                    edge_id: edge.id,
                    move_start,
                    available,
                    at_start_of_target,
                    on_face_a,
                    arc: None,
                });
            }
            if along.dot(&dir) < 1.0 - 1e-7 {
                return Err(format!(
                    "Neighbouring edge {} on face {face_index} does not run in the direction the blend retreats",
                    edge.id
                ));
            }
            let deviation = max_chord_deviation(
                &edge.curve,
                edge.start_vertex.point,
                along * if move_start { 1.0 } else { -1.0 },
                available,
            );
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
                arc: None,
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

        return Err("A trimmed loop left a gap that neither end of the blend closes".to_string());
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

/// 曲線が始点から `dir` に伸びる直線からどれだけ離れるかの上限（4-960）。
///
/// 重みが全部正の有理曲線は制御点の凸包に入り、直線からの距離は凸な関数なので、
/// 制御点の距離の最大が曲線全体の距離の上限になる（標本を取らない）。重みに正で
/// ないものがあれば上限が取れないので無限大（直線と見なさない）。
fn max_chord_deviation(curve: &NurbsCurve3, origin: Point3, dir: Vec3, _length: f64) -> f64 {
    let mut worst: f64 = 0.0;
    for control in &curve.control_points {
        if !(control.weight > 0.0) {
            return f64::INFINITY;
        }
        let offset = control.point - origin;
        worst = worst.max((offset - dir * offset.dot(&dir)).norm());
    }
    worst
}

/// 凹の角で隣の稜を延ばす（4-940）。既定オフ。
fn reflex_end_enabled() -> bool {
    std::env::var("ZENITH_REFLEX_END")
        .map(|value| value != "0")
        .unwrap_or(true)
}

/// 立体の全ての稜 ID
fn edge_ids(solid: &Solid) -> Vec<u64> {
    let mut ids = Vec::new();
    for face in &solid.outer_shell.faces {
        for wire in std::iter::once(&face.outer_wire).chain(face.inner_wires.iter()) {
            for oriented in &wire.edges {
                if !ids.contains(&oriented.edge.id) {
                    ids.push(oriented.edge.id);
                }
            }
        }
    }
    ids
}

/// `fillet_edges` で、前の鎖が同じ半径で丸めて消した稜を飛ばすか（4-947、既定オン）
fn skip_consumed_enabled() -> bool {
    std::env::var("ZENITH_FILLET_EDGES_SKIP_CONSUMED")
        .map(|value| value != "0")
        .unwrap_or(true)
}

/// 点 `point`（円の上）が稜 `edge` の円弧の範囲にあるか。弧の中点を通る向きで、
/// 始点からの角度を比べる。
fn on_arc(edge: &Edge, center: Point3, radius: f64, point: Point3) -> bool {
    let (t0, t1) = edge.curve.param_range();
    let start = edge.curve.evaluate(t0) - center;
    let end = edge.curve.evaluate(t1) - center;
    let middle = edge.curve.evaluate((t0 + t1) * 0.5) - center;
    let Some(normal) = start.cross(&middle).try_normalize_safe(1e-12) else {
        return false;
    };
    let x = start / radius;
    let y = normal.cross(&x);
    let angle = |v: Vec3| {
        let a = v.dot(&y).atan2(v.dot(&x));
        if a < 0.0 { a + std::f64::consts::TAU } else { a }
    };
    let sweep = angle(end);
    let sweep = if sweep <= 1e-12 { std::f64::consts::TAU } else { sweep };
    angle(point - center) <= sweep
}

/// 稜（直線か円弧）の上で、`bounds` の各1次式 g·(p−origin) が [lo, hi] に入る部分の、
/// 1次式 objective·(p−origin) の最小（4-956）。入る部分が無ければ `Ok(None)`、
/// 直線でも円弧でもなければ `Err(())`。1次式の最小は、切り詰めた両端か、円弧なら
/// 目的の向きの極値点と境の線との交点のどれかに在るので、それらを厳密に並べる。
fn min_linear_on_edge(
    edge: &Edge,
    origin: Point3,
    objective: Vec3,
    bounds: &[(Vec3, f64, f64)],
) -> Result<Option<f64>, ()> {
    let (start, end) = (edge.start_vertex.point, edge.end_vertex.point);
    let length = (end - start).norm();
    let value = |g: Vec3, p: Point3| (p - origin).dot(&g);
    let straight = length > 1e-12
        && max_chord_deviation(&edge.curve, start, (end - start) / length, length)
            <= 1e-9 * length.max(1.0);
    let mut candidates: Vec<Point3> = Vec::new();
    if straight {
        let (mut a, mut b) = (0.0f64, 1.0f64);
        for &(g, lo, hi) in bounds {
            let (g0, g1) = (value(g, start), value(g, end));
            if (g1 - g0).abs() > 1e-15 {
                let (t_lo, t_hi) = ((lo - g0) / (g1 - g0), (hi - g0) / (g1 - g0));
                a = a.max(t_lo.min(t_hi));
                b = b.min(t_lo.max(t_hi));
            } else if g0 < lo || g0 > hi {
                return Ok(None);
            }
        }
        if a > b {
            return Ok(None);
        }
        candidates.push(start + (end - start) * a);
        candidates.push(start + (end - start) * b);
    } else {
        let Some((center, radius, _)) = crate::circular_fillet::circle_from_edge(edge) else {
            return Err(());
        };
        let (t0, t1) = edge.curve.param_range();
        let middle = edge.curve.evaluate((t0 + t1) * 0.5);
        let Some(normal) = (start - center).cross(&(middle - center)).try_normalize_safe(1e-12)
        else {
            return Err(());
        };
        let x = (start - center) / radius;
        let y = normal.cross(&x);
        let at = |phi: f64| center + (x * phi.cos() + y * phi.sin()) * radius;
        candidates.push(start);
        candidates.push(end);
        let flat = objective - normal * objective.dot(&normal);
        if let Some(flat) = flat.try_normalize_safe(1e-15) {
            candidates.push(center - flat * radius);
        }
        // 境 g·(p−origin) = lo / hi と円の交点: A cos φ + B sin φ = C
        for &(g, lo, hi) in bounds {
            let (aa, bb) = (x.dot(&g) * radius, y.dot(&g) * radius);
            let rho = (aa * aa + bb * bb).sqrt();
            if rho <= 1e-15 {
                continue;
            }
            for level in [lo, hi] {
                if !level.is_finite() {
                    continue;
                }
                let c = (level - value(g, center)) / rho;
                if c.abs() <= 1.0 {
                    let base = bb.atan2(aa);
                    let off = c.acos();
                    candidates.push(at(base + off));
                    candidates.push(at(base - off));
                }
            }
        }
        candidates.retain(|&p| on_arc(edge, center, radius, p));
    }
    let slack = 1e-9 * length.max(1.0);
    let best = candidates
        .into_iter()
        .filter(|&p| {
            bounds.iter().all(|&(g, lo, hi)| {
                let v = value(g, p);
                v >= lo - slack && v <= hi + slack
            })
        })
        .map(|p| value(objective, p))
        .fold(f64::INFINITY, f64::min);
    Ok(best.is_finite().then_some(best))
}

/// 面の中の点 `vertex` から、面の中の向き `dir` へ進む半直線が、面の境界（`vertex`
/// に繋がる稜を除く）に最初にぶつかるまでの距離（4-958）。直線と円弧は厳密に
/// 交わりを取り、ほかの曲線の境界は測れないので断る。ぶつからなければ無限大。
fn ray_exit(face: &Face, vertex: Point3, dir: Vec3, normal: Vec3) -> Result<f64, String> {
    let side = normal.cross(&dir);
    let Some(side) = side.try_normalize_safe(1e-12) else {
        return Err("The blend retreats out of the plane of its face".to_string());
    };
    let mut nearest = f64::INFINITY;
    for wire in std::iter::once(&face.outer_wire).chain(face.inner_wires.iter()) {
        for oriented in &wire.edges {
            let edge = &oriented.edge;
            let near = |p: Point3| (p - vertex).norm() <= 1e-9;
            if near(edge.start_vertex.point) || near(edge.end_vertex.point) {
                continue;
            }
            let bounds = [(side, 0.0, 0.0), (dir, 1e-9, f64::INFINITY)];
            match min_linear_on_edge(edge, vertex, dir, &bounds) {
                Ok(Some(hit)) => nearest = nearest.min(hit),
                Ok(None) => {}
                Err(()) => {
                    return Err(format!(
                        "Edge {} bounds the face the blend extends into, and it is neither straight nor a circular arc",
                        edge.id
                    ))
                }
            }
        }
    }
    Ok(nearest)
}

/// 凹の角で延ばす隣の稜に、面の境界までの上限を入れるか（4-958、`=0` で切れる）
fn reflex_reach_enabled() -> bool {
    std::env::var("ZENITH_REFLEX_REACH").map(|value| value != "0").unwrap_or(true)
}

/// 平面×平面の丸めで、両側の面の穴との隔たりを上限に入れるか（4-954、`=0` で切れる）
fn hole_clearance_enabled() -> bool {
    std::env::var("ZENITH_BLEND_HOLE_CLEARANCE")
        .map(|value| value != "0")
        .unwrap_or(true)
}

/// 円柱の縁の部分円弧の面取り（4-966、既定オン、`ZENITH_ARC_RIM_CHAMFER=0` で切る）。
///
/// 稜は、軸に直交する平面 P と凸の円柱 Q の間の円弧で、両端は軸に平行な平面 E に当たる
/// （D字ブロックの上下の円弧）。面取りの面は円錐で、端は円錐と E の交わりの双曲線
/// （有理2次）。P の円弧は半径 R−s、Q の円弧は s だけ下へ、P の直線と Q の母線を詰める。
/// 削れる体積は ∫_{R−s}^{R} (s−R+r)·r·Θ(r) dr（Θ は半径 r の円が2枚の E に挟まれる角）。
fn arc_rim_chamfer_enabled() -> bool {
    std::env::var("ZENITH_ARC_RIM_CHAMFER").map_or(true, |v| v != "0")
}

/// 円柱の面の向き: 中ほどの法線が軸から外を向けば true（立体は内側）。
fn cylinder_faces_out(surface: &NurbsSurface3, forward: bool, base: Point3, axis: Vec3) -> Option<bool> {
    let ((u0, u1), (w0, w1)) = surface.param_range();
    let (um, wm) = ((u0 + u1) * 0.5, (w0 + w1) * 0.5);
    let normal = surface.normal(um, wm)?;
    let offset = surface.evaluate(um, wm) - base;
    let radial = offset - axis * offset.dot(&axis);
    let sign = normal.dot(&radial) * if forward { 1.0 } else { -1.0 };
    Some(sign > 0.0)
}

/// 中心 centre・半径 radius、平面の基底 (x, y) の円の角 lo..hi の制御点と節（四半周ごと）。
fn circle_span(
    centre: Point3,
    radius: f64,
    x: Vec3,
    y: Vec3,
    lo: f64,
    hi: f64,
) -> (Vec<ControlPoint3>, KnotVector) {
    let sweep = hi - lo;
    let spans = (sweep / std::f64::consts::FRAC_PI_2 - 1e-9).ceil().max(1.0) as usize;
    let step = sweep / spans as f64;
    let half = step * 0.5;
    let at = |phi: f64| centre + (x * phi.cos() + y * phi.sin()) * radius;
    let mut points = vec![ControlPoint3::unweighted(at(lo))];
    let mut knots = vec![0.0, 0.0, 0.0];
    for span in 0..spans {
        let phi = lo + step * span as f64;
        let mid = centre + (x * (phi + half).cos() + y * (phi + half).sin()) * (radius / half.cos());
        points.push(ControlPoint3::new(mid, half.cos()));
        points.push(ControlPoint3::unweighted(at(phi + step)));
        if span + 1 == spans {
            knots.extend([1.0, 1.0, 1.0]);
        } else {
            let k = (span + 1) as f64 / spans as f64;
            knots.extend([k, k]);
        }
    }
    (points, KnotVector::new(knots))
}

/// Gauss–Legendre の節と重み（[-1, 1]、Newton で根を取る）。
fn gauss_legendre(n: usize) -> Vec<(f64, f64)> {
    let mut out = Vec::with_capacity(n);
    for i in 0..n {
        let mut x = (std::f64::consts::PI * (i as f64 + 0.75) / (n as f64 + 0.5)).cos();
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

struct RimEnd {
    vertex: Point3,
    face: usize,
    p_line: Edge,
    q_line: Edge,
    /// P の直線の、頂点から離れる向き（単位）
    u: Vec3,
    top: Point3,
    /// 端の平面の法線
    normal: Vec3,
}

/// 面のワイヤで、点に触れる稜（skip の id を除く）。
fn edges_touching(face: &Face, vertex: Point3, skip: &[u64]) -> Vec<Edge> {
    std::iter::once(&face.outer_wire)
        .chain(face.inner_wires.iter())
        .flat_map(|w| w.edges.iter())
        .filter(|o| !skip.contains(&o.edge.id))
        .filter(|o| {
            (o.edge.start_vertex.point - vertex).norm() <= 1e-9
                || (o.edge.end_vertex.point - vertex).norm() <= 1e-9
        })
        .map(|o| o.edge.clone())
        .collect()
}

fn far_end(edge: &Edge, vertex: Point3) -> Point3 {
    if (edge.start_vertex.point - vertex).norm() <= 1e-9 {
        edge.end_vertex.point
    } else {
        edge.start_vertex.point
    }
}

fn try_chamfer_arc_rim(
    solid: &Solid,
    edge_id: u64,
    s: f64,
) -> Result<Option<(Solid, EdgeBlendReport)>, String> {
    if !arc_rim_chamfer_enabled() {
        return Ok(None);
    }
    let tol = Tolerance::default();
    let faces = &solid.outer_shell.faces;
    let users = |id: u64| -> Vec<(usize, Orientation, Edge)> {
        let mut out = Vec::new();
        for (index, face) in faces.iter().enumerate() {
            for wire in std::iter::once(&face.outer_wire).chain(face.inner_wires.iter()) {
                for oriented in &wire.edges {
                    if oriented.edge.id == id {
                        out.push((index, oriented.orientation, oriented.edge.clone()));
                    }
                }
            }
        }
        out
    };
    let uses = users(edge_id);
    if uses.len() != 2 || uses[0].0 == uses[1].0 {
        return Ok(None);
    }
    let edge = uses[0].2.clone();
    let Some((centre, radius, _)) = crate::circular_fillet::circle_from_edge(&edge) else {
        return Ok(None);
    };
    let (mut pi, mut qi) = (None, None);
    for (index, _, _) in &uses {
        match &faces[*index].geometry {
            FaceGeometry::Plane(_) => pi = Some(*index),
            FaceGeometry::Nurbs(_) => qi = Some(*index),
            _ => {}
        }
    }
    let (Some(pi), Some(qi)) = (pi, qi) else { return Ok(None) };
    let FaceGeometry::Plane(plane) = &faces[pi].geometry else { return Ok(None) };
    let up = if faces[pi].orientation.is_forward() { plane.normal } else { -plane.normal };
    let scale = radius.max(1.0);
    // 円柱の面か（同じ軸・半径、凸）
    let q_ok = |index: usize| -> bool {
        let FaceGeometry::Nurbs(surface) = &faces[index].geometry else { return false };
        let Some((base, big, axis, _)) = crate::brep_intersection::cylinder_patch_reading(surface, &tol)
        else {
            return false;
        };
        let Some(axis) = axis.try_normalize_safe(1e-12) else { return false };
        let off = centre - base;
        axis.dot(&up).abs() >= 1.0 - 1e-12
            && (big - radius).abs() <= 1e-7 * scale
            && (off - axis * off.dot(&axis)).norm() <= 1e-7 * scale
            && cylinder_faces_out(surface, faces[index].orientation.is_forward(), base, axis) == Some(true)
    };
    if !q_ok(qi) {
        return Ok(None);
    }
    let same_circle = |e: &Edge| {
        crate::circular_fillet::circle_from_edge(e).is_some_and(|(c, r, _)| {
            (c - centre).norm() <= 1e-7 * scale && (r - radius).abs() <= 1e-7 * scale
        })
    };

    // 同じ円の上で接して続く円弧を鎖にたどる（円柱の面が継ぎ目で分かれている）
    let mut chain: Vec<(Edge, usize)> = vec![(edge.clone(), qi)];
    let (mut v0, mut v1) = (edge.start_vertex.point, edge.end_vertex.point);
    for forward in [true, false] {
        loop {
            let ids: Vec<u64> = chain.iter().map(|c| c.0.id).collect();
            let x = if forward { v1 } else { v0 };
            let arcs: Vec<Edge> =
                edges_touching(&faces[pi], x, &ids).into_iter().filter(|e| same_circle(e)).collect();
            if arcs.is_empty() {
                break;
            }
            if arcs.len() > 1 || chain.len() >= 16 {
                return Ok(None);
            }
            let next = arcs[0].clone();
            let next_users = users(next.id);
            if next_users.len() != 2 {
                return Ok(None);
            }
            let Some(q) = next_users.iter().map(|u| u.0).find(|&i| i != pi) else { return Ok(None) };
            if !q_ok(q) {
                return Ok(None);
            }
            let other = far_end(&next, x);
            let closes = if forward { (other - v0).norm() <= 1e-9 } else { (other - v1).norm() <= 1e-9 };
            if closes {
                return Ok(None); // 全周（`try_*_cylinder_rim` の領分）
            }
            if forward {
                chain.push((next, q));
                v1 = other;
            } else {
                chain.insert(0, (next, q));
                v0 = other;
            }
        }
    }
    let chain_ids: Vec<u64> = chain.iter().map(|c| c.0.id).collect();
    // 鎖の向きに対する各弧の向きと、内側の頂点
    let mut flags = Vec::new();
    let mut joints = Vec::new();
    let mut cursor = v0;
    for (arc, _) in &chain {
        let forward = (arc.start_vertex.point - cursor).norm() <= 1e-9;
        flags.push(forward);
        cursor = far_end(arc, cursor);
        joints.push(cursor);
    }
    joints.pop();

    // ここから先はこの形。合わなければ理由を付けて断る。
    let refuse = |why: &str| -> Result<Option<(Solid, EdgeBlendReport)>, String> {
        Err(format!("Edge {edge_id} is a cylinder rim arc; {why}"))
    };
    if !(s > 0.0) || s >= radius {
        return refuse("the chamfer is not smaller than the cylinder radius");
    }
    let r_top = radius - s;
    let radial = |p: Point3| {
        let o = p - centre;
        o - up * o.dot(&up)
    };
    let mut ends: Vec<RimEnd> = Vec::new();
    for (vertex, q_end) in [(v0, chain[0].1), (v1, chain[chain.len() - 1].1)] {
        let others: Vec<usize> = (0..faces.len())
            .filter(|&i| i != pi && i != q_end && !edges_touching(&faces[i], vertex, &chain_ids).is_empty())
            .collect();
        if others.len() != 1 {
            return refuse("an end does not meet exactly one other face");
        }
        let ei = others[0];
        let FaceGeometry::Plane(end_plane) = &faces[ei].geometry else {
            return refuse("an end face is not planar");
        };
        if end_plane.normal.dot(&up).abs() > 1e-9 {
            return refuse("an end face is not parallel to the cylinder axis");
        }
        let (p_lines, q_lines) =
            (edges_touching(&faces[pi], vertex, &chain_ids), edges_touching(&faces[q_end], vertex, &chain_ids));
        if p_lines.len() != 1 || q_lines.len() != 1 {
            return refuse("an end has more than one neighbouring edge");
        }
        let (p_line, q_line) = (p_lines[0].clone(), q_lines[0].clone());
        let (p_far, q_far) = (far_end(&p_line, vertex), far_end(&q_line, vertex));
        let p_len = (p_far - vertex).norm();
        let q_len = (q_far - vertex).norm();
        if p_len <= 1e-12 || q_len <= 1e-12 {
            return refuse("a neighbouring edge has no length");
        }
        let u = (p_far - vertex) / p_len;
        let w = (q_far - vertex) / q_len;
        if max_chord_deviation(&p_line.curve, vertex, u, p_len) > 1e-9 * p_len.max(1.0)
            || max_chord_deviation(&q_line.curve, vertex, w, q_len) > 1e-9 * q_len.max(1.0)
        {
            return refuse("a neighbouring edge is not straight");
        }
        if w.dot(&-up) < 1.0 - 1e-9 || u.dot(&up).abs() > 1e-9 {
            return refuse("the neighbouring edges do not run along the axis and across it");
        }
        if s >= q_len * (1.0 - 1e-9) {
            return refuse("the chamfer is longer than the cylinder's ruling");
        }
        // P の直線の上で、軸から r_top の点（頂点に近い根）
        let wv = radial(vertex);
        let b = wv.dot(&u);
        let disc = b * b - wv.dot(&wv) + r_top * r_top;
        if disc < 0.0 {
            return refuse("the chamfer misses the line on the top face");
        }
        let t = -b - disc.sqrt();
        let t = if t > 1e-12 { t } else { -b + disc.sqrt() };
        if !(t > 0.0) || t >= p_len * (1.0 - 1e-9) {
            return refuse("the chamfer is longer than the line on the top face");
        }
        ends.push(RimEnd { vertex, face: ei, p_line, q_line, u, top: vertex + u * t, normal: end_plane.normal });
    }
    if ends[0].face == ends[1].face {
        return refuse("both ends meet the same face");
    }
    // 継ぎ目の母線（鎖の内側の頂点で、隣り合う円柱の面が共有する直線）
    let mut seams: Vec<Edge> = Vec::new();
    for (k, joint) in joints.iter().enumerate() {
        let here = edges_touching(&faces[chain[k].1], *joint, &chain_ids);
        let there = edges_touching(&faces[chain[k + 1].1], *joint, &chain_ids);
        if here.len() != 1 || there.len() != 1 || here[0].id != there[0].id {
            return refuse("the arcs do not meet at a seam of the cylinder");
        }
        let seam = here[0].clone();
        let far = far_end(&seam, *joint);
        let len = (far - *joint).norm();
        if len <= 1e-12
            || max_chord_deviation(&seam.curve, *joint, (far - *joint) / len, len) > 1e-9 * len.max(1.0)
            || ((far - *joint) / len).dot(&-up) < 1.0 - 1e-9
        {
            return refuse("a seam of the cylinder is not a ruling");
        }
        if s >= len * (1.0 - 1e-9) {
            return refuse("the chamfer is longer than a seam of the cylinder");
        }
        seams.push(seam);
    }

    // 回る向きと角（鎖の向きに左回り）
    let x = radial(v0).normalize();
    let first = &chain[0].0;
    let (t0, t1) = first.curve.param_range();
    let y0 = up.cross(&x);
    let ang0 = |v: Vec3| {
        let a = v.dot(&y0).atan2(v.dot(&x));
        if a < 0.0 { a + std::f64::consts::TAU } else { a }
    };
    let (m, e) = (radial(first.curve.evaluate((t0 + t1) * 0.5)), radial(far_end(first, v0)));
    let _ = t1;
    let sense = if ang0(m) <= ang0(e) { up } else { -up };
    let y = sense.cross(&x);
    let angle = |p: Point3| {
        let v = radial(p);
        v.dot(&y).atan2(v.dot(&x))
    };
    let mut angles = vec![0.0];
    for point in joints.iter().chain(std::iter::once(&v1)) {
        let last = *angles.last().unwrap_or(&0.0);
        let mut a = last + (angle(*point) - last).rem_euclid(std::f64::consts::TAU);
        if a - last <= 1e-12 {
            a += std::f64::consts::TAU;
        }
        angles.push(a);
    }
    let sweep = *angles.last().unwrap_or(&0.0);
    if sweep >= std::f64::consts::TAU - 1e-9 {
        return refuse("the arcs go all the way round");
    }
    let near = |a: f64, target: f64| {
        a + std::f64::consts::TAU * ((target - a) / std::f64::consts::TAU).round()
    };
    let (ts, te) = (near(angle(ends[0].top), 0.0), near(angle(ends[1].top), sweep));
    if !(te > ts) {
        return refuse("the chamfer closes the arc on the top face");
    }
    let lo = ts.min(0.0);
    let hi = te.max(sweep);
    let low = centre - up * s;

    // 頂点とブレンドの辺
    let vt: Vec<Vertex> = ends.iter().map(|e| Vertex::from_point(e.top)).collect();
    let vb: Vec<Vertex> = ends.iter().map(|e| Vertex::from_point(e.vertex - up * s)).collect();
    let vj: Vec<Vertex> = joints.iter().map(|j| Vertex::from_point(*j - up * s)).collect();
    let arc_curve = |c: Point3, r: f64, a: f64, b: f64| -> Result<NurbsCurve3, String> {
        let (points, knots) = circle_span(c, r, x, y, a, b);
        Ok(NurbsCurve3::new(2, points, knots)?)
    };
    let top_arc = Edge::new(arc_curve(centre, r_top, ts, te)?, vt[0].clone(), vt[1].clone(), 1e-6);
    let mut pieces = Vec::new();
    for k in 0..chain.len() {
        let a = if k == 0 { vb[0].clone() } else { vj[k - 1].clone() };
        let b = if k + 1 == chain.len() { vb[1].clone() } else { vj[k].clone() };
        pieces.push(Edge::new(arc_curve(low, radius, angles[k], angles[k + 1])?, a, b, 1e-6));
    }
    // 双曲線: r ↦ (直線の上で軸から r の点, 高さ z0 − s + R − r)。返すのは点と d/dr。
    let conic_point = |end: &RimEnd, r: f64| -> Option<(Point3, Vec3)> {
        let wv = radial(end.vertex);
        let b = wv.dot(&end.u);
        let disc = b * b - wv.dot(&wv) + r * r;
        if disc < 0.0 {
            return None;
        }
        let root = disc.sqrt();
        // 頂点（t=0, r=R）から続く根: r<R では t>0 の小さいほう
        let (t_minus, t_plus) = (-b - root, -b + root);
        let t = if (r - radius).abs() <= 1e-12 * scale {
            0.0
        } else if t_minus > 0.0 {
            t_minus
        } else {
            t_plus
        };
        let rel = wv + end.u * t;
        let dt = r / rel.dot(&end.u);
        let point = end.vertex + end.u * t - up * (s - (radius - r));
        Some((point, end.u * dt - up))
    };
    let (top_points, knots) = circle_span(centre, r_top, x, y, lo, hi);
    let (bottom_points, _) = circle_span(low, radius, x, y, lo, hi);
    let mut hyperbolas = Vec::new();
    let mut exact_pcurves = Vec::new();
    for (k, end) in ends.iter().enumerate() {
        let Some((curve3, curve2)) = cone_plane_curves(
            &top_points,
            &bottom_points,
            &knots,
            end.normal,
            end.normal.dot(&end.vertex.coords),
            end.top,
            end.vertex - up * s,
            scale,
        ) else {
            return refuse("the end curve could not be laid on the cone's own parameter");
        };
        let edge = Edge::new(curve3, vt[k].clone(), vb[k].clone(), 1e-6);
        exact_pcurves.push((edge.id, curve2));
        hyperbolas.push(edge);
    }

    // 詰めた直線（両端の直線と母線、継ぎ目の母線）
    let mut trimmed: BTreeMap<u64, Edge> = BTreeMap::new();
    let mut moves: Vec<(&Edge, Point3, &Vertex)> = Vec::new();
    for (k, end) in ends.iter().enumerate() {
        moves.push((&end.p_line, end.vertex, &vt[k]));
        moves.push((&end.q_line, end.vertex, &vb[k]));
    }
    for (k, seam) in seams.iter().enumerate() {
        moves.push((seam, joints[k], &vj[k]));
    }
    for (line, old, vertex) in moves {
        let at_start = (line.start_vertex.point - old).norm() <= 1e-9;
        let (a, b) = if at_start {
            (vertex.clone(), line.end_vertex.clone())
        } else {
            (line.start_vertex.clone(), vertex.clone())
        };
        let curve = NurbsCurve3::bspline_from_points(1, vec![a.point, b.point])?;
        trimmed.insert(
            line.id,
            Edge { id: line.id, curve, start_vertex: a, end_vertex: b, tolerance: line.tolerance },
        );
    }

    let mut top_forward = None;
    let mut new_faces: Vec<Face> = Vec::with_capacity(faces.len() + 1);
    for (index, face) in faces.iter().enumerate() {
        let mut placed_top = false;
        let mut substitute = |oriented: &OrientedEdge| -> Option<OrientedEdge> {
            if let Some(k) = chain_ids.iter().position(|&id| id == oriented.edge.id) {
                let along = if flags[k] { oriented.orientation } else { oriented.orientation.reversed() };
                if index == pi {
                    if placed_top {
                        return None;
                    }
                    placed_top = true;
                    top_forward = Some(along.is_forward());
                    return Some(OrientedEdge::new(top_arc.clone(), along));
                }
                return Some(OrientedEdge::new(pieces[k].clone(), along));
            }
            Some(match trimmed.get(&oriented.edge.id) {
                Some(edge) => OrientedEdge::new(edge.clone(), oriented.orientation),
                None => oriented.clone(),
            })
        };
        let is_end = ends.iter().any(|e| e.face == index);
        let mut rebuild = |wire: &Wire| -> Result<Wire, String> {
            let edges: Vec<OrientedEdge> = wire.edges.iter().filter_map(&mut substitute).collect();
            if is_end {
                close_gap(edges, &[&hyperbolas[0], &hyperbolas[1]], &tol)
            } else {
                Ok(Wire::new(edges))
            }
        };
        let outer = rebuild(&face.outer_wire)?;
        let mut inners = Vec::new();
        for wire in &face.inner_wires {
            inners.push(rebuild(wire)?);
        }
        new_faces.push(Face::new(face.geometry.clone(), outer, inners, face.orientation, face.tolerance));
    }
    let top_forward = top_forward.ok_or("the top face lost the arc")?;
    // 円錐の面の輪: 隣の面と逆向きに使う
    let mut cone_edges = Vec::new();
    if top_forward {
        cone_edges.push(OrientedEdge::reversed(top_arc.clone()));
        cone_edges.push(OrientedEdge::forward(hyperbolas[0].clone()));
        cone_edges.extend(pieces.iter().map(|p| OrientedEdge::forward(p.clone())));
        cone_edges.push(OrientedEdge::reversed(hyperbolas[1].clone()));
    } else {
        cone_edges.push(OrientedEdge::forward(top_arc.clone()));
        cone_edges.push(OrientedEdge::forward(hyperbolas[1].clone()));
        cone_edges.extend(pieces.iter().rev().map(|p| OrientedEdge::reversed(p.clone())));
        cone_edges.push(OrientedEdge::reversed(hyperbolas[0].clone()));
    }
    let rows: Vec<Vec<ControlPoint3>> = top_points
        .iter()
        .zip(bottom_points.iter())
        .map(|(a, b)| vec![*a, *b])
        .collect();
    let cone = NurbsSurface3::new(2, 1, rows, knots, KnotVector::clamped_uniform(2, 1))?;
    let mid_angle = 0.5 * (lo + hi);
    let outward = (x * mid_angle.cos() + y * mid_angle.sin()) + up;
    let ((u0, u1), (w0, w1)) = cone.param_range();
    let normal = cone
        .normal((u0 + u1) * 0.5, (w0 + w1) * 0.5)
        .ok_or("the chamfer cone has no normal")?;
    let orientation =
        if normal.dot(&outward) > 0.0 { Orientation::Forward } else { Orientation::Reversed };
    let mut cone_face =
        Face::new(FaceGeometry::Nurbs(cone), Wire::new(cone_edges), vec![], orientation, faces[qi].tolerance);
    // 双曲線の p-curve を当てはめから厳密なものへ（端点の合う向きで）
    if let Some(pcurves) = cone_face.pcurves.as_mut() {
        for segment in pcurves.outer_loop.segments.iter_mut() {
            if let Some((_, exact)) = exact_pcurves.iter().find(|(id, _)| *id == segment.edge_id) {
                let (f0, f1) = segment.curve.param_range();
                let (e0, e1) = exact.param_range();
                let same = (segment.curve.evaluate(f0) - exact.evaluate(e0)).norm()
                    + (segment.curve.evaluate(f1) - exact.evaluate(e1)).norm();
                let flipped = (segment.curve.evaluate(f0) - exact.evaluate(e1)).norm()
                    + (segment.curve.evaluate(f1) - exact.evaluate(e0)).norm();
                segment.curve = if same <= flipped { exact.clone() } else { reverse_bezier2(exact) };
            }
        }
    }
    new_faces.push(cone_face);
    let result = Solid::try_new(Shell::closed(new_faces), solid.inner_shells.clone(), &tol)
        .map_err(|err| err.to_string())?;

    // 削れる体積
    let theta = |r: f64| -> Option<f64> {
        let (a, _) = conic_point(&ends[0], r)?;
        let (b, _) = conic_point(&ends[1], r)?;
        Some(near(angle(b), sweep) - near(angle(a), 0.0))
    };
    let mut predicted = 0.0;
    let nodes = gauss_legendre(24);
    let parts = 4;
    for part in 0..parts {
        let a = r_top + s * part as f64 / parts as f64;
        let b = r_top + s * (part + 1) as f64 / parts as f64;
        for &(xi, wi) in &nodes {
            let r = 0.5 * (a + b) + 0.5 * (b - a) * xi;
            let th = theta(r).ok_or("the removed volume could not be integrated")?;
            predicted += 0.5 * (b - a) * wi * (s - radius + r) * r * th;
        }
    }
    Ok(Some((
        result,
        EdgeBlendReport {
            dihedral_angle_deg: 90.0,
            edge_length: radius * sweep,
            setback: s,
            predicted_removed_volume: predicted,
        },
    )))
}

/// 円錐（上の円の行と下の円の行の、重みの同じ線織面）と、平面 n·p = h の交わりを、
/// 円錐の面の u の引数で厳密に（4-966）。v = a/(a−b)（a, b は各行の n·P − h の
/// 2次式）。p-curve (u, v) は有理3次、3D は S(u, v(u)) で有理4次。
/// `from`（v=0 の側）から `to`（v=1 の側）へ。1区間に収まらなければ `None`。
fn cone_plane_curves(
    top: &[ControlPoint3],
    bottom: &[ControlPoint3],
    knots: &KnotVector,
    normal: Vec3,
    h: f64,
    from: Point3,
    to: Point3,
    scale: f64,
) -> Option<(NurbsCurve3, zenith_geom::NurbsCurve2)> {
    use zenith_geom::{ControlPoint2, NurbsCurve2};
    use zenith_math::Point2;
    let n = top.len();
    if n != bottom.len() || n % 2 == 0 {
        return None;
    }
    let spans = (n - 1) / 2;
    let k = &knots.knots;
    for span in 0..spans {
        let (ka, kb) = (k[2 * span + 2], k[2 * span + 3]);
        let delta = kb - ka;
        if !(delta > 0.0) {
            continue;
        }
        let idx = [2 * span, 2 * span + 1, 2 * span + 2];
        let w: Vec<f64> = idx.iter().map(|&i| top[i].weight).collect();
        if idx.iter().any(|&i| (bottom[i].weight - top[i].weight).abs() > 1e-15) {
            return None;
        }
        let a: Vec<f64> = idx.iter().map(|&i| top[i].weight * (normal.dot(&top[i].point.coords) - h)).collect();
        let b: Vec<f64> = idx.iter().map(|&i| bottom[i].weight * (normal.dot(&bottom[i].point.coords) - h)).collect();
        let d: Vec<f64> = (0..3).map(|i| a[i] - b[i]).collect();
        // 2次式の [0,1] の根
        let roots = |c: &[f64]| -> Vec<f64> {
            let (qa, qb, qc) = (c[0] - 2.0 * c[1] + c[2], 2.0 * (c[1] - c[0]), c[0]);
            let mut out = Vec::new();
            if qa.abs() > 1e-14 * (qb.abs() + qc.abs()).max(1e-300) {
                let disc = qb * qb - 4.0 * qa * qc;
                if disc >= 0.0 {
                    let root = disc.sqrt();
                    out.push((-qb + root) / (2.0 * qa));
                    out.push((-qb - root) / (2.0 * qa));
                }
            } else if qb.abs() > 0.0 {
                out.push(-qc / qb);
            }
            out.into_iter().filter(|s| *s >= -1e-6 && *s <= 1.0 + 1e-6).map(|s| s.clamp(0.0, 1.0)).collect()
        };
        // 3D の点（行 v、局所 s）
        let point_at = |s: f64, v: f64| -> Point3 {
            let bs = [(1.0 - s) * (1.0 - s), 2.0 * s * (1.0 - s), s * s];
            let mut num = Vec3::zeros();
            let mut den = 0.0;
            for j in 0..3 {
                let p = top[idx[j]].point.coords * (1.0 - v) + bottom[idx[j]].point.coords * v;
                num += p * (bs[j] * w[j]);
                den += bs[j] * w[j];
            }
            Point3::from(num / den)
        };
        let s_from = roots(&a).into_iter().find(|&s| (point_at(s, 0.0) - from).norm() <= 1e-9 * scale);
        let s_to = roots(&b).into_iter().find(|&s| (point_at(s, 1.0) - to).norm() <= 1e-9 * scale);
        if std::env::var("ZENITH_ARC_RIM_WHY").is_ok_and(|v| v == "1") {
            let ra = roots(&a);
            let rb = roots(&b);
            eprintln!(
                "ARCRIM span {span} a根 {:?} b根 {:?} from {:?} to {:?} a点 {:?} b点 {:?}",
                ra, rb, from, to,
                ra.iter().map(|&s| point_at(s, 0.0)).collect::<Vec<_>>(),
                rb.iter().map(|&s| point_at(s, 1.0)).collect::<Vec<_>>()
            );
        }
        let (Some(s_from), Some(s_to)) = (s_from, s_to) else { continue };
        let (lo, hi) = (s_from.min(s_to), s_from.max(s_to));
        if hi - lo <= 1e-12 {
            return None;
        }
        // 2D: (u·D, N, D) を3次に（N = a）
        let elevate = |q: &[f64]| [q[0], (q[0] + 2.0 * q[1]) / 3.0, (2.0 * q[1] + q[2]) / 3.0, q[2]];
        let (d3, n3) = (elevate(&d), elevate(&a));
        let ud = [
            ka * d3[0],
            ka * d3[1] + delta * d[0] / 3.0,
            ka * d3[2] + delta * 2.0 * d[1] / 3.0,
            ka * d3[3] + delta * d[2],
        ];
        let hom2: Vec<[f64; 3]> = (0..4).map(|i| [ud[i], n3[i], d3[i]]).collect();
        // 3D: (D−N)·Ã + N·B̃（Ã, B̃ は行の同次座標の2次式）、2次×2次 → 4次
        let product = |f: &[f64], g: &[f64]| -> [f64; 5] {
            let c2 = [1.0, 2.0, 1.0];
            let c4 = [1.0, 4.0, 6.0, 4.0, 1.0];
            let mut out = [0.0; 5];
            for i in 0..3 {
                for j in 0..3 {
                    out[i + j] += c2[i] * c2[j] * f[i] * g[j];
                }
            }
            for (k, value) in out.iter_mut().enumerate() {
                *value /= c4[k];
            }
            out
        };
        let dn: Vec<f64> = (0..3).map(|i| d[i] - a[i]).collect();
        let mut hom3: Vec<[f64; 4]> = vec![[0.0; 4]; 5];
        for c in 0..4 {
            let at: Vec<f64> = idx
                .iter()
                .map(|&i| {
                    let p = top[i].point;
                    let value = if c < 3 { p[c] } else { 1.0 };
                    value * top[i].weight
                })
                .collect();
            let bt: Vec<f64> = idx
                .iter()
                .map(|&i| {
                    let p = bottom[i].point;
                    let value = if c < 3 { p[c] } else { 1.0 };
                    value * bottom[i].weight
                })
                .collect();
            let (x, y) = (product(&dn, &at), product(&a, &bt));
            for k in 0..5 {
                hom3[k][c] = x[k] + y[k];
            }
        }
        let mut seg3 = bezier_segment(&hom3, lo, hi);
        let mut seg2 = bezier_segment(&hom2, lo, hi);
        if s_from > s_to {
            seg3.reverse();
            seg2.reverse();
        }
        // 同次の重みの符号をそろえる（D の符号が負でも同じ曲線）
        let sign3 = if seg3[0][3] < 0.0 { -1.0 } else { 1.0 };
        let sign2 = if seg2[0][2] < 0.0 { -1.0 } else { 1.0 };
        if seg3.iter().any(|q| !(q[3] * sign3 > 0.0)) || seg2.iter().any(|q| !(q[2] * sign2 > 0.0)) {
            return None;
        }
        let curve3 = NurbsCurve3::new(
            4,
            seg3.iter()
                .map(|q| ControlPoint3::new(Point3::new(q[0] / q[3], q[1] / q[3], q[2] / q[3]), q[3] * sign3))
                .collect(),
            KnotVector::clamped_uniform(5, 4),
        )
        .ok()?;
        let curve2 = NurbsCurve2::new(
            3,
            seg2.iter()
                .map(|q| ControlPoint2::new(Point2::new(q[0] / q[2], q[1] / q[2]), q[2] * sign2))
                .collect(),
            KnotVector::clamped_uniform(4, 3),
        )
        .ok()?;
        return Some((curve3, curve2));
    }
    None
}

/// 円柱の縁の部分円弧の上限（4-966）。面取りの距離の条件はどれも上限なので、作れるかで
/// 二分する。フィレットは端が有理曲線にならないので 0。
fn arc_rim_blendable(solid: &Solid, edge_id: u64) -> Option<BlendableEdge> {
    if !arc_rim_chamfer_enabled() {
        return None;
    }
    let edge = find_edge(&solid.outer_shell.faces, edge_id)?;
    let (_, radius, _) = crate::circular_fillet::circle_from_edge(&edge)?;
    let builds = |s: f64| matches!(try_chamfer_arc_rim(solid, edge_id, s), Ok(Some(_)));
    let (mut lo, mut hi) = (0.0, radius);
    // 形が当てはまらなければ（小さな距離でも Ok(None) か断り）上限は出さない
    match try_chamfer_arc_rim(solid, edge_id, radius * 1e-6) {
        Ok(Some(_)) => {}
        _ => return None,
    }
    for _ in 0..50 {
        let mid = 0.5 * (lo + hi);
        if builds(mid) {
            lo = mid;
        } else {
            hi = mid;
        }
    }
    let (_, report) = try_chamfer_arc_rim(solid, edge_id, radius * 1e-6).ok()??;
    Some(BlendableEdge {
        edge_id,
        length: report.edge_length,
        dihedral_angle_deg: 90.0,
        max_fillet_radius: 0.0,
        max_chamfer_distance: lo * 0.999,
    })
}

/// 断られた稜を、同じ平面で割れた面を 1 枚にまとめた立体でもう一度試すか（HANDOVER 4-977、
/// 既定オフ `ZENITH_BLEND_MERGE_RETRY=1`）。
fn merge_retry_enabled() -> bool {
    std::env::var("ZENITH_BLEND_MERGE_RETRY").map_or(true, |v| v != "0")
}

/// 同じ平面の面をまとめた立体と、その中で稜 `edge_id` と同じ幾何の稜。面が減らない、
/// または稜が割れたり消えたりして同じ幾何の稜が 1 本に決まらないなら None。
fn merged_twin(solid: &Solid, edge_id: u64) -> Option<(Solid, u64)> {
    if !merge_retry_enabled() {
        return None;
    }
    let edge = solid
        .outer_shell
        .faces
        .iter()
        .flat_map(|f| std::iter::once(&f.outer_wire).chain(f.inner_wires.iter()))
        .flat_map(|w| w.edges.iter())
        .find(|o| o.edge.id == edge_id)?
        .edge
        .clone();
    let (merged, _) = crate::FaceMerger::merge_coplanar(solid, &zenith_math::Tolerance::default()).ok()?;
    if merged.outer_shell.faces.len() >= solid.outer_shell.faces.len() {
        return None;
    }
    let mut twins: Vec<u64> = Vec::new();
    for o in merged
        .outer_shell
        .faces
        .iter()
        .flat_map(|f| std::iter::once(&f.outer_wire).chain(f.inner_wires.iter()))
        .flat_map(|w| w.edges.iter())
    {
        if crate::circular_fillet::same_edge_geometry(&edge, &o.edge) && !twins.contains(&o.edge.id) {
            twins.push(o.edge.id);
        }
    }
    (twins.len() == 1).then(|| (merged, twins[0]))
}
