use crate::{
    DirectModeling, ExtrudeBuilder, LoftBuilder, PrimitiveBuilder, ShellBuilder, SweepBuilder,
    ThickenBuilder,
};
use serde::{Deserialize, Serialize};
use zenith_geom::{ControlPoint3, KnotVector, NurbsCurve3};
use zenith_math::{Point3, Tolerance, Vec3};
use zenith_topo::{
    Edge, EdgeSignature, GeometricMatcher, GeometricSignature, OrientedEdge, Solid, Vertex, Wire,
};

/// パラメトリック・フィーチャー操作の種別
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub enum FeatureOp {
    /// 直方体プリミティブ
    CreateBox { dx: f64, dy: f64, dz: f64 },
    /// 円柱プリミティブ
    CreateCylinder { radius: f64, height: f64 },
    /// 円錐台プリミティブ
    CreateCone { r1: f64, r2: f64, height: f64 },
    /// トーラスプリミティブ
    CreateTorus { major_r: f64, minor_r: f64 },
    /// 単一エッジ・フィレット
    FilletEdge { dx: f64, dy: f64, dz: f64, edge_index: usize, radius: f64 },
    /// 単一エッジ・面取り
    ChamferEdge { dx: f64, dy: f64, dz: f64, edge_index: usize, distance: f64 },
    /// 中空ボックス・シェル化（単一面開口）
    HollowBox { dx: f64, dy: f64, dz: f64, thickness: f64, open_face_index: usize },
    /// 両端開口中空角パイプソリッド
    HollowThroughBox { dx: f64, dy: f64, dz: f64, thickness: f64 },
    /// 中空・穴あきプロファイルの押し出し

    ExtrudeHollow {
        outer_points: Vec<[f64; 3]>,
        inner_points: Vec<Vec<[f64; 3]>>,
        dir: [f64; 3],
    },
    /// 閉断面群からのロフトソリッド
    LoftSolid {
        sections: Vec<Vec<[f64; 3]>>,
        degree_v: usize,
    },
    /// ドラフト（抜き勾配）付き押し出し
    ExtrudeDraft {
        points: Vec<[f64; 3]>,
        dir: [f64; 3],
        draft_angle_rad: f64,
    },
    /// 閉断面ワイヤの回転体閉ソリッド（360度全周）
    RevolveSolid {
        profile_points: Vec<[f64; 3]>,
        axis_origin: [f64; 3],
        axis_dir: [f64; 3],
    },
    /// 閉断面ワイヤの部分角度回転体閉ソリッド（端面キャップ付き）
    RevolvePartialSolid {
        profile_points: Vec<[f64; 3]>,
        axis_origin: [f64; 3],
        axis_dir: [f64; 3],
        angle_rad: f64,
    },
    /// 任意閉断面ワイヤの3Dパススイープソリッド
    SweepWire {
        profile_points: Vec<[f64; 3]>,
        path_points: Vec<[f64; 3]>,
        num_sections: usize,
    },
    /// 閉断面ワイヤの3D螺旋（ヘリカル）スイープソリッド（スプリング・ネジ等）
    SweepHelix {
        profile_points: Vec<[f64; 3]>,
        radius: f64,
        pitch: f64,
        turns: f64,
        axis_origin: [f64; 3],
        axis_dir: [f64; 3],
        num_sections: usize,
    },
    /// 角丸めポリラインに沿ったパイプ掃引ソリッド
    PolylinePipe {
        path_points: Vec<[f64; 3]>,
        pipe_radius: f64,
        corner_radius: f64,
    },
    /// 任意対称平面に対するソリッドの鏡像反転複製

    MirrorSolid {
        plane_origin: [f64; 3],
        plane_normal: [f64; 3],
    },
    /// 面 Push-Pull（押し出し移動）

    PushPullFace {
        target_signature: GeometricSignature,
        distance: f64,
    },
    /// 面 厚み付け
    ThickenFace {
        target_signature: GeometricSignature,
        thickness: f64,
    },

    /// これまでの結果を平行移動する
    Translate { offset: [f64; 3] },

    /// これまでの結果を軸まわりに回転する
    Rotate {
        axis_origin: [f64; 3],
        axis_dir: [f64; 3],
        angle_deg: f64,
    },

    /// これまでの結果と、`tool` を組み立てた立体とのブーリアン。
    ///
    /// `tool` は**それ自身が短いフィーチャー列**で、空の状態から順に評価
    /// される。位置合わせは `tool` の末尾に `Translate` / `Rotate` を
    /// 置いて行う。これがあるまで、履歴ツリーは各段が前段を捨てる
    /// 「独立したビルダーの列」で、ブーリアンを含む形は表現できなかった。
    Boolean {
        op: BooleanKind,
        tool: Vec<FeatureOp>,
    },

    /// 現在の立体の1本の稜にフィレットを掛ける。
    ///
    /// 稜は ID ではなく形（中点・向き・長さ・二面角）で指す。寸法を変えて
    /// 作り直しても、同じ稜が最も近いまま残るので選び直せる。似た稜しか
    /// 見つからないときは、黙って別の稜を丸めずに失敗する。
    FilletSolidEdge {
        target: EdgeSignature,
        radius: f64,
    },

    /// 同じく面取り
    ChamferSolidEdge {
        target: EdgeSignature,
        distance: f64,
    },

    /// 金型抜き勾配ブロック
    DraftBlock {
        dx: f64,
        dy: f64,
        dz: f64,
        draft_angle_deg: f64,
    },
    /// 三角柱ガセット補強リブ
    TriangularRib {
        length: f64,
        height: f64,
        thickness: f64,
    },
    /// 正六角柱（ボルト頭）
    HexPrism {
        across_flats: f64,
        height: f64,
    },
    /// 六角ナットブランク
    HexNut {
        across_flats: f64,
        height: f64,
        hole_radius: f64,
    },
    /// 六角穴付きボルト
    SocketHeadCapScrew {
        shank_radius: f64,
        shank_length: f64,
        head_radius: f64,
        head_height: f64,
        socket_across_flats: f64,
        socket_depth: f64,
    },
    /// 平座金
    PlainWasher {
        inner_radius: f64,
        outer_radius: f64,
        thickness: f64,
    },
    /// フランジ付き六角ボルト
    FlangedHexBolt {
        shank_radius: f64,
        shank_length: f64,
        flange_radius: f64,
        flange_height: f64,
        hex_across_flats: f64,
        hex_head_height: f64,
    },
    /// 皿モミ穴ブロック
    CountersinkHole {
        box_w: f64,
        box_d: f64,
        box_h: f64,
        hole_radius: f64,
        sink_radius: f64,
        angle_deg: f64,
        center_x: f64,
        center_y: f64,
    },
    /// 座ぐり長穴ブロック
    CounterboredSlot {
        box_w: f64,
        box_d: f64,
        box_h: f64,
        slot_length: f64,
        slot_radius: f64,
        cb_length: f64,
        cb_radius: f64,
        cb_depth: f64,
        center_x: f64,
        center_y: f64,
    },
    /// スプリングワッシャー（ばね座金）
    SpringWasher {
        inner_radius: f64,
        outer_radius: f64,
        thickness: f64,
        free_height: f64,
        gap_deg: f64,
    },
    /// C形止め輪（サークリップ）
    RetainingRing {
        inner_radius: f64,
        outer_radius: f64,
        thickness: f64,
        gap_angle_deg: f64,
    },
}

/// 履歴に書けるブーリアン種別（`BooleanOpType` の直列化可能版）
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum BooleanKind {
    Union,
    Difference,
    Intersection,
}

impl From<BooleanKind> for crate::BooleanOpType {
    fn from(kind: BooleanKind) -> Self {
        match kind {
            BooleanKind::Union => crate::BooleanOpType::Union,
            BooleanKind::Difference => crate::BooleanOpType::Difference,
            BooleanKind::Intersection => crate::BooleanOpType::Intersection,
        }
    }
}

/// フィーチャーツリー内の単一ノード
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct FeatureNode {
    pub id: String,
    pub name: String,
    pub op: FeatureOp,
    pub enabled: bool,
}

/// パラメトリック・フィーチャーツリー（非破壊・TNP自己修復エンジン）
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct FeatureTree {
    pub nodes: Vec<FeatureNode>,
}

impl FeatureTree {
    pub fn new() -> Self {
        Self { nodes: Vec::new() }
    }

    /// フィーチャーの追加
    pub fn add_feature(&mut self, name: &str, op: FeatureOp) -> String {
        let id = format!("Feature_{}", self.nodes.len() + 1);
        self.nodes.push(FeatureNode {
            id: id.clone(),
            name: name.to_string(),
            op,
            enabled: true,
        });
        id
    }

    /// 指定フィーチャーのパラメータを更新
    pub fn update_feature_op(&mut self, feature_id: &str, new_op: FeatureOp) -> Result<(), String> {
        let node = self
            .nodes
            .iter_mut()
            .find(|n| n.id == feature_id)
            .ok_or_else(|| format!("Feature {} not found", feature_id))?;
        node.op = new_op;
        Ok(())
    }

    /// フィーチャーツリー全体を上流から順に再計算（TNP自己修復つき）
    pub fn recompute(&self) -> Result<Solid, String> {
        if self.nodes.is_empty() {
            return Err("Feature tree is empty".to_string());
        }

        let ops: Vec<FeatureOp> = self
            .nodes
            .iter()
            .filter(|node| node.enabled)
            .map(|node| node.op.clone())
            .collect();
        Self::evaluate(&ops)
    }

    /// 一連のフィーチャーを、何も無い状態から順に評価する。
    ///
    /// `recompute` はこれを有効なノードだけで呼ぶ。`FeatureOp::Boolean` の
    /// ツール側もこれを呼ぶので、ブーリアンの片側を同じ語彙で組み立てられる。
    pub fn evaluate(ops: &[FeatureOp]) -> Result<Solid, String> {
        let mut current_solid: Option<Solid> = None;
        let tol = Tolerance::default();

        let make_wire = |pts: &[[f64; 3]]| -> Result<Wire, String> {
            let n = pts.len();
            let vertices: Vec<Vertex> = pts
                .iter()
                .map(|p| Vertex::from_point(Point3::new(p[0], p[1], p[2])))
                .collect();
            let mut edges = Vec::with_capacity(n);
            for i in 0..n {
                let next_i = (i + 1) % n;
                let edge = Edge::line_between(vertices[i].clone(), vertices[next_i].clone())?;
                edges.push(OrientedEdge::forward(edge));
            }
            Ok(Wire::new(edges))
        };

        for op in ops {
            match op {
                FeatureOp::CreateBox { dx, dy, dz } => {
                    current_solid = Some(PrimitiveBuilder::make_box(*dx, *dy, *dz)?);
                }
                FeatureOp::CreateCylinder { radius, height } => {
                    current_solid = Some(PrimitiveBuilder::make_cylinder(*radius, *height)?);
                }
                FeatureOp::CreateCone { r1, r2, height } => {
                    current_solid = Some(PrimitiveBuilder::make_cone(*r1, *r2, *height)?);
                }
                FeatureOp::CreateTorus { major_r, minor_r } => {
                    current_solid = Some(PrimitiveBuilder::make_torus(*major_r, *minor_r)?);
                }
                FeatureOp::FilletEdge { dx, dy, dz, edge_index, radius } => {
                    current_solid = Some(DirectModeling::fillet_box_single_edge(
                        *dx,
                        *dy,
                        *dz,
                        *edge_index,
                        *radius,
                    )?);
                }
                FeatureOp::ChamferEdge { dx, dy, dz, edge_index, distance } => {
                    current_solid = Some(DirectModeling::chamfer_box_single_edge(
                        *dx,
                        *dy,
                        *dz,
                        *edge_index,
                        *distance,
                    )?);
                }
                FeatureOp::HollowBox { dx, dy, dz, thickness, open_face_index } => {
                    current_solid = Some(ShellBuilder::make_hollow_box(
                        *dx,
                        *dy,
                        *dz,
                        *thickness,
                        *open_face_index,
                    )?);
                }
                FeatureOp::HollowThroughBox { dx, dy, dz, thickness } => {
                    current_solid = Some(ShellBuilder::make_through_hollow_box(
                        *dx,
                        *dy,
                        *dz,
                        *thickness,
                    )?);
                }
                FeatureOp::ExtrudeHollow { outer_points, inner_points, dir } => {

                    let outer_wire = make_wire(outer_points)?;
                    let mut inner_wires = Vec::with_capacity(inner_points.len());
                    for hole in inner_points {
                        inner_wires.push(make_wire(hole)?);
                    }
                    let dir_vec = Vec3::new(dir[0], dir[1], dir[2]);
                    current_solid = Some(ExtrudeBuilder::extrude_face_with_holes(
                        &outer_wire,
                        &inner_wires,
                        dir_vec,
                        &tol,
                    )?);
                }
                FeatureOp::LoftSolid { sections, degree_v } => {
                    let mut section_wires = Vec::with_capacity(sections.len());
                    for sec in sections {
                        section_wires.push(make_wire(sec)?);
                    }
                    current_solid = Some(LoftBuilder::loft_solid(&section_wires, *degree_v, &tol)?);
                }
                FeatureOp::ExtrudeDraft { points, dir, draft_angle_rad } => {
                    let wire = make_wire(points)?;
                    let dir_vec = Vec3::new(dir[0], dir[1], dir[2]);
                    current_solid = Some(ExtrudeBuilder::extrude_wire_with_draft(
                        &wire,
                        dir_vec,
                        *draft_angle_rad,
                        &tol,
                    )?);
                }
                FeatureOp::RevolveSolid { profile_points, axis_origin, axis_dir } => {
                    let wire = make_wire(profile_points)?;
                    let origin = Point3::new(axis_origin[0], axis_origin[1], axis_origin[2]);
                    let dir_vec = Vec3::new(axis_dir[0], axis_dir[1], axis_dir[2]);
                    current_solid = Some(crate::RevolveBuilder::revolve_wire_solid(
                        &wire,
                        origin,
                        dir_vec,
                        &tol,
                    )?);
                }
                FeatureOp::RevolvePartialSolid { profile_points, axis_origin, axis_dir, angle_rad } => {
                    let wire = make_wire(profile_points)?;
                    let origin = Point3::new(axis_origin[0], axis_origin[1], axis_origin[2]);
                    let dir_vec = Vec3::new(axis_dir[0], axis_dir[1], axis_dir[2]);
                    current_solid = Some(crate::RevolveBuilder::revolve_wire_partial_solid(
                        &wire,
                        origin,
                        dir_vec,
                        *angle_rad,
                        &tol,
                    )?);
                }
                FeatureOp::SweepWire { profile_points, path_points, num_sections } => {


                    let profile_wire = make_wire(profile_points)?;
                    let n_path = path_points.len();
                    let degree = (n_path - 1).min(3);
                    let path_cps = path_points
                        .iter()
                        .map(|p| ControlPoint3::unweighted(Point3::new(p[0], p[1], p[2])))
                        .collect();
                    let knots = KnotVector::clamped_uniform(n_path, degree);
                    let path = NurbsCurve3::new(degree, path_cps, knots)?;
                    current_solid = Some(SweepBuilder::sweep_wire_along_curve(
                        &profile_wire,
                        &path,
                        *num_sections,
                        &tol,
                    )?);
                }
                FeatureOp::SweepHelix {
                    profile_points,
                    radius,
                    pitch,
                    turns,
                    axis_origin,
                    axis_dir,
                    num_sections,
                } => {
                    let profile_wire = make_wire(profile_points)?;
                    let origin = Point3::new(axis_origin[0], axis_origin[1], axis_origin[2]);
                    let dir_vec = Vec3::new(axis_dir[0], axis_dir[1], axis_dir[2]);
                    current_solid = Some(crate::HelixBuilder::sweep_wire_along_helix(
                        &profile_wire,
                        *radius,
                        *pitch,
                        *turns,
                        origin,
                        dir_vec,
                        *num_sections,
                        &tol,
                    )?);
                }
                FeatureOp::PolylinePipe { path_points, pipe_radius, corner_radius } => {

                    let pts: Vec<_> = path_points.iter().map(|p| Point3::new(p[0], p[1], p[2])).collect();
                    current_solid = Some(crate::PolylineBuilder::sweep_pipe_polyline(
                        &pts,
                        *pipe_radius,
                        *corner_radius,
                        &tol,
                    )?);
                }
                FeatureOp::MirrorSolid {
                    plane_origin,
                    plane_normal,
                } => {
                    let solid = current_solid.ok_or("No base solid for mirror")?;
                    let orig = Point3::new(plane_origin[0], plane_origin[1], plane_origin[2]);
                    let norm = Vec3::new(plane_normal[0], plane_normal[1], plane_normal[2]);
                    current_solid = Some(crate::MirrorBuilder::mirror_solid(
                        &solid,
                        orig,
                        norm,
                        &tol,
                    )?);
                }
                FeatureOp::PushPullFace {

                    target_signature,
                    distance,
                } => {
                    let solid = current_solid.ok_or("No base solid for push-pull")?;
                    let (matched_face_idx, score) = GeometricMatcher::find_best_matching_face(
                        target_signature,
                        &solid.outer_shell.faces,
                    ).ok_or("Failed to match target face for PushPull (topology changed too drastically)")?;

                    if score < 0.6 {
                        return Err(format!(
                            "Low confidence ({:.2}) matching target face",
                            score
                        ));
                    }

                    current_solid = Some(DirectModeling::push_pull_face(
                        &solid,
                        matched_face_idx,
                        *distance,
                    )?);
                }
                FeatureOp::ThickenFace {
                    target_signature,
                    thickness,
                } => {
                    let solid = current_solid.ok_or("No base solid for thicken")?;
                    let (matched_face_idx, _) = GeometricMatcher::find_best_matching_face(
                        target_signature,
                        &solid.outer_shell.faces,
                    )
                    .ok_or("Failed to match target face for Thicken")?;

                    let target_face = &solid.outer_shell.faces[matched_face_idx];
                    current_solid =
                        Some(ThickenBuilder::thicken_face(target_face, *thickness, &tol)?);
                }
                FeatureOp::Translate { offset } => {
                    let solid = current_solid.ok_or("No base solid to translate")?;
                    current_solid = Some(crate::BrepTransform::translate_solid(
                        &solid,
                        Vec3::new(offset[0], offset[1], offset[2]),
                    ));
                }
                FeatureOp::Rotate {
                    axis_origin,
                    axis_dir,
                    angle_deg,
                } => {
                    let solid = current_solid.ok_or("No base solid to rotate")?;
                    let axis = Vec3::new(axis_dir[0], axis_dir[1], axis_dir[2]);
                    if axis.norm() <= 1e-12 {
                        return Err("The rotation axis has no direction".to_string());
                    }
                    let origin = Vec3::new(axis_origin[0], axis_origin[1], axis_origin[2]);
                    let transform = zenith_math::Transform3::from_translation(origin)
                        .compose(&zenith_math::Transform3::from_axis_angle(
                            &axis,
                            angle_deg.to_radians(),
                        ))
                        .compose(&zenith_math::Transform3::from_translation(-origin));
                    current_solid = Some(crate::BrepTransform::transform_solid(
                        &solid, &transform,
                    )?);
                }
                FeatureOp::Boolean { op, tool } => {
                    let solid = current_solid.ok_or("No base solid for a boolean")?;
                    let tool_solid = Self::evaluate(tool)
                        .map_err(|err| format!("building the boolean tool: {err}"))?;
                    current_solid = Some(crate::BooleanEngine::boolean_solids_exact(
                        &solid,
                        &tool_solid,
                        (*op).into(),
                        &tol,
                    )?);
                }
                FeatureOp::FilletSolidEdge { target, radius } => {
                    let solid = current_solid.ok_or("No base solid to fillet")?;
                    let edge_id = match_edge(&solid, target)?;
                    current_solid =
                        Some(crate::EdgeBlender::fillet_edge(&solid, edge_id, *radius)?);
                }
                FeatureOp::ChamferSolidEdge { target, distance } => {
                    let solid = current_solid.ok_or("No base solid to chamfer")?;
                    let edge_id = match_edge(&solid, target)?;
                    current_solid =
                        Some(crate::EdgeBlender::chamfer_edge(&solid, edge_id, *distance)?);
                }
                FeatureOp::DraftBlock { dx, dy, dz, draft_angle_deg } => {
                    current_solid = Some(crate::DraftBuilder::make_drafted_block(
                        *dx,
                        *dy,
                        *dz,
                        draft_angle_deg.to_radians(),
                        &tol,
                    )?);
                }
                FeatureOp::TriangularRib { length, height, thickness } => {
                    current_solid = Some(crate::RibBuilder::make_triangular_rib(*length, *height, *thickness, &tol)?);
                }
                FeatureOp::HexPrism { across_flats, height } => {
                    current_solid = Some(crate::FastenerBuilder::make_hex_prism(*across_flats, *height, &tol)?);
                }
                FeatureOp::HexNut { across_flats, height, hole_radius } => {
                    current_solid = Some(crate::FastenerBuilder::make_hex_nut_blank(*across_flats, *height, *hole_radius, &tol)?);
                }
                FeatureOp::SocketHeadCapScrew {
                    shank_radius,
                    shank_length,
                    head_radius,
                    head_height,
                    socket_across_flats,
                    socket_depth,
                } => {
                    current_solid = Some(crate::FastenerBuilder::make_socket_head_cap_screw(
                        *shank_radius,
                        *shank_length,
                        *head_radius,
                        *head_height,
                        *socket_across_flats,
                        *socket_depth,
                        &tol,
                    )?);
                }
                FeatureOp::PlainWasher { inner_radius, outer_radius, thickness } => {
                    current_solid = Some(crate::FastenerBuilder::make_plain_washer(*inner_radius, *outer_radius, *thickness, &tol)?);
                }
                FeatureOp::FlangedHexBolt {
                    shank_radius,
                    shank_length,
                    flange_radius,
                    flange_height,
                    hex_across_flats,
                    hex_head_height,
                } => {
                    current_solid = Some(crate::FastenerBuilder::make_flanged_hex_bolt(
                        *shank_radius,
                        *shank_length,
                        *flange_radius,
                        *flange_height,
                        *hex_across_flats,
                        *hex_head_height,
                        &tol,
                    )?);
                }
                FeatureOp::CountersinkHole {
                    box_w,
                    box_d,
                    box_h,
                    hole_radius,
                    sink_radius,
                    angle_deg,
                    center_x,
                    center_y,
                } => {
                    current_solid = Some(crate::HoleBuilder::make_countersink_hole_box(
                        *box_w,
                        *box_d,
                        *box_h,
                        *hole_radius,
                        *sink_radius,
                        *angle_deg,
                        *center_x,
                        *center_y,
                    )?);
                }
                FeatureOp::CounterboredSlot {
                    box_w,
                    box_d,
                    box_h,
                    slot_length,
                    slot_radius,
                    cb_length,
                    cb_radius,
                    cb_depth,
                    center_x,
                    center_y,
                } => {
                    current_solid = Some(crate::HoleBuilder::make_counterbored_slot_box(
                        *box_w,
                        *box_d,
                        *box_h,
                        *slot_length,
                        *slot_radius,
                        *cb_length,
                        *cb_radius,
                        *cb_depth,
                        *center_x,
                        *center_y,
                    )?);
                }
                FeatureOp::SpringWasher {
                    inner_radius,
                    outer_radius,
                    thickness,
                    free_height,
                    gap_deg,
                } => {
                    current_solid = Some(crate::FastenerBuilder::make_spring_washer(
                        *inner_radius,
                        *outer_radius,
                        *thickness,
                        *free_height,
                        *gap_deg,
                        &tol,
                    )?);
                }
                FeatureOp::RetainingRing {
                    inner_radius,
                    outer_radius,
                    thickness,
                    gap_angle_deg,
                } => {
                    current_solid = Some(crate::FastenerBuilder::make_retaining_ring(
                        *inner_radius,
                        *outer_radius,
                        *thickness,
                        *gap_angle_deg,
                        &tol,
                    )?);
                }
            }
        }

        current_solid.ok_or("No valid solid produced by feature tree".to_string())
    }
}

impl Default for FeatureTree {
    fn default() -> Self {
        Self::new()
    }
}

/// 履歴に書いてある稜のシグネチャに、今の立体のどの稜が当たるかを探す。
///
/// 一致度が足りないとき、および2位との差が小さくてどちらとも言えないときは、
/// 近い方を黙って丸めずに失敗する。間違った稜を丸めた結果は「閉じた別の形」
/// になり、後から見て気付けないため。
pub fn match_edge(solid: &Solid, target: &EdgeSignature) -> Result<u64, String> {
    let mut scored: Vec<(f64, u64)> = Vec::new();
    let mut seen: Vec<u64> = Vec::new();

    for face in &solid.outer_shell.faces {
        for wire in std::iter::once(&face.outer_wire).chain(face.inner_wires.iter()) {
            for oriented in &wire.edges {
                let id = oriented.edge.id;
                if seen.contains(&id) {
                    continue;
                }
                seen.push(id);

                let dihedral = DirectModeling::inspect_solid_edge(solid, id)
                    .ok()
                    .and_then(|inspection| inspection.dihedral_angle_deg);
                let signature = EdgeSignature::from_edge(&oriented.edge, dihedral);
                scored.push((target.similarity(&signature), id));
            }
        }
    }

    scored.sort_by(|a, b| b.0.total_cmp(&a.0));
    let (best_score, best_id) = *scored
        .first()
        .ok_or("The solid has no edges to match against")?;

    if best_score < 0.9 {
        return Err(format!(
            "No edge matches the one this feature was made on (best score {best_score:.3})"
        ));
    }
    if let Some((runner_up, _)) = scored.get(1) {
        if best_score - runner_up < 1e-6 {
            return Err(format!(
                "Two edges match this feature equally well (both {best_score:.3}); the selection is ambiguous"
            ));
        }
    }

    Ok(best_id)
}

/// 立体の中の1本の稜から、履歴に書けるシグネチャを作る
pub fn edge_signature(solid: &Solid, edge_id: u64) -> Result<EdgeSignature, String> {
    for face in &solid.outer_shell.faces {
        for wire in std::iter::once(&face.outer_wire).chain(face.inner_wires.iter()) {
            for oriented in &wire.edges {
                if oriented.edge.id == edge_id {
                    let dihedral = DirectModeling::inspect_solid_edge(solid, edge_id)
                        .ok()
                        .and_then(|inspection| inspection.dihedral_angle_deg);
                    return Ok(EdgeSignature::from_edge(&oriented.edge, dihedral));
                }
            }
        }
    }
    Err(format!("Edge {edge_id} is not in this solid"))
}
