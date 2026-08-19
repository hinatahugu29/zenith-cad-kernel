use crate::{
    DirectModeling, ExtrudeBuilder, LoftBuilder, PrimitiveBuilder, ShellBuilder, SweepBuilder,
    ThickenBuilder,
};
use serde::{Deserialize, Serialize};
use zenith_geom::{ControlPoint3, KnotVector, NurbsCurve3};
use zenith_math::{Point3, Tolerance, Vec3};
use zenith_topo::{Edge, GeometricMatcher, GeometricSignature, OrientedEdge, Solid, Vertex, Wire};

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

        for node in &self.nodes {
            if !node.enabled {
                continue;
            }

            match &node.op {
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
