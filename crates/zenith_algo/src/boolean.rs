use zenith_math::{Point3, RobustPredicates, Tolerance, Vec3};
use zenith_tess::{tessellate_solid, TessellationParams, TriangleMesh};
use zenith_topo::Solid;

/// ブーリアン演算の種類
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum BooleanOpType {
    Union,        // 結合 (A + B)
    Difference,   // 差分 (A - B)
    Intersection, // 共通部分 (A * B)
}

/// B-Rep / ポリゴンブーリアン演算エンジン
pub struct BooleanEngine;

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct ExactBooleanPreparationReport {
    pub face_pair_candidate_count: usize,
    pub intersection_edge_candidate_count: usize,
    pub planar_split_candidate_count: usize,
    pub planar_batch_split_face_count: usize,
    pub planar_batch_applied_split_count: usize,
    pub classified_split_candidate_count: usize,
    pub selected_face_piece_count: usize,
    pub planar_cap_loop_count: usize,
    pub planar_cap_face_count: usize,
    pub selected_with_caps_face_piece_count: usize,
    pub selected_with_caps_unmatched_edge_use_count: usize,
    pub selected_with_caps_non_manifold_edge_use_count: usize,
    pub selected_with_caps_same_direction_edge_use_count: usize,
    pub selected_face_unmatched_edge_use_count: usize,
    pub selected_face_non_manifold_edge_use_count: usize,
    pub selected_face_same_direction_edge_use_count: usize,
}

impl BooleanEngine {
    /// 2つの Solid に対する表示用メッシュブーリアン。
    ///
    /// これは正確なB-Repを返さない。編集・STEP出力・feature履歴には
    /// `boolean_solids_exact()` の実装を使う必要がある。
    pub fn boolean_solids_mesh_preview(
        solid_a: &Solid,
        solid_b: &Solid,
        op: BooleanOpType,
        tess_params: &TessellationParams,
        _tol: &Tolerance,
    ) -> Result<TriangleMesh, String> {
        let mesh_a = tessellate_solid(solid_a, tess_params);
        let mesh_b = tessellate_solid(solid_b, tess_params);

        Self::boolean_meshes(&mesh_a, &mesh_b, op)
    }

    /// 2つの Solid に対する正確なB-Repブーリアン入口。
    pub fn boolean_solids_exact(
        solid_a: &Solid,
        solid_b: &Solid,
        op: BooleanOpType,
        tol: &Tolerance,
    ) -> Result<Solid, String> {
        if std::ptr::eq(solid_a, solid_b)
            && matches!(op, BooleanOpType::Union | BooleanOpType::Intersection)
        {
            if !solid_a.is_topologically_valid(tol) {
                return Err("Exact B-Rep boolean input A is not topologically valid".to_string());
            }
            return Ok(solid_a.clone());
        }

        let report = Self::prepare_exact_boolean(solid_a, solid_b, op, tol)?;
        let selection = crate::BrepIntersectionBuilder::collect_selected_boolean_face_pieces(
            solid_a, solid_b, op, tol,
        );
        if selection.stitch_report.is_closed_manifold() {
            return crate::BrepIntersectionBuilder::build_solid_from_selected_face_pieces(
                &selection.selected_face_pieces,
                tol,
            );
        }

        Err(format!(
            "Exact B-Rep boolean is not implemented yet; preparation reached {} face-pair candidates, {} intersection edges, {} planar split candidates, {} batch-split faces, {} applied batch splits, {} classified split candidates, {} selected face pieces, {} cap loops, and {} cap faces; selected face stitching has {} unmatched edge uses, {} non-manifold edge uses, and {} same-direction edge uses; with caps it has {} face pieces, {} unmatched edge uses, {} non-manifold edge uses, and {} same-direction edge uses. Use boolean_solids_mesh_preview only for display/preview mesh results",
            report.face_pair_candidate_count,
            report.intersection_edge_candidate_count,
            report.planar_split_candidate_count,
            report.planar_batch_split_face_count,
            report.planar_batch_applied_split_count,
            report.classified_split_candidate_count,
            report.selected_face_piece_count,
            report.planar_cap_loop_count,
            report.planar_cap_face_count,
            report.selected_face_unmatched_edge_use_count,
            report.selected_face_non_manifold_edge_use_count,
            report.selected_face_same_direction_edge_use_count,
            report.selected_with_caps_face_piece_count,
            report.selected_with_caps_unmatched_edge_use_count,
            report.selected_with_caps_non_manifold_edge_use_count,
            report.selected_with_caps_same_direction_edge_use_count
        ))
    }

    pub fn prepare_exact_boolean(
        solid_a: &Solid,
        solid_b: &Solid,
        op: BooleanOpType,
        tol: &Tolerance,
    ) -> Result<ExactBooleanPreparationReport, String> {
        if !solid_a.is_topologically_valid(tol) {
            return Err("Exact B-Rep boolean input A is not topologically valid".to_string());
        }
        if !solid_b.is_topologically_valid(tol) {
            return Err("Exact B-Rep boolean input B is not topologically valid".to_string());
        }

        let face_pair_candidate_count =
            crate::BrepIntersectionBuilder::collect_face_pair_candidates(
                &solid_a.outer_shell.faces,
                &solid_b.outer_shell.faces,
                tol,
            )
            .len();
        if face_pair_candidate_count == 0 {
            return Err(
                "Exact B-Rep boolean found no face-pair intersection candidates".to_string(),
            );
        }

        let intersection_edge_candidate_count =
            crate::BrepIntersectionBuilder::collect_intersection_edge_candidates(
                &solid_a.outer_shell.faces,
                &solid_b.outer_shell.faces,
                tol,
            )
            .len();
        let planar_split_candidate_count =
            crate::BrepIntersectionBuilder::collect_planar_face_split_candidates(
                &solid_a.outer_shell.faces,
                &solid_b.outer_shell.faces,
                tol,
            )
            .len();
        let classified_splits =
            crate::BrepIntersectionBuilder::collect_classified_planar_face_split_candidates(
                solid_a, solid_b, tol,
            );
        let shell_assembly = crate::BrepIntersectionBuilder::collect_boolean_shell_assembly(
            solid_a, solid_b, op, tol,
        );
        let planar_batch_split_face_count = shell_assembly.selection.batch_splits.splits_a.len()
            + shell_assembly.selection.batch_splits.splits_b.len();
        let planar_batch_applied_split_count = shell_assembly
            .selection
            .batch_splits
            .splits_a
            .iter()
            .chain(shell_assembly.selection.batch_splits.splits_b.iter())
            .map(|split| split.result.applied_split_count)
            .sum();

        Ok(ExactBooleanPreparationReport {
            face_pair_candidate_count,
            intersection_edge_candidate_count,
            planar_split_candidate_count,
            planar_batch_split_face_count,
            planar_batch_applied_split_count,
            classified_split_candidate_count: classified_splits.len(),
            selected_face_piece_count: shell_assembly.selection.selected_face_pieces.len(),
            planar_cap_loop_count: shell_assembly
                .cap_generation
                .edge_loop_extraction
                .loops
                .len(),
            planar_cap_face_count: shell_assembly.cap_generation.cap_faces.len(),
            selected_with_caps_face_piece_count: shell_assembly.assembly.selected_face_pieces.len(),
            selected_with_caps_unmatched_edge_use_count: shell_assembly
                .assembly
                .stitch_report
                .unmatched_edge_use_count,
            selected_with_caps_non_manifold_edge_use_count: shell_assembly
                .assembly
                .stitch_report
                .non_manifold_edge_use_count,
            selected_with_caps_same_direction_edge_use_count: shell_assembly
                .assembly
                .stitch_report
                .same_direction_edge_use_count,
            selected_face_unmatched_edge_use_count: shell_assembly
                .selection
                .stitch_report
                .unmatched_edge_use_count,
            selected_face_non_manifold_edge_use_count: shell_assembly
                .selection
                .stitch_report
                .non_manifold_edge_use_count,
            selected_face_same_direction_edge_use_count: shell_assembly
                .selection
                .stitch_report
                .same_direction_edge_use_count,
        })
    }

    /// 2つの Solid に対する互換API。現在は表示用メッシュブーリアンを返す。
    pub fn boolean_solids(
        solid_a: &Solid,
        solid_b: &Solid,
        op: BooleanOpType,
        tess_params: &TessellationParams,
        tol: &Tolerance,
    ) -> Result<TriangleMesh, String> {
        Self::boolean_solids_mesh_preview(solid_a, solid_b, op, tess_params, tol)
    }

    /// 2つの閉じたTriangleMeshに対するブーリアン演算
    pub fn boolean_meshes(
        mesh_a: &TriangleMesh,
        mesh_b: &TriangleMesh,
        op: BooleanOpType,
    ) -> Result<TriangleMesh, String> {
        let mut result = TriangleMesh::new();

        // 1. 各メッシュの三角形の中心点をサンプリングして内外判定
        // メッシュAの各三角形がメッシュBの内部にあるかを判定
        let mut a_tri_inside_b = Vec::with_capacity(mesh_a.num_triangles());
        for tri in &mesh_a.indices {
            let p0 = mesh_a.positions[tri[0] as usize];
            let p1 = mesh_a.positions[tri[1] as usize];
            let p2 = mesh_a.positions[tri[2] as usize];
            let centroid = Point3::from((p0.coords + p1.coords + p2.coords) * (1.0 / 3.0));
            a_tri_inside_b.push(Self::is_point_inside_mesh(centroid, mesh_b));
        }

        // メッシュBの各三角形がメッシュAの内部にあるかを判定
        let mut b_tri_inside_a = Vec::with_capacity(mesh_b.num_triangles());
        for tri in &mesh_b.indices {
            let p0 = mesh_b.positions[tri[0] as usize];
            let p1 = mesh_b.positions[tri[1] as usize];
            let p2 = mesh_b.positions[tri[2] as usize];
            let centroid = Point3::from((p0.coords + p1.coords + p2.coords) * (1.0 / 3.0));
            b_tri_inside_a.push(Self::is_point_inside_mesh(centroid, mesh_a));
        }

        // 2. ブーリアン演算タイプに応じた三角形の収集
        match op {
            BooleanOpType::Union => {
                // A の外側三角形 + B の外側三角形
                Self::append_selected_triangles(&mut result, mesh_a, &a_tri_inside_b, false, false);
                Self::append_selected_triangles(&mut result, mesh_b, &b_tri_inside_a, false, false);
            }
            BooleanOpType::Difference => {
                // A の外側三角形 + B の内側三角形（法線反転）
                Self::append_selected_triangles(&mut result, mesh_a, &a_tri_inside_b, false, false);
                Self::append_selected_triangles(&mut result, mesh_b, &b_tri_inside_a, true, true);
            }
            BooleanOpType::Intersection => {
                // A の内側三角形 + B の内側三角形
                Self::append_selected_triangles(&mut result, mesh_a, &a_tri_inside_b, true, false);
                Self::append_selected_triangles(&mut result, mesh_b, &b_tri_inside_a, true, false);
            }
        }

        Ok(result)
    }

    /// 点が閉じたメッシュの内部にあるかをRay-Casting法（交差回数の奇偶）で判定
    pub fn is_point_inside_mesh(p: Point3, mesh: &TriangleMesh) -> bool {
        // 任意方向の半直線（X軸正方向など）
        let ray_dir = Vec3::new(1.0, 0.000137, 0.000289).normalize();
        let mut hit_count = 0;

        for tri in &mesh.indices {
            let a = mesh.positions[tri[0] as usize];
            let b = mesh.positions[tri[1] as usize];
            let c = mesh.positions[tri[2] as usize];

            if RobustPredicates::ray_triangle_intersect(p, ray_dir, a, b, c).is_some() {
                hit_count += 1;
            }
        }

        hit_count % 2 == 1 // 奇数回交差なら内部
    }

    /// 選択された三角形群を結果メッシュに追加
    fn append_selected_triangles(
        target: &mut TriangleMesh,
        source: &TriangleMesh,
        inside_flags: &[bool],
        select_inside: bool,
        flip_normals: bool,
    ) {
        let base_idx = target.positions.len() as u32;

        target.positions.extend_from_slice(&source.positions);
        if flip_normals {
            for n in &source.normals {
                target.normals.push(-*n);
            }
        } else {
            target.normals.extend_from_slice(&source.normals);
        }
        target.uvs.extend_from_slice(&source.uvs);

        for (idx, tri) in source.indices.iter().enumerate() {
            if inside_flags[idx] == select_inside {
                let i0 = tri[0] + base_idx;
                let i1 = tri[1] + base_idx;
                let i2 = tri[2] + base_idx;

                if flip_normals {
                    target.indices.push([i0, i2, i1]); // ワインディング反転
                } else {
                    target.indices.push([i0, i1, i2]);
                }
            }
        }
    }
}
