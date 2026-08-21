use zenith_math::{Point3, RobustPredicates, Tolerance, Vec3};
use zenith_tess::{tessellate_solid, TessellationParams, TriangleMesh};
use zenith_topo::{Shape, Solid};

/// ブーリアン演算の種類
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum BooleanOpType {
    Union,        // 結合 (A + B)
    Difference,   // 差分 (A - B)
    Intersection, // 共通部分 (A * B)
}

/// B-Rep / ポリゴンブーリアン演算エンジン
pub struct BooleanEngine;

#[derive(Debug, Clone, PartialEq)]
pub struct ExactBooleanResult {
    pub solids: Vec<Solid>,
}

impl ExactBooleanResult {
    pub fn from_solids(solids: Vec<Solid>) -> Self {
        Self { solids }
    }

    pub fn single(solid: Solid) -> Self {
        Self {
            solids: vec![solid],
        }
    }

    pub fn len(&self) -> usize {
        self.solids.len()
    }

    pub fn is_empty(&self) -> bool {
        self.solids.is_empty()
    }

    pub fn try_single(self) -> Result<Solid, String> {
        match self.solids.len() {
            1 => Ok(self.solids.into_iter().next().unwrap()),
            0 => Err("Exact B-Rep boolean produced an empty result".to_string()),
            count => Err(format!(
                "Exact B-Rep boolean produced {count} disjoint solids; use boolean_solids_exact_result for multi-solid results"
            )),
        }
    }

    pub fn tessellate(&self, params: &TessellationParams) -> TriangleMesh {
        let mut mesh = TriangleMesh::new();
        for solid in &self.solids {
            mesh.merge(&tessellate_solid(solid, params));
        }
        mesh
    }

    pub fn to_shape(&self) -> Shape {
        Shape::compound_solids(self.solids.clone())
    }

    pub fn into_shape(self) -> Shape {
        Shape::compound_solids(self.solids)
    }
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct ExactBooleanPreparationReport {
    pub face_pair_candidate_count: usize,
    pub intersection_edge_candidate_count: usize,
    pub planar_split_candidate_count: usize,
    pub planar_batch_split_face_count: usize,
    pub planar_batch_applied_split_count: usize,
    pub planar_batch_skipped_split_count: usize,
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
        Self::boolean_solids_exact_result(solid_a, solid_b, op, tol)?.try_single()
    }

    /// Computes an exact B-Rep boolean and verifies the answer before handing
    /// it back.
    ///
    /// A closed manifold shell is not proof of a correct boolean - returning
    /// one operand untouched satisfies it - so the result is additionally
    /// checked against the volume bounds and the point membership implied by
    /// the operation. A result that fails becomes an error rather than a
    /// plausible-looking wrong solid. Use
    /// [`Self::boolean_solids_exact_result_unverified`] to skip the gate.
    pub fn boolean_solids_exact_result(
        solid_a: &Solid,
        solid_b: &Solid,
        op: BooleanOpType,
        tol: &Tolerance,
    ) -> Result<ExactBooleanResult, String> {
        let result = Self::boolean_solids_exact_result_unverified(solid_a, solid_b, op, tol)?;

        let report = crate::BooleanResultVerifier::verify(solid_a, solid_b, &result.solids, op, tol);
        if !report.is_valid() {
            return Err(format!(
                "Exact B-Rep boolean produced a result that fails verification: {}",
                report.summary()
            ));
        }

        Ok(result)
    }

    /// Whether the two solids' bounding boxes share more than a face.
    ///
    /// Boxes that only touch enclose no volume between them, so nothing can be
    /// in both solids. This is a sound test in one direction only: boxes that
    /// do overlap say nothing about whether the solids do, and those go on to
    /// the ordinary path.
    fn bounds_overlap_in_volume(solid_a: &Solid, solid_b: &Solid, tol: &Tolerance) -> bool {
        let params = zenith_tess::TessellationParams::default();
        let bounds = |solid: &Solid| {
            let mesh = zenith_tess::tessellate_solid(solid, &params);
            let mut low = [f64::INFINITY; 3];
            let mut high = [f64::NEG_INFINITY; 3];
            for point in &mesh.positions {
                for (axis, value) in [point.x, point.y, point.z].into_iter().enumerate() {
                    low[axis] = low[axis].min(value);
                    high[axis] = high[axis].max(value);
                }
            }
            (low, high)
        };

        let (low_a, high_a) = bounds(solid_a);
        let (low_b, high_b) = bounds(solid_b);
        (0..3).all(|axis| {
            low_a[axis].is_finite()
                && low_b[axis].is_finite()
                && high_a[axis].min(high_b[axis]) - low_a[axis].max(low_b[axis]) > tol.linear
        })
    }

    /// The raw boolean pipeline, without the correctness gate.
    ///
    /// Intended for diagnosing the pipeline itself; callers that need a
    /// trustworthy solid should use [`Self::boolean_solids_exact_result`].
    pub fn boolean_solids_exact_result_unverified(
        solid_a: &Solid,
        solid_b: &Solid,
        op: BooleanOpType,
        tol: &Tolerance,
    ) -> Result<ExactBooleanResult, String> {
        if std::ptr::eq(solid_a, solid_b)
            && matches!(op, BooleanOpType::Union | BooleanOpType::Intersection)
        {
            if !solid_a.is_topologically_valid(tol) {
                return Err("Exact B-Rep boolean input A is not topologically valid".to_string());
            }
            return Ok(ExactBooleanResult::single(solid_a.clone()));
        }
        if !solid_a.is_topologically_valid(tol) {
            return Err("Exact B-Rep boolean input A is not topologically valid".to_string());
        }
        if !solid_b.is_topologically_valid(tol) {
            return Err("Exact B-Rep boolean input B is not topologically valid".to_string());
        }

        // 空洞（内側シェル）を持つ立体は、まだブーリアンの**入力**にできません。
        //
        // 下のどの経路も `outer_shell` しか見ないので、通すと**空洞が黙って
        // 消えます**。40^3 から 10^3 を抜いた立体（体積 63000）に箱を足すと、
        // 空洞ぶんの 1000 が戻ってきて 92000 になり、差でも同じだけ増えます。
        //
        // **しかも384点のゲートを通ります。** 空洞は境界箱の 0.6% しかなく、
        // 許容している食い違い 1% に収まるからです。誤答として出るので、
        // 呼び出し側はそれと気づけません。断るほうが安全です（4-23）。
        //
        // 作るほうは正しく動きます。`A - B` で B が A の内側に完全に入って
        // いれば、空洞を持つ立体が返ります。**作れるが消費できない**、が
        // いまの状態です。
        if !solid_a.inner_shells.is_empty() || !solid_b.inner_shells.is_empty() {
            return Err(
                "Exact B-Rep boolean does not carry cavities through yet: an operand has an inner shell"
                    .to_string(),
            );
        }

        // 境界箱が体積を持って重ならないなら、積は空だと確かめられる。
        // 面が触れているだけの配置はここに落ちる: 交線の候補はあるので
        // 下の経路に入ってしまい、「未実装」と報告されていた。
        if op == BooleanOpType::Intersection && !Self::bounds_overlap_in_volume(solid_a, solid_b, tol)
        {
            return Ok(ExactBooleanResult::from_solids(Vec::new()));
        }

        if !Self::has_face_pair_candidates(solid_a, solid_b, tol) {
            return Self::boolean_solids_exact_without_intersections(solid_a, solid_b, op, tol);
        }
        if let Some(result) =
            crate::cylinder_boolean::CylinderBoolean::boolean_axis_cylinder_and_slab_exact_result(
                solid_a, solid_b, op, tol,
            )?
        {
            return Ok(result);
        }
        if let Some(result) =
            crate::orthogonal_boolean::OrthogonalBoxBoolean::boolean_axis_aligned_boxes_exact(
                solid_a, solid_b, op, tol,
            )?
        {
            return Ok(result);
        }

        // 面で接しているだけで中身が重なっていない場合、差は A そのもの。
        // 一般経路に流すと、B の同一平面が Boundary として採られて A の面と
        // 重複し、非多様体になる。
        if matches!(op, BooleanOpType::Difference) && !Self::interiors_overlap(solid_a, solid_b) {
            return Ok(ExactBooleanResult::single(solid_a.clone()));
        }

        let shell_assembly = crate::BrepIntersectionBuilder::collect_boolean_shell_assembly(
            solid_a, solid_b, op, tol,
        );
        if shell_assembly.selection.stitch_report.is_closed_manifold() {
            return crate::BrepIntersectionBuilder::build_solids_from_selected_face_pieces(
                &shell_assembly.selection.selected_face_pieces,
                tol,
            )
            .map(ExactBooleanResult::from_solids);
        }
        if shell_assembly.assembly.stitch_report.is_closed_manifold() {
            return crate::BrepIntersectionBuilder::build_solids_from_selected_face_pieces(
                &shell_assembly.assembly.selected_face_pieces,
                tol,
            )
            .map(ExactBooleanResult::from_solids);
        }

        let report =
            Self::preparation_report_from_shell_assembly(solid_a, solid_b, &shell_assembly, tol)?;
        Err(format!(
            "Exact B-Rep boolean is not implemented yet; preparation reached {} face-pair candidates, {} intersection edges, {} planar split candidates, {} batch-split faces, {} applied batch splits, {} skipped batch splits, {} classified split candidates, {} selected face pieces, {} cap loops, and {} cap faces; selected face stitching has {} unmatched edge uses, {} non-manifold edge uses, and {} same-direction edge uses; with caps it has {} face pieces, {} unmatched edge uses, {} non-manifold edge uses, and {} same-direction edge uses. Use boolean_solids_mesh_preview only for display/preview mesh results",
            report.face_pair_candidate_count,
            report.intersection_edge_candidate_count,
            report.planar_split_candidate_count,
            report.planar_batch_split_face_count,
            report.planar_batch_applied_split_count,
            report.planar_batch_skipped_split_count,
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

    /// True when the two solids share interior volume, as opposed to merely
    /// touching along a face or an edge.
    ///
    /// Touching solids still produce face-pair candidates, so the presence of
    /// candidates says nothing about whether there is anything to cut away.
    fn interiors_overlap(solid_a: &Solid, solid_b: &Solid) -> bool {
        let params = TessellationParams {
            u_divisions: 12,
            v_divisions: 12,
        };
        let mesh_a = tessellate_solid(solid_a, &params);
        let mesh_b = tessellate_solid(solid_b, &params);
        if mesh_a.positions.is_empty() || mesh_b.positions.is_empty() {
            return false;
        }

        let bounds = |mesh: &TriangleMesh| {
            let mut min_pt = Point3::new(f64::INFINITY, f64::INFINITY, f64::INFINITY);
            let mut max_pt = Point3::new(f64::NEG_INFINITY, f64::NEG_INFINITY, f64::NEG_INFINITY);
            for point in &mesh.positions {
                min_pt.x = min_pt.x.min(point.x);
                min_pt.y = min_pt.y.min(point.y);
                min_pt.z = min_pt.z.min(point.z);
                max_pt.x = max_pt.x.max(point.x);
                max_pt.y = max_pt.y.max(point.y);
                max_pt.z = max_pt.z.max(point.z);
            }
            (min_pt, max_pt)
        };

        // 共通のバウンディングボックス内だけを見れば十分。
        let (min_a, max_a) = bounds(&mesh_a);
        let (min_b, max_b) = bounds(&mesh_b);
        let min_pt = Point3::new(
            min_a.x.max(min_b.x),
            min_a.y.max(min_b.y),
            min_a.z.max(min_b.z),
        );
        let max_pt = Point3::new(
            max_a.x.min(max_b.x),
            max_a.y.min(max_b.y),
            max_a.z.min(max_b.z),
        );
        if min_pt.x >= max_pt.x || min_pt.y >= max_pt.y || min_pt.z >= max_pt.z {
            return false;
        }

        let span = Vec3::new(
            max_pt.x - min_pt.x,
            max_pt.y - min_pt.y,
            max_pt.z - min_pt.z,
        );

        const SAMPLES: usize = 512;
        for index in 1..=SAMPLES {
            let point = Point3::new(
                min_pt.x + span.x * halton(index, 2),
                min_pt.y + span.y * halton(index, 3),
                min_pt.z + span.z * halton(index, 5),
            );
            if Self::is_point_inside_mesh(point, &mesh_a)
                && Self::is_point_inside_mesh(point, &mesh_b)
            {
                return true;
            }
        }

        false
    }

    fn has_face_pair_candidates(solid_a: &Solid, solid_b: &Solid, tol: &Tolerance) -> bool {
        // 「あるかないか」だけの問いに交線の走査は要らない。
        crate::BrepIntersectionBuilder::any_face_pair_may_intersect(
            &solid_a.outer_shell.faces,
            &solid_b.outer_shell.faces,
            tol,
        )
    }

    fn boolean_solids_exact_without_intersections(
        solid_a: &Solid,
        solid_b: &Solid,
        op: BooleanOpType,
        tol: &Tolerance,
    ) -> Result<ExactBooleanResult, String> {
        let a_inside_b = Self::solid_is_inside_or_on_boundary(solid_a, solid_b, tol);
        let b_inside_a = Self::solid_is_inside_or_on_boundary(solid_b, solid_a, tol);

        match op {
            BooleanOpType::Union => {
                if a_inside_b {
                    Ok(ExactBooleanResult::single(solid_b.clone()))
                } else if b_inside_a {
                    Ok(ExactBooleanResult::single(solid_a.clone()))
                } else {
                    // 交わらない2立体の和は、1つの立体にはならないが、
                    // 2つの立体からなる結果としては正しく表せる。
                    Ok(ExactBooleanResult::from_solids(vec![
                        solid_a.clone(),
                        solid_b.clone(),
                    ]))
                }
            }
            BooleanOpType::Intersection => {
                if a_inside_b {
                    Ok(ExactBooleanResult::single(solid_a.clone()))
                } else if b_inside_a {
                    Ok(ExactBooleanResult::single(solid_b.clone()))
                } else {
                    // 交わらない2立体の積は空。空であることは失敗ではなく
                    // 答えなので、エラーではなく空の結果で返す。ここは
                    // 「重なりが無い」と幾何的に確かめた枝であって、
                    // 「求められなかった」枝ではない。
                    Ok(ExactBooleanResult::from_solids(Vec::new()))
                }
            }
            BooleanOpType::Difference => {
                if a_inside_b {
                    // A が B に含まれるなら A - B は空。これも答えのほう。
                    Ok(ExactBooleanResult::from_solids(Vec::new()))
                } else if b_inside_a {
                    Solid::try_new(
                        solid_a.outer_shell.clone(),
                        vec![solid_b.outer_shell.clone()],
                        tol,
                    )
                    .map(ExactBooleanResult::single)
                    .map_err(|err| err.to_string())
                } else {
                    Ok(ExactBooleanResult::single(solid_a.clone()))
                }
            }
        }
    }

    fn solid_is_inside_or_on_boundary(solid: &Solid, container: &Solid, tol: &Tolerance) -> bool {
        solid.outer_shell.faces.iter().all(|face| {
            matches!(
                crate::BrepIntersectionBuilder::classify_face_against_solid(face, container, tol),
                crate::FaceRegionLocation::Inside | crate::FaceRegionLocation::Boundary
            )
        })
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

        let shell_assembly = crate::BrepIntersectionBuilder::collect_boolean_shell_assembly(
            solid_a, solid_b, op, tol,
        );
        Self::preparation_report_from_shell_assembly(solid_a, solid_b, &shell_assembly, tol)
    }

    /// 既に組み立ててあるシェルから、そのままの数え上げを返す。
    ///
    /// この報告は数え上げのためだけにあるのに、以前はここで面の組の走査を
    /// 5回やり直していました（面の組・交線・分割候補・色分け・シェルの
    /// 組み立てが順に呼ばれ、後ろのものは前のものを内側でやり直します）。
    /// 走査は面の組ごとにマーチングを走らせるので、報告を出す代償が
    /// 演算そのものより大きくなっていました。組み立てが済んでいるなら、
    /// その中身を数えれば同じ答えが出ます。
    fn preparation_report_from_shell_assembly(
        solid_a: &Solid,
        solid_b: &Solid,
        shell_assembly: &crate::BooleanShellAssembly,
        tol: &Tolerance,
    ) -> Result<ExactBooleanPreparationReport, String> {
        let face_pair_candidate_count = shell_assembly.face_pair_candidate_count;
        if face_pair_candidate_count == 0 {
            return Err(
                "Exact B-Rep boolean found no face-pair intersection candidates".to_string(),
            );
        }

        let intersection_edge_candidate_count = shell_assembly.edge_candidates.len();
        let planar_splits =
            crate::BrepIntersectionBuilder::planar_face_split_candidates_from_edge_candidates(
                &solid_a.outer_shell.faces,
                &solid_b.outer_shell.faces,
                shell_assembly.edge_candidates.clone(),
                tol,
            );
        let planar_split_candidate_count = planar_splits.len();
        let classified_splits =
            crate::BrepIntersectionBuilder::classified_planar_face_split_candidates_from_splits(
                solid_a,
                solid_b,
                planar_splits,
                tol,
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
        let planar_batch_skipped_split_count = shell_assembly
            .selection
            .batch_splits
            .splits_a
            .iter()
            .chain(shell_assembly.selection.batch_splits.splits_b.iter())
            .map(|split| split.result.skipped_split_count)
            .sum();

        Ok(ExactBooleanPreparationReport {
            face_pair_candidate_count,
            intersection_edge_candidate_count,
            planar_split_candidate_count,
            planar_batch_split_face_count,
            planar_batch_applied_split_count,
            planar_batch_skipped_split_count,
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

/// Halton sequence, so the overlap samples spread evenly without a random
/// source and stay identical between runs.
fn halton(mut index: usize, base: usize) -> f64 {
    let mut result = 0.0;
    let mut fraction = 1.0 / base as f64;
    while index > 0 {
        result += (index % base) as f64 * fraction;
        index /= base;
        fraction /= base as f64;
    }
    result
}
