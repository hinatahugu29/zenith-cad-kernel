use zenith_math::{Point3, RobustPredicates, Tolerance, Vec3};
use zenith_tess::{tessellate_solid, TessellationParams, TriangleMesh};
use zenith_topo::{FaceGeometry, Solid};

/// ブーリアン演算の種類
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum BooleanOpType {
    Union,        // 結合 (A + B)
    Difference,   // 差分 (A - B)
    Intersection, // 共通部分 (A * B)
}

/// B-Rep / ポリゴンブーリアン演算エンジン
pub struct BooleanEngine;

#[derive(Debug, Clone, Copy, PartialEq)]
struct AxisAlignedBoxBounds {
    min: Point3,
    max: Point3,
}

impl AxisAlignedBoxBounds {
    fn intersection(self, other: Self, tol: &Tolerance) -> Option<Self> {
        let min = Point3::new(
            self.min.x.max(other.min.x),
            self.min.y.max(other.min.y),
            self.min.z.max(other.min.z),
        );
        let max = Point3::new(
            self.max.x.min(other.max.x),
            self.max.y.min(other.max.y),
            self.max.z.min(other.max.z),
        );
        Self::from_min_max_if_positive(min, max, tol)
    }

    fn union_if_single_box(self, other: Self, tol: &Tolerance) -> Option<Self> {
        for axis in 0..3 {
            if self.same_span_on_other_axes(other, axis, tol)
                && intervals_overlap_or_touch(
                    self.axis_min(axis),
                    self.axis_max(axis),
                    other.axis_min(axis),
                    other.axis_max(axis),
                    tol,
                )
            {
                return Some(Self {
                    min: Point3::new(
                        self.min.x.min(other.min.x),
                        self.min.y.min(other.min.y),
                        self.min.z.min(other.min.z),
                    ),
                    max: Point3::new(
                        self.max.x.max(other.max.x),
                        self.max.y.max(other.max.y),
                        self.max.z.max(other.max.z),
                    ),
                });
            }
        }

        None
    }

    fn difference_if_single_box(self, subtract: Self, tol: &Tolerance) -> Option<Self> {
        for axis in 0..3 {
            if !subtract.covers_other_axes(self, axis, tol) {
                continue;
            }

            let a_min = self.axis_min(axis);
            let a_max = self.axis_max(axis);
            let b_min = subtract.axis_min(axis);
            let b_max = subtract.axis_max(axis);

            if b_min <= a_min + tol.linear
                && b_max > a_min + tol.linear
                && b_max < a_max - tol.linear
            {
                return Self::from_axis_interval(self, axis, b_max, a_max, tol);
            }
            if b_max >= a_max - tol.linear
                && b_min > a_min + tol.linear
                && b_min < a_max - tol.linear
            {
                return Self::from_axis_interval(self, axis, a_min, b_min, tol);
            }
        }

        None
    }

    fn from_min_max_if_positive(min: Point3, max: Point3, tol: &Tolerance) -> Option<Self> {
        let size = max - min;
        (size.x > tol.linear && size.y > tol.linear && size.z > tol.linear)
            .then_some(Self { min, max })
    }

    fn from_axis_interval(
        source: Self,
        axis: usize,
        min_value: f64,
        max_value: f64,
        tol: &Tolerance,
    ) -> Option<Self> {
        let mut min = source.min;
        let mut max = source.max;
        set_axis_value(&mut min, axis, min_value);
        set_axis_value(&mut max, axis, max_value);
        Self::from_min_max_if_positive(min, max, tol)
    }

    fn same_span_on_other_axes(self, other: Self, axis: usize, tol: &Tolerance) -> bool {
        (0..3)
            .filter(|candidate| *candidate != axis)
            .all(|candidate| {
                (self.axis_min(candidate) - other.axis_min(candidate)).abs() <= tol.linear
                    && (self.axis_max(candidate) - other.axis_max(candidate)).abs() <= tol.linear
            })
    }

    fn covers_other_axes(self, other: Self, axis: usize, tol: &Tolerance) -> bool {
        (0..3)
            .filter(|candidate| *candidate != axis)
            .all(|candidate| {
                self.axis_min(candidate) <= other.axis_min(candidate) + tol.linear
                    && self.axis_max(candidate) >= other.axis_max(candidate) - tol.linear
            })
    }

    fn axis_min(self, axis: usize) -> f64 {
        axis_value(self.min, axis)
    }

    fn axis_max(self, axis: usize) -> f64 {
        axis_value(self.max, axis)
    }
}

fn intervals_overlap_or_touch(
    a_min: f64,
    a_max: f64,
    b_min: f64,
    b_max: f64,
    tol: &Tolerance,
) -> bool {
    a_min <= b_max + tol.linear && b_min <= a_max + tol.linear
}

fn axis_value(point: Point3, axis: usize) -> f64 {
    match axis {
        0 => point.x,
        1 => point.y,
        2 => point.z,
        _ => unreachable!("axis must be 0, 1, or 2"),
    }
}

fn set_axis_value(point: &mut Point3, axis: usize, value: f64) {
    match axis {
        0 => point.x = value,
        1 => point.y = value,
        2 => point.z = value,
        _ => unreachable!("axis must be 0, 1, or 2"),
    }
}

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
        if !solid_a.is_topologically_valid(tol) {
            return Err("Exact B-Rep boolean input A is not topologically valid".to_string());
        }
        if !solid_b.is_topologically_valid(tol) {
            return Err("Exact B-Rep boolean input B is not topologically valid".to_string());
        }

        if !Self::has_face_pair_candidates(solid_a, solid_b, tol) {
            return Self::boolean_solids_exact_without_intersections(solid_a, solid_b, op, tol);
        }
        if let Some(solid) = Self::boolean_axis_aligned_boxes_exact(solid_a, solid_b, op, tol)? {
            return Ok(solid);
        }

        let shell_assembly = crate::BrepIntersectionBuilder::collect_boolean_shell_assembly(
            solid_a, solid_b, op, tol,
        );
        if shell_assembly.selection.stitch_report.is_closed_manifold() {
            return crate::BrepIntersectionBuilder::build_solid_from_selected_face_pieces(
                &shell_assembly.selection.selected_face_pieces,
                tol,
            );
        }
        if shell_assembly.assembly.stitch_report.is_closed_manifold() {
            return crate::BrepIntersectionBuilder::build_solid_from_selected_face_pieces(
                &shell_assembly.assembly.selected_face_pieces,
                tol,
            );
        }

        let report = Self::prepare_exact_boolean(solid_a, solid_b, op, tol)?;
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

    fn has_face_pair_candidates(solid_a: &Solid, solid_b: &Solid, tol: &Tolerance) -> bool {
        !crate::BrepIntersectionBuilder::collect_face_pair_candidates(
            &solid_a.outer_shell.faces,
            &solid_b.outer_shell.faces,
            tol,
        )
        .is_empty()
    }

    fn boolean_axis_aligned_boxes_exact(
        solid_a: &Solid,
        solid_b: &Solid,
        op: BooleanOpType,
        tol: &Tolerance,
    ) -> Result<Option<Solid>, String> {
        let Some(bounds_a) = Self::axis_aligned_box_bounds(solid_a, tol) else {
            return Ok(None);
        };
        let Some(bounds_b) = Self::axis_aligned_box_bounds(solid_b, tol) else {
            return Ok(None);
        };

        match op {
            BooleanOpType::Intersection => {
                let Some(overlap) = bounds_a.intersection(bounds_b, tol) else {
                    return Err(
                        "Exact B-Rep boolean intersection has no positive volume overlap"
                            .to_string(),
                    );
                };
                Self::make_box_from_bounds(overlap).map(Some)
            }
            BooleanOpType::Union => {
                if let Some(union) = bounds_a.union_if_single_box(bounds_b, tol) {
                    Self::make_box_from_bounds(union).map(Some)
                } else {
                    Ok(None)
                }
            }
            BooleanOpType::Difference => {
                if let Some(difference) = bounds_a.difference_if_single_box(bounds_b, tol) {
                    Self::make_box_from_bounds(difference).map(Some)
                } else {
                    Ok(None)
                }
            }
        }
    }

    fn make_box_from_bounds(bounds: AxisAlignedBoxBounds) -> Result<Solid, String> {
        let size = bounds.max - bounds.min;
        let solid = crate::PrimitiveBuilder::make_box(size.x, size.y, size.z)?;
        Ok(crate::BrepTransform::translate_solid(
            &solid,
            bounds.min.coords,
        ))
    }

    fn axis_aligned_box_bounds(solid: &Solid, tol: &Tolerance) -> Option<AxisAlignedBoxBounds> {
        if !solid.inner_shells.is_empty() || solid.outer_shell.faces.len() != 6 {
            return None;
        }
        if solid.outer_shell.faces.iter().any(|face| {
            !face.inner_wires.is_empty() || !matches!(face.geometry, FaceGeometry::Plane(_))
        }) {
            return None;
        }

        let points = Self::solid_outer_wire_points(solid);
        if points.len() < 8
            || points
                .iter()
                .any(|point| !point.coords.iter().all(|v| v.is_finite()))
        {
            return None;
        }

        let mut min = points[0];
        let mut max = points[0];
        for point in points.iter().skip(1) {
            min.x = min.x.min(point.x);
            min.y = min.y.min(point.y);
            min.z = min.z.min(point.z);
            max.x = max.x.max(point.x);
            max.y = max.y.max(point.y);
            max.z = max.z.max(point.z);
        }

        if max.x - min.x <= tol.linear || max.y - min.y <= tol.linear || max.z - min.z <= tol.linear
        {
            return None;
        }

        let expected_corners = [
            Point3::new(min.x, min.y, min.z),
            Point3::new(max.x, min.y, min.z),
            Point3::new(max.x, max.y, min.z),
            Point3::new(min.x, max.y, min.z),
            Point3::new(min.x, min.y, max.z),
            Point3::new(max.x, min.y, max.z),
            Point3::new(max.x, max.y, max.z),
            Point3::new(min.x, max.y, max.z),
        ];
        if !expected_corners.iter().all(|corner| {
            points
                .iter()
                .any(|point| (*point - *corner).norm() <= tol.linear)
        }) {
            return None;
        }

        for face in &solid.outer_shell.faces {
            let face_points = face.outer_wire.sample_points(1);
            if face_points.len() < 4 {
                return None;
            }
            let on_box_side = [
                face_points
                    .iter()
                    .all(|point| (point.x - min.x).abs() <= tol.linear),
                face_points
                    .iter()
                    .all(|point| (point.x - max.x).abs() <= tol.linear),
                face_points
                    .iter()
                    .all(|point| (point.y - min.y).abs() <= tol.linear),
                face_points
                    .iter()
                    .all(|point| (point.y - max.y).abs() <= tol.linear),
                face_points
                    .iter()
                    .all(|point| (point.z - min.z).abs() <= tol.linear),
                face_points
                    .iter()
                    .all(|point| (point.z - max.z).abs() <= tol.linear),
            ];
            if !on_box_side.iter().any(|side| *side) {
                return None;
            }
        }

        Some(AxisAlignedBoxBounds { min, max })
    }

    fn solid_outer_wire_points(solid: &Solid) -> Vec<Point3> {
        let mut points = Vec::new();
        for face in &solid.outer_shell.faces {
            for edge in &face.outer_wire.edges {
                points.push(edge.start_vertex().point);
                points.push(edge.end_vertex().point);
            }
        }
        points
    }

    fn boolean_solids_exact_without_intersections(
        solid_a: &Solid,
        solid_b: &Solid,
        op: BooleanOpType,
        tol: &Tolerance,
    ) -> Result<Solid, String> {
        let a_inside_b = Self::solid_is_inside_or_on_boundary(solid_a, solid_b, tol);
        let b_inside_a = Self::solid_is_inside_or_on_boundary(solid_b, solid_a, tol);

        match op {
            BooleanOpType::Union => {
                if a_inside_b {
                    Ok(solid_b.clone())
                } else if b_inside_a {
                    Ok(solid_a.clone())
                } else {
                    Err("Exact B-Rep boolean union of disjoint solids requires compound result support".to_string())
                }
            }
            BooleanOpType::Intersection => {
                if a_inside_b {
                    Ok(solid_a.clone())
                } else if b_inside_a {
                    Ok(solid_b.clone())
                } else {
                    Err("Exact B-Rep boolean intersection is empty for disjoint solids".to_string())
                }
            }
            BooleanOpType::Difference => {
                if a_inside_b {
                    Err("Exact B-Rep boolean difference is empty because input A is contained in input B".to_string())
                } else if b_inside_a {
                    Solid::try_new(
                        solid_a.outer_shell.clone(),
                        vec![solid_b.outer_shell.clone()],
                        tol,
                    )
                    .map_err(|err| err.to_string())
                } else {
                    Ok(solid_a.clone())
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
