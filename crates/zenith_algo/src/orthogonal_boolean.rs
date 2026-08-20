use crate::{BooleanOpType, BrepTransform, PrimitiveBuilder};
use zenith_geom::PlaneSurface3;
use zenith_math::{Point3, Tolerance};
use zenith_topo::{Edge, Face, FaceGeometry, OrientedEdge, Shell, Solid, Vertex, Wire};

pub(crate) struct OrthogonalBoxBoolean;

#[derive(Debug, Clone, Copy, PartialEq)]
pub(crate) struct AxisAlignedBoxBounds {
    pub(crate) min: Point3,
    pub(crate) max: Point3,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
struct GridCell {
    i: usize,
    j: usize,
    k: usize,
}

impl OrthogonalBoxBoolean {
    pub(crate) fn boolean_axis_aligned_boxes_exact(
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
                    // 重なりが無いことを境界箱で確かめた枝。積は空であって、
                    // 求められなかったわけではない。呼び出し側が空の結果を
                    // 作れるよう、ここでは「この近道の出番ではない」と返す。
                    return Ok(None);
                };
                Self::make_box_from_bounds(overlap).map(Some)
            }
            BooleanOpType::Union => {
                if let Some(union) = bounds_a.union_if_single_box(bounds_b, tol) {
                    Self::make_box_from_bounds(union).map(Some)
                } else if bounds_a.intersection(bounds_b, tol).is_some() {
                    Self::build_orthogonal_box_boolean(bounds_a, bounds_b, op, tol).map(Some)
                } else {
                    Ok(None)
                }
            }
            BooleanOpType::Difference => {
                if let Some(difference) = bounds_a.difference_if_single_box(bounds_b, tol) {
                    Self::make_box_from_bounds(difference).map(Some)
                } else if bounds_a.intersection(bounds_b, tol).is_some() {
                    Self::build_orthogonal_box_boolean(bounds_a, bounds_b, op, tol).map(Some)
                } else {
                    Ok(None)
                }
            }
        }
    }

    fn build_orthogonal_box_boolean(
        bounds_a: AxisAlignedBoxBounds,
        bounds_b: AxisAlignedBoxBounds,
        op: BooleanOpType,
        tol: &Tolerance,
    ) -> Result<Solid, String> {
        let xs = sorted_unique_coords(
            &[
                bounds_a.min.x,
                bounds_a.max.x,
                bounds_b.min.x,
                bounds_b.max.x,
            ],
            tol,
        );
        let ys = sorted_unique_coords(
            &[
                bounds_a.min.y,
                bounds_a.max.y,
                bounds_b.min.y,
                bounds_b.max.y,
            ],
            tol,
        );
        let zs = sorted_unique_coords(
            &[
                bounds_a.min.z,
                bounds_a.max.z,
                bounds_b.min.z,
                bounds_b.max.z,
            ],
            tol,
        );
        if xs.len() < 2 || ys.len() < 2 || zs.len() < 2 {
            return Err("Exact axis-aligned box boolean has degenerate grid".to_string());
        }

        let nx = xs.len() - 1;
        let ny = ys.len() - 1;
        let nz = zs.len() - 1;
        let mut occupied = vec![false; nx * ny * nz];
        for i in 0..nx {
            for j in 0..ny {
                for k in 0..nz {
                    let center = Point3::new(
                        (xs[i] + xs[i + 1]) * 0.5,
                        (ys[j] + ys[j + 1]) * 0.5,
                        (zs[k] + zs[k + 1]) * 0.5,
                    );
                    let in_a = bounds_a.contains_point(center, tol);
                    let in_b = bounds_b.contains_point(center, tol);
                    occupied[cell_index(i, j, k, ny, nz)] = match op {
                        BooleanOpType::Union => in_a || in_b,
                        BooleanOpType::Difference => in_a && !in_b,
                        BooleanOpType::Intersection => in_a && in_b,
                    };
                }
            }
        }

        if occupied.iter().all(|is_occupied| !*is_occupied) {
            return Err("Exact axis-aligned box boolean produced an empty result".to_string());
        }
        if !occupied_cells_are_connected(&occupied, nx, ny, nz) {
            return Err(
                "Exact axis-aligned box boolean produced multiple disjoint regions".to_string(),
            );
        }

        let mut faces = Vec::new();
        for i in 0..nx {
            for j in 0..ny {
                for k in 0..nz {
                    if !occupied[cell_index(i, j, k, ny, nz)] {
                        continue;
                    }
                    for side in 0..6 {
                        if neighbor_is_occupied(&occupied, nx, ny, nz, i, j, k, side) {
                            continue;
                        }
                        faces.push(make_grid_boundary_face(&xs, &ys, &zs, i, j, k, side, tol)?);
                    }
                }
            }
        }

        Solid::try_simple(Shell::closed(faces), tol).map_err(|err| err.to_string())
    }

    fn make_box_from_bounds(bounds: AxisAlignedBoxBounds) -> Result<Solid, String> {
        let size = bounds.max - bounds.min;
        let solid = PrimitiveBuilder::make_box(size.x, size.y, size.z)?;
        Ok(BrepTransform::translate_solid(&solid, bounds.min.coords))
    }

    pub(crate) fn axis_aligned_box_bounds(
        solid: &Solid,
        tol: &Tolerance,
    ) -> Option<AxisAlignedBoxBounds> {
        if !solid.inner_shells.is_empty() || solid.outer_shell.faces.len() != 6 {
            return None;
        }
        if solid.outer_shell.faces.iter().any(|face| {
            !face.inner_wires.is_empty() || !matches!(&face.geometry, FaceGeometry::Plane(_))
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

    fn contains_point(self, point: Point3, tol: &Tolerance) -> bool {
        point.x >= self.min.x - tol.linear
            && point.x <= self.max.x + tol.linear
            && point.y >= self.min.y - tol.linear
            && point.y <= self.max.y + tol.linear
            && point.z >= self.min.z - tol.linear
            && point.z <= self.max.z + tol.linear
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

fn sorted_unique_coords(values: &[f64], tol: &Tolerance) -> Vec<f64> {
    let mut coords = values.to_vec();
    coords.sort_by(|a, b| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal));
    coords.dedup_by(|a, b| (*a - *b).abs() <= tol.linear);
    coords
}

fn cell_index(i: usize, j: usize, k: usize, ny: usize, nz: usize) -> usize {
    (i * ny + j) * nz + k
}

fn occupied_cells_are_connected(occupied: &[bool], nx: usize, ny: usize, nz: usize) -> bool {
    let Some(start_index) = occupied.iter().position(|is_occupied| *is_occupied) else {
        return false;
    };

    let mut visited = vec![false; occupied.len()];
    let mut stack = vec![index_to_cell(start_index, ny, nz)];
    visited[start_index] = true;
    let mut visited_count = 0;

    while let Some(cell) = stack.pop() {
        visited_count += 1;
        for side in 0..6 {
            let Some(neighbor) = neighbor_cell(cell.i, cell.j, cell.k, nx, ny, nz, side) else {
                continue;
            };
            let index = cell_index(neighbor.i, neighbor.j, neighbor.k, ny, nz);
            if occupied[index] && !visited[index] {
                visited[index] = true;
                stack.push(neighbor);
            }
        }
    }

    visited_count == occupied.iter().filter(|is_occupied| **is_occupied).count()
}

fn index_to_cell(index: usize, ny: usize, nz: usize) -> GridCell {
    GridCell {
        i: index / (ny * nz),
        j: (index / nz) % ny,
        k: index % nz,
    }
}

fn neighbor_is_occupied(
    occupied: &[bool],
    nx: usize,
    ny: usize,
    nz: usize,
    i: usize,
    j: usize,
    k: usize,
    side: usize,
) -> bool {
    neighbor_cell(i, j, k, nx, ny, nz, side)
        .map(|cell| occupied[cell_index(cell.i, cell.j, cell.k, ny, nz)])
        .unwrap_or(false)
}

fn neighbor_cell(
    i: usize,
    j: usize,
    k: usize,
    nx: usize,
    ny: usize,
    nz: usize,
    side: usize,
) -> Option<GridCell> {
    match side {
        0 => (i > 0).then(|| GridCell { i: i - 1, j, k }),
        1 => (i + 1 < nx).then(|| GridCell { i: i + 1, j, k }),
        2 => (j > 0).then(|| GridCell { i, j: j - 1, k }),
        3 => (j + 1 < ny).then(|| GridCell { i, j: j + 1, k }),
        4 => (k > 0).then(|| GridCell { i, j, k: k - 1 }),
        5 => (k + 1 < nz).then(|| GridCell { i, j, k: k + 1 }),
        _ => unreachable!("side must be 0 through 5"),
    }
}

fn make_grid_boundary_face(
    xs: &[f64],
    ys: &[f64],
    zs: &[f64],
    i: usize,
    j: usize,
    k: usize,
    side: usize,
    tol: &Tolerance,
) -> Result<Face, String> {
    let x0 = xs[i];
    let x1 = xs[i + 1];
    let y0 = ys[j];
    let y1 = ys[j + 1];
    let z0 = zs[k];
    let z1 = zs[k + 1];

    let points = match side {
        0 => vec![
            Point3::new(x0, y0, z0),
            Point3::new(x0, y0, z1),
            Point3::new(x0, y1, z1),
            Point3::new(x0, y1, z0),
        ],
        1 => vec![
            Point3::new(x1, y0, z0),
            Point3::new(x1, y1, z0),
            Point3::new(x1, y1, z1),
            Point3::new(x1, y0, z1),
        ],
        2 => vec![
            Point3::new(x0, y0, z0),
            Point3::new(x1, y0, z0),
            Point3::new(x1, y0, z1),
            Point3::new(x0, y0, z1),
        ],
        3 => vec![
            Point3::new(x0, y1, z0),
            Point3::new(x0, y1, z1),
            Point3::new(x1, y1, z1),
            Point3::new(x1, y1, z0),
        ],
        4 => vec![
            Point3::new(x0, y0, z0),
            Point3::new(x0, y1, z0),
            Point3::new(x1, y1, z0),
            Point3::new(x1, y0, z0),
        ],
        5 => vec![
            Point3::new(x0, y0, z1),
            Point3::new(x1, y0, z1),
            Point3::new(x1, y1, z1),
            Point3::new(x0, y1, z1),
        ],
        _ => unreachable!("side must be 0 through 5"),
    };

    make_quad_face(points, tol)
}

fn make_quad_face(points: Vec<Point3>, tol: &Tolerance) -> Result<Face, String> {
    let plane = PlaneSurface3::new(points[0], points[1] - points[0], points[3] - points[0])
        .ok_or("Failed to create grid boundary plane")?;
    let vertices: Vec<Vertex> = points
        .iter()
        .map(|point| Vertex::new(*point, tol.linear))
        .collect();
    let mut edges = Vec::with_capacity(4);
    for i in 0..4 {
        edges.push(OrientedEdge::forward(Edge::line_between(
            vertices[i].clone(),
            vertices[(i + 1) % 4].clone(),
        )?));
    }

    Ok(Face::new(
        FaceGeometry::Plane(plane),
        Wire::new(edges),
        Vec::new(),
        zenith_topo::Orientation::Forward,
        tol.linear,
    ))
}
