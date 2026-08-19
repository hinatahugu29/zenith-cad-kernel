//! Zenith Algo: 断面スライス＆2D輪郭抽出エンジン (Section Slicing)
//! 任意の3D切断平面でB-Repソリッドを切断し、閉じた断面ワイヤループ群と断面特性を抽出。

use zenith_math::{Point3, Tolerance, Vec3};
use zenith_topo::{Edge, OrientedEdge, Solid, Vertex, Wire};

/// 断面スライスの結果データ構造
#[derive(Debug, Clone)]
pub struct SectionSliceResult {
    /// 抽出された閉じた断面ワイヤループ群
    pub section_wires: Vec<Wire>,
    /// 総断面積 (mm^2)
    pub total_area: f64,
    /// 断面外周の総周長 (mm)
    pub total_perimeter: f64,
}

pub struct SectionSlicer;

impl SectionSlicer {
    /// 任意の切断平面（原点 origin, 法線 normal）でソリッドを切断し、断面ループ群を抽出
    pub fn slice_solid(
        solid: &Solid,
        plane_origin: Point3,
        plane_normal: Vec3,
        tol: &Tolerance,
    ) -> Result<SectionSliceResult, String> {
        let normal = plane_normal.normalize();
        if normal.norm() < 1e-6 {
            return Err("Section plane normal must not be zero".to_string());
        }

        // 1. 各面のエッジと平面の交点を収集し、交差セグメント（2点間の線分）を構成
        let mut segments: Vec<(Point3, Point3)> = Vec::new();

        let all_faces = solid.outer_shell.faces.iter().chain(
            solid.inner_shells.iter().flat_map(|s| s.faces.iter())
        );

        for face in all_faces {
            let mut face_points: Vec<Point3> = Vec::new();

            for oe in face.outer_wire.edges.iter().chain(
                face.inner_wires.iter().flat_map(|w| w.edges.iter())
            ) {
                let p_start = oe.start_vertex().point;
                let p_end = oe.end_vertex().point;

                let d_start = (p_start - plane_origin).dot(&normal);
                let d_end = (p_end - plane_origin).dot(&normal);

                // 符号が異なる場合、エッジは平面を貫通している
                if (d_start > tol.linear && d_end < -tol.linear)
                    || (d_start < -tol.linear && d_end > tol.linear)
                {
                    let t = d_start / (d_start - d_end);
                    let p_intersect = oe.edge.curve.evaluate(
                        oe.edge.curve.knots.start_param(oe.edge.curve.degree)
                            + t * (oe.edge.curve.knots.end_param(oe.edge.curve.control_points.len() - oe.edge.curve.degree)
                                - oe.edge.curve.knots.start_param(oe.edge.curve.degree)),
                    );
                    face_points.push(p_intersect);
                } else if d_start.abs() <= tol.linear {
                    face_points.push(p_start);
                }
            }

            // 重複点の除去（近接点のマージ）
            let mut unique_pts: Vec<Point3> = Vec::new();
            for pt in face_points {
                if !unique_pts.iter().any(|u| (*u - pt).norm() <= tol.linear * 5.0) {
                    unique_pts.push(pt);
                }
            }

            // 面内で2点交差が得られた場合、線分セグメントを登録
            if unique_pts.len() == 2 {
                segments.push((unique_pts[0], unique_pts[1]));
            } else if unique_pts.len() > 2 {
                for i in 0..unique_pts.len() {
                    let next = (i + 1) % unique_pts.len();
                    segments.push((unique_pts[i], unique_pts[next]));
                }
            }
        }

        if segments.is_empty() {
            return Ok(SectionSliceResult {
                section_wires: Vec::new(),
                total_area: 0.0,
                total_perimeter: 0.0,
            });
        }

        // 2. セグメント群をチェイニングして閉ループ（Wire）を構築
        let loops = Self::chain_segments_into_loops(&segments, tol.linear * 10.0)?;

        let mut section_wires = Vec::new();
        let mut total_area = 0.0;
        let mut total_perimeter = 0.0;

        for pts in loops {
            if pts.len() < 3 {
                continue;
            }

            let mut wire_edges = Vec::with_capacity(pts.len());
            let n = pts.len();
            let mut perimeter = 0.0;

            for i in 0..n {
                let p1 = pts[i];
                let p2 = pts[(i + 1) % n];
                let len = (p2 - p1).norm();
                if len <= tol.linear {
                    continue;
                }
                perimeter += len;

                let v1 = Vertex::from_point(p1);
                let v2 = Vertex::from_point(p2);
                let edge = Edge::line_between(v1, v2)?;
                wire_edges.push(OrientedEdge::forward(edge));
            }

            if wire_edges.len() >= 3 {
                let area = Self::compute_polygon_area_on_plane(&pts, plane_origin, normal);
                total_area += area;
                total_perimeter += perimeter;
                section_wires.push(Wire::new(wire_edges));
            }
        }

        Ok(SectionSliceResult {
            section_wires,
            total_area,
            total_perimeter,
        })
    }

    fn chain_segments_into_loops(
        segments: &[(Point3, Point3)],
        tol: f64,
    ) -> Result<Vec<Vec<Point3>>, String> {
        let mut remaining = segments.to_vec();
        let mut loops = Vec::new();

        while !remaining.is_empty() {
            let (first_p1, first_p2) = remaining.remove(0);
            let mut current_loop = vec![first_p1, first_p2];

            let mut closed = false;
            while !closed {
                let current_end = *current_loop.last().unwrap();
                let mut found_idx = None;
                let mut reverse_match = false;

                for (idx, (p1, p2)) in remaining.iter().enumerate() {
                    if (*p1 - current_end).norm() <= tol {
                        found_idx = Some(idx);
                        reverse_match = false;
                        break;
                    } else if (*p2 - current_end).norm() <= tol {
                        found_idx = Some(idx);
                        reverse_match = true;
                        break;
                    }
                }

                if let Some(idx) = found_idx {
                    let (p1, p2) = remaining.remove(idx);
                    let next_pt = if reverse_match { p1 } else { p2 };

                    if (next_pt - current_loop[0]).norm() <= tol {
                        closed = true;
                    } else {
                        current_loop.push(next_pt);
                    }
                } else {
                    if (current_end - current_loop[0]).norm() <= tol * 2.0 {
                        closed = true;
                    }
                    break;
                }
            }

            if current_loop.len() >= 3 {
                loops.push(current_loop);
            }
        }

        Ok(loops)
    }

    fn compute_polygon_area_on_plane(pts: &[Point3], origin: Point3, normal: Vec3) -> f64 {
        if pts.len() < 3 {
            return 0.0;
        }

        let mut cross_sum = Vec3::zeros();
        let n = pts.len();

        for i in 0..n {
            let p1 = pts[i] - origin;
            let p2 = pts[(i + 1) % n] - origin;
            cross_sum += p1.cross(&p2);
        }

        (cross_sum.dot(&normal)).abs() * 0.5
    }
}
