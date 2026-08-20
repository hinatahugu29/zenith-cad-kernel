//! 面を、その上に乗った1本の曲線で2枚に割る。
//!
//! # なぜ別に作るか
//!
//! `brep_intersection` の分割は、切り口が**軸まわりの円**であり、面の境界が
//! 「断面2辺 ＋ 側辺2辺」に読めることを前提にしている。回転面を軸に垂直な
//! 平面で切るときはそうなるが、曲面同士が交わるときの交線はパラメータ線では
//! なく、境界のどこにでも着地する。
//!
//! ここは前提を1つに減らす。
//!
//! > 分割線は面の**上**にあり、両端が面の**境界の上**にある。
//!
//! それだけを測って確かめ、あとは境界の巡回を2本に割って、それぞれを分割線で
//! 閉じる。軸も、断面と側辺の区別も、辺の本数も要らない。
//!
//! # 何を確かめるか
//!
//! 割ったあとに**面積を測って足す**。2枚の和が元の面積に戻らなければ、
//! 領域を取り違えている（重複か取りこぼし）。閉じたワイヤになったこと、
//! p-curve が辺に乗っていることだけでは、そこは分からない。
//!
//! 面積は導出した p-curve でトリムした領域を積むので、[`zenith_tess`] が
//! p-curve を持たない面を「トリム前の面」として積んでいた間は、この検査は
//! 意味を成さなかった。

use zenith_geom::{ExtremumEngine, NurbsCurve3};
use zenith_math::{Point3, Tolerance};
use zenith_tess::TessellationParams;
use zenith_topo::{Edge, Face, FaceGeometry, OrientedEdge, Vertex, Wire};

use crate::mass_properties::MassCalculator;

/// 割った結果と、その割り方が正しかったかを測った値。
#[derive(Debug, Clone, PartialEq)]
pub struct FaceSplitReport {
    /// 元の面の面積。
    pub original_area: f64,
    /// 出来た各片の面積。
    pub piece_areas: Vec<f64>,
    /// 片の面積の和が元からどれだけずれたか（相対）。
    pub area_residual: f64,
    /// 分割線が面から離れていた最大距離。
    pub curve_off_surface: f64,
    /// 分割線の端が境界から離れていた距離。
    pub ends_off_boundary: f64,
}

/// 面の上の曲線で面を割る。
pub struct FaceSplitter;

impl FaceSplitter {
    /// `split` で `face` を2枚に割る。
    ///
    /// `split` は面の上に乗り、両端が `face` の外周ワイヤの上になければ
    /// ならない。内周（穴）を持つ面は、まだ扱わない。
    pub fn split_by_curve(
        face: &Face,
        split: &Edge,
        tol: &Tolerance,
    ) -> Result<(Vec<Face>, FaceSplitReport), String> {
        if !face.inner_wires.is_empty() {
            return Err("splitting a face that has holes is not implemented".to_string());
        }
        let edges = &face.outer_wire.edges;
        if edges.len() < 2 {
            return Err("a face boundary needs at least two edges to be split".to_string());
        }

        let scale = boundary_extent(&face.outer_wire).max(1.0);
        let limit = tol.linear * 10.0 * scale;

        // 1. 分割線が本当にこの面の上にあるか。構成に使っていない位置で測る。
        let curve_off_surface = Self::distance_to_surface(face, &split.curve, 23)?;
        if curve_off_surface > limit {
            return Err(format!(
                "the splitting curve leaves the face by {curve_off_surface:.3e}, over {limit:.3e}"
            ));
        }

        // 2. 両端が境界のどこに乗るか。乗っていなければ割れない。
        let start = split.start_vertex.point;
        let end = split.end_vertex.point;
        let (from, from_distance) = locate_on_wire(&face.outer_wire, start, tol)
            .ok_or_else(|| "the splitting curve does not start on the boundary".to_string())?;
        let (to, to_distance) = locate_on_wire(&face.outer_wire, end, tol)
            .ok_or_else(|| "the splitting curve does not end on the boundary".to_string())?;
        let ends_off_boundary = from_distance.max(to_distance);
        if ends_off_boundary > limit {
            return Err(format!(
                "the splitting curve ends {ends_off_boundary:.3e} away from the boundary"
            ));
        }

        let count = edges.len() as f64;
        let separation = ((to - from).rem_euclid(count)).min((from - to).rem_euclid(count));
        if separation <= 1e-9 {
            return Err("both ends of the splitting curve land at the same place".to_string());
        }

        // 3. 巡回を2本に割り、それぞれを分割線で閉じる。
        let forward = walk(edges, from, to, tol)?;
        let backward = walk(edges, to, from, tol)?;

        let split_forward = orient_between(split, start, end, tol)
            .ok_or_else(|| "the splitting edge does not run between its own ends".to_string())?;
        let split_backward = OrientedEdge::new(
            split_forward.edge.clone(),
            split_forward.orientation.reversed(),
        );

        let mut first = forward;
        first.push(split_backward);
        let mut second = backward;
        second.push(split_forward);

        let pieces: Vec<Face> = [first, second]
            .into_iter()
            .map(|wire_edges| {
                Face::new(
                    face.geometry.clone(),
                    Wire::new(wire_edges),
                    Vec::new(),
                    face.orientation,
                    face.tolerance,
                )
            })
            .collect();

        for (index, piece) in pieces.iter().enumerate() {
            if piece.outer_wire.edges.len() < 2 {
                return Err(format!("piece {index} came out with too few edges"));
            }
            if !piece.outer_wire.is_closed(tol) {
                return Err(format!("piece {index} came out with an open wire"));
            }
        }

        // 4. 面積を測って足す。ここが合わなければ領域を取り違えている。
        let params = TessellationParams::default();
        let original_area = MassCalculator::compute_face_integral(face, &params).0;
        let piece_areas: Vec<f64> = pieces
            .iter()
            .map(|piece| MassCalculator::compute_face_integral(piece, &params).0)
            .collect();
        let summed: f64 = piece_areas.iter().sum();
        let area_residual = if original_area.abs() > 1e-12 {
            (summed - original_area).abs() / original_area.abs()
        } else {
            (summed - original_area).abs()
        };

        Ok((
            pieces,
            FaceSplitReport {
                original_area,
                piece_areas,
                area_residual,
                curve_off_surface,
                ends_off_boundary,
            },
        ))
    }

    /// 曲線が面からどれだけ離れているか。標本の数は面の作りと互いに素にする。
    fn distance_to_surface(
        face: &Face,
        curve: &NurbsCurve3,
        samples: usize,
    ) -> Result<f64, String> {
        let (t0, t1) = curve.param_range();
        let mut worst: f64 = 0.0;
        for step in 0..=samples {
            let point = curve.evaluate(t0 + (t1 - t0) * step as f64 / samples as f64);
            let distance = match &face.geometry {
                FaceGeometry::Plane(plane) => {
                    let normal = plane.normal.normalize();
                    (point - plane.origin).dot(&normal).abs()
                }
                FaceGeometry::Nurbs(surface) => {
                    ExtremumEngine::point_to_surface(point, surface, 64, 1e-13)
                        .map_err(|err| format!("could not project onto the face: {err}"))?
                        .distance
                }
                _ => return Err("this face geometry cannot be split yet".to_string()),
            };
            worst = worst.max(distance);
        }
        Ok(worst)
    }
}

/// 外周ワイヤの広がり。公差を形の大きさに合わせるために使う。
fn boundary_extent(wire: &Wire) -> f64 {
    let Some(first) = wire.edges.first() else {
        return 1.0;
    };
    let origin = first.start_vertex().point;
    wire.edges.iter().fold(0.0f64, |worst, oriented| {
        worst
            .max((oriented.start_vertex().point - origin).norm())
            .max((oriented.end_vertex().point - origin).norm())
    })
}

/// 点が巡回のどこに乗るかを、`辺の番号 + 辺内の割合` で返す。
///
/// 割合は曲線の媒介変数で測る。弧長ではないが、同じ物差しで一貫していれば
/// 巡回を割るには足りる。
fn locate_on_wire(wire: &Wire, point: Point3, tol: &Tolerance) -> Option<(f64, f64)> {
    let mut best: Option<(f64, f64)> = None;
    for (index, oriented) in wire.edges.iter().enumerate() {
        let projection =
            ExtremumEngine::point_to_curve(point, &oriented.edge.curve, 128, 1e-13).ok()?;
        let (t0, t1) = oriented.edge.curve.param_range();
        if (t1 - t0).abs() <= f64::EPSILON {
            continue;
        }
        let raw = ((projection.parameter - t0) / (t1 - t0)).clamp(0.0, 1.0);
        let fraction = if oriented.orientation.is_forward() {
            raw
        } else {
            1.0 - raw
        };
        let distance = projection.distance;
        if best.as_ref().map(|(_, d)| distance < *d).unwrap_or(true) {
            best = Some((index as f64 + fraction, distance));
        }
    }
    let _ = tol;
    best
}

/// 巡回座標 `from` から `to` まで、巡回の向きに辿った辺の並び。
fn walk(
    edges: &[OrientedEdge],
    from: f64,
    to: f64,
    tol: &Tolerance,
) -> Result<Vec<OrientedEdge>, String> {
    let count = edges.len();
    let total = count as f64;
    let mut end = to;
    if end <= from + 1e-12 {
        end += total;
    }

    let mut out = Vec::new();
    let mut cursor = from;
    let mut guard = 0;
    while cursor < end - 1e-12 {
        guard += 1;
        if guard > count * 3 + 4 {
            return Err("walking the boundary did not terminate".to_string());
        }
        let base = cursor.floor();
        let index = (base as usize) % count;
        let next = (base + 1.0).min(end);
        let low = cursor - base;
        let high = next - base;
        if high - low > 1e-12 {
            out.push(sub_edge(&edges[index], low, high, tol)?);
        }
        cursor = next;
    }

    if out.is_empty() {
        return Err("walking the boundary produced nothing".to_string());
    }
    Ok(out)
}

/// 辺の、辿る向きで `low` から `high` までの部分。割合は 0..1。
fn sub_edge(
    oriented: &OrientedEdge,
    low: f64,
    high: f64,
    tol: &Tolerance,
) -> Result<OrientedEdge, String> {
    let whole = low <= 1e-12 && high >= 1.0 - 1e-12;
    if whole {
        return Ok(oriented.clone());
    }

    let (t0, t1) = oriented.edge.curve.param_range();
    let span = t1 - t0;
    // 辿る向きの割合を、曲線そのものの媒介変数に直す。
    let (a, b) = if oriented.orientation.is_forward() {
        (t0 + span * low, t0 + span * high)
    } else {
        (t0 + span * (1.0 - high), t0 + span * (1.0 - low))
    };

    let piece = subcurve(&oriented.edge.curve, a, b)
        .ok_or_else(|| format!("could not take the curve between {a} and {b}"))?;
    let (p0, p1) = piece.param_range();
    let start_point = piece.evaluate(p0);
    let end_point = piece.evaluate(p1);

    // 端が元の頂点と同じなら、その頂点をそのまま使う。新しく作ると、隣の面が
    // 使っている頂点と別物になる。
    let reuse = |point: Point3| -> Vertex {
        for candidate in [&oriented.edge.start_vertex, &oriented.edge.end_vertex] {
            if (candidate.point - point).norm() <= tol.linear {
                return candidate.clone();
            }
        }
        Vertex::new(point, tol.linear)
    };

    let edge = Edge::new(
        piece,
        reuse(start_point),
        reuse(end_point),
        oriented.edge.tolerance,
    );
    Ok(OrientedEdge::new(edge, oriented.orientation))
}

/// 曲線の `a` から `b` までを取り出す。`a < b`。
fn subcurve(curve: &NurbsCurve3, a: f64, b: f64) -> Option<NurbsCurve3> {
    let (t0, t1) = curve.param_range();
    let span = (t1 - t0).abs().max(1.0);
    let mut piece = curve.clone();
    if a > t0 + span * 1e-12 {
        piece = piece.split_at(a)?.1;
    }
    if b < t1 - span * 1e-12 {
        piece = piece.split_at(b)?.0;
    }
    Some(piece)
}

/// `start` から `end` へ向くように辺の向きを決める。
fn orient_between(
    edge: &Edge,
    start: Point3,
    end: Point3,
    tol: &Tolerance,
) -> Option<OrientedEdge> {
    let limit = tol.linear.max(1e-9) * 10.0;
    if (edge.start_vertex.point - start).norm() <= limit
        && (edge.end_vertex.point - end).norm() <= limit
    {
        return Some(OrientedEdge::forward(edge.clone()));
    }
    if (edge.start_vertex.point - end).norm() <= limit
        && (edge.end_vertex.point - start).norm() <= limit
    {
        return Some(OrientedEdge::reversed(edge.clone()));
    }
    None
}

#[cfg(test)]
mod tests {
    use super::FaceSplitter;
    use std::f64::consts::{FRAC_1_SQRT_2, FRAC_PI_2};
    use zenith_geom::{ControlPoint3, KnotVector, NurbsCurve3, NurbsSurface3, PlaneSurface3};
    use zenith_math::{Point3, Tolerance, Vec3};
    use zenith_topo::{Edge, Face, FaceGeometry, OrientedEdge, Orientation, Vertex, Wire};

    /// 半径 `r`、高さ `0..h` の円柱側面の四半パッチ。
    fn cylinder_quarter(r: f64, h: f64) -> Face {
        let w = FRAC_1_SQRT_2;
        let grid: Vec<Vec<ControlPoint3>> = [(r, 0.0, 1.0), (r, r, w), (0.0, r, 1.0)]
            .iter()
            .map(|(x, y, weight)| {
                vec![
                    ControlPoint3::new(Point3::new(*x, *y, 0.0), *weight),
                    ControlPoint3::new(Point3::new(*x, *y, h), *weight),
                ]
            })
            .collect();
        let surface = NurbsSurface3::new(
            2,
            1,
            grid,
            KnotVector::clamped_uniform(3, 2),
            KnotVector::clamped_uniform(2, 1),
        )
        .unwrap();
        let arc = |z: f64| {
            NurbsCurve3::new(
                2,
                vec![
                    ControlPoint3::unweighted(Point3::new(r, 0.0, z)),
                    ControlPoint3::new(Point3::new(r, r, z), w),
                    ControlPoint3::unweighted(Point3::new(0.0, r, z)),
                ],
                KnotVector::clamped_uniform(3, 2),
            )
            .unwrap()
        };
        let bottom_start = Vertex::from_point(Point3::new(r, 0.0, 0.0));
        let bottom_end = Vertex::from_point(Point3::new(0.0, r, 0.0));
        let top_start = Vertex::from_point(Point3::new(r, 0.0, h));
        let top_end = Vertex::from_point(Point3::new(0.0, r, h));
        Face::new(
            FaceGeometry::Nurbs(surface),
            Wire::new(vec![
                OrientedEdge::forward(Edge::new(
                    arc(0.0),
                    bottom_start.clone(),
                    bottom_end.clone(),
                    1e-6,
                )),
                OrientedEdge::forward(
                    Edge::line_between(bottom_end.clone(), top_end.clone()).unwrap(),
                ),
                OrientedEdge::reversed(Edge::new(arc(h), top_start.clone(), top_end.clone(), 1e-6)),
                OrientedEdge::reversed(
                    Edge::line_between(bottom_start.clone(), top_start.clone()).unwrap(),
                ),
            ]),
            Vec::new(),
            Orientation::Forward,
            1e-6,
        )
    }

    /// 傾いた平面が円柱を切ってできる楕円弧。
    ///
    /// 楕円は円のアフィン像なので、円弧の制御点に同じ写像をかければ**厳密に**
    /// 表せる。折れ線で近づけると、確かめたいものが測れなくなる。
    fn tilted_section(r: f64, z0: f64, slope: f64) -> Edge {
        let w = FRAC_1_SQRT_2;
        let lift = |x: f64, y: f64| Point3::new(x, y, z0 + slope * x);
        let curve = NurbsCurve3::new(
            2,
            vec![
                ControlPoint3::unweighted(lift(r, 0.0)),
                ControlPoint3::new(lift(r, r), w),
                ControlPoint3::unweighted(lift(0.0, r)),
            ],
            KnotVector::clamped_uniform(3, 2),
        )
        .unwrap();
        let (t0, t1) = curve.param_range();
        Edge::new(
            curve.clone(),
            Vertex::from_point(curve.evaluate(t0)),
            Vertex::from_point(curve.evaluate(t1)),
            1e-6,
        )
    }

    fn planar_square(side: f64) -> Face {
        let plane = PlaneSurface3::new(
            Point3::new(0.0, 0.0, 0.0),
            Vec3::new(1.0, 0.0, 0.0),
            Vec3::new(0.0, 1.0, 0.0),
        )
        .unwrap();
        let corners = [
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(side, 0.0, 0.0),
            Point3::new(side, side, 0.0),
            Point3::new(0.0, side, 0.0),
        ];
        let vertices: Vec<Vertex> = corners.into_iter().map(Vertex::from_point).collect();
        Face::new(
            FaceGeometry::Plane(plane),
            Wire::new(
                (0..4)
                    .map(|index| {
                        OrientedEdge::forward(
                            Edge::line_between(
                                vertices[index].clone(),
                                vertices[(index + 1) % 4].clone(),
                            )
                            .unwrap(),
                        )
                    })
                    .collect(),
            ),
            Vec::new(),
            Orientation::Forward,
            1e-6,
        )
    }

    /// パラメータ線でない曲線で四辺形パッチを割れること。
    ///
    /// **これが曲面同士の交差の本当の壁である。** 既存の分割は「切り口は軸
    /// まわりの円」「境界は断面2辺と側辺2辺」を前提にしており、そこを外れると
    /// 割れない。
    #[test]
    fn a_quadrilateral_patch_splits_along_a_curve_that_is_not_a_parameter_line() {
        let tol = Tolerance::default();
        let radius = 10.0;
        // 下側の面積は円柱を開いて積める: r * ∫(z0 + slope r cos t) dt, t = 0..pi/2
        for (z0, slope) in [(20.0, 0.6), (25.0, -0.9), (12.0, 0.4)] {
            let face = cylinder_quarter(radius, 40.0);
            let split = tilted_section(radius, z0, slope);
            let (pieces, report) = FaceSplitter::split_by_curve(&face, &split, &tol)
                .unwrap_or_else(|err| panic!("z0 {z0} slope {slope}: {err}"));

            assert_eq!(pieces.len(), 2);
            assert!(
                report.curve_off_surface < 1e-9,
                "the split curve was not on the face: {:.3e}",
                report.curve_off_surface
            );
            // 面積の和が元に戻ること。閉じたワイヤになっただけでは、領域の
            // 重複や取りこぼしは分からない。
            assert!(
                report.area_residual < 1e-9,
                "z0 {z0} slope {slope}: the pieces do not add up, residual {:.3e}",
                report.area_residual
            );

            let lower = radius * (z0 * FRAC_PI_2 + slope * radius);
            let best = report
                .piece_areas
                .iter()
                .map(|area| (area - lower).abs() / lower)
                .fold(f64::INFINITY, f64::min);
            assert!(
                best < 1e-6,
                "z0 {z0} slope {slope}: no piece matches the closed form {lower}, \
                 got {:?} (closest {best:.3e})",
                report.piece_areas
            );

            for piece in &pieces {
                assert!(piece.outer_wire.is_closed(&tol));
                let pcurves = piece.validate_pcurves(&tol, 37).expect("p-curves");
                assert!(
                    pcurves.is_valid(),
                    "a split piece's p-curves left its edges: {} mismatches",
                    pcurves.mismatch_count
                );
            }
        }
    }

    /// 平面を割るのは厳密でなければならない。曲がっていないので、近似の余地が
    /// どこにも無い。
    #[test]
    fn splitting_a_planar_face_is_exact() {
        let tol = Tolerance::default();

        // 角から角へ: ちょうど半分になる。
        let face = planar_square(10.0);
        let split = Edge::line_between(
            Vertex::from_point(Point3::new(10.0, 0.0, 0.0)),
            Vertex::from_point(Point3::new(0.0, 10.0, 0.0)),
        )
        .unwrap();
        let (_, report) = FaceSplitter::split_by_curve(&face, &split, &tol).expect("corner cut");
        assert!(report.area_residual < 1e-14);
        for area in &report.piece_areas {
            assert!(
                (area - 50.0).abs() < 1e-12,
                "half of the square is 50, got {area}"
            );
        }

        // 辺の途中から辺の途中へ: 角を切り落とす三角形と、残りの五角形。
        let face = planar_square(10.0);
        let split = Edge::line_between(
            Vertex::from_point(Point3::new(10.0, 4.0, 0.0)),
            Vertex::from_point(Point3::new(3.0, 10.0, 0.0)),
        )
        .unwrap();
        let (pieces, report) = FaceSplitter::split_by_curve(&face, &split, &tol).expect("mid cut");
        assert_eq!(pieces.len(), 2);
        assert!(report.area_residual < 1e-14);
        let triangle = 0.5 * 6.0 * 7.0;
        let best = report
            .piece_areas
            .iter()
            .map(|area| (area - triangle).abs())
            .fold(f64::INFINITY, f64::min);
        assert!(
            best < 1e-12,
            "the corner piece should be {triangle}, got {:?}",
            report.piece_areas
        );
    }

    /// 面の上に無い曲線、境界に届かない曲線は**断らなければならない**。
    /// もっともらしい2枚を返すほうが悪い。
    #[test]
    fn a_curve_that_does_not_lie_on_the_face_is_refused() {
        let tol = Tolerance::default();
        let face = cylinder_quarter(10.0, 40.0);

        // 円柱から離れたところを通る直線
        let off_surface = Edge::line_between(
            Vertex::from_point(Point3::new(10.0, 0.0, 20.0)),
            Vertex::from_point(Point3::new(0.0, 5.0, 20.0)),
        )
        .unwrap();
        assert!(FaceSplitter::split_by_curve(&face, &off_surface, &tol).is_err());

        // 面の上ではあるが、境界に届かない（片端が内部にある）
        let whole = tilted_section(10.0, 20.0, 0.6);
        let (t0, t1) = whole.curve.param_range();
        let half = whole.curve.split_at((t0 + t1) * 0.5).unwrap().0;
        let (m0, m1) = half.param_range();
        let stub = Edge::new(
            half.clone(),
            Vertex::from_point(half.evaluate(m0)),
            Vertex::from_point(half.evaluate(m1)),
            1e-6,
        );
        assert!(
            FaceSplitter::split_by_curve(&face, &stub, &tol).is_err(),
            "a curve that stops inside the face must not produce two pieces"
        );
    }
}
