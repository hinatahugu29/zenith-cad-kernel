//! Zenith Algo: 任意ソリッド間最短距離・最近傍点探索エンジン (Distance Engine)
//!
//! 2つの B-Rep ソリッドの表面どうしの最短距離と、その最近接点の組を返します。
//!
//! ## 以前の実装が返していたもの
//!
//! テッセレーションした2つのメッシュの**頂点どうし**を総当たりしていました。
//! 三角形の内側を一切見ないので、最近接点が頂点に来ない配置——つまり実務の
//! ほとんど——で答えが桁で外れます。
//!
//! | 配置 | 正しい距離 | 旧実装 |
//! | :--- | --: | --: |
//! | 200x200x2 の板の中央の上 3 mm に小球 | 3.0 | **136.6** |
//! | 同じ板の上 0.5 mm に小さな角材 | 0.5 | **138.6** |
//! | 5 mm めり込んだ2つの直方体 | 0.0 | **5.0** |
//!
//! 直方体の頂点は8個しかないので、板の上の物体は必ず**板の隅**との距離で
//! 測られていました。めり込んだ立体には正の隙間を返していました。クリアランス
//! 検証に使う値なので、干渉している設計が「隙間あり」と出ます。
//!
//! ## いまの実装
//!
//! 1. まず頂点どうしで上界を取る（これは真の距離以上なので上界として正しい）
//! 2. 三角形どうしの最短距離を、その上界で枝刈りしながら総当たりする
//!    （辺と辺、頂点と面の全組み合わせ。面の内側も辺も見る）
//! 3. 触れている・交わっている・片方が他方の内側にある場合は 0 を返す
//! 4. 最後に、得た最近接点を**B-Rep の面そのものへ交互に射影**して詰める。
//!    メッシュは弦なので、曲面では刻みの誤差が残る。射影した点がメッシュから
//!    離れていなければ（＝トリムされた面の上にあれば）採用する。

use zenith_geom::ExtremumEngine;
use zenith_math::{Point2, Point3, Tolerance};
use zenith_tess::{tessellate_solid, TessellationParams, TriangleMesh};
use zenith_topo::{FaceGeometry, Solid};

#[derive(Debug, Clone, PartialEq)]
pub struct DistanceResult {
    pub min_distance: f64,
    pub closest_point_a: Point3,
    pub closest_point_b: Point3,
}

pub struct DistanceEngine;

impl DistanceEngine {
    /// 2つのソリッド間の表面最短距離および最近傍点ペアを算出
    pub fn compute_min_distance(
        solid_a: &Solid,
        solid_b: &Solid,
        tol: &Tolerance,
    ) -> DistanceResult {
        Self::compute_min_distance_with_tessellation(
            solid_a,
            solid_b,
            tol,
            &TessellationParams {
                u_divisions: 16,
                v_divisions: 16,
            },
        )
    }

    /// 刻みの細かさを指定して最短距離を求める。
    ///
    /// 刻みは**探索の出発点**にしか効かない。答えは最後に B-Rep の面へ射影して
    /// 詰めるので、平面だけの立体では刻みによらず厳密になる。
    pub fn compute_min_distance_with_tessellation(
        solid_a: &Solid,
        solid_b: &Solid,
        tol: &Tolerance,
        params: &TessellationParams,
    ) -> DistanceResult {
        let mesh_a = tessellate_solid(solid_a, params);
        let mesh_b = tessellate_solid(solid_b, params);

        if mesh_a.positions.is_empty() || mesh_b.positions.is_empty() {
            return DistanceResult {
                min_distance: f64::INFINITY,
                closest_point_a: Point3::origin(),
                closest_point_b: Point3::origin(),
            };
        }

        // 1. 頂点どうしで上界を取る。真の距離以上なので枝刈りに使える。
        let (mut best, mut point_a, mut point_b) = vertex_bound(&mesh_a, &mesh_b);

        // 2. 三角形どうし。面の内側も辺も見る。
        triangle_pairs(&mesh_a, &mesh_b, &mut best, &mut point_a, &mut point_b);

        // 3. 触れている・交わっている・包含している場合は 0。
        if best <= tol.linear || overlaps(&mesh_a, &mesh_b, tol.linear) {
            return DistanceResult {
                min_distance: 0.0,
                closest_point_a: point_a,
                closest_point_b: point_b,
            };
        }

        // 4. B-Rep の面へ交互に射影して詰める。
        let (refined, refined_a, refined_b) =
            settle_on_surfaces(solid_a, solid_b, point_a, point_b, best);

        DistanceResult {
            min_distance: refined,
            closest_point_a: refined_a,
            closest_point_b: refined_b,
        }
    }
}

fn vertex_bound(mesh_a: &TriangleMesh, mesh_b: &TriangleMesh) -> (f64, Point3, Point3) {
    let mut best = f64::INFINITY;
    let mut point_a = mesh_a.positions[0];
    let mut point_b = mesh_b.positions[0];
    for a in &mesh_a.positions {
        for b in &mesh_b.positions {
            let distance = (a - b).norm_squared();
            if distance < best {
                best = distance;
                point_a = *a;
                point_b = *b;
            }
        }
    }
    (best.sqrt(), point_a, point_b)
}

/// 三角形の重心と外接半径。組を飛ばす判定に使う。
struct Bound {
    centre: Point3,
    radius: f64,
    corners: [Point3; 3],
}

fn bounds_of(mesh: &TriangleMesh) -> Vec<Bound> {
    mesh.indices
        .iter()
        .map(|triangle| {
            let corners = [
                mesh.positions[triangle[0] as usize],
                mesh.positions[triangle[1] as usize],
                mesh.positions[triangle[2] as usize],
            ];
            let centre = Point3::from(
                (corners[0].coords + corners[1].coords + corners[2].coords) / 3.0,
            );
            let radius = corners
                .iter()
                .map(|corner| (corner - centre).norm())
                .fold(0.0f64, f64::max);
            Bound {
                centre,
                radius,
                corners,
            }
        })
        .collect()
}

fn triangle_pairs(
    mesh_a: &TriangleMesh,
    mesh_b: &TriangleMesh,
    best: &mut f64,
    point_a: &mut Point3,
    point_b: &mut Point3,
) {
    let bounds_a = bounds_of(mesh_a);
    let bounds_b = bounds_of(mesh_b);

    for a in &bounds_a {
        // この三角形からは、どうやっても現在の最小を下回れない
        for b in &bounds_b {
            let separation = (a.centre - b.centre).norm() - a.radius - b.radius;
            if separation >= *best {
                continue;
            }
            let (distance, closest_a, closest_b) = triangle_distance(&a.corners, &b.corners);
            if distance < *best {
                *best = distance;
                *point_a = closest_a;
                *point_b = closest_b;
                if *best <= 0.0 {
                    return;
                }
            }
        }
    }
}

/// 2つの三角形の最短距離と、その最近接点。
///
/// 辺と辺の全組み合わせ（9通り）と、頂点と三角形の全組み合わせ（6通り）を
/// 取れば、交わっていない三角形の最短距離は必ずそのどれかで実現する。
fn triangle_distance(a: &[Point3; 3], b: &[Point3; 3]) -> (f64, Point3, Point3) {
    let mut best = f64::INFINITY;
    let mut best_a = a[0];
    let mut best_b = b[0];

    for i in 0..3 {
        for j in 0..3 {
            let (distance, pa, pb) = segment_distance(
                a[i],
                a[(i + 1) % 3],
                b[j],
                b[(j + 1) % 3],
            );
            if distance < best {
                best = distance;
                best_a = pa;
                best_b = pb;
            }
        }
    }

    for corner in a {
        let (distance, on_b) = point_triangle(*corner, b);
        if distance < best {
            best = distance;
            best_a = *corner;
            best_b = on_b;
        }
    }
    for corner in b {
        let (distance, on_a) = point_triangle(*corner, a);
        if distance < best {
            best = distance;
            best_a = on_a;
            best_b = *corner;
        }
    }

    (best, best_a, best_b)
}

fn segment_distance(
    p0: Point3,
    p1: Point3,
    q0: Point3,
    q1: Point3,
) -> (f64, Point3, Point3) {
    let d1 = p1 - p0;
    let d2 = q1 - q0;
    let r = p0 - q0;
    let a = d1.dot(&d1);
    let e = d2.dot(&d2);
    let f = d2.dot(&r);

    let (mut s, mut t);
    if a <= 1e-18 && e <= 1e-18 {
        return ((p0 - q0).norm(), p0, q0);
    }
    if a <= 1e-18 {
        s = 0.0;
        t = (f / e).clamp(0.0, 1.0);
    } else {
        let c = d1.dot(&r);
        if e <= 1e-18 {
            t = 0.0;
            s = (-c / a).clamp(0.0, 1.0);
        } else {
            let b = d1.dot(&d2);
            let denominator = a * e - b * b;
            s = if denominator > 1e-18 {
                ((b * f - c * e) / denominator).clamp(0.0, 1.0)
            } else {
                0.0
            };
            t = (b * s + f) / e;
            if t < 0.0 {
                t = 0.0;
                s = (-c / a).clamp(0.0, 1.0);
            } else if t > 1.0 {
                t = 1.0;
                s = ((b - c) / a).clamp(0.0, 1.0);
            }
        }
    }

    let on_p = p0 + d1 * s;
    let on_q = q0 + d2 * t;
    ((on_p - on_q).norm(), on_p, on_q)
}

fn point_triangle(point: Point3, triangle: &[Point3; 3]) -> (f64, Point3) {
    let (a, b, c) = (triangle[0], triangle[1], triangle[2]);
    let ab = b - a;
    let ac = c - a;
    let ap = point - a;

    let d1 = ab.dot(&ap);
    let d2 = ac.dot(&ap);
    if d1 <= 0.0 && d2 <= 0.0 {
        return ((point - a).norm(), a);
    }

    let bp = point - b;
    let d3 = ab.dot(&bp);
    let d4 = ac.dot(&bp);
    if d3 >= 0.0 && d4 <= d3 {
        return ((point - b).norm(), b);
    }

    let vc = d1 * d4 - d3 * d2;
    if vc <= 0.0 && d1 >= 0.0 && d3 <= 0.0 {
        let t = d1 / (d1 - d3);
        let on = a + ab * t;
        return ((point - on).norm(), on);
    }

    let cp = point - c;
    let d5 = ab.dot(&cp);
    let d6 = ac.dot(&cp);
    if d6 >= 0.0 && d5 <= d6 {
        return ((point - c).norm(), c);
    }

    let vb = d5 * d2 - d1 * d6;
    if vb <= 0.0 && d2 >= 0.0 && d6 <= 0.0 {
        let t = d2 / (d2 - d6);
        let on = a + ac * t;
        return ((point - on).norm(), on);
    }

    let va = d3 * d6 - d5 * d4;
    if va <= 0.0 && (d4 - d3) >= 0.0 && (d5 - d6) >= 0.0 {
        let t = (d4 - d3) / ((d4 - d3) + (d5 - d6));
        let on = b + (c - b) * t;
        return ((point - on).norm(), on);
    }

    let denominator = 1.0 / (va + vb + vc);
    let v = vb * denominator;
    let w = vc * denominator;
    let on = a + ab * v + ac * w;
    ((point - on).norm(), on)
}

/// 片方の表面の点が、もう一方の内側に入っているか。
///
/// 三角形どうしの距離は、辺と辺・頂点と面しか見ないので、**交差している**
/// 三角形の組では正の値になる。浅く食い込んだ立体（板に 0.01 mm 押し込んだ球）
/// はそれだけでは捕まらないので、点の内外でも見る。
///
/// 1点だけでは、その点がたまたま食い込んでいる側かどうかに答えが乗る。
/// 両方のメッシュから間引いて複数点を見る。
pub(crate) fn overlaps(mesh_a: &TriangleMesh, mesh_b: &TriangleMesh, margin: f64) -> bool {
    const SAMPLES: usize = 512;
    let probe = |from: &TriangleMesh, into: &TriangleMesh| -> bool {
        if from.positions.is_empty() || into.indices.is_empty() {
            return false;
        }
        let stride = (from.positions.len() / SAMPLES).max(1);
        from.positions.iter().step_by(stride).any(|point| {
            // 面の上に乗った点は、射線の偶奇では内とも外とも出る。**深さ**を
            // 見て、境界の上の点（面を共有して触れているだけの立体）を
            // 食い込みと言わないようにする。
            crate::BooleanEngine::is_point_inside_mesh(*point, into)
                && distance_to_mesh(*point, into) > margin
        })
    };
    probe(mesh_a, mesh_b) || probe(mesh_b, mesh_a)
}

/// 点からメッシュ表面までの最短距離
fn distance_to_mesh(point: Point3, mesh: &TriangleMesh) -> f64 {
    let mut best = f64::INFINITY;
    for triangle in &mesh.indices {
        let corners = [
            mesh.positions[triangle[0] as usize],
            mesh.positions[triangle[1] as usize],
            mesh.positions[triangle[2] as usize],
        ];
        let (distance, _) = point_triangle(point, &corners);
        if distance < best {
            best = distance;
            if best <= 0.0 {
                return 0.0;
            }
        }
    }
    best
}

/// メッシュ上の最近接点を、B-Rep の面へ交互に射影して詰める。
///
/// メッシュは弦なので、曲面では刻みの誤差が残る。射影した点が**その面の
/// トリム境界の内側にあるか**を p-curve で確かめてから採用する。
/// 詰まらなければメッシュの答えをそのまま返す（悪くはしない）。
fn settle_on_surfaces(
    solid_a: &Solid,
    solid_b: &Solid,
    mut point_a: Point3,
    mut point_b: Point3,
    mut best: f64,
) -> (f64, Point3, Point3) {
    for _round in 0..8 {
        let moved_b = project_onto_solid(point_a, solid_b);
        let moved_a = project_onto_solid(moved_b.unwrap_or(point_b), solid_a);

        let candidate_b = moved_b.unwrap_or(point_b);
        let candidate_a = moved_a.unwrap_or(point_a);
        let candidate = (candidate_a - candidate_b).norm();

        if candidate + 1e-15 >= best {
            break;
        }
        best = candidate;
        point_a = candidate_a;
        point_b = candidate_b;
    }

    (best, point_a, point_b)
}

/// 点をこの立体の面へ射影する。トリム境界の外に落ちた射影は採らない。
fn project_onto_solid(point: Point3, solid: &Solid) -> Option<Point3> {
    let mut best: Option<(f64, Point3)> = None;

    for shell in std::iter::once(&solid.outer_shell).chain(solid.inner_shells.iter()) {
        for face in &shell.faces {
            let (candidate, uv) = match &face.geometry {
                FaceGeometry::Plane(plane) => {
                    let offset = (point - plane.origin).dot(&plane.normal);
                    let foot = point - plane.normal * offset;
                    let local = foot - plane.origin;
                    (
                        foot,
                        Point2::new(local.dot(&plane.u_axis), local.dot(&plane.v_axis)),
                    )
                }
                FaceGeometry::Nurbs(surface) => {
                    match ExtremumEngine::point_to_surface(point, surface, 48, 1e-12) {
                        Ok(projection) => (
                            surface.evaluate(projection.u, projection.v),
                            Point2::new(projection.u, projection.v),
                        ),
                        Err(_) => continue,
                    }
                }
                _ => continue,
            };

            // 面の支持曲面の上ではあっても、**トリムされた領域の外**なら
            // この面の点ではない。以前はメッシュからの距離で代用していたが、
            // その帯を三角形の大きさから取っていたため、直方体では 14 mm も
            // あり、まったく別の面への射影まで通っていた（離れた2つの直方体の
            // 距離が 0 になっていた）。
            if !uv_is_inside_face(face, uv) {
                continue;
            }

            let distance = (candidate - point).norm();
            if best.map(|(d, _)| distance < d).unwrap_or(true) {
                best = Some((distance, candidate));
            }
        }
    }

    best.map(|(_, candidate)| candidate)
}

/// この (u, v) が、面のトリム境界の内側にあるか。
///
/// p-curve のループを折れ線に落として偶奇で判定する。内側ワイヤ（穴）の中は
/// 面の外。
fn uv_is_inside_face(face: &zenith_topo::Face, uv: Point2) -> bool {
    let Ok(pcurves) = face.pcurves(&Tolerance::default()) else {
        return false;
    };

    if !point_in_loop(&pcurves.outer_loop, uv) {
        return false;
    }
    for hole in &pcurves.inner_loops {
        if point_in_loop(hole, uv) {
            return false;
        }
    }
    true
}

fn point_in_loop(pcurve_loop: &zenith_topo::FacePcurveLoop, uv: Point2) -> bool {
    let mut polygon: Vec<Point2> = Vec::new();
    for segment in &pcurve_loop.segments {
        let (t_min, t_max) = segment.curve.param_range();
        const SAMPLES: usize = 24;
        for step in 0..SAMPLES {
            let t = t_min + (t_max - t_min) * step as f64 / SAMPLES as f64;
            polygon.push(segment.curve.evaluate(t));
        }
    }
    if polygon.len() < 3 {
        return false;
    }

    let mut inside = false;
    let count = polygon.len();
    for index in 0..count {
        let a = polygon[index];
        let b = polygon[(index + 1) % count];
        if (a.y > uv.y) != (b.y > uv.y) {
            let crossing = a.x + (uv.y - a.y) / (b.y - a.y) * (b.x - a.x);
            if crossing > uv.x {
                inside = !inside;
            }
        }
    }
    inside
}

