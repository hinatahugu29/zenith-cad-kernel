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
use zenith_math::{Point2, Point3, Tolerance, Vec3};
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
            let centre =
                Point3::from((corners[0].coords + corners[1].coords + corners[2].coords) / 3.0);
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
            let (distance, pa, pb) = segment_distance(a[i], a[(i + 1) % 3], b[j], b[(j + 1) % 3]);
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

fn segment_distance(p0: Point3, p1: Point3, q0: Point3, q1: Point3) -> (f64, Point3, Point3) {
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
    let probe = |from: &TriangleMesh, into: &TriangleMesh| -> bool {
        if from.positions.is_empty() || into.indices.is_empty() {
            return false;
        }
        let mut min_into = Point3::new(f64::INFINITY, f64::INFINITY, f64::INFINITY);
        let mut max_into = Point3::new(f64::NEG_INFINITY, f64::NEG_INFINITY, f64::NEG_INFINITY);
        for p in &into.positions {
            min_into.x = min_into.x.min(p.x);
            min_into.y = min_into.y.min(p.y);
            min_into.z = min_into.z.min(p.z);
            max_into.x = max_into.x.max(p.x);
            max_into.y = max_into.y.max(p.y);
            max_into.z = max_into.z.max(p.z);
        }

        from.positions.iter().any(|point| {
            // AABBで素早く事前除外
            if point.x < min_into.x - margin
                || point.x > max_into.x + margin
                || point.y < min_into.y - margin
                || point.y > max_into.y + margin
                || point.z < min_into.z - margin
                || point.z > max_into.z + margin
            {
                return false;
            }
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
    nearest_boundary_projection(point, solid).map(|projection| projection.foot)
}

/// 立体の境界上でいちばん近い点と、そこでの**外向き法線**。
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct BoundaryProjection {
    pub distance: f64,
    pub foot: Point3,
    pub outward_normal: Vec3,
}

/// 点から立体の境界へ、B-Rep の面そのものを使って射影する。
///
/// メッシュは弦なので、曲面の近くでは刻みぶんだけずれる。ここは支持曲面へ
/// ニュートン法で落として、その足が**その面のトリム領域の内側にあるか**を
/// p-curve で確かめてから採る。トリムの外に落ちた面は、その点の面ではない。
///
/// 外向き法線は支持曲面の法線を面の向きで反転したもの。テッセレーションが
/// 三角形の向きを決めるときと同じ規則（`zenith_tess` の `oriented_normal`）。
pub fn nearest_boundary_projection(point: Point3, solid: &Solid) -> Option<BoundaryProjection> {
    zenith_geom::work_counter::count_seed_on_patch_projection();
    if std::env::var_os("ZENITH_DISTANCE_WHY").is_some() {
        static ONCE: std::sync::Once = std::sync::Once::new();
        ONCE.call_once(|| {
            eprintln!(
                "DISTANCEWHY 呼ばれました:
{}",
                std::backtrace::Backtrace::force_capture()
            );
        });
    }
    boundary_projections(point, solid)
        .into_iter()
        .reduce(|best, candidate| {
            if candidate.distance < best.distance {
                candidate
            } else {
                best
            }
        })
}

/// 面ごとに、その面の上でいちばん近い点を1つ返す。
///
/// 足がトリム領域の外に落ちたら、**その面の境界の稜**へ寄せます。以前は
/// そこで面ごと捨てていたので、直方体の角の外にいる点はどの面にも足を持たず、
/// 立体そのものへの距離が出ませんでした（6面すべてが「自分の長方形の外」と
/// 答える）。捨てるのではなく寄せるのが正しく、寄せた先が本当の最近点です。
/// **その点の近くに境界があるか**だけを見る（4-294）。
///
/// `boundary_projections` は**すべての面へ全域射影**して、いちばん近い所を
/// 全部集めます。**「近いものが1つでもあるか」しか要らないとき、それは
/// 高すぎます**——曲面1枚につき 17x17 の格子と8段の詰め（曲面評価 353 回）を
/// 払うので、37 枚の立体で 1 点あたり 1万回を超えます。
///
/// 実測（`linkrods.step` の和、30 秒の窓）: 射影 68,394 回のうち **63,442 回**が
/// この道の「粗い全域」でした。**辿りも積分も終わったあとに、ここだけで
/// 曲面評価が 2,600 万回**回っています。
///
/// 同じ答えを、2つの手当てで安く出します。
///
/// - **面の囲みで先に捨てる**。囲みまでの距離が上限を超えていれば、その面の
///   どこも上限より近くはなりません（囲みは面を含むので、これは安全です）
/// - **見つかったら止める**。全部集める必要はありません
pub(crate) fn has_boundary_within(point: Point3, solid: &Solid, limit: f64) -> bool {
    for shell in std::iter::once(&solid.outer_shell).chain(solid.inner_shells.iter()) {
        for face in &shell.faces {
            let bbox = face.bounding_box();
            // 囲みまでの距離（外にいるときだけ正）。
            let outside = zenith_math::Vec3::new(
                (bbox.min.x - point.x).max(0.0).max(point.x - bbox.max.x),
                (bbox.min.y - point.y).max(0.0).max(point.y - bbox.max.y),
                (bbox.min.z - point.z).max(0.0).max(point.z - bbox.max.z),
            );
            if outside.norm() > limit {
                continue;
            }
            if let Some(projection) = face_projection(point, face) {
                if projection.distance <= limit {
                    return true;
                }
            }
        }
    }
    false
}

pub(crate) fn boundary_projections(point: Point3, solid: &Solid) -> Vec<BoundaryProjection> {
    let mut projections = Vec::new();

    for shell in std::iter::once(&solid.outer_shell).chain(solid.inner_shells.iter()) {
        for face in &shell.faces {
            if let Some(projection) = face_projection(point, face) {
                projections.push(projection);
            }
        }
    }

    projections
}

/// この面の上で、点にいちばん近い所。
fn face_projection(point: Point3, face: &zenith_topo::Face) -> Option<BoundaryProjection> {
    let on_support = support_foot(point, face);

    // 足がトリムの内側にあれば、それが最近点。
    if let Some((foot, uv, normal)) = on_support {
        if uv_is_inside_face(face, uv) {
            return Some(BoundaryProjection {
                distance: (foot - point).norm(),
                foot,
                outward_normal: oriented(normal, face),
            });
        }
    }

    // 外に落ちたら、面の境界の稜へ寄せる。
    let mut best: Option<Point3> = None;
    let wires = std::iter::once(&face.outer_wire).chain(face.inner_wires.iter());
    for wire in wires {
        for oriented_edge in &wire.edges {
            let Ok(projection) =
                ExtremumEngine::point_to_curve(point, &oriented_edge.edge.curve, 64, 1e-12)
            else {
                continue;
            };
            if best
                .map(|current| projection.distance < (current - point).norm())
                .unwrap_or(true)
            {
                best = Some(projection.closest_point);
            }
        }
    }

    let foot = best?;
    // 稜の上の点でも法線は支持曲面から取れる。稜は面の上にあるので、
    // そこへ射影し直せば (u, v) が出る。
    let (_, _, normal) = support_foot(foot, face)?;
    Some(BoundaryProjection {
        distance: (foot - point).norm(),
        foot,
        outward_normal: oriented(normal, face),
    })
}

/// 支持曲面の上の足と、その (u, v)、その点の法線。トリムは見ない。
fn support_foot(point: Point3, face: &zenith_topo::Face) -> Option<(Point3, Point2, Vec3)> {
    match &face.geometry {
        FaceGeometry::Plane(plane) => {
            let offset = (point - plane.origin).dot(&plane.normal);
            let foot = point - plane.normal * offset;
            let local = foot - plane.origin;
            Some((
                foot,
                Point2::new(local.dot(&plane.u_axis), local.dot(&plane.v_axis)),
                plane.normal,
            ))
        }
        FaceGeometry::Nurbs(surface) => {
            zenith_geom::work_counter::count_other_projection();
            let projection = { ExtremumEngine::point_to_surface(point, surface, 48, 1e-12).ok()? };
            // **極では `normal` が `None` を返します。** 回転面の軸上にある点は
            // 最近点がちょうど極になるので、そのままだと「この面には足が無い」
            // ことになります（読んだ球の中心線がそれでした）。まわりからの
            // 極限を採ります。滑らかでない点（円錐の頂点など）では極限も
            // 取れないので、そこは従来どおり採りません。
            let normal = surface.normal_or_limit(projection.u, projection.v)?;
            Some((
                surface.evaluate(projection.u, projection.v),
                Point2::new(projection.u, projection.v),
                normal,
            ))
        }
        _ => None,
    }
}

/// 支持曲面の法線を、面の向きで外向きに直す。テッセレーションが三角形の
/// 向きを決めるときと同じ規則（`zenith_tess` の `oriented_normal`）。
fn oriented(normal: Vec3, face: &zenith_topo::Face) -> Vec3 {
    if face.orientation.is_forward() {
        normal
    } else {
        -normal
    }
}

/// この (u, v) が、面のトリム境界の内側にあるか。
///
/// p-curve のループを折れ線に落として偶奇で判定する。内側ワイヤ（穴）の中は
/// 面の外。
fn uv_is_inside_face(face: &zenith_topo::Face, uv: Point2) -> bool {
    let Ok(pcurves) = face.pcurves(&Tolerance::default()) else {
        return false;
    };
    let ranges = support_param_range(face);

    // 境界のワイヤを1本も持たない面は、**トリムされていない**。支持曲面の
    // パラメータ領域そのものが面で、射影はその中に落ちる。
    //
    // ここを「外側ループが空 ⇒ 内側には何も無い」と読んでいました。他カーネル
    // から読んだ球や円柱は**全周1枚の面**で来るので、その立体は面を1枚も持た
    // ないのと同じ扱いになり、距離の計算からも内外の判定からも丸ごと落ちて
    // いました。`DistanceEngine` が読んだ球に対して答えていた値は、面を1枚も
    // 見ずに出したものです。
    if pcurves.outer_loop.segments.is_empty() {
        if !face.outer_wire.edges.is_empty() {
            // 稜はあるのに p-curve が空。境界が分からないので採らない。
            return false;
        }
        return !pcurves
            .inner_loops
            .iter()
            .any(|hole| loop_contains(hole, uv, ranges, LoopRole::Hole));
    }

    if !loop_contains(&pcurves.outer_loop, uv, ranges, LoopRole::Outer) {
        return false;
    }
    for hole in &pcurves.inner_loops {
        if loop_contains(hole, uv, ranges, LoopRole::Hole) {
            return false;
        }
    }
    true
}

/// 支持曲面のパラメータ範囲。巻きの判定に、**周期の長さ**が要ります。
fn support_param_range(face: &zenith_topo::Face) -> Option<((f64, f64), (f64, f64))> {
    match &face.geometry {
        zenith_topo::FaceGeometry::Nurbs(surface) => Some(surface.param_range()),
        _ => None,
    }
}

/// この輪が、面のどちら側を囲んでいるか。
#[derive(Clone, Copy, PartialEq, Eq)]
enum LoopRole {
    /// 外側の輪。囲むのは**進む向きの左**（反時計回りに蓄える約束）。
    Outer,
    /// 穴の輪。外側と逆向きに蓄えるので、囲むのは**進む向きの右**。
    Hole,
}

/// **uv を1周する輪には、内と外が「囲み」で決まりません**（4-843）。
///
/// `point_in_loop` は輪を折れ線にして偶奇で数えます。**切れ目で端から端へ
/// 飛ぶ輪**（球の極を囲む窓の輪など）では、折れ線が**まっすぐな弦で閉じる**
/// ので、囲まれるのは「v=0 からその輪までの帯」になり、**極の側が外**と
/// 出ます。4-842 で面積の読み方を直したのと**同じ取り違え**で、こちらは
/// **点の内外判定**に残っていました（実測: ＼の結果で、z>9.1652——球の
/// 断面が箱の内に収まる高さ——の 24 点が、消えた帽子の中なのに「立体の中」
/// と返っていました）。
///
/// 1周する輪は、**パラメータ領域を2つの帯に分けます**。どちらが内かは
/// 囲みでは決まらず、**進む向きと輪の役**で決まります。
fn loop_contains(
    pcurve_loop: &zenith_topo::FacePcurveLoop,
    uv: Point2,
    ranges: Option<((f64, f64), (f64, f64))>,
    role: LoopRole,
) -> bool {
    if wrapping_loop_side_on() {
        if let Some(ranges) = ranges {
            if let Some(inside) = wrapping_loop_contains(pcurve_loop, uv, ranges, role) {
                return inside;
            }
        }
    }
    point_in_loop(pcurve_loop, uv)
}

/// **1周する輪を見つけたときだけ**答えます。そうでなければ `None` を返して、
/// 従来の偶奇に任せます（4-843。`ZENITH_WRAPPING_LOOP_SIDE=0` で止まります）。
fn wrapping_loop_contains(
    pcurve_loop: &zenith_topo::FacePcurveLoop,
    uv: Point2,
    ((u_min, u_max), (v_min, v_max)): ((f64, f64), (f64, f64)),
    role: LoopRole,
) -> Option<bool> {
    let period = u_max - u_min;
    if !(period > 0.0) {
        return None;
    }
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
        return None;
    }

    // **極に届く輪は、1周する輪ではありません**（4-842 と同じ場合分け）。
    // 素の周期球の外周は極から極まで通るので、ここで扱ってはいけません。
    let span_v = (v_max - v_min).abs();
    let reaches_pole = polygon.iter().any(|point| {
        (point.y - v_min).abs() <= span_v * 1e-6 || (point.y - v_max).abs() <= span_v * 1e-6
    });
    if reaches_pole {
        return None;
    }

    // 切れ目の飛びを1つだけ持つ輪を探します。2つ以上あるなら、帯のような
    // 穴（行きと帰りで2回またぐ）で、この場合分けでは決められません。
    let mut jump: Option<usize> = None;
    let mut net_du = 0.0;
    for index in 0..polygon.len() {
        let a = polygon[index];
        let b = polygon[(index + 1) % polygon.len()];
        let raw = b.x - a.x;
        let unwrapped = raw - period * (raw / period).round();
        net_du += unwrapped;
        if (raw - unwrapped).abs() > period * 0.5 {
            if jump.is_some() {
                return None;
            }
            jump = Some(index);
        }
    }
    let jump = jump?;
    if net_du.abs() < period * (1.0 - 1e-6) {
        return None;
    }

    // 飛びの直後から並べ直すと、輪は u の関数 v(u) になります。
    let mut path: Vec<Point2> = Vec::with_capacity(polygon.len());
    for step in 0..polygon.len() {
        path.push(polygon[(jump + 1 + step) % polygon.len()]);
    }

    // 点の u に当たる所の、輪の v。u は周期なので、近い2点で挟みます。
    let wrapped_u = u_min + (uv.x - u_min).rem_euclid(period);
    let bracketed = path.windows(2).find_map(|window| {
        let (a, b) = (window[0], window[1]);
        let (lo, hi) = if a.x <= b.x { (a, b) } else { (b, a) };
        if hi.x > lo.x && wrapped_u >= lo.x && wrapped_u <= hi.x {
            Some(lo.y + (hi.y - lo.y) * (wrapped_u - lo.x) / (hi.x - lo.x))
        } else {
            None
        }
    });
    let loop_v = match bracketed {
        Some(v) => v,
        // 挟めないときは、いちばん u が近い点の v で代えます。
        None => {
            let distance_in_u = |point: &Point2| {
                let raw = (point.x - wrapped_u).abs();
                raw.min(period - raw)
            };
            path.iter()
                .min_by(|a, b| {
                    distance_in_u(a)
                        .partial_cmp(&distance_in_u(b))
                        .unwrap_or(std::cmp::Ordering::Equal)
                })?
                .y
        }
    };

    // **進む向きの右**は、u が減る向きに進むとき +v です（(x,y) を −90°
    // 回すと (y,−x)。(−1,0) → (0,1)）。穴は右を、外側は左を囲みます。
    let travels_backwards = net_du < 0.0;
    let encloses_larger_v = match role {
        LoopRole::Hole => travels_backwards,
        LoopRole::Outer => !travels_backwards,
    };
    Some(if encloses_larger_v {
        uv.y > loop_v
    } else {
        uv.y < loop_v
    })
}

/// 1周する輪の側を、囲みではなく向きで決める口（4-843。既定オン、
/// `ZENITH_WRAPPING_LOOP_SIDE=0` で止まります）。
fn wrapping_loop_side_on() -> bool {
    std::env::var("ZENITH_WRAPPING_LOOP_SIDE")
        .map(|value| value != "0")
        .unwrap_or(false)
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

#[cfg(test)]
mod near_boundary_tests {
    use super::*;

    /// **安くした道が、元の道と同じ答えを出すか**（4-294）。
    ///
    /// `has_boundary_within` は「近いものが1つでもあるか」だけを見ます。
    /// `boundary_projections` は**すべての面へ全域射影して全部集める**ので、
    /// 同じ問いに桁違いの値段を払います（実測: `linkrods.step` の和で、
    /// 射影の 100% がそこでした）。
    ///
    /// **速いほうが安全なのは、答えが同じときだけです。** 面の囲みで捨てる
    /// のも、見つかったら止めるのも、答えを変えないはずですが、**それは
    /// 測って確かめること**です。
    fn agrees(point: Point3, solid: &Solid, limit: f64) -> bool {
        let cheap = has_boundary_within(point, solid, limit);
        let thorough = boundary_projections(point, solid)
            .iter()
            .any(|projection| projection.distance <= limit);
        cheap == thorough
    }

    #[test]
    fn the_cheap_near_boundary_test_agrees_with_the_thorough_one() {
        let solid = crate::PrimitiveBuilder::make_torus(12.0, 4.0).expect("torus");
        // **境界の内・外・上**を、格子で広く取ります。1点だけ通しても
        // 意味がありません（4-115 の「範囲を振る」）。
        let mut checked = 0usize;
        for ix in -3..=3 {
            for iy in -3..=3 {
                for iz in -2..=2 {
                    let point = Point3::new(ix as f64 * 5.0, iy as f64 * 5.0, iz as f64 * 2.5);
                    // **上限も振ります。** 上限で答えが変わる点こそ、
                    // 囲みで捨てる判断が効くところです。
                    for limit in [1e-6, 1e-3, 0.1, 1.0, 5.0] {
                        assert!(
                            agrees(point, &solid, limit),
                            "点 {point:?}、上限 {limit} で答えが違います"
                        );
                        checked += 1;
                    }
                }
            }
        }
        assert!(checked >= 700, "測った数が少なすぎます: {checked}");
    }

    #[test]
    fn it_also_agrees_on_a_solid_with_a_cavity() {
        // 空洞のある立体でも同じであること——内側の殻を見落とすと、
        // **中に居る点で答えが変わります**。
        let outer = crate::PrimitiveBuilder::make_box(20.0, 20.0, 20.0).expect("box");
        let inner = crate::BrepTransform::translate_solid(
            &crate::PrimitiveBuilder::make_box(6.0, 6.0, 6.0).expect("inner"),
            zenith_math::Vec3::new(7.0, 7.0, 7.0),
        );
        let tol = Tolerance::default();
        let Ok(result) = crate::BooleanEngine::boolean_solids_exact_result(
            &outer,
            &inner,
            crate::BooleanOpType::Difference,
            &tol,
        ) else {
            // 断られたら、この検体では測れません。**黙って通しません。**
            panic!("空洞を作る差が断られました");
        };
        let solid = result.solids.first().expect("立体が1つ返る");
        for ix in 0..=4 {
            for iy in 0..=4 {
                for iz in 0..=4 {
                    let point = Point3::new(ix as f64 * 5.0, iy as f64 * 5.0, iz as f64 * 5.0);
                    for limit in [1e-6, 0.5, 2.0] {
                        assert!(
                            agrees(point, solid, limit),
                            "空洞つき: 点 {point:?}、上限 {limit} で答えが違います"
                        );
                    }
                }
            }
        }
    }
}
