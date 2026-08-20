//! Zenith Algo: アセンブリ干渉・衝突判定エンジン (Interference / Clash Detection)
//!
//! # 以前どうだったか
//!
//! 判定は**軸並行の箱だけ**で行われていました。しかも箱は面の**頂点**からしか
//! 作っておらず、曲面が頂点より外へ膨らむぶんを見ていません。
//!
//! - 箱が重なれば `Clash`。離れた2立体でも、箱が重なれば干渉と答えます。
//!   実測: 半径5の球（原点）と隅が (3,3,3) の箱は **0.196 離れて**いますが、
//!   `Clash` で「重なり体積 8.00」と報告していました。
//! - `min_distance` は箱同士の距離で、立体同士の距離ではありません。
//! - `overlap_volume` は箱の重なりの体積で、立体の重なりではありません。
//!
//! # いまどうするか
//!
//! 箱は**篩い**にだけ使い、答えは表面で決めます。
//!
//! 1. 2つの箱が重なる範囲を格子で刻み、**両方の内側に入る点**を数える。
//!    1つでもあれば `Clash` で、数から重なりの体積が出る。
//! 2. 無ければ、表面同士の最短距離を測る。公差以内なら `Touching`、
//!    そうでなければ `Clearance`。
//!
//! 表面の**頂点**が相手の内側にあるかで見る書き方も試しましたが、足りません。
//! 平面の面は隅にしか頂点を持たないので、直交する2本の角棒が互いを貫いていても
//! どの頂点も相手の内側に来ず、`Clearance` と答えます（実測で「19.0 離れて
//! いる」と報告しました）。内側かどうかは、表面ではなく**体積を**標本する
//! ほうが素直です。
//!
//! # 何を測っていて、何を測っていないか
//!
//! 距離は**三角形に割った表面**の上で測っています。分割の細かさは
//! [`InterferenceReport::sample_divisions`] が持ち帰ります。曲面は内接
//! 多角形になるので、距離はわずかに大きめに出ます。
//!
//! 体積は格子の点を数えた見積りで、**格子の目より細い重なりは数えられません**。
//! 厳密な重なりの体積が要るなら、対応している配置では
//! [`crate::BooleanEngine`] の積のほうが桁違いに正確です。

use zenith_math::{Point3, Tolerance, Vec3};
use zenith_tess::{tessellate_solid, TessellationParams, TriangleMesh};
use zenith_topo::Solid;

/// 干渉判定の結果種別
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ClashStatus {
    /// 完全に離れている（干渉なし）
    Clearance,
    /// 表面同士が接触している（公差内で接している）
    Touching,
    /// 立体同士がめり込んでいる（干渉あり）
    Clash,
}

/// 干渉判定の詳細レポート
#[derive(Debug, Clone, PartialEq)]
pub struct InterferenceReport {
    /// 干渉状態
    pub status: ClashStatus,
    /// 表面同士の最短距離 (mm)。触れている・食い込んでいる場合は 0.0。
    ///
    /// 三角形に割った表面の間で測る。曲面は内接多角形になるので、真の距離より
    /// わずかに大きく出る。
    pub min_distance: f64,
    /// 重なりの体積 (mm^3) の見積り。`Clash` のときだけ数える。
    ///
    /// 重なりの箱を格子で刻み、両方の内側に入った点の数から出す。格子の目より
    /// 細い重なりは数えられない。
    pub overlap_volume: f64,
    /// 距離と体積を測るのに使った分割の細かさ。
    pub sample_divisions: usize,
    /// 交差判定メッセージ
    pub message: String,
}

pub struct InterferenceChecker;

impl InterferenceChecker {
    /// 表面を割る細かさの既定値。距離と体積の精度はこれで決まる。
    pub const DEFAULT_DIVISIONS: usize = 16;

    /// 2つのソリッド間の干渉・クリアランスを判定
    pub fn check(solid_a: &Solid, solid_b: &Solid, tol: &Tolerance) -> InterferenceReport {
        Self::check_with_divisions(solid_a, solid_b, Self::DEFAULT_DIVISIONS, tol)
    }

    /// 分割の細かさを指定して判定する。細かくすれば距離も体積も真の値に寄る。
    pub fn check_with_divisions(
        solid_a: &Solid,
        solid_b: &Solid,
        divisions: usize,
        tol: &Tolerance,
    ) -> InterferenceReport {
        let params = TessellationParams {
            u_divisions: divisions.max(4),
            v_divisions: divisions.max(4),
        };
        let mesh_a = tessellate_solid(solid_a, &params);
        let mesh_b = tessellate_solid(solid_b, &params);

        let Some((min_a, max_a)) = mesh_bounds(&mesh_a) else {
            return empty_report(divisions, "one of the solids has no surface to measure");
        };
        let Some((min_b, max_b)) = mesh_bounds(&mesh_b) else {
            return empty_report(divisions, "one of the solids has no surface to measure");
        };

        // 1. 箱で篩う。ここで離れていれば、立体も必ず離れている。
        let gap = box_gap(min_a, max_a, min_b, max_b);
        if gap > tol.linear {
            let distance = surface_distance(&mesh_a, &mesh_b).max(gap);
            return InterferenceReport {
                status: ClashStatus::Clearance,
                min_distance: distance,
                overlap_volume: 0.0,
                sample_divisions: divisions,
                message: format!("Solids are {distance:.6} mm apart"),
            };
        }

        // 2. 重なる箱の中を数える。両方の内側に入る点が1つでもあれば食い込み。
        let volume = overlap_volume_estimate(&mesh_a, &mesh_b, min_a, max_a, min_b, max_b);
        if volume > 0.0 {
            return InterferenceReport {
                status: ClashStatus::Clash,
                min_distance: 0.0,
                overlap_volume: volume,
                sample_divisions: divisions,
                message: format!("Solids overlap by about {volume:.6} mm^3"),
            };
        }

        // 3. 食い込んでいないなら、触れているか離れているか。
        let distance = surface_distance(&mesh_a, &mesh_b);
        if distance <= tol.linear {
            InterferenceReport {
                status: ClashStatus::Touching,
                min_distance: 0.0,
                overlap_volume: 0.0,
                sample_divisions: divisions,
                message: "Solids touch without overlapping".to_string(),
            }
        } else {
            InterferenceReport {
                status: ClashStatus::Clearance,
                min_distance: distance,
                overlap_volume: 0.0,
                sample_divisions: divisions,
                message: format!("Solids are {distance:.6} mm apart"),
            }
        }
    }
}

fn empty_report(divisions: usize, message: &str) -> InterferenceReport {
    InterferenceReport {
        status: ClashStatus::Clearance,
        min_distance: f64::INFINITY,
        overlap_volume: 0.0,
        sample_divisions: divisions,
        message: message.to_string(),
    }
}

fn mesh_bounds(mesh: &TriangleMesh) -> Option<(Point3, Point3)> {
    if mesh.positions.is_empty() {
        return None;
    }
    let mut low = Point3::new(f64::INFINITY, f64::INFINITY, f64::INFINITY);
    let mut high = Point3::new(f64::NEG_INFINITY, f64::NEG_INFINITY, f64::NEG_INFINITY);
    for point in &mesh.positions {
        low.x = low.x.min(point.x);
        low.y = low.y.min(point.y);
        low.z = low.z.min(point.z);
        high.x = high.x.max(point.x);
        high.y = high.y.max(point.y);
        high.z = high.z.max(point.z);
    }
    Some((low, high))
}

/// 2つの箱の隙間。重なっていれば 0。
fn box_gap(min_a: Point3, max_a: Point3, min_b: Point3, max_b: Point3) -> f64 {
    let axis = |low_a: f64, high_a: f64, low_b: f64, high_b: f64| {
        (low_b - high_a).max(low_a - high_b).max(0.0)
    };
    let dx = axis(min_a.x, max_a.x, min_b.x, max_b.x);
    let dy = axis(min_a.y, max_a.y, min_b.y, max_b.y);
    let dz = axis(min_a.z, max_a.z, min_b.z, max_b.z);
    (dx * dx + dy * dy + dz * dz).sqrt()
}

/// 三角形に割った2つの表面の最短距離。
///
/// 片方の頂点からもう片方の三角形までを両方向に測る。辺と辺が最も近い配置は
/// この測り方では拾えないので、分割が粗いとわずかに大きく出る。
fn surface_distance(mesh_a: &TriangleMesh, mesh_b: &TriangleMesh) -> f64 {
    // 三角形ごとに中心と半径を先に出しておき、いま分かっている最小より遠い
    // ものは中身を見ない。これが無いと、球のような細かい面では総当たりが
    // 数千万回になる。
    let spheres = |mesh: &TriangleMesh| -> Vec<(Point3, f64, [Point3; 3])> {
        mesh.indices
            .iter()
            .map(|triangle| {
                let a = mesh.positions[triangle[0] as usize];
                let b = mesh.positions[triangle[1] as usize];
                let c = mesh.positions[triangle[2] as usize];
                let centre = Point3::from((a.coords + b.coords + c.coords) / 3.0);
                let radius = (a - centre)
                    .norm()
                    .max((b - centre).norm())
                    .max((c - centre).norm());
                (centre, radius, [a, b, c])
            })
            .collect()
    };
    let triangles_a = spheres(mesh_a);
    let triangles_b = spheres(mesh_b);

    let one_way = |points: &[Point3], targets: &[(Point3, f64, [Point3; 3])], best: f64| -> f64 {
        let mut worst = best;
        for point in points {
            for (centre, radius, corners) in targets {
                if (*point - *centre).norm() - radius >= worst {
                    continue;
                }
                worst = worst.min(point_triangle_distance(
                    *point,
                    corners[0],
                    corners[1],
                    corners[2],
                ));
                if worst <= 0.0 {
                    return 0.0;
                }
            }
        }
        worst
    };
    let forward = one_way(&mesh_a.positions, &triangles_b, f64::INFINITY);
    one_way(&mesh_b.positions, &triangles_a, forward)
}

fn point_triangle_distance(point: Point3, a: Point3, b: Point3, c: Point3) -> f64 {
    // 三角形の平面へ落とし、外れていれば辺に落とす。
    let ab = b - a;
    let ac = c - a;
    let normal = ab.cross(&ac);
    let area_twice = normal.norm();
    if area_twice <= f64::EPSILON {
        return segment_distance(point, a, b).min(segment_distance(point, b, c));
    }
    let unit = normal / area_twice;
    let projected = point - unit * (point - a).dot(&unit);

    // 重心座標で内外を見る。
    let inside = |u: Vec3, v: Vec3| u.cross(&v).dot(&unit) >= 0.0;
    if inside(b - a, projected - a) && inside(c - b, projected - b) && inside(a - c, projected - c) {
        return (point - projected).norm();
    }
    segment_distance(point, a, b)
        .min(segment_distance(point, b, c))
        .min(segment_distance(point, c, a))
}

fn segment_distance(point: Point3, a: Point3, b: Point3) -> f64 {
    let direction = b - a;
    let length_squared = direction.norm_squared();
    let t = if length_squared <= f64::EPSILON {
        0.0
    } else {
        ((point - a).dot(&direction) / length_squared).clamp(0.0, 1.0)
    };
    (point - (a + direction * t)).norm()
}

/// 重なりの体積を、重なった箱の中の格子点から見積もる。
fn overlap_volume_estimate(
    mesh_a: &TriangleMesh,
    mesh_b: &TriangleMesh,
    min_a: Point3,
    max_a: Point3,
    min_b: Point3,
    max_b: Point3,
) -> f64 {
    const GRID: usize = 20;

    let low = Point3::new(
        min_a.x.max(min_b.x),
        min_a.y.max(min_b.y),
        min_a.z.max(min_b.z),
    );
    let high = Point3::new(
        max_a.x.min(max_b.x),
        max_a.y.min(max_b.y),
        max_a.z.min(max_b.z),
    );
    let size = high - low;
    if size.x <= 0.0 || size.y <= 0.0 || size.z <= 0.0 {
        return 0.0;
    }

    let cell = Vec3::new(
        size.x / GRID as f64,
        size.y / GRID as f64,
        size.z / GRID as f64,
    );
    let mut inside_both = 0usize;
    for i in 0..GRID {
        for j in 0..GRID {
            for k in 0..GRID {
                let sample = Point3::new(
                    low.x + cell.x * (i as f64 + 0.5),
                    low.y + cell.y * (j as f64 + 0.5),
                    low.z + cell.z * (k as f64 + 0.5),
                );
                if crate::BooleanEngine::is_point_inside_mesh(sample, mesh_a)
                    && crate::BooleanEngine::is_point_inside_mesh(sample, mesh_b)
                {
                    inside_both += 1;
                }
            }
        }
    }

    inside_both as f64 * cell.x * cell.y * cell.z
}
