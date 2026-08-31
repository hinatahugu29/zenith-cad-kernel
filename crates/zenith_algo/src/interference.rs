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
//! 2. 格子で捕まらない薄い食い込みを、**表面の点がもう一方の内側にどれだけ
//!    深く入っているか**で拾う。深さを見るのは、面を共有して触れているだけの
//!    立体を食い込みと言わないため（面の上の点は射線の偶奇では内とも外とも出る）。
//! 3. 無ければ、表面同士の最短距離で `Touching` か `Clearance` を決める。
//!
//! 表面の**頂点だけ**を見る書き方も試しましたが、足りません。平面の面は隅に
//! しか頂点を持たないので、直交する2本の角棒が互いを貫いていてもどの頂点も
//! 相手の内側に来ず、`Clearance` と答えます（実測で「19.0 離れている」と
//! 報告しました）。格子と表面の点は、どちらか一方では足りず、両方要ります。
//!
//! # 何を測っていて、何を測っていないか
//!
//! 距離は [`crate::DistanceEngine`] が返す値で、最近接点を B-Rep の面まで
//! 詰めてあります。**表示の刻みでは動きません**。以前はメッシュの弦の上で
//! 測っていたので、曲面ではわずかに大きめに出ていました。
//!
//! 体積は格子の点を数えた見積りで、**格子の目より細い重なりは数えられません**。
//! 数えられなくても食い込みは検出します（メッセージにその旨が出ます）。
//! 厳密な重なりの体積が要るなら、対応している配置では
//! [`crate::BooleanEngine`] の積のほうが桁違いに正確です。
//!
//! # 実測
//!
//! `cargo run -p zenith_algo --example interference_depth_probe`。食い込み量を
//! 5 mm から 0.001 mm まで減らした 21 配置（直方体どうし・板とピン・板と球）
//! すべてで `Clash`、隙間のある5配置すべてで報告距離が閉じた式と一致。
//! 従来は板に 0.01 mm 押し込んだ球が「隙間 0.010」と報告されていました。

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
    /// [`crate::DistanceEngine`] が返す値で、最近接点は B-Rep の面まで詰めて
    /// ある。表示の刻みでは動かない。
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
        //    離れていると分かっても、**返す距離は表面の値**でなければならない。
        //    メッシュの弦のままでは曲面で刻みの誤差が乗るので、B-Rep の面まで
        //    詰めた値を使う。
        let gap = box_gap(min_a, max_a, min_b, max_b);
        if gap > tol.linear {
            let distance = crate::DistanceEngine::compute_min_distance(solid_a, solid_b, tol)
                .min_distance
                .max(gap);
            return InterferenceReport {
                status: ClashStatus::Clearance,
                min_distance: distance,
                overlap_volume: 0.0,
                sample_divisions: divisions,
                message: format!("Solids are {distance:.6} mm apart"),
            };
        }

        // 2. 重なる箱の中を数える。両方の内側に入る点が1つでもあれば食い込み。
        //
        // 格子だけでは、格子の目より薄い食い込みが落ちる。三角形どうしの距離は
        // 辺と辺・頂点と面しか見ないので、**交差している**三角形の組では正の値に
        // なり、浅い食い込みは距離からも分からない。板に 0.01 mm 押し込んだ球が
        // 「隙間 0.010」と報告されていた。見落としは製造まで流れる。
        // 表面の点がもう一方の内側にあるかでも見る。
        let volume = overlap_volume_estimate(&mesh_a, &mesh_b, min_a, max_a, min_b, max_b);
        if volume > 0.0 || crate::distance::overlaps(&mesh_a, &mesh_b, tol.linear) {
            return InterferenceReport {
                status: ClashStatus::Clash,
                min_distance: 0.0,
                overlap_volume: volume,
                sample_divisions: divisions,
                message: if volume > 0.0 {
                    format!("Solids overlap by about {volume:.6} mm^3")
                } else {
                    "Solids overlap, too thinly for the sampling grid to measure".to_string()
                },
            };
        }

        // 3. 食い込んでいないなら、触れているか離れているか。距離は B-Rep の面
        //    まで詰めた値を使う（メッシュの弦のままでは曲面で刻みの誤差が乗る）。
        let dist_res = crate::DistanceEngine::compute_min_distance(solid_a, solid_b, tol);
        let distance = dist_res.min_distance;
        if distance <= tol.linear {
            // 最近傍点が相手ソリッドの内部（深さ > tol.linear）に入っていれば浅い食い込み（Clash）
            let in_b =
                crate::BooleanEngine::is_point_inside_mesh(dist_res.closest_point_a, &mesh_b)
                    && distance_to_mesh_point(dist_res.closest_point_a, &mesh_b) > tol.linear;
            let in_a =
                crate::BooleanEngine::is_point_inside_mesh(dist_res.closest_point_b, &mesh_a)
                    && distance_to_mesh_point(dist_res.closest_point_b, &mesh_a) > tol.linear;

            if in_b || in_a {
                InterferenceReport {
                    status: ClashStatus::Clash,
                    min_distance: 0.0,
                    overlap_volume: volume,
                    sample_divisions: divisions,
                    message: "Solids overlap, detected at surface projection point".to_string(),
                }
            } else {
                InterferenceReport {
                    status: ClashStatus::Touching,
                    min_distance: 0.0,
                    overlap_volume: 0.0,
                    sample_divisions: divisions,
                    message: "Solids touch without overlapping".to_string(),
                }
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

    /// 高速篩いと厳密 B-Rep ブーリアン積を組み合わせたハイブリッド干渉解析
    ///
    /// 食い込み（`Clash`）が検出された場合、可能であれば [`crate::BooleanEngine`] の
    /// 厳密な積（Intersection）を計算して、真の干渉体積を算出します。
    pub fn check_exact(
        solid_a: &Solid,
        solid_b: &Solid,
        tol: &Tolerance,
    ) -> (InterferenceReport, Option<Solid>) {
        let mut report = Self::check(solid_a, solid_b, tol);
        if report.status != ClashStatus::Clash {
            return (report, None);
        }

        if let Ok(intersection_solid) = crate::BooleanEngine::boolean_solids_exact(
            solid_a,
            solid_b,
            crate::BooleanOpType::Intersection,
            tol,
        ) {
            let params = TessellationParams::default();
            // 体積しか読まないので、慣性まで積まない口を使います（4-156）。
            let overlap =
                crate::MassCalculator::compute_volume_from_brep(&intersection_solid, &params);
            if overlap > 0.0 {
                report.overlap_volume = overlap;
                report.message = format!(
                    "Solids overlap by exact {:.6} mm^3 (B-Rep intersection)",
                    overlap
                );
                return (report, Some(intersection_solid));
            }
        }

        (report, None)
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

/// 点からメッシュ表面までの最短距離
fn distance_to_mesh_point(point: Point3, mesh: &TriangleMesh) -> f64 {
    let mut best = f64::INFINITY;
    for triangle in &mesh.indices {
        let corners = [
            mesh.positions[triangle[0] as usize],
            mesh.positions[triangle[1] as usize],
            mesh.positions[triangle[2] as usize],
        ];
        let p0 = corners[0];
        let p1 = corners[1];
        let p2 = corners[2];
        let v0 = p1 - p0;
        let v1 = p2 - p0;
        let v2 = point - p0;
        let d00 = v0.dot(&v0);
        let d01 = v0.dot(&v1);
        let d11 = v1.dot(&v1);
        let d20 = v2.dot(&v0);
        let d21 = v2.dot(&v1);
        let denom = d00 * d11 - d01 * d01;
        let (u, v) = if denom.abs() > 1e-18 {
            (
                (d11 * d20 - d01 * d21) / denom,
                (d00 * d21 - d01 * d20) / denom,
            )
        } else {
            (0.0, 0.0)
        };
        let closest = if u >= 0.0 && v >= 0.0 && u + v <= 1.0 {
            p0 + v0 * u + v1 * v
        } else {
            let seg = |a: Point3, b: Point3| {
                let ab = b - a;
                let t = ((point - a).dot(&ab) / ab.dot(&ab)).clamp(0.0, 1.0);
                a + ab * t
            };
            let c0 = seg(p0, p1);
            let c1 = seg(p1, p2);
            let c2 = seg(p2, p0);
            let d0 = (point - c0).norm();
            let d1 = (point - c1).norm();
            let d2 = (point - c2).norm();
            if d0 <= d1 && d0 <= d2 {
                c0
            } else if d1 <= d2 {
                c1
            } else {
                c2
            }
        };
        let distance = (point - closest).norm();
        if distance < best {
            best = distance;
        }
    }
    best
}
