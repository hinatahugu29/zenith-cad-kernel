//! 他カーネルが書いた立体への、距離と内外。
//!
//! **ここは長らく測っていませんでした。** 距離のプローブ（`distance_probe`）の
//! 7件はすべて自前ビルダーの立体です。自前の立体は必ず境界ワイヤを持つので、
//! 「境界ワイヤを持たない全周1枚の面」を1件も踏みません。読んだ球や円柱は
//! まさにそれで、**面を1枚も持たない立体**として扱われていました（4-68）。
//!
//! 測るものは3つです。どれも**閉じた式**が相手で、こちらの実装とは無関係に
//! 決まります。
//!
//! 1. 立体の境界への最短距離（`nearest_boundary_projection`）
//! 2. 点が内か外か（`exact_inside`）
//! 3. 公開 API の最短距離（`DistanceEngine::compute_min_distance`）
//!
//! 検体の置き方が式の前提と合っていることを、境界箱で**先に**確かめます。
//! ここが崩れたら式のほうが誤りなので、そのときは赤にします。
//!
//! ```bash
//! cargo run --release -p zenith_algo --example foreign_distance_probe
//! ```

use std::path::PathBuf;

use zenith_algo::{
    exact_inside, nearest_boundary_projection, BrepTransform, DistanceEngine, PrimitiveBuilder,
};
use zenith_io::StepImporter;
use zenith_math::{Point3, Tolerance, Vec3};
use zenith_tess::{tessellate_solid, TessellationParams};
use zenith_topo::Solid;

/// 閉じた式が返すもの。負なら内側、正なら外側、絶対値が境界への距離。
type Signed = fn(Point3) -> f64;

struct Subject {
    name: &'static str,
    file: &'static str,
    /// 期待する境界箱。式の前提が崩れていないかを先に見る。
    bounds: (Point3, Point3),
    signed: Signed,
    points: Vec<Point3>,
}

fn fixture(name: &str) -> PathBuf {
    PathBuf::from(concat!(env!("CARGO_MANIFEST_DIR"), "/tests/fixtures"))
        .join(format!("occ_reference_{name}.step"))
}

fn read(name: &str) -> Option<Solid> {
    StepImporter::import_solids_from_file(&fixture(name))
        .ok()?
        .into_iter()
        .next()
}

fn mesh_bounds(solid: &Solid) -> (Point3, Point3) {
    let mesh = tessellate_solid(
        solid,
        &TessellationParams {
            u_divisions: 64,
            v_divisions: 64,
        },
    );
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
    (low, high)
}

/// 球 r10、原点。
fn sphere_signed(point: Point3) -> f64 {
    point.coords.norm() - 10.0
}

/// 円柱 r10、z は 0 から 40。上下に平らな蓋がある。
fn cylinder_signed(point: Point3) -> f64 {
    let radial = (point.x * point.x + point.y * point.y).sqrt() - 10.0;
    let axial = (0.0 - point.z).max(point.z - 40.0);
    if radial <= 0.0 && axial <= 0.0 {
        // 内側。いちばん近い面までの距離に負号を付ける。
        -(-radial).min(-axial)
    } else {
        (radial.max(0.0).powi(2) + axial.max(0.0).powi(2)).sqrt()
    }
}

/// トーラス R12 / r4、軸は z、原点。
fn torus_signed(point: Point3) -> f64 {
    let radial = (point.x * point.x + point.y * point.y).sqrt() - 12.0;
    (radial * radial + point.z * point.z).sqrt() - 4.0
}

/// 円錐 r10 → 0、高さ 20、底は z = 0。軸は z。
///
/// 断面（rho, z）で見ると、境界は底の線分 (0,0)-(10,0) と、母線の線分
/// (10,0)-(0,20) の2本だけです。軸は境界ではありません。
fn cone_full_signed(point: Point3) -> f64 {
    let rho = (point.x * point.x + point.y * point.y).sqrt();
    let z = point.z;

    let distance_to_segment = |ax: f64, ay: f64, bx: f64, by: f64| -> f64 {
        let (dx, dy) = (bx - ax, by - ay);
        let length_squared = dx * dx + dy * dy;
        let t = if length_squared <= 0.0 {
            0.0
        } else {
            (((rho - ax) * dx + (z - ay) * dy) / length_squared).clamp(0.0, 1.0)
        };
        let (px, py) = (ax + dx * t, ay + dy * t);
        ((rho - px).powi(2) + (z - py).powi(2)).sqrt()
    };

    let distance =
        distance_to_segment(0.0, 0.0, 10.0, 0.0).min(distance_to_segment(10.0, 0.0, 0.0, 20.0));
    let inside = (0.0..=20.0).contains(&z) && rho <= 10.0 * (1.0 - z / 20.0);
    if inside {
        -distance
    } else {
        distance
    }
}

fn subjects() -> Vec<Subject> {
    vec![
        Subject {
            name: "sphere r10",
            file: "sphere",
            bounds: (
                Point3::new(-10.0, -10.0, -10.0),
                Point3::new(10.0, 10.0, 10.0),
            ),
            signed: sphere_signed,
            points: vec![
                Point3::new(0.0, 0.0, 0.0),
                Point3::new(20.0, 0.0, 0.0),
                Point3::new(3.0, 4.0, 1.0),
                Point3::new(-7.0, 2.0, -5.0),
                // **軸の上。** 最近点がちょうど極に来るので、法線が決まらず
                // 「この面には足が無い」となっていました（読んだ球の中心線）。
                Point3::new(0.0, 0.0, 25.0),
                Point3::new(0.0, 0.0, -25.0),
                Point3::new(0.0, 0.0, 5.0),
                Point3::new(0.0, 0.0, -5.0),
                // 面のすぐ内と、すぐ外。**メッシュでは割れる幅**。
                Point3::new(9.9998, 0.0, 0.0),
                Point3::new(10.0002, 0.0, 0.0),
            ],
        },
        Subject {
            name: "cylinder r10 h40",
            file: "cylinder",
            bounds: (
                Point3::new(-10.0, -10.0, 0.0),
                Point3::new(10.0, 10.0, 40.0),
            ),
            signed: cylinder_signed,
            points: vec![
                Point3::new(0.0, 0.0, 20.0),
                Point3::new(30.0, 0.0, 20.0),
                // 蓋の外、真上。
                Point3::new(0.0, 0.0, 50.0),
                // 蓋と側面が出会う稜の外。足は稜の上にある。
                Point3::new(20.0, 0.0, 50.0),
                Point3::new(0.0, 20.0, -10.0),
                Point3::new(5.0, 5.0, 39.0),
                // 軸の上。円柱の蓋は平面なので、極は踏まない。
                Point3::new(0.0, 0.0, 10.0),
                Point3::new(9.9998, 0.0, 20.0),
                Point3::new(10.0002, 0.0, 20.0),
            ],
        },
        Subject {
            name: "cone r10 h20",
            file: "cone_full",
            bounds: (
                Point3::new(-10.0, -10.0, 0.0),
                Point3::new(10.0, 10.0, 20.0),
            ),
            signed: cone_full_signed,
            points: vec![
                // **頂点の真上。** いちばん近いのは頂点そのもの。頂点は
                // 母線の面の退化点で、まわりから寄せても法線が定まりません。
                Point3::new(0.0, 0.0, 30.0),
                Point3::new(0.0, 0.0, 22.0),
                // 軸の上、中ほど。いちばん近いのは母線。
                Point3::new(0.0, 0.0, 10.0),
                Point3::new(0.0, 0.0, 1.0),
                Point3::new(20.0, 0.0, 5.0),
                Point3::new(3.0, 2.0, 4.0),
                Point3::new(0.0, 0.0, -6.0),
            ],
        },
        Subject {
            name: "torus R12 r4",
            file: "torus",
            bounds: (
                Point3::new(-16.0, -16.0, -4.0),
                Point3::new(16.0, 16.0, 4.0),
            ),
            signed: torus_signed,
            points: vec![
                // 穴の真ん中。いちばん近いのは内側の腹。
                Point3::new(0.0, 0.0, 0.0),
                Point3::new(12.0, 0.0, 0.0),
                Point3::new(30.0, 0.0, 0.0),
                Point3::new(0.0, 12.0, 10.0),
                Point3::new(8.0, 8.0, 2.0),
                Point3::new(15.9998, 0.0, 0.0),
                Point3::new(16.0002, 0.0, 0.0),
            ],
        },
    ]
}

fn main() {
    let tol = Tolerance::default();
    let mut failures = 0usize;
    let mut checks = 0usize;
    let mut worst: f64 = 0.0;

    println!("他カーネルが書いた立体への距離と内外（相手は閉じた式）");
    println!();

    for subject in subjects() {
        let Some(solid) = read(subject.file) else {
            println!("{:<18} 読めませんでした", subject.name);
            failures += 1;
            continue;
        };

        // 式の前提を先に確かめる。置き方が違えば、式のほうが誤り。
        let (low, high) = mesh_bounds(&solid);
        let (want_low, want_high) = subject.bounds;
        let slack = 0.05;
        let placed = (low.x - want_low.x).abs() < slack
            && (low.y - want_low.y).abs() < slack
            && (low.z - want_low.z).abs() < slack
            && (high.x - want_high.x).abs() < slack
            && (high.y - want_high.y).abs() < slack
            && (high.z - want_high.z).abs() < slack;
        if !placed {
            println!(
                "{:<18} 置き方が式の前提と違います: {:?} .. {:?}",
                subject.name, low, high
            );
            failures += 1;
            continue;
        }

        println!("{} — 面 {} 枚", subject.name, solid.outer_shell.faces.len());
        println!(
            "  {:>30} {:>13} {:>14} {:>11}  {:>5} {:>6}",
            "point", "closed form", "measured", "error", "in?", "closed"
        );

        for point in &subject.points {
            let want = (subject.signed)(*point);
            let want_inside = want < 0.0;

            let measured = nearest_boundary_projection(*point, &solid).map(|p| p.distance);
            let inside = exact_inside(*point, &solid, &tol);

            checks += 1;
            let (shown, error, ok_distance) = match measured {
                Some(distance) => {
                    let error = (distance - want.abs()).abs();
                    (format!("{distance:.9}"), error, error < 1e-6)
                }
                None => ("no foot".to_string(), f64::INFINITY, false),
            };
            let ok_side = inside == Some(want_inside);
            if error.is_finite() {
                worst = worst.max(error);
            }
            if !ok_distance || !ok_side {
                failures += 1;
            }

            println!(
                "  ({:>8.4} {:>8.4} {:>8.4}) {:>13.9} {:>14} {:>11.2e}  {:>5} {:>6}  {}",
                point.x,
                point.y,
                point.z,
                want.abs(),
                shown,
                error,
                match inside {
                    Some(true) => "in",
                    Some(false) => "out",
                    None => "?",
                },
                if want_inside { "in" } else { "out" },
                if ok_distance && ok_side { "ok" } else { "MISS" }
            );
        }
        println!();
    }

    // 公開 API も測ります。読んだ立体と、そこから既知の隙間だけ離した小球。
    println!("公開 API（DistanceEngine::compute_min_distance）");
    println!(
        "  {:<30} {:>10} {:>14} {:>11}",
        "case", "expected", "measured", "error"
    );
    for (name, gap) in [("sphere", 5.0f64), ("sphere", 0.25), ("cylinder", 5.0)] {
        let Some(solid) = read(name) else {
            failures += 1;
            continue;
        };
        // 小球（r1）を、境界から `gap` だけ離した所に置く。
        // 球は原点中心 r10、円柱は z 軸まわり r10。どちらも +x 方向へ。
        let ball = BrepTransform::translate_solid(
            &PrimitiveBuilder::make_sphere(1.0).expect("ball"),
            Vec3::new(
                10.0 + gap + 1.0,
                0.0,
                if name == "cylinder" { 20.0 } else { 0.0 },
            ),
        );
        let measured = DistanceEngine::compute_min_distance(&solid, &ball, &tol).min_distance;
        checks += 1;
        let (shown, error) = (format!("{measured:.9}"), (measured - gap).abs());
        // 小球も NURBS なので、こちらは弦の刻みぶんが乗る。1e-3 で見る。
        let ok = error < 1e-3;
        if !ok {
            failures += 1;
        }
        if error.is_finite() {
            worst = worst.max(error);
        }
        println!(
            "  {:<30} {:>10.6} {:>14} {:>11.2e}  {}",
            format!("{name} and a ball {gap} away"),
            gap,
            shown,
            error,
            if ok { "ok" } else { "MISS" }
        );
    }

    println!();
    println!("{}", "-".repeat(90));
    println!(
        "{} checks, {} miss(es), worst error {:.2e}",
        checks, failures, worst
    );
    if failures > 0 {
        std::process::exit(1);
    }
    println!("every distance and every side lands on the closed form");
}
