//! 他カーネルが書いた立体の断面。
//!
//! 断面のプローブ（`slice_probe` / `slice_robustness_probe`）は、いずれも
//! **自前ビルダーの立体**を切っています。読んだ立体を切ったことは一度も
//! ありません。読んだ立体は全周1枚の面で来るので、刻み方も継ぎ目の位置も
//! 自前のものとは違います。
//!
//! 相手は**閉じた式**です。球・円柱・円錐・トーラスの断面は、面積も周長も
//! 手で書けます。トーラスの水平断面は穴のある領域になるので、ループが2本
//! 出ること（符号つきで引かれること）も一緒に見られます。
//!
//! **1つの分割数で合っていても、それは合っている証拠になりません**（5章）。
//! 64・128・256 の3つで測り、**細かくすると誤差が縮むか**を見ます。縮まない
//! ものは、たまたま合っているだけです。
//!
//! ```bash
//! cargo run --release -p zenith_algo --example foreign_slice_probe
//! ```

use std::f64::consts::PI;
use std::path::PathBuf;

use zenith_algo::SectionSlicer;
use zenith_io::StepImporter;
use zenith_math::{Point3, Tolerance, Vec3};
use zenith_tess::TessellationParams;
use zenith_topo::Solid;

struct Case {
    subject: &'static str,
    file: &'static str,
    what: &'static str,
    origin: Point3,
    normal: Vec3,
    /// 解析解の面積。穴は引かれた後の値。
    area: f64,
    /// 解析解の周長。穴の縁も足した値。
    perimeter: f64,
    /// 出てくるべき閉ループの本数。
    loops: usize,
    /// 平面だけで決まる断面か。そうならどの分割数でも厳密であるべき。
    exact: bool,
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

fn cases() -> Vec<Case> {
    // 球 r10。中心から d のところで切ると、半径 sqrt(100 - d^2) の円。
    let sphere_circle = |d: f64| {
        let r = (100.0 - d * d).sqrt();
        (PI * r * r, 2.0 * PI * r)
    };
    let (a0, p0) = sphere_circle(0.0);
    let (a6, p6) = sphere_circle(6.0);
    // 傾けた平面。中心からの距離だけで決まるので、向きを変えても同じ円。
    let (at, pt) = sphere_circle(5.0);

    // 円錐 r10 → 0、高さ 20。z = h では半径 10 (1 - h/20)。
    let cone_circle = |h: f64| {
        let r = 10.0 * (1.0 - h / 20.0);
        (PI * r * r, 2.0 * PI * r)
    };
    let (ac, pc) = cone_circle(10.0);

    // トーラス R12 r4。z = h では、半径 12 ± sqrt(16 - h^2) の円環。
    let torus_annulus = |h: f64| {
        let half = (16.0 - h * h).sqrt();
        let (outer, inner) = (12.0 + half, 12.0 - half);
        (
            PI * (outer * outer - inner * inner),
            2.0 * PI * (outer + inner),
        )
    };
    let (at0, pt0) = torus_annulus(0.0);
    let (at2, pt2) = torus_annulus(2.0);

    vec![
        Case {
            subject: "sphere r10",
            file: "sphere",
            what: "z = 0 (great circle)",
            origin: Point3::new(0.0, 0.0, 0.0),
            normal: Vec3::new(0.0, 0.0, 1.0),
            area: a0,
            perimeter: p0,
            loops: 1,
            exact: false,
        },
        Case {
            subject: "sphere r10",
            file: "sphere",
            what: "z = 6",
            origin: Point3::new(0.0, 0.0, 6.0),
            normal: Vec3::new(0.0, 0.0, 1.0),
            area: a6,
            perimeter: p6,
            loops: 1,
            exact: false,
        },
        Case {
            subject: "sphere r10",
            file: "sphere",
            what: "tilted, 5 from the centre",
            // 法線 (1,1,1)/sqrt(3) の向きに 5 だけ進んだ所を通る平面。
            origin: Point3::new(5.0 / 3f64.sqrt(), 5.0 / 3f64.sqrt(), 5.0 / 3f64.sqrt()),
            normal: Vec3::new(1.0, 1.0, 1.0),
            area: at,
            perimeter: pt,
            loops: 1,
            exact: false,
        },
        Case {
            subject: "cylinder r10 h40",
            file: "cylinder",
            what: "z = 20 (across the axis)",
            origin: Point3::new(0.0, 0.0, 20.0),
            normal: Vec3::new(0.0, 0.0, 1.0),
            area: 100.0 * PI,
            perimeter: 20.0 * PI,
            loops: 1,
            exact: false,
        },
        Case {
            subject: "cylinder r10 h40",
            file: "cylinder",
            what: "x = 0 (along the axis)",
            origin: Point3::new(0.0, 0.0, 20.0),
            normal: Vec3::new(1.0, 0.0, 0.0),
            // 20 x 40 の長方形。境界はすべて直線なので、刻みによらず厳密。
            area: 800.0,
            perimeter: 120.0,
            loops: 1,
            exact: true,
        },
        Case {
            subject: "cone r10 h20",
            file: "cone_full",
            what: "z = 10",
            origin: Point3::new(0.0, 0.0, 10.0),
            normal: Vec3::new(0.0, 0.0, 1.0),
            area: ac,
            perimeter: pc,
            loops: 1,
            exact: false,
        },
        Case {
            subject: "torus R12 r4",
            file: "torus",
            what: "z = 0 (an annulus)",
            origin: Point3::new(0.0, 0.0, 0.0),
            normal: Vec3::new(0.0, 0.0, 1.0),
            area: at0,
            perimeter: pt0,
            loops: 2,
            exact: false,
        },
        Case {
            subject: "torus R12 r4",
            file: "torus",
            what: "z = 2 (a narrower annulus)",
            origin: Point3::new(0.0, 0.0, 2.0),
            normal: Vec3::new(0.0, 0.0, 1.0),
            area: at2,
            perimeter: pt2,
            loops: 2,
            exact: false,
        },
    ]
}

fn divisions(n: usize) -> TessellationParams {
    TessellationParams {
        u_divisions: n,
        v_divisions: n,
    }
}

fn main() {
    let tol = Tolerance::default();
    let mut failures = 0usize;
    let mut checks = 0usize;
    let mut worst: f64 = 0.0;

    println!("他カーネルが書いた立体の断面（相手は閉じた式）");
    println!();
    println!(
        "{:<18} {:<28} {:>7} {:>15} {:>15} {:>11} {:>11}  {}",
        "subject", "section", "div", "area", "closed form", "rel error", "perimeter", "loops"
    );
    println!("{}", "-".repeat(126));

    for case in cases() {
        let Some(solid) = read(case.file) else {
            println!("{:<18} 読めませんでした", case.subject);
            failures += 1;
            continue;
        };

        let mut errors = Vec::new();
        for n in [64usize, 128, 256] {
            let result = SectionSlicer::slice_solid_with_tessellation(
                &solid,
                case.origin,
                case.normal,
                &tol,
                &divisions(n),
            );
            checks += 1;
            let Ok(section) = result else {
                println!(
                    "{:<18} {:<28} {n:>7}  切れませんでした: {}",
                    case.subject,
                    case.what,
                    result.unwrap_err().chars().take(50).collect::<String>()
                );
                failures += 1;
                errors.push(f64::INFINITY);
                continue;
            };

            let relative = (section.total_area - case.area).abs() / case.area;
            let perimeter_relative =
                (section.total_perimeter - case.perimeter).abs() / case.perimeter;
            errors.push(relative);
            worst = worst.max(relative);

            let loops_ok = section.signed_loop_areas.len() == case.loops;
            // 平面だけで決まる断面は、どの刻みでも厳密。曲面を含むものは、
            // ここでは緩く見て、**縮むか**を下で見ます。
            let bound = if case.exact { 1e-12 } else { 1e-3 };
            let ok = relative < bound && perimeter_relative < bound && loops_ok;
            if !ok {
                failures += 1;
            }

            println!(
                "{:<18} {:<28} {n:>7} {:>15.6} {:>15.6} {:>11.2e} {:>11.6} {:>4} / {}  {}",
                if n == 64 { case.subject } else { "" },
                if n == 64 { case.what } else { "" },
                section.total_area,
                case.area,
                relative,
                section.total_perimeter,
                section.signed_loop_areas.len(),
                case.loops,
                if ok { "ok" } else { "MISS" }
            );
        }

        // **細かくすると縮むか。** 1つの刻みで合っていても、それは合っている
        // 証拠になりません。厳密なはずのものは、そもそも動いてはいけません。
        let verdict = if case.exact {
            let moved = errors
                .iter()
                .all(|error| error.is_finite() && *error < 1e-12);
            if moved {
                "刻みによらず厳密".to_string()
            } else {
                failures += 1;
                "刻みで動いた（平面だけの断面なので、動いてはいけない）".to_string()
            }
        } else if errors.iter().all(|error| error.is_finite()) {
            // 2倍にして縮んでいるか。同じ桁で止まっていたら、収束していない。
            let shrinking = errors[1] < errors[0] * 0.9 && errors[2] < errors[1] * 0.9;
            let order = if errors[2] > 0.0 {
                (errors[0] / errors[2]).log2() / 2.0
            } else {
                f64::INFINITY
            };
            if shrinking {
                format!("縮んでいる（見かけの次数 {order:.1}）")
            } else {
                failures += 1;
                format!(
                    "縮んでいない（{:.2e} -> {:.2e} -> {:.2e}）",
                    errors[0], errors[1], errors[2]
                )
            }
        } else {
            "測れませんでした".to_string()
        };
        println!("{:>18}   {}", "", verdict);
        println!();
    }

    println!("{}", "-".repeat(126));
    println!(
        "{} checks, {} miss(es), worst relative area error {:.2e}",
        checks, failures, worst
    );
    if failures > 0 {
        std::process::exit(1);
    }
    println!("every section lands on the closed form and converges with the tessellation");
}
