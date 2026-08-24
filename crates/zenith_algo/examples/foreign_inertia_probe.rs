//! 他カーネルが書いた立体の、重心と慣性。
//!
//! `inertia_probe` の検体は**すべて自前ビルダー**です。読んだ立体で重心と
//! 慣性を測ったことは一度もありません。体積は `foreign_reexport` が解析解と
//! 突き合わせていますが、**体積が合っていても重心が合っているとは限らず**、
//! 慣性はさらに一段高い次数の積分です。
//!
//! 今日、`inspect_face` が開いた面1枚に**体積の重心**を返していたのが出ました
//! （上蓋が (0,0,30)、真値 (0,0,40)）。同じ種類の取り違えが、立体の側に
//! 残っていないかを見ます。
//!
//! 規約は `inertia_probe` と揃えます——**密度 1**（質量 = 体積）、慣性は
//! **原点まわり**の $(I_{xx}, I_{yy}, I_{zz})$。
//!
//! 検体の置き方が式の前提と合っていることを、境界箱で**先に**確かめます。
//! 置き方が違えば式のほうが誤りなので、そのときは赤にします。
//!
//! ```bash
//! cargo run --release -p zenith_algo --example foreign_inertia_probe
//! ```

use std::f64::consts::PI;
use std::path::PathBuf;

use zenith_algo::MassCalculator;
use zenith_io::StepImporter;
use zenith_math::{Point3, Vec3};
use zenith_tess::{tessellate_solid, TessellationParams};
use zenith_topo::Solid;

struct Subject {
    name: &'static str,
    file: &'static str,
    /// 期待する境界箱。式の前提が崩れていないかを先に見る。
    bounds: (Point3, Point3),
    volume: f64,
    centroid: Point3,
    /// 原点まわりの (Ixx, Iyy, Izz)、密度 1。
    inertia_about_origin: Vec3,
}

fn read(name: &str) -> Option<Solid> {
    let path = PathBuf::from(concat!(env!("CARGO_MANIFEST_DIR"), "/tests/fixtures"))
        .join(format!("occ_reference_{name}.step"));
    StepImporter::import_solids_from_file(&path)
        .ok()?
        .into_iter()
        .next()
}

fn subjects() -> Vec<Subject> {
    let mut out = Vec::new();

    // 球 r10、原点中心。原点まわり Ixx = Iyy = Izz = (2/5) V r^2。
    let r = 10.0f64;
    let v = 4.0 / 3.0 * PI * r.powi(3);
    out.push(Subject {
        name: "sphere r10",
        file: "sphere",
        bounds: (
            Point3::new(-10.0, -10.0, -10.0),
            Point3::new(10.0, 10.0, 10.0),
        ),
        volume: v,
        centroid: Point3::new(0.0, 0.0, 0.0),
        inertia_about_origin: Vec3::new(0.4 * v * r * r, 0.4 * v * r * r, 0.4 * v * r * r),
    });

    // 円柱 r10 h40、底面中心が原点、軸 +Z。
    // 原点まわり Izz = V r^2 / 2、Ixx = Iyy = V (r^2 / 4 + h^2 / 3)。
    let (r, h) = (10.0f64, 40.0f64);
    let v = PI * r * r * h;
    let side = v * (r * r * 0.25 + h * h / 3.0);
    out.push(Subject {
        name: "cylinder r10 h40",
        file: "cylinder",
        bounds: (
            Point3::new(-10.0, -10.0, 0.0),
            Point3::new(10.0, 10.0, 40.0),
        ),
        volume: v,
        centroid: Point3::new(0.0, 0.0, h * 0.5),
        inertia_about_origin: Vec3::new(side, side, v * r * r * 0.5),
    });

    // 円錐 r10 → 0、高さ 20、底面中心が原点、軸 +Z。
    //
    // 原点（底面中心）まわり:
    //   Izz = (3/10) V r^2
    //   Ixx = Iyy = V (3 r^2 / 20 + h^2 / 10)
    //
    // 高さ z での半径は r (1 - z/h)。∫y^2 dV = π r^4 h / 20、
    // ∫z^2 dV = π r^2 h^3 / 30 で、π r^2 h = 3V を使うと上の形になる。
    let (r, h) = (10.0f64, 20.0f64);
    let v = PI * r * r * h / 3.0;
    let side = v * (3.0 * r * r / 20.0 + h * h / 10.0);
    out.push(Subject {
        name: "cone r10 h20",
        file: "cone_full",
        bounds: (
            Point3::new(-10.0, -10.0, 0.0),
            Point3::new(10.0, 10.0, 20.0),
        ),
        volume: v,
        centroid: Point3::new(0.0, 0.0, h * 0.25),
        inertia_about_origin: Vec3::new(side, side, 0.3 * v * r * r),
    });

    // トーラス R12 r4、原点中心、軸 z。
    // 中心まわり Izz = V (R^2 + 3 r^2 / 4)、Ixx = Iyy = V (5 r^2 / 8 + R^2 / 2)。
    let (big, small) = (12.0f64, 4.0f64);
    let v = 2.0 * PI * PI * big * small * small;
    out.push(Subject {
        name: "torus R12 r4",
        file: "torus",
        bounds: (
            Point3::new(-16.0, -16.0, -4.0),
            Point3::new(16.0, 16.0, 4.0),
        ),
        volume: v,
        centroid: Point3::new(0.0, 0.0, 0.0),
        inertia_about_origin: Vec3::new(
            v * (5.0 * small * small / 8.0 + big * big / 2.0),
            v * (5.0 * small * small / 8.0 + big * big / 2.0),
            v * (big * big + 3.0 * small * small / 4.0),
        ),
    });

    out
}

fn mesh_bounds(solid: &Solid, params: &TessellationParams) -> (Point3, Point3) {
    let mesh = tessellate_solid(solid, params);
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

fn main() {
    // 曲面を含むので、刻みぶんは残ります。**1つの刻みで合っていても証拠に
    // なりません**（5章）ので、2つで測って縮むかを見ます。
    let coarse = TessellationParams {
        u_divisions: 64,
        v_divisions: 64,
    };
    let fine = TessellationParams {
        u_divisions: 128,
        v_divisions: 128,
    };

    let mut failures = 0usize;
    let mut checks = 0usize;
    let mut worst: f64 = 0.0;

    println!("他カーネルが書いた立体の重心と慣性（密度1、原点まわり、相手は閉じた式）");
    println!();

    for subject in subjects() {
        let Some(solid) = read(subject.file) else {
            println!("{:<18} 読めませんでした", subject.name);
            failures += 1;
            continue;
        };

        let (low, high) = mesh_bounds(&solid, &coarse);
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

        println!("{}", subject.name);
        println!(
            "  {:<26} {:>18} {:>18} {:>11} {:>11}",
            "quantity", "measured (128)", "closed form", "rel (64)", "rel (128)"
        );

        // 借用を跨がないように、判定は関数で出して数え上げは呼び手が持ちます。
        fn row(label: &str, coarse_value: f64, fine_value: f64, want: f64) -> (bool, f64) {
            let scale = want.abs().max(1e-12);
            let rel_coarse = (coarse_value - want).abs() / scale;
            let rel_fine = (fine_value - want).abs() / scale;
            // 曲面を含む立体の慣性は、刻みで収束します。**細かいほうで見て、
            // かつ細かくして悪くなっていないこと**を要求します。
            let ok = rel_fine < 1e-4 && rel_fine <= rel_coarse * 1.5 + 1e-15;
            println!(
                "  {:<26} {:>18.6} {:>18.6} {:>11.2e} {:>11.2e}  {}",
                label,
                fine_value,
                want,
                rel_coarse,
                rel_fine,
                if ok { "ok" } else { "MISS" }
            );
            (ok, rel_fine)
        }

        let mass_coarse = MassCalculator::compute_from_brep(&solid, &coarse);
        let mass_fine = MassCalculator::compute_from_brep(&solid, &fine);

        let record = |(ok, rel): (bool, f64), checks: &mut usize, failures: &mut usize, worst: &mut f64| {
            *checks += 1;
            *worst = worst.max(rel);
            if !ok {
                *failures += 1;
            }
        };

        record(
            row("volume", mass_coarse.volume, mass_fine.volume, subject.volume),
            &mut checks,
            &mut failures,
            &mut worst,
        );

        // 重心は成分ごとに見ます。**0 と比べるときは絶対値で**——相対で
        // 見ると 0 割りになり、ずれていても通ってしまいます。
        for (label, measured_coarse, measured_fine, want) in [
            (
                "centre of mass x",
                mass_coarse.center_of_mass.x,
                mass_fine.center_of_mass.x,
                subject.centroid.x,
            ),
            (
                "centre of mass y",
                mass_coarse.center_of_mass.y,
                mass_fine.center_of_mass.y,
                subject.centroid.y,
            ),
            (
                "centre of mass z",
                mass_coarse.center_of_mass.z,
                mass_fine.center_of_mass.z,
                subject.centroid.z,
            ),
        ] {
            checks += 1;
            // 立体の大きさで正規化する。閉じた式が 0 でも意味を持つ。
            let scale = (high - low).norm();
            let rel_coarse = (measured_coarse - want).abs() / scale;
            let rel_fine = (measured_fine - want).abs() / scale;
            worst = worst.max(rel_fine);
            let ok = rel_fine < 1e-6;
            if !ok {
                failures += 1;
            }
            println!(
                "  {:<26} {:>18.9} {:>18.9} {:>11.2e} {:>11.2e}  {}",
                label,
                measured_fine,
                want,
                rel_coarse,
                rel_fine,
                if ok { "ok" } else { "MISS" }
            );
        }

        let inertia_coarse = mass_coarse.inertia_tensor();
        let inertia_fine = mass_fine.inertia_tensor();
        for (label, axis, want) in [
            ("Ixx about the origin", 0usize, subject.inertia_about_origin.x),
            ("Iyy about the origin", 1, subject.inertia_about_origin.y),
            ("Izz about the origin", 2, subject.inertia_about_origin.z),
        ] {
            record(
                row(label, inertia_coarse[axis][axis], inertia_fine[axis][axis], want),
                &mut checks,
                &mut failures,
                &mut worst,
            );
        }

        // 主慣性モーメントは、立体を動かしても変わってはいけません。
        // ここでは**回転**して確かめます（平行移動は原点まわりを変えるので別物）。
        let principal = mass_fine.principal_moments();
        let turned = zenith_algo::BrepTransform::transform_solid(
            &solid,
            &zenith_math::Transform3::from_axis_angle(&Vec3::new(1.0, 1.0, 1.0), 0.7),
        )
        .ok();
        if let Some(turned) = turned {
            let turned_principal = MassCalculator::compute_from_brep(&turned, &fine)
                .principal_moments();
            let mut sorted = [principal.x, principal.y, principal.z];
            let mut turned_sorted = [
                turned_principal.x,
                turned_principal.y,
                turned_principal.z,
            ];
            sorted.sort_by(|a, b| a.partial_cmp(b).unwrap());
            turned_sorted.sort_by(|a, b| a.partial_cmp(b).unwrap());
            let scale = sorted[2].abs().max(1e-12);
            let drift = (0..3)
                .map(|i| (sorted[i] - turned_sorted[i]).abs() / scale)
                .fold(0.0f64, f64::max);
            checks += 1;
            worst = worst.max(drift);
            let ok = drift < 1e-6;
            if !ok {
                failures += 1;
            }
            println!(
                "  {:<26} {:>18} {:>18} {:>11} {:>11.2e}  {}",
                "principal moments, turned",
                "-",
                "unchanged",
                "-",
                drift,
                if ok { "ok" } else { "MISS" }
            );
        }
        println!();
    }

    println!("{}", "-".repeat(96));
    println!(
        "{checks} check(s), {failures} miss(es), worst relative error {worst:.2e}"
    );
    if failures > 0 {
        std::process::exit(1);
    }
    println!("every mass property lands on the closed form");
}
