//! **細分の判定と、検証の判定を、同じ 1 本の稜で並べます**（4-545 の宿題）。
//!
//! # なぜ要るのか
//!
//! `linkrods` の積を検証すると、**まっすぐな縦の稜 2 本**で
//! **p-curve が 3D の稜から最大 5.636e-4 外れている**と言われます（4-544）。
//! **形は合っています**——**p-curve は u が動かない等パラメータ線で、
//! 射影の最悪は 1.535e-8**（4-545）。
//!
//! **外れているのは点と点のあいだ**です。**9 点の折れ線**で、
//! **節の上ではゼロ、区間の真ん中で最大。**
//!
//! **辻褄が合いません**——**細分の判定は、まさにその量を見ています。**
//! **区間の真ん中で 5.6e-4 ずれているなら、割るはず**なのに、
//! **点は 9 のまま**でした。
//!
//! **この口が、両方を同じ稜で出します。**
//!
//! # 走らせ方
//!
//! ```text
//! cargo run --release -p zenith_algo --example pcurve_span_probe
//! ```

use zenith_geom::{ExtremumEngine, NurbsCurve3, Surface3};
#[allow(unused_imports)]
use zenith_geom::NurbsSurface3;
use zenith_io::StepImporter;
use zenith_math::Point3;
use zenith_topo::FaceGeometry;

fn main() {
    let path = "reference/OCCT/data/step/linkrods.step";
    let Ok(solids) = StepImporter::import_solids_from_file(path) else {
        println!("{path} が読めません");
        return;
    };
    let Some(read) = solids.iter().max_by_key(|s| s.outer_shell.faces.len()) else {
        return;
    };
    let start = Point3::new(4.273668, 2.614733, 0.599931);
    let end = Point3::new(4.273668, 2.614733, 1.349931);
    // **この稜を載せている面は 1 枚ではありません**（4-546）。
    // **`PCURVEPATH` は、同じ稜に 2 つのパッチを出します。**
    // **どちらで測るかで答えが変わる**ので、載っている面を全部見ます。
    let mut carriers: Vec<usize> = Vec::new();
    for (index, face) in read.outer_shell.faces.iter().enumerate() {
        let FaceGeometry::Nurbs(surface) = &face.geometry else {
            continue;
        };
        let near = |point: Point3| {
            ExtremumEngine::point_to_surface(point, surface, 64, 1e-13)
                .map(|p| p.distance)
                .unwrap_or(f64::INFINITY)
        };
        if near(start) < 1e-5 && near(end) < 1e-5 {
            carriers.push(index);
        }
    }
    println!("この稜を載せている面: {carriers:?}");
    println!();
    for carrier in carriers {
        let FaceGeometry::Nurbs(surface) = &read.outer_shell.faces[carrier].geometry else {
            continue;
        };
        println!("======== 面{carrier} ========");
        report(surface, start, end);
    }
}

fn report(surface: &zenith_geom::NurbsSurface3, start: Point3, end: Point3) {
    let curve = NurbsCurve3::bspline_from_points(1, vec![start, end]).expect("稜");
    let (t0, t1) = curve.param_range();
    let at = |t: f64| curve.evaluate(t0 + (t1 - t0) * t);

    // **導出と同じ 9 点**（`samples_per_edge` の既定は 8）。
    let spans = 8usize;
    let mut uvs = Vec::new();
    let mut worst_projection = 0.0f64;
    for i in 0..=spans {
        let t = i as f64 / spans as f64;
        let projection = ExtremumEngine::point_to_surface(at(t), surface, 64, 1e-13).expect("射影");
        worst_projection = worst_projection.max(projection.distance);
        uvs.push((projection.u, projection.v));
    }
    println!("稜 ({start:?}) - ({end:?})");
    println!("射影の最悪 {worst_projection:.3e}、点 {}", uvs.len());
    println!();
    println!("v は等間隔か（等間隔なら、折れ線で厳密）:");
    for i in 0..=spans {
        let t = i as f64 / spans as f64;
        let straight = uvs[0].1 + (uvs[spans].1 - uvs[0].1) * t;
        println!(
            "  t={t:.3}  v={:.9}  まっすぐなら {:.9}  差 {:+.3e}",
            uvs[i].1,
            straight,
            uvs[i].1 - straight
        );
    }

    println!();
    println!("細分の判定（区間の真ん中で、uv の弦を曲面へ写して稜と比べる）:");
    let mut worst_strayed = 0.0f64;
    for i in 0..spans {
        let (a, b) = (i as f64 / spans as f64, (i + 1) as f64 / spans as f64);
        let middle = (a + b) * 0.5;
        let chord = (
            (uvs[i].0 + uvs[i + 1].0) * 0.5,
            (uvs[i].1 + uvs[i + 1].1) * 0.5,
        );
        let strayed = (surface.evaluate(chord.0, chord.1) - at(middle)).norm();
        worst_strayed = worst_strayed.max(strayed);
        println!("  区間{i} t={middle:.3}  strayed {strayed:.6e}");
    }

    println!();
    println!("検証の判定（30 点で、折れ線を曲面へ写して稜と比べる）:");
    let mut worst_validation = 0.0f64;
    let samples = 30usize;
    for i in 0..=samples {
        let t = i as f64 / samples as f64;
        // 次数 1・節が標本の媒介変数なので、折れ線の評価は線形内挿と同じ。
        let scaled = (t * spans as f64).min(spans as f64 - 1e-12);
        let index = scaled.floor() as usize;
        let local = scaled - index as f64;
        let uv = (
            uvs[index].0 + (uvs[index + 1].0 - uvs[index].0) * local,
            uvs[index].1 + (uvs[index + 1].1 - uvs[index].1) * local,
        );
        let distance = (surface.evaluate(uv.0, uv.1) - at(t)).norm();
        worst_validation = worst_validation.max(distance);
        if i % 3 == 0 {
            println!("  t={t:.3}  外れ {distance:.6e}");
        }
    }

    println!();
    println!("区間 0 の中を刻む（細分は真ん中しか見ません）:");
    for k in 0..=12 {
        let local = k as f64 / 12.0;
        let t = local / spans as f64;
        let uv = (
            uvs[0].0 + (uvs[1].0 - uvs[0].0) * local,
            uvs[0].1 + (uvs[1].1 - uvs[0].1) * local,
        );
        let distance = (surface.evaluate(uv.0, uv.1) - at(t)).norm();
        println!(
            "  区間内 {local:.3}（t={t:.4}） 外れ {distance:.6e}{}",
            if (local - 0.5).abs() < 1e-9 {
                "  ← 細分が見る点"
            } else {
                ""
            }
        );
    }

    println!();
    println!("**細分が見る最悪 {worst_strayed:.6e} / 検証が見る最悪 {worst_validation:.6e}**");
}
