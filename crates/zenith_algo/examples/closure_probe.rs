//! 自分が作れるものを、自分で受け取れるか。
//!
//! # なぜこれが要るか
//!
//! 4-23 で、ブーリアンが空洞を持つ立体を**返せる**のに**受け取れない**ことが
//! 分かりました。返した立体を次の演算に渡すのは実務のモデリングそのもの
//! なので、そこが切れていると使えません。しかもその欠陥は、返り値が
//! 閉じた立体で体積も返るぶん、**誤答として出ます**。
//!
//! ここは同じ切断が他にも無いかを、総当たりで見ます。各ビルダーの出力を、
//! 後段の4つに順に渡します。
//!
//! - **boolean**: 小さい箱で削る。カーネル自身の出力を入力に戻す経路。
//! - **section**: 平面で切る。
//! - **step**: 書き出して読み直し、体積が保たれるか。
//! - **mass**: 体積が有限で正か。ここが崩れていれば他は測れません。
//!
//! `-` は成功、`ERR` は断られたもの、`WRONG` は答えが返ったのに値が違うもの。
//! **断ることは欠陥ではありません。** 危険なのは WRONG と PANIC です。

use std::panic::{catch_unwind, AssertUnwindSafe};

use zenith_algo::{
    BooleanEngine, BooleanOpType, BrepTransform, GearBuilder, HelixBuilder, HoleBuilder,
    LoftBuilder, MassCalculator, PrimitiveBuilder, RevolveBuilder, SectionSlicer, ShellBuilder,
    StepInterop, SweepBuilder,
};
use zenith_geom::NurbsCurve3;
use zenith_io::StepImporter;
use zenith_math::{Point3, Tolerance, Vec3};
use zenith_tess::TessellationParams;
use zenith_topo::{Edge, OrientedEdge, Solid, Vertex, Wire};

fn params() -> TessellationParams {
    TessellationParams {
        u_divisions: 24,
        v_divisions: 24,
    }
}

fn rect_wire(half_x: f64, half_y: f64, z: f64) -> Wire {
    let points = [
        Point3::new(-half_x, -half_y, z),
        Point3::new(half_x, -half_y, z),
        Point3::new(half_x, half_y, z),
        Point3::new(-half_x, half_y, z),
    ];
    let vertices: Vec<Vertex> = points.into_iter().map(Vertex::from_point).collect();
    let edges = (0..4)
        .filter_map(|index| {
            Edge::line_between(vertices[index].clone(), vertices[(index + 1) % 4].clone())
                .ok()
                .map(OrientedEdge::forward)
        })
        .collect();
    Wire::new(edges)
}

fn offset_rect_wire(cx: f64, half: f64, z: f64) -> Wire {
    let points = [
        Point3::new(cx - half, -half, z),
        Point3::new(cx + half, -half, z),
        Point3::new(cx + half, half, z),
        Point3::new(cx - half, half, z),
    ];
    let vertices: Vec<Vertex> = points.into_iter().map(Vertex::from_point).collect();
    let edges = (0..4)
        .filter_map(|index| {
            Edge::line_between(vertices[index].clone(), vertices[(index + 1) % 4].clone())
                .ok()
                .map(OrientedEdge::forward)
        })
        .collect();
    Wire::new(edges)
}

/// 回転軸を含む平面（XZ）に置いた矩形。
///
/// 最初は XY 平面に置いていて、Z 軸まわりに回しても体積が 0 になりました。
/// **プローブ側の誤りです。** 軸に垂直な平面の断面は、回しても同じ平面の中を
/// 動くだけで、何も掃きません。ただしカーネルはそれを**エラーにせず、体積0の
/// 立体を返しました**。そちらは記録に値します（4-24）。
fn axial_rect_wire(cx: f64, half: f64) -> Wire {
    let points = [
        Point3::new(cx - half, 0.0, -half),
        Point3::new(cx + half, 0.0, -half),
        Point3::new(cx + half, 0.0, half),
        Point3::new(cx - half, 0.0, half),
    ];
    let vertices: Vec<Vertex> = points.into_iter().map(Vertex::from_point).collect();
    let edges = (0..4)
        .filter_map(|index| {
            Edge::line_between(vertices[index].clone(), vertices[(index + 1) % 4].clone())
                .ok()
                .map(OrientedEdge::forward)
        })
        .collect();
    Wire::new(edges)
}

fn subjects() -> Vec<(&'static str, Result<Solid, String>)> {
    let tol = Tolerance::default();
    vec![
        ("box", PrimitiveBuilder::make_box(20.0, 20.0, 20.0)),
        ("cylinder", PrimitiveBuilder::make_cylinder(8.0, 20.0)),
        ("sphere", PrimitiveBuilder::make_sphere(10.0)),
        ("cone", PrimitiveBuilder::make_cone(10.0, 4.0, 20.0)),
        ("torus", PrimitiveBuilder::make_torus(12.0, 4.0)),
        ("drilled box", HoleBuilder::make_drilled_box(30.0, 30.0, 15.0, 5.0)),
        (
            "hollow box (shelled)",
            ShellBuilder::make_hollow_box(30.0, 30.0, 30.0, 2.0, 5),
        ),
        (
            "revolved ring",
            RevolveBuilder::revolve_wire_solid(
                &axial_rect_wire(10.0, 2.0),
                Point3::new(0.0, 0.0, 0.0),
                Vec3::new(0.0, 0.0, 1.0),
                &tol,
            ),
        ),
        (
            "loft between squares",
            LoftBuilder::loft_solid(&[rect_wire(10.0, 10.0, 0.0), rect_wire(6.0, 6.0, 20.0)], 1, &tol),
        ),
        (
            "swept pipe",
            NurbsCurve3::bspline_from_points(
                3,
                vec![
                    Point3::new(0.0, 0.0, 0.0),
                    Point3::new(10.0, 0.0, 10.0),
                    Point3::new(20.0, 20.0, 25.0),
                    Point3::new(30.0, 20.0, 40.0),
                ],
            )
            .and_then(|path| SweepBuilder::sweep_circle_along_curve(&path, 3.5, 16)),
        ),
        (
            "helix spring",
            HelixBuilder::sweep_wire_along_helix(
                &offset_rect_wire(10.0, 1.0, 0.0),
                10.0,
                6.0,
                2.0,
                Point3::new(0.0, 0.0, 0.0),
                Vec3::new(0.0, 0.0, 1.0),
                64,
                &tol,
            ),
        ),
        ("spur gear", GearBuilder::make_spur_gear(2.0, 18, 20.0, 8.0, 3.0)),
        (
            "boolean result (box minus cylinder)",
            PrimitiveBuilder::make_box(30.0, 30.0, 30.0).and_then(|block| {
                let drill = PrimitiveBuilder::make_cylinder(6.0, 60.0)?;
                BooleanEngine::boolean_solids_exact(
                    &block,
                    &BrepTransform::translate_solid(&drill, Vec3::new(15.0, 15.0, -15.0)),
                    BooleanOpType::Difference,
                    &Tolerance::default(),
                )
            }),
        ),
    ]
}

/// 後段の1つを走らせ、短い判定を返す。
fn stage(label: &str, run: impl FnOnce() -> Result<Option<String>, String>) -> String {
    match catch_unwind(AssertUnwindSafe(run)) {
        Err(_) => format!("{label}:PANIC"),
        Ok(Err(err)) => format!("{label}:ERR({})", err.chars().take(60).collect::<String>()),
        Ok(Ok(None)) => format!("{label}:ok"),
        Ok(Ok(Some(note))) => format!("{label}:WRONG({note})"),
    }
}

fn main() {
    let tol = Tolerance::default();

    println!(
        "{:<38} {:>14}  {}",
        "subject", "volume", "boolean / section / step"
    );
    println!("{}", "-".repeat(112));

    let mut wrong = 0usize;
    let mut panicked = 0usize;
    let mut refused = 0usize;
    let mut fine = 0usize;

    for (name, built) in subjects() {
        let solid = match built {
            Ok(solid) => solid,
            Err(err) => {
                println!(
                    "{:<38} {:>14}  build refused: {}",
                    name,
                    "-",
                    err.chars().take(48).collect::<String>()
                );
                continue;
            }
        };

        let volume = MassCalculator::compute_from_brep(&solid, &params()).volume;
        if !volume.is_finite() || volume <= 0.0 {
            println!("{name:<38} {volume:>14.4}  mass:WRONG(volume is not positive)");
            wrong += 1;
            continue;
        }

        let mut verdicts = Vec::new();

        // 1. 自分の出力をブーリアンの入力に戻す。
        verdicts.push(stage("boolean", || {
            let knife = BrepTransform::translate_solid(
                &PrimitiveBuilder::make_box(4.0, 4.0, 200.0)?,
                Vec3::new(-2.0, -2.0, -100.0),
            );
            let result =
                BooleanEngine::boolean_solids_exact_result(&solid, &knife, BooleanOpType::Difference, &tol)
                    .map_err(|err| err.to_string())?;
            let after: f64 = result
                .solids
                .iter()
                .map(|s| MassCalculator::compute_from_brep(s, &params()).volume)
                .sum();
            // 削ったのだから減っていなければおかしい。増えていたら誤答。
            if after > volume + 1e-6 {
                return Ok(Some(format!("{after:.3} > {volume:.3}")));
            }
            Ok(None)
        }));

        // 2. 平面で切る。
        verdicts.push(stage("section", || {
            let result = SectionSlicer::slice_solid(
                &solid,
                Point3::new(0.0, 0.0, 0.0),
                Vec3::new(0.0, 0.0, 1.0),
                &tol,
            )
            .map_err(|err| err.to_string())?;
            if !result.total_area.is_finite() || result.total_area < 0.0 {
                return Ok(Some(format!("area {:.4}", result.total_area)));
            }
            Ok(None)
        }));

        // 3. 書き出して読み直す。体積が保たれているか。
        verdicts.push(stage("step", || {
            let (text, _report) = StepInterop::export_solid_to_string(&solid, "subject", &tol);
            let read = StepImporter::import_solid_from_str(&text).map_err(|err| err.to_string())?;
            let after = MassCalculator::compute_from_brep(&read, &params()).volume;
            let relative = (after - volume).abs() / volume.abs();
            if relative > 1e-6 {
                return Ok(Some(format!("{relative:.2e}")));
            }
            Ok(None)
        }));

        for verdict in &verdicts {
            if verdict.contains("PANIC") {
                panicked += 1;
            } else if verdict.contains("WRONG") {
                wrong += 1;
            } else if verdict.contains("ERR") {
                refused += 1;
            } else {
                fine += 1;
            }
        }

        println!("{name:<38} {volume:>14.4}  {}", verdicts.join("  "));
    }

    println!("{}", "-".repeat(112));
    println!("ok {fine}   refused {refused}   WRONG {wrong}   PANIC {panicked}");
    println!();
    println!("refused is not a defect. WRONG and PANIC are, because the caller");
    println!("cannot tell either of them from an answer.");
}
