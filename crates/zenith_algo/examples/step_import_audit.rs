//! Measures what the STEP importer actually reads back.
//!
//! Export has been verified against OpenCASCADE; import has not been measured
//! at all. Two questions matter for an addon that opens other people's files:
//! does a solid survive a round trip through our own writer and reader, and can
//! the reader open a file written by a different kernel?
//!
//! Run with: cargo run --release -p zenith_algo --example step_import_audit

use std::f64::consts::PI;
use std::fs;
use std::path::Path;

use zenith_algo::{MassCalculator, PrimitiveBuilder};
use zenith_io::{StepExporter, StepImporter};
use zenith_math::Tolerance;
use zenith_tess::TessellationParams;
use zenith_topo::Solid;

fn volume(solid: &Solid) -> f64 {
    MassCalculator::compute_from_brep(
        solid,
        &TessellationParams {
            u_divisions: 32,
            v_divisions: 32,
        },
    )
    .volume
}

/// **問題の数**を返します（4-653）。
///
/// **これまでは印字するだけ**でした——**`IMPORT FAILED` も `INVALID` も、
/// 面の数が変わったことも、門には届きません**（`exit` が無く、
/// 門の 4 語にも当たらない）。**どれも探り自身が失敗と名付けているもの**
/// なので、**閾値の判断は要りません。**
///
/// **体積の相対差は床にしません**（いまは 1e-13 台）——**それは
/// 「いくつ以下なら合格」を決める話**で、ここでは決めていません。
fn round_trip(name: &str, solid: &Solid, analytic: Option<f64>) -> usize {
    let tol = Tolerance::default();
    let original_volume = volume(solid);
    let original_faces = solid.outer_shell.faces.len();

    let step = StepExporter::export_solid_to_string(solid, name);

    match StepImporter::import_solid_from_str(&step) {
        Ok(imported) => {
            let imported_volume = volume(&imported);
            let relative =
                (imported_volume - original_volume).abs() / original_volume.abs().max(1e-12);
            let shell_ok = imported.outer_shell.validate_closed(&tol).is_valid();

            let analytic_note = analytic
                .map(|expected| {
                    format!(
                        " vs analytic {:.2e}",
                        (imported_volume - expected).abs() / expected.abs()
                    )
                })
                .unwrap_or_default();

            println!(
                "{name:<28} {original_faces:>3} -> {:>3} faces  volume {original_volume:>13.4} -> {imported_volume:>13.4}  rel {relative:.2e}  shell {}{analytic_note}",
                imported.outer_shell.faces.len(),
                if shell_ok { "valid" } else { "INVALID" }
            );
            let mut problems = 0usize;
            if !shell_ok {
                problems += 1;
            }
            if imported.outer_shell.faces.len() != original_faces {
                problems += 1;
            }
            problems
        }
        Err(err) => {
            println!("{name:<28} IMPORT FAILED: {err}");
            1
        }
    }
}

/// **問題の数**を返します（4-653。`round_trip` と同じ理屈）。
fn read_foreign(path: &Path) -> usize {
    let name = path.file_name().unwrap().to_string_lossy().to_string();
    match StepImporter::import_solids_from_file(path) {
        Ok(solids) => {
            if solids.is_empty() {
                println!("{name:<44} read 0 solids");
                return 1;
            }
            let total: f64 = solids.iter().map(volume).sum();
            let faces: usize = solids
                .iter()
                .map(|solid| solid.outer_shell.faces.len())
                .sum();
            println!(
                "{name:<44} {} solid(s), {faces} face(s), volume {total:.4}",
                solids.len()
            );
            let mut problems = 0usize;
            let mut shares = 0.0f64;
            for solid in &solids {
                let tol = Tolerance::default();
                let report = solid.outer_shell.validate_closed(&tol);
                if !report.is_valid() {
                    println!(
                        "        shell invalid: {}",
                        report.errors.first().cloned().unwrap_or_default()
                    );
                    problems += 1;
                }
                for (index, face) in solid.outer_shell.faces.iter().enumerate() {
                    let (area, contribution) = MassCalculator::compute_face_integral(
                        face,
                        &TessellationParams {
                            u_divisions: 64,
                            v_divisions: 64,
                        },
                    );
                    println!(
                        "        face {index}: area {area:.4}, volume share {contribution:.4}"
                    );
                    shares += contribution;
                }
                // **面ごとの寄与の和は、その立体の体積**（4-653）。
                // **探りは両辺を印字していました**が、**比べてはいません**
                // でした。**恒等式なので、閾値の判断は要りません**
                // （**相対 1e-9**——**和の丸め分だけ見ます**）。
                //
                // **外殻だけ足してはいけません。** **最初そうして、
                // `occ_reference_hollow_box.step` で 8400 対 4240 と
                // 出ました**——**空洞のぶん**です。**内側の殻の面も
                // 足します**（**向きで符号が付く**ので、引き算は要りません）。
                // **内側の殻は、引きます。** **足したら 12560 になりました**
                // （8400 ＋ 4160）——**内殻の面も外向きに持たれている**ので、
                // **向きでは符号が付きません。** **8400 − 4160 = 4240** が
                // 立体の体積です。**2 度間違えてから、ここに辿り着きました。**
                for shell in &solid.inner_shells {
                    for face in &shell.faces {
                        shares -= MassCalculator::compute_face_integral(
                            face,
                            &TessellationParams {
                                u_divisions: 64,
                                v_divisions: 64,
                            },
                        )
                        .1;
                    }
                }
                let whole = volume(solid);
                if (shares - whole).abs() > whole.abs().max(1e-12) * 1e-9 {
                    println!(
                        "        **寄与の和 {shares:.6} が体積 {whole:.6} と合いません**（4-653）。"
                    );
                    problems += 1;
                }
                shares = 0.0;
            }
            problems
        }
        Err(err) => {
            println!(
                "{name:<44} FAILED: {}",
                err.chars().take(400).collect::<String>()
            );
            1
        }
    }
}

fn main() {
    let mut problems = 0usize;
    println!("=== round trip through our own writer and reader");
    problems += round_trip(
        "box",
        &PrimitiveBuilder::make_box(20.0, 30.0, 40.0).unwrap(),
        Some(24000.0),
    );
    problems += round_trip(
        "cylinder",
        &PrimitiveBuilder::make_cylinder(10.0, 40.0).unwrap(),
        Some(PI * 100.0 * 40.0),
    );
    problems += round_trip(
        "sphere",
        &PrimitiveBuilder::make_sphere(10.0).unwrap(),
        Some(4.0 / 3.0 * PI * 1000.0),
    );
    problems += round_trip(
        "cone",
        &PrimitiveBuilder::make_cone(10.0, 4.0, 20.0).unwrap(),
        Some(PI * 20.0 / 3.0 * (100.0 + 40.0 + 16.0)),
    );
    problems += round_trip(
        "torus",
        &PrimitiveBuilder::make_torus(12.0, 4.0).unwrap(),
        Some(2.0 * PI * PI * 12.0 * 16.0),
    );

    println!();
    println!("=== reading the showcase files back");
    let showcase = Path::new("target/showcase");
    if showcase.is_dir() {
        let mut names: Vec<_> = fs::read_dir(showcase)
            .unwrap()
            .filter_map(|entry| entry.ok())
            .map(|entry| entry.path())
            .filter(|path| path.extension().map(|ext| ext == "step").unwrap_or(false))
            .collect();
        names.sort();
        // **0 件は 0 件**（4-657。**4-656 と同じ穴が、ここにも在りました**）。
        //
        // **`target/showcase` が無ければ節ごと飛ばし、`problems` は 0 の
        // まま**でした——**実測: 退避して回すと検体が 54 → 20 に減っても
        // rc=0。** **4-653 でここに床を入れたのに、すり抜けました。**
        //
        // **数は固定ではありません**（生成物なので増えます）ので、
        // **「増えたら赤」にはしません**——**0 件だけを赤**にします。
        if names.is_empty() {
            println!("        **target/showcase に step が 1 つもありません**（4-657）。");
            problems += 1;
        }
        println!("        （showcase から {} 件読みました）", names.len());
        for path in names {
            problems += read_foreign(&path);
        }
    } else {
        println!("    target/showcase is missing; run the export_showcase example first");
        // **無いことに気づいて印字しながら、緑で通っていました**（4-657）。
        problems += 1;
    }

    println!();
    println!("=== reading files written by OpenCASCADE");
    let validation = Path::new("target/validation");
    if validation.is_dir() {
        let mut names: Vec<_> = fs::read_dir(validation)
            .unwrap()
            .filter_map(|entry| entry.ok())
            .map(|entry| entry.path())
            .filter(|path| {
                path.file_name()
                    .map(|name| name.to_string_lossy().starts_with("occ_reference"))
                    .unwrap_or(false)
            })
            .collect();
        names.sort();
        // **0 件は 0 件**（4-657）。**気づいて印字しながら、緑で通って
        // いました**——`target/showcase` と同じ形です。
        if names.is_empty() {
            println!("    no OpenCASCADE reference files; run tools/occ_reference_export.py");
            problems += 1;
        } else {
            println!("    （OCC が書いたものを {} 件読みました）", names.len());
        }
        for path in names {
            problems += read_foreign(&path);
        }
    } else {
        // **節ごと飛ばして緑**でした（4-657）。**`target/showcase` の
        // ほうには `else` が在ったのに、こちらには無かった**だけです。
        println!("    target/validation is missing; run tools/occ_reference_export.py");
        problems += 1;
    }

    // **床**（4-653）。**探り自身が失敗と名付けたものだけ**を数えています
    // ——`IMPORT FAILED` / `INVALID` / `FAILED` / 読めた立体 0 /
    // 面の数が変わった / 寄与の和が体積と合わない。
    if problems != 0 {
        println!();
        println!("**{problems} 件の問題があります**（4-653）。");
        std::process::exit(1);
    }

}
