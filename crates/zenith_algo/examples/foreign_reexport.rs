//! Reads files another kernel wrote and writes them back out.
//!
//! Everything else in the suite starts from shapes this kernel built. This
//! starts from OpenCASCADE's own STEP, reads it, and re-emits it, so the whole
//! path - their writer, our reader, our writer, their reader - is exercised at
//! once.

use std::fs;
use std::path::Path;

use zenith_algo::MassCalculator;
use zenith_io::{StepExporter, StepImporter};
use zenith_tess::TessellationParams;
use zenith_topo::Solid;

fn volume(solid: &Solid) -> f64 {
    MassCalculator::compute_from_brep(
        solid,
        &TessellationParams {
            u_divisions: 64,
            v_divisions: 64,
        },
    )
    .volume
}

fn main() {
    let out = Path::new("target/reexport");
    fs::create_dir_all(out).unwrap();

    let subjects = [
        ("cone", 3267.2564),
        ("cone_full", 2094.3951),
        ("sphere", 4188.7902),
        ("sphere_capped", 2094.3951),
        ("torus", 3789.9281),
        ("torus_segment", 947.4820),
        ("cylinder", 12566.3706),
    ];

    println!(
        "{:<16} {:>14} {:>14} {:>10} {:>14} {:>10}  {}",
        "subject", "OCC wrote", "we read", "rel", "we read back", "rel", "re-exported to"
    );
    println!("{}", "-".repeat(112));

    for (name, occ_volume) in subjects {
        let source = Path::new("target/validation").join(format!("occ_reference_{name}.step"));
        let solids = match StepImporter::import_solids_from_file(&source) {
            Ok(solids) => solids,
            Err(err) => {
                println!("{name:<16} FAILED: {}", err.chars().take(60).collect::<String>());
                continue;
            }
        };

        let solid = &solids[0];
        let read = volume(solid);
        let relative = (read - occ_volume).abs() / occ_volume;

        let target = out.join(format!("reexport_{name}.step"));
        let text = StepExporter::export_solid_to_string(solid, name);
        fs::write(&target, text).unwrap();

        // 書き出したものを自前で読み直す。ここが一致していれば、書き手と
        // 読み手の間で情報は落ちていない。OpenCASCADE とだけずれるなら、
        // 食い違っているのは表現の解釈のほう。
        let round_trip = StepImporter::import_solids_from_file(&target)
            .ok()
            .and_then(|solids| solids.first().map(volume));

        match round_trip {
            Some(back) => println!(
                "{name:<16} {occ_volume:>14.4} {read:>14.4} {relative:>10.2e} {back:>14.4} {:>10.2e}  {}",
                (back - read).abs() / read,
                target.display()
            ),
            None => println!(
                "{name:<16} {occ_volume:>14.4} {read:>14.4} {relative:>10.2e} {:>14} {:>10}  {}",
                "unreadable", "-", target.display()
            ),
        }
    }
}
