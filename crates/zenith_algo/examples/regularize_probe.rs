//! 正規化が形を動かしていないかを測る。
//!
//! 全周1枚のパッチ・全周1本の辺を刻んでも、体積・表面積は動かないはずである。
//! ここで動いていたら、刻み方が形を変えている。外部カーネルに渡す前に、
//! まず自分の物差しで確かめる段。
//!
//! 対象は他カーネルが書いた STEP。自前ビルダーの出力は元から刻まれている
//! ので、そちらは「何も起きない」ことの確認になる。

use zenith_algo::mass_properties::MassCalculator;
use zenith_algo::Regularizer;
use zenith_math::Tolerance;
use zenith_tess::TessellationParams;

fn main() {
    let tol = Tolerance::default();
    let params = TessellationParams::default();

    let subjects = [
        ("occ cone", "target/validation/occ_reference_cone.step", 3267.2563597),
        ("occ cone_full", "target/validation/occ_reference_cone_full.step", 2094.3951023932),
        ("occ cylinder", "target/validation/occ_reference_cylinder.step", 12566.3706143592),
        ("occ cylinder_nurbs", "target/validation/occ_reference_cylinder_nurbs.step", 12566.3706143592),
        ("occ sphere", "target/validation/occ_reference_sphere.step", 4188.7902047864),
        ("occ sphere_capped", "target/validation/occ_reference_sphere_capped.step", 2094.3951023932),
        ("occ torus", "target/validation/occ_reference_torus.step", 3789.9280732),
        ("occ torus_segment", "target/validation/occ_reference_torus_segment.step", 947.4820183),
        ("native cylinder", "target/validation/cylinder_r10_h40.step", 12566.3706143592),
        ("native sphere", "target/validation/sphere_r10.step", 4188.7902047864),
    ];

    println!(
        "{:<22} {:>6} {:>6} {:>5} {:>5} {:>16} {:>16} {:>10} {:>8}",
        "subject", "faces", "->", "split", "left", "volume before", "volume after", "rel move", "shell"
    );
    println!("{}", "-".repeat(112));

    let mut moved_worst: f64 = 0.0;
    let mut left_alone_total = 0usize;

    for (label, path, _truth) in subjects {
        let solids = match zenith_io::StepImporter::import_solids_from_file(path) {
            Ok(solids) if !solids.is_empty() => solids,
            _ => {
                println!("{label:<22} could not be read");
                continue;
            }
        };
        let solid = &solids[0];

        let before = MassCalculator::compute_from_brep(solid, &params);
        let face_count_before = solid.outer_shell.faces.len();

        let (regular, report) = Regularizer::regularize_solid(solid, &tol);
        let after = MassCalculator::compute_from_brep(&regular, &params);

        let rel = if before.volume.abs() > 1e-12 {
            (after.volume - before.volume).abs() / before.volume.abs()
        } else {
            (after.volume - before.volume).abs()
        };
        moved_worst = moved_worst.max(rel);
        left_alone_total += report.wrapped_faces_left_alone;

        let shell = regular.outer_shell.validate_closed(&tol);
        println!(
            "{:<22} {:>6} {:>6} {:>5} {:>5} {:>16.6} {:>16.6} {:>10.2e} {:>8}",
            label,
            face_count_before,
            regular.outer_shell.faces.len(),
            report.wrapped_faces_split,
            report.wrapped_faces_left_alone,
            before.volume,
            after.volume,
            rel,
            if shell.errors.is_empty() { "valid" } else { "BROKEN" }
        );
        for reason in &report.left_alone_reasons {
            println!("{:>24}left alone: {reason}", "");
        }
        if !shell.errors.is_empty() {
            for error in shell.errors.iter().take(3) {
                println!("{:>24}{}", "", error);
            }
        }
    }

    println!("{}", "-".repeat(112));
    println!(
        "worst relative volume move {moved_worst:.3e}; wrapped faces left alone: {left_alone_total}"
    );
}
