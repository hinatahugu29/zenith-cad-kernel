//! 整理した立体を STEP に書き出し、OpenCASCADE 側の検証にかけられるようにする。
//!
//! `tools/verify_simplified.py` と対で使う。整理は面と稜を減らすが、
//! **他カーネルが同じ形として読めるか**は外から測らないと分からない。

use std::fs;

use zenith_algo::{
    BooleanEngine, BooleanOpType, BrepTransform, FaceMerger, HoleBuilder, MassCalculator,
    PrimitiveBuilder, StepInterop,
};
use zenith_math::{Tolerance, Vec3};
use zenith_tess::TessellationParams;
use zenith_topo::Solid;

fn main() -> std::io::Result<()> {
    let tol = Tolerance::default();
    let directory = std::path::Path::new("target/simplified");
    fs::create_dir_all(directory)?;

    let block = PrimitiveBuilder::make_box(40.0, 40.0, 20.0).unwrap();
    let corner = BrepTransform::translate_solid(
        &PrimitiveBuilder::make_box(20.0, 20.0, 20.0).unwrap(),
        Vec3::new(20.0, 20.0, 0.0),
    );
    let bore = BrepTransform::translate_solid(
        &PrimitiveBuilder::make_cylinder(6.0, 20.0).unwrap(),
        Vec3::new(20.0, 20.0, 0.0),
    );

    let subjects: Vec<(&str, Solid)> = vec![
        (
            "drilled_box_simplified",
            HoleBuilder::make_drilled_box(40.0, 40.0, 20.0, 8.0).unwrap(),
        ),
        (
            "l_prism_simplified",
            BooleanEngine::boolean_solids_exact(&block, &corner, BooleanOpType::Difference, &tol)
                .unwrap(),
        ),
        (
            "bored_block_simplified",
            BooleanEngine::boolean_solids_exact(&block, &bore, BooleanOpType::Difference, &tol)
                .unwrap(),
        ),
        (
            "counterbore_simplified",
            HoleBuilder::make_counterbore_hole_box(40.0, 40.0, 20.0, 5.0, 9.0, 6.0).unwrap(),
        ),
    ];

    let mut manifest = String::from("[\n");
    for (index, (name, solid)) in subjects.iter().enumerate() {
        let (simplified, report) = FaceMerger::simplify_solid(solid, &tol)
            .unwrap_or_else(|err| panic!("simplifying {name}: {err}"));
        let volume = MassCalculator::compute_from_brep(
            &simplified,
            &TessellationParams {
                u_divisions: 64,
                v_divisions: 64,
            },
        )
        .volume;

        let path = format!("target/simplified/{name}.step");
        StepInterop::export_solid_to_file(&simplified, &path, name, &tol)?;
        println!("{name:<28} {}", report.summary());

        manifest.push_str(&format!(
            "  {{\"name\": \"{name}\", \"path\": \"{path}\", \"volume\": {volume}}}{}\n",
            if index + 1 == subjects.len() { "" } else { "," }
        ));
    }
    manifest.push_str("]\n");
    fs::write(directory.join("manifest.json"), manifest)?;

    println!("wrote {} simplified subject(s)", subjects.len());
    Ok(())
}
