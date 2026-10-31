//! 空洞立体×OCC立体の3演算が、どの組で断られるかを出す（4-840の切り分け）。
use std::path::PathBuf;
use zenith_algo::{BooleanEngine, BooleanOpType, BrepTransform, MassCalculator, Regularizer};
use zenith_io::StepImporter;
use zenith_math::{Tolerance, Vec3};
use zenith_tess::TessellationParams;
use zenith_topo::Solid;

fn params() -> TessellationParams { TessellationParams { u_divisions: 24, v_divisions: 24 } }

fn load(name: &str, tol: &Tolerance) -> Option<Solid> {
    let path = PathBuf::from(concat!(env!("CARGO_MANIFEST_DIR"), "/tests/fixtures"))
        .join(format!("occ_reference_{name}.step"));
    StepImporter::import_solids_from_file(&path).ok()?
        .first().map(|s| Regularizer::hold_like_our_own(s, tol))
}

fn centred(solid: &Solid) -> Solid {
    let c = MassCalculator::compute_from_brep(solid, &params()).center_of_mass;
    BrepTransform::translate_solid(solid, Vec3::new(-c.x, -c.y, -c.z))
}

fn main() {
    let tol = Tolerance::default();
    let hollow = centred(&load("hollow_box", &tol).unwrap());
    println!("hollow 体積 {:.6}", MassCalculator::compute_from_brep(&hollow, &params()).volume);
    for name in ["elliptic_prism", "sphere", "stepped_shaft"] {
        let other = centred(&load(name, &tol).unwrap());
        println!("{name} 体積 {:.6}", MassCalculator::compute_from_brep(&other, &params()).volume);
        for op in [BooleanOpType::Union, BooleanOpType::Intersection, BooleanOpType::Difference] {
            match BooleanEngine::boolean_solids_exact_result(&hollow, &other, op, &tol) {
                Ok(r) => println!("  {op:?}: 体積 {:.6}", r.solids.iter().map(|s| MassCalculator::compute_from_brep(s, &params()).volume).sum::<f64>()),
                Err(e) => println!("  {op:?}: 断り {}", &e.chars().take(140).collect::<String>()),
            }
        }
    }
}
