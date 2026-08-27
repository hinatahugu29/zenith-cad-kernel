//! 一時診断: 他カーネルの立体どうし、**異なる検体**の総当たり。
use std::path::PathBuf;
use zenith_algo::{BooleanEngine, BooleanOpType, BrepTransform, MassCalculator, Regularizer};
use zenith_io::StepImporter;
use zenith_math::{Tolerance, Vec3};
use zenith_topo::Solid;

fn load(name: &str, tol: &Tolerance) -> Option<Solid> {
    let path = PathBuf::from(concat!(env!("CARGO_MANIFEST_DIR"), "/tests/fixtures"))
        .join(format!("occ_reference_{name}.step"));
    StepImporter::import_solids_from_file(&path).ok()?.first().map(|s| Regularizer::hold_like_our_own(s, tol))
}

fn main() {
    let tol = Tolerance::default();
    let params = zenith_tess::TessellationParams { u_divisions: 24, v_divisions: 24 };
    let vol = |s: &[Solid]| -> f64 { s.iter().map(|x| MassCalculator::compute_from_brep(x, &params).volume).sum() };
    let one = |s: &Solid| MassCalculator::compute_from_brep(s, &params).volume;

    let names = ["cylinder", "sphere", "torus", "cone", "hollow_box", "stepped_shaft", "plate_with_holes", "chamfered_box", "elliptic_prism", "slotted_block"];
    let mut solids: Vec<(&str, Solid)> = Vec::new();
    for n in names { if let Some(s) = load(n, &tol) { solids.push((n, s)); } }
    println!("読めた検体 {}", solids.len());

    let mut worst = 0.0f64; let mut full = 0usize; let mut partial = 0usize; let mut pairs = 0usize;
    let mut worst_pair = String::new();
    for i in 0..solids.len() {
        for j in (i + 1)..solids.len() {
            let (na, a) = (&solids[i].0, &solids[i].1);
            let (nb, b0) = (&solids[j].0, &solids[j].1);
            // B を A の中心へ寄せて、必ず重なるようにする。
            let ba = a.bounding_box(); let bb = b0.bounding_box();
            let ca = (ba.min.coords + ba.max.coords) * 0.5;
            let cb = (bb.min.coords + bb.max.coords) * 0.5;
            let b = BrepTransform::translate_solid(b0, Vec3::new(ca.x - cb.x, ca.y - cb.y, ca.z - cb.z));
            pairs += 1;
            let (va, vb) = (one(a), one(&b));
            let u = BooleanEngine::boolean_solids_exact_result(a, &b, BooleanOpType::Union, &tol).ok().map(|r| vol(&r.solids));
            let m = BooleanEngine::boolean_solids_exact_result(a, &b, BooleanOpType::Intersection, &tol).ok().map(|r| vol(&r.solids));
            let d = BooleanEngine::boolean_solids_exact_result(a, &b, BooleanOpType::Difference, &tol).ok().map(|r| vol(&r.solids));
            match (u, m, d) {
                (Some(u), Some(m), Some(d)) => {
                    full += 1;
                    let incl = ((u + m) - (va + vb)).abs() / va.abs().max(1.0);
                    let split = ((d + m) - va).abs() / va.abs().max(1.0);
                    let w = incl.max(split);
                    if w > worst { worst = w; worst_pair = format!("{na} x {nb}"); }
                    if w > 1e-5 { println!("**BAD** {na} x {nb}: incl {incl:.3e}, split {split:.3e}"); }
                }
                _ => partial += 1,
            }
        }
    }
    println!("組 {pairs}, 3演算そろった {full}, そろわなかった {partial}");
    println!("恒等式の最悪残差 {worst:.3e}（{worst_pair}）");
}
