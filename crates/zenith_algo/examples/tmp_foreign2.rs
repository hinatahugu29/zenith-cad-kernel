//! 一時診断: **他カーネルが書いた立体どうし**のブーリアン。
//!
//! いままで測っていたのは「他カーネルの立体 × 自作の切り手」だけでした。
//! ここは両方とも OpenCASCADE が書いた STEP です。
//!
//! 閉じた式はありません。**恒等式で見ます**——
//!   |A ∪ B| + |A ∩ B| = |A| + |B|
//!   |A \ B| + |A ∩ B| = |A|
use std::path::PathBuf;
use zenith_algo::{BooleanEngine, BooleanOpType, BrepTransform, MassCalculator, PrimitiveBuilder, Regularizer};
use zenith_io::StepImporter;
use zenith_math::{Tolerance, Vec3};
use zenith_topo::Solid;

fn load(name: &str, tol: &Tolerance) -> Option<Solid> {
    let path = PathBuf::from(concat!(env!("CARGO_MANIFEST_DIR"), "/tests/fixtures"))
        .join(format!("occ_reference_{name}.step"));
    let solids = StepImporter::import_solids_from_file(&path).ok()?;
    solids.first().map(|s| Regularizer::hold_like_our_own(s, tol))
}

fn main() {
    let tol = Tolerance::default();
    let params = zenith_tess::TessellationParams { u_divisions: 32, v_divisions: 32 };
    let vol = |s: &[Solid]| -> f64 { s.iter().map(|x| MassCalculator::compute_from_brep(x, &params).volume).sum() };
    let one = |s: &Solid| MassCalculator::compute_from_brep(s, &params).volume;

    let names = ["cylinder", "sphere", "torus", "cone", "hollow_box", "stepped_shaft", "plate_with_holes", "chamfered_box"];
    let mut loaded: Vec<(&str, Solid)> = Vec::new();
    for n in names {
        match load(n, &tol) {
            Some(s) => loaded.push((n, s)),
            None => println!("{n}: 読めない"),
        }
    }
    println!("読めた検体 {}", loaded.len());

    // 相手は「同じ検体を、境界箱の対角の 40% だけずらしたもの」。
    // 必ず重なるので、恒等式が意味を持つ。
    println!("{:<20} {:>12} {:>12} {:>12} {:>11} {:>11}", "subject", "A", "union", "intersection", "incl-excl", "split");
    let mut worst = 0.0f64;
    let mut refused = 0usize;
    let mut total = 0usize;
    for (name, a) in &loaded {
        let bb = a.bounding_box();
        let shift = (bb.max - bb.min) * 0.4;
        let b = BrepTransform::translate_solid(a, Vec3::new(shift.x, shift.y * 0.3, shift.z * 0.2));
        total += 1;
        let va = one(a);
        let vb = one(&b);
        let u = BooleanEngine::boolean_solids_exact_result(a, &b, BooleanOpType::Union, &tol).ok().map(|r| vol(&r.solids));
        let i = BooleanEngine::boolean_solids_exact_result(a, &b, BooleanOpType::Intersection, &tol).ok().map(|r| vol(&r.solids));
        let d = BooleanEngine::boolean_solids_exact_result(a, &b, BooleanOpType::Difference, &tol).ok().map(|r| vol(&r.solids));
        match (u, i, d) {
            (Some(u), Some(i), Some(d)) => {
                let incl = ((u + i) - (va + vb)).abs() / va.abs().max(1.0);
                let split = ((d + i) - va).abs() / va.abs().max(1.0);
                worst = worst.max(incl).max(split);
                println!("{name:<20} {va:>12.4} {u:>12.4} {i:>12.4} {incl:>11.3e} {split:>11.3e}");
            }
            _ => {
                refused += 1;
                println!("{name:<20} {va:>12.4} {:>12} {:>12} {:>11} {:>11}",
                    u.map(|x| format!("{x:.4}")).unwrap_or("REFUSED".into()),
                    i.map(|x| format!("{x:.4}")).unwrap_or("REFUSED".into()), "-", "-");
            }
        }
    }
    println!("\n恒等式の最悪残差 {worst:.3e}, 3演算そろわなかった検体 {refused} / {total}");
    let _ = PrimitiveBuilder::make_box(1.0, 1.0, 1.0);
}
