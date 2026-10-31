//! 球3種(4分割／周期／OCC)の単体体積の符号を見る。
use std::path::PathBuf;
use zenith_algo::{MassCalculator, PrimitiveBuilder, Regularizer};
use zenith_io::StepImporter;
use zenith_math::Tolerance;
use zenith_tess::TessellationParams;
fn main() {
    let p = TessellationParams { u_divisions: 24, v_divisions: 24 };
    let tol = Tolerance::default();
    let s4 = PrimitiveBuilder::make_sphere(10.0).unwrap();
    let sp = PrimitiveBuilder::make_sphere_periodic(10.0).unwrap();
    println!("4分割 {:.6}", MassCalculator::compute_volume_from_brep(&s4, &p));
    println!("周期  {:.6}", MassCalculator::compute_volume_from_brep(&sp, &p));
    let path = PathBuf::from(concat!(env!("CARGO_MANIFEST_DIR"), "/tests/fixtures")).join("occ_reference_sphere.step");
    if let Ok(v) = StepImporter::import_solids_from_file(&path) {
        let s = Regularizer::hold_like_our_own(v.first().unwrap(), &tol);
        println!("OCC   {:.6}", MassCalculator::compute_volume_from_brep(&s, &p));
    }
    for (n, f) in sp.outer_shell.faces.iter().enumerate() {
        println!("周期 面{n} 向き {:?} 寄与 {:.6}", f.orientation, MassCalculator::compute_face_integral(f, &p).1);
    }
}
