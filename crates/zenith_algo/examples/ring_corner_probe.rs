//! 輪の角を箱で削れるか。読んだ立体と、同じ形をビルダーで作ったもので。
//!
//! `foreign_boolean_probe` の残りで最大の塊が `revolved_ring` です。読んだ
//! ものだけが落ちるなら持ち方の話（4-43）、**両方落ちるならブーリアン自身の
//! 穴**で、そのときは読み込みを挟まない再現で追うほうが速い。
//!
//! ```bash
//! cargo run --release -p zenith_algo --example ring_corner_probe
//! ```

use std::path::PathBuf;

use zenith_algo::{BooleanEngine, BooleanOpType, BrepTransform, MassCalculator, PrimitiveBuilder};
use zenith_io::StepImporter;
use zenith_math::{Tolerance, Vec3};
use zenith_tess::TessellationParams;
use zenith_topo::Solid;

fn params() -> TessellationParams {
    TessellationParams {
        u_divisions: 48,
        v_divisions: 48,
    }
}

fn volume(solids: &[Solid]) -> f64 {
    solids
        .iter()
        .map(|solid| MassCalculator::compute_from_brep(solid, &params()).volume)
        .sum()
}

/// 外半径 10・内半径 4・高さ 6 の輪をビルダーで作る。
fn builder_ring() -> Option<Solid> {
    let tol = Tolerance::default();
    let outer = PrimitiveBuilder::make_cylinder(10.0, 6.0).ok()?;
    let bore = BrepTransform::translate_solid(
        &PrimitiveBuilder::make_cylinder(4.0, 30.0).ok()?,
        Vec3::new(0.0, 0.0, -12.0),
    );
    BooleanEngine::boolean_solids_exact_result(&outer, &bore, BooleanOpType::Difference, &tol)
        .ok()?
        .solids
        .into_iter()
        .next()
}

fn read_ring() -> Option<Solid> {
    let path = PathBuf::from(concat!(
        env!("CARGO_MANIFEST_DIR"),
        "/tests/fixtures/occ_reference_revolved_ring.step"
    ));
    StepImporter::import_solids_from_file(&path)
        .ok()?
        .into_iter()
        .next()
}

/// `foreign_boolean_probe` と同じ角の箱。境界箱は (-10,-10,0)-(10,10,6)。
fn corner_block() -> Option<Solid> {
    let solid = PrimitiveBuilder::make_box(20.0 * 0.45, 20.0 * 0.45, 6.0 * 0.45).ok()?;
    Some(BrepTransform::translate_solid(
        &solid,
        Vec3::new(10.0 - 20.0 * 0.30, 10.0 - 20.0 * 0.30, 6.0 - 6.0 * 0.30),
    ))
}

fn report(label: &str, ring: &Solid, block: &Solid) {
    let tol = Tolerance::default();
    let before = volume(std::slice::from_ref(ring));
    println!(
        "  {label}: V(A) {before:.6}, {} face(s)",
        ring.outer_shell.faces.len()
    );
    for (name, op) in [
        ("difference", BooleanOpType::Difference),
        ("intersection", BooleanOpType::Intersection),
        ("union", BooleanOpType::Union),
    ] {
        match BooleanEngine::boolean_solids_exact_result(ring, block, op, &tol) {
            Ok(result) => println!(
                "    {name:<13} {} solid(s)  {:.6}",
                result.solids.len(),
                volume(&result.solids)
            ),
            Err(err) => println!(
                "    {name:<13} refused: {}",
                err.split(';')
                    .next()
                    .unwrap_or(&err)
                    .chars()
                    .take(52)
                    .collect::<String>()
            ),
        }
    }
}

fn main() {
    let Some(block) = corner_block() else {
        println!("the corner block could not be built");
        return;
    };

    println!("cutting the corner of a ring (outer 10, bore 4, height 6)");
    match builder_ring() {
        Some(ring) => report("builder", &ring, &block),
        None => println!("  builder: the ring itself could not be built"),
    }
    match read_ring() {
        Some(ring) => report("read", &ring, &block),
        None => println!("  read: the fixture could not be read"),
    }

    println!();
    println!("OpenCASCADE on the read one (tools/occ_cut_reference.py revolved_ring corner):");
    println!("  V(A) 1583.362697  V(A-B) 1553.253150  V(A^B) 30.109547  V(AuB) 1771.953150");
    println!();
    println!("If both routes refuse, the gap is in the boolean, not in how the");
    println!("file is held, and it can be chased without reading a file at all.");
}
