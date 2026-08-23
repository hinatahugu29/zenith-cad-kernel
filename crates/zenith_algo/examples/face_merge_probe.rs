//! 同一平面の隣接面を併合すると、面と稜がどこまで減るか。
//!
//! 面数・稜数が実形状の2倍近くあることは、これまで測られていなかった。
//! ここでは併合の前後を並べ、**体積が動いていないこと**も同時に見る。

use zenith_algo::{
    BooleanEngine, BooleanOpType, BrepTransform, EdgeBlender, FaceMerger, HoleBuilder,
    MassCalculator, PrimitiveBuilder,
};
use zenith_math::{Tolerance, Vec3};
use zenith_tess::TessellationParams;
use zenith_topo::Solid;

fn volume_of(solid: &Solid) -> f64 {
    MassCalculator::compute_from_brep(
        solid,
        &TessellationParams {
            u_divisions: 48,
            v_divisions: 48,
        },
    )
    .volume
}

fn edges_of(solid: &Solid) -> usize {
    let mut ids: Vec<u64> = Vec::new();
    for face in &solid.outer_shell.faces {
        for wire in std::iter::once(&face.outer_wire).chain(face.inner_wires.iter()) {
            for oriented in &wire.edges {
                if !ids.contains(&oriented.edge.id) {
                    ids.push(oriented.edge.id);
                }
            }
        }
    }
    ids.len()
}

fn probe(name: &str, solid: &Solid, expected_faces: usize) -> bool {
    let tol = Tolerance::default();
    let before_volume = volume_of(solid);

    match FaceMerger::simplify_solid(solid, &tol) {
        Ok((merged, report)) => {
            let after_volume = volume_of(&merged);
            let drift = (after_volume - before_volume).abs() / before_volume.abs().max(1e-12);
            let valid = merged
                .outer_shell
                .validate_closed(&tol)
                .is_valid();
            println!(
                "{name:<34} {:>3} -> {:>3} faces  {:>3} -> {:>3} edges  (want {expected_faces})  volume drift {drift:.2e}  closed {valid}  blendable {} -> {}",
                solid.outer_shell.faces.len(),
                merged.outer_shell.faces.len(),
                edges_of(solid),
                edges_of(&merged),
                EdgeBlender::blendable_edges(solid).len(),
                EdgeBlender::blendable_edges(&merged).len(),
            );
            for reason in &report.skipped {
                println!("{:<34} skipped: {reason}", "");
            }
            valid && drift < 1e-12 && merged.outer_shell.faces.len() == expected_faces
        }
        Err(err) => {
            println!("{name:<34} failed: {err}");
            false
        }
    }
}

fn main() {
    let tol = Tolerance::default();
    let mut all_good = true;

    let boxed = PrimitiveBuilder::make_box(40.0, 40.0, 20.0).unwrap();
    all_good &= probe("box (nothing to merge)", &boxed, 6);

    all_good &= probe(
        "drilled box r8",
        &HoleBuilder::make_drilled_box(40.0, 40.0, 20.0, 8.0).unwrap(),
        // 側面4 + 環状の上下面2 + 円筒の4分割4。円筒を1枚にしないのは、
        // 全周1枚のパッチを OpenCASCADE が正しく積めないため（意図的）。
        10,
    );

    let corner = BrepTransform::translate_solid(
        &PrimitiveBuilder::make_box(20.0, 20.0, 20.0).unwrap(),
        Vec3::new(20.0, 20.0, 0.0),
    );
    let l_shape =
        BooleanEngine::boolean_solids_exact(&boxed, &corner, BooleanOpType::Difference, &tol)
            .unwrap();
    all_good &= probe("box minus corner box (L prism)", &l_shape, 8);

    let bore = BrepTransform::translate_solid(
        &PrimitiveBuilder::make_cylinder(6.0, 20.0).unwrap(),
        Vec3::new(20.0, 20.0, 0.0),
    );
    let bored =
        BooleanEngine::boolean_solids_exact(&boxed, &bore, BooleanOpType::Difference, &tol).unwrap();
    all_good &= probe("box minus a bore", &bored, 10);

    let raised = BrepTransform::translate_solid(
        &PrimitiveBuilder::make_box(20.0, 20.0, 30.0).unwrap(),
        Vec3::new(10.0, 10.0, 20.0),
    );
    let stacked =
        BooleanEngine::boolean_solids_exact(&boxed, &raised, BooleanOpType::Union, &tol).unwrap();
    all_good &= probe("box union raised block", &stacked, 11);

    println!("{}", "-".repeat(70));
    if all_good {
        println!("every case merged to the expected face count with no volume drift");
    } else {
        println!("at least one case did not reach the expected face count");
        std::process::exit(1);
    }
}
