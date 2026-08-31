//! 円錐の角を箱で削ると、削り残す。どこで落ちているかを段ごとに見る。
//!
//! 恒等式が指した誤答です（4-52）。積と和は OpenCASCADE と合っているのに、
//! 差だけが**満杯の円錐**を返します。
//!
//! ```text
//! こちら       V(A-B) = 3267.256360   (= V(A))
//! OpenCASCADE  V(A-B) = 3267.253121   削るべき 0.003239
//! ```
//!
//! 縫合は 0/0/0 で通っており、面片の構成も正しく見えます。だから見るのは
//! **縫合より後**——選ばれた面片から立体を組む段、縫い直す段、そして
//! 体積を積む段です。
//!
//! ```bash
//! cargo run --release -p zenith_algo --example cone_corner_probe
//! ```

use zenith_algo::{
    BooleanEngine, BooleanOpType, BrepIntersectionBuilder, BrepTransform, MassCalculator,
    PrimitiveBuilder, Regularizer,
};
use zenith_math::{Tolerance, Vec3};
use zenith_tess::{tessellate_solid, TessellationParams};
use zenith_topo::Solid;

fn params() -> TessellationParams {
    TessellationParams {
        u_divisions: 64,
        v_divisions: 64,
    }
}

fn brep_volume(solid: &Solid) -> f64 {
    MassCalculator::compute_from_brep(solid, &params()).volume
}

/// メッシュから積んだ体積。B-Rep の積分と食い違えば、**面ではなく積分**の
/// 話になります。どちらも同じ面から作るので、揃わないほうがおかしい。
fn mesh_volume(solid: &Solid) -> f64 {
    MassCalculator::compute_from_mesh(&tessellate_solid(solid, &params())).volume
}

fn main() {
    let tol = Tolerance::default();

    // `foreign_boolean_probe` と同じ配置。境界箱は (-10,-10,0)-(10,10,20)。
    let cone = PrimitiveBuilder::make_cone(10.0, 4.0, 20.0).expect("cone");
    let block = BrepTransform::translate_solid(
        &PrimitiveBuilder::make_box(9.0, 9.0, 9.0).expect("block"),
        Vec3::new(4.0, 4.0, 14.0),
    );

    println!("cone r10/r4 h20 minus a 9x9x9 block at (4,4,14)");
    println!("  OpenCASCADE: V(A) 3267.256360, V(A-B) 3267.253121, removed 0.003239");
    println!();

    let a = Regularizer::hold_like_our_own(&cone, &tol);
    let b = Regularizer::hold_like_our_own(&block, &tol);
    println!(
        "  A: {} face(s), brep {:.6}, mesh {:.6}",
        a.outer_shell.faces.len(),
        brep_volume(&a),
        mesh_volume(&a)
    );
    println!(
        "  B: {} face(s), brep {:.6}",
        b.outer_shell.faces.len(),
        brep_volume(&b)
    );

    for (name, op) in [
        ("difference", BooleanOpType::Difference),
        ("intersection", BooleanOpType::Intersection),
        ("union", BooleanOpType::Union),
    ] {
        println!();
        println!("  --- {name}");

        // 縫合の直前まで。選ばれた面片から、そのまま立体を組む。
        let assembly = BrepIntersectionBuilder::collect_boolean_shell_assembly(&a, &b, op, &tol);
        let report = &assembly.assembly.stitch_report;
        println!(
            "      pieces {}, caps {}, stitch {}/{}/{}",
            assembly.assembly.selected_face_pieces.len(),
            assembly.assembly.cap_face_count,
            report.unmatched_edge_use_count,
            report.non_manifold_edge_use_count,
            report.same_direction_edge_use_count
        );

        match BrepIntersectionBuilder::build_solids_from_selected_face_pieces(
            &assembly.assembly.selected_face_pieces,
            &tol,
        ) {
            Ok(solids) => {
                for (index, solid) in solids.iter().enumerate() {
                    println!(
                        "      before sewing, solid {index}: {} face(s), brep {:.6}, mesh {:.6}",
                        solid.outer_shell.faces.len(),
                        brep_volume(solid),
                        mesh_volume(solid)
                    );
                }
            }
            Err(err) => println!("      build refused: {err}"),
        }

        // 公開の口。縫い直しを通ったあと。
        match BooleanEngine::boolean_solids_exact_result_unverified(&a, &b, op, &tol) {
            Ok(result) => {
                for (index, solid) in result.solids.iter().enumerate() {
                    println!(
                        "      after sewing,  solid {index}: {} face(s), brep {:.6}, mesh {:.6}",
                        solid.outer_shell.faces.len(),
                        brep_volume(solid),
                        mesh_volume(solid)
                    );
                }
            }
            Err(err) => println!(
                "      refused: {}",
                err.split(';')
                    .next()
                    .unwrap_or(&err)
                    .chars()
                    .take(60)
                    .collect::<String>()
            ),
        }
    }

    println!();
    println!("If the volume is already full before sewing, the wrong faces were");
    println!("selected. If it changes at sewing, the sewing is what loses it. If");
    println!("brep and mesh disagree on the same solid, it is the integral.");
}
