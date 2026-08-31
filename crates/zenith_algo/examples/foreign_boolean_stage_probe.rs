//! 断った配置を、**どの段で足りなくなったか**で並べる。
//!
//! 残りを1件ずつ追う前に、同じ機構で落ちているものがどれだけあるかを見ます。
//! 1つの直しが何件に効くかは、ここを見ないと分かりません。
//!
//! 出すのはパイプラインが自分で数えている内訳です。特に効くのは:
//!
//! - **交線 0** — 交わっていない。包含関係で答えるべき配置（4-47 で対応済み）。
//! - **キャップ 0 で縫えない稜が残る** — 切り口の面が作れていない。
//! - **非多様体の稜** — 面片の選び方が合っていない。
//!
//! ```bash
//! cargo run --release -p zenith_algo --example foreign_boolean_stage_probe
//! ```

use std::path::PathBuf;

use zenith_algo::{BooleanEngine, BooleanOpType, BrepTransform, PrimitiveBuilder};
use zenith_io::StepImporter;
use zenith_math::{Point3, Tolerance, Vec3};
use zenith_tess::{tessellate_solid, TessellationParams, TriangleMesh};
use zenith_topo::Solid;

/// **`foreign_boolean_probe` と同じ刻み**。切り手は境界箱から置くので、
/// ここが違うと同じ名前の配置が別の配置になります（実測でずれていました）。
fn params() -> TessellationParams {
    TessellationParams {
        u_divisions: 64,
        v_divisions: 64,
    }
}

fn mesh_bounds(mesh: &TriangleMesh) -> (Point3, Point3) {
    let mut low = Point3::new(f64::MAX, f64::MAX, f64::MAX);
    let mut high = Point3::new(f64::MIN, f64::MIN, f64::MIN);
    for vertex in &mesh.positions {
        low.x = low.x.min(vertex.x);
        low.y = low.y.min(vertex.y);
        low.z = low.z.min(vertex.z);
        high.x = high.x.max(vertex.x);
        high.y = high.y.max(vertex.y);
        high.z = high.z.max(vertex.z);
    }
    (low, high)
}

fn cutter(kind: &str, low: &Point3, high: &Point3) -> Option<Solid> {
    let size = Vec3::new(high.x - low.x, high.y - low.y, high.z - low.z);
    match kind {
        "slab" => {
            let solid =
                PrimitiveBuilder::make_box(size.x * 0.6, size.y * 2.0, size.z * 2.0).ok()?;
            Some(BrepTransform::translate_solid(
                &solid,
                Vec3::new(
                    low.x - size.x * 0.11,
                    low.y - size.y * 0.5,
                    low.z - size.z * 0.5,
                ),
            ))
        }
        "drill" => {
            let radius = size.x.min(size.y) * 0.18;
            let solid = PrimitiveBuilder::make_cylinder(radius, size.z * 3.0).ok()?;
            Some(BrepTransform::translate_solid(
                &solid,
                Vec3::new(
                    (low.x + high.x) * 0.5,
                    (low.y + high.y) * 0.5,
                    low.z - size.z,
                ),
            ))
        }
        "corner" => {
            let solid =
                PrimitiveBuilder::make_box(size.x * 0.45, size.y * 0.45, size.z * 0.45).ok()?;
            Some(BrepTransform::translate_solid(
                &solid,
                Vec3::new(
                    high.x - size.x * 0.30,
                    high.y - size.y * 0.30,
                    high.z - size.z * 0.30,
                ),
            ))
        }
        _ => None,
    }
}

fn main() {
    let tol = Tolerance::default();
    let subjects = [
        "cone",
        "cone_full",
        "cylinder_nurbs",
        "elliptic_prism",
        "extruded_spline",
        "revolved_ring",
        "sphere",
        "sphere_capped",
        "torus",
        "torus_segment",
    ];
    let ops = [
        ("difference", BooleanOpType::Difference),
        ("intersection", BooleanOpType::Intersection),
        ("union", BooleanOpType::Union),
    ];

    println!(
        "{:<17} {:<7} {:<13} {:>5} {:>5} {:>5} {:>5} {:>5} {:>5} {:>5}",
        "subject", "cutter", "op", "pairs", "edges", "caps", "pieces", "unmat", "nonmf", "samedir"
    );
    println!("{}", "-".repeat(92));

    // 縫えない稜の本数ごとに、何件あるかを数える。
    let mut by_unmatched: std::collections::BTreeMap<usize, usize> = Default::default();
    let mut refused = 0usize;

    for name in subjects {
        let path = PathBuf::from(concat!(env!("CARGO_MANIFEST_DIR"), "/tests/fixtures"))
            .join(format!("occ_reference_{name}.step"));
        let Ok(solids) = StepImporter::import_solids_from_file(&path) else {
            continue;
        };
        let Some(a) = solids.first() else { continue };
        let (low, high) = mesh_bounds(&tessellate_solid(a, &params()));

        for kind in ["slab", "drill", "corner"] {
            let Some(b) = cutter(kind, &low, &high) else {
                continue;
            };
            for (op_name, op) in ops {
                // 通るものはここでは見ない。断ったものだけ並べる。
                match BooleanEngine::boolean_solids_exact_result(a, &b, op, &tol) {
                    Ok(_) => continue,
                    Err(err) if std::env::var_os("ZENITH_SHOW_REASON").is_some() => {
                        println!(
                            "{name:<17} {kind:<7} {op_name:<13} REASON {}",
                            err.as_str().chars().take(900).collect::<String>()
                        );
                    }
                    Err(_) => {}
                }
                refused += 1;
                match BooleanEngine::prepare_exact_boolean(a, &b, op, &tol) {
                    Ok(r) => {
                        *by_unmatched
                            .entry(r.selected_face_unmatched_edge_use_count)
                            .or_insert(0) += 1;
                        println!(
                            "{name:<17} {kind:<7} {op_name:<13} {:>5} {:>5} {:>5} {:>5} {:>5} {:>5} {:>5}",
                            r.face_pair_candidate_count,
                            r.intersection_edge_candidate_count,
                            r.planar_cap_face_count,
                            r.selected_face_piece_count,
                            r.selected_face_unmatched_edge_use_count,
                            r.selected_face_non_manifold_edge_use_count,
                            r.selected_face_same_direction_edge_use_count,
                        );
                    }
                    Err(err) => println!(
                        "{name:<17} {kind:<7} {op_name:<13} preparation failed: {}",
                        err.chars().take(40).collect::<String>()
                    ),
                }
            }
        }
    }

    println!("{}", "-".repeat(92));
    println!("{refused} refused");
    println!();
    println!("unmatched edge uses -> how many cases");
    for (unmatched, count) in &by_unmatched {
        println!("  {unmatched:>3} -> {count}");
    }
    println!();
    println!("caps 0 with unmatched edges left means the cut face was never built.");
    println!("That is one mechanism, not many, if the same column is empty");
    println!("across most rows.");
}
