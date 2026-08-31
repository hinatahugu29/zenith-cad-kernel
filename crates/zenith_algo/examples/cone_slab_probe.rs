//! 全周円錐をスラブで切ると、**側面と切断面の交線が1本も出ません**。
//!
//! `foreign_boolean_stage_probe` は cone_full/slab を「面の組 3・交線 1」と
//! 出します。組は「切断面 × 側面の半分2つ」＋「切断面 × 底面」で、出ている
//! 1本は底面との直線です。**側面との交線が両方とも空**です。
//!
//! ここは、その3組を1つずつ、面の種類と境界箱つきで出します。
//!
//! ```bash
//! cargo run --release -p zenith_algo --example cone_slab_probe
//! ```

use std::path::PathBuf;

use zenith_algo::{BrepIntersectionBuilder, BrepTransform, PrimitiveBuilder, Regularizer};
use zenith_io::StepImporter;
use zenith_math::{Tolerance, Vec3};
use zenith_topo::{Face, FaceGeometry, Solid};

fn kind_name(face: &Face) -> &'static str {
    match &face.geometry {
        FaceGeometry::Plane(_) => "plane",
        FaceGeometry::Nurbs(_) => "nurbs",
        FaceGeometry::Coons(_) => "coons",
        FaceGeometry::Gordon(_) => "gordon",
        FaceGeometry::Triangular(_) => "tri",
    }
}

fn bounds(face: &Face) -> String {
    let points = face.outer_wire.sample_points(16);
    if points.is_empty() {
        return "(no boundary)".to_string();
    }
    let mut low = points[0];
    let mut high = points[0];
    for p in &points {
        low.x = low.x.min(p.x);
        low.y = low.y.min(p.y);
        low.z = low.z.min(p.z);
        high.x = high.x.max(p.x);
        high.y = high.y.max(p.y);
        high.z = high.z.max(p.z);
    }
    format!(
        "({:7.3} {:7.3} {:7.3})-({:7.3} {:7.3} {:7.3})",
        low.x, low.y, low.z, high.x, high.y, high.z
    )
}
fn faces(solid: &Solid) -> Vec<Face> {
    solid
        .outer_shell
        .faces
        .iter()
        .cloned()
        .chain(
            solid
                .inner_shells
                .iter()
                .flat_map(|shell| shell.faces.iter().cloned()),
        )
        .collect()
}

fn main() {
    let tol = Tolerance::default();
    let path = PathBuf::from(concat!(
        env!("CARGO_MANIFEST_DIR"),
        "/tests/fixtures/occ_reference_cone_full.step"
    ));
    let cone = StepImporter::import_solids_from_file(&path)
        .expect("the cone fixture must be readable")
        .into_iter()
        .next()
        .expect("one cone");

    // `foreign_boolean_stage_probe` と同じスラブ。境界箱は (-10,-10,0)-(10,10,20)。
    let slab = BrepTransform::translate_solid(
        &PrimitiveBuilder::make_box(12.0, 40.0, 40.0).expect("slab"),
        Vec3::new(-12.2, -20.0, -10.0),
    );

    let held_cone = Regularizer::hold_like_our_own(&cone, &tol);
    let held_slab = Regularizer::hold_like_our_own(&slab, &tol);
    let faces_a = faces(&held_cone);
    let faces_b = faces(&held_slab);

    println!("cone as held: {} face(s)", faces_a.len());
    for (index, face) in faces_a.iter().enumerate() {
        println!("  A{index} {:6} {}", kind_name(face), bounds(face));
    }
    println!("slab as held: {} face(s)", faces_b.len());
    for (index, face) in faces_b.iter().enumerate() {
        println!("  B{index} {:6} {}", kind_name(face), bounds(face));
    }

    let candidates =
        BrepIntersectionBuilder::collect_face_pair_candidates(&faces_a, &faces_b, &tol);
    println!("\n{} face pair candidate(s)", candidates.len());
    for candidate in &candidates {
        println!(
            "  A{} x B{} -> {}",
            candidate.face_a_index,
            candidate.face_b_index,
            format!("{:?}", candidate.kind)
                .chars()
                .take(90)
                .collect::<String>()
        );
    }

    let edges =
        BrepIntersectionBuilder::intersection_edge_candidates_from_face_pairs(candidates, &tol);
    println!("\n{} intersection edge(s)", edges.len());
    for edge in &edges {
        let (t0, t1) = edge.edge.curve.param_range();
        let start = edge.edge.curve.evaluate(t0);
        let end = edge.edge.curve.evaluate(t1);
        println!(
            "  A{} x B{}  ({:7.3} {:7.3} {:7.3}) -> ({:7.3} {:7.3} {:7.3})",
            edge.face_a_index, edge.face_b_index, start.x, start.y, start.z, end.x, end.y, end.z
        );
    }

    println!("\nthrough the verified API:");
    verified_report(&cone, &slab, &tol);
    for index in [1usize, 2] {
        println!("\nA{index} against B5:");
        if let FaceGeometry::Nurbs(surface) = &faces_a[index].geometry {
            march_report(surface, &tol);
        }
    }
}

/// 断っているのが**どの段か**を見る。面の組が `Unsupported` を返す道は
/// 「マーチングが枝を返さない」と「返したが精度に届かない」の2つで、
/// 上の表からは見分けが付きません。
fn march_report(surface: &zenith_geom::NurbsSurface3, tol: &Tolerance) {
    use zenith_geom::{ControlPoint3, IntersectionMarcher, KnotVector, NurbsSurface3};
    use zenith_math::Point3;

    // B5（x = -0.2、y は -20..20、z は -10..30）ちょうどの1次×1次パッチ。
    let corner = |y: f64, z: f64| ControlPoint3::unweighted(Point3::new(-0.2, y, z));
    let plane_patch = NurbsSurface3::new(
        1,
        1,
        vec![
            vec![corner(-20.0, -10.0), corner(-20.0, 30.0)],
            vec![corner(20.0, -10.0), corner(20.0, 30.0)],
        ],
        KnotVector::clamped_uniform(2, 1),
        KnotVector::clamped_uniform(2, 1),
    )
    .expect("plane patch");

    let extent = 40.0_f64;
    let first_step = (extent * 0.1).max(tol.linear * 100.0);
    println!("  first_step {first_step}, deviation limit {}", tol.linear);

    let seeds = IntersectionMarcher::find_seeds(&plane_patch, surface, 12, 4);
    println!("  seeds: {}", seeds.len());
    for (seed_u, seed_v) in seeds.iter().take(4) {
        let point = plane_patch.evaluate(*seed_u, *seed_v);
        print!(
            "    seed ({seed_u:.4} {seed_v:.4}) at ({:7.3} {:7.3} {:7.3}) ->",
            point.x, point.y, point.z
        );
        let mut step = first_step;
        let mut told = false;
        for _ in 0..6 {
            if let Some(marched) =
                IntersectionMarcher::march(&plane_patch, surface, *seed_u, *seed_v, step, 2048, tol)
            {
                if let Some((_, deviation)) =
                    IntersectionMarcher::fit_curve(&plane_patch, surface, &marched, 3)
                {
                    print!(
                        " step {step:.4}: {} pts, off {:.2e}, fit {:.2e};",
                        marched.points.len(),
                        marched.worst_off_surface,
                        deviation
                    );
                    told = true;
                } else {
                    print!(" step {step:.4}: {} pts, no fit;", marched.points.len());
                    told = true;
                }
            } else {
                print!(" step {step:.4}: no march;");
                told = true;
            }
            step *= 0.5;
        }
        if !told {
            print!(" nothing");
        }
        println!();
    }
}

/// 検証つきの公開 API が何と言うか。恒等式は通っているのに、ここが
/// 断るなら、直すべきはゲートか結果かのどちらかです。
fn verified_report(cone: &Solid, slab: &Solid, tol: &Tolerance) {
    use zenith_algo::{BooleanEngine, BooleanOpType};
    for op in [
        BooleanOpType::Difference,
        BooleanOpType::Intersection,
        BooleanOpType::Union,
    ] {
        match BooleanEngine::boolean_solids_exact_result(cone, slab, op, tol) {
            Ok(result) => println!("  {op:?}: ok, {} solid(s)", result.solids.len()),
            Err(err) => println!("  {op:?}: {err}"),
        }
    }
}
