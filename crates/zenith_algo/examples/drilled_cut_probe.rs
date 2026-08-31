//! 穴の開いた立体を、箱で削れるか。
//!
//! `closure_probe` で、穴あきの板に箱を当てると「未実装」で断られました。
//! 同じ板に**円柱**を当てるのは通ります（連鎖ドリルは成功する）。実務では
//! 「穴を開けてから外形を整える」が普通なので、ここが切れていると使えません。
//!
//! ナイフの置き方を変えて、位置の問題なのか組み合わせの問題なのかを分けます。

use zenith_algo::{
    BooleanEngine, BooleanOpType, BrepTransform, HoleBuilder, MassCalculator, PrimitiveBuilder,
};
use zenith_math::{Tolerance, Vec3};
use zenith_tess::TessellationParams;
use zenith_topo::Solid;

fn params() -> TessellationParams {
    TessellationParams {
        u_divisions: 24,
        v_divisions: 24,
    }
}

fn volume(solid: &Solid) -> f64 {
    MassCalculator::compute_from_brep(solid, &params()).volume
}

fn main() {
    let tol = Tolerance::default();
    let plain = PrimitiveBuilder::make_box(30.0, 30.0, 15.0).expect("plain box");
    let drilled = HoleBuilder::make_drilled_box(30.0, 30.0, 15.0, 5.0).expect("drilled box");

    println!("plain box    {:.4}", volume(&plain));
    println!(
        "drilled box  {:.4}  (hole radius 5 through 15)",
        volume(&drilled)
    );
    println!();
    println!("{:<44} {:<10} {:<10}", "knife", "plain", "drilled");
    println!("{}", "-".repeat(70));

    let knives: Vec<(&str, Solid)> = vec![
        (
            "a slab off the top (z >= 12)",
            BrepTransform::translate_solid(
                &PrimitiveBuilder::make_box(60.0, 60.0, 10.0).expect("k"),
                Vec3::new(-15.0, -15.0, 12.0),
            ),
        ),
        (
            "a corner block, clear of the hole",
            BrepTransform::translate_solid(
                &PrimitiveBuilder::make_box(6.0, 6.0, 40.0).expect("k"),
                Vec3::new(-3.0, -3.0, -10.0),
            ),
        ),
        (
            "a slot across the middle, through the hole",
            BrepTransform::translate_solid(
                &PrimitiveBuilder::make_box(60.0, 6.0, 40.0).expect("k"),
                Vec3::new(-15.0, 12.0, -10.0),
            ),
        ),
        (
            "a slab off one side (x >= 24)",
            BrepTransform::translate_solid(
                &PrimitiveBuilder::make_box(20.0, 60.0, 40.0).expect("k"),
                Vec3::new(24.0, -15.0, -10.0),
            ),
        ),
        (
            "a box swallowing the whole solid",
            BrepTransform::translate_solid(
                &PrimitiveBuilder::make_box(80.0, 80.0, 80.0).expect("k"),
                Vec3::new(-25.0, -25.0, -30.0),
            ),
        ),
    ];

    for (name, knife) in &knives {
        let mut cells = Vec::new();
        for subject in [&plain, &drilled] {
            let cell = match BooleanEngine::boolean_solids_exact_result(
                subject,
                knife,
                BooleanOpType::Difference,
                &tol,
            ) {
                Ok(result) => {
                    let after: f64 = result.solids.iter().map(volume).sum();
                    format!("{after:.3} ({}s)", result.solids.len())
                }
                Err(err) => format!(
                    "ERR {}",
                    err.split("; selected")
                        .nth(1)
                        .unwrap_or("")
                        .chars()
                        .take(200)
                        .collect::<String>()
                ),
            };
            cells.push(cell);
        }
        println!(
            "{name:<44}
    plain   {}
    drilled {}",
            cells[0], cells[1]
        );
    }

    println!();
    println!("Where plain works and drilled does not, the hole is what stops it,");
    println!("not the knife.");
}
