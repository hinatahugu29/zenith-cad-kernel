//! 皿モミ穴が、どの寸法まで作れるのか。
//!
//! テストは 1 組の寸法（下穴 r3 / 皿 r6 / 90 度）だけを見ている。実務では
//! 下穴と皿の比も角度も変わるので、格子状に振って**どこから壊れるか**を測る。
//!
//! 壊れ方も見る。エラーを返すのは健全だが、返ってきたエラーが「寸法が範囲外」
//! ではなく p-curve のずれの羅列なら、呼び出し側は何を直せばよいか分からない。

use zenith_algo::{HoleBuilder, MassCalculator};
use zenith_math::Tolerance;
use zenith_tess::TessellationParams;

fn main() {
    let tol = Tolerance::default();
    let (w, d, h) = (40.0, 40.0, 20.0);
    let (cx, cy) = (20.0, 20.0);

    println!(
        "{:>7} {:>7} {:>7}  {:<9} {:>14}  {}",
        "hole_r", "cs_r", "angle", "outcome", "volume", "note"
    );
    println!("{}", "-".repeat(96));

    let mut ok = 0;
    let mut refused = 0;
    let mut unclear = 0;

    for hole_r in [2.0f64, 3.0, 4.0, 5.0] {
        for ratio in [1.5f64, 1.8, 2.0, 2.5] {
            for angle in [60.0f64, 82.0, 90.0, 120.0] {
                let cs_r = hole_r * ratio;
                match HoleBuilder::make_countersink_hole_box(w, d, h, hole_r, cs_r, angle, cx, cy) {
                    Ok(solid) => {
                        let volume = MassCalculator::compute_from_brep(
                            &solid,
                            &TessellationParams {
                                u_divisions: 32,
                                v_divisions: 32,
                            },
                        )
                        .volume;
                        let valid = solid.outer_shell.validate_closed(&tol).is_valid();
                        println!(
                            "{hole_r:>7.1} {cs_r:>7.1} {angle:>7.1}  {:<9} {volume:>14.4}  {}",
                            "ok",
                            if valid { "closed" } else { "NOT CLOSED" }
                        );
                        ok += 1;
                    }
                    Err(err) => {
                        // 「寸法が範囲外」と読めるか、それとも内部の数値の羅列か
                        let readable = err.contains("must")
                            || err.contains("Invalid")
                            || err.contains("smaller")
                            || err.contains("larger");
                        println!(
                            "{hole_r:>7.1} {cs_r:>7.1} {angle:>7.1}  {:<9} {:>14}  {}",
                            if readable { "refused" } else { "UNCLEAR" },
                            "-",
                            first_line(&err)
                        );
                        if readable {
                            refused += 1;
                        } else {
                            unclear += 1;
                        }
                    }
                }
            }
        }
    }

    println!("{}", "-".repeat(96));
    println!("ok {ok}   refused with a readable reason {refused}   refused unclearly {unclear}");
    if unclear > 0 {
        println!();
        println!("An unclear refusal is a defect of its own: the caller is told the p-curves");
        println!("disagree, not which dimension is out of range.");
    }
}

fn first_line(message: &str) -> String {
    let trimmed: String = message.chars().take(110).collect();
    trimmed.replace('\n', " ")
}
