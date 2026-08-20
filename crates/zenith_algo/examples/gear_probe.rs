//! インボリュート歯車を、2つの物差しに当てる。
//!
//! 1. 歯面の点が、真のインボリュートからどれだけ離れているか（位置）
//! 2. 体積が、閉じた式からどれだけ離れているか
//!
//! 歯面だけが3次の補間なので、どちらも標本点を増やせば4乗で落ちるはずである。
//! ただし**形の忠実さを表すのは 1 のほう**で、2 は補間の誤差が標本点の間で
//! 符号を変えるぶんが打ち消し合うので、実力より良く出ることがある。

use zenith_algo::{GearBuilder, MassCalculator};
use zenith_math::Tolerance;
use zenith_tess::TessellationParams;

fn flank_error(samples: usize) -> f64 {
    use std::f64::consts::PI;
    let (module, teeth, angle) = (2.0f64, 18usize, 20.0f64);
    let z = teeth as f64;
    let alpha: f64 = angle.to_radians();
    let pitch_radius = module * z * 0.5;
    let base_radius = pitch_radius * alpha.cos();
    let tip_radius = pitch_radius + module;
    let half_at_base = PI / (2.0 * z) + (alpha.tan() - alpha);
    let pitch_angle = 2.0 * PI / z;

    let gear = GearBuilder::make_spur_gear_with_samples(module, teeth, angle, 8.0, 6.0, samples)
        .expect("gear");
    let mut worst: f64 = 0.0;
    for face in &gear.outer_shell.faces {
        for oriented in &face.outer_wire.edges {
            for step in 0..=64 {
                let point = oriented.evaluate_normalized(step as f64 / 64.0);
                let radius = (point.x * point.x + point.y * point.y).sqrt();
                if radius <= base_radius + 1e-6 || radius >= tip_radius - 1e-6 {
                    continue;
                }
                let t = ((radius / base_radius).powi(2) - 1.0).max(0.0).sqrt();
                let offset = half_at_base - (t - t.atan());
                let angle_here = point.y.atan2(point.x);
                let mut nearest = f64::INFINITY;
                for tooth in 0..teeth {
                    for side in [-1.0f64, 1.0] {
                        let expected = tooth as f64 * pitch_angle + side * offset;
                        let mut gap = (angle_here - expected).rem_euclid(2.0 * PI);
                        if gap > PI {
                            gap -= 2.0 * PI;
                        }
                        nearest = nearest.min(gap.abs());
                    }
                }
                worst = worst.max(nearest * radius);
            }
        }
    }
    worst
}

fn main() {
    println!("flank deviation from the true involute (m2 z18 alpha20)");
    let mut previous: Option<f64> = None;
    for samples in [4usize, 6, 8, 12, 16, 24, 32, 48] {
        let worst = flank_error(samples);
        let gain = previous.map(|p: f64| p / worst).unwrap_or(f64::NAN);
        println!("  samples {samples:3}: worst {worst:.4e}  gain {gain:6.2}");
        previous = Some(worst);
    }

    let tol = Tolerance::default();
    let params = TessellationParams {
        u_divisions: 64,
        v_divisions: 64,
    };

    for (module, teeth, angle, thickness, bore) in
        [(2.0, 18, 20.0, 8.0, 5.0), (3.0, 24, 20.0, 10.0, 8.0), (1.5, 40, 14.5, 6.0, 10.0)]
    {
        let area = GearBuilder::involute_profile_area(module, teeth, angle, bore).unwrap();
        let expected = area * thickness;
        println!(
            "\nm{module} z{teeth} alpha{angle} t{thickness}: closed form area {area:.9}, volume {expected:.9}"
        );

        let mut previous: Option<f64> = None;
        for samples in [4usize, 6, 8, 12, 16, 24, 32] {
            let solid = GearBuilder::make_spur_gear_with_samples(
                module, teeth, angle, thickness, bore, samples,
            )
            .unwrap_or_else(|err| panic!("samples {samples}: {err}"));

            let report = solid.outer_shell.validate_closed(&tol);
            let volume = MassCalculator::compute_from_brep(&solid, &params).volume;
            let relative = (volume - expected).abs() / expected;
            let ratio = previous.map(|p: f64| p / relative).unwrap_or(f64::NAN);
            println!(
                "  samples {samples:3}: faces {:4}  closed {:5}  volume {volume:.9}  rel {relative:.3e}  gain {ratio:6.1}",
                solid.outer_shell.faces.len(),
                report.is_valid(),
            );
            if !report.is_valid() {
                println!("    errors: {:?}", report.errors);
            }
            previous = Some(relative);
        }
    }
}
