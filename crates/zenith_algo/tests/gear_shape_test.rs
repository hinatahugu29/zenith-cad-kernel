//! 平歯車が実際にどんな形をしているか。
//!
//! 2026年8月20日まで、歯形はインボリュートではありませんでした。歯1つにつき
//! 4点を取って直線で結んだ多角形の押し出しで、圧力角は歯底半径の下限にしか
//! 効いていませんでした。このファイルは当時、その事実を**測って固定する**ため
//! に書かれ、「将来インボリュートにするなら、この2つは落ちるはずで、それが
//! 変えたことの証拠になる」と書いてありました。落としたので、書き直しました。
//!
//! いま測るのは
//!
//! 1. 断面積の閉じた式が、**別の道で積んだ数値積分**と合うこと
//! 2. 立体の体積がその閉じた式に乗り、標本点を増やすと次数どおり落ちること
//! 3. 歯面の上の点が、基礎円の**インボリュートの上**にあること
//! 4. その点が、昔の直線の歯形からは**はっきり離れている**こと
//! 5. `bore_radius` を渡しても**軸穴は開かない**こと

use std::f64::consts::PI;

use zenith_algo::{GearBuilder, MassCalculator};
use zenith_tess::TessellationParams;

const MODULE: f64 = 2.0;
const TEETH: usize = 18;
const ANGLE: f64 = 20.0;
const THICKNESS: f64 = 8.0;
const BORE: f64 = 6.0;

fn involute_of(angle: f64) -> f64 {
    angle.tan() - angle
}

/// 生成側と同じ規則で引き直した寸法。
struct Dimensions {
    base_radius: f64,
    tip_radius: f64,
    root_radius: f64,
    half_at_base: f64,
    half_at_tip: f64,
}

fn dimensions() -> Dimensions {
    let z = TEETH as f64;
    let alpha = ANGLE.to_radians();
    let pitch_radius = MODULE * z * 0.5;
    let base_radius = pitch_radius * alpha.cos();
    let tip_radius = pitch_radius + MODULE;
    let root_radius = (pitch_radius - 1.25 * MODULE)
        .max(BORE + 0.5 * MODULE)
        .min(base_radius);
    let half_at_base = PI / (2.0 * z) + involute_of(alpha);
    let half_at_tip = half_at_base - involute_of((base_radius / tip_radius).acos());
    Dimensions {
        base_radius,
        tip_radius,
        root_radius,
        half_at_base,
        half_at_tip,
    }
}

fn volume_of(solid: &zenith_topo::Solid, divisions: usize) -> f64 {
    MassCalculator::compute_from_brep(
        solid,
        &TessellationParams {
            u_divisions: divisions,
            v_divisions: divisions,
        },
    )
    .volume
}

/// 閉じた式を、**別の道**で確かめる。
///
/// 閉じた式はインボリュートの媒介変数 `t` で積んでいる。こちらは極角 `θ` の
/// ほうで `A = (1/2) ∫ R(θ)^2 dθ` を数値的に積む。歯車の輪郭は中心から見て
/// どの向きにも1点しかないので、`R(θ)` は一価である。歯面のところだけ
/// `ψ_b - inv(arccos(r_b/R)) = θ` を二分法で解いて `R` を出す。
///
/// 同じ式を書き写しているのではなく、積分変数も経路も違う。式のほうが
/// 間違っていれば、ここで合わなくなる。
#[test]
fn the_closed_form_area_agrees_with_an_independent_quadrature() {
    let d = dimensions();
    let z = TEETH as f64;

    // 歯面の外形半径を、極角のずれ psi から逆に解く。
    let radius_at = |psi: f64| -> f64 {
        let (mut low, mut high) = (0.0f64, d.half_at_base - d.half_at_tip + 1.0);
        for _ in 0..200 {
            let mid = 0.5 * (low + high);
            // t が増えると psi は減る。
            if d.half_at_base - (mid - mid.atan()) > psi {
                low = mid;
            } else {
                high = mid;
            }
        }
        let t = 0.5 * (low + high);
        d.base_radius * (1.0 + t * t).sqrt()
    };

    // 歯面ぶんの (1/2) R^2 dpsi を、複合シンプソンで積む。
    let steps = 200_000usize;
    let (a, b) = (d.half_at_tip, d.half_at_base);
    let h = (b - a) / steps as f64;
    let mut flank = radius_at(a).powi(2) + radius_at(b).powi(2);
    for step in 1..steps {
        let psi = a + h * step as f64;
        let weight = if step % 2 == 1 { 4.0 } else { 2.0 };
        flank += weight * radius_at(psi).powi(2);
    }
    flank *= h / 3.0 * 0.5;

    let quadrature = z
        * (d.tip_radius * d.tip_radius * d.half_at_tip
            + 2.0 * flank
            + d.root_radius * d.root_radius * (PI / z - d.half_at_base));

    let closed = GearBuilder::involute_profile_area(MODULE, TEETH, ANGLE, BORE).expect("area");
    let error = (closed - quadrature).abs() / quadrature;
    assert!(
        error < 1e-9,
        "the closed form says {closed} where the quadrature says {quadrature} ({error:.3e})"
    );
}

/// 体積が閉じた式に乗り、標本点を増やすと歯面の補間誤差が落ちること。
#[test]
fn the_gear_volume_is_its_involute_area_times_its_thickness() {
    let expected =
        GearBuilder::involute_profile_area(MODULE, TEETH, ANGLE, BORE).expect("area") * THICKNESS;

    let error_at = |samples: usize| {
        let gear =
            GearBuilder::make_spur_gear_with_samples(MODULE, TEETH, ANGLE, THICKNESS, BORE, samples)
                .expect("gear");
        assert!(
            gear.outer_shell.validate_closed(&zenith_math::Tolerance::default()).is_valid(),
            "the gear at {samples} samples is not a closed shell"
        );
        (volume_of(&gear, 48) - expected).abs() / expected
    };

    // 歯面は3次なので、標本点を倍にすれば誤差は十数倍から数十倍落ちる。
    let coarse = error_at(6);
    let fine = error_at(12);
    assert!(
        fine < coarse / 8.0,
        "doubling the flank samples took the error from {coarse:.3e} only to {fine:.3e}"
    );

    // 既定の標本数では、閉じた式に十分乗っていること。
    let default = error_at(GearBuilder::DEFAULT_FLANK_SAMPLES);
    assert!(
        default < 1e-8,
        "the default gear is {default:.3e} off its closed form"
    );
}

/// 歯面の上の点は、基礎円のインボリュートの上にある。
///
/// 半径 `R` のところでは、インボリュートは歯の中心から
/// `psi_b - inv(arccos(r_b/R))` だけ回った角にいなければならない。歯先円と
/// 歯底円の弧、半径方向の直線はこの範囲の外にいるので、`r_b < R < r_a` に
/// ある点はすべて歯面の点である。
#[test]
fn the_tooth_flanks_lie_on_the_involute_of_the_base_circle() {
    let (worst, checked) = flank_deviation(GearBuilder::DEFAULT_FLANK_SAMPLES);
    assert!(checked > 100, "only {checked} points landed on a flank");
    assert!(
        worst < 1e-6,
        "a flank point sits {worst:.3e} off the true involute ({checked} points checked)"
    );

    // 補間なので、標本点を増やせば落ちる。落ちないなら、ずれの出所は補間では
    // ない。3次なので、倍にすれば十数倍落ちてよいところを、緩めに4倍で見る。
    let coarse = flank_deviation(6).0;
    let fine = flank_deviation(12).0;
    assert!(
        fine < coarse / 4.0,
        "doubling the flank samples took the deviation from {coarse:.3e} only to {fine:.3e}"
    );
}

/// 歯面の点が真のインボリュートからどれだけ離れているか（最大値と、見た点数）。
///
/// 稜の上を細かく見ないと、山を見落とす。最初は稜あたり16点で見て 5.5e-6 と
/// 出ましたが、64点にすると 1.70e-5 でした——補間の誤差は標本点の**間**で
/// 最大になるので、粗く見ると小さく見えます。
fn flank_deviation(samples: usize) -> (f64, usize) {
    let d = dimensions();
    let gear =
        GearBuilder::make_spur_gear_with_samples(MODULE, TEETH, ANGLE, THICKNESS, BORE, samples)
            .expect("gear");
    let pitch_angle = 2.0 * PI / TEETH as f64;

    let mut worst: f64 = 0.0;
    let mut checked = 0usize;
    for face in &gear.outer_shell.faces {
        for oriented in &face.outer_wire.edges {
            for step in 0..=64 {
                let point = oriented.evaluate_normalized(step as f64 / 64.0);
                let radius = (point.x * point.x + point.y * point.y).sqrt();
                if radius <= d.base_radius + 1e-6 || radius >= d.tip_radius - 1e-6 {
                    continue;
                }
                let t = ((radius / d.base_radius).powi(2) - 1.0).max(0.0).sqrt();
                let offset = d.half_at_base - (t - t.atan());

                // どの歯の、どちら側の歯面かは問わない。いちばん近いものに当てる。
                let angle = point.y.atan2(point.x);
                let mut nearest = f64::INFINITY;
                for tooth in 0..TEETH {
                    for side in [-1.0f64, 1.0] {
                        let expected = tooth as f64 * pitch_angle + side * offset;
                        let mut gap = (angle - expected).rem_euclid(2.0 * PI);
                        if gap > PI {
                            gap -= 2.0 * PI;
                        }
                        nearest = nearest.min(gap.abs());
                    }
                }
                // 角のずれを弧長に直して測る。
                worst = worst.max(nearest * radius);
                checked += 1;
            }
        }
    }

    (worst, checked)
}

/// 歯面が、昔の直線の歯形から**はっきり離れている**こと。
///
/// インボリュートに乗っているだけでは足りない。直線でもインボリュートでも
/// 通る点はあるので、離れていることも見ておく。歯車の大きさ（歯先円半径 20）
/// に対して、0.1 以上離れていれば見間違えようがない。
#[test]
fn the_flanks_are_no_longer_the_straight_lines_they_used_to_be() {
    let d = dimensions();
    let gear =
        GearBuilder::make_spur_gear(MODULE, TEETH, ANGLE, THICKNESS, BORE).expect("gear");
    let pitch_angle = 2.0 * PI / TEETH as f64;
    let half_tooth = pitch_angle * 0.25;

    // 2026年8月20日までの歯形: 歯1つにつき4点を直線で結んだ多角形。
    let mut polygon: Vec<(f64, f64)> = Vec::with_capacity(TEETH * 4);
    for index in 0..TEETH {
        let centre = index as f64 * pitch_angle;
        for (angle, radius) in [
            (centre - half_tooth * 1.5, d.root_radius),
            (centre - half_tooth * 0.5, d.tip_radius),
            (centre + half_tooth * 0.5, d.tip_radius),
            (centre + half_tooth * 1.5, d.root_radius),
        ] {
            polygon.push((radius * angle.cos(), radius * angle.sin()));
        }
    }

    let distance_to_polygon = |x: f64, y: f64| {
        let mut best = f64::INFINITY;
        for index in 0..polygon.len() {
            let (x0, y0) = polygon[index];
            let (x1, y1) = polygon[(index + 1) % polygon.len()];
            let (dx, dy) = (x1 - x0, y1 - y0);
            let length_squared = dx * dx + dy * dy;
            let t = if length_squared <= f64::EPSILON {
                0.0
            } else {
                (((x - x0) * dx + (y - y0) * dy) / length_squared).clamp(0.0, 1.0)
            };
            best = best.min(((x - x0 - dx * t).powi(2) + (y - y0 - dy * t).powi(2)).sqrt());
        }
        best
    };

    let mut furthest: f64 = 0.0;
    for face in &gear.outer_shell.faces {
        for oriented in &face.outer_wire.edges {
            for step in 0..=16 {
                let point = oriented.evaluate_normalized(step as f64 / 16.0);
                furthest = furthest.max(distance_to_polygon(point.x, point.y));
            }
        }
    }

    assert!(
        furthest > 0.1,
        "the gear's boundary never gets further than {furthest:.3e} from the old \
         straight-sided polygon, so the flanks may not have changed"
    );
}

#[test]
fn the_bore_radius_does_not_actually_bore_a_hole() {
    let d = dimensions();
    let gear =
        GearBuilder::make_spur_gear(MODULE, TEETH, ANGLE, THICKNESS, BORE).expect("gear");

    // 穴があるなら内周ワイヤか、軸まわりの面が要る。どちらも無い。
    let holes: usize = gear
        .outer_shell
        .faces
        .iter()
        .map(|face| face.inner_wires.len())
        .sum();
    assert_eq!(holes, 0, "a bore would leave an inner wire behind");

    let mut nearest_to_axis = f64::INFINITY;
    for face in &gear.outer_shell.faces {
        for oriented in &face.outer_wire.edges {
            for step in 0..=8 {
                let point = oriented.evaluate_normalized(step as f64 / 8.0);
                nearest_to_axis =
                    nearest_to_axis.min((point.x * point.x + point.y * point.y).sqrt());
            }
        }
    }

    // 歯底は有理2次の**弧**なので、いちばん軸に近い点は歯底円そのものにある。
    // 多角形だった頃は弦だったので、中点が `r_f cos(pitch/8)` まで内側に
    // 入っていました（当時のテストはそちらを見ていました）。
    assert!(
        (nearest_to_axis - d.root_radius).abs() < 1e-9,
        "nothing should sit closer to the axis than the root circle {}, got {nearest_to_axis}",
        d.root_radius
    );
    assert!(
        d.root_radius > BORE,
        "the root circle {} should clear the bore radius {BORE}",
        d.root_radius
    );
    assert!(d.tip_radius > d.root_radius);
}
