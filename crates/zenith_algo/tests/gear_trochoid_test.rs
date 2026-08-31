//! 歯元フィレットが、ホブの歯先丸みが描く創成トロコイドになっているか。
//!
//! ここは長いあいだ、半径を線形・角度を smoothstep (`3u^2 - 2u^3`) で補間し、
//! `* 0.2` という係数で寄せたものを「ホブ盤創成トロコイド」と呼んでいました。
//! 工具のころがりは一度も計算していません。検査は「体積が正であること」しか
//! 見ていなかったので、S字に見えるかぎり通り続けます。
//!
//! トロコイドかどうかは、形を眺めても分かりません。**定義から測ります。**
//!
//! フィレットは、工具の歯先丸み（半径 $\rho$）が転がりながら掃いた円の
//! **包絡線**です。したがって次の2つが同時に成り立たなければなりません。
//!
//! - どの転がり角の工具円も、歯の中に**食い込まない**（フィレット上の点は
//!   すべて中心から $\rho$ 以上離れている）。
//! - どの工具円も、フィレットに**必ず触れる**（最も近い点でちょうど $\rho$）。
//!
//! 加えて、フィレットの外端はインボリュートの始まりと位置も接線も一致して
//! いなければなりません。インボリュートは創成運動とはまったく別の経路
//! （`flank_point` の解析式）で作られているので、これは独立な突き合わせです。

use zenith_algo::{GearBuilder, RootFilletGeneration};

const MODULE: f64 = 2.0;
const TEETH: usize = 18;
const PRESSURE_ANGLE: f64 = 20.0;
const BORE: f64 = 0.0;

fn generation() -> RootFilletGeneration {
    GearBuilder::root_fillet_generation(MODULE, TEETH, PRESSURE_ANGLE, BORE)
        .expect("the hob motion must be defined for a standard gear")
}

fn involute_of(angle: f64) -> f64 {
    angle.tan() - angle
}

/// 歯の中心を 0 とした、インボリュート歯面（右側）の角度差。
///
/// 創成運動とは無関係に、インボリュートの定義だけから出す。
fn involute_delta_at(radius: f64) -> f64 {
    let z = TEETH as f64;
    let alpha = PRESSURE_ANGLE.to_radians();
    let pitch_radius = MODULE * z * 0.5;
    let base_radius = pitch_radius * alpha.cos();
    let half_angle_at_base = std::f64::consts::PI / (2.0 * z) + involute_of(alpha);
    let t = ((radius / base_radius).powi(2) - 1.0).max(0.0).sqrt();
    -(half_angle_at_base - (t - t.atan()))
}

fn polar_to_xy(radius: f64, angle: f64) -> (f64, f64) {
    (radius * angle.cos(), radius * angle.sin())
}

#[test]
fn test_the_fillet_starts_on_the_root_circle_and_ends_on_the_involute() {
    let g = generation();

    let (inner_radius, _) = g.fillet[0];
    assert!(
        (inner_radius - g.root_radius).abs() < 1e-9,
        "the fillet must begin on the root circle: {inner_radius} against {}",
        g.root_radius
    );

    let (outer_radius, outer_delta) = *g.fillet.last().expect("samples");
    assert!(
        (outer_radius - g.form_radius).abs() < 1e-9,
        "the fillet must end at the form radius"
    );
    assert!(
        outer_radius > g.base_radius,
        "the form radius {outer_radius} must lie outside the base circle {}",
        g.base_radius
    );

    // ここが独立な突き合わせ。フィレットの角度はラックの転がりから、
    // インボリュートの角度は inv 関数から出ている。
    let want = involute_delta_at(outer_radius);
    assert!(
        (outer_delta - want).abs() < 1e-6,
        "the fillet ends at {outer_delta} rad from the tooth centre; the involute \
         starts at {want} rad. These come from unrelated derivations, so they \
         agreeing is the check that the rolling is set up right."
    );
}

#[test]
fn test_the_fillet_is_the_envelope_of_the_hob_tip_round() {
    let g = generation();
    let rho = g.tip_round_radius;
    assert!(rho > 0.0, "a hob with no tip round cuts no fillet");

    // 工具の中心が歯車軸にいちばん近づく転がり角。ここから両側へ振る。
    let (u_c, _) = g.tip_round_centre;
    let theta_deepest = -u_c / g.pitch_radius;
    let span = PRESSURE_ANGLE.to_radians() * 2.0;

    let centre_at = |theta: f64| {
        let (radius, delta) = g.tip_round_centre_at(theta, TEETH);
        polar_to_xy(radius, delta)
    };
    let distance =
        |a: (f64, f64), b: (f64, f64)| ((a.0 - b.0).powi(2) + (a.1 - b.1).powi(2)).sqrt();

    // (A) どの転がり角の工具円も、フィレットに食い込まない。
    //
    // 転がり角ごとに、フィレット上でいちばん近い点を探し、それが rho より
    // 内側にあれば食い込んでいる。ここは**最大**で取る。1つでも食い込めば
    // 工具が削り残しを作っているので、その形は創成されない。
    let mut worst_bite: f64 = 0.0;
    for step in 0..=800 {
        let theta = theta_deepest - span + 2.0 * span * step as f64 / 800.0;
        let centre = centre_at(theta);
        let closest = g
            .fillet
            .iter()
            .map(|(radius, delta)| distance(polar_to_xy(*radius, *delta), centre))
            .fold(f64::INFINITY, f64::min);
        worst_bite = worst_bite.max((rho - closest).max(0.0));
    }
    assert!(
        worst_bite < 1e-9,
        "the hob cuts {worst_bite} into the fillet; an envelope is never crossed"
    );

    // (B) フィレット上のどの点も、どれかの工具円に**乗っている**。
    //
    // 点ごとに、工具中心の軌跡までの最短距離を測る。それが rho でなければ、
    // その点は工具が削った跡ではない。ここも**最大**で取る。1点でも外れて
    // いれば、そこは創成された形ではない。
    //
    // (A) だけでは足りない。歯底円をそのまま伸ばした形は工具に食い込まないが、
    // 工具が触れないので創成された形でもない。
    let mut worst_miss: f64 = 0.0;
    for (radius, delta) in &g.fillet {
        let point = polar_to_xy(*radius, *delta);
        let mut closest = f64::INFINITY;
        for step in 0..=4000 {
            let theta = theta_deepest - span + 2.0 * span * step as f64 / 4000.0;
            closest = closest.min(distance(point, centre_at(theta)));
        }
        worst_miss = worst_miss.max((closest - rho).abs());
    }
    // 転がり角を 4000 で刻んでいるので、接点を厳密には踏まない。刻み幅から
    // 来る取りこぼしのオーダー（1e-6）で押さえる。
    assert!(
        worst_miss < 1e-5,
        "a fillet point sits {worst_miss} away from every tool circle; \
         an envelope point lies on one"
    );
}

/// 半径方向の直線でも smoothstep でもないこと。
///
/// どちらも歯の中心から見た角度差が**動かない**か、動いても工具の転がりとは
/// 無関係な動き方をします。本物のトロコイドは、歯底から立ち上がるにつれて
/// 歯面のほうへ回り込みます。
#[test]
fn test_the_fillet_actually_turns() {
    let g = generation();
    let (_, inner_delta) = g.fillet[0];
    let (_, outer_delta) = *g.fillet.last().expect("samples");

    // 歯底側は歯溝の奥（角度差が大きい負の値）、外側は歯面の付け根。
    assert!(
        inner_delta < outer_delta,
        "the fillet must sweep towards the flank as it rises: {inner_delta} -> {outer_delta}"
    );
    let sweep = outer_delta - inner_delta;
    assert!(
        sweep > 0.02,
        "a radial line would sweep 0; this one swept only {sweep} rad"
    );

    // 半径は単調に上がる。折り返すと輪郭が自分と交差する。
    for pair in g.fillet.windows(2) {
        assert!(
            pair[1].0 >= pair[0].0 - 1e-12,
            "the fillet radius went backwards: {} then {}",
            pair[0].0,
            pair[1].0
        );
    }
}

/// 歯数と圧力角を振っても成り立つこと。
#[test]
fn test_the_envelope_holds_across_sizes() {
    for (module, teeth, pressure) in [
        (1.0f64, 20usize, 20.0f64),
        (2.0, 18, 20.0),
        (3.0, 24, 20.0),
        (2.0, 30, 14.5),
        (2.5, 40, 25.0),
    ] {
        let g = GearBuilder::root_fillet_generation(module, teeth, pressure, 0.0)
            .unwrap_or_else(|e| panic!("m={module} z={teeth} a={pressure}: {e}"));

        assert!(
            (g.fillet[0].0 - g.root_radius).abs() < 1e-9,
            "m={module} z={teeth} a={pressure}: fillet must start on the root circle"
        );

        let (outer_radius, outer_delta) = *g.fillet.last().expect("samples");
        let z = teeth as f64;
        let alpha = pressure.to_radians();
        let pitch_radius = module * z * 0.5;
        let base_radius = pitch_radius * alpha.cos();
        let half_angle_at_base = std::f64::consts::PI / (2.0 * z) + involute_of(alpha);
        let t = ((outer_radius / base_radius).powi(2) - 1.0).max(0.0).sqrt();
        let want = -(half_angle_at_base - (t - t.atan()));

        assert!(
            (outer_delta - want).abs() < 1e-6,
            "m={module} z={teeth} a={pressure}: the fillet ends at {outer_delta}, \
             the involute starts at {want}"
        );
    }
}
