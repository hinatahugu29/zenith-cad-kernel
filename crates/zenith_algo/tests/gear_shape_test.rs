//! 平歯車が実際にどんな形をしているか。
//!
//! 文書は長らく「インボリュート平歯車」と書いていましたが、歯形は
//! インボリュートではありません。歯1つにつき「歯底左・歯先左・歯先右・
//! 歯底右」の4点を取り、直線で結んだ多角形の押し出しです。
//!
//! 名前と実物が違うことは、測れば分かります。ここでは
//!
//! 1. 体積が**多角形の面積 × 厚み**と一致すること（曲がった歯面なら合わない）
//! 2. 歯面の上の点が、その多角形の**辺の上**にあること
//! 3. `bore_radius` を渡しても**軸穴は開かない**こと
//!
//! を測って、いまの形を固定します。将来インボリュートにするなら、1 と 2 は
//! 落ちるはずで、それが「変えた」ことの証拠になります。

use std::f64::consts::PI;

use zenith_algo::{GearBuilder, MassCalculator};
use zenith_math::Point3;
use zenith_tess::TessellationParams;

struct GearProfile {
    points: Vec<(f64, f64)>,
    root_radius: f64,
    tip_radius: f64,
}

/// 生成側と同じ規則で作った、歯車のプロファイル多角形。
fn spur_gear_profile(module: f64, teeth: usize, pressure_angle_deg: f64, bore: f64) -> GearProfile {
    let z = teeth as f64;
    let alpha = pressure_angle_deg.to_radians();
    let pitch_radius = module * z * 0.5;
    let base_radius = pitch_radius * alpha.cos();
    let tip_radius = pitch_radius + module;
    let root_radius = (pitch_radius - 1.25 * module)
        .max(base_radius * 0.8)
        .max(bore + 0.5 * module);

    let pitch_angle = 2.0 * PI / z;
    let half_tooth = pitch_angle * 0.25;

    let mut points = Vec::with_capacity(teeth * 4);
    for index in 0..teeth {
        let centre = index as f64 * pitch_angle;
        for (angle, radius) in [
            (centre - half_tooth * 1.5, root_radius),
            (centre - half_tooth * 0.5, tip_radius),
            (centre + half_tooth * 0.5, tip_radius),
            (centre + half_tooth * 1.5, root_radius),
        ] {
            points.push((radius * angle.cos(), radius * angle.sin()));
        }
    }

    GearProfile {
        points,
        root_radius,
        tip_radius,
    }
}

fn polygon_area(points: &[(f64, f64)]) -> f64 {
    let mut twice = 0.0;
    for index in 0..points.len() {
        let (x0, y0) = points[index];
        let (x1, y1) = points[(index + 1) % points.len()];
        twice += x0 * y1 - x1 * y0;
    }
    twice.abs() * 0.5
}

/// 点から多角形の辺までの最短距離。
fn distance_to_polygon(point: (f64, f64), polygon: &[(f64, f64)]) -> f64 {
    let mut best = f64::INFINITY;
    for index in 0..polygon.len() {
        let (x0, y0) = polygon[index];
        let (x1, y1) = polygon[(index + 1) % polygon.len()];
        let (dx, dy) = (x1 - x0, y1 - y0);
        let length_squared = dx * dx + dy * dy;
        let t = if length_squared <= f64::EPSILON {
            0.0
        } else {
            (((point.0 - x0) * dx + (point.1 - y0) * dy) / length_squared).clamp(0.0, 1.0)
        };
        let (cx, cy) = (x0 + dx * t, y0 + dy * t);
        best = best.min(((point.0 - cx).powi(2) + (point.1 - cy).powi(2)).sqrt());
    }
    best
}

#[test]
fn the_gear_volume_is_its_polygon_area_times_its_thickness() {
    let (module, teeth, angle, thickness, bore) = (2.0, 18usize, 20.0, 8.0, 6.0);
    let gear = GearBuilder::make_spur_gear(module, teeth, angle, thickness, bore).expect("gear");

    let profile = spur_gear_profile(module, teeth, angle, bore);
    let expected = polygon_area(&profile.points) * thickness;

    let volume = MassCalculator::compute_from_brep(
        &gear,
        &TessellationParams {
            u_divisions: 48,
            v_divisions: 48,
        },
    )
    .volume;

    let error = (volume - expected).abs() / expected;
    assert!(
        error < 1e-9,
        "the gear is {volume} where the polygon prism is {expected} ({error:.3e})"
    );
}

#[test]
fn the_tooth_flanks_are_straight_lines_and_not_involute_curves() {
    let (module, teeth, angle, thickness, bore) = (2.0, 18usize, 20.0, 8.0, 6.0);
    let gear = GearBuilder::make_spur_gear(module, teeth, angle, thickness, bore).expect("gear");
    let profile = spur_gear_profile(module, teeth, angle, bore);

    // 側面の点を、プロファイル多角形の辺に当てて測る。インボリュートなら
    // 歯面は弧を描くので、直線の多角形からは離れる。
    let mut worst: f64 = 0.0;
    for face in &gear.outer_shell.faces {
        for oriented in &face.outer_wire.edges {
            for step in 0..=8 {
                let point: Point3 = oriented.evaluate_normalized(step as f64 / 8.0);
                // 上下のキャップの縁も同じ多角形の上にあるので、まとめて見る。
                worst = worst.max(distance_to_polygon((point.x, point.y), &profile.points));
            }
        }
    }

    assert!(
        worst < 1e-9,
        "the gear's boundary is {worst:.3e} away from the straight-sided polygon, \
         so the flanks are no longer straight - if that is deliberate, this test \
         and builder_audit's spur_gear_profile_area both need rewriting"
    );
}

#[test]
fn the_bore_radius_does_not_actually_bore_a_hole() {
    let (module, teeth, angle, thickness, bore) = (2.0, 18usize, 20.0, 8.0, 6.0);
    let gear = GearBuilder::make_spur_gear(module, teeth, angle, thickness, bore).expect("gear");
    let profile = spur_gear_profile(module, teeth, angle, bore);

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
            for step in 0..=4 {
                let point = oriented.evaluate_normalized(step as f64 / 4.0);
                nearest_to_axis = nearest_to_axis.min((point.x * point.x + point.y * point.y).sqrt());
            }
        }
    }
    // 歯底は**弧ではなく弦**なので、中点は歯底円より内側に入る。歯底の張る
    // 角は 1周 / (4 * 歯数) なので、いちばん軸に近い点は
    // `root_radius * cos(pitch_angle / 8)` にある。最初はここを歯底円
    // そのものだと書いて落ちた——期待値のほうが違っていた。
    let pitch_angle = 2.0 * PI / teeth as f64;
    let closest = profile.root_radius * (pitch_angle / 8.0).cos();
    assert!(
        (nearest_to_axis - closest).abs() < 1e-9,
        "nothing should sit closer to the axis than {closest} (the root chord's middle),          got {nearest_to_axis}"
    );
    assert!(
        profile.root_radius > bore,
        "the root circle {} should clear the bore radius {bore}",
        profile.root_radius
    );
    assert!(profile.tip_radius > profile.root_radius);
}
