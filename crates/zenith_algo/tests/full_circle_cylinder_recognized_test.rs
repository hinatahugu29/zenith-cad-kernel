//! **1 周ぶんの円柱も、円柱と読めること**（4-503）。
//!
//! 認識器は長らく **1/4 円のパッチ（制御点 3×2）しか受け付けません**でした。
//! **読んだファイルは、1 周を 1 枚で持っています**——実測
//! （`step_surface_shape_probe`）: **`linkrods.step` の曲面 31 枚のうち 4 枚**、
//! **`screw.step` は 6 枚のうち 3 枚**が「次数 2×1・制御点 9×2」です。
//!
//! **行の数を緩めただけでは、まだ落ちました。** `fit_section_circle` が
//! **外接円を、最初・真ん中・最後の標本**から取っていたからです——
//! **1 周では最初と最後が同じ点**で、円が決まりません。**1/3 ずつ離して**
//! 取るようにしました。
//!
//! ここでは**1 周ぶんの円柱を手で組んで**、**読み取った中心・半径・軸・高さ**
//! が閉じた式と合うことを見ます。

use zenith_algo::cylinder_patch_reading;
use zenith_geom::{ControlPoint3, KnotVector, NurbsSurface3};
use zenith_math::{Point3, Tolerance};

const RADIUS: f64 = 3.0;
const HEIGHT: f64 = 7.0;

/// 1 周ぶんの有理 2 次の円（制御点 9 個、重みは角で `cos 45°`）。
fn full_circle_cylinder() -> NurbsSurface3 {
    let corner = std::f64::consts::FRAC_1_SQRT_2;
    // 正方形の角を通る 9 点。四分円 4 つを繋いだ、教科書どおりの持ち方。
    let ring: [(f64, f64, f64); 9] = [
        (1.0, 0.0, 1.0),
        (1.0, 1.0, corner),
        (0.0, 1.0, 1.0),
        (-1.0, 1.0, corner),
        (-1.0, 0.0, 1.0),
        (-1.0, -1.0, corner),
        (0.0, -1.0, 1.0),
        (1.0, -1.0, corner),
        (1.0, 0.0, 1.0),
    ];
    let control_points: Vec<Vec<ControlPoint3>> = ring
        .iter()
        .map(|(x, y, weight)| {
            vec![
                ControlPoint3::new(Point3::new(x * RADIUS, y * RADIUS, 0.0), *weight),
                ControlPoint3::new(Point3::new(x * RADIUS, y * RADIUS, HEIGHT), *weight),
            ]
        })
        .collect();
    NurbsSurface3::new(
        2,
        1,
        control_points,
        KnotVector::new(vec![
            0.0, 0.0, 0.0, 1.0, 1.0, 2.0, 2.0, 3.0, 3.0, 4.0, 4.0, 4.0,
        ]),
        KnotVector::new(vec![0.0, 0.0, 1.0, 1.0]),
    )
    .expect("1 周ぶんの円柱")
}

#[test]
fn a_cylinder_held_as_one_full_turn_is_recognized() {
    let tol = Tolerance::default();
    let surface = full_circle_cylinder();

    let reading = cylinder_patch_reading(&surface, &tol)
        .expect("1 周ぶんの円柱が、円柱として読めるはずです");
    let (base_center, radius, axis, height) = reading;

    assert!(
        (radius - RADIUS).abs() < 1e-9,
        "半径が {radius}、閉じた式は {RADIUS}"
    );
    assert!(
        (height - HEIGHT).abs() < 1e-9,
        "高さが {height}、閉じた式は {HEIGHT}"
    );
    assert!(
        base_center.coords.norm() < 1e-9,
        "底の中心が原点からずれています: {base_center:?}"
    );
    assert!(
        (axis.z.abs() - 1.0).abs() < 1e-9,
        "軸が z 方向ではありません: {axis:?}"
    );
}
