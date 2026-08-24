//! 極（退化した行）の近くで、点を曲面へ射影する。
//!
//! # なぜこれが要るか
//!
//! 球の極では u をいくら動かしても 3D の点が変わりません。粗い格子の当たりが
//! 極に落ちると、そこからは**どちらへ動いても遠くなる**ので、詰めも
//! ニュートンも動けず、**極そのものが答えとして残ります**。
//!
//! 実測（2026/08/25、HANDOVER 4-79）: 半径10の球を `(20,10,10)` に置き、
//! **球面上の点**を射影すると **0.446 ずれた極**が返っていました。点は面の
//! 上にあるので、正しい答えは 0 です。
//!
//! **同じ球を原点に置くと起きません。** 粗格子の当たりがたまたま極に
//! ならないからです。つまり**置き方で答えが変わっていました**。ここは
//! その置き方を固定して測ります。
//!
//! 射影は p-curve の導出・距離・内外判定・面積の検算が全部使うので、ここが
//! 静かに外れると、上の階が理由の分からない失敗をします。

use zenith_geom::{ControlPoint3, ExtremumEngine, KnotVector, NurbsSurface3};
use zenith_math::Point3;

/// 半径 `radius`、中心 `centre` の球の八分パッチ（極を1つ含む）。
///
/// 有理2次 × 有理2次。v = 0 の行が極に潰れています。
fn sphere_octant(centre: Point3, radius: f64) -> NurbsSurface3 {
    let w = std::f64::consts::FRAC_1_SQRT_2;
    // 経度方向（u）に 90 度、緯度方向（v）に 90 度。v=0 が北極。
    let grid: Vec<Vec<ControlPoint3>> = (0..3)
        .map(|i| {
            // u 方向: (1,0) -> (1,1)/√2 -> (0,1)
            let (cu, su, wu) = match i {
                0 => (1.0, 0.0, 1.0),
                1 => (1.0, 1.0, w),
                _ => (0.0, 1.0, 1.0),
            };
            (0..3)
                .map(|j| {
                    // v 方向: 極 -> 45度 -> 赤道
                    let (rho, z, wv) = match j {
                        0 => (0.0, 1.0, 1.0),
                        1 => (1.0, 1.0, w),
                        _ => (1.0, 0.0, 1.0),
                    };
                    ControlPoint3::new(
                        Point3::new(
                            centre.x + radius * rho * cu,
                            centre.y + radius * rho * su,
                            centre.z + radius * z,
                        ),
                        wu * wv,
                    )
                })
                .collect()
        })
        .collect();

    NurbsSurface3::new(
        2,
        2,
        grid,
        KnotVector::clamped_uniform(3, 2),
        KnotVector::clamped_uniform(3, 2),
    )
    .expect("sphere octant")
}

/// 面の上の点を射影したら、距離は 0 でなければなりません。
///
/// **極のすぐ隣を含めて**測ります。そこが、粗格子の当たりが極になる帯です。
#[test]
fn projecting_a_point_that_is_on_the_surface_returns_zero_even_beside_a_pole() {
    // 置き方で答えが変わっていたので、原点と原点以外の両方で測ります。
    for centre in [
        Point3::origin(),
        Point3::new(20.0, 10.0, 10.0),
        Point3::new(-7.5, 3.25, 100.0),
    ] {
        let surface = sphere_octant(centre, 10.0);
        let ((u_min, u_max), (v_min, v_max)) = surface.param_range();

        let mut worst = 0.0f64;
        let mut worst_uv = (0.0, 0.0);
        // v を極（v_min）に寄せた帯を厚めに見ます。
        for i in 0..=12 {
            let u = u_min + (u_max - u_min) * i as f64 / 12.0;
            for fraction in [0.0, 1e-4, 1e-3, 0.01, 0.05, 0.1, 0.25, 0.5, 1.0] {
                let v = v_min + (v_max - v_min) * fraction;
                // **面そのものの上の点**。正しい距離は 0。
                let point = surface.evaluate(u, v);
                let projection = ExtremumEngine::point_to_surface(point, &surface, 48, 1e-13)
                    .expect("the projection must not fail on a point of the surface itself");
                if projection.distance > worst {
                    worst = projection.distance;
                    worst_uv = (u, v);
                }
            }
        }

        assert!(
            worst < 1e-9,
            "centre {centre:?}: a point on the surface projected {worst:.3e} away \
             (worst at u={}, v={}). Before 4-79 this returned the pole itself, 0.446 off.",
            worst_uv.0,
            worst_uv.1
        );
    }
}

/// 極そのものを射影しても、極が返ること。
///
/// 逃げ道を足したせいで、**本当に極が答えのときに極を外す**ようになっては
/// いけません。
#[test]
fn the_pole_itself_still_projects_onto_the_pole() {
    let centre = Point3::new(20.0, 10.0, 10.0);
    let surface = sphere_octant(centre, 10.0);
    let ((u_min, _), (v_min, _)) = surface.param_range();
    let pole = surface.evaluate(u_min, v_min);

    let projection =
        ExtremumEngine::point_to_surface(pole, &surface, 48, 1e-13).expect("projection");
    assert!(
        projection.distance < 1e-12,
        "the pole should project onto itself, got {:.3e}",
        projection.distance
    );

    // 極の**真上**（球の外側）も、極が最近点です。
    let above = Point3::new(centre.x, centre.y, centre.z + 15.0);
    let projection =
        ExtremumEngine::point_to_surface(above, &surface, 48, 1e-13).expect("projection");
    assert!(
        (projection.distance - 5.0).abs() < 1e-9,
        "a point 5 above the pole is 5 from the surface, got {:.6}",
        projection.distance
    );
}
