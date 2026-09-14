//! **円柱を読み取る精度**を測る（4-457）。
//!
//! # なぜ要るのか
//!
//! **交線の位置は、読み取った中心と半径だけで決まります**
//! （`intersect_ruling_plane_cylinder_patch` は
//! `patch.radius` と `patch.base_center` から交点を出します）。
//!
//! **読み取りが 1e-7 ずれると、交点は 1e-7 ではずれません。**
//! **接する所の近くでは、`r / sqrt(r² - d²)` 倍に拡大されます**——
//! 半径 5・離れ 4.99 なら **15.8 倍**。4-456 で測った「鎖が繋がらない
//! 1.537e-6」は、**この拡大の後の姿**です。
//!
//! # 何を測るか
//!
//! **穴のある板を、いつもの道（スケッチ → 押し出し）で作り**、
//! **その穴の面を認識器に渡して、中心 (15, 15)・半径 5 からの
//! ずれを出します。**
//!
//! **閉じた式が分かっている形だけ**を測ります。**測れるのは
//! 読み取りの精度そのもの**で、ブーリアンの結果ではありません。

use zenith_algo::{cylinder_patch_reading, extrude_sketch, SketchSolver, WorkPlane};
use zenith_math::Tolerance;
use zenith_topo::FaceGeometry;

fn add_circle(solver: &mut SketchSolver, cx: f64, cy: f64, r: f64) {
    let centre = solver.add_point(cx, cy);
    let east = solver.add_point(cx + r, cy);
    let north = solver.add_point(cx, cy + r);
    let west = solver.add_point(cx - r, cy);
    let south = solver.add_point(cx, cy - r);
    solver.add_arc(centre, east, north, true);
    solver.add_arc(centre, north, west, true);
    solver.add_arc(centre, west, south, true);
    solver.add_arc(centre, south, east, true);
}

fn rectangle(x0: f64, y0: f64, width: f64, height: f64) -> SketchSolver {
    let mut solver = SketchSolver::new();
    let a = solver.add_point(x0, y0);
    let b = solver.add_point(x0 + width, y0);
    let c = solver.add_point(x0 + width, y0 + height);
    let d = solver.add_point(x0, y0 + height);
    solver.add_line(a, b);
    solver.add_line(b, c);
    solver.add_line(c, d);
    solver.add_line(d, a);
    solver
}

/// **穴の 4 枚の四半円筒すべてで、中心と半径が閉じた式に乗るか。**
///
/// **許容は 1e-9。** **公差（1e-6）ではありません**——ここは
/// 有理 NURBS の四半円から中心と半径を戻すだけで、**近似の入る所が
/// ありません**。**1e-9 を超えるなら、戻し方に欠陥があります。**
#[test]
fn cylinder_patch_reading_matches_closed_form() {
    let tol = Tolerance::default();
    let plane = WorkPlane::xy();
    let mut sketch = rectangle(0.0, 0.0, 30.0, 30.0);
    add_circle(&mut sketch, 15.0, 15.0, 5.0);
    let solid = extrude_sketch(&sketch, &plane, 6.0, &tol).expect("穴のある板");

    let mut readings = 0;
    let mut worst_radius = 0.0_f64;
    let mut worst_center = 0.0_f64;
    for (index, face) in solid.outer_shell.faces.iter().enumerate() {
        let FaceGeometry::Nurbs(surface) = &face.geometry else {
            continue;
        };
        let Some((center, radius, _axis, _height)) = cylinder_patch_reading(surface, &tol) else {
            continue;
        };
        readings += 1;
        let radius_error = (radius - 5.0).abs();
        let center_error = ((center.x - 15.0).powi(2) + (center.y - 15.0).powi(2)).sqrt();
        worst_radius = worst_radius.max(radius_error);
        worst_center = worst_center.max(center_error);
        assert!(
            radius_error <= 1e-9,
            "面 {index}: 半径の読み取りが {radius:.15} で、5 から {radius_error:.3e} ずれています"
        );
        assert!(
            center_error <= 1e-9,
            "面 {index}: 中心の読み取りが ({:.15}, {:.15}) で、(15, 15) から {center_error:.3e} ずれています",
            center.x,
            center.y
        );
    }

    assert_eq!(
        readings, 4,
        "穴は四半円筒 4 枚のはずですが、認識できたのは {readings} 枚です"
    );
    println!(
        "円柱の読み取り: 4 枚、半径の最悪 {worst_radius:.3e}、中心の最悪 {worst_center:.3e}"
    );
}
