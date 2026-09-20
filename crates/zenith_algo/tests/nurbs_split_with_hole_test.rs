//! **穴のある曲面が、いまどう割れているか**（4-494 の性格試験）。
//!
//! 曲面を割る 3 つの道（水平・垂直・断面）は、どれも**外周の巡回だけ**から
//! 切片を作り、入口で
//! 「NURBS face splitting with inner wires is not implemented yet」と
//! 断ります——**`linkrods` の分割の断り 12 本がこれ**（4-493）。
//!
//! **しかし、それで割れなくなってはいません。** 断られた先に
//! **一般の道**（`FaceSplitter::split_by_curve`。**4-491 で穴に対応済み**）が
//! あり、そちらが受け取ります。**この試験は、それを固定します**——
//! 4-494 で 3 つの断りを外して穴を配り直す手を入れてみましたが、
//! **`linkrods` の数字は 1 つも動かず**（49 当たり / 10 飛ばし / あぶれ 47、
//! **完全に同じ**）、**この試験も変更の有無で通る**ので、**入れませんでした**。
//!
//! 見るのは 2 つ——**穴が上の片に 1 つだけ付くこと**と、
//! **2 片の面積の和が元の面に戻ること**です。
//! 和が戻らなければ、穴が消えているか、二重に数えています。

use zenith_algo::{BrepIntersectionBuilder, MassCalculator, PrimitiveBuilder};
use zenith_geom::{ControlPoint3, KnotVector, NurbsCurve3};
use zenith_math::{Point3, Tolerance};
use zenith_tess::TessellationParams;
use zenith_topo::{Edge, Face, OrientedEdge, Vertex, Wire};

const R: f64 = 5.0;

fn point_on(theta: f64, z: f64) -> Point3 {
    Point3::new(R * theta.cos(), R * theta.sin(), z)
}

/// **円弧そのもの**（有理2次）。接線の交点を中央の制御点に、重みは cos(Δ/2)。
fn arc(from: f64, to: f64, z: f64) -> Edge {
    let half = (to - from) / 2.0;
    let weight = half.cos();
    let mid = (from + to) / 2.0;
    let middle = Point3::new(
        R / weight * mid.cos(),
        R / weight * mid.sin(),
        z,
    );
    let curve = NurbsCurve3::new(
        2,
        vec![
            ControlPoint3::unweighted(point_on(from, z)),
            ControlPoint3::new(middle, weight),
            ControlPoint3::unweighted(point_on(to, z)),
        ],
        KnotVector::new(vec![0.0, 0.0, 0.0, 1.0, 1.0, 1.0]),
    )
    .expect("an arc on the cylinder");
    Edge::new(
        curve,
        Vertex::from_point(point_on(from, z)),
        Vertex::from_point(point_on(to, z)),
        1e-9,
    )
}

/// 母線（まっすぐ、z 方向）。
fn ruling(theta: f64, from_z: f64, to_z: f64) -> Edge {
    let from = point_on(theta, from_z);
    let to = point_on(theta, to_z);
    let curve = NurbsCurve3::bspline_from_points(1, vec![from, to]).expect("a ruling");
    Edge::new(
        curve,
        Vertex::from_point(from),
        Vertex::from_point(to),
        1e-9,
    )
}

fn area(face: &Face) -> f64 {
    let params = TessellationParams {
        u_divisions: 96,
        v_divisions: 96,
    };
    MassCalculator::compute_face_integral(face, &params).0
}

#[test]
fn a_curved_face_with_a_hole_still_splits() {
    let tol = Tolerance::default();
    let cylinder = PrimitiveBuilder::make_cylinder(R, 10.0).expect("a cylinder");
    // 側面の 1/4 パッチ（θ は 0 から π/2、z は 0 から 10）。
    let patch = &cylinder.outer_shell.faces[0];

    // **穴**は θ[0.4,1.0]・z[6,8]——**上半分の中に、すっかり収まります**。
    let hole = Wire::new(
        vec![
            arc(0.4, 1.0, 6.0),
            ruling(1.0, 6.0, 8.0),
            arc(1.0, 0.4, 8.0),
            ruling(0.4, 8.0, 6.0),
        ]
        .into_iter()
        .map(OrientedEdge::forward)
        .collect(),
    );
    let holed = Face::new(
        patch.geometry.clone(),
        patch.outer_wire.clone(),
        vec![hole],
        patch.orientation,
        patch.tolerance,
    );

    // 切り込みは z = 5 の水平な断面（パッチを端から端へ渡ります）。
    let cut = arc(0.0, std::f64::consts::FRAC_PI_2, 5.0);

    let pieces = BrepIntersectionBuilder::split_face_by_edge(&holed, &cut, &tol)
        .expect("a curved face with a hole should split");
    assert_eq!(pieces.len(), 2, "上と下の 2 枚");

    let with_hole: Vec<_> = pieces
        .iter()
        .filter(|piece| !piece.inner_wires.is_empty())
        .collect();
    assert_eq!(with_hole.len(), 1, "穴は 1 枚にだけ付きます");
    assert_eq!(with_hole[0].inner_wires.len(), 1);

    // **面積の和**——穴が消えていれば大きくなり、二重に数えれば小さくなります。
    let whole = area(&holed);
    let sum: f64 = pieces.iter().map(area).sum();
    assert!(
        (sum - whole).abs() < whole * 1e-3,
        "面積の和 {sum} が元の {whole} に戻りません"
    );
}
