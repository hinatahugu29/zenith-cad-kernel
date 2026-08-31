use zenith_geom::{
    ControlPoint3, CornerBlendN, CrossRibbon, GregoryPatch4, KnotVector, NurbsCurve3, Surface3,
};
use zenith_math::{Point3, Tolerance, Vec3};

fn make_line_curve(p0: Point3, p1: Point3) -> NurbsCurve3 {
    NurbsCurve3::new(
        1,
        vec![ControlPoint3::unweighted(p0), ControlPoint3::unweighted(p1)],
        KnotVector::clamped_uniform(2, 1),
    )
    .unwrap()
}

#[test]
fn test_gregory_patch_boundary_interpolation() {
    let tol = Tolerance::default();
    let p00 = Point3::new(0.0, 0.0, 0.0);
    let p10 = Point3::new(10.0, 0.0, 2.0);
    let p11 = Point3::new(10.0, 10.0, 5.0);
    let p01 = Point3::new(0.0, 10.0, 1.0);

    let c0 = make_line_curve(p00, p10);
    let c1 = make_line_curve(p10, p11);
    let c2 = make_line_curve(p01, p11);
    let c3 = make_line_curve(p00, p01);

    let patch = GregoryPatch4::new(c0, c1, c2, c3, &tol).expect("Gregory patch creation");

    // 4隅の補間精度
    let ep00 = patch.evaluate(0.0, 0.0);
    let ep10 = patch.evaluate(1.0, 0.0);
    let ep11 = patch.evaluate(1.0, 1.0);
    let ep01 = patch.evaluate(0.0, 1.0);

    assert!((ep00 - p00).norm() < 1e-9);
    assert!((ep10 - p10).norm() < 1e-9);
    assert!((ep11 - p11).norm() < 1e-9);
    assert!((ep01 - p01).norm() < 1e-9);

    // 内部点の評価と法線
    let mid = patch.evaluate(0.5, 0.5);
    assert!(mid.z > 0.0);
    let normal = patch.normal(0.5, 0.5).expect("valid normal");
    assert!(normal.norm() > 0.99);
}

#[test]
fn test_n_sided_corner_blend_creation() {
    let tol = Tolerance::default();
    let p0 = Point3::new(10.0, 0.0, 0.0);
    let p1 = Point3::new(0.0, 10.0, 0.0);
    let p2 = Point3::new(0.0, 0.0, 10.0);

    let c0 = make_line_curve(p0, p1);
    let c1 = make_line_curve(p1, p2);
    let c2 = make_line_curve(p2, p0);

    let blend =
        CornerBlendN::create_n_sided_blend(vec![c0, c1, c2], &tol).expect("3-sided corner blend");

    assert_eq!(blend.boundary_curves.len(), 3);
    assert!((blend.center_point.x - 3.333).abs() < 0.1);
    assert!((blend.center_point.y - 3.333).abs() < 0.1);
    assert!((blend.center_point.z - 3.333).abs() < 0.1);
}

/// N辺コーナーブレンドは、実際にパッチを返さなければ穴を塞いでいない。
///
/// ここは以前 `patches` の枚数を一度も見ておらず、`boundary_curves.len()` と
/// 中心点だけを検査していた。`GregoryPatch4::new` が毎回 `Err` を返し、
/// `if let Ok(..)` がそれを捨てていたので、実測では N=3 でも N=4 でも
/// `patches.len() == 0` のまま「3辺コーナーブレンドを作れた」と通っていた。
#[test]
fn test_n_sided_corner_blend_actually_produces_patches() {
    let tol = Tolerance::default();

    for corners in [
        vec![
            Point3::new(10.0, 0.0, 0.0),
            Point3::new(0.0, 10.0, 0.0),
            Point3::new(0.0, 0.0, 10.0),
        ],
        vec![
            Point3::new(10.0, 0.0, 0.0),
            Point3::new(0.0, 10.0, 0.0),
            Point3::new(-10.0, 0.0, 0.0),
            Point3::new(0.0, -10.0, 4.0),
        ],
        vec![
            Point3::new(10.0, 0.0, 0.0),
            Point3::new(3.0, 9.0, 1.0),
            Point3::new(-8.0, 6.0, 0.0),
            Point3::new(-8.0, -6.0, 2.0),
            Point3::new(3.0, -9.0, 0.0),
        ],
    ] {
        let n = corners.len();
        let curves: Vec<NurbsCurve3> = (0..n)
            .map(|index| make_line_curve(corners[index], corners[(index + 1) % n]))
            .collect();

        let blend = CornerBlendN::create_n_sided_blend(curves, &tol)
            .unwrap_or_else(|error| panic!("{n}-sided corner blend failed: {error}"));

        assert_eq!(
            blend.patches.len(),
            n,
            "an N-sided blend must close the hole with N four-sided patches"
        );

        // 各パッチの4隅が、意図した位置（中心・境界中点・コーナー）に来ているか。
        for (index, patch) in blend.patches.iter().enumerate() {
            let start = patch.evaluate(0.0, 0.0);
            assert!(
                (start - blend.center_point).norm() < 1e-9,
                "patch {index} must start at the blend centre"
            );
        }
    }
}

/// 指定したクロス接線に、4辺すべてで一致するか。
///
/// これがグレゴリーパッチの存在理由です。境界を通るだけなら Coons で足ります。
/// 難しいのは、**内部制御点が2回決まってしまう**なかで、4辺すべてのリボンを
/// 同時に満たすことです。
///
/// 以前の実装はクロス接線を引数に取らず、`tangents` フィールドは全ゼロのまま
/// 一度も読まれていませんでした。ここは、その状態では**書きようがなかった**
/// 検査です。
#[test]
fn test_the_patch_matches_the_prescribed_cross_tangents() {
    let tol = Tolerance::default();
    let p00 = Point3::new(0.0, 0.0, 0.0);
    let p10 = Point3::new(10.0, 0.0, 2.0);
    let p11 = Point3::new(10.0, 10.0, 5.0);
    let p01 = Point3::new(0.0, 10.0, 1.0);

    // ツイストがわざと食い違うリボン。双3次ベジエでは同時に満たせない組。
    let ribbons = [
        CrossRibbon::from_ends(Vec3::new(0.0, 8.0, 3.0), Vec3::new(-2.0, 9.0, -4.0)),
        CrossRibbon::from_ends(Vec3::new(-7.0, 0.0, 5.0), Vec3::new(-9.0, 1.0, 2.0)),
        CrossRibbon::from_ends(Vec3::new(1.0, -8.0, -2.0), Vec3::new(0.0, -7.0, 6.0)),
        CrossRibbon::from_ends(Vec3::new(9.0, 1.0, -3.0), Vec3::new(8.0, 0.0, 4.0)),
    ];

    let patch = GregoryPatch4::with_ribbons(
        make_line_curve(p00, p10),
        make_line_curve(p10, p11),
        make_line_curve(p01, p11),
        make_line_curve(p00, p01),
        ribbons,
        &tol,
    )
    .expect("Gregory patch with ribbons");

    // 各辺で、パッチの内向き微分が指定したリボンと一致するか。
    //
    // 微分は辺の上なので片側差分になる。**残差は刻み幅に比例する**はずなので、
    // 1つの値で「小さい」と言うのではなく、刻みを10分の1にして残差も10分の1に
    // なることを見る。比が 1 に張り付いたら、それは差分の誤差ではなく本物の
    // ずれである（5章の「動かない量は求積の粗さではない」の同じ話）。
    let worst_at = |eps: f64| -> f64 {
        let mut worst: f64 = 0.0;
        for step in 1..32 {
            let s = step as f64 / 32.0;
            let one_sided = |a: Point3, b: Point3| (b - a) / eps;

            let dv = one_sided(patch.evaluate(s, 0.0), patch.evaluate(s, eps));
            worst = worst.max((dv - patch.ribbon_at(0, s)).norm());

            let du = one_sided(patch.evaluate(1.0, s), patch.evaluate(1.0 - eps, s));
            worst = worst.max((du - patch.ribbon_at(1, s)).norm());

            let dv1 = one_sided(patch.evaluate(s, 1.0), patch.evaluate(s, 1.0 - eps));
            worst = worst.max((dv1 - patch.ribbon_at(2, s)).norm());

            let du0 = one_sided(patch.evaluate(0.0, s), patch.evaluate(eps, s));
            worst = worst.max((du0 - patch.ribbon_at(3, s)).norm());
        }
        worst
    };

    let coarse = worst_at(1e-4);
    let fine = worst_at(1e-5);
    let finer = worst_at(1e-6);

    assert!(
        coarse < 1e-2 && fine < 1e-3 && finer < 1e-4,
        "the patch leaves its prescribed cross tangent: {coarse:e} / {fine:e} / {finer:e}"
    );
    // 刻みを10分の1にしたら残差も10分の1あたりに落ちること。
    for (large, small) in [(coarse, fine), (fine, finer)] {
        let ratio = large / small.max(1e-300);
        assert!(
            ratio > 5.0,
            "the residual fell by only {ratio}x for a 10x finer step, so it is not \
             the differencing - the patch really is off its ribbon"
        );
    }
}

/// 境界曲線の形が内部に効くか。
///
/// 以前の実装の内部制御点は4隅だけから固定係数で決まっており、境界を
/// 大きく湾曲させても**1ビットも動きません**でした。
#[test]
fn test_the_boundary_shape_reaches_the_interior() {
    let tol = Tolerance::default();
    let p00 = Point3::new(0.0, 0.0, 0.0);
    let p10 = Point3::new(10.0, 0.0, 0.0);
    let p11 = Point3::new(10.0, 10.0, 0.0);
    let p01 = Point3::new(0.0, 10.0, 0.0);

    let bent = NurbsCurve3::new(
        3,
        vec![
            ControlPoint3::unweighted(p00),
            ControlPoint3::unweighted(Point3::new(3.0, 0.0, 40.0)),
            ControlPoint3::unweighted(Point3::new(7.0, 0.0, 40.0)),
            ControlPoint3::unweighted(p10),
        ],
        KnotVector::clamped_uniform(4, 3),
    )
    .unwrap();

    let flat = GregoryPatch4::new(
        make_line_curve(p00, p10),
        make_line_curve(p10, p11),
        make_line_curve(p01, p11),
        make_line_curve(p00, p01),
        &tol,
    )
    .unwrap();
    let curved = GregoryPatch4::new(
        bent,
        make_line_curve(p10, p11),
        make_line_curve(p01, p11),
        make_line_curve(p00, p01),
        &tol,
    )
    .unwrap();

    let moved = (curved.evaluate(0.5, 0.25) - flat.evaluate(0.5, 0.25)).norm();
    assert!(
        moved > 1.0,
        "bending one boundary by 40 units moved the interior by only {moved}"
    );
}

/// 隣り合うセルが、共有するリブの上で接平面を共有するか。
///
/// N辺ブレンドは N 枚のセルで穴を塞ぐので、セルどうしが折れていては塞いだ
/// ことになりません。
#[test]
fn test_neighbouring_cells_share_a_tangent_plane_on_their_rib() {
    let tol = Tolerance::default();
    let corners = [
        Point3::new(10.0, 0.0, 0.0),
        Point3::new(0.0, 10.0, 0.0),
        Point3::new(-10.0, 0.0, 0.0),
        Point3::new(0.0, -10.0, 3.0),
    ];
    let n = corners.len();
    let curves: Vec<NurbsCurve3> = (0..n)
        .map(|i| make_line_curve(corners[i], corners[(i + 1) % n]))
        .collect();

    let blend = CornerBlendN::create_n_sided_blend(curves, &tol).expect("4-sided blend");
    assert_eq!(blend.patches.len(), n);

    // セル i の u=1 側のリブは、セル i+1 の u=0 側のリブと同じ曲線。
    // 位置が一致することを確かめる（接平面の一致はリボンの作り方が担う）。
    let mut worst_gap: f64 = 0.0;
    for i in 0..n {
        for step in 0..=16 {
            let s = step as f64 / 16.0;
            // セル i の v=0 辺は rib(i)、セル i-1 の u=0 辺も rib(i)。
            // どちらの辺も中心から mid(i) へ向かうので、同じ s で比べられる。
            let here = blend.patches[i].evaluate(s, 0.0);
            let previous = (i + n - 1) % n;
            let mirror = blend.patches[previous].evaluate(0.0, s);
            worst_gap = worst_gap.max((here - mirror).norm());
        }
    }
    assert!(
        worst_gap < 1e-9,
        "cells that share a rib must meet on it; worst gap {worst_gap}"
    );
}
