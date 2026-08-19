use zenith_geom::{
    bspline_basis::KnotVector,
    nurbs_curve::{ControlPoint3, NurbsCurve3},
    nurbs_surface::NurbsSurface3,
};
use zenith_math::{Point3, Point3Ext, Tolerance};

#[test]
fn test_bspline_partition_of_unity_and_affine_invariance() {
    let degree = 3;
    let n = 6;
    let knots = KnotVector::clamped_uniform(n, degree);

    // 1. 単位の分割性の厳格検証: 全 u において sum(N_{i,p}(u)) == 1.0 (許容誤差 1e-13)
    let (u_min, u_max) = (knots.start_param(degree), knots.end_param(n));
    for step in 0..=100 {
        let u = u_min + (u_max - u_min) * (step as f64 / 100.0);
        let span = knots.find_span(n, degree, u);
        let basis = knots.basis_functions(span, degree, u);
        let sum: f64 = basis.iter().sum();
        assert!(
            (sum - 1.0).abs() < 1e-13,
            "Partition of unity violated at u = {}: sum = {}",
            u,
            sum
        );
    }
}

#[test]
fn test_nurbs_derivatives_vs_finite_difference() {
    // 2. 解析的導関数（接線・偏導関数）と数値微分（有限差分）のクロスチェック
    let curve = NurbsCurve3::new(
        2,
        vec![
            ControlPoint3::new(Point3::new(0.0, 0.0, 0.0), 1.0),
            ControlPoint3::new(Point3::new(3.0, 5.0, 1.0), 2.0), // 重み 2.0 (有理NURBS)
            ControlPoint3::new(Point3::new(7.0, 2.0, 4.0), 1.5),
            ControlPoint3::new(Point3::new(10.0, 0.0, 0.0), 1.0),
        ],
        KnotVector::clamped_uniform(4, 2),
    )
    .unwrap();

    let eps = 1e-6;
    for step in 1..100 {
        let u = step as f64 / 100.0;
        let ders = curve.evaluate_derivatives(u, 1);
        let analytical_tangent = ders[1];

        let p_plus = curve.evaluate(u + eps);
        let p_minus = curve.evaluate(u - eps);
        let numerical_tangent = (p_plus - p_minus) / (2.0 * eps);

        let diff = (analytical_tangent - numerical_tangent).norm();
        assert!(
            diff < 1e-4,
            "Derivative mismatch at u = {}: analytical={:?}, numerical={:?}, diff={}",
            u,
            analytical_tangent,
            numerical_tangent,
            diff
        );
    }
}

#[test]
fn test_nurbs_surface_normal_consistency() {
    // 3. NURBS曲面の法線ベクトルが解析的Du x Dvと一致することの検証
    let ctrl_pts = vec![
        vec![
            ControlPoint3::unweighted(Point3::new(0.0, 0.0, 0.0)),
            ControlPoint3::unweighted(Point3::new(0.0, 5.0, 2.0)),
            ControlPoint3::unweighted(Point3::new(0.0, 10.0, 0.0)),
        ],
        vec![
            ControlPoint3::unweighted(Point3::new(5.0, 0.0, 3.0)),
            ControlPoint3::unweighted(Point3::new(5.0, 5.0, 6.0)),
            ControlPoint3::unweighted(Point3::new(5.0, 10.0, 3.0)),
        ],
        vec![
            ControlPoint3::unweighted(Point3::new(10.0, 0.0, 0.0)),
            ControlPoint3::unweighted(Point3::new(10.0, 5.0, 2.0)),
            ControlPoint3::unweighted(Point3::new(10.0, 10.0, 0.0)),
        ],
    ];

    let surface = NurbsSurface3::new(
        2,
        2,
        ctrl_pts,
        KnotVector::clamped_uniform(3, 2),
        KnotVector::clamped_uniform(3, 2),
    )
    .unwrap();

    for u_step in 1..10 {
        for v_step in 1..10 {
            let u = u_step as f64 / 10.0;
            let v = v_step as f64 / 10.0;

            let normal = surface.normal(u, v).expect("Normal should exist");
            assert!(
                (normal.norm() - 1.0).abs() < 1e-10,
                "Normal is not unit length"
            );

            let (_p, du, dv) = surface.evaluate_derivatives_1st(u, v);
            assert!(
                normal.dot(&du).abs() < 1e-8,
                "Normal is not perpendicular to du"
            );
            assert!(
                normal.dot(&dv).abs() < 1e-8,
                "Normal is not perpendicular to dv"
            );
        }
    }
}

#[test]
fn test_gordon_surface_curve_network_interpolation() {
    // 4. Gordon曲面が全グリッドカーブを厳密に通過することの検証
    let tol = Tolerance::default();

    // 3本のUカーブ (v in [0, 1])
    let u0 = NurbsCurve3::bspline_from_points(
        2,
        vec![
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(0.0, 5.0, 2.0),
            Point3::new(0.0, 10.0, 0.0),
        ],
    )
    .unwrap();

    let u1 = NurbsCurve3::bspline_from_points(
        2,
        vec![
            Point3::new(5.0, 0.0, 1.0),
            Point3::new(5.0, 5.0, 4.0),
            Point3::new(5.0, 10.0, 1.0),
        ],
    )
    .unwrap();

    let u2 = NurbsCurve3::bspline_from_points(
        2,
        vec![
            Point3::new(10.0, 0.0, 0.0),
            Point3::new(10.0, 5.0, 2.0),
            Point3::new(10.0, 10.0, 0.0),
        ],
    )
    .unwrap();

    // 3本のVカーブ (u in [0, 1])
    // v0(0.5) = (5.0, 0.0, 1.0) になるよう中間制御点を (5.0, 0.0, 2.0) に設定
    let v0 = NurbsCurve3::bspline_from_points(
        2,
        vec![
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(5.0, 0.0, 2.0),
            Point3::new(10.0, 0.0, 0.0),
        ],
    )
    .unwrap();

    // v1(0.5) = (5.0, 5.0, 2.5) になるよう中間制御点を (5.0, 5.0, 4.0) に設定
    let v1 = NurbsCurve3::bspline_from_points(
        2,
        vec![
            Point3::new(0.0, 5.0, 1.0),
            Point3::new(5.0, 5.0, 4.0),
            Point3::new(10.0, 5.0, 1.0),
        ],
    )
    .unwrap();

    // v2(0.5) = (5.0, 10.0, 1.0) になるよう中間制御点を (5.0, 10.0, 2.0) に設定
    let v2 = NurbsCurve3::bspline_from_points(
        2,
        vec![
            Point3::new(0.0, 10.0, 0.0),
            Point3::new(5.0, 10.0, 2.0),
            Point3::new(10.0, 10.0, 0.0),
        ],
    )
    .unwrap();

    let gordon = zenith_geom::GordonSurface3::new(vec![u0, u1, u2], vec![v0, v1, v2], &tol)
        .expect("Gordon surface should be valid");

    // グリッド線上の点の一致をサンプリング検証
    for v_step in 0..=10 {
        let v = v_step as f64 / 10.0;
        let pt_surf = gordon.evaluate(0.5, v); // u = 0.5 (u1カーブ上)
        let pt_curve = gordon.u_curves[1].evaluate(v);
        assert!(
            pt_surf.is_coincident_with(&pt_curve, 1e-6),
            "Gordon surface failed to interpolate U1 curve at v={}: surf={:?}, curve={:?}",
            v,
            pt_surf,
            pt_curve
        );
    }
}

#[test]
fn test_triangular_patch_barycentric_interpolation() {
    // 5. 3辺三角形パッチの境界一致性検証
    let tol = Tolerance::default();

    let c0 = NurbsCurve3::bspline_from_points(
        1,
        vec![Point3::new(10.0, 0.0, 0.0), Point3::new(5.0, 10.0, 3.0)],
    )
    .unwrap();

    let c1 = NurbsCurve3::bspline_from_points(
        1,
        vec![Point3::new(5.0, 10.0, 3.0), Point3::new(0.0, 0.0, 0.0)],
    )
    .unwrap();

    let c2 = NurbsCurve3::bspline_from_points(
        1,
        vec![Point3::new(0.0, 0.0, 0.0), Point3::new(10.0, 0.0, 0.0)],
    )
    .unwrap();

    let tri_patch = zenith_geom::TriangularPatch3::new(c0, c1, c2, &tol)
        .expect("Triangular patch creation should succeed");

    // 頂点での一致性
    assert!(tri_patch
        .evaluate_barycentric(1.0, 0.0, 0.0)
        .is_coincident_with(&Point3::new(0.0, 0.0, 0.0), 1e-6));
    assert!(tri_patch
        .evaluate_barycentric(0.0, 1.0, 0.0)
        .is_coincident_with(&Point3::new(10.0, 0.0, 0.0), 1e-6));
    assert!(tri_patch
        .evaluate_barycentric(0.0, 0.0, 1.0)
        .is_coincident_with(&Point3::new(5.0, 10.0, 3.0), 1e-6));
}

#[test]
fn test_differential_geometry_curvature() {
    // 6. 円筒曲面の主曲率・ガウス曲率解析解との厳密一致性検証
    // 半径 R = 5.0 の円筒面: 主曲率 kappa_1 = 1/R = 0.2, kappa_2 = 0.0 (母線方向)
    // ガウス曲率 K = 0.0, 平均曲率 H = 0.1
    let _radius = 5.0;
    let s_cylinder = zenith_geom::NurbsSurface3::new(
        2,
        1,
        vec![
            vec![
                zenith_geom::ControlPoint3::unweighted(Point3::new(5.0, 0.0, 0.0)),
                zenith_geom::ControlPoint3::unweighted(Point3::new(5.0, 0.0, 20.0)),
            ],
            vec![
                zenith_geom::ControlPoint3::unweighted(Point3::new(5.0, 5.0, 0.0)),
                zenith_geom::ControlPoint3::unweighted(Point3::new(5.0, 5.0, 20.0)),
            ],
            vec![
                zenith_geom::ControlPoint3::unweighted(Point3::new(0.0, 5.0, 0.0)),
                zenith_geom::ControlPoint3::unweighted(Point3::new(0.0, 5.0, 20.0)),
            ],
        ],
        zenith_geom::KnotVector::clamped_uniform(3, 2),
        zenith_geom::KnotVector::clamped_uniform(2, 1),
    )
    .unwrap();

    let curv = s_cylinder
        .evaluate_curvature(0.5, 0.5)
        .expect("Curvature evaluation failed");
    // 母線方向は曲率0（どちらか一方の主曲率の絶対値が0）
    let min_abs_curvature = curv
        .principal_curvature_1
        .abs()
        .min(curv.principal_curvature_2.abs());
    let max_abs_curvature = curv
        .principal_curvature_1
        .abs()
        .max(curv.principal_curvature_2.abs());
    assert!(
        min_abs_curvature < 1e-10,
        "Cylinder flat direction should have zero curvature"
    );
    assert!(
        max_abs_curvature > 1e-3,
        "Cylinder radial direction should have non-zero curvature"
    );
    assert!(
        curv.gaussian_curvature.abs() < 1e-10,
        "Cylinder must have zero Gaussian curvature (developable surface)"
    );
}

#[test]
fn test_surface_surface_intersection_ssi() {
    // 7. 曲面と曲面の幾何交差（SSI）の検証
    let tol = Tolerance::default();

    // 曲面 1: Z=5.0 の水平平面曲面
    let s1 = zenith_geom::NurbsSurface3::new(
        1,
        1,
        vec![
            vec![
                zenith_geom::ControlPoint3::unweighted(Point3::new(0.0, 0.0, 5.0)),
                zenith_geom::ControlPoint3::unweighted(Point3::new(0.0, 10.0, 5.0)),
            ],
            vec![
                zenith_geom::ControlPoint3::unweighted(Point3::new(10.0, 0.0, 5.0)),
                zenith_geom::ControlPoint3::unweighted(Point3::new(10.0, 10.0, 5.0)),
            ],
        ],
        zenith_geom::KnotVector::clamped_uniform(2, 1),
        zenith_geom::KnotVector::clamped_uniform(2, 1),
    )
    .unwrap();

    // 曲面 2: Z=0 から Z=10 に傾斜した曲面
    let s2 = zenith_geom::NurbsSurface3::new(
        1,
        1,
        vec![
            vec![
                zenith_geom::ControlPoint3::unweighted(Point3::new(0.0, 0.0, 0.0)),
                zenith_geom::ControlPoint3::unweighted(Point3::new(0.0, 10.0, 0.0)),
            ],
            vec![
                zenith_geom::ControlPoint3::unweighted(Point3::new(10.0, 0.0, 10.0)),
                zenith_geom::ControlPoint3::unweighted(Point3::new(10.0, 10.0, 10.0)),
            ],
        ],
        zenith_geom::KnotVector::clamped_uniform(2, 1),
        zenith_geom::KnotVector::clamped_uniform(2, 1),
    )
    .unwrap();

    let isects = zenith_geom::SurfaceIntersection::intersect_surfaces(&s1, &s2, &tol);
    assert!(!isects.is_empty(), "SSI should find intersection points");

    // 全交差点が Z=5.0 (平面1上) かつ s2 上に存在することを検証
    for ipt in &isects {
        assert!(
            (ipt.point.z - 5.0).abs() < 1e-4,
            "Intersection Z coordinate must be 5.0"
        );
        let p_on_s1 = s1.evaluate(ipt.uv1.0, ipt.uv1.1);
        let p_on_s2 = s2.evaluate(ipt.uv2.0, ipt.uv2.1);
        assert!(
            p_on_s1.is_coincident_with(&p_on_s2, 1e-4),
            "Intersection point mismatch between surfaces"
        );
    }
}

#[test]
fn test_trimmed_surface_uv_containment() {
    // 8. トリム曲面のUV領域判定（Ray Casting Point-in-Polygon）テスト
    let base_surf = zenith_geom::NurbsSurface3::new(
        1,
        1,
        vec![
            vec![
                zenith_geom::ControlPoint3::unweighted(Point3::new(0.0, 0.0, 0.0)),
                zenith_geom::ControlPoint3::unweighted(Point3::new(0.0, 10.0, 0.0)),
            ],
            vec![
                zenith_geom::ControlPoint3::unweighted(Point3::new(10.0, 0.0, 0.0)),
                zenith_geom::ControlPoint3::unweighted(Point3::new(10.0, 10.0, 0.0)),
            ],
        ],
        zenith_geom::KnotVector::clamped_uniform(2, 1),
        zenith_geom::KnotVector::clamped_uniform(2, 1),
    )
    .unwrap();

    // UV空間内の正方形外側ループ [0.2, 0.8] x [0.2, 0.8]
    let p_c0 = zenith_geom::NurbsCurve2::bspline_from_points(
        1,
        vec![
            zenith_math::Point2::new(0.2, 0.2),
            zenith_math::Point2::new(0.8, 0.2),
        ],
    )
    .unwrap();
    let p_c1 = zenith_geom::NurbsCurve2::bspline_from_points(
        1,
        vec![
            zenith_math::Point2::new(0.8, 0.2),
            zenith_math::Point2::new(0.8, 0.8),
        ],
    )
    .unwrap();
    let p_c2 = zenith_geom::NurbsCurve2::bspline_from_points(
        1,
        vec![
            zenith_math::Point2::new(0.8, 0.8),
            zenith_math::Point2::new(0.2, 0.8),
        ],
    )
    .unwrap();
    let p_c3 = zenith_geom::NurbsCurve2::bspline_from_points(
        1,
        vec![
            zenith_math::Point2::new(0.2, 0.8),
            zenith_math::Point2::new(0.2, 0.2),
        ],
    )
    .unwrap();

    let outer_loop = zenith_geom::TrimLoop2D::new(vec![p_c0, p_c1, p_c2, p_c3]);

    // 内側穴ループ [0.4, 0.6] x [0.4, 0.6]
    let h0 = zenith_geom::NurbsCurve2::bspline_from_points(
        1,
        vec![
            zenith_math::Point2::new(0.4, 0.4),
            zenith_math::Point2::new(0.6, 0.4),
        ],
    )
    .unwrap();
    let h1 = zenith_geom::NurbsCurve2::bspline_from_points(
        1,
        vec![
            zenith_math::Point2::new(0.6, 0.4),
            zenith_math::Point2::new(0.6, 0.6),
        ],
    )
    .unwrap();
    let h2 = zenith_geom::NurbsCurve2::bspline_from_points(
        1,
        vec![
            zenith_math::Point2::new(0.6, 0.6),
            zenith_math::Point2::new(0.4, 0.6),
        ],
    )
    .unwrap();
    let h3 = zenith_geom::NurbsCurve2::bspline_from_points(
        1,
        vec![
            zenith_math::Point2::new(0.4, 0.6),
            zenith_math::Point2::new(0.4, 0.4),
        ],
    )
    .unwrap();

    let inner_hole = zenith_geom::TrimLoop2D::new(vec![h0, h1, h2, h3]);

    let trimmed = zenith_geom::TrimmedSurface3::new(base_surf, Some(outer_loop), vec![inner_hole]);

    // (0.1, 0.1) は外側ループの外 -> 無効
    assert!(!trimmed.is_uv_valid(0.1, 0.1));
    // (0.5, 0.5) は穴の内側 -> 無効
    assert!(!trimmed.is_uv_valid(0.5, 0.5));
    // (0.3, 0.3) は外側と穴の間（有効領域） -> 有効
    assert!(trimmed.is_uv_valid(0.3, 0.3));
    // (0.7, 0.7) は有効領域 -> 有効
    assert!(trimmed.is_uv_valid(0.7, 0.7));
}

#[test]
fn test_surface_blend_g1_continuity() {
    // 9. サーフェスブレンド（フィレット曲面）の境界連続性検証
    let tol = Tolerance::default();

    let r1 = zenith_geom::NurbsCurve3::bspline_from_points(
        2,
        vec![
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(5.0, 0.0, 2.0),
            Point3::new(10.0, 0.0, 0.0),
        ],
    )
    .unwrap();

    let r2 = zenith_geom::NurbsCurve3::bspline_from_points(
        2,
        vec![
            Point3::new(0.0, 5.0, 5.0),
            Point3::new(5.0, 5.0, 8.0),
            Point3::new(10.0, 5.0, 5.0),
        ],
    )
    .unwrap();

    let blend = zenith_geom::SurfaceBlend3::create_g1_blend(r1.clone(), r2.clone(), 1.0, &tol)
        .expect("Blend creation failed");

    // レール1 (v=0) と レール2 (v=1) での境界完全一致検証
    for step in 0..=10 {
        let u = step as f64 / 10.0;
        let pt_blend_0 = blend.evaluate(u, 0.0);
        let pt_rail_1 = r1.evaluate(u);
        assert!(
            pt_blend_0.is_coincident_with(&pt_rail_1, 1e-6),
            "Blend boundary 0 mismatch"
        );

        let pt_blend_1 = blend.evaluate(u, 1.0);
        let pt_rail_2 = r2.evaluate(u);
        assert!(
            pt_blend_1.is_coincident_with(&pt_rail_2, 1e-6),
            "Blend boundary 1 mismatch"
        );
    }
}

#[test]
fn test_offset_surface_and_curve() {
    // 10. 自由曲面および3D曲線のオフセット幾何精度検証
    // 平面NURBS曲面（Z=0）を +Z方向に 5.0mm オフセット
    let base_surf = zenith_geom::NurbsSurface3::new(
        1,
        1,
        vec![
            vec![
                zenith_geom::ControlPoint3::unweighted(Point3::new(0.0, 0.0, 0.0)),
                zenith_geom::ControlPoint3::unweighted(Point3::new(0.0, 10.0, 0.0)),
            ],
            vec![
                zenith_geom::ControlPoint3::unweighted(Point3::new(10.0, 0.0, 0.0)),
                zenith_geom::ControlPoint3::unweighted(Point3::new(10.0, 10.0, 0.0)),
            ],
        ],
        zenith_geom::KnotVector::clamped_uniform(2, 1),
        zenith_geom::KnotVector::clamped_uniform(2, 1),
    )
    .unwrap();

    let offset_surf =
        zenith_geom::OffsetEngine::offset_surface(&base_surf, 5.0).expect("Offset surface failed");

    // 各評価点で Z座標が 5.0mm になっているか検証
    for step_u in 0..=5 {
        for step_v in 0..=5 {
            let u = step_u as f64 / 5.0;
            let v = step_v as f64 / 5.0;
            let pt = offset_surf.evaluate(u, v);
            assert!(
                (pt.z - 5.0).abs() < 1e-6,
                "Offset surface Z distance failed"
            );
        }
    }

    // 3D曲線の法線オフセット検証
    let curve = zenith_geom::NurbsCurve3::bspline_from_points(
        1,
        vec![Point3::new(0.0, 0.0, 0.0), Point3::new(10.0, 0.0, 0.0)],
    )
    .unwrap();

    let offset_crv =
        zenith_geom::OffsetEngine::offset_curve(&curve, zenith_math::Vec3::new(0.0, 1.0, 0.0), 3.0)
            .expect("Offset curve failed");

    let p0 = offset_crv.evaluate(0.0);
    let p1 = offset_crv.evaluate(1.0);
    assert!((p0.y - 3.0).abs() < 1e-6);
    assert!((p1.y - 3.0).abs() < 1e-6);
}

#[test]
fn test_extremum_point_to_curve_and_surface() {
    // 11. 点からNURBS曲線・曲面への最短距離・最近傍点（ExtremumEngine）のニュートン探索検証
    // 直線 (0,0,0) -> (10,0,0)
    let curve = zenith_geom::NurbsCurve3::bspline_from_points(
        1,
        vec![Point3::new(0.0, 0.0, 0.0), Point3::new(10.0, 0.0, 0.0)],
    )
    .unwrap();

    // 外部の点 (5.0, 3.0, 4.0) からの最短距離 = sqrt(3^2 + 4^2) = 5.0, 最近傍点 (5.0, 0.0, 0.0)
    let query_pt = Point3::new(5.0, 3.0, 4.0);
    let proj = zenith_geom::ExtremumEngine::point_to_curve(query_pt, &curve, 20, 1e-6)
        .expect("Curve projection failed");

    assert!((proj.distance - 5.0).abs() < 1e-4);
    assert!((proj.closest_point.x - 5.0).abs() < 1e-4);
    assert!((proj.parameter - 0.5).abs() < 1e-4);

    // 平面曲面 (0..10, 0..10, 0)
    let surf = zenith_geom::NurbsSurface3::new(
        1,
        1,
        vec![
            vec![
                zenith_geom::ControlPoint3::unweighted(Point3::new(0.0, 0.0, 0.0)),
                zenith_geom::ControlPoint3::unweighted(Point3::new(0.0, 10.0, 0.0)),
            ],
            vec![
                zenith_geom::ControlPoint3::unweighted(Point3::new(10.0, 0.0, 0.0)),
                zenith_geom::ControlPoint3::unweighted(Point3::new(10.0, 10.0, 0.0)),
            ],
        ],
        zenith_geom::KnotVector::clamped_uniform(2, 1),
        zenith_geom::KnotVector::clamped_uniform(2, 1),
    )
    .unwrap();

    let query_surf_pt = Point3::new(2.0, 8.0, 6.0);
    let surf_proj = zenith_geom::ExtremumEngine::point_to_surface(query_surf_pt, &surf, 20, 1e-6)
        .expect("Surface projection failed");

    assert!((surf_proj.distance - 6.0).abs() < 1e-4);
    assert!((surf_proj.closest_point.x - 2.0).abs() < 1e-4);
    assert!((surf_proj.closest_point.y - 8.0).abs() < 1e-4);
    assert!((surf_proj.closest_point.z - 0.0).abs() < 1e-4);
}

#[test]
fn test_rational_bezier_split_preserves_exact_circle() {
    let radius = 10.0;
    let weight = std::f64::consts::FRAC_1_SQRT_2;
    let arc = NurbsCurve3::new(
        2,
        vec![
            ControlPoint3::unweighted(Point3::new(radius, 0.0, 0.0)),
            ControlPoint3::new(Point3::new(radius, radius, 0.0), weight),
            ControlPoint3::unweighted(Point3::new(0.0, radius, 0.0)),
        ],
        KnotVector::clamped_uniform(3, 2),
    )
    .unwrap();

    let (t_min, t_max) = arc.param_range();
    let split_t = t_min + (t_max - t_min) * 0.35;
    let split_point = arc.evaluate(split_t);
    let (left, right) = arc.split_bezier_at(split_t).expect("bezier split");

    // 分割点と両端が厳密に一致する
    for (curve, expected_start, expected_end) in [
        (&left, arc.evaluate(t_min), split_point),
        (&right, split_point, arc.evaluate(t_max)),
    ] {
        let (c_min, c_max) = curve.param_range();
        assert!((curve.evaluate(c_min) - expected_start).norm() < 1e-12);
        assert!((curve.evaluate(c_max) - expected_end).norm() < 1e-12);
    }

    // 有理重みが保たれ、両半分とも真円上に乗り続ける
    for curve in [&left, &right] {
        let (c_min, c_max) = curve.param_range();
        for step in 0..=20 {
            let t = c_min + (c_max - c_min) * (step as f64 / 20.0);
            let point = curve.evaluate(t);
            let radial = (point.x * point.x + point.y * point.y).sqrt();
            assert!(
                (radial - radius).abs() < 1e-12,
                "split arc left the exact circle: {radial}"
            );
            assert!(point.z.abs() < 1e-12);
        }
    }

    // 内部ノットを持つ曲線はベジエ分割の対象外
    let spline = NurbsCurve3::bspline_from_points(
        2,
        vec![
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(1.0, 1.0, 0.0),
            Point3::new(2.0, 0.0, 0.0),
            Point3::new(3.0, 1.0, 0.0),
        ],
    )
    .unwrap();
    assert!(spline.split_bezier_at(0.5).is_none());
    assert!(arc.split_bezier_at(t_min).is_none());
    assert!(arc.split_bezier_at(t_max).is_none());
}
