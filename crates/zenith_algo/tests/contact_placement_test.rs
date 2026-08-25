//! 接触している配置。**規約は「接触は、それ自体では位相を作らない」**
//! （HANDOVER 3-1）。
//!
//! ここで押さえるのは2つです。
//!
//! - 答えが多様体になる接触は、**返す**
//! - 答えが本当に非多様体になる接触は、**場所を名指しして断る**
//!
//! どちらも、この文書が長らく「すべて接線配置」と書いていたものの中身です。
//! 実測したら機構が2つあり、内訳も違いました（HANDOVER 3-1、3-N-1）。

use zenith_algo::{BooleanEngine, BooleanOpType, BrepTransform, MassCalculator, PrimitiveBuilder};
use zenith_math::{Tolerance, Transform3, Vec3};
use zenith_tess::TessellationParams;
use zenith_topo::Solid;

fn volume(solids: &[Solid]) -> f64 {
    solids
        .iter()
        .map(|solid| {
            MassCalculator::compute_from_brep(
                solid,
                &TessellationParams {
                    u_divisions: 48,
                    v_divisions: 48,
                },
            )
            .volume
        })
        .sum()
}

/// メッシュの稜のうち、ちょうど2枚の三角形に共有されていない本数。
fn non_manifold_edges(solid: &Solid) -> usize {
    let mesh = zenith_tess::tessellate_solid(
        solid,
        &TessellationParams {
            u_divisions: 24,
            v_divisions: 24,
        },
    );
    let mut uses: std::collections::HashMap<(u32, u32), usize> = std::collections::HashMap::new();
    for triangle in &mesh.indices {
        for step in 0..3 {
            let (a, b) = (triangle[step], triangle[(step + 1) % 3]);
            if a == b {
                continue;
            }
            let key = if a < b { (a, b) } else { (b, a) };
            *uses.entry(key).or_insert(0) += 1;
        }
    }
    uses.values().filter(|count| **count != 2).count()
}

/// 20立方の箱を、(10,10,0) ずらしてから**世界原点まわりに** 45 度回す。
///
/// 回した箱の角 (10,10) が (0, 10√2) に来て、**縦の稜がまるごと A の面
/// x = 0 の中に乗ります**。`boolean_envelope` と同じ置き方です（箱の中心
/// まわりに回すと接触しなくなり、別の配置を測ったことになります）。
fn rotated_pair() -> (Solid, Solid) {
    let boxa = PrimitiveBuilder::make_box(20.0, 20.0, 20.0).expect("box");
    let turn = Transform3::from_axis_angle(&Vec3::new(0.0, 0.0, 1.0), std::f64::consts::FRAC_PI_4);
    let shifted = BrepTransform::translate_solid(&boxa, Vec3::new(10.0, 10.0, 0.0));
    let turned = BrepTransform::transform_solid(&shifted, &turn).expect("turn");
    (boxa, turned)
}

/// 重なりは、頂角 45 度の楔を y = 20 で切ったもの。閉じた式で出ます。
///
/// ```text
/// 面積 = (20 - 10√2)^2 / 2,  体積 = 面積 * 20
/// ```
fn rotated_overlap_volume() -> f64 {
    let reach = 20.0 - 10.0 * 2.0_f64.sqrt();
    reach * reach / 2.0 * 20.0
}

/// 球の継ぎ目を回しても、極を通る切断面は半球を作る。
///
/// この球面片の境界は、同じ大円上に複数のトポロジー稜を持つ。earcut が
/// 同じ稜の点だけからなるearを連続して作ったとき、1枚ずつのedge flipでは
/// 塊の入口で止まり、平面capと重なるメッシュ稜が6本残っていた（4-87）。
#[test]
fn a_spun_sphere_cut_through_its_pole_has_a_manifold_mesh() {
    let tol = Tolerance::default();
    let block = PrimitiveBuilder::make_box(20.0, 20.0, 20.0).expect("box");
    let sphere = PrimitiveBuilder::make_sphere(10.0).expect("sphere");
    let spin = Transform3::from_axis_angle(&Vec3::new(0.0, 0.0, 1.0), 45f64.to_radians());
    let spun = BrepTransform::transform_solid(&sphere, &spin).expect("spin");
    let placed = BrepTransform::translate_solid(&spun, Vec3::new(20.0, 10.0, 10.0));

    let result = BooleanEngine::boolean_solids_exact_result(
        &block,
        &placed,
        BooleanOpType::Intersection,
        &tol,
    )
    .expect("the half sphere should be representable");

    assert_eq!(result.solids.len(), 1);
    assert_eq!(non_manifold_edges(&result.solids[0]), 0);
    let expected = 2.0 * std::f64::consts::PI * 10.0_f64.powi(3) / 3.0;
    let got = volume(&result.solids);
    assert!((got - expected).abs() <= expected * 1e-8, "volume {got}");
}

/// 外周と穴が一点で接する平面capには、重なったflapを残さない。
///
/// 円柱の継ぎ目を切断平面から外した配置では、接点の三角形1枚が辺使用数
/// `[3, 3, 1]` を作り、meshに非多様体辺が3本残っていた（4-88）。
#[test]
fn a_turned_tangent_cylinder_union_has_a_manifold_mesh() {
    let tol = Tolerance::default();
    let block = PrimitiveBuilder::make_box(20.0, 20.0, 20.0).expect("box");
    let cylinder = PrimitiveBuilder::make_cylinder(6.0, 40.0).expect("cylinder");
    let spin = Transform3::from_axis_angle(&Vec3::new(0.0, 0.0, 1.0), 33f64.to_radians());
    let turned = BrepTransform::transform_solid(&cylinder, &spin).expect("turn");
    let placed = BrepTransform::translate_solid(&turned, Vec3::new(6.0, 10.0, -10.0));

    let result = BooleanEngine::boolean_solids_exact_result(
        &block,
        &placed,
        BooleanOpType::Union,
        &tol,
    )
    .expect("the tangent union should be representable");

    assert_eq!(result.solids.len(), 1);
    assert_eq!(non_manifold_edges(&result.solids[0]), 0);
}

/// 稜が相手の面の中に乗っている配置は、**3演算とも多様体の立体になります**。
///
/// ここが通らなかった原因は接触の交線ではなく、**面積を囲まない面片**でした。
/// 面をその稜で割ると「面そのもの」と「行って戻るだけの切れ目」が出て、
/// 切れ目のほうが同じ稜をもう2回使い、縫合が非多様体になっていました
/// （実測: 差で 12、積で 22）。
#[test]
fn an_edge_lying_in_a_face_still_gives_a_manifold_answer() {
    let tol = Tolerance::default();
    let (a, b) = rotated_pair();
    let overlap = rotated_overlap_volume();

    let expected = [
        (BooleanOpType::Union, 8000.0 + 8000.0 - overlap),
        (BooleanOpType::Difference, 8000.0 - overlap),
        (BooleanOpType::Intersection, overlap),
    ];

    for (op, want) in expected {
        let result = BooleanEngine::boolean_solids_exact_result(&a, &b, op, &tol)
            .unwrap_or_else(|err| panic!("{op:?} was refused: {err}"));
        assert_eq!(result.solids.len(), 1, "{op:?} should give one solid");

        let got = volume(&result.solids);
        assert!(
            (got - want).abs() <= want.abs() * 1e-9,
            "{op:?} volume {got} does not match the closed form {want}"
        );

        for solid in &result.solids {
            assert_eq!(
                non_manifold_edges(solid),
                0,
                "{op:?} returned a solid that is not manifold"
            );
        }
    }
}

/// 円柱が箱の壁に**内側から**接している配置。差だけが非多様体になります。
///
/// 壁のところで材料の厚みが 0 になり、線でしか繋がらない2つの塊に割れます。
/// `Solid` は多様体 B-Rep なので持てません。**断るのが正しい答え**で、
/// 「未実装」ではありません。
#[test]
fn a_tangent_contact_that_pinches_the_material_is_refused_by_name() {
    let tol = Tolerance::default();
    let boxa = PrimitiveBuilder::make_box(20.0, 20.0, 20.0).expect("box");
    // 半径6を中心 (6, 10) に置くと、側面 x = 0 にちょうど接します。
    let cylinder = BrepTransform::translate_solid(
        &PrimitiveBuilder::make_cylinder(6.0, 40.0).expect("cylinder"),
        Vec3::new(6.0, 10.0, -10.0),
    );

    let refused =
        BooleanEngine::boolean_solids_exact_result(&boxa, &cylinder, BooleanOpType::Difference, &tol)
            .expect_err("the difference pinches the wall and cannot be a manifold solid");

    assert!(
        refused.contains("non-manifold"),
        "the refusal should say why: {refused}"
    );
    // **場所を名指しします。** 接触線は x = 0, y = 10 の上にあります。
    assert!(
        refused.contains("0.000000 10.000000"),
        "the refusal should name where: {refused}"
    );

    // 同じ配置でも、和と積は繋がったままなので返ります。
    for op in [BooleanOpType::Union, BooleanOpType::Intersection] {
        let result = BooleanEngine::boolean_solids_exact_result(&boxa, &cylinder, op, &tol)
            .unwrap_or_else(|err| panic!("{op:?} on the same placement was refused: {err}"));
        for solid in &result.solids {
            assert_eq!(
                non_manifold_edges(solid),
                0,
                "{op:?} returned a solid that is not manifold"
            );
        }
    }
}
