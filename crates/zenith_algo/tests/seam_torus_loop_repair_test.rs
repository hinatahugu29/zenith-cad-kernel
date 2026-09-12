//! **継ぎ目どうしのトーラスが、3 演算とも正しく返ること**（4-441、4-442）。
//!
//! # なぜ要るのか
//!
//! **2026/09/09 から 09/12 まで、この検体は 3 演算とも断られて**いました
//! ——**外部ファイルを 1 つも使わずに H8 の壁が出る検体**（4-412）。
//!
//! **4-441 で抜けました**——**閉じた輪という不変量**（どの端点も
//! ちょうど 2 回）が破れている所から、**足りない交線を拾い直す**
//! ようにしたためです。
//!
//! **門（`contact_placement_probe`）にもありますが、あれは 900 秒**
//! かかります。**通しテストは、もっと頻繁に回ります。**
//!
//! # 何を見るか
//!
//! **返ることだけでは足りません。** **恒等式**で確かめます——
//!
//! ```text
//! V(A ∪ B) + V(A ∩ B) = V(A) + V(B)
//! V(A − B) = V(A) − V(A ∩ B)
//! ```
//!
//! **もっともらしい立体を返すのが、このリポジトリがいちばん避けてきた
//! 失敗**です（4-136、4-138）。
//!
//! **多様体かどうかも見ます**——**カーネル自身の検査**で。
//! **「非多様体 0」は数え方によります**（4-442 で 1 度つまずきました）。

use zenith_algo::{BooleanEngine, BooleanOpType, BrepTransform, MassCalculator, PrimitiveBuilder};
use zenith_math::{Tolerance, Transform3, Vec3};
use zenith_tess::TessellationParams;

/// **`seam_torus_wall_probe` と、同じ 2 つ**です。
/// **値を変えたら、あちらも変えてください**——**別の形を同じ名前で
/// 語ると、次の人が突き合わせられません。**
fn turned_tori() -> (zenith_topo::Solid, zenith_topo::Solid) {
    let torus = PrimitiveBuilder::make_torus(12.0, 4.0).expect("torus");
    let spin = Transform3::from_axis_angle(&Vec3::new(0.0, 0.0, 1.0), 29f64.to_radians());
    let tip = Transform3::from_axis_angle(&Vec3::new(1.0, 0.0, 0.0), 61f64.to_radians());
    let a = BrepTransform::transform_solid(&torus, &spin).expect("turn");
    let b = BrepTransform::translate_solid(
        &BrepTransform::transform_solid(&torus, &tip).expect("tip"),
        Vec3::new(18.0, 0.0, 0.0),
    );
    (a, b)
}

fn volume_of(solids: &[zenith_topo::Solid]) -> f64 {
    let params = TessellationParams {
        u_divisions: 64,
        v_divisions: 64,
    };
    solids
        .iter()
        .map(|solid| MassCalculator::compute_from_brep(solid, &params).volume)
        .sum()
}

#[test]
fn the_turned_tori_return_a_solid_for_all_three_operations() {
    let tol = Tolerance::default();
    let (a, b) = turned_tori();

    for (label, op) in [
        ("union", BooleanOpType::Union),
        ("difference", BooleanOpType::Difference),
        ("intersection", BooleanOpType::Intersection),
    ] {
        let result = BooleanEngine::boolean_solids_exact_result(&a, &b, op, &tol)
            .unwrap_or_else(|reason| panic!("{label} should return a solid, got {reason}"));
        assert_eq!(
            result.solids.len(),
            1,
            "{label} should return exactly one solid"
        );
        // **カーネル自身の検査**で見ます。**自前の数え方は、数え方に
        // よって答えが変わります**（4-442）。
        assert!(
            result.solids[0].is_topologically_valid(&tol),
            "{label} returned a solid that the kernel's own validator rejects"
        );
    }
}

#[test]
fn the_turned_tori_close_the_inclusion_exclusion_identity() {
    let tol = Tolerance::default();
    let (a, b) = turned_tori();

    let union = BooleanEngine::boolean_solids_exact_result(&a, &b, BooleanOpType::Union, &tol)
        .expect("union");
    let difference =
        BooleanEngine::boolean_solids_exact_result(&a, &b, BooleanOpType::Difference, &tol)
            .expect("difference");
    let intersection =
        BooleanEngine::boolean_solids_exact_result(&a, &b, BooleanOpType::Intersection, &tol)
            .expect("intersection");

    let union_volume = volume_of(&union.solids);
    let difference_volume = volume_of(&difference.solids);
    let intersection_volume = volume_of(&intersection.solids);

    // **トーラス R=12 r=4 は 2π²Rr²。** **回しても動きません。**
    let one = 2.0 * std::f64::consts::PI * std::f64::consts::PI * 12.0 * 16.0;

    let sum = union_volume + intersection_volume;
    let residual = (sum - 2.0 * one).abs() / (2.0 * one);
    assert!(
        residual <= 1e-6,
        "V(union) + V(intersection) should be V(A) + V(B) = {}, got {} (relative {residual:.3e})",
        2.0 * one,
        sum
    );

    let expected_difference = one - intersection_volume;
    let difference_residual =
        (difference_volume - expected_difference).abs() / expected_difference.abs().max(1.0);
    assert!(
        difference_residual <= 1e-6,
        "V(difference) should be V(A) - V(intersection) = {expected_difference}, got {difference_volume} (relative {difference_residual:.3e})"
    );
}

/// **修理が無ければ閉じない**ことを、環境変数を触らずに示す。
///
/// **`collect_intersection_edge_candidates` は、修理を通りません**
/// ——**ブーリアンの中でだけ修理が走る**ので、この口から見ると
/// **修理前の姿**が見えます。
///
/// **そこに浮いた端が 4 つある**のに、**ブーリアンは通る**——
/// **間を埋めているのが修理**です。
///
/// > **⚠ 環境変数で切り替える試験は書けません**（4-443）。
/// > **`std::env::set_var` はプロセス全体に効き**、**試験は同じ
/// > プロセスの別スレッドで並行に走ります**。**実際に、他の 2 本へ
/// > 漏れて落としました。** **`ZENITH_NO_LOOP_REPAIR` を確かめるのは
/// > 門の側**（別のプロセス）です。
#[test]
fn without_the_repair_the_intersection_curves_do_not_close() {
    use std::collections::BTreeMap;
    use zenith_algo::BrepIntersectionBuilder;
    use zenith_math::Point3;

    let tol = Tolerance::default();
    let (a, b) = turned_tori();

    let candidates = BrepIntersectionBuilder::collect_intersection_edge_candidates(
        &a.outer_shell.faces,
        &b.outer_shell.faces,
        &tol,
    );

    // **端点は丸めて数えます**——**同じ点が別の曲線から来ると、
    // 最後の桁が違います**（4-412）。
    let key = |point: Point3| {
        (
            (point.x * 1e7).round() as i64,
            (point.y * 1e7).round() as i64,
            (point.z * 1e7).round() as i64,
        )
    };
    let mut uses: BTreeMap<(i64, i64, i64), usize> = BTreeMap::new();
    for candidate in &candidates {
        for point in [
            candidate.edge.start_vertex.point,
            candidate.edge.end_vertex.point,
        ] {
            *uses.entry(key(point)).or_insert(0) += 1;
        }
    }
    let loose = uses.values().filter(|count| **count != 2).count();

    assert_eq!(
        loose, 4,
        "the un-repaired intersection curves should still leave 4 loose ends;          if this changed, the repair moved somewhere else and this test is stale"
    );

    // **それでもブーリアンは通ります**——**間を埋めているのが修理**です。
    BooleanEngine::boolean_solids_exact_result(&a, &b, BooleanOpType::Union, &tol)
        .expect("the boolean should still succeed, because the repair runs inside it");
}
