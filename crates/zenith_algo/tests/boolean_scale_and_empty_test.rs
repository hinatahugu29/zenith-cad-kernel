//! 桁の離れた立体と、答えが空になる演算。
//!
//! どちらも `boolean_envelope` の45ケースには入っていません。あの表は半径も
//! 距離も桁が揃った配置ばかりで、実務のデータが持ち込む性質——筐体とネジの
//! ような桁差、同じ形どうしの演算——が無いからです。`robustness_probe` が
//! 両方を見つけました。ここはその再発を見張ります。

use zenith_algo::{
    BooleanEngine, BooleanOpType, BooleanResultVerifier, BrepTransform, HoleBuilder,
    MassCalculator, PrimitiveBuilder,
};
use zenith_math::{Tolerance, Vec3};
use zenith_tess::TessellationParams;

fn params() -> TessellationParams {
    TessellationParams {
        u_divisions: 24,
        v_divisions: 24,
    }
}

#[test]
fn an_intersection_much_smaller_than_its_operands_is_not_read_as_zero() {
    // 検証ゲートは「体積の境界比較」と「体積が正か」の両方に同じ緩衝を使い、
    // どちらも**大きいほうの立体**で正規化していました。境界比較はそれで
    // 良いのですが、ゼロ判定は違います。積は定義上、小さいほうの立体を
    // 超えられないからです。
    //
    // 一辺 1e6 の箱と一辺 1 の箱の積は単位立方体で、正解は 1.0 です。緩衝が
    // 1e-6 x 1e18 = 1e12 になっていたため、正解が「正でない」と報告されて
    // いました。**筐体と小部品の積が必ず失敗する**、という形で実務に出ます。
    let tol = Tolerance::default();
    let big = PrimitiveBuilder::make_box(1.0e6, 1.0e6, 1.0e6).expect("big box");
    let small = PrimitiveBuilder::make_box(1.0, 1.0, 1.0).expect("small box");

    let result =
        BooleanEngine::boolean_solids_exact_result(&big, &small, BooleanOpType::Intersection, &tol)
            .expect("an intersection six orders smaller than one operand should still be an answer");

    let volume: f64 = result
        .solids
        .iter()
        .map(|solid| MassCalculator::compute_from_brep(solid, &params()).volume)
        .sum();
    let error = (volume - 1.0).abs();
    assert!(
        error < 1e-6,
        "the intersection should be the unit cube, measured {volume}"
    );

    let report = BooleanResultVerifier::verify(
        &big,
        &small,
        &result.solids,
        BooleanOpType::Intersection,
        &tol,
    );
    assert!(
        report.is_valid(),
        "the gate should accept a correct answer that is small next to its operands: {:?}",
        report.errors.first()
    );
}

#[test]
fn a_small_but_correct_intersection_is_not_read_as_zero() {
    // 上の検査を通した最初の修正には、`.max(1.0)` という**絶対値の床**が
    // 残っていました。桁差だけを見て、絶対的な小ささを見ていなかったのです。
    // 1e6 対 1 は直りましたが、80000 対 0.008 は直っていませんでした。
    //
    // 200x200x2 の板を 0.02x0.02x20 の針が貫くと、積は 0.0008 です。閾値が
    // 1e-3 x 1.0 = 0.001 になっていたので、正解がその下に来ていました。
    // 床は寸法から引いた `tol.linear^3` にしてあります。
    let tol = Tolerance::default();
    let plate = PrimitiveBuilder::make_box(200.0, 200.0, 2.0).expect("plate");
    let needle = BrepTransform::translate_solid(
        &PrimitiveBuilder::make_box(0.02, 0.02, 20.0).expect("needle"),
        Vec3::new(100.0, 100.0, -9.0),
    );

    let result =
        BooleanEngine::boolean_solids_exact_result(&plate, &needle, BooleanOpType::Intersection, &tol)
            .expect("a needle through a plate has a small but real intersection");

    let volume: f64 = result
        .solids
        .iter()
        .map(|solid| MassCalculator::compute_from_brep(solid, &params()).volume)
        .sum();
    let expected = 0.02 * 0.02 * 2.0;
    assert!(
        (volume - expected).abs() / expected < 1e-9,
        "the intersection should be {expected}, measured {volume}"
    );
}

#[test]
fn the_volume_bounds_still_bite_on_a_small_model() {
    // 同じ床は逆方向にも効いていました。体積が 1 を下回るモデルでは
    // `eps` が体積そのものより大きくなり、`vr < max(va, vb) - eps` のような
    // 境界チェックが**恒真になって何も見なくなります**。大きい側では正解を
    // 弾き、小さい側では検査が消える——絶対値の床は、スケールの両端で
    // 別々に壊れます。
    //
    // 一辺 0.2 の箱2つ（体積 0.008）で、和として片方だけを差し出します。
    // 境界チェックが生きていれば「大きいほうの立体より小さい」で弾かれます。
    let tol = Tolerance::default();
    let a = PrimitiveBuilder::make_box(0.2, 0.2, 0.2).expect("small box a");
    let b = BrepTransform::translate_solid(&a, Vec3::new(0.1, 0.0, 0.0));

    let report =
        BooleanResultVerifier::verify(&a, &b, std::slice::from_ref(&a), BooleanOpType::Union, &tol);
    assert!(
        !report.is_valid(),
        "a union that hands back one operand must be refused even when the model is small"
    );
}

#[test]
fn a_solid_minus_itself_is_empty_rather_than_an_error() {
    // 4-6 が一般経路について「空の交差は答えであって失敗ではない」と直した
    // のに、軸平行の箱の近道には入っていませんでした。戻り値が
    // `Option<Solid>` で、`None` が「この近道の出番ではない」を意味して
    // いたため、**空の答えを置く場所が型に無かった**のです。実装ではなく
    // 型が表現できていませんでした。
    let tol = Tolerance::default();
    let solid = PrimitiveBuilder::make_box(20.0, 20.0, 20.0).expect("box");

    let result = BooleanEngine::boolean_solids_exact_result(
        &solid,
        &solid.clone(),
        BooleanOpType::Difference,
        &tol,
    )
    .expect("A - A is empty, which is an answer");

    assert!(
        result.solids.is_empty(),
        "A - A should come back with no solids, got {}",
        result.solids.len()
    );
}

#[test]
fn a_solid_minus_a_copy_moved_by_a_hair_is_still_empty() {
    // 1e-12 は許容より下なので、同じ形として扱われるのが正しい挙動です。
    // 上の検査だけだと `std::ptr::eq` による早期の枝で通ってしまい、
    // 近道そのものを確かめたことになりません。
    let tol = Tolerance::default();
    let solid = PrimitiveBuilder::make_box(20.0, 20.0, 20.0).expect("box");
    let nudged = BrepTransform::translate_solid(&solid, Vec3::new(1e-12, 0.0, 0.0));

    let result =
        BooleanEngine::boolean_solids_exact_result(&solid, &nudged, BooleanOpType::Difference, &tol)
            .expect("a difference against a copy moved below tolerance is empty, not a failure");

    assert!(
        result.solids.is_empty(),
        "the difference should be empty, got {} solid(s)",
        result.solids.len()
    );
}

/// 面を辺で繋いで塊を数える。位置で繋ぐのは、分割された面が同じ稜を別々の
/// `Edge` として持つことがあるからです。
fn connected_pieces(solid: &zenith_topo::Solid, grid: f64) -> usize {
    use std::collections::{HashMap, HashSet};
    let key = |point: zenith_math::Point3| {
        (
            (point.x / grid).round() as i64,
            (point.y / grid).round() as i64,
            (point.z / grid).round() as i64,
        )
    };
    let faces = &solid.outer_shell.faces;
    let mut users: HashMap<((i64, i64, i64), (i64, i64, i64)), Vec<usize>> = HashMap::new();
    for (index, face) in faces.iter().enumerate() {
        for wire in std::iter::once(&face.outer_wire).chain(face.inner_wires.iter()) {
            for edge in &wire.edges {
                let (start, end) = edge.edge.curve.param_range();
                let a = key(edge.edge.curve.evaluate(start));
                let b = key(edge.edge.curve.evaluate(end));
                users.entry(if a <= b { (a, b) } else { (b, a) }).or_default().push(index);
            }
        }
    }
    let mut neighbours: Vec<Vec<usize>> = vec![Vec::new(); faces.len()];
    for sharing in users.values() {
        for (position, left) in sharing.iter().enumerate() {
            for right in sharing.iter().skip(position + 1) {
                if left != right {
                    neighbours[*left].push(*right);
                    neighbours[*right].push(*left);
                }
            }
        }
    }
    let mut seen: HashSet<usize> = HashSet::new();
    let mut components = 0;
    for start in 0..faces.len() {
        if seen.contains(&start) {
            continue;
        }
        components += 1;
        let mut stack = vec![start];
        seen.insert(start);
        while let Some(index) = stack.pop() {
            for next in &neighbours[index] {
                if seen.insert(*next) {
                    stack.push(*next);
                }
            }
        }
    }
    components
}

#[test]
fn a_cut_that_splits_a_solid_returns_separate_bodies() {
    // 板をスロットで分断すると答えは2つの塊です。以前はそれを1枚のシェルに
    // まとめて**1つの `Solid`** として返していました。
    //
    // **ゲートのどの検査にも掛かりません。** 体積は発散定理が両方を足すので
    // 正しく出て、各塊が閉じているのでシェルは「閉じている」と判定され、
    // 384点の内外判定も通ります。位相だけが違い、それを見る検査が無いのです。
    // 他カーネルは非連結なシェルを `MANIFOLD_SOLID_BREP` として読めません。
    let tol = Tolerance::default();
    let plate = PrimitiveBuilder::make_box(30.0, 30.0, 15.0).expect("plate");
    let slot = BrepTransform::translate_solid(
        &PrimitiveBuilder::make_box(60.0, 6.0, 40.0).expect("slot"),
        Vec3::new(-15.0, 12.0, -10.0),
    );

    let result =
        BooleanEngine::boolean_solids_exact_result(&plate, &slot, BooleanOpType::Difference, &tol)
            .expect("a slot right through the plate is an answer, not a failure");

    assert_eq!(
        result.solids.len(),
        2,
        "the slot leaves two bodies, so the result should carry two solids"
    );
    let grid = tol.linear.max(1e-9);
    for (index, solid) in result.solids.iter().enumerate() {
        assert_eq!(
            connected_pieces(solid, grid),
            1,
            "solid {index} should be a single connected body"
        );
    }

    let total: f64 = result
        .solids
        .iter()
        .map(|solid| MassCalculator::compute_from_brep(solid, &params()).volume)
        .sum();
    let expected = 30.0 * 30.0 * 15.0 - 30.0 * 6.0 * 15.0;
    assert!(
        (total - expected).abs() / expected < 1e-9,
        "the two halves should total {expected}, measured {total}"
    );
}

#[test]
fn a_drilled_plate_can_be_cut_by_a_slab() {
    // 穴あきの板をスラブで削ると、シェルは閉じて多様体なのに**同方向の辺使用**
    // が16 残って止まっていました。入力は無傷（同方向0）なので、分割が向きを
    // 変えています。
    //
    // 原因は、円柱面の分割が**辺の固有方向**を読んでいたことでした。`Edge` は
    // 始点と終点を持ちますが、ワイヤはそれを逆向きに使うことがあります。元の
    // 面が逆に巡回していた辺では、分割後の巡回が反転し、隣の無傷な面と同じ
    // 向きで辺を共有します。4-5 が別の経路で直したのと同じ罠でした。
    let tol = Tolerance::default();
    let plate = HoleBuilder::make_drilled_box(30.0, 30.0, 15.0, 5.0).expect("drilled plate");
    let slab = BrepTransform::translate_solid(
        &PrimitiveBuilder::make_box(60.0, 60.0, 10.0).expect("slab"),
        Vec3::new(-15.0, -15.0, 12.0),
    );

    let result =
        BooleanEngine::boolean_solids_exact_result(&plate, &slab, BooleanOpType::Difference, &tol)
            .expect("taking the top off a drilled plate is an ordinary cut");

    let volume: f64 = result
        .solids
        .iter()
        .map(|solid| MassCalculator::compute_from_brep(solid, &params()).volume)
        .sum();
    // 30 x 30 x 12 から半径5・高さ12 の穴を抜いたもの。
    let expected = 30.0 * 30.0 * 12.0 - std::f64::consts::PI * 25.0 * 12.0;
    assert!(
        (volume - expected).abs() / expected < 1e-9,
        "the cut plate should measure {expected}, got {volume}"
    );
}

#[test]
fn a_solid_with_a_cavity_carries_the_cavity_through_a_boolean() {
    // 空洞（inner_shells）を持つ立体に対しても、全シェルを考慮して
    // ブーリアン処理を行い、空洞が保持されることを検証する。
    let tol = Tolerance::default();
    let outer = PrimitiveBuilder::make_box(40.0, 40.0, 40.0).expect("outer");
    let inner = BrepTransform::translate_solid(
        &PrimitiveBuilder::make_box(10.0, 10.0, 10.0).expect("inner"),
        Vec3::new(15.0, 15.0, 15.0),
    );
    let hollow =
        BooleanEngine::boolean_solids_exact(&outer, &inner, BooleanOpType::Difference, &tol)
            .expect("subtracting a fully enclosed box makes a cavity");

    // まず、作るほうが本当に空洞になっていることを確かめます。ここが
    // 崩れていると、下の検査は何も見ていないことになります。
    assert_eq!(
        hollow.inner_shells.len(),
        1,
        "the difference should leave one cavity"
    );
    let hollow_volume = MassCalculator::compute_from_brep(&hollow, &params()).volume;
    assert!(
        (hollow_volume - 63000.0).abs() < 1e-6,
        "the hollow solid should measure 63000, got {hollow_volume}"
    );

    let knife = BrepTransform::translate_solid(
        &PrimitiveBuilder::make_box(60.0, 60.0, 10.0).expect("knife"),
        Vec3::new(-10.0, -10.0, 35.0),
    );
    let diff = BooleanEngine::boolean_solids_exact(&hollow, &knife, BooleanOpType::Difference, &tol)
        .expect("difference on hollow solid");
    assert_eq!(diff.inner_shells.len(), 1, "difference should keep the cavity");
    let diff_vol = MassCalculator::compute_from_brep(&diff, &params()).volume;
    assert!((diff_vol - 55000.0).abs() < 1e-4, "expected 55000, got {diff_vol}");

    let union = BooleanEngine::boolean_solids_exact(&hollow, &knife, BooleanOpType::Union, &tol)
        .expect("union on hollow solid");
    assert_eq!(union.inner_shells.len(), 1, "union should keep the cavity");
    let union_vol = MassCalculator::compute_from_brep(&union, &params()).volume;
    assert!((union_vol - 91000.0).abs() < 1e-4, "expected 91000, got {union_vol}");

    let inter = BooleanEngine::boolean_solids_exact(&hollow, &knife, BooleanOpType::Intersection, &tol)
        .expect("intersection on hollow solid");
    assert_eq!(inter.inner_shells.len(), 0, "intersection does not contain cavity");
    let inter_vol = MassCalculator::compute_from_brep(&inter, &params()).volume;
    assert!((inter_vol - 8000.0).abs() < 1e-4, "expected 8000, got {inter_vol}");
}

#[test]
fn the_gate_still_refuses_a_result_that_really_is_empty_when_it_should_not_be() {
    // ゼロ判定を緩めた側に倒したので、**本当に空であってはいけない場合を
    // 見逃していないか**を反対から確かめます。重なる2つの箱の積に空を
    // 差し出すと、384点の内外一貫性で弾かれなければなりません。
    let tol = Tolerance::default();
    let a = PrimitiveBuilder::make_box(20.0, 20.0, 20.0).expect("box a");
    let b = BrepTransform::translate_solid(&a, Vec3::new(10.0, 0.0, 0.0));

    let report =
        BooleanResultVerifier::verify(&a, &b, &[], BooleanOpType::Intersection, &tol);
    assert!(
        !report.is_valid(),
        "an empty intersection of two overlapping boxes must be refused"
    );
}
