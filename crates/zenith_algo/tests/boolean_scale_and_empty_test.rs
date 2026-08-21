//! 桁の離れた立体と、答えが空になる演算。
//!
//! どちらも `boolean_envelope` の45ケースには入っていません。あの表は半径も
//! 距離も桁が揃った配置ばかりで、実務のデータが持ち込む性質——筐体とネジの
//! ような桁差、同じ形どうしの演算——が無いからです。`robustness_probe` が
//! 両方を見つけました。ここはその再発を見張ります。

use zenith_algo::{
    BooleanEngine, BooleanOpType, BooleanResultVerifier, BrepTransform, MassCalculator,
    PrimitiveBuilder,
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
