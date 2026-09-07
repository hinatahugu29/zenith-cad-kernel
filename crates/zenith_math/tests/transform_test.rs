//! **座標変換の逆**を、閉じた式で測る（4-397）。
//!
//! # なぜ要るのか
//!
//! `Transform3::inverse` は、**このリポジトリのどこからも呼ばれていません**
//! ——試験にも例にも出てこず、`src` の中にも呼び手がいません（4-397 の
//! 棚卸し）。**公開しているのに、誰も測っていない関数**でした。
//!
//! **逆変換は、間違っても形が「それらしく」出ます。** 回転が逆向きでも、
//! 平行移動の符号が違っても、**立体は立体のまま**です。**閉じた式で
//! 縛らないと、気づけません。**
//!
//! # 閉じた式
//!
//! ```text
//! T⁻¹(T(p)) = p        どんな点でも、元へ戻る
//! T ∘ T⁻¹  = 恒等変換   合成すると何もしない
//! ```

use zenith_math::{Point3, Transform3, Vec3};

/// 平行移動・回転・拡大を混ぜた変換。**1 種類だけだと、取り違えが
/// 打ち消し合って見えなくなります。**
fn mixed() -> Transform3 {
    Transform3::from_translation(Vec3::new(13.0, -7.0, 41.0))
        .compose(&Transform3::from_axis_angle(
            &Vec3::new(1.0, 2.0, 3.0),
            0.7,
        ))
        .compose(&Transform3::from_scale(2.5))
}

#[test]
fn the_inverse_takes_every_point_back() {
    let transform = mixed();
    let inverse = transform.inverse().expect("逆が取れません");

    // **原点だけで測ってはいけません**——平行移動の誤りは通ってしまいます。
    for point in [
        Point3::origin(),
        Point3::new(1.0, 0.0, 0.0),
        Point3::new(-31.0, 17.0, 5.0),
        Point3::new(137.0, -91.0, 53.0),
    ] {
        let there = transform.transform_point(&point);
        let back = inverse.transform_point(&there);
        let residual = (back - point).norm() / point.coords.norm().max(1.0);
        assert!(
            residual <= 1e-12,
            "({:.3} {:.3} {:.3}) が戻りません: ({:.9} {:.9} {:.9})、相対 {residual:.3e}",
            point.x,
            point.y,
            point.z,
            back.x,
            back.y,
            back.z
        );
    }
}

#[test]
fn composing_with_the_inverse_is_the_identity() {
    let transform = mixed();
    let inverse = transform.inverse().expect("逆が取れません");
    let identity = Transform3::identity();

    // **合成してから点に掛けます。** 行列そのものを覗くのではなく、
    // **振る舞いで測ります**。
    for (label, composed) in [
        ("T ∘ T⁻¹", transform.compose(&inverse)),
        ("T⁻¹ ∘ T", inverse.compose(&transform)),
    ] {
        for point in [
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(-31.0, 17.0, 5.0),
            Point3::new(137.0, -91.0, 53.0),
        ] {
            let moved = composed.transform_point(&point);
            let want = identity.transform_point(&point);
            let residual = (moved - want).norm() / point.coords.norm().max(1.0);
            assert!(
                residual <= 1e-12,
                "{label} が恒等になりません（相対 {residual:.3e}）"
            );
        }
    }
}

#[test]
fn the_inverse_undoes_the_direction_of_a_vector_too() {
    // **点と向きでは掛かり方が違います**（平行移動が効くかどうか）。
    // **両方測ります。**
    let transform = mixed();
    let inverse = transform.inverse().expect("逆が取れません");
    for vector in [
        Vec3::new(1.0, 0.0, 0.0),
        Vec3::new(0.0, 1.0, 0.0),
        Vec3::new(-3.0, 5.0, 7.0),
    ] {
        let there = transform.transform_vector(&vector);
        let back = inverse.transform_vector(&there);
        let residual = (back - vector).norm() / vector.norm();
        assert!(
            residual <= 1e-12,
            "向きが戻りません（相対 {residual:.3e}）"
        );
    }
}

#[test]
fn a_flattening_transform_has_no_inverse() {
    // **潰れた変換には逆がありません。** `None` を返すこと——
    // **もっともらしい行列を返してはいけません。**
    let flat = Transform3::from_scale(0.0);
    assert!(
        flat.inverse().is_none(),
        "潰れた変換なのに逆が返りました"
    );
}
