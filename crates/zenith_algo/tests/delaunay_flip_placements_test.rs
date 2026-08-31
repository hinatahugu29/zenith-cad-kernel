//! **内側の対角線の入れ替えで壊れた3件**が、多様体のままであること（4-209）。
//!
//! 4-204 はこの入れ替えを入れて、`contact_placement_probe` のメッシュ非多様体が
//! 0件 → 3件になったので戻しました。4-209 で禁則を3つ足して入れ直しています。
//! **テストも B-Rep の検査も緑のまま、表示メッシュだけが壊れる**ところなので、
//! ここに常設で置きます。
//!
//! 壊れていた形は「接している面どうしが、境界に沿った同じ薄片や、境界の直線
//! 区間をまたぐ同じ弦を、両方の面の内側に持ってしまう」ことでした。

use zenith_algo::{BooleanEngine, BooleanOpType, BrepTransform, PrimitiveBuilder};
use zenith_math::{Tolerance, Transform3, Vec3};
use zenith_tess::TessellationParams;
use zenith_topo::Solid;

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

#[test]
fn the_three_placements_that_the_flip_used_to_break_stay_manifold() {
    let tol = Tolerance::default();
    let boxa = PrimitiveBuilder::make_box(20.0, 20.0, 20.0).expect("box");
    let cylinder = PrimitiveBuilder::make_cylinder(6.0, 40.0).expect("cylinder");
    let cone = PrimitiveBuilder::make_cone(10.0, 0.0, 20.0).expect("cone");
    let torus = PrimitiveBuilder::make_torus(12.0, 4.0).expect("torus");

    // 円錐を半頂角だけ回すと母線が鉛直になる。その母線が箱の面 x=20 に乗る。
    let half_angle = (10f64 / 20.0).atan();
    let generatrix_x = 20.0 / 5f64.sqrt();
    let standing_cone = BrepTransform::translate_solid(
        &BrepTransform::transform_solid(
            &cone,
            &Transform3::from_axis_angle(&Vec3::new(0.0, 1.0, 0.0), half_angle),
        )
        .expect("stand the cone"),
        Vec3::new(20.0 - generatrix_x, 10.0, 5.0),
    );

    // 切る側と切られる側を両方回す（箱 19 度、円柱 27 度）。
    let spun_box = BrepTransform::transform_solid(
        &boxa,
        &Transform3::from_axis_angle(&Vec3::new(0.0, 0.0, 1.0), 19f64.to_radians()),
    )
    .expect("spin the box");
    let turned_cylinder = BrepTransform::translate_solid(
        &BrepTransform::transform_solid(
            &cylinder,
            &Transform3::from_axis_angle(&Vec3::new(1.0, 0.0, 0.0), 27f64.to_radians()),
        )
        .expect("tilt the cylinder"),
        Vec3::new(6.0, 10.0, -10.0),
    );

    // 16パッチのトーラスを 25 度傾けて箱と交差させる。
    let inclined_torus = BrepTransform::translate_solid(
        &BrepTransform::transform_solid(
            &torus,
            &Transform3::from_axis_angle(&Vec3::new(1.0, 1.0, 0.0), 25f64.to_radians()),
        )
        .expect("incline the torus"),
        Vec3::new(10.0, 10.0, 10.0),
    );

    // **A の側も配置ごとに違います**（`box x cylinder (both turned)` は箱も
    // 回します）。掃き出しと同じ置き方でなければ、同じ欠陥を踏みません。
    let cases = [
        ("box x cone (generatrix in a face)", boxa.clone(), standing_cone),
        ("box x cylinder (both turned)", spun_box, turned_cylinder),
        ("box x torus (inclined 25deg)", boxa.clone(), inclined_torus),
    ];
    let ops = [
        ("union", BooleanOpType::Union),
        ("difference", BooleanOpType::Difference),
        ("intersection", BooleanOpType::Intersection),
    ];

    for (name, a, b) in cases {
        for (label, op) in ops {
            let Ok(result) = BooleanEngine::boolean_solids_exact_result(&a, &b, op, &tol) else {
                // 断ること自体は、ここでは赤にしません（規約は 3-1）。
                continue;
            };
            for (index, solid) in result.solids.iter().enumerate() {
                let bad = non_manifold_edges(solid);
                assert_eq!(
                    bad, 0,
                    "{name} / {label} / 立体 {index}: 非多様体の稜が {bad} 本"
                );
            }
        }
    }
}
