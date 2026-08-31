//! `cone × torus` の表示メッシュが多様体であること（4-208）。
//!
//! **ここは長く赤かったところです。** 4-207 で「境界の辺が1本足りない」ぶんを
//! 埋めても4演算が残り、機構に名前が付いていませんでした。原因は**稜の曲線の
//! 端が頂点の位置と 1.1〜1.3e-7 ずれている**ことで、境界の標本を曲線から取ると
//! 継ぎ目に「同じはずの点」が2つでき、**溶接の距離 (1e-7) より大きいので
//! 束ねられず**、そこが穴になっていました。
//!
//! B-Rep はどれも多様体で、恒等式も破れません。**表示メッシュだけが壊れます。**
//! テストも B-Rep の検査も緑のまま通り抜けるので、ここに常設で置きます。

use zenith_algo::{BooleanEngine, BooleanOpType, BrepTransform, PrimitiveBuilder};
use zenith_math::{Tolerance, Vec3};
use zenith_tess::TessellationParams;
use zenith_topo::Solid;

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

#[test]
fn a_cone_meeting_a_torus_tessellates_into_a_manifold_mesh() {
    let tol = Tolerance::default();
    let cone = PrimitiveBuilder::make_cone(10.0, 0.0, 20.0).expect("cone");
    let torus = PrimitiveBuilder::make_torus(12.0, 4.0).expect("torus");

    // 4-205 の配置（縁を管に当てる）と、4-207 の配置（そこから z に 3
    // 持ち上げる）。**壊れていたのはこの2つで、合わせて4演算**でした。
    let placements = [
        ("rim on the tube", Vec3::new(10.0, 0.0, 0.0)),
        ("lifted off the base plane", Vec3::new(10.0, 0.0, 3.0)),
    ];
    let ops = [
        ("union", BooleanOpType::Union),
        ("difference", BooleanOpType::Difference),
        ("intersection", BooleanOpType::Intersection),
    ];

    for (placement, offset) in placements {
        let b = BrepTransform::translate_solid(&torus, offset);
        for (label, op) in ops {
            let result = BooleanEngine::boolean_solids_exact_result(&cone, &b, op, &tol)
                .unwrap_or_else(|reason| panic!("{placement} / {label} が断られた: {reason}"));
            for (index, solid) in result.solids.iter().enumerate() {
                let bad = non_manifold_edges(solid);
                assert_eq!(
                    bad, 0,
                    "{placement} / {label} / 立体 {index}: 非多様体の稜が {bad} 本"
                );
            }
        }
    }
}

/// **上流の値そのものを押さえます。**
///
/// 表示側で頂点へ寄せるのは対症でした。上流（`sew` が頂点を束ねるところ）を
/// 直したので、ここは丸め誤差の桁です。実測（2026/08/31）: 最大 **3.972e-15**。
#[test]
fn the_gap_between_an_edge_curve_end_and_its_vertex_stays_where_it_was_measured() {
    let tol = Tolerance::default();
    let cone = PrimitiveBuilder::make_cone(10.0, 0.0, 20.0).expect("cone");
    let torus = PrimitiveBuilder::make_torus(12.0, 4.0).expect("torus");
    let b = BrepTransform::translate_solid(&torus, Vec3::new(10.0, 0.0, 3.0));
    let result =
        BooleanEngine::boolean_solids_exact_result(&cone, &b, BooleanOpType::Intersection, &tol)
            .expect("intersection");

    let mut worst = 0.0f64;
    for solid in &result.solids {
        for shell in std::iter::once(&solid.outer_shell).chain(solid.inner_shells.iter()) {
            for face in &shell.faces {
                for wire in std::iter::once(&face.outer_wire).chain(face.inner_wires.iter()) {
                    for oriented in &wire.edges {
                        for (fraction, vertex) in [
                            (0.0, oriented.start_vertex().point),
                            (1.0, oriented.end_vertex().point),
                        ] {
                            worst = worst
                                .max((oriented.evaluate_normalized(fraction) - vertex).norm());
                        }
                    }
                }
            }
        }
    }
    // **上流を直したので、ここは丸め誤差の桁になりました**（4-208）。
    // 縫うときに頂点だけ束ねて曲線を置き去りにしていたのが原因で、
    // `Edge::with_vertices` を通すようにしたら 1.330e-7 → 3.972e-15。
    // 桁が戻ったら鳴らします。
    assert!(
        worst <= 1.0e-9,
        "稜の曲線の端と頂点の差が {worst:.3e} まで開いた（2026/08/31 の実測は 3.972e-15）"
    );
}
