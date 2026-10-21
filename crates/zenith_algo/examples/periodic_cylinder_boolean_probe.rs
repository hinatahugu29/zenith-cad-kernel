//! **ROADMAP 柱③の最後の実装・検証フェーズへの第一歩**（HANDOVER 4-808〜
//! 4-814）。
//!
//! これまでの検証（`validate_closed`・FreeCADでの読み込み・体積・
//! フィレット）はすべて**単独のプリミティブ**に対してだった。**ブーリアン
//! は交線を辿るコードが周期曲面のu方向境界をまたぐケースを初めて踏む**
//! ので、ここで実際に通す。
//!
//! `PrimitiveBuilder::make_cylinder_periodic`（周期1面・既定にはまだ
//! していない新しい構成）と`make_cylinder`（既定の4分割）、それぞれを
//! 同じ箱と3種のブーリアン演算（差・積・和）で突き合わせ、**体積・面数
//! まで1ビットも動かないか**を見る。

use zenith_algo::{BooleanEngine, BooleanOpType, BrepTransform, MassCalculator, PrimitiveBuilder};
use zenith_math::{Tolerance, Transform3, Vec3};
use zenith_tess::TessellationParams;

fn main() {
    let radius = 10.0;
    let height = 20.0;
    let tol = Tolerance::default();
    let params = TessellationParams { u_divisions: 64, v_divisions: 64 };

    // 円柱を貫く箱——円柱の軸を斜めに横切るのではなく、単純に重なる配置
    // （既存の`cone_slab_probe`などと同じ「箱 × 回転体」の型）。
    let box_solid = PrimitiveBuilder::make_box(30.0, 30.0, 30.0).unwrap();
    let standard_cylinder = PrimitiveBuilder::make_cylinder(radius, height).unwrap();
    let periodic_cylinder = PrimitiveBuilder::make_cylinder_periodic(radius, height).unwrap();

    // 円柱を箱の中心へ寄せる（原点基準の円柱を箱の中心軸に合わせる）。
    // ここでは単純に、箱の中心 (15,15,*) に円柱の軸 (0,0,*) が来るよう
    // 移動した円柱を作り直すのではなく、箱を円柱の軸に合わせて置く
    // ——円柱はどちらも原点中心の軸なので、箱を (-15,-15,-5) ずらして
    // 重ねればよい。
    let box_centered = BrepTransform::translate_solid(&box_solid, Vec3::new(-15.0, -15.0, 5.0));

    let run = |name: &str, a: &zenith_topo::Solid, standard_b: &zenith_topo::Solid, periodic_b: &zenith_topo::Solid| {
        for op in [
            BooleanOpType::Difference,
            BooleanOpType::Intersection,
            BooleanOpType::Union,
        ] {
            let standard_result = BooleanEngine::boolean_solids_exact(a, standard_b, op, &tol);
            let periodic_result = BooleanEngine::boolean_solids_exact(a, periodic_b, op, &tol);

            match (standard_result, periodic_result) {
                (Ok(ra), Ok(rb)) => {
                    let va = MassCalculator::compute_volume_from_brep(&ra, &params);
                    let vb = MassCalculator::compute_volume_from_brep(&rb, &params);
                    let fa = ra.outer_shell.faces.len();
                    let fb = rb.outer_shell.faces.len();
                    println!(
                        "{name} {op:?}: 4分割版 体積={va:.6} 面{fa}枚 / 周期1面版 体積={vb:.6} 面{fb}枚 / 体積の相対差={:.3e}",
                        (va - vb).abs() / va.abs().max(1e-9)
                    );
                }
                (Err(ea), Err(eb)) => {
                    println!("{name} {op:?}: 両方とも断られた（4分割版: {ea} / 周期1面版: {eb}）");
                }
                (Ok(_), Err(eb)) => {
                    println!("{name} {op:?}: 4分割版は通ったが周期1面版が断られた: {eb}");
                }
                (Err(ea), Ok(_)) => {
                    println!("{name} {op:?}: 周期1面版は通ったが4分割版が断られた: {ea}");
                }
            }
        }
    };

    run("箱×円柱", &box_centered, &standard_cylinder, &periodic_cylinder);

    // **円柱×円柱を直交させる**（シュタインメッツ立体。4-478が「周期境界
    // をまたぐ交線」で踏んだ難所と同じ型の配置）——より厳しい検体。
    let length_b = height * 2.0;
    let standard_cylinder_b = PrimitiveBuilder::make_cylinder(radius * 0.6, length_b).unwrap();
    let periodic_cylinder_b = PrimitiveBuilder::make_cylinder_periodic(radius * 0.6, length_b).unwrap();
    // `cross_cylinder_sweep_probe`と同じ型: 中心を原点に合わせて寝かせ、
    // y軸まわりに90度回してx軸向きにする。
    let lay_on_x_axis = |solid: &zenith_topo::Solid| -> zenith_topo::Solid {
        let centred = BrepTransform::translate_solid(solid, Vec3::new(0.0, 0.0, -length_b * 0.5));
        let lay = Transform3::from_axis_angle(&Vec3::y(), std::f64::consts::FRAC_PI_2);
        let placed = BrepTransform::transform_solid(&centred, &lay).expect("回す");
        BrepTransform::translate_solid(&placed, Vec3::new(0.0, 0.0, height / 2.0))
    };
    let standard_cylinder_b_placed = lay_on_x_axis(&standard_cylinder_b);
    let periodic_cylinder_b_placed = lay_on_x_axis(&periodic_cylinder_b);

    run(
        "円柱×円柱(直交)",
        &standard_cylinder,
        &standard_cylinder_b_placed,
        &periodic_cylinder_b_placed,
    );
}
