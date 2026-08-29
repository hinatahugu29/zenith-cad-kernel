//! 円柱の面を、**面の中身を刻まずに**積む経路（4-156）。
//!
//! 45ケースの面積分の 44.8% が真の円柱です（4-154）。そこは
//! `∬ dθ dv` の境界積分だけで閉じるので、三角形が1枚も要りません。
//!
//! ここで押さえるのは2つです。
//!
//! - **三角形で積んだ答えと一致すること**（自分の式を自分で検算しない）
//! - **閉じた式に乗ること**（三角形の側が間違っていても気づけるように）

use zenith_algo::{
    BooleanEngine, BooleanOpType, BrepTransform, MassCalculator, PrimitiveBuilder,
};
use zenith_math::{Tolerance, Vec3};
use zenith_tess::TessellationParams;
use zenith_topo::{Face, FaceGeometry, Solid};

fn params() -> TessellationParams {
    TessellationParams {
        u_divisions: 48,
        v_divisions: 48,
    }
}

/// 解析経路を止めて積む。**同じ口を切り替えて2回測る**ための栓。
///
/// **環境変数はプロセス全体に効きます。** テストは既定で並列に走るので、
/// この錠で直列にします。付けずに走らせると、隣のテストが栓の開け閉めに
/// 巻き込まれます。
static SWITCH: std::sync::Mutex<()> = std::sync::Mutex::new(());

fn without_analytic<T>(body: impl FnOnce() -> T) -> T {
    let guard = SWITCH.lock().unwrap_or_else(|poisoned| poisoned.into_inner());
    std::env::set_var("ZENITH_NO_ANALYTIC_FACE", "1");
    let value = body();
    std::env::remove_var("ZENITH_NO_ANALYTIC_FACE");
    drop(guard);
    value
}

fn nurbs_faces(solid: &Solid) -> Vec<&Face> {
    solid
        .outer_shell
        .faces
        .iter()
        .filter(|face| matches!(face.geometry, FaceGeometry::Nurbs(_)))
        .collect()
}

/// 素の円柱の側面。**閉じた式が書けます**——四半パッチの面積は
/// `r · (π/2) · h`。
#[test]
fn a_cylinder_side_patch_matches_the_closed_form() {
    let cylinder = PrimitiveBuilder::make_cylinder(6.0, 40.0).expect("cylinder");
    let exact = 6.0 * std::f64::consts::FRAC_PI_2 * 40.0;

    let faces = nurbs_faces(&cylinder);
    assert!(!faces.is_empty(), "円柱には曲面の側面があるはず");
    for face in faces {
        let (area, _) = MassCalculator::compute_face_integral(face, &params());
        assert!(
            (area - exact).abs() <= exact * 1e-9,
            "四半パッチの面積 {area} は r*(pi/2)*h = {exact} に乗るはず"
        );
    }
}

/// **解析と三角形が一致すること。** どちらかだけを信じないための検査です。
#[test]
fn the_analytic_cylinder_agrees_with_the_tessellated_integral() {
    let cylinder = PrimitiveBuilder::make_cylinder(6.0, 40.0).expect("cylinder");
    for face in nurbs_faces(&cylinder) {
        let analytic = MassCalculator::compute_face_integral(face, &params());
        let tessellated = without_analytic(|| MassCalculator::compute_face_integral(face, &params()));
        assert!(
            (analytic.0 - tessellated.0).abs() <= tessellated.0.abs() * 1e-9,
            "面積が食い違う: 解析 {} vs 三角形 {}",
            analytic.0,
            tessellated.0
        );
        assert!(
            (analytic.1 - tessellated.1).abs() <= tessellated.1.abs().max(1.0) * 1e-9,
            "体積が食い違う: 解析 {} vs 三角形 {}",
            analytic.1,
            tessellated.1
        );
    }
}

/// **トリムされた円柱面**（ボア壁）でも一致すること。素の四半パッチだけ
/// 合っていても、実際に効くのは割られた面のほうです。
#[test]
fn a_trimmed_bore_wall_agrees_with_the_tessellated_integral() {
    let tol = Tolerance::default();
    let block = PrimitiveBuilder::make_box(20.0, 20.0, 20.0).expect("box");
    let drill = BrepTransform::translate_solid(
        &PrimitiveBuilder::make_cylinder(6.0, 40.0).expect("cylinder"),
        Vec3::new(10.0, 10.0, -10.0),
    );
    let holed = BooleanEngine::boolean_solids_exact_result(
        &block,
        &drill,
        BooleanOpType::Difference,
        &tol,
    )
    .expect("穴あけは通るはず");

    let solid = &holed.solids[0];
    let walls = nurbs_faces(solid);
    assert!(!walls.is_empty(), "ボア壁は曲面のはず");

    let mut analytic_total = 0.0;
    let mut tessellated_total = 0.0;
    for face in &walls {
        let analytic = MassCalculator::compute_face_integral(face, &params());
        let tessellated = without_analytic(|| MassCalculator::compute_face_integral(face, &params()));
        assert!(
            (analytic.0 - tessellated.0).abs() <= tessellated.0.abs() * 1e-9,
            "ボア壁の面積が食い違う: 解析 {} vs 三角形 {}",
            analytic.0,
            tessellated.0
        );
        analytic_total += analytic.0;
        tessellated_total += tessellated.0;
    }

    // ボア壁を全部足すと、貫通穴の側面 2πrh になる。
    let exact = 2.0 * std::f64::consts::PI * 6.0 * 20.0;
    assert!(
        (analytic_total - exact).abs() <= exact * 1e-6,
        "ボア壁の合計 {analytic_total} は 2*pi*r*h = {exact} に乗るはず（三角形側は {tessellated_total}）"
    );
}
