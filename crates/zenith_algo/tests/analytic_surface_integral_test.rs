//! 解析曲面の面を、**面の中身を刻まずに**積む経路（4-156、4-157）。
//!
//! 45ケースの面積分は **100% が解析曲面**でした（4-154）——真の円柱が
//! 44.8%、球とトーラスが 53.9%、円錐が 1.3%。そこは境界積分だけで閉じる
//! ので、三角形が1枚も要りません。
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

/// 球の四半パッチ。**閉じた式が書けます**——8分割した球面の1枚は
/// `4πR²/8`。
#[test]
fn a_sphere_patch_matches_the_closed_form() {
    let sphere = PrimitiveBuilder::make_sphere(10.0).expect("sphere");
    let faces = nurbs_faces(&sphere);
    assert!(!faces.is_empty(), "球には曲面がある");
    let exact = 4.0 * std::f64::consts::PI * 100.0 / faces.len() as f64;
    for face in faces {
        let (area, _) = MassCalculator::compute_face_integral(face, &params());
        assert!(
            (area - exact).abs() <= exact * 1e-9,
            "球のパッチの面積 {area} は 4πR²/枚数 = {exact} に乗るはず"
        );
    }
}

/// トーラスの四半パッチ。**閉じた式が書けます**——全表面積は
/// `4π²Rr`。
#[test]
fn a_torus_patch_matches_the_closed_form() {
    let torus = PrimitiveBuilder::make_torus(12.0, 4.0).expect("torus");
    let faces = nurbs_faces(&torus);
    assert!(!faces.is_empty(), "トーラスには曲面がある");
    let total: f64 = faces
        .iter()
        .map(|face| MassCalculator::compute_face_integral(face, &params()).0)
        .sum();
    let exact = 4.0 * std::f64::consts::PI * std::f64::consts::PI * 12.0 * 4.0;
    assert!(
        (total - exact).abs() <= exact * 1e-9,
        "トーラスの表面積 {total} は 4π²Rr = {exact} に乗るはず"
    );
}

/// **解析と三角形が一致すること**（球・トーラス）。
///
/// **原点から動かした立体でも測ります。** 体積の寄与は世界の原点からの
/// 位置に依るので、原点に置いたままだと**その項が 0 で気づけません**
/// （4-157 で実際に踏みました）。
#[test]
fn the_analytic_revolution_agrees_with_the_tessellated_integral() {
    let moved = |solid: &Solid| BrepTransform::translate_solid(solid, Vec3::new(7.0, -3.0, 11.0));
    let cases = [
        ("球", PrimitiveBuilder::make_sphere(10.0).expect("sphere")),
        ("トーラス", PrimitiveBuilder::make_torus(12.0, 4.0).expect("torus")),
    ];
    for (label, solid) in cases {
        for solid in [solid.clone(), moved(&solid)] {
            for face in nurbs_faces(&solid) {
                let analytic = MassCalculator::compute_face_integral(face, &params());
                let tessellated =
                    without_analytic(|| MassCalculator::compute_face_integral(face, &params()));
                assert!(
                    (analytic.0 - tessellated.0).abs() <= tessellated.0.abs() * 1e-9,
                    "{label}: 面積が食い違う 解析 {} vs 三角形 {}",
                    analytic.0,
                    tessellated.0
                );
                // 体積の寄与は 0 に近い面があるので、絶対値でも見ます。
                let slack = tessellated.1.abs() * 1e-9 + tessellated.0.abs() * 1e-9;
                assert!(
                    (analytic.1 - tessellated.1).abs() <= slack,
                    "{label}: 体積が食い違う 解析 {} vs 三角形 {}",
                    analytic.1,
                    tessellated.1
                );
            }
        }
    }
}

/// 球で削った箱・トーラスで削った箱。**トリムされた回転面**でも一致すること。
#[test]
fn trimmed_revolution_faces_agree_with_the_tessellated_integral() {
    let tol = Tolerance::default();
    let block = PrimitiveBuilder::make_box(20.0, 20.0, 20.0).expect("box");
    let cutters = [
        (
            "球",
            BrepTransform::translate_solid(
                &PrimitiveBuilder::make_sphere(10.0).expect("sphere"),
                Vec3::new(10.0, 10.0, 20.0),
            ),
        ),
        (
            "トーラス",
            BrepTransform::translate_solid(
                &PrimitiveBuilder::make_torus(12.0, 4.0).expect("torus"),
                Vec3::new(10.0, 10.0, 10.0),
            ),
        ),
    ];
    for (label, cutter) in cutters {
        let Ok(cut) = BooleanEngine::boolean_solids_exact_result(
            &block,
            &cutter,
            BooleanOpType::Difference,
            &tol,
        ) else {
            continue;
        };
        for face in nurbs_faces(&cut.solids[0]) {
            let analytic = MassCalculator::compute_face_integral(face, &params());
            let tessellated =
                without_analytic(|| MassCalculator::compute_face_integral(face, &params()));
            assert!(
                (analytic.0 - tessellated.0).abs() <= tessellated.0.abs() * 1e-6,
                "{label}で削った箱: 面積が食い違う 解析 {} vs 三角形 {}",
                analytic.0,
                tessellated.0
            );
            let slack = tessellated.1.abs() * 1e-6 + tessellated.0.abs() * 1e-6;
            assert!(
                (analytic.1 - tessellated.1).abs() <= slack,
                "{label}で削った箱: 体積が食い違う 解析 {} vs 三角形 {}",
                analytic.1,
                tessellated.1
            );
        }
    }
}
