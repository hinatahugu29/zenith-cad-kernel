//! 中に入っている立体を引くと、空洞（内側シェル）になる。
//!
//! **向きの約束が要ります。** `MassCalculator::compute_from_brep` は
//! 「空洞シェルは外殻と同じ向きで保持されるため、寄与を反転して足す」と
//! 書いてあります。差の B 側は面を反転して採るので、空洞になる塊は
//! **内向き**で出てくることがあり、そのまま入れると符号が2回変わって
//! 体積が `A - B` ではなく `A + B` になります（4-133）。
//!
//! **`volume > 0` では捕まりません**（`A + B` にも体積はあります）。
//! ここは全部、閉じた式で押さえます。

use zenith_algo::{
    BooleanEngine, BooleanOpType, BrepTransform, MassCalculator, PrimitiveBuilder,
};
use zenith_math::{Tolerance, Vec3};
use zenith_tess::TessellationParams;
use zenith_topo::Solid;

fn params() -> TessellationParams {
    TessellationParams {
        u_divisions: 64,
        v_divisions: 64,
    }
}

fn ball(radius: f64) -> f64 {
    4.0 / 3.0 * std::f64::consts::PI * radius * radius * radius
}

/// 空洞になった結果を、体積と位相の両方で確かめる。
fn check_cavity(name: &str, a: &Solid, b: &Solid, expected: f64) {
    let tol = Tolerance::default();
    let result = BooleanEngine::boolean_solids_exact_result(a, b, BooleanOpType::Difference, &tol)
        .unwrap_or_else(|err| panic!("{name}: contained difference was refused: {err}"));

    assert_eq!(result.solids.len(), 1, "{name}: should return one solid");
    let solid = &result.solids[0];
    assert_eq!(
        solid.inner_shells.len(),
        1,
        "{name}: the tool should become one cavity shell"
    );

    // **空洞シェルは外殻と同じ向きで持ちます。** 単体で積むと正になります。
    let cavity_alone = Solid::new(solid.inner_shells[0].clone(), Vec::new());
    let cavity_volume = MassCalculator::compute_from_brep(&cavity_alone, &params()).volume;
    assert!(
        cavity_volume > 0.0,
        "{name}: the cavity shell is inside-out (signed volume {cavity_volume})"
    );

    let volume = MassCalculator::compute_from_brep(solid, &params()).volume;
    assert!(
        (volume - expected).abs() <= expected * 1e-6,
        "{name}: volume {volume} is not the closed form {expected}"
    );
}

/// 箱の中の球。**この配置だけは 4-133 の前から通っていました。**
/// 回帰として残します。
#[test]
fn a_box_minus_a_sphere_strictly_inside_becomes_a_cavity() {
    let boxa = PrimitiveBuilder::make_box(20.0, 20.0, 20.0).unwrap();
    let sphere = BrepTransform::translate_solid(
        &PrimitiveBuilder::make_sphere(5.0).unwrap(),
        Vec3::new(10.0, 10.0, 10.0),
    );
    check_cavity("box - sphere (r=5)", &boxa, &sphere, 8000.0 - ball(5.0));
}

/// **円錐の中の球。** 触れてもいないのに、4-133 まで断られていました。
/// 空洞シェルが内向きで入り、体積が `A + B` になって検証ゲートが
/// 落としていたためです。
#[test]
fn a_cone_minus_a_sphere_strictly_inside_becomes_a_cavity() {
    let cone = PrimitiveBuilder::make_cone(10.0, 0.0, 20.0).unwrap();
    let cone_volume = std::f64::consts::PI * 100.0 * 20.0 / 3.0;
    let half_angle = (10f64 / 20.0).atan();
    // 側面に内接する球の中心は、頂点から r / sin(半頂角) 下がったところ。
    // ここは半径 3 の内接位置に、半径 1.5 の球を置くので**触れません**。
    let centre_z = 20.0 - 3.0 / half_angle.sin();
    let sphere = BrepTransform::translate_solid(
        &PrimitiveBuilder::make_sphere(1.5).unwrap(),
        Vec3::new(0.0, 0.0, centre_z),
    );
    check_cavity(
        "cone - sphere (r=1.5)",
        &cone,
        &sphere,
        cone_volume - ball(1.5),
    );
}

/// **触れている内接。** 球が円錐の側面に円まるごとで接します。
#[test]
fn a_cone_minus_an_inscribed_sphere_becomes_a_cavity() {
    let cone = PrimitiveBuilder::make_cone(10.0, 0.0, 20.0).unwrap();
    let cone_volume = std::f64::consts::PI * 100.0 * 20.0 / 3.0;
    let half_angle = (10f64 / 20.0).atan();
    let radius = 3.0;
    let sphere = BrepTransform::translate_solid(
        &PrimitiveBuilder::make_sphere(radius).unwrap(),
        Vec3::new(0.0, 0.0, 20.0 - radius / half_angle.sin()),
    );
    check_cavity(
        "cone - sphere (inscribed)",
        &cone,
        &sphere,
        cone_volume - ball(radius),
    );
}

/// **箱の6面に接する球。** 触れているので「厳密に内側」ではありません。
#[test]
fn a_box_minus_a_sphere_touching_all_six_faces_becomes_a_cavity() {
    let boxa = PrimitiveBuilder::make_box(20.0, 20.0, 20.0).unwrap();
    let sphere = BrepTransform::translate_solid(
        &PrimitiveBuilder::make_sphere(10.0).unwrap(),
        Vec3::new(10.0, 10.0, 10.0),
    );
    check_cavity(
        "box - sphere (inscribed)",
        &boxa,
        &sphere,
        8000.0 - ball(10.0),
    );
}
