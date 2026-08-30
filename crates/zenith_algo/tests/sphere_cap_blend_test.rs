//! 球を平面で切った立体の、**円い縁**のフィレットと面取り。
//!
//! # 何を物差しにするか
//!
//! この形は**回転体**なので、縁を落として消える体積に**閉じた式**があります。
//! 面の枚数や位相が合っているだけでは「答えが合っている」とは言えないので、
//! ここは体積で採点します。
//!
//! 断面は「球の弧 → つなぎ → 蓋の直線」で、つなぎがフィレットなら円弧
//! （厳密なトーラス）、面取りなら直線（厳密な円錐台）です。円板法で積むと
//! どちらも初等関数で書けます。**カーネルの中の式とは別に、ここで書き直して
//! います**——同じ式を両側から呼ぶと、式が間違っていても一致してしまいます。
//!
//! # 検体
//!
//! OpenCASCADE が書いた半球（`occ_reference_sphere_capped.step`、体積
//! `(2/3)πR³`）。**外から来た立体**なので、自前のビルダーの癖には依りません。

use std::f64::consts::PI;

use zenith_algo::{BlendKind, EdgeBlender, MassCalculator};
use zenith_io::StepImporter;
use zenith_tess::TessellationParams;
use zenith_topo::Solid;

/// **絶対パスで指します。** 相対パスにすると `cargo test` では通り、
/// `tools/fast_test.sh`（テストバイナリを直に走らせる）では落ちます——
/// 作業ディレクトリが違うためです。実際に一度踏みました。
const FIXTURE: &str = concat!(
    env!("CARGO_MANIFEST_DIR"),
    "/tests/fixtures/occ_reference_sphere_capped.step"
);
const R_SPHERE: f64 = 10.0;

fn volume(solid: &Solid) -> f64 {
    MassCalculator::compute_from_brep(
        solid,
        &TessellationParams {
            u_divisions: 96,
            v_divisions: 96,
        },
    )
    .volume
}

fn hemisphere() -> Solid {
    StepImporter::import_solid_from_file(FIXTURE).expect("the capped sphere fixture must read")
}

/// 半球の縁を半径 `r` で丸めたとき、消える体積。
///
/// 球の中心を原点、縁を `z = 0` として、材料は `z <= 0`。
/// 転がる球の中心は `(a, -r)`、`a = sqrt(R² - 2Rr)`。
fn fillet_removed(r: f64) -> f64 {
    let a = (R_SPHERE * R_SPHERE - 2.0 * R_SPHERE * r).sqrt();
    let ring = R_SPHERE - r;
    let z_t = -r * R_SPHERE / ring;

    // 球の断面: 半径² = R² - z²
    let sphere = |z: f64| R_SPHERE * R_SPHERE * z - z * z * z / 3.0;
    // つなぎの断面: 半径 = a + sqrt(r² - (z + r)²)
    let blend = |z: f64| {
        let u = z + r;
        let root = (r * r - u * u).max(0.0).sqrt();
        (a * a + r * r) * u + a * (u * root + r * r * (u / r).clamp(-1.0, 1.0).asin())
            - u * u * u / 3.0
    };

    let original = sphere(0.0) - sphere(-R_SPHERE);
    let blended = (sphere(z_t) - sphere(-R_SPHERE)) + (blend(0.0) - blend(z_t));
    PI * (original - blended)
}

/// 同じく、距離 `d` の面取りで消える体積。
///
/// 蓋側は半径方向に `d`、球側は**子午線の弧長**で `d` 下がります。
fn chamfer_removed(d: f64) -> f64 {
    let plane_radius = R_SPHERE - d;
    let polar = std::f64::consts::FRAC_PI_2 + d / R_SPHERE;
    let sphere_radius = R_SPHERE * polar.sin();
    let sphere_height = R_SPHERE * polar.cos();

    let sphere = |z: f64| R_SPHERE * R_SPHERE * z - z * z * z / 3.0;
    let dz = -sphere_height;
    let slope = (plane_radius - sphere_radius) / dz;
    let cone = |t: f64| {
        sphere_radius * sphere_radius * t
            + sphere_radius * slope * t * t
            + slope * slope * t * t * t / 3.0
    };

    let original = sphere(0.0) - sphere(-R_SPHERE);
    let blended = (sphere(sphere_height) - sphere(-R_SPHERE)) + (cone(-sphere_height) - cone(0.0));
    PI * (original - blended)
}

/// 縁が「丸められる稜」として見えること。
///
/// **ここが 0 本だと、上の階（選択 UI や自動フィレット）はこの縁を一生
/// 触れません。** `blend_coverage_probe` が数えているのもここです。
#[test]
fn the_circular_rim_is_reported_as_blendable() {
    let solid = hemisphere();
    let edges = EdgeBlender::blendable_edges(&solid);
    assert!(
        !edges.is_empty(),
        "the capped sphere's rim must be blendable; got none"
    );
    for edge in &edges {
        // 半球の縁は、蓋と球の接平面が直交します。
        assert!(
            (edge.dihedral_angle_deg - 90.0).abs() < 1e-6,
            "the rim of a hemisphere is a right angle, got {}",
            edge.dihedral_angle_deg
        );
        // 縁は半径 R の真円。
        assert!(
            (edge.length - 2.0 * PI * R_SPHERE).abs() < 1e-6,
            "the rim should be a full circle of radius {R_SPHERE}, got length {}",
            edge.length
        );
        // 転がる球が反対側へ抜ける手前が上限。
        assert!(
            (edge.max_fillet_radius - R_SPHERE * 0.5 * 0.999).abs() < 1e-6,
            "the fillet limit here is R/2, got {}",
            edge.max_fillet_radius
        );
    }
}

/// フィレットの体積が閉じた式に乗ること。
///
/// **半径を1つだけ試さないでください。** 4-32 で、寸法を1組しか見ていな
/// かったせいで 64組中7組の失敗が隠れていました。
#[test]
fn filleting_the_rim_matches_the_closed_form() {
    let solid = hemisphere();
    let before = volume(&solid);
    assert!(
        (before - 2.0 / 3.0 * PI * R_SPHERE.powi(3)).abs() < 1e-6,
        "the fixture should be a hemisphere, got {before}"
    );
    let edges = EdgeBlender::blendable_edges(&solid);
    let edge_id = edges[0].edge_id;

    for radius in [0.5_f64, 1.0, 2.0, 3.0, 4.0, 4.5] {
        let (out, report) = EdgeBlender::blend_edge(&solid, edge_id, BlendKind::Fillet { radius })
            .unwrap_or_else(|err| panic!("fillet {radius} was refused: {err}"));

        let removed = before - volume(&out);
        let want = fillet_removed(radius);
        assert!(
            (removed - want).abs() <= want.abs() * 1e-9,
            "fillet {radius}: removed {removed}, the closed form says {want}"
        );
        // 予告と実測が合っていること。予告だけ合っていても意味がありません。
        assert!(
            (report.predicted_removed_volume - want).abs() <= want.abs() * 1e-9,
            "fillet {radius}: the report predicted {}, the closed form says {want}",
            report.predicted_removed_volume
        );
        assert!(
            out.outer_shell.validate_closed(&Default::default()).is_valid(),
            "fillet {radius} produced a shell that does not close"
        );
    }
}

/// 面取りも同じ物差しで。
#[test]
fn chamfering_the_rim_matches_the_closed_form() {
    let solid = hemisphere();
    let before = volume(&solid);
    let edge_id = EdgeBlender::blendable_edges(&solid)[0].edge_id;

    for distance in [0.5_f64, 1.0, 2.0, 4.0, 6.0] {
        let (out, report) =
            EdgeBlender::blend_edge(&solid, edge_id, BlendKind::Chamfer { distance })
                .unwrap_or_else(|err| panic!("chamfer {distance} was refused: {err}"));

        let removed = before - volume(&out);
        let want = chamfer_removed(distance);
        assert!(
            (removed - want).abs() <= want.abs() * 1e-9,
            "chamfer {distance}: removed {removed}, the closed form says {want}"
        );
        assert!(
            (report.predicted_removed_volume - want).abs() <= want.abs() * 1e-9,
            "chamfer {distance}: the report predicted {}, the closed form says {want}",
            report.predicted_removed_volume
        );
    }
}

/// 上限を超えたら、**もっともらしい立体を返さずに断る**こと。
///
/// `r = R/2` で転がる球は球面の反対側へ抜けます。ここを黙って通すと、
/// 裏返った立体が出ます。
#[test]
fn a_fillet_that_runs_off_the_sphere_is_refused() {
    let solid = hemisphere();
    let edge_id = EdgeBlender::blendable_edges(&solid)[0].edge_id;

    for radius in [R_SPHERE * 0.5, R_SPHERE * 0.75, R_SPHERE] {
        assert!(
            EdgeBlender::blend_edge(&solid, edge_id, BlendKind::Fillet { radius }).is_err(),
            "a fillet of {radius} runs off the sphere and must be refused"
        );
    }
    // 蓋を食い切る面取りも同じ。
    for distance in [R_SPHERE, R_SPHERE * 1.5] {
        assert!(
            EdgeBlender::blend_edge(&solid, edge_id, BlendKind::Chamfer { distance }).is_err(),
            "a chamfer of {distance} eats the whole cap and must be refused"
        );
    }
}
