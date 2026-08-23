//! 回転面（`SURFACE_OF_REVOLUTION`）を読めること。
//!
//! **既存の10検体には1つも入っていませんでした。** インポーターは対応して
//! おらず、「読める曲面」の一覧にも挙げていません。回転体は旋盤で挽く部品でも
//! パイプでも普通に出てくる形です。
//!
//! 検体は OpenCASCADE に書かせた挽き物です
//! （`tools/occ_reference_shapes.py` の `revolved_vase`）。スプラインの母線を
//! Z 軸まわりに一周させたもので、解析曲面には落ちません。
//!
//! # 期待値は OpenCASCADE から取っていません
//!
//! **OCC の立体の求積は、この形で 1.3e-5 外れます。** 母線からグリーンの
//! 定理で直接積分すると（`tools/revolved_volume_reference.py`）:
//!
//! ```text
//!   Green の定理    4171.053368   刻みを 200 から 200000 まで振って収束
//!   Zenith 読み値   4171.053368   分割 32 以上で 10 桁動かない
//!   OCC 立体        4170.999302
//! ```
//!
//! 相手の値をそのまま期待値にすると、**相手の誤差を仕様として焼き付ける**
//! ことになります。有理 B-spline の上での OCC の求積が緩いことは 4-45 でも
//! 見ています。

use zenith_algo::MassCalculator;
use zenith_math::Tolerance;
use zenith_tess::TessellationParams;
use zenith_topo::Solid;

fn params(divisions: usize) -> TessellationParams {
    TessellationParams {
        u_divisions: divisions,
        v_divisions: divisions,
    }
}

fn vase() -> Solid {
    let path = std::path::PathBuf::from(concat!(
        env!("CARGO_MANIFEST_DIR"),
        "/tests/fixtures/occ_reference_revolved_vase.step"
    ));
    zenith_io::StepImporter::import_solids_from_file(&path)
        .expect("a surface of revolution must be readable")
        .into_iter()
        .next()
        .expect("one solid")
}

/// 母線から直接積分した体積。
const ANALYTIC_VOLUME: f64 = 4171.053368;

#[test]
fn the_turned_profile_is_read_at_all() {
    let solid = vase();
    let faces = solid.outer_shell.faces.len();
    assert_eq!(faces, 3, "the vase has a side, a top and a bottom");
}

#[test]
fn the_volume_lands_on_the_closed_form() {
    let volume = MassCalculator::compute_from_brep(&vase(), &params(64)).volume;
    let relative = (volume - ANALYTIC_VOLUME).abs() / ANALYTIC_VOLUME;
    assert!(
        relative <= 1e-8,
        "volume {volume} against the closed form {ANALYTIC_VOLUME} (relative {relative:.3e})"
    );
}

/// **1つの刻みで合わせない。** 通る値を1つ選んだだけでは、合っているのが
/// 形なのか刻みなのか分かれません（第5章）。
#[test]
fn the_volume_does_not_move_with_the_step() {
    let solid = vase();
    let mut previous: Option<f64> = None;
    for divisions in [32usize, 64, 128] {
        let volume = MassCalculator::compute_from_brep(&solid, &params(divisions)).volume;
        let relative = (volume - ANALYTIC_VOLUME).abs() / ANALYTIC_VOLUME;
        assert!(
            relative <= 1e-8,
            "at {divisions} divisions the volume is {volume} (relative {relative:.3e})"
        );
        if let Some(before) = previous {
            assert!(
                (volume - before).abs() / ANALYTIC_VOLUME <= 1e-9,
                "the volume moved between steps: {before} then {volume}"
            );
        }
        previous = Some(volume);
    }
}

#[test]
fn the_shell_closes() {
    let tol = Tolerance::default();
    assert!(
        vase().outer_shell.validate_closed(&tol).is_valid(),
        "the outer shell of the vase does not close"
    );
}
