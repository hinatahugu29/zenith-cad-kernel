//! 掃引面と楕円を持つ、他カーネルが書いたファイルを読めるか。
//!
//! `SURFACE_OF_LINEAR_EXTRUSION` と `ELLIPSE` は、自由曲線や楕円の断面を
//! 押し出したときに OpenCASCADE が選ぶ形である（円や直線の押し出しは
//! `CYLINDRICAL_SURFACE` / `PLANE` に落ちるので、この2つは自前ビルダーの
//! 出力には決して現れない）。読めていなかったあいだ、前者は面が読めずエラー、
//! 後者は**端点を結ぶ直線に置き換わって**いた。楕円の弦は元の平面から外れない
//! ので、下流の p-curve 検証を素通りしうる形の誤りである。
//!
//! 検体は `tools/occ_reference_swept.py` が OpenCASCADE 7.8 に書かせたもの。
//! 期待値は、閉じた式を持つものは閉じた式で、持たないものは OpenCASCADE が
//! そのファイルを読み直して測った値で置いている。

use std::f64::consts::PI;

use zenith_algo::MassCalculator;
use zenith_io::StepImporter;
use zenith_tess::TessellationParams;
use zenith_topo::Solid;

fn read(name: &str) -> Solid {
    let text = match name {
        "revolved_ring" => include_str!("fixtures/occ_reference_revolved_ring.step"),
        "extruded_spline" => include_str!("fixtures/occ_reference_extruded_spline.step"),
        "elliptic_prism" => include_str!("fixtures/occ_reference_elliptic_prism.step"),
        other => panic!("unknown fixture {other}"),
    };
    StepImporter::import_solid_from_str(text)
        .unwrap_or_else(|error| panic!("{name} must import: {error}"))
}

fn volume(solid: &Solid, divisions: usize) -> f64 {
    MassCalculator::compute_from_brep(
        solid,
        &TessellationParams {
            u_divisions: divisions,
            v_divisions: divisions,
        },
    )
    .volume
}

/// 楕円断面の柱。体積は $\pi a b h$ で閉じている。
#[test]
fn test_an_elliptic_prism_lands_on_the_closed_form() {
    let solid = read("elliptic_prism");
    assert_eq!(solid.outer_shell.faces.len(), 3, "two caps and one swept side");

    // 許容は実測してから決めた。32分割で 4.5e-11、64分割と128分割で 7e-14 台。
    // 刻みを上げると落ちるので、残っているのは求積の粗さであって偏りではない。
    let expected = PI * 12.0 * 7.0 * 15.0;
    for (divisions, allowed) in [(32usize, 1e-10), (64, 1e-12), (128, 1e-12)] {
        let measured = volume(&solid, divisions);
        let error = (measured - expected).abs() / expected;
        assert!(
            error < allowed,
            "at {divisions} divisions the volume was {measured} against {expected} ({error:e})"
        );
    }
}

/// 掃引面の媒介変数の範囲は、境界がちょうど収まるところで止めること。
///
/// 一度、覆い漏らしを恐れて `v` を 2e-6 だけ広げた。この面の境界は
/// パラメータ矩形そのものなので、広げたぶんがそのまま積分に乗り、側面積が
/// 3.0e-6、体積が 1.3e-6 だけ大きく出た。しかも**分割数を振っても動かない**。
/// 平面キャップのほうは 1.5e-14 で厳密なままだったので、楕円そのものは
/// 正しく、経路のほうが違っていたと分かる。
#[test]
fn test_the_swept_side_is_not_larger_than_the_face() {
    let solid = read("elliptic_prism");
    let expected_cap = PI * 12.0 * 7.0;
    let expected_side = expected_cap * 2.0; // 高さ15、周長 ≈ 60.728 → 側面 ≈ 910.9

    let params = TessellationParams {
        u_divisions: 64,
        v_divisions: 64,
    };
    let mut caps = 0usize;
    for face in &solid.outer_shell.faces {
        let (area, _volume_share) = MassCalculator::compute_face_integral(face, &params);
        if (area - expected_cap).abs() / expected_cap < 1e-9 {
            caps += 1;
        } else {
            // 側面。厳密な閉じた式は楕円積分なので、桁だけ押さえる。
            assert!(
                area > expected_side * 1.5 && area < expected_side * 1.8,
                "the swept side measured {area}, which is not the lateral area of this prism"
            );
        }
    }
    assert_eq!(caps, 2, "both planar caps must land on pi*a*b");
}

/// スプライン断面の押し出し。閉じた式が無いので OpenCASCADE の読み値と突き合わせる。
#[test]
fn test_an_extruded_spline_agrees_with_the_kernel_that_wrote_it() {
    let solid = read("extruded_spline");
    assert_eq!(solid.outer_shell.faces.len(), 6);

    // OpenCASCADE 7.8 が、この同じファイルを読み直して測った値。
    let occ = 5220.435297;
    for divisions in [32usize, 64, 128] {
        let measured = volume(&solid, divisions);
        let error = (measured - occ).abs() / occ;
        assert!(
            error < 1e-9,
            "at {divisions} divisions the volume was {measured} against OpenCASCADE's {occ} ({error:e})"
        );
    }
}

/// 母線を回した輪。OCC は解析曲面に落とすので、掃引面は出てこない。
/// 読めていたことを固定するためだけに置いてある。
#[test]
fn test_a_revolved_profile_still_reads() {
    let solid = read("revolved_ring");
    let occ = 1583.362697;
    let measured = volume(&solid, 64);
    let error = (measured - occ).abs() / occ;
    assert!(
        error < 1e-8,
        "the revolved ring measured {measured} against OpenCASCADE's {occ} ({error:e})"
    );
}
