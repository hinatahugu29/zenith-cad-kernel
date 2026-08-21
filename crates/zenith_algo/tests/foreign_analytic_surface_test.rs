//! Reading analytic surfaces out of files another kernel wrote.
//!
//! STEP states a cone, a sphere or a torus as an unbounded surface plus a
//! radius; the face's own boundary is the only thing that says which piece is
//! in use. The reader used to build a fixed-size patch instead, so the boundary
//! sat off the surface and every one of these files was refused. Nothing inside
//! this kernel could show that, because our own exporter writes these shapes as
//! B-splines and never takes the analytic path.
//!
//! The fixtures were written by OpenCASCADE 7.8 (`tools/occ_reference_export.py`),
//! so a disagreement here is a disagreement with another kernel rather than with
//! a number this repository chose.
//!
//! 期待値は OpenCASCADE の出力ではなく**閉じた式**で書く。以前はここに OCC が
//! 印字した4桁の数（`3267.2564` など）を写していた。その literal は自分自身が
//! 3e-8 の粗さを持つので、許容をそれより締められない。実際の許容は 1e-3 に
//! 置かれていて、5桁の劣化を素通りさせる幅だった。これらの形はすべて閉じた式を
//! 持つのだから、外れようのないものを比較相手に置く。

use zenith_algo::MassCalculator;
use zenith_io::{StepExporter, StepImporter};
use zenith_math::Tolerance;
use zenith_tess::TessellationParams;
use zenith_topo::Solid;

fn volume(solid: &Solid) -> f64 {
    MassCalculator::compute_from_brep(
        solid,
        &TessellationParams {
            u_divisions: 64,
            v_divisions: 64,
        },
    )
    .volume
}

fn read_fixture(name: &str) -> Solid {
    let text = match name {
        "cone" => include_str!("fixtures/occ_reference_cone.step"),
        "cone_full" => include_str!("fixtures/occ_reference_cone_full.step"),
        "sphere_capped" => include_str!("fixtures/occ_reference_sphere_capped.step"),
        "torus_segment" => include_str!("fixtures/occ_reference_torus_segment.step"),
        "torus" => include_str!("fixtures/occ_reference_torus.step"),
        "sphere" => include_str!("fixtures/occ_reference_sphere.step"),
        "cylinder_nurbs" => include_str!("fixtures/occ_reference_cylinder_nurbs.step"),
        other => panic!("no fixture named {other}"),
    };

    let solids = StepImporter::import_solids_from_str(text)
        .unwrap_or_else(|err| panic!("{name} should import: {err}"));
    assert_eq!(solids.len(), 1, "{name} should hold one solid");

    let solid = solids.into_iter().next().unwrap();
    let report = solid.outer_shell.validate_closed(&Tolerance::default());
    assert!(
        report.is_valid(),
        "{name} should close: {:?}",
        report.errors.first()
    );
    solid
}

fn assert_volume(name: &str, expected: f64, allowed: f64) {
    let measured = volume(&read_fixture(name));
    let relative = (measured - expected).abs() / expected.abs();
    assert!(
        relative < allowed,
        "{name}: read {measured:.9}, closed form {expected:.9} (relative {relative:.2e}, allowed {allowed:.1e})"
    );
}

const PI: f64 = std::f64::consts::PI;

/// 円錐台の体積 `pi h (R^2 + R r + r^2) / 3`。
fn frustum_volume(big: f64, small: f64, height: f64) -> f64 {
    PI * height * (big * big + big * small + small * small) / 3.0
}

#[test]
fn test_a_conical_face_is_sized_from_its_boundary() {
    // Part.makeCone(10, 4, 20)
    // Part.makeCone(10, 4, 20): pi h (R^2 + R r + r^2) / 3 = 1040 pi
    assert_volume("cone", frustum_volume(10.0, 4.0, 20.0), 1e-11);
}

#[test]
fn test_a_conical_face_running_to_the_apex_is_readable() {
    // Part.makeCone(10, 0, 20). The apex end has zero radius, which is a
    // degenerate row rather than a reason to refuse the face.
    // 頂点まで走る円錐: pi r^2 h / 3
    assert_volume("cone_full", PI * 100.0 * 20.0 / 3.0, 1e-11);
}

#[test]
fn test_a_spherical_face_bounded_by_real_edges_is_readable() {
    // A sphere of radius 10 cut in half. The spherical face's loop walks its
    // seam meridian up and back down again before going round the equator, so
    // one edge is used twice by the one face.
    // 半球: (2/3) pi r^3
    assert_volume("sphere_capped", 2.0 / 3.0 * PI * 1000.0, 1e-11);
}

#[test]
fn test_a_toroidal_face_is_sized_from_its_boundary() {
    // A quarter of a torus, R=12 r=4: the elbow shape a pipe run is made of.
    // 四半トーラス: 2 pi^2 R r^2 / 4
    assert_volume("torus_segment", 2.0 * PI * PI * 12.0 * 16.0 / 4.0, 1e-11);
}

#[test]
fn test_a_torus_written_as_one_face_is_readable() {
    // OpenCASCADE writes a whole torus as a single face whose bound is nothing
    // but seam: two circles, each walked once each way. Such a loop covers the
    // whole parameter domain, but its p-curves cannot say so, because a point
    // on the seam maps to both ends of the domain. Read from the p-curves the
    // face came out at exactly half the surface, and so did the volume.
    // 全周トーラス: 2 pi^2 R r^2
    assert_volume("torus", 2.0 * PI * PI * 12.0 * 16.0, 1e-11);

    let solid = read_fixture("torus");
    assert_eq!(solid.outer_shell.faces.len(), 1);
    let area = MassCalculator::compute_face_integral(
        &solid.outer_shell.faces[0],
        &TessellationParams {
            u_divisions: 64,
            v_divisions: 64,
        },
    )
    .0;
    // 4 pi^2 R r
    let expected = 4.0 * std::f64::consts::PI * std::f64::consts::PI * 12.0 * 4.0;
    assert!(
        (area - expected).abs() / expected < 1e-11,
        "torus surface area {area:.4}, closed form {expected:.4}"
    );
}

#[test]
fn test_a_sphere_written_as_one_face_with_no_boundary_is_readable() {
    // OpenCASCADE writes a whole sphere as one face bounded by a VERTEX_LOOP:
    // a single point at the south pole and no edges at all. That is not a loop
    // missing its edges, it is a face with nothing to trim away, and the point
    // is the only thing saying where on the sphere the face sits.
    // 球: (4/3) pi r^3
    assert_volume("sphere", 4.0 / 3.0 * PI * 1000.0, 1e-11);

    let solid = read_fixture("sphere");
    assert_eq!(solid.outer_shell.faces.len(), 1);
    assert!(solid.outer_shell.faces[0].outer_wire.edges.is_empty());

    let area = MassCalculator::compute_face_integral(
        &solid.outer_shell.faces[0],
        &TessellationParams {
            u_divisions: 64,
            v_divisions: 64,
        },
    )
    .0;
    // 4 pi r^2
    let expected = 4.0 * std::f64::consts::PI * 100.0;
    assert!(
        (area - expected).abs() / expected < 1e-11,
        "sphere surface area {area:.4}, closed form {expected:.4}"
    );
}

#[test]
fn test_a_solid_converted_to_b_splines_keeps_its_size() {
    // OpenCASCADE の円柱を toNurbs したもの。解析曲面は一つも無く、平面の
    // キャップまで1次のパッチになっている。曲線でトリムされた B-spline 面が
    // どこまで正しく読めるかが、そのまま出る。
    //
    // かつてはキャップが八角形として通り、面積 282.47（正しくは 314.16）、
    // 体積 12144.19（正しくは 12566.37）だった。原因は二つ重なっていて、
    // p-curve が折れ線近似だったことと、テッセレータがトリムループを
    // 分割数に比例した粗さでしか折らなかったこと。
    //
    // そのあとも 314.1512 で止まっていた。厳密な平面の線積分は既にあったのに、
    // この面は種別が Nurbs なので通っていなかった。いまは幾何を測って
    // アフィンだと分かればそこへ通す。
    let solid = read_fixture("cylinder_nurbs");

    // キャップは真の円であって内接多角形ではない。ここは分割数に依存しない。
    let exact_cap = std::f64::consts::PI * 100.0;
    let mut caps = 0;
    for divisions in [16usize, 64, 256] {
        let params = TessellationParams {
            u_divisions: divisions,
            v_divisions: divisions,
        };
        caps = 0;
        for face in &solid.outer_shell.faces {
            let area = MassCalculator::compute_face_integral(face, &params).0;
            if (area - exact_cap).abs() / exact_cap < 1e-3 {
                caps += 1;
                let relative = (area - exact_cap).abs() / exact_cap;
                assert!(
                    relative < 1e-10,
                    "cap area {area:.9} against {exact_cap:.9} at {divisions} divisions                      (relative {relative:.2e}); a chorded trim boundary reads short and                      does not move with the division count"
                );
            }
        }
        assert_eq!(caps, 2, "both caps should come out at pi r^2");
    }
    assert_eq!(caps, 2);

    // 残るのは側面の求積だけで、こちらは偏りではないので分割数で落ちる。
    // **値の小ささではなく、落ち方を見ます。** かつてこの検体は 64分割で
    // 1.38e-6 と、512分割の 8.43e-6 より「良い」値を出していました。キャップの
    // 不足（負)と側面の求積誤差（正）が、その分割数でたまたま打ち消し合った
    // からです。小さいほうを引用すると、偏りが消えたときに「悪化」に見えます。
    let exact = std::f64::consts::PI * 100.0 * 40.0;
    let error_at = |divisions: usize| {
        let params = TessellationParams {
            u_divisions: divisions,
            v_divisions: divisions,
        };
        (MassCalculator::compute_from_brep(&solid, &params).volume - exact).abs() / exact
    };

    let coarse = error_at(64);
    let fine = error_at(512);
    assert!(
        fine < 3e-7,
        "converted cylinder volume is off by {fine:.2e} at 512 divisions"
    );
    // 分割数を8倍にすれば、2次収束なら 64分の1 になる。偏りが残っていると
    // 頭打ちになるので、比そのものを見る。
    assert!(
        coarse / fine > 30.0,
        "the error should fall with the division count (64 div {coarse:.2e},          512 div {fine:.2e}, ratio {:.1}); a ratio near 1 means a bias that          refining cannot reach",
        coarse / fine
    );
}

#[test]
fn test_what_we_read_from_another_kernel_we_can_write_and_read_again() {
    // 他カーネル → 自前リーダー → 自前ライター → 自前リーダー、の一周。
    // 半球はここで弾かれていた。境界点が曲面から 1.827273 外れている、と。
    // ファイルの中身は元と一致していて、違ったのは最近傍点の探索のほうだった。
    // 継ぎ目の向こう側に答えがある点で、領域の端に阻まれて回り込めず、
    // 継ぎ目そのものを最近傍として返していた。半径10の球で 1.83。
    let subjects = [
        "cone",
        "cone_full",
        "sphere",
        "sphere_capped",
        "torus",
        "torus_segment",
    ];

    for name in subjects {
        let solid = read_fixture(name);
        let before = volume(&solid);
        let step = StepExporter::export_solid_to_string(&solid, name);
        let read_back = StepImporter::import_solid_from_str(&step)
            .unwrap_or_else(|err| panic!("{name} should survive being written out: {err}"));
        let after = volume(&read_back);
        let relative = (after - before).abs() / before.abs();
        assert!(
            relative < 1e-9,
            "{name} round trip {before:.6} -> {after:.6} (relative {relative:.2e})"
        );
    }
}

#[test]
fn test_the_analytic_faces_carry_their_analytic_area() {
    // Volume alone can be right while a face is the wrong size, so the areas
    // are checked against the closed forms as well.
    let params = TessellationParams {
        u_divisions: 64,
        v_divisions: 64,
    };
    let subjects: [(&str, f64); 3] = [
        // Lateral surface of a frustum: pi (r1 + r2) * slant.
        (
            "cone",
            std::f64::consts::PI * 14.0 * (20.0f64 * 20.0 + 6.0 * 6.0).sqrt(),
        ),
        // Half a sphere of radius 10: 2 pi r^2.
        ("sphere_capped", 2.0 * std::f64::consts::PI * 100.0),
        // A quarter of a torus: (2 pi R)(2 pi r) / 4.
        (
            "torus_segment",
            std::f64::consts::PI * std::f64::consts::PI * 12.0 * 4.0,
        ),
    ];

    for (name, expected) in subjects {
        let solid = read_fixture(name);
        // The analytic face is the one carrying the most area.
        let largest = solid
            .outer_shell
            .faces
            .iter()
            .map(|face| MassCalculator::compute_face_integral(face, &params).0)
            .fold(0.0f64, f64::max);
        let relative = (largest - expected).abs() / expected;
        assert!(
            relative < 1e-11,
            "{name}: analytic face area {largest:.4}, closed form {expected:.4} (relative {relative:.2e})"
        );
    }
}
