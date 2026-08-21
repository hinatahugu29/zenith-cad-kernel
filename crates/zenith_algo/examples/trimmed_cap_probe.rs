//! 他カーネルが B-spline に変換して書いた円柱の、キャップが内側に削れる件を測る。
//!
//! 解析曲面から読んだ6検体は体積が 2.4e-12 で乗るのに、`toNurbs` された円柱
//! だけが 1.38e-6 外れる。キャップの面積は 314.1512、真値は 314.1593 で
//! **必ず小さいほう**に出る。向きが決まっている誤差は、細かくしても消えない。
//!
//! ここで分けたいのは2つである。
//!
//! - 積分の粗さなら、分割数を上げれば真値へ寄る。
//! - 境界の取りこぼしなら、**分割数を上げても動かない**。トリムの輪郭が
//!   短く読まれている以上、その内側をいくら細かく積んでも足りない分は
//!   戻らない。
//!
//! 4-3 と 4-11 が同じ型を2度踏んでいる。どちらも「境界の折れを落とした」で、
//! どちらも面積が一方向に削れた。

use zenith_algo::MassCalculator;
use zenith_io::StepImporter;
use zenith_math::Tolerance;
use zenith_tess::TessellationParams;
use zenith_topo::{Face, FaceGeometry};

const EXACT_CAP_AREA: f64 = std::f64::consts::PI * 100.0;
const EXACT_VOLUME: f64 = std::f64::consts::PI * 100.0 * 40.0;

fn main() {
    let text = include_str!("../tests/fixtures/occ_reference_cylinder_nurbs.step");
    let solids = StepImporter::import_solids_from_str(text).expect("fixture should import");
    let solid = solids.into_iter().next().expect("one solid");
    let tol = Tolerance::default();

    println!("faces: {}", solid.outer_shell.faces.len());
    for (index, face) in solid.outer_shell.faces.iter().enumerate() {
        describe_face(index, face);
    }

    println!();
    println!("=== does refining move it? ===");
    println!(
        "{:>6}  {:>18} {:>12}  {:>18} {:>12}",
        "div", "cap area", "rel", "volume", "rel"
    );
    println!("(lat = 側面。真値 2 pi r h = {:.6})", 2.0 * std::f64::consts::PI * 10.0 * 40.0);
    for divisions in [16usize, 32, 64, 128, 256, 512] {
        let params = TessellationParams { u_divisions: divisions, v_divisions: divisions };
        let lateral = MassCalculator::compute_face_integral(&solid.outer_shell.faces[0], &params).0;
        let exact_lateral = 2.0 * std::f64::consts::PI * 10.0 * 40.0;
        println!(
            "  lat div {divisions:>4}: {lateral:>16.9}  rel {:>11.2e}",
            (lateral - exact_lateral) / exact_lateral
        );
    }
    for divisions in [16usize, 32, 64, 128, 256, 512] {
        let params = TessellationParams {
            u_divisions: divisions,
            v_divisions: divisions,
        };
        let cap_area = solid
            .outer_shell
            .faces
            .iter()
            .map(|face| MassCalculator::compute_face_integral(face, &params).0)
            .filter(|area| (area - EXACT_CAP_AREA).abs() / EXACT_CAP_AREA < 0.2)
            .fold(0.0f64, f64::max);
        let volume = MassCalculator::compute_from_brep(&solid, &params).volume;
        println!(
            "{divisions:>6}  {cap_area:>18.9} {:>12.2e}  {volume:>18.9} {:>12.2e}",
            (cap_area - EXACT_CAP_AREA) / EXACT_CAP_AREA,
            (volume - EXACT_VOLUME) / EXACT_VOLUME
        );
    }

    println!();
    println!("=== is the trim boundary itself short? ===");
    println!("キャップの輪郭を3Dで辿った長さ。真値は 2 pi r = {:.9}", 2.0 * std::f64::consts::PI * 10.0);
    for (index, face) in solid.outer_shell.faces.iter().enumerate() {
        if !matches!(face.geometry, FaceGeometry::Nurbs(_)) {
            continue;
        }
        let area = MassCalculator::compute_face_integral(
            face,
            &TessellationParams {
                u_divisions: 64,
                v_divisions: 64,
            },
        )
        .0;
        if (area - EXACT_CAP_AREA).abs() / EXACT_CAP_AREA >= 0.2 {
            continue;
        }
        for (wire_index, edge) in face.outer_wire.edges.iter().enumerate() {
            let samples = 4096;
            let mut length = 0.0;
            let mut previous = edge.edge.curve.evaluate(edge.edge.curve.param_range().0);
            let (start, end) = edge.edge.curve.param_range();
            for step in 1..=samples {
                let t = start + (end - start) * (step as f64) / (samples as f64);
                let point = edge.edge.curve.evaluate(t);
                length += (point - previous).norm();
                previous = point;
            }
            println!(
                "  face {index} edge {wire_index}: degree {}, {} control points, chord length {length:.9}",
                edge.edge.curve.degree,
                edge.edge.curve.control_points.len()
            );
        }
    }

    let report = solid.outer_shell.validate_closed(&tol);
    println!();
    println!("shell closed: {}", report.is_valid());
}

fn describe_face(index: usize, face: &Face) {
    let params = TessellationParams {
        u_divisions: 64,
        v_divisions: 64,
    };
    let (area, _) = MassCalculator::compute_face_integral(face, &params);
    let kind = match &face.geometry {
        FaceGeometry::Plane(_) => "plane".to_string(),
        FaceGeometry::Nurbs(surface) => format!(
            "nurbs deg {}x{}, {}x{} control points",
            surface.degree_u,
            surface.degree_v,
            surface.control_points.len(),
            surface.control_points[0].len()
        ),
        other => format!("{other:?}").chars().take(24).collect(),
    };
    println!(
        "  face {index}: area {area:>14.6}  edges {}  stored pcurves {}  {kind}",
        face.outer_wire.edges.len(),
        face.pcurves.is_some()
    );
}
