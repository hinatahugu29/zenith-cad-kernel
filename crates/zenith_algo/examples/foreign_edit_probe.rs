//! 他カーネルが書いた立体を、編集する。
//!
//! **ここは一度も測っていません。** フィレット・面取り・プッシュプルの検査は
//! すべて自前ビルダーの立体に対するもので、読んだ立体に掛けたことはありません。
//!
//! 読んだ立体は持ち方が違います——全周1枚の面、境界ワイヤを持たない面、
//! 3辺の境界、稜の刻み方。ブーリアンでも距離でも断面でも、**そこに欠陥が
//! 隠れていました**（4-68 〜 4-71）。編集操作も同じ経路を通ります。
//!
//! 測るものは3つです。
//!
//! 1. **稜の列挙。** `blendable_edges` が何本返し、二面角をどう見るか。
//! 2. **フィレットと面取り。** 掛けた結果が閉じた立体か、体積が減ったか、
//!    そして**直線の稜で二面角が 90 度なら、減った量は閉じた式で決まる**。
//!    半径 $r$ のフィレットは $(1 - \pi/4) r^2 L$、距離 $c$ の面取りは
//!    $c^2 L / 2$ を削ります。
//! 3. **プッシュプル。** 平らな面を法線方向に $d$ 動かすと、側面が法線に
//!    平行な立体では体積が **面積 × d** だけ変わります。円柱の蓋がそれです。
//!
//! 掛からないこと自体は、ここでは欠陥として数えません（実装していない組み
//! 合わせがあります）。**数えるのは「掛かったのに答えが違う」ほう**です。
//! 壊れた立体を返す、体積が増える、閉じた式から外れる——それが赤です。
//!
//! ```bash
//! cargo run --release -p zenith_algo --example foreign_edit_probe
//! ```

use std::path::PathBuf;

use zenith_algo::{DirectModeling, EdgeBlender, MassCalculator};
use zenith_io::StepImporter;
use zenith_math::Tolerance;
use zenith_tess::TessellationParams;
use zenith_topo::Solid;

fn params() -> TessellationParams {
    TessellationParams {
        u_divisions: 64,
        v_divisions: 64,
    }
}

fn volume(solid: &Solid) -> f64 {
    MassCalculator::compute_from_brep(solid, &params()).volume
}

fn read(name: &str) -> Option<Solid> {
    let path = PathBuf::from(concat!(env!("CARGO_MANIFEST_DIR"), "/tests/fixtures"))
        .join(format!("occ_reference_{name}.step"));
    StepImporter::import_solids_from_file(&path)
        .ok()?
        .into_iter()
        .next()
}

fn closed(solid: &Solid, tol: &Tolerance) -> bool {
    std::iter::once(&solid.outer_shell)
        .chain(solid.inner_shells.iter())
        .all(|shell| shell.validate_closed(tol).is_valid())
}

/// この稜は直線で、隣り合う2面のなす角が 90 度か。そうなら削れる量が
/// 閉じた式で決まる。
fn right_angled_straight(
    solid: &Solid,
    edge_id: u64,
    dihedral_deg: f64,
    length: f64,
) -> Option<f64> {
    if (dihedral_deg - 90.0).abs() > 1e-6 {
        return None;
    }
    // 稜が直線かは、中点が端点を結ぶ線分の上にあるかで見る。
    for shell in std::iter::once(&solid.outer_shell).chain(solid.inner_shells.iter()) {
        for face in &shell.faces {
            for wire in std::iter::once(&face.outer_wire).chain(face.inner_wires.iter()) {
                for oriented in &wire.edges {
                    if oriented.edge.id != edge_id {
                        continue;
                    }
                    let start = oriented.edge.start_vertex.point;
                    let end = oriented.edge.end_vertex.point;
                    let (t0, t1) = oriented.edge.curve.param_range();
                    let middle = oriented.edge.curve.evaluate((t0 + t1) * 0.5);
                    let chord = end - start;
                    let expected = start + chord * 0.5;
                    if (middle - expected).norm() > 1e-9 * chord.norm().max(1.0) {
                        return None;
                    }
                    return Some(length);
                }
            }
        }
    }
    None
}

/// Removed volume for a fillet on the selected planar cap of a pure cone.
/// The lower frustum and the circular upper profile are integrated separately;
/// this does not call the implementation under test.
fn conical_rim_removed(
    opposite_radius: f64,
    selected_radius: f64,
    height: f64,
    fillet: f64,
) -> f64 {
    let slope = (selected_radius - opposite_radius) / height;
    let norm = slope.hypot(1.0);
    let centre_radius = selected_radius - fillet * (norm + slope);
    let centre_z = height - fillet;
    let side_radius = centre_radius + fillet / norm;
    let side_z = centre_z - fillet * slope / norm;
    let side_angle = (-slope).atan();
    let lower = std::f64::consts::PI
        * side_z
        * (opposite_radius * opposite_radius
            + opposite_radius * side_radius
            + side_radius * side_radius)
        / 3.0;
    let primitive = |angle: f64| {
        let sine = angle.sin();
        let cosine = angle.cos();
        fillet * centre_radius * centre_radius * sine
            + fillet * fillet * centre_radius * (angle + sine * cosine)
            + fillet.powi(3) * (sine - sine.powi(3) / 3.0)
    };
    let upper =
        std::f64::consts::PI * (primitive(std::f64::consts::FRAC_PI_2) - primitive(side_angle));
    let original = std::f64::consts::PI
        * height
        * (opposite_radius * opposite_radius
            + opposite_radius * selected_radius
            + selected_radius * selected_radius)
        / 3.0;
    original - lower - upper
}

fn main() {
    let tol = Tolerance::default();
    let mut failures = 0usize;
    let mut wrong = 0usize;
    let mut applied = 0usize;
    let mut refused = 0usize;
    let mut exact_checks = 0usize;
    let mut worst: f64 = 0.0;

    let subjects = [
        "cone",
        "cone_full",
        "cylinder",
        "elliptic_prism",
        "extruded_spline",
        "revolved_ring",
        "sphere",
        "sphere_capped",
        "torus",
        "torus_segment",
        "chamfered_box",
        "hollow_box",
        "stepped_shaft",
        "filleted_box",
        "plate_with_holes",
    ];

    println!("他カーネルが書いた立体を編集する");
    println!();
    println!(
        "{:<18} {:>6} {:>7} {:>14} {:>10} {:>10} {:>11}  {}",
        "fixture", "edges", "op", "volume before", "after", "removed", "closed form", "verdict"
    );
    println!("{}", "-".repeat(112));

    for name in subjects {
        let Some(solid) = read(name) else {
            continue;
        };
        let before = volume(&solid);
        let edges = EdgeBlender::blendable_edges(&solid);

        if edges.is_empty() {
            println!(
                "{name:<18} {:>6} {:>7}  丸められる稜がありません（曲面だけの立体、または列挙が0本）",
                0, "-"
            );
            continue;
        }

        // 稜は1本だけ選びます。**上限の 1/4** にしておけば、隣の稜を
        // 食い切る心配はありません。
        let target = if name == "plate_with_holes" {
            edges
                .iter()
                .filter(|edge| edge.max_chamfer_distance == 0.0)
                .max_by(|a, b| a.length.partial_cmp(&b.length).unwrap())
                .expect("the imported multi-hole plate has circular mouths")
        } else {
            edges
                .iter()
                .max_by(|a, b| a.length.partial_cmp(&b.length).unwrap())
                .expect("non-empty")
        };

        for (op, size, expected_removed) in [
            (
                "fillet",
                target.max_fillet_radius * 0.25,
                if name == "cylinder" {
                    // OCC fixture is the independently known r10 × h40
                    // cylinder. A selected cap arc propagates around the full
                    // circular rim; the removed ring follows by integrating
                    // the quarter-circle profile around the axis.
                    let fillet = target.max_fillet_radius * 0.25;
                    let major = 10.0 - fillet;
                    Some(
                        std::f64::consts::PI
                            * (major * fillet * fillet * (2.0 - std::f64::consts::PI * 0.5)
                                + fillet.powi(3) / 3.0),
                    )
                } else if name == "cone" || name == "cone_full" {
                    let fillet = target.max_fillet_radius * 0.25;
                    Some(conical_rim_removed(
                        if name == "cone" { 4.0 } else { 0.0 },
                        10.0,
                        20.0,
                        fillet,
                    ))
                } else if name == "plate_with_holes" {
                    let fillet = target.max_fillet_radius * 0.25;
                    let hole = target.length / std::f64::consts::TAU;
                    Some(
                        std::f64::consts::PI
                            * (hole * fillet * fillet * (2.0 - std::f64::consts::PI * 0.5)
                                + fillet.powi(3) * (5.0 / 3.0 - std::f64::consts::PI * 0.5)),
                    )
                } else {
                    right_angled_straight(
                        &solid,
                        target.edge_id,
                        target.dihedral_angle_deg,
                        target.length,
                    )
                    .map(|length| {
                        (1.0 - std::f64::consts::FRAC_PI_4)
                            * (target.max_fillet_radius * 0.25).powi(2)
                            * length
                    })
                },
            ),
            (
                "chamfer",
                target.max_chamfer_distance * 0.25,
                if name == "cylinder" {
                    let distance = target.max_chamfer_distance * 0.25;
                    Some(std::f64::consts::PI * distance * distance * (10.0 - distance / 3.0))
                } else {
                    right_angled_straight(
                        &solid,
                        target.edge_id,
                        target.dihedral_angle_deg,
                        target.length,
                    )
                    .map(|length| (target.max_chamfer_distance * 0.25).powi(2) * length * 0.5)
                },
            ),
        ] {
            if !(size > 1e-9) {
                continue;
            }
            let result = if op == "fillet" {
                EdgeBlender::fillet_edge(&solid, target.edge_id, size)
            } else {
                EdgeBlender::chamfer_edge(&solid, target.edge_id, size)
            };

            let Ok(edited) = result else {
                refused += 1;
                println!(
                    "{:<18} {:>6} {:>7} {:>14.4} {:>10} {:>10} {:>11}  掛かりませんでした（欠陥として数えません）",
                    name, edges.len(), op, before, "-", "-", "-"
                );
                continue;
            };
            applied += 1;

            let after = volume(&edited);
            let removed = before - after;
            let still_closed = closed(&edited, &tol);

            // **掛かったのに答えが違うほうを数えます。**
            let mut verdict = String::new();
            let mut bad = false;
            if !still_closed {
                verdict.push_str("WRONG: 閉じていない立体を返した");
                bad = true;
            } else if removed <= 0.0 {
                verdict.push_str("WRONG: 体積が減っていない");
                bad = true;
            } else if let Some(want) = expected_removed {
                exact_checks += 1;
                let relative = (removed - want).abs() / want;
                worst = worst.max(relative);
                // 体積はメッシュ積分なので、曲面を含む立体では刻みぶんが乗る。
                if relative < 1e-3 {
                    verdict = format!("ok（閉じた式と {relative:.2e}）");
                } else {
                    verdict = format!("WRONG: 閉じた式から {relative:.2e}");
                    bad = true;
                }
            } else {
                verdict.push_str("ok（閉じた式なし。閉性と体積の向きだけ見た）");
            }
            if bad {
                wrong += 1;
                failures += 1;
            }

            println!(
                "{:<18} {:>6} {:>7} {:>14.4} {:>10.4} {:>10.6} {:>11}  {}",
                name,
                edges.len(),
                op,
                before,
                after,
                removed,
                expected_removed
                    .map(|w| format!("{w:.6}"))
                    .unwrap_or_else(|| "-".to_string()),
                verdict
            );
        }
    }

    // 面の計測。円柱の蓋は面積 100π、重心は軸の上、法線は軸に平行。
    println!();
    println!("面の計測（`inspect_face`。円柱 r10 h40 の蓋、相手は閉じた式）");
    println!(
        "{:<10} {:>15} {:>15} {:>11}  {:>26} {:>26}",
        "face", "area", "closed form", "rel error", "centroid", "normal"
    );
    if let Some(cylinder) = read("cylinder") {
        for (face_index, face) in cylinder.outer_shell.faces.iter().enumerate() {
            if !matches!(face.geometry, zenith_topo::FaceGeometry::Plane(_)) {
                continue;
            }
            let Ok(inspection) = DirectModeling::inspect_face(face) else {
                continue;
            };
            let want_area = 100.0 * std::f64::consts::PI;
            let relative = (inspection.area - want_area).abs() / want_area;
            worst = worst.max(relative);
            exact_checks += 1;
            // 重心は蓋の中心（軸の上、z は蓋の高さ）、法線は軸に平行。
            // **z も見ます。** 見ていなかったときは、上蓋の重心が (0,0,30)
            // ——真値 (0,0,40) の 3/4 ——でも通っていました。
            let want_z = if inspection.normal.z > 0.0 { 40.0 } else { 0.0 };
            let centroid_off =
                (inspection.centroid - zenith_math::Point3::new(0.0, 0.0, want_z)).norm();
            let normal_off = 1.0 - inspection.normal.z.abs();
            let ok = relative < 1e-9 && centroid_off < 1e-9 && normal_off < 1e-12;
            if !ok {
                wrong += 1;
                failures += 1;
            }
            println!(
                "{:<10} {:>15.9} {:>15.9} {:>11.2e}  ({:>7.4} {:>7.4} {:>7.4}) ({:>7.4} {:>7.4} {:>7.4})  {}",
                face_index,
                inspection.area,
                want_area,
                relative,
                inspection.centroid.x,
                inspection.centroid.y,
                inspection.centroid.z,
                inspection.normal.x,
                inspection.normal.y,
                inspection.normal.z,
                if ok {
                    "ok".to_string()
                } else {
                    format!(
                        "WRONG (area {relative:.2e}, centroid off {centroid_off:.2e}, normal off {normal_off:.2e})"
                    )
                }
            );
        }
    }

    // プッシュプル。円柱の蓋を動かすと、体積は 面積 x d だけ変わる。
    println!();
    println!("プッシュプル（平らな面を法線方向へ動かす）");
    println!(
        "{:<18} {:>6} {:>10} {:>14} {:>14} {:>11}  {}",
        "fixture", "face", "distance", "measured dV", "area x d", "rel error", "verdict"
    );
    if let Some(cylinder) = read("cylinder") {
        let before = volume(&cylinder);
        for face_index in 0..cylinder.outer_shell.faces.len() {
            let face = &cylinder.outer_shell.faces[face_index];
            if !matches!(face.geometry, zenith_topo::FaceGeometry::Plane(_)) {
                continue;
            }
            let Ok(inspection) = DirectModeling::inspect_face(face) else {
                continue;
            };
            for distance in [5.0f64, -3.0] {
                let Ok(moved) = DirectModeling::push_pull_face(&cylinder, face_index, distance)
                else {
                    refused += 1;
                    println!(
                        "{:<18} {:>6} {:>10.3}  掛かりませんでした（欠陥として数えません）",
                        "cylinder", face_index, distance
                    );
                    continue;
                };
                applied += 1;
                let measured = volume(&moved) - before;
                let want = inspection.area * distance;
                let relative = (measured - want).abs() / want.abs();
                worst = worst.max(relative);
                exact_checks += 1;
                let still_closed = closed(&moved, &tol);
                let ok = still_closed && relative < 1e-3;
                if !ok {
                    wrong += 1;
                    failures += 1;
                }
                println!(
                    "{:<18} {:>6} {:>10.3} {:>14.6} {:>14.6} {:>11.2e}  {}",
                    "cylinder",
                    face_index,
                    distance,
                    measured,
                    want,
                    relative,
                    if !still_closed {
                        "WRONG: 閉じていない立体を返した"
                    } else if ok {
                        "ok"
                    } else {
                        "WRONG: 面積 x d から外れた"
                    }
                );
            }
        }
    }

    println!();
    println!("{}", "-".repeat(112));
    println!(
        "{applied} operation(s) applied, {refused} refused (not graded), {exact_checks} against a closed form, {wrong} WRONG"
    );
    if worst > 0.0 {
        println!("worst relative error against a closed form: {worst:.2e}");
    }
    println!();
    println!("**掛からないことは、ここでは欠陥として数えません。** 実装していない");
    println!("組み合わせがあります。数えるのは「掛かったのに答えが違う」ほうです。");

    if failures > 0 {
        std::process::exit(1);
    }
}
