//! **小さい模型でズレる体積を、面ごとに名指しする**（9-H の H6。4-234）。
//!
//! # なぜ要るのか
//!
//! 4-233 で「崩れているのはブーリアンそのもの」まで来ました。**その中の
//! どこか**を、推測ではなく数字で決めます。
//!
//! # どう測るか
//!
//! 4-232 で B-Rep を縮められるようになったので、**同じ立体を2通りに**用意
//! できます。
//!
//! - 桁 1 で切ってから `scale_solid` で縮める（**正しいほう**。桁 1 では
//!   恒等式が 4.758e-13 で閉じています）
//! - 桁 0.01 で切る（**崩れるほう**）
//!
//! **どちらも同じ立体のはず**です。面ごとに**体積への寄与**（発散定理の
//! `x·n` の積分）を出し、いちばん近い面どうしで突き合わせます。**寄与の差が
//! 大きい面が、誤差を運んでいる面**です。
//!
//! 面積ではなく寄与で見るのは、**面がどこにあるかが効く**からです（4-225 で
//! 「面積のずれより体積のずれが 64 倍大きい」と測っています）。
use zenith_algo::{
    BooleanEngine, BooleanOpType, BrepTransform, MassCalculator, PrimitiveBuilder,
};
use zenith_math::{Point3, Tolerance, Vec3};
use zenith_tess::TessellationParams;
use zenith_topo::{Face, Shell, Solid};

fn params() -> TessellationParams {
    TessellationParams {
        u_divisions: 32,
        v_divisions: 32,
    }
}

/// 1枚の面が体積に入れる寄与と、その面の重心のめやす。
///
/// **閉じていない殻でも積分は回ります**（発散定理の面ごとの項です）。立体と
/// して閉じている必要はありません。
fn face_contribution(face: &Face, solid_id: u64) -> (f64, Point3) {
    let one = Solid {
        id: solid_id,
        outer_shell: Shell {
            id: 0,
            faces: vec![face.clone()],
            is_closed: false,
        },
        inner_shells: Vec::new(),
    };
    let properties = MassCalculator::compute_from_brep(&one, &params());
    let bbox = face.bounding_box();
    let centre = Point3::new(
        (bbox.min.x + bbox.max.x) * 0.5,
        (bbox.min.y + bbox.max.y) * 0.5,
        (bbox.min.z + bbox.max.z) * 0.5,
    );
    (properties.volume, centre)
}

fn faces_of(solid: &Solid) -> Vec<&Face> {
    std::iter::once(&solid.outer_shell)
        .chain(solid.inner_shells.iter())
        .flat_map(|shell| shell.faces.iter())
        .collect()
}

fn build(s: f64) -> (Solid, Solid) {
    let torus = PrimitiveBuilder::make_torus(12.0 * s, 4.0 * s).expect("torus");
    let rod = BrepTransform::translate_solid(
        &PrimitiveBuilder::make_cylinder(9.0 * s, 40.0 * s).expect("cylinder"),
        Vec3::new(0.0, 0.0, -20.0 * s),
    );
    (torus, rod)
}

fn main() {
    let tol = Tolerance::default();
    let small = 0.01_f64;

    for (label, op) in [
        ("intersection", BooleanOpType::Intersection),
        ("difference", BooleanOpType::Difference),
    ] {
        println!("=== {label} ===");

        let (a1, b1) = build(1.0);
        let Ok(big) = BooleanEngine::boolean_solids_exact_result(&a1, &b1, op, &tol) else {
            println!("桁 1 で断られました");
            continue;
        };
        let (a2, b2) = build(small);
        let Ok(small_result) = BooleanEngine::boolean_solids_exact_result(&a2, &b2, op, &tol)
        else {
            println!("桁 {small} で断られました");
            continue;
        };
        let (Some(big_solid), Some(small_solid)) =
            (big.solids.first(), small_result.solids.first())
        else {
            println!("立体が返りません");
            continue;
        };
        let Ok(shrunk) = BrepTransform::scale_solid(big_solid, small) else {
            println!("縮められません");
            continue;
        };

        let reference: Vec<(f64, Point3)> = faces_of(&shrunk)
            .iter()
            .map(|face| face_contribution(face, shrunk.id))
            .collect();
        let measured: Vec<(f64, Point3)> = faces_of(small_solid)
            .iter()
            .map(|face| face_contribution(face, small_solid.id))
            .collect();

        println!(
            "面の数: 縮めたほう {} 枚、小さく切ったほう {} 枚",
            reference.len(),
            measured.len()
        );

        // **この口が信用できるかを、先に確かめます。** 面ごとの寄与を足したら、
        // 立体の体積になるはずです。ならなければ、1枚ずつ切り出せていません。
        let summed: f64 = measured.iter().map(|(volume, _)| volume).sum();
        let whole = MassCalculator::compute_from_brep(small_solid, &params()).volume;
        let agreement = (summed - whole).abs() / whole.abs().max(f64::MIN_POSITIVE);
        println!(
            "  検算: 面ごとの寄与の合計 {summed:.12}、立体の体積 {whole:.12}（相対差 {agreement:.3e}）"
        );
        if agreement > 1e-9 {
            println!("  **この口は信用できません**——面を1枚ずつ切り出せていません。下の表は読まないでください。");
        }

        // **p-curve が稜からどれだけ離れているか**を、桁ごとに出します（4-235）。
        //
        // 面のトリム境界はこの p-curve で決まります。**絶対のずれなら、桁を
        // 変えても同じ大きさのまま残り、小さい模型ほど相対では効きます。**
        // コードは変えずに、いまある `validate_pcurves` で測ります。
        let pcurve_worst = |solid: &Solid| -> f64 {
            let mut worst = 0.0f64;
            let mut worst_kind = "-";
            for face in faces_of(solid) {
                let Ok(report) = face.validate_pcurves(&tol, 8) else {
                    continue;
                };
                if report.max_distance > worst {
                    worst = report.max_distance;
                    // **どの種類の面が最悪値を出しているか。** 平面の p-curve は
                    // uv では直線なので、稜が曲がっていればその膨らみぶん外れます。
                    worst_kind = match &face.geometry {
                        zenith_topo::FaceGeometry::Plane(_) => "平面",
                        zenith_topo::FaceGeometry::Nurbs(_) => "曲面",
                        _ => "その他",
                    };
                }
            }
            if worst > 0.0 {
                println!("    最悪を出したのは {worst_kind} の面");
            }
            worst
        };
        // **割合の対応そのものを測ります**（4-239）。
        //
        // `validate_pcurves` は「同じ割合の点どうし」を比べます。**p-curve が
        // 正確でも、割合の対応がずれていれば距離は縮みません。** そこで
        // 2つ並べます——同じ割合での距離と、**稜のどこでもよいとしたときの
        // 最短距離**。後者が小さくて前者が大きければ、**形は合っていて
        // 対応だけがずれています**。
        let correspondence = |solid: &Solid| {
            let (mut same_fraction, mut nearest_anywhere) = (0.0f64, 0.0f64);
            let mut worst_edge = 0u64;
            let oriented_edge_id = |edge: &zenith_topo::OrientedEdge| edge.edge.id;
            for face in faces_of(solid) {
                let Ok(pcurves) = face.pcurves(&tol) else {
                    continue;
                };
                let zenith_topo::FaceGeometry::Nurbs(surface) = &face.geometry else {
                    continue;
                };
                for (edge, segment) in face.outer_wire.edges.iter().zip(pcurves.outer_loop.segments.iter())
                {
                    let (t_min, t_max) = segment.curve.param_range();
                    for step in 0..=64 {
                        let fraction = step as f64 / 64.0;
                        let uv = segment.curve.evaluate(t_min + (t_max - t_min) * fraction);
                        let from_pcurve = surface.evaluate(uv.x, uv.y);
                        let from_edge = edge.evaluate_normalized(fraction);
                        let gap = (from_pcurve - from_edge).norm();
                        if gap > same_fraction {
                            same_fraction = gap;
                            worst_edge = oriented_edge_id(edge);
                        }
                        // 稜のどこでもよいとしたときの最短。
                        let mut best = f64::INFINITY;
                        for probe in 0..=256 {
                            let other = probe as f64 / 256.0;
                            best = best.min((from_pcurve - edge.evaluate_normalized(other)).norm());
                        }
                        nearest_anywhere = nearest_anywhere.max(best);
                    }
                }
            }
            (same_fraction, nearest_anywhere, worst_edge)
        };
        // **どの稜が浮いているのかを名指しします**（4-240）。
        //
        // 稜ごとに、支持曲面からの浮きと**素性**（番号・長さ・曲線の次数・
        // その番号を使っている面の数）を並べます。**交線から来た稜は2枚の
        // 面が共有し**、元の立体から切り取られた稜や蓋の縁はそうとは限りません。
        {
            let mut uses: std::collections::HashMap<u64, usize> = Default::default();
            for face in faces_of(small_solid) {
                for wire in std::iter::once(&face.outer_wire).chain(face.inner_wires.iter()) {
                    for oriented in &wire.edges {
                        *uses.entry(oriented.edge.id).or_insert(0) += 1;
                    }
                }
            }
            let mut rows: Vec<(f64, u64, f64, usize, usize)> = Vec::new();
            for face in faces_of(small_solid) {
                let zenith_topo::FaceGeometry::Nurbs(surface) = &face.geometry else {
                    continue;
                };
                for wire in std::iter::once(&face.outer_wire).chain(face.inner_wires.iter()) {
                    for oriented in &wire.edges {
                        let mut worst = 0.0f64;
                        let mut length = 0.0f64;
                        let mut previous: Option<Point3> = None;
                        // **標本は多めに取ります**（4-241）。32 では足りず、
                        // 浮きを 1.894e-7 と見誤りました。
                        for step in 0..=256 {
                            let point = oriented.evaluate_normalized(step as f64 / 256.0);
                            if let Some(last) = previous {
                                length += (point - last).norm();
                            }
                            previous = Some(point);
                            // **曲面までの距離は、厳密に射影して測ります。**
                            // 格子で撒くと格子の目で頭打ちになり、**順位まで
                            // 当てになりません**（一度それで測って外しました）。
                            let best = zenith_geom::ExtremumEngine::point_to_surface(
                                point, surface, 32, 1e-12,
                            )
                            .map(|projection| projection.distance)
                            .unwrap_or(f64::INFINITY);
                            worst = worst.max(best);
                        }
                        rows.push((
                            worst,
                            oriented.edge.id,
                            length,
                            oriented.edge.curve.degree,
                            uses.get(&oriented.edge.id).copied().unwrap_or(0),
                        ));
                    }
                }
            }
            rows.sort_by(|left, right| right.0.total_cmp(&left.0));
            rows.dedup_by(|a, b| a.1 == b.1);
            println!("  稜ごとの浮き（厳密な射影で測っています）");
            for (float, id, length, degree, shared) in rows.iter().take(5) {
                println!(
                    "    稜 {id}: 浮き {float:.3e}、長さ {length:.6}、次数 {degree}、共有 {shared} 面"
                );
            }
        }

        let (same_small, nearest_small, worst_edge) = correspondence(small_solid);
        println!(
            "  割合の対応: 同じ割合での最悪 {same_small:.3e}、**稜までの最短の最悪 {nearest_small:.3e}**（桁 {small}、いちばん悪い稜は **{worst_edge}**）"
        );

        let big_pcurve = pcurve_worst(big_solid);
        let small_pcurve = pcurve_worst(small_solid);
        println!(
            "  p-curve が稜から離れる量: 桁 1 で {big_pcurve:.3e}、桁 {small} で {small_pcurve:.3e}（桁 1 を {small} 倍すると {:.3e}）",
            big_pcurve * small
        );

        // いちばん近い重心どうしで突き合わせる。
        let mut rows: Vec<(f64, f64, f64, Point3, f64)> = Vec::new();
        for (volume, centre) in &measured {
            let Some((closest, distance)) = reference
                .iter()
                .map(|(other_volume, other_centre)| {
                    (*other_volume, (other_centre - centre).norm())
                })
                .min_by(|left, right| left.1.total_cmp(&right.1))
            else {
                continue;
            };
            rows.push((
                (volume - closest).abs(),
                *volume,
                closest,
                *centre,
                distance,
            ));
        }
        rows.sort_by(|left, right| right.0.total_cmp(&left.0));

        let total: f64 = rows.iter().map(|row| row.0).sum();
        println!(
            "{:>14}{:>18}{:>18}{:>12}  {}",
            "寄与の差", "小さく切った", "縮めた", "重心の差", "重心"
        );
        println!("{}", "-".repeat(96));
        for (gap, mine, theirs, centre, distance) in rows.iter().take(8) {
            println!(
                "{:>14.3e}{:>18.9}{:>18.9}{:>12.3e}  ({:.4}, {:.4}, {:.4})",
                gap, mine, theirs, distance, centre.x, centre.y, centre.z
            );
        }
        println!("{}", "-".repeat(96));
        println!(
            "寄与の差の合計 {total:.3e}（立体の体積 {whole:.3e} に対して**相対 {:.3e}**）",
            total / whole.abs().max(f64::MIN_POSITIVE)
        );
        // **1枚に集まっているのか、全面に散っているのか。**
        let largest = rows.first().map(|row| row.0).unwrap_or(0.0);
        println!(
            "いちばん大きい1枚が {largest:.3e}——合計の {:.1}% です（{} 枚中）",
            largest / total.max(f64::MIN_POSITIVE) * 100.0,
            rows.len()
        );
        println!();
    }

    println!("**寄与の差が大きい面が、誤差を運んでいる面です。** 重心の差が大きい行は");
    println!("突き合わせ自体が外れているので、そこは読まないでください。");
}
