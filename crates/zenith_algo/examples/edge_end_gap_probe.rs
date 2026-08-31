//! **稜の曲線の端は、頂点の位置と一致しているか**を、組を変えて掃きます（4-208）。
//!
//! 4-208 で `cone × torus` の表示メッシュが壊れていた原因です。境界の標本は
//! 稜の曲線から取るので、曲線の端が頂点からずれていると、隣り合う2本の稜が
//! 継ぎ目に「同じはずの点」を2つ作ります。溶接の距離 (1e-7) より大きいと
//! 束ねられず、そこが穴になります。
//!
//! **表示側では両端を頂点へ寄せて塞ぎました。上流の差は残っています。**
//! この口は、その差が `cone × torus` だけの話なのか、交線を作る組に広く
//! あるのかを、推測せずに見るためのものです。
//!
//! 見るのは3つ。
//!
//! - 差の最大（`worst`）
//! - 溶接の距離 (1e-7) を超えている箇所の数（`over weld`）
//! - 稜の端の総数（`ends`）
use zenith_algo::{BooleanEngine, BooleanOpType, BrepTransform, PrimitiveBuilder};
use zenith_math::{Point3, Tolerance, Vec3};
use zenith_topo::Solid;

const WELD: f64 = 1e-7;

/// 稜の曲線の端と、その端の頂点との差。
fn end_gaps(solid: &Solid) -> (usize, f64, usize) {
    let (mut ends, mut worst, mut over) = (0usize, 0.0f64, 0usize);
    for shell in std::iter::once(&solid.outer_shell).chain(solid.inner_shells.iter()) {
        for face in &shell.faces {
            for wire in std::iter::once(&face.outer_wire).chain(face.inner_wires.iter()) {
                for oriented in &wire.edges {
                    for (fraction, vertex) in [
                        (0.0, oriented.start_vertex().point),
                        (1.0, oriented.end_vertex().point),
                    ] {
                        let gap = (oriented.evaluate_normalized(fraction) - vertex).norm();
                        ends += 1;
                        worst = worst.max(gap);
                        if gap > WELD {
                            over += 1;
                        }
                    }
                }
            }
        }
    }
    (ends, worst, over)
}

/// **稜の途中に、別の稜の頂点が乗っていないか**（T字の頂点。4-209）。
///
/// 乗っていると、その稜を持つ面と、頂点を持つ面とで、**境界の点列が
/// 食い違います**。B-Rep の多様体判定は稜の使われ方しか見ないので、ここは
/// 通り抜けます。効くのはテッセレーションで、**面をまたいだ継ぎ目が開きます**。
fn t_vertices(solid: &Solid) -> Vec<(Point3, f64, f64)> {
    let mut vertices: Vec<Point3> = Vec::new();
    let mut edges: Vec<(Point3, Point3, Vec<Point3>)> = Vec::new();
    for shell in std::iter::once(&solid.outer_shell).chain(solid.inner_shells.iter()) {
        for face in &shell.faces {
            for wire in std::iter::once(&face.outer_wire).chain(face.inner_wires.iter()) {
                for oriented in &wire.edges {
                    vertices.push(oriented.start_vertex().point);
                    vertices.push(oriented.end_vertex().point);
                    let samples = (1..32)
                        .map(|step| oriented.evaluate_normalized(step as f64 / 32.0))
                        .collect();
                    edges.push((
                        oriented.start_vertex().point,
                        oriented.end_vertex().point,
                        samples,
                    ));
                }
            }
        }
    }

    let mut out = Vec::new();
    for vertex in &vertices {
        for (start, end, samples) in &edges {
            if (vertex - start).norm() <= 1e-9 || (vertex - end).norm() <= 1e-9 {
                continue;
            }
            let Some(closest) = samples
                .iter()
                .map(|sample| (sample - vertex).norm())
                .min_by(|a, b| a.partial_cmp(b).unwrap())
            else {
                continue;
            };
            if closest > 1e-6 {
                continue;
            }
            let along = (vertex - start).norm() / (end - start).norm().max(1e-30);
            out.push((*vertex, closest, along));
        }
    }
    out.sort_by(|a, b| a.1.partial_cmp(&b.1).unwrap());
    out.dedup_by(|a, b| (a.0 - b.0).norm() <= 1e-9);
    out
}

/// **同じ線の上に、別々の稜が部分的に重なっていないか**（4-209）。
///
/// 端点まで一致していれば「同じ稜が2本」で、それは別に測ってあります
/// （0組でした）。ここで探すのは**片方がもう片方の一部を覆っている**形です。
/// 覆われている側の面は、覆っている側の面が持たない点を境界に持つので、
/// **面ごとの境界の点列が食い違います**。
fn overlapping_edges(solid: &Solid) -> Vec<(u64, u64, usize, f64)> {
    let mut sampled: Vec<(u64, Vec<Point3>)> = Vec::new();
    for shell in std::iter::once(&solid.outer_shell).chain(solid.inner_shells.iter()) {
        for face in &shell.faces {
            for wire in std::iter::once(&face.outer_wire).chain(face.inner_wires.iter()) {
                for oriented in &wire.edges {
                    if sampled.iter().any(|(id, _)| *id == oriented.edge.id) {
                        continue;
                    }
                    let points = (0..=16)
                        .map(|step| oriented.evaluate_normalized(step as f64 / 16.0))
                        .collect();
                    sampled.push((oriented.edge.id, points));
                }
            }
        }
    }

    let distance_to_polyline = |point: &Point3, polyline: &[Point3]| -> f64 {
        let mut best = f64::INFINITY;
        for pair in polyline.windows(2) {
            let along = pair[1] - pair[0];
            let length_sq = along.norm_squared();
            let t = if length_sq <= 0.0 {
                0.0
            } else {
                ((point - pair[0]).dot(&along) / length_sq).clamp(0.0, 1.0)
            };
            best = best.min((point - (pair[0] + along * t)).norm());
        }
        best
    };

    let mut out = Vec::new();
    for i in 0..sampled.len() {
        for j in 0..sampled.len() {
            if i == j {
                continue;
            }
            let (left, right) = (&sampled[i], &sampled[j]);
            // 端点どうしが一致する（＝同じ稜）ものは、ここでは見ません。
            let ends_match = (left.1[0] - right.1[0]).norm() <= 1e-9
                && (left.1[16] - right.1[16]).norm() <= 1e-9;
            let ends_match_reversed = (left.1[0] - right.1[16]).norm() <= 1e-9
                && (left.1[16] - right.1[0]).norm() <= 1e-9;
            if ends_match || ends_match_reversed {
                continue;
            }
            let (mut on, mut worst) = (0usize, 0.0f64);
            for point in &left.1 {
                let gap = distance_to_polyline(point, &right.1);
                if gap <= 1e-7 {
                    on += 1;
                    worst = worst.max(gap);
                }
            }
            // 端点1つだけ触れているのは、ふつうに繋がっているだけです。
            if on >= 3 {
                out.push((left.0, right.0, on, worst));
            }
        }
    }
    out
}

/// **同じ場所を通る稜が2本ないか**（4-209。判定を直しました）。
///
/// 前は「中間の最大差が 0 より大きい」ものだけを見ていたので、**完全に
/// 一致する2本**を見落としていました。稜の番号が別なら、刻みの計画
/// （`SamplePlan::segments_for`）も別に引かれます。**同じ稜のはずなのに
/// 面ごとに刻み数が違えば、境界の点列が食い違います。**
fn identical_edges(solid: &Solid) -> Vec<(u64, u64, f64)> {
    let mut sampled: Vec<(u64, Vec<Point3>)> = Vec::new();
    for shell in std::iter::once(&solid.outer_shell).chain(solid.inner_shells.iter()) {
        for face in &shell.faces {
            for wire in std::iter::once(&face.outer_wire).chain(face.inner_wires.iter()) {
                for oriented in &wire.edges {
                    if sampled.iter().any(|(id, _)| *id == oriented.edge.id) {
                        continue;
                    }
                    let points = (0..=8)
                        .map(|step| oriented.evaluate_normalized(step as f64 / 8.0))
                        .collect();
                    sampled.push((oriented.edge.id, points));
                }
            }
        }
    }
    let mut out = Vec::new();
    for i in 0..sampled.len() {
        for j in (i + 1)..sampled.len() {
            let (a, b) = (&sampled[i], &sampled[j]);
            for reversed in [false, true] {
                let worst = (0..=8)
                    .map(|step| {
                        let other = if reversed { 8 - step } else { step };
                        (b.1[other] - a.1[step]).norm()
                    })
                    .fold(0.0f64, f64::max);
                if worst <= 1e-7 {
                    out.push((a.0, b.0, worst));
                }
            }
        }
    }
    out
}

fn main() {
    let tol = Tolerance::default();
    let boxa = PrimitiveBuilder::make_box(20.0, 20.0, 20.0).expect("box");
    let cylinder = PrimitiveBuilder::make_cylinder(6.0, 40.0).expect("cylinder");
    let sphere = PrimitiveBuilder::make_sphere(10.0).expect("sphere");
    let cone = PrimitiveBuilder::make_cone(10.0, 0.0, 20.0).expect("cone");
    let torus = PrimitiveBuilder::make_torus(12.0, 4.0).expect("torus");

    let shifted = |solid: &Solid, x: f64, y: f64, z: f64| {
        BrepTransform::translate_solid(solid, Vec3::new(x, y, z))
    };

    let cases: Vec<(&str, Solid, Solid)> = vec![
        ("box x box (corner)", boxa.clone(), shifted(&boxa, 10.0, 10.0, 10.0)),
        ("box x cylinder", boxa.clone(), shifted(&cylinder, 5.0, 5.0, -10.0)),
        ("box x sphere", boxa.clone(), shifted(&sphere, 10.0, 10.0, 15.0)),
        ("box x cone", boxa.clone(), shifted(&cone, 8.0, 8.0, 5.0)),
        ("box x torus", boxa.clone(), shifted(&torus, 5.0, 5.0, 10.0)),
        ("cylinder x sphere", cylinder.clone(), shifted(&sphere, 3.0, 0.0, 0.0)),
        ("cylinder x cone", cylinder.clone(), shifted(&cone, 3.0, 0.0, -5.0)),
        ("cylinder x torus", cylinder.clone(), shifted(&torus, 0.0, 0.0, 0.0)),
        ("sphere x cone", sphere.clone(), shifted(&cone, 0.0, 0.0, -5.0)),
        ("sphere x torus", sphere.clone(), shifted(&torus, 0.0, 0.0, 0.0)),
        ("cone x torus (rim on the tube)", cone.clone(), shifted(&torus, 10.0, 0.0, 0.0)),
        ("cone x torus (lifted)", cone.clone(), shifted(&torus, 10.0, 0.0, 3.0)),
        ("torus x torus", torus.clone(), shifted(&torus, 12.0, 0.0, 0.0)),
        // **4-209 で壊れる3件**。面をまたいで境界の点が食い違うところを
        // 探しています。
        ("box x cone (generatrix in a face)", boxa.clone(), {
            let half_angle = (10f64 / 20.0).atan();
            let stand = zenith_math::Transform3::from_axis_angle(
                &Vec3::new(0.0, 1.0, 0.0),
                half_angle,
            );
            let generatrix_x = 20.0 / 5f64.sqrt();
            BrepTransform::translate_solid(
                &BrepTransform::transform_solid(
                    &PrimitiveBuilder::make_cone(10.0, 0.0, 20.0).expect("cone"),
                    &stand,
                )
                .expect("stand cone"),
                Vec3::new(20.0 - generatrix_x, 10.0, 5.0),
            )
        }),
    ];
    let ops = [
        ("union", BooleanOpType::Union),
        ("difference", BooleanOpType::Difference),
        ("intersection", BooleanOpType::Intersection),
    ];

    println!("稜の曲線の端と、その端の頂点との差（表示側の寄せは別。ここは上流の値です）");
    println!();
    println!("{:<34}{:<14}{:>8}{:>14}{:>12}", "case", "op", "ends", "worst", "over weld");
    println!("{}", "-".repeat(84));

    let (mut worst_all, mut worst_where) = (0.0f64, String::from("-"));
    let (mut over_all, mut ends_all) = (0usize, 0usize);
    for (name, a, b) in &cases {
        for (label, op) in ops {
            let Ok(result) = BooleanEngine::boolean_solids_exact_result(a, b, op, &tol) else {
                println!("{:<34}{:<14}{:>8}{:>14}{:>12}", name, label, "-", "断られた", "-");
                continue;
            };
            let (mut ends, mut worst, mut over) = (0usize, 0.0f64, 0usize);
            for solid in &result.solids {
                let (e, w, o) = end_gaps(solid);
                ends += e;
                worst = worst.max(w);
                over += o;
            }
            for solid in &result.solids {
                for (a, b, worst) in identical_edges(solid) {
                    println!(
                        "{:<34}{:<14}  **同じ場所を通る稜が2本**: 稜 {a} と 稜 {b}（最大差 {worst:.3e}）",
                        name, label
                    );
                }
                for (a, b, on, worst) in overlapping_edges(solid) {
                    println!(
                        "{:<34}{:<14}  **稜 {a} の 17 点中 {on} 点が 稜 {b} の上に乗っている**（最大 {worst:.3e}）",
                        name, label
                    );
                }
            }
            let tees: usize = result.solids.iter().map(|solid| t_vertices(solid).len()).sum();
            if tees > 0 {
                println!("{:<34}{:<14}{:>8}{:>14}{:>12}  **稜の途中に頂点 {tees} 個**", name, label, ends, format!("{worst:.3e}"), over);
            }
            ends_all += ends;
            over_all += over;
            if worst > worst_all {
                worst_all = worst;
                worst_where = format!("{name} / {label}");
            }
            println!(
                "{:<34}{:<14}{:>8}{:>14.3e}{:>12}",
                name, label, ends, worst, over
            );
        }
    }
    println!("{}", "-".repeat(84));
    println!(
        "稜の端 {ends_all} 箇所のうち、溶接の距離 (1e-7) を超えているもの {over_all} 箇所。最大 {worst_all:.3e}（{worst_where}）"
    );
    println!();
    println!("**0 でないこと自体は誤答ではありません。** B-Rep は多様体のままで、");
    println!("恒等式も破れません。効くのは表示メッシュだけで、そこは 4-208 で");
    println!("両端を頂点へ寄せて塞いであります。ここは**上流が良くなったか**を");
    println!("見るための口です。");
}
