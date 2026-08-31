//! `cone × torus` で残っているメッシュ非多様体の**機構を名指しする**ための口。
//!
//! 4-207 で「境界の辺が1本足りない」ぶんは埋まり、同じ口で数えると欠けは
//! 0 本です。**残る4演算は別の機構**で、まだ名前が付いていません。
//!
//! この probe は推測せずに、非多様体の稜のそばで**何が起きているか**だけを
//! 出します。
//!
//! - 稜が何回使われているか（1 なら穴、3 以上なら重なり）
//! - その稜の端点に、**溶接距離のすぐ外に居る別の頂点**があるか
//!   （`weld` は 1e-7 で束ねます。1e-7 と 1e-6 の間に相手が居るなら、
//!   「同じ点のはずのものが2つ残っている」ことになります）
//! - いちばん近い頂点対の距離（メッシュ全体）
use std::collections::HashMap;
use zenith_algo::{BooleanEngine, BooleanOpType, BrepTransform, PrimitiveBuilder};
use zenith_math::{Tolerance, Vec3};
use zenith_tess::{TessellationParams, TriangleMesh};

const WELD: f64 = 1e-7;

fn edge_uses(mesh: &TriangleMesh) -> HashMap<(u32, u32), usize> {
    let mut uses: HashMap<(u32, u32), usize> = HashMap::new();
    for triangle in &mesh.indices {
        for step in 0..3 {
            let (a, b) = (triangle[step], triangle[(step + 1) % 3]);
            if a == b {
                continue;
            }
            let key = if a < b { (a, b) } else { (b, a) };
            *uses.entry(key).or_insert(0) += 1;
        }
    }
    uses
}

/// その頂点から、**溶接距離のすぐ外**に居る頂点を並べる。
fn near_twins(mesh: &TriangleMesh, vertex: u32, upper: f64) -> Vec<(u32, f64)> {
    let here = mesh.positions[vertex as usize];
    let mut out = Vec::new();
    for (index, position) in mesh.positions.iter().enumerate() {
        if index as u32 == vertex {
            continue;
        }
        let distance = (position - here).norm();
        if distance <= upper {
            out.push((index as u32, distance));
        }
    }
    out.sort_by(|a, b| a.1.partial_cmp(&b.1).unwrap());
    out
}

/// その稜の上に、**端点ではない別の頂点が乗っている**か（T字）。
///
/// 乗っているなら、片側の面はその点を使って2本に割り、もう片側は割らずに
/// 1本のまま結んでいます。**継ぎ目はそこで開きます。**
fn straddling_vertex(mesh: &TriangleMesh, a: u32, b: u32) -> Option<(u32, f64)> {
    let (pa, pb) = (mesh.positions[a as usize], mesh.positions[b as usize]);
    let direction = pb - pa;
    let length_sq = direction.norm_squared();
    if length_sq <= 0.0 {
        return None;
    }
    let mut best: Option<(u32, f64)> = None;
    for (index, position) in mesh.positions.iter().enumerate() {
        let index = index as u32;
        if index == a || index == b {
            continue;
        }
        let t = (position - pa).dot(&direction) / length_sq;
        if !(1e-6..=1.0 - 1e-6).contains(&t) {
            continue;
        }
        let distance = (position - (pa + direction * t)).norm();
        // **弦からの外れは 0 ではありません。** 片側が刻みを1つ飛ばすと、
        // 飛ばした側の弦は曲線の膨らみ（サジッタ）ぶんだけ外れます——実測
        // 6.2e-3（半径 4.6 の弧を 0.85 の弦で結んだとき）。ここを 1e-9 に
        // していたときは、T字を1本も見つけられませんでした。
        if distance > 1e-2 {
            continue;
        }
        if best.as_ref().map(|(_, d)| distance < *d).unwrap_or(true) {
            best = Some((index, distance));
        }
    }
    best
}

fn report(name: &str, mesh: &TriangleMesh) {
    let uses = edge_uses(mesh);
    let mut bad: Vec<_> = uses.iter().filter(|(_, count)| **count != 2).collect();
    bad.sort_by_key(|((a, b), _)| (*a, *b));
    println!(
        "\n{name}: 三角形 {}、頂点 {}、非多様体の稜 {} 本",
        mesh.indices.len(),
        mesh.positions.len(),
        bad.len()
    );
    if bad.is_empty() {
        return;
    }
    let (mut twins, mut tees, mut other) = (0usize, 0usize, 0usize);
    for ((a, b), _) in &bad {
        if straddling_vertex(mesh, *a, *b).is_some() {
            tees += 1;
        } else if !near_twins(mesh, *a, 1e-5).is_empty() || !near_twins(mesh, *b, 1e-5).is_empty() {
            twins += 1;
        } else {
            other += 1;
        }
    }
    println!("  内訳: 双子 {twins} 本、T字 {tees} 本、どちらでもない {other} 本");
    for ((a, b), count) in bad {
        let (pa, pb) = (mesh.positions[*a as usize], mesh.positions[*b as usize]);
        println!(
            "  稜 [{a}]-[{b}] 使用 {count} 回、長さ {:.3e}  ({:.9},{:.9},{:.9}) -> ({:.9},{:.9},{:.9})",
            (pb - pa).norm(),
            pa.x, pa.y, pa.z, pb.x, pb.y, pb.z
        );
        if let Some((vertex, distance)) = straddling_vertex(mesh, *a, *b) {
            println!(
                "    **T字**: 稜の上に [{vertex}] が乗っている（外れ {:.3e}）——片側だけが割っている",
                distance
            );
        }
        for (vertex, label) in [(*a, "a"), (*b, "b")] {
            let twins = near_twins(mesh, vertex, 1e-5);
            if twins.is_empty() {
                println!("    {label}=[{vertex}] の 1e-5 以内に別の頂点は無い");
                continue;
            }
            for (other, distance) in twins.iter().take(4) {
                println!(
                    "    {label}=[{vertex}] の {:.3e} に [{other}]{}",
                    distance,
                    if *distance > WELD {
                        "  <- 溶接距離のすぐ外（束ねられていない）"
                    } else {
                        ""
                    }
                );
            }
        }
    }
}

/// **B-Rep の側に「同じはずの点」が2つ無いか。**
///
/// メッシュの双子が B-Rep から来ているのか、テッセレーションが作ったのかは、
/// ここで分かれます。稜の端点（頂点）と、稜を等分に取った点の両方を見ます。
fn brep_twins(solid: &zenith_topo::Solid) -> Vec<(f64, zenith_math::Point3)> {
    let mut points: Vec<zenith_math::Point3> = Vec::new();
    for shell in std::iter::once(&solid.outer_shell).chain(solid.inner_shells.iter()) {
        for face in &shell.faces {
            for wire in std::iter::once(&face.outer_wire).chain(face.inner_wires.iter()) {
                for oriented in &wire.edges {
                    points.push(oriented.start_vertex().point);
                    points.push(oriented.end_vertex().point);
                }
            }
        }
    }
    let mut out = Vec::new();
    for i in 0..points.len() {
        for j in (i + 1)..points.len() {
            let distance = (points[j] - points[i]).norm();
            if distance > 0.0 && distance <= 1e-5 {
                out.push((distance, points[i]));
            }
        }
    }
    out.sort_by(|a, b| a.0.partial_cmp(&b.0).unwrap());
    out.dedup_by(|a, b| (a.0 - b.0).abs() < 1e-15);
    out
}

/// **同じ場所を通る稜が2本ないか。** 端点は一致するのに、中間の標本だけが
/// ずれるなら、2つの面が「同じ稜」を別々の曲線で持っています。
fn duplicated_edge_curves(solid: &zenith_topo::Solid) -> Vec<(u64, u64, f64, f64)> {
    let mut sampled: Vec<(u64, Vec<zenith_math::Point3>)> = Vec::new();
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
            // 端点が一致する向きだけを見る（逆向きも見る）。
            for reversed in [false, true] {
                let paired: Vec<f64> = (0..=8)
                    .map(|step| {
                        let other = if reversed { 8 - step } else { step };
                        (b.1[other] - a.1[step]).norm()
                    })
                    .collect();
                let worst = paired.iter().cloned().fold(0.0f64, f64::max);
                let ends = paired[0].max(paired[8]);
                if worst <= 1e-5 && ends <= 1e-12 && worst > 0.0 {
                    out.push((a.0, b.0, ends, worst));
                }
            }
        }
    }
    out
}

/// **稜の曲線の端は、頂点の位置と一致しているか。**
///
/// 境界の標本は稜の曲線から取ります（`evaluate_normalized`）。隣り合う2本の
/// 稜がそれぞれ自分の曲線を評価するので、曲線の端が頂点からずれていると、
/// 継ぎ目に**同じはずの点が2つ**できます。
fn curve_end_vs_vertex(solid: &zenith_topo::Solid) -> (usize, f64, usize) {
    let (mut checked, mut worst, mut over_weld) = (0usize, 0.0f64, 0usize);
    for shell in std::iter::once(&solid.outer_shell).chain(solid.inner_shells.iter()) {
        for face in &shell.faces {
            for wire in std::iter::once(&face.outer_wire).chain(face.inner_wires.iter()) {
                for oriented in &wire.edges {
                    for (fraction, vertex) in [
                        (0.0, oriented.start_vertex().point),
                        (1.0, oriented.end_vertex().point),
                    ] {
                        let gap = (oriented.evaluate_normalized(fraction) - vertex).norm();
                        checked += 1;
                        worst = worst.max(gap);
                        if gap > 1e-7 {
                            over_weld += 1;
                        }
                    }
                }
            }
        }
    }
    (checked, worst, over_weld)
}

fn main() {
    let tol = Tolerance::default();
    let cone = PrimitiveBuilder::make_cone(10.0, 0.0, 20.0).expect("cone");
    let torus = PrimitiveBuilder::make_torus(12.0, 4.0).expect("torus");
    let params = TessellationParams {
        u_divisions: 24,
        v_divisions: 24,
    };

    let placements = [
        (
            "rim on the tube",
            BrepTransform::translate_solid(&torus, Vec3::new(10.0, 0.0, 0.0)),
        ),
        (
            "lifted off the base plane",
            BrepTransform::translate_solid(&torus, Vec3::new(10.0, 0.0, 3.0)),
        ),
    ];
    let ops = [
        ("union", BooleanOpType::Union),
        ("difference", BooleanOpType::Difference),
        ("intersection", BooleanOpType::Intersection),
    ];

    for (placement, b) in &placements {
        for (label, op) in ops {
            let Ok(result) = BooleanEngine::boolean_solids_exact_result(&cone, b, op, &tol) else {
                println!("\n{placement} / {label}: 断られた");
                continue;
            };
            for (index, solid) in result.solids.iter().enumerate() {
                for (a, b, ends, worst) in duplicated_edge_curves(solid) {
                    println!(
                        "
{placement} / {label} / 立体 {index}: **同じ場所を通る稜が2本** 稜 {a} と 稜 {b}（端点の差 {ends:.3e}、中間の最大差 {worst:.3e}）"
                    );
                }
                let (checked, worst, over_weld) = curve_end_vs_vertex(solid);
                println!(
                    "
{placement} / {label} / 立体 {index}: 稜の端 {checked} 箇所、曲線と頂点の差は最大 {worst:.3e}、溶接距離より大きいもの {over_weld} 箇所"
                );
                let twins = brep_twins(solid);
                if !twins.is_empty() {
                    println!(
                        "
{placement} / {label} / 立体 {index}: **B-Rep の頂点に双子** {} 組（最短 {:.3e}、最長 {:.3e}）",
                        twins.len(),
                        twins.first().unwrap().0,
                        twins.last().unwrap().0
                    );
                    for (distance, point) in twins.iter().take(6) {
                        println!(
                            "  {:.3e} 離れた対、片方は ({:.9},{:.9},{:.9})",
                            distance, point.x, point.y, point.z
                        );
                    }
                }
                let mesh = zenith_tess::tessellate_solid(solid, &params);
                report(&format!("{placement} / {label} / 立体 {index}"), &mesh);
            }
        }
    }
}
