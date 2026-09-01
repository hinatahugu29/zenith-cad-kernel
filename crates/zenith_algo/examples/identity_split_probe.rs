//! **恒等式の破れを、どちらの立体の境界から来たかに割る**（9-H の H6。4-244）。
//!
//! # 筋
//!
//! `|A∪B| + |A∩B| = |A| + |B|` が成り立つのは、**A の境界が、和と積で
//! 相補的に使われるから**です——B の外側にある部分は和へ、内側にある部分は
//! 積へ。B の境界も同じです。
//!
//! だから、**A から来た面の寄与を和と積で足すと `|A|` になる**はずです。
//! ならなければ、**A の境界の切り分けが相補になっていません**。
//!
//! 4-243 で「位相は同じ、面ごとの一致も良いのに恒等式だけ破れる」と分かった
//! ので、**破れをこの2つに割ります**。
//!
//! # 面の出どころの見分け方
//!
//! 支持曲面の**制御点そのもの**で照合します。ブーリアンは面を切り分けても
//! **支持曲面は共有する**ので、元の立体の曲面と一致します。
use zenith_algo::{
    BooleanEngine, BooleanOpType, BrepTransform, MassCalculator, PrimitiveBuilder,
};
use zenith_math::{Point2, Point3, Tolerance, Vec3};
use zenith_tess::TessellationParams;
use zenith_topo::{Face, FaceGeometry, Shell, Solid};

fn params() -> TessellationParams {
    TessellationParams {
        u_divisions: 32,
        v_divisions: 32,
    }
}

fn face_contribution(face: &Face) -> f64 {
    let one = Solid {
        id: 0,
        outer_shell: Shell {
            id: 0,
            faces: vec![face.clone()],
            is_closed: false,
        },
        inner_shells: Vec::new(),
    };
    MassCalculator::compute_from_brep(&one, &params()).volume
}

fn faces_of(solid: &Solid) -> Vec<&Face> {
    std::iter::once(&solid.outer_shell)
        .chain(solid.inner_shells.iter())
        .flat_map(|shell| shell.faces.iter())
        .collect()
}

/// 支持曲面の指紋。制御点をそのまま並べます。
fn surface_key(face: &Face) -> Option<Vec<[i64; 3]>> {
    let FaceGeometry::Nurbs(surface) = &face.geometry else {
        return None;
    };
    // 1e-9 の格子に丸めて比べます。**縮めた立体とは比べない**ので、
    // 同じ桁のあいだでは丸め誤差だけが問題です。
    let quantise = |value: f64| (value * 1e9).round() as i64;
    Some(
        surface
            .control_points
            .iter()
            .flat_map(|row| {
                row.iter()
                    .map(|control| {
                        [
                            quantise(control.point.x),
                            quantise(control.point.y),
                            quantise(control.point.z),
                        ]
                    })
                    .collect::<Vec<_>>()
            })
            .collect(),
    )
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
    let scales = [1.0_f64, 0.02, 0.01, 0.005];

    println!("恒等式の破れを、A（トーラス）側と B（円柱）側に割る");
    println!();
    println!(
        "{:>8}{:>18}{:>18}{:>18}  {}",
        "scale", "A 側の残り", "B 側の残り", "恒等式の破れ", "読み"
    );
    println!("{}", "-".repeat(88));

    for scale in scales {
        let (a, b) = build(scale);
        let Ok(union) = BooleanEngine::boolean_solids_exact_result(&a, &b, BooleanOpType::Union, &tol)
        else {
            println!("{scale:>8}  和が断られました");
            continue;
        };
        let Ok(intersection) =
            BooleanEngine::boolean_solids_exact_result(&a, &b, BooleanOpType::Intersection, &tol)
        else {
            println!("{scale:>8}  積が断られました");
            continue;
        };

        // 元の立体の曲面ごとに、寄与を集めます。
        let mut a_keys: Vec<Vec<[i64; 3]>> = Vec::new();
        let mut a_whole = 0.0f64;
        for face in faces_of(&a) {
            if let Some(key) = surface_key(face) {
                a_keys.push(key);
            }
            a_whole += face_contribution(face);
        }
        let mut b_keys: Vec<Vec<[i64; 3]>> = Vec::new();
        let mut b_whole = 0.0f64;
        for face in faces_of(&b) {
            if let Some(key) = surface_key(face) {
                b_keys.push(key);
            }
            b_whole += face_contribution(face);
        }

        let mut from_a = 0.0f64;
        let mut from_b = 0.0f64;
        let mut unmatched = 0.0f64;
        for solid in union.solids.iter().chain(intersection.solids.iter()) {
            for face in faces_of(solid) {
                let contribution = face_contribution(face);
                match surface_key(face) {
                    Some(key) if a_keys.contains(&key) => from_a += contribution,
                    Some(key) if b_keys.contains(&key) => from_b += contribution,
                    _ => unmatched += contribution,
                }
            }
        }

        // **どのトーラスのパッチで相補になっていないか**（4-245）。
        // 曲面ごとに、元の面の寄与と、和・積に入った片の寄与の合計を比べます。
        {
            // **パッチの位置も添えます**（4-246）。どの 8 枚かが分からないと、
            // 切られる側の半分なのか継ぎ目の側なのかを決められません。
            let centre_of = |face: &Face| -> Point3 {
                let FaceGeometry::Nurbs(surface) = &face.geometry else {
                    return Point3::new(0.0, 0.0, 0.0);
                };
                let mut sum = Vec3::new(0.0, 0.0, 0.0);
                let mut count = 0.0f64;
                for row in &surface.control_points {
                    for control in row {
                        sum += Vec3::new(control.point.x, control.point.y, control.point.z);
                        count += 1.0;
                    }
                }
                if count <= 0.0 {
                    return Point3::new(0.0, 0.0, 0.0);
                }
                Point3::new(sum.x / count, sum.y / count, sum.z / count)
            };
            let mut per_surface: Vec<(Vec<[i64; 3]>, f64, f64, Point3)> = Vec::new();
            for face in faces_of(&a) {
                if let Some(key) = surface_key(face) {
                    per_surface.push((key, face_contribution(face), 0.0, centre_of(face)));
                }
            }
            for solid in union.solids.iter().chain(intersection.solids.iter()) {
                for face in faces_of(solid) {
                    let Some(key) = surface_key(face) else {
                        continue;
                    };
                    if let Some(entry) = per_surface.iter_mut().find(|entry| entry.0 == key) {
                        entry.2 += face_contribution(face);
                    }
                }
            }
            let mut rows: Vec<(f64, f64, f64, Point3)> = per_surface
                .iter()
                .map(|(_, whole, pieces, centre)| {
                    ((pieces - whole).abs(), *whole, *pieces, *centre)
                })
                .collect();
            rows.sort_by(|left, right| right.0.total_cmp(&left.0));
            let broken = rows.iter().filter(|row| row.0 > 1e-12).count();
            println!(
                "    トーラスの面 {} 枚のうち、相補になっていないもの {broken} 枚",
                rows.len()
            );
            for (gap, _whole, _pieces, centre) in rows.iter().take(8) {
                if *gap <= 1e-12 {
                    break;
                }
                println!(
                    "      ずれ {gap:.3e}  パッチの中心 ({:.6}, {:.6}, {:.6})",
                    centre.x, centre.y, centre.z
                );
            }
        }

        // **同じ切り口の稜が、和の片と積の片で同じ p-curve を持っているか**
        // （4-247）。持っていなければ、同じ切り口を2回別々に近似していること
        // になり、その差がそのまま隙間になります。
        {
            let collect = |solids: &[Solid]| {
                let mut out: std::collections::HashMap<u64, Vec<Point2>> = Default::default();
                for solid in solids {
                    for face in faces_of(solid) {
                        let Ok(pcurves) = face.pcurves(&tol) else {
                            continue;
                        };
                        for segment in pcurves.outer_loop.segments.iter().chain(
                            pcurves.inner_loops.iter().flat_map(|loops| loops.segments.iter()),
                        ) {
                            out.entry(segment.edge_id).or_insert_with(|| {
                                segment
                                    .curve
                                    .control_points
                                    .iter()
                                    .map(|control| control.point)
                                    .collect()
                            });
                        }
                    }
                }
                out
            };
            let in_union = collect(&union.solids);
            let in_intersection = collect(&intersection.solids);
            let (mut shared, mut differ, mut worst) = (0usize, 0usize, 0.0f64);
            for (edge_id, left) in &in_union {
                let Some(right) = in_intersection.get(edge_id) else {
                    continue;
                };
                shared += 1;
                if left.len() != right.len() {
                    differ += 1;
                    worst = f64::INFINITY;
                    continue;
                }
                let gap = left
                    .iter()
                    .zip(right.iter())
                    .map(|(a, b)| (a - b).norm())
                    .fold(0.0f64, f64::max);
                if gap > 0.0 {
                    differ += 1;
                    worst = worst.max(gap);
                }
            }
            println!(
                "    和と積で共有する稜 {shared} 本のうち、p-curve が違うもの {differ} 本（uv の最大差 {worst:.3e}）"
            );
        }

        let a_residual = from_a - a_whole;
        let b_residual = from_b - b_whole;
        let identity = (from_a + from_b + unmatched) - (a_whole + b_whole);
        println!(
            "{:>8}{:>18.3e}{:>18.3e}{:>18.3e}  {}",
            scale,
            a_residual,
            b_residual,
            identity,
            if unmatched.abs() > 0.0 {
                format!("出どころ不明の寄与 {unmatched:.3e}")
            } else {
                "全部の面の出どころが付きました".to_string()
            }
        );
    }

    println!("{}", "-".repeat(88));
    println!("**B 側の列は当てになりません**——円柱の面は、切り分けたあとに");
    println!("パッチが張り直されるので、**制御点が元と一致しません**。出どころ不明の");
    println!("寄与に丸ごと入ります。**A 側（トーラス）の列だけを読んでください。**");
    println!();
    println!("実測（4-244）: **A 側の残りが、恒等式の破れとぴたり一致します**");
    println!("（桁 0.01 で -5.182e-9 と -5.182e-9）。**破れはトーラスの境界から");
    println!("来ています**——和と積で、トーラスの面が相補的に切り分けられていません。");
    let _ = Point3::new(0.0, 0.0, 0.0);
}
