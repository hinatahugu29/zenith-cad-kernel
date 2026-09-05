//! 読んだ STEP で、**隣り合う面が本当に同じ場所で終わっているか**を測る。
//!
//! `linkrods.step` の縫合で相手のいない稜を追うと、**同じ切断面に対して
//! 2つの A の面が出した交線の端が 0.0069343 食い違う**ところに行き当たり
//! ます（4-320、4-321）。隣り合う面なら、同じ点で終わるはずです。
//!
//! **どちらが正しいかは、面そのものに当てて決まります。** 食い違った 2 点を
//! それぞれ両方の面へ射影し、どれだけ外れるかを見ます。
//!
//! - 両方の点が両方の面に乗る → **面は本当に接している**。食い違いは
//!   交線の求め方の側にある
//! - 片方しか乗らない → **ファイルの側で面が離れている**。公差では埋まらない
//!
//! **これは診断です。** 赤にはしません——読んだファイルが自分の申告より
//! 粗いことは実際にあり（4-266）、それ自体は欠陥ではありません。

use zenith_algo::Regularizer;
use zenith_geom::ExtremumEngine;
use zenith_io::StepImporter;
use zenith_math::{Point3, Tolerance};
use zenith_topo::{Face, FaceGeometry, Solid};
use std::path::PathBuf;

fn occt_sample(name: &str) -> PathBuf {
    PathBuf::from(concat!(
        env!("CARGO_MANIFEST_DIR"),
        "/../../reference/OCCT/data/step"
    ))
    .join(name)
}

fn face_count(solid: &Solid) -> usize {
    solid.outer_shell.faces.len()
        + solid
            .inner_shells
            .iter()
            .map(|shell| shell.faces.len())
            .sum::<usize>()
}

/// `all_solid_faces` と同じ並び（外側の殻 → 内側の殻）。
fn faces_in_order(solid: &Solid) -> Vec<Face> {
    let mut faces = solid.outer_shell.faces.clone();
    for inner in &solid.inner_shells {
        faces.extend(inner.faces.clone());
    }
    faces
}

/// 点がその面の**曲面**からどれだけ外れているか。トリムは見ません。
fn distance_to_surface(face: &Face, point: Point3, tol: &Tolerance) -> Option<f64> {
    match &face.geometry {
        FaceGeometry::Plane(plane) => Some((point - plane.origin).dot(&plane.normal).abs()),
        FaceGeometry::Nurbs(surface) => {
            ExtremumEngine::point_to_surface(point, surface, 32, tol.parametric)
                .ok()
                .map(|projection| projection.distance)
        }
        _ => None,
    }
}

fn main() {
    let tol = Tolerance::default();
    // 4-321 で名指しした食い違い。**座標を直に書きます**——面の番号は
    // 読み方が変われば動きますが、座標は動きません（4-247 と同じ流儀）。
    let watches: &[(&str, usize, usize, Point3, Point3)] = &[(
        "linkrods.step",
        1,
        35,
        Point3::new(7.994676, 2.814543, 1.410000),
        Point3::new(7.994676, 2.821477, 1.410000),
    )];

    println!("読んだ STEP で、隣り合う面が同じ場所で終わっているかを測る");
    println!();

    for (name, left_index, right_index, left_point, right_point) in watches {
        let path = occt_sample(name);
        let Ok(solids) = StepImporter::import_solids_from_file(&path) else {
            println!("{name:<16} **読めません**");
            continue;
        };
        let Some(subject) = solids
            .iter()
            .max_by_key(|solid| face_count(solid))
            .map(|solid| Regularizer::hold_like_our_own(solid, &tol))
        else {
            continue;
        };
        let faces = faces_in_order(&subject);
        println!("{name}  面 {} 枚", faces.len());

        let (Some(left), Some(right)) = (faces.get(*left_index), faces.get(*right_index)) else {
            println!("  面 {left_index} か {right_index} がありません");
            continue;
        };

        let kind = |face: &Face| match &face.geometry {
            FaceGeometry::Plane(_) => "平面",
            FaceGeometry::Nurbs(_) => "NURBS",
            _ => "その他",
        };
        println!(
            "  面 {left_index} は {}（自分の粗さ {:.3e} / p-curve {:.3e}）",
            kind(left),
            left.tolerance,
            left.pcurve_tolerance
        );
        println!(
            "  面 {right_index} は {}（自分の粗さ {:.3e} / p-curve {:.3e}）",
            kind(right),
            right.tolerance,
            right.pcurve_tolerance
        );
        // **番号は2種類あります**（4-325）。`face_a_index` は並び順、
        // `Face::id` は位相の番号です。**診断がどちらを出しているかを
        // 確かめずに読むと、別の面の数字を追います**——実際に追いました。
        println!(
            "  番号: 面{left_index} の id は {}、面{right_index} の id は {}",
            left.id, right.id
        );
        println!(
            "  外周の稜: 面 {left_index} は {} 本、面 {right_index} は {} 本",
            left.outer_wire.edges.len(),
            right.outer_wire.edges.len()
        );
        println!(
            "  **内側の輪**: 面 {left_index} は {} 個、面 {right_index} は {} 個",
            left.inner_wires.len(),
            right.inner_wires.len()
        );
        println!(
            "  2点の隔たり: {:.9}",
            (right_point - left_point).norm()
        );
        println!();

        // **境界どうしがどれだけ離れているか**（4-321）。
        //
        // 上の点は、それぞれの面の**トリム境界の上**にあります。2枚が本当に
        // 隣り合っているなら、境界は同じ線を共有しているはずです。**曲面が
        // 離れているのか、境界が離れているのか**で、直す先が違います。
        let nearest_on_wire = |face: &Face, point: Point3| -> f64 {
            face.outer_wire
                .edges
                .iter()
                .filter_map(|oriented| {
                    ExtremumEngine::point_to_curve(point, &oriented.edge.curve, 128, 1e-13)
                        .ok()
                        .map(|projection| projection.distance)
                })
                .fold(f64::INFINITY, f64::min)
        };
        println!(
            "  面 {left_index} 側の点 → 面 {right_index} の**境界**まで {:.9}",
            nearest_on_wire(right, *left_point)
        );
        println!(
            "  面 {right_index} 側の点 → 面 {left_index} の**境界**まで {:.9}",
            nearest_on_wire(left, *right_point)
        );
        println!(
            "  面 {left_index} 側の点 → 面 {left_index} 自身の境界まで {:.9}（0 なら境界の上）",
            nearest_on_wire(left, *left_point)
        );
        println!(
            "  面 {right_index} 側の点 → 面 {right_index} 自身の境界まで {:.9}",
            nearest_on_wire(right, *right_point)
        );
        println!();

        // **そもそも 2 枚は隣り合っているのか**（4-321）。
        //
        // 上の 2 点は**どちらも自分の面の内部**にあります（境界まで 0.345 と
        // 0.695）。**「隣り合う面の端が食い違っている」とは限りません**——
        // 別々の曲面が、そこをたまたま近い場所で通っているだけかもしれません。
        // **境界どうしがどれだけ近づくか**で決まります。
        let mut closest = f64::INFINITY;
        for a in left.outer_wire.edges.iter() {
            for step in 0..=32 {
                let point = a.evaluate_normalized(step as f64 / 32.0);
                for b in right.outer_wire.edges.iter() {
                    if let Ok(projection) =
                        ExtremumEngine::point_to_curve(point, &b.edge.curve, 64, 1e-12)
                    {
                        closest = closest.min(projection.distance);
                    }
                }
            }
        }
        println!(
            "  面 {left_index} と面 {right_index} の**境界どうしの最接近**: {closest:.9}{}",
            if closest <= tol.linear * 10.0 {
                "  ← 稜を共有しています（隣り合っています）"
            } else {
                "  ← 稜を共有していません"
            }
        );
        println!();

        // **その点は、面のトリムの中にあるか**（4-325）。
        //
        // 4-324 で、交線の端が**面の囲みを 0.94 出ている**のに**外周には
        // 一度も 0.0194 より近づかない**と測れました。**境界を跨がずに面から
        // 出ている**——トリムされた面では起こらないはずの形です。
        //
        // **トリムの中にあるかを直接見れば決まります。** 外なら、交線が面を
        // はみ出して作られています（SSI か候補の切り詰めの側）。
        let in_trim = |face: &Face, point: Point3| -> Option<bool> {
            let FaceGeometry::Nurbs(surface) = &face.geometry else {
                return None;
            };
            let projection =
                ExtremumEngine::point_to_surface(point, surface, 32, tol.parametric).ok()?;
            let pcurves = face.pcurves(&tol).ok()?;
            let mut polygon: Vec<(f64, f64)> = Vec::new();
            for segment in pcurves.outer_loop.segments.iter() {
                let (a, b) = segment.curve.param_range();
                for step in 0..24 {
                    let uv = segment.curve.evaluate(a + (b - a) * (step as f64 / 24.0));
                    polygon.push((uv.x, uv.y));
                }
            }
            if polygon.len() < 3 {
                return None;
            }
            let (u, v) = (projection.u, projection.v);
            let mut inside = false;
            let count = polygon.len();
            for index in 0..count {
                let (x1, y1) = polygon[index];
                let (x2, y2) = polygon[(index + 1) % count];
                if (y1 > v) != (y2 > v) {
                    let cross = x1 + (v - y1) / (y2 - y1) * (x2 - x1);
                    if cross > u {
                        inside = !inside;
                    }
                }
            }
            Some(inside)
        };
        for (label, index, face, point) in [
            ("面 1 側の点", *left_index, left, left_point),
            ("面 35 側の点", *right_index, right, right_point),
        ] {
            for (target_index, target) in [(*left_index, left), (*right_index, right)] {
                match in_trim(target, *point) {
                    Some(true) => println!(
                        "  {label} は 面 {target_index} の**トリムの中**にあります"
                    ),
                    Some(false) => println!(
                        "  {label} は 面 {target_index} の**トリムの外**です"
                    ),
                    None => println!("  {label} → 面 {target_index}: トリムを読めません"),
                }
            }
            let _ = (index, face);
        }
        // **p-curve と 3D の境界は一致しているか**（4-325）。
        //
        // 上の2つは噛み合いません——**uv ではトリムの外**なのに、
        // **3D では外周に 0.0194 より近づかない**（4-324）。uv でトリムを
        // 出るなら、3D では境界の近くを通るはずです。**片方が嘘をついて
        // います。**
        // **p-curve と 3D の境界は一致しているか**（4-325）。
        //
        // 上の2つは噛み合いません——**uv ではトリムの外**なのに、
        // **3D では外周に 0.0194 より近づかない**（4-324）。uv でトリムを
        // 出るなら、3D では境界の近くを通るはずです。**片方が嘘をついて
        // います。**
        // **交線が丸ごとトリムの外なら、境界を跨がないのは当たり前です**（4-325）。
        //
        // 端だけでなく、始点・中点・終点を当てます。**端点だけでは、
        // 出入りしているのか丸ごと外なのかが分かりません。**
        for (label, point) in [
            ("交線の始点", Point3::new(7.994676, 2.821477, 1.410000)),
            ("交線の中点", Point3::new(5.662277, 3.049389, 1.410000)),
            ("交線の終点", Point3::new(3.381585, 3.172162, 1.410000)),
        ] {
            let in_out = match in_trim(right, point) {
                Some(true) => "トリムの**中**",
                Some(false) => "トリムの**外**",
                None => "トリムを読めません",
            };
            println!(
                "  {label}: {in_out}、外周まで {:.9}",
                nearest_on_wire(right, point)
            );
        }
        println!();

        for (index, face) in [(*left_index, left), (*right_index, right)] {
            match face.validate_pcurves(&tol, 37) {
                Ok(report) => println!(
                    "  面 {index} の p-curve と 3D 境界: 食い違い {} 件、最大 {:.9}（許容 {:.3e}）",
                    report.mismatch_count,
                    report.max_distance,
                    face.tolerance + face.pcurve_tolerance
                ),
                Err(reason) => println!("  面 {index} の p-curve を検査できません: {reason}"),
            }
        }
        println!();

        for (label, point) in [("面 1 側の点", left_point), ("面 35 側の点", right_point)] {
            for (index, face) in [(*left_index, left), (*right_index, right)] {
                match distance_to_surface(face, *point, &tol) {
                    Some(distance) => println!(
                        "  {label} → 面 {index} の曲面まで {distance:.9}{}",
                        if distance <= face.tolerance.max(tol.linear * 10.0) {
                            "  ← 乗っています"
                        } else {
                            ""
                        }
                    ),
                    None => println!("  {label} → 面 {index}: この曲面では測れません"),
                }
            }
        }
        println!();
    }

    println!("**これは診断です。赤にはしません。** 読んだファイルが自分の申告より");
    println!("粗いことは実際にあります（4-266）。ここで見たいのは、食い違いが");
    println!("**ファイルの側にあるのか、交線の求め方の側にあるのか**です。");
}
