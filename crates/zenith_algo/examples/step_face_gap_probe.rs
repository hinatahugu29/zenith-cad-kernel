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
