//! 稜の上の「耳」が、**どちらの面の uv で潰れているか**を測る。
//!
//! `screw.step` の表示メッシュに残る穴は、**面 9 と面 21 が共有する稜の上に
//! 同じ三角形を 1 枚ずつ作り**、溶接が 1 枚落として釣り合いが崩れるものです
//! （4-335、4-336）。**耳は 1 枚だけが作るべき**ですが、**どちらが作るべきか**
//! は幾何で決まります。
//!
//! **見分け方の候補**: その 3 点は、**片方の面の uv では一直線**（＝耳は
//! 面積 0 で、そこに三角形を置く理由が無い）、**もう片方では曲がっている**
//! （＝普通の三角形として要る）。**そうなっていれば、潰れている側が
//! 作らなければよい**。
//!
//! **これは診断です。** 赤にはしません——測って、どちらが潰れているかを
//! 言うだけです。

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

fn faces_in_order(solid: &Solid) -> Vec<Face> {
    let mut faces = solid.outer_shell.faces.clone();
    for inner in &solid.inner_shells {
        faces.extend(inner.faces.clone());
    }
    faces
}

fn main() {
    let tol = Tolerance::default();

    // **面は座標で選びます**（4-337）。`Face::id` を使おうとして外しました
    // ——**`id` は組み立てのたびに振り直され**、同じファイルを 2 度読むだけで
    // 変わります。**3 点すべてが乗っている面**を拾えば、番号は要りません。
    let watches: &[(&str, [Point3; 3])] = &[
        (
            "screw.step",
            [
                Point3::new(-25.379795977, -0.302436228, 2.936330000),
                Point3::new(-25.386385198, -0.407024074, 2.936330000),
                Point3::new(-25.391514000, -0.511735000, 2.936330000),
            ],
        ),
        (
            "screw.step",
            [
                Point3::new(-25.391514000, -1.140859000, 2.936330000),
                Point3::new(-25.386385198, -1.245570000, 2.936330000),
                Point3::new(-25.379795977, -1.350158000, 2.936330000),
            ],
        ),
    ];

    println!("稜の上の耳が、どちらの面の uv で潰れているかを測る");
    println!();

    for (name, points) in watches {
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

        println!(
            "{name}  耳 ({:.4},{:.4},{:.4}) / ({:.4},{:.4},{:.4}) / ({:.4},{:.4},{:.4})",
            points[0].x,
            points[0].y,
            points[0].z,
            points[1].x,
            points[1].y,
            points[1].z,
            points[2].x,
            points[2].y,
            points[2].z
        );
        // 3D での面積。**ここが 0 なら、そもそもどちらも作らなくてよい**。
        let twice_3d = (points[1] - points[0]).cross(&(points[2] - points[0])).norm();
        println!("  3D での面積の 2 倍: {twice_3d:.9}");

        // **3 点すべてが乗っている面を拾います。** 平面も NURBS も同じ扱い。
        let uv_of = |face: &Face, point: Point3| -> Option<((f64, f64), f64)> {
            match &face.geometry {
                FaceGeometry::Plane(plane) => {
                    let offset = point - plane.origin;
                    let distance = offset.dot(&plane.normal).abs();
                    Some(((offset.dot(&plane.u_axis), offset.dot(&plane.v_axis)), distance))
                }
                FaceGeometry::Nurbs(surface) => {
                    ExtremumEngine::point_to_surface(point, surface, 32, tol.parametric)
                        .ok()
                        .map(|p| ((p.u, p.v), p.distance))
                }
                _ => None,
            }
        };
        let mut hits = 0usize;
        for face in faces.iter() {
            let mut uvs = Vec::new();
            let mut worst_off = 0.0f64;
            for point in points.iter() {
                match uv_of(face, *point) {
                    Some((uv, distance)) => {
                        worst_off = worst_off.max(distance);
                        uvs.push(uv);
                    }
                    None => break,
                }
            }
            if uvs.len() != 3 {
                continue;
            }
            // **その面に乗っていなければ、関係ありません。** 読んだ面は
            // 自分の粗さを持ち歩くので、そこに合わせます（4-266）。
            let accept = (tol.linear * 10.0).max(face.tolerance + face.pcurve_tolerance);
            if worst_off > accept {
                continue;
            }
            hits += 1;
            let id = face.id;
            let kind = match &face.geometry {
                FaceGeometry::Plane(_) => "平面",
                FaceGeometry::Nurbs(_) => "NURBS",
                _ => "その他",
            };
            println!("  面 {id}（{kind}）に 3 点とも乗っています");
            let twice_uv = (uvs[1].0 - uvs[0].0) * (uvs[2].1 - uvs[0].1)
                - (uvs[1].1 - uvs[0].1) * (uvs[2].0 - uvs[0].0);
            println!(
                "    uv ({:.6},{:.6}) ({:.6},{:.6}) ({:.6},{:.6})",
                uvs[0].0, uvs[0].1, uvs[1].0, uvs[1].1, uvs[2].0, uvs[2].1
            );
            println!(
                "    uv での面積の 2 倍 {:.3e}、曲面からの外れ 最大 {worst_off:.3e}{}",
                twice_uv.abs(),
                if twice_uv.abs() <= 1e-12 {
                    "  ← **潰れています**"
                } else {
                    ""
                }
            );
        }
        if hits == 0 {
            println!("  3 点とも乗っている面がありません");
        }
        println!();
    }

    println!("**片方だけが潰れているなら、潰れている側が作らなければ足ります。**");
    println!("**両方潰れている／両方潰れていないなら、この見分け方は使えません**");
    println!("——そのときは別の決め方が要ります。**測ってから決めてください。**");
}
