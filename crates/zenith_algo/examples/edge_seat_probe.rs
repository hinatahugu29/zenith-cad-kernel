//! **読んだ稜を、両側の面へ載せ直せるか**（4-518）。
//!
//! 4-517 で「輪（境界の稜）が面から 1.2e-4 浮いている限り、面の上で揃える
//! ことと輪で閉じることは両立しない」と分かりました。**直すなら読み込み側で
//! 稜を面へ載せ直す**ことになりますが、稜は 2 枚の面が共有しているので、
//! **両方に同時に載る位置が近くにあるか**が先の問いです。
//!
//! 稜ごとに内側の点を取り、**2 枚の面へ交互に射影**して、
//!
//! * 浮き（それぞれの面までの距離）
//! * 動かす量（元の点から、交互射影が落ち着いた点まで）
//! * 残る食い違い（落ち着いた点での、2 枚の面の射影どうしの距離）
//!
//! を出します。**測るだけで、何も直しません。**
//!
//! `cargo run --release -p zenith_algo --example edge_seat_probe [ファイル名]`

use std::collections::HashMap;
use std::path::PathBuf;
use zenith_geom::ExtremumEngine;
use zenith_io::StepImporter;
use zenith_math::Point3;
use zenith_topo::{Face, FaceGeometry};

fn occt_sample(name: &str) -> PathBuf {
    PathBuf::from(concat!(
        env!("CARGO_MANIFEST_DIR"),
        "/../../reference/OCCT/data/step"
    ))
    .join(name)
}

/// 面の上のいちばん近い点。平面と NURBS だけ。
fn project(face: &Face, point: Point3) -> Option<Point3> {
    match &face.geometry {
        FaceGeometry::Plane(plane) => {
            let normal = plane.normal.normalize();
            Some(point - normal * (point - plane.origin).dot(&normal))
        }
        FaceGeometry::Nurbs(surface) => {
            let projection = ExtremumEngine::point_to_surface(point, surface, 64, 1e-13).ok()?;
            Some(surface.evaluate(projection.u, projection.v))
        }
        _ => None,
    }
}

fn main() {
    let name = std::env::args()
        .nth(1)
        .unwrap_or_else(|| "linkrods.step".to_string());
    let solids = StepImporter::import_solids_from_file(&occt_sample(&name)).expect("読めません");
    let solid = solids
        .iter()
        .max_by_key(|solid| solid.outer_shell.faces.len())
        .expect("立体が 0 個");
    let faces = &solid.outer_shell.faces;
    let iterations: usize = std::env::var("ZENITH_SEAT_ITER")
        .ok()
        .and_then(|v| v.parse().ok())
        .unwrap_or(60);

    // 稜 → その稜を使う面。
    let mut users: HashMap<u64, Vec<usize>> = HashMap::new();
    let mut curves = HashMap::new();
    for (index, face) in faces.iter().enumerate() {
        for wire in std::iter::once(&face.outer_wire).chain(face.inner_wires.iter()) {
            for oriented in &wire.edges {
                users.entry(oriented.edge.id).or_default().push(index);
                curves
                    .entry(oriented.edge.id)
                    .or_insert(oriented.edge.curve.clone());
            }
        }
    }

    println!("{name}: 面 {} 枚、稜 {} 本", faces.len(), users.len());
    println!(
        "面の申告する粗さ（face.tolerance）の最大: {:.3e}",
        faces.iter().map(|face| face.tolerance).fold(0.0, f64::max)
    );
    println!();
    println!(
        "{:>6} {:>4} {:>4} {:>10} {:>10} {:>10} {:>10}",
        "稜", "面A", "面B", "浮きA", "浮きB", "動かす量", "残る差"
    );

    let mut ids: Vec<_> = users.keys().copied().collect();
    ids.sort();
    let mut rows = Vec::new();
    for id in ids {
        let owners = &users[&id];
        if owners.len() != 2 || owners[0] == owners[1] {
            continue;
        }
        let (a, b) = (&faces[owners[0]], &faces[owners[1]]);
        let curve = &curves[&id];
        let (t0, t1) = curve.param_range();
        let (mut float_a, mut float_b, mut moved, mut residual) = (0.0f64, 0.0f64, 0.0f64, 0.0f64);
        let mut measured = true;
        let mut angle = 0.0f64;
        for step in 1..8 {
            let original = curve.evaluate(t0 + (t1 - t0) * step as f64 / 8.0);
            let (Some(on_a), Some(on_b)) = (project(a, original), project(b, original)) else {
                measured = false;
                break;
            };
            float_a = float_a.max((on_a - original).norm());
            float_b = float_b.max((on_b - original).norm());
            // 交互射影。中点から始めて、2 枚の間を行き来させます。
            let mut point = Point3::from((on_a.coords + on_b.coords) * 0.5);
            let mut gap = f64::INFINITY;
            for _ in 0..iterations {
                let (Some(pa), Some(pb)) = (project(a, point), project(b, point)) else {
                    break;
                };
                gap = (pa - pb).norm();
                point = Point3::from((pa.coords + pb.coords) * 0.5);
                if gap < 1e-12 {
                    break;
                }
            }
            if let (Some(na), Some(nb)) = (normal_at(a, point), normal_at(b, point)) {
                angle = angle.max(na.dot(&nb).abs().min(1.0).acos().to_degrees());
            }
            moved = moved.max((point - original).norm());
            residual = residual.max(gap);
        }
        if !measured {
            continue;
        }
        println!("{id:>6} {:>4} {:>4} {float_a:>10.3e} {float_b:>10.3e} {moved:>10.3e} {residual:>10.3e} {angle:>7.2}°",
            owners[0], owners[1]);
        rows.push((float_a.max(float_b), moved, residual));
    }

    println!();
    let count = |f: &dyn Fn(&(f64, f64, f64)) -> bool| rows.iter().filter(|row| f(row)).count();
    println!("測った稜 {} 本", rows.len());
    println!(
        "  浮き  > 4e-5: {} 本 / > 1e-4: {} 本",
        count(&|row| row.0 > 4e-5),
        count(&|row| row.0 > 1e-4)
    );
    println!(
        "  載せ直したあとの残る差 > 1e-7: {} 本 / > 1e-5: {} 本",
        count(&|row| row.2 > 1e-7),
        count(&|row| row.2 > 1e-5)
    );
    println!(
        "  動かす量の最大: {:.3e}",
        rows.iter().map(|row| row.1).fold(0.0, f64::max)
    );
}

/// 面の法線（単位）。平面と NURBS だけ。
fn normal_at(face: &Face, point: Point3) -> Option<zenith_math::Vec3> {
    match &face.geometry {
        FaceGeometry::Plane(plane) => Some(plane.normal.normalize()),
        FaceGeometry::Nurbs(surface) => {
            let projection = ExtremumEngine::point_to_surface(point, surface, 64, 1e-13).ok()?;
            let (_, du, dv) = surface.evaluate_derivatives_1st(projection.u, projection.v);
            let normal = du.cross(&dv);
            (normal.norm() > 1e-300).then(|| normal.normalize())
        }
        _ => None,
    }
}
