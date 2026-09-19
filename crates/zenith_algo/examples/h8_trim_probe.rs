//! **H8 の壁を、uv の絵で見る**（4-488）。
//!
//! # なぜ要るのか
//!
//! `linkrods.step` を切ると、**面が割れずに縫合があぶれます**。
//! 断り文は「**切り込みが境界から N 離れている**」で、**N は 0.1 〜 1.09**
//! ——公差の話ではありません。**では、交線はどこにいるのか。**
//!
//! **この口が、それを uv で見せます。** 実測（4-488）——
//!
//! ```text
//! A面13 の uv: u[-1.5708,1.5708] v[-0.20977,-0.00302]
//!   輪の uv: (-0.785,-0.2098) ... (0.785,-0.2098) ...   ← トリムは u の半分だけ
//!   交線の始: uv=(1.5708,-0.04648)                      ← **トリムの外**
//!   交線の終: uv=(-0.5036,-0.20977)
//! ```
//!
//! **交線は、面が使っていない所まで辿られ、パッチの縁で止まっています。**
//! **境界の手前で終わっているのではなく、境界の外まで伸びている**のでした。
//!
//! # 何を出すか
//!
//! **割れなかった面**について、
//!
//! * パラメータ領域と、**輪の uv**（トリムがどこにあるか）
//! * **交線の端の uv**（トリムの中か外か）
//! * **輪が面からどれだけ浮いているか**（読んだファイルは粗い。4-266）
//! * 端で**2 つの面が接しているか**（接していれば交線は本当にそこで終わり）
//!
//! # つまみ
//!
//! * `ZENITH_H8_FACES=13,17` — 見る面を選びます（既定は下の一覧）
//! * `ZENITH_NURBS_CLIP=1` — **トリムで切ってから**見ます
//!   （実測: A面13 の交線が断片 2 本 → 輪から輪へ 1 本になります）
//!
//! **これは診断です。門ではありません**——赤にはなりません。
//!
//! ```bash
//! cargo run --release -p zenith_algo --example h8_trim_probe
//! ```

use zenith_algo::{BrepIntersectionBuilder, BrepTransform, PrimitiveBuilder};
use zenith_io::StepImporter;
use zenith_math::{BoundingBox3, Point3, Tolerance, Vec3};
use zenith_topo::{Face, FaceGeometry, Solid};

const SAMPLE: &str = "reference/OCCT/data/step/linkrods.step";

fn boundary_bounding_box(solid: &Solid) -> Option<BoundingBox3> {
    let mut bbox: Option<BoundingBox3> = None;
    for face in &solid.outer_shell.faces {
        for wire in std::iter::once(&face.outer_wire).chain(face.inner_wires.iter()) {
            for point in wire.sample_points(12) {
                match &mut bbox {
                    Some(box3) => box3.extend_point(point),
                    None => bbox = Some(BoundingBox3::from_point(point)),
                }
            }
        }
    }
    bbox
}

/// 点における面の法線。**接しているかを見る**ために使います。
fn face_normal(face: &Face, point: Point3) -> Option<Vec3> {
    match &face.geometry {
        FaceGeometry::Plane(plane) => Some(plane.normal.normalize()),
        FaceGeometry::Nurbs(surface) => {
            let projection =
                zenith_geom::ExtremumEngine::point_to_surface(point, surface, 64, 1e-12).ok()?;
            let (_, du, dv) = surface.evaluate_derivatives_1st(projection.u, projection.v);
            let normal = du.cross(&dv);
            (normal.norm() > 1e-15).then(|| normal.normalize())
        }
        _ => None,
    }
}

fn main() {
    let tol = Tolerance::default();
    let Ok(solids) = StepImporter::import_solids_from_file(SAMPLE) else {
        println!("**{SAMPLE} が読めません。**");
        println!("OCCT の `data/step/` から置いてください（4-487）。**ここは赤にしません。**");
        return;
    };
    let Some(subject) = solids
        .into_iter()
        .max_by_key(|solid| solid.outer_shell.faces.len())
    else {
        println!("立体がありません。");
        return;
    };

    let bbox = boundary_bounding_box(&subject).unwrap_or_else(|| subject.bounding_box());
    let span = Vec3::new(
        bbox.max.x - bbox.min.x,
        bbox.max.y - bbox.min.y,
        bbox.max.z - bbox.min.z,
    );
    // **`read_and_cut_probe` と同じ切り手**です（3% 内へ、高さ 0.47）。
    let (inset, height) = (0.03, 0.47);
    let cutter = BrepTransform::translate_solid(
        &PrimitiveBuilder::make_box(
            span.x * (1.0 - inset * 2.0),
            span.y * (1.0 - inset * 2.0),
            span.z * height,
        )
        .expect("箱"),
        Vec3::new(
            bbox.min.x + span.x * inset,
            bbox.min.y + span.y * inset,
            bbox.min.z + span.z * (height * 0.5),
        ),
    );

    let faces_a = subject.outer_shell.faces.clone();
    let faces_b = cutter.outer_shell.faces.clone();
    let candidates =
        BrepIntersectionBuilder::collect_intersection_edge_candidates(&faces_a, &faces_b, &tol);

    let watch: Vec<usize> = match std::env::var("ZENITH_H8_FACES") {
        Ok(text) => text
            .split(',')
            .filter_map(|piece| piece.trim().parse().ok())
            .collect(),
        // **4-488 で「割れなかった」面**です。
        Err(_) => vec![1, 13, 14, 17, 18, 25, 30, 31, 35],
    };

    println!("H8 の壁を uv で見る（4-488）");
    println!();
    println!("**交線 {} 本**、面 {} 枚（読んだ立体）", candidates.len(), faces_a.len());
    println!("**`ZENITH_NURBS_CLIP=1` を付けると、トリムで切ってから見ます。**");
    println!();

    for face_index in watch {
        let Some(face) = faces_a.get(face_index) else {
            continue;
        };
        let mine: Vec<_> = candidates
            .iter()
            .filter(|candidate| candidate.face_a_index == face_index)
            .collect();
        if mine.is_empty() {
            continue;
        }
        let FaceGeometry::Nurbs(surface) = &face.geometry else {
            continue;
        };
        let ((u0, u1), (v0, v1)) = surface.param_range();

        println!(
            "A面{face_index}: 交線 {} 本、外周の稜 {} 本、内側の輪 {} 個",
            mine.len(),
            face.outer_wire.edges.len(),
            face.inner_wires.len()
        );
        println!("  領域 u[{u0:.4},{u1:.4}] v[{v0:.5},{v1:.5}]");

        // **輪が uv のどこにいるか**と、**面からどれだけ浮いているか**。
        let mut lo_u = f64::INFINITY;
        let mut hi_u = f64::NEG_INFINITY;
        let mut worst_off = 0.0f64;
        for point in face.outer_wire.sample_points(24) {
            if let Ok(projection) =
                zenith_geom::ExtremumEngine::point_to_surface(point, surface, 64, 1e-13)
            {
                lo_u = lo_u.min(projection.u);
                hi_u = hi_u.max(projection.u);
                worst_off = worst_off.max(projection.distance);
            }
        }
        println!(
            "  輪の u は [{lo_u:.4},{hi_u:.4}]、輪が面から浮いている量は最大 {worst_off:.3e}"
        );

        for candidate in mine {
            for (label, point) in [
                ("始", candidate.edge.start_vertex.point),
                ("終", candidate.edge.end_vertex.point),
            ] {
                let Ok(projection) =
                    zenith_geom::ExtremumEngine::point_to_surface(point, surface, 64, 1e-13)
                else {
                    continue;
                };
                let outside = projection.u < lo_u - 1e-9 || projection.u > hi_u + 1e-9;
                let sine = match (
                    face_normal(face, point),
                    face_normal(&faces_b[candidate.face_b_index], point),
                ) {
                    (Some(na), Some(nb)) => na.cross(&nb).norm(),
                    _ => f64::NAN,
                };
                println!(
                    "    B面{:>2} の{label}: uv=({:.4},{:.5}){}、接し具合 {:.2e}",
                    candidate.face_b_index,
                    projection.u,
                    projection.v,
                    if outside { " ← **トリムの外**" } else { "" },
                    sine
                );
            }
        }
        println!();
    }

    println!("**読み方**（4-488）:");
    println!("* **トリムの外**と出たら、交線が面の使っていない所まで伸びています");
    println!("* **接し具合が 0 でない**なら、そこは交線の本当の終わりではありません");
    println!("* **輪が面から浮いている量**は、このファイルの粗さです（4-266）");
}
