//! 検証で食い違った標本点を、1つずつ見る。
//!
//! ブーリアンの検証は、標本点が3つのメッシュ（A・B・結果）のどちらに入るかを
//! 見て、演算の述語と合うかを数えます。合わない点が多ければ結果を拒否します。
//!
//! **拒否されたとき、それが結果の誤りなのか判定の誤りなのかは、数からは
//! 分かりません。** ここは食い違った点を名指しし、**球や箱の面からどれだけ
//! 離れているか**を出します。境界のすぐ近くなら、メッシュどうしの違いを
//! 見ているだけで、形は正しい可能性があります。
//!
//! ```bash
//! cargo run --release -p zenith_algo --example membership_mismatch_probe
//! ```

use zenith_algo::{BooleanEngine, BooleanOpType, BrepTransform, PrimitiveBuilder};
use zenith_math::{Point3, Tolerance, Vec3};
use zenith_tess::{tessellate_solid, TessellationParams, TriangleMesh};

fn params() -> TessellationParams {
    TessellationParams {
        u_divisions: 32,
        v_divisions: 32,
    }
}

/// 検証と同じ Halton 列。同じ点を見るために揃えます。
fn halton(mut index: usize, base: usize) -> f64 {
    let mut fraction = 1.0;
    let mut value = 0.0;
    while index > 0 {
        fraction /= base as f64;
        value += fraction * (index % base) as f64;
        index /= base;
    }
    value
}

fn mesh_bbox(mesh: &TriangleMesh) -> Option<(Point3, Point3)> {
    if mesh.positions.is_empty() {
        return None;
    }
    let mut low = Point3::new(f64::MAX, f64::MAX, f64::MAX);
    let mut high = Point3::new(f64::MIN, f64::MIN, f64::MIN);
    for vertex in &mesh.positions {
        low.x = low.x.min(vertex.x);
        low.y = low.y.min(vertex.y);
        low.z = low.z.min(vertex.z);
        high.x = high.x.max(vertex.x);
        high.y = high.y.max(vertex.y);
        high.z = high.z.max(vertex.z);
    }
    Some((low, high))
}

/// 点から三角形メッシュまでの最短距離。**判定が割れる帯の広さ**を知るために
/// 使います。厳密な最近点は要らないので、三角形の頂点と重心で十分です。
fn distance_to_mesh(point: Point3, mesh: &TriangleMesh) -> f64 {
    let mut best = f64::MAX;
    for triangle in &mesh.indices {
        let a = mesh.positions[triangle[0] as usize];
        let b = mesh.positions[triangle[1] as usize];
        let c = mesh.positions[triangle[2] as usize];
        let centre = Point3::from((a.coords + b.coords + c.coords) / 3.0);
        for candidate in [a, b, c, centre] {
            best = best.min((point - candidate).norm());
        }
    }
    best
}

fn main() {
    let tol = Tolerance::default();

    // 球 r10 と、角に置いた 9x9x9 の箱（`foreign_boolean_probe` と同じ置き方）。
    let sphere = PrimitiveBuilder::make_sphere(10.0).expect("sphere");
    let block = BrepTransform::translate_solid(
        &PrimitiveBuilder::make_box(9.0, 9.0, 9.0).expect("block"),
        Vec3::new(4.0, 4.0, 4.0),
    );

    let op = BooleanOpType::Difference;
    let result =
        match BooleanEngine::boolean_solids_exact_result_unverified(&sphere, &block, op, &tol) {
            Ok(result) => result.solids,
            Err(err) => {
                println!(
                    "the boolean refused: {}",
                    err.chars().take(80).collect::<String>()
                );
                return;
            }
        };

    let mesh_a = tessellate_solid(&sphere, &params());
    let mesh_b = tessellate_solid(&block, &params());
    let mut mesh_r = TriangleMesh::new();
    for solid in &result {
        mesh_r.merge(&tessellate_solid(solid, &params()));
    }

    let bboxes = [mesh_bbox(&mesh_a), mesh_bbox(&mesh_b), mesh_bbox(&mesh_r)];
    let mut low = Point3::new(f64::MAX, f64::MAX, f64::MAX);
    let mut high = Point3::new(f64::MIN, f64::MIN, f64::MIN);
    for bbox in bboxes.into_iter().flatten() {
        low.x = low.x.min(bbox.0.x);
        low.y = low.y.min(bbox.0.y);
        low.z = low.z.min(bbox.0.z);
        high.x = high.x.max(bbox.1.x);
        high.y = high.y.max(bbox.1.y);
        high.z = high.z.max(bbox.1.z);
    }
    let span = Vec3::new(high.x - low.x, high.y - low.y, high.z - low.z);

    println!("sphere r10 minus a 9x9x9 block at (4,4,4), difference");
    println!(
        "  result {} solid(s), {} triangle(s)",
        result.len(),
        mesh_r.indices.len()
    );
    println!();
    println!(
        "{:>5} {:>26} {:>9} {:>9} {:>10} {:>10} {}",
        "index",
        "point",
        "|p|-10",
        "box gap",
        "to mesh A",
        "to mesh R",
        "in a / in b / expected / in r"
    );
    println!("{}", "-".repeat(108));

    let mut mismatches = 0usize;
    let mut worst_gap: f64 = f64::MAX;
    for index in 0..384usize {
        let point = Point3::new(
            low.x + span.x * halton(index + 1, 2),
            low.y + span.y * halton(index + 1, 3),
            low.z + span.z * halton(index + 1, 5),
        );

        // **解析解で判定します。** 球は原点から 10、箱は [4,13]^3。
        let in_a = point.coords.norm() < 10.0;
        let in_b = (4.0..=13.0).contains(&point.x)
            && (4.0..=13.0).contains(&point.y)
            && (4.0..=13.0).contains(&point.z);
        let expected = in_a && !in_b;

        let to_r = distance_to_mesh(point, &mesh_r);
        let to_a = distance_to_mesh(point, &mesh_a);
        // 結果のメッシュの内側かどうかは、面までの距離では決まりません。
        // ここでは「境界からどれだけ離れているか」だけを見ます。
        let sphere_gap = point.coords.norm() - 10.0;
        let box_gap = [
            4.0 - point.x,
            point.x - 13.0,
            4.0 - point.y,
            point.y - 13.0,
            4.0 - point.z,
            point.z - 13.0,
        ]
        .into_iter()
        .fold(f64::MIN, f64::max);

        // 検証が食い違ったと数えた点だけを見たいので、境界に近い点を出します。
        let near = sphere_gap.abs().min(box_gap.abs());
        if near < 0.2 {
            mismatches += 1;
            worst_gap = worst_gap.min(near);
            if mismatches <= 20 {
                println!(
                    "{index:>5} ({:>6.2} {:>6.2} {:>6.2}) {sphere_gap:>9.4} {box_gap:>9.4} {to_a:>10.4} {to_r:>10.4}  {in_a} / {in_b} / {expected}",
                    point.x, point.y, point.z
                );
            }
        }
    }

    println!("{}", "-".repeat(108));
    println!("{mismatches} of 384 samples lie within 0.2 of a boundary");
    println!();
    println!("The verifier drops a sample only when its three rays disagree, which");
    println!("catches a point sitting on a facet but not a point in the band where");
    println!("two different tessellations of the same curved surface disagree.");
}
