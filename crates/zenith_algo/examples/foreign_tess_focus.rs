//! 読んだ立体を面ごとにテッセレーションして、枚数・境界箱・所要を出す。
//!
//! `foreign_boolean_probe` がトーラスで返ってこなかったので、止まっている段を
//! 特定するために作りました。面ごとに吐くので、どの面で止まるかが分かります。
//!
//! ```bash
//! cargo run --release -p zenith_algo --example foreign_tess_focus -- torus 8
//! ```

use std::io::Write as _;
use std::path::PathBuf;
use std::time::Instant;

use zenith_io::StepImporter;
use zenith_math::Point3;
use zenith_tess::{tessellate_solid, TessellationParams, TriangleMesh};

fn say(message: &str) {
    println!("{message}");
    std::io::stdout().flush().ok();
}

fn bounds(mesh: &TriangleMesh) -> (Point3, Point3) {
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
    (low, high)
}

fn main() {
    let args: Vec<String> = std::env::args().skip(1).collect();
    let subject = args.first().cloned().unwrap_or_else(|| "torus".to_string());
    let divisions: usize = args
        .get(1)
        .and_then(|value| value.parse().ok())
        .unwrap_or(8);

    let path = PathBuf::from(concat!(env!("CARGO_MANIFEST_DIR"), "/tests/fixtures"))
        .join(format!("occ_reference_{subject}.step"));
    let solids = match StepImporter::import_solids_from_file(&path) {
        Ok(solids) => solids,
        Err(err) => {
            say(&format!("read failed: {err}"));
            return;
        }
    };
    let solid = &solids[0];
    let params = TessellationParams {
        u_divisions: divisions,
        v_divisions: divisions,
    };
    say(&format!(
        "subject {subject}  divisions {divisions}  {} face(s)",
        solid.outer_shell.faces.len()
    ));

    // 稜の刻み数が上限に張り付いていたので、まず稜そのものを見る。
    // `evaluate_normalized` を等間隔に取って、弦の中点からのたわみを出す。
    for (face_index, face) in solid.outer_shell.faces.iter().enumerate() {
        for (edge_index, oriented) in face.outer_wire.edges.iter().enumerate() {
            let a = oriented.evaluate_normalized(0.0);
            let mid = oriented.evaluate_normalized(0.5);
            let b = oriented.evaluate_normalized(1.0);
            let quarter = oriented.evaluate_normalized(0.25);
            let mut worst: f64 = 0.0;
            let n = 64;
            for step in 0..n {
                let t0 = step as f64 / n as f64;
                let t1 = (step + 1) as f64 / n as f64;
                let p0 = oriented.evaluate_normalized(t0);
                let p1 = oriented.evaluate_normalized(t1);
                let centre = oriented.evaluate_normalized((t0 + t1) * 0.5);
                let chord = Point3::from((p0.coords + p1.coords) * 0.5);
                worst = worst.max((centre - chord).norm());
            }
            say(&format!(
                "  face {face_index} edge {edge_index}: t=0 ({:.3} {:.3} {:.3})  t=.25 ({:.3} {:.3} {:.3})  t=.5 ({:.3} {:.3} {:.3})  t=1 ({:.3} {:.3} {:.3})  worst sag at 64 = {worst:.6}",
                a.x, a.y, a.z, quarter.x, quarter.y, quarter.z, mid.x, mid.y, mid.z, b.x, b.y, b.z
            ));
        }
    }

    // 面ごとに単独の立体へ包み直せないので、面の数だけ tessellate_solid を
    // 呼ぶことはできない。代わりに立体ごとの所要を測り、面の内訳は
    // 境界箱で見る。
    let start = Instant::now();
    let mesh = tessellate_solid(solid, &params);
    let (low, high) = bounds(&mesh);
    say(&format!(
        "{:>8.2}s  {} triangle(s)  bbox ({:.3} {:.3} {:.3}) - ({:.3} {:.3} {:.3})",
        start.elapsed().as_secs_f64(),
        mesh.indices.len() / 3,
        low.x,
        low.y,
        low.z,
        high.x,
        high.y,
        high.z
    ));
}
