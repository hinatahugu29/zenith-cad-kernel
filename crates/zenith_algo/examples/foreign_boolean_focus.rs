//! `foreign_boolean_probe` が返ってこないので、どの組で止まるかを1つずつ見る。
//!
//! 総当たりの表は、止まる場所を教えてくれません。ここは検体・切り手・演算を
//! 引数で1つだけ受け取り、**各段で即座に吐きます**（Rust の標準出力はファイルへ
//! 流すとブロック緩衝になるので、明示的に flush します）。
//!
//! ```bash
//! cargo run --release -p zenith_algo --example foreign_boolean_focus -- cone drill difference
//! ```

use std::io::Write as _;
use std::path::PathBuf;
use std::time::Instant;

use zenith_algo::{BooleanEngine, BooleanOpType, BrepTransform, MassCalculator, PrimitiveBuilder};
use zenith_io::StepImporter;
use zenith_math::{Point3, Tolerance, Vec3};
use zenith_tess::{tessellate_solid, TessellationParams, TriangleMesh};
use zenith_topo::Solid;

fn params() -> TessellationParams {
    TessellationParams {
        u_divisions: 64,
        v_divisions: 64,
    }
}

fn say(message: &str) {
    println!("{message}");
    std::io::stdout().flush().ok();
}

fn mesh_bounds(mesh: &TriangleMesh) -> (Point3, Point3) {
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

fn cutter(kind: &str, low: &Point3, high: &Point3) -> Result<Solid, String> {
    let size = Vec3::new(high.x - low.x, high.y - low.y, high.z - low.z);
    match kind {
        "slab" => {
            let solid = PrimitiveBuilder::make_box(size.x * 0.6, size.y * 2.0, size.z * 2.0)?;
            Ok(BrepTransform::translate_solid(
                &solid,
                Vec3::new(
                    low.x - size.x * 0.11,
                    low.y - size.y * 0.5,
                    low.z - size.z * 0.5,
                ),
            ))
        }
        "drill" => {
            let radius = size.x.min(size.y) * 0.18;
            let solid = PrimitiveBuilder::make_cylinder(radius, size.z * 3.0)?;
            Ok(BrepTransform::translate_solid(
                &solid,
                Vec3::new(
                    (low.x + high.x) * 0.5,
                    (low.y + high.y) * 0.5,
                    low.z - size.z,
                ),
            ))
        }
        "corner" => {
            let solid = PrimitiveBuilder::make_box(size.x * 0.45, size.y * 0.45, size.z * 0.45)?;
            Ok(BrepTransform::translate_solid(
                &solid,
                Vec3::new(
                    high.x - size.x * 0.30,
                    high.y - size.y * 0.30,
                    high.z - size.z * 0.30,
                ),
            ))
        }
        other => Err(format!("unknown cutter {other}; use slab, drill or corner")),
    }
}

fn main() {
    let args: Vec<String> = std::env::args().skip(1).collect();
    let subject = args.first().cloned().unwrap_or_else(|| "cone".to_string());
    let kind = args.get(1).cloned().unwrap_or_else(|| "slab".to_string());
    let op_name = args
        .get(2)
        .cloned()
        .unwrap_or_else(|| "difference".to_string());
    let op = match op_name.as_str() {
        "union" => BooleanOpType::Union,
        "difference" => BooleanOpType::Difference,
        "intersection" => BooleanOpType::Intersection,
        other => {
            say(&format!("unknown op {other}"));
            return;
        }
    };

    let path = PathBuf::from(concat!(env!("CARGO_MANIFEST_DIR"), "/tests/fixtures"))
        .join(format!("occ_reference_{subject}.step"));
    say(&format!("subject {subject}  cutter {kind}  op {op_name}"));

    let start = Instant::now();
    let solids = match StepImporter::import_solids_from_file(&path) {
        Ok(solids) => solids,
        Err(err) => {
            say(&format!("read failed: {err}"));
            return;
        }
    };
    let a = &solids[0];
    say(&format!(
        "read      {:>8.2}s  {} face(s)",
        start.elapsed().as_secs_f64(),
        a.outer_shell.faces.len()
    ));

    let start = Instant::now();
    let mesh = tessellate_solid(a, &params());
    let (low, high) = mesh_bounds(&mesh);
    say(&format!(
        "tessellate {:>7.2}s  {} triangle(s)",
        start.elapsed().as_secs_f64(),
        mesh.indices.len() / 3
    ));

    let start = Instant::now();
    let volume_a = MassCalculator::compute_from_brep(a, &params()).volume;
    say(&format!(
        "V(A)      {:>8.2}s  {volume_a:.6}",
        start.elapsed().as_secs_f64()
    ));

    let b = match cutter(&kind, &low, &high) {
        Ok(b) => b,
        Err(err) => {
            say(&format!("cutter failed: {err}"));
            return;
        }
    };
    say(&format!("cutter    {} face(s)", b.outer_shell.faces.len()));

    let start = Instant::now();
    let result = BooleanEngine::boolean_solids_exact_result_unverified(a, &b, op, &Tolerance::default());
    let seconds = start.elapsed().as_secs_f64();
    match result {
        Ok(result) => {
            let volume: f64 = result
                .solids
                .iter()
                .map(|s| MassCalculator::compute_from_brep(s, &params()).volume)
                .sum();
            say(&format!(
                "boolean   {seconds:>8.2}s  {} solid(s)  volume {volume:.6}",
                result.solids.len()
            ));
        }
        Err(err) => say(&format!("boolean   {seconds:>8.2}s  refused: {err}")),
    }
}
