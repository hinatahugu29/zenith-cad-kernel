//! 検体の境界箱を、**プローブが実際に使う数字で**出す。
//!
//! `foreign_boolean_probe` は切り手を境界箱から置きます。その箱はメッシュの
//! 頂点から取ったもので、OpenCASCADE 側の `BoundBox` とは限りません
//! （OCC の箱には余裕（gap）が入りますし、丸い形はメッシュで内側に寄ります）。
//!
//! **箱が違えば、同じ名前の配置が別の配置になります。** 2つのカーネルの
//! 答えを並べる前に、ここを合わせないと、差が欠陥なのか置き方なのか
//! 分かりません。`tools/occ_cut_reference.py --box ...` にそのまま渡せる形で
//! 出します。
//!
//! ```bash
//! cargo run --release -p zenith_algo --example cutter_placement_probe
//! ```

use std::path::PathBuf;

use zenith_io::StepImporter;
use zenith_math::Point3;
use zenith_tess::{tessellate_solid, TessellationParams};

/// **`foreign_boolean_probe` と同じ刻み。** ここが違うと別の箱になります。
fn params() -> TessellationParams {
    TessellationParams {
        u_divisions: 64,
        v_divisions: 64,
    }
}

const SUBJECTS: [&str; 10] = [
    "cone",
    "cone_full",
    "cylinder",
    "cylinder_nurbs",
    "sphere",
    "sphere_capped",
    "torus",
    "torus_segment",
    "revolved_ring",
    "extruded_spline",
];

fn main() {
    for subject in SUBJECTS {
        let path = PathBuf::from(concat!(env!("CARGO_MANIFEST_DIR"), "/tests/fixtures/"))
            .join(format!("occ_reference_{subject}.step"));
        let Ok(solids) = StepImporter::import_solids_from_file(&path) else {
            println!("{subject:<18} unreadable");
            continue;
        };
        let Some(solid) = solids.into_iter().next() else {
            println!("{subject:<18} no solid");
            continue;
        };
        let mesh = tessellate_solid(&solid, &params());
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
        println!(
            "{subject:<18} --box {:.10} {:.10} {:.10} {:.10} {:.10} {:.10}",
            low.x, low.y, low.z, high.x, high.y, high.z
        );
    }
}
