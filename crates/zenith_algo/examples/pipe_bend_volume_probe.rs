//! **`pipe_bend` の体積が 78.8% 足りない**のは、どの面か（4-410）。
//!
//! `shape_variety_probe` の `pipe_bend`（他カーネルが書いた、曲がった管）は
//! **1579.136704 のはずが 335.103216** を返します。**2026/08/29 の `3570fb5`
//! （射影に種を渡す）で反転し、10 日間そのまま**でした。**通しテストは
//! ずっと緑**です——この検体はテストではなくプローブにしか出てきません。
//!
//! **種渡しを切っても直りません**（`ZENITH_NO_PCURVE_SEED=1`
//! `ZENITH_NO_BOUNDARY_SEED=1`）。**8/29 以降に、別の原因も重なっています。**
//!
//! ここは**面ごとに切り分ける**ためのものです。体積は面ごとの寄与の和なので、
//! **どの面が落ちているか**が分かれば、次に読む所が決まります。
//!
//! ```bash
//! cargo run --release -p zenith_algo --example pipe_bend_volume_probe
//! ```

use std::path::PathBuf;

use zenith_algo::MassCalculator;
use zenith_io::StepImporter;
use zenith_tess::{tessellate_face, tessellate_solid, TessellationParams};
use zenith_topo::{Face, FaceGeometry, Solid};

fn params() -> TessellationParams {
    TessellationParams {
        u_divisions: 64,
        v_divisions: 64,
    }
}

fn fixture(name: &str) -> PathBuf {
    PathBuf::from(concat!(env!("CARGO_MANIFEST_DIR"), "/tests/fixtures"))
        .join(format!("occ_reference_{name}.step"))
}

fn kind(face: &Face) -> &'static str {
    match &face.geometry {
        FaceGeometry::Plane(_) => "plane",
        FaceGeometry::Nurbs(_) => "nurbs",
        FaceGeometry::Coons(_) => "coons",
        FaceGeometry::Gordon(_) => "gordon",
        FaceGeometry::Triangular(_) => "triangular",
    }
}

/// その面だけを持つ「殻」を作って測ることはできないので、**面ごとの
/// 三角形の数と、その三角形が囲む符号つき体積**を出します。発散定理で
/// 体積は面ごとの寄与の和になるので、**足りない面がそのまま見えます**。
fn signed_volume_of(triangles: &[[zenith_math::Point3; 3]]) -> f64 {
    triangles
        .iter()
        .map(|[a, b, c]| {
            let ab = *b - *a;
            let ac = *c - *a;
            (a.coords).dot(&ab.cross(&ac)) / 6.0
        })
        .sum()
}

fn main() {
    let want = 1579.136704_f64;

    let solids = match StepImporter::import_solids_from_file(&fixture("pipe_bend")) {
        Ok(solids) => solids,
        Err(err) => {
            println!("読めませんでした: {err}");
            return;
        }
    };
    let Some(solid) = solids.into_iter().next() else {
        println!("立体が 1 つも返りませんでした");
        return;
    };

    let volume = MassCalculator::compute_from_brep(&solid, &params()).volume;
    println!("pipe_bend（他カーネルが書いた、曲がった管）");
    println!();
    println!("  面 {} 枚", solid.outer_shell.faces.len());
    println!("  閉じた式（OCC が返した値）  {want:.6}");
    println!(
        "  compute_from_brep           {volume:.6}   相対差 {:.3e}",
        (volume - want).abs() / want
    );
    println!();

    report_faces(&solid);
}

fn report_faces(solid: &Solid) {
    println!(
        "{:<5}{:<12}{:>8}{:>10}{:>18}{:>16}",
        "面", "曲面", "外周", "内側の輪", "三角形", "符号つき体積"
    );
    println!("{}", "-".repeat(72));

    let mesh = tessellate_solid(solid, &params());
    // 面ごとに分けて数えられないときは、全体だけ出します。
    let total = signed_volume_of(&collect_triangles(&mesh));
    for (index, face) in solid.outer_shell.faces.iter().enumerate() {
        let mesh = tessellate_face(face, &params());
        let triangles = collect_triangles(&mesh);
        let (tris, contribution) = (triangles.len(), signed_volume_of(&triangles));
        println!(
            "{:<5}{:<12}{:>8}{:>10}{:>18}{:>16.6}",
            index,
            kind(face),
            face.outer_wire.edges.len(),
            face.inner_wires.len(),
            tris,
            contribution
        );
    }
    println!("{}", "-".repeat(72));
    println!("{:<5}{:<12}{:>8}{:>10}{:>18}{:>16.6}", "", "", "", "", "", total);
}

fn collect_triangles(mesh: &zenith_tess::TriangleMesh) -> Vec<[zenith_math::Point3; 3]> {
    mesh.indices
        .iter()
        .map(|tri| {
            [
                mesh.positions[tri[0] as usize],
                mesh.positions[tri[1] as usize],
                mesh.positions[tri[2] as usize],
            ]
        })
        .collect()
}
