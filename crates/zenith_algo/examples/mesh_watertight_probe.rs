//! 表示・出力用メッシュが、閉じた三角形メッシュになっているか。
//!
//! B-Rep が閉じていることと、そこから作ったメッシュが閉じていることは別です。
//! 面ごとに三角形分割すると、隣り合う面の境界で**頂点が別々に作られ**、
//! 稜に沿って隙間の空いたメッシュになります。見た目には分かりません。
//!
//! 効くのは出力先です。STL は三角形しか持たないので、稜で開いていれば
//! スライサが「閉じていない」と言い、3Dプリンタに送れません。体積計算も
//! 発散定理が成り立たなくなります。
//!
//! ここで測るのは
//!
//! 1. 各稜（三角形の辺）がちょうど2枚の三角形に共有されているか
//! 2. メッシュの体積が B-Rep の体積とどれだけ違うか
//! 3. 退化三角形（面積0）が混じっていないか

use std::collections::BTreeMap;

use zenith_algo::{
    BooleanEngine, BooleanOpType, BrepTransform, HoleBuilder, MassCalculator, PrimitiveBuilder,
};
use zenith_math::{Point3, Tolerance, Vec3};
use zenith_tess::{TessellationParams, TriangleMesh};
use zenith_topo::Solid;

/// 座標が一致する頂点を1つに束ねてから、辺の使われ方を数える
fn edge_use_counts(mesh: &TriangleMesh, tol: f64) -> BTreeMap<(i64, i64, i64, i64, i64, i64), usize> {
    let quantize = |p: Point3| {
        let scale = 1.0 / tol;
        (
            (p.x * scale).round() as i64,
            (p.y * scale).round() as i64,
            (p.z * scale).round() as i64,
        )
    };

    let mut counts = BTreeMap::new();
    for triangle in &mesh.indices {
        let points: Vec<_> = triangle
            .iter()
            .map(|index| quantize(mesh.positions[*index as usize]))
            .collect();
        for corner in 0..3 {
            let a = points[corner];
            let b = points[(corner + 1) % 3];
            let key = if a <= b {
                (a.0, a.1, a.2, b.0, b.1, b.2)
            } else {
                (b.0, b.1, b.2, a.0, a.1, a.2)
            };
            *counts.entry(key).or_insert(0) += 1;
        }
    }
    counts
}

fn probe(name: &str, solid: &Solid, divisions: usize) -> bool {
    let params = TessellationParams {
        u_divisions: divisions,
        v_divisions: divisions,
    };
    let mesh = if std::env::args().any(|a| a == "--stitched") {
        zenith_tess::tessellate_solid_stitched(solid, &params)
    } else {
        zenith_tess::tessellate_solid(solid, &params)
    };
    let counts = edge_use_counts(&mesh, 1e-6);

    let open = counts.values().filter(|count| **count == 1).count();
    let non_manifold = counts.values().filter(|count| **count > 2).count();

    let degenerate = mesh
        .indices
        .iter()
        .filter(|triangle| {
            let p0 = mesh.positions[triangle[0] as usize];
            let p1 = mesh.positions[triangle[1] as usize];
            let p2 = mesh.positions[triangle[2] as usize];
            (p1 - p0).cross(&(p2 - p0)).norm() <= 1e-12
        })
        .count();

    let brep_volume = MassCalculator::compute_from_brep(
        solid,
        &TessellationParams {
            u_divisions: 64,
            v_divisions: 64,
        },
    )
    .volume;
    let mesh_volume = MassCalculator::compute_from_mesh(&mesh).volume;
    let drift = (mesh_volume - brep_volume).abs() / brep_volume.abs().max(1e-12);

    let watertight = open == 0 && non_manifold == 0;
    println!(
        "{name:<36}{:>8} tris{:>9} edges{:>8} open{:>8} n-mani{:>8} degen   volume drift {drift:.2e}  {}",
        mesh.indices.len(),
        counts.len(),
        open,
        non_manifold,
        degenerate,
        if watertight { "closed" } else { "OPEN" }
    );
    watertight
}

fn main() {
    let tol = Tolerance::default();
    let mut all_closed = true;

    for divisions in [4usize, 6, 8, 10, 12, 16, 20, 24, 32] {
        println!("--- {divisions} divisions per patch");
        all_closed &= probe("box", &PrimitiveBuilder::make_box(20.0, 30.0, 40.0).unwrap(), divisions);
        all_closed &= probe(
            "cylinder",
            &PrimitiveBuilder::make_cylinder(10.0, 25.0).unwrap(),
            divisions,
        );
        all_closed &= probe("sphere", &PrimitiveBuilder::make_sphere(12.0).unwrap(), divisions);
        all_closed &= probe("cone", &PrimitiveBuilder::make_cone(10.0, 4.0, 20.0).unwrap(), divisions);
        all_closed &= probe("torus", &PrimitiveBuilder::make_torus(12.0, 4.0).unwrap(), divisions);
        all_closed &= probe(
            "drilled box",
            &HoleBuilder::make_drilled_box(40.0, 40.0, 20.0, 8.0).unwrap(),
            divisions,
        );

        let block = PrimitiveBuilder::make_box(40.0, 40.0, 20.0).unwrap();
        let corner = BrepTransform::translate_solid(
            &PrimitiveBuilder::make_box(20.0, 20.0, 20.0).unwrap(),
            Vec3::new(20.0, 20.0, 0.0),
        );
        all_closed &= probe(
            "boolean L prism",
            &BooleanEngine::boolean_solids_exact(&block, &corner, BooleanOpType::Difference, &tol)
                .unwrap(),
            divisions,
        );
        println!();
    }

    println!("{}", "-".repeat(120));
    if all_closed {
        println!("every mesh is closed: each edge is shared by exactly two triangles");
    } else {
        println!("at least one mesh is open along its edges; STL from it will not slice");
        println!();
        println!("This is the state of the shipping path today. `--stitched` runs the");
        println!("edge-shared tessellation instead; it closes 4, 8, 16 and 32 divisions but");
        println!("still leaves non-manifold edges at 6, 10, 12, 20 and 24. See HANDOVER 4-34.");
    }
}
