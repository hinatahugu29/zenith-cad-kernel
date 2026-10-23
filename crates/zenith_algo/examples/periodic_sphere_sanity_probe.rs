//! `PrimitiveBuilder::make_sphere_periodic`（primitive.rsに追加した新しい
//! 選択肢）が、単独で正しい体積・水密な表示メッシュを返すかの確認。

use zenith_algo::{MassCalculator, PrimitiveBuilder};
use zenith_tess::{TessellationParams, TriangleMesh};

fn edge_use_counts(mesh: &TriangleMesh, tol: f64) -> std::collections::BTreeMap<(i64, i64, i64, i64, i64, i64), usize> {
    let quantize = |p: zenith_math::Point3| {
        let scale = 1.0 / tol;
        ((p.x * scale).round() as i64, (p.y * scale).round() as i64, (p.z * scale).round() as i64)
    };
    let mut counts = std::collections::BTreeMap::new();
    for triangle in &mesh.indices {
        let points: Vec<_> = triangle.iter().map(|index| quantize(mesh.positions[*index as usize])).collect();
        for corner in 0..3 {
            let a = points[corner];
            let b = points[(corner + 1) % 3];
            let key = if a <= b { (a.0, a.1, a.2, b.0, b.1, b.2) } else { (b.0, b.1, b.2, a.0, a.1, a.2) };
            *counts.entry(key).or_insert(0) += 1;
        }
    }
    counts
}

fn main() {
    let radius = 10.0;
    let solid = PrimitiveBuilder::make_sphere_periodic(radius).unwrap();
    let exact = 4.0 / 3.0 * std::f64::consts::PI * radius.powi(3);

    for divisions in [24usize, 64, 128] {
        let params = TessellationParams { u_divisions: divisions, v_divisions: divisions };
        let measured = MassCalculator::compute_volume_from_brep(&solid, &params);
        println!(
            "volume (divisions={divisions}) = {:.6} (exact {:.6}, rel_err {:.3e})",
            measured, exact, (measured - exact).abs() / exact
        );
    }

    for divisions in [8usize, 24, 64] {
        let params = TessellationParams { u_divisions: divisions, v_divisions: divisions };
        let mesh = zenith_tess::tessellate_solid(&solid, &params);
        let counts = edge_use_counts(&mesh, 1e-6);
        let open = counts.values().filter(|c| **c == 1).count();
        let non_manifold = counts.values().filter(|c| **c > 2).count();
        println!(
            "mesh (divisions={divisions}): {} triangles, {} edges, open={open}, non_manifold={non_manifold} {}",
            mesh.indices.len(), counts.len(),
            if open == 0 && non_manifold == 0 { "(watertight)" } else { "(OPEN)" }
        );
    }
}
