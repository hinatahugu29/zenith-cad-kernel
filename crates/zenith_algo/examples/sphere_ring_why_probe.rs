//! HANDOVER 4-837の続き——`boundary_rings`が球の周期1面(8穴)で
//! Noneを返しているかを直接見るための一時的な検体。

use zenith_algo::{BooleanEngine, BooleanOpType, BrepTransform, PrimitiveBuilder};
use zenith_math::{Tolerance, Vec3};

fn main() {
    let tol = Tolerance::default();
    let radius = 10.0;
    let periodic = PrimitiveBuilder::make_sphere_periodic(radius).unwrap();
    let box_solid = PrimitiveBuilder::make_box(12.0, 12.0, 12.0).unwrap();
    let box_centered = BrepTransform::translate_solid(&box_solid, Vec3::new(-6.0, -6.0, -6.0));

    let result = BooleanEngine::boolean_solids_exact_result_unverified(
        &box_centered,
        &periodic,
        BooleanOpType::Union,
        &tol,
    )
    .unwrap();
    let solid = &result.solids[0];
    let sphere_face = solid
        .outer_shell
        .faces
        .iter()
        .find(|f| matches!(f.geometry, zenith_topo::FaceGeometry::Nurbs(_)))
        .unwrap();
    println!("outer_wire 稜数: {}", sphere_face.outer_wire.edges.len());
    println!("inner_wires 本数: {}", sphere_face.inner_wires.len());
    for (i, w) in sphere_face.inner_wires.iter().enumerate() {
        println!("  内輪{i} 稜数: {}", w.edges.len());
    }
    if let Some(p) = &sphere_face.pcurves {
        println!("pcurves.outer_loop 区間数: {}", p.outer_loop.segments.len());
        println!("pcurves.inner_loops 本数: {}", p.inner_loops.len());
        for (i, l) in p.inner_loops.iter().enumerate() {
            println!("  内loop{i} 区間数: {}", l.segments.len());
        }
    } else {
        println!("pcurves: None");
    }
    println!("sphere face id: {}", sphere_face.id);
    let mesh = zenith_tess::tessellate_solid(solid, &zenith_tess::TessellationParams::default());
    println!("全体三角形数: {}", mesh.indices.len() / 3);
}
