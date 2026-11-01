//! 4分割版の結果が、交線の稜をどちら向きに使っているかを見る（4-847）。
use zenith_algo::{BooleanEngine, BooleanOpType, BrepTransform, PrimitiveBuilder};
use zenith_math::{Tolerance, Vec3};
use zenith_topo::FaceGeometry;

fn main() {
    let tol = Tolerance::default();
    let tool = BrepTransform::translate_solid(
        &PrimitiveBuilder::make_box(8.0, 8.0, 8.0).unwrap(),
        Vec3::new(5.0, -4.0, -4.0),
    );
    let sphere = PrimitiveBuilder::make_sphere(10.0).unwrap();
    let result =
        BooleanEngine::boolean_solids_exact(&sphere, &tool, BooleanOpType::Union, &tol).unwrap();
    for (index, face) in result.outer_shell.faces.iter().enumerate() {
        let kind = if matches!(face.geometry, FaceGeometry::Nurbs(_)) {
            "NURBS"
        } else {
            "平面"
        };
        for oriented in &face.outer_wire.edges {
            let mid = oriented.evaluate_normalized(0.5);
            if (mid - zenith_math::Point3::new(8.947381, 1.986046, -4.0)).norm() < 1e-3 {
                let start = oriented.start_vertex().point;
                let end = oriented.end_vertex().point;
                println!(
                    "面{index} {kind}: ({:.3},{:.3},{:.3}) -> ({:.3},{:.3},{:.3})",
                    start.x, start.y, start.z, end.x, end.y, end.z
                );
            }
        }
    }
}
