//! 消えた帽子の中の点を、厳密な内外判定とメッシュの両方に訊く（4-843）。
use zenith_algo::{exact_inside, BooleanEngine, BooleanOpType, BrepTransform, PrimitiveBuilder};
use zenith_math::{Point3, Tolerance, Vec3};
use zenith_tess::{tessellate_solid, TessellationParams};

fn ray_inside(p: Point3, m: &zenith_tess::TriangleMesh, d: Vec3) -> bool {
    let mut hits = 0;
    for t in &m.indices {
        let (a, b, c) = (
            m.positions[t[0] as usize],
            m.positions[t[1] as usize],
            m.positions[t[2] as usize],
        );
        let (e1, e2) = (b - a, c - a);
        let h = d.cross(&e2);
        let det = e1.dot(&h);
        if det.abs() < 1e-12 {
            continue;
        }
        let inv = 1.0 / det;
        let s = p - a;
        let u = s.dot(&h) * inv;
        if u < 0.0 || u > 1.0 {
            continue;
        }
        let q = s.cross(&e1);
        let v = d.dot(&q) * inv;
        if v < 0.0 || u + v > 1.0 {
            continue;
        }
        if e2.dot(&q) * inv > 1e-9 {
            hits += 1;
        }
    }
    hits % 2 == 1
}

fn main() {
    let tol = Tolerance::default();
    let tool = BrepTransform::translate_solid(
        &PrimitiveBuilder::make_box(8.0, 8.0, 8.0).unwrap(),
        Vec3::new(-4.0, -4.0, 5.0),
    );
    for (name, s) in [
        ("周期", PrimitiveBuilder::make_sphere_periodic(10.0).unwrap()),
    ] {
        let res = BooleanEngine::boolean_solids_exact_result_unverified(
            &s, &tool, BooleanOpType::Difference, &tol,
        )
        .unwrap();
        let solid = &res.solids[0];
        let mesh = tessellate_solid(solid, &TessellationParams::default());
        println!("--- {name} ---");
        for pt in [
            Point3::new(0.3750, 0.2469, 9.8400),

        ] {
            let exact = exact_inside(pt, solid, &tol);
            let m = [
                ray_inside(pt, &mesh, Vec3::new(1.0, 0.000137, 0.000289).normalize()),
                ray_inside(pt, &mesh, Vec3::new(0.000191, 1.0, -0.000233).normalize()),
                ray_inside(pt, &mesh, Vec3::new(-0.000271, 0.000163, 1.0).normalize()),
            ];
            println!(
                "({:.4},{:.4},{:.4}) 厳密{:?} メッシュ{:?}",
                pt.x, pt.y, pt.z, exact, m
            );
        }
    }
}
