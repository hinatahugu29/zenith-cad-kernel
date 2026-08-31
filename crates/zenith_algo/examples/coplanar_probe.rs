//! Finds coplanar face pairs whose regions overlap.
//!
//! Two faces lying in the same plane have no intersection curve, so nothing
//! splits them and both are kept whole - which is why the rotated-box union
//! keeps a region twice. This reports which pairs are in that situation, and
//! whether their outward normals agree or oppose, since that decides what the
//! shared region should become.
//!
//! Run with: cargo run --release -p zenith_algo --example coplanar_probe

use zenith_algo::{BrepTransform, PrimitiveBuilder};
use zenith_math::{Point3, Tolerance, Vec3};
use zenith_topo::{Face, FaceGeometry, Solid};

/// The face's plane as (point on it, outward unit normal).
fn oriented_plane(face: &Face) -> Option<(Point3, Vec3)> {
    let FaceGeometry::Plane(plane) = &face.geometry else {
        return None;
    };
    let normal = if face.orientation.is_forward() {
        plane.normal
    } else {
        -plane.normal
    };
    Some((plane.origin, normal.normalize()))
}

/// Do the two faces' boundary polygons overlap, seen in their shared plane?
fn regions_overlap(a: &Face, b: &Face, origin: Point3, normal: Vec3, tol: &Tolerance) -> bool {
    let seed = if normal.x.abs() < 0.9 {
        Vec3::new(1.0, 0.0, 0.0)
    } else {
        Vec3::new(0.0, 1.0, 0.0)
    };
    let axis_u = (seed - normal * seed.dot(&normal)).normalize();
    let axis_v = normal.cross(&axis_u);

    let project = |face: &Face| -> Vec<(f64, f64)> {
        face.outer_wire
            .sample_points(24)
            .iter()
            .map(|point| {
                let offset = *point - origin;
                (offset.dot(&axis_u), offset.dot(&axis_v))
            })
            .collect()
    };

    let poly_a = project(a);
    let poly_b = project(b);
    if poly_a.len() < 3 || poly_b.len() < 3 {
        return false;
    }

    let inside = |point: (f64, f64), polygon: &[(f64, f64)]| {
        let mut hits = false;
        let mut j = polygon.len() - 1;
        for i in 0..polygon.len() {
            let (xi, yi) = polygon[i];
            let (xj, yj) = polygon[j];
            if (yi > point.1) != (yj > point.1)
                && point.0 < (xj - xi) * (point.1 - yi) / (yj - yi) + xi
            {
                hits = !hits;
            }
            j = i;
        }
        hits
    };

    let _ = tol;
    poly_a.iter().any(|point| inside(*point, &poly_b))
        || poly_b.iter().any(|point| inside(*point, &poly_a))
}

fn report(name: &str, a: &Solid, b: &Solid) {
    let tol = Tolerance::default();
    println!("=== {name}");

    let mut found = 0;
    for (index_a, face_a) in a.outer_shell.faces.iter().enumerate() {
        let Some((origin_a, normal_a)) = oriented_plane(face_a) else {
            continue;
        };
        for (index_b, face_b) in b.outer_shell.faces.iter().enumerate() {
            let Some((origin_b, normal_b)) = oriented_plane(face_b) else {
                continue;
            };

            let parallel = normal_a.cross(&normal_b).norm() <= 1e-9;
            if !parallel {
                continue;
            }
            let offset = (origin_b - origin_a).dot(&normal_a).abs();
            if offset > tol.linear * 10.0 {
                continue;
            }

            let same_side = normal_a.dot(&normal_b) > 0.0;
            if !regions_overlap(face_a, face_b, origin_a, normal_a, &tol) {
                continue;
            }

            found += 1;
            println!(
                "    A{index_a} and B{index_b} are coplanar and overlap; normals {}",
                if same_side { "agree" } else { "oppose" }
            );
        }
    }

    if found == 0 {
        println!("    no coplanar overlapping pairs");
    }
    println!();
}

fn main() {
    let boxa = PrimitiveBuilder::make_box(20.0, 20.0, 20.0).unwrap();

    let rotated = BrepTransform::transform_solid(
        &BrepTransform::translate_solid(&boxa, Vec3::new(10.0, 10.0, 0.0)),
        &zenith_math::Transform3::from_axis_angle(
            &Vec3::new(0.0, 0.0, 1.0),
            std::f64::consts::FRAC_PI_4,
        ),
    )
    .unwrap();
    report("rotated boxes (fails)", &boxa, &rotated);

    let rotated_lifted = BrepTransform::translate_solid(&rotated, Vec3::new(0.0, 0.0, 7.0));
    report(
        "rotated boxes lifted in Z (also fails)",
        &boxa,
        &rotated_lifted,
    );

    let corner = BrepTransform::translate_solid(&boxa, Vec3::new(10.0, 10.0, 10.0));
    report("corner overlap (works)", &boxa, &corner);

    let flush = BrepTransform::translate_solid(&boxa, Vec3::new(20.0, 0.0, 0.0));
    report("flush faces, no interior overlap (works)", &boxa, &flush);
}
