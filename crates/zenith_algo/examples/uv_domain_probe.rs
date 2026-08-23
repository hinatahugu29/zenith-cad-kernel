//! Measures how much of each face's parameter domain the tessellator covers.
//!
//! Mass integration runs over the triangulated trim domain, so if the covered
//! area moves when the tessellation is refined, the integral cannot converge.
//! This prints the covered UV area against the full parameter rectangle for
//! every face, at several densities.
//!
//! Run with: cargo run --release -p zenith_algo --example uv_domain_probe

use zenith_algo::{PrimitiveBuilder, SweepBuilder};
use zenith_geom::NurbsCurve3;
use zenith_math::Point3;
use zenith_tess::{face_uv_triangulation, TessellationParams};
use zenith_topo::{FaceGeometry, Solid};

fn covered_uv_area(triangulation: &zenith_tess::UvTriangulation) -> f64 {
    let mut total = 0.0;
    for triangle in &triangulation.triangles {
        let a = triangulation.uvs[triangle[0]];
        let b = triangulation.uvs[triangle[1]];
        let c = triangulation.uvs[triangle[2]];
        total += ((b.x - a.x) * (c.y - a.y) - (c.x - a.x) * (b.y - a.y)).abs() * 0.5;
    }
    total
}

fn probe(name: &str, solid: &Solid) {
    println!("=== {name}");
    for divisions in [12usize, 24, 48, 96] {
        let params = TessellationParams {
            u_divisions: divisions,
            v_divisions: divisions,
        };

        let mut worst_ratio_error: f64 = 0.0;
        let mut worst_face = 0usize;
        let mut total_covered = 0.0;
        let mut total_domain = 0.0;

        for (index, face) in solid.outer_shell.faces.iter().enumerate() {
            let ((u_min, u_max), (v_min, v_max)) = match &face.geometry {
                FaceGeometry::Nurbs(surface) => surface.param_range(),
                _ => continue,
            };
            let domain = (u_max - u_min) * (v_max - v_min);
            if domain <= 0.0 {
                continue;
            }

            let covered = covered_uv_area(&face_uv_triangulation(face, &params));
            total_covered += covered;
            total_domain += domain;

            let ratio_error = (covered / domain - 1.0).abs();
            if ratio_error > worst_ratio_error {
                worst_ratio_error = ratio_error;
                worst_face = index;
            }
        }

        println!(
            "    {divisions:>3} divisions: covered {total_covered:.9} of {total_domain:.9} domain"
            ,
        );
        println!(
            "                  worst face {worst_face} deviates {worst_ratio_error:.3e} from its full domain"
        );
    }
    println!();
}

/// Per-face convergence: if the total oscillates, one face is responsible.
fn per_face_convergence(name: &str, solid: &Solid) {
    println!("=== per-face integral convergence: {name}");
    let densities = [12usize, 24, 48, 96, 192];

    for (index, face) in solid.outer_shell.faces.iter().enumerate() {
        let mut areas = Vec::new();
        let mut volumes = Vec::new();
        for divisions in densities {
            let params = TessellationParams {
                u_divisions: divisions,
                v_divisions: divisions,
            };
            let (area, volume) = zenith_algo::MassCalculator::compute_face_integral(face, &params);
            areas.push(area);
            volumes.push(volume);
        }

        let spread = areas
            .iter()
            .cloned()
            .fold(f64::NEG_INFINITY, f64::max)
            - areas.iter().cloned().fold(f64::INFINITY, f64::min);
        let kind = match &face.geometry {
            FaceGeometry::Plane(_) => "plane",
            FaceGeometry::Nurbs(_) => "nurbs",
            _ => "other",
        };

        let triangle_counts: Vec<usize> = densities
            .iter()
            .map(|divisions| {
                face_uv_triangulation(
                    face,
                    &TessellationParams {
                        u_divisions: *divisions,
                        v_divisions: *divisions,
                    },
                )
                .triangles
                .len()
            })
            .collect();

        let series = areas
            .iter()
            .map(|area| format!("{area:.6}"))
            .collect::<Vec<_>>()
            .join(" ");

        println!("    face {index:>2} ({kind}): spread {spread:.3e}");
        println!("        areas     {series}");
        println!("        triangles {triangle_counts:?}");
        let _ = &volumes;
    }
    println!();
}

fn main() {
    probe(
        "cylinder r10 h40",
        &PrimitiveBuilder::make_cylinder(10.0, 40.0).unwrap(),
    );

    let path = NurbsCurve3::bspline_from_points(
        3,
        vec![
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(10.0, 0.0, 10.0),
            Point3::new(20.0, 20.0, 25.0),
            Point3::new(30.0, 20.0, 40.0),
        ],
    )
    .unwrap();
    let pipe = SweepBuilder::sweep_circle_along_curve(&path, 3.5, 16).unwrap();
    probe("swept pipe", &pipe);
    per_face_convergence("cylinder", &PrimitiveBuilder::make_cylinder(10.0, 40.0).unwrap());
    per_face_convergence("swept pipe", &pipe);
}
