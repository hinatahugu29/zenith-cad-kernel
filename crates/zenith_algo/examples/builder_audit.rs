//! Audits every solid builder the same way the boolean engine and the slicer
//! were audited: against a closed form where one exists, and against
//! invariants where one does not.
//!
//! The two defect shapes found so far both hide from inside the kernel - an
//! answer that is plausible but wrong, and an integral that never settles - so
//! each builder is checked for a valid closed shell, a positive volume, an
//! integral that stops moving under refinement, and agreement with the
//! analytic volume when it is known.
//!
//! Run with: cargo run --release -p zenith_algo --example builder_audit

use std::f64::consts::PI;

use zenith_algo::{
    ChamferBuilder, ExtrudeBuilder, FilletBuilder, GearBuilder, HelixBuilder, HoleBuilder,
    LoftBuilder, MassCalculator, MirrorBuilder, PatternBuilder, PrimitiveBuilder, RevolveBuilder,
    ShellingBuilder, SweepBuilder,
};
use zenith_geom::NurbsCurve3;
use zenith_math::{Point3, Tolerance, Vec3};
use zenith_tess::TessellationParams;
use zenith_topo::{Edge, OrientedEdge, Solid, Vertex, Wire};

struct Case {
    name: &'static str,
    solid: Result<Solid, String>,
    analytic_volume: Option<f64>,
}

fn volume_at(solid: &Solid, divisions: usize) -> f64 {
    MassCalculator::compute_from_brep(
        solid,
        &TessellationParams {
            u_divisions: divisions,
            v_divisions: divisions,
        },
    )
    .volume
}

fn rect_wire(half_x: f64, half_y: f64, z: f64) -> Wire {
    let points = [
        Point3::new(-half_x, -half_y, z),
        Point3::new(half_x, -half_y, z),
        Point3::new(half_x, half_y, z),
        Point3::new(-half_x, half_y, z),
    ];
    let vertices: Vec<Vertex> = points.into_iter().map(Vertex::from_point).collect();
    let edges = (0..4)
        .map(|index| {
            let edge =
                Edge::line_between(vertices[index].clone(), vertices[(index + 1) % 4].clone())
                    .unwrap();
            OrientedEdge::forward(edge)
        })
        .collect();
    Wire::new(edges)
}

/// A closed profile in the XZ half-plane, for revolving about Z.
fn revolve_profile() -> Wire {
    let points = [
        Point3::new(4.0, 0.0, 0.0),
        Point3::new(8.0, 0.0, 0.0),
        Point3::new(8.0, 0.0, 10.0),
        Point3::new(4.0, 0.0, 10.0),
    ];
    let vertices: Vec<Vertex> = points.into_iter().map(Vertex::from_point).collect();
    let edges = (0..4)
        .map(|index| {
            let edge =
                Edge::line_between(vertices[index].clone(), vertices[(index + 1) % 4].clone())
                    .unwrap();
            OrientedEdge::forward(edge)
        })
        .collect();
    Wire::new(edges)
}

fn build_cases() -> Vec<Case> {
    let tol = Tolerance::default();
    let mut cases = Vec::new();

    cases.push(Case {
        name: "box 20x30x40",
        solid: PrimitiveBuilder::make_box(20.0, 30.0, 40.0),
        analytic_volume: Some(24000.0),
    });
    cases.push(Case {
        name: "cylinder r10 h40",
        solid: PrimitiveBuilder::make_cylinder(10.0, 40.0),
        analytic_volume: Some(PI * 100.0 * 40.0),
    });
    cases.push(Case {
        name: "sphere r10",
        solid: PrimitiveBuilder::make_sphere(10.0),
        analytic_volume: Some(4.0 / 3.0 * PI * 1000.0),
    });
    cases.push(Case {
        name: "cone r10/r4 h20",
        solid: PrimitiveBuilder::make_cone(10.0, 4.0, 20.0),
        analytic_volume: Some(PI * 20.0 / 3.0 * (100.0 + 40.0 + 16.0)),
    });
    cases.push(Case {
        name: "torus R12 r4",
        solid: PrimitiveBuilder::make_torus(12.0, 4.0),
        analytic_volume: Some(2.0 * PI * PI * 12.0 * 16.0),
    });

    // 角丸め: 4本の縦稜が半径 r で丸まると、断面積が (4 - pi) r^2 だけ減る。
    let fillet_radius = 4.0;
    cases.push(Case {
        name: "filleted box 20x30x40 r4",
        solid: FilletBuilder::fillet_box_z_edges(20.0, 30.0, 40.0, fillet_radius, &tol),
        analytic_volume: Some(
            (20.0 * 30.0 - (4.0 - PI) * fillet_radius * fillet_radius) * 40.0,
        ),
    });

    // 面取り: 各角から一辺 c の直角二等辺三角形が落ちる。
    let chamfer = 4.0;
    cases.push(Case {
        name: "chamfered box 20x30x40 c4",
        solid: ChamferBuilder::chamfer_box_z_edges(20.0, 30.0, 40.0, chamfer, &tol),
        analytic_volume: Some((20.0 * 30.0 - 2.0 * chamfer * chamfer) * 40.0),
    });

    cases.push(Case {
        name: "drilled box 30x30x15 r5",
        solid: HoleBuilder::make_drilled_box(30.0, 30.0, 15.0, 5.0),
        analytic_volume: Some(30.0 * 30.0 * 15.0 - PI * 25.0 * 15.0),
    });

    // 薄肉化: 外形から内部空洞を引く。
    cases.push(Case {
        name: "open box 40x30x20 t2",
        solid: ShellingBuilder::make_open_box(40.0, 30.0, 20.0, 2.0),
        analytic_volume: Some(40.0 * 30.0 * 20.0 - 36.0 * 26.0 * 18.0),
    });

    cases.push(Case {
        name: "extruded rectangle 30x20 h25",
        solid: ExtrudeBuilder::extrude_wire(&rect_wire(15.0, 10.0, 0.0), Vec3::new(0.0, 0.0, 25.0), &tol),
        analytic_volume: Some(30.0 * 20.0 * 25.0),
    });

    cases.push(Case {
        name: "hollow extrusion 30x20 - 16x10, h25",
        solid: ExtrudeBuilder::extrude_face_with_holes(
            &rect_wire(15.0, 10.0, 0.0),
            &[rect_wire(8.0, 5.0, 0.0)],
            Vec3::new(0.0, 0.0, 25.0),
            &tol,
        ),
        analytic_volume: Some((30.0 * 20.0 - 16.0 * 10.0) * 25.0),
    });

    // 回転体: 内径4/外径8/高さ10 のリング。
    cases.push(Case {
        name: "revolved ring r4..r8 h10",
        solid: RevolveBuilder::revolve_wire_solid(
            &revolve_profile(),
            Point3::new(0.0, 0.0, 0.0),
            Vec3::new(0.0, 0.0, 1.0),
            &tol,
        ),
        analytic_volume: Some(PI * (64.0 - 16.0) * 10.0),
    });

    // ロフト: 同一断面を2枚重ねると角柱になる。
    cases.push(Case {
        name: "loft between equal squares (prism)",
        solid: LoftBuilder::loft_solid(
            &[rect_wire(10.0, 10.0, 0.0), rect_wire(10.0, 10.0, 30.0)],
            1,
            &tol,
        ),
        analytic_volume: Some(20.0 * 20.0 * 30.0),
    });

    // ミラー: 体積は保存されなければならない。
    if let Ok(box_solid) = PrimitiveBuilder::make_box(20.0, 30.0, 40.0) {
        cases.push(Case {
            name: "mirrored box (volume preserved)",
            solid: MirrorBuilder::mirror_solid(
                &box_solid,
                Point3::new(0.0, 0.0, 0.0),
                Vec3::new(1.0, 0.0, 0.0),
                &tol,
            ),
            analytic_volume: Some(24000.0),
        });

        // パターン: n 個ぶんの体積になる。
        if let Ok(copies) = PatternBuilder::linear_pattern(
            &box_solid,
            Vec3::new(1.0, 0.0, 0.0),
            50.0,
            3,
        ) {
            for (index, copy) in copies.into_iter().enumerate() {
                cases.push(Case {
                    name: match index {
                        0 => "linear pattern copy 0",
                        1 => "linear pattern copy 1",
                        _ => "linear pattern copy 2",
                    },
                    solid: Ok(copy),
                    analytic_volume: Some(24000.0),
                });
            }
        }
    }

    let straight = NurbsCurve3::bspline_from_points(
        3,
        vec![
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(0.0, 0.0, 10.0),
            Point3::new(0.0, 0.0, 20.0),
            Point3::new(0.0, 0.0, 30.0),
        ],
    )
    .unwrap();
    cases.push(Case {
        name: "sweep along a straight path (cylinder)",
        solid: SweepBuilder::sweep_circle_along_curve(&straight, 5.0, 16),
        analytic_volume: Some(PI * 25.0 * 30.0),
    });

    let curved = NurbsCurve3::bspline_from_points(
        3,
        vec![
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(10.0, 0.0, 10.0),
            Point3::new(20.0, 20.0, 25.0),
            Point3::new(30.0, 20.0, 40.0),
        ],
    )
    .unwrap();
    cases.push(Case {
        name: "sweep along a curved path",
        solid: SweepBuilder::sweep_circle_along_curve(&curved, 3.5, 16),
        analytic_volume: None,
    });

    cases.push(Case {
        name: "helix sweep",
        solid: HelixBuilder::sweep_wire_along_helix(
            &rect_wire(1.0, 1.0, 0.0),
            10.0,
            6.0,
            2.0,
            Point3::new(0.0, 0.0, 0.0),
            Vec3::new(0.0, 0.0, 1.0),
            64,
            &tol,
        ),
        analytic_volume: None,
    });

    cases.push(Case {
        name: "spur gear m2 z18",
        solid: GearBuilder::make_spur_gear(2.0, 18, 20.0, 8.0, 6.0),
        analytic_volume: None,
    });

    cases
}

fn main() {
    let tol = Tolerance::default();
    let cases = build_cases();

    println!(
        "{:<42} {:<7} {:>14} {:>12} {:>12}  {}",
        "builder case", "shell", "volume", "converge", "vs analytic", "verdict"
    );
    println!("{}", "-".repeat(118));

    let mut passed = 0usize;
    let mut failed = 0usize;

    for case in &cases {
        let solid = match &case.solid {
            Ok(solid) => solid,
            Err(err) => {
                failed += 1;
                println!(
                    "{:<42} {:<7} {:>14} {:>12} {:>12}  BUILD FAILED: {}",
                    case.name,
                    "-",
                    "-",
                    "-",
                    "-",
                    err.chars().take(40).collect::<String>()
                );
                continue;
            }
        };

        let shell_report = solid.outer_shell.validate_closed(&tol);
        let shell_ok = shell_report.is_valid();

        let coarse = volume_at(solid, 24);
        let fine = volume_at(solid, 96);
        let convergence = if fine.abs() > 1e-12 {
            (fine - coarse).abs() / fine.abs()
        } else {
            f64::INFINITY
        };

        let analytic_error = case
            .analytic_volume
            .map(|expected| (fine - expected).abs() / expected.abs());

        let mut problems = Vec::new();
        if !shell_ok {
            problems.push("shell invalid");
        }
        if !(fine > 0.0) {
            problems.push("volume not positive");
        }
        if convergence > 1e-8 {
            problems.push("does not converge");
        }
        if analytic_error.map(|error| error > 1e-6).unwrap_or(false) {
            problems.push("off analytic");
        }

        if problems.is_empty() {
            passed += 1;
        } else {
            failed += 1;
        }

        println!(
            "{:<42} {:<7} {:>14.6} {:>12.2e} {:>12}  {}",
            case.name,
            if shell_ok { "valid" } else { "INVALID" },
            fine,
            convergence,
            analytic_error
                .map(|error| format!("{error:.2e}"))
                .unwrap_or_else(|| "-".to_string()),
            if problems.is_empty() {
                "ok".to_string()
            } else {
                problems.join(", ")
            }
        );
    }

    println!("{}", "-".repeat(118));
    println!("{passed} of {} builder cases clean, {failed} with problems", cases.len());
}
