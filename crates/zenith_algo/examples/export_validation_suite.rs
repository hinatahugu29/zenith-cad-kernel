//! Exports a fixed set of solids to STEP together with the numbers this kernel
//! computes for them, so an independent kernel can be asked the same questions.
//!
//! Pairs with `tools/freecad_cross_validate.py`, which reads the manifest,
//! re-reads every STEP through OpenCASCADE, and reports where the two kernels
//! disagree.
//!
//! Run with: cargo run -p zenith_algo --example export_validation_suite

use std::f64::consts::PI;
use std::fs;
use std::path::Path;
use zenith_algo::StepInterop;

use serde_json::{json, Value};
use zenith_algo::{
    ChamferBuilder, EdgeBlender, FaceMerger, FilletBuilder, GearBuilder, HelixBuilder, HoleBuilder,
    MassCalculator, PrimitiveBuilder, SectionSlicer, ShellingBuilder, SweepBuilder,
};
use zenith_geom::NurbsCurve3;
use zenith_math::{Point3, Tolerance, Vec3};
use zenith_tess::TessellationParams;
use zenith_topo::{Edge, OrientedEdge, Solid, Vertex, Wire};

struct Subject {
    name: &'static str,
    solid: Solid,
    /// Closed-form volume, where one exists.
    analytic_volume: Option<f64>,
    /// A section plane worth cross-checking, with its closed-form area.
    section: Option<(Point3, Vec3, Option<f64>)>,
}

fn main() {
    let tol = Tolerance::default();
    let out_dir = Path::new("target/validation");
    fs::create_dir_all(out_dir).expect("create target/validation");

    let subjects = build_subjects();
    let mut entries: Vec<Value> = Vec::new();

    let integration = TessellationParams {
        u_divisions: 48,
        v_divisions: 48,
    };

    for subject in &subjects {
        let step_path = out_dir.join(format!("{}.step", subject.name));
        StepInterop::export_solid_to_file(
            &subject.solid,
            step_path.to_str().unwrap(),
            &subject.name.to_uppercase(),
            &tol,
        )
        .unwrap_or_else(|err| panic!("STEP export failed for {}: {err}", subject.name));

        let mass = MassCalculator::compute_from_brep(&subject.solid, &integration);
        let shell_report = subject.solid.outer_shell.validate_closed(&tol);

        let section = subject.section.as_ref().map(|(origin, normal, expected)| {
            match SectionSlicer::slice_solid(&subject.solid, *origin, *normal, &tol) {
                Ok(result) => json!({
                    "origin": [origin.x, origin.y, origin.z],
                    "normal": [normal.x, normal.y, normal.z],
                    "area": result.total_area,
                    "perimeter": result.total_perimeter,
                    "loop_count": result.section_wires.len(),
                    "analytic_area": expected,
                    "error": Value::Null,
                }),
                Err(err) => json!({
                    "origin": [origin.x, origin.y, origin.z],
                    "normal": [normal.x, normal.y, normal.z],
                    "analytic_area": expected,
                    "error": err,
                }),
            }
        });

        entries.push(json!({
            "name": subject.name,
            "step_file": step_path.to_string_lossy(),
            "face_count": subject.solid.outer_shell.faces.len(),
            "cavity_count": subject.solid.inner_shells.len(),
            "kernel_volume": mass.volume,
            "kernel_area": mass.surface_area,
            "kernel_center_of_mass": [
                mass.center_of_mass.x,
                mass.center_of_mass.y,
                mass.center_of_mass.z
            ],
            "analytic_volume": subject.analytic_volume,
            "shell_valid": shell_report.is_valid(),
            "shell_errors": shell_report.errors,
            "section": section,
        }));
    }

    let manifest = json!({
        "generated_by": "zenith_algo::examples::export_validation_suite",
        "integration_tessellation": {
            "u_divisions": integration.u_divisions,
            "v_divisions": integration.v_divisions,
        },
        "subjects": entries,
    });

    let manifest_path = out_dir.join("manifest.json");
    fs::write(
        &manifest_path,
        serde_json::to_string_pretty(&manifest).unwrap(),
    )
    .expect("write manifest");

    println!(
        "wrote {} subject(s) and {}",
        subjects.len(),
        manifest_path.display()
    );
}

fn conical_top_fillet_volume(r_bottom: f64, r_top: f64, height: f64, fillet: f64) -> f64 {
    let slope = (r_top - r_bottom) / height;
    let norm = slope.hypot(1.0);
    let centre_radius = r_top - fillet * (norm + slope);
    let centre_z = height - fillet;
    let side_radius = centre_radius + fillet / norm;
    let side_z = centre_z - fillet * slope / norm;
    let side_angle = (-slope).atan();
    let lower =
        PI * side_z * (r_bottom * r_bottom + r_bottom * side_radius + side_radius * side_radius)
            / 3.0;
    let primitive = |angle: f64| {
        let sine = angle.sin();
        let cosine = angle.cos();
        fillet * centre_radius * centre_radius * sine
            + fillet * fillet * centre_radius * (angle + sine * cosine)
            + fillet.powi(3) * (sine - sine.powi(3) / 3.0)
    };
    lower + PI * (primitive(PI * 0.5) - primitive(side_angle))
}

fn build_subjects() -> Vec<Subject> {
    let mut subjects = Vec::new();

    subjects.push(Subject {
        name: "box_20x30x40",
        solid: PrimitiveBuilder::make_box(20.0, 30.0, 40.0).unwrap(),
        analytic_volume: Some(24000.0),
        section: Some((
            Point3::new(0.0, 0.0, 20.0),
            Vec3::new(0.0, 0.0, 1.0),
            Some(600.0),
        )),
    });

    subjects.push(Subject {
        name: "box_diagonal_section",
        solid: PrimitiveBuilder::make_box(20.0, 30.0, 40.0).unwrap(),
        analytic_volume: Some(24000.0),
        section: Some((
            Point3::new(10.0, 15.0, 20.0),
            Vec3::new(1.0, 1.0, 1.0),
            Some(575.0 * 3.0_f64.sqrt()),
        )),
    });

    subjects.push(Subject {
        name: "cylinder_r10_h40",
        solid: PrimitiveBuilder::make_cylinder(10.0, 40.0).unwrap(),
        analytic_volume: Some(PI * 100.0 * 40.0),
        section: Some((
            Point3::new(0.0, 0.0, 20.0),
            Vec3::new(0.0, 0.0, 1.0),
            Some(PI * 100.0),
        )),
    });

    let rim_fillet = 2.0;
    let rim_major = 10.0 - rim_fillet;
    let rim_removed =
        PI * (rim_major * rim_fillet * rim_fillet * (2.0 - PI * 0.5) + rim_fillet.powi(3) / 3.0);
    subjects.push(Subject {
        name: "cylinder_top_fillet_r2",
        solid: FilletBuilder::fillet_cylinder_top_edge(
            10.0,
            40.0,
            rim_fillet,
            &Tolerance::default(),
        )
        .unwrap(),
        analytic_volume: Some(PI * 100.0 * 40.0 - rim_removed),
        section: Some((
            Point3::new(0.0, 0.0, 20.0),
            Vec3::new(0.0, 0.0, 1.0),
            Some(PI * 100.0),
        )),
    });

    let rim_chamfer = 2.0;
    let chamfer_removed = PI * rim_chamfer * rim_chamfer * (10.0 - rim_chamfer / 3.0);
    subjects.push(Subject {
        name: "cylinder_top_chamfer_c2",
        solid: ChamferBuilder::chamfer_cylinder_top_edge(
            10.0,
            40.0,
            rim_chamfer,
            &Tolerance::default(),
        )
        .unwrap(),
        analytic_volume: Some(PI * 100.0 * 40.0 - chamfer_removed),
        section: Some((
            Point3::new(0.0, 0.0, 20.0),
            Vec3::new(0.0, 0.0, 1.0),
            Some(PI * 100.0),
        )),
    });

    subjects.push(Subject {
        name: "sphere_r10",
        solid: PrimitiveBuilder::make_sphere(10.0).unwrap(),
        analytic_volume: Some(4.0 / 3.0 * PI * 1000.0),
        section: Some((
            Point3::new(0.0, 0.0, 0.0),
            Vec3::new(0.0, 0.0, 1.0),
            Some(PI * 100.0),
        )),
    });

    subjects.push(Subject {
        name: "cone_r10_r4_h20",
        solid: PrimitiveBuilder::make_cone(10.0, 4.0, 20.0).unwrap(),
        analytic_volume: Some(PI * 20.0 / 3.0 * (100.0 + 40.0 + 16.0)),
        section: None,
    });

    subjects.push(Subject {
        name: "cone_top_fillet_r1",
        solid: FilletBuilder::fillet_cone_top_edge(10.0, 4.0, 20.0, 1.0, &Tolerance::default())
            .unwrap(),
        analytic_volume: Some(conical_top_fillet_volume(10.0, 4.0, 20.0, 1.0)),
        section: Some((
            Point3::new(0.0, 0.0, 5.0),
            Vec3::new(0.0, 0.0, 1.0),
            Some(PI * 8.5 * 8.5),
        )),
    });

    subjects.push(Subject {
        name: "true_cone_cap_fillet_r1",
        solid: FilletBuilder::fillet_cone_top_edge(0.0, 10.0, 20.0, 1.0, &Tolerance::default())
            .unwrap(),
        analytic_volume: Some(conical_top_fillet_volume(0.0, 10.0, 20.0, 1.0)),
        section: Some((
            Point3::new(0.0, 0.0, 5.0),
            Vec3::new(0.0, 0.0, 1.0),
            Some(PI * 2.5 * 2.5),
        )),
    });

    subjects.push(Subject {
        name: "torus_R12_r4",
        solid: PrimitiveBuilder::make_torus(12.0, 4.0).unwrap(),
        analytic_volume: Some(2.0 * PI * PI * 12.0 * 16.0),
        section: None,
    });

    subjects.push(Subject {
        name: "drilled_box_30x30x15_r5",
        solid: HoleBuilder::make_drilled_box(30.0, 30.0, 15.0, 5.0).unwrap(),
        analytic_volume: Some(30.0 * 30.0 * 15.0 - PI * 25.0 * 15.0),
        section: Some((
            Point3::new(0.0, 0.0, 7.5),
            Vec3::new(0.0, 0.0, 1.0),
            Some(900.0 - PI * 25.0),
        )),
    });

    let drilled = HoleBuilder::make_drilled_box(30.0, 30.0, 15.0, 5.0).unwrap();
    let drilled = FaceMerger::simplify_solid(&drilled, &Tolerance::default())
        .unwrap()
        .0;
    let mouth = EdgeBlender::blendable_edges(&drilled)
        .into_iter()
        .find(|edge| (edge.length - std::f64::consts::TAU * 5.0).abs() < 1e-6)
        .expect("the simplified drilled box has selectable circular mouths");
    let hole_fillet = 1.0;
    let hole_removed = PI
        * (5.0 * hole_fillet * hole_fillet * (2.0 - PI * 0.5)
            + hole_fillet.powi(3) * (5.0 / 3.0 - PI * 0.5));
    subjects.push(Subject {
        name: "drilled_box_mouth_fillet_r1",
        solid: EdgeBlender::fillet_edge(&drilled, mouth.edge_id, hole_fillet).unwrap(),
        analytic_volume: Some(30.0 * 30.0 * 15.0 - PI * 25.0 * 15.0 - hole_removed),
        section: Some((
            Point3::new(0.0, 0.0, 7.5),
            Vec3::new(0.0, 0.0, 1.0),
            Some(900.0 - PI * 25.0),
        )),
    });
    let hole_chamfer = 1.0;
    let hole_chamfer_removed = PI * hole_chamfer * hole_chamfer * (5.0 + hole_chamfer / 3.0);
    subjects.push(Subject {
        name: "drilled_box_mouth_chamfer_c1",
        solid: EdgeBlender::chamfer_edge(&drilled, mouth.edge_id, hole_chamfer).unwrap(),
        analytic_volume: Some(30.0 * 30.0 * 15.0 - PI * 25.0 * 15.0 - hole_chamfer_removed),
        section: Some((
            Point3::new(0.0, 0.0, 7.5),
            Vec3::new(0.0, 0.0, 1.0),
            Some(900.0 - PI * 25.0),
        )),
    });

    if let Ok(solid) = ShellingBuilder::make_open_box(40.0, 30.0, 20.0, 2.0) {
        subjects.push(Subject {
            name: "shelled_open_box",
            solid,
            analytic_volume: None,
            section: None,
        });
    }

    // 直線経路の掃引は厳密に円柱になるので、解析解で決着がつけられる。
    if let Ok(straight) = NurbsCurve3::bspline_from_points(
        3,
        vec![
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(0.0, 0.0, 10.0),
            Point3::new(0.0, 0.0, 20.0),
            Point3::new(0.0, 0.0, 30.0),
        ],
    ) {
        if let Ok(solid) = SweepBuilder::sweep_circle_along_curve(&straight, 5.0, 16) {
            subjects.push(Subject {
                name: "swept_straight_pipe",
                solid,
                analytic_volume: Some(PI * 25.0 * 30.0),
                section: None,
            });
        }
    }

    if let Ok(path) = NurbsCurve3::bspline_from_points(
        3,
        vec![
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(10.0, 0.0, 10.0),
            Point3::new(20.0, 20.0, 25.0),
            Point3::new(30.0, 20.0, 40.0),
        ],
    ) {
        if let Ok(solid) = SweepBuilder::sweep_circle_along_curve(&path, 3.5, 16) {
            subjects.push(Subject {
                name: "swept_pipe",
                solid,
                analytic_volume: None,
                section: None,
            });
        }
    }

    // 螺旋スイープは断面ワイヤを要求するので、一辺 2.0 の正方形断面を組む。
    let tol = Tolerance::default();
    let profile_points = [
        Point3::new(9.0, -1.0, 0.0),
        Point3::new(11.0, -1.0, 0.0),
        Point3::new(11.0, 1.0, 0.0),
        Point3::new(9.0, 1.0, 0.0),
    ];
    let profile_vertices: Vec<Vertex> =
        profile_points.into_iter().map(Vertex::from_point).collect();
    let profile_edges: Vec<OrientedEdge> = (0..4)
        .filter_map(|index| {
            let edge = Edge::line_between(
                profile_vertices[index].clone(),
                profile_vertices[(index + 1) % 4].clone(),
            )
            .ok()?;
            Some(OrientedEdge::forward(edge))
        })
        .collect();

    if profile_edges.len() == 4 {
        let profile = Wire::new(profile_edges);
        if let Ok(solid) = HelixBuilder::sweep_wire_along_helix(
            &profile,
            10.0,
            6.0,
            2.0,
            Point3::new(0.0, 0.0, 0.0),
            Vec3::new(0.0, 0.0, 1.0),
            64,
            &tol,
        ) {
            subjects.push(Subject {
                name: "helix_spring",
                solid,
                analytic_volume: None,
                section: None,
            });
        }
    }

    // 直線の内側ループを持つ中空押し出し。FACE_BOUND の扱いの対照実験になる。
    {
        let tol = Tolerance::default();
        let rect = |half_x: f64, half_y: f64| {
            let points = [
                Point3::new(-half_x, -half_y, 0.0),
                Point3::new(half_x, -half_y, 0.0),
                Point3::new(half_x, half_y, 0.0),
                Point3::new(-half_x, half_y, 0.0),
            ];
            let vertices: Vec<Vertex> = points.into_iter().map(Vertex::from_point).collect();
            let edges = (0..4)
                .filter_map(|index| {
                    let edge = Edge::line_between(
                        vertices[index].clone(),
                        vertices[(index + 1) % 4].clone(),
                    )
                    .ok()?;
                    Some(OrientedEdge::forward(edge))
                })
                .collect();
            Wire::new(edges)
        };

        if let Ok(solid) = zenith_algo::ExtrudeBuilder::extrude_face_with_holes(
            &rect(15.0, 10.0),
            &[rect(8.0, 5.0)],
            Vec3::new(0.0, 0.0, 25.0),
            &tol,
        ) {
            subjects.push(Subject {
                name: "hollow_extrusion",
                solid,
                analytic_volume: Some((30.0 * 20.0 - 16.0 * 10.0) * 25.0),
                section: None,
            });
        }
    }

    // ブーリアンで開けた穴。専用ビルダーではなく汎用演算の結果を検証する。
    if let (Ok(block), Ok(drill)) = (
        PrimitiveBuilder::make_box(40.0, 40.0, 20.0),
        PrimitiveBuilder::make_cylinder(6.0, 60.0),
    ) {
        let drill =
            zenith_algo::BrepTransform::translate_solid(&drill, Vec3::new(20.0, 20.0, -20.0));
        if let Ok(result) = zenith_algo::BooleanEngine::boolean_solids_exact_result(
            &block,
            &drill,
            zenith_algo::BooleanOpType::Difference,
            &Tolerance::default(),
        ) {
            if let Some(solid) = result.solids.into_iter().next() {
                subjects.push(Subject {
                    name: "boolean_drilled_block",
                    solid,
                    analytic_volume: Some(40.0 * 40.0 * 20.0 - PI * 36.0 * 20.0),
                    section: Some((
                        Point3::new(0.0, 0.0, 10.0),
                        Vec3::new(0.0, 0.0, 1.0),
                        Some(1600.0 - PI * 36.0),
                    )),
                });
            }
        }
    }

    // 止まり穴。円柱の底が立体の内部で終わるので、穴が抜ける面は1枚だけ。
    if let (Ok(block), Ok(drill)) = (
        PrimitiveBuilder::make_box(40.0, 40.0, 20.0),
        PrimitiveBuilder::make_cylinder(6.0, 40.0),
    ) {
        let drill = zenith_algo::BrepTransform::translate_solid(&drill, Vec3::new(20.0, 20.0, 8.0));
        if let Ok(result) = zenith_algo::BooleanEngine::boolean_solids_exact_result(
            &block,
            &drill,
            zenith_algo::BooleanOpType::Difference,
            &Tolerance::default(),
        ) {
            if let Some(solid) = result.solids.into_iter().next() {
                subjects.push(Subject {
                    name: "boolean_blind_hole",
                    solid,
                    analytic_volume: Some(40.0 * 40.0 * 20.0 - PI * 36.0 * 12.0),
                    section: None,
                });
            }
        }
    }

    if let Ok(solid) = GearBuilder::make_spur_gear(2.0, 18, 20.0, 8.0, 6.0) {
        subjects.push(Subject {
            name: "spur_gear_m2_z18",
            solid,
            analytic_volume: None,
            section: None,
        });
    }

    subjects
}
