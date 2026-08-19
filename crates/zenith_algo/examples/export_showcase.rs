//! Writes a curated set of STEP files showing what the kernel currently
//! produces, with the numbers to check them against printed alongside.
//!
//! Run with: cargo run --release -p zenith_algo --example export_showcase
//! Output:   target/showcase/

use std::f64::consts::PI;
use std::fs;
use std::path::Path;

use zenith_algo::{
    BooleanEngine, BooleanOpType, BrepTransform, GearBuilder, HelixBuilder, MassCalculator,
    PrimitiveBuilder, SweepBuilder,
};
use zenith_geom::NurbsCurve3;
use zenith_io::StepExporter;
use zenith_math::{Point3, Tolerance, Transform3, Vec3};
use zenith_tess::TessellationParams;
use zenith_topo::{Edge, OrientedEdge, Solid, Vertex, Wire};

struct Item {
    name: &'static str,
    note: &'static str,
    solid: Solid,
    analytic_volume: Option<f64>,
}

fn rect_wire(cx: f64, half: f64) -> Wire {
    let points = [
        Point3::new(cx - half, -half, 0.0),
        Point3::new(cx + half, -half, 0.0),
        Point3::new(cx + half, half, 0.0),
        Point3::new(cx - half, half, 0.0),
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

fn main() {
    let tol = Tolerance::default();
    let out_dir = Path::new("target/showcase");
    fs::create_dir_all(out_dir).expect("create target/showcase");

    let mut items: Vec<Item> = Vec::new();

    // --- 多パッチに組み直したプリミティブ ---
    items.push(Item {
        name: "01_sphere_r20",
        note: "8 rational patches; used to be one self-wrapping face that OCC read as invalid",
        solid: PrimitiveBuilder::make_sphere(20.0).unwrap(),
        analytic_volume: Some(4.0 / 3.0 * PI * 8000.0),
    });
    items.push(Item {
        name: "02_torus_R30_r10",
        note: "16 rational patches, no degenerate edges",
        solid: PrimitiveBuilder::make_torus(30.0, 10.0).unwrap(),
        analytic_volume: Some(2.0 * PI * PI * 30.0 * 100.0),
    });
    items.push(Item {
        name: "03_cone_r20_r8_h40",
        note: "cap planes trimmed by spline arcs; needed the CURVE() fix to read as a solid",
        solid: PrimitiveBuilder::make_cone(20.0, 8.0, 40.0).unwrap(),
        analytic_volume: Some(PI * 40.0 / 3.0 * (400.0 + 160.0 + 64.0)),
    });

    // --- 3次補間で滑らかになった掃引 ---
    let path = NurbsCurve3::bspline_from_points(
        3,
        vec![
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(30.0, 0.0, 20.0),
            Point3::new(50.0, 40.0, 50.0),
            Point3::new(80.0, 40.0, 90.0),
        ],
    )
    .unwrap();
    items.push(Item {
        name: "04_swept_pipe",
        note: "sections interpolated with a cubic, so the tube is C2 rather than faceted",
        solid: SweepBuilder::sweep_circle_along_curve(&path, 8.0, 24).unwrap(),
        analytic_volume: None,
    });

    items.push(Item {
        name: "05_helix_spring",
        note: "square section swept along a helix, three turns",
        solid: HelixBuilder::sweep_wire_along_helix(
            &rect_wire(25.0, 3.0),
            25.0,
            18.0,
            3.0,
            Point3::new(0.0, 0.0, 0.0),
            Vec3::new(0.0, 0.0, 1.0),
            96,
            &tol,
        )
        .unwrap(),
        analytic_volume: None,
    });

    items.push(Item {
        name: "06_spur_gear_m3_z24",
        note: "involute spur gear, module 3, 24 teeth",
        solid: GearBuilder::make_spur_gear(3.0, 24, 20.0, 12.0, 8.0).unwrap(),
        analytic_volume: None,
    });

    // --- ここからが今回の新機能: 汎用ブーリアンによる穴あけ ---
    let plate = PrimitiveBuilder::make_box(80.0, 60.0, 20.0).unwrap();

    let through = BrepTransform::translate_solid(
        &PrimitiveBuilder::make_cylinder(10.0, 60.0).unwrap(),
        Vec3::new(40.0, 30.0, -20.0),
    );
    items.push(Item {
        name: "07_boolean_through_hole",
        note: "block minus a cylinder, straight through",
        solid: BooleanEngine::boolean_solids_exact_result(
            &plate,
            &through,
            BooleanOpType::Difference,
            &tol,
        )
        .expect("through hole")
        .solids
        .remove(0),
        analytic_volume: Some(80.0 * 60.0 * 20.0 - PI * 100.0 * 20.0),
    });

    let blind = BrepTransform::translate_solid(
        &PrimitiveBuilder::make_cylinder(12.0, 40.0).unwrap(),
        Vec3::new(40.0, 30.0, 8.0),
    );
    items.push(Item {
        name: "08_boolean_blind_hole",
        note: "the drill stops inside, so only the top face is broken through",
        solid: BooleanEngine::boolean_solids_exact_result(
            &plate,
            &blind,
            BooleanOpType::Difference,
            &tol,
        )
        .expect("blind hole")
        .solids
        .remove(0),
        analytic_volume: Some(80.0 * 60.0 * 20.0 - PI * 144.0 * 12.0),
    });

    let rotation = Transform3::from_axis_angle(&Vec3::new(0.0, 1.0, 0.0), std::f64::consts::FRAC_PI_2);
    let sideways = BrepTransform::translate_solid(
        &BrepTransform::transform_solid(
            &PrimitiveBuilder::make_cylinder(8.0, 120.0).unwrap(),
            &rotation,
        )
        .unwrap(),
        Vec3::new(-20.0, 30.0, 10.0),
    );
    items.push(Item {
        name: "09_boolean_cross_hole",
        note: "same operation on the X axis, showing the drill is not axis-locked",
        solid: BooleanEngine::boolean_solids_exact_result(
            &plate,
            &sideways,
            BooleanOpType::Difference,
            &tol,
        )
        .expect("cross hole")
        .solids
        .remove(0),
        analytic_volume: Some(80.0 * 60.0 * 20.0 - PI * 64.0 * 80.0),
    });

    let off_centre = BrepTransform::translate_solid(
        &PrimitiveBuilder::make_cylinder(9.0, 60.0).unwrap(),
        Vec3::new(20.0, 18.0, -20.0),
    );
    items.push(Item {
        name: "10_boolean_off_centre_hole",
        note: "holes are rarely centred in real parts; position does not change the accuracy",
        solid: BooleanEngine::boolean_solids_exact_result(
            &plate,
            &off_centre,
            BooleanOpType::Difference,
            &tol,
        )
        .expect("off-centre hole")
        .solids
        .remove(0),
        analytic_volume: Some(80.0 * 60.0 * 20.0 - PI * 81.0 * 20.0),
    });

    let plug = BrepTransform::translate_solid(
        &PrimitiveBuilder::make_cylinder(10.0, 60.0).unwrap(),
        Vec3::new(40.0, 30.0, -20.0),
    );
    items.push(Item {
        name: "11_boolean_intersection_plug",
        note: "the same pair intersected instead of subtracted: the plug that fills the hole",
        solid: BooleanEngine::boolean_solids_exact_result(
            &plate,
            &plug,
            BooleanOpType::Intersection,
            &tol,
        )
        .expect("plug")
        .solids
        .remove(0),
        analytic_volume: Some(PI * 100.0 * 20.0),
    });

    // --- ブーリアン結果を再びブーリアンに掛ける ---
    let mut bolted = PrimitiveBuilder::make_box(80.0, 60.0, 20.0).unwrap();
    for (x, y) in [(15.0, 15.0), (65.0, 15.0), (65.0, 45.0), (15.0, 45.0)] {
        let cutter = BrepTransform::translate_solid(
            &PrimitiveBuilder::make_cylinder(5.0, 60.0).unwrap(),
            Vec3::new(x, y, -20.0),
        );
        bolted = BooleanEngine::boolean_solids_exact_result(
            &bolted,
            &cutter,
            BooleanOpType::Difference,
            &tol,
        )
        .expect("bolt hole")
        .solids
        .remove(0);
    }
    items.push(Item {
        name: "12_boolean_four_bolt_holes",
        note: "four subtractions in a row, each cutting the result of the last",
        solid: bolted,
        analytic_volume: Some(80.0 * 60.0 * 20.0 - 4.0 * PI * 25.0 * 20.0),
    });

    let pilot_block = PrimitiveBuilder::make_box(40.0, 40.0, 20.0).unwrap();
    let pilot_cut = BrepTransform::translate_solid(
        &PrimitiveBuilder::make_cylinder(5.0, 60.0).unwrap(),
        Vec3::new(20.0, 20.0, -20.0),
    );
    let pilot_done = BooleanEngine::boolean_solids_exact_result(
        &pilot_block,
        &pilot_cut,
        BooleanOpType::Difference,
        &tol,
    )
    .expect("pilot")
    .solids
    .remove(0);
    let bore_cut = BrepTransform::translate_solid(
        &PrimitiveBuilder::make_cylinder(9.0, 40.0).unwrap(),
        Vec3::new(20.0, 20.0, 14.0),
    );
    items.push(Item {
        name: "13_boolean_counterbore",
        note: "a wider shallow cut over an existing hole; the top face gains a ring",
        solid: BooleanEngine::boolean_solids_exact_result(
            &pilot_done,
            &bore_cut,
            BooleanOpType::Difference,
            &tol,
        )
        .expect("counterbore")
        .solids
        .remove(0),
        analytic_volume: Some(
            40.0 * 40.0 * 20.0 - PI * 25.0 * 20.0 - (PI * 81.0 - PI * 25.0) * 6.0,
        ),
    });

    // --- 任意角度の多面体ブーリアン ---
    let base = PrimitiveBuilder::make_box(40.0, 40.0, 40.0).unwrap();
    let turned = BrepTransform::translate_solid(
        &BrepTransform::transform_solid(
            &BrepTransform::translate_solid(&base, Vec3::new(20.0, 20.0, 0.0)),
            &Transform3::from_axis_angle(&Vec3::new(0.0, 0.0, 1.0), std::f64::consts::FRAC_PI_4),
        )
        .unwrap(),
        Vec3::new(0.0, 0.0, 14.0),
    );

    items.push(Item {
        name: "14_boolean_rotated_union",
        note: "two cubes at 45 degrees; the general polyhedral case",
        solid: BooleanEngine::boolean_solids_exact_result(
            &base,
            &turned,
            BooleanOpType::Union,
            &tol,
        )
        .expect("rotated union")
        .solids
        .remove(0),
        analytic_volume: None,
    });

    items.push(Item {
        name: "15_boolean_rotated_difference",
        note: "the same pair subtracted, leaving the bite the turned cube takes",
        solid: BooleanEngine::boolean_solids_exact_result(
            &base,
            &turned,
            BooleanOpType::Difference,
            &tol,
        )
        .expect("rotated difference")
        .solids
        .remove(0),
        analytic_volume: None,
    });

    items.push(Item {
        name: "16_boolean_rotated_intersection",
        note: "and intersected: the region the two cubes share",
        solid: BooleanEngine::boolean_solids_exact_result(
            &base,
            &turned,
            BooleanOpType::Intersection,
            &tol,
        )
        .expect("rotated intersection")
        .solids
        .remove(0),
        analytic_volume: None,
    });

    // --- 出力 ---
    let integration = TessellationParams {
        u_divisions: 48,
        v_divisions: 48,
    };

    println!(
        "{:<32} {:>7} {:>14} {:>12}  {}",
        "file", "faces", "volume", "vs analytic", "note"
    );
    println!("{}", "-".repeat(130));

    for item in &items {
        let path = out_dir.join(format!("{}.step", item.name));
        StepExporter::export_solid_to_file(&item.solid, path.to_str().unwrap(), item.name)
            .unwrap_or_else(|err| panic!("export failed for {}: {err}", item.name));

        let mass = MassCalculator::compute_from_brep(&item.solid, &integration);
        let against = item
            .analytic_volume
            .map(|expected| format!("{:.2e}", (mass.volume - expected).abs() / expected.abs()))
            .unwrap_or_else(|| "-".to_string());

        println!(
            "{:<32} {:>7} {:>14.4} {:>12}  {}",
            format!("{}.step", item.name),
            item.solid.outer_shell.faces.len(),
            mass.volume,
            against,
            item.note
        );
    }

    println!("{}", "-".repeat(130));
    println!("wrote {} file(s) to {}", items.len(), out_dir.display());
}
