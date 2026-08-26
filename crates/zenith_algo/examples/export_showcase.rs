//! Writes a curated set of STEP files showing what the kernel currently
//! produces, with the numbers to check them against printed alongside.
//!
//! Run with: cargo run --release -p zenith_algo --example export_showcase
//! Output:   target/showcase/

use std::f64::consts::PI;
use std::fs;
use std::path::Path;
use zenith_algo::StepInterop;

use zenith_algo::{
    BooleanEngine, BooleanOpType, BrepTransform, DraftBuilder, EdgeBlender, ExtrudeBuilder,
    FastenerBuilder, GearBuilder, HelixBuilder, HoleBuilder, LoftBuilder, MassCalculator,
    PrimitiveBuilder, ProfileBuilder, RevolveBuilder, RibBuilder, ShaftBuilder, ShellingBuilder,
    SweepBuilder,
};
use zenith_geom::NurbsCurve3;
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

    let rotation =
        Transform3::from_axis_angle(&Vec3::new(0.0, 1.0, 0.0), std::f64::consts::FRAC_PI_2);
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

    // --- 回転面を平面で切ったもの（このセッションで通るようになった範囲）---
    let cone = PrimitiveBuilder::make_cone(10.0, 4.0, 20.0).expect("cone");
    let cone_cutter = BrepTransform::translate_solid(
        &PrimitiveBuilder::make_box(20.0, 20.0, 20.0).expect("box"),
        Vec3::new(-10.0, -10.0, 10.0),
    );
    let frustum =
        |r0: f64, r1: f64, h: f64| std::f64::consts::PI * h / 3.0 * (r0 * r0 + r0 * r1 + r1 * r1);

    items.push(Item {
        name: "17_cone_union_box",
        note: "a box seated on a cone; the cut is a circle, exact from the control net",
        solid: BooleanEngine::boolean_solids_exact_result(
            &cone,
            &cone_cutter,
            BooleanOpType::Union,
            &tol,
        )
        .expect("cone union box")
        .solids
        .remove(0),
        analytic_volume: Some(frustum(10.0, 4.0, 20.0) + 8000.0 - frustum(7.0, 4.0, 10.0)),
    });

    items.push(Item {
        name: "18_cone_difference_box",
        note: "the cone below that cut, a frustum with the tip taken off",
        solid: BooleanEngine::boolean_solids_exact_result(
            &cone,
            &cone_cutter,
            BooleanOpType::Difference,
            &tol,
        )
        .expect("cone minus box")
        .solids
        .remove(0),
        analytic_volume: Some(frustum(10.0, 4.0, 20.0) - frustum(7.0, 4.0, 10.0)),
    });

    items.push(Item {
        name: "19_cone_intersection_box",
        note: "and the piece above it",
        solid: BooleanEngine::boolean_solids_exact_result(
            &cone,
            &cone_cutter,
            BooleanOpType::Intersection,
            &tol,
        )
        .expect("cone meets box")
        .solids
        .remove(0),
        analytic_volume: Some(frustum(7.0, 4.0, 10.0)),
    });

    // トーラスをスラブで切る。断面は2本の円で、塞ぐのは円環1枚。
    let torus = PrimitiveBuilder::make_torus(12.0, 4.0).expect("torus");
    let slab = BrepTransform::translate_solid(
        &PrimitiveBuilder::make_box(60.0, 60.0, 20.0).expect("slab"),
        Vec3::new(-30.0, -30.0, -2.0),
    );
    let torus_below = {
        let antiderivative = |z: f64| 0.5 * z * (16.0 - z * z).sqrt() + 8.0 * (z / 4.0).asin();
        4.0 * std::f64::consts::PI * 12.0 * (antiderivative(-2.0) - antiderivative(-4.0))
    };
    let torus_whole = 2.0 * std::f64::consts::PI * std::f64::consts::PI * 12.0 * 16.0;

    items.push(Item {
        name: "20_torus_sliced_below",
        note: "a torus cut by a plane; the cap is one annulus, not two discs",
        solid: BooleanEngine::boolean_solids_exact_result(
            &torus,
            &slab,
            BooleanOpType::Difference,
            &tol,
        )
        .expect("torus minus slab")
        .solids
        .remove(0),
        analytic_volume: Some(torus_below),
    });

    items.push(Item {
        name: "21_torus_sliced_above",
        note: "the rest of the same torus, capped by the same annulus the other way",
        solid: BooleanEngine::boolean_solids_exact_result(
            &torus,
            &slab,
            BooleanOpType::Intersection,
            &tol,
        )
        .expect("torus meets slab")
        .solids
        .remove(0),
        analytic_volume: Some(torus_whole - torus_below),
    });

    // 球をスラブで切る。極が退化した三辺パッチの分割が要る。
    let sphere = PrimitiveBuilder::make_sphere(10.0).expect("sphere");
    let sphere_slab = BrepTransform::translate_solid(
        &PrimitiveBuilder::make_box(60.0, 60.0, 40.0).expect("slab"),
        Vec3::new(-30.0, -30.0, -2.0),
    );
    let cap = std::f64::consts::PI * 64.0 * (30.0 - 8.0) / 3.0;

    items.push(Item {
        name: "23_sphere_cap",
        note: "a sphere cut by a plane; the pole is a corner, not an edge",
        solid: BooleanEngine::boolean_solids_exact_result(
            &sphere,
            &sphere_slab,
            BooleanOpType::Difference,
            &tol,
        )
        .expect("sphere minus slab")
        .solids
        .remove(0),
        analytic_volume: Some(cap),
    });

    items.push(Item {
        name: "24_sphere_minus_cap",
        note: "the rest of that sphere, closed by the same disc",
        solid: BooleanEngine::boolean_solids_exact_result(
            &sphere,
            &sphere_slab,
            BooleanOpType::Intersection,
            &tol,
        )
        .expect("sphere meets slab")
        .solids
        .remove(0),
        analytic_volume: Some(4.0 / 3.0 * std::f64::consts::PI * 1000.0 - cap),
    });

    // 穴あけを重ねた板。ブーリアンの結果をさらに加工できることを見せる。
    let plate = PrimitiveBuilder::make_box(60.0, 40.0, 12.0).expect("plate");
    let mut drilled = plate;
    for (x, y, radius) in [
        (12.0, 10.0, 3.0),
        (12.0, 30.0, 3.0),
        (48.0, 10.0, 3.0),
        (48.0, 30.0, 3.0),
        (30.0, 20.0, 8.0),
    ] {
        let drill = BrepTransform::translate_solid(
            &PrimitiveBuilder::make_cylinder(radius, 40.0).expect("drill"),
            Vec3::new(x, y, -14.0),
        );
        drilled = BooleanEngine::boolean_solids_exact_result(
            &drilled,
            &drill,
            BooleanOpType::Difference,
            &tol,
        )
        .expect("plate drilling")
        .solids
        .remove(0);
    }
    let bolt_holes: f64 = 4.0 * std::f64::consts::PI * 9.0 * 12.0;
    let centre_hole = std::f64::consts::PI * 64.0 * 12.0;

    items.push(Item {
        name: "22_drilled_mounting_plate",
        note: "five holes cut one after another, each into the result of the last",
        solid: drilled,
        analytic_volume: Some(60.0 * 40.0 * 12.0 - bolt_holes - centre_hole),
    });

    // 段付き軸の凹円周を、立体全体を作り直さず厳密トーラスで丸める。
    let stepped = ShaftBuilder::make_stepped_shaft(&[(10.0, 12.0), (7.0, 10.0)])
        .expect("showcase stepped shaft");
    let root = EdgeBlender::blendable_edges(&stepped)
        .into_iter()
        .find(|edge| (edge.length - 2.0 * PI * 7.0).abs() < 1e-6)
        .expect("showcase shoulder root");
    let root_fillet = 1.25;
    let rounded_shaft = EdgeBlender::fillet_edge(&stepped, root.edge_id, root_fillet)
        .expect("showcase shoulder-root fillet");
    let root_added = PI
        * (7.0 * root_fillet * root_fillet * (2.0 - PI * 0.5)
            + root_fillet.powi(3) * (5.0 / 3.0 - PI * 0.5));
    items.push(Item {
        name: "25_stepped_shaft_root_fillet",
        note: "a local concave circular fillet; exact torus patches add material at the shoulder",
        solid: rounded_shaft,
        analytic_volume: Some(PI * (10.0f64.powi(2) * 12.0 + 7.0f64.powi(2) * 10.0) + root_added),
    });

    // --- 新機能: スロット（長円柱）プリミティブ ---
    let slot_l = 30.0;
    let slot_r = 10.0;
    let slot_h = 25.0;
    let slot_prism = PrimitiveBuilder::make_slot_prism(slot_l, slot_r, slot_h).expect("slot prism");
    let slot_vol = (2.0 * slot_l * slot_r + PI * slot_r * slot_r) * slot_h;
    items.push(Item {
        name: "26_slot_prism",
        note: "stadium slot column primitive (2 planar sides + 4 rational cylindrical patches + caps)",
        solid: slot_prism.clone(),
        analytic_volume: Some(slot_vol),
    });

    // --- 新機能: スロット天面凸稜の解析ブレンド（面取り） ---
    let rim_edge = EdgeBlender::blendable_edges(&slot_prism)
        .into_iter()
        .find(|edge| (edge.dihedral_angle_deg - 90.0).abs() < 1.0)
        .expect("slot rim edge");
    let chamfer_d = 2.0;
    let (slot_chamfered, _) = EdgeBlender::blend_edge(
        &slot_prism,
        rim_edge.edge_id,
        zenith_algo::BlendKind::Chamfer { distance: chamfer_d },
    )
    .expect("slot rim chamfer");
    let slot_chamfer_removed = slot_l * chamfer_d * chamfer_d + PI * chamfer_d * chamfer_d * (slot_r - chamfer_d / 3.0);
    items.push(Item {
        name: "27_slot_top_rim_chamfer",
        note: "slot top convex rim blended with exact planar and conical chamfer patches",
        solid: slot_chamfered,
        analytic_volume: Some(slot_vol - slot_chamfer_removed),
    });

    // --- 新機能: スロット天面凸稜の解析フィレット ---
    let fillet_r = 2.5;
    let (slot_filleted, _) = EdgeBlender::blend_edge(
        &slot_prism,
        rim_edge.edge_id,
        zenith_algo::BlendKind::Fillet { radius: fillet_r },
    )
    .expect("slot rim fillet");
    let slot_fillet_removed = 2.0 * slot_l * fillet_r * fillet_r * (1.0 - PI * 0.25)
        + PI * ((slot_r - fillet_r) * fillet_r * fillet_r * (2.0 - PI * 0.5) + fillet_r.powi(3) / 3.0);
    items.push(Item {
        name: "28_slot_top_rim_fillet",
        note: "slot top convex rim rounded with exact quarter-cylinder and quarter-torus patches",
        solid: slot_filleted,
        analytic_volume: Some(slot_vol - slot_fillet_removed),
    });

    // --- 新機能: 任意姿勢・接触配置ブーリアン（傾斜円柱差分） ---
    let block = PrimitiveBuilder::make_box(50.0, 50.0, 30.0).expect("block");
    let tilt = Transform3::from_axis_angle(&Vec3::new(1.0, 1.0, 0.0).normalize(), 0.5);
    let tilted_cutter = BrepTransform::translate_solid(
        &BrepTransform::transform_solid(
            &PrimitiveBuilder::make_cylinder(8.0, 60.0).expect("cyl"),
            &tilt,
        )
        .expect("transform"),
        Vec3::new(25.0, 25.0, -10.0),
    );
    let tilted_cut = BooleanEngine::boolean_solids_exact_result(
        &block,
        &tilted_cutter,
        BooleanOpType::Difference,
        &tol,
    )
    .expect("tilted cut")
    .solids
    .remove(0);
    items.push(Item {
        name: "29_boolean_tilted_cylinder_cut",
        note: "tilted cylinder cut through a block in general 3D orientation (100% manifold B-Rep)",
        solid: tilted_cut,
        analytic_volume: None,
    });

    // --- 新機能: スロット貫通穴口ブレンド（面取り・フィレット） ---
    let plate_w = 80.0;
    let plate_d = 60.0;
    let plate_h = 20.0;
    let plate = PrimitiveBuilder::make_box(plate_w, plate_d, plate_h).expect("slotted plate box");
    let slot_tool = BrepTransform::translate_solid(
        &PrimitiveBuilder::make_slot_prism(20.0, 8.0, plate_h + 20.0).expect("slot tool"),
        Vec3::new(plate_w * 0.5, plate_d * 0.5, -10.0),
    );
    let slotted = BooleanEngine::boolean_solids_exact_result(
        &plate,
        &slot_tool,
        BooleanOpType::Difference,
        &tol,
    )
    .expect("slotted plate difference")
    .solids
    .remove(0);

    let top_face = slotted
        .outer_shell
        .faces
        .iter()
        .find(|face| {
            if let zenith_topo::FaceGeometry::Plane(p) = &face.geometry {
                (p.origin.z - plate_h).abs() < 1e-6 && !face.inner_wires.is_empty()
            } else {
                false
            }
        })
        .expect("slotted plate top face");
    let mouth_edge_id = top_face.inner_wires[0].edges[0].edge.id;

    let base_plate_vol = plate_w * plate_d * plate_h;
    let slot_hole_vol = (2.0 * 20.0 * 8.0 + PI * 8.0 * 8.0) * plate_h;
    let slotted_net_vol = base_plate_vol - slot_hole_vol;

    // 30_slot_hole_chamfer
    let chamfer_d = 2.0;
    let (slot_hole_chamfered, _) = EdgeBlender::blend_edge(
        &slotted,
        mouth_edge_id,
        zenith_algo::BlendKind::Chamfer { distance: chamfer_d },
    )
    .expect("slot hole chamfer");
    let hole_chamfer_removed = 20.0 * chamfer_d * chamfer_d + PI * chamfer_d * chamfer_d * (8.0 + chamfer_d / 3.0);
    items.push(Item {
        name: "30_slot_hole_chamfer",
        note: "through-slot hole mouth chamfered with exact planar and conical bevel patches",
        solid: slot_hole_chamfered,
        analytic_volume: Some(slotted_net_vol - hole_chamfer_removed),
    });

    // 31_slot_hole_fillet
    let fillet_r = 2.5;
    let (slot_hole_filleted, _) = EdgeBlender::blend_edge(
        &slotted,
        mouth_edge_id,
        zenith_algo::BlendKind::Fillet { radius: fillet_r },
    )
    .expect("slot hole fillet");
    let hole_fillet_removed = 2.0 * 20.0 * fillet_r * fillet_r * (1.0 - PI * 0.25)
        + PI * ((8.0 + fillet_r) * fillet_r * fillet_r * (2.0 - PI * 0.5) - fillet_r.powi(3) / 3.0);
    items.push(Item {
        name: "31_slot_hole_fillet",
        note: "through-slot hole mouth rounded with exact quarter-cylinder and quarter-torus patches",
        solid: slot_hole_filleted,
        analytic_volume: Some(slotted_net_vol - hole_fillet_removed),
    });

    // --- 新機能: 薄肉中空シェル化（Box, Cylinder, Slot Tray） ---
    // 32_open_box_shell
    let box_dx = 50.0;
    let box_dy = 40.0;
    let box_dz = 25.0;
    let box_t = 2.5;
    let open_box = ShellingBuilder::make_open_box(box_dx, box_dy, box_dz, box_t).expect("open box shell");
    let open_box_vol = (box_dx * box_dy * box_dz) - ((box_dx - 2.0 * box_t) * (box_dy - 2.0 * box_t) * (box_dz - box_t));
    items.push(Item {
        name: "32_open_box_shell",
        note: "thin-wall hollow box container with open top face and uniform wall thickness",
        solid: open_box,
        analytic_volume: Some(open_box_vol),
    });

    // 33_open_cylinder_shell
    let cyl_r = 20.0;
    let cyl_h = 35.0;
    let cyl_t = 2.0;
    let open_cyl = ShellingBuilder::make_open_cylinder(cyl_r, cyl_h, cyl_t).expect("open cylinder shell");
    let open_cyl_vol = (PI * cyl_r * cyl_r * cyl_h) - (PI * (cyl_r - cyl_t) * (cyl_r - cyl_t) * (cyl_h - cyl_t));
    items.push(Item {
        name: "33_open_cylinder_shell",
        note: "thin-wall hollow cylindrical cup with open top rim and exact rational NURBS cavity",
        solid: open_cyl,
        analytic_volume: Some(open_cyl_vol),
    });

    // 34_open_slot_tray_shell
    let slot_t_l = 30.0;
    let slot_t_r = 12.0;
    let slot_t_h = 20.0;
    let slot_t_t = 2.0;
    let open_tray = ShellingBuilder::make_open_slot_prism(slot_t_l, slot_t_r, slot_t_h, slot_t_t).expect("open slot tray");
    let v_tray_out = (2.0 * slot_t_l * slot_t_r + PI * slot_t_r * slot_t_r) * slot_t_h;
    let v_tray_in = (2.0 * slot_t_l * (slot_t_r - slot_t_t) + PI * (slot_t_r - slot_t_t).powi(2)) * (slot_t_h - slot_t_t);
    items.push(Item {
        name: "34_open_slot_tray_shell",
        note: "thin-wall hollow stadium slot tray with open rim and exact rational NURBS cavity",
        solid: open_tray,
        analytic_volume: Some(v_tray_out - v_tray_in),
    });

    // --- 新機能: 2D スケッチプロファイル押出・回転（ProfileBuilder） ---
    // 35_extruded_rounded_rect_with_hole
    let ext_w = 60.0;
    let ext_h = 40.0;
    let ext_cr = 6.0;
    let ext_hr = 10.0;
    let ext_dz = 20.0;
    let ext_outer = ProfileBuilder::make_rounded_rectangle(
        ext_w,
        ext_h,
        ext_cr,
        Point3::new(0.0, 0.0, 0.0),
        Vec3::new(0.0, 0.0, 1.0),
        Vec3::new(1.0, 0.0, 0.0),
    ).expect("ext outer wire");
    let ext_hole = ProfileBuilder::make_circle(
        ext_hr,
        Point3::new(0.0, 0.0, 0.0),
        Vec3::new(0.0, 0.0, 1.0),
        Vec3::new(1.0, 0.0, 0.0),
    ).expect("ext hole wire");
    let ext_solid = ExtrudeBuilder::extrude_face_with_holes(
        &ext_outer,
        &[ext_hole],
        Vec3::new(0.0, 0.0, ext_dz),
        &tol,
    ).expect("extrude rounded rect with hole");
    let ext_area = (ext_w * ext_h) - 4.0 * ext_cr * ext_cr + PI * ext_cr * ext_cr - PI * ext_hr * ext_hr;
    items.push(Item {
        name: "35_extruded_rounded_rect_with_hole",
        note: "extruded rounded rectangle with center circular through-hole from exact 2D profile",
        solid: ext_solid,
        analytic_volume: Some(ext_area * ext_dz),
    });

    // 36_revolved_flanged_cup
    let rev_pts = [
        Point3::new(12.0, 0.0, 0.0),
        Point3::new(28.0, 0.0, 0.0),
        Point3::new(28.0, 0.0, 6.0),
        Point3::new(18.0, 0.0, 6.0),
        Point3::new(18.0, 0.0, 35.0),
        Point3::new(12.0, 0.0, 35.0),
    ];
    let rev_verts: Vec<Vertex> = rev_pts.iter().map(|&p| Vertex::from_point(p)).collect();
    let mut rev_edges = Vec::with_capacity(6);
    for i in 0..6 {
        let next = (i + 1) % 6;
        let line = Edge::line_between(rev_verts[i].clone(), rev_verts[next].clone()).expect("line");
        rev_edges.push(OrientedEdge::forward(line));
    }
    let rev_wire = Wire::new(rev_edges);
    let rev_solid = RevolveBuilder::revolve_wire_solid(
        &rev_wire,
        Point3::new(0.0, 0.0, 0.0),
        Vec3::new(0.0, 0.0, 1.0),
        &tol,
    ).expect("revolve flanged cup");
    let rev_flange_vol = PI * (28.0 * 28.0 - 12.0 * 12.0) * 6.0;
    let rev_neck_vol = PI * (18.0 * 18.0 - 12.0 * 12.0) * (35.0 - 6.0);
    items.push(Item {
        name: "36_revolved_flanged_cup",
        note: "revolved flanged collar cup solid produced by full 360-deg rational NURBS sweep",
        solid: rev_solid,
        analytic_volume: Some(rev_flange_vol + rev_neck_vol),
    });

    // 37_drafted_taper_block
    let draft_dx = 50.0;
    let draft_dy = 35.0;
    let draft_dz = 25.0;
    let draft_angle_deg = 4.0;
    let draft_angle_rad = draft_angle_deg * PI / 180.0;
    let drafted_block = DraftBuilder::make_drafted_block(
        draft_dx,
        draft_dy,
        draft_dz,
        draft_angle_rad,
        &tol,
    ).expect("drafted block");
    let draft_delta = draft_dz * draft_angle_rad.tan();
    let draft_vol = draft_dz * (draft_dx * draft_dy + draft_delta * (draft_dx + draft_dy) + (4.0 / 3.0) * draft_delta * draft_delta);
    items.push(Item {
        name: "37_drafted_taper_block",
        note: "molding drafted block with exact taper angle for mold release",
        solid: drafted_block,
        analytic_volume: Some(draft_vol),
    });

    // 38_multi_section_loft_duct
    let loft_w0 = ProfileBuilder::make_circle(
        22.0,
        Point3::new(0.0, 0.0, 0.0),
        Vec3::new(0.0, 0.0, 1.0),
        Vec3::new(1.0, 0.0, 0.0),
    ).expect("loft circle");
    let loft_w1 = ProfileBuilder::make_rectangle(
        38.0,
        26.0,
        Point3::new(0.0, 0.0, 30.0),
        Vec3::new(0.0, 0.0, 1.0),
        Vec3::new(1.0, 0.0, 0.0),
    ).expect("loft rect");
    let loft_w2 = ProfileBuilder::make_ellipse(
        32.0,
        16.0,
        Point3::new(0.0, 0.0, 60.0),
        Vec3::new(0.0, 0.0, 1.0),
        Vec3::new(1.0, 0.0, 0.0),
    ).expect("loft ellipse");
    let loft_solid = LoftBuilder::loft_solid(&[loft_w0, loft_w1, loft_w2], 2, &tol)
        .expect("loft duct");
    items.push(Item {
        name: "38_multi_section_loft_duct",
        note: "transition duct solid lofted smoothly across 3 distinct profiles (circle -> rect -> ellipse)",
        solid: loft_solid,
        analytic_volume: None,
    });

    // 39_triangular_prism_rib
    let rib_l = 40.0;
    let rib_h = 30.0;
    let rib_t = 8.0;
    let rib_solid = RibBuilder::make_triangular_rib(rib_l, rib_h, rib_t, &tol)
        .expect("triangular prism rib");
    let rib_vol = 0.5 * rib_l * rib_h * rib_t;
    items.push(Item {
        name: "39_triangular_prism_rib",
        note: "triangular gusset prism rib for mechanical bracket stiffening",
        solid: rib_solid,
        analytic_volume: Some(rib_vol),
    });

    // 40_boolean_countersink_hole
    let cs_box_w = 60.0;
    let cs_box_d = 50.0;
    let cs_box_h = 20.0;
    let cs_hole_r = 5.0;
    let cs_sink_r = 9.0;
    let cs_angle_deg = 90.0;
    let cs_cx = 30.0;
    let cs_cy = 25.0;
    let cs_solid = HoleBuilder::make_countersink_hole_box(
        cs_box_w,
        cs_box_d,
        cs_box_h,
        cs_hole_r,
        cs_sink_r,
        cs_angle_deg,
        cs_cx,
        cs_cy,
    ).expect("countersink hole box");
    let cs_half_rad: f64 = (cs_angle_deg * 0.5).to_radians();
    let cs_depth = (cs_sink_r - cs_hole_r) / cs_half_rad.tan();
    let cs_v_box = cs_box_w * cs_box_d * cs_box_h;
    let cs_v_drill = PI * cs_hole_r * cs_hole_r * (cs_box_h - cs_depth);
    let cs_v_cone = (PI * cs_depth / 3.0) * (cs_sink_r * cs_sink_r + cs_sink_r * cs_hole_r + cs_hole_r * cs_hole_r);
    items.push(Item {
        name: "40_boolean_countersink_hole",
        note: "countersunk screw through-hole block combining cylindrical drill and conical bevel cavity",
        solid: cs_solid,
        analytic_volume: Some(cs_v_box - cs_v_drill - cs_v_cone),
    });

    // 41_hex_bolt_head
    let hex_s = 32.0; // 二面幅 S=32
    let hex_h = 18.0;
    let hex_prism = FastenerBuilder::make_hex_prism(hex_s, hex_h, &tol)
        .expect("hex prism");
    let hex_vol = (3.0_f64.sqrt() * 0.5) * hex_s * hex_s * hex_h;
    items.push(Item {
        name: "41_hex_bolt_head",
        note: "standard hexagonal bolt head solid (6 planar side faces + top and bottom hex caps)",
        solid: hex_prism,
        analytic_volume: Some(hex_vol),
    });

    // 42_hex_nut_blank
    let nut_s = 32.0;
    let nut_h = 16.0;
    let nut_r_hole = 8.0; // M16 ナット下穴
    let nut_solid = FastenerBuilder::make_hex_nut_blank(nut_s, nut_h, nut_r_hole, &tol)
        .expect("hex nut blank");
    let nut_vol = (3.0_f64.sqrt() * 0.5) * nut_s * nut_s * nut_h - PI * nut_r_hole * nut_r_hole * nut_h;
    items.push(Item {
        name: "42_hex_nut_blank",
        note: "hexagonal fastener nut blank with central clearance through-hole",
        solid: nut_solid,
        analytic_volume: Some(nut_vol),
    });

    // 43_stepped_shaft_with_keyway
    let shaft_raw = ShaftBuilder::make_stepped_shaft(&[(16.0, 40.0), (12.0, 30.0)])
        .expect("stepped shaft");
    let shaft_keyway = ShaftBuilder::make_shaft_with_keyway(
        &shaft_raw,
        12.0,
        5.0,
        3.0,
        20.0,
        45.0,
    ).expect("shaft with keyway");
    items.push(Item {
        name: "43_stepped_shaft_with_keyway",
        note: "power transmission stepped shaft with parallel drive keyway pocket",
        solid: shaft_keyway,
        analytic_volume: None,
    });

    // 44_socket_head_cap_screw
    let cap_shank_r = 4.0; // M8
    let cap_shank_l = 30.0;
    let cap_head_r = 6.5;
    let cap_head_h = 8.0;
    let cap_socket_s = 6.0;
    let cap_socket_d = 4.0;
    let cap_screw = FastenerBuilder::make_socket_head_cap_screw(
        cap_shank_r,
        cap_shank_l,
        cap_head_r,
        cap_head_h,
        cap_socket_s,
        cap_socket_d,
        &tol,
    ).expect("socket head cap screw");
    let cap_shank_vol = PI * cap_shank_r * cap_shank_r * cap_shank_l;
    let cap_head_vol = PI * cap_head_r * cap_head_r * cap_head_h;
    let cap_socket_vol = (3.0_f64.sqrt() * 0.5) * cap_socket_s * cap_socket_s * cap_socket_d;
    let cap_vol = cap_shank_vol + cap_head_vol - cap_socket_vol;
    items.push(Item {
        name: "44_socket_head_cap_screw",
        note: "JIS/ISO socket head cap screw with cylindrical head and internal hexagonal drive socket",
        solid: cap_screw,
        analytic_volume: Some(cap_vol),
    });

    // 45_plain_washer
    let washer_inner_r = 4.25; // M8 用平座金
    let washer_outer_r = 8.0;
    let washer_t = 1.6;
    let washer_solid = FastenerBuilder::make_plain_washer(
        washer_inner_r,
        washer_outer_r,
        washer_t,
        &tol,
    ).expect("plain washer");
    let washer_vol = PI * (washer_outer_r * washer_outer_r - washer_inner_r * washer_inner_r) * washer_t;
    items.push(Item {
        name: "45_plain_washer",
        note: "JIS/ISO standard plain flat washer ring solid with annular planar caps",
        solid: washer_solid,
        analytic_volume: Some(washer_vol),
    });

    // 46_counterbored_slot_hole
    let cb_slot_box_w = 80.0;
    let cb_slot_box_d = 60.0;
    let cb_slot_box_h = 20.0;
    let cb_slot_l = 20.0;
    let cb_slot_r = 5.0;
    let cb_slot_cb_l = 20.0;
    let cb_slot_cb_r = 8.0;
    let cb_slot_cb_d = 6.0;
    let cb_slot_cx = 40.0;
    let cb_slot_cy = 30.0;
    let cb_slot_solid = HoleBuilder::make_counterbored_slot_box(
        cb_slot_box_w,
        cb_slot_box_d,
        cb_slot_box_h,
        cb_slot_l,
        cb_slot_r,
        cb_slot_cb_l,
        cb_slot_cb_r,
        cb_slot_cb_d,
        cb_slot_cx,
        cb_slot_cy,
    ).expect("counterbored slot box");
    let s_thru = cb_slot_l * (2.0 * cb_slot_r) + PI * cb_slot_r * cb_slot_r;
    let s_cb = cb_slot_cb_l * (2.0 * cb_slot_cb_r) + PI * cb_slot_cb_r * cb_slot_cb_r;
    let cb_slot_vol = (cb_slot_box_w * cb_slot_box_d * cb_slot_box_h) - s_thru * (cb_slot_box_h - cb_slot_cb_d) - s_cb * cb_slot_cb_d;
    items.push(Item {
        name: "46_counterbored_slot_hole",
        note: "mounting base plate with stepped counterbored stadium slot hole for position adjustment",
        solid: cb_slot_solid,
        analytic_volume: Some(cb_slot_vol),
    });

    // 47_flanged_hex_bolt
    let fl_shank_r = 4.0; // M8
    let fl_shank_l = 25.0;
    let fl_flange_r = 8.5;
    let fl_flange_h = 2.0;
    let fl_hex_s = 12.0;
    let fl_hex_h = 6.0;
    let fl_bolt = FastenerBuilder::make_flanged_hex_bolt(
        fl_shank_r,
        fl_shank_l,
        fl_flange_r,
        fl_flange_h,
        fl_hex_s,
        fl_hex_h,
        &tol,
    ).expect("flanged hex bolt");
    let fl_bolt_vol = PI * fl_shank_r * fl_shank_r * fl_shank_l
        + PI * fl_flange_r * fl_flange_r * fl_flange_h
        + (3.0_f64.sqrt() * 0.5) * fl_hex_s * fl_hex_s * fl_hex_h;
    items.push(Item {
        name: "47_flanged_hex_bolt",
        note: "JIS/ISO flanged hexagonal head bolt with cylindrical washer flange and threaded stud",
        solid: fl_bolt,
        analytic_volume: Some(fl_bolt_vol),
    });

    // 48_spring_lock_washer
    let sp_inner_r = 4.25; // M8
    let sp_outer_r = 7.4;
    let sp_t = 2.0;
    let sp_free_h = 3.5;
    let sp_gap_deg = 20.0;
    let sp_washer = FastenerBuilder::make_spring_washer(
        sp_inner_r,
        sp_outer_r,
        sp_t,
        sp_free_h,
        sp_gap_deg,
        &tol,
    ).expect("spring washer");
    items.push(Item {
        name: "48_spring_lock_washer",
        note: "JIS/ISO spring lock washer helical split ring solid with rectangular wire cross-section",
        solid: sp_washer,
        analytic_volume: None,
    });

    // 49_retaining_circlip
    let rr_inner_r = 4.8; // M10 shaft
    let rr_outer_r = 6.2;
    let rr_t = 1.0;
    let rr_gap_deg = 45.0;
    let rr_ring = FastenerBuilder::make_retaining_ring(
        rr_inner_r,
        rr_outer_r,
        rr_t,
        rr_gap_deg,
        &tol,
    ).expect("retaining ring");
    let rr_vol = PI * (rr_outer_r * rr_outer_r - rr_inner_r * rr_inner_r) * rr_t * ((360.0 - rr_gap_deg) / 360.0);
    items.push(Item {
        name: "49_retaining_circlip",
        note: "JIS/ISO C-type external retaining snap ring (circlip) with open gap for shaft retention",
        solid: rr_ring,
        analytic_volume: Some(rr_vol),
    });

    // 50_countersunk_socket_screw
    let cs_shank_r = 4.0; // M8
    let cs_shank_l = 20.0;
    let cs_head_r = 8.0;
    let cs_head_h = 4.4;
    let cs_socket_s = 5.0;
    let cs_socket_d = 2.8;
    let cs_screw = FastenerBuilder::make_countersunk_socket_screw(
        cs_shank_r,
        cs_shank_l,
        cs_head_r,
        cs_head_h,
        cs_socket_s,
        cs_socket_d,
        &tol,
    ).expect("countersunk socket screw");
    let cs_shank_vol = PI * cs_shank_r * cs_shank_r * cs_shank_l;
    let cs_head_vol = (PI / 3.0) * cs_head_h * (cs_head_r * cs_head_r + cs_head_r * cs_shank_r + cs_shank_r * cs_shank_r);
    let cs_socket_vol = (3.0_f64.sqrt() * 0.5) * cs_socket_s * cs_socket_s * cs_socket_d;
    let cs_vol = cs_shank_vol + cs_head_vol - cs_socket_vol;
    items.push(Item {
        name: "50_countersunk_socket_screw",
        note: "JIS/ISO countersunk flat head screw with internal hexagonal drive socket",
        solid: cs_screw,
        analytic_volume: Some(cs_vol),
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
        StepInterop::export_solid_to_file(&item.solid, path.to_str().unwrap(), item.name, &tol)
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
