//! どのビルダーが、平面を NURBS のまま持っているか。
//!
//! 有理 NURBS の像は制御点の凸包に入るので、制御点が1枚の平面に乗っていれば
//! その面は平面である。にもかかわらず `FaceGeometry::Nurbs` で持っていると、
//!
//! - 平面しか受け付けない演算（面の併合、稜のフィレット・面取り）が掛からない
//! - 質量積分が線積分の閉じた経路ではなく求積に落ちる
//! - STEP に `PLANE` ではなく `B_SPLINE_SURFACE` が出る
//!
//! ここは「平面として持つべきなのに NURBS で持っている面」を数える。
//! 1枚でもあれば非ゼロ終了するので、直した後はゲートとして使える。

use zenith_algo::*;
use zenith_math::{Point3, Tolerance, Vec3};
use zenith_topo::{FaceGeometry, Solid};

/// この面の制御点が公差内で同一平面に乗っているか
fn is_really_planar(face: &zenith_topo::Face, tol: &Tolerance) -> bool {
    let FaceGeometry::Nurbs(surface) = &face.geometry else {
        return false;
    };
    let points: Vec<Point3> = surface
        .control_points
        .iter()
        .flat_map(|row| row.iter())
        .map(|control| control.point)
        .collect();
    if points.len() < 3 {
        return false;
    }
    let Some(normal) = surface.normal(0.5, 0.5) else {
        return false;
    };
    let origin = points[0];
    let extent = points
        .iter()
        .map(|point| (*point - origin).norm())
        .fold(0.0_f64, f64::max)
        .max(1.0);
    points
        .iter()
        .all(|point| (*point - origin).dot(&normal).abs() <= tol.linear * extent)
}

fn audit(name: &str, solid: &Solid) -> usize {
    let tol = Tolerance::default();
    let faces = &solid.outer_shell.faces;
    let planes = faces
        .iter()
        .filter(|face| matches!(face.geometry, FaceGeometry::Plane(_)))
        .count();
    let disguised = faces
        .iter()
        .filter(|face| is_really_planar(face, &tol))
        .count();
    let curved = faces.len() - planes - disguised;

    let flag = if disguised > 0 { " <== flat faces held as NURBS" } else { "" };
    println!(
        "{name:<40} {:>3} faces = {:>3} plane + {:>3} curved + {:>3} disguised{flag}",
        faces.len(),
        planes,
        curved,
        disguised
    );
    disguised
}

fn main() {
    let tol = Tolerance::default();
    let mut disguised = 0;

    disguised += audit("box", &PrimitiveBuilder::make_box(20.0, 30.0, 40.0).unwrap());
    disguised += audit(
        "regular_prism 6",
        &PrimitiveBuilder::make_regular_prism(6, 10.0, 25.0).unwrap(),
    );
    disguised += audit(
        "cylinder",
        &PrimitiveBuilder::make_cylinder(10.0, 25.0).unwrap(),
    );
    disguised += audit("cone", &PrimitiveBuilder::make_cone(10.0, 4.0, 20.0).unwrap());
    disguised += audit("sphere", &PrimitiveBuilder::make_sphere(10.0).unwrap());
    disguised += audit("torus", &PrimitiveBuilder::make_torus(12.0, 4.0).unwrap());

    disguised += audit(
        "hole: drilled_box",
        &HoleBuilder::make_drilled_box(40.0, 40.0, 20.0, 8.0).unwrap(),
    );
    disguised += audit(
        "hole: counterbore",
        &HoleBuilder::make_counterbore_hole_box(40.0, 40.0, 20.0, 5.0, 9.0, 6.0).unwrap(),
    );
    disguised += audit(
        "hole: countersink",
        &HoleBuilder::make_countersink_hole_box(40.0, 40.0, 20.0, 3.0, 6.0, 90.0, 20.0, 20.0).unwrap(),
    );
    disguised += audit(
        "hole: hex_nut",
        &HoleBuilder::make_hex_nut(16.0, 5.0, 6.0).unwrap(),
    );

    disguised += audit(
        "shell: hollow_box",
        &ShellBuilder::make_hollow_box(40.0, 30.0, 20.0, 3.0, 1).unwrap(),
    );
    disguised += audit(
        "shell: through_hollow_box",
        &ShellBuilder::make_through_hollow_box(40.0, 30.0, 20.0, 3.0).unwrap(),
    );
    disguised += audit(
        "shelling: open_box",
        &ShellingBuilder::make_open_box(40.0, 30.0, 20.0, 3.0).unwrap(),
    );

    disguised += audit(
        "fillet: box_z_edges",
        &FilletBuilder::fillet_box_z_edges(20.0, 30.0, 40.0, 4.0, &tol).unwrap(),
    );
    disguised += audit(
        "chamfer: box_z_edges",
        &ChamferBuilder::chamfer_box_z_edges(20.0, 30.0, 40.0, 4.0, &tol).unwrap(),
    );
    disguised += audit(
        "direct: fillet_box_single_edge",
        &DirectModeling::fillet_box_single_edge(20.0, 30.0, 40.0, 0, 4.0).unwrap(),
    );
    disguised += audit(
        "direct: chamfer_box_single_edge",
        &DirectModeling::chamfer_box_single_edge(20.0, 30.0, 40.0, 0, 4.0).unwrap(),
    );

    disguised += audit(
        "shaft: stepped",
        &ShaftBuilder::make_stepped_shaft(&[(10.0, 20.0), (6.0, 15.0)]).unwrap(),
    );
    disguised += audit(
        "shaft: keyway",
        &ShaftBuilder::make_shaft_with_keyway(
            &PrimitiveBuilder::make_cylinder(10.0, 40.0).unwrap(),
            10.0,
            6.0,
            3.0,
            20.0,
            10.0,
        )
        .unwrap(),
    );
    disguised += audit(
        "bolt: hex",
        &BoltBuilder::make_hex_bolt(16.0, 10.0, 5.0, 30.0).unwrap(),
    );
    disguised += audit(
        "flange: circular",
        &FlangeBuilder::make_circular_flange(40.0, 8.0, 10.0, 30.0, 4, 4.0).unwrap(),
    );
    disguised += audit(
        "gear: spur m2 z18",
        &GearBuilder::make_spur_gear(2.0, 18, 20.0, 10.0, 0.0).unwrap(),
    );

    let block = PrimitiveBuilder::make_box(40.0, 40.0, 20.0).unwrap();
    let corner = BrepTransform::translate_solid(
        &PrimitiveBuilder::make_box(20.0, 20.0, 20.0).unwrap(),
        Vec3::new(20.0, 20.0, 0.0),
    );
    disguised += audit(
        "boolean: box minus corner",
        &BooleanEngine::boolean_solids_exact(&block, &corner, BooleanOpType::Difference, &tol)
            .unwrap(),
    );

    println!("{}", "-".repeat(96));
    if disguised == 0 {
        println!("no builder hands back a flat face disguised as a NURBS surface");
    } else {
        println!("{disguised} flat face(s) are still held as NURBS surfaces");
        std::process::exit(1);
    }
}
