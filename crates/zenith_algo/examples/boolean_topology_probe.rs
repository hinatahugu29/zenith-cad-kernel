//! ブーリアン結果のトポロジーが、稜を実体として共有しているかを見る。
//!
//! シェルが閉じていることは既に確認されている。ここで見るのは、
//! **同じ稜が両側の面から同じ ID で参照されているか**。共有されていない
//! 立体は、閉じてはいても「面の集まり」であって、稜を選んで編集する
//! 演算子（フィレット・面取り・稜の選択）を掛けられない。

use std::collections::BTreeMap;

use zenith_algo::{BooleanEngine, BooleanOpType, BrepTransform, EdgeBlender, PrimitiveBuilder};
use zenith_math::{Point3, Tolerance, Vec3};
use zenith_topo::Solid;

fn report(name: &str, solid: &Solid) -> bool {
    let mut uses: BTreeMap<u64, usize> = BTreeMap::new();
    let mut by_geometry: BTreeMap<(i64, i64, i64, i64, i64, i64), Vec<u64>> = BTreeMap::new();
    let quantize = |p: Point3| {
        (
            (p.x * 1e6).round() as i64,
            (p.y * 1e6).round() as i64,
            (p.z * 1e6).round() as i64,
        )
    };

    for face in &solid.outer_shell.faces {
        for wire in std::iter::once(&face.outer_wire).chain(face.inner_wires.iter()) {
            for oriented in &wire.edges {
                *uses.entry(oriented.edge.id).or_insert(0) += 1;
                let a = quantize(oriented.edge.start_vertex.point);
                let b = quantize(oriented.edge.end_vertex.point);
                let key = if a <= b {
                    (a.0, a.1, a.2, b.0, b.1, b.2)
                } else {
                    (b.0, b.1, b.2, a.0, a.1, a.2)
                };
                let slot = by_geometry.entry(key).or_default();
                if !slot.contains(&oriented.edge.id) {
                    slot.push(oriented.edge.id);
                }
            }
        }
    }

    let distinct_edges = uses.len();
    let shared_twice = uses.values().filter(|count| **count == 2).count();
    let used_once = uses.values().filter(|count| **count == 1).count();
    let duplicated: Vec<_> = by_geometry
        .iter()
        .filter(|(_, ids)| ids.len() > 1)
        .map(|(_, ids)| ids.len())
        .collect();

    println!("--- {name}");
    println!("  faces                 : {}", solid.outer_shell.faces.len());
    println!("  distinct edge ids     : {distinct_edges}");
    println!("  ids used by two faces : {shared_twice}");
    println!("  ids used by one face  : {used_once}");
    println!(
        "  positions carrying more than one id : {} (total extra ids {})",
        duplicated.len(),
        duplicated.iter().map(|n| n - 1).sum::<usize>()
    );
    println!(
        "  closed shell          : {}",
        solid
            .outer_shell
            .validate_closed(&Tolerance::default())
            .is_valid()
    );
    println!(
        "  blendable edges       : {}",
        EdgeBlender::blendable_edges(solid).len()
    );

    let clean = used_once == 0 && duplicated.is_empty() && shared_twice == distinct_edges;
    if !clean {
        println!("  ** this solid is closed but its edges are not shared as entities **");
    }
    clean
}

fn main() {
    let tol = Tolerance::default();
    let boxed = PrimitiveBuilder::make_box(40.0, 40.0, 20.0).unwrap();

    let corner = BrepTransform::translate_solid(
        &PrimitiveBuilder::make_box(20.0, 20.0, 20.0).unwrap(),
        Vec3::new(20.0, 20.0, 0.0),
    );
    let raised = BrepTransform::translate_solid(
        &PrimitiveBuilder::make_box(20.0, 20.0, 30.0).unwrap(),
        Vec3::new(10.0, 10.0, 20.0),
    );
    let overlap = BrepTransform::translate_solid(
        &PrimitiveBuilder::make_box(40.0, 40.0, 20.0).unwrap(),
        Vec3::new(20.0, 20.0, 10.0),
    );
    let bore = BrepTransform::translate_solid(
        &PrimitiveBuilder::make_cylinder(6.0, 20.0).unwrap(),
        Vec3::new(20.0, 20.0, 0.0),
    );
    let boss = BrepTransform::translate_solid(
        &PrimitiveBuilder::make_cylinder(6.0, 30.0).unwrap(),
        Vec3::new(20.0, 20.0, 20.0),
    );

    let cases: Vec<(&str, &Solid, BooleanOpType)> = vec![
        ("box minus corner box", &corner, BooleanOpType::Difference),
        ("box union raised block", &raised, BooleanOpType::Union),
        (
            "box intersect shifted box",
            &overlap,
            BooleanOpType::Intersection,
        ),
        ("box minus a bore", &bore, BooleanOpType::Difference),
        ("box union a boss", &boss, BooleanOpType::Union),
    ];

    let mut clean = report("box (builder output, for reference)", &boxed);
    for (name, tool, op) in cases {
        match BooleanEngine::boolean_solids_exact(&boxed, tool, op, &tol) {
            Ok(result) => clean &= report(name, &result),
            Err(err) => {
                println!("--- {name}\n  refused: {err}");
            }
        }
    }

    println!("{}", "-".repeat(70));
    if clean {
        println!("every result shares its edges as single entities");
    } else {
        println!("at least one result is closed but not edge-shared");
        std::process::exit(1);
    }
}
