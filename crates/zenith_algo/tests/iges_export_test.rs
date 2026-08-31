//! IGES 5.3 出力が、渡した立体を実際に書いているか。
//!
//! 以前の `test_iges_export_sections` は**空のシェル**を渡し、`"S0000001"` の
//! ような文字列が含まれるかだけを見ていた。エクスポータ側も引数の立体を
//! 一度も読んでいなかったので、両者は矛盾なく通り続けていた。
//!
//! ここでは形に依存する量だけを見る。面の数、レコードの桁数、終端レコードが
//! 数え上げている行数、制御点の座標が本当に入っているか。

use zenith_algo::{HoleBuilder, PrimitiveBuilder};
use zenith_io::IgesExporter;

fn records(iges: &str) -> Vec<&str> {
    iges.lines().collect()
}

fn section_count(iges: &str, section: char) -> usize {
    records(iges)
        .iter()
        .filter(|line| line.len() >= 73 && line.as_bytes()[72] == section as u8)
        .count()
}

#[test]
fn test_every_record_is_eighty_columns() {
    let solid = PrimitiveBuilder::make_box(20.0, 30.0, 40.0).unwrap();
    let iges = IgesExporter::export_solid_to_string(&solid, "BOX").expect("export");
    for (index, line) in records(&iges).iter().enumerate() {
        assert_eq!(
            line.len(),
            80,
            "record {index} is {} columns, IGES fixes them at 80: {line:?}",
            line.len()
        );
    }
}

#[test]
fn test_one_entity_128_per_face() {
    for (name, solid) in [
        ("BOX", PrimitiveBuilder::make_box(20.0, 30.0, 40.0).unwrap()),
        ("CYL", PrimitiveBuilder::make_cylinder(10.0, 40.0).unwrap()),
        ("SPH", PrimitiveBuilder::make_sphere(10.0).unwrap()),
        (
            "DRILLED",
            HoleBuilder::make_drilled_box(30.0, 30.0, 15.0, 5.0).unwrap(),
        ),
    ] {
        let faces = solid.outer_shell.faces.len()
            + solid
                .inner_shells
                .iter()
                .map(|shell| shell.faces.len())
                .sum::<usize>();
        let iges = IgesExporter::export_solid_to_string(&solid, name).expect("export");

        // Directory Entry は1エンティティにつき2行。
        assert_eq!(
            section_count(&iges, 'D'),
            faces * 2,
            "{name}: {faces} faces need {} directory records",
            faces * 2
        );
        assert!(
            section_count(&iges, 'P') >= faces,
            "{name}: every entity needs at least one parameter record"
        );
    }
}

#[test]
fn test_the_terminate_record_counts_what_was_written() {
    let solid = PrimitiveBuilder::make_cylinder(10.0, 40.0).unwrap();
    let iges = IgesExporter::export_solid_to_string(&solid, "CYL").expect("export");
    let lines = records(&iges);
    let terminate = lines.last().expect("a terminate record");

    let counted = |prefix: char, field: usize| -> usize {
        let body = &terminate[..72];
        let start = field * 8;
        let text = &body[start..start + 8];
        assert_eq!(
            text.as_bytes()[0],
            prefix as u8,
            "terminate field {field} must start with {prefix}: {text:?}"
        );
        text[1..].trim().parse::<usize>().expect("a count")
    };

    assert_eq!(counted('S', 0), section_count(&iges, 'S'));
    assert_eq!(counted('G', 1), section_count(&iges, 'G'));
    assert_eq!(counted('D', 2), section_count(&iges, 'D'));
    assert_eq!(counted('P', 3), section_count(&iges, 'P'));
}

/// 立体を変えたら中身が変わること。以前のエクスポータはここで落ちる。
#[test]
fn test_the_output_depends_on_the_solid() {
    let small = PrimitiveBuilder::make_box(10.0, 10.0, 10.0).unwrap();
    let large = PrimitiveBuilder::make_box(200.0, 300.0, 400.0).unwrap();

    let small_iges = IgesExporter::export_solid_to_string(&small, "PART").expect("export");
    let large_iges = IgesExporter::export_solid_to_string(&large, "PART").expect("export");

    assert_ne!(
        small_iges, large_iges,
        "two different boxes must not produce byte-identical IGES"
    );
    assert!(
        large_iges.contains("400.0000000000"),
        "the control points of the larger box must appear in its own file"
    );
}

#[test]
fn test_an_empty_solid_is_refused_rather_than_written() {
    let empty = zenith_topo::Solid::new(zenith_topo::Shell::new(vec![], true), vec![]);
    assert!(
        IgesExporter::export_solid_to_string(&empty, "EMPTY").is_err(),
        "a solid with no faces has nothing to write; saying so beats writing a stub"
    );
}
