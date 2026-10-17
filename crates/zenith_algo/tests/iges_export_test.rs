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

/// Directory Entry の1行目（型番が先頭8桁）から、ある型番の実体数を数える。
/// **各実体は D セクションを2行使う**ので、1行目だけ（偶数番目）を見る。
fn entity_type_count(iges: &str, entity_type: u32) -> usize {
    let needle = format!("{entity_type:>8}");
    records(iges)
        .iter()
        .enumerate()
        .filter(|(index, line)| {
            index % 2 == 0
                && line.len() >= 73
                && line.as_bytes()[72] == b'D'
                && line.starts_with(&needle)
        })
        .count()
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

        // Directory Entry は1実体につき2行。**4-769 でトリムを書くように
        // なったので、面1枚につき 128 だけでなく 126/142/144 も増える**
        // ——数えるのは「曲面（128）がちょうど面の数だけあるか」だけに絞る。
        assert_eq!(
            entity_type_count(&iges, 128),
            faces,
            "{name}: {faces} faces need {faces} Entity 128 (surface)"
        );
        assert_eq!(
            section_count(&iges, 'D') % 2,
            0,
            "{name}: Directory Entry records must come in pairs"
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

#[test]
fn a_face_with_a_stored_pcurve_gets_trimmed_not_just_the_raw_surface() {
    // 4-769: `pcurves` を持つ面は、曲面（128）だけでなく境界曲線（126）・
    // 曲面上の曲線（142）・トリムされた曲面（144）も書く。
    let solid = PrimitiveBuilder::make_cylinder(10.0, 40.0).unwrap();
    assert!(
        solid.outer_shell.faces.iter().any(|f| f.pcurves.is_some()),
        "この検体は少なくとも1面が pcurves を持つはず"
    );
    let iges = IgesExporter::export_solid_to_string(&solid, "CYL").expect("export");

    assert!(entity_type_count(&iges, 144) > 0, "Entity 144（トリムされた曲面）が1つも無い");
    assert!(entity_type_count(&iges, 142) > 0, "Entity 142（曲面上の曲線）が1つも無い");
    assert!(entity_type_count(&iges, 126) > 0, "Entity 126（境界の2D曲線）が1つも無い");
}

#[test]
fn trimming_does_not_change_which_faces_have_a_surface() {
    // トリムを足す前と、書く曲面（128）の枚数が変わってはいけない——
    // 増えた実体（126/142/144）は境界の情報で、面の数そのものではない。
    let solid = HoleBuilder::make_drilled_box(30.0, 30.0, 15.0, 5.0).unwrap();
    let faces = solid.outer_shell.faces.len();
    let iges = IgesExporter::export_solid_to_string(&solid, "DRILLED").expect("export");
    assert_eq!(entity_type_count(&iges, 128), faces);
}
