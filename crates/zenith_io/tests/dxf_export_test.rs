use zenith_io::DxfExporter;
use zenith_math::Point3;

#[test]
fn test_dxf_export_contains_lwpolyline() {
    let loop1 = vec![
        Point3::new(0.0, 0.0, 0.0),
        Point3::new(20.0, 0.0, 0.0),
        Point3::new(20.0, 10.0, 0.0),
        Point3::new(0.0, 10.0, 0.0),
    ];

    let loops = vec![loop1];
    let dxf_str = DxfExporter::generate_dxf_string(&loops);

    assert!(dxf_str.contains("LWPOLYLINE"), "DXF must contain LWPOLYLINE entity");
    assert!(dxf_str.contains("AcDbPolyline"), "DXF must contain AcDbPolyline subclass");
    assert!(dxf_str.contains("20.000000"), "DXF must contain coordinate 20.000000");
    assert!(dxf_str.ends_with("0\nEOF\n"), "DXF must terminate with EOF");
}

#[test]
fn test_dxf_export_layered_structure() {
    let outline = vec![
        Point3::new(0.0, 0.0, 0.0),
        Point3::new(50.0, 0.0, 0.0),
        Point3::new(50.0, 50.0, 0.0),
        Point3::new(0.0, 50.0, 0.0),
    ];
    let hole = vec![
        Point3::new(20.0, 20.0, 0.0),
        Point3::new(30.0, 20.0, 0.0),
        Point3::new(30.0, 30.0, 0.0),
        Point3::new(20.0, 30.0, 0.0),
    ];

    let layered = vec![
        (zenith_io::DxfLayer::Outline, outline.as_slice()),
        (zenith_io::DxfLayer::Hole, hole.as_slice()),
    ];

    let dxf_str = DxfExporter::generate_dxf_string_layered(&layered);

    assert!(dxf_str.contains("OUTLINE"), "Must contain OUTLINE layer");
    assert!(dxf_str.contains("HOLE"), "Must contain HOLE layer");
    assert!(dxf_str.contains("CONTINUOUS"), "Must define CONTINUOUS linetype");
    assert!(dxf_str.contains("AC1015"), "Must define AutoCAD AC1015 version header");
}

/// 層は**向き**で決まる。索引ではない。
///
/// `SectionSlicer` は外形を反時計回り、穴を時計回りで返す。以前この
/// エクスポータは「最初のループが外形、残りは全部穴」と決めていたので、
/// 断面に外形が2つ出る形（上から溝を掘った棒を溝の底より上で切る、など）
/// では2つ目の外形が `HOLE` 層に落ちていた。
#[test]
fn the_layer_follows_the_winding_not_the_position_in_the_list() {
    let counter_clockwise = |x: f64| {
        vec![
            Point3::new(x, 0.0, 0.0),
            Point3::new(x + 10.0, 0.0, 0.0),
            Point3::new(x + 10.0, 10.0, 0.0),
            Point3::new(x, 10.0, 0.0),
        ]
    };
    let clockwise = vec![
        Point3::new(2.0, 2.0, 0.0),
        Point3::new(2.0, 8.0, 0.0),
        Point3::new(8.0, 8.0, 0.0),
        Point3::new(8.0, 2.0, 0.0),
    ];

    // 外形2つ。索引で決めていると2つ目が HOLE に落ちる。
    let two_outlines = vec![counter_clockwise(0.0), counter_clockwise(30.0)];
    let dxf = DxfExporter::generate_dxf_string(&two_outlines);
    assert_eq!(
        layer_uses(&dxf, "OUTLINE"),
        2,
        "both counter-clockwise contours belong on OUTLINE:\n{dxf}"
    );
    assert_eq!(layer_uses(&dxf, "HOLE"), 0, "neither contour is a hole");

    // 外形1つ・穴1つ。穴を**先に**渡しても、向きで正しく分かれる。
    let hole_first = vec![clockwise.clone(), counter_clockwise(0.0)];
    let dxf = DxfExporter::generate_dxf_string(&hole_first);
    assert_eq!(layer_uses(&dxf, "OUTLINE"), 1, "the outer contour is on OUTLINE:\n{dxf}");
    assert_eq!(layer_uses(&dxf, "HOLE"), 1, "the clockwise contour is the hole:\n{dxf}");
}

/// エンティティが使っている層の数を数える。TABLES の層定義は数えない。
fn layer_uses(dxf: &str, layer: &str) -> usize {
    let lines: Vec<&str> = dxf.lines().collect();
    let mut count = 0;
    for index in 0..lines.len().saturating_sub(1) {
        if lines[index] == "8" && lines[index + 1] == layer {
            count += 1;
        }
    }
    count
}
