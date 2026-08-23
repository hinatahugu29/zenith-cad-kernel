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
