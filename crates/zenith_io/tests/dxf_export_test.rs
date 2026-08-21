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
