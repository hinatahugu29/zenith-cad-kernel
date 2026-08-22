use zenith_io::{GltfExporter, IgesExporter, ObjExporter, StlExporter};
use zenith_math::{Point3, Vec3};
use zenith_tess::TriangleMesh;
use zenith_topo::Solid;

fn make_test_mesh() -> TriangleMesh {
    let mut mesh = TriangleMesh::new();
    mesh.positions = vec![
        Point3::new(0.0, 0.0, 0.0),
        Point3::new(10.0, 0.0, 0.0),
        Point3::new(10.0, 10.0, 0.0),
        Point3::new(0.0, 10.0, 0.0),
    ];
    mesh.normals = vec![
        Vec3::new(0.0, 0.0, 1.0),
        Vec3::new(0.0, 0.0, 1.0),
        Vec3::new(0.0, 0.0, 1.0),
        Vec3::new(0.0, 0.0, 1.0),
    ];
    mesh.uvs = vec![
        zenith_math::Vec2::new(0.0, 0.0),
        zenith_math::Vec2::new(1.0, 0.0),
        zenith_math::Vec2::new(1.0, 1.0),
        zenith_math::Vec2::new(0.0, 1.0),
    ];
    mesh.indices = vec![[0, 1, 2], [0, 2, 3]];
    mesh
}

#[test]
fn test_obj_export_contains_vertices_normals_and_faces() {
    let mesh = make_test_mesh();
    let obj_str = mesh.to_obj_string("TestQuad");
    assert!(obj_str.contains("o TestQuad"));
    assert!(obj_str.contains("v 0.000000 0.000000 0.000000"));
    assert!(obj_str.contains("v 10.000000 10.000000 0.000000"));
    assert!(obj_str.contains("vn 0.000000 0.000000 1.000000"));
    assert!(obj_str.contains("f 1/1/1 2/2/2 3/3/3"));
    assert!(obj_str.contains("f 1/1/1 3/3/3 4/4/4"));
}

#[test]
fn test_stl_binary_export_structure() {
    let mesh = make_test_mesh();
    let temp_dir = std::env::temp_dir();
    let stl_path = temp_dir.join("zenith_test_export.stl");
    StlExporter::export_binary(&mesh, &stl_path).expect("STL export should succeed");

    let bytes = std::fs::read(&stl_path).expect("read STL bytes");
    assert!(bytes.len() >= 84); // 80 bytes header + 4 bytes tri count
    let tri_count = u32::from_le_bytes(bytes[80..84].try_into().unwrap());
    assert_eq!(tri_count, 2);
    assert_eq!(bytes.len(), 84 + 2 * 50); // each facet is 50 bytes

    let _ = std::fs::remove_file(stl_path);
}

#[test]
fn test_gltf_export_valid_json() {
    let mesh = make_test_mesh();
    let json_str = GltfExporter::export_to_json(&mesh).expect("glTF export should succeed");
    assert!(json_str.contains("\"asset\":"));
    assert!(json_str.contains("\"version\": \"2.0\""));
    assert!(json_str.contains("\"POSITION\""));
    assert!(json_str.contains("\"NORMAL\""));
    assert!(json_str.contains("\"indices\""));
}

#[test]
fn test_obj_export_to_file() {
    let mesh = make_test_mesh();
    let temp_dir = std::env::temp_dir();
    let obj_path = temp_dir.join("zenith_test_export.obj");
    ObjExporter::export_to_file(&mesh, &obj_path, "TestQuad").expect("OBJ file export should succeed");
    let content = std::fs::read_to_string(&obj_path).expect("read OBJ string");
    assert!(content.contains("o TestQuad"));
    let _ = std::fs::remove_file(obj_path);
}

#[test]
fn test_iges_export_sections() {
    let solid = Solid::new(zenith_topo::Shell::new(vec![], true), vec![]);
    let iges_str = IgesExporter::export_solid_to_string(&solid, "TestPart")
        .expect("IGES export should succeed");
    assert!(iges_str.contains("Zenith CAD Kernel IGES 5.3 Export - TestPart"));
    assert!(iges_str.contains("S0000001"));
    assert!(iges_str.contains("G0000001"));
    assert!(iges_str.contains("D      1"));
    assert!(iges_str.contains("P      1"));
    assert!(iges_str.contains("T0000001"));
}
