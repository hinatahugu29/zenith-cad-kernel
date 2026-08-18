use zenith_algo::{CurvePatchBuilder, PrimitiveBuilder};
use zenith_geom::{ControlPoint3, KnotVector, NurbsCurve3};
use zenith_io::ObjExporter;
use zenith_math::{Point3, Tolerance};
use zenith_tess::{tessellate_face, tessellate_solid, TessellationParams};

#[test]
fn test_curve_patch_and_mesh_export() {
    let tol = Tolerance::default();

    // 1. 空間内に湾曲した4本の3次NURBS曲線を定義（Plasticity風の3Dカーブパッチ）
    // 下辺 c0: (0,0,0) -> (5, 0, 2) -> (10, 0, 0)
    let c0 = NurbsCurve3::new(
        2,
        vec![
            ControlPoint3::unweighted(Point3::new(0.0, 0.0, 0.0)),
            ControlPoint3::unweighted(Point3::new(5.0, 0.0, 2.0)),
            ControlPoint3::unweighted(Point3::new(10.0, 0.0, 0.0)),
        ],
        KnotVector::clamped_uniform(3, 2),
    )
    .unwrap();

    // 上辺 c1: (0,10,0) -> (5, 10, 3) -> (10, 10, 0)
    let c1 = NurbsCurve3::new(
        2,
        vec![
            ControlPoint3::unweighted(Point3::new(0.0, 10.0, 0.0)),
            ControlPoint3::unweighted(Point3::new(5.0, 10.0, 3.0)),
            ControlPoint3::unweighted(Point3::new(10.0, 10.0, 0.0)),
        ],
        KnotVector::clamped_uniform(3, 2),
    )
    .unwrap();

    // 左辺 d0: (0,0,0) -> (0, 5, 1) -> (0, 10, 0)
    let d0 = NurbsCurve3::new(
        2,
        vec![
            ControlPoint3::unweighted(Point3::new(0.0, 0.0, 0.0)),
            ControlPoint3::unweighted(Point3::new(0.0, 5.0, 1.0)),
            ControlPoint3::unweighted(Point3::new(0.0, 10.0, 0.0)),
        ],
        KnotVector::clamped_uniform(3, 2),
    )
    .unwrap();

    // 右辺 d1: (10,0,0) -> (10, 5, -1) -> (10, 10, 0)
    let d1 = NurbsCurve3::new(
        2,
        vec![
            ControlPoint3::unweighted(Point3::new(10.0, 0.0, 0.0)),
            ControlPoint3::unweighted(Point3::new(10.0, 5.0, -1.0)),
            ControlPoint3::unweighted(Point3::new(10.0, 10.0, 0.0)),
        ],
        KnotVector::clamped_uniform(3, 2),
    )
    .unwrap();

    // 2. カーブパッチFaceの自動生成
    let patch_face = CurvePatchBuilder::build_from_4_curves(c0, c1, d0, d1, &tol)
        .expect("CurvePatchBuilder should succeed");

    assert!(patch_face.outer_wire.is_closed(&tol));

    // 3. テッセレーション
    let params = TessellationParams {
        u_divisions: 30,
        v_divisions: 30,
    };
    let mesh = tessellate_face(&patch_face, &params);

    assert_eq!(mesh.num_vertices(), 31 * 31);
    assert_eq!(mesh.num_triangles(), 30 * 30 * 2);

    // 4. OBJ文字列およびファイルへの出力確認
    let obj_str = mesh.to_obj_string("curve_patch");
    assert!(obj_str.contains("v 0.000000 0.000000 0.000000"));
    assert!(obj_str.contains("f "));

    // 成果物フォルダにサンプルOBJを出力
    std::fs::create_dir_all("target/samples").unwrap();
    ObjExporter::export_to_file(&mesh, "target/samples/curved_patch.obj", "curved_patch").unwrap();
}

#[test]
fn test_box_primitive_creation_and_tessellation() {
    let solid = PrimitiveBuilder::make_box(10.0, 20.0, 30.0).expect("make_box failed");
    assert_eq!(solid.outer_shell.faces.len(), 6);

    let params = TessellationParams {
        u_divisions: 2,
        v_divisions: 2,
    };
    let mesh = tessellate_solid(&solid, &params);
    assert!(mesh.num_triangles() > 0);

    ObjExporter::export_to_file(&mesh, "target/samples/box_solid.obj", "box_solid").unwrap();
}
