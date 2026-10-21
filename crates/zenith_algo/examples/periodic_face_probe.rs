//! **ROADMAP 柱③の次の一手①の検証**（4-790の引き渡し）。
//!
//! `make_torus_patches`（`primitive.rs`）のコメントは「単一面が自分自身に
//! 巻き付く表現（シーム辺を1本だけ持つ1面シェル）はOpenCASCADEで体積0の
//! 不正ソリッドとして読まれる」と言う。**だが`occ_reference_cylinder_nurbs.step`
//! （OCCT自身が書いた参照ファイル）を読むと、円柱の側面は実際に
//! `B_SPLINE_SURFACE(...,.T.,.F.,.F.)`（u_closed=.T.）の**1枚の面**として
//! 書かれており、その`EDGE_LOOP`は4辺——`[底円(逆), シーム(逆), 上円(順),
//! シーム(順)]`——**同じシーム稜を向きを変えて2回使っている**。
//!
//! 「シーム辺を1本だけ持つ」という前提そのものが違うかもしれない——
//! **シーム辺を2回使う正しい構成なら、zenithが書いても読めるのでは**、
//! というのがこの検証。
//!
//! 使い方:
//!     cargo run -p zenith_algo --example periodic_face_probe
//!     # 出力: target/periodic_face_probe/cylinder_side_only.step
//!     # その後 FreeCAD で読み、Volume（閉じたシェルではないので0のはず
//!     # だが、少なくとも面が1枚・Shapeが壊れずに読めるかを見る）を見る。

use std::f64::consts::FRAC_1_SQRT_2;
use std::fs;
use std::path::PathBuf;

use zenith_geom::{nurbs_curve::ControlPoint3, nurbs_curve::NurbsCurve3, KnotVector, NurbsSurface3};
use zenith_io::StepExporter;
use zenith_math::Point3;
use zenith_topo::{Edge, Face, FaceGeometry, Orientation, OrientedEdge, Shell, Solid, Vertex, Wire};

fn closed_circle_points(radius: f64, z: f64) -> Vec<ControlPoint3> {
    let w = FRAC_1_SQRT_2;
    let p0 = Point3::new(radius, 0.0, z);
    let c0 = Point3::new(radius, radius, z);
    let p1 = Point3::new(0.0, radius, z);
    let c1 = Point3::new(-radius, radius, z);
    let p2 = Point3::new(-radius, 0.0, z);
    let c2 = Point3::new(-radius, -radius, z);
    let p3 = Point3::new(0.0, -radius, z);
    let c3 = Point3::new(radius, -radius, z);
    vec![
        ControlPoint3::unweighted(p0),
        ControlPoint3::new(c0, w),
        ControlPoint3::unweighted(p1),
        ControlPoint3::new(c1, w),
        ControlPoint3::unweighted(p2),
        ControlPoint3::new(c2, w),
        ControlPoint3::unweighted(p3),
        ControlPoint3::new(c3, w),
        ControlPoint3::unweighted(p0),
    ]
}

fn closed_circle_knots() -> KnotVector {
    KnotVector::new(vec![
        0.0, 0.0, 0.0, 0.25, 0.25, 0.5, 0.5, 0.75, 0.75, 1.0, 1.0, 1.0,
    ])
}

fn main() {
    let radius = 10.0;
    let height = 20.0;
    let tol = 1e-6;

    let bottom_points = closed_circle_points(radius, 0.0);
    let top_points = closed_circle_points(radius, height);
    let knots_u = closed_circle_knots();

    // 曲面: u方向（周方向）は閉じた9制御点、v方向は底・天の2制御点のみ。
    let mut grid = Vec::with_capacity(9);
    for i in 0..9 {
        grid.push(vec![bottom_points[i].clone(), top_points[i].clone()]);
    }
    let surface = NurbsSurface3::new(2, 1, grid, knots_u.clone(), KnotVector::clamped_uniform(2, 1))
        .expect("surface construction failed");

    // 境界の頂点: 底の θ=0 点、天の θ=0 点。それぞれ自分自身へ戻る
    // 閉じた円エッジ（start==end）の両端として使う。
    let v_bottom = Vertex::new(Point3::new(radius, 0.0, 0.0), tol);
    let v_top = Vertex::new(Point3::new(radius, 0.0, height), tol);

    let bottom_curve = NurbsCurve3::new(2, bottom_points, knots_u.clone()).unwrap();
    let top_curve = NurbsCurve3::new(2, top_points, knots_u).unwrap();

    let bottom_edge = Edge::new(bottom_curve, v_bottom.clone(), v_bottom.clone(), tol);
    let top_edge = Edge::new(top_curve, v_top.clone(), v_top.clone(), tol);
    let seam_edge = Edge::line_between(v_bottom.clone(), v_top.clone())
        .expect("seam edge construction failed");

    // **OCCT自身の参照ファイルと同じ並び**: 底(逆)・シーム(逆)・天(順)・シーム(順)。
    let outer_wire = Wire::new(vec![
        OrientedEdge::new(bottom_edge, Orientation::Reversed),
        OrientedEdge::new(seam_edge.clone(), Orientation::Reversed),
        OrientedEdge::new(top_edge, Orientation::Forward),
        OrientedEdge::new(seam_edge, Orientation::Forward),
    ]);

    let face = Face::new(
        FaceGeometry::Nurbs(surface),
        outer_wire,
        Vec::new(),
        Orientation::Forward,
        tol,
    );

    // **閉じたソリッドではない**（蓋が無い、側面1枚だけ）。`Solid::new`は
    // `is_closed`が立っていないと`assert!`で落ちるため、ここでは診断用に
    // `true`と偽る——**この1枚だけの"ソリッド"は体積の検証には使わない**。
    // 見たいのは「この1枚の面がOCCTに面として読めるか」だけ。
    let shell = Shell::new(vec![face], true);
    let solid = Solid::new(shell, Vec::new());

    let out_dir = PathBuf::from("target/periodic_face_probe");
    fs::create_dir_all(&out_dir).expect("mkdir failed");
    let out_path = out_dir.join("cylinder_side_only.step");
    StepExporter::export_solid_to_file(&solid, &out_path, "periodic_face_probe")
        .expect("STEP export failed");

    println!("wrote {}", out_path.display());
    println!(
        "この面の正しい側面積（開いた円柱側面 2πrh）= {:.6}",
        2.0 * std::f64::consts::PI * radius * height
    );
    println!("次は FreeCAD でこのファイルを読み、面数・形状種別を見ること。");
}
