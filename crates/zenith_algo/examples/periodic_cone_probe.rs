//! **`periodic_torus_probe`の続き**（ROADMAP 柱③の次の一手②。HANDOVER 4-811）。
//!
//! 円柱・トーラスでは周期1面化がFreeCADで有効なソリッドとして読め、体積も
//! zenith自身の発散定理積分で確認した（4-808〜4-810）。**円錐は頂点で
//! 退化する**——`occ_reference_cone_full.step`（OCCT自身の参照ファイル）を
//! 読むと、円錐側面は**2頂点**（底の輪の1点・頂点）・**2本の稜**
//! （輪になる底円＋頂点へ向かう「稜線」1本）だけを持ち、`EDGE_LOOP`は
//! **3辺**——`[稜線(逆), 底円(順), 稜線(順)]`。頂点自身は自己ループの
//! 稜を持たない（退化点なので不要）。円柱・トーラスの4辺ループより
//! **1辺少ない**。

use std::f64::consts::FRAC_1_SQRT_2;
use std::fs;
use std::path::PathBuf;

use zenith_algo::MassCalculator;
use zenith_geom::{nurbs_curve::ControlPoint3, nurbs_curve::NurbsCurve3, KnotVector, NurbsSurface3, PlaneSurface3};
use zenith_io::StepExporter;
use zenith_math::{Point3, Tolerance, Vec3};
use zenith_tess::TessellationParams;
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

fn closed_circle_weights() -> [f64; 9] {
    let w = FRAC_1_SQRT_2;
    [1.0, w, 1.0, w, 1.0, w, 1.0, w, 1.0]
}

fn closed_circle_knots() -> KnotVector {
    KnotVector::new(vec![
        0.0, 0.0, 0.0, 0.25, 0.25, 0.5, 0.5, 0.75, 0.75, 1.0, 1.0, 1.0,
    ])
}

fn main() {
    let radius = 10.0;
    let height = 20.0;
    let tol = Tolerance::default();

    let bottom_points = closed_circle_points(radius, 0.0);
    let bottom_weights = closed_circle_weights();
    let knots_u = closed_circle_knots();
    let apex = Point3::new(0.0, 0.0, height);

    // v方向2行: 底の閉じた円（9点）と、頂点に縮退した行（9点とも同じ
    // apex点・同じ重み——make_cone_with_apexの既存コメント「縮退する
    // 天面側も、行ごとに同じ重みを保つ」と同じ作法）。
    let mut grid: Vec<Vec<ControlPoint3>> = Vec::with_capacity(9);
    for i in 0..9 {
        let bottom_cp = bottom_points[i].clone();
        let w = bottom_weights[i];
        grid.push(vec![bottom_cp, ControlPoint3::new(apex, w)]);
    }
    let surface = NurbsSurface3::new(2, 1, grid, knots_u.clone(), KnotVector::clamped_uniform(2, 1))
        .expect("surface failed");

    let v_bottom = Vertex::new(Point3::new(radius, 0.0, 0.0), tol.linear);
    let v_apex = Vertex::new(apex, tol.linear);

    let bottom_curve = NurbsCurve3::new(2, bottom_points, knots_u).unwrap();
    let bottom_edge = Edge::new(bottom_curve, v_bottom.clone(), v_bottom.clone(), tol.linear);
    // 稜線（底の輪の1点 → 頂点）。退化行の付け根なので直線でよい
    // （make_cone_with_apexの`rulings`と同じ役割、ただし1本だけ）。
    let ruling_edge = Edge::line_between(v_bottom.clone(), v_apex)
        .expect("ruling edge construction failed");

    // **occ_reference_cone_full.step と同じ並び**: 稜線(逆)・底円(順)・稜線(順)。
    let side_wire = Wire::new(vec![
        OrientedEdge::new(ruling_edge.clone(), Orientation::Reversed),
        OrientedEdge::new(bottom_edge.clone(), Orientation::Forward),
        OrientedEdge::new(ruling_edge, Orientation::Forward),
    ]);
    let side_face = Face::new(
        FaceGeometry::Nurbs(surface),
        side_wire,
        Vec::new(),
        Orientation::Forward,
        tol.linear,
    );

    // 底蓋。側面側はこの稜をForwardで使うので、蓋側はReversed
    // （4-808の円柱と同じ共有規約）。
    let p_bot = PlaneSurface3::new(
        Point3::new(0.0, 0.0, 0.0),
        Vec3::new(0.0, 1.0, 0.0),
        Vec3::new(1.0, 0.0, 0.0),
    )
    .expect("bottom plane failed");
    let bottom_wire = Wire::new(vec![OrientedEdge::new(bottom_edge, Orientation::Reversed)]);
    let bottom_face = Face::simple(FaceGeometry::Plane(p_bot), bottom_wire);

    let shell = Shell::closed(vec![side_face, bottom_face]);
    let report = shell.validate_closed(&tol);
    println!("validate_closed: {:#?}", report);
    println!("is_valid: {}", report.is_valid());

    let solid = match Solid::try_simple(shell, &tol) {
        Ok(solid) => solid,
        Err(err) => {
            println!("Solid::try_simple が断りました: {err}");
            println!("（この検証自体が目的——どこで断られるかを見る）");
            return;
        }
    };

    let out_dir = PathBuf::from("target/periodic_face_probe");
    fs::create_dir_all(&out_dir).expect("mkdir failed");
    let out_path = out_dir.join("cone_single_periodic_face.step");
    StepExporter::export_solid_to_file(&solid, &out_path, "periodic_cone_probe")
        .expect("STEP export failed");

    println!("wrote {}", out_path.display());
    let exact = std::f64::consts::PI * radius * radius * height / 3.0;
    println!("閉じた式の体積（πr²h/3）= {:.6}", exact);

    for divisions in [24usize, 64, 128] {
        let params = TessellationParams {
            u_divisions: divisions,
            v_divisions: divisions,
        };
        let measured = MassCalculator::compute_volume_from_brep(&solid, &params);
        let relative = (measured - exact).abs() / exact;
        println!(
            "  zenith自身の体積（分割{divisions}）= {:.6}（相対差 {:.3e}）",
            measured, relative
        );
    }
    println!("次は FreeCAD でこのファイルを読み、Volume を見ること。");
}
