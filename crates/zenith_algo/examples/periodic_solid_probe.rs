//! **`periodic_face_probe`の続き**（ROADMAP 柱③の次の一手①）。
//!
//! `periodic_face_probe`は円柱の側面だけ（蓋なし、1面のみ）を、OCCT自身の
//! 参照ファイルと同じ「シーム稜を向きを変えて2回使う」構成で書き、FreeCAD
//! で1枚の有効な面として読めることを確かめた。**ここでは蓋2枚を足し、
//! 本当に閉じたソリッドとして測れるか**——`Shell::validate_closed`が
//! この特殊な構成（同じ稜を同じ面が2回使う）を壊さず扱えるか、FreeCADで
//! 読んだ`Volume`が閉じた式（πr²h）と一致するかを見る。

use std::f64::consts::FRAC_1_SQRT_2;
use std::fs;
use std::path::PathBuf;

use zenith_algo::MassCalculator;
use zenith_geom::{nurbs_curve::ControlPoint3, nurbs_curve::NurbsCurve3, KnotVector, NurbsSurface3, PlaneSurface3};
use zenith_io::StepExporter;
use zenith_math::{Point3, Tolerance, Vec3};
use zenith_tess::{TessellationParams, TriangleMesh};
use zenith_topo::{Edge, Face, FaceGeometry, Orientation, OrientedEdge, Shell, Solid, Vertex, Wire};

/// `mesh_watertight_probe`と同じ辺の使用回数の数え方（4-811の「テッセレー
/// ションが周期曲面のuv境界を正しく扱えるか」を確かめるため）。
fn edge_use_counts(mesh: &TriangleMesh, tol: f64) -> std::collections::BTreeMap<(i64, i64, i64, i64, i64, i64), usize> {
    let quantize = |p: Point3| {
        let scale = 1.0 / tol;
        (
            (p.x * scale).round() as i64,
            (p.y * scale).round() as i64,
            (p.z * scale).round() as i64,
        )
    };
    let mut counts = std::collections::BTreeMap::new();
    for triangle in &mesh.indices {
        let points: Vec<_> = triangle
            .iter()
            .map(|index| quantize(mesh.positions[*index as usize]))
            .collect();
        for corner in 0..3 {
            let a = points[corner];
            let b = points[(corner + 1) % 3];
            let key = if a <= b {
                (a.0, a.1, a.2, b.0, b.1, b.2)
            } else {
                (b.0, b.1, b.2, a.0, a.1, a.2)
            };
            *counts.entry(key).or_insert(0) += 1;
        }
    }
    counts
}

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
    let tol = Tolerance::default();

    let bottom_points = closed_circle_points(radius, 0.0);
    let top_points = closed_circle_points(radius, height);
    let knots_u = closed_circle_knots();

    let mut grid = Vec::with_capacity(9);
    for i in 0..9 {
        grid.push(vec![bottom_points[i].clone(), top_points[i].clone()]);
    }
    let surface = NurbsSurface3::new(2, 1, grid, knots_u.clone(), KnotVector::clamped_uniform(2, 1))
        .expect("surface construction failed");

    let v_bottom = Vertex::new(Point3::new(radius, 0.0, 0.0), tol.linear);
    let v_top = Vertex::new(Point3::new(radius, 0.0, height), tol.linear);

    let bottom_curve = NurbsCurve3::new(2, bottom_points, knots_u.clone()).unwrap();
    let top_curve = NurbsCurve3::new(2, top_points, knots_u).unwrap();

    let bottom_edge = Edge::new(bottom_curve, v_bottom.clone(), v_bottom.clone(), tol.linear);
    let top_edge = Edge::new(top_curve, v_top.clone(), v_top.clone(), tol.linear);
    let seam_edge = Edge::line_between(v_bottom.clone(), v_top.clone())
        .expect("seam edge construction failed");

    // 側面: `seam_edge`の生の向きは v_bottom→v_top（`line_between`の引数順）。
    // 輪が実際に閉じるよう連鎖させる——底(逆、trivial)・シーム(順、
    // 底→天へ進む)・天(順、trivial)・シーム(逆、天→底へ戻って輪を閉じる)。
    // 最初の向き（下のコメントの並び）では3面とも符号が反転していたため、
    // 輪全体を逆向きに歩く構成（列を逆順にし、各辺の向きも反転）に直した。
    // [bottom(逆)・seam(順)・top(順)・seam(逆)] → 逆向きに歩くと
    // [seam(順)・top(逆)・seam(逆)・bottom(順)]。
    let side_wire = Wire::new(vec![
        OrientedEdge::new(seam_edge.clone(), Orientation::Forward),
        OrientedEdge::new(top_edge.clone(), Orientation::Reversed),
        OrientedEdge::new(seam_edge, Orientation::Reversed),
        OrientedEdge::new(bottom_edge.clone(), Orientation::Forward),
    ]);
    let side_face = Face::new(
        FaceGeometry::Nurbs(surface),
        side_wire,
        Vec::new(),
        Orientation::Forward,
        tol.linear,
    );

    // 底蓋（-Z 法線で外向き）。側面と同じ `bottom_edge` を共有する——
    // 逆向きで使う（側面側は Reversed、蓋側は Forward にすると、2つの
    // 面からの使用が向かい合う）。
    let p_bot = PlaneSurface3::new(
        Point3::new(0.0, 0.0, 0.0),
        Vec3::new(0.0, 1.0, 0.0),
        Vec3::new(1.0, 0.0, 0.0),
    )
    .expect("bottom plane failed");
    let bottom_wire = Wire::new(vec![OrientedEdge::new(bottom_edge, Orientation::Reversed)]);
    // ↑ 側面側はこの稜を Forward で使うので、共有面側は逆向き（Reversed）。
    let bottom_face = Face::simple(FaceGeometry::Plane(p_bot), bottom_wire);

    // 天蓋（+Z 法線で外向き）。側面側は Forward なので、蓋側は Reversed。
    let p_top = PlaneSurface3::new(
        Point3::new(0.0, 0.0, height),
        Vec3::new(1.0, 0.0, 0.0),
        Vec3::new(0.0, 1.0, 0.0),
    )
    .expect("top plane failed");
    let top_wire = Wire::new(vec![OrientedEdge::new(top_edge, Orientation::Forward)]);
    // ↑ 側面側はこの稜を Reversed で使うので、共有面側は逆向き（Forward）。
    let top_face = Face::simple(FaceGeometry::Plane(p_top), top_wire);

    let shell = Shell::closed(vec![side_face, bottom_face, top_face]);
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
    let out_path = out_dir.join("cylinder_closed_periodic.step");
    StepExporter::export_solid_to_file(&solid, &out_path, "periodic_solid_probe")
        .expect("STEP export failed");

    println!("wrote {}", out_path.display());
    let exact = std::f64::consts::PI * radius * radius * height;
    println!("閉じた式の体積（πr²h）= {:.6}", exact);

    // **zenith自身の体積積分**（FreeCADとは独立な測り方。divergence定理、
    // 発散定理に基づく自前の`MassCalculator`）でも測る——FreeCADの体積が
    // ずれていたとき、「このソリッド自体が歪んでいる」のか「FreeCAD側の
    // 数値積分の精度」なのかを切り分けるため。
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

    // **表示メッシュ（`zenith_tess::tessellate_solid`）が、周期曲面の
    // u=0/u=1境界（シーム）をまたいで正しく閉じるか**——`MassCalculator`
    // はB-Repを直接積分するだけでこのパスを通らないので、別に確かめる
    // （4-811の「テッセレーション側の確認も別途必要」）。
    for divisions in [8usize, 24, 64] {
        let params = TessellationParams {
            u_divisions: divisions,
            v_divisions: divisions,
        };
        let mesh = zenith_tess::tessellate_solid(&solid, &params);
        let counts = edge_use_counts(&mesh, 1e-6);
        let open = counts.values().filter(|c| **c == 1).count();
        let non_manifold = counts.values().filter(|c| **c > 2).count();
        println!(
            "  表示メッシュ（分割{divisions}）: {}三角形・{}辺・open{}・n-mani{}  {}",
            mesh.indices.len(),
            counts.len(),
            open,
            non_manifold,
            if open == 0 && non_manifold == 0 { "閉じている" } else { "開いている" }
        );
    }
}
