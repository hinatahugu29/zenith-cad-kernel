//! **`periodic_torus_probe`の続き——球（2つの極で退化する、まだ未検証の
//! パターン）**（ROADMAP柱③）。
//!
//! 円柱（u方向だけ閉じる）・トーラス（u・v両方向閉じる）・円錐（頂点で
//! 退化、u方向だけ閉じる）は4-807〜4-818で検証済み。**球は円錐と同じ
//! 退化（頂点）が、v方向の両端（北極・南極）にある**——まだ試していない
//! パターン。
//!
//! `occ_reference_sphere.step`（OCCT自身の参照ファイル）を読むと、
//! OCCTは解析曲面（`SPHERICAL_SURFACE`）を使うため、境界に`VERTEX_LOOP`
//! （稜0本、頂点1個だけの境界）という特殊な表現を使っている——これは
//! zenithのNURBS曲面には使えない（解析的に厳密な周期性を持たないため、
//! 継ぎ目の稜が要る）。**代わりに、円錐と同じ「退化行＋稜線1本を2回
//! 使う」構成をv方向の両端に当てはめる**——北極から南極へ向かう子午線
//! そのものを稜線（seam）として、向きを変えて2回使う。

use std::f64::consts::FRAC_1_SQRT_2;
use std::fs;
use std::path::PathBuf;

use zenith_algo::MassCalculator;
use zenith_geom::{nurbs_curve::ControlPoint3, nurbs_curve::NurbsCurve3, KnotVector, NurbsSurface3};
use zenith_io::StepExporter;
use zenith_math::{Point3, Tolerance};
use zenith_tess::{TessellationParams, TriangleMesh};
use zenith_topo::{Edge, Face, FaceGeometry, Orientation, OrientedEdge, Shell, Solid, Vertex, Wire};

fn edge_use_counts(mesh: &TriangleMesh, tol: f64) -> std::collections::BTreeMap<(i64, i64, i64, i64, i64, i64), usize> {
    let quantize = |p: Point3| {
        let scale = 1.0 / tol;
        ((p.x * scale).round() as i64, (p.y * scale).round() as i64, (p.z * scale).round() as i64)
    };
    let mut counts = std::collections::BTreeMap::new();
    for triangle in &mesh.indices {
        let points: Vec<_> = triangle.iter().map(|index| quantize(mesh.positions[*index as usize])).collect();
        for corner in 0..3 {
            let a = points[corner];
            let b = points[(corner + 1) % 3];
            let key = if a <= b { (a.0, a.1, a.2, b.0, b.1, b.2) } else { (b.0, b.1, b.2, a.0, a.1, a.2) };
            *counts.entry(key).or_insert(0) += 1;
        }
    }
    counts
}

fn main() {
    let radius = 10.0;
    let tol = Tolerance::default();
    let w = FRAC_1_SQRT_2;

    // 経度方向（u）の閉じた9点の単位円（半径1。実際の半径はj行ごとに
    // 違うスケールを掛ける）。
    let unit_xy: [(f64, f64); 9] = [
        (1.0, 0.0), (1.0, 1.0), (0.0, 1.0), (-1.0, 1.0), (-1.0, 0.0),
        (-1.0, -1.0), (0.0, -1.0), (1.0, -1.0), (1.0, 0.0),
    ];
    let longitude_weights = [1.0, w, 1.0, w, 1.0, w, 1.0, w, 1.0];
    let knots_u = KnotVector::new(vec![
        0.0, 0.0, 0.0, 0.25, 0.25, 0.5, 0.5, 0.75, 0.75, 1.0, 1.0, 1.0,
    ]);

    // 緯度方向（v）の5行: 北極・北回帰（北緯45度相当の制御点）・赤道・
    // 南回帰・南極。既存`make_sphere`の`profile`と同じ制御点・重み。
    // (radial, z) のペア。
    let latitude_rows: [(f64, f64); 5] = [
        (0.0, radius),           // 北極（退化）
        (radius, radius),        // 北の角（接線交点）
        (radius, 0.0),           // 赤道
        (radius, -radius),       // 南の角
        (0.0, -radius),          // 南極（退化）
    ];
    let latitude_weights = [1.0, w, 1.0, w, 1.0];
    let knots_v = KnotVector::new(vec![0.0, 0.0, 0.0, 0.5, 0.5, 1.0, 1.0, 1.0]);

    let mut grid: Vec<Vec<ControlPoint3>> = Vec::with_capacity(9);
    for i in 0..9 {
        let (lx, ly) = unit_xy[i];
        let mut row = Vec::with_capacity(5);
        for j in 0..5 {
            let (radial, z) = latitude_rows[j];
            let point = if radial == 0.0 {
                Point3::new(0.0, 0.0, z)
            } else {
                Point3::new(radial * lx, radial * ly, z)
            };
            let weight = longitude_weights[i] * latitude_weights[j];
            row.push(ControlPoint3::new(point, weight));
        }
        grid.push(row);
    }
    let surface = NurbsSurface3::new(2, 2, grid, knots_u, knots_v.clone()).expect("surface failed");

    // 子午線（経度0度、i=0の列）そのものを稜線にする——
    // 既存make_sphereのprofileと同じ制御点・重み。
    let north = Point3::new(0.0, 0.0, radius);
    let south = Point3::new(0.0, 0.0, -radius);
    let meridian_points = vec![
        ControlPoint3::unweighted(north),
        ControlPoint3::new(Point3::new(radius, 0.0, radius), w),
        ControlPoint3::unweighted(Point3::new(radius, 0.0, 0.0)),
        ControlPoint3::new(Point3::new(radius, 0.0, -radius), w),
        ControlPoint3::unweighted(south),
    ];
    let v_north = Vertex::new(north, tol.linear);
    let v_south = Vertex::new(south, tol.linear);
    let meridian_curve = NurbsCurve3::new(2, meridian_points, knots_v).unwrap();
    let meridian_edge = Edge::new(meridian_curve, v_north.clone(), v_south.clone(), tol.linear);

    // 輪: 稜線を2回（向きを変えて）使うだけで閉じる——両端が退化点
    // （北極・南極）なので、円柱・トーラスのような別の蓋や2本目の
    // シームは要らない。
    let wire = Wire::new(vec![
        OrientedEdge::new(meridian_edge.clone(), Orientation::Reversed),
        OrientedEdge::new(meridian_edge, Orientation::Forward),
    ]);
    let face = Face::new(FaceGeometry::Nurbs(surface), wire, Vec::new(), Orientation::Reversed, tol.linear);

    println!("face.pcurves is_some = {}", face.pcurves.is_some());
    if let Some(pcurves) = &face.pcurves {
        for segment in &pcurves.outer_loop.segments {
            let (t0, t1) = segment.curve.param_range();
            let p0 = segment.curve.evaluate(t0);
            let p1 = segment.curve.evaluate(t1);
            println!(
                "  pcurve segment: edge_id={} uv範囲=({:.4},{:.4})..({:.4},{:.4})",
                segment.edge_id, p0.x, p0.y, p1.x, p1.y
            );
        }
    }
    let shell = Shell::closed(vec![face]);
    let report = shell.validate_closed(&tol);
    println!("validate_closed: {:#?}", report);
    println!("is_valid: {}", report.is_valid());

    let solid = match Solid::try_simple(shell, &tol) {
        Ok(solid) => solid,
        Err(err) => {
            println!("Solid::try_simple が断りました: {err}");
            return;
        }
    };

    let out_dir = PathBuf::from("target/periodic_face_probe");
    fs::create_dir_all(&out_dir).expect("mkdir failed");
    let out_path = out_dir.join("sphere_single_periodic_face.step");
    StepExporter::export_solid_to_file(&solid, &out_path, "periodic_sphere_probe").expect("STEP export failed");
    println!("wrote {}", out_path.display());

    let exact = 4.0 / 3.0 * std::f64::consts::PI * radius.powi(3);
    println!("閉じた式の体積（4/3 πr³）= {:.6}", exact);
    for divisions in [24usize, 64, 128] {
        let params = TessellationParams { u_divisions: divisions, v_divisions: divisions };
        let measured = MassCalculator::compute_volume_from_brep(&solid, &params);
        let relative = (measured - exact).abs() / exact;
        println!("  zenith自身の体積（分割{divisions}）= {:.6}（相対差 {:.3e}）", measured, relative);
    }

    {
        let params = TessellationParams { u_divisions: 24, v_divisions: 24 };
        let face_mesh = zenith_tess::tessellate_face(&solid.outer_shell.faces[0], &params);
        println!("  face単体のテッセレーション: {}三角形", face_mesh.indices.len());
    }

    for divisions in [8usize, 24, 64] {
        let params = TessellationParams { u_divisions: divisions, v_divisions: divisions };
        let mesh = zenith_tess::tessellate_solid(&solid, &params);
        let counts = edge_use_counts(&mesh, 1e-6);
        let open = counts.values().filter(|c| **c == 1).count();
        let non_manifold = counts.values().filter(|c| **c > 2).count();
        println!(
            "  表示メッシュ（分割{divisions}）: {}三角形・{}辺・open{}・n-mani{}  {}",
            mesh.indices.len(), counts.len(), open, non_manifold,
            if open == 0 && non_manifold == 0 { "閉じている" } else { "開いている" }
        );
    }
    println!("次は FreeCAD でこのファイルを読み、Volume を見ること。");
    println!(
        "\n**見つけた宿題**: 子午線を2回使う輪の自動導出p-curveは、両方とも\
         u=0に落ちる（上の2行で確認済み）——円柱・トーラスは他の辺\
         （上下の円）がu方向いっぱいに幅を持っていたので気づかなかったが、\
         球はこのシーム2本しか境界を持たないため、u方向の幅が0のまま\
         （退化したスリット）になる。`tessellate_face`単体は1152三角形\
         を返すが、`tessellate_solid`（溶接付き、面をまたいで稜の刻みを\
         揃える経路）は0三角形——おそらく共有辺の刻み方の前提がこの\
         退化スリットと噛み合わない。手でu=0とu=1を別々に割り当てた\
         p-curveを作る（自動導出に頼らない）必要があり、円柱・トーラス・\
         円錐より1段難しい、まだ閉じていない宿題。"
    );
}
