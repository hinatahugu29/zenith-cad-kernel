//! **`periodic_solid_probe`の続き**（ROADMAP 柱③の次の一手②。HANDOVER 4-808）。
//!
//! 円柱（u方向だけが閉じる）では、蓋付きの完全な閉じたソリッドが
//! `zenith_topo::Shell::validate_closed`に受理され、FreeCADでも体積が
//! ほぼ一致する1個の有効なソリッドとして読めた。**トーラスはu・v
//! 両方向が閉じる、1段難しいケース**——`occ_reference_torus.step`
//! （OCCT自身の参照ファイル）を読むと、トーラス面は**たった1個の頂点**
//! （両方のシーム稜が共有する点）と**2本のシーム稜**（それぞれ自分自身
//! へ戻る閉じた稜。1本は主円方向、もう1本は管の断面方向）を持ち、
//! `EDGE_LOOP`はその2本を向きを変えてそれぞれ2回ずつ、計4辺で構成される
//! ——蓋は要らない（トーラス面自体に境界が無いため）。
//!
//! 使い方:
//!     cargo run -p zenith_algo --example periodic_torus_probe

use std::f64::consts::FRAC_1_SQRT_2;
use std::fs;
use std::path::PathBuf;

use zenith_algo::MassCalculator;
use zenith_geom::{nurbs_curve::ControlPoint3, nurbs_curve::NurbsCurve3, KnotVector, NurbsSurface3};
use zenith_io::StepExporter;
use zenith_math::{Point3, Tolerance};
use zenith_tess::{TessellationParams, TriangleMesh};
use zenith_topo::{Edge, Face, FaceGeometry, Orientation, OrientedEdge, Shell, Solid, Vertex, Wire};

/// `mesh_watertight_probe`と同じ辺の使用回数の数え方。
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

/// 閉じた次数2有理円の制御点9個ぶんの重み（標準の4分割ベジエ束ね）。
fn closed_circle_weights() -> [f64; 9] {
    let w = FRAC_1_SQRT_2;
    [1.0, w, 1.0, w, 1.0, w, 1.0, w, 1.0]
}

fn closed_circle_knots() -> KnotVector {
    KnotVector::new(vec![
        0.0, 0.0, 0.0, 0.25, 0.25, 0.5, 0.5, 0.75, 0.75, 1.0, 1.0, 1.0,
    ])
}

/// 半径1の閉じた円の制御点9個を、ローカル(x,y)座標で返す
/// （次数2の有理4分割: オン・カーブ点が(±1,0)/(0,±1)、制御点が
/// (±1,±1)——重みは`closed_circle_weights`と対応）。
fn unit_circle_local_xy() -> [(f64, f64); 9] {
    [
        (1.0, 0.0),
        (1.0, 1.0),
        (0.0, 1.0),
        (-1.0, 1.0),
        (-1.0, 0.0),
        (-1.0, -1.0),
        (0.0, -1.0),
        (1.0, -1.0),
        (1.0, 0.0),
    ]
}

fn main() {
    let r_major = 12.0;
    let r_minor = 4.0;
    let tol = Tolerance::default();

    let weights = closed_circle_weights();
    let unit_xy = unit_circle_local_xy();
    let knots = closed_circle_knots();

    // grid[i][j]: 主円方向 i・管の断面方向 j。
    // 断面方向 j の点 (dr_j, dz_j) から、その断面の半径 s_j と高さ z_j を得て、
    // 主円方向 i の単位円 (lx_i, ly_i) をその半径ぶん伸ばして3D点にする。
    let mut grid: Vec<Vec<ControlPoint3>> = Vec::with_capacity(9);
    for i in 0..9 {
        let (lx, ly) = unit_xy[i];
        let mut row = Vec::with_capacity(9);
        for j in 0..9 {
            let (dr, dz) = unit_xy[j];
            let s = r_major + r_minor * dr;
            let z = r_minor * dz;
            let weight = weights[i] * weights[j];
            row.push(ControlPoint3::new(
                Point3::new(s * lx, s * ly, z),
                weight,
            ));
        }
        grid.push(row);
    }
    let surface =
        NurbsSurface3::new(2, 2, grid, knots.clone(), knots.clone()).expect("surface failed");

    // 共有頂点: i=0, j=0 ——主円角0・断面角0の点 = ((r_major+r_minor), 0, 0)。
    let shared_point = Point3::new(r_major + r_minor, 0.0, 0.0);
    let vertex = Vertex::new(shared_point, tol.linear);

    // u-シーム（主円方向、j=0 固定）: 半径 (r_major+r_minor) の円、z=0。
    let u_seam_points: Vec<ControlPoint3> = (0..9)
        .map(|i| {
            let (lx, ly) = unit_xy[i];
            let s = r_major + r_minor; // j=0 の (dr,dz)=(1,0) なので s=r_major+r_minor
            ControlPoint3::new(Point3::new(s * lx, s * ly, 0.0), weights[i])
        })
        .collect();
    let u_seam_curve = NurbsCurve3::new(2, u_seam_points, knots.clone()).unwrap();
    let u_seam_edge = Edge::new(u_seam_curve, vertex.clone(), vertex.clone(), tol.linear);

    // v-シーム（断面方向、i=0 固定）: 主円角0での管の断面そのもの。
    let v_seam_points: Vec<ControlPoint3> = (0..9)
        .map(|j| {
            let (dr, dz) = unit_xy[j];
            let s = r_major + r_minor * dr;
            let z = r_minor * dz;
            // i=0 の (lx,ly)=(1,0) なので x=s, y=0。
            ControlPoint3::new(Point3::new(s, 0.0, z), weights[j])
        })
        .collect();
    let v_seam_curve = NurbsCurve3::new(2, v_seam_points, knots).unwrap();
    let v_seam_edge = Edge::new(v_seam_curve, vertex.clone(), vertex, tol.linear);

    // **occ_reference_torus.step と同じ並び**: u-seam(逆)・v-seam(逆)・
    // u-seam(順)・v-seam(順)。両方とも同じ頂点へ戻る閉じた稜なので、
    // 連鎖の向きはどの並びでも「繋がる」が、符号（面の向きとp-curveの
    // 整合）は`validate_closed`で確認する。
    let wire = Wire::new(vec![
        OrientedEdge::new(u_seam_edge.clone(), Orientation::Reversed),
        OrientedEdge::new(v_seam_edge.clone(), Orientation::Reversed),
        OrientedEdge::new(u_seam_edge, Orientation::Forward),
        OrientedEdge::new(v_seam_edge, Orientation::Forward),
    ]);

    let face = Face::new(
        FaceGeometry::Nurbs(surface),
        wire,
        Vec::new(),
        Orientation::Forward,
        tol.linear,
    );

    let shell = Shell::closed(vec![face]);
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
    let out_path = out_dir.join("torus_single_periodic_face.step");
    StepExporter::export_solid_to_file(&solid, &out_path, "periodic_torus_probe")
        .expect("STEP export failed");

    println!("wrote {}", out_path.display());
    let exact = 2.0 * std::f64::consts::PI.powi(2) * r_major * r_minor * r_minor;
    println!("閉じた式の体積（2π²R r²）= {:.6}", exact);

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

    // 表示メッシュ側: u・v両方向の境界（2本のシーム）をまたいで正しく
    // 閉じるか（4-811）。
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
