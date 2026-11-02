//! **切り欠きの道（`split_hole_to_hole`）に入る配置を集めて測る**
//! （4-863。4-856・4-857 の続き）。
//!
//! 4-856 で `ZENITH_TANGENT_IMPRINT` と `ZENITH_BOUNDARY_NOTCH` を入れて
//! 4-777 が返るようになりましたが、**効き目を測れたのは ring_rim の
//! 1配置だけ**です。4-857 で板＋円柱を5通り並べたら、**その道に
//! 入りませんでした**（`NOTCHWHY` 0行）。
//!
//! 入るのは **「穴あきの平面の面」に、相手の接線が外周に乗る形で
//! 載る**配置です。ring_rim と同じ骨にして、**載せる輪の位置だけ**を
//! 動かします: 穴あきの板（20x10x5、中央にドリル穴）の天面に、
//! ワッシャー（内半径5・外半径8・高さ3の回転体）を z=5 で載せ、
//! **中心の y を動かして**
//!
//! - 外周 y=0 をまたぐ（ring_rim と同じ側）
//! - もっと深くまたぐ
//! - 外周に内側から接する
//! - 外周に触らない
//!
//! を作ります。3演算そろって初めて採点します（∪＋∩＝A＋B、
//! ＼＋∩＝A）。**返っただけでは、答えが合っているとは言えません。**
//!
//! これは診断です。何も直していません。`ZENITH_TANGENT_IMPRINT=1
//! ZENITH_BOUNDARY_NOTCH=1` を立てた状態と既定の状態の両方で回して、
//! 差を見てください。`ZENITH_NP_ONLY=2` で1配置だけ回せます。

use zenith_algo::{
    revolve_sketch, BooleanEngine, BooleanOpType, BrepTransform, ExtrudeBuilder, PrimitiveBuilder,
    SketchSolver, WorkPlane,
};
use zenith_math::{Point2, Point3, Tolerance, Vec3};
use zenith_topo::{Edge, OrientedEdge, Solid, Vertex, Wire};

fn make_wire(pts: &[Point3]) -> Wire {
    let vertices: Vec<Vertex> = pts.iter().map(|p| Vertex::from_point(*p)).collect();
    let n = vertices.len();
    let edges = (0..n)
        .map(|i| {
            OrientedEdge::forward(
                Edge::line_between(vertices[i].clone(), vertices[(i + 1) % n].clone()).unwrap(),
            )
        })
        .collect();
    Wire::new(edges)
}

fn non_manifold_edges(solid: &Solid) -> usize {
    let mut uses: Vec<(u64, usize)> = Vec::new();
    for face in &solid.outer_shell.faces {
        for wire in std::iter::once(&face.outer_wire).chain(face.inner_wires.iter()) {
            for oe in &wire.edges {
                match uses.iter_mut().find(|(id, _)| *id == oe.edge.id) {
                    Some(entry) => entry.1 += 1,
                    None => uses.push((oe.edge.id, 1)),
                }
            }
        }
    }
    uses.iter().filter(|(_, count)| *count != 2).count()
}

fn main() {
    let tol = Tolerance::default();
    let params = zenith_tess::TessellationParams::default();
    let volume_of =
        |solid: &Solid| zenith_algo::MassCalculator::compute_volume_from_brep(solid, &params).abs();

    // 穴あきの板（ring_rim と同じ骨）。
    let plate_pts = [
        Point3::new(0.0, 0.0, 0.0),
        Point3::new(20.0, 0.0, 0.0),
        Point3::new(20.0, 10.0, 0.0),
        Point3::new(0.0, 10.0, 0.0),
    ];
    let hole_pts = [
        Point3::new(7.0, 2.0, 0.0),
        Point3::new(13.0, 2.0, 0.0),
        Point3::new(13.0, 8.0, 0.0),
        Point3::new(7.0, 8.0, 0.0),
    ];
    let plate = ExtrudeBuilder::extrude_face_with_holes(
        &make_wire(&plate_pts),
        &[make_wire(&hole_pts)],
        Vec3::new(0.0, 0.0, 5.0),
        &tol,
    )
    .expect("extrude plate with hole");
    let drill = BrepTransform::translate_solid(
        &PrimitiveBuilder::make_cylinder(2.0, 10.0).expect("cylinder"),
        Vec3::new(10.0, 5.0, -2.5),
    );
    let drilled =
        BooleanEngine::boolean_solids_exact(&plate, &drill, BooleanOpType::Difference, &tol)
            .expect("drill plate");
    let a = volume_of(&drilled);
    println!(
        "穴あきの板: 体積 {a:.6}、面 {} 枚",
        drilled.outer_shell.faces.len()
    );

    // ワッシャー（内半径5・外半径8・高さ3の回転体）。ring_rim と同じ
    // スケッチ回転。
    let mut solver = SketchSolver::new();
    let p0 = solver.add_point(5.0, 0.0);
    let p1 = solver.add_point(8.0, 0.0);
    let p2 = solver.add_point(8.0, 3.0);
    let p3 = solver.add_point(5.0, 3.0);
    solver.add_line(p0, p1);
    solver.add_line(p1, p2);
    solver.add_line(p2, p3);
    solver.add_line(p3, p0);
    let washer = revolve_sketch(
        &solver,
        &WorkPlane::xy(),
        Point2::new(0.0, 0.0),
        Point2::new(0.0, 1.0),
        &tol,
    )
    .expect("revolve washer");

    // ワッシャーの軸は原点。y を足すと、外半径8の円が板の外周 y=0 を
    // どう切るかが変わる。
    let cases: [(&str, f64, f64); 4] = [
        ("ring_rim と同じ（軸が外周の上）", 0.0, 0.0),
        ("もっと深くまたぐ", 0.0, 2.0),
        ("外周に内側から接する（軸を y=8 へ）", 0.0, 8.0),
        ("外周に触らない（軸を板の真ん中へ）", 10.0, 5.0),
    ];

    let only: Option<usize> = std::env::var("ZENITH_NP_ONLY")
        .ok()
        .and_then(|v| v.parse().ok());
    let mut refused = 0usize;
    let mut broken = 0usize;
    let mut worst = 0.0f64;
    for (index, (name, x, y)) in cases.into_iter().enumerate() {
        if only.is_some_and(|want| want != index) {
            continue;
        }
        let moved = BrepTransform::translate_solid(&washer, Vec3::new(x, y, 5.0));
        let b = volume_of(&moved);
        let mut volumes: Vec<Option<f64>> = Vec::new();
        let mut notes: Vec<String> = Vec::new();
        for (label, op) in [
            ("∪", BooleanOpType::Union),
            ("∩", BooleanOpType::Intersection),
            ("＼", BooleanOpType::Difference),
        ] {
            match BooleanEngine::boolean_solids_exact(&drilled, &moved, op, &tol) {
                Ok(solid) => {
                    notes.push(format!(
                        "{label} 面{}枚 非多様体{}",
                        solid.outer_shell.faces.len(),
                        non_manifold_edges(&solid)
                    ));
                    volumes.push(Some(volume_of(&solid)));
                }
                // **空の答えは断りではありません**（4-857）。
                Err(reason) if reason.contains("empty result") => {
                    notes.push(format!("{label} 空（体積0）"));
                    volumes.push(Some(0.0));
                }
                Err(reason) => {
                    refused += 1;
                    notes.push(format!(
                        "{label} 断り（{}）",
                        reason.chars().take(400).collect::<String>()
                    ));
                    volumes.push(None);
                }
            }
        }
        print!("  {name} (x={x}, y={y}): B 体積 {b:.6}、{}", notes.join("、"));
        if let (Some(union), Some(intersection), Some(difference)) =
            (volumes[0], volumes[1], volumes[2])
        {
            let now = ((union + intersection - a - b).abs() / (a + b))
                .max((difference + intersection - a).abs() / a);
            worst = worst.max(now);
            if now > 1e-6 {
                broken += 1;
                print!("、**恒等式が破れました** {now:.3e}");
            } else {
                print!("、恒等式 {now:.3e}");
            }
        } else {
            print!("、3演算そろわないので採点しません");
        }
        println!();
    }
    println!(
        "**まとめ**: 4配置、断り {refused} 件、恒等式が破れた配置 {broken} 件、残差の最悪 {worst:.3e}"
    );
    println!("**これは診断です。何も直していません。**");
}
