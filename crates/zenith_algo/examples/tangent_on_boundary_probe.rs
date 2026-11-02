//! **接線が面の外周に乗る配置を、たくさん集めて測る**（4-857、4-856 の続き）。
//!
//! 4-856 で `ZENITH_TANGENT_IMPRINT` と `ZENITH_BOUNDARY_NOTCH` の
//! 2つを立てると、4-777（積み重ねたブーリアン）が3演算そろって返る
//! ようになりました。**ただし測ったのは1配置だけ**です。
//! 4-852 では、広すぎる口が `box x torus` の和を赤くしました——
//! **1配置で足りると思うと、同じ罠を踏みます。**
//!
//! ここでは**接触の円が、相手の平面の外周にどう当たるか**を
//! 連続的に動かして集めます: 板の天面（z=5、20x10）に円柱を立て、
//! 中心の y を動かして
//!
//! - 丸ごと内側
//! - 外周 y=0 に内側から**接する**
//! - 外周を**またぐ**
//! - ちょうど半分
//! - 外周に外側から接する
//!
//! の5通りを作ります。**接触する面（円柱の底 z=5）は板の天面と
//! 同じ平面に乗っています**ので、4-852 で絞った「相手に同じ平面に
//! 乗った面があるときだけ」の条件に当たります。
//!
//! 3演算そろって初めて採点します（∪＋∩＝A＋B、＼＋∩＝A）。
//! **返っただけでは、答えが合っているとは言えません。**
//!
//! これは診断です。何も直していません。口を立てた状態と既定の
//! 状態の両方で回して、差を見てください。

use zenith_algo::{
    BooleanEngine, BooleanOpType, BrepTransform, ExtrudeBuilder, PrimitiveBuilder,
};
use zenith_math::{Point3, Tolerance, Vec3};
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

/// 板の稜のうち、**使われ方が2回でないもの**を数える（非多様体の目安）。
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

    // 20x10 の板を z=0..5 に押し出す。天面は z=5 の平面。
    let plate_pts = [
        Point3::new(0.0, 0.0, 0.0),
        Point3::new(20.0, 0.0, 0.0),
        Point3::new(20.0, 10.0, 0.0),
        Point3::new(0.0, 10.0, 0.0),
    ];
    let plate = ExtrudeBuilder::extrude_face_with_holes(
        &make_wire(&plate_pts),
        &[],
        Vec3::new(0.0, 0.0, 5.0),
        &tol,
    )
    .expect("extrude plate");
    let a = volume_of(&plate);
    println!("板: 体積 {a:.6}、面 {} 枚", plate.outer_shell.faces.len());

    let radius = 3.0;
    // 半径3の円柱を天面 z=5 に立てる。y を動かして外周 y=0 への
    // 当たり方を変える。
    let cases: [(&str, f64); 5] = [
        ("丸ごと内側（外周に触らない）", 5.0),
        ("外周 y=0 に内側から接する", radius),
        ("外周をまたぐ", 1.5),
        ("ちょうど半分（中心が外周の上）", 0.0),
        ("外周に外側から接する", -radius),
    ];

    let mut refused = 0usize;
    let mut broken = 0usize;
    let mut worst = 0.0f64;
    for (name, y) in cases {
        let stud = BrepTransform::translate_solid(
            &PrimitiveBuilder::make_cylinder(radius, 4.0).expect("cylinder"),
            Vec3::new(10.0, y, 5.0),
        );
        let b = volume_of(&stud);
        let mut volumes: Vec<Option<f64>> = Vec::new();
        let mut notes: Vec<String> = Vec::new();
        for (label, op) in [
            ("∪", BooleanOpType::Union),
            ("∩", BooleanOpType::Intersection),
            ("＼", BooleanOpType::Difference),
        ] {
            match BooleanEngine::boolean_solids_exact(&plate, &stud, op, &tol) {
                Ok(solid) => {
                    let bad = non_manifold_edges(&solid);
                    notes.push(format!(
                        "{label} 面{}枚 非多様体{bad}",
                        solid.outer_shell.faces.len()
                    ));
                    volumes.push(Some(volume_of(&solid)));
                }
                // **空の答えは断りではありません。** 円柱が板に乗って
                // いるだけの配置では、積は本当に空です。体積 0 として
                // 採点に入れます（4-857）。
                Err(reason) if reason.contains("empty result") => {
                    notes.push(format!("{label} 空（体積0）"));
                    volumes.push(Some(0.0));
                }
                Err(reason) => {
                    refused += 1;
                    notes.push(format!(
                        "{label} 断り（{}）",
                        reason.chars().take(200).collect::<String>()
                    ));
                    volumes.push(None);
                }
            }
        }
        print!("  {name} (y={y}): B 体積 {b:.6}、{}", notes.join("、"));
        if let (Some(union), Some(intersection), Some(difference)) =
            (volumes[0], volumes[1], volumes[2])
        {
            let identity = (union + intersection - a - b).abs() / (a + b);
            let other = (difference + intersection - a).abs() / a;
            let now = identity.max(other);
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
        "**まとめ**: 5配置、断り {refused} 件、恒等式が破れた配置 {broken} 件、残差の最悪 {worst:.3e}"
    );
    println!("**これは診断です。何も直していません。**");
}
