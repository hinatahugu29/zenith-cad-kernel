//! **同一平面で「部分的に」接する配置を集めて測る**（4-874。4-873 の続き）。
//!
//! 4-873 で `ZENITH_COVERED_SPLIT`（覆っている片を、覆われている片の
//! 輪郭で割る）を入れ、`notch_placement_probe` の配置1 の和が返る
//! ようになりました。**ただし効き目を測れたのは2つの形だけ**です。
//!
//! 昇格の前に、**この口が効く形そのもの**を集めます。4-124 の註が
//! 「**残るメッシュ非多様体は全件この形**」と書いている形——
//! **同じ平面に乗った面が、一部だけ重なっている**配置です。
//!
//! `resolve_coincident_face_pieces` の既にある判定は**面全体の一致**
//! （重心と広がり）しか見ないので、**丸ごと重なる配置は前から通り**、
//! **一部だけ重なる配置が残っていました**。だから、ここでは
//! **わざと「一部だけ」にずらした組**を並べます。
//!
//! 3演算そろって初めて採点します（∪＋∩＝A＋B、＼＋∩＝A）。
//! **空の答えは断りではありません**（4-857）——体積 0 として入れます。
//!
//! これは診断です。何も直していません。`ZENITH_COVERED_SPLIT=1` を
//! 立てた状態と既定の状態の両方で回して、差を見てください。
//! `ZENITH_CC_ONLY=2` で1配置だけ回せます。

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

fn plate(x0: f64, y0: f64, x1: f64, y1: f64, z0: f64, height: f64, tol: &Tolerance) -> Solid {
    let pts = [
        Point3::new(x0, y0, z0),
        Point3::new(x1, y0, z0),
        Point3::new(x1, y1, z0),
        Point3::new(x0, y1, z0),
    ];
    ExtrudeBuilder::extrude_face_with_holes(
        &make_wire(&pts),
        &[],
        Vec3::new(0.0, 0.0, height),
        tol,
    )
    .expect("extrude plate")
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

    // 下の板: 20x10、z 0..5。天面は z=5。
    let base = plate(0.0, 0.0, 20.0, 10.0, 0.0, 5.0, &tol);
    // **穴あきの板**（4-875）。穴の壁の平面は y=2・y=8・x=7・x=13。
    // **ここに食い込むと、割りの結果どうしが同一平面で部分的に
    // 重なる**——4-874 が「届かない」と書いた形です。
    let holed = {
        let outer = make_wire(&[
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(20.0, 0.0, 0.0),
            Point3::new(20.0, 10.0, 0.0),
            Point3::new(0.0, 10.0, 0.0),
        ]);
        let hole = make_wire(&[
            Point3::new(7.0, 2.0, 0.0),
            Point3::new(13.0, 2.0, 0.0),
            Point3::new(13.0, 8.0, 0.0),
            Point3::new(7.0, 8.0, 0.0),
        ]);
        ExtrudeBuilder::extrude_face_with_holes(
            &outer,
            &[hole],
            Vec3::new(0.0, 0.0, 5.0),
            &tol,
        )
        .expect("extrude holed plate")
    };
    let laid = |radius: f64, height: f64, at: Vec3| -> Solid {
        let rotation = zenith_math::Transform3::from_axis_angle(
            &Vec3::new(1.0, 0.0, 0.0),
            -std::f64::consts::FRAC_PI_2,
        );
        let turned = BrepTransform::transform_solid(
            &PrimitiveBuilder::make_cylinder(radius, height).expect("cylinder"),
            &rotation,
        )
        .expect("rotate cylinder");
        BrepTransform::translate_solid(&turned, at)
    };

    // 上に載せるものを変えます。**どれも天面 z=5 に面で接し、
    // 重なりは「一部だけ」**です。
    let cases: Vec<(&str, &Solid, Solid)> = vec![
        (
            "箱が天面の端に半分だけ載る（x -5..5）",
            &base,
            plate(-5.0, 2.0, 5.0, 8.0, 5.0, 4.0, &tol),
        ),
        (
            "箱が天面の角に四分の一だけ載る",
            &base,
            plate(-5.0, -5.0, 5.0, 5.0, 5.0, 4.0, &tol),
        ),
        (
            "箱が天面の中に丸ごと入る（覆われるのは上の箱）",
            &base,
            plate(6.0, 3.0, 12.0, 7.0, 5.0, 4.0, &tol),
        ),
        (
            "箱が天面より大きく、天面を丸ごと覆う",
            &base,
            plate(-2.0, -2.0, 22.0, 12.0, 5.0, 4.0, &tol),
        ),
        (
            "円柱が天面の端に半分だけ載る",
            &base,
            BrepTransform::translate_solid(
                &PrimitiveBuilder::make_cylinder(4.0, 4.0).expect("cylinder"),
                Vec3::new(0.0, 5.0, 5.0),
            ),
        ),
        (
            "円柱が天面の中に丸ごと載る",
            &base,
            BrepTransform::translate_solid(
                &PrimitiveBuilder::make_cylinder(3.0, 4.0).expect("cylinder"),
                Vec3::new(10.0, 5.0, 5.0),
            ),
        ),
        // **ここからは「食い込んでいて、かつ壁が同一平面」**です
        // （4-874）。上の6つは**面で触れているだけ**なので、
        // 既定で全部通りました——**4-873 が直した形に届いて
        // いません**。届くのは、**他のところで実際に交わりながら、
        // 壁だけが同じ平面に乗って一部重なる**配置です。
        (
            "箱が沈み込み、x=0 の壁が同じ平面に乗る（一部重なる）",
            &base,
            plate(0.0, 2.0, 6.0, 8.0, 2.0, 7.0, &tol),
        ),
        (
            "箱が沈み込み、y=0 の壁が同じ平面に乗る（一部重なる）",
            &base,
            plate(4.0, 0.0, 12.0, 6.0, 2.0, 7.0, &tol),
        ),
        (
            "箱が沈み込み、x=0 と y=0 の壁が両方同じ平面に乗る",
            &base,
            plate(0.0, 0.0, 6.0, 6.0, 2.0, 7.0, &tol),
        ),
        (
            "箱が沈み込み、底面 z=0 だけが同じ平面に乗る",
            &base,
            plate(4.0, 2.0, 12.0, 8.0, 0.0, 8.0, &tol),
        ),
        (
            "円柱が沈み込み、底面 z=0 だけが同じ平面に乗る",
            &base,
            BrepTransform::translate_solid(
                &PrimitiveBuilder::make_cylinder(3.0, 8.0).expect("cylinder"),
                Vec3::new(10.0, 5.0, 0.0),
            ),
        ),
        // **ここからは「覆っている側の輪郭が曲がっている」**形です
        // （4-874）。箱どうしの同一平面は、既定で全部通りました
        // ——**4-873 が直したのは、円で囲まれた平面の片が
        // 四角の壁を一部だけ覆う**形でした（4-866）。
        // 円柱を横に倒して、**底の円盤を板の壁の平面に
        // 乗せます**。
        (
            "横に倒した円柱の円盤が、y=0 の壁の平面に乗る",
            &base,
            {
                let rotation = zenith_math::Transform3::from_axis_angle(
                    &Vec3::new(1.0, 0.0, 0.0),
                    -std::f64::consts::FRAC_PI_2,
                );
                let laid = BrepTransform::transform_solid(
                    &PrimitiveBuilder::make_cylinder(4.0, 6.0).expect("cylinder"),
                    &rotation,
                )
                .expect("rotate cylinder");
                BrepTransform::translate_solid(&laid, Vec3::new(10.0, 0.0, 4.0))
            },
        ),
        (
            "横に倒した円柱の円盤が、y=0 の壁をはみ出す",
            &base,
            {
                let rotation = zenith_math::Transform3::from_axis_angle(
                    &Vec3::new(1.0, 0.0, 0.0),
                    -std::f64::consts::FRAC_PI_2,
                );
                let laid = BrepTransform::transform_solid(
                    &PrimitiveBuilder::make_cylinder(7.0, 6.0).expect("cylinder"),
                    &rotation,
                )
                .expect("rotate cylinder");
                BrepTransform::translate_solid(&laid, Vec3::new(10.0, 0.0, 2.0))
            },
        ),
        // **4-874 が「これを集めろ」と書いた形**（4-875）。
        // 穴あきの板の**穴の壁の平面**に、横に倒した円柱の
        // **円盤を乗せて、板に食い込ませます**。
        (
            "穴あきの板、倒した円柱の円盤が穴の壁 y=2 に乗る",
            &holed,
            laid(4.0, 6.0, Vec3::new(10.0, 2.0, 4.0)),
        ),
        (
            "穴あきの板、円盤が穴の壁 y=2 に全部入る（半径 2）",
            &holed,
            laid(2.0, 6.0, Vec3::new(10.0, 2.0, 3.0)),
        ),
        (
            "穴あきの板、円盤が穴の壁 y=2 をはみ出す（半径 7）",
            &holed,
            laid(7.0, 6.0, Vec3::new(10.0, 2.0, 2.0)),
        ),
        // **この家系を振って増やします**（4-883）。
        // 4-881 の口が効くと言えるのはまだ「配置13 の和」だけなので、
        // **半径・乗せる壁・壁に沿ったずれ・穴の数**を振ります。
        (
            "穴あきの板、円盤（半径 3）が壁 y=2 の内側に收まる",
            &holed,
            laid(3.0, 6.0, Vec3::new(10.0, 2.0, 3.0)),
        ),
        (
            "穴あきの板、円盤（半径 5）が壁 y=2 の上下にはみ出す",
            &holed,
            laid(5.0, 6.0, Vec3::new(10.0, 2.0, 2.5)),
        ),
        (
            "穴あきの板、円盤を壁に沿って x=8 へずらす（非対称）",
            &holed,
            laid(4.0, 6.0, Vec3::new(8.0, 2.0, 4.0)),
        ),
        (
            "穴あきの板、円盤を壁に沿って x=12 へずらす（非対称）",
            &holed,
            laid(4.0, 6.0, Vec3::new(12.0, 2.0, 4.0)),
        ),
        (
            "穴あきの板、円盤を x=7 の壁に乗せる（軸を x へ）",
            &holed,
            {
                let rotation = zenith_math::Transform3::from_axis_angle(
                    &Vec3::new(0.0, 1.0, 0.0),
                    std::f64::consts::FRAC_PI_2,
                );
                let turned = BrepTransform::transform_solid(
                    &PrimitiveBuilder::make_cylinder(4.0, 6.0).expect("cylinder"),
                    &rotation,
                )
                .expect("rotate cylinder");
                BrepTransform::translate_solid(&turned, Vec3::new(7.0, 5.0, 4.0))
            },
        ),
    ];

    println!("下の板: 体積 {:.6}、面 {} 枚", volume_of(&base), base.outer_shell.faces.len());
    println!("穴あきの板: 体積 {:.6}、面 {} 枚", volume_of(&holed), holed.outer_shell.faces.len());

    let only: Option<usize> = std::env::var("ZENITH_CC_ONLY")
        .ok()
        .and_then(|v| v.parse().ok());
    let only_op: Option<usize> = std::env::var("ZENITH_CC_OP")
        .ok()
        .and_then(|v| v.parse().ok());
    let mut refused = 0usize;
    let mut broken = 0usize;
    let mut bad = 0usize;
    let mut worst = 0.0f64;
    for (index, (name, base, top)) in cases.into_iter().enumerate() {
        if only.is_some_and(|want| want != index) {
            continue;
        }
        let a = volume_of(base);
        let b = volume_of(&top);
        let mut volumes: Vec<Option<f64>> = Vec::new();
        let mut notes: Vec<String> = Vec::new();
        for (at, (label, op)) in [
            ("∪", BooleanOpType::Union),
            ("∩", BooleanOpType::Intersection),
            ("＼", BooleanOpType::Difference),
        ]
        .into_iter()
        .enumerate()
        {
            if only_op.is_some_and(|want| want != at) {
                volumes.push(None);
                continue;
            }
            match BooleanEngine::boolean_solids_exact(base, &top, op, &tol) {
                Ok(solid) => {
                    let non_manifold = non_manifold_edges(&solid);
                    if non_manifold > 0 {
                        bad += 1;
                    }
                    notes.push(format!(
                        "{label} 面{}枚 非多様体{non_manifold}",
                        solid.outer_shell.faces.len()
                    ));
                    volumes.push(Some(volume_of(&solid)));
                }
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
        print!("  {name}: B 体積 {b:.6}、{}", notes.join("、"));
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
        "**まとめ**: 21配置、断り {refused} 件、非多様体を返した演算 {bad} 件、恒等式が破れた配置 {broken} 件、残差の最悪 {worst:.3e}"
    );
    println!("**これは診断です。何も直していません。**");
}
