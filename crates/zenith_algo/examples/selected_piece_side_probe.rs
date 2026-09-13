//! **選ばれた面片は、本当に外か**（4-449）。
//!
//! # なぜ要るのか
//!
//! 4-448 で、**接する検体が詰まっているのは交線の段ではない**ところまで
//! 来ました——**端を寄せて浮いた端を 5 個から 1 個にしても、縫合は
//! 「合わない稜 8、非多様体 6」のまま**でした。
//!
//! **`ZENITH_STITCH_WHY=1` が指した合わない稜は、板の底面にある穴の
//! 円弧**でした。**その円弧に相手がいません。**
//!
//! **「相手がいない」には 2 通り**あります——**割れていない**か、
//! **割れたが選ばれていない**か。**稜の側からは区別できません**
//! （`collect_stitch_edge_uses` の註）。
//!
//! # 何を測るか
//!
//! **面片の側から、分類が合っているか**を測ります。
//!
//! **和では「相手の外にある面片」だけが残るはず**です。**`Outside` と
//! 名札が付いた面片の上に、相手の中の点があったら、その名札は間違い**
//! です。
//!
//! **測り方**——面片を三角に割り、**三角の重心**を [`exact_inside`] に
//! 掛けます。**重心は必ずトリムの中**なので、**面の外を測る心配が
//! ありません**。**境界の上の点は `None` で返る**ので、数えません。
//!
//! # 使い方
//!
//! ```bash
//! cargo run --release -p zenith_algo --example selected_piece_side_probe
//! ```
//!
//! **これは診断です。赤にはしません。**

use zenith_algo::{
    exact_inside, extrude_sketch, BooleanOpType, BooleanOperand, BrepIntersectionBuilder,
    SketchSolver, WorkPlane,
};
use zenith_math::{Point3, Tolerance, Vec3};
use zenith_tess::TessellationParams;
use zenith_topo::Solid;

fn rectangle(x0: f64, y0: f64, width: f64, height: f64) -> SketchSolver {
    let mut solver = SketchSolver::new();
    let a = solver.add_point(x0, y0);
    let b = solver.add_point(x0 + width, y0);
    let c = solver.add_point(x0 + width, y0 + height);
    let d = solver.add_point(x0, y0 + height);
    solver.add_line(a, b);
    solver.add_line(b, c);
    solver.add_line(c, d);
    solver.add_line(d, a);
    solver
}

fn add_circle(solver: &mut SketchSolver, cx: f64, cy: f64, r: f64) {
    let centre = solver.add_point(cx, cy);
    let east = solver.add_point(cx + r, cy);
    let north = solver.add_point(cx, cy + r);
    let west = solver.add_point(cx - r, cy);
    let south = solver.add_point(cx, cy - r);
    solver.add_arc(centre, east, north, true);
    solver.add_arc(centre, north, west, true);
    solver.add_arc(centre, west, south, true);
    solver.add_arc(centre, south, east, true);
}

fn main() {
    let tol = Tolerance::default();
    let plane = WorkPlane::xy();

    // **`sketch_boolean_probe` の 6 番目**と同じ 2 つです。
    let mut holed = rectangle(0.0, 0.0, 30.0, 30.0);
    add_circle(&mut holed, 15.0, 15.0, 5.0);
    let bar = rectangle(10.0, -20.0, 8.0, 70.0);
    let a = extrude_sketch(&holed, &plane, 8.0, &tol).expect("穴のある板");
    let b = zenith_algo::BrepTransform::translate_solid(
        &extrude_sketch(&bar, &plane, 8.0, &tol).expect("帯"),
        Vec3::new(0.0, 0.0, -2.0),
    );

    println!("選ばれた面片は、本当に外か（4-449）");
    println!();

    let candidates = BrepIntersectionBuilder::collect_intersection_edge_candidates(
        &a.outer_shell.faces,
        &b.outer_shell.faces,
        &tol,
    );
    let selection = BrepIntersectionBuilder::selected_face_pieces_from_candidates(
        &a,
        &b,
        candidates,
        BooleanOpType::Union,
        &tol,
    );

    println!(
        "和で選ばれた面片 {} 枚",
        selection.selected_face_pieces.len()
    );
    println!();
    println!(
        "{:>3} {:<9}{:<10}{:>7}{:>7}{:>7} {:<34} {}",
        "番", "どちら", "名札", "標本", "中", "外", "囲み箱", "食い違い"
    );
    println!("{}", "-".repeat(106));

    let params = TessellationParams::default();
    let mut wrong = 0usize;
    for (index, piece) in selection.selected_face_pieces.iter().enumerate() {
        // **相手**は、自分と逆のほう。
        let other: &Solid = match piece.operand {
            BooleanOperand::A => &b,
            BooleanOperand::B => &a,
        };
        let mesh = zenith_tess::tessellate_face(&piece.face, &params);
        let mut inside = 0usize;
        let mut outside = 0usize;
        for triangle in &mesh.indices {
            let p0 = mesh.positions[triangle[0] as usize];
            let p1 = mesh.positions[triangle[1] as usize];
            let p2 = mesh.positions[triangle[2] as usize];
            let centre = Point3::new(
                (p0.x + p1.x + p2.x) / 3.0,
                (p0.y + p1.y + p2.y) / 3.0,
                (p0.z + p1.z + p2.z) / 3.0,
            );
            match exact_inside(centre, other, &tol) {
                Some(true) => inside += 1,
                Some(false) => outside += 1,
                // **境界の上**。数えません。
                None => {}
            }
        }
        // **和では、相手の中にある面は残ってはいけません。**
        let mismatch = inside > 0;
        if mismatch {
            wrong += 1;
        }
        // **どの面から来た破片か、囲み箱で名指しします**（重心では
        // 決まりません——円筒の四半分は、重心が面に乗りません）。
        let mut low = Point3::new(f64::MAX, f64::MAX, f64::MAX);
        let mut high = Point3::new(f64::MIN, f64::MIN, f64::MIN);
        for oriented in &piece.face.outer_wire.edges {
            for point in [
                oriented.edge.start_vertex.point,
                oriented.edge.end_vertex.point,
            ] {
                low.x = low.x.min(point.x);
                low.y = low.y.min(point.y);
                low.z = low.z.min(point.z);
                high.x = high.x.max(point.x);
                high.y = high.y.max(point.y);
                high.z = high.z.max(point.z);
            }
        }
        println!(
            "{index:>3} {:<9}{:<10}{:>7}{:>7}{:>7} {:<34} {}",
            format!("{:?}", piece.operand),
            format!("{:?}", piece.location),
            mesh.indices.len(),
            inside,
            outside,
            format!(
                "({:.0},{:.0},{:.0})〜({:.0},{:.0},{:.0}) 内輪{}",
                low.x, low.y, low.z, high.x, high.y, high.z,
                piece.face.inner_wires.len()
            ),
            if mismatch { "**中の点がある**" } else { "" }
        );
    }

    println!();
    if wrong == 0 {
        println!("**分類の食い違いはありません。**");
        println!();
        println!("**つまり、選ばれている面片は全部「外」**です。**足りないのは、");
        println!("選び方ではなく、割り方**——**あるべき破片が、そもそも作られて");
        println!("いない**ほうを見てください。");
    } else {
        println!("**{wrong} 枚が、相手の中の点を持ったまま選ばれています。**");
        println!();
        println!("**そこが、合わない稜の出どころ**です。");
    }
    // ---- **3 演算が何を返すか** ----
    //
    // **接している点があるので、1 つの立体では持てないはず**です
    // （3-1）。**塊ごとに分かれて返るなら、それは持てます。**
    // **何枚・いくつ返るか**を、はっきり出します。
    println!();
    println!("3 演算が返すもの");
    println!();
    for (label, op) in [
        ("和", BooleanOpType::Union),
        ("積", BooleanOpType::Intersection),
        ("差", BooleanOpType::Difference),
    ] {
        match zenith_algo::BooleanEngine::boolean_solids_exact_result(&a, &b, op, &tol).map(|result| result.solids) {
            Ok(solids) => {
                let volume: f64 = solids
                    .iter()
                    .map(|solid| {
                        zenith_algo::MassCalculator::compute_from_brep(solid, &params).volume
                    })
                    .sum();
                let valid = solids
                    .iter()
                    .all(|solid| solid.outer_shell.validate_closed(&tol).errors.is_empty());
                println!(
                    "  {label}: 立体 {} 個、体積 {:.4}、閉じた殻 {}",
                    solids.len(),
                    volume,
                    if valid { "**全部 ok**" } else { "**破れあり**" }
                );
            }
            Err(reason) => println!("  {label}: 断り — {reason}"),
        }
    }

    println!();
    println!("**これは診断です。何も直していません。**");
}
