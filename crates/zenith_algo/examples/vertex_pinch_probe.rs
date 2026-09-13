//! **点でだけ繋がっている立体を、見つける**（4-450）。
//!
//! # なぜ要るのか
//!
//! 4-449 で 2 件出ました——**体積は小数第 4 位まで合っていて、形は
//! 割れている**立体です。**どちらも、接している所で 2 つの塊が
//! 繋がっているだけ**でした。
//!
//! **稜の数え方では出ません。** 稜は 2 回ずつ使われています。
//! **繋がっているのが稜ではなく点**だからです。
//!
//! **恒等式も出しません**（4-449）。**体積は本当に合っている**ので。
//!
//! # 何で測るか
//!
//! **頂点のまわり**です。
//!
//! 閉じた 2 次元多様体なら、**頂点のまわりの面は、稜を伝って 1 周
//! で繋がります**（頂点のリンクが 1 本の輪になる）。**2 つの塊が
//! 点で触れているなら、その頂点のまわりは 2 周**になります。
//!
//! **測り方**——頂点を公差で束ね、**その頂点に集まる面**を、
//! **その頂点に集まる稜を共有していれば同じ組**として繋げます。
//! **組が 2 つ以上できたら、そこがつままれた点**です。
//!
//! # 使い方
//!
//! ```bash
//! cargo run --release -p zenith_algo --example vertex_pinch_probe
//! ```
//!
//! **これは診断です。赤にはしません。**

use std::collections::BTreeMap;
use zenith_algo::{
    extrude_sketch, BooleanEngine, BooleanOpType, PrimitiveBuilder, SketchSolver, WorkPlane,
};
use zenith_math::{Point3, Tolerance, Vec3};
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

/// 公差で束ねた頂点の席。**丸めではなく、代表への寄せ**です。
fn seat_of(seats: &mut Vec<Point3>, point: Point3, limit: f64) -> usize {
    for (index, seat) in seats.iter().enumerate() {
        if (*seat - point).norm() <= limit {
            return index;
        }
    }
    seats.push(point);
    seats.len() - 1
}

fn root(parent: &mut [usize], mut index: usize) -> usize {
    while parent[index] != index {
        parent[index] = parent[parent[index]];
        index = parent[index];
    }
    index
}

/// **頂点のまわりが 1 周になっていない所**を返す。
fn pinched_vertices(solid: &Solid, tol: &Tolerance) -> Vec<(Point3, usize)> {
    let mut seats: Vec<Point3> = Vec::new();
    // 頂点 → その頂点に集まる (面, 稜の鍵)
    let mut around: BTreeMap<usize, Vec<(usize, (usize, usize))>> = BTreeMap::new();

    for (face_index, face) in solid.outer_shell.faces.iter().enumerate() {
        let wires = std::iter::once(&face.outer_wire).chain(face.inner_wires.iter());
        for wire in wires {
            for oriented in &wire.edges {
                let start = seat_of(&mut seats, oriented.edge.start_vertex.point, tol.linear);
                let end = seat_of(&mut seats, oriented.edge.end_vertex.point, tol.linear);
                if start == end {
                    continue;
                }
                // **稜の鍵は、端の組**（小さいほうを先に）。
                let key = (start.min(end), start.max(end));
                around.entry(start).or_default().push((face_index, key));
                around.entry(end).or_default().push((face_index, key));
            }
        }
    }

    let mut pinched = Vec::new();
    for (vertex, uses) in &around {
        let mut faces: Vec<usize> = uses.iter().map(|(face, _)| *face).collect();
        faces.sort_unstable();
        faces.dedup();
        if faces.len() < 3 {
            continue;
        }
        let index_of =
            |face: usize| faces.iter().position(|other| *other == face).unwrap_or_default();
        let mut parent: Vec<usize> = (0..faces.len()).collect();
        let mut by_edge: BTreeMap<(usize, usize), Vec<usize>> = BTreeMap::new();
        for (face, key) in uses {
            by_edge.entry(*key).or_default().push(index_of(*face));
        }
        for sharers in by_edge.values() {
            for pair in sharers.windows(2) {
                let left = root(&mut parent, pair[0]);
                let right = root(&mut parent, pair[1]);
                if left != right {
                    parent[left] = right;
                }
            }
        }
        let mut groups: Vec<usize> = (0..faces.len()).map(|i| root(&mut parent, i)).collect();
        groups.sort_unstable();
        groups.dedup();
        if groups.len() > 1 {
            pinched.push((seats[*vertex], groups.len()));
        }
    }
    pinched
}

struct Case {
    name: &'static str,
    a: Solid,
    b: Solid,
}

fn main() {
    // **この口を立てると、接している所で割れた答えが返ります**（4-449）。
    //
    // **試験ではなく例なので、ここで立てても他所に漏れません**
    // （4-443——試験は並列の糸なので、同じことをしてはいけません）。
    // **立てないと、断られて、測るものが手に入りません。**
    std::env::set_var("ZENITH_NO_PINCH_CHECK", "1");

    let tol = Tolerance::default();
    let plane = WorkPlane::xy();
    let mut cases: Vec<Case> = Vec::new();

    // 1. **穴を跨ぐ箱**（4-449 で出たほう）。箱は穴の外接正方形で、
    //    **4 点で接します**。積は 4 隅が接点でしか繋がりません。
    let mut plate = rectangle(0.0, 0.0, 40.0, 30.0);
    add_circle(&mut plate, 20.0, 15.0, 6.0);
    if let (Ok(solid), Ok(cutter)) = (
        extrude_sketch(&plate, &plane, 10.0, &tol),
        PrimitiveBuilder::make_box(12.0, 12.0, 40.0),
    ) {
        cases.push(Case {
            name: "穴のある板 × 箱（穴を跨ぐ）",
            a: solid,
            b: zenith_algo::BrepTransform::translate_solid(&cutter, Vec3::new(14.0, 9.0, -10.0)),
        });
    }

    // 2. **穴に接する帯**（4-449 のもう 1 件）。
    let mut holed = rectangle(0.0, 0.0, 30.0, 30.0);
    add_circle(&mut holed, 15.0, 15.0, 5.0);
    let bar = rectangle(10.0, -20.0, 8.0, 70.0);
    if let (Ok(a), Ok(b)) = (
        extrude_sketch(&holed, &plane, 8.0, &tol),
        extrude_sketch(&bar, &plane, 8.0, &tol),
    ) {
        cases.push(Case {
            name: "穴のある板 × 帯（穴に接する）",
            a,
            b: zenith_algo::BrepTransform::translate_solid(&b, Vec3::new(0.0, 0.0, -2.0)),
        });
    }

    // 3. **つままれていない置き方**（対照）。**見つけないことも測ります**
    //    ——**何にでも当たる物差しは、何も測っていません。**
    let mut plate3 = rectangle(0.0, 0.0, 40.0, 30.0);
    add_circle(&mut plate3, 20.0, 15.0, 6.0);
    if let (Ok(solid), Ok(cutter)) = (
        extrude_sketch(&plate3, &plane, 10.0, &tol),
        PrimitiveBuilder::make_box(12.0, 12.0, 40.0),
    ) {
        cases.push(Case {
            name: "穴のある板 × 箱（穴から離す）",
            a: solid,
            b: zenith_algo::BrepTransform::translate_solid(&cutter, Vec3::new(2.0, 2.0, -10.0)),
        });
    }
    if let (Ok(box_a), Ok(box_b)) = (
        PrimitiveBuilder::make_box(10.0, 10.0, 10.0),
        PrimitiveBuilder::make_box(10.0, 10.0, 10.0),
    ) {
        cases.push(Case {
            name: "箱 × 箱（重ねる）",
            a: box_a,
            b: zenith_algo::BrepTransform::translate_solid(&box_b, Vec3::new(5.0, 5.0, 5.0)),
        });
    }

    // 4. **本当に点で触れる置き方**。**物差しが何かに当たるか**を
    //    見ます——**当たらない物差しは、置いても意味がありません。**
    if let (Ok(box_a), Ok(box_b)) = (
        PrimitiveBuilder::make_box(10.0, 10.0, 10.0),
        PrimitiveBuilder::make_box(10.0, 10.0, 10.0),
    ) {
        cases.push(Case {
            name: "箱 × 箱（角で触れる）",
            a: box_a,
            b: zenith_algo::BrepTransform::translate_solid(&box_b, Vec3::new(10.0, 10.0, 10.0)),
        });
    }
    // 5. **稜で触れる置き方**。**点ではなく線**なので、
    //    **頂点の検査には出ないはず**です——**出ないことも測ります。**
    if let (Ok(box_a), Ok(box_b)) = (
        PrimitiveBuilder::make_box(10.0, 10.0, 10.0),
        PrimitiveBuilder::make_box(10.0, 10.0, 10.0),
    ) {
        cases.push(Case {
            name: "箱 × 箱（稜で触れる）",
            a: box_a,
            b: zenith_algo::BrepTransform::translate_solid(&box_b, Vec3::new(10.0, 10.0, 0.0)),
        });
    }

    println!("点でだけ繋がっている立体を、見つける（4-450）");
    println!();
    println!("**接触の検査は止めてあります**（ZENITH_NO_PINCH_CHECK=1）");
    println!("——**止めないと断られて、測るものが手に入りません。**");
    println!();
    println!(
        "{:<34}{:<14}{:>12}  {}",
        "置き方", "演算", "つままれた点", "場所"
    );
    println!("{}", "-".repeat(96));

    let mut found = 0usize;
    for case in &cases {
        for (label, op) in [
            ("union", BooleanOpType::Union),
            ("intersection", BooleanOpType::Intersection),
            ("difference", BooleanOpType::Difference),
        ] {
            match BooleanEngine::boolean_solids_exact_result(&case.a, &case.b, op, &tol) {
                Ok(result) => {
                    let mut all: Vec<(Point3, usize)> = Vec::new();
                    for solid in &result.solids {
                        all.extend(pinched_vertices(solid, &tol));
                    }
                    if !all.is_empty() {
                        found += 1;
                    }
                    let where_at = all
                        .iter()
                        .take(2)
                        .map(|(point, groups)| {
                            format!("({:.3},{:.3},{:.3})×{groups}", point.x, point.y, point.z)
                        })
                        .collect::<Vec<_>>()
                        .join(" ");
                    // **塊がいくつ返ったか**も出します（4-450）。
                    //
                    // **点でなく線で触れているなら、頂点の検査には
                    // 出ません。** **そして、触れているだけの 2 つの塊は、
                    // 別々の閉じた殻として返ることがあります**——
                    // **それは非多様体ではなく、合成体**です。
                    let closed = result
                        .solids
                        .iter()
                        .filter(|solid| {
                            solid.outer_shell.validate_closed(&tol).errors.is_empty()
                        })
                        .count();
                    println!(
                        "{:<34}{:<14}{:>12}  塊 {}（閉じ {}）  {}",
                        case.name,
                        label,
                        all.len(),
                        result.solids.len(),
                        closed,
                        where_at
                    );
                }
                Err(_) => println!("{:<34}{:<14}{:>12}  断り", case.name, label, "-"),
            }
        }
    }

    println!();
    println!("**つままれた点が見つかった演算: {found} 件**");
    println!();
    println!("**これは診断です。何も直していません。**");
}
