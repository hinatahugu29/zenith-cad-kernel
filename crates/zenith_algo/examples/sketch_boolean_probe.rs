//! **スケッチから作った立体を、ブーリアンの相手にする**（4-398）。
//!
//! # なぜ要るのか
//!
//! 2026/09/07 に、スケッチから**回転**（4-391）と**穴**（4-392）が
//! 通るようになりました。**そこで作った立体は、ビルダーのプリミティブ
//! とは位相が違います。**
//!
//! | | ビルダーの箱・円柱 | **スケッチから作った立体** |
//! | :--- | :--- | :--- |
//! | 面の持ち方 | 決まった枚数・決まった形 | **輪の本数で変わる** |
//! | 穴 | `HoleBuilder` が開ける | **最初から内側の輪として入っている** |
//! | 円弧 | 四半弧 4 本で全円 | **輪の一部**として混ざる |
//!
//! **その立体をブーリアンに掛けたことが、1 度もありません。**
//! このリポジトリで見つかった欠陥は、ほぼ全部「**測っていなかった領域に
//! プローブを当てたら出てきた**」ものでした（HANDOVER 3-0-0）。
//!
//! # 何で測るか
//!
//! **恒等式**です。**閉じた式が無くても、誤答を映します**（4-142、4-191）。
//!
//! ```text
//! |A∪B| + |A∩B| = |A| + |B|
//! |A＼B| + |A∩B| = |A|
//! ```
//!
//! **「立体が返った」で終わらせません。** **非多様体の稜も数えます。**
//!
//! **断りは誤答ではありません。** このリポジトリは**もっともらしい立体を
//! 返すより断るほう**を選びます（3-1）。**断った置き方は、理由ごと
//! 並べます。**
//!
//! # 読み方
//!
//! **恒等式が 1 つでも破れたら exit 1。** 断りは数えるだけです。

use zenith_algo::{
    extrude_sketch, revolve_sketch, BooleanEngine, BooleanOpType, MassCalculator, PrimitiveBuilder,
    SketchSolver, WorkPlane,
};
use zenith_math::{Point2, Tolerance, Vec3};
use zenith_tess::TessellationParams;
use zenith_topo::Solid;

fn volume_of(solids: &[Solid]) -> f64 {
    solids
        .iter()
        .map(|solid| MassCalculator::compute_from_brep(solid, &TessellationParams::default()).volume)
        .sum()
}

/// 立体のうち、相手のいない稜と非多様体の稜を数える。
fn edge_health(solids: &[Solid]) -> (usize, usize) {
    let mut unmatched = 0usize;
    let mut non_manifold = 0usize;
    for solid in solids {
        let report = solid.outer_shell.validate_closed(&Tolerance::default());
        for error in &report.errors {
            let text = format!("{error:?}");
            if text.contains("Unmatched") || text.contains("Boundary") {
                unmatched += 1;
            }
            if text.contains("NonManifold") {
                non_manifold += 1;
            }
        }
    }
    (unmatched, non_manifold)
}

/// 長方形のスケッチ。
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

/// 円を、四半弧 4 本でスケッチに足す。
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

struct Case {
    name: &'static str,
    a: Solid,
    b: Solid,
}

fn cases(tol: &Tolerance) -> Vec<Case> {
    let plane = WorkPlane::xy();
    let mut cases = Vec::new();

    // 1. **穴のある板** × 箱。**内側の輪を持つ立体**が相手です。
    let mut plate = rectangle(0.0, 0.0, 40.0, 30.0);
    add_circle(&mut plate, 20.0, 15.0, 6.0);
    if let Ok(solid) = extrude_sketch(&plate, &plane, 10.0, tol) {
        if let Ok(cutter) = PrimitiveBuilder::make_box(12.0, 12.0, 40.0) {
            cases.push(Case {
                name: "穴のある板 × 箱（穴から離す）",
                a: solid.clone(),
                b: zenith_algo::BrepTransform::translate_solid(
                    &cutter,
                    Vec3::new(2.0, 2.0, -10.0),
                ),
            });
            cases.push(Case {
                name: "穴のある板 × 箱（穴を跨ぐ）",
                a: solid,
                b: zenith_algo::BrepTransform::translate_solid(
                    &cutter,
                    Vec3::new(14.0, 9.0, -10.0),
                ),
            });
        }
    }

    // 2. **穴のある板** × 円柱。**曲面どうしが穴のところで出会います。**
    let mut plate2 = rectangle(0.0, 0.0, 40.0, 30.0);
    add_circle(&mut plate2, 20.0, 15.0, 6.0);
    if let (Ok(solid), Ok(pin)) = (
        extrude_sketch(&plate2, &plane, 10.0, tol),
        PrimitiveBuilder::make_cylinder(4.0, 40.0),
    ) {
        cases.push(Case {
            name: "穴のある板 × 円柱（縁にかける）",
            a: solid,
            b: zenith_algo::BrepTransform::translate_solid(&pin, Vec3::new(8.0, 15.0, -10.0)),
        });
    }

    // 3. **回した立体**（リング） × 箱。**掃いた曲面**が相手です。
    let ring_sketch = rectangle(10.0, 0.0, 4.0, 6.0);
    if let Ok(ring) = revolve_sketch(
        &ring_sketch,
        &plane,
        Point2::new(0.0, 0.0),
        Point2::new(0.0, 1.0),
        tol,
    ) {
        if let Ok(cutter) = PrimitiveBuilder::make_box(30.0, 30.0, 4.0) {
            cases.push(Case {
                name: "回した輪 × 箱（半分を切る）",
                a: ring.clone(),
                b: zenith_algo::BrepTransform::translate_solid(
                    &cutter,
                    Vec3::new(-15.0, -15.0, 1.0),
                ),
            });
        }
        if let Ok(post) = PrimitiveBuilder::make_cylinder(5.0, 30.0) {
            cases.push(Case {
                name: "回した輪 × 円柱（軸を通す）",
                a: ring,
                b: zenith_algo::BrepTransform::translate_solid(
                    &post,
                    Vec3::new(0.0, 0.0, -12.0),
                ),
            });
        }
    }

    // 4. **回した立体どうし。** どちらも掃いた曲面です。
    let inner = rectangle(6.0, 0.0, 3.0, 5.0);
    let outer = rectangle(10.0, 1.0, 3.0, 5.0);
    if let (Ok(a), Ok(b)) = (
        revolve_sketch(
            &inner,
            &plane,
            Point2::new(0.0, 0.0),
            Point2::new(0.0, 1.0),
            tol,
        ),
        revolve_sketch(
            &outer,
            &plane,
            Point2::new(0.0, 0.0),
            Point2::new(0.0, 1.0),
            tol,
        ),
    ) {
        cases.push(Case {
            name: "回した輪 × 回した輪（離れている）",
            a,
            b,
        });
    }

    // 5. **押し出した板どうし**（片方は穴つき）。
    let mut holed = rectangle(0.0, 0.0, 30.0, 30.0);
    add_circle(&mut holed, 15.0, 15.0, 5.0);
    // **帯の縁を、穴に接しない所へ置きます。**
    //
    // **最初は `x = 10` から始めました**——穴は中心 (15, 15)・半径 5 なので、
    // **帯の縁が穴にちょうど接します**。3 演算とも断られました（積は
    // 「接触だけで 2 つに割れる」と点まで名指し。**規約どおりの断り**です）。
    // **接する置き方は、下に別の検体として残します。**
    let bar = rectangle(11.0, -20.0, 8.0, 70.0);
    if let (Ok(a), Ok(b)) = (
        extrude_sketch(&holed, &plane, 8.0, tol),
        extrude_sketch(&bar, &plane, 8.0, tol),
    ) {
        cases.push(Case {
            name: "穴のある板 × 帯（穴を通る）",
            a,
            b: zenith_algo::BrepTransform::translate_solid(&b, Vec3::new(0.0, 0.0, -2.0)),
        });
    }

    // 6. **帯の縁が、穴にちょうど接する置き方。**
    //
    // **接触は、それ自体では位相を作りません**（3-1）。**断るのが正しい
    // 置き方**で、**もっともらしい立体を返すほうが困ります。**
    let mut holed2 = rectangle(0.0, 0.0, 30.0, 30.0);
    add_circle(&mut holed2, 15.0, 15.0, 5.0);
    let tangent_bar = rectangle(10.0, -20.0, 8.0, 70.0);
    if let (Ok(a), Ok(b)) = (
        extrude_sketch(&holed2, &plane, 8.0, tol),
        extrude_sketch(&tangent_bar, &plane, 8.0, tol),
    ) {
        cases.push(Case {
            name: "穴のある板 × 帯（穴に接する）",
            a,
            b: zenith_algo::BrepTransform::translate_solid(&b, Vec3::new(0.0, 0.0, -2.0)),
        });
    }

    cases
}

fn main() {
    let tol = Tolerance::default();
    let cases = cases(&tol);

    println!("スケッチから作った立体を、ブーリアンの相手にする（4-398）");
    println!();
    println!(
        "{:<34}{:>13}{:>13}{:>13}{:>12}  {}",
        "置き方", "|A∪B|", "|A∩B|", "|A＼B|", "恒等式の残差", "非多様体"
    );
    println!("{}", "-".repeat(112));

    let mut broken = 0usize;
    let mut refused = 0usize;
    let mut worst_residual = 0.0f64;
    let mut notes: Vec<String> = Vec::new();

    for case in &cases {
        let mut volumes: [Option<f64>; 3] = [None, None, None];
        let mut non_manifold_total = 0usize;
        for (index, (label, op)) in [
            ("union", BooleanOpType::Union),
            ("intersection", BooleanOpType::Intersection),
            ("difference", BooleanOpType::Difference),
        ]
        .into_iter()
        .enumerate()
        {
            match BooleanEngine::boolean_solids_exact_result(&case.a, &case.b, op, &tol) {
                Ok(result) => {
                    volumes[index] = Some(volume_of(&result.solids));
                    let (_, non_manifold) = edge_health(&result.solids);
                    non_manifold_total += non_manifold;
                }
                Err(reason) => {
                    refused += 1;
                    notes.push(format!("  {} / {label}: {reason}", case.name));
                }
            }
        }

        let va = volume_of(std::slice::from_ref(&case.a));
        let vb = volume_of(std::slice::from_ref(&case.b));
        let (union, intersection, difference) = (volumes[0], volumes[1], volumes[2]);

        let residual = match (union, intersection, difference) {
            (Some(u), Some(i), Some(d)) => {
                let scale = (va + vb).abs().max(1.0);
                let first = ((u + i) - (va + vb)).abs() / scale;
                let second = ((d + i) - va).abs() / scale;
                let worst = first.max(second);
                if worst > 1e-9 {
                    broken += 1;
                }
                worst_residual = worst_residual.max(worst);
                Some(worst)
            }
            _ => None,
        };

        let show = |value: Option<f64>| match value {
            Some(v) => format!("{v:.4}"),
            None => "断り".to_string(),
        };
        println!(
            "{:<34}{:>13}{:>13}{:>13}{:>12}  {}",
            case.name,
            show(union),
            show(intersection),
            show(difference),
            residual
                .map(|r| format!("{r:.3e}"))
                .unwrap_or_else(|| "-".to_string()),
            non_manifold_total
        );
    }

    println!("{}", "-".repeat(112));
    if !notes.is_empty() {
        println!();
        println!("断り文:");
        for note in &notes {
            println!("{note}");
        }
    }
    println!();
    println!(
        "置き方 {} 個、恒等式の破れ {broken} 件（残差の最悪 {worst_residual:.3e}）、断り {refused} 件",
        cases.len()
    );
    if broken > 0 {
        std::process::exit(1);
    }
}
