//! ブーリアンの検証ゲートの答えが、テッセレーションの設定で変わらないか。
//!
//! ゲートは内外判定を**表示用メッシュへの射線**で行っている。メッシュが真の面
//! からどれだけ離れているかがそのまま誤判定の帯になるので、分割数を変えると
//! 答えが変わりうる。ゲートは正しさを見る仕組みなので、表示の設定で結論が
//! 動いてはいけない。
//!
//! ここでは
//!
//! 1. **正しい結果**を分割数を振ってかけ、どの設定でも通るか
//! 2. **わざと間違えた結果**（オペランドAをそのまま答えとして渡す）を同じく
//!    かけ、どの設定でも落ちるか
//!
//! を見る。1で落ちれば偽陽性（正しい形が弾かれる）、2で通れば偽陰性
//! （誤答が素通りする）。偽陰性のほうが重い。

use zenith_algo::{
    BooleanEngine, BooleanOpType, BooleanResultVerifier, BooleanVerificationParams, BrepTransform,
    PrimitiveBuilder,
};
use zenith_math::{Tolerance, Vec3};
use zenith_tess::TessellationParams;
use zenith_topo::Solid;

struct Case {
    name: &'static str,
    a: Solid,
    b: Solid,
    op: BooleanOpType,
    result: Vec<Solid>,
    /// 正しい結果なら true
    should_pass: bool,
}

fn main() {
    let tol = Tolerance::default();
    let densities = [4usize, 6, 8, 12, 16, 24, 32];

    let block = PrimitiveBuilder::make_box(40.0, 40.0, 20.0).unwrap();
    let bore = BrepTransform::translate_solid(
        &PrimitiveBuilder::make_cylinder(6.0, 40.0).unwrap(),
        Vec3::new(20.0, 20.0, -10.0),
    );
    let corner = BrepTransform::translate_solid(
        &PrimitiveBuilder::make_box(20.0, 20.0, 20.0).unwrap(),
        Vec3::new(20.0, 20.0, 0.0),
    );
    let boss = BrepTransform::translate_solid(
        &PrimitiveBuilder::make_cylinder(6.0, 30.0).unwrap(),
        Vec3::new(20.0, 20.0, 20.0),
    );
    // 曲面がよく効く相手も入れる。射線が当たる面がすべて曲面だと、
    // メッシュの近似誤差が一番効く。
    let ball = BrepTransform::translate_solid(
        &PrimitiveBuilder::make_sphere(14.0).unwrap(),
        Vec3::new(20.0, 20.0, 10.0),
    );
    let ring = BrepTransform::translate_solid(
        &PrimitiveBuilder::make_torus(14.0, 5.0).unwrap(),
        Vec3::new(20.0, 20.0, 10.0),
    );

    let mut cases: Vec<Case> = Vec::new();
    for (name, tool, op) in [
        ("block minus a bore", &bore, BooleanOpType::Difference),
        ("block minus a corner", &corner, BooleanOpType::Difference),
        ("block union a boss", &boss, BooleanOpType::Union),
        ("block minus a ball", &ball, BooleanOpType::Difference),
        ("block intersect a ball", &ball, BooleanOpType::Intersection),
        ("block minus a ring", &ring, BooleanOpType::Difference),
    ] {
        let Ok(result) =
            BooleanEngine::boolean_solids_exact_result(&block, tool, op, &tol)
        else {
            continue;
        };
        cases.push(Case {
            name,
            a: block.clone(),
            b: tool.clone(),
            op,
            result: result.solids,
            should_pass: true,
        });
        // 誤答: オペランドAをそのまま答えとして返す。閉多様体だが答えではない。
        cases.push(Case {
            name,
            a: block.clone(),
            b: tool.clone(),
            op,
            result: vec![block.clone()],
            should_pass: false,
        });
    }

    // 体積の境界では捕まらない誤答。離れた道具との差は A そのものなので、
    // **A を平行移動した複製**は体積が同じで、境界の検査を全部通る。
    // 内外判定だけが捕まえられる。
    let far_tool = BrepTransform::translate_solid(
        &PrimitiveBuilder::make_box(10.0, 10.0, 10.0).unwrap(),
        Vec3::new(200.0, 0.0, 0.0),
    );
    if let Ok(result) = BooleanEngine::boolean_solids_exact_result(
        &block,
        &far_tool,
        BooleanOpType::Difference,
        &tol,
    ) {
        cases.push(Case {
            name: "minus a disjoint tool",
            a: block.clone(),
            b: far_tool.clone(),
            op: BooleanOpType::Difference,
            result: result.solids,
            should_pass: true,
        });
    }
    cases.push(Case {
        name: "minus a disjoint tool",
        a: block.clone(),
        b: far_tool.clone(),
        op: BooleanOpType::Difference,
        // 体積は A と同じ。形だけが違う。
        result: vec![BrepTransform::translate_solid(&block, Vec3::new(0.0, 0.0, 60.0))],
        should_pass: false,
    });

    print!("{:<24}{:<9}", "case", "answer");
    for density in densities {
        print!("{:>10}", format!("{density}x{density}"));
    }
    println!();
    println!("{}", "-".repeat(24 + 9 + 10 * densities.len()));

    let mut unstable = 0;
    let mut false_negative = 0;
    for case in &cases {
        print!(
            "{:<24}{:<9}",
            case.name,
            if case.should_pass { "correct" } else { "WRONG" }
        );
        let mut verdicts = Vec::new();
        for density in densities {
            let params = BooleanVerificationParams {
                tessellation: TessellationParams {
                    u_divisions: density,
                    v_divisions: density,
                },
                ..Default::default()
            };
            let report = BooleanResultVerifier::verify_with_params(
                &case.a,
                &case.b,
                &case.result,
                case.op,
                &tol,
                &params,
            );
            let passed = report.is_valid();
            verdicts.push(passed);
            print!(
                "{:>10}",
                if passed == case.should_pass {
                    if passed { "pass" } else { "reject" }.to_string()
                } else if passed {
                    "LET IN".to_string()
                } else {
                    "REJECTED".to_string()
                }
            );
        }
        println!();

        if verdicts.iter().any(|v| *v != verdicts[0]) {
            unstable += 1;
        }
        if !case.should_pass && verdicts.iter().any(|v| *v) {
            false_negative += 1;
        }
    }

    println!("{}", "-".repeat(24 + 9 + 10 * densities.len()));
    println!("verdicts that change with the tessellation setting: {unstable}");
    println!("wrong answers let through at some setting          : {false_negative}");
    if unstable > 0 || false_negative > 0 {
        println!();
        println!("A gate whose answer depends on a display setting is not a gate.");
        std::process::exit(1);
    }
    println!("the gate gives the same answer at every tessellation tried");
}
