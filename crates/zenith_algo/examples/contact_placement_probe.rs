//! 接触している配置を、演算ごとに数える。
//!
//! `boolean_envelope` の45ケースのうち、いま6件が通りません。この文書は
//! 長らく「6件はすべて接線配置」と書いていましたが、2026/08/24 に測ったら
//! **内訳が違いました**（HANDOVER 3-1）。ここはその内訳を、いつでも
//! 測り直せるようにしたものです。
//!
//! ## 規約（HANDOVER 3-1 で決めたもの）
//!
//! **接触は、それ自体では位相を作りません。** 2つの曲面が交わらずに触れて
//! いるだけの場所には、交線を作らない。そのうえで、
//!
//! - 真の答えが多様体の立体になるなら、**返す**
//! - 真の答えが**本当に非多様体**なら、**名指しして断る**
//!
//! `Solid` は多様体 B-Rep なので、非多様体を「もっともらしい立体」にして
//! 返すのは誤答です。**断るのは実装の不足ではなく、型が表現できないものを
//! 表現したふりをしない、という設計判断です。**
//!
//! ## この表の読み方
//!
//! **「非多様体のはずだ」と決めつけません。返ってきた立体を測ります。**
//! 立体が返ったら、そのメッシュの稜が**ちょうど2枚の三角形に共有されて
//! いるか**を数えます。共有されていない稜が1本でもあれば、それは多様体
//! ではありません。
//!
//! - `REFUSED` — 断られた。**赤にはしません**。まだ実装していないだけの
//!   ことがあるからです（直したぶんだけ `ok` が増えます）
//! - `ok` かつ 非多様体の稜 0 — 多様体の立体が返った
//! - `ok` かつ 非多様体の稜 > 0 — **赤**。非多様体を立体として返すのは、
//!   断るより悪い
//!
//! 予想は `note` の列に書いてあります。**予想と実測が食い違ったら、まず
//! 予想を疑ってください**（5章、4-33）。
//!
//! ```bash
//! cargo run --release -p zenith_algo --example contact_placement_probe
//! ZENITH_SPLIT_WHY=1 cargo run --release -p zenith_algo --example contact_placement_probe
//! ```

use zenith_algo::{BooleanEngine, BooleanOpType, BrepTransform, PrimitiveBuilder};
use zenith_math::{Tolerance, Transform3, Vec3};
use zenith_topo::Solid;

struct Case {
    name: &'static str,
    /// なぜ接触配置なのか。
    why: &'static str,
    a: Solid,
    b: Solid,
    /// union / difference / intersection それぞれについての**予想**。
    /// 実測の相手であって、判定の根拠ではない。
    note: [&'static str; 3],
}

/// メッシュの稜のうち、ちょうど2枚の三角形に共有されていない本数。
///
/// 0 でなければ多様体ではない。`tessellate_solid` は頂点を束ねてから
/// 返すので、座標が一致する頂点は同じ添字になっている。
fn non_manifold_edges(solid: &Solid) -> usize {
    let mesh = zenith_tess::tessellate_solid(
        solid,
        &zenith_tess::TessellationParams {
            u_divisions: 24,
            v_divisions: 24,
        },
    );
    let mut uses: std::collections::HashMap<(u32, u32), usize> = std::collections::HashMap::new();
    for triangle in &mesh.indices {
        for step in 0..3 {
            let (a, b) = (triangle[step], triangle[(step + 1) % 3]);
            if a == b {
                continue;
            }
            let key = if a < b { (a, b) } else { (b, a) };
            *uses.entry(key).or_insert(0) += 1;
        }
    }
    uses.values().filter(|count| **count != 2).count()
}

fn shifted(solid: &Solid, x: f64, y: f64, z: f64) -> Solid {
    BrepTransform::translate_solid(solid, Vec3::new(x, y, z))
}

fn cases() -> Vec<Case> {
    let boxa = PrimitiveBuilder::make_box(20.0, 20.0, 20.0).expect("box");
    let cylinder = PrimitiveBuilder::make_cylinder(6.0, 40.0).expect("cylinder");
    let sphere = PrimitiveBuilder::make_sphere(10.0).expect("sphere");

    let mut out = Vec::new();

    // `boolean_envelope` と同じ置き方。B は (10,10,0) ずらしてから
    // **世界原点まわりに** 45 度回す（箱の中心まわりではない。そこを
    // 取り違えると 3演算とも通ってしまい、再現しない）。
    let turn = Transform3::from_axis_angle(&Vec3::new(0.0, 0.0, 1.0), std::f64::consts::FRAC_PI_4);
    out.push(Case {
        name: "box x box (45deg about Z)",
        why: "B の角 (10,10) が回って (0, 10√2) に来る。頂点が A の面 x=0 の上に乗る",
        a: boxa.clone(),
        b: BrepTransform::transform_solid(&shifted(&boxa, 10.0, 10.0, 0.0), &turn).expect("turn"),
        note: [
            "通っている",
            "接触の交線を落とせば多様体になるはず",
            "同上",
        ],
    });

    // 円柱が箱の側面にちょうど接する。
    out.push(Case {
        name: "box x cylinder (tangent)",
        why: "円柱の側面が箱の面に線で接する",
        a: boxa.clone(),
        // `boolean_envelope` と同じ: 半径6を中心 (6, 10) に置くと x=0 面に接する。
        b: shifted(&cylinder, 6.0, 10.0, -10.0),
        note: [
            "接触線で4面が集まるので非多様体のはず",
            "接触は体積を囲まないので A そのもの",
            "空が正しい答え",
        ],
    });

    // 箱の4面が球に接し、5面目が中心を通る。
    out.push(Case {
        name: "box x sphere (four faces tangent)",
        why: "球が箱の断面に内接し、5面目（x=20）が中心を通る。**その平面は球の極 (20,10,0) と (20,10,20) を両方通る**——交線がそこで点に潰れる",
        a: boxa.clone(),
        // `boolean_envelope` と同じ置き方。**向きが効きます。** 中心を
        // (10,10,20) に置いて赤道で切ると3演算とも通りますが、それは
        // 継ぎ目を踏んでいないだけで、別の配置を測ったことになります。
        b: shifted(&sphere, 20.0, 10.0, 10.0),
        note: [
            "半球の縁が箱の面の境界に4点で触れる",
            "箱から半球を引いたもの",
            "半球",
        ],
    });

    // **同じ「4面が接する」配置を、向きだけ変えたもの。**
    // 中心を (10,10,20) に置くと、切る平面 z=20 は赤道の面で、**極を通り
    // ません**。上の配置（x=20 で切る）は極 (20,10,0) と (20,10,20) を
    // 両方通り、交線がそこで点に潰れます。
    // 接している面の数は同じなので、**差が出るならそれは接触のせいでは
    // ありません**。
    out.push(Case {
        name: "box x sphere (cut at the equator)",
        why: "上と同じ4面接触。ただし切る平面（z=20、赤道の面）は極を通らない",
        a: boxa.clone(),
        b: shifted(&sphere, 10.0, 10.0, 20.0),
        note: ["上との違いは向きだけ", "同上", "同上"],
    });

    out
}

fn main() {
    let tol = Tolerance::default();
    let mut wrong = 0usize;
    let mut returned = 0usize;
    let mut refused = 0usize;

    println!("接触している配置（規約: 接触は、それ自体では位相を作らない）");
    println!();
    println!(
        "{:<30} {:<13} {:>9} {:>7} {:>13}  {}",
        "case", "op", "result", "solids", "non-manifold", "verdict / 予想"
    );
    println!("{}", "-".repeat(118));

    for case in cases() {
        for (index, (label, op)) in [
            ("union", BooleanOpType::Union),
            ("difference", BooleanOpType::Difference),
            ("intersection", BooleanOpType::Intersection),
        ]
        .into_iter()
        .enumerate()
        {
            let outcome = BooleanEngine::boolean_solids_exact_result(&case.a, &case.b, op, &tol);

            let (result, solids, bad_edges, verdict) = match &outcome {
                Err(_) => {
                    refused += 1;
                    (
                        "REFUSED",
                        "-".to_string(),
                        "-".to_string(),
                        // 断るのは、まだ実装していないだけのことがあります。
                        // ここでは赤にしません。
                        format!("断られた（{}）", case.note[index]),
                    )
                }
                Ok(result) => {
                    returned += 1;
                    // **返ってきたものを測ります。** 稜がちょうど2枚の三角形に
                    // 共有されていなければ、それは多様体ではありません。
                    let bad: usize = result.solids.iter().map(non_manifold_edges).sum();
                    if bad > 0 {
                        wrong += 1;
                    }
                    (
                        "ok",
                        result.solids.len().to_string(),
                        bad.to_string(),
                        if bad > 0 {
                            format!("WRONG: 非多様体を立体として返した（{}）", case.note[index])
                        } else {
                            format!("ok, 多様体（{}）", case.note[index])
                        },
                    )
                }
            };

            println!(
                "{:<30} {:<13} {:>9} {:>7} {:>13}  {}",
                if index == 0 { case.name } else { "" },
                label,
                result,
                solids,
                bad_edges,
                verdict
            );
        }
        println!("{:<30} {}", "", case.why);
        println!();
    }

    println!("{}", "-".repeat(118));
    println!("{returned} 件が立体を返し、{refused} 件が断られました。**非多様体を返したもの {wrong} 件。**");
    println!();
    println!("**断られること自体は、ここでは赤にしません。** まだ実装していないだけの");
    println!("ことがあります。赤にするのは「非多様体を立体として返した」ほうです。");
    println!("予想は括弧の中です。**予想と実測が食い違ったら、まず予想を疑ってください。**");

    if wrong > 0 {
        std::process::exit(1);
    }
}
