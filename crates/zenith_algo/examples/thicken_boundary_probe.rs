//! **板を厚くする**を、**縁と穴**で測る（`ThickenBuilder`）。
//!
//! # なぜ要るのか
//!
//! 4-390 で `thicken_shell` を閉じた式に当てたとき、**置いた検体は
//! 長方形の板だけ**でした。**縁が直線で、穴が無い形**です。
//!
//! HANDOVER 3-0-0 は、それを見て「平面の板は完了」と書いています。
//! **測っていないのは、縁が弧のとき（丸板・角丸の板）と、穴のあるとき**です。
//! **どちらも板金では普通に出てきます。**
//!
//! # 何を測るか
//!
//! **体積だけ**です。**閉じた式で出せるものしか置きません。**
//!
//! ```text
//! 丸板       体積 = π r² × 厚み
//! 穴あき板   体積 = (W·H − π r²) × 厚み
//! 角丸の板   体積 = (W·H − (4 − π) r²) × 厚み
//! ```
//!
//! # 読み方
//!
//! **断られたら、それも答え**です。**「まだ扱えない」と名乗って断るのは
//! 正しい**——**困るのは、閉じた多様体で、形も一見それらしいのに、
//! 大きさだけ違う立体が返ってくること**です。**それは形の検査では
//! 捕まりません**（3-0-0 の「ミリ以外の単位の STEP」と同じ型）。

use std::f64::consts::PI;

use zenith_algo::{MassCalculator, ProfileBuilder, ThickenBuilder};
use zenith_geom::PlaneSurface3;
use zenith_math::{Point3, Tolerance, Vec3};
use zenith_tess::TessellationParams;
use zenith_topo::{Edge, Face, FaceGeometry, Orientation, OrientedEdge, Vertex, Wire};

/// xy 平面上の長方形ワイヤ（反時計回り）。
fn rect_wire(x0: f64, y0: f64, w: f64, h: f64) -> Wire {
    let pts = [
        Point3::new(x0, y0, 0.0),
        Point3::new(x0 + w, y0, 0.0),
        Point3::new(x0 + w, y0 + h, 0.0),
        Point3::new(x0, y0 + h, 0.0),
    ];
    let verts: Vec<Vertex> = pts.iter().map(|p| Vertex::from_point(*p)).collect();
    let mut edges = Vec::with_capacity(4);
    for index in 0..4 {
        let next = (index + 1) % 4;
        edges.push(OrientedEdge::forward(
            Edge::line_between(verts[index].clone(), verts[next].clone()).unwrap(),
        ));
    }
    Wire::new(edges)
}

fn xy_plane() -> PlaneSurface3 {
    PlaneSurface3::new(
        Point3::new(0.0, 0.0, 0.0),
        Vec3::new(1.0, 0.0, 0.0),
        Vec3::new(0.0, 1.0, 0.0),
    )
    .unwrap()
}

fn planar_face(outer: Wire, inner: Vec<Wire>) -> Face {
    Face::new(
        FaceGeometry::Plane(xy_plane()),
        outer,
        inner,
        Orientation::Forward,
        1e-6,
    )
}

struct Case {
    name: &'static str,
    face: Face,
    thickness: f64,
    /// 閉じた式で出した体積。
    expected: f64,
    /// **もし縁を端点だけで結んだら**こうなる、という値。**予想であって
    /// 要求ではありません**——一致したら、そう作られていると分かります。
    polygonized: Option<f64>,
    /// 立体が乗っているべき z の範囲。**体積では表裏を見分けられません**
    /// ——反対側に同じだけ厚くなっても、体積は同じです。
    z_range: (f64, f64),
    note: &'static str,
}

fn cases() -> Vec<Case> {
    let t = 3.0;
    let r = 10.0;

    let circle = ProfileBuilder::make_circle(
        r,
        Point3::new(0.0, 0.0, 0.0),
        Vec3::new(0.0, 0.0, 1.0),
        Vec3::new(1.0, 0.0, 0.0),
    )
    .expect("circle wire");

    let hole = ProfileBuilder::make_circle(
        5.0,
        Point3::new(20.0, 15.0, 0.0),
        Vec3::new(0.0, 0.0, 1.0),
        Vec3::new(1.0, 0.0, 0.0),
    )
    .expect("hole wire");

    let hole_a = ProfileBuilder::make_circle(
        4.0,
        Point3::new(12.0, 15.0, 0.0),
        Vec3::new(0.0, 0.0, 1.0),
        Vec3::new(1.0, 0.0, 0.0),
    )
    .expect("hole a");
    let hole_b = ProfileBuilder::make_circle(
        3.0,
        Point3::new(28.0, 15.0, 0.0),
        Vec3::new(0.0, 0.0, 1.0),
        Vec3::new(1.0, 0.0, 0.0),
    )
    .expect("hole b");

    vec![
        Case {
            name: "長方形の板（既知の緑）",
            face: planar_face(rect_wire(0.0, 0.0, 40.0, 30.0), vec![]),
            thickness: t,
            expected: 40.0 * 30.0 * t,
            polygonized: None,
            z_range: (0.0, 3.0),
            note: "4-390 で測ってある形。ここが赤なら、測り方のほうが壊れています",
        },
        Case {
            name: "丸板（縁が弧）",
            face: planar_face(circle.clone(), vec![]),
            thickness: t,
            expected: PI * r * r * t,
            // 弧の端点は (±r,0)・(0,±r) の 4 点。直線で結ぶと、対角 2r の
            // 正方形（面積 2r²）になります。
            polygonized: Some(2.0 * r * r * t),
            z_range: (0.0, 3.0),
            note: "縁を端点だけで結ぶと、円が内接正方形になります",
        },
        Case {
            name: "丸板（厚みが負）",
            face: planar_face(circle, vec![]),
            thickness: -t,
            // **符号は向きだけを決めます。** 反対側へ同じだけ厚く
            // なるので、体積は同じです。**押し出しの口は「押し出す
            // 向きから見て反時計回り」を前提にする**ので、負の厚みでは
            // 輪を反転してから渡さなければなりません。
            expected: PI * r * r * t,
            polygonized: None,
            z_range: (-3.0, 0.0),
            note: "負の厚みでも、輪の向きを揃え直しているか。**体積は表裏で同じ**なので、z も見ます",
        },
        Case {
            name: "角丸の板（弧と直線）",
            face: planar_face(
                ProfileBuilder::make_rounded_rectangle(
                    40.0,
                    30.0,
                    6.0,
                    Point3::new(0.0, 0.0, 0.0),
                    Vec3::new(0.0, 0.0, 1.0),
                    Vec3::new(1.0, 0.0, 0.0),
                )
                .expect("rounded rectangle"),
                vec![],
            ),
            thickness: t,
            // 角丸は、4 隅から (4 − π) r² ぶんを削った形です。
            expected: (40.0 * 30.0 - (4.0 - PI) * 36.0) * t,
            polygonized: None,
            z_range: (0.0, 3.0),
            note: "弧と直線が混じる輪",
        },
        Case {
            name: "穴のある板",
            face: planar_face(rect_wire(0.0, 0.0, 40.0, 30.0), vec![hole]),
            thickness: t,
            expected: (40.0 * 30.0 - PI * 25.0) * t,
            // 内側の輪を読まなければ、穴の無い板がそのまま返ります。
            polygonized: Some(40.0 * 30.0 * t),
            z_range: (0.0, 3.0),
            note: "内側の輪（穴）を読んでいるか",
        },
        Case {
            name: "穴が2つある板",
            face: planar_face(rect_wire(0.0, 0.0, 40.0, 30.0), vec![hole_a, hole_b]),
            thickness: t,
            expected: (40.0 * 30.0 - PI * 16.0 - PI * 9.0) * t,
            polygonized: Some(40.0 * 30.0 * t),
            z_range: (0.0, 3.0),
            note: "内側の輪が 2 本あっても、両方読んでいるか",
        },
    ]
}

fn main() {
    let tol = Tolerance::default();
    let params = TessellationParams::default();

    println!("板を厚くする（thicken_face）を、縁と穴で測る");
    println!();
    println!(
        "{:<26}{:>16}{:>16}{:>12}  {}",
        "置き方", "閉じた式", "測った体積", "相対差", "結果"
    );
    println!("{}", "-".repeat(96));

    let mut wrong = 0usize;
    let mut refused = 0usize;
    let mut notes: Vec<String> = Vec::new();

    for case in cases() {
        match ThickenBuilder::thicken_face(&case.face, case.thickness, &tol) {
            Ok(solid) => {
                let volume = MassCalculator::compute_from_brep(&solid, &params).volume;
                let residual = (volume - case.expected).abs() / case.expected.abs().max(1.0);
                let bbox = solid.bounding_box();
                // **x と y は見ません**——有理2次の弧は制御点が弧の外へ
                // 出るので、`bounding_box()` は制御点ぶん広く出ます
                // （4-269 と同じ穴）。**z は制御点も面の上に乗る**ので、
                // ここだけは正しく読めます。
                let z_ok = (bbox.min.z - case.z_range.0).abs() <= 1e-9
                    && (bbox.max.z - case.z_range.1).abs() <= 1e-9;
                let ok = residual <= 1e-6 && z_ok;
                if !ok {
                    wrong += 1;
                }
                if !z_ok {
                    notes.push(format!(
                        "  {}: **z の範囲が違います**——[{:.6}, {:.6}]、あるべきは [{:.6}, {:.6}]",
                        case.name, bbox.min.z, bbox.max.z, case.z_range.0, case.z_range.1
                    ));
                }
                println!(
                    "{:<26}{:>16.6}{:>16.6}{:>12.3e}  {}",
                    case.name,
                    case.expected,
                    volume,
                    residual,
                    if ok { "ok" } else { "**ちがう**" }
                );
                if !ok {
                    if let Some(guess) = case.polygonized {
                        let gap = (volume - guess).abs() / guess.abs().max(1.0);
                        if gap <= 1e-6 {
                            notes.push(format!(
                                "  {}: **予想と一致しました**（{guess:.6}）。{}",
                                case.name, case.note
                            ));
                        } else {
                            notes.push(format!(
                                "  {}: 予想（{guess:.6}）とも違います。{}",
                                case.name, case.note
                            ));
                        }
                    } else {
                        notes.push(format!("  {}: {}", case.name, case.note));
                    }
                }
            }
            Err(reason) => {
                refused += 1;
                println!(
                    "{:<26}{:>16.6}{:>16}{:>12}  断り",
                    case.name, case.expected, "-", "-"
                );
                notes.push(format!("  {}: 断り文「{reason}」", case.name));
            }
        }
    }

    if !notes.is_empty() {
        println!();
        for note in &notes {
            println!("{note}");
        }
    }

    println!();
    println!("誤答 {wrong} 件 / 断り {refused} 件");
    if wrong > 0 {
        println!();
        println!("**誤答は、閉じた多様体で返ってきています。** 形の検査では捕まりません。");
    }
}
