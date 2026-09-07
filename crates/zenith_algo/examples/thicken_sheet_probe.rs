//! **板を厚くする**（`ThickenBuilder::thicken_shell`）を、閉じた式で測る。
//!
//! # なぜ要るのか
//!
//! HANDOVER 3-0-0 の「複数面シート厚み付け」は**部分的**のままでした。
//!
//! > `thicken_shell` は各面を個別に厚み付けして Union するだけ。
//! > **検体は同一平面の長方形2枚のみ**
//!
//! **同一平面の 2 枚**は、いちばん優しい形です。**面が折れたとき**を
//! 1 度も測っていませんでした。
//!
//! # 何を測るか
//!
//! **体積だけ**です。**閉じた式で出せるものしか置きません。**
//!
//! **厚みの向きは、面の法線（`u × v`）です。** **折れた板を厚くすると、
//! 板どうしは触れるだけとは限らず、角で重なります**——**重なりを引いた
//! 包除の式**を、置き方ごとに手で書いてあります。
//!
//! ```text
//! 平らな板   体積 = 面積 × 厚み
//! L 字       体積 = 板A + 板B − 重なり
//! ```
//!
//! **法線の向きを確かめずに式を書くと、外します**——**最初にそれを
//! やりました**（4-390。4050 と書いて、正しいのは 3780 でした）。
//! **カーネルを疑う前に、自分の式を疑ってください**（3-0-0 の注意書き）。
//!
//! # 読み方
//!
//! **断られたら、それも答え**です。**規約どおりの断りなのか、未実装なのか**を
//! 名前で見てください。**「もっともらしい立体」が返るほうが困ります。**
//! **離れた 2 枚は、断るのが正しい置き方**です——`thicken_shell` は
//! 立体を 1 つ返す口を使うので、2 つに分かれる答えは返せません。

use zenith_algo::{MassCalculator, ThickenBuilder};
use zenith_geom::PlaneSurface3;
use zenith_math::{Point3, Tolerance};
use zenith_tess::TessellationParams;
use zenith_topo::{Edge, Face, FaceGeometry, Orientation, OrientedEdge, Shell, Vertex, Wire};

fn quad(p0: Point3, p1: Point3, p2: Point3, p3: Point3) -> Face {
    let (v0, v1, v2, v3) = (
        Vertex::from_point(p0),
        Vertex::from_point(p1),
        Vertex::from_point(p2),
        Vertex::from_point(p3),
    );
    let wire = Wire::new(vec![
        OrientedEdge::forward(Edge::line_between(v0.clone(), v1.clone()).unwrap()),
        OrientedEdge::forward(Edge::line_between(v1.clone(), v2.clone()).unwrap()),
        OrientedEdge::forward(Edge::line_between(v2.clone(), v3.clone()).unwrap()),
        OrientedEdge::forward(Edge::line_between(v3.clone(), v0.clone()).unwrap()),
    ]);
    let plane = PlaneSurface3::new(p0, (p1 - p0).normalize(), (p3 - p0).normalize()).unwrap();
    Face::new(
        FaceGeometry::Plane(plane),
        wire,
        vec![],
        Orientation::Forward,
        1e-6,
    )
}

struct Case {
    name: &'static str,
    shell: Shell,
    thickness: f64,
    /// 閉じた式で出した体積。**置けない置き方は `None`**。
    expected: Option<f64>,
    note: &'static str,
}

fn cases() -> Vec<Case> {
    let t = 3.0;
    let flat_a = || {
        quad(
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(20.0, 0.0, 0.0),
            Point3::new(20.0, 30.0, 0.0),
            Point3::new(0.0, 30.0, 0.0),
        )
    };
    vec![
        Case {
            name: "同一平面の2枚（既存）",
            shell: Shell::new(
                vec![
                    flat_a(),
                    quad(
                        Point3::new(20.0, 0.0, 0.0),
                        Point3::new(40.0, 0.0, 0.0),
                        Point3::new(40.0, 30.0, 0.0),
                        Point3::new(20.0, 30.0, 0.0),
                    ),
                ],
                false,
            ),
            thickness: t,
            expected: Some(40.0 * 30.0 * t),
            note: "ここだけが今まで測られていました",
        },
        Case {
            name: "直角に折れた2枚（L）",
            shell: Shell::new(
                vec![
                    flat_a(),
                    quad(
                        Point3::new(20.0, 0.0, 0.0),
                        Point3::new(20.0, 0.0, 25.0),
                        Point3::new(20.0, 30.0, 25.0),
                        Point3::new(20.0, 30.0, 0.0),
                    ),
                ],
                false,
            ),
            thickness: t,
            // **厚みの向きは、面の法線です**（`u × v`）。
            //
            // 1枚目: p0=(0,0,0)、u=+x、v=+y → 法線 **+z**
            //   板 A = x[0,20] × y[0,30] × z[0,3]           = 1800
            // 2枚目: p0=(20,0,0)、u=+z、v=+y → 法線 **−x**
            //   板 B = x[17,20] × y[0,30] × z[0,25]         = 2250
            //
            // **重なります**——x[17,20] × y[0,30] × z[0,3]   = 270
            //
            //   和 = 1800 + 2250 − 270 = **3780**
            //
            // **最初は 4050（= 1800 + 2250）と書きました。**
            // **「触れるだけで重ならない」と思い込んで、法線の向きを
            // 確かめていませんでした。** カーネルは 3780 を返し、
            // **正しいのはカーネルのほう**でした（4-390）。
            expected: Some(1800.0 + 2250.0 - 270.0),
            note: "厚みの向きは面の法線。板は触れるだけでなく、重なります",
        },
        Case {
            name: "離れた2枚",
            shell: Shell::new(
                vec![
                    flat_a(),
                    quad(
                        Point3::new(50.0, 0.0, 0.0),
                        Point3::new(70.0, 0.0, 0.0),
                        Point3::new(70.0, 30.0, 0.0),
                        Point3::new(50.0, 30.0, 0.0),
                    ),
                ],
                false,
            ),
            thickness: t,
            // **断られるのが正しい置き方**です。`thicken_shell` は
            // `boolean_solids_exact`（立体を1つ返す口）を使うので、
            // **2つに分かれる答えは返せません**。**断り文がそう名乗り、
            // 代わりの口（`boolean_solids_exact_result`）まで挙げること**を
            // 見ます。**黙って片方だけ返すほうが困ります。**
            expected: None,
            note: "断るのが正しい。断り文が代わりの口を挙げるかを見ます",
        },
        Case {
            name: "3枚のコの字",
            shell: Shell::new(
                vec![
                    flat_a(),
                    quad(
                        Point3::new(20.0, 0.0, 0.0),
                        Point3::new(20.0, 0.0, 25.0),
                        Point3::new(20.0, 30.0, 25.0),
                        Point3::new(20.0, 30.0, 0.0),
                    ),
                    quad(
                        Point3::new(0.0, 0.0, 25.0),
                        Point3::new(20.0, 0.0, 25.0),
                        Point3::new(20.0, 30.0, 25.0),
                        Point3::new(0.0, 30.0, 25.0),
                    ),
                ],
                false,
            ),
            thickness: t,
            // 板 A = 1800（z[0,3]）、板 B = 2250（x[17,20]、z[0,25]）、
            // 板 C = 1800（z[25,28]）。
            //
            // 重なりは **A∩B = 270** だけです——**B∩C は z=25 で触れるだけ**
            // （厚みが 0）、**A∩C は離れています**。
            //
            //   和 = 1800 + 2250 + 1800 − 270 = **5580**
            expected: Some(1800.0 + 2250.0 + 1800.0 - 270.0),
            note: "重なるのは A と B だけ。B と C は触れるだけ",
        },
    ]
}

fn main() {
    let tol = Tolerance::default();
    let params = TessellationParams::default();

    println!("板を厚くする（thicken_shell）を、閉じた式で測る");
    println!();
    println!(
        "{:<26}{:>6}{:>16}{:>16}{:>12}  {}",
        "置き方", "面", "閉じた式", "測った体積", "相対差", "結果"
    );
    println!("{}", "-".repeat(104));

    let mut wrong = 0usize;
    let mut refused = 0usize;
    let mut notes: Vec<String> = Vec::new();

    for case in cases() {
        let faces = case.shell.faces.len();
        match ThickenBuilder::thicken_shell(&case.shell, case.thickness, &tol) {
            Ok(solid) => {
                let volume = MassCalculator::compute_from_brep(&solid, &params).volume;
                match case.expected {
                    Some(expected) => {
                        let residual = (volume - expected).abs() / expected.abs().max(1.0);
                        if residual > 1e-6 {
                            wrong += 1;
                        }
                        println!(
                            "{:<26}{:>6}{:>16.6}{:>16.6}{:>12.3e}  {}",
                            case.name,
                            faces,
                            expected,
                            volume,
                            residual,
                            if residual <= 1e-6 { "ok" } else { "**ちがう**" }
                        );
                    }
                    None => println!(
                        "{:<26}{:>6}{:>16}{:>16.6}{:>12}  返りました（式なし）",
                        case.name, faces, "-", volume, "-"
                    ),
                }
            }
            Err(reason) => {
                refused += 1;
                // **断るのが正しい置き方で断られたなら、誤答ではありません。**
                if case.expected.is_some() {
                    wrong += 1;
                }
                println!(
                    "{:<26}{:>6}{:>16}{:>16}{:>12}  **断られました**",
                    case.name,
                    faces,
                    case.expected
                        .map(|v| format!("{v:.6}"))
                        .unwrap_or_else(|| "-".to_string()),
                    "-",
                    "-"
                );
                notes.push(format!("  {}: {}", case.name, reason));
            }
        }
    }

    println!("{}", "-".repeat(104));
    if !notes.is_empty() {
        println!();
        println!("断り文:");
        for note in &notes {
            println!("{note}");
        }
    }
    println!();
    for case in cases() {
        println!("  {}: {}", case.name, case.note);
    }
    println!();
    println!("誤答 {wrong} 件、断り {refused} 件");
    if wrong > 0 {
        std::process::exit(1);
    }
}
