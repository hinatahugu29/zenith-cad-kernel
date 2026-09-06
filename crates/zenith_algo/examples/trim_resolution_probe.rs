//! **内外判定は、どこまで細かく効くか**（4-364）。
//!
//! # なぜ要るのか
//!
//! 4-342 は「多角形の粗さが、そのまま刻む場所の誤差になる。1 区間 24 点では
//! 1.375e-4 残った」と書いています。**その後 128 点に増やしました。**
//! **増やしたあとの数字は、まだ измер っていません。**
//!
//! 4-361 は「`point_inside_face_trim` が切る用には粗すぎる」と書いて、
//! **直す順番の 1 番**に置きました。**本当に粗いのかを、先に測ります。**
//!
//! # 何を見るか
//!
//! **円い境界を持つ面**（円柱の天面）で、**縁からいろいろな距離**に点を置き、
//! 内外の答えが**どこで正しくなくなるか**を見ます。
//!
//! **答えが分かっている点だけを使います**——半径 r の円なら、中心から
//! `r - δ` は中、`r + δ` は外です。
use zenith_algo::{BooleanEngine, BooleanOpType, PrimitiveBuilder};
use zenith_math::{Point3, Tolerance};
use zenith_topo::{Face, FaceGeometry, Solid};

const RADIUS: f64 = 10.0;

fn top_face(solid: &Solid) -> Option<&Face> {
    solid
        .outer_shell
        .faces
        .iter()
        .filter(|face| matches!(face.geometry, FaceGeometry::Plane(_)))
        .max_by(|left, right| {
            let height = |face: &Face| {
                face.outer_wire
                    .edges
                    .iter()
                    .map(|oriented| oriented.edge.start_vertex.point.z)
                    .fold(f64::NEG_INFINITY, f64::max)
            };
            height(left).total_cmp(&height(right))
        })
}

fn main() {
    let tol = Tolerance::default();
    let cylinder = PrimitiveBuilder::make_cylinder(RADIUS, 20.0).expect("円柱");
    let face = top_face(&cylinder).expect("天面");
    let z = face.outer_wire.edges[0].edge.start_vertex.point.z;

    println!("**内外判定は、どこまで細かく効くか**（4-364）");
    println!();
    println!("円柱の天面（半径 {RADIUS}）の縁から、距離を変えて点を置きます。");
    println!("**中は「中」、外は「外」と答えるのが正解**です。");
    println!();
    println!(
        "{:>12}  {:>10}  {:>10}  {}",
        "縁からの距離", "内側の点", "外側の点", ""
    );
    println!("{}", "-".repeat(56));

    // **角度も振ります**——多角形の頂点の上と、辺の真ん中では違うはずです。
    let angles: Vec<f64> = (0..37)
        .map(|step| step as f64 * std::f64::consts::TAU / 37.0)
        .collect();
    let mut worst_wrong = 0.0f64;

    for exponent in 1..=9 {
        let delta = 10f64.powi(-exponent);
        let mut inside_wrong = 0usize;
        let mut outside_wrong = 0usize;
        for angle in &angles {
            let (sin, cos) = angle.sin_cos();
            let inner = Point3::new((RADIUS - delta) * cos, (RADIUS - delta) * sin, z);
            let outer = Point3::new((RADIUS + delta) * cos, (RADIUS + delta) * sin, z);
            if zenith_algo::point_inside_face_trim_for_probe(face, inner, &tol) != Some(true) {
                inside_wrong += 1;
            }
            if zenith_algo::point_inside_face_trim_for_probe(face, outer, &tol) != Some(false) {
                outside_wrong += 1;
            }
        }
        if inside_wrong + outside_wrong > 0 {
            worst_wrong = worst_wrong.max(delta);
        }
        println!(
            "{:>12}  {:>8} 本  {:>8} 本  {}",
            format!("{delta:.0e}"),
            inside_wrong,
            outside_wrong,
            if inside_wrong + outside_wrong == 0 {
                "ok"
            } else {
                "← **間違えます**"
            }
        );
    }

    println!("{}", "-".repeat(56));
    if worst_wrong > 0.0 {
        println!("**縁から {worst_wrong:.0e} までは間違えます。**");
    } else {
        println!("**1e-9 まで、37 方向すべてで正しく答えます。**");
    }
    println!();
    println!("**これは診断です。赤にはしません。**");

    let _ = BooleanEngine::boolean_solids_exact(&cylinder, &cylinder, BooleanOpType::Union, &tol);
}
