//! **稜の曲線の端は、頂点の位置と一致しているか**を、組を変えて掃きます（4-208）。
//!
//! 4-208 で `cone × torus` の表示メッシュが壊れていた原因です。境界の標本は
//! 稜の曲線から取るので、曲線の端が頂点からずれていると、隣り合う2本の稜が
//! 継ぎ目に「同じはずの点」を2つ作ります。溶接の距離 (1e-7) より大きいと
//! 束ねられず、そこが穴になります。
//!
//! **表示側では両端を頂点へ寄せて塞ぎました。上流の差は残っています。**
//! この口は、その差が `cone × torus` だけの話なのか、交線を作る組に広く
//! あるのかを、推測せずに見るためのものです。
//!
//! 見るのは3つ。
//!
//! - 差の最大（`worst`）
//! - 溶接の距離 (1e-7) を超えている箇所の数（`over weld`）
//! - 稜の端の総数（`ends`）
use zenith_algo::{BooleanEngine, BooleanOpType, BrepTransform, PrimitiveBuilder};
use zenith_math::{Tolerance, Vec3};
use zenith_topo::Solid;

const WELD: f64 = 1e-7;

/// 稜の曲線の端と、その端の頂点との差。
fn end_gaps(solid: &Solid) -> (usize, f64, usize) {
    let (mut ends, mut worst, mut over) = (0usize, 0.0f64, 0usize);
    for shell in std::iter::once(&solid.outer_shell).chain(solid.inner_shells.iter()) {
        for face in &shell.faces {
            for wire in std::iter::once(&face.outer_wire).chain(face.inner_wires.iter()) {
                for oriented in &wire.edges {
                    for (fraction, vertex) in [
                        (0.0, oriented.start_vertex().point),
                        (1.0, oriented.end_vertex().point),
                    ] {
                        let gap = (oriented.evaluate_normalized(fraction) - vertex).norm();
                        ends += 1;
                        worst = worst.max(gap);
                        if gap > WELD {
                            over += 1;
                        }
                    }
                }
            }
        }
    }
    (ends, worst, over)
}

fn main() {
    let tol = Tolerance::default();
    let boxa = PrimitiveBuilder::make_box(20.0, 20.0, 20.0).expect("box");
    let cylinder = PrimitiveBuilder::make_cylinder(6.0, 40.0).expect("cylinder");
    let sphere = PrimitiveBuilder::make_sphere(10.0).expect("sphere");
    let cone = PrimitiveBuilder::make_cone(10.0, 0.0, 20.0).expect("cone");
    let torus = PrimitiveBuilder::make_torus(12.0, 4.0).expect("torus");

    let shifted = |solid: &Solid, x: f64, y: f64, z: f64| {
        BrepTransform::translate_solid(solid, Vec3::new(x, y, z))
    };

    let cases: Vec<(&str, Solid, Solid)> = vec![
        ("box x box (corner)", boxa.clone(), shifted(&boxa, 10.0, 10.0, 10.0)),
        ("box x cylinder", boxa.clone(), shifted(&cylinder, 5.0, 5.0, -10.0)),
        ("box x sphere", boxa.clone(), shifted(&sphere, 10.0, 10.0, 15.0)),
        ("box x cone", boxa.clone(), shifted(&cone, 8.0, 8.0, 5.0)),
        ("box x torus", boxa.clone(), shifted(&torus, 5.0, 5.0, 10.0)),
        ("cylinder x sphere", cylinder.clone(), shifted(&sphere, 3.0, 0.0, 0.0)),
        ("cylinder x cone", cylinder.clone(), shifted(&cone, 3.0, 0.0, -5.0)),
        ("cylinder x torus", cylinder.clone(), shifted(&torus, 0.0, 0.0, 0.0)),
        ("sphere x cone", sphere.clone(), shifted(&cone, 0.0, 0.0, -5.0)),
        ("sphere x torus", sphere.clone(), shifted(&torus, 0.0, 0.0, 0.0)),
        ("cone x torus (rim on the tube)", cone.clone(), shifted(&torus, 10.0, 0.0, 0.0)),
        ("cone x torus (lifted)", cone.clone(), shifted(&torus, 10.0, 0.0, 3.0)),
        ("torus x torus", torus.clone(), shifted(&torus, 12.0, 0.0, 0.0)),
    ];
    let ops = [
        ("union", BooleanOpType::Union),
        ("difference", BooleanOpType::Difference),
        ("intersection", BooleanOpType::Intersection),
    ];

    println!("稜の曲線の端と、その端の頂点との差（表示側の寄せは別。ここは上流の値です）");
    println!();
    println!("{:<34}{:<14}{:>8}{:>14}{:>12}", "case", "op", "ends", "worst", "over weld");
    println!("{}", "-".repeat(84));

    let (mut worst_all, mut worst_where) = (0.0f64, String::from("-"));
    let (mut over_all, mut ends_all) = (0usize, 0usize);
    for (name, a, b) in &cases {
        for (label, op) in ops {
            let Ok(result) = BooleanEngine::boolean_solids_exact_result(a, b, op, &tol) else {
                println!("{:<34}{:<14}{:>8}{:>14}{:>12}", name, label, "-", "断られた", "-");
                continue;
            };
            let (mut ends, mut worst, mut over) = (0usize, 0.0f64, 0usize);
            for solid in &result.solids {
                let (e, w, o) = end_gaps(solid);
                ends += e;
                worst = worst.max(w);
                over += o;
            }
            ends_all += ends;
            over_all += over;
            if worst > worst_all {
                worst_all = worst;
                worst_where = format!("{name} / {label}");
            }
            println!(
                "{:<34}{:<14}{:>8}{:>14.3e}{:>12}",
                name, label, ends, worst, over
            );
        }
    }
    println!("{}", "-".repeat(84));
    println!(
        "稜の端 {ends_all} 箇所のうち、溶接の距離 (1e-7) を超えているもの {over_all} 箇所。最大 {worst_all:.3e}（{worst_where}）"
    );
    println!();
    println!("**0 でないこと自体は誤答ではありません。** B-Rep は多様体のままで、");
    println!("恒等式も破れません。効くのは表示メッシュだけで、そこは 4-208 で");
    println!("両端を頂点へ寄せて塞いであります。ここは**上流が良くなったか**を");
    println!("見るための口です。");
}
