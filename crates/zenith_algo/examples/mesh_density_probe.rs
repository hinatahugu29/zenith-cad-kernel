//! **刻みを振って、表示メッシュが水密かを見る**（4-297）。
//!
//! # なぜ要るのか
//!
//! 掃き出しはこれまで **24 分割だけ**で測っていました。実測（4-296）で、
//! 読んだ `screw.step` は **8・12・24 では水密なのに、16・20・32・48 では
//! 壊れます**。**24 が緑だったのは偶然**でした。
//!
//! **利用者は刻みを自分で選びます。** 1つの刻みだけを見るのは、測り方の穴です。
//!
//! # 何を見るか
//!
//! 稜がちょうど2枚の三角形に共有されていない本数を、**穴（1回）**と
//! **重なり（3回以上）**に分けて数えます。**0 でなければ STL に書けません。**
//!
//! # 赤にするか
//!
//! **自分で作った立体は赤にします。** ここが壊れているなら、それは
//! こちらの欠陥です。**読み込んだファイルは見せるだけ**にします（相手の
//! 粗さに依るので、同じ物差しでは測れません。4-266）。
use zenith_algo::{BooleanEngine, BooleanOpType, BrepTransform, PrimitiveBuilder};
use zenith_math::{Tolerance, Transform3, Vec3};
use zenith_tess::TessellationParams;
use zenith_topo::Solid;

/// 表示メッシュの穴（1回しか使われない稜）と重なり（3回以上）。
fn seams(solid: &Solid, divisions: usize) -> (usize, usize, usize) {
    let mesh = zenith_tess::tessellate_solid(
        solid,
        &TessellationParams {
            u_divisions: divisions,
            v_divisions: divisions,
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
    (
        mesh.indices.len(),
        uses.values().filter(|count| **count == 1).count(),
        uses.values().filter(|count| **count > 2).count(),
    )
}

fn main() {
    let tol = Tolerance::default();
    // **粗いところと細かいところの両方**を見ます。**2 のべき乗だけにしない
    // でください**——16 と 24 で答えが違った（4-296）のは、そこが理由です。
    let densities = [6usize, 8, 10, 12, 16, 20, 24, 32, 48, 64];

    let torus = PrimitiveBuilder::make_torus(12.0, 4.0).expect("torus");
    let cylinder = PrimitiveBuilder::make_cylinder(9.0, 40.0).expect("cylinder");
    let cone = PrimitiveBuilder::make_cone(10.0, 0.0, 20.0).expect("cone");
    let sphere = PrimitiveBuilder::make_sphere(10.0).expect("sphere");
    let boxa = PrimitiveBuilder::make_box(20.0, 20.0, 20.0).expect("box");

    let rod = BrepTransform::translate_solid(&cylinder, Vec3::new(0.0, 0.0, -20.0));
    let tilted = BrepTransform::translate_solid(
        &BrepTransform::transform_solid(
            &torus,
            &Transform3::from_axis_angle(&Vec3::new(1.0, 1.0, 0.0), 25f64.to_radians()),
        )
        .expect("tilt"),
        Vec3::new(10.0, 10.0, 10.0),
    );

    let mut subjects: Vec<(String, Solid)> = vec![
        ("torus".to_string(), torus.clone()),
        ("cylinder".to_string(), cylinder.clone()),
        ("cone".to_string(), cone.clone()),
        ("sphere".to_string(), sphere.clone()),
        ("box".to_string(), boxa.clone()),
    ];
    // **ブーリアンの結果も見ます。** 素形状は継ぎ目が素直ですが、割った面は
    // そうではありません（4-296 で壊れたのは、割られたねじ山の面でした）。
    for (name, a, b, op) in [
        ("torus - rod", &torus, &rod, BooleanOpType::Difference),
        ("torus + rod", &torus, &rod, BooleanOpType::Union),
        ("box x tilted torus", &boxa, &tilted, BooleanOpType::Difference),
        ("cone x sphere", &cone, &sphere, BooleanOpType::Intersection),
    ] {
        if let Ok(result) = BooleanEngine::boolean_solids_exact_result(a, b, op, &tol) {
            for (index, solid) in result.solids.iter().enumerate() {
                subjects.push((format!("{name} [{index}]"), solid.clone()));
            }
        }
    }

    println!("刻みを振って、表示メッシュが水密かを見る（穴＝1回、重なり＝3回以上）");
    println!();
    print!("{:<24}", "subject");
    for divisions in densities {
        print!("{divisions:>8}");
    }
    println!("  verdict");
    println!("{}", "-".repeat(24 + 8 * densities.len() + 10));

    let mut broken = 0usize;
    for (name, solid) in &subjects {
        print!("{name:<24}");
        let mut bad_here = 0usize;
        for divisions in densities {
            let (_, open, over) = seams(solid, divisions);
            let total = open + over;
            if total > 0 {
                bad_here += 1;
            }
            print!("{:>8}", if total == 0 { "-".to_string() } else { total.to_string() });
        }
        if bad_here > 0 {
            broken += 1;
        }
        println!(
            "  {}",
            if bad_here == 0 {
                "ok".to_string()
            } else {
                format!("**{bad_here} つの刻みで壊れます**")
            }
        );
    }
    println!("{}", "-".repeat(24 + 8 * densities.len() + 10));
    println!(
        "{} 件中 {broken} 件が、どこかの刻みで壊れます。",
        subjects.len()
    );
    println!();
    println!("**数字は「ちょうど2枚に共有されていない稜」の本数です。** 0 でなければ");
    println!("STL に書けません。**1つの刻みだけを見るのは測り方の穴です**——");
    println!("読んだ `screw.step` は 8・12・24 で水密なのに 16・20・32・48 で壊れます（4-296）。");

    if broken > 0 {
        std::process::exit(1);
    }
}
