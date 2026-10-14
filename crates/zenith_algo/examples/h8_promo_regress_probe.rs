//! **4-754 で見つかった退行の切り分け**——薄い輪を切ったトーラスの
//! `A^B`（深さ 1e-4）が、19 本のどれかで断られるようになった。
use zenith_algo::{BooleanEngine, BooleanOpType, BrepTransform, PrimitiveBuilder};
use zenith_math::{Tolerance, Vec3};
use zenith_topo::Solid;

const R_MAJOR: f64 = 6.0;
const R_MINOR: f64 = 2.0;
const BOX_SIDE: f64 = 40.0;

const PORTS: &[(&str, &str)] = &[
    ("ZENITH_SSI_EMPTY_RETRY", "1"),
    ("ZENITH_SSI_EXTRA_BRANCHES", "1"),
    ("ZENITH_LEFTOVER_LOOPS", "1"),
    ("ZENITH_HOLE_CUT", "1"),
    ("ZENITH_SPLIT_ONE_PIECE", "1"),
    ("ZENITH_DEDUP_SELECTED", "1"),
    ("ZENITH_ID_WELD", "1"),
    ("ZENITH_HOLE_BOX_GUARD", "1"),
    ("ZENITH_FACE_SNAP", "1"),
    ("ZENITH_CHAIN_AREA_TOL", "1"),
    ("ZENITH_HOLECUT_SIGN", "1"),
    ("ZENITH_WRAPPED_AREA", "1"),
    ("ZENITH_KEEP_MIXED_HOLES", "1"),
    ("ZENITH_EDGE_TOL_FROM_FACES", "1"),
    ("ZENITH_EDGE_TOL_BOUNDARY", "1"),
    ("ZENITH_IMPORT_ROUGHNESS_SAMPLES", "64"),
    ("ZENITH_VERTEX_IMPRINT_TOL", "1"),
    ("ZENITH_CAP_ORIENT_BY_GEOMETRY", "1"),
    ("ZENITH_ID_WELD_PLANES", "1"),
];

/// **全部「0」にします**（4-754 で既定オンになったので、
/// `remove_var` では「立てない」ことになりません）。
fn clear_all() {
    for (name, _) in PORTS {
        std::env::set_var(name, "0");
    }
}

fn try_it(a: &Solid, b: &Solid, tol: &Tolerance) -> Result<(), String> {
    BooleanEngine::boolean_solids_exact_result(a, b, BooleanOpType::Intersection, tol)
        .map(|_| ())
        .map_err(|error| error.split(';').next().unwrap_or(&error).to_string())
}

fn main() {
    let tol = Tolerance::default();
    let depth = 1e-4_f64;
    let z0 = R_MINOR - depth;
    let a = PrimitiveBuilder::make_torus(R_MAJOR, R_MINOR).expect("トーラス");
    let b = BrepTransform::translate_solid(
        &PrimitiveBuilder::make_box(BOX_SIDE, BOX_SIDE, BOX_SIDE).expect("箱"),
        Vec3::new(-BOX_SIDE * 0.5, -BOX_SIDE * 0.5, z0),
    );

    clear_all();
    println!("口なし: {:?}", try_it(&a, &b, &tol));

    for (name, value) in PORTS {
        clear_all();
        std::env::set_var(name, value);
        let result = try_it(&a, &b, &tol);
        let flag = if result.is_err() { "  <-- 断られる" } else { "" };
        println!("{name:<34} {result:?}{flag}");
        clear_all();
    }

    for (name, value) in PORTS {
        std::env::set_var(name, value);
    }
    println!();
    println!("全 19 本: {:?}", try_it(&a, &b, &tol));
    clear_all();

    // **単独では再現しなかったので、二分探索で組を絞ります。**
    println!();
    println!("=== 二分探索（前半 vs 後半） ===");
    let mut suspects: Vec<usize> = (0..PORTS.len()).collect();
    loop {
        if suspects.len() <= 1 {
            println!("残り: {:?}", suspects.iter().map(|&i| PORTS[i].0).collect::<Vec<_>>());
            break;
        }
        let mid = suspects.len() / 2;
        let (first, second) = (&suspects[..mid], &suspects[mid..]);

        clear_all();
        for &i in first {
            std::env::set_var(PORTS[i].0, PORTS[i].1);
        }
        let first_fails = try_it(&a, &b, &tol).is_err();
        clear_all();

        clear_all();
        for &i in second {
            std::env::set_var(PORTS[i].0, PORTS[i].1);
        }
        let second_fails = try_it(&a, &b, &tol).is_err();
        clear_all();

        println!(
            "前半 {:?} -> {}　後半 {:?} -> {}",
            first.iter().map(|&i| PORTS[i].0).collect::<Vec<_>>(),
            if first_fails { "断られる" } else { "ok" },
            second.iter().map(|&i| PORTS[i].0).collect::<Vec<_>>(),
            if second_fails { "断られる" } else { "ok" },
        );

        if first_fails {
            suspects = first.to_vec();
        } else if second_fails {
            suspects = second.to_vec();
        } else {
            println!("どちらの半分だけでも再現しません——2 本以上の組み合わせを総当たりします。");
            break;
        }
    }

    println!();
    println!("=== 2 本の組み合わせを総当たり（{} 通り） ===", PORTS.len() * (PORTS.len() - 1) / 2);
    let mut found = false;
    for i in 0..PORTS.len() {
        for j in (i + 1)..PORTS.len() {
            clear_all();
            std::env::set_var(PORTS[i].0, PORTS[i].1);
            std::env::set_var(PORTS[j].0, PORTS[j].1);
            let fails = try_it(&a, &b, &tol).is_err();
            clear_all();
            if fails {
                println!("{} + {} -> **断られる**", PORTS[i].0, PORTS[j].0);
                found = true;
            }
        }
    }
    if !found {
        println!("2 本の組では再現しませんでした——3 本以上の組み合わせです。");
    }
}
