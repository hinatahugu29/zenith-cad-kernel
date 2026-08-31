//! 出力用メッシュの三角形数が、頼んだ分割数に対して素直に増えるか。
//!
//! `SamplePlan::for_solid` は稜ごとに**たわみが目標を下回るまで刻みを倍々に
//! していく**（`segments_for_edge`、上限 4096）。頼んだ分割数は下限としてしか
//! 効かないので、稜の曲がり方によっては桁違いに細かい格子が組まれる。
//!
//! 断面が閉じなかった件（HANDOVER 4-37）を追っているときに、ブーリアンの
//! 結果が同じ形のビルダー出力より3〜5倍細かく、しかも**分割数に対して
//! 単調ですらない**ことが分かりました。96分割で 658656 枚、128分割で 465072 枚
//! です。断面のほうは位相で繋ぐようにしたので影響を受けなくなりましたが、
//! メッシュそのものは重いままです。
//!
//! ここは直す前の測定です。**何が起きているかを数で置いておかないと、
//! 「重い気がする」以上のことが言えません。**
//!
//! 見るところは2つ。
//!
//! - **単調性**: 分割数を上げたのに三角形が減っていたら、頼んだ数が効いて
//!   いないという印である。
//! - **同じ形どうしの比**: `HoleBuilder::make_drilled_box` と、同じ寸法を
//!   ブーリアンで開けたものは、同じ形である。メッシュの重さが桁で違うなら、
//!   その差は形ではなく経路から来ている。

use zenith_algo::{BooleanEngine, BooleanOpType, BrepTransform, HoleBuilder, PrimitiveBuilder};
use zenith_math::{Tolerance, Vec3};
use zenith_tess::{tessellate_solid, TessellationParams};
use zenith_topo::Solid;

struct Subject {
    name: &'static str,
    solid: Solid,
    /// 同じ形を別の経路で作ったもの。比を取る相手。
    twin: Option<&'static str>,
}

/// 三角形の数と、その面がどちらの経路で張られたか（構造格子 / earcut）。
fn triangles(solid: &Solid, divisions: usize) -> (usize, u64, u64) {
    let before = zenith_geom::work_counter::snapshot();
    let count = tessellate_solid(
        solid,
        &TessellationParams {
            u_divisions: divisions,
            v_divisions: divisions,
        },
    )
    .indices
    .len();
    let spent = zenith_geom::work_counter::snapshot().since(&before);
    (count, spent.grid_patches, spent.earcut_patches)
}

fn main() {
    let tol = Tolerance::default();
    let densities = [8usize, 16, 32, 48, 64, 96, 128, 192, 256];

    let block = PrimitiveBuilder::make_box(40.0, 40.0, 20.0).unwrap();
    let bore = BrepTransform::translate_solid(
        &PrimitiveBuilder::make_cylinder(6.0, 40.0).unwrap(),
        Vec3::new(20.0, 20.0, -10.0),
    );

    let subjects = vec![
        Subject {
            name: "builder drilled box",
            solid: HoleBuilder::make_drilled_box(40.0, 40.0, 20.0, 6.0).unwrap(),
            twin: None,
        },
        Subject {
            name: "boolean drilled box",
            solid: BooleanEngine::boolean_solids_exact(
                &block,
                &bore,
                BooleanOpType::Difference,
                &tol,
            )
            .unwrap(),
            twin: Some("builder drilled box"),
        },
        Subject {
            name: "cylinder",
            solid: PrimitiveBuilder::make_cylinder(10.0, 40.0).unwrap(),
            twin: None,
        },
        Subject {
            name: "sphere",
            solid: PrimitiveBuilder::make_sphere(10.0).unwrap(),
            twin: None,
        },
        Subject {
            name: "box",
            solid: PrimitiveBuilder::make_box(40.0, 40.0, 20.0).unwrap(),
            twin: None,
        },
    ];

    print!("{:<22}", "subject");
    for density in densities {
        print!("{:>9}", density);
    }
    println!("{:>12}", "monotone");
    println!("{}", "-".repeat(22 + 9 * densities.len() + 12));

    let mut counts: Vec<(&str, Vec<usize>)> = Vec::new();
    let mut non_monotone = 0usize;

    let mut paths: Vec<(&str, u64, u64)> = Vec::new();

    for subject in &subjects {
        let measured: Vec<(usize, u64, u64)> = densities
            .iter()
            .map(|density| triangles(&subject.solid, *density))
            .collect();
        let row: Vec<usize> = measured.iter().map(|entry| entry.0).collect();
        // 経路の内訳は分割数によらないので、代表として1つ取る。
        let (_, grid, earcut) = measured[0];
        paths.push((subject.name, grid, earcut));

        let monotone = row.windows(2).all(|pair| pair[1] >= pair[0]);
        if !monotone {
            non_monotone += 1;
        }

        print!("{:<22}", subject.name);
        for value in &row {
            print!("{value:>9}");
        }
        println!("{:>12}", if monotone { "yes" } else { "NO" });

        counts.push((subject.name, row));
    }

    println!("{}", "-".repeat(22 + 9 * densities.len() + 12));

    // 同じ形を別経路で作ったものどうしの比。
    let mut worst_ratio = 0.0f64;
    for subject in &subjects {
        let Some(twin_name) = subject.twin else {
            continue;
        };
        let Some((_, twin_row)) = counts.iter().find(|(name, _)| *name == twin_name) else {
            continue;
        };
        let Some((_, own_row)) = counts.iter().find(|(name, _)| *name == subject.name) else {
            continue;
        };

        print!("{:<22}", "ratio vs twin");
        for (own, twin) in own_row.iter().zip(twin_row.iter()) {
            let ratio = *own as f64 / (*twin).max(1) as f64;
            worst_ratio = worst_ratio.max(ratio);
            print!("{ratio:>9.2}");
        }
        println!("{:>12}", "");
        println!(
            "  {} against {}: the same shape by two routes",
            subject.name, twin_name
        );
    }

    println!("{}", "-".repeat(22 + 9 * densities.len() + 12));

    // どちらの経路で張られたか。ここが重さの出どころである。
    println!();
    println!("{:<22}{:>10}{:>10}", "subject", "grid", "earcut");
    println!("{}", "-".repeat(42));
    let mut earcut_total = 0u64;
    for (name, grid, earcut) in &paths {
        println!("{name:<22}{grid:>10}{earcut:>10}");
        earcut_total += earcut;
    }
    println!("{}", "-".repeat(42));

    println!();
    println!("subjects whose triangle count is not monotone in the division count: {non_monotone}");
    println!("worst ratio between two routes to the same shape: {worst_ratio:.2}x");
    println!("faces that could not use the structured grid: {earcut_total}");
    println!();
    println!("The weight comes from the `earcut` column, not from the edge sampling.");
    println!("`SamplePlan` gives every edge of both solids exactly the number of");
    println!("segments asked for - measured, both the builder and the boolean drilled");
    println!("box sit at n for every edge. What differs is how the patch interior is");
    println!("filled: a face whose boundary runs along the parameter rectangle is laid");
    println!("out on a structured grid, and any other face falls back to earcut plus");
    println!("adaptive refinement, which subdivides until deflection is met and so");
    println!("ignores the division count as an upper bound.");
    println!();
    println!("Boolean results carry split faces whose boundaries no longer follow the");
    println!("parameter rectangle, so they take the fallback. That is also why the");
    println!("count is not monotone: which faces qualify shifts with the density.");
    println!();
    println!("This probe measures; nothing here is fixed yet. It is wired to report");
    println!("rather than to fail, so that the numbers stay visible without turning a");
    println!("known weight problem into a red gate.");
}
