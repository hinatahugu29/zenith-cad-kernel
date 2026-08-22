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

use zenith_algo::{
    BooleanEngine, BooleanOpType, BrepTransform, HoleBuilder, PrimitiveBuilder,
};
use zenith_math::{Tolerance, Vec3};
use zenith_tess::{tessellate_solid, TessellationParams};
use zenith_topo::Solid;

struct Subject {
    name: &'static str,
    solid: Solid,
    /// 同じ形を別の経路で作ったもの。比を取る相手。
    twin: Option<&'static str>,
}

fn triangles(solid: &Solid, divisions: usize) -> usize {
    tessellate_solid(
        solid,
        &TessellationParams {
            u_divisions: divisions,
            v_divisions: divisions,
        },
    )
    .indices
    .len()
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

    for subject in &subjects {
        let row: Vec<usize> = densities
            .iter()
            .map(|density| triangles(&subject.solid, *density))
            .collect();

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
    println!("subjects whose triangle count is not monotone in the division count: {non_monotone}");
    println!("worst ratio between two routes to the same shape: {worst_ratio:.2}x");
    println!();
    println!("A count that falls when the division count rises means the number asked");
    println!("for is not what decides the grid. `segments_for_edge` doubles until the");
    println!("edge's own deflection is under target, so a curved edge can pull the whole");
    println!("patch far past what was requested, and which edge wins changes with the");
    println!("division count.");
    println!();
    println!("This probe measures; nothing here is fixed yet. It is wired to report");
    println!("rather than to fail, so that the numbers stay visible without turning a");
    println!("known weight problem into a red gate.");
}
