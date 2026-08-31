//! 構造格子から落ちた面を数える。
//!
//! テッセレーションには2つの経路があります。境界がパラメータ矩形の縁に
//! なっている曲面パッチは**構造格子**で張り、そうでないものは earcut ＋
//! 適応細分に落ちます。落ちても**メッシュは閉じるし、体積も正しい**ので、
//! 既存のゲートはすべて緑のままです。**落ちたことは外から見えません。**
//!
//! 見えないまま何が起きるかは実測してあります。読んだ円錐は境界が3辺
//! （頂点が退化して稜が無い）だったために毎回落ちていて、
//!
//! - 三角形が自前の円錐の 5〜10 倍（64分割で 141,915 対 32,766）
//! - 退化三角形が 54 枚
//! - 粗密が偏るので**断面の輪郭が壊れ**、周長が解析解 31.4 に対して **447**、
//!   しかも刻みを細かくするほど発散
//!
//! という状態でした（4-70）。**だから数えるゲートが要ります。**
//!
//! ## この表の読み方
//!
//! `earcut` が 0 でないこと自体は欠陥ではありません。ブーリアンで割られた面や、
//! 6辺で切り取られた面は、そもそも格子を張れる形ではないからです。ここは
//! **検体ごとに「いくつまでは分かっている」を持ち**、それを超えたら赤にします。
//! 分かっている落ち方には理由を書いてあります。直したら許容を下げてください。
//!
//! ```bash
//! cargo run --release -p zenith_algo --example grid_fallback_probe
//! ```

use std::path::PathBuf;

use zenith_geom::work_counter;
use zenith_io::StepImporter;
use zenith_tess::{tessellate_solid, TessellationParams};
use zenith_topo::Solid;

struct Subject {
    name: &'static str,
    /// この検体で earcut に落ちてよいパッチの数。
    allowed: usize,
    /// なぜ落ちるのか。空文字なら「落ちない見込み」。
    reason: &'static str,
}

fn subjects() -> Vec<Subject> {
    vec![
        Subject {
            name: "cone",
            allowed: 0,
            reason: "",
        },
        Subject {
            name: "cone_full",
            allowed: 0,
            reason: "",
        },
        Subject {
            name: "cylinder",
            allowed: 0,
            reason: "",
        },
        Subject {
            name: "cylinder_nurbs",
            allowed: 2,
            reason: "蓋が円形トリムを持つ NURBS 面。パラメータ矩形の縁が境界ではない",
        },
        Subject {
            name: "elliptic_prism",
            allowed: 0,
            reason: "",
        },
        Subject {
            name: "extruded_spline",
            allowed: 0,
            reason: "",
        },
        Subject {
            name: "revolved_ring",
            allowed: 0,
            reason: "",
        },
        Subject {
            name: "sphere",
            allowed: 0,
            reason: "",
        },
        Subject {
            name: "sphere_capped",
            allowed: 1,
            reason: "境界が6辺。箱で切られた球はパラメータ矩形ではない",
        },
        Subject {
            name: "torus",
            allowed: 0,
            reason: "",
        },
        Subject {
            name: "torus_segment",
            allowed: 0,
            reason: "",
        },
        Subject {
            name: "filleted_box",
            allowed: 0,
            reason: "",
        },
        Subject {
            name: "chamfered_box",
            allowed: 0,
            reason: "",
        },
        Subject {
            name: "hollow_box",
            allowed: 0,
            reason: "",
        },
        Subject {
            name: "stepped_shaft",
            allowed: 0,
            reason: "",
        },
    ]
}

fn read(name: &str) -> Option<Solid> {
    let path = PathBuf::from(concat!(env!("CARGO_MANIFEST_DIR"), "/tests/fixtures"))
        .join(format!("occ_reference_{name}.step"));
    StepImporter::import_solids_from_file(&path)
        .ok()?
        .into_iter()
        .next()
}

struct Measured {
    triangles: usize,
    grid: usize,
    earcut: usize,
    degenerate: usize,
}

fn measure(solid: &Solid, n: usize) -> Measured {
    work_counter::reset();
    let mesh = tessellate_solid(
        solid,
        &TessellationParams {
            u_divisions: n,
            v_divisions: n,
        },
    );
    let snapshot = work_counter::snapshot();
    let mut degenerate = 0usize;
    for triangle in &mesh.indices {
        let p = [
            mesh.positions[triangle[0] as usize],
            mesh.positions[triangle[1] as usize],
            mesh.positions[triangle[2] as usize],
        ];
        if (p[1] - p[0]).cross(&(p[2] - p[0])).norm() * 0.5 < 1e-12 {
            degenerate += 1;
        }
    }
    Measured {
        triangles: mesh.indices.len(),
        grid: snapshot.grid_patches as usize,
        earcut: snapshot.earcut_patches as usize,
        degenerate,
    }
}

fn main() {
    let mut failures = 0usize;
    let mut read_count = 0usize;

    println!("構造格子から落ちた面を数える（他カーネルが書いた検体、32分割）");
    println!();
    println!(
        "{:<18} {:>6} {:>10} {:>6} {:>7} {:>7} {:>8}  {}",
        "fixture", "faces", "triangles", "grid", "earcut", "allow", "degen", "verdict"
    );
    println!("{}", "-".repeat(104));

    for subject in subjects() {
        let Some(solid) = read(subject.name) else {
            // 検体が無いのは、このプローブの落ち度ではありません。
            // 名前が変わったのなら気づけるように、行だけ出します。
            println!(
                "{:<18} 読めませんでした（検体が無いか、読み取りが落ちた）",
                subject.name
            );
            failures += 1;
            continue;
        };
        read_count += 1;

        let measured = measure(&solid, 32);
        let within = measured.earcut <= subject.allowed;
        // **退化三角形は、どの経路でも 0 でなければなりません。**
        let clean = measured.degenerate == 0;
        if !within || !clean {
            failures += 1;
        }

        println!(
            "{:<18} {:>6} {:>10} {:>6} {:>7} {:>7} {:>8}  {}",
            subject.name,
            solid.outer_shell.faces.len(),
            measured.triangles,
            measured.grid,
            measured.earcut,
            subject.allowed,
            measured.degenerate,
            if !clean {
                "DEGENERATE".to_string()
            } else if !within {
                format!("MORE THAN ALLOWED (+{})", measured.earcut - subject.allowed)
            } else if measured.earcut > 0 {
                format!("ok — {}", subject.reason)
            } else {
                "ok".to_string()
            }
        );
    }

    println!("{}", "-".repeat(104));
    println!("{read_count} fixture(s) measured, {failures} over the allowance");
    println!();
    println!("earcut に落ちること自体は欠陥ではありません。落ちてよい数は検体ごとに");
    println!("持っていて、理由も書いてあります。**増えたら赤**、直したら許容を下げます。");

    if failures > 0 {
        std::process::exit(1);
    }
}
