//! **読んだ立体の体積は、刻みを細かくすると増えるのか**（4-559）。
//!
//! # なぜ要るか
//!
//! 4-558 で積が返り、**OCC と 1.682e-4 ずれている**と書きました。
//! **ところが、こちらが読んだ `A` そのものの体積も、OCC の `V(A)` から
//! 1.4e-4 ずれています**——**ブーリアンを 1 回もかけない段階で**です。
//!
//! **体積は三角形分割から積んでいます**（`compute_volume_from_brep`）。
//! **曲がった面を平らな三角形で覆えば、体積は必ず小さめに出ます。**
//! **なら、刻みを細かくすれば増えるはず**です。**増えるなら、
//! ずれているのは立体ではなく、測り方**。
//!
//! 使い方: `read_volume_convergence_probe`
use zenith_algo::MassCalculator;
use zenith_io::StepImporter;
use zenith_tess::TessellationParams;

fn main() {
    let sample = "reference/OCCT/data/step/linkrods.step";
    let Ok(solids) = StepImporter::import_solids_from_file(sample) else {
        println!("{sample} が読めません");
        return;
    };
    let Some(read) = solids.into_iter().max_by_key(|s| s.outer_shell.faces.len()) else {
        return;
    };

    // **OCC の数**（4-511）: `V(A)` = 3.847002。
    const OCC: f64 = 3.847002;
    let base = TessellationParams::default();
    println!("OCC の V(A) = {OCC:.6}");
    println!();
    println!("分割     体積          OCC との差     相対");
    for divisions in [8usize, 12, 16, 24, 32, 48, 64, 96] {
        let mut params = base.clone();
        params.u_divisions = divisions;
        params.v_divisions = divisions;
        let volume = MassCalculator::compute_volume_from_brep(&read, &params);
        println!(
            "{divisions:>4}   {volume:.6}   {:+.3e}   {:+.3e}",
            volume - OCC,
            (volume - OCC) / OCC
        );
    }
    println!();
    println!("**増え続けるなら、ずれているのは立体ではなく測り方です。**");
}
