//! **継ぎ目に乗った区間を、次数と制御点と「途中」で数える**（4-411）。
//!
//! # なぜ要るのか
//!
//! 4-410 の②は、**読み筋だけ置いて止めてあります**。`efa9c92` が
//! `settle_seam_segment_axis` へ足した `mixed`——**どの制御点も min か max の
//! どちらかに乗っている**——が、**円錐の底円まで曖昧と数えてしまい**、
//! 4 つの門が赤になりました。
//!
//! そこにこう書きました——
//!
//! > **制御点の位置だけでは、この 2 つを見分けられません**。見分けるには、
//! > 区間が途中で内側を通るか、それとも継ぎ目に乗ったままかを見る必要が
//! > あります。**ただし `linkrods` の当該区間の次数と制御点の並びを、まだ
//! > 数えていません**——**そこを数えるところから始めてください。**
//!
//! **これが、その「数えるところ」です。**
//!
//! # 何を見るか
//!
//! `ZENITH_SEAM_CENSUS=1` を立てて読み込み、判定の中が出す行を集めます。
//! 各区間について**次数・制御点の数・軸の値・途中の標本 3 点**が出ます。
//!
//! **知りたいのは 1 つだけ**です——
//!
//! > **`mixed` が立った区間のうち、途中が端に張り付いていないものは
//! > いくつあるか。**
//!
//! そこが 0 でなければ、**途中を見れば 2 つを見分けられます**。
//! そこが 0 なら、**途中を見ても見分けられません**——別の手が要ります。
//!
//! # 読み方
//!
//! **これは診断です。赤にはしません。** 数を出すだけです。
//! **既定の答えは 1 つも変えません。**

use std::path::PathBuf;
use zenith_algo::Regularizer;
use zenith_io::StepImporter;
use zenith_math::Tolerance;
use zenith_topo::{FaceGeometry, Solid};

fn occt_sample(name: &str) -> PathBuf {
    PathBuf::from(concat!(
        env!("CARGO_MANIFEST_DIR"),
        "/../../reference/OCCT/data/step"
    ))
    .join(name)
}

fn fixture(name: &str) -> PathBuf {
    PathBuf::from(concat!(env!("CARGO_MANIFEST_DIR"), "/tests/fixtures")).join(name)
}

fn read(path: PathBuf, tol: &Tolerance) -> Vec<Solid> {
    match StepImporter::import_solids_from_file(&path) {
        Ok(solids) => solids
            .into_iter()
            .map(|solid| Regularizer::hold_like_our_own(&solid, tol))
            .collect(),
        Err(_) => Vec::new(),
    }
}

fn census(label: &str, path: PathBuf, tol: &Tolerance) {
    println!("== {label} ==");
    let solids = read(path, tol);
    if solids.is_empty() {
        println!("  読めませんでした");
        println!();
        return;
    }
    let mut nurbs_faces = 0usize;
    let mut built = 0usize;
    for solid in &solids {
        let mut faces = solid.outer_shell.faces.clone();
        for inner in &solid.inner_shells {
            faces.extend(inner.faces.clone());
        }
        for face in &faces {
            if !matches!(face.geometry, FaceGeometry::Nurbs(_)) {
                continue;
            }
            nurbs_faces += 1;
            // **判定の中の行は、ここで出ます**（`ZENITH_SEAM_CENSUS=1`）。
            if face.derive_nurbs_boundary_pcurves(tol, 8).is_ok() {
                built += 1;
            }
        }
    }
    println!("  立体 {} 個、nurbs の面 {nurbs_faces} 枚、p-curve が組めた面 {built} 枚", solids.len());
    println!();
}

fn main() {
    if std::env::var_os("ZENITH_SEAM_CENSUS").is_none() {
        println!("**`ZENITH_SEAM_CENSUS=1` を立ててください。**");
        println!("判定の中の行は、その口からしか出ません。");
        return;
    }
    let tol = Tolerance::default();

    println!("継ぎ目に乗った区間を、次数と制御点と「途中」で数える（4-411）");
    println!();
    println!("各行の見方: at_min / at_max は「制御点が全部その端」、");
    println!("mixed は `efa9c92` が足した「どの制御点も min か max のどちらか」。");
    println!("**途中=[...] が端に張り付いていなければ、途中を見て見分けられます。**");
    println!();

    // **追跡済みの検体を先に置きます**（4-411）。`reference/` は
    // `.gitignore` 済みで、**2026/09/08 に消えました**——**無くても
    // この口が回るように**、周期的な曲面を持つ検体を並べてあります。
    for (label, name) in [
        ("円錐（OCC が書いたもの）", "occ_reference_cone_full.step"),
        ("球", "occ_reference_sphere.step"),
        ("円柱", "occ_reference_cylinder.step"),
        ("トーラス", "occ_reference_torus.step"),
        ("曲がり管", "occ_reference_pipe_bend.step"),
    ] {
        census(label, fixture(name), &tol);
    }

    // **`linkrods.step` は `reference/` にあります。** 無ければ、
    // **無いと言って飛ばします**——**黙って通さない**ために
    // `external_data_probe` が別に赤にします。
    let linkrods = occt_sample("linkrods.step");
    if linkrods.exists() {
        census("linkrods.step", linkrods, &tol);
    } else {
        println!("== linkrods.step ==");
        println!("  **ありません**（`reference/OCCT/data/step/`）。");
        println!("  **飛ばしました**——`external_data_probe` を回してください。");
        println!();
    }
}
