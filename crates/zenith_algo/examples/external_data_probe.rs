//! **門を回すのに要る外部ファイルが、揃っているか**（4-411）。
//!
//! # なぜ要るのか
//!
//! 2026/09/08 に、`reference/`（`.gitignore` 済み、約 420MB）の中身を
//! **消してしまいました**。worktree に張ったジャンクションごと
//! `git worktree remove --force` したためです（5 章の落とし穴）。
//!
//! **通しテストは 139 本 / 723 件通過・0 失敗のままでした。**
//! 読めないファイルは飛ばす作りだからです。**赤になりません。**
//!
//! **それがいちばん危ないところ**でした——**「出力が無い」を「緑」と
//! 読み違えられます。** 4-410 が「門はテストではありません」と書いた
//! のと、同じ形の落とし穴です。
//!
//! # 何を見るか
//!
//! **コードが実際に読んでいる外部ファイル**を並べ、**1 つでも無ければ
//! 非ゼロで終わります**。
//!
//! **「読めないファイルは赤にしない」（4-266）とは、別の話**です。
//! あちらは**読めたが中身が粗い**とき。ここは**そもそも無い**ときで、
//! **門が回っていない**ことを意味します。
//!
//! # 揃えかた
//!
//! `linkrods.step` と `screw.step` は、**OCCT が公開しているサンプル
//! データ**（`data/step/`）です。`reference/OCCT/data/step/` に置きます。
//!
//! **追跡済みの検体**（`crates/zenith_algo/tests/fixtures/`、29 個）は
//! git に入っているので、こちらは普通は揃っています。

use std::path::{Path, PathBuf};

struct Need {
    path: PathBuf,
    tracked: bool,
    used_by: &'static str,
}

fn needs() -> Vec<Need> {
    let root = PathBuf::from(concat!(env!("CARGO_MANIFEST_DIR"), "/../.."));
    let fixtures = PathBuf::from(concat!(env!("CARGO_MANIFEST_DIR"), "/tests/fixtures"));
    let mut list = vec![
        Need {
            path: root.join("reference/OCCT/data/step/linkrods.step"),
            tracked: false,
            used_by: "read_and_cut / closure / step_face_gap / step_surface_shape / unused_builder / seam_census",
        },
        Need {
            path: root.join("reference/OCCT/data/step/screw.step"),
            tracked: false,
            used_by: "read_and_cut / closure / step_surface_shape",
        },
    ];
    for name in [
        "occ_reference_cone_full.step",
        "occ_reference_sphere.step",
        "occ_reference_cylinder.step",
        "occ_reference_torus.step",
        "occ_reference_pipe_bend.step",
    ] {
        list.push(Need {
            path: fixtures.join(name),
            tracked: true,
            used_by: "foreign_slice / foreign_distance / foreign_inertia / shape_variety",
        });
    }
    list
}

fn shorten(path: &Path) -> String {
    let text = path.to_string_lossy().replace('\\', "/");
    let text = text.replace("crates/zenith_algo/../../", "");
    match text.find("/CAD-Kernel/") {
        Some(index) => text[index + "/CAD-Kernel/".len()..].to_string(),
        None => text,
    }
}

fn main() {
    println!("門を回すのに要る外部ファイルが、揃っているか（4-411）");
    println!();
    println!("**「無い」は「緑」ではありません。** 読めないファイルを飛ばす作りなので、");
    println!("**通しテストは無くても緑のまま**です。ここだけが、それを赤にします。");
    println!();

    let mut missing: Vec<String> = Vec::new();
    let mut present = 0usize;
    for need in needs() {
        let exists = need.path.exists();
        if exists {
            present += 1;
        } else {
            missing.push(format!(
                "  {}  （{}が使います{}）",
                shorten(&need.path),
                need.used_by,
                if need.tracked {
                    "。**git で追跡済みのはずです——`git status` を見てください**"
                } else {
                    "。**`.gitignore` 済みなので、git からは戻せません**"
                }
            ));
        }
        println!(
            "{}  {}",
            if exists { "ある    " } else { "**無い**" },
            shorten(&need.path)
        );
    }

    println!();
    println!("{} 個あり、{} 個ありません。", present, missing.len());
    if missing.is_empty() {
        println!();
        println!("**揃っています。** 門を回してください。");
        return;
    }

    println!();
    println!("無いもの:");
    for line in &missing {
        println!("{line}");
    }
    println!();
    println!("**`linkrods.step` と `screw.step` は、OCCT が公開している**");
    println!("**サンプルデータ**（`data/step/`）です。");
    println!("`reference/OCCT/data/step/` に置けば、門は全部回せます。");
    println!();
    println!("**この一式が赤のあいだ、門が緑だとは書けません**——");
    println!("**回っていないだけ**です（4-411 の落とし穴）。");
    std::process::exit(1);
}
