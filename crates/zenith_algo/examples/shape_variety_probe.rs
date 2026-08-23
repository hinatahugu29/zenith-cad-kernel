//! 実務で普通に出てくる形を読んで、体積と面数を OpenCASCADE と突き合わせる。
//!
//! これまでの検体は解析曲面（円柱・円錐・球・トーラス）と掃引が少しで、
//! **フィレット・面取り・複数の穴・ロフト・スロット・中空**が1つも
//! ありませんでした。部品ファイルを開けばまず出てくる形です。
//!
//! 検体は OpenCASCADE 自身に書かせています（`tools/occ_reference_shapes.py`）。
//! こちらで組み立てたものは、こちらの思い込みをそのまま検査してしまいます。
//!
//! 期待値は OCC が**書いたファイルを読み直して**測った値です。渡した形では
//! なく、ファイルに入った形が突き合わせの対象になります。
//!
//! ```bash
//! cargo run --release -p zenith_algo --example shape_variety_probe
//! ```

use std::path::PathBuf;

use zenith_algo::MassCalculator;
use zenith_math::Tolerance;
use zenith_tess::{tessellate_solid, TessellationParams};
use zenith_topo::Solid;

/// 求積の刻み。他のプローブと同じにします。
fn params() -> TessellationParams {
    TessellationParams {
        u_divisions: 64,
        v_divisions: 64,
    }
}

struct Subject {
    name: &'static str,
    /// OpenCASCADE が書いたファイルを読み直して測った体積。
    volume: f64,
    /// 同じく面数。
    faces: usize,
    /// この検体で初めて測るもの。
    brings: &'static str,
}

const SUBJECTS: [Subject; 9] = [
    Subject {
        name: "filleted_box",
        volume: 6757.168026,
        faces: 26,
        brings: "edge blends: cylinders and spheres meeting three at a corner",
    },
    Subject {
        name: "chamfered_box",
        volume: 6508.333333,
        faces: 26,
        brings: "six planes at one vertex, with no curved face involved",
    },
    Subject {
        name: "plate_with_holes",
        volume: 5170.951377,
        faces: 9,
        brings: "one face carrying three inner wires",
    },
    Subject {
        name: "slotted_block",
        volume: 5760.000000,
        faces: 10,
        brings: "an outer wire that is concave",
    },
    Subject {
        name: "lofted_solid",
        volume: 2871.542562,
        faces: 7,
        brings: "a B-spline surface curved in both u and v",
    },
    Subject {
        name: "pipe_bend",
        volume: 1579.136704,
        faces: 3,
        brings: "a section swept along a curved spine",
    },
    Subject {
        name: "revolved_vase",
        // **OCC の立体求積ではなく、母線から直接積分した値。** OCC は
        // この形で 4170.999302 と言い、1.3e-5 外れています
        // （`tools/revolved_volume_reference.py`）。
        volume: 4171.053368,
        faces: 3,
        brings: "a spline profile turned about an axis: SURFACE_OF_REVOLUTION",
    },
    Subject {
        name: "hollow_box",
        volume: 4240.000000,
        faces: 12,
        brings: "a solid with an inner shell",
    },
    Subject {
        name: "stepped_shaft",
        volume: 6130.818063,
        faces: 7,
        brings: "annular planes where the radius changes on one axis",
    },
];

fn fixture(name: &str) -> PathBuf {
    PathBuf::from(concat!(env!("CARGO_MANIFEST_DIR"), "/tests/fixtures"))
        .join(format!("occ_reference_{name}.step"))
}

fn face_count(solid: &Solid) -> usize {
    solid.outer_shell.faces.len()
        + solid
            .inner_shells
            .iter()
            .map(|shell| shell.faces.len())
            .sum::<usize>()
}

fn main() {
    let tol = Tolerance::default();
    println!(
        "{:<20} {:>7} {:>7} {:>15} {:>15} {:>11}  {}",
        "subject", "faces", "want", "volume", "want", "relative", "verdict"
    );
    println!("{}", "-".repeat(120));

    let mut read = 0usize;
    let mut matched = 0usize;
    let mut wrong = 0usize;

    for subject in &SUBJECTS {
        let solids = match zenith_io::StepImporter::import_solids_from_file(&fixture(subject.name))
        {
            Ok(solids) if !solids.is_empty() => solids,
            Ok(_) => {
                println!("{:<20} no solids in the file", subject.name);
                continue;
            }
            Err(err) => {
                println!(
                    "{:<20} unreadable: {}",
                    subject.name,
                    err.chars().take(70).collect::<String>()
                );
                continue;
            }
        };
        read += 1;

        let faces: usize = solids.iter().map(face_count).sum();
        // **メッシュが返ってこない形は、体積も測れません。** 先に呼んで
        // おけば、返らないものはここで止まります。
        let triangles: usize = solids
            .iter()
            .map(|solid| tessellate_solid(solid, &params()).indices.len())
            .sum();
        let volume: f64 = solids
            .iter()
            .map(|solid| MassCalculator::compute_from_brep(solid, &params()).volume)
            .sum();

        let relative = (volume - subject.volume).abs() / subject.volume;
        // 曲面をメッシュから測るので、桁いっぱいでは合いません。1e-3 は
        // 「形が違う」と「刻みが粗い」を分ける線です。
        let verdict = if relative <= 1e-3 {
            matched += 1;
            "ok"
        } else {
            wrong += 1;
            "WRONG"
        };

        println!(
            "{:<20} {faces:>7} {:>7} {volume:>15.6} {:>15.6} {relative:>11.2e}  {verdict:<5} {} tri",
            subject.name, subject.faces, subject.volume, triangles
        );
        if faces != subject.faces {
            println!(
                "{:<20}   face count differs: OpenCASCADE says {}",
                "", subject.faces
            );
        }
        let closed = solids
            .iter()
            .all(|solid| solid.outer_shell.validate_closed(&tol).is_valid());
        if !closed {
            println!("{:<20}   the outer shell does not close", "");
        }
    }

    println!("{}", "-".repeat(120));
    println!(
        "read {read} of {}   volume ok {matched}   WRONG {wrong}",
        SUBJECTS.len()
    );
    println!();
    for subject in &SUBJECTS {
        println!("{:<20} {}", subject.name, subject.brings);
    }
}
