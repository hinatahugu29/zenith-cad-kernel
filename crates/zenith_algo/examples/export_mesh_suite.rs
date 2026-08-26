//! STL / OBJ / glTF / DXF を実際に書き出し、外から検算できる台帳を残す。
//!
//! ## なぜ要るのか
//!
//! この4つの出力には、これまで**形に依存する検査が1つもありませんでした**。
//! Rust 側のテストは固定文字列の有無（`"LWPOLYLINE"` が入っているか、
//! `"version": "2.0"` が入っているか）を見るだけで、`tools/verify_mesh_exports.py`
//! に至っては検査関数を定義したまま `main()` からは**1つも呼んでおらず**、
//! 常に「All format validators loaded and verified.」と印字して 0 で終わって
//! いました。読む立体も無く、置き場も決まっていませんでした。
//!
//! ここで書き出すのは、**外部が閉じた式と突き合わせられる形**です。台帳
//! (`manifest.json`) に B-Rep 側の体積・三角形数・境界箱・断面積を書くので、
//! `tools/verify_mesh_exports.py` はファイルを解いた結果だけでそれを検算できます。
//!
//! ```bash
//! cargo run --release -p zenith_algo --example export_mesh_suite
//! py tools/verify_mesh_exports.py
//! ```
//!
//! 書き出しに失敗したら非ゼロで終わります。

use std::fs;
use std::path::Path;

use zenith_algo::{
    BooleanEngine, BooleanOpType, BrepTransform, HoleBuilder, MassCalculator, PrimitiveBuilder,
    SectionSlicer,
};
use zenith_io::{DxfExporter, GltfExporter, ObjExporter, StlExporter};
use zenith_math::{Point3, Tolerance, Vec3};
use zenith_tess::{tessellate_solid, TessellationParams};
use zenith_topo::Solid;

const DIVISIONS: usize = 24;

struct Subject {
    name: &'static str,
    solid: Solid,
    /// 断面を採る高さ（Z）。ここで水平に切る。
    section_z: f64,
}

fn subjects() -> Result<Vec<Subject>, String> {
    let tol = Tolerance::default();
    let mut out = Vec::new();

    out.push(Subject {
        name: "box_20x30x40",
        solid: PrimitiveBuilder::make_box(20.0, 30.0, 40.0)?,
        section_z: 20.0,
    });
    out.push(Subject {
        name: "cylinder_r10_h25",
        solid: PrimitiveBuilder::make_cylinder(10.0, 25.0)?,
        section_z: 12.5,
    });
    out.push(Subject {
        name: "sphere_r12",
        solid: PrimitiveBuilder::make_sphere(12.0)?,
        section_z: 0.0,
    });
    out.push(Subject {
        name: "cone_r10_r4_h20",
        solid: PrimitiveBuilder::make_cone(10.0, 4.0, 20.0)?,
        section_z: 10.0,
    });
    out.push(Subject {
        name: "torus_R12_r4",
        solid: PrimitiveBuilder::make_torus(12.0, 4.0)?,
        section_z: 0.0,
    });
    out.push(Subject {
        name: "drilled_box_30x30x15_r5",
        solid: HoleBuilder::make_drilled_box(30.0, 30.0, 15.0, 5.0)?,
        section_z: 7.5,
    });

    // ブーリアンが割った曲面を含む立体。稜の刻みが両側で揃っているかは、
    // ここがいちばん出やすい。
    let plate = PrimitiveBuilder::make_box(40.0, 40.0, 12.0)?;
    let pin = BrepTransform::translate_solid(
        &PrimitiveBuilder::make_cylinder(6.0, 40.0)?,
        Vec3::new(20.0, 20.0, -10.0),
    );
    let cut = BooleanEngine::boolean_solids_exact(&plate, &pin, BooleanOpType::Difference, &tol)?;
    out.push(Subject {
        name: "boolean_plate_minus_pin",
        solid: cut,
        section_z: 6.0,
    });

    // 断面が**分かれた外形2つ**になる立体。DXF の層分けを索引で決めていると、
    // 2つ目の外形が「穴」の層に落ちる。上から溝を掘った棒を、溝の底より上で
    // 切ると、長方形が2つ出る。
    let bar = PrimitiveBuilder::make_box(60.0, 20.0, 20.0)?;
    let slot = BrepTransform::translate_solid(
        &PrimitiveBuilder::make_box(20.0, 40.0, 20.0)?,
        Vec3::new(20.0, -10.0, 8.0),
    );
    let slotted =
        BooleanEngine::boolean_solids_exact(&bar, &slot, BooleanOpType::Difference, &tol)?;
    out.push(Subject {
        name: "slotted_bar_two_outlines",
        solid: slotted,
        section_z: 14.0,
    });

    Ok(out)
}

fn main() {
    if let Err(error) = run() {
        eprintln!("export_mesh_suite failed: {error}");
        std::process::exit(1);
    }
}

fn run() -> Result<(), String> {
    let tol = Tolerance::default();
    let directory = Path::new("target").join("mesh_exports");
    fs::create_dir_all(&directory).map_err(|e| format!("could not create {directory:?}: {e}"))?;

    let params = TessellationParams {
        u_divisions: DIVISIONS,
        v_divisions: DIVISIONS,
    };
    // 体積は表示用メッシュより細かい刻みで、B-Rep から積む。
    let mass_params = TessellationParams {
        u_divisions: 64,
        v_divisions: 64,
    };

    let mut entries = Vec::new();
    println!(
        "{:<28}{:>9}{:>10}{:>16}{:>16}",
        "subject", "tris", "verts", "brep volume", "section area"
    );
    println!("{}", "-".repeat(79));

    for subject in subjects()? {
        let mesh = tessellate_solid(&subject.solid, &params);
        if mesh.indices.is_empty() {
            return Err(format!("{} tessellated to nothing", subject.name));
        }

        let stl = directory.join(format!("{}.stl", subject.name));
        let obj = directory.join(format!("{}.obj", subject.name));
        let gltf = directory.join(format!("{}.gltf", subject.name));
        let dxf = directory.join(format!("{}.dxf", subject.name));

        StlExporter::export_binary(&mesh, &stl)?;
        ObjExporter::export_to_file(&mesh, &obj, subject.name)
            .map_err(|e| format!("{}: OBJ export failed: {e}", subject.name))?;
        GltfExporter::export_to_file(&mesh, &gltf)?;

        let brep_volume = MassCalculator::compute_from_brep(&subject.solid, &mass_params).volume;

        let section = SectionSlicer::slice_solid(
            &subject.solid,
            Point3::new(0.0, 0.0, subject.section_z),
            Vec3::new(0.0, 0.0, 1.0),
            &tol,
        )?;
        let loops: Vec<Vec<Point3>> = section
            .section_wires
            .iter()
            .map(|wire| {
                wire.edges
                    .iter()
                    .map(|edge| edge.start_vertex().point)
                    .collect()
            })
            .collect();
        DxfExporter::export_loops_to_file(&loops, &dxf)?;

        let bbox = subject.solid.bounding_box();
        let outer_loops = section.signed_loop_areas.iter().filter(|a| **a > 0.0).count();
        let hole_loops = section.signed_loop_areas.iter().filter(|a| **a < 0.0).count();

        println!(
            "{:<28}{:>9}{:>10}{:>16.6}{:>16.6}",
            subject.name,
            mesh.indices.len(),
            mesh.positions.len(),
            brep_volume,
            section.total_area
        );

        entries.push(format!(
            r#"  {{
    "name": "{}",
    "triangles": {},
    "vertices": {},
    "brep_volume": {:.12},
    "low": [{:.9}, {:.9}, {:.9}],
    "high": [{:.9}, {:.9}, {:.9}],
    "section_z": {:.9},
    "section_area": {:.9},
    "section_outer_loops": {},
    "section_hole_loops": {}
  }}"#,
            subject.name,
            mesh.indices.len(),
            mesh.positions.len(),
            brep_volume,
            bbox.min.x,
            bbox.min.y,
            bbox.min.z,
            bbox.max.x,
            bbox.max.y,
            bbox.max.z,
            subject.section_z,
            section.total_area,
            outer_loops,
            hole_loops
        ));
    }

    let manifest = format!("[\n{}\n]\n", entries.join(",\n"));
    let manifest_path = directory.join("manifest.json");
    fs::write(&manifest_path, manifest)
        .map_err(|e| format!("could not write {manifest_path:?}: {e}"))?;

    println!("{}", "-".repeat(79));
    println!("wrote STL / OBJ / glTF / DXF for each subject into {directory:?}");
    println!("check them from outside with:  py tools/verify_mesh_exports.py");
    Ok(())
}
