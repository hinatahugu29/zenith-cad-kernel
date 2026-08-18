use std::fs;
use std::path::Path;
use zenith_algo::{
    ChamferBuilder, DirectModeling, FilletBuilder, HoleBuilder, MassCalculator, PrimitiveBuilder,
    ShellBuilder, SweepBuilder,
};
use zenith_geom::NurbsCurve3;
use zenith_io::{StepExporter, StlExporter};
use zenith_math::Point3;
use zenith_tess::{tessellate_solid, TessellationParams};

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let out_dir = Path::new("target/samples/showcase");
    fs::create_dir_all(out_dir)?;

    println!("============================================================");
    println!("🌟 Zenith CAD Kernel - 新規作例 STEP / STL エクスポート開始 🌟");
    println!("============================================================\n");

    let params = TessellationParams {
        u_divisions: 32,
        v_divisions: 32,
    };

    // ------------------------------------------------------------
    // 作例 1: メカニカル・マウントブラケット (Mechanical Bracket)
    // 穴あきベースプレート (40x50x10) + 直径12mm貫通穴
    // ------------------------------------------------------------
    println!("📦 1. メカニカル・マウントブラケット (mechanical_bracket.stp) を生成中...");
    let bracket = HoleBuilder::make_drilled_box(40.0, 50.0, 10.0, 6.0)?;
    let mesh1 = tessellate_solid(&bracket, &params);
    let mass1 = MassCalculator::compute_from_mesh(&mesh1);
    StepExporter::export_solid_to_file(
        &bracket,
        out_dir.join("mechanical_bracket.stp").to_str().unwrap(),
        "MECHANICAL_BRACKET",
    )?;
    StlExporter::export_binary(
        &mesh1,
        out_dir.join("mechanical_bracket.stl").to_str().unwrap(),
    )?;
    println!(
        "   -> 面数: {}, 体積: {:.2} mm³, 表面積: {:.2} mm²",
        bracket.outer_shell.faces.len(),
        mass1.volume,
        mass1.surface_area
    );

    // ------------------------------------------------------------
    // 作例 2: エレクトロニクス中空筐体 (Enclosure Casing)
    // 幅60 x 奥行40 x 高さ25, 肉厚 t=2.0mm の中空シェルソリッド
    // ------------------------------------------------------------
    println!("📦 2. エレクトロニクス中空筐体 (enclosure_casing.stp) を生成中...");
    let casing = ShellBuilder::make_hollow_box(60.0, 40.0, 25.0, 2.0, 1)?;
    let mesh2 = tessellate_solid(&casing, &params);
    let mass2 = MassCalculator::compute_from_mesh(&mesh2);
    StepExporter::export_solid_to_file(
        &casing,
        out_dir.join("enclosure_casing.stp").to_str().unwrap(),
        "ENCLOSURE_CASING",
    )?;
    StlExporter::export_binary(
        &mesh2,
        out_dir.join("enclosure_casing.stl").to_str().unwrap(),
    )?;
    println!(
        "   -> 面数: {}, 体積: {:.2} mm³, 表面積: {:.2} mm²",
        casing.outer_shell.faces.len(),
        mass2.volume,
        mass2.surface_area
    );

    // ------------------------------------------------------------
    // 作例 3: 工業用円錐レデューサー・ノズル (Industrial Cone Reducer)
    // 底面半径 R=25, 天面半径 R=10, 高さ H=45 の有理NURBS円錐台
    // ------------------------------------------------------------
    println!("📦 3. 工業用円錐レデューサー・ノズル (cone_reducer.stp) を生成中...");
    let cone = PrimitiveBuilder::make_cone(25.0, 10.0, 45.0)?;
    let mesh3 = tessellate_solid(&cone, &params);
    let mass3 = MassCalculator::compute_from_mesh(&mesh3);
    StepExporter::export_solid_to_file(
        &cone,
        out_dir.join("cone_reducer.stp").to_str().unwrap(),
        "CONE_REDUCER",
    )?;
    StlExporter::export_binary(&mesh3, out_dir.join("cone_reducer.stl").to_str().unwrap())?;
    println!(
        "   -> 面数: {}, 体積: {:.2} mm³, 表面積: {:.2} mm²",
        cone.outer_shell.faces.len(),
        mass3.volume,
        mass3.surface_area
    );

    // ------------------------------------------------------------
    // 作例 4: 精密Oリング・トーラスソリッド (Precision Torus)
    // 主半径 R=30, 断面半径 r=6 の完全真円回転NURBSソリッド
    // ------------------------------------------------------------
    println!("📦 4. 精密Oリング・トーラス (precision_torus.stp) を生成中...");
    let torus = PrimitiveBuilder::make_torus(30.0, 6.0)?;
    let mesh4 = tessellate_solid(&torus, &params);
    let mass4 = MassCalculator::compute_from_mesh(&mesh4);
    StepExporter::export_solid_to_file(
        &torus,
        out_dir.join("precision_torus.stp").to_str().unwrap(),
        "PRECISION_TORUS",
    )?;
    StlExporter::export_binary(
        &mesh4,
        out_dir.join("precision_torus.stl").to_str().unwrap(),
    )?;
    println!(
        "   -> 面数: {}, 体積: {:.2} mm³, 表面積: {:.2} mm²",
        torus.outer_shell.faces.len(),
        mass4.volume,
        mass4.surface_area
    );

    // ------------------------------------------------------------
    // 作例 5: 3Dスプライン・スイープ配管 (Sweep Exhaust Pipe)
    // S字3Dパスに沿った外径10mmの滑らかなスイープソリッド
    // ------------------------------------------------------------
    println!("📦 5. 3Dスプライン・スイープ配管 (sweep_pipe.stp) を生成中...");
    let path = NurbsCurve3::bspline_from_points(
        3,
        vec![
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(10.0, 20.0, 15.0),
            Point3::new(30.0, 10.0, 35.0),
            Point3::new(50.0, 30.0, 50.0),
        ],
    )?;
    let sweep = SweepBuilder::sweep_circle_along_curve(&path, 5.0, 32)?;
    let mesh5 = tessellate_solid(&sweep, &params);
    let mass5 = MassCalculator::compute_from_mesh(&mesh5);
    StepExporter::export_solid_to_file(
        &sweep,
        out_dir.join("sweep_pipe.stp").to_str().unwrap(),
        "SWEEP_PIPE",
    )?;
    StlExporter::export_binary(&mesh5, out_dir.join("sweep_pipe.stl").to_str().unwrap())?;
    println!(
        "   -> 面数: {}, 体積: {:.2} mm³, 表面積: {:.2} mm²",
        sweep.outer_shell.faces.len(),
        mass5.volume,
        mass5.surface_area
    );

    // ------------------------------------------------------------
    // 作例 6: 4隅R5フィレット直方体 (Filleted Block)
    // 30x40x20 の4隅にR=5.0mmのフィレット
    // ------------------------------------------------------------
    println!("📦 6. 4隅R5フィレット直方体 (filleted_block.stp) を生成中...");
    let tol = zenith_math::Tolerance::default();
    let filleted = FilletBuilder::fillet_box_z_edges(30.0, 40.0, 20.0, 5.0, &tol)?;
    let mesh6 = tessellate_solid(&filleted, &params);
    let mass6 = MassCalculator::compute_from_mesh(&mesh6);
    StepExporter::export_solid_to_file(
        &filleted,
        out_dir.join("filleted_block.stp").to_str().unwrap(),
        "FILLETED_BLOCK",
    )?;
    StlExporter::export_binary(&mesh6, out_dir.join("filleted_block.stl").to_str().unwrap())?;
    println!(
        "   -> 面数: {}, 体積: {:.2} mm³, 表面積: {:.2} mm²",
        filleted.outer_shell.faces.len(),
        mass6.volume,
        mass6.surface_area
    );

    // ------------------------------------------------------------
    // 作例 7: 単一エッジ・ダイレクトフィレット (Single Fillet Box)
    // 25x35x20 の特定エッジのみにR=6.0mmのフィレット
    // ------------------------------------------------------------
    println!("📦 7. 単一エッジ・ダイレクトフィレット (single_fillet_box.stp) を生成中...");
    let single_fillet = DirectModeling::fillet_box_single_edge(25.0, 35.0, 20.0, 0, 6.0)?;
    let mesh7 = tessellate_solid(&single_fillet, &params);
    let mass7 = MassCalculator::compute_from_mesh(&mesh7);
    StepExporter::export_solid_to_file(
        &single_fillet,
        out_dir.join("single_fillet_box.stp").to_str().unwrap(),
        "SINGLE_FILLET_BOX",
    )?;
    StlExporter::export_binary(
        &mesh7,
        out_dir.join("single_fillet_box.stl").to_str().unwrap(),
    )?;
    println!(
        "   -> 面数: {}, 体積: {:.2} mm³, 表面積: {:.2} mm²",
        single_fillet.outer_shell.faces.len(),
        mass7.volume,
        mass7.surface_area
    );

    // ------------------------------------------------------------
    // 作例 8: C3面取りブロック (Chamfered Block)
    // 30x40x20 の4エッジに 3mm の面取り平面
    // ------------------------------------------------------------
    println!("📦 8. C3面取りブロック (chamfered_block.stp) を生成中...");
    let chamfered = ChamferBuilder::chamfer_box_z_edges(30.0, 40.0, 20.0, 3.0, &tol)?;
    let mesh8 = tessellate_solid(&chamfered, &params);
    let mass8 = MassCalculator::compute_from_mesh(&mesh8);
    StepExporter::export_solid_to_file(
        &chamfered,
        out_dir.join("chamfered_block.stp").to_str().unwrap(),
        "CHAMFERED_BLOCK",
    )?;
    StlExporter::export_binary(
        &mesh8,
        out_dir.join("chamfered_block.stl").to_str().unwrap(),
    )?;
    println!(
        "   -> 面数: {}, 体積: {:.2} mm³, 表面積: {:.2} mm²",
        chamfered.outer_shell.faces.len(),
        mass8.volume,
        mass8.surface_area
    );

    println!("\n🎉 全8種類の作例 STEP / STL ファイルの生成が完了しました！");
    println!("📁 保存先: {}", out_dir.canonicalize()?.display());
    println!("============================================================");

    Ok(())
}
